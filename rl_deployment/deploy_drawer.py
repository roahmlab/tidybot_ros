import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener
import onnxruntime as ort
import numpy as np
import time
import math
from scipy.spatial.transform import Rotation as R

class RLDeployerNode(Node):
    def __init__(self):
        super().__init__('rl_policy_deployer')

        self.DEBUG_MODE = False
        self.policy_path = 'drawer_DR.onnx'
        self.control_rate_hz = 60.0 
        
        self.arm_joint_names = [f'joint_{i}' for i in range(1, 8)]
        self.gripper_joint_name = 'finger_joint'
        
        self.base_frame = 'world'  
        
        # --- EE TCP Offset from Isaac Sim ---
        self.tcp_offset_pos = np.array([0.0, 0.0, 0.2015])
        
        # --- Handle State Tracking ---
        self.cached_handle_pos = np.array([1.25, 0.000, 0.333])
        # Stored in Isaac format: [w, x, y, z]
        self.cached_handle_quat_isaac = np.array([0.5, 0.5, 0.5, 0.5])

        # --- Hardware Safety Limits ---
        self.joint_bounds = {
            0: None,                   
            1: (-2.240000,  2.240000), 
            2: None,                   
            3: (-2.570000,  2.570000), 
            4: None,                   
            5: (-2.090000,  2.090000), 
            6: None,                   
        }

        # --- ONNX Setup ---
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        self.session = ort.InferenceSession(self.policy_path, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        self.get_logger().info(f"Policy loaded. Provider: {self.session.get_providers()}")
        
        # --- TF2 Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- State Tracking & Robustness ---
        self.current_arm_pos = np.zeros(7)
        self.last_arm_pos = np.zeros(7)
        self.arm_vel = np.zeros(7)
        self.ros_gripper_pos = 0.0  
        self.last_isaac_gripper_pos = 0.025
        
        self.last_joint_msg_time = 0.0
        self.staleness_threshold_sec = 0.5 
        self.vel_ema_alpha = 0.6             

        self.default_arm_pos = np.array([
            0.0, -0.35, 3.14, -2.36, 0.0, -1.13, 1.57
        ])
        
        self.last_action = np.zeros(8, dtype=np.float32)

        # --- Publishers & Subscribers ---
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        if self.DEBUG_MODE:
            arm_topic = '/debug/arm_commands'
            grip_topic = '/debug/gripper_commands'
        else:
            arm_topic = '/tidybot/hardware/arm/commands'
            grip_topic = '/tidybot/hardware/gripper/commands'

        self.arm_cmd_pub = self.create_publisher(JointState, arm_topic, 10)
        self.gripper_cmd_pub = self.create_publisher(Float64, grip_topic, 10)

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)
        
        self.is_grasping = False
        self.last_gripper_cmd = 0.0 # 0 for open, 1 for closed
        self.grasp_handle_offset = np.zeros(3)
        self.grasp_handle_quat_isaac = np.zeros(4)
        
        # --- Gripper Dynamics Matching (Sim-to-Real) ---
        self.interpolated_gripper_cmd = 0.0
        # Tune this to exactly how many seconds it takes the gripper to close in Isaac Sim
        self.sim_gripper_transit_time = 0.8

    # ==========================================
    # --- MATH UTILS (MATCHING ISAAC SIM) ---
    # ==========================================
    def wrap_to_pi(self, angles):
        return (angles + np.pi) % (2 * np.pi) - np.pi

    def isaac_quat_conjugate(self, q):
        """Matches math_utils.quat_conjugate for [w, x, y, z]"""
        return np.array([q[0], -q[1], -q[2], -q[3]])

    def isaac_quat_mul(self, a, b):
        """Matches math_utils.quat_mul for [w, x, y, z] (Hamilton product)"""
        w1, x1, y1, z1 = a
        w2, x2, y2, z2 = b
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
    # ==========================================

    def map_gripper_proprioception(self, ros_pos):
        """
        Normalizes ROS gripper position to match the Sim observation:
        0.0 = Fully Open, 1.0 = Fully Closed
        """
        ros_open = 0.0
        ros_closed = 1.0  # Adjust to your hardware's fully closed value

        normalized_pos = (ros_pos - ros_open) / (ros_closed - ros_open)
        return float(np.clip(normalized_pos, 0.0, 1.0))

    def joint_state_callback(self, msg):
        try:
            current_time = time.time()
            joint_dict = dict(zip(msg.name, msg.position))
            self.current_arm_pos = np.array([joint_dict[name] for name in self.arm_joint_names])
            
            if self.gripper_joint_name in joint_dict:
                self.ros_gripper_pos = joint_dict[self.gripper_joint_name]

            if self.last_joint_msg_time > 0:
                dt = current_time - self.last_joint_msg_time
                if 0 < dt < 0.5:  
                    raw_vel = self.wrap_to_pi(self.current_arm_pos - self.last_arm_pos) / dt
                    self.arm_vel = (self.vel_ema_alpha * raw_vel) + ((1.0 - self.vel_ema_alpha) * self.arm_vel)
            
            self.last_arm_pos = self.current_arm_pos.copy()
            self.last_joint_msg_time = current_time

        except KeyError as e:
            self.get_logger().warn(f"Missing joint in /joint_states: {e}", throttle_duration_sec=5.0)

    def get_ee_to_handle_state(self):
        """Computes relative distance and orientation exactly like Isaac Sim."""
        now_ros = self.get_clock().now()

        try:
            trans = self.tf_buffer.lookup_transform(self.base_frame, 'tool_frame', rclpy.time.Time())
            
            trans_time = trans.header.stamp.sec + (trans.header.stamp.nanosec * 1e-9)
            age = (now_ros.nanoseconds * 1e-9) - trans_time
            
            if age > self.staleness_threshold_sec:
                self.get_logger().warn(f"TF data is stale! Age: {age:.3f}s", throttle_duration_sec=1.0)
                return None, None
                
            # 1. Compute Position Error
            t_world_bracelet = np.array([
                trans.transform.translation.x, 
                trans.transform.translation.y, 
                trans.transform.translation.z
            ])
            r_world_bracelet_scipy = R.from_quat([
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ])
            ee_pos = t_world_bracelet + r_world_bracelet_scipy.apply(self.tcp_offset_pos)
            pos_error = self.cached_handle_pos - ee_pos
            
            # 2. Compute Rotation Error (q_rel = q_ee_inv * q_handle)
            ee_quat_isaac = np.array([
                trans.transform.rotation.w,
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z
            ])
            
            ee_quat_inv = self.isaac_quat_conjugate(ee_quat_isaac)
            rot_error = self.isaac_quat_mul(ee_quat_inv, self.cached_handle_quat_isaac)
            
            return pos_error, rot_error
            
        except Exception as e:
            self.get_logger().warn(f"TF Lookup failed: {e}", throttle_duration_sec=1.0)
            return None, None

    def control_loop(self):
        # 1. Health & Proprioception
        current_time = time.time()
        if self.last_joint_msg_time == 0.0 or (current_time - self.last_joint_msg_time) > self.staleness_threshold_sec:
            return

        isaac_gripper_pos = self.map_gripper_proprioception(self.ros_gripper_pos)

        # 2. Environment Context (Vision vs Locked Grasp)
        if self.is_grasping:
            pos_error, rot_error = self.get_locked_grasp_state()
        else:
            pos_error, rot_error = self.get_ee_to_handle_state()
        
        if pos_error is None:
            return

        # 3. Assemble Observation & Inference
        relative_arm_pos = self.wrap_to_pi(self.current_arm_pos - self.default_arm_pos)
        obs_list = np.concatenate([
            relative_arm_pos,         # 7
            self.arm_vel,             # 7
            [isaac_gripper_pos],      # 1
            pos_error,                # 3
            rot_error,                # 4
            self.last_action          # 8
        ]).astype(np.float32)

        try:
            action = self.session.run(None, {self.input_name: np.expand_dims(obs_list, 0)})[0][0]
        except Exception as e:
            self.get_logger().error(f"Inference Error: {e}")
            return

        # 4. Extract Binary Action
        policy_wants_closed = action[7] <= 0.0
        target_gripper_cmd = 1.0 if policy_wants_closed else 0.0
        
        # Detect leading edges for logging
        if target_gripper_cmd == 1.0 and self.last_gripper_cmd == 0.0:
            self.get_logger().info("Policy triggered CLOSE. Interpolating grasp to match Sim dynamics...")
        elif target_gripper_cmd == 0.0 and self.last_gripper_cmd == 1.0:
            self.get_logger().info("Policy triggered OPEN. Interpolating release...")

        # 5. Slew Rate Limiter (The Interpolation)
        # Calculate how much the gripper is allowed to move per control tick (20Hz)
        max_delta = 1.0 / (self.control_rate_hz * self.sim_gripper_transit_time)
        
        if self.interpolated_gripper_cmd < target_gripper_cmd:
            self.interpolated_gripper_cmd = min(target_gripper_cmd, self.interpolated_gripper_cmd + max_delta)
        elif self.interpolated_gripper_cmd > target_gripper_cmd:
            self.interpolated_gripper_cmd = max(target_gripper_cmd, self.interpolated_gripper_cmd - max_delta)

        # 6. Normal Arm Execution
        action_scale = 0.005
        target_arm_pos = self.current_arm_pos + (action[:7] * action_scale)

        # Hardware Safety
        for i in range(7):
            if self.joint_bounds[i] is not None:
                target_arm_pos[i] = np.clip(target_arm_pos[i], *self.joint_bounds[i])

        # 7. CONTINUOUS Grasp Evaluation & Slip Detection
        # We check the real proprioception, and only apply logic if it is trying to close
        if self.interpolated_gripper_cmd > 0.1: 
            if 0.67 < isaac_gripper_pos < 0.72:
                if not self.is_grasping:
                    self.get_logger().info("GRASP SECURED. Frame hijacking active (Zero Error).")
                    self.is_grasping = True
            else:
                if self.is_grasping:
                    self.get_logger().warn(f"GRASP LOST! Gripper position ({isaac_gripper_pos:.3f}). Reverting to visual tracking.")
                    self.is_grasping = False
        else:
            if self.is_grasping:
                self.get_logger().info("Policy opened gripper. Grasp released.")
                self.is_grasping = False

        # 8. Finalize State
        # CRITICAL: We publish the INTERPOLATED command, not the binary one
        self.publish_commands(target_arm_pos, self.interpolated_gripper_cmd)
        
        self.last_action = action.copy()
        self.last_gripper_cmd = target_gripper_cmd

    def get_locked_grasp_state(self):
        """Returns exactly zero error to the policy."""
        # Position error: Handle is at EE position
        pos_err = np.zeros(3, dtype=np.float32)
        # Rotation error: Identity quaternion [w, x, y, z] (Aligned)
        rot_err = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        return pos_err, rot_err
        
    def publish_commands(self, arm_pos, gripper_val):
        arm_msg = JointState()
        arm_msg.header.stamp = self.get_clock().now().to_msg()
        arm_msg.name = self.arm_joint_names
        arm_msg.position = arm_pos.tolist()
        self.arm_cmd_pub.publish(arm_msg)

        gripper_msg = Float64()
        gripper_msg.data = float(gripper_val)
        self.gripper_cmd_pub.publish(gripper_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RLDeployerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()