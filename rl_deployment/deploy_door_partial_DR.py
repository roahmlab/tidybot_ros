import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener
import onnxruntime as ort
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import csv
import os

class RLDeployerNode(Node):
    def __init__(self):
        super().__init__('rl_policy_deployer')

        self.DEBUG_MODE = False
        self.policy_path = 'door_partial_DR.onnx'
        self.control_rate_hz = 60.0 
        
        self.arm_joint_names = [f'joint_{i}' for i in range(1, 8)]
        self.gripper_joint_name = 'finger_joint'
        self.base_frame = 'world'  
        
        # Calibration Offsets
        delta_x, delta_y, delta_z = 0.04, 0.03, -0.04
        shift = np.array([delta_x, delta_y, delta_z], dtype=np.float32)

        self.hinge_origin_pos = np.array([1.0087, 0.00377, 0.7832], dtype=np.float32) + shift
        self.init_handle_pos = np.array([0.96722, 0.00709, 0.63134], dtype=np.float32) + shift
        
        # Static Hinge Orientation
        self.hinge_quat_w = np.array([0.70710678, 0.0, 0.70710678, 0.0], dtype=np.float32)

        # Initial Quaternion
        self.init_quat = np.array([0.70710678, -0.70710678, 0.0, 0.0], dtype=np.float32)
        
        v0 = self.init_handle_pos - self.hinge_origin_pos
        self.theta_0 = float(np.arctan2(v0[2], v0[0]))

        # --- EE TCP Offset from Isaac Sim ---
        # Isaac Sim rot=(0.0, 1.0, 0.0, 0.0) is [w, x, y, z]. 
        # SciPy requires [x, y, z, w], so we use [1.0, 0.0, 0.0, 0.0]
        self.tcp_offset_pos = np.array([0.0, 0.0, -0.2015], dtype=np.float32)
        self.tcp_offset_rot_scipy = R.from_quat([1.0, 0.0, 0.0, 0.0])

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

        # --- Sim-to-Real Joint Mapping ---
        self.ros_to_isaac_offsets = np.array([
            0.0,           # joint_1 
            0.35,          # joint_2 
            -3.14159,      # joint_3 
            2.55,          # joint_4 
            0.0,           # joint_5 
            0.87,          # joint_6 
            -1.5707963     # joint_7 
        ], dtype=np.float32)
        
        self.joint_directions = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0], dtype=np.float32)

        # --- ONNX Setup ---
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        self.session = ort.InferenceSession(self.policy_path, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        
        expected_shape = self.session.get_inputs()[0].shape
        self.get_logger().info(f"Policy loaded. Expected input shape: {expected_shape}")
        
        # --- TF2 Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- State Tracking & Robustness ---
        self.current_arm_pos = np.zeros(7, dtype=np.float32)
        self.last_arm_pos = np.zeros(7, dtype=np.float32)
        self.arm_vel = np.zeros(7, dtype=np.float32)
        self.ros_gripper_pos = 0.0  
        
        self.last_joint_msg_time = 0.0
        self.staleness_threshold_sec = 0.5 
        self.vel_ema_alpha = 1.0
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
        
        # --- Grasp State & Cooldown Logic ---
        self.is_grasping = False
        self.has_grasped_once = False  
        self.last_gripper_cmd = 0.0 
        self.last_gripper_toggle_time = 0.0
        self.gripper_transit_time_sec = 0.8  
        self.grasp_theta_0 = None 

        # --- Observation Logging Setup ---
        self.log_filepath = 'hardware_rollout_log.csv'
        self.log_file = open(self.log_filepath, mode='w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        
        header = [f'obs_{i}' for i in range(40)] + [f'action_{i}' for i in range(8)]
        self.csv_writer.writerow(header)
        self.get_logger().info(f"Logging observations to {self.log_filepath}")

    def destroy_node(self):
        self.get_logger().info("Closing CSV log file safely...")
        self.log_file.close()
        super().destroy_node()

    # ==========================================
    # --- MATH UTILS ---
    # ==========================================
    def wrap_to_pi(self, angles):
        return (angles + np.pi) % (2 * np.pi) - np.pi

    def isaac_quat_conjugate(self, q):
        return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float32)

    def isaac_quat_mul(self, a, b):
        w1, x1, y1, z1 = a
        w2, x2, y2, z2 = b
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ], dtype=np.float32)

    def isaac_quat_apply(self, q, v):
        q_w = q[0]
        q_vec = q[1:]
        t = 2.0 * np.cross(q_vec, v)
        return v + q_w * t + np.cross(q_vec, t)
    
    def joint_state_callback(self, msg):
        try:
            current_time = time.time()
            joint_dict = dict(zip(msg.name, msg.position))
            self.current_arm_pos = np.array([joint_dict[name] for name in self.arm_joint_names], dtype=np.float32)
            
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
            self.get_logger().warn(f"Missing joint: {e}", throttle_duration_sec=5.0)

    # ==========================================
    # --- OBSERVATION GENERATORS ---
    # ==========================================
    def get_ee_transform(self):
        now_ros = self.get_clock().now()
        try:
            trans = self.tf_buffer.lookup_transform(self.base_frame, 'bracelet_link', rclpy.time.Time())
            trans_time = trans.header.stamp.sec + (trans.header.stamp.nanosec * 1e-9)
            age = (now_ros.nanoseconds * 1e-9) - trans_time
            
            if age > self.staleness_threshold_sec:
                self.get_logger().warn(f"TF stale! Age: {age:.3f}s", throttle_duration_sec=1.0)
                return None, None
                
            t_world_bracelet = np.array([
                trans.transform.translation.x, 
                trans.transform.translation.y, 
                trans.transform.translation.z
            ], dtype=np.float32)
            
            r_world_bracelet_scipy = R.from_quat([
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ])
            
            ee_pos = t_world_bracelet + r_world_bracelet_scipy.apply(self.tcp_offset_pos)
            ee_pos = ee_pos.astype(np.float32)
            
            r_ee_scipy = r_world_bracelet_scipy * self.tcp_offset_rot_scipy
            q_ee_xyzw = r_ee_scipy.as_quat()
            
            ee_quat = np.array([
                q_ee_xyzw[3], 
                q_ee_xyzw[0], 
                q_ee_xyzw[1], 
                q_ee_xyzw[2]  
            ], dtype=np.float32)
            
            return ee_pos, ee_quat
        except Exception as e:
            self.get_logger().warn(f"TF Lookup failed: {e}", throttle_duration_sec=1.0)
            return None, None

    def get_rel_ee_handle_transform(self, ee_pos, ee_quat, handle_pos, handle_quat):
        pos_error_world = handle_pos - ee_pos
        ee_quat_inv = self.isaac_quat_conjugate(ee_quat)
        pos_error_local = self.isaac_quat_apply(ee_quat_inv, pos_error_world)
        
        rot_error = self.isaac_quat_mul(ee_quat_inv, handle_quat)
        
        x_axis = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        
        handle_x_in_ee = self.isaac_quat_apply(rot_error, x_axis)
        handle_z_in_ee = self.isaac_quat_apply(rot_error, z_axis)
        
        return np.concatenate([pos_error_local, handle_x_in_ee, handle_z_in_ee]).astype(np.float32)

    def get_ee_to_hinge_in_ee_frame(self, ee_pos, ee_quat, hinge_pos_w):
        ee_to_hinge_w = hinge_pos_w - ee_pos
        ee_quat_inv = self.isaac_quat_conjugate(ee_quat)
        return self.isaac_quat_apply(ee_quat_inv, ee_to_hinge_w).astype(np.float32)

    def get_hinge_axis_in_ee_frame(self, ee_quat, hinge_quat_w):
        ee_quat_inv = self.isaac_quat_conjugate(ee_quat)
        rot_ee_to_hinge = self.isaac_quat_mul(ee_quat_inv, hinge_quat_w)
        local_z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        return self.isaac_quat_apply(rot_ee_to_hinge, local_z_axis).astype(np.float32)
    
    # ==========================================
    # --- MAIN LOOP ---
    # ==========================================
    def control_loop(self):
        if not rclpy.ok():
            return

        current_time = time.time()
            
        if self.last_joint_msg_time == 0.0 or (current_time - self.last_joint_msg_time) > self.staleness_threshold_sec:
            return

        isaac_gripper_pos = self.ros_gripper_pos

        # 1. COMPUTE GRIPPER COOLDOWN
        time_since_toggle = current_time - self.last_gripper_toggle_time
        gripper_cooldown = float(max(0.0, 1.0 - (time_since_toggle / self.gripper_transit_time_sec)))

        # 2. CONTINUOUS GRASP EVALUATION
        if self.last_gripper_cmd == 1.0: 
            self.is_grasping = True
            if gripper_cooldown == 0.0:
                if not self.is_grasping:
                    self.get_logger().info("GRASP SECURED. Initiating dynamic kinematic inference!")
                    self.is_grasping = True
                    self.has_grasped_once = True
        else:
            if self.is_grasping:
                self.get_logger().info("Grasp fully released. Reverting to static visual tracking.")
                self.is_grasping = False
                self.grasp_theta_0 = None 

        # 3. ENVIRONMENT CONTEXT & KINEMATIC INFERENCE
        tf_result = self.get_ee_transform()
        if tf_result[0] is None:
            return
        ee_pos, ee_quat = tf_result
        
        if self.is_grasping or self.has_grasped_once:
            v_t = ee_pos - self.hinge_origin_pos
            theta_t = float(np.arctan2(v_t[2], v_t[0]))
            
            if self.grasp_theta_0 is None:
                self.grasp_theta_0 = theta_t
                self.get_logger().info(f"Captured rigid grasp point angle: {self.grasp_theta_0:.3f}")

            door_pos = float(self.wrap_to_pi(self.grasp_theta_0 - theta_t))
            isaac_gripper_pos = min(isaac_gripper_pos, 0.74)
            gripper_cooldown = max(gripper_cooldown, 0.2)
        else:
            door_pos = 0.0

        rot_y_quat = np.array([np.cos(door_pos/2.0), 0.0, np.sin(door_pos/2.0), 0.0], dtype=np.float32)
        v_init = self.init_handle_pos - self.hinge_origin_pos
        handle_pos = self.hinge_origin_pos + self.isaac_quat_apply(rot_y_quat, v_init)
        door_quat = self.isaac_quat_mul(rot_y_quat, self.init_quat)

        # 4. GENERATE NEW GEOMETRIC OBSERVATIONS
        rel_transform = self.get_rel_ee_handle_transform(ee_pos, ee_quat, handle_pos, door_quat)
        
        ee_to_hinge = self.get_ee_to_hinge_in_ee_frame(ee_pos, ee_quat, self.hinge_origin_pos)
        hinge_axis = self.get_hinge_axis_in_ee_frame(ee_quat, self.hinge_quat_w)

        # 5. POLICY INFERENCE
        isaac_arm_pos = self.wrap_to_pi((self.current_arm_pos + self.ros_to_isaac_offsets) * self.joint_directions)
        isaac_arm_vel = self.arm_vel * self.joint_directions
        
        obs_list = np.concatenate([
            isaac_arm_pos,               
            isaac_arm_vel,               
            [isaac_gripper_pos],         
            [gripper_cooldown],          
            rel_transform,
            ee_to_hinge,                 
            hinge_axis,                  
            [door_pos],                  
            self.last_action             
        ]).astype(np.float32)

        try:
            action = self.session.run(None, {self.input_name: np.expand_dims(obs_list, 0)})[0][0]
        except Exception as e:
            self.get_logger().error(f"Inference Error: {e}")
            return

        policy_wants_closed = action[7] <= 0.0
        
        # --- DISTANCE MASKING ---
        live_pos_error = handle_pos - ee_pos
        dist_to_handle = float(np.linalg.norm(live_pos_error))
        
        if policy_wants_closed and dist_to_handle > 0.12 and not self.has_grasped_once:
            self.get_logger().info(f"Suppressing early grasp (Distance: {dist_to_handle:.2f}m)", throttle_duration_sec=1.0)
            target_gripper_cmd = 0.0 
        else:
            target_gripper_cmd = 1.0 if policy_wants_closed else 0.0

        # 6. ACTION LOCK & GRIPPER TOGGLE
        if gripper_cooldown > 0.0:
            target_gripper_cmd = self.last_gripper_cmd

        if target_gripper_cmd != self.last_gripper_cmd:
            direction = "CLOSE" if target_gripper_cmd == 1.0 else "OPEN"
            self.get_logger().info(f"Policy triggered {direction}. Starting 0.8s cooldown lock.")
            self.last_gripper_toggle_time = current_time

        # 7. NORMAL ARM EXECUTION
        action_scale = 0.005
        
        target_arm_pos = self.current_arm_pos + (action[:7] * action_scale * self.joint_directions)

        for i in range(7):
            if self.joint_bounds[i] is not None:
                target_arm_pos[i] = np.clip(target_arm_pos[i], *self.joint_bounds[i])

        true_action = np.zeros(8, dtype=np.float32)
        true_action[:7] = ((target_arm_pos - self.current_arm_pos) / action_scale) * self.joint_directions
        true_action[7] = -1.0 if target_gripper_cmd == 1.0 else 1.0

        log_row = np.concatenate([obs_list, true_action]).tolist()
        self.csv_writer.writerow(log_row)

        self.publish_commands(target_arm_pos, target_gripper_cmd)
        
        self.last_action = action.copy()
        self.last_gripper_cmd = target_gripper_cmd

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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()