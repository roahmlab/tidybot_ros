import rclpy
import rclpy.exceptions
from rclpy.node import Node
from rclpy.clock import JumpThreshold
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.logging import LoggingSeverity
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float64, Float64MultiArray, String
from tidybot_utils.msg import WSMsg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from scipy.spatial.transform import Rotation as R
from tf_transformations import quaternion_multiply, quaternion_inverse
import tf2_ros
from tf2_ros import TransformBroadcaster
import numpy as np
import math
import gc

class WSRelay(Node):
    def __init__(self):
        super().__init__("ws_relay")
        self.declare_parameter("use_sim", True)
        self.use_sim = self.get_parameter("use_sim").get_parameter_value().bool_value
        self.get_logger().info('Web server relay initialized with use_sim: {}'.format(self.use_sim))
        if self.use_sim:
            self.joint_states_sub = self.create_subscription(
                JointState, "/joint_states", self.joint_states_callback, 10
            )
            self.base_pub = self.create_publisher(Float64MultiArray, "/tidybot_base_pos_controller/commands", 10)
        else:
            self.base_state_sub = self.create_subscription(
                JointState, "/tidybot/base/joint_states", self.joint_states_callback, 10
            )
            self.base_pub = self.create_publisher(Float64MultiArray, "/tidybot/base/commands", 10)

        self.state_pub = self.create_publisher(String, "/ws_state", 10)
        self.ws_sub = self.create_subscription(
            WSMsg, "/ws_commands", self.ws_callback, 10
        )

        self.arm_pub = self.create_publisher(Pose, "/tidybot/arm/command", 10)
        self.gripper_pub = self.create_publisher(Float64, "/tidybot/gripper/command", 10)
        if self.use_sim:
            self.arm_state_sub = self.create_subscription(
                JointState, "/joint_states", self.joint_states_callback, 10
            )
        else:
            self.arm_state_sub = self.create_subscription(
                PoseStamped, "/tidybot/arm/pose", self.joint_states_callback, 10
            )

        self.rc_br = TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.enable_counts = {}

        self.clock = self.get_clock()

        threshold = JumpThreshold(min_forward=None,
                                  min_backward=Duration(seconds=-1),
                                  on_clock_change=True)
        self.jump_handle = self.clock.create_jump_callback(threshold, post_callback=self.time_jump_callback)

        # WebXR reference position
        self.base_xr_ref_pos = None
        self.base_xr_ref_quat = None
        self.arm_xr_ref_pos = None
        self.arm_xr_ref_quat = None

        # Robot reference position
        self.base_ref_pos = None
        self.base_ref_quat = None
        self.arm_ref_pos = None
        self.arm_ref_quat = None
        self.gripper_ref = None

        # Observed position
        self.base_obs = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.arm_obs_pos = [0.0, 0.0, 1.0]
        self.arm_obs_quat = R.from_quat([0.0, 0.0, 0.0, 1.0])
        self.gripper_obs = 0

    def ws_callback(self, msg):
        if msg.state_update:
            state_command = String()
            state_command.data = msg.state_update
            self.state_pub.publish(state_command)
            return
        else:
            self.enable_counts[msg.device_id] = (
                self.enable_counts.get(msg.device_id, 0) + 1 if msg.teleop_mode else 0
            )

        if self.enable_counts[msg.device_id] > 2:
            xr_pos = np.array([msg.pos_x, msg.pos_y, msg.pos_z])
            xr_quat = np.array([msg.or_x, msg.or_y, msg.or_z, msg.or_w])

            match msg.teleop_mode:
                case "base":
                    if self.base_xr_ref_pos is None:
                        self.base_xr_ref_pos = np.array([msg.pos_x, msg.pos_y, msg.pos_z])
                        self.base_xr_ref_quat = np.array([msg.or_x, msg.or_y, msg.or_z, msg.or_w])
                        self.base_ref_pos = np.array([self.base_obs[1], self.base_obs[0], 0.0])
                        self.base_ref_quat = np.array([0.0, 0.0, self.base_obs[2], 1.0])
                        self.xr_ref_r = R.from_quat(self.base_xr_ref_quat)
                        self.base_re_r =  R.from_quat(self.base_ref_quat)
                    else:
                        # convert the incoming position and orientation wrt world frame to the wx_ref frame
                        delta_quat = quaternion_multiply(
                            quaternion_inverse(self.base_xr_ref_quat),
                            xr_quat
                        )
                        delta_pos =  xr_pos - self.base_xr_ref_pos
                        delta_pos = self.xr_ref_r.inv().apply(delta_pos)
                        delta_pos = self.base_re_r.apply(delta_pos)
                        yaw = math.atan2(2 * (delta_quat[3] * delta_quat[2] + delta_quat[0] * delta_quat[1]),
                                            delta_quat[3]**2 - delta_quat[2]**2 - delta_quat[1]**2 + delta_quat[0]**2)
                        command = Float64MultiArray()
                        command.data = [
                            delta_pos[1] + self.base_ref_pos[1],
                            -delta_pos[0] - self.base_ref_pos[0],
                            yaw + self.base_ref_quat[2]
                        ]
                        self.base_pub.publish(command)
                        self.get_logger().info(f'Base pos: {self.base_ref_pos}, Base quat: {self.base_ref_quat}, Delta pos: {delta_pos}, Delta quat: {delta_quat}')
                        return

                case "arm":
                    xr_pos, xr_quat = convert_webxr_pose(xr_pos, xr_quat)

                    # Store reference poses
                    if self.arm_xr_ref_pos is None:
                        self.arm_xr_ref_pos = xr_pos
                        self.arm_xr_ref_rot_inv = xr_quat.inv()
                        self.arm_ref_pos = self.arm_obs_pos.copy()
                        self.arm_ref_quat = self.arm_obs_quat
                        self.arm_ref_base_pose = self.base_obs.copy()
                        self.gripper_ref = self.gripper_obs

                    # Rotations around z-axis to go between global frame (base) and local frame (arm)
                    z_rot = R.from_rotvec(np.array([0.0, 0.0, 1.0]) * self.base_obs[2])
                    z_rot_inv = z_rot.inv()
                    ref_z_rot = R.from_rotvec(np.array([0.0, 0.0, 1.0]) * self.arm_ref_base_pose[2])

                    # Position
                    pos_diff = xr_pos - self.arm_xr_ref_pos  # WebXR
                    pos_diff += ref_z_rot.apply(self.arm_ref_pos) - z_rot.apply(self.arm_ref_pos)  # Secondary base control: Compensate for base rotation
                    pos_diff[:2] += self.arm_ref_base_pose[:2] - self.base_obs[:2]  # Secondary base control: Compensate for base translation
                    arm_target_pos = self.arm_ref_pos + z_rot_inv.apply(pos_diff)

                    # Orientation
                    arm_target_quat = (z_rot_inv * (xr_quat * self.arm_xr_ref_rot_inv) * ref_z_rot) * self.arm_ref_quat

                    arm_command = Pose()
                    arm_command.position.x = arm_target_pos[0]
                    arm_command.position.y = arm_target_pos[1]
                    arm_command.position.z = arm_target_pos[2]
                    arm_command.orientation.x, \
                    arm_command.orientation.y, \
                    arm_command.orientation.z, \
                    arm_command.orientation.w = arm_target_quat.as_quat()
                    self.arm_pub.publish(arm_command)

                    gripper_command = Float64()
                    gripper_command.data = np.clip(self.gripper_ref + msg.gripper_delta, 0, 1.0)
                    self.gripper_obs = gripper_command.data
                    self.gripper_pub.publish(gripper_command)
                    return

        elif self.enable_counts[msg.device_id] == 0:
            self.base_xr_ref_pos = None
            self.base_xr_ref_quat = None
            self.base_ref_pos = None
            self.base_ref_quat = None
            self.arm_xr_ref_pos = None
            self.arm_xr_ref_quat = None
            self.arm_ref_pos = None
            self.arm_ref_quat = None
            self.gripper_ref = None

    def joint_states_callback(self, msg):
        # self.base_obs[0] = msg.position[msg.name.index("joint_x")]
        # self.base_obs[1] = msg.position[msg.name.index("joint_y")]
        # self.base_obs[2] = msg.position[msg.name.index("joint_th")]
        # self.get_logger().info(f'Base joint states: {self.base_obs}')

        if self.use_sim:
            # Forward kinematics to find end effector position
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame="world",
                    source_frame="end_effector_link",
                    time=rclpy.time.Time(),    
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                # Position
                t = transform.transform.translation
                self.arm_obs_pos = [t.x, t.y, t.z]
                # Orientation
                q = transform.transform.rotation
                self.arm_obs_quat = R.from_quat([q.x, q.y, q.z, q.w])

            except tf2_ros.LookupException as e:
                self.get_logger().warn(f"Transform not found: {e}")
        else:
            self.arm_obs_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            self.arm_obs_quat = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        
        # self.get_logger().info(f'Arm position: {self.arm_obs_pos}')

    def base_state_callback(self, msg):
        pass

    def time_jump_callback(self, time: Time):
        # re-instantiate the TF listener and buffer to avoid TF_OLD_DATA warning
        self.destroy_subscription(self.tf_listener.tf_sub)
        self.destroy_subscription(self.tf_listener.tf_static_sub)
        del self.tf_listener
        del self.tf_buffer
        gc.collect()
        # Clear both dynamic and static entries
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info('TF buffer reset')


DEVICE_CAMERA_OFFSET = np.array([0.0, 0.085, -0.12])  # iPad

def convert_webxr_pose(pos, quat):
    # WebXR: +x right, +y up, +z back; Robot: +x forward, +y left, +z up
    pos = np.array([-pos[2], -pos[0], pos[1]], dtype=np.float64)
    rot = R.from_quat([-quat[2], -quat[0], quat[1], quat[3]])
    
    # Apply offset so that rotations are around device center instead of device camera
    pos = pos + rot.apply(DEVICE_CAMERA_OFFSET)

    return pos, rot

def main():
    rclpy.init()
    ws_relay = WSRelay()
    rclpy.spin(ws_relay)
    ws_relay.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()