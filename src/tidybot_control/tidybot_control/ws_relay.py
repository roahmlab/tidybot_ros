import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray, String
from tidybot_msgs.msg import WSMsg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from scipy.spatial.transform import Rotation as R
from tf_transformations import quaternion_multiply, quaternion_inverse
from tf2_ros import TransformBroadcaster
import numpy as np
import math

class WSRelay(Node):
    def __init__(self):
        super().__init__("ws_relay")
        self.get_logger().info('Web server relay initialized')
        self.base_pub = self.create_publisher(Float64MultiArray, "/base_controller/command", 10)
        self.arm_pub = self.create_publisher(Pose, "/arm_controller/command", 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, "/gripper_controller/command", 10
        )
        self.rc_br = TransformBroadcaster(self)
        self.state_pub = self.create_publisher(String, "/ws_state", 10)
        self.ws_sub = self.create_subscription(
            WSMsg, "/ws_commands", self.ws_callback, 10
        )
        self.joint_states_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, 10
        )
        self.enable_counts = {}

        # WebXR reference position
        self.base_xr_ref_pos = None
        self.base_xr_ref_quat = None
        self.arm_xr_ref_pos = None
        self.arm_xr_ref_quat = None

        # Robot reference position
        self.base_ref = None
        self.arm_ref = None
        self.gripper_ref = None

        # Observed position
        self.base_obs = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.arm_obs = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]) # [x, y, z, ]
        self.gripper_obs = []

        # Publish frequency
        self.last_publish_time = self.get_clock().now()
        self.publish_interval = rclpy.duration.Duration(seconds=0.2)  # 10 Hz


    def ws_callback(self, msg):
        if msg.state_update:
            state_command = String()
            state_command.data = msg.state_update
            self.state_pub.publish(state_command)
            self.get_logger().info(f"State update: {msg.state_update}")
            return
        else:
            self.enable_counts[msg.device_id] = (
                self.enable_counts.get(msg.device_id, 0) + 1 if msg.teleop_mode else 0
            )

        if self.enable_counts[msg.device_id] > 2:
            match msg.teleop_mode:
                case "base":
                    xr_pos = np.array([msg.pos_x, msg.pos_y, msg.pos_z])
                    xr_quat = np.array([msg.or_x, msg.or_y, msg.or_z, msg.or_w])
                    if self.base_xr_ref_pos is None:
                        self.base_xr_ref_pos = np.array([msg.pos_x, msg.pos_y, msg.pos_z])
                        self.base_xr_ref_quat = np.array([msg.or_x, msg.or_y, msg.or_z, msg.or_w])
                        self.base_ref = self.base_obs.copy()
                        self.r = R.from_quat(self.base_xr_ref_quat)
                    else:
                        # convert the incoming position and orientation wrt world frame to the wx_ref frame
                        delta_quat = quaternion_multiply(
                            quaternion_inverse(self.base_xr_ref_quat),
                            xr_quat
                        )
                        delta_pos =  xr_pos - self.base_xr_ref_pos
                        delta_pos = self.r.inv().apply(delta_pos)
                        yaw = math.atan2(2 * (delta_quat[3] * delta_quat[2] + delta_quat[0] * delta_quat[1]),
                                         delta_quat[3]**2 - delta_quat[2]**2 - delta_quat[1]**2 + delta_quat[0]**2)
                        command = Float64MultiArray()
                        command.data = [
                            delta_pos[0] + self.base_ref[0],
                            delta_pos[1] + self.base_ref[1],
                            yaw + self.base_ref[2]
                        ]
                        self.base_pub.publish(command)
                        return

                case "arm":
                    xr_pos = np.array([msg.pos_x, msg.pos_y, msg.pos_z])
                    xr_quat = np.array([msg.or_x, msg.or_y, msg.or_z, msg.or_w])
                    if self.arm_xr_ref_pos is None:
                        self.arm_xr_ref_pos = np.array([msg.pos_x, msg.pos_y, msg.pos_z])
                        self.arm_xr_ref_quat = np.array([msg.or_x, msg.or_y, msg.or_z, msg.or_w])
                        self.arm_ref = self.arm_obs.copy()
                        self.r = R.from_quat(self.arm_xr_ref_quat)
                    
                    now = self.get_clock().now()
                    if (now - self.last_publish_time) < self.publish_interval:
                        return 
                    self.last_publish_time = now

                    # convert the incoming position and orientation wrt world frame to the wx_ref frame
                    delta_quat = quaternion_multiply(
                        quaternion_inverse(self.arm_xr_ref_quat),
                        xr_quat
                    )
                    delta_pos =  xr_pos - self.arm_xr_ref_pos
                    delta_pos = self.r.inv().apply(delta_pos)

                    arm_command = Pose()
                    arm_command.position.x = delta_pos[0] + self.arm_ref[0]
                    arm_command.position.y = delta_pos[1] + self.arm_ref[1]
                    arm_command.position.z = delta_pos[2] + self.arm_ref[2]
                    arm_command.orientation.x = delta_quat[0] + self.arm_ref[3]
                    arm_command.orientation.y = delta_quat[1] + self.arm_ref[4]
                    arm_command.orientation.z = delta_quat[2] + self.arm_ref[5]
                    arm_command.orientation.w = delta_quat[3] + self.arm_ref[6]
                    self.arm_pub.publish(arm_command)
                    return

        elif self.enable_counts[msg.device_id] == 0:
            self.base_xr_ref_pos = None
            self.base_xr_ref_quat = None
            self.base_ref = None
            self.arm_xr_ref_pos = None
            self.arm_xr_ref_quat = None
            self.arm_ref = None
            self.gripper_xr_ref = None
            self.gripper_ref = None

    def joint_states_callback(self, msg):
        self.base_obs[0] = msg.position[msg.name.index("joint_x")]
        self.base_obs[1] = msg.position[msg.name.index("joint_y")]
        self.base_obs[2] = msg.position[msg.name.index("joint_th")]

def main():
    rclpy.init()
    ws_relay = WSRelay()
    rclpy.spin(ws_relay)
    ws_relay.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()