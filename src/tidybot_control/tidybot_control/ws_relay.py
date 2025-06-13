import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray, String
from tidybot_msgs.msg import WSMsg
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
import numpy as np

class WSRelay(Node):
    def __init__(self):
        super().__init__("ws_relay")
        self.base_pub = self.create_publisher(Float64MultiArray, "/base_controller/command", 10)
        self.arm_pub = self.create_publisher(Pose, "/arm_controller/command", 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, "/gripper_controller/command", 10
        )
        self.state_pub = self.create_publisher(String, "/ws_state", 10)
        self.ws_sub = self.create_subscription(
            WSMsg, "/ws_commands", self.ws_callback, 10
        )
        self.joint_states_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, 10
        )
        self.enable_counts = {}

        # WebXR reference position
        self.base_xr_ref = None
        self.arm_xr_ref = None

        # Robot reference position
        self.base_ref = None
        self.arm_ref = None
        self.gripper_ref = None

        # Observed position
        self.base_obs = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.arm_obs = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # [x, y, z, ]
        self.gripper_obs = []

    def ws_callback(self, msg):
        if msg.state_update:
            self.state_pub.publish(msg.state_update)
            self.get_logger().info(f"State update: {msg.state_update}")
            return
        else:
            self.enable_counts[msg.device_id] = (
                self.enable_counts.get(msg.device_id, 0) + 1 if msg.teleop_mode else 0
            )

        if self.enable_counts[msg.device_id] > 2:
            match msg.teleop_mode:
                case "base":
                    r = R.from_quat(
                        [
                            msg.or_x,
                            msg.or_y,
                            msg.or_z,
                            msg.or_w,
                        ]
                    )
                    euler = r.as_euler("xyz", degrees=False)
                    if self.base_xr_ref is None:
                        self.base_xr_ref = np.array([msg.pos_x, msg.pos_z, euler[0]])
                        self.base_ref = self.base_obs.copy()
                
                    base_command = Float64MultiArray()
                    base_command.data = self.base_ref + (np.array([msg.pos_x, msg.pos_z, euler[0]]) - self.base_xr_ref)
                    self.base_pub.publish(base_command)
                    return
                case "arm":
                    if self.arm_xr_ref is None:
                        self.arm_xr_ref = np.array([msg.pos_x, msg.pos_y, msg.pos_z, msg.or_x, msg.or_y, msg.or_z, msg.or_w])
                        self.arm_ref = self.arm_obs.copy()
                    arm_command = Pose()
                    arm_command.position.x = self.arm_ref[0] + msg.pos_x - self.arm_xr_ref[0]
                    arm_command.position.y = self.arm_ref[1] + msg.pos_y - self.arm_xr_ref[1]
                    arm_command.position.z = self.arm_ref[2] + msg.pos_z - self.arm_xr_ref[2]
                    arm_command.orientation.x = self.arm_ref[3] + msg.or_x - self.arm_xr_ref[3]
                    arm_command.orientation.y = self.arm_ref[4] + msg.or_y - self.arm_xr_ref[4]
                    arm_command.orientation.z = self.arm_ref[5] + msg.or_z - self.arm_xr_ref[5]
                    arm_command.orientation.w = self.arm_ref[6] + msg.or_w - self.arm_xr_ref[6]
                    self.arm_pub.publish(arm_command)
                    return

                
        elif self.enable_counts[msg.device_id] == 0:
            self.base_xr_ref = None
            self.base_ref = None
            self.arm_xr_ref = None
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