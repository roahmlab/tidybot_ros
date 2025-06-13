import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from tidybot_msgs.msg import WSMsg
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
import numpy as np

class WSRelay(Node):
    def __init__(self):
        super().__init__("ws_relay")
        self.base_pub = self.create_publisher(Float64MultiArray, "/tidybot_base_pos_controller/commands", 10)
        self.arm_pub = self.create_publisher(Float64MultiArray, "/arm_controller/command", 10)
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
        # TODO
        self.arm_obs = []
        self.gripper_obs = []

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
                        self.base_xr_ref = np.array([msg.pos_x, msg.pos_z, euler[1]])
                        self.base_ref = self.base_obs.copy()
                
                    base_command = Float64MultiArray()
                    base_command.data = (self.base_ref + (np.array([msg.pos_x, msg.pos_z, euler[1]]) - self.base_xr_ref)).tolist()
                    self.base_pub.publish(base_command)
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