import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import ControlWorld, SpawnEntity
from ament_index_python.packages import get_package_share_directory
import subprocess

robot_description_path = get_package_share_directory("tidybot_description")

RED   = "\x1b[31m"
GREEN = "\x1b[32m"
RESET = "\x1b[0m"

class StateController(Node):
    def __init__(self):
        super().__init__("state_controller")
        self.declare_parameter("use_sim", True)
        self.use_sim = self.get_parameter("use_sim").get_parameter_value().bool_value

        self.state_sub = self.create_subscription(
            String, "/ws_state", self.state_callback, 10
        )

    def state_callback(self, msg):
        self.get_logger().info(f"Received state command: {msg.data}")
        match msg.data:
            case "reset_env":
                subprocess.run(["ros2", "run", "tidybot_control", "reset_env",    
                                f"--ros-args", "-p", f"use_sim:={str(self.use_sim).lower()}"])
            case "episode_started":
                self.get_logger().info(f"{GREEN}Episode started.{RESET}")
            case "episode_finished":
                self.get_logger().info(f"{GREEN}Episode finished.{RESET}")



def main(args=None):
    rclpy.init(args=args)
    state_controller = StateController()
    rclpy.spin(state_controller)
    state_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
