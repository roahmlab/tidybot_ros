import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros_gz_interfaces.msg import WorldReset, WorldControl
from ros_gz_interfaces.srv import ControlWorld
from ament_index_python.packages import get_package_share_directory

robot_description_path = get_package_share_directory("tidybot_description")

class StateController(Node):
    def __init__(self):
        super().__init__("state_controller")
        self.state_sub = self.create_subscription(String, "/ws_state", self.state_callback, 10)
        self.state_service = self.create_client(ControlWorld, "/world/empty/control")
        while not self.state_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {self.state_service} service to become available...')
        self.get_logger().info(f'Connected to {self.state_service}')

    def state_callback(self, msg):
        self.get_logger().info(f"Received state command: {msg.data}")
        match msg.data:
            case "reset_env":
                self.get_logger().info("Resetting world...")
                request = ControlWorld.Request()
                control = WorldControl()
                reset = WorldReset()
                reset.all = True
                control.reset = reset
                request.world_control = control
                future = self.state_service.call_async(request)
                # rclpy.spin_until_future_complete(self, future)
                # if future.result() is not None:
                #     self.get_logger().info("World reset successfully.")
                # else:
                #     self.get_logger().error("Failed to reset world.")
            case "episode_started":
                self.get_logger().info("Episode started.")
            case "episode_finished":
                self.get_logger().info("Episode finished.")

def main(args=None):
    rclpy.init(args=args)
    state_controller = StateController()
    rclpy.spin(state_controller)
    state_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()