import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Empty
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
import time
import numpy as np

from tidybot_driver.base import Vehicle, CONTROL_PERIOD
import os

GREEN = "\x1b[32m"
YELLOW = "\x1b[33m"
RED = "\x1b[31m"
RESET = "\x1b[0m"
BOLD = "\x1b[1m"

class BaseServer(Node):
    def __init__(self):
        super().__init__("tidybot_base_server")
        self.vehicle = Vehicle()
        self.declare_parameter("mode", "position")
        self.declare_parameter("frame", "local")

        self.drive_mode = self.get_parameter("mode").get_parameter_value().string_value
        if self.drive_mode not in ["position", "velocity"]:
            self.get_logger().warn(
                f"{YELLOW}Invalid base mode: {self.drive_mode}, defaulting to 'position'{RESET}"
            )
            self.drive_mode = "position"
        self.frame = self.get_parameter("frame").get_parameter_value().string_value
        if self.frame not in ["local", "global"]:
            self.get_logger().warn(
                f"{YELLOW}Invalid base frame: {self.frame}, defaulting to 'local'{RESET}"
            )
            self.frame = "local"

        # Create a subscription to the command topic
        if self.drive_mode == "position":
            self.cmd_sub = self.create_subscription(
                Float64MultiArray, "/tidybot/hardware/base/target_pos", self.cmd_callback, 1
            )
        else:  # velocity mode
            self.cmd_sub = self.create_subscription(
                Float64MultiArray, "/tidybot/hardware/base/target_vel", self.cmd_callback, 1
            )

        self.state_pub = self.create_publisher(
            JointState, "/tidybot/hardware/base/joint_states", 10
        )
        self.base_state_timer = self.create_timer(0.01, self.publish_base_state)

        self.reset_srv = self.create_service(Empty, "/tidybot/hardware/base/reset", self.reset)
        self.vehicle.start_control()

        self.get_logger().info(f"{GREEN}{BOLD}Base server initialized successfully in {self.drive_mode} mode{RESET}")

    def cmd_callback(self, msg):
        if self.drive_mode == "position":
            # Convert the incoming message to a numpy array
            target = np.array(msg.data, dtype=np.float64)
            # Send the command to the vehicle
            self.vehicle.set_target_position(target)
        else:  # velocity mode
            target = np.array(msg.data, dtype=np.float64)
            self.vehicle.set_target_velocity(target, self.frame)

    def reset(self, request, response):
        if self.vehicle is not None:
            if self.vehicle.control_loop_running:
                self.vehicle.stop_control()
        self.get_logger().info(f"{GREEN}Resetting base...{RESET}")
        self.vehicle = Vehicle()  # Reinitialize the vehicle
        self.vehicle.start_control()
        self.get_logger().info(f"{GREEN}Base reset complete{RESET}")
        return response

    def publish_base_state(self):
        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = ["joint_x", "joint_y", "joint_th"]
        state_msg.position = self.vehicle.get_position()
        # self.get_logger().info(f"Publishing base state: {state_msg.position}")
        self.state_pub.publish(state_msg)

def main(args=None):
    os.environ["CTR_TARGET"] = "Hardware"  # pylint: disable=wrong-import-position
    rclpy.init(args=args)
    base_server = BaseServer()
    try:
        # os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO)))
        rclpy.spin(base_server)
    # except PermissionError:
    #     print('Failed to set real-time scheduling policy, please edit /etc/security/limits.d/99-realtime.conf')
    #     pass
    except KeyboardInterrupt:
        pass
    finally:
        base_server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
