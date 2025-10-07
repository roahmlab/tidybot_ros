import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Empty
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
import time
import numpy as np

from tidybot_driver.base import Vehicle, CONTROL_PERIOD
import os


class BaseServer(Node):
    def __init__(self):
        super().__init__("tidybot_base_server")
        self.vehicle = Vehicle()
        self.get_logger().info("Tidybot base server started")

        # Create a subscription to the command topic
        self.cmd_sub = self.create_subscription(
            Float64MultiArray, "/tidybot/hardware/base/commands", self.cmd_callback, 1
        )

        self.state_pub = self.create_publisher(
            JointState, "/tidybot/hardware/base/joint_states", 10
        )
        self.base_state_timer = self.create_timer(0.05, self.publish_base_state)

        self.reset_srv = self.create_service(Empty, "/tidybot/hardware/base/reset", self.reset)
        self.vehicle.start_control()

    def cmd_callback(self, msg):
        # Convert the incoming message to a numpy array
        target = np.array(msg.data, dtype=np.float64)

        # Send the command to the vehicle
        self.vehicle.set_target_position(target)

    def reset(self, request, response):
        if self.vehicle is not None:
            if self.vehicle.control_loop_running:
                self.vehicle.stop_control()
                self.get_logger().info("Stopping control...")
        self.get_logger().info("Resetting base...")
        self.vehicle = Vehicle()  # Reinitialize the vehicle
        self.vehicle.start_control()
        self.get_logger().info("Base reset complete")
        return response

    def publish_base_state(self):
        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = ["joint_x", "joint_y", "joint_th"]
        state_msg.position = self.vehicle.get_position()
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
