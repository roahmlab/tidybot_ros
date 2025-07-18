import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time
import numpy as np

from tidybot_driver.base import Vehicle, CONTROL_PERIOD
import os

class BaseServer(Node):
    def __init__(self):
        super().__init__('tidybot_base_server')
        self.vehicle = Vehicle()
        self.get_logger().info('Tidybot base server started')

        # Create a subscription to the command topic
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/tidybot/base/commands',
            self.cmd_callback,
            10
        )

        self.state_pub = self.create_publisher(
            JointState,
            '/tidybot/base/joint_states',
            10
        )

        self.clock = self.get_clock()

        self.control_timer = self.create_timer(CONTROL_PERIOD, self.control_callback)

    def cmd_callback(self, msg):
        # Convert the incoming message to a numpy array
        target = np.array(msg.data, dtype=np.float64)
        # Log the received command
        self.get_logger().info(f'Received command: {target}')

        # Send the command to the vehicle
        self.vehicle.set_target_position(target)

    def control_callback(self):
        self.vehicle.update_state()
        joint_state = JointState()
        joint_state.header.stamp = self.clock.now().to_msg()
        joint_state.name = ['joint_x', 'joint_y', 'joint_th']
        joint_state.position = self.vehicle.get_position()
        self.state_pub.publish(joint_state)
        self.vehicle.update_control()

def main(args=None):
    os.environ['CTR_TARGET'] = 'Hardware'  # pylint: disable=wrong-import-position
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

if __name__ == '__main__':
    main()