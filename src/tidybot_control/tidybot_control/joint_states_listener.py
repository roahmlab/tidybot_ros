import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatesListener(Node):
    def __init__(self):
        super().__init__('joint_states_listener')
        self.subscription = self.create_subscription(
            JointState,
            '/tidybot/joint_states',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received joint states: {msg.name} - {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    joint_states_listener = JointStatesListener()
    rclpy.spin(joint_states_listener)
    joint_states_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
