import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class TwistToMultiArray(Node):
    def __init__(self):
        super().__init__('twist_to_multiarray')
        self.subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/base_controller/commands',
            10
        )
    def twist_callback(self, msg):
        multi_array_msg = Float64MultiArray()
        multi_array_msg.data = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.publisher.publish(multi_array_msg)

def main(args=None):
    rclpy.init(args=args)
    twist_to_multiarray = TwistToMultiArray()
    rclpy.spin(twist_to_multiarray)
    twist_to_multiarray.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()