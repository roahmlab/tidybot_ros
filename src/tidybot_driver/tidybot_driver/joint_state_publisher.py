import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from message_filters import ApproximateTimeSynchronizer, Subscriber

class TidybotJointStatePublisher(Node):
    def __init__(self):
        super().__init__('tidybot')
        self.declare_parameter('mode', 'full')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.get_logger().info(f"Running in mode: {self.mode}")
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        if self.mode == 'full':
            self.arm_sub = Subscriber(self, JointState, '/tidybot/arm/joint_states')
            self.base_sub = Subscriber(self, JointState, '/tidybot/base/joint_states')
            self.ts = ApproximateTimeSynchronizer([self.arm_sub, self.base_sub], queue_size=10, slop=0.05)
            self.ts.registerCallback(self.callback_full)

        elif self.mode == 'arm_only':
            self.arm_sub = self.create_subscription(JointState, '/tidybot/arm/joint_states', self.callback_arm, 10)

        elif self.mode == 'base_only':
            self.base_sub = self.create_subscription(JointState, '/tidybot/base/joint_states', self.callback_base, 10)

        else:
            self.get_logger().error(f"Invalid joint_mode: {self.mode}. Use 'arm_only', 'base_only', or 'full'.")

    def callback_full(self, arm_joint_state, base_joint_state):
        merged = JointState()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.name = arm_joint_state.name + base_joint_state.name
        merged.position = arm_joint_state.position + base_joint_state.position
        merged.velocity = arm_joint_state.velocity + base_joint_state.velocity
        merged.effort = arm_joint_state.effort + base_joint_state.effort
        self.joint_state_pub.publish(merged)
        
    def callback_arm(self, arm_joint_state):
        # Add fixed base joints
        arm_joint_state.header.stamp = self.get_clock().now().to_msg()
        arm_joint_state.name.extend(['joint_x', 'joint_y', 'joint_th'])
        arm_joint_state.position.extend([0.0, 0.0, 0.0])
        self.joint_state_pub.publish(arm_joint_state)

    def callback_base(self, base_joint_state):
        # Add fixed arm joints
        base_joint_state.header.stamp = self.get_clock().now().to_msg()
        base_joint_state.name.extend([
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6', 'joint_7',
            'left_outer_knuckle_joint'
        ])
        base_joint_state.position.extend([0.0] * 8)
        self.joint_state_pub.publish(base_joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = TidybotJointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()