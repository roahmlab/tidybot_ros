import rclpy
from rclpy.node import Node
from tidybot_msgs.msg import WSMsg
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32

class RelayNode(Node):
    def __init__(self):
        super().__init__('web_server_relay')
        self.base_publisher = self.create_publisher(Pose, 'ws_base_command', 1)
        self.arm_publisher = self.create_publisher(Pose, 'ws_arm_command', 1)
        self.gripper_publisher = self.create_publisher(Float32, 'ws_gripper_command', 1)
        self.subscription = self.create_subscription(
            WSMsg,
            'ws_commands',
            self.callback,
            10)

    def callback(self, msg):
        if msg.teleop_mode == "gripper":
            msg_out = Float32()
            msg_out.data = msg.gripper_delta
            self.gripper_publisher.publish(msg_out)
            return
        
        target_pose = Pose()
        target_pose.position.x = float(msg.pos_x)
        target_pose.position.y = float(msg.pos_y)
        target_pose.position.z = float(msg.pos_z)
        target_pose.orientation.x = float(msg.or_x)
        target_pose.orientation.y = float(msg.or_y)
        target_pose.orientation.z = float(msg.or_z)
        target_pose.orientation.w = float(msg.or_w)

        if msg.teleop_mode == "base":
            self.base_publisher.publish(target_pose)
        if msg.teleop_mode == "arm":
            self.arm_publisher.publish(target_pose)

def main(args=None):
    rclpy.init(args=args)
    node = RelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
