import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
import csv
import math

class CSVSumPublisherNode(Node):
    def __init__(self):
        super().__init__('csv_sum_publisher_node')

        self.arm_pub = self.create_publisher(Pose, 'tidybot/arm/command', 10)
        self.gripper_pub = self.create_publisher(Float64, 'tidybot/gripper/command', 10)
        self.timer = self.create_timer(0.07, self.timer_callback)

        self.csv_file_path = '/home/yuandi/tidybot_platform/eef_poses.csv'
        self.csv_reader = self.load_csv(self.csv_file_path)
        self.get_logger().info(f'Loaded CSV: {self.csv_file_path}')

    def load_csv(self, path):
        try:
            file = open(path, 'r')
            reader = csv.reader(file)
            return iter(reader)
        except Exception as e:
            self.get_logger().error(f"Failed to open CSV: {e}")
            self.destroy_node()

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
             math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
             math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return qx, qy, qz, qw

    def timer_callback(self):
        try:
            row = next(self.csv_reader)
            values = [float(x) for x in row]

            if len(values) < 7:
                raise ValueError("Expected at least 7 values per row (XYZRPY + gripper)")

            x, y, z, roll, pitch, yaw = values[:6]
            qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

            pose_msg = Pose()
            pose_msg.position.x = x
            pose_msg.position.y = y
            pose_msg.position.z = z
            pose_msg.orientation.x = qx
            pose_msg.orientation.y = qy
            pose_msg.orientation.z = qz
            pose_msg.orientation.w = qw

            self.arm_pub.publish(pose_msg)
            self.get_logger().info(f'Published Pose: {pose_msg}')

            gripper_value = values[6]
            gripper_msg = Float64()
            gripper_msg.data = gripper_value
            self.gripper_pub.publish(gripper_msg)
            self.get_logger().info(f'Published Gripper: {gripper_value:.2f}')

        except StopIteration:
            self.get_logger().info('End of CSV file reached. Stopping node.')
            self.destroy_node()
        except Exception as e:
            self.get_logger().error(f'Error reading row: {e}')
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CSVSumPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# ros2 topic pub --rate 10 /tidybot/arm/command geometry_msgs/msg/Pose "{position: {x: 0.3024991, y: -0.054297563, z: 0.5487616}, orientation: {x: 0.703183137270208, y: 0.710732772271335, z: 0.019590555382053117, w: 0.0029346240224369276}}"