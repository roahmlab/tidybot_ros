import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import csv

class CSVSumPublisherNode(Node):
    def __init__(self):
        super().__init__('csv_sum_publisher_node')

        self.publisher_ = self.create_publisher(Float64MultiArray, 'tidybot/arm/delta_command', 10)
        self.timer = self.create_timer(1, self.timer_callback)  # 1 second

        self.csv_file_path = '/home/yuandi/tidybot_platform/eef_deltas.csv'
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

    def timer_callback(self):
        rows = []
        try:
            for _ in range(15):
                row = next(self.csv_reader)
                rows.append([float(x) for x in row])
                
            # Transpose rows to columns
            columns = list(zip(*rows))

            # Sum first 6 elements (XYZRPY)
            summed = [sum(columns[i]) for i in range(6)]

            # Average the 7th element (gripper)
            gripper_avg = sum(columns[6]) / len(columns[6])

            # Combine
            summed.append(gripper_avg)

            msg = Float64MultiArray(data=summed)
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published sum of 15 rows: {msg.data}')
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
