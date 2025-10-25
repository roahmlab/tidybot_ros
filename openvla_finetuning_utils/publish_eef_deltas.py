#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import csv
import time
import os

class DeltaPublisher(Node):
    def __init__(self, csv_path: str, loop: bool = False):
        super().__init__('delta_publisher')
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/tidybot/arm/delta_commands',
            10
        )

        # Load all rows from the CSV file at startup
        if not os.path.exists(csv_path):
            raise FileNotFoundError(f"CSV file not found: {csv_path}")

        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            self.data = [list(map(float, row)) for row in reader if row]

        if not self.data:
            raise ValueError("CSV file is empty or malformed")

        self.loop = loop
        self.index = 0
        self.timer_period = 1.0 / 15.0  # 15 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(f"Loaded {len(self.data)} delta commands from {csv_path}")

    def timer_callback(self):
        if self.index >= len(self.data):
            if self.loop:
                self.index = 0
            else:
                self.get_logger().info("Finished publishing all commands.")
                rclpy.shutdown()
                return

        msg = Float64MultiArray()
        msg.data = self.data[self.index]
        self.publisher_.publish(msg)
        self.get_logger().debug(f"Published line {self.index}: {msg.data}")
        self.index += 1


def main(args=None):
    rclpy.init(args=args)

    # Adjust path to your CSV file if needed
    csv_path = os.path.join(os.getcwd(), 'eef_deltas.csv')

    try:
        node = DeltaPublisher(csv_path=csv_path, loop=False)
        rclpy.spin(node)
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
