#!/usr/bin/env python3
"""
Sensor Data Recorder Node for TidyBot++

Records contact forces and drawer handle state to CSV files in a format
compatible with Isaac Lab's data collection output.

Subscribes to:
  - /tidybot/contact/left_finger (geometry_msgs/WrenchStamped) - XYZ force
  - /tidybot/contact/right_finger (geometry_msgs/WrenchStamped) - XYZ force
  - /tidybot/drawer/state (std_msgs/Float64MultiArray) - drawer handle XYZ pos/vel/acc

Services:
  - /sensor_recorder/start (std_srvs/Empty)
  - /sensor_recorder/stop (std_srvs/Empty)
  - /sensor_recorder/save (std_srvs/SetBool)

Output CSV format:
  Time(s), Handle_x, Handle_y, Handle_z, Handle_vx, Handle_vy, Handle_vz,
  Handle_ax, Handle_ay, Handle_az, Left_Fx, Left_Fy, Left_Fz,
  Right_Fx, Right_Fy, Right_Fz
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty, SetBool
import csv
import os
from datetime import datetime
from collections import deque
import threading


class SensorDataRecorder(Node):
    """Records contact forces and sensor data to CSV."""

    def __init__(self):
        super().__init__('sensor_data_recorder')

        # Parameters
        self.declare_parameter('output_dir', './sensor_data/')
        self.declare_parameter('record_rate', 20.0)  # Hz
        
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.record_rate = self.get_parameter('record_rate').get_parameter_value().double_value

        # State
        self.is_recording = False
        self.data_buffer = []
        self.lock = threading.Lock()
        self.start_time = None
        
        # Latest sensor readings
        self.left_force = [0.0, 0.0, 0.0]
        self.right_force = [0.0, 0.0, 0.0]
        # Drawer handle state: [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z]
        self.handle_state = [0.0] * 9

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribers for WrenchStamped contact forces (XYZ)
        self.left_contact_sub = self.create_subscription(
            WrenchStamped,
            '/tidybot/contact/left_finger',
            self._left_contact_callback,
            sensor_qos
        )
        self.right_contact_sub = self.create_subscription(
            WrenchStamped,
            '/tidybot/contact/right_finger',
            self._right_contact_callback,
            sensor_qos
        )
        self.drawer_state_sub = self.create_subscription(
            Float64MultiArray,
            '/tidybot/drawer/state',
            self._drawer_state_callback,
            sensor_qos
        )

        # Services
        self.start_srv = self.create_service(
            Empty,
            '/sensor_recorder/start',
            self._start_recording_callback
        )
        self.stop_srv = self.create_service(
            Empty,
            '/sensor_recorder/stop',
            self._stop_recording_callback
        )
        self.save_srv = self.create_service(
            SetBool,
            '/sensor_recorder/save',
            self._save_recording_callback
        )

        # Recording timer
        self.record_timer = self.create_timer(
            1.0 / self.record_rate,
            self._record_callback
        )

        self.get_logger().info(
            f'SensorDataRecorder initialized. Output dir: {self.output_dir}, Rate: {self.record_rate}Hz'
        )

    def _left_contact_callback(self, msg: WrenchStamped):
        """Update left finger contact force (XYZ)."""
        self.left_force = [
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z
        ]

    def _right_contact_callback(self, msg: WrenchStamped):
        """Update right finger contact force (XYZ)."""
        self.right_force = [
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z
        ]

    def _drawer_state_callback(self, msg: Float64MultiArray):
        """Update drawer handle state from /tidybot/drawer/state."""
        if len(msg.data) >= 9:
            self.handle_state = list(msg.data[:9])

    def _record_callback(self):
        """Record current sensor values if recording is active."""
        if not self.is_recording:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time if self.start_time else 0.0

        row = {
            'Time(s)': elapsed_time,
            'Handle_x': self.handle_state[0],
            'Handle_y': self.handle_state[1],
            'Handle_z': self.handle_state[2],
            'Handle_vx': self.handle_state[3],
            'Handle_vy': self.handle_state[4],
            'Handle_vz': self.handle_state[5],
            'Handle_ax': self.handle_state[6],
            'Handle_ay': self.handle_state[7],
            'Handle_az': self.handle_state[8],
            'Left_Fx': self.left_force[0],
            'Left_Fy': self.left_force[1],
            'Left_Fz': self.left_force[2],
            'Right_Fx': self.right_force[0],
            'Right_Fy': self.right_force[1],
            'Right_Fz': self.right_force[2],
        }

        with self.lock:
            self.data_buffer.append(row)

    def _start_recording_callback(self, request, response):
        """Start recording sensor data."""
        self.is_recording = True
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        with self.lock:
            self.data_buffer = []
        self.get_logger().info('Started recording sensor data')
        return response

    def _stop_recording_callback(self, request, response):
        """Stop recording (data remains in buffer)."""
        self.is_recording = False
        self.get_logger().info(f'Stopped recording. Buffer has {len(self.data_buffer)} samples')
        return response

    def _save_recording_callback(self, request, response):
        """Save or discard recorded data."""
        if request.data:
            # Save
            success = self._save_to_csv()
            response.success = success
            response.message = 'Saved sensor data' if success else 'Failed to save'
        else:
            # Discard
            with self.lock:
                self.data_buffer = []
            response.success = True
            response.message = 'Discarded sensor data'
            self.get_logger().info('Discarded sensor data')
        return response

    def _save_to_csv(self) -> bool:
        """Save buffered data to CSV file."""
        if not self.data_buffer:
            self.get_logger().warn('No data to save')
            return False

        # Create output directory if needed
        os.makedirs(self.output_dir, exist_ok=True)

        # Generate timestamped filename
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'sensor_data_{timestamp}.csv'
        filepath = os.path.join(self.output_dir, filename)

        try:
            with self.lock:
                with open(filepath, 'w', newline='') as f:
                    if self.data_buffer:
                        writer = csv.DictWriter(f, fieldnames=self.data_buffer[0].keys())
                        writer.writeheader()
                        writer.writerows(self.data_buffer)
                
                sample_count = len(self.data_buffer)
                self.data_buffer = []

            self.get_logger().info(f'Saved {sample_count} samples to {filepath}')
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to save: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = SensorDataRecorder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save any remaining data
        if node.data_buffer:
            node.get_logger().info('Saving remaining data before shutdown...')
            node._save_to_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
