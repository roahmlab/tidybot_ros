#!/usr/bin/env python3
"""
Sensor Data Recorder Node for TidyBot++

Records contact forces and other sensor data to CSV files in a format
compatible with Isaac Lab's data collection output.

Subscribes to:
  - /tidybot/contact/left_finger (geometry_msgs/WrenchStamped) - XYZ force
  - /tidybot/contact/right_finger (geometry_msgs/WrenchStamped) - XYZ force
  - /joint_states (sensor_msgs/JointState) - for drawer velocity proxy

Services:
  - /sensor_recorder/start (std_srvs/Empty)
  - /sensor_recorder/stop (std_srvs/Empty)
  - /sensor_recorder/save (std_srvs/SetBool)

Output CSV format matches Isaac Lab:
  Time(s), Left_Fx, Left_Fy, Left_Fz, Right_Fx, Right_Fy, Right_Fz, Drawer_Vx, Drawer_Vy, Drawer_Vz
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
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
        self.declare_parameter('output_dir', '/tmp/sensor_data')
        self.declare_parameter('record_rate', 10.0)  # Hz
        self.declare_parameter('drawer_joint_name', 'top_drawer_joint')
        
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.record_rate = self.get_parameter('record_rate').get_parameter_value().double_value
        self.drawer_joint_name = self.get_parameter('drawer_joint_name').get_parameter_value().string_value

        # State
        self.is_recording = False
        self.data_buffer = []
        self.lock = threading.Lock()
        self.start_time = None
        
        # Latest sensor readings (XYZ force vectors)
        self.left_force = [0.0, 0.0, 0.0]
        self.right_force = [0.0, 0.0, 0.0]
        self.drawer_velocity = [0.0, 0.0, 0.0]  # Approximated from joint velocity
        self.last_drawer_position = None
        self.last_drawer_time = None

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
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
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

    def _joint_state_callback(self, msg: JointState):
        """Extract drawer velocity from joint states."""
        if self.drawer_joint_name in msg.name:
            idx = msg.name.index(self.drawer_joint_name)
            if idx < len(msg.velocity):
                # Drawer is a prismatic joint, so velocity is linear
                # We report it as X velocity (drawer opens along X typically)
                drawer_vel = msg.velocity[idx]
                self.drawer_velocity = [drawer_vel, 0.0, 0.0]
            elif idx < len(msg.position):
                # Approximate velocity from position changes
                current_pos = msg.position[idx]
                current_time = self.get_clock().now().nanoseconds / 1e9
                
                if self.last_drawer_position is not None and self.last_drawer_time is not None:
                    dt = current_time - self.last_drawer_time
                    if dt > 0:
                        vel = (current_pos - self.last_drawer_position) / dt
                        self.drawer_velocity = [vel, 0.0, 0.0]
                
                self.last_drawer_position = current_pos
                self.last_drawer_time = current_time

    def _record_callback(self):
        """Record current sensor values if recording is active."""
        if not self.is_recording:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time if self.start_time else 0.0

        row = {
            'Time(s)': elapsed_time,
            'Left_Fx': self.left_force[0],
            'Left_Fy': self.left_force[1],
            'Left_Fz': self.left_force[2],
            'Right_Fx': self.right_force[0],
            'Right_Fy': self.right_force[1],
            'Right_Fz': self.right_force[2],
            'Drawer_Vx': self.drawer_velocity[0],
            'Drawer_Vy': self.drawer_velocity[1],
            'Drawer_Vz': self.drawer_velocity[2],
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
