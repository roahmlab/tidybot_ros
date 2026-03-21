import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np
import time
import csv

class ActionPlaybackDeployer(Node):
    def __init__(self):
        super().__init__('action_playback_deployer')

        # --- Configuration ---
        self.csv_path = '2026-03-14_23-03-26.csv'
        self.control_rate_hz = 60.0 
        self.action_scale = 0.005
        
        self.arm_joint_names = [f'joint_{i}' for i in range(1, 8)]
        
        # Open-loop position accumulator. 
        # WARNING: This assumes the robot starts at exactly 0.0 on all joints.
        self.simulated_arm_pos = np.zeros(7, dtype=np.float32)

        # --- Publishers ---
        self.arm_cmd_pub = self.create_publisher(JointState, '/tidybot/hardware/arm/commands', 10)
        self.gripper_cmd_pub = self.create_publisher(Float64, '/tidybot/hardware/gripper/commands', 10)

        self.get_logger().info(f"Action Playback Node Initialized. Reading {self.csv_path}")

    def run_playback(self):
        """Reads ONLY the actions from the CSV and plays them blindly at 60Hz."""
        self.get_logger().info("STAND BY ROBOT. Starting blind open-loop playback in 3s...")
        time.sleep(3.0)

        try:
            with open(self.csv_path, mode='r') as f:
                reader = csv.reader(f)
                next(reader) # skip header

                for row_idx, row in enumerate(reader):
                    if not rclpy.ok(): break

                    # 1. Extract exactly the last 8 columns (indices 40 to 47)
                    raw_actions = np.array(row[40:48], dtype=np.float32)

                    # 2. Process Actions
                    arm_deltas = raw_actions[:7]
                    policy_wants_closed = raw_actions[7] <= 0.0
                    gripper_cmd = 1.0 if policy_wants_closed else 0.0

                    # 3. Accumulate deltas to create the absolute target position
                    self.simulated_arm_pos += (arm_deltas * self.action_scale)

                    # 4. Publish
                    self.publish_to_hardware(self.simulated_arm_pos, gripper_cmd)

                    # 5. Maintain Control Frequency
                    time.sleep(1.0 / self.control_rate_hz)
                    
                    if row_idx % 60 == 0:
                        self.get_logger().info(f"Playing row {row_idx}...")

        except Exception as e:
            self.get_logger().error(f"Playback Failed: {e}")
        finally:
            self.get_logger().info("Playback finished.")

    def publish_to_hardware(self, arm_pos, gripper_val):
        arm_msg = JointState()
        arm_msg.header.stamp = self.get_clock().now().to_msg()
        arm_msg.name = self.arm_joint_names
        arm_msg.position = arm_pos.tolist()
        self.arm_cmd_pub.publish(arm_msg)

        gripper_msg = Float64()
        gripper_msg.data = float(gripper_val)
        self.gripper_cmd_pub.publish(gripper_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ActionPlaybackDeployer()
    
    # Run the blocking loop directly
    node.run_playback()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()