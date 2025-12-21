#!/usr/bin/env python3
"""
Isaac Sim Bridge Node

Bridges TidyBot's ROS 2 control interfaces to Isaac Sim's /joint_command topic.

Subscribed Topics:
    /tidybot_base_pos_controller/commands (Float64MultiArray) - Base position [x, y, theta]
    /tidybot_base_vel_controller/commands (Float64MultiArray) - Base velocity [vx, vy, omega]
    /gen3_7dof_controller/joint_trajectory (JointTrajectory) - Arm joint trajectory
    /robotiq_2f_85_controller/commands (Float64MultiArray) - Gripper position
    /tidybot/gripper/commands (Float64) - Simple gripper command

Published Topics:
    /joint_command (JointState) - Combined joint commands for Isaac Sim

The gripper uses mimic joints which are expanded here since Isaac Sim doesn't
support mimic joints natively.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class IsaacSimBridge(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('use_velocity_control', False)
        
        publish_rate = self.get_parameter('publish_rate').value
        self.use_velocity_control = self.get_parameter('use_velocity_control').value
        
        # Joint configurations
        self.base_joints = ['joint_x', 'joint_y', 'joint_th']
        self.arm_joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 
                          'joint_5', 'joint_6', 'joint_7']
        
        # Gripper joints with their mimic multipliers relative to the leader joint
        # left_outer_knuckle_joint is the leader
        self.gripper_leader = 'left_outer_knuckle_joint'
        self.gripper_mimic_config = {
            'left_outer_knuckle_joint': 1.0,    # Leader
            'left_inner_knuckle_joint': 1.0,    # Same direction
            'left_inner_finger_joint': -1.0,    # Opposite direction
            'right_outer_knuckle_joint': 1.0,   # Same direction
            'right_inner_knuckle_joint': 1.0,   # Same direction
            'right_inner_finger_joint': -1.0,   # Opposite direction
        }
        
        # Current target state
        self.base_positions = [0.0, 0.0, 0.0]
        self.base_velocities = [0.0, 0.0, 0.0]
        self.arm_positions = [0.0, -0.35, 3.14, -2.36, 0.0, -1.13, 1.57]  # Home position
        self.gripper_position = 0.0  # Leader joint position (0 = open, ~0.8 = closed)
        
        # Subscribers
        # Base position controller
        self.base_pos_sub = self.create_subscription(
            Float64MultiArray,
            '/tidybot_base_pos_controller/commands',
            self.base_pos_callback,
            10
        )
        
        # Base velocity controller
        self.base_vel_sub = self.create_subscription(
            Float64MultiArray,
            '/tidybot_base_vel_controller/commands',
            self.base_vel_callback,
            10
        )
        
        # Arm trajectory controller
        self.arm_traj_sub = self.create_subscription(
            JointTrajectory,
            '/gen3_7dof_controller/joint_trajectory',
            self.arm_traj_callback,
            10
        )
        
        # Gripper controller (ros2_control style)
        self.gripper_sub = self.create_subscription(
            Float64MultiArray,
            '/robotiq_2f_85_controller/commands',
            self.gripper_callback,
            10
        )
        
        # Simple gripper command (from teleop)
        self.gripper_simple_sub = self.create_subscription(
            Float64,
            '/tidybot/gripper/commands',
            self.gripper_simple_callback,
            10
        )
        
        # Subscribe to Isaac Sim joint states for feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publisher
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_command', 10)
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_command)
        
        self.get_logger().info(
            f'Isaac Sim Bridge started at {publish_rate} Hz\n'
            f'  Base control: {"velocity" if self.use_velocity_control else "position"}\n'
            f'  Listening to:\n'
            f'    - /tidybot_base_pos_controller/commands\n'
            f'    - /tidybot_base_vel_controller/commands\n'
            f'    - /gen3_7dof_controller/joint_trajectory\n'
            f'    - /robotiq_2f_85_controller/commands\n'
            f'    - /tidybot/gripper/commands\n'
            f'  Publishing to: /joint_command'
        )

    def joint_state_callback(self, msg: JointState):
        """Update current positions from Isaac Sim joint states (for feedback/debug)."""
        # This can be used to track actual positions if needed
        pass

    def base_pos_callback(self, msg: Float64MultiArray):
        """Handle base position commands [x, y, theta]."""
        if len(msg.data) >= 3:
            self.base_positions = list(msg.data[:3])
            self.get_logger().debug(f'Base pos: {self.base_positions}')

    def base_vel_callback(self, msg: Float64MultiArray):
        """Handle base velocity commands [vx, vy, omega]."""
        if len(msg.data) >= 3:
            self.base_velocities = list(msg.data[:3])
            self.get_logger().debug(f'Base vel: {self.base_velocities}')

    def arm_traj_callback(self, msg: JointTrajectory):
        """Handle arm joint trajectory commands."""
        if not msg.points:
            return
        
        # Use the first (or last) trajectory point as the target
        point = msg.points[-1] if len(msg.points) > 1 else msg.points[0]
        
        for i, name in enumerate(msg.joint_names):
            if name in self.arm_joints and i < len(point.positions):
                idx = self.arm_joints.index(name)
                self.arm_positions[idx] = point.positions[i]
        
        self.get_logger().debug(f'Arm positions: {self.arm_positions}')

    def gripper_callback(self, msg: Float64MultiArray):
        """Handle gripper command from ros2_control topic (leader joint position)."""
        if msg.data:
            self.gripper_position = msg.data[0]
            self.get_logger().debug(f'Gripper pos: {self.gripper_position}')

    def gripper_simple_callback(self, msg: Float64):
        """Handle simple Float64 gripper command."""
        self.gripper_position = msg.data
        self.get_logger().debug(f'Gripper pos (simple): {self.gripper_position}')

    def publish_command(self):
        """Publish combined joint command to Isaac Sim."""
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        
        # Build joint names and positions
        cmd.name = []
        cmd.position = []
        cmd.velocity = []
        
        # Base joints
        cmd.name.extend(self.base_joints)
        if self.use_velocity_control:
            cmd.position.extend([0.0, 0.0, 0.0])  # Position not used for velocity control
            cmd.velocity.extend(self.base_velocities)
        else:
            cmd.position.extend(self.base_positions)
            cmd.velocity.extend([0.0, 0.0, 0.0])
        
        # Arm joints
        cmd.name.extend(self.arm_joints)
        cmd.position.extend(self.arm_positions)
        cmd.velocity.extend([0.0] * len(self.arm_joints))
        
        # Gripper joints - expand mimic joints
        for joint_name, multiplier in self.gripper_mimic_config.items():
            cmd.name.append(joint_name)
            cmd.position.append(self.gripper_position * multiplier)
            cmd.velocity.append(0.0)
        
        self.joint_cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = IsaacSimBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

