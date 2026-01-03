#!/usr/bin/env python3
"""
Isaac Sim Bridge Node

Bridges TidyBot's ROS 2 control interfaces to Isaac Sim's /joint_command topic
and provides simulation control services for resetting the environment.

Subscribed Topics:
    /tidybot_base_pos_controller/commands (Float64MultiArray) - Base position [x, y, theta]
    /tidybot_base_vel_controller/commands (Float64MultiArray) - Base velocity [vx, vy, omega]
    /gen3_7dof_controller/joint_trajectory (JointTrajectory) - Arm joint trajectory
    /robotiq_2f_85_controller/commands (Float64MultiArray) - Gripper position
    /tidybot/gripper/commands (Float64) - Simple gripper command
    /joint_states (JointState) - Current joint states from Isaac Sim

Published Topics:
    /joint_command (JointState) - Combined joint commands for Isaac Sim

Provided Services:
    /isaac_sim/reset_env (std_srvs/Empty) - Reset Isaac Sim environment and drive to initial state

Simulation Control:
    Uses simulation_interfaces package to control Isaac Sim via:
    - /reset_simulation - Reset all entities to initial state

Gripper Control:
    Uses Isaac Sim's Mimic Joint API for the Robotiq 2F-85 gripper.
    Only the leader joint (left_outer_knuckle_joint) is commanded - other
    joints use PhysxMimicJointAPI to follow the leader automatically.

Initial joint positions from tidybot.ros2_control.xacro:
    Base: joint_x=0.0, joint_y=0.0, joint_th=0.0
    Arm:  joint_1=0.0, joint_2=-0.35, joint_3=3.141522, 
          joint_4=-2.36, joint_5=0.0, joint_6=-1.13, joint_7=1.574186
    Gripper: outer knuckle joints at 0.0 (inner joints passive)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64, Float64MultiArray
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import time

# Import Isaac Sim simulation control interfaces
from simulation_interfaces.srv import ResetSimulation
from simulation_interfaces.msg import Result


# Initial joint positions from tidybot.ros2_control.xacro
INITIAL_JOINT_POSITIONS = {
    # Base joints
    "joint_x": 0.0,
    "joint_y": 0.0,
    "joint_th": 0.0,
    # Arm joints (gen3_7dof)
    "joint_1": 0.0,
    "joint_2": -0.35,
    "joint_3": 3.141522,
    "joint_4": -2.36,
    "joint_5": 0.0,
    "joint_6": -1.13,
    "joint_7": 1.574186,
    # Gripper - only leader joint (others use Mimic Joint API)
    "left_outer_knuckle_joint": 0.0,
}

# Joint position tolerance for checking if robot reached initial state
POSITION_TOLERANCE = 0.05  # radians/meters


class IsaacSimBridge(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge')
        
        # Use reentrant callback group to allow service calls within callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Declare parameters
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('use_velocity_control', False)
        self.declare_parameter('reset_timeout', 10.0)  # seconds to wait for reset
        
        publish_rate = self.get_parameter('publish_rate').value
        self.use_velocity_control = self.get_parameter('use_velocity_control').value
        self.reset_timeout = self.get_parameter('reset_timeout').value
        
        # Joint configurations
        self.base_joints = ['joint_x', 'joint_y', 'joint_th']
        self.arm_joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 
                          'joint_5', 'joint_6', 'joint_7']
        
        # Gripper configuration: only leader joint is commanded
        # Other joints use Mimic Joint API in Isaac Sim to follow the leader
        self.gripper_joints = {
            'left_outer_knuckle_joint': 1.0,    # Leader joint only
        }
        
        # Current target state (initialize to home position)
        self.base_positions = [
            INITIAL_JOINT_POSITIONS['joint_x'],
            INITIAL_JOINT_POSITIONS['joint_y'],
            INITIAL_JOINT_POSITIONS['joint_th']
        ]
        self.base_velocities = [0.0, 0.0, 0.0]
        self.arm_positions = [
            INITIAL_JOINT_POSITIONS['joint_1'],
            INITIAL_JOINT_POSITIONS['joint_2'],
            INITIAL_JOINT_POSITIONS['joint_3'],
            INITIAL_JOINT_POSITIONS['joint_4'],
            INITIAL_JOINT_POSITIONS['joint_5'],
            INITIAL_JOINT_POSITIONS['joint_6'],
            INITIAL_JOINT_POSITIONS['joint_7'],
        ]
        self.gripper_position = 0.0  # Leader joint position (0 = open, ~0.8 = closed)
        
        # Current actual joint states from Isaac Sim
        self.current_joint_states = {}
        
        # Flag to indicate reset is in progress
        self.reset_in_progress = False
        
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
        
        # Isaac Sim simulation control client
        self.reset_sim_cli = self.create_client(
            ResetSimulation, 
            '/reset_simulation',
            callback_group=self.callback_group
        )
        
        # Reset service provided by this bridge
        self.reset_env_srv = self.create_service(
            Empty,
            '/isaac_sim/reset_env',
            self.reset_env_callback,
            callback_group=self.callback_group
        )
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_command)
        
        # Check if Isaac Sim simulation control is available
        self.isaac_sim_available = self.reset_sim_cli.wait_for_service(timeout_sec=5.0)
        if not self.isaac_sim_available:
            self.get_logger().warn(
                "Isaac Sim /reset_simulation service not available. "
                "Reset functionality will be limited."
            )
        
        self.get_logger().info(
            f'Isaac Sim Bridge started at {publish_rate} Hz\n'
            f'  Base control: {"velocity" if self.use_velocity_control else "position"}\n'
            f'  Isaac Sim control: {"available" if self.isaac_sim_available else "NOT available"}\n'
            f'  Listening to:\n'
            f'    - /tidybot_base_pos_controller/commands\n'
            f'    - /tidybot_base_vel_controller/commands\n'
            f'    - /gen3_7dof_controller/joint_trajectory\n'
            f'    - /robotiq_2f_85_controller/commands\n'
            f'    - /tidybot/gripper/commands\n'
            f'  Publishing to: /joint_command\n'
            f'  Services:\n'
            f'    - /isaac_sim/reset_env'
        )

    def joint_state_callback(self, msg: JointState):
        """Update current positions from Isaac Sim joint states."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_states[name] = msg.position[i]

    def base_pos_callback(self, msg: Float64MultiArray):
        """Handle base position commands [x, y, theta]."""
        if self.reset_in_progress:
            return  # Ignore commands during reset
        if len(msg.data) >= 3:
            self.base_positions = list(msg.data[:3])
            self.get_logger().debug(f'Base pos: {self.base_positions}')

    def base_vel_callback(self, msg: Float64MultiArray):
        """Handle base velocity commands [vx, vy, omega]."""
        if self.reset_in_progress:
            return  # Ignore commands during reset
        if len(msg.data) >= 3:
            self.base_velocities = list(msg.data[:3])
            self.get_logger().debug(f'Base vel: {self.base_velocities}')

    def arm_traj_callback(self, msg: JointTrajectory):
        """Handle arm joint trajectory commands."""
        if self.reset_in_progress:
            return  # Ignore commands during reset
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
        if self.reset_in_progress:
            return  # Ignore commands during reset
        if msg.data:
            self.gripper_position = msg.data[0]
            self.get_logger().debug(f'Gripper pos: {self.gripper_position}')

    def gripper_simple_callback(self, msg: Float64):
        """Handle simple Float64 gripper command."""
        if self.reset_in_progress:
            return  # Ignore commands during reset
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
        
        # Gripper - command only leader joint (left_outer_knuckle_joint)
        # Other joints follow via Mimic Joint API in Isaac Sim
        for joint_name, multiplier in self.gripper_joints.items():
            cmd.name.append(joint_name)
            cmd.position.append(self.gripper_position * multiplier)
            cmd.velocity.append(0.0)
        
        self.joint_cmd_pub.publish(cmd)

    def set_initial_positions(self):
        """Set internal state to initial joint positions."""
        self.base_positions = [
            INITIAL_JOINT_POSITIONS['joint_x'],
            INITIAL_JOINT_POSITIONS['joint_y'],
            INITIAL_JOINT_POSITIONS['joint_th']
        ]
        self.base_velocities = [0.0, 0.0, 0.0]
        self.arm_positions = [
            INITIAL_JOINT_POSITIONS['joint_1'],
            INITIAL_JOINT_POSITIONS['joint_2'],
            INITIAL_JOINT_POSITIONS['joint_3'],
            INITIAL_JOINT_POSITIONS['joint_4'],
            INITIAL_JOINT_POSITIONS['joint_5'],
            INITIAL_JOINT_POSITIONS['joint_6'],
            INITIAL_JOINT_POSITIONS['joint_7'],
        ]
        self.gripper_position = 0.0

    def check_joints_at_initial(self):
        """Check if all joints have reached initial positions."""
        for joint_name, target_pos in INITIAL_JOINT_POSITIONS.items():
            current_pos = self.current_joint_states.get(joint_name)
            if current_pos is None:
                return False  # Joint state not received yet
            if abs(current_pos - target_pos) > POSITION_TOLERANCE:
                return False
        return True

    def reset_env_callback(self, request, response):
        """Handle reset environment service request."""
        self.get_logger().info("Reset environment requested...")
        self.reset_in_progress = True
        
        try:
            # Step 1: Call Isaac Sim reset_simulation service
            if self.isaac_sim_available:
                self.get_logger().info("Calling Isaac Sim /reset_simulation...")
                reset_request = ResetSimulation.Request()
                future = self.reset_sim_cli.call_async(reset_request)
                
                # Wait for the reset to complete
                start_time = time.time()
                while not future.done():
                    if time.time() - start_time > 5.0:
                        self.get_logger().error("Timeout waiting for /reset_simulation")
                        break
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                if future.done():
                    result = future.result()
                    if result.result.result == Result.RESULT_OK:
                        self.get_logger().info("Isaac Sim simulation reset successfully")
                    else:
                        self.get_logger().warn(f"Reset returned: {result.result.result}")
            else:
                self.get_logger().warn("Isaac Sim not available, skipping simulation reset")
            
            # Step 2: Set internal state to initial positions
            self.get_logger().info("Setting internal state to initial positions...")
            self.set_initial_positions()
            
            # Step 3: Publish initial joint commands repeatedly until robot reaches position
            self.get_logger().info("Publishing initial joint commands...")
            start_time = time.time()
            reached_initial = False
            
            while time.time() - start_time < self.reset_timeout:
                # Publish command (will use initial positions from set_initial_positions)
                self.publish_command()
                
                # Allow joint states to update
                rclpy.spin_once(self, timeout_sec=0.05)
                
                # Check if robot has reached initial positions
                if self.check_joints_at_initial():
                    reached_initial = True
                    self.get_logger().info("Robot reached initial joint positions!")
                    break
                
                # Continue publishing at ~20Hz
                time.sleep(0.05)
            
            if not reached_initial:
                self.get_logger().warn(
                    f"Timeout ({self.reset_timeout}s) waiting for robot to reach initial positions. "
                    "Robot may not be fully reset."
                )
                # Log which joints haven't reached target
                for joint_name, target_pos in INITIAL_JOINT_POSITIONS.items():
                    current_pos = self.current_joint_states.get(joint_name)
                    if current_pos is not None:
                        error = abs(current_pos - target_pos)
                        if error > POSITION_TOLERANCE:
                            self.get_logger().warn(
                                f"  {joint_name}: current={current_pos:.4f}, "
                                f"target={target_pos:.4f}, error={error:.4f}"
                            )
            
            self.get_logger().info("Isaac Sim environment reset complete!")
            
        except Exception as e:
            self.get_logger().error(f"Reset failed: {e}")
        finally:
            self.reset_in_progress = False
        
        return response


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
