# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Isaac Lab Simulation Script for TidyBot with ROS2 Integration.

This script runs the TidyBot simulation in Isaac Lab and bridges it with
the ROS2 stack for policy deployment and visualization.

Features:
- Publishes /joint_states, /clock to ROS2
- Subscribes to trajectory commands from multi_stage_planner
- Publishes camera images via Replicator (wrist and base cameras)
- Logs sensor data (contact forces, drawer state) to CSV

Usage:
    cd isaaclab && ./isaaclab.sh -p tidybot_isaac/scripts/isaac_lab_sim.py \\
        --task Isaac-TidyBot-Drawer-v0 --num_envs 1 --real-time

Then launch the ROS2 stack:
    ros2 launch tidybot_description launch_isaac_lab.launch.py
"""

import argparse
import sys
import os
import time
from datetime import datetime

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Isaac Lab Simulation with ROS2 Bridge.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="Isaac-TidyBot-Drawer-v0", help="Name of the task.")
parser.add_argument("--save_path", type=str, default=None, help="Path to save sensor data CSV.")
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time.")

# append AppLauncher cli args (includes --enable_cameras)
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()

# Handle default save path with timestamp
if args_cli.save_path is None:
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    args_cli.save_path = f"logs/ros2_bridge/sensor_data_{timestamp}.csv"

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Enable ROS2 Bridge Extension
import omni.kit.app
ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)

"""Rest everything follows after Isaac Sim is initialized."""

import numpy as np
import pandas as pd
import torch
import gymnasium as gym

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Image
from trajectory_msgs.msg import JointTrajectory
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

from isaaclab.envs import DirectRLEnvCfg, ManagerBasedRLEnvCfg

from isaaclab_tasks.utils.hydra import hydra_task_config

import tidybot_isaac.tasks  # noqa: F401

# Replicator for camera publishing (optional)
try:
    import omni.replicator.core as rep
    HAS_REPLICATOR = True
except ImportError:
    HAS_REPLICATOR = False
    print("[WARNING] omni.replicator.core not available, camera publishing disabled")


class IsaacLabROS2Node(Node):
    """ROS2 Node that bridges Isaac Lab with the TidyBot ROS2 stack."""
    
    # Joint names matching the URDF
    ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]
    GRIPPER_JOINTS = ["finger_joint", "right_outer_knuckle_joint"]
    BASE_JOINTS = ["joint_x", "joint_y", "joint_th"]
    
    def __init__(self):
        super().__init__('isaac_lab_sim')
        
        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # === Subscribers ===
        # Arm trajectory commands from multi_stage_planner
        self.arm_traj_sub = self.create_subscription(
            JointTrajectory,
            '/gen3_7dof_controller/joint_trajectory',
            self.arm_traj_callback,
            10
        )
        
        # Gripper commands from multi_stage_planner
        self.gripper_sub = self.create_subscription(
            Float64MultiArray,
            '/robotiq_2f_85_controller/commands',
            self.gripper_callback,
            10
        )
        
        # === Publishers ===
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        
        # Camera publishers (will be initialized later if cameras exist)
        self.wrist_camera_pub = self.create_publisher(Image, '/tidybot/camera_wrist/color/raw', 10)
        self.base_camera_pub = self.create_publisher(Image, '/tidybot/camera_base/color/raw', 10)
        
        # === State ===
        self.target_arm_positions = None  # Will be set from robot initial state
        self.target_gripper_positions = [0.0, 0.0]  # [finger_joint, right_outer_knuckle]
        self.last_arm_command_time = time.time()
        self.last_gripper_command_time = time.time()
        
        self.get_logger().info("Isaac Lab ROS2 Node initialized")
    
    def arm_traj_callback(self, msg: JointTrajectory):
        """Handle incoming arm trajectory commands."""
        if not msg.points:
            return
        
        # Get the last point in the trajectory (goal)
        point = msg.points[-1]
        
        # Map joint names to positions
        new_positions = list(self.target_arm_positions) if self.target_arm_positions else [0.0] * 7
        
        for i, name in enumerate(msg.joint_names):
            if name in self.ARM_JOINTS and i < len(point.positions):
                idx = self.ARM_JOINTS.index(name)
                new_positions[idx] = point.positions[i]
        
        self.target_arm_positions = new_positions
        self.last_arm_command_time = time.time()
        self.get_logger().debug(f"Received arm trajectory: {new_positions}")
    
    def gripper_callback(self, msg: Float64MultiArray):
        """Handle incoming gripper commands."""
        if len(msg.data) >= 2:
            self.target_gripper_positions = [msg.data[0], msg.data[1]]
        elif len(msg.data) >= 1:
            # Assume symmetric gripper command
            self.target_gripper_positions = [msg.data[0], msg.data[0]]
        self.last_gripper_command_time = time.time()
        self.get_logger().debug(f"Received gripper command: {self.target_gripper_positions}")
    
    def publish_joint_states(self, robot_articulation, sim_time: float):
        """Publish current joint states to ROS2."""
        msg = JointState()
        msg.header.stamp = self._float_to_stamp(sim_time)
        
        # Get joint positions and velocities from the robot
        joint_pos = robot_articulation.data.joint_pos[0].cpu().numpy()
        joint_vel = robot_articulation.data.joint_vel[0].cpu().numpy()
        joint_names = robot_articulation.joint_names
        
        msg.name = list(joint_names)
        msg.position = joint_pos.tolist()
        msg.velocity = joint_vel.tolist()
        msg.effort = [0.0] * len(joint_names)
        
        self.joint_state_pub.publish(msg)
    
    def publish_clock(self, sim_time: float):
        """Publish simulation clock to ROS2."""
        msg = Clock()
        msg.clock = self._float_to_stamp(sim_time)
        self.clock_pub.publish(msg)
    
    def _float_to_stamp(self, t: float) -> Time:
        """Convert float seconds to ROS2 Time message."""
        stamp = Time()
        stamp.sec = int(t)
        stamp.nanosec = int((t - stamp.sec) * 1e9)
        return stamp
    
    def publish_camera_images(self, scene, sim_time: float):
        """Publish camera images to ROS2 topics."""
        stamp = self._float_to_stamp(sim_time)
        
        # Publish wrist camera
        if "wrist_camera" in scene.sensors:
            try:
                wrist_cam = scene.sensors["wrist_camera"]
                rgb_data = wrist_cam.data.output["rgb"]  # Shape: (num_envs, H, W, C)
                if rgb_data is not None and rgb_data.numel() > 0:
                    # Get env 0, convert to uint8 numpy
                    img_np = rgb_data[0].cpu().numpy()
                    if img_np.dtype != np.uint8:
                        img_np = (img_np * 255).astype(np.uint8)
                    
                    msg = Image()
                    msg.header.stamp = stamp
                    msg.header.frame_id = "arm_camera_link"
                    msg.height = img_np.shape[0]
                    msg.width = img_np.shape[1]
                    msg.encoding = "rgb8" if img_np.shape[2] == 3 else "rgba8"
                    msg.is_bigendian = False
                    msg.step = msg.width * img_np.shape[2]
                    msg.data = img_np.tobytes()
                    self.wrist_camera_pub.publish(msg)
            except Exception as e:
                self.get_logger().debug(f"Wrist camera publish error: {e}")
        
        # Publish base camera
        if "base_camera" in scene.sensors:
            try:
                base_cam = scene.sensors["base_camera"]
                rgb_data = base_cam.data.output["rgb"]
                if rgb_data is not None and rgb_data.numel() > 0:
                    img_np = rgb_data[0].cpu().numpy()
                    if img_np.dtype != np.uint8:
                        img_np = (img_np * 255).astype(np.uint8)
                    
                    msg = Image()
                    msg.header.stamp = stamp
                    msg.header.frame_id = "base_camera_link"
                    msg.height = img_np.shape[0]
                    msg.width = img_np.shape[1]
                    msg.encoding = "rgb8" if img_np.shape[2] == 3 else "rgba8"
                    msg.is_bigendian = False
                    msg.step = msg.width * img_np.shape[2]
                    msg.data = img_np.tobytes()
                    self.base_camera_pub.publish(msg)
            except Exception as e:
                self.get_logger().debug(f"Base camera publish error: {e}")
    
    def get_action_tensor(self, robot_articulation, device: str) -> torch.Tensor:
        """
        Convert ROS2 commands to Isaac Lab action tensor.
        
        Returns a tensor matching the action space:
        [base_x, base_y, base_th, arm_1...7, gripper]
        """
        joint_names = robot_articulation.joint_names
        
        # Initialize action with zeros (no movement)
        # Action space: base(3) + arm(7) + gripper(1) = 11 dims
        action = np.zeros(11)
        
        # Arm: Set target positions (relative to default)
        if self.target_arm_positions is not None:
            for i, arm_joint in enumerate(self.ARM_JOINTS):
                if arm_joint in joint_names:
                    joint_idx = list(joint_names).index(arm_joint)
                    # Action is scaled, so we need to invert the scale
                    # The env uses scale=0.1, use_default_offset=True
                    # Action = (target - default) / scale
                    default_pos = robot_articulation.data.default_joint_pos[0, joint_idx].item()
                    target = self.target_arm_positions[i]
                    action[3 + i] = (target - default_pos) / 0.1
        
        # Gripper: Map position to action (0=open, 1=closed)
        # finger_joint: 0.0 (open) to 0.82 (closed)
        gripper_pos = self.target_gripper_positions[0]
        gripper_action = gripper_pos / 0.82  # Normalize to [0, 1]
        action[10] = gripper_action
        
        return torch.tensor(action, dtype=torch.float32, device=device).unsqueeze(0)


def log_camera_status(scene):
    """Log camera sensor availability."""
    if "wrist_camera" in scene.sensors:
        print("[INFO] Wrist camera sensor found, will publish to /tidybot/camera_wrist/color/raw")
    else:
        print("[WARNING] Wrist camera sensor not found in scene")
    
    if "base_camera" in scene.sensors:
        print("[INFO] Base camera sensor found, will publish to /tidybot/camera_base/color/raw")
    else:
        print("[WARNING] Base camera sensor not found in scene")


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg, agent_cfg):
    """Main function to run the Isaac Lab simulation with ROS2 bridge."""
    
    # Configure environment
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else "cuda:0"
    
    # Create environment
    print(f"[INFO] Creating environment: {args_cli.task}")
    env = gym.make(args_cli.task, cfg=env_cfg)
    
    # Initialize ROS2
    rclpy.init()
    ros_node = IsaacLabROS2Node()
    
    # Access scene objects
    scene = env.unwrapped.scene
    robot = scene["robot"]
    cabinet = scene["cabinet"]
    contact_sensor = scene.sensors.get("contact_forces")
    
    # Log camera status
    if args_cli.enable_cameras:
        log_camera_status(scene)
    
    # Initialize target arm positions from robot default
    initial_arm_pos = []
    for arm_joint in ros_node.ARM_JOINTS:
        if arm_joint in robot.joint_names:
            idx = list(robot.joint_names).index(arm_joint)
            initial_arm_pos.append(robot.data.default_joint_pos[0, idx].item())
        else:
            initial_arm_pos.append(0.0)
    ros_node.target_arm_positions = initial_arm_pos
    
    # Data logging
    data_list = []
    
    # Drawer body index for velocity tracking
    drawer_body_name = "drawer_top"
    try:
        drawer_body_idx = cabinet.body_names.index(drawer_body_name)
    except ValueError:
        print(f"[WARNING] '{drawer_body_name}' not found in cabinet bodies. Using index 0.")
        drawer_body_idx = 0
    
    # Get gripper contact body indices
    if contact_sensor:
        sensor_body_names = contact_sensor.body_names
        print(f"[INFO] Contact sensor tracking: {sensor_body_names}")
        left_pad_indices = [i for i, name in enumerate(sensor_body_names) if "left" in name.lower()]
        right_pad_indices = [i for i, name in enumerate(sensor_body_names) if "right" in name.lower()]
    else:
        left_pad_indices = []
        right_pad_indices = []
    
    # Simulation loop
    print("[INFO] Starting Isaac Lab simulation with ROS2 bridge...")
    print("[INFO] Topics published: /joint_states, /clock")
    print("[INFO] Topics subscribed: /gen3_7dof_controller/joint_trajectory, /robotiq_2f_85_controller/commands")
    
    dt = env.unwrapped.step_dt
    step_count = 0
    
    obs = env.reset()[0]
    
    while simulation_app.is_running():
        start_time = time.time()
        
        # Process ROS2 messages (non-blocking)
        rclpy.spin_once(ros_node, timeout_sec=0)
        
        # Get action from ROS2 node
        action = ros_node.get_action_tensor(robot, env.unwrapped.device)
        
        # Step environment
        obs, reward, terminated, truncated, info = env.step(action)
        
        # Get simulation time
        sim_time = env.unwrapped.sim.current_time
        
        # Publish to ROS2
        ros_node.publish_joint_states(robot, sim_time)
        ros_node.publish_clock(sim_time)
        
        # Publish camera images (every 3 steps to reduce overhead, ~10Hz)
        if args_cli.enable_cameras and step_count % 3 == 0:
            ros_node.publish_camera_images(scene, sim_time)
        
        # Collect sensor data (every 10 steps to reduce overhead)
        if contact_sensor and step_count % 10 == 0:
            env_idx = 0
            
            # Contact forces
            net_forces_w = contact_sensor.data.net_forces_w[env_idx]
            left_force_w = torch.sum(net_forces_w[left_pad_indices], dim=0) if left_pad_indices else torch.zeros(3)
            right_force_w = torch.sum(net_forces_w[right_pad_indices], dim=0) if right_pad_indices else torch.zeros(3)
            
            # Drawer velocity
            drawer_vel_w = cabinet.data.body_lin_vel_w[env_idx, drawer_body_idx]
            
            # Log data row
            row = {
                "Time(s)": sim_time,
                "Left_Fx": left_force_w[0].item() if torch.is_tensor(left_force_w) else 0.0,
                "Left_Fy": left_force_w[1].item() if torch.is_tensor(left_force_w) else 0.0,
                "Left_Fz": left_force_w[2].item() if torch.is_tensor(left_force_w) else 0.0,
                "Right_Fx": right_force_w[0].item() if torch.is_tensor(right_force_w) else 0.0,
                "Right_Fy": right_force_w[1].item() if torch.is_tensor(right_force_w) else 0.0,
                "Right_Fz": right_force_w[2].item() if torch.is_tensor(right_force_w) else 0.0,
                "Drawer_Vx": drawer_vel_w[0].item(),
                "Drawer_Vy": drawer_vel_w[1].item(),
                "Drawer_Vz": drawer_vel_w[2].item(),
            }
            data_list.append(row)
        
        step_count += 1
        
        # Real-time pacing
        if args_cli.real_time:
            elapsed = time.time() - start_time
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        # Status update
        if step_count % 500 == 0:
            print(f"[INFO] Step {step_count}, Sim Time: {sim_time:.2f}s")
    
    # Save sensor data
    if data_list:
        df = pd.DataFrame(data_list)
        save_dir = os.path.dirname(args_cli.save_path)
        if save_dir and not os.path.exists(save_dir):
            os.makedirs(save_dir)
        df.to_csv(args_cli.save_path, index=False, float_format='%.6e')
        print(f"[INFO] Sensor data saved to {args_cli.save_path}")
    
    # Cleanup
    ros_node.destroy_node()
    rclpy.shutdown()
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
