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
        --task Isaac-TidyBot-HandE-Drawer-v0 --num_envs 1 --real-time

Then launch the ROS2 stack:
    ros2 launch tidybot_description launch_isaac_lab.launch.py
"""

import argparse
import sys
import os
import time


from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Isaac Lab Simulation with ROS2 Bridge.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="Isaac-TidyBot-HandE-Drawer-v0", help="Name of the task.")
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time.")

# append AppLauncher cli args (includes --enable_cameras)
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()



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

import torch
import gymnasium as gym

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import WrenchStamped
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
    BASE_JOINTS = ["joint_x", "joint_y", "joint_th"]
    
    # Gripper configurations keyed by type
    GRIPPER_CONFIGS = {
        "hande": {
            "joints": ["hande_left_finger_joint", "hande_right_finger_joint"],
            "topic": "/robotiq_hande_controller/commands",
            "open_pos": [0.025, 0.025],   # prismatic, meters
            "close_pos": [0.0, 0.0],
            "range": 0.025,               # full travel
            "inverted": True,             # 0.025=open, 0.0=closed
        },
        "2f85": {
            "joints": ["finger_joint", "right_outer_knuckle_joint"],
            "topic": "/robotiq_2f_85_controller/commands",
            "open_pos": [0.0, 0.0],       # revolute, radians
            "close_pos": [0.82, -0.82],
            "range": 0.82,                # full travel
            "inverted": False,            # 0.0=open, 0.82=closed
        },
    }
    
    def __init__(self, gripper_type: str = "hande"):
        super().__init__('isaac_lab_sim')
        
        # Gripper configuration
        self.gripper_type = gripper_type
        self.gripper_cfg = self.GRIPPER_CONFIGS[gripper_type]
        self.GRIPPER_JOINTS = self.gripper_cfg["joints"]
        
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
            self.gripper_cfg["topic"],
            self.gripper_callback,
            10
        )
        
        # === Publishers ===
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        
        # Camera publishers (will be initialized later if cameras exist)
        self.wrist_camera_pub = self.create_publisher(Image, '/tidybot/camera_wrist/color/raw', 10)
        self.base_camera_pub = self.create_publisher(Image, '/tidybot/camera_base/color/raw', 10)
        
        # Contact force publishers (matching contact_force_publisher.py topics)
        self.left_contact_pub = self.create_publisher(
            WrenchStamped, '/tidybot/contact/left_finger', 10)
        self.right_contact_pub = self.create_publisher(
            WrenchStamped, '/tidybot/contact/right_finger', 10)
        
        # Drawer state publisher (matching contact_force_publisher.py topic)
        self.drawer_state_pub = self.create_publisher(
            Float64MultiArray, '/tidybot/drawer/state', 10)
        
        # === State ===
        self.target_arm_positions = None  # Will be set from robot initial state
        self.target_gripper_positions = list(self.gripper_cfg["open_pos"])
        self.last_arm_command_time = time.time()
        self.last_gripper_command_time = time.time()
        
        # Drawer state tracking for numerical differentiation
        self._prev_drawer_pos = None
        self._prev_drawer_vel = None
        
        self.get_logger().info("Isaac Lab ROS2 Node initialized")
        self.get_logger().info(f"  Gripper type: {gripper_type}")
        self.get_logger().info(f"  Gripper topic: {self.gripper_cfg['topic']}")
        self.get_logger().info("  Contact topics: /tidybot/contact/{left,right}_finger")
        self.get_logger().info("  Drawer topic: /tidybot/drawer/state")
    
    def arm_traj_callback(self, msg: JointTrajectory):
        """Handle incoming arm trajectory commands.
        
        Matches isaac_sim_bridge.py: take the last trajectory point as the
        goal and let the stiff position drive track it directly.
        No interpolation needed — Isaac Sim does the same thing.
        """
        if not msg.points:
            return
        
        # Use the last trajectory point as the target (same as isaac_sim_bridge.py)
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
    
    def publish_contact_forces(
        self, left_sensor, right_sensor,
        env_idx: int, sim_time: float
    ):
        """Publish gripper contact forces (friction only, filtered to drawer handle).
        
        Uses per-finger ContactSensorCfg with filter_prim_paths_expr targeting
        the drawer handle, matching open_drawer_collect_data.py approach.
        friction_forces_w gives only forces against the filtered target.
        """
        stamp = self._float_to_stamp(sim_time)
        
        # friction_forces_w shape: (N, B, M, 3) — B=bodies, M=filter prims
        # Each sensor tracks one finger, so sum over bodies and filter prims
        f_left_raw = left_sensor.data.friction_forces_w[env_idx]   # (B, M, 3)
        f_right_raw = right_sensor.data.friction_forces_w[env_idx]  # (B, M, 3)
        
        # Replace NaN with 0 (NaN means no contact with that filter body)
        f_left_raw = torch.nan_to_num(f_left_raw, nan=0.0)
        f_right_raw = torch.nan_to_num(f_right_raw, nan=0.0)
        
        # Sum over bodies (dim=0) and filter prims (dim=0 after first sum)
        if f_left_raw.dim() == 3:
            left_force = f_left_raw.sum(dim=0).sum(dim=0)   # (3,)
        else:
            left_force = f_left_raw.sum(dim=0)               # (3,)
        
        if f_right_raw.dim() == 3:
            right_force = f_right_raw.sum(dim=0).sum(dim=0)  # (3,)
        else:
            right_force = f_right_raw.sum(dim=0)              # (3,)
        
        # Publish left finger
        left_msg = WrenchStamped()
        left_msg.header.stamp = stamp
        left_msg.header.frame_id = "left_inner_finger"
        left_msg.wrench.force.x = float(left_force[0])
        left_msg.wrench.force.y = float(left_force[1])
        left_msg.wrench.force.z = float(left_force[2])
        self.left_contact_pub.publish(left_msg)
        
        # Publish right finger
        right_msg = WrenchStamped()
        right_msg.header.stamp = stamp
        right_msg.header.frame_id = "right_inner_finger"
        right_msg.wrench.force.x = float(right_force[0])
        right_msg.wrench.force.y = float(right_force[1])
        right_msg.wrench.force.z = float(right_force[2])
        self.right_contact_pub.publish(right_msg)
    
    def publish_drawer_state(self, scene, env_idx: int, dt: float):
        """Publish drawer handle position, velocity, and acceleration.
        
        Uses the cabinet_frame FrameTransformer to get the handle's world 
        position, then computes velocity and acceleration via numerical 
        differentiation (same as contact_force_publisher.py).
        """
        # Get handle world position from FrameTransformer
        handle_pos = scene["cabinet_frame"].data.target_pos_w[env_idx, 0].cpu().numpy()
        drawer_pos = np.array(handle_pos)
        
        # Numerical differentiation for velocity
        if self._prev_drawer_pos is not None and dt > 0:
            drawer_vel = (drawer_pos - self._prev_drawer_pos) / dt
        else:
            drawer_vel = np.zeros(3)
        
        # Numerical differentiation for acceleration
        if self._prev_drawer_vel is not None and dt > 0:
            drawer_acc = (drawer_vel - self._prev_drawer_vel) / dt
        else:
            drawer_acc = np.zeros(3)
        
        self._prev_drawer_pos = drawer_pos.copy()
        self._prev_drawer_vel = drawer_vel.copy()
        
        # Publish as Float64MultiArray [pos_xyz, vel_xyz, acc_xyz]
        msg = Float64MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='state', size=9, stride=9)
        ]
        msg.data = list(drawer_pos) + list(drawer_vel) + list(drawer_acc)
        self.drawer_state_pub.publish(msg)
    
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
    
    def get_action_tensor(self, robot_articulation, device) -> torch.Tensor:
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
                    # Action = (target - default) / scale
                    # In ROS2 bridge mode, scale is overridden to 1.0
                    default_pos = robot_articulation.data.default_joint_pos[0, joint_idx].item()
                    target = self.target_arm_positions[i]
                    action[3 + i] = (target - default_pos) / 1.0
        
        # Gripper
        if self.target_gripper_positions is not None:
            # multi_stage_planner sends hardware-specific commands:
            #   Hand-E: meters (0.025=open, 0.0=closed)
            #   2F-85:  radians (0.0=open, 0.82=closed)
            # MimicGripperAction expects [0, 1] where 0=open, 1=closed
            cmd = self.target_gripper_positions[0]  # Leader joint position
            
            if self.gripper_cfg["inverted"]:
                # Hand-E: 0.025 (open) -> 0.0, 0.0 (closed) -> 1.0
                gripper_action = 1.0 - (cmd / self.gripper_cfg["range"])
            else:
                # 2F-85: 0.0 (open) -> 0.0, 0.82 (closed) -> 1.0
                gripper_action = cmd / self.gripper_cfg["range"]
            
            # Clamp to [0, 1]
            gripper_action = max(0.0, min(1.0, gripper_action))
            
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
    
    # ── ROS2 bridge mode overrides ──────────────────────────────────────
    # These overrides adapt the RL-oriented env config for real-time
    # ROS2 control without touching the shared training config.
    
    # 1. Disable automatic resets (drawer_policy manages lifecycle)
    env_cfg.episode_length_s = 600  # 10 min — effectively no timeout
    env_cfg.terminations.time_out = None
    if hasattr(env_cfg.terminations, 'success'):
        env_cfg.terminations.success = None
    
    # 2. Arm action scale 0.1→1.0: the RL scale compresses actions
    #    into a [-1,1] range for policy learning. For ROS2 bridge we
    #    want a 1:1 mapping so trajectory positions pass through directly.
    env_cfg.actions.arm_pos.scale = 1.0
    
    # 3. Override arm effort limit for fast tracking:
    #    Isaac Sim's direct PhysX drives ignore URDF effort limits (~39 Nm).
    #    Isaac Lab's ImplicitActuator enforces them, capping drive torque.
    #    With real-time pacing now forced, wall-clock ≈ sim-time, so
    #    the URDF velocity limits (~2.1 rad/s) won't cause stage misses.
    env_cfg.scene.robot.actuators["arm"].effort_limit = 1e6
    
    # 4. Reduce decimation for higher sensor publish rate:
    #    Default decimation=4 → 60Hz loop. Decimation=2 → 120Hz loop,
    #    well above the sensor_data_recorder's 50Hz capture rate.
    env_cfg.decimation = 1
    
    # 5. Increase contact sensor buffer for friction tracking:
    #    Default max_contact_data_count_per_prim=4 overflows when the gripper
    #    pulls the drawer handle (fingers slide, generating many contacts).
    #    PhysX crashes with "srcIndex < srcSelectDimSize" CUDA assertion.
    for sensor_name in ['contact_forces_left', 'contact_forces_right', 'contact_forces']:
        if hasattr(env_cfg.scene, sensor_name):
            getattr(env_cfg.scene, sensor_name).max_contact_data_count_per_prim = 16
    
    print("[INFO] ROS2 bridge mode: arm_scale=1.0, effort_limit=1e6, real-time pacing ON")
    print("[INFO] ROS2 bridge mode: disabled auto-resets (episode=600s, no terminations)")
    
    # Create environment
    print(f"[INFO] Creating environment: {args_cli.task}")
    env = gym.make(args_cli.task, cfg=env_cfg)
    
    # Detect gripper type from task name
    if "2F85" in args_cli.task:
        gripper_type = "2f85"
    else:
        gripper_type = "hande"
    print(f"[INFO] Detected gripper type: {gripper_type} (from task: {args_cli.task})")
    
    # Initialize ROS2
    rclpy.init()
    ros_node = IsaacLabROS2Node(gripper_type=gripper_type)
    
    # Access scene objects
    scene = env.unwrapped.scene
    robot = scene["robot"]
    cabinet = scene["cabinet"]
    left_contact_sensor = scene.sensors.get("contact_forces_left")
    right_contact_sensor = scene.sensors.get("contact_forces_right")
    has_contact_sensors = left_contact_sensor is not None and right_contact_sensor is not None
    
    if has_contact_sensors:
        print(f"[INFO] Left contact sensor bodies: {left_contact_sensor.body_names}")
        print(f"[INFO] Right contact sensor bodies: {right_contact_sensor.body_names}")
    else:
        print("[WARNING] Per-finger contact sensors not found in scene")
    
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
    
    # Drawer body index for velocity tracking
    drawer_body_name = "drawer_top"
    try:
        drawer_body_idx = cabinet.body_names.index(drawer_body_name)
    except ValueError:
        print(f"[WARNING] '{drawer_body_name}' not found in cabinet bodies. Using index 0.")
        drawer_body_idx = 0
    
    # Simulation loop
    print("[INFO] Starting Isaac Lab simulation with ROS2 bridge...")
    print("[INFO] Topics published: /joint_states, /clock, /tidybot/contact/{left,right}_finger, /tidybot/drawer/state")
    print(f"[INFO] Topics subscribed: /gen3_7dof_controller/joint_trajectory, {ros_node.gripper_cfg['topic']}")
    print(f"[INFO] Real-time pacing: {'ENABLED' if args_cli.real_time else 'DISABLED (max speed)'}")
    
    dt = env.unwrapped.step_dt
    step_count = 0
    wall_start = time.time()
    
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
        
        # Publish to ROS2 (joint states + clock every step)
        ros_node.publish_joint_states(robot, sim_time)
        ros_node.publish_clock(sim_time)
        
        # Publish contact forces + drawer state every 4 steps to reduce overhead
        if step_count % 4 == 0:
            if has_contact_sensors:
                ros_node.publish_contact_forces(
                    left_contact_sensor, right_contact_sensor,
                    env_idx=0, sim_time=sim_time
                )
            ros_node.publish_drawer_state(scene, env_idx=0, dt=dt)
        
        # Publish camera images (every 10 steps to reduce overhead)
        if args_cli.enable_cameras and step_count % 10 == 0:
            ros_node.publish_camera_images(scene, sim_time)
        
        step_count += 1
        
        # Real-time pacing (only when --real-time flag is set)
        if args_cli.real_time:
            elapsed = time.time() - start_time
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        # Status update with speed ratio
        if step_count % 500 == 0:
            wall_elapsed = time.time() - wall_start
            speed_ratio = sim_time / wall_elapsed if wall_elapsed > 0 else 0
            fps = step_count / wall_elapsed if wall_elapsed > 0 else 0
            print(f"[INFO] Step {step_count}, Sim: {sim_time:.2f}s, Wall: {wall_elapsed:.1f}s, "
                  f"Speed: {speed_ratio:.2f}x real-time, FPS: {fps:.1f}")

    
    # Cleanup
    ros_node.destroy_node()
    rclpy.shutdown()
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
