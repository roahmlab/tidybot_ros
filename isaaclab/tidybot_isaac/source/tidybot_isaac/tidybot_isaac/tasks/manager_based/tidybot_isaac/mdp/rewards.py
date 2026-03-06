# Copyright (c) 2022-2026, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

from isaaclab.utils.math import quat_apply, quat_apply_inverse
import isaaclab.utils.math as math_utils

# ==========================================
# REWARDS
# ==========================================

def approach_ee_handle(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Dense reward for moving the EE towards the handle using an exponential kernel."""
    ee_pos = env.scene["ee_frame"].data.target_pos_w[..., 0, :]
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]
    
    # Compute Euclidean distance
    distance = torch.norm(handle_pos - ee_pos, dim=-1, p=2)
    
    # Sharp exponential curve: high reward only when very close (< 10cm)
    return torch.exp(-distance / 0.1)

VEC_Z = torch.tensor([0.0, 0.0, 1.0])
VEC_X = torch.tensor([1.0, 0.0, 0.0])
VEC_Y = torch.tensor([0.0, 1.0, 0.0])
def align_ee_handle(
    env: ManagerBasedRLEnv,
) -> torch.Tensor:
    """Reward for aligning EE tool frame with handle frame.
    
    Conditions:
    1. Z_ee aligns with Z_handle (pointing direction)
    2. X_ee aligns with Y_handle (pinch orientation)
    
    Scaled by proximity to the handle.
    """
    # Get Frames
    ee_pos = env.scene["ee_frame"].data.target_pos_w[..., 0, :]
    ee_quat = env.scene["ee_frame"].data.target_quat_w[..., 0, :]
    
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]
    handle_quat = env.scene["cabinet_frame"].data.target_quat_w[..., 0, :]
    
    # Distance
    distance = torch.norm(handle_pos - ee_pos, dim=-1)
    
    # Proximity Factor (continuous)
    # Base: 1 / (1+d^2)
    proximity = 1.0 / (1.0 + distance**2)
    
    # Smooth Boosts using exponential decay
    # Boost 1: Short range (starts ~0.1m)
    boost_close = torch.exp(-distance / 0.1)
    # Boost 2: Very short range (starts ~0.02m)
    boost_very_close = 2.0 * torch.exp(-distance / 0.02)
    
    # Combined scalar
    scaling = proximity + boost_close + boost_very_close
    
    # Expand base vectors for batching
    vec_z = VEC_Z.to(env.device).expand(env.num_envs, 3)
    vec_x = VEC_X.to(env.device).expand(env.num_envs, 3)    
    vec_y = VEC_Y.to(env.device).expand(env.num_envs, 3)
    
    # Rotate Basis Vectors to World
    ee_z = quat_apply(ee_quat, vec_z)
    ee_x = quat_apply(ee_quat, vec_x)
    handle_z = quat_apply(handle_quat, vec_z)
    
    # Extract the Y-axis of the handle
    handle_y = quat_apply(handle_quat, vec_y)

    # Compute Dot Products
    dot_z = torch.sum(ee_z * handle_z, dim=-1)
    dot_x = torch.sum(ee_x * handle_y, dim=-1)

    # Reward only starts to matter when alignment is > 0.9 (approx 25 degrees)
    # and hits 1.0 only at 1.0.
    align_z = torch.clamp(dot_z, min=0.0)**8
    align_x = torch.clamp(dot_x, min=0.0)**8

    total_alignment = align_z * align_x
    
    return total_alignment * scaling

def grasp_handle(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    gripper_joint_name: str = "hande_left_finger_joint",
) -> torch.Tensor:
    """Reward for closing the gripper, scaled by how well the handle is centered in the jaws."""
    robot: Articulation = env.scene[robot_cfg.name]
    
    # Get Positions for Proximity
    ee_pos = env.scene["ee_frame"].data.target_pos_w[..., 0, :]
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]
    distance = torch.norm(handle_pos - ee_pos, dim=-1)
    
    # Local Frame Transformation
    # Instead of a 0/1 gate, see how centered the handle is.
    ee_tcp_quat = env.scene["ee_frame"].data.target_quat_w[..., 0, :]
    vec_to_handle = handle_pos - ee_pos
    local_handle_pos = quat_apply_inverse(ee_tcp_quat, vec_to_handle)
    
    # X (Pinch/Centered)
    centering_factor = torch.clamp(1.0 - (torch.abs(local_handle_pos[:, 0]) / 0.015), 0.0, 1.0)
    
    # Y (Elevation)
    elevation_factor = torch.clamp(1.0 - (torch.abs(local_handle_pos[:, 1]) / 0.01), 0.0, 1.0)
    
    # Z (Approach/Insertion)
    insertion_factor = torch.clamp(1.0 - (torch.abs(local_handle_pos[:, 2]) / 0.02), 0.0, 1.0)
    
    # Require all three dimensions to be good
    spatial_quality = centering_factor * elevation_factor * insertion_factor

    # Gripper Logic
    gripper_joint_idx = robot.find_joints(gripper_joint_name)[0][0]
    gripper_pos = robot.data.joint_pos[:, gripper_joint_idx]
    
    # Constants from your Hand-E spec
    target_width = 0.015  # Handle thickness
    max_open = 0.025      # Fully open state
    
    # Normalized Closedness (Ramp hits 1.0 at handle thickness)
    closedness = (max_open - gripper_pos) / (max_open - target_width)
    closedness = torch.clamp(closedness, 0.0, 1.0) 
    
    proximity_slope = torch.clamp(1.0 - (distance / 0.10), 0.0, 1.0)
    
    return proximity_slope * spatial_quality * closedness

def _is_properly_grasping(
    env: ManagerBasedRLEnv, 
    robot_cfg: SceneEntityCfg, 
    gripper_joint_name: str
) -> torch.Tensor:
    """Checks proper spatial and orientation position."""
    # Get Frames
    ee_tcp_pos = env.scene["ee_frame"].data.target_pos_w[..., 0, :]
    ee_tcp_quat = env.scene["ee_frame"].data.target_quat_w[..., 0, :]
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]
    handle_quat = env.scene["cabinet_frame"].data.target_quat_w[..., 0, :]
    
    # Local Position Check
    vec_to_handle = handle_pos - ee_tcp_pos
    local_handle_pos = quat_apply_inverse(ee_tcp_quat, vec_to_handle)
    is_centered = (torch.abs(local_handle_pos[:, 0]) < 0.008).float()
    is_inserted = (torch.abs(local_handle_pos[:, 2]) < 0.01).float()

    # Orientation check
    ee_z = quat_apply(ee_tcp_quat, VEC_Z.to(env.device).expand(env.num_envs, 3))
    h_z = quat_apply(handle_quat, VEC_Z.to(env.device).expand(env.num_envs, 3))
    
    # Check primary axis (Pointing)
    dot_z = torch.sum(ee_z * h_z, dim=-1)
    
    # Check secondary axis (Roll/Tilt)
    ee_x = quat_apply(ee_tcp_quat, VEC_X.to(env.device).expand(env.num_envs, 3))
    h_y = quat_apply(handle_quat, VEC_Y.to(env.device).expand(env.num_envs, 3))
    dot_x = torch.sum(ee_x * h_y, dim=-1)
    
    # 0.996 for both means gripper must be within ~5 degrees of perfect
    is_aligned = ((dot_z > 0.925) & (dot_x > 0.925)).float()
    
    # Gripper Check
    robot = env.scene[robot_cfg.name]
    gripper_joint_idx = robot.find_joints(gripper_joint_name)[0][0]
    gripper_pos = robot.data.joint_pos[:, gripper_joint_idx]
    is_closed = (gripper_pos <= 0.02).float()
    
    return is_centered * is_inserted * is_closed * is_aligned

def gated_open_drawer(
    env: ManagerBasedRLEnv, 
    threshold: float,
    cabinet_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg,
    gripper_joint_name: str = "hande_left_finger_joint",
) -> torch.Tensor:
    """Reward for drawer position up to a threshold, only if actively grasped."""
    # Get drawer position
    cabinet = env.scene[cabinet_cfg.name]
    drawer_joint_idx = cabinet.find_joints("drawer_top_joint")[0][0]
    drawer_pos = cabinet.data.joint_pos[:, drawer_joint_idx]
    
    # Saturate position reward
    reward_pos = torch.clamp(drawer_pos, min=0.0, max=threshold)
    
    # Apply grasp gate
    grasp_gate = _is_properly_grasping(env, robot_cfg, gripper_joint_name)
    
    return reward_pos * grasp_gate

def hold_open_bonus(
    env: ManagerBasedRLEnv, 
    threshold: float, 
    cabinet_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg,
    gripper_joint_name: str = "hande_left_finger_joint",
) -> torch.Tensor:
    """Gated bonus for full open."""
    cabinet = env.scene[cabinet_cfg.name]
    drawer_joint_idx = cabinet.find_joints("drawer_top_joint")[0][0]
    drawer_pos = cabinet.data.joint_pos[:, drawer_joint_idx]
    
    # Check conditions: drawer is open, robot is still holding the handle
    is_open = (drawer_pos > threshold).float()
    is_grasping = _is_properly_grasping(env, robot_cfg, gripper_joint_name)
    
    return is_open * is_grasping

# ==========================================
# PENALTIES
# ==========================================

def action_rate_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize the change between current and previous actions for smoothness."""
    current_action = env.action_manager.action
    previous_action = env.action_manager.prev_action
    
    # Calculate the squared difference
    # Penalizing (a_t - a_{t-1})^2
    return -torch.sum(torch.square(current_action - previous_action), dim=1)

def ee_velocity_penalty(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    ee_body_name: str = "bracelet_link",
) -> torch.Tensor:
    """Penalize high linear velocity of the end-effector body to encourage smooth pulls."""
    # Get the robot articulation
    robot: Articulation = env.scene[robot_cfg.name]
    
    # Find the index of the end-effector link in the physics engine
    body_idx = robot.find_bodies(ee_body_name)[0][0]
    
    # Get the linear velocity of that specific body in the world frame
    ee_vel = robot.data.body_lin_vel_w[:, body_idx, :]
    
    # Compute the magnitude of the velocity
    vel_mag = torch.norm(ee_vel, dim=-1)
    
    # Penalize velocities above a safe threshold (0.2 m/s)
    excess_vel = torch.clamp(vel_mag - 0.2, min=0.0)
    
    return -excess_vel
    
def unaligned_approach_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize the robot for being close to the handle without being aligned.
    Added because policy was knocking into the drawer while aligning its grasp.
    """
    # Get positions and distance
    ee_pos = env.scene["ee_frame"].data.target_pos_w[..., 0, :]
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]
    distance = torch.norm(handle_pos - ee_pos, dim=-1)
    
    is_close = (distance < 0.10).float()

    # Get Orientations
    ee_quat = env.scene["ee_frame"].data.target_quat_w[..., 0, :]
    handle_quat = env.scene["cabinet_frame"].data.target_quat_w[..., 0, :]
    
    # Check Z-Axis (Pointing) Alignment
    vec_z = VEC_Z.to(env.device).expand(env.num_envs, 3)
    ee_z = quat_apply(ee_quat, vec_z)
    handle_z = quat_apply(handle_quat, vec_z)
    dot_z = torch.sum(ee_z * handle_z, dim=-1)
    
    # If the alignment is poor (less than ~25 degrees from perfect)
    # dot_z of 0.9 is roughly 25 degrees.
    is_badly_aligned = (dot_z < 0.9).float()
    
    # Scale the penalty so it gets worse the closer you are while unaligned
    # (0.10 - distance) gives a linear scale that gets larger as distance approaches 0
    severity = torch.clamp(0.10 - distance, min=0.0) * 10.0 # Scales from 0.0 to 1.0
    
    # Apply penalty
    return -1.0 * is_close * is_badly_aligned * severity

def rest_at_goal_penalty(
    env: ManagerBasedRLEnv, 
    threshold: float, 
    cabinet_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Penalize the robot for continuing to move its arm after the drawer is successfully open.
    """
    # Check if the task is complete (Drawer is Open)
    cabinet = env.scene[cabinet_cfg.name]
    drawer_joint_idx = cabinet.find_joints("drawer_top_joint")[0][0]
    drawer_pos = cabinet.data.joint_pos[:, drawer_joint_idx]
    
    is_open = (drawer_pos >= threshold).float()
    
    # Get the velocities of the robot's arm joints
    robot: Articulation = env.scene[robot_cfg.name]
    
    # Only want to penalize arm joints 
    arm_joint_indices, _ = robot.find_joints("joint_[1-7]") 
    
    # Get the velocities of those specific joints
    arm_velocities = robot.data.joint_vel[:, arm_joint_indices]
    
    # Calculate the sum of squared velocities (L2 norm squared)
    velocity_penalty = torch.sum(torch.square(arm_velocities), dim=-1)
    
    # Apply penalty only if the drawer is open
    return -1.0 * is_open * velocity_penalty

# ==========================================
# TERMINATIONS
# ==========================================

def drawer_opened(
    env: ManagerBasedRLEnv, 
    threshold: float, 
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Termination condition for successfully opening the drawer."""
    cabinet: Articulation = env.scene[asset_cfg.name]
    drawer_joint_idx = cabinet.find_joints("drawer_top_joint")[0][0]
    drawer_pos = cabinet.data.joint_pos[:, drawer_joint_idx]
    return drawer_pos > threshold