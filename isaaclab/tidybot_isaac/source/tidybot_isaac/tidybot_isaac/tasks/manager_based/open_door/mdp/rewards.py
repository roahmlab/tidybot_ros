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
    handle_pos = env.scene["handle_frame"].data.target_pos_w[..., 0, :]
    
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
    1. Z_ee aligns with X_handle (pointing direction)
    2. X_ee aligns with Y_handle with symmetry (pinch orientation)
    """
    ee_pos = env.scene["ee_frame"].data.target_pos_w[..., 0, :]
    ee_quat = env.scene["ee_frame"].data.target_quat_w[..., 0, :]
    
    handle_pos = env.scene["handle_frame"].data.target_pos_w[..., 0, :]
    handle_quat = env.scene["handle_frame"].data.target_quat_w[..., 0, :]
    
    # Distance & Proximity Factors
    distance = torch.norm(handle_pos - ee_pos, dim=-1)
    proximity = 1.0 / (1.0 + distance**2)
    boost_close = torch.exp(-distance / 0.1)
    boost_very_close = 2.0 * torch.exp(-distance / 0.02)
    scaling = proximity + boost_close + boost_very_close
    
    # Expand base vectors for batching
    vec_x = VEC_X.to(env.device).expand(env.num_envs, 3)    
    vec_y = VEC_Y.to(env.device).expand(env.num_envs, 3)
    vec_z = VEC_Z.to(env.device).expand(env.num_envs, 3)
    
    # Rotate Basis Vectors to World
    ee_z = quat_apply(ee_quat, vec_z) # EE Pointing
    ee_x = quat_apply(ee_quat, vec_x) # EE Pinch (Corrected to X)
    
    handle_x = quat_apply(handle_quat, vec_x) # Handle Pointing
    handle_y = quat_apply(handle_quat, vec_y) # Handle Pinch
    
    # Compute Dot Products
    dot_z = torch.sum(ee_z * handle_x, dim=-1)
    dot_x = torch.sum(ee_x * handle_y, dim=-1)

    # Reward only starts to matter when alignment is > 0.8 
    align_z = torch.clamp(dot_z, min=0.0)**7
    align_x = torch.abs(dot_x)**7 # allows 180-degree roll symmetry on X/Y

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
    handle_pos = env.scene["handle_frame"].data.target_pos_w[..., 0, :]
    distance = torch.norm(handle_pos - ee_pos, dim=-1)
    
    # Local Frame Transformation
    # Instead of a 0/1 gate, see how centered the handle is.
    ee_tcp_quat = env.scene["ee_frame"].data.target_quat_w[..., 0, :]
    vec_to_handle = handle_pos - ee_pos
    local_handle_pos = quat_apply_inverse(ee_tcp_quat, vec_to_handle)
    
    # X (Pinch/Centered)
    centering_factor = torch.clamp(1.0 - (torch.abs(local_handle_pos[:, 0]) / 0.0125), 0.0, 1.0)
    
    # Y (Elevation)
    elevation_factor = torch.clamp(1.0 - (torch.abs(local_handle_pos[:, 1]) / 0.02), 0.0, 1.0)
    
    # Z (Approach/Insertion)
    insertion_factor = torch.clamp(1.0 - (torch.abs(local_handle_pos[:, 2]) / 0.02), 0.0, 1.0)
    
    # Require all three dimensions to be good on average
    spatial_quality = (centering_factor + elevation_factor + insertion_factor) / 3.0

    # Gripper Logic
    gripper_joint_idx = robot.find_joints(gripper_joint_name)[0][0]
    gripper_pos = robot.data.joint_pos[:, gripper_joint_idx]
    
    # Constants from Hand-E spec
    target_width = 0.020  # Handle thickness
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
    """Checks proper spatial and orientation position, scaled continuously by alignment."""
    ee_tcp_pos = env.scene["ee_frame"].data.target_pos_w[..., 0, :]
    ee_tcp_quat = env.scene["ee_frame"].data.target_quat_w[..., 0, :]
    handle_pos = env.scene["handle_frame"].data.target_pos_w[..., 0, :]
    handle_quat = env.scene["handle_frame"].data.target_quat_w[..., 0, :]
    
    # Local Position Check (Using X for pinch and Z for insertion)
    vec_to_handle = handle_pos - ee_tcp_pos
    local_handle_pos = quat_apply_inverse(ee_tcp_quat, vec_to_handle)
    is_centered = (torch.abs(local_handle_pos[:, 0]) < 0.0125).float() # Pinch Axis
    is_inserted = (torch.abs(local_handle_pos[:, 2]) < 0.01).float()   # Insertion Axis

    vec_x = VEC_X.to(env.device).expand(env.num_envs, 3)
    vec_y = VEC_Y.to(env.device).expand(env.num_envs, 3)
    vec_z = VEC_Z.to(env.device).expand(env.num_envs, 3)

    # Alignment logic
    ee_z = quat_apply(ee_tcp_quat, vec_z)
    handle_x = quat_apply(handle_quat, vec_x)
    dot_z = torch.sum(ee_z * handle_x, dim=-1)
    
    ee_x = quat_apply(ee_tcp_quat, vec_x)
    handle_y = quat_apply(handle_quat, vec_y)
    dot_x = torch.sum(ee_x * handle_y, dim=-1)
    
    align_z = torch.clamp(dot_z, min=0.0)**7
    align_x = torch.abs(dot_x)**7
    alignment_multiplier = align_z * align_x
    
    # Gripper Check
    robot = env.scene[robot_cfg.name]
    gripper_joint_idx = robot.find_joints(gripper_joint_name)[0][0]
    gripper_pos = robot.data.joint_pos[:, gripper_joint_idx]
    is_closed = (gripper_pos <= 0.02).float()
    
    return is_centered * is_inserted * is_closed * alignment_multiplier

def ungated_open_door(
    env: ManagerBasedRLEnv, 
    threshold: float,
    door_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Reward for door position up to a threshold."""
    # Get door position
    door = env.scene[door_cfg.name]
    door_joint_idx = door.find_joints("HingeJoint")[0][0]
    door_pos = torch.abs(door.data.joint_pos[:, door_joint_idx]) # Changed to abs
    # Saturate position reward
    reward_pos = torch.clamp(door_pos, min=0.0, max=threshold)
    return reward_pos 

def gated_open_door(
    env: ManagerBasedRLEnv, 
    threshold: float,
    door_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg,
    gripper_joint_name: str = "hande_left_finger_joint",
) -> torch.Tensor:
    """Reward for door position up to a threshold, only if actively grasped."""
    # Get door position
    door = env.scene[door_cfg.name]
    door_joint_idx = door.find_joints("HingeJoint")[0][0]
    door_pos = torch.abs(door.data.joint_pos[:, door_joint_idx])
    
    # Saturate position reward
    reward_pos = torch.clamp(door_pos, min=0.0, max=threshold)
    
    # Apply grasp gate
    grasp_gate = _is_properly_grasping(env, robot_cfg, gripper_joint_name)
    
    return reward_pos * grasp_gate

def hold_open_bonus(
    env: ManagerBasedRLEnv, 
    threshold: float, 
    door_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg,
    gripper_joint_name: str = "hande_left_finger_joint",
) -> torch.Tensor:
    """Gated bonus for full open."""
    door = env.scene[door_cfg.name]
    door_joint_idx = door.find_joints("HingeJoint")[0][0]
    door_pos = torch.abs(door.data.joint_pos[:, door_joint_idx])
    
    # Check conditions: door is open, robot is still holding the handle
    is_open = (door_pos > threshold).float()
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

def gripper_chatter_penalty(
    env: ManagerBasedRLEnv, 
    gripper_action_idx: int = -1
) -> torch.Tensor:
    """Heavily penalizes switching the binary gripper state across the 0.0 threshold.
       Disscourages micro-slipping the handle instead of learning to follow the opening arc."""
    # Extract the raw continuous actions for the gripper
    curr_action = env.action_manager.action[:, gripper_action_idx]
    prev_action = env.action_manager.prev_action[:, gripper_action_idx]
    curr_binary = torch.where(curr_action > 0.0, 1.0, -1.0)
    prev_binary = torch.where(prev_action > 0.0, 1.0, -1.0)
    
    return -torch.square(curr_binary - prev_binary)

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
    Updated to match correct axes: EE Z to Handle X, EE X to Handle Y.
    """
    # Get positions and distance
    ee_pos = env.scene["ee_frame"].data.target_pos_w[..., 0, :]
    handle_pos = env.scene["handle_frame"].data.target_pos_w[..., 0, :]
    distance = torch.norm(handle_pos - ee_pos, dim=-1)
    
    is_close = (distance < 0.10).float()

    # Get Orientations
    ee_quat = env.scene["ee_frame"].data.target_quat_w[..., 0, :]
    handle_quat = env.scene["handle_frame"].data.target_quat_w[..., 0, :]
    
    # Expand base vectors
    vec_x = VEC_X.to(env.device).expand(env.num_envs, 3)
    vec_y = VEC_Y.to(env.device).expand(env.num_envs, 3)
    vec_z = VEC_Z.to(env.device).expand(env.num_envs, 3)
    
    # Rotate basis vectors to world frame
    ee_z = quat_apply(ee_quat, vec_z)  # EE Pointing
    ee_x = quat_apply(ee_quat, vec_x)  # EE Pinch
    
    handle_x = quat_apply(handle_quat, vec_x)  # Handle Pointing 
    handle_y = quat_apply(handle_quat, vec_y)  # Handle Pinch
    
    # Compute dot products with the corrected axes
    dot_z = torch.sum(ee_z * handle_x, dim=-1)
    dot_x = torch.sum(ee_x * handle_y, dim=-1)
    
    # Bad alignment if pointing is off OR roll is off (accounting for symmetry)
    is_badly_aligned = ((dot_z < 0.9) | (torch.abs(dot_x) < 0.9)).float()
    
    # Scale the penalty so it gets worse the closer you are while unaligned
    severity = torch.clamp(0.10 - distance, min=0.0) * 10.0 # Scales from 0.0 to 1.0
    
    # Apply penalty
    return -1.0 * is_close * is_badly_aligned * severity
    
def rest_at_goal_penalty(
    env: ManagerBasedRLEnv, 
    threshold: float, 
    door_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Penalize the robot for continuing to move its arm after the door is successfully open.
    """
    # Check if the task is complete (door is Open)
    door = env.scene[door_cfg.name]
    door_joint_idx = door.find_joints("HingeJoint")[0][0]
    door_pos = torch.abs(door.data.joint_pos[:, door_joint_idx])
    
    is_open = (door_pos >= threshold).float()
    
    # Get the velocities of the robot's arm joints
    robot: Articulation = env.scene[robot_cfg.name]
    
    # Only want to penalize arm joints 
    arm_joint_indices, _ = robot.find_joints("joint_[1-7]") 
    
    # Get the velocities of those specific joints
    arm_velocities = robot.data.joint_vel[:, arm_joint_indices]
    
    # Calculate the sum of squared velocities (L2 norm squared)
    velocity_penalty = torch.sum(torch.square(arm_velocities), dim=-1)
    
    # Apply penalty only if the door is open
    return -1.0 * is_open * velocity_penalty

# ==========================================
# TERMINATIONS
# ==========================================

def door_opened(
    env: ManagerBasedRLEnv, 
    threshold: float, 
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Termination condition for successfully opening the door."""
    door: Articulation = env.scene[asset_cfg.name]
    door_joint_idx = door.find_joints("HingeJoint")[0][0]
    door_pos = door.data.joint_pos[:, door_joint_idx]
    return door_pos > threshold