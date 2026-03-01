# Copyright (c) 2022-2026, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

from isaaclab.utils.math import quat_apply

# ==========================================
# OBSERVATIONS
# ==========================================

def rel_ee_drawer_distance(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Returns the 3D relative vector from the end-effector to the handle."""
    ee_pos = env.scene["ee_frame"].data.target_pos_w[..., 0, :]
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]
    return handle_pos - ee_pos

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

def align_ee_handle(
    env: ManagerBasedRLEnv,
) -> torch.Tensor:
    """Reward for aligning EE tool frame with handle frame.
    
    Conditions:
    1. Z_ee aligns with Z_handle
    2. X_ee aligns with Y_handle
    
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
    
    # Basis Vectors
    vec_z = torch.tensor([0.0, 0.0, 1.0], device=env.device).expand(env.num_envs, 3)
    vec_x = torch.tensor([1.0, 0.0, 0.0], device=env.device).expand(env.num_envs, 3)
    vec_y = torch.tensor([0.0, 1.0, 0.0], device=env.device).expand(env.num_envs, 3)
    
    # Rotate to World
    ee_z = quat_apply(ee_quat, vec_z)
    ee_x = quat_apply(ee_quat, vec_x)
    
    handle_z = quat_apply(handle_quat, vec_z)
    handle_y = quat_apply(handle_quat, vec_y)
    
    # Dot Products (Alignments)
    align_z_z = torch.sum(ee_z * handle_z, dim=-1)
    align_x_y = torch.sum(ee_x * handle_y, dim=-1)
    
    # Reward
    # Max value = 2.0 * scaling
    alignment = (align_z_z + align_x_y)
    
    return alignment * scaling

def grasp_handle(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    gripper_joint_name: str = "hande_left_finger_joint",
) -> torch.Tensor:
    robot: Articulation = env.scene[robot_cfg.name]
    
    # 1. Distance check (using your existing frames)
    ee_pos = env.scene["ee_frame"].data.target_pos_w[..., 0, :]
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]
    distance = torch.norm(handle_pos - ee_pos, dim=-1)
    
    # 2. Gripper State (Check your specific USD limits!)
    # Let's assume 0.025 is open and 0.0 is closed.
    gripper_joint_idx = robot.find_joints(gripper_joint_name)[0][0]
    gripper_pos = robot.data.joint_pos[:, gripper_joint_idx]
    
    # 3. Normalized "Closedness" (1.0 = fully closed, 0.0 = fully open)
    # This creates a "Slope" for the agent to follow
    closedness = 1.0 - (gripper_pos / 0.025)
    closedness = torch.clamp(closedness, 0.0, 1.0)
    
    # 4. Only reward closing if we are within 5cm of the handle
    # This prevents the robot from just standing in the air and opening/closing for points
    proximity_gate = torch.where(distance < 0.05, 1.0, 0.0)
    
    return proximity_gate * closedness

def open_drawer_bonus(
    env: ManagerBasedRLEnv, 
    cabinet_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Bonus for opening the drawer - returns drawer position directly."""
    cabinet: Articulation = env.scene[cabinet_cfg.name]
    drawer_joint_idx = cabinet.find_joints("drawer_top_joint")[0][0]
    drawer_pos = cabinet.data.joint_pos[:, drawer_joint_idx]
    return drawer_pos

def action_rate_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize the change between current and previous actions for smoothness."""
    current_action = env.action_manager.action
    previous_action = env.action_manager.prev_action
    
    # Calculate the squared difference
    # Penalizing (a_t - a_{t-1})^2
    return -torch.sum(torch.square(current_action - previous_action), dim=1)

def hold_open_bonus(
    env: ManagerBasedRLEnv, 
    threshold: float, 
    cabinet_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Provides a continuous stream of points every timestep the drawer is held open."""
    cabinet: Articulation = env.scene[cabinet_cfg.name]
    drawer_joint_idx = cabinet.find_joints("drawer_top_joint")[0][0]
    drawer_pos = cabinet.data.joint_pos[:, drawer_joint_idx]
    
    # Returns 1.0 EVERY timestep the drawer is > threshold, 0.0 otherwise
    return (drawer_pos > threshold).float()
    
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