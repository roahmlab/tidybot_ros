# Copyright (c) 2022-2026, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

from isaaclab.utils.math import quat_rotate


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
    proximity = 1.0 / (1.0 + distance**2)
    
    # Add boost when very close to keep gradient strong
    proximity = torch.where(distance < 0.05, proximity * 2.0, proximity)
    proximity = torch.where(distance < 0.02, proximity * 3.0, proximity)
    
    # Basis Vectors
    vec_z = torch.tensor([0.0, 0.0, 1.0], device=env.device).expand(env.num_envs, 3)
    vec_x = torch.tensor([1.0, 0.0, 0.0], device=env.device).expand(env.num_envs, 3)
    vec_y = torch.tensor([0.0, 1.0, 0.0], device=env.device).expand(env.num_envs, 3)
    
    # Rotate to World
    ee_z = quat_rotate(ee_quat, vec_z)
    ee_x = quat_rotate(ee_quat, vec_x)
    
    handle_z = quat_rotate(handle_quat, vec_z)
    handle_y = quat_rotate(handle_quat, vec_y)
    
    # Dot Products (Alignments)
    align_z_z = torch.sum(ee_z * handle_z, dim=-1)
    align_x_y = torch.sum(ee_x * handle_y, dim=-1)
    
    # Reward
    # Max value = 2.0 * scaling
    alignment = (align_z_z + align_x_y)
    
    return alignment * proximity


def approach_ee_handle(
    env: ManagerBasedRLEnv, 
    threshold: float,
    robot_cfg: SceneEntityCfg,
    cabinet_cfg: SceneEntityCfg,
    ee_body_name: str = "robotiq_arg2f_base_link",
) -> torch.Tensor:
    """Reward for approaching the handle.
    
    Smoothed inverse-square reward with bonuses for being close.
    """
    robot: Articulation = env.scene[robot_cfg.name]
    # cabinet: Articulation = env.scene[cabinet_cfg.name] # Not needed for frame lookup
    
    # Get end-effector position from FrameTransformer
    ee_pos = env.scene["ee_frame"].data.target_pos_w[..., 0, :]

    # Get handle position from FrameTransformer (Canonical Way)
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]

    # Compute distance
    distance = torch.norm(handle_pos - ee_pos, dim=-1, p=2)
    
    # Inverse-square reward (smoother gradient)
    reward = 1.0 / (1.0 + distance**2)
    reward = torch.pow(reward, 2)
    
    # Multi-stage bonuses for being close
    # Threshold (param) is typically 0.5
    reward = torch.where(distance <= threshold, 1.5 * reward, reward)
    reward = torch.where(distance <= 0.1, 2.0 * reward, reward)
    reward = torch.where(distance <= 0.05, 3.0 * reward, reward) # Very close bonus
    
    return reward


def grasp_handle(
    env: ManagerBasedRLEnv,
    threshold: float,
    open_joint_pos: float,
    robot_cfg: SceneEntityCfg,
    cabinet_cfg: SceneEntityCfg,
    ee_body_name: str = "robotiq_arg2f_base_link",
    gripper_joint_name: str = "finger_joint",
) -> torch.Tensor:
    """Reward for closing the fingers when being close to the handle.
    
    Only rewards gripper closing when within threshold distance of handle.
    """
    robot: Articulation = env.scene[robot_cfg.name]
    cabinet: Articulation = env.scene[cabinet_cfg.name]
    
    # Get end-effector position from FrameTransformer
    ee_pos = env.scene["ee_frame"].data.target_pos_w[..., 0, :]
    
    # Get handle position
    # Get handle position from FrameTransformer (Canonical Way)
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]
    
    # Compute distance
    distance = torch.norm(handle_pos - ee_pos, dim=-1, p=2)
    is_close = distance <= threshold
    
    # Get gripper joint position
    gripper_joint_idx = robot.find_joints(gripper_joint_name)[0][0]
    gripper_pos = robot.data.joint_pos[:, gripper_joint_idx]
    
    # Reward closing gripper (higher pos = more closed) when close
    # open_joint_pos - gripper_pos gives 0 when fully closed, max when open
    # We want to reward closing, so use gripper_pos directly
    return is_close.float() * gripper_pos


def open_drawer_bonus(
    env: ManagerBasedRLEnv, 
    cabinet_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Bonus for opening the drawer - returns drawer position directly."""
    cabinet: Articulation = env.scene[cabinet_cfg.name]
    drawer_joint_idx = cabinet.find_joints("drawer_top_joint")[0][0]
    drawer_pos = cabinet.data.joint_pos[:, drawer_joint_idx]
    return drawer_pos


def multi_stage_open_drawer(
    env: ManagerBasedRLEnv, 
    cabinet_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Multi-stage bonus for opening the drawer.

    Provides progressive rewards at different stages:
    - 0.5 bonus when drawer > 0.01 (any movement)
    - 1.0 bonus when drawer > 0.15 (medium open)
    - 1.0 bonus when drawer > 0.30 (fully open)
    """
    cabinet: Articulation = env.scene[cabinet_cfg.name]
    drawer_joint_idx = cabinet.find_joints("drawer_top_joint")[0][0]
    drawer_pos = cabinet.data.joint_pos[:, drawer_joint_idx]
    
    # Progressive bonuses
    open_easy = (drawer_pos > 0.01).float() * 0.5
    open_medium = (drawer_pos > 0.15).float() * 1.0
    open_hard = (drawer_pos > 0.30).float() * 1.0
    
    return open_easy + open_medium + open_hard


def action_rate_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize large changes in actions for smoother motion."""
    # Penalize action magnitude
    actions = env.action_manager.action
    return -torch.sum(torch.square(actions), dim=1)
