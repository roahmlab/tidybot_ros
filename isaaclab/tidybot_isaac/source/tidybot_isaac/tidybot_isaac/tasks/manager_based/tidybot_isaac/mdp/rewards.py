# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import wrap_to_pi

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def joint_pos_target_l2(env: ManagerBasedRLEnv, target: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize joint position deviation from a target value. Returns negative error."""
    asset: Articulation = env.scene[asset_cfg.name]
    joint_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]
    # Return NEGATIVE squared error (penalty)
    return -torch.sum(torch.square(joint_pos - target), dim=1)


def end_effector_to_handle_distance(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    cabinet_cfg: SceneEntityCfg,
    ee_frame_name: str = "robotiq_arg2f_base_link",
) -> torch.Tensor:
    """Reward for moving end-effector closer to drawer handle.
    
    Returns negative distance (closer = higher reward).
    """
    robot: Articulation = env.scene[robot_cfg.name]
    cabinet: Articulation = env.scene[cabinet_cfg.name]
    
    # Get end-effector position (tool frame)
    ee_pos = robot.data.body_pos_w[:, robot.find_bodies(ee_frame_name)[0][0], :3]
    
    # Get drawer handle position (approximate as drawer link position + offset)
    # The drawer_top link is the drawer body
    drawer_link_idx = cabinet.find_bodies("drawer_top")[0][0]
    handle_pos = cabinet.data.body_pos_w[:, drawer_link_idx, :3]
    # Offset handle position forward (in local x direction of drawer)
    handle_offset = torch.tensor([0.0, 0.3, 0.0], device=handle_pos.device)
    handle_pos = handle_pos + handle_offset
    
    # Compute distance
    distance = torch.norm(ee_pos - handle_pos, dim=1)
    
    # Return negative distance (closer = less penalty)
    return -distance


def gripper_grasp_reward(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    cabinet_cfg: SceneEntityCfg,
    ee_frame_name: str = "robotiq_arg2f_base_link",
    grasp_threshold: float = 0.15,
) -> torch.Tensor:
    """Reward for closing gripper when near the handle.
    
    Only rewards gripper closing when end-effector is close to handle.
    """
    robot: Articulation = env.scene[robot_cfg.name]
    cabinet: Articulation = env.scene[cabinet_cfg.name]
    
    # Get end-effector position
    ee_pos = robot.data.body_pos_w[:, robot.find_bodies(ee_frame_name)[0][0], :3]
    
    # Get handle position
    drawer_link_idx = cabinet.find_bodies("drawer_top")[0][0]
    handle_pos = cabinet.data.body_pos_w[:, drawer_link_idx, :3]
    handle_offset = torch.tensor([0.0, 0.3, 0.0], device=handle_pos.device)
    handle_pos = handle_pos + handle_offset
    
    # Distance to handle
    distance = torch.norm(ee_pos - handle_pos, dim=1)
    
    # Get gripper position (finger_joint)
    finger_joint_idx = robot.find_joints("finger_joint")[0][0]
    gripper_pos = robot.data.joint_pos[:, finger_joint_idx]
    
    # Reward closing gripper (higher position = more closed) when near handle
    near_handle = (distance < grasp_threshold).float()
    grasp_reward = near_handle * gripper_pos  # [0, 0.82] when near
    
    return grasp_reward


def drawer_opening_progress(
    env: ManagerBasedRLEnv,
    cabinet_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Reward for drawer opening progress. Returns drawer position directly."""
    cabinet: Articulation = env.scene[cabinet_cfg.name]
    drawer_joint_idx = cabinet.find_joints("drawer_top_joint")[0][0]
    drawer_pos = cabinet.data.joint_pos[:, drawer_joint_idx]
    return drawer_pos  # Positive when drawer is pulled out


def drawer_opened_bonus(
    env: ManagerBasedRLEnv,
    cabinet_cfg: SceneEntityCfg,
    threshold: float = 0.35,
) -> torch.Tensor:
    """Large bonus when drawer is opened past threshold."""
    cabinet: Articulation = env.scene[cabinet_cfg.name]
    drawer_joint_idx = cabinet.find_joints("drawer_top_joint")[0][0]
    drawer_pos = cabinet.data.joint_pos[:, drawer_joint_idx]
    return (drawer_pos > threshold).float()


def action_smoothness(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize large action changes for smoother motion."""
    # Use action manager's processed actions
    actions = env.action_manager.action
    return -torch.sum(torch.square(actions), dim=1)
