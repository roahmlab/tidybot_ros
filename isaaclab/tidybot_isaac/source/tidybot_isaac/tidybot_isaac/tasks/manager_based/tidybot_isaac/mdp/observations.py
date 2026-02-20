# Copyright (c) 2022-2026, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def end_effector_pos(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    ee_body_name: str = "robotiq_arg2f_base_link",
) -> torch.Tensor:
    """End-effector world position (3D).
    
    Returns the world position of the end-effector body.
    """
    # Use canonical frame if available
    return env.scene["ee_frame"].data.target_pos_w[..., 0, :]


def handle_pos(
    env: ManagerBasedRLEnv,
    cabinet_cfg: SceneEntityCfg,
    handle_offset: tuple[float, float, float] = (0.0, 0.3, 0.0),
) -> torch.Tensor:
    """Drawer handle world position (3D).
    
    Returns the world position of the drawer handle (drawer_top link + offset).
    """
    # Use canonical frame if available
    return env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]


def ee_to_handle_vector(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    cabinet_cfg: SceneEntityCfg,
    ee_body_name: str = "robotiq_arg2f_base_link",
    handle_offset: tuple[float, float, float] = (0.0, 0.3, 0.0),
) -> torch.Tensor:
    """Vector from end-effector to drawer handle (3D).
    
    This is the most useful observation for reaching - it directly tells
    the policy which direction and how far to move.
    """
    # Get end-effector position from FrameTransformer
    ee_pos = env.scene["ee_frame"].data.target_pos_w[..., 0, :]
    
    # Get handle position
    # Get handle position from FrameTransformer
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]
    
    # Return vector from EE to handle
    return handle_pos - ee_pos


def gripper_open_amount(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    gripper_joint_name: str = "finger_joint",
) -> torch.Tensor:
    """Gripper opening amount (1D).
    
    Returns the finger_joint position (0 = open, 0.82 = closed).
    """
    robot: Articulation = env.scene[robot_cfg.name]
    gripper_joint_idx = robot.find_joints(gripper_joint_name)[0][0]
    return robot.data.joint_pos[:, gripper_joint_idx:gripper_joint_idx+1]
