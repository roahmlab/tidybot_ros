# Copyright (c) 2022-2026, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def end_effector_pos(env: ManagerBasedRLEnv) -> torch.Tensor:
    """End-effector world position (3D).
    
    Returns the world position of the end-effector body.
    """
    # Use canonical frame if available
    return env.scene["ee_frame"].data.target_pos_w[..., 0, :]


def handle_pos(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Drawer handle world position (3D).
    Returns the world position of the drawer handle (drawer_top link + offset).
    """
    # Use canonical frame if available
    return env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]


def ee_to_handle_vector(env: ManagerBasedRLEnv) -> torch.Tensor:
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
    gripper_joint_name: str,
    open_pos: float,
    close_pos: float,
) -> torch.Tensor:
    """Normalized gripper opening amount (1D).
    
    Normalizes any hardware's raw joint position into a [0.0, 1.0] scale, 
    where 1.0 is fully CLOSED and 0.0 is fully OPEN.
    """
    robot: Articulation = env.scene[robot_cfg.name]
    gripper_joint_idx = robot.find_joints(gripper_joint_name)[0][0]
    
    raw_pos = robot.data.joint_pos[:, gripper_joint_idx:gripper_joint_idx+1]
    
    # 1.0 is Closed, 0.0 is Open
    normalized_pos = (raw_pos - open_pos) / (close_pos - open_pos)
    
    return torch.clamp(normalized_pos, min=0.0, max=1.0)