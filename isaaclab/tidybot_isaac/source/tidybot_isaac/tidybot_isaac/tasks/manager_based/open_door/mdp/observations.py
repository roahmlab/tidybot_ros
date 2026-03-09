# Copyright (c) 2022-2026, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
import isaaclab.utils.math as math_utils

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

# ==========================================
# OBSERVATIONS
# ==========================================

def rel_ee_handle_transform(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Returns the relative translation (3D) and rotation (4D quat) from EE to handle.
    Output shape: [num_envs, 7]
    """
    # Get positions
    ee_pos = env.scene["ee_frame"].data.target_pos_w[..., 0, :]
    handle_pos = env.scene["handle_frame"].data.target_pos_w[..., 0, :]
    
    # Get orientations
    ee_quat = env.scene["ee_frame"].data.target_quat_w[..., 0, :]
    handle_quat = env.scene["handle_frame"].data.target_quat_w[..., 0, :]
    
    # Relative translation (World frame difference)
    pos_error = handle_pos - ee_pos
    
    # Relative rotation (q_rel = q_ee_inv * q_handle)
    ee_quat_inv = math_utils.quat_conjugate(ee_quat)
    rot_error = math_utils.quat_mul(ee_quat_inv, handle_quat)
    
    # Concatenate into a single tensor: [x, y, z, qw, qx, qy, qz]
    return torch.cat([pos_error, rot_error], dim=-1)

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

def hinge_origin_position(env: ManagerBasedRLEnv, door_cfg: SceneEntityCfg) -> torch.Tensor:
    """
    Returns the hinge origin and the opening axis in the world frame.
    Outputs a 6D tensor: [origin_x, origin_y, origin_z, axis_x, axis_y, axis_z]
    """
    door = env.scene[door_cfg.name]
    
    # Hinge Origin (Anchor)
    root_pos_w = door.data.root_pos_w  # Shape: (num_envs, 3)
    root_quat_w = door.data.root_quat_w  # Shape: (num_envs, 4)
    
    # Opening Axis
    # The canonical door hinge rotates on the local Z-axis [0, 0, 1].
    # We rotate this local Z-axis by the root's quaternion to get the world-frame axis.
    local_z_axis = torch.zeros((env.num_envs, 3), device=env.device)
    local_z_axis[:, 2] = 1.0
    hinge_axis_w = math_utils.quat_apply(root_quat_w, local_z_axis)
    
    # Concatenate origin and axis into a 6D observation
    return torch.cat([root_pos_w, hinge_axis_w], dim=-1)


def handle_initial_position(env: ManagerBasedRLEnv, door_cfg: SceneEntityCfg) -> torch.Tensor:
    """
    Calculates the 'closed' position of the handle dynamically by rotating 
    the current handle position backwards around the hinge axis by -theta.
    """
    door = env.scene[door_cfg.name]
    
    # Get current Handle position using the body_ids resolved by SceneEntityCfg
    handle_idx = door_cfg.body_ids[0]
    handle_pos_w = door.data.body_pos_w[:, handle_idx, :]  # Shape: (num_envs, 3)
    
    # Get current Joint angle (theta)
    theta = door.data.joint_pos[:, 0]  # Shape: (num_envs,)
    
    # Get Origin and Axis
    root_pos_w = door.data.root_pos_w
    root_quat_w = door.data.root_quat_w
    
    local_z_axis = torch.zeros((env.num_envs, 3), device=env.device)
    local_z_axis[:, 2] = 1.0
    axis_w = math_utils.quat_apply(root_quat_w, local_z_axis)
    
    # Perform the reverse rotation
    # Vector from hinge origin to current handle
    v_current = handle_pos_w - root_pos_w
    
    # Create a quaternion representing the reverse rotation (-theta) around the axis
    q_rot_reverse = math_utils.quat_from_angle_axis(-theta, axis_w)
    
    # Apply the rotation to the vector
    v_initial = math_utils.quat_apply(q_rot_reverse, v_current)
    
    # Translate the rotated vector back to the world frame origin
    handle_initial_pos_w = root_pos_w + v_initial
    
    return handle_initial_pos_w