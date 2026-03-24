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

def rel_ee_drawer_transform(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Returns the relative translation (3D) and rotation (4D quat) from EE to handle.
    Output shape: [num_envs, 7]
    """
    # Get positions
    ee_pos = env.scene["ee_frame"].data.target_pos_w[..., 0, :]
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]
    
    # Get orientations
    ee_quat = env.scene["ee_frame"].data.target_quat_w[..., 0, :]
    handle_quat = env.scene["cabinet_frame"].data.target_quat_w[..., 0, :]
    
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

def log_cumulative_mechanical_work(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Tracks total mechanical work per episode and logs it to TensorBoard."""
    if not hasattr(env, "_episode_energy_buf"):
        env._episode_energy_buf = torch.zeros(env.num_envs, device=env.device)

    robot = env.scene[asset_cfg.name]
    
    # torques * velocities = Power
    power = torch.sum(torch.abs(robot.data.applied_torque * robot.data.joint_vel), dim=-1)
    env._episode_energy_buf += power * env.step_dt

    # When environments reset, calculate the mean and ship to extras
    if env.reset_buf.any():
        reset_ids = env.reset_buf.nonzero(as_tuple=True)[0]
        
        if "log" not in env.extras:
            env.extras["log"] = {}

        # Log the average energy of all resetting environments
        env.extras["log"]["Metric/Total_Energy"] = torch.mean(env._episode_energy_buf[reset_ids]).item()

        # Wipe the buffer for the new episode
        env._episode_energy_buf[reset_ids] = 0.0

    # Weight of 1e-8 ensures this runs every step
    return torch.zeros(env.num_envs, device=env.device)

def log_squared_torque_effort(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Tracks squared joint torque to expose wasted isometric pulling effort for specific joints."""
    if not hasattr(env, "_episode_effort_buf"):
        env._episode_effort_buf = torch.zeros(env.num_envs, device=env.device)

    robot = env.scene[asset_cfg.name]
    joint_ids = asset_cfg.joint_ids
    arm_torques = robot.data.applied_torque[:, joint_ids]
    
    effort = torch.sum(torch.square(arm_torques), dim=-1)
    env._episode_effort_buf += effort * env.step_dt

    if env.reset_buf.any():
        reset_ids = env.reset_buf.nonzero(as_tuple=True)[0]
        
        if "log" not in env.extras:
            env.extras["log"] = {}

        # Log the average wasted effort of all resetting environments
        env.extras["log"]["Metric/Wasted_Motor_Effort"] = torch.mean(env._episode_effort_buf[reset_ids]).item()
        env._episode_effort_buf[reset_ids] = 0.0

    return torch.zeros(env.num_envs, device=env.device)