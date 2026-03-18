# Copyright (c) 2022-2026, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
import isaaclab.utils.math as math_utils
from isaaclab.utils.math import quat_conjugate, quat_mul, quat_apply

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

# ==========================================
# OBSERVATIONS
# ==========================================

def rel_ee_handle_transform(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Returns the relative translation and directional vectors (X, Z) from EE to handle.
    Everything is expressed in the End Effector's local frame for translation invariance.
    Output shape: [num_envs, 9] (3 for pos, 3 for X vec, 3 for Z vec)
    """
    # Get world positions
    ee_pos = env.scene["ee_frame"].data.target_pos_w[..., 0, :]
    handle_pos = env.scene["handle_frame"].data.target_pos_w[..., 0, :]
    
    # Get world orientations
    ee_quat = env.scene["ee_frame"].data.target_quat_w[..., 0, :]
    handle_quat = env.scene["handle_frame"].data.target_quat_w[..., 0, :]
    
    # 1. Relative Position (in EE Local Frame)
    pos_error_world = handle_pos - ee_pos
    ee_quat_inv = quat_conjugate(ee_quat)
    pos_error_local = quat_apply(ee_quat_inv, pos_error_world)
    
    # 2. Relative Rotation (Quat from EE to Handle)
    rot_error = quat_mul(ee_quat_inv, handle_quat)
    
    # 3. Handle's Local X and Z axes expressed in EE Frame
    x_axis = torch.zeros_like(pos_error_world); x_axis[..., 0] = 1.0
    z_axis = torch.zeros_like(pos_error_world); z_axis[..., 2] = 1.0
    
    handle_x_in_ee = quat_apply(rot_error, x_axis)
    handle_z_in_ee = quat_apply(rot_error, z_axis)
    
    # Concatenate: [local_x, local_y, local_z, x_x, x_y, x_z, z_x, z_y, z_z]
    return torch.cat([pos_error_local, handle_x_in_ee, handle_z_in_ee], dim=-1)

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

def gripper_cooldown_state(env: ManagerBasedRLEnv, action_term_name: str = "gripper") -> torch.Tensor:
    """Returns the normalized remaining cooldown time [0.0 to 1.0]. 0.0 means ready."""
    action_term = env.action_manager.get_term(action_term_name)
    
    # Normalize the timer so the neural network digests it easily
    max_steps = action_term.cooldown_steps
    current_timers = action_term.cooldown_timers.float()
    
    # Add a dimension so it concatenates properly in the observation vector
    return (current_timers / max_steps).unsqueeze(-1)

def ee_to_hinge_in_ee_frame(env: ManagerBasedRLEnv, hinge_cfg: SceneEntityCfg) -> torch.Tensor:
    """
    Returns the 3D vector pointing from the End Effector to the Hinge Origin, 
    expressed entirely in the EE's local coordinate frame.
    Output shape: [num_envs, 3]
    """
    door_asset = env.scene[hinge_cfg.name]
    
    # 1. Get positions in the world frame
    ee_pos_w = env.scene["ee_frame"].data.target_pos_w[..., 0, :]
    hinge_pos_w = door_asset.data.body_pos_w[:, hinge_cfg.body_ids[0], :]
    
    # 2. Get EE orientation in the world frame
    ee_quat_w = env.scene["ee_frame"].data.target_quat_w[..., 0, :]
    
    # 3. Calculate the vector from EE to Hinge in the World Frame
    ee_to_hinge_w = hinge_pos_w - ee_pos_w
    
    # 4. Rotate the world vector into the EE Local Frame
    ee_quat_inv = math_utils.quat_conjugate(ee_quat_w)
    ee_to_hinge_ee = math_utils.quat_apply(ee_quat_inv, ee_to_hinge_w)
    
    return ee_to_hinge_ee

def hinge_axis_in_ee_frame(env: ManagerBasedRLEnv, door_cfg: SceneEntityCfg) -> torch.Tensor:
    """
    Returns the hinge's opening axis (Z-axis of the hinge) 
    expressed in the EE's local frame.
    Output shape: [num_envs, 3]
    """
    door = env.scene[door_cfg.name]
    
    # 1. Get orientations in the world frame
    hinge_quat_w = door.data.body_quat_w[:, door_cfg.body_ids[0]]
    ee_quat_w = env.scene["ee_frame"].data.target_quat_w[..., 0, :]
    
    # 2. Calculate the relative rotation from EE to Hinge
    ee_quat_inv = math_utils.quat_conjugate(ee_quat_w)
    rot_ee_to_hinge = math_utils.quat_mul(ee_quat_inv, hinge_quat_w)
    
    # 3. Define the hinge's local opening axis (Assuming Z-axis is the hinge pin)
    local_z_axis = torch.zeros((env.num_envs, 3), device=env.device)
    local_z_axis[:, 2] = 1.0
    
    # 4. Apply the relative rotation to map the hinge's axis into the EE frame
    hinge_axis_ee = math_utils.quat_apply(rot_ee_to_hinge, local_z_axis)
    
    return hinge_axis_ee

def door_position(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Returns the current joint position (angle) of the door's hinge.
    Output shape: [num_envs, 1]
    """
    door_asset = env.scene[asset_cfg.name]
    joint_pos = door_asset.data.joint_pos[:, asset_cfg.joint_ids]
    
    return joint_pos

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