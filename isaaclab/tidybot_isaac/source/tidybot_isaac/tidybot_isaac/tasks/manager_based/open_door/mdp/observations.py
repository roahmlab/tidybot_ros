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

def gripper_cooldown_state(env: ManagerBasedRLEnv, action_term_name: str = "gripper") -> torch.Tensor:
    """Returns the normalized remaining cooldown time [0.0 to 1.0]. 0.0 means ready."""
    action_term = env.action_manager.get_term(action_term_name)
    
    # Normalize the timer so the neural network digests it easily
    max_steps = action_term.cooldown_steps
    current_timers = action_term.cooldown_timers.float()
    
    # Add a dimension so it concatenates properly in the observation vector
    return (current_timers / max_steps).unsqueeze(-1)

def hinge_origin(env, door_cfg) -> torch.Tensor:
    """
    Returns the exact hinge origin and opening axis in the environment-local frame.
    Outputs a 6D tensor: [origin_x, origin_y, origin_z, axis_x, axis_y, axis_z]
    """
    door = env.scene[door_cfg.name]
    
    # Get the world position/orientation of the tracked HingeSite body
    hinge_pos_w = door.data.body_pos_w[:, door_cfg.body_ids[0]]
    hinge_quat_w = door.data.body_quat_w[:, door_cfg.body_ids[0]]
    
    # Convert the position to the Environment-Local frame
    hinge_pos_env = hinge_pos_w - env.scene.env_origins
    
    # Opening Axis
    local_z_axis = torch.zeros((env.num_envs, 3), device=env.device)
    local_z_axis[:, 2] = 1.0
    
    # Rotate the local Z axis into the world frame based on the hinge site's orientation
    hinge_axis = math_utils.quat_apply(hinge_quat_w, local_z_axis)
    
    return torch.cat([hinge_pos_env, hinge_axis], dim=-1)

def handle_initial_position(
    env: ManagerBasedRLEnv, 
    handle_cfg: SceneEntityCfg,
    hinge_cfg: SceneEntityCfg
) -> torch.Tensor:
    """
    Calculates the exact 'closed' position of the handle by pivoting the current 
    handle position backwards around the true hinge origin by the joint angle (-theta).
    Returns the coordinates in the environment-local frame.
    """
    door = env.scene[handle_cfg.name]
    
    # Get exact World Positions for Handle and HingeOrigin
    handle_idx = handle_cfg.body_ids[0]
    hinge_idx = hinge_cfg.body_ids[0]
    
    handle_pos_w = door.data.body_pos_w[:, handle_idx, :]   # Shape: (num_envs, 3)
    hinge_pos_w = door.data.body_pos_w[:, hinge_idx, :]     # Shape: (num_envs, 3)
    
    # Get current Joint angle (theta)
    theta = door.data.joint_pos[:, 0]  # Shape: (num_envs,)
    
    # Determine the Hinge Axis in the World Frame
    # The hinge rotates around the Z-axis.
    # We rotate the local Z-axis [0, 0, 1] by the door's root quaternion 
    # in case the door itself is rotated (e.g., yaw randomization).
    local_z_axis = torch.zeros((env.num_envs, 3), device=env.device)
    local_z_axis[:, 2] = 1.0
    axis_w = math_utils.quat_apply(door.data.root_quat_w, local_z_axis)
    
    # Perform the geometrically perfect reverse rotation
    # Vector from the actual hinge pin to the current handle
    v_current = handle_pos_w - hinge_pos_w
    
    # Create reverse rotation (-theta) around the derived Z-axis
    q_rot_reverse = math_utils.quat_from_angle_axis(-theta, axis_w)
    v_initial = math_utils.quat_apply(q_rot_reverse, v_current)
    
    # Translate back to world frame using the HINGE as the origin point
    handle_initial_pos_w = hinge_pos_w + v_initial
    
    # Convert to Environment-Local Frame
    handle_initial_pos_env = handle_initial_pos_w - env.scene.env_origins
    
    return handle_initial_pos_env

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
    """Tracks squared joint torque to expose wasted isometric pulling effort."""
    if not hasattr(env, "_episode_effort_buf"):
        env._episode_effort_buf = torch.zeros(env.num_envs, device=env.device)

    robot = env.scene[asset_cfg.name]
    
    # Torque squared represents electrical heat loss / wasted isometric strain
    effort = torch.sum(torch.square(robot.data.applied_torque), dim=-1)
    env._episode_effort_buf += effort * env.step_dt

    if env.reset_buf.any():
        reset_ids = env.reset_buf.nonzero(as_tuple=True)[0]
        
        if "log" not in env.extras:
            env.extras["log"] = {}

        # Log the average wasted effort of all resetting environments
        env.extras["log"]["Metric/Wasted_Motor_Effort"] = torch.mean(env._episode_effort_buf[reset_ids]).item()
        env._episode_effort_buf[reset_ids] = 0.0

    return torch.zeros(env.num_envs, device=env.device)