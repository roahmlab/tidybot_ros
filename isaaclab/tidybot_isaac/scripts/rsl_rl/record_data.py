# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to play a checkpoint and log data to CSV."""

import argparse
import sys
import os
import time
import torch
import gymnasium as gym
import pandas as pd
import numpy as np
from datetime import datetime

from isaaclab.app import AppLauncher

# local imports
import cli_args  # isort: skip

# add argparse arguments
parser = argparse.ArgumentParser(description="Run RL agent and record data to CSV.")
parser.add_argument("--save_path", type=str, default=None, help="Path to save the CSV data.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--agent", type=str, default="rsl_rl_cfg_entry_point", help="Name of the RL agent configuration entry point.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument("--use_pretrained_checkpoint", action="store_true", help="Use the pre-trained checkpoint from Nucleus.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")

# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()

# Handle default save path with timestamp
if args_cli.save_path is None:
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    args_cli.save_path = f"logs/data_log/data_log_{timestamp}.csv"

# Check if task is providednable cameras if recording video
if args_cli.video:
    args_cli.enable_cameras = True

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Valid imports after launching app
from rsl_rl.runners import DistillationRunner, OnPolicyRunner
from isaaclab.envs import DirectMARLEnv, DirectRLEnvCfg, ManagerBasedRLEnvCfg, multi_agent_to_single_agent
from isaaclab.utils.assets import retrieve_file_path
from isaaclab_rl.rsl_rl import RslRlBaseRunnerCfg, RslRlVecEnvWrapper, export_policy_as_jit, export_policy_as_onnx
from isaaclab_rl.utils.pretrained_checkpoint import get_published_pretrained_checkpoint
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

import tidybot_isaac.tasks  # noqa: F401


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg, agent_cfg: RslRlBaseRunnerCfg):
    """Play with RSL-RL agent and record data."""
    
    # 1. Configure Environment
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else 1
    env_cfg.seed = agent_cfg.seed if agent_cfg.seed is not None else 42
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else "cuda:0"

    # 2. Create Environment
    print(f"[INFO] Creating environment: {args_cli.task}")
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
    
    # Wrap for RSL-RL
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)
    
    # Video recording wrapper (optional)
    if args_cli.video:
        env = gym.wrappers.RecordVideo(env, video_folder="videos", step_trigger=lambda x: x==0, video_length=args_cli.video_length)

    # RSL-RL wrapper
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    # 3. Load Policy
    # Determine checkpoint path
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    train_task_name = args_cli.task.split(":")[-1].replace("-Play", "")
    
    if args_cli.use_pretrained_checkpoint:
        resume_path = get_published_pretrained_checkpoint("rsl_rl", train_task_name)
    elif args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    print(f"[INFO] Loading checkpoint from: {resume_path}")
    
    # Load runner
    if agent_cfg.class_name == "OnPolicyRunner":
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    elif agent_cfg.class_name == "DistillationRunner":
        runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    else:
        raise ValueError(f"Unsupported runner class: {agent_cfg.class_name}")
    
    runner.load(resume_path)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    # 4. Prepare Data Logging
    data_list = []
    
    # Access internal scene for direct data extraction
    scene = env.unwrapped.scene
    left_contact_sensor = scene.sensors["contact_forces_left"]
    right_contact_sensor = scene.sensors["contact_forces_right"]
    cabinet = scene["cabinet"]
    
    print(f"[INFO] Left contact sensor bodies: {left_contact_sensor.body_names}")
    print(f"[INFO] Right contact sensor bodies: {right_contact_sensor.body_names}")
    
    # Identify Drawer Link for Velocity
    drawer_body_name = "drawer_top"
    try:
        drawer_body_idx = cabinet.body_names.index(drawer_body_name)
    except ValueError:
        print(f"[WARNING] '{drawer_body_name}' not found in cabinet bodies: {cabinet.body_names}. Using index 0.")
        drawer_body_idx = 0

    print(f"[INFO] Drawer Body Index: {drawer_body_idx} ({cabinet.body_names[drawer_body_idx]})")

    # 5. Run Simulation Loop
    obs = env.get_observations()
    
    print("[INFO] Starting simulation loop...")
    
    sim_dt = env.unwrapped.step_dt
    for i in range(1000): # Run for fixed steps or length
        # Policy Step
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)
        
        if not simulation_app.is_running():
            break

        # Extract Data (from 0-th environment)
        env_idx = 0 
        
        # Time
        current_time = i * sim_dt
        
        # Forces — friction_forces_w filtered to drawer handle
        # (matches open_drawer_collect_data.py approach)
        # Shape: (B, M, 3) where B=bodies tracked, M=filter prims
        f_left_raw = left_contact_sensor.data.friction_forces_w[env_idx]
        f_right_raw = right_contact_sensor.data.friction_forces_w[env_idx]
        
        # Replace NaN with 0 (NaN means no contact with filter body)
        f_left_raw = torch.nan_to_num(f_left_raw, nan=0.0)
        f_right_raw = torch.nan_to_num(f_right_raw, nan=0.0)
        
        # Sum over bodies and filter prims to get (3,) per finger
        if f_left_raw.dim() == 3:
            left_force_w = f_left_raw.sum(dim=0).sum(dim=0)
        else:
            left_force_w = f_left_raw.sum(dim=0)
        
        if f_right_raw.dim() == 3:
            right_force_w = f_right_raw.sum(dim=0).sum(dim=0)
        else:
            right_force_w = f_right_raw.sum(dim=0)
        
        # Get Orientations for Local Frame Transformation
        # Helper to get quat for a body name pattern
        def get_body_quat(articulation, pattern):
            # Find index
            for idx, name in enumerate(articulation.body_names):
                if pattern in name:
                    return articulation.data.body_quat_w[env_idx, idx]
            return torch.tensor([1.0, 0.0, 0.0, 0.0], device=articulation.device) # Identity default

        left_quat = get_body_quat(scene["robot"], "left_inner_finger") # Approximate link name
        right_quat = get_body_quat(scene["robot"], "right_inner_finger")
        ee_quat = get_body_quat(scene["robot"], "robotiq_arg2f_base_link") # Hand frame
        
        # Transform Forces to Local Frame
        from isaaclab.utils.math import quat_apply_inverse
        
        left_force_l = quat_apply_inverse(left_quat, left_force_w)
        right_force_l = quat_apply_inverse(right_quat, right_force_w)
        
        # Hand Force: Sum of World Forces -> Rotated to Base Link Frame
        total_force_w = left_force_w + right_force_w
        hand_force_l = quat_apply_inverse(ee_quat, total_force_w)
        
        # Drawer State
        # Linear Velocity in World Frame: (num_envs, num_bodies, 3)
        drawer_vel_w = cabinet.data.body_lin_vel_w[env_idx, drawer_body_idx]
        
        # Rotate Drawer Velocity to Local Frame
        drawer_quat = cabinet.data.body_quat_w[env_idx, drawer_body_idx]
        drawer_vel_l = quat_apply_inverse(drawer_quat, drawer_vel_w)
        
        # Let's populate the row
        row = {
            "Time(s)": current_time,
            
            "Hand_Fx": hand_force_l[0].item(),
            "Hand_Fy": hand_force_l[1].item(),
            "Hand_Fz": hand_force_l[2].item(),
            
            "Left_Fx": left_force_l[0].item(),
            "Left_Fy": left_force_l[1].item(),
            "Left_Fz": left_force_l[2].item(),
            
            "Right_Fx": right_force_l[0].item(),
            "Right_Fy": right_force_l[1].item(),
            "Right_Fz": right_force_l[2].item(),
            
            "Drawer_Vx": drawer_vel_l[0].item(),
            "Drawer_Vy": drawer_vel_l[1].item(),
            "Drawer_Vz": drawer_vel_l[2].item(),
            
            # Acceleration placeholder
            "Drawer_Ax": 0.0, 
            "Drawer_Ay": 0.0, 
            "Drawer_Az": 0.0, 
        }
        
        data_list.append(row)
        
        if i % 100 == 0:
            print(f"Step {i}: Hand Force (Local Z): {hand_force_l[2].item():.2f} N")

    # 6. Save Data
    df = pd.DataFrame(data_list)
    
    # Ensure directory exists
    save_dir = os.path.dirname(args_cli.save_path)
    if save_dir and not os.path.exists(save_dir):
        os.makedirs(save_dir)
        
    df.to_csv(args_cli.save_path, index=False, float_format='%.18e')
    print(f"[INFO] Data saved to {args_cli.save_path}")
    
    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()
