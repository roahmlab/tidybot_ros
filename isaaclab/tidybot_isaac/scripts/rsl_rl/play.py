# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to play a checkpoint if an RL agent from RSL-RL."""

"""Launch Isaac Sim Simulator first."""

import argparse
import sys

from isaaclab.app import AppLauncher

# local imports
import cli_args  # isort: skip

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument(
    "--agent", type=str, default="rsl_rl_cfg_entry_point", help="Name of the RL agent configuration entry point."
)
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument(
    "--use_pretrained_checkpoint",
    action="store_true",
    help="Use the pre-trained checkpoint from Nucleus.",
)
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time, if possible.")

# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()
# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True
# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

kit_args = []
if args_cli.headless:
    # --- TEMPORARY KIT ARGS INJECTION ---
    kit_args = [
        "--/app/livestream/publicEndpointAddress=100.107.14.98",
        "--/app/livestream/fabricEnabled=false",
        "--/app/window/enabled=false",
        "--/app/asyncRendering=false",
        "--/renderer/multithreading/enabled=false",
        "--/physics/useGpuPicking=false",
        "--/physics/useGpuSceneQueries=false",
        "--/omni.physx.plugin/useDirectApi=false",
        "--/rtx/hydra/picking/enabled=true",
        "--/app/livestream/forceGpuCapture=false"
    ]
    sys.argv.extend(kit_args)

# launch omniverse app (It reads sys.argv, sees our flags, and configures rendering perfectly)
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

if args_cli.headless:
    # Clean up: Remove the flags so Hydra doesn't crash when the RL loop starts
    for arg in kit_args:
        sys.argv.remove(arg)
    
"""Rest everything follows."""

import os
import time
import csv

import gymnasium as gym
import torch
from rsl_rl.runners import DistillationRunner, OnPolicyRunner

from isaaclab.envs import (
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.dict import print_dict

from isaaclab_rl.rsl_rl import RslRlBaseRunnerCfg, RslRlVecEnvWrapper, export_policy_as_jit, export_policy_as_onnx
from isaaclab_rl.utils.pretrained_checkpoint import get_published_pretrained_checkpoint

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

import tidybot_isaac.tasks  # noqa: F401

@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlBaseRunnerCfg):
    """Play with RSL-RL agent."""
    # grab task name for checkpoint path
    task_name = args_cli.task.split(":")[-1]
    train_task_name = task_name.replace("-Play", "")

    # override configurations with non-hydra CLI arguments
    agent_cfg: RslRlBaseRunnerCfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs

    # set the environment seed
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    if args_cli.use_pretrained_checkpoint:
        resume_path = get_published_pretrained_checkpoint("rsl_rl", train_task_name)
        if not resume_path:
            print("[INFO] Unfortunately a pre-trained checkpoint is currently unavailable for this task.")
            return
    elif args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    log_dir = os.path.dirname(resume_path)

    # set the log directory for the environment (works for all environment types)
    env_cfg.log_dir = log_dir

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    # convert to single-agent instance if required by the RL algorithm
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    # wrap for video recording
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "play"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # wrap around environment for rsl-rl
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    print(f"[INFO]: Loading model checkpoint from: {resume_path}")
    if agent_cfg.class_name == "OnPolicyRunner":
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    elif agent_cfg.class_name == "DistillationRunner":
        runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    else:
        raise ValueError(f"Unsupported runner class: {agent_cfg.class_name}")
    runner.load(resume_path)

    policy = runner.get_inference_policy(device=env.unwrapped.device)

    try:
        policy_nn = runner.alg.policy
    except AttributeError:
        policy_nn = runner.alg.actor_critic

    if hasattr(policy_nn, "actor_obs_normalizer"):
        normalizer = policy_nn.actor_obs_normalizer
    elif hasattr(policy_nn, "student_obs_normalizer"):
        normalizer = policy_nn.student_obs_normalizer
    else:
        normalizer = None

    export_model_dir = os.path.join(os.path.dirname(resume_path), "exported")
    export_policy_as_jit(policy_nn, normalizer=normalizer, path=export_model_dir, filename="policy.pt")
    export_policy_as_onnx(policy_nn, normalizer=normalizer, path=export_model_dir, filename="policy.onnx")

    dt = env.unwrapped.step_dt
    
    # --- CSV LOGGING SETUP ---
    csv_filepath = "sim_rollout_log.csv"
    csv_file = open(csv_filepath, mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    header = [f'obs_{i}' for i in range(40)] + [f'action_{i}' for i in range(8)]
    csv_writer.writerow(header)
    print(f"[INFO] Logging simulation observations for Env 0 to {csv_filepath}")
    # -------------------------

    obs = env.get_observations()
    timestep = 0

    try:
        while simulation_app.is_running():
            start_time = time.time()
            with torch.inference_mode():
                actions = policy(obs)
                
                # --- FOOLPROOF TENSOR EXTRACTION ---
                # Checks if it has dict properties, bypassing strict isinstance checks
                if hasattr(obs, "keys") and "policy" in obs:
                    _obs_tensor = obs["policy"]
                elif isinstance(obs, dict) and "policy" in obs:
                    _obs_tensor = obs["policy"]
                else:
                    _obs_tensor = obs
                
                # Slice out Environment 0
                obs_env0 = _obs_tensor[0] if _obs_tensor.ndim > 1 else _obs_tensor
                act_env0 = actions[0] if actions.ndim > 1 else actions
                
                # Convert explicitly to flat python lists to guarantee normal CSV formatting
                obs_list = obs_env0.detach().cpu().flatten().tolist()
                act_list = act_env0.detach().cpu().flatten().tolist()
                
                # Write to CSV
                csv_writer.writerow(obs_list + act_list)
                # -----------------------------------
                
                # Step environment forward
                obs, _, dones, _ = env.step(actions)

                if "episode" in env.unwrapped.extras:
                    if "Metric/Total_Energy" in env.unwrapped.extras["episode"]:
                        val = env.unwrapped.extras["episode"]["Metric/Total_Energy"]
                        print(f"DEBUG: Energy Value: {val:.2f} Joules")
                        
                actual_dt = time.time() - start_time
                target_dt = env.unwrapped.step_dt
                rt_factor = target_dt / actual_dt

                policy_nn.reset(dones)
                
            if args_cli.video:
                timestep += 1
                if timestep == args_cli.video_length:
                    break

            sleep_time = dt - (time.time() - start_time)
            if args_cli.real_time and sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[INFO] Playback interrupted. Saving log file safely.")
    finally:
        # Guarantee the file writes out properly upon closing or crashing
        csv_file.close()
        env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()