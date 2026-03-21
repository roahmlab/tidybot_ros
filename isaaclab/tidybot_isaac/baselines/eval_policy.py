import argparse
import sys
import os
import torch
import numpy as np
from isaaclab.app import AppLauncher

# 1. Define and parse arguments first
parser = argparse.ArgumentParser(description="Evaluate RL baseline.")
parser.add_argument("--task", type=str, required=True, help="Name of the task.")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to the .pt checkpoint.")
parser.add_argument("--num_envs", type=int, default=2000, help="Number of environments.")
parser.add_argument("--agent", type=str, default="rsl_rl_cfg_entry_point", help="Agent config entry point.")

# Add standard AppLauncher and RSL_RL args
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()

# 2. CLEAR sys.argv for Hydra! 
sys.argv = [sys.argv[0]] + hydra_args

# 3. Launch the simulator
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- Imports that require the simulator to be running ---
import gymnasium as gym
from rsl_rl.runners import OnPolicyRunner
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, RslRlBaseRunnerCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab_tasks.utils.hydra import hydra_task_config
import tidybot_isaac.tasks  # Ensure your tasks are registered

@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg, agent_cfg: RslRlBaseRunnerCfg):
    # Override env count and device
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.sim.device = args_cli.device if args_cli.device else "cuda:0"
    
    # Create Environment
    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    # Load Runner
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(args_cli.checkpoint)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    obs = env.get_observations()

    # --- Dynamic Task Detection ---
    is_door_task = "Door" in args_cli.task
    target_obj_name = "door" if is_door_task else "cabinet"
    
    # Set thresholds and joint names based on the task
    if is_door_task:
        open_threshold = 0.78
        target_joint_name = "HingeJoint"
    else:
        open_threshold = 0.25 
        target_joint_name = "drawer_top_joint"
    
    # Trackers
    finished_envs = torch.zeros(env.num_envs, device=env.device, dtype=torch.bool)
    has_crossed_threshold = torch.zeros(env.num_envs, device=env.device, dtype=torch.bool)
    successes = []
    efforts = []

    # Local effort buffer to accumulate energy per-environment
    local_effort_buf = torch.zeros(env.num_envs, device=env.device)

    # --- NEW: Dynamic Joint Lookup for Effort Calculation ---
    # We define the SceneEntityCfg exactly like in your training config
    effort_asset_cfg = SceneEntityCfg("robot", joint_names=["joint_[1-7]"])
    # Resolve the config against the unwrapped scene to populate the actual joint_ids
    effort_asset_cfg.resolve(env.unwrapped.scene)

    print(f"[INFO] Evaluating {args_cli.num_envs} envs for: {os.path.basename(args_cli.checkpoint)}")
    print(f"[INFO] Tracking effort for joint IDs: {effort_asset_cfg.joint_ids}")

    with torch.inference_mode():
        while not finished_envs.all():
            actions = policy(obs)
            
            # --- 1. CAPTURE METRICS BEFORE THE STEP ---
            target_obj = env.unwrapped.scene[target_obj_name]
            
            # Capture target position and update the persistent threshold tracker
            joint_idx = target_obj.find_joints(target_joint_name)[0][0]
            current_target_pos = target_obj.data.joint_pos[:, joint_idx]
            
            # Keep it True if it was ever True during the episode
            has_crossed_threshold |= (current_target_pos > open_threshold)
            
            # Capture the exact torque buffer your MDP uses
            if hasattr(env.unwrapped, "_episode_effort_buf"):
                pre_step_effort = env.unwrapped._episode_effort_buf.clone()
            else:
                # Fallback using dynamic joint IDs resolved via SceneEntityCfg
                robot = env.unwrapped.scene[effort_asset_cfg.name]
                arm_torques = robot.data.applied_torque[:, effort_asset_cfg.joint_ids]
                
                effort_step = torch.sum(torch.square(arm_torques), dim=-1)
                local_effort_buf += effort_step * env.unwrapped.step_dt
                pre_step_effort = local_effort_buf.clone()

            # --- 2. STEP ENVIRONMENT ---
            obs, _, _, dones = env.step(actions)
            
            # Robust nested dones flattening
            reset_tensor = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
            if isinstance(dones, dict):
                items_to_check = list(dones.values())
                while items_to_check:
                    item = items_to_check.pop(0)
                    if isinstance(item, torch.Tensor):
                        reset_tensor |= item.bool()
                    elif isinstance(item, dict):
                        items_to_check.extend(item.values())
            else:
                reset_tensor = dones.bool()

            # --- 3. RECORD FINISHED EPISODES ---
            if reset_tensor.any():
                reset_ids = reset_tensor.nonzero(as_tuple=True)[0]
                
                for idx in reset_ids:
                    if not finished_envs[idx]:
                        # Record metrics using the persistent tracker
                        successes.append(float(has_crossed_threshold[idx].item()))
                        efforts.append(float(pre_step_effort[idx].item()))
                        
                        finished_envs[idx] = True
                        
                # Reset the threshold tracker ONLY for the environments that just finished
                has_crossed_threshold[reset_ids] = False
                
                # Reset local fallback buffer if needed
                if not hasattr(env.unwrapped, "_episode_effort_buf"):
                    local_effort_buf[reset_ids] = 0.0

            # Heartbeat printout
            finished_count = finished_envs.sum().item()
            if finished_count % 200 == 0 and finished_count > 0:
                print(f"[PROGRESS] Captured {int(finished_count)} / {env.num_envs} episodes...")

    # Aggregate and Print
    avg_success = np.mean(successes) if successes else 0.0
    avg_effort = np.mean(efforts) if efforts else 0.0
    
    print(f"\n>> RESULT | Success: {avg_success:.4f} | Torque: {avg_effort:.4e}\n")
    
    with open("eval_results.tmp", "a") as f:
        f.write(f"{args_cli.task},{args_cli.checkpoint},{avg_success},{avg_effort}\n")

    # Clean up the environment properly to avoid hangs
    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()
    # Force the Python process to exit so the bash script can continue
    sys.exit(0)