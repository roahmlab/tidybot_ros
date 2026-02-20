# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to an environment with random action agent."""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Random agent for Isaac Lab environments.")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import torch

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg

import tidybot_isaac.tasks  # noqa: F401


def main():
    """Random actions agent with Isaac Lab environment."""
    # create environment configuration
    env_cfg = parse_env_cfg(
        args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )
    # create environment
    env = gym.make(args_cli.task, cfg=env_cfg)

    # print info (this is vectorized environment)
    print(f"[INFO]: Gym observation space: {env.observation_space}")
    print(f"[INFO]: Gym action space: {env.action_space}")
    # reset environment
    env.reset()
    step_count = 0
    # simulate environment
    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            step_count += 1
            # initialize random actions (base and arm stay at home)
            actions = torch.rand(env.action_space.shape, device=env.unwrapped.device)
            
            # Explicitly control gripper (last dimension) with a sine wave
            # Frequency 0.05, range [0.0, 0.82] mapped to full range of joint
            gripper_cmd = 0.41 + 0.41 * torch.sin(torch.tensor(step_count * 0.05, device=env.unwrapped.device))
            actions[:, -1] = gripper_cmd
            
            # Reset base velocity
            actions[:, 0] = 0.0

            if step_count % 20 == 0:
                # Debug print
                # Access robot from scene
                robot = env.unwrapped.scene["robot"]
                
                if step_count == 20:
                    print(f"[INFO] Robot Joint Names: {robot.data.joint_names}")

                # Try to print gripper info
                try:
                    leader_idx = robot.data.joint_names.index("finger_joint")
                    follower_idx = robot.data.joint_names.index("right_outer_knuckle_joint")
                    
                    pos_L = robot.data.joint_pos[0, leader_idx]
                    pos_F = robot.data.joint_pos[0, follower_idx]
                    
                    print(f"Step {step_count}: Cmd={gripper_cmd.item():.4f}, PosL={pos_L:.4f}, PosF={pos_F:.4f}")
                except ValueError:
                    pass

            # apply actions
            env.step(actions)

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
