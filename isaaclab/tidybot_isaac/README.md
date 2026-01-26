# TidyBot Isaac Lab Extension

This extension provides reinforcement learning environments and training pipelines for the TidyBot mobile manipulator in Isaac Lab.

## Overview

The TidyBot Isaac Lab extension enables training RL policies for a mobile manipulator to perform manipulation tasks like drawer opening. It includes:

- **Custom robot assets** with configurable actuators for base, arm, and gripper
- **Manager-based RL environment** with configurable observations, actions, rewards, and terminations
- **Custom Mimic Gripper Action** for controlling multi-joint grippers with a single command
- **Training scripts** for multiple RL libraries (RSL-RL, rl_games, Stable-Baselines3, SKRL)

---

## Quick Start

### Running with Docker (Recommended)

From the main `tidybot_platform` directory:

```bash
# Start container and run random agent
./docker/isaac-lab/run.sh restart python.sh scripts/random_agent.py --task Isaac-TidyBot-Drawer-v0 --num_envs 1

# Run training with RSL-RL
./docker/isaac-lab/run.sh python.sh scripts/rsl_rl/train.py --task Isaac-TidyBot-Drawer-v0 --num_envs 4096

# Run training with headless mode
./docker/isaac-lab/run.sh python.sh scripts/rsl_rl/train.py --task Isaac-TidyBot-Drawer-v0 --num_envs 4096 --headless
```

---

## Directory Structure

```
tidybot_isaac/
├── scripts/                          # Executable scripts
│   ├── random_agent.py               # Random action agent for testing
│   ├── zero_agent.py                 # Zero action agent for testing
│   ├── list_envs.py                  # List available environments
│   ├── rsl_rl/                       # RSL-RL training scripts
│   │   ├── train.py                  # Training script
│   │   └── play.py                   # Policy playback script
│   ├── rl_games/                     # rl_games training scripts
│   ├── sb3/                          # Stable-Baselines3 scripts
│   └── skrl/                         # SKRL training scripts
├── source/tidybot_isaac/             # Python package
│   └── tidybot_isaac/
│       ├── assets.py                 # Robot and object asset configurations
│       └── tasks/manager_based/
│           └── tidybot_isaac/
│               ├── tidybot_isaac_env_cfg.py  # Environment configuration
│               └── mdp/
│                   ├── actions.py    # Custom action terms (MimicGripperAction)
│                   ├── rewards.py    # Custom reward functions
│                   └── observations.py
└── logs/                             # Training logs and checkpoints
```

---

## Scripts

### `random_agent.py`
Runs the environment with random actions. Useful for testing environment setup and gripper behavior.

```bash
python.sh scripts/random_agent.py --task Isaac-TidyBot-Drawer-v0 --num_envs 1
```

**Key Features:**
- Gripper controlled with sine wave for testing full range of motion
- Logs `finger_joint` (Left) and `right_outer_knuckle_joint` (Right) positions

### `zero_agent.py`
Runs the environment with zero actions. Useful for observing initial state and physics behavior.

```bash
python.sh scripts/zero_agent.py --task Isaac-TidyBot-Drawer-v0 --num_envs 1
```

### `list_envs.py`
Lists all registered TidyBot environments.

```bash
python.sh scripts/list_envs.py
```

### Training Scripts (`scripts/rsl_rl/train.py`)
Train RL policies using RSL-RL library.

```bash
python.sh scripts/rsl_rl/train.py --task Isaac-TidyBot-Drawer-v0 --num_envs 4096 --headless
```

---

## Configuration Files

### `assets.py` - Robot and Object Configurations

Defines the TidyBot robot and cabinet assets with:

| Asset | Description |
|-------|-------------|
| `TIDYBOT_CFG` | TidyBot mobile manipulator configuration |
| `CABINET_CFG` | Sektion cabinet with drawer for manipulation task |

**TidyBot Actuator Groups:**

| Group | Joints | Stiffness | Damping | Description |
|-------|--------|-----------|---------|-------------|
| `base` | `joint_x`, `joint_y`, `joint_th` | 1e7 | 1e4 | Mobile base (prismatic x/y, revolute yaw) |
| `arm` | `joint_[1-7]` | 1e7 | 1e4 | 7-DOF Kinova arm |
| `gripper` | `finger_joint`, `right_outer_knuckle_joint` | 1e5 | 1.0 | Active gripper drivers (Hybrid Control) |
| `gripper_passive` | `*_inner_finger_joint`, `*_inner_finger_knuckle_joint` | 0.0 | 0.0 | Passive compliant linkages |

### `tidybot_isaac_env_cfg.py` - Environment Configuration

Defines the RL environment MDP:

**Action Space (11 dimensions):**
| Index | Name | Dimension | Description |
|-------|------|-----------|-------------|
| 0-2 | `base_pos` | 3 | Base position control (x, y, yaw) |
| 3-9 | `arm_pos` | 7 | Arm joint position control |
| 10 | `gripper` | 1 | Gripper command [0=open, 1=closed] |

**Observation Space (37 dimensions):**
| Name | Shape | Description |
|------|-------|-------------|
| `joint_pos` | (18,) | All joint positions |
| `joint_vel` | (18,) | All joint velocities |
| `drawer_pos` | (1,) | Drawer extension position |

**Rewards:**
| Name | Weight | Description |
|------|--------|-------------|
| `alive` | 1.0 | Survival reward |
| `drawer_opened` | 1.0 | Drawer opening progress |

### `mdp/actions.py` - Custom Action Terms

**`MimicGripperAction`**: Maps a single scalar command `[0, 1]` to multiple gripper joints.

- **Input**: Single value (0 = open, 1 = closed)
- **Output**: Joint position targets for leader + mimic joints
- **Leader Joint**: `finger_joint` (range: 0.0 to 0.82)
- **Mimic Joint**: `right_outer_knuckle_joint` (multiplier: 1.0)

This implements **Hybrid Passive Control** where:
1. **Active drivers** (`finger_joint`, `right_outer_knuckle_joint`) are position-controlled
2. **Passive linkages** (inner fingers/knuckles) are compliant (0 stiffness) and follow via physics

---

## Registered Environments

| Task Name | Description |
|-----------|-------------|
| `Isaac-TidyBot-Drawer-v0` | Drawer opening task with TidyBot |

---

## Training

### Local Training (with Display)

```bash
./docker/isaac-lab/run.sh restart python.sh scripts/rsl_rl/train.py \
    --task Isaac-TidyBot-Drawer-v0 \
    --num_envs 4096
```

### SSH/Remote Training (Headless)

```bash
# Use the ssh mode for headless training
./docker/isaac-lab/run.sh ssh python.sh scripts/rsl_rl/train.py \
    --task Isaac-TidyBot-Drawer-v0 \
    --num_envs 4096 \
    --headless

# Select specific GPU (if multiple GPUs available)
CUDA_VISIBLE_DEVICES=1 ./docker/isaac-lab/run.sh ssh python.sh scripts/rsl_rl/train.py \
    --task Isaac-TidyBot-Drawer-v0 \
    --num_envs 4096 \
    --headless
```

### Training Options

| Option | Description |
|--------|-------------|
| `--num_envs N` | Number of parallel environments (default: 4096) |
| `--headless` | Disable rendering (faster, required for SSH) |
| `--video` | Record training videos |
| `--seed N` | Random seed for reproducibility |

### Checkpoints

Checkpoints are saved to `logs/rsl_rl/tidybot_drawer/<timestamp>/`:
- `model_<iter>.pt` - Policy checkpoints (every 50 iterations)
- `events.out.tfevents.*` - TensorBoard logs

---

## Policy Evaluation

### Run Latest Checkpoint (Local with Display)

```bash
./docker/isaac-lab/run.sh restart python.sh scripts/rsl_rl/play.py \
    --task Isaac-TidyBot-Drawer-v0 \
    --num_envs 1 \
    --checkpoint logs/rsl_rl/tidybot_drawer/<timestamp>/model_400.pt
```

### Evaluate While Training (Separate Terminal)

Use `docker exec` to run play script without stopping training:

```bash
# In a new terminal
docker exec -it isaac-lab-tidybot python.sh scripts/rsl_rl/play.py \
    --task Isaac-TidyBot-Drawer-v0 \
    --num_envs 1 \
    --checkpoint /workspace/tidybot_isaac/logs/rsl_rl/tidybot_drawer/<timestamp>/model_400.pt
```

### Record Video (Headless)

```bash
CUDA_VISIBLE_DEVICES=0 ./docker/isaac-lab/run.sh ssh python.sh scripts/rsl_rl/play.py \
    --task Isaac-TidyBot-Drawer-v0 \
    --num_envs 1 \
    --headless \
    --video \
    --checkpoint logs/rsl_rl/tidybot_drawer/<timestamp>/model_400.pt
```

Videos are saved to `videos/` folder.

---

## Monitoring Training

### TensorBoard

```bash
# Inside container
tensorboard --logdir=/workspace/tidybot_isaac/logs/rsl_rl/ --port=6006 --bind_all

# SSH tunnel from local machine
ssh -L 6006:localhost:6006 user@server
# Then open http://localhost:6006 in browser
```

### Key Metrics

| Metric | What to Watch |
|--------|---------------|
| `Train/mean_reward` | Should increase over time |
| `Loss/value_function` | Critic loss (should decrease) |
| `Loss/surrogate` | Policy loss |
| `Policy/mean_noise_std` | Exploration noise (should decrease) |

---

## Troubleshooting

### Gripper Not Moving
- Check actuator configuration in `assets.py` - ensure joints are in an actuator group
- Verify multiplier signs in `MimicGripperActionCfg` - both left and right knuckles move in same direction (multiplier should be 1.0, not -1.0)

### "Not all actuators configured" Warning
This warning is expected when using Hybrid Passive Control. The passive gripper joints intentionally have 0 stiffness.

### Slow Training Progress
- Increase `num_envs` to 8192+ if GPU memory allows
- Check if rewards are properly scaled (use TensorBoard to monitor individual reward components)
- Consider adjusting `entropy_coef` in agent config (default 0.01, try 0.02-0.05 for more exploration)

### Container Issues
```bash
# Force remove and restart container
docker rm -f isaac-lab-tidybot
./docker/isaac-lab/run.sh restart python.sh scripts/random_agent.py --task Isaac-TidyBot-Drawer-v0
```

### USD File Not Found (Server)
Ensure the collected USD assets are committed:
```bash
git add src/tidybot_description/usd/Collected_tidybot/
git commit -m "Add collected USD assets"
git push
```