# tidybot_policy

## 📖 Overview

`tidybot_policy` is the central control package for the TidyBot++ platform. It handles user inputs (from phone or gamepad) and policy commands (local or remote), manages the episode lifecycle, and interfaces with the robot's low-level controllers.

Key features:
- **Phone Teleoperation**: WebXR-based interface for extensive data collection using an iPhone/iPad.
- **Gamepad Teleoperation**: Joystick checking for rapid testing and demonstrations.
- **Remote Policy Inference**: Bridge to run Python-based policies (Diffusion, VLA) on external GPU servers.
- **Episode Management**: Automated recording hooks and environment reset services.

## 🚀 Launch Files

### 1. Phone Teleoperation (WebXR)
Starts the WebXR server and teleoperation node. This is the primary mode for collecting high-quality demonstrations.

```bash
ros2 launch tidybot_policy launch_phone_policy.launch.py [args]
```

| Argument | Default | Description |
|----------|---------|-------------|
| `sim_mode` | `hardware` | `hardware` (real robot), `gazebo`, or `isaac` |
| `record` | `false` | Enable rosbag recording of episodes |
| `cameras` | `["base","arm"]` | List of cameras to record |

### 2. Gamepad Teleoperation
Controls the robot using an Xbox controller. Supports base and arm velocity control.

```bash
ros2 launch tidybot_policy launch_gamepad_policy.launch.py [args]
```

| Argument | Default | Description |
|----------|---------|-------------|
| `sim_mode` | `hardware` | `hardware`, `gazebo`, or `isaac` |
| `controller_type` | `Xbox_SeriesX_Wire` | `Xbox_SeriesX_Wire` or `Xbox_SeriesX_Wireless` |
| `record` | `false` | Enable recording |

### 3. Remote Policy Inference
Connects to a remote policy server (e.g., running a Diffusion Policy or OpenVLA) via ZeroMQ.

```bash
ros2 launch tidybot_policy launch_remote_policy_diffusion.launch.py 
```
| Argument | Default | Description |
|----------|---------|-------------|
| `sim_mode` | `hardware` | `hardware`, `gazebo`, or `isaac` |

**Setup:**
1.  Start the policy server on your GPU machine.
2.  Tunnel the port to local: `ssh -L 5555:localhost:5555 <gpu-server>`
3.  Launch this file to bridge observations and actions.

## 🛠️ Configuration
*   **`config/*.yaml`**: Gamepad mappings.
*   **`config/index.html`**: WebXR client frontend.
