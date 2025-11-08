# tidybot_teleop

## 📖 Overview

`tidybot_teleop` delivers the user-facing control stack for TidyBot++. It combines a WebXR phone interface, remote-policy bridge, and joystick teleoperation while coordinating environment reset and episode management services. The package streams user input into the low-level controllers provided by `tidybot_solver` and exposes optional recording hooks through `tidybot_episode`.

## 📦 Package Layout

```
tidybot_teleop/
├── config/
│   ├── index.html                  # WebXR frontend served by phone_teleop_server
│   ├── webxr-button.js             # WebXR helper assets
│   ├── joy_node_config.yaml        # Generic ROS joy driver configuration
│   ├── Xbox_SeriesX_Wire_config.yaml
│   └── Xbox_SeriesX_Wireless_config.yaml
├── launch/                         # Entry points for common control flows
├── tidybot_teleop/
│   ├── phone_teleop_server.py      # Flask + Socket.IO bridge to ROS
│   ├── phone_teleop.py             # WebXR command processor
│   ├── remote_teleop.py            # Policy-server driven controller
│   ├── joystick_teleop.py          # Gamepad interface feeding MoveIt Servo
│   ├── state_controller.py         # Episode lifecycle + reset coordination
│   └── reset_env.py                # Simulation reset helper
├── setup.py / setup.cfg
└── README.md
```

## 🚀 Launch Profiles

### `teleop.launch.py`
Spin up the complete WebXR stack. The launch file starts the Flask WebXR server, the WebXR command translator, the shared `state_controller`, and the MoveIt pose IK bridge (`tidybot_solver/moveit_ee_pose_ik`). When `record:=true`, it also launches `tidybot_episode`'s synchronized recorder and wires the UI buttons into `/start_recording`, `/stop_recording`, and `/finalize_recording`.

```bash
# Simulation (default)
ros2 launch tidybot_teleop teleop.launch.py

# Hardware with recording enabled
ros2 launch tidybot_teleop teleop.launch.py use_sim:=false record:=true
```

Key parameters:
- `use_sim` — toggles ROS time, Gazebo topic remaps, and base command outputs.
- `record` — enables tidybot_episode services and WebUI prompts for save/discard.

WebUI quick facts:
- Opens at `http://<host>:5000` (HTTPS required for full WebXR on iOS/Android).
- Modes: tap on different region of the screen to enable arm/base, swipe upwards/downwards for gripper open/close.
- Streams `TeleopMsg` messages over `/teleop_commands` and `/teleop_state`.

### `remote.launch.py`
Targets remote policy execution while keeping the WebXR interface available for human override. It launches the same Web server, the `remote_teleop` node, `state_controller`, and `tidybot_solver/moveit_ee_pose_ik`.

```bash
ros2 launch tidybot_teleop remote.launch.py use_sim:=true
```

Highlights:
- `remote_teleop` connects to a ZMQ policy server at `tcp://localhost:5555` (tunnel with `ssh -L 5555:localhost:5555 ...` when needed).
- Publishes policy actions to `/tidybot/base/target_pose`, `/tidybot/arm/target_pose`, and `/tidybot/gripper/commands`, mirroring the phone controller interface.
- Exposes `/tidybot/controller/reset` so `state_controller` can synchronize resets.

### `joystick.launch.py`
Couples the ROS `joy` driver, the `joystick_teleop` node, and the velocity IK bridge from `tidybot_solver`.

```bash
# Simulation velocity IK
ros2 launch tidybot_teleop joystick.launch.py controller_type:=Xbox_SeriesX_Wire use_sim:=true

# Hardware velocity IK
ros2 launch tidybot_teleop joystick.launch.py use_sim:=false
```

What it does:
- Loads the selected controller mapping YAML (`controller_type` argument).
- Starts `joystick_teleop` which publishes `/tidybot/arm/target_vel`, `/tidybot/base/target_vel`, and `/tidybot/gripper/commands`.
- Includes `tidybot_solver/launch_moveit_vel_ik.launch.py`, automatically choosing the simulation or hardware servo configuration.

## 🧩 Node Reference

### `phone_teleop_server`
- Flask + Socket.IO app serving the WebXR UI from `config/index.html` on port 5000.
- Bridges incoming JSON payloads into `TeleopMsg` and pushes messages onto `/teleop_state` and `/teleop_commands`.
- Accepts a `record` parameter (mirrors the launch argument) to enable save/discard buttons.

### `phone_teleop`
- Subscribes to `/teleop_commands` and interprets WebXR poses, converting them into `/tidybot/base/target_pose`, `/tidybot/arm/target_pose`, and `/tidybot/gripper/commands`.
- Provides `/tidybot/controller/reset` to clear XR references and send one-shot home commands.

### `remote_teleop`
- Listens to `/teleop_commands` to gate policy execution and ingest enable signals from the phone UI.
- Samples observations (TF, joint states, base & wrist images) and queries a ZMQ policy server at 10 Hz.
- Publishes resulting actions on the same pose/gripper topics used by the phone controller and mirrors commands to the hardware interfaces when available.

### `joystick_teleop`
- Reads `/joy` input, supports configurable axes/buttons, and toggles between base and arm control modes.
- Publishes twist commands to `/tidybot/arm/target_vel`, base velocity arrays to `/tidybot/base/target_vel`, and gripper commands to `/tidybot/gripper/commands`. Simulation-specific outputs are sent to `/tidybot_base_vel_controller/commands`.
- Provides `/tidybot/hardware/{arm,base}/reset` clients and a `/tidybot/controller/reset` trigger for state synchronization.

### `state_controller`
- Consumes `/teleop_state` events from the phone WebUI (start, end, reset, save, discard).
- When `record:=true`, coordinates tidybot_episode services and forwards environment reset requests to `/tidybot_teleop/reset_env` (sim) or the hardware reset services.
- Ensures controller reset requests are propagated to `/tidybot/controller/reset`.

### `reset_env`
- Utility script invoked by `state_controller` in simulation mode.
- Calls Gazebo world reset via `/world/empty/control`, respawns the TidyBot robot, and reloads the base/arm/gripper controllers with appropriate mode (position vs. velocity).

## ⚙️ Configuration Notes

- **Web assets**: customise `config/index.html` and `config/webxr-button.js` for branding or UI tweaks. The files are copied into the shared install directory and served directly by `phone_teleop_server`.
- **Joystick mappings**: edit `config/Xbox_SeriesX_*.yaml` entries to adjust axis/button bindings or scaling. These YAML files are passed directly into `joystick_teleop` as ROS parameters.
- **Joy driver**: `config/joy_node_config.yaml` sets dead zones and auto-repeat parameters for the `joy` package.
- **WebXR security**: browsers enforce HTTPS for sensor access. Terminate TLS in front of port 5000 or reverse-proxy through a secure endpoint when deploying on a network.

## 🧰 Console Scripts

Each script can be run individually (ensure the ROS graph contains the required supporting nodes):
- `ros2 run tidybot_teleop phone_teleop_server`
- `ros2 run tidybot_teleop phone_teleop`
- `ros2 run tidybot_teleop remote_teleop`
- `ros2 run tidybot_teleop joystick_teleop`
- `ros2 run tidybot_teleop state_controller`
- `ros2 run tidybot_teleop reset_env`

## 🐛 Troubleshooting

- **No WebXR sensor access**: ensure the page is served over HTTPS or use Chrome flags on development devices.
- **Joystick commands ignored**: verify the `enable_normal` / `enable_turbo` buttons are pressed; the node will otherwise publish zeroed commands for safety.
- **Policy server timeouts**: confirm `ssh -L 5555:localhost:5555 <gpu-host>` is active and reachable from the teleop machine.
- **Recording services unavailable**: launch with `record:=true` so `tidybot_episode` is included, and ensure the recorder services respond on `/start_recording`, `/stop_recording`, and `/finalize_recording`.
