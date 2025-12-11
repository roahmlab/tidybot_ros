# tidybot_policy
## 📖 Overview
`tidybot_policy` delivers the user-facing control stack for teleoperation and policy deployment. It includes a WebXR phone interface, remote inference bridge, and gamepad teleoperation with environment reset and episode management services. The package streams user input into the low-level controllers provided by `tidybot_solver` and exposes optional recording hooks through `tidybot_episode`.

## 📁 Package Layout

```
tidybot_policy/
├── config/                         # Controller mappings and web assets
│   ├── index.html                  # WebXR frontend served by phone_teleop_server
│   ├── joy_node_config.yaml        # ROS joy driver configuration
│   ├── webxr-button.js             # Frontend logic
│   └── Xbox_SeriesX_*.yaml         # Gamepad mappings
│   └── cyclonedds.xml              # Template for message publishing over network
├── launch/
│   ├── launch_gamepad_policy.launch.py             # Gamepad control launch
│   ├── launch_remote_policy_diffusion.launch.py    # Remote inference bridge launch
│   └── launch_phone_policy.launch.py               # WebXR phone teleop launch
├── tidybot_policy/
│   ├── phone_teleop_server.py      # Flask + Socket.IO bridge to ROS
│   ├── phone_policy.py             # WebXR command processor
│   ├── remote_policy_diffusion.py  # Policy-server driven controller
│   ├── tidybot_openvla.py          # Policy-server driven controller
│   ├── gamepad_policy.py           # Gamepad interface feeding MoveIt Servo
│   ├── reset_env.py                # Environment reset service client
│   └── state_controller.py         # Episode lifecycle manager
└── setup.py / package.xml
```

## 🚀 Launch

### `launch_phone_policy.launch.py`
Starts the WebXR teleoperation stack. This is the primary entry point for data collection.
```bash
# Standard launch (simulation)
ros2 launch tidybot_policy launch_phone_policy.launch.py

# Hardware mode with recording enabled
ros2 launch tidybot_policy launch_phone_policy.launch.py use_sim:=false record:=true
```

### `launch_remote_policy_diffusion.launch.py`
Allows executing remote inference while keeping the WebXR interface available for human override. It launches the same Web server, the `remote_policy_diffusion` node, `state_controller`, and `tidybot_solver/moveit_ee_pose_ik`.
```bash
# Launch remote inference bridge
ros2 launch tidybot_policy launch_remote_policy_diffusion.launch.py use_sim:=true
```
- `remote_policy_diffusion` connects to a ZMQ policy server at `tcp://localhost:5555` (tunnel with `ssh -L 5555:localhost:5555 ...` when needed).

### `launch_remote_policy_vla.launch.py`
Loads finetuned `peft` model and initiates policy inference through published ROS messages. First ensure that the remote server and client can communicate via cycloneDDS (try `ros2 run demo_nodes_cpp talker` and `listener`). See the configs folder and external/openvla for setup instructions.
```bash
# Launch remote inference bridge
ros2 launch tidybot_policy launch_remote_policy_vla.launch.py
```
- `openvla_node` loads the base openvla model, applies the finetuned peft model, listens `camera_ext` or `camera_wrist` messages and publishes inferred `/tidybot/arm/delta_commands`.

### `launch_gamepad_policy.launch.py`
Couples the ROS `joy` driver, the `gamepad_policy` node, and the velocity IK bridge from `tidybot_solver`.
