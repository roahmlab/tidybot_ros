# tidybot_driver
## 📖 Overview
# tidybot_driver

## 📖 Overview
`tidybot_driver` provides the ROS 2 hardware bridge for the TidyBot++ platform. It wraps the Kinova Gen3 arm, Robotiq 2F-85 gripper, Powered-caster holonomic mobile base, and onboard vision devices behind ROS topics, services, and launch files. The package is designed to pair with `tidybot_teleop` (for command generation) and `tidybot_solver` (for IK and servo pipelines).

## 🎯 Highlights
- Unified launch (`launch_hardware_robot.launch.py`) to bring up robot description, TF relay, drivers, and cameras with selectable modes (`full`, `arm_only`, `base_only`).
- Arm server with compliant joint controller, hardware reset service, and real-time joint-state publishing.
- Base server supporting position or velocity control modes via CANivore USB-C bridge.
- Wrist RTSP and external USB camera publishers with configurable frame rate/resolution.
- Joint state publisher to combine base and arm states for downstream consumers.
- Optional RViz configuration (`config/hardware.rviz`) for quick visual diagnostics.

## 📁 Package Layout

```
tidybot_driver/
├── launch/
│   └── launch_hardware_robot.launch.py   # Main entry point for hardware bring-up
├── config/
│   └── hardware.rviz                     # RViz layout used by the launch file
├── tidybot_driver/
│   ├── arm_controller.py                 # Low-level joint control utilities
│   ├── arm_server.py                     # ROS node wrapping Kinova Kortex API
│   ├── base_server.py                    # ROS node interfacing with Kraken X60 motors
│   ├── base.py                           # Helper functions for mecanum base control
│   ├── camera_wrist.py                   # RTSP wrist camera streamer
│   ├── camera_external.py                # USB camera streamer
│   ├── joint_state_publisher.py          # Aggregates base/arm joint states
│   ├── kinova.py / ik_solver.py          # Kinova helpers and compliant IK routines
│   ├── play_csv_pose.py / play_csv_delta.py # Utility nodes for replaying CSV motions
│   └── utils.py / constants.py           # Shared constants and helpers
├── models/                               # Reference models and URDFs
├── resource/tidybot_driver               # Required by ament index
└── setup.py / package.xml / README.md
```

## 🚀 Launch

### `launch_hardware_robot.launch.py`
```bash
# Full hardware bring-up (default)
ros2 launch tidybot_driver launch_hardware_robot.launch.py

# Arm only, position base disabled
ros2 launch tidybot_driver launch_hardware_robot.launch.py mode:=arm_only

# Base only with velocity mode for joystick control
ros2 launch tidybot_driver launch_hardware_robot.launch.py mode:=base_only base_mode:=velocity
```

Launch actions:
- Loads robot description (`tidybot_description`) and TF relay.
- Starts RViz with the hardware configuration.
- Boots the arm server, wrist camera, external camera, and joint-state publisher when arm is enabled.
- Boots the base server and Orbbec Femto Bolt camera pipeline when base is enabled.

## 🎛️ Nodes & Interfaces

### `arm_server`
- Subscribed topics:
  - `/tidybot/hardware/arm/commands` (`sensor_msgs/JointState`)
  - `/tidybot/hardware/gripper/commands` (`std_msgs/Float64`)
- Published topics:
  - `/tidybot/arm/joint_states` (`sensor_msgs/JointState`)
  - `/tidybot/gripper/state` (`std_msgs/Float64`, optional depending on config)
- Services:
  - `/tidybot/arm/reset` (`std_srvs/Empty`)
- Notes: Uses Kinova Kortex API with compliant controller and built-in fault handling.

### `base_server`
- Parameters:
  - `mode` = `position` (default) or `velocity`
- Subscribed topics:
  - `/tidybot/hardware/base/target_pos` (`std_msgs/Float64MultiArray`)
  - `/tidybot/hardware/base/target_vel` (`std_msgs/Float64MultiArray`)
- Published topics:
  - `/tidybot/base/joint_states` (`sensor_msgs/JointState`) for odometry
  - `/tf` transform updates via `tidybot_description/tf_relay`
- Services:
  - `/tidybot/base/reset` (`std_srvs/Empty`)
- Notes: Communicates with Falcons via CANivore USB-C adapter.

### `camera_wrist`
- Streams RTSP from the Kinova wrist (default `rtsp://192.168.1.10/color`).
- Published topics: 
  - `/tidybot/camera_wrist/color/raw` (`sensor_msgs/Image`)
  - `/tidybot/camera_wrist/color/compressed` (`sensor_msgs/CompressedImage`)
- Parameters: `fps`, `width`, `height`, `crop_size`.

### `camera_ext`
- Streams from `/dev/video*` USB camera.
- Published topics mirror wrist camera.
- Supports parameter overrides for resolution and crop.

### `joint_state_publisher`
- Reads hardware feedback and republishes consolidated joint states for MoveIt and logging.
- Handles the selected `mode` to include only active subsystems.

### Utility scripts
- `play_csv_pose.py` / `play_csv_delta.py`: replay motions recorded offline (expects CSVs with teleop pose/delta columns).
- `arm_controller.py`, `ik_solver.py`: support compliant control loops with collision detection and joint limit enforcement.

## ⚙️ Configuration
- Network:
  - Ensure the Ethernet connection is setup between the host machine and the arm.
- Environment:
  - Ensure the Canivore-USB kernel modules are installed and the CAN tunnel between the host machine and docker container is setup. 
  - Ensure the Kortex API is installed and added to the PYTHONPATH.
  - Ensure udev rules for Orbbec cameras are installed (`orbbec_camera` package helper).
  - Ensure the pre-built OpenCV wheel with GStreamer support is installed.
- Parameters:
  - `base_mode`: switch between position or velocity control.
  - Camera nodes accept FPS, crop, resolution overrides via ROS parameters.

## 🐛 Troubleshooting
- **Arm server reports connection errors**: verify Ethernet link and Kinova credentials; reboot the arm if faults persist (`ros2 service call /tidybot/arm/reset std_srvs/Empty {}`).
- **Base server timeout**: ensure CANivore is connected and recognized (`lsusb`, `dmesg`); run CTR driver install inside container.
- **No camera stream**: check RTSP credentials/IP for wrist camera; for USB camera ensure `/dev/video*` permissions (`sudo usermod -a -G video $USER`).
- **Joint states missing**: confirm `tidybot_joint_state_publisher` is running and `mode` matches the active subsystems.

## 🔗 Dependencies
- Kinova Kortex API (`kortex_api`)
- CTR Electronics Phoenix 6 drivers (via CTR apt repo)
- Orbbec ROS2 camera stack for base camera (`orbbec_camera`)
- `image_transport`, `cv_bridge`, `tf2_ros`, `sensor_msgs`, `geometry_msgs`, `std_msgs`

## 📚 References
- Kinova Gen3 User Guide & Kortex SDK docs
- CTR Electronics Phoenix 6 documentation
- Orbbec Femto Bolt ROS2 integration guide
