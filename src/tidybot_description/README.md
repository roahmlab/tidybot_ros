# tidybot_description

## 📖 Overview

`tidybot_description` includes the URDF/XACRO source for the TidyBot++ platform, including meshes, controller interfaces, and Gazebo plugins. The package feeds robot_state_publisher on hardware (via `tidybot_driver`) and acts as the foundation for simulation launch files consumed by `tidybot_policy`, `tidybot_solver`, and Gazebo.

## 🤖 Robot Components

### **Base Platform**
- Powered-caster holonomic base with omnidirectional drive (x, y, yaw)
- Configurable in position or velocity control modes (`tidybot_controllers.yaml`)

### **Manipulator Stack**
- Kinova Gen3 7-DOF arm with compliant joint controller support
- Transmission macros and ROS2 control interfaces for joint trajectory execution

### **End Effectors**
- Robotiq 2F-85 grasping configuration with visual/collision meshes
- Placeholder macros for Kinova Lite gripper variants

### **Vision & Sensors**
- Wrist camera frame hierarchy and extrinsics
- External camera links for Orbbec base camera and auxiliary sensors

## 📁 Package Structure

```
tidybot_description/
├── config/
│   ├── tidybot_controllers.yaml      # ROS2 control configuration
│   ├── arm.rviz                      # RViz config for arm-only test
│   └── tidybot.rviz                  # RViz config for full robot
├── launch/
│   ├── description.launch.py         # Basic robot state publisher
│   ├── display.launch.py             # RViz visualization
│   ├── launch_sim_arm.launch.py      # Arm-only simulation for manipulation test
│   └── launch_sim_robot.launch.py    # Full robot simulation
├── urdf/
│   ├── tidybot.xacro                 # Main robot description
│   ├── bot.xacro                     # Robot assembly macro
│   ├── arm.xacro                     # Arm-only description
│   ├── tidybot.ros2_control.xacro    # Control interface definition
│   ├── tidybot.gazebo.xacro          # Gazebo plugins and properties
│   ├── arms/                         # Arm-specific descriptions
│   ├── grippers/                     # Gripper descriptions
│   ├── base/                         # Base platform descriptions       
│   └── world/                        # Demo world
├── meshes/                           # 3D model files (STL, DAE) for the robot
└── models/                           # 3D models obtained from openrobotics
```

## 🚀 Launch Files

### `description.launch.py`
Minimal launch file for robot state publisher.

```bash
ros2 launch tidybot_description description.launch.py
```

**Parameters:**
- `robot_description` (string): Path to URDF model (default: tidybot.xacro)
- `use_sim_time` (bool): Use simulation time (default: false)
- `ignore_timestamp` (bool): Ignore timestamps in robot_state_publisher (default: false)

### `display.launch.py`
Launch RViz visualization with optional joint state publisher GUI to visualize the robot and test joint configurations.

```bash
ros2 launch tidybot_description display.launch.py jsp_gui:=true
```

**Parameters:**
- `jsp_gui` (bool): Launch joint state publisher GUI (default: false)
- `rviz_config` (string): Path to RViz configuration file (default: tidybot.rviz)
- `robot_description_content` (string): URDF content (default: tidybot.xacro)
- `use_sim_time` (bool): Use simulation time (default: false)

### `launch_sim_arm.launch.py`
Launch arm-only simulation in Gazebo.

```bash
ros2 launch tidybot_description sim_arm.launch.py
```

**Parameters:**
- `use_rviz` (bool): Launch RViz alongside Gazebo (default: true)
- `rviz_config` (string): RViz configuration file (default: arm.rviz)
- `world` (string): The simulated world (default: empty, choices: office|warehouse|fetch_coke|fetch_cube)

### `launch_sim_robot.launch.py`
Launch full robot simulation in Gazebo for manipulation test.

```bash
ros2 launch tidybot_description sim_tidybot.launch.py base_mode:=velocity
```

**Parameters:**
- `use_rviz` (bool): Launch RViz visualization (default: true)
- `rviz_config` (string): RViz configuration file (default: tidybot.rviz)
- `base_mode` (string): Base control mode - `position` or `velocity` (default: position)
- `world` (string): Gazebo world file (default: empty, choices: office|warehouse|fetch_coke|fetch_cube )

## 🎛️ Controller Configuration

The package defines multiple controller configurations in `config/tidybot_controllers.yaml`:

### **Joint State Broadcaster**
- Publishes `/joint_states` from simulated sensors

### **Base Controllers**
- `tidybot_base_pos_controller` — pose-based control (teleop/simulation)
- `tidybot_base_vel_controller` — velocity inputs (joystick servo pipeline)

### **Arm Controller**
- `gen3_7dof_controller` — joint trajectory interface for MoveIt

### **Gripper Controller**
- `robotiq_2f_85_controller` — parallel gripper position controller

## 🔧 Customization & Tips

- **Swap components**: copy an existing macro (e.g., `gen3_7dof_macro.xacro`) as a template for new arms/grippers/bases and update the inclusion in `bot.xacro`.
- **Adjust joint limits / PID gains**: edit `config/tidybot_controllers.yaml` and regenerate controllers.
- **Update visuals**: edit the corresponding XACRO/mesh files under `meshes/` and `urdf/`.
- **Add sensors**: extend `tidybot.gazebo.xacro` with new Gazebo plugins and link attachments.

## 🐛 Troubleshooting

### Common Issues

1. **URDF Parsing Errors:**
   ```bash
   # Check URDF validity
   xacro tidybot.xacro > tidybot.urdf
   check_urdf tidybot.urdf
   
   # View URDF structure
   urdf_to_graphiz tidybot.urdf
   ```

2. **Missing Meshes:**
   - Ensure all STL/DAE files are in the `meshes/` directory
   - Check file paths in XACRO files are correct
   - Verify mesh file permissions

3. **Controller Failures:**
   ```bash
   # List available controllers
   ros2 control list_controllers
   
   # Check controller status
   ros2 control list_hardware_components
   ```

4. **Transform Issues:**
   ```bash
   # View transform tree
   ros2 run tf2_tools view_frames.py
   
   # Check specific transforms
   ros2 run tf2_ros tf2_echo base_link end_effector_link
   ```

## 🔗 Dependencies

### ROS 2 Packages
- `robot_state_publisher`
- `joint_state_publisher`
- `xacro`
- `ros_gz_sim`
- `controller_manager`
- `moveit_configs_utils`

### External Dependencies
- Gazebo Harmonic
- RViz2

## 📚 Additional Resources

- [URDF Tutorials](https://wiki.ros.org/urdf)
- [XACRO Documentation](https://wiki.ros.org/xacro)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)
- [Gazebo ROS2 Integration](https://docs.ros.org/en/jazzy/p/ros_gz_sim)
- [ROS2 Control Framework](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)



