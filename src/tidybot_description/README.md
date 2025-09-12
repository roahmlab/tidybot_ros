# tidybot_description

## 📖 Overview

This package contains the complete robot description and simulation setup for the TidyBot++ mobile manipulator. It defines the robot's physical structure, visual appearance, collision properties, and simulation behavior using URDF/XACRO files.

## 🤖 Robot Components

The TidyBot++ description includes:

### **Mobile Base (TidyBot++)**
- Omnidirectional mobile platform
- Holonomic drive system (3-DOF: x, y, θ)
- Custom base geometry and inertial properties

### **Manipulator (Kinova Gen3 7-DOF)**
- 7 degree-of-freedom articulated arm
- Spherical wrist design
- Collision meshes for safe planning

### **End Effector (Robotiq 2F-85)**
- Parallel jaw gripper
- Adaptive finger design
- Customizable finger tips

### **Sensor Suite**
- Integrated camera systems

## 📁 Package Structure

```
tidybot_description/
├── config/
│   ├── tidybot_controllers.yaml      # ROS2 control configuration
│   ├── arm.rviz                      # RViz config for arm-only
│   └── tidybot.rviz                  # RViz config for full robot
├── launch/
│   ├── description.launch.py         # Basic robot state publisher
│   ├── display.launch.py             # RViz visualization
│   ├── sim_arm.launch.py             # Arm-only simulation
│   └── sim_tidybot.launch.py         # Full robot simulation
├── urdf/
│   ├── tidybot.xacro                 # Main robot description
│   ├── bot.xacro                     # Robot assembly macro
│   ├── arm.xacro                     # Arm-only description
│   ├── tidybot.ros2_control.xacro    # Control interface definition
│   ├── tidybot.gazebo.xacro          # Gazebo plugins and properties
│   ├── arms/                         # Arm-specific descriptions
│   │   └── gen3_7dof/
│   ├── grippers/                     # Gripper descriptions
│   │   └── robotiq_2f_85/
│   └── base/                         # Base platform descriptions
│       └── tidybot++/
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
Launch RViz visualization with optional joint state publisher GUI.

```bash
ros2 launch tidybot_description display.launch.py jsp_gui:=true
```

**Parameters:**
- `jsp_gui` (bool): Launch joint state publisher GUI (default: false)
- `rviz_config` (string): Path to RViz configuration file (default: tidybot.rviz)
- `robot_description_content` (string): URDF content (default: tidybot.xacro)
- `use_sim_time` (bool): Use simulation time (default: false)

### `sim_arm.launch.py`
Launch arm-only simulation in Gazebo.

```bash
ros2 launch tidybot_description sim_arm.launch.py
```

**Parameters:**
- `use_rviz` (bool): Launch RViz alongside Gazebo (default: true)
- `rviz_config` (string): RViz configuration file (default: arm.rviz)
- `world` (string): The simulated world (default: empty, choices: office|warehouse|fetch_coke|fetch_cube)

### `sim_tidybot.launch.py`
Launch full robot simulation in Gazebo.

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
Publishes current joint states to `/joint_states` topic.

### **Base Controllers**
- `tidybot_base_pos_controller`: Position control for teleoperation
- `tidybot_base_vel_controller`: Velocity control for joystick input

### **Arm Controller**
- `gen3_7dof_controller`: Joint trajectory controller for planned motions

### **Gripper Controller**
- `robotiq_2f_85_controller`: Position controller for gripper actuation

## 🔧 Customization

### Adding New Components

1. **New Arm Configuration:**
   ```bash
   mkdir urdf/arms/new_arm/
   # Create new_arm_macro.xacro with load_arm macro
   ```

2. **New Gripper:**
   ```bash
   mkdir urdf/grippers/new_gripper/
   # Create new_gripper_macro.xacro with load_gripper macro
   ```

3. **Custom Base:**
   ```bash
   mkdir urdf/base/custom_base/
   # Create base_macro.xacro with load_base macro
   ```

### Modifying Robot Properties

1. **Joint Limits:** Edit controller configurations in `config/tidybot_controllers.yaml`
2. **Visual Properties:** Modify XACRO files in respective component directories
3. **Physics Properties:** Update inertial properties and collision meshes
4. **Sensor Integration:** Add sensor plugins in `tidybot.gazebo.xacro`

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



