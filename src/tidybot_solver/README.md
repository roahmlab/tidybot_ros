# tidybot_solver

## 📖 Overview

This package provides motion planning, trajectory optimization, and real-time control capabilities for the TidyBot++ mobile manipulator. It integrates with MoveIt2 for advanced motion planning and provides servo control for real-time joystick control and reactive behaviors.

## 🎯 Key Features

### **Motion Planning Integration**
- **MoveIt2 Integration**: Full integration with MoveIt2 motion planning framework
- **Collision Avoidance**: Real-time collision detection and avoidance
- **Path Optimization**: Trajectory smoothing and optimization
- **Multi-DOF Planning**: Coordinated planning for arm and base

### **Real-Time Control**
- **Cartesian Control**: End-effector pose control in Cartesian space
- **Joint Space Control**: Direct joint angle control
- **Velocity Control**: Real-time velocity-based control in end-effector frame or base frame

### **Input Processing**
- **Teleoperation Bridge**: Converts WebXR commands to motion plans
- **Joystick Integration**: Gamepad control with MoveIt2 Servo rea
- **Keyboard Control**: Development and testing interface
- **Policy Interface**: AI/ML policy integration

## 📁 Package Structure

```
tidybot_solver/
├── config/
│   ├── demo_servo_config.yaml      # Servo control parameters for keyboard input demo
│   ├── servo_parameters.yaml       # General servo settings
│   └── tidybot_servo_config.yaml   # TidyBot-specific servo config
├── launch/
│   ├── demo_twist.launch.py        # Demo servo control for arm
│   └── joystick_twist.launch.py    # Joystick servo control
├── src/
│   ├── demo_plan.cpp               # Motion planning with Moveit2 API demonstration
│   ├── keyboard_input.cpp          # Keyboard control interface
│   └── teleop_to_moveit.cpp        # Teleoperation to MoveIt bridge
├── include/
│   └── tidybot_solver/             # C++ header files
└── CMakeLists.txt
```

## 🚀 Launch Files

### `demo_twist.launch.py`
Launch demonstration of motion planning and control capabilities.

```bash
# Launch motion planning demo
ros2 launch tidybot_solver demo_twist.launch.py

# Run keyboard input node
ros2 run tidybot_solver keyboard_input
```

**Features:**
- Real-time servo motion planning demonstration
- Keyboard control interface for testing
- Collision avoidance visualization
- Trajectory execution monitoring

### `joystick_twist.launch.py`
Launch joystick-controlled servo system with MoveIt2 integration. Included in tidybot_control package

**Features:**
- Real-time servo control via joystick
- Collision-aware motion
- Emergency stop integration
- Smooth trajectory generation

## 🎛️ Control Nodes

### **Teleop to MoveIt Bridge (`teleop_to_moveit`)**
Converts teleoperation commands to MoveIt2 motion plans and servo commands.

```bash
ros2 run tidybot_solver teleop_to_moveit
```

**Subscribed Topics:**
- `/tidybot/arm/target_pose` (geometry_msgs/Pose): Desired pose for the end-effector read from the WebXR app
- `/tidybot/arm/delta_commands` (std_msgs/Float64MultiArray): Commanded end effector deltas, formatted as [delta_pos, delta_rot, gripper]
- `/tidybot/gripper/commands` (std_msgs/Float64): Gripper state command read from the WebXR

**Published Topics:**
- `/gen3_7dof_controller/joint_trajectory` (trajectory_msgs/JointTrajectory): Publish the computed joint trajectory for the arm to the arm ros2 controller
- `/robotiq_2f_85_controller/commands` (std_msgs/Float64MultiArray): Publish the gripper state to the gripper ros2 controller
- `/tidybot/hardware/arm/commands` (sensor_msgs/JointState): Solved joint angles to publish to hardware

**Features:**
- **Real-time Planning**: Continuous trajectory updates
- **Collision Avoidance**: Automatic collision detection and avoidance
- **Smooth Motion**: Velocity-based servo control for natural movement
- **Safety Monitoring**: Joint limits and workspace constraints

### **Joystick to MoveIt Bridge (`joystick_to_moveit`)**
TODO

### **Motion Planning Demo (`demo_plan`)**
Demonstrates using Moveit2 C++ API to do motion planning for the arm

```bash
ros2 run tidybot_solver demo_plan
```

### **Keyboard Control (`keyboard_input`)**
Simple keyboard interface for development and testing.

```bash
ros2 run tidybot_solver keyboard_input
```

**Key Bindings:**
- **Arrow keys and '.' ';'**: Cartesian Jog
- **1|2|3|4|5|6|7**: Joint Jog
- **r**: Reverse the direction of jogging
- **j**: Select joint jog mode
- **t**: Select twist mode
- **w|e**: Switch between sending command in base frame or end effector fram
- **p**: Pause
- **q**: Quit

## ⚙️ Configuration

### **Servo Control Parameters**
Refer to `config/servo_paramters.yaml` for a complete parameter list

## 🔧 Integration with MoveIt2

### **Planning Groups**
The package works with the following MoveIt2 planning groups:
- **gen3_7dof**: 7-DOF Kinova arm

### **Motion Planning Pipeline**
1. **Goal Specification**: Define target poses or joint configurations
2. **Collision Scene**: Update planning scene with obstacles
3. **Path Planning**: Generate collision-free trajectories
4. **Trajectory Optimization**: Smooth and optimize paths
5. **Execution**: Execute trajectories on real or simulated robot

## 🐛 Troubleshooting

### **Common Issues**

1. **Motion Planning Failures**
   ```bash
   # Check planning scene
   ros2 topic echo /move_group/monitored_planning_scene
   
   # Verify robot model
   ros2 param get /move_group robot_description
   
   # Check joint states
   ros2 topic echo /joint_states
   ```

2. **Servo Control Issues**
   ```bash
   # Check servo status
   ros2 topic echo /servo_node/status
   
   # Monitor command input
   ros2 topic echo /<your_incoming_command_topic_defined_in_the servo_config>
   
   # Verify servo parameters
   ros2 param list /servo_node
   ```

## 🔗 Dependencies

### **ROS 2 Packages**
- `moveit_core` (MoveIt2 core functionality)
- `moveit_ros_planning_interface` (Planning interface)
- `moveit_servo` (Real-time servo control)
- `moveit_visual_tools` (Visualization utilities)
- `moveit_msgs` (MoveIt message types)
- `geometry_msgs`, `trajectory_msgs`
- `tidybot_utils` (Custom message types)

### **External Libraries**
- `Eigen3` (Linear algebra)
- `OMPL` (Open Motion Planning Library)
- `FCL` (Flexible Collision Library)

## 📚 Additional Resources

- [MoveIt2 Documentation](https://docs.ros.org/en/jazzy)
- [MoveIt Servo Tutorial](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
