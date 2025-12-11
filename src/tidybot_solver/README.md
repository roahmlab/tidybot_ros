# tidybot_solver

## 📖 Overview

This package provides motion planning and trajectory optimization using MoveIt2. It includes a velocity-based kinematics solver for real-time joystick control and position-based kinematics solver for phone teleoperation or remote inference.

## 📁 Package Structure

```
tidybot_solver/
├── config/
│   ├── demo_servo_config.yaml      #  MoveIt Servo config parameters for keyboard input demo
│   ├── servo_parameters.yaml       # General MoveIt Servo settings
│   └── tidybot_servo_config.yaml   # TidyBot-specific MoveIt Servo config
├── launch/
│   ├── demo_twist.launch.py        # Demo servo control for arm
│   ├── launch_moveit_pose_ik.launch.py # Position based solver bridge launch file
│   └── launch_moveit_vel_ik.launch.py # Velocity based solver bridge launch file
├── src/
│   ├── keyboard_input.cpp          # Keyboard control interface for velocity control demonstration
│   ├── moveit_ee_pose_ik.cpp       # Kinematics solver bridge for position based commands using MoveIt ik solver
│   └── moveit_ee_vel_ik.cpp        # Kinematics solver bridge for velocity based commands using Moveit Servo
├── include/
│   └── tidybot_solver/             # C++ header files
└── CMakeLists.txt
```

## 🚀 Keyboard Teleoperation Demo in Gazebo

Launch a simulation scene with the Kinova Gen3 arm and a MoveIt Servo node for realtime servoing

```bash
# Launch motion planning demo
ros2 launch tidybot_solver launch_realtime_servo.launch.py
```
Then in another terminal, launch the keyboard input 
```bash
# Run keyboard input node
ros2 run tidybot_solver keyboard_input
```
The `keyboard_input` node should prompt you with the key bindings to control the simulated arm in Gazebo

## `launch_moveit_vel_ik.launch.py`
Launches velocity based solver bridge with MoveIt Servo API integration. Used by tidybot_policy package

The velocity based solver bridge will subscribe to 
   - `/tidybot/arm/target_vel`: the target velocity of the end-effector in the planning frame 
   - `/tidybot/gripper/commands`: the gripper commands indicating the desired gripper state

Then computes the joint space command for the arm and relay it together with the gripper command to the simulation/hardware control interface

ROS2 arguments:
   - `use_sim` whether to use simulation time

## `launch_moveit_pose_ik.launch.py`
Launches position based solver bridge with MoveIt IK solver. Used by tidybot_policy package for phone teleoperation and remote policy control. We choose MoveIt API instead of MoveIt Servo API because the MoveIt Servo does not seem to do as well for position based control as for velocity control.

The position based solver bridge will subscribe to:
   - `/tidybot/arm/target_pose`: the target pose of the end-effector in the planning frame
   - `/tidybot/arm/delta_commands`: delta commands for position and orientation
   - `/tidybot/gripper/commands`: the gripper commands

Then computes the joint space command for the arm using MoveIt's IK solver and relays it to the simulation/hardware control interface.

ROS2 arguments:
   - `use_sim` whether to use simulation time

## ⚙️ Configuration

### **Servo Control Parameters**
Refer to `config/servo_paramters.yaml` for a complete parameter list
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

## 📚 Additional Resources

- [MoveIt2 Documentation](https://docs.ros.org/en/jazzy)
- [MoveIt Servo Tutorial](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
