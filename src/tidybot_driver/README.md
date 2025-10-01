# tidybot_driver
## 📖 Overview
This package contains the hardware interface for the Tidybot components. It wraps the original Tidybot++ controllers as ROS nodes, which broadcast command topics for a Kinova Gen3 arm, Robotiq 2f 85 gripper and Tidybot++ mobile base. Additional nodes are implemented for the integrated wrist camera and external camera.

## 🎯 Key Features
TODO

## 📁 Package Structure

```
tidybot_driver/
├── launch/
│   ├── drive.launch.py             # Launch control nodes for arm and base, publishers for cameras and joint state
├── models/                         
│   ├── assets/                     
│   ├── kinova_gen3/                # Arm mujoco models
│   ├── stanford_tidybot/           # Base mujoco models
│   ├── gen3_robotiq_2f_85.urdf     # URDF used by pinocchio to implement compliant joint controller
├── src/
│   ├── arm_controller.py           # Implements core command loop
│   ├── arm_server.py               # ROS wrapper node to receive arm commands
│   ├── base_server.py              # ROS wrapper node to receive base commands
│   ├── base.py                     # Low-level control for mobile base
│   ├── camera_external.py          # Publishes from connected /dev camera
│   ├── camera_wrist.py             # Publishes from Kinova arm's RTSP stream
│   ├── constants.py                # Used by controller nodes
│   ├── ik_solver.py                # IK solver used for compliant joint controller
│   ├── kinova.py                   # Low-level control for kinova arm
│   ├── tidybot.py                  # Joint state publisher for arm and base
```

## 🚀 Launch Files

### `drive.launch.py`
Launch driver nodes and camera publishers, with option for controlling only the arm or the base.

```bash
# Launch only arm driver nodes
ros2 launch tidybot_driver drive.launch.py mode:=arm_only

# Launch only base driver nodes
ros2 launch tidybot_driver drive.launch.py mode:=base_only

# By default, launches all driver nodes
```

## 🎛️ Driver Nodes

### **Arm Driver (`tidybot_arm_server`)**
Connects to the Kinova arm via IP address, listens to joint state or end effector commands and executes on hardware. Simultaneously publishes joint states and end effector pose as observed from hardware and offers a reset service to return to high-level control and bring the arm to its home position.

```bash
ros2 run tidybot_driver arm_server
```

**Subscribed Topics:**
- `/tidybot/arm/command` (sensor_msgs/JointState): Commanded joint angles (joint_1 to joint_7)
- `/tidybot/arm/delta_ee_command` (std_msgs/Float64MultiArray): Commanded end effector deltas, formatted as [delta_pos, delta_rot, gripper]
- `/tidybot/gripper/command` (std_msgs/Float64): Gripper state command

**Published Topics:**
- `/tidybot/arm/pose` (geometry_msgs/pose): End effector pose as read from hardware, with respect to arm base
- `/tidybot/arm/joint_states` (sensor_msgs/JointState): Joint states (joint_1 to joint_7) as observed from hardware

**Services:**
- `/tidybot/arm/reset` (Empty): Temporarily returns to high-level control and brings arm to home position

**Features:**
- **Compliant Joint Controller**: Joints allow deflection while in low-level control mode, as in the original Tidybot++
- **Fault Detection**: Kortex API will detect fault states and disable control in event of self-collision, following error, etc.

### **Base Driver (`tidybot_base_server`)**
Connects to the Phoenix 6 motors via CANivore USB C adapter. Listens to position commands and executes on hardware. Simultaneously publishes odometry as observed from hardware as a joint_state.

```bash
ros2 run tidybot_driver base_server
```

**Subscribed Topics:**
- `/tidybot/base/command` (std_msgs/Float64MultiArray): Commanded position (pos_x, pos_y, theta)

**Published Topics:**
- `/tidybot/base/joint_states` (sensor_msgs/JointState): Observed position and rotation from odometry

**Services:**
- `/tidybot/base/reset` (Empty): Resets motor connection and odometry

### **External Camera Publisher (`camera_ext`)**
Publishes raw and compressed images over ROS, with options for framerate, image resolution and center crop size. By default reads from /dev/video0.

```bash
ros2 run tidybot_driver camera_ext
```

**Published Topics:**
- `/tidybot/camera_ext/color/raw` (sensor_msgs/Image): Cropped and resized raw images, viewable in rqt. By default publishes at 5fps.
- `/tidybot/camera_ext/color/compressed` (sensor_msgs/CompressedImage): Compressed jpg images, useful for streaming over discovery server

### **Wrist Camera Publisher (`camera_wrist`)**
Publishes raw and compressed images over ROS, with options for framerate, image resolution and center crop size. By default reads from RTSP stream over Ethernet at rtsp://192.168.1.10/color. 

```bash
ros2 run tidybot_driver camera_wrist
```

**Published Topics:**
- `/tidybot/camera_wrist/color/raw` (sensor_msgs/Image): Cropped and resized raw images, viewable in rqt. By default publishes at 5fps.
- `/tidybot/camera_wrist/color/compressed` (sensor_msgs/CompressedImage): Compressed jpg images, useful for streaming over discovery server

## 🐛 Troubleshooting

### **Common Issues**

TODO

## 🔗 Dependencies

TODO

## 📚 Additional Resources

TODO
