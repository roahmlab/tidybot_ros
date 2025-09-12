# tidybot_episode

## 📖 Overview

This package provides comprehensive data recording, replay, and dataset generation capabilities for the TidyBot++ mobile manipulator. It captures synchronized multi-modal data during robot operation episodes and converts them into formats suitable for machine learning research and analysis.

## 🎯 Key Features

### **Multi-Modal Data Recording**
- **Robot States**: Joint positions
- **Visual Data**: Synchronized camera feeds from multiple viewpoints
- **Control Commands**: User inputs and policy decisions
- **Metadata**: Timestamps, episode markers

### **Flexible Data Formats**
- **ROS Bags**: Native ROS 2 format for complete system replay
- **HDF5 Files**: Structured format optimized for ML workflows
- **Video Streams**: Compressed camera feeds for visualization

## 🚀 Launch Files

### `record_obs.launch.py`
Launch observation data recording system.

```bash
# Start recording with default settings
ros2 launch tidybot_episode record_obs.launch.py
```

## 🎛️ Recording Nodes

### **Observation Recorder (`obs_recorder`)**
Records multi-modal sensor data and robot states.

**Subscribed Topics:**
- `/joint_states` (sensor_msgs/JointState): Robot joint information
- `/base_camera/image` (sensor_msgs/Image): Base camera image
- `/arm_camera/image` (sensor_msgs/Image): Arm camera image

**Published Services**
- `/obs/start_recording` (std_srvs/Empty): Start episode recording
- `/obs/stop_recording` (std_srvs/Empty): Stop episode recording

### **Actions Recorder (`actions_recorder`)**
Records control commands, user inputs, and policy decisions.

**Subscribed Topics:**
- If the node is run with use_sim:=true:
    - `/tidybot_base_pos_controller/commands` (std_msgs/Float64MultiArray): Base command in the simulation
    - `/gen3_7dof_controller/joint_trajectory`(trajectory_msgs/JointTrajectory): Arm command in the simulation
    - `/robotiq_2f_85_controller/commands` (std_msgs/Float64MultiArray) Gripper command in the simulation
- Otherwise: 
    - `/tidybot/base/commands` (std_msgs/Float64MultiArray): Base command on the physical robot
    - `/tidybot/arm/pose` (trajectory_msgs/JointTrajectory): Arm command on the physical robot
    - `/tidybot/gripper/state` (std_msgs/Float64MultiArray): Gripper command on the physical robot

**Published Services**
- `/actions/start_recording` (std_srvs/Empty): Start episode recording
- `/actions/stop_recording` (std_srvs/Empty): Stop episode recording

### **Data Converter (`rosbag_to_hdf5`)**
Converts ROS bags to HDF5 format for machine learning applications.

```bash
ros2 run tidybot_episode rosbag_to_hdf5 input_dir:=episode_bag output_dir:=data.hdf5
```

## 🔧 Usage

### **Recording Episodes**
```bash
# Start recording
ros2 service call /obs/start_recording std_srvs/Empty

# Stop recording
ros2 service call /obs/stop_recording std_srvs/Empty
```

### **Processing Data**
```bash
# Convert ROS bag to HDF5
ros2 run tidybot_episode rosbag_to_hdf5

# List recorded episodes
ls ~/episode_bag
```

## 📊 Data Structure

### **HDF5 Dataset Format**
```
/root
└── /data
    └── /demo_<demo_id>             
        ├── actions               # Action data (N x 10)
        └── /obs                  # Observed data
            ├── arm_pos           # (N x 3)
            ├── arm_quat          # (N x 4)
            ├── base_image        # (N x 84 x84 x3)
            ├── base_pose         # (N x 3)
            ├── gripper_pos       # (N x 1)
            └── wrist_image       # (N x 84 x 84 x 3)
```

## 🔗 Dependencies

### **ROS 2 Packages**
- `rclcpp` (C++ ROS 2 client library)
- `rosbag2_cpp` (ROS bag recording/playback)
- `sensor_msgs`, `geometry_msgs`, `trajectory_msgs`
- `cv_bridge` (OpenCV-ROS bridge)

### **External Libraries**
- `OpenCV` (Image processing)
- `HDF5` (Hierarchical data format)