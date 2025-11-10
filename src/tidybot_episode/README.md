# tidybot_episode

## 📖 Overview

`tidybot_episode` bundles the synchronized episode recorder and dataset conversion utilities for TidyBot++. The package sits alongside `tidybot_teleop`: when teleop is launched with `record:=true`, the `synchronized_recorder` node is started automatically and its services are bridged into the WebXR UI so operators can trigger recording, stop runs, and choose to save or discard finished episodes. Collected data is organized per episode with matching ROS 2 bags and MP4 camera streams.

## 🎯 Key Features

### **Synchronized Capture**
- Aligns action commands (base, arm, gripper) with robot observations in a single timer loop
- Produces two ROS 2 bags (`actions/`, `observations/`) plus MP4 videos (`base_camera`, `arm_camera`, `ext_camera`)
### **ROS Integration**
- Exposes `/start_recording`, `/stop_recording`, `/finalize_recording` services for teleop UI control
- Supports both simulation and hardware topics with consistent command/observation interfaces
### **Dataset Conversion**
- `rosbag_to_hdf5` converts recorded episodes into ML-friendly HDF5 layouts

## 🚀 Launch Files

### `synchronized_recorder.launch.py`
Launch the full synchronized recorder node (used automatically by `tidybot_teleop`).

```bash
ros2 launch tidybot_episode synchronized_recorder.launch.py storage_uri:=episode_bag fps:=10.0 use_sim:=true
```

## 🎛️ Recording Nodes

### **Synchronized Recorder (`synchronized_recorder`)**
- Subscribes to observation streams:
  - `/joint_states` (joint positions, used to extract gripper state)
  - `/tidybot/camera_base/color/raw`, `/tidybot/camera_wrist/color/raw`, `/tidybot/camera_ext/color/raw`
  - TF transforms `world→base` and `arm_base_link→bracelet_link` for base/arm poses
- Subscribes to command streams:
  - `/tidybot/base/target_pose`
  - `/tidybot/arm/target_pose`
  - `/tidybot/gripper/commands`
- Services:
  - `/start_recording` (`std_srvs/Empty`)
  - `/stop_recording` (`std_srvs/Empty`)
  - `/finalize_recording` (`std_srvs/SetBool`) — save when `true`, discard when `false`

### **Data Converter (`rosbag_to_hdf5`)**
Converts an actions/observations bag pair into an HDF5 dataset.

```bash
ros2 run tidybot_episode rosbag_to_hdf5 input_dir:=episode_bag output:=data.hdf5
```

### **Data Converter (`rosbag_to_parquet`)**
Converts an actions/observations bag pair into a parquet format.

```bash
ros2 run tidybot_episode rosbag_to_parquet input_dir:=episode_bag output:=data.hdf5
```

## 🔧 Usage

### **Recording from Teleop**
- WebXR UI: press “Start Episode”, “End Episode”, then “Save” or “Discard”.
- CLI:
  ```bash
  ros2 service call /start_recording std_srvs/Empty
  ros2 service call /stop_recording std_srvs/Empty
  ros2 service call /finalize_recording std_srvs/SetBool "{data: true}"   # save
  ```

### **Processing Data**
```bash
# Convert ROS bag to HDF5
ros2 run tidybot_episode rosbag_to_hdf5 input_dir:=episode_bag output:=dataset.hdf5
```

## 📊 Data Structure

### **HDF5 Dataset Format (For diffusion pollicy training)**
```
/root
└── /data
    └── /demo_<demo_id>             
        ├── actions               # Action data (N x 10)
        └── /obs                  # Observed data
            ├── arm_pos           # (N x 3)
            ├── arm_quat          # (N x 4)
            ├── base_image        # (N x 84 x 84 x 3)
            ├── base_pose         # (N x 3)
            ├── gripper_pos       # (N x 1)
            └── wrist_image       # (N x 84 x 84 x 3)
```

### **Parquet Dataset Format (For RLDS generation)**
```
/episode_<timestamp>.parquet
├── observed state          # EEF XYZ (3) + Quaternion (4) + Gripper Open/Close (1) 
├── action                  # EEF Delta XYZ (3) + Roll-Pitch-Yaw (3) + Gripper Open/Close (1)
├── language instruction 
├── index                   # Unique to each episode
├── frame_index 
├── timestamp 
├── task_index              # Unique to each language instruction
```

## 🔗 Dependencies

- `rclcpp`, `rosbag2_cpp`, `rosbag2_storage`
- `sensor_msgs`, `geometry_msgs`, `std_msgs`
- `tf2_ros`, `image_transport`, `cv_bridge`
- `OpenCV` (MP4 encoding for camera streams)
- `HDF5` (dataset export)
