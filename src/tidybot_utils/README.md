# tidybot_utils

## 📖 Overview

This package defines custom message, service, and action types for the TidyBot++ platform.

## 📦 Package Structure

```
tidybot_utils/
├── msg/
│   ├── TeleopMsg.msg               # Phone teleoperation message
│   └── MotionStage.msg             # Multi-stage motion primitive
├── srv/
│   ├── ResetEnv.srv                # Environment reset service
│   └── OpenDrawerTask.srv          # Drawer task specification service
├── action/
│   ├── OpenDrawer.action           # Legacy drawer action (deprecated)
│   └── ExecuteStages.action        # Multi-stage motion execution action
├── include/
│   └── tidybot_utils/              # C++ header files
└── CMakeLists.txt
```

## 📨 Message Types

### **TeleopMSG**
WebXR teleoperation command message from smartphone/web interface.

**File**: `msg/TeleopMsg.msg`

```
# Timestamp for synchronization
uint64 timestamp

# Control state information
string state_update              # Current control state ("start", "active", "end")
string device_id                 # Unique device identifier
string teleop_mode               # Control mode ("base", "arm", "gripper")

# 3D position command (meters)
float64 pos_x
float64 pos_y  
float64 pos_z

# Quaternion orientation (normalized)
float64 or_x
float64 or_y
float64 or_z
float64 or_w

# Gripper control (-1.0 to 1.0, negative=open, positive=close)
float64 gripper_delta
```

## 🔧 Service Types

### **ResetEnv**
Service for resetting the robot environment to initial conditions.

**File**: `srv/ResetEnv.srv`

```
# Request
bool reset                     # True to reset environment
---
# Response  
bool success                   # True if reset completed successfully
```

### **MotionStage**
Single motion primitive for multi-stage task execution.

**File**: `msg/MotionStage.msg`

```
# Stage type constants
uint8 STAGE_PTP = 0      # Point-to-point (free-space)
uint8 STAGE_LIN = 1      # Linear Cartesian motion
uint8 STAGE_CIRC = 2     # Circular motion (arc)
uint8 STAGE_GRIPPER = 3  # Gripper open/close
uint8 stage_type

# Target pose (for PTP, LIN, CIRC endpoint)
geometry_msgs/Pose target_pose

# For CIRC: arc center and axis
geometry_msgs/Point arc_center
geometry_msgs/Vector3 arc_axis
float64 arc_angle              # radians, CCW around axis

# For GRIPPER
float64 gripper_position       # 0.0=open, 1.0=closed

# Motion parameters
float64 velocity_scaling
float64 duration               # seconds
string description             # for feedback
```

## 🎬 Action Types

### **ExecuteStages**
Multi-stage motion execution action.

**File**: `action/ExecuteStages.action`

```
# Goal
MotionStage[] stages           # Ordered stages to execute
---
# Result
bool success
string message
int32 stages_completed
---
# Feedback
int32 current_stage_index
string current_stage_description
float32 stage_progress         # 0.0 to 1.0
```

### **OpenDrawerTask**
Service for specifying drawer manipulation tasks.

**File**: `srv/OpenDrawerTask.srv`

```
# Request
string joint_type              # "prismatic" or "revolute"
geometry_msgs/Pose handle_pose # Handle position and grasp orientation
geometry_msgs/Point joint_axis_origin  # Pivot for revolute
geometry_msgs/Vector3 joint_axis       # Unit vector
float64 pull_amount            # meters (prismatic) or radians (revolute)
---
# Response
bool accepted
string message
```

## 🔗 Dependencies

### **ROS 2 Packages**
- `std_msgs` (Standard message types)
- `geometry_msgs` (Geometric message types)
- `sensor_msgs` (Sensor message types)
- `rosidl_default_generators` (Message generation)
- `rosidl_default_runtime` (Runtime support)

### **Build Dependencies**
- `ament_cmake` (Build system)
- `builtin_interfaces` (Time and duration types)

## 📚 Usage in Other Packages

### **Package.xml Dependencies**
Add to your package.xml:
```xml
<depend>tidybot_utils</depend>
<exec_depend>rosidl_default_runtime</exec_depend>
```

### **CMakeLists.txt Integration**
Add to your CMakeLists.txt:
```cmake
find_package(tidybot_utils REQUIRED)

# For C++ nodes
ament_target_dependencies(your_node tidybot_utils)

# For Python nodes (automatic with ament_python)
```

### **Python Import**
```python
from tidybot_utils.msg import TeleopMsg
from tidybot_utils.srv import ResetEnv
```

### **C++ Include**
```cpp
#include "tidybot_utils/msg/TeleopMsg.hpp"
#include "tidybot_utils/srv/ResetEnv.hpp"
```

## 🐛 Troubleshooting

### **Message Generation Issues**
```bash
# Rebuild message definitions
colcon build --packages-select tidybot_utils

# Source workspace
source install/setup.bash

# Verify message availability
ros2 interface show tidybot_utils/msg/TeleopMsg
```

### **Import Errors**
```bash
# Check package installation
ros2 pkg list | grep tidybot_utils

# Verify message types
ros2 interface list | grep tidybot_utils
```

