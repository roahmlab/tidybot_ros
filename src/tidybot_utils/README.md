# tidybot_utils

## 📖 Overview

This package defines custom message and service types specific to the TidyBot++ platform. It provides standardized communication interfaces for teleoperation, environment control, and system coordination across all TidyBot++ packages.

## 📦 Package Structure

```
tidybot_utils/
├── msg/
│   ├── TeleopMsg.msg                    # Phone teleoperation message
├── srv/
│   ├── ResetEnv.srv                # Environment reset service
├── include/
│   └── tidybot_utils/              # C++ header files
└── CMakeLists.txt
```

## 📨 Message Types

### **WSMsg**
WebXR teleoperation command message from smartphone/web interface.

**File**: `msg/TeleopMsg.msg`

```
# Timestamp for synchronization
uint64 timestamp

# Control state information
string state_update              # Current control state ("start", "active", "end")
string device_id                 # Unique device identifier
string teleop_mode              # Control mode ("base", "arm", "gripper")

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

