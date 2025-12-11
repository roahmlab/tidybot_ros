# tidybot_moveit_config

## 📖 Overview

This package contains the auto-generated moveit_config package from the Tidybot URDF in `tidybot_description` and additional config files

## 🎯 Key Components

### **Robot Model Configuration**
- URDF Integration with tidybot_description package
- Joint groups for arm, gripper, base, and combined motion planning
- Collision detection and self-collision matrix

### **Kinematics**
- KDL kinematic solver configurations
- Inverse kinematics solver settings
- Solver parameters: timeout, tolerance, iterations

## 🚀 Launch Files

### `demo.launch.py`
Launch complete MoveIt2 demonstration with RViz visualization.

```bash
# Launch MoveIt demo
ros2 launch tidybot_moveit_config demo.launch.py

```

### `move_group.launch.py`
Launch the core MoveIt2 move_group node for motion planning.

```bash
# Launch move_group node
ros2 launch tidybot_moveit_config move_group.launch.py

# Launch with fake execution
ros2 launch tidybot_moveit_config move_group.launch.py fake_execution:=true
```

## 🎛️ Configuration Files

### **Semantic Robot Description (`tidybot.srdf`)**
- **Planning Groups**: gen3_7dof (arm), base (mobile), whole_body (combined)
- **End Effectors**: gripper configuration
- **Virtual Joints**: base_joint for world connection
- **Collision Settings**: self-collision matrix optimization

### **Joint Limits (`joint_limits.yaml`)**
Velocity, acceleration, and jerk limits for all joints:
- Arm joints: max_velocity 1.3963 rad/s, max_acceleration 8.6 rad/s²
### **Kinematics (`kinematics.yaml`)**
KDL kinematic solver settings for each planning group:
- Search resolution: 0.005
- Solver timeout: 0.05-0.1 seconds
- Solution attempts: 3-5

## 📊 Planning Groups

### **gen3_7dof (Arm)**
- Joints: joint_1 through joint_7
- DOF: 7 degrees of freedom
- Use: Manipulation tasks, pick and place

## 🔧 Customization

### **Modifying Joint Limits**
Edit `config/joint_limits.yaml` to adjust velocity and acceleration limits.

### **Adding Planning Groups**
Use MoveIt Setup Assistant:
```bash
ros2 launch tidybot_moveit_config setup_assistant.launch.py
```
Warning: make sure the auto-generated config package does not overwrite the existing one. It is recommended to compare the generated package and this package and manually add the new planning groups

## 🐛 Troubleshooting

### **Common Issues**

1. **Planning Failures**
   ```bash
   # Check joint limits and robot description
   ros2 param get /move_group robot_description
   
   # Test kinematic solver
   ros2 run moveit_kinematics test_kinematics_plugin gen3_7dof
   ```

2. **Controller Integration**
   ```bash
   # List available controllers
   ros2 control list_controllers
   ```

## 🔗 Dependencies

### **MoveIt2 Packages**
- `moveit_core`, `moveit_ros_planning`
- `moveit_ros_move_group`
- `moveit_kinematics`

### **Robot Description**
- `tidybot_description`
- `robot_state_publisher`
- `joint_state_publisher`

---
