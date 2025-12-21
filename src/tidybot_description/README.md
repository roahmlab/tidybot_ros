# tidybot_description

## 📖 Overview

`tidybot_description` includes the URDF/XACRO source for the TidyBot++ platform, including meshes, controller interfaces, and Gazebo plugins. The package feeds robot_state_publisher on hardware (via `tidybot_driver`) and acts as the foundation for simulation launch files consumed by `tidybot_policy`, `tidybot_solver`, and Gazebo.

## 🤖 Robot Components

### **Mobile Base**
- Powered-caster holonomic base with omnidirectional drive (x, y, yaw)
- Configurable in position or velocity control modes (`tidybot_controllers.yaml`)

### **Manipulator**
- Kinova Gen3 7-DOF arm with compliant joint controller support
- Transmission macros and ROS2 control interfaces for executing trajectories

### **End Effectors**
- Robotiq 2F-85 with visual/collision meshes
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
│   ├── launch_sim_arm.launch.py      # Arm-only Gazebo simulation
│   ├── launch_sim_robot.launch.py    # Full robot Gazebo simulation
│   └── launch_isaac_sim.launch.py    # Isaac Sim integration
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
ros2 launch tidybot_description launch_sim_robot.launch.py base_mode:=velocity
```

**Parameters:**
- `use_rviz` (bool): Launch RViz visualization (default: true)
- `rviz_config` (string): RViz configuration file (default: tidybot.rviz)
- `base_mode` (string): Base control mode - `position` or `velocity` (default: position)
- `world` (string): Gazebo world file (default: empty, choices: office|warehouse|fetch_coke|fetch_cube)

### `launch_isaac_sim.launch.py`
Launch TidyBot ROS 2 stack for use with Isaac Sim (instead of Gazebo).

```bash
ros2 launch tidybot_description launch_isaac_sim.launch.py
```

**Parameters:**
- `use_rviz` (bool): Launch RViz for visualization (default: true)
- `publish_rate` (float): Isaac Sim bridge publishing rate in Hz (default: 50.0)
- `use_velocity_control` (bool): Use velocity control for base instead of position (default: false)
- `rviz_config` (string): Path to RViz config file (default: tidybot.rviz)

> **Note:** Isaac Sim must be running separately with the ROS 2 Action Graph configured. See the [Isaac Sim Integration](#-isaac-sim-integration) section below.

## 🎮 Isaac Sim Integration

This section describes how to use TidyBot with NVIDIA Isaac Sim instead of Gazebo.

### Prerequisites

- Isaac Sim running in the `isaac_sim_tidybot` container launched by the [Isaac Sim container run script](../../docker/isaac-sim-ros2/run.sh) with the ros2 bridge setup

### Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         ISAAC SIM (Docker Container)                         │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │  Action Graph (OmniGraph):                                               │ │
│  │    • PublishClock → /clock                                               │ │
│  │    • PublishJointState → /joint_states                                   │ │
│  │    • SubscribeJointState ← /joint_command                                │ │
│  │    • ArticulationController (applies joint commands)                     │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │ FastDDS
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                       TIDYBOT ROS 2 (Docker Container)                       │
│  ┌─────────────────┐     ┌─────────────────┐     ┌────────────────────────┐ │
│  │ isaac_sim_bridge │ ←── │   MoveIt IK     │ ←── │   Policy Node          │ │
│  │                 │     │                 │     │   (phone/gamepad/etc)  │ │
│  │ Publishes:      │     │ Publishes:      │     │                        │ │
│  │ • /joint_command│     │ • arm traj      │     │ Publishes:             │ │
│  └─────────────────┘     │ • gripper cmds  │     │ • /tidybot/arm/target  │ │
│                          └─────────────────┘     │ • base cmds            │ │
│                                                   └────────────────────────┘ │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │  robot_state_publisher (publishes /tf from /joint_states)               │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Step 1: Import Robot into Isaac Sim and Load Exntensions

1. **Start Isaac Sim** (via Docker or native installation)

2. **Import the URDF:**
   - Go to `File` → `Import`
   - Select `/tidybot_description/urdf/tidybot_isaac.urdf` (a pre-configured URDF for Isaac Sim)
   - **Important settings:**
     - ✅ Static Base (checked)
     - Output directory: `/isaac_workspace/` or similar writable path
     - Leave other configurations as default
   - Click "Import"

3. **Verify the robot** appears in the stage at `/World/tidybot`
4. **Load Simulation Control Extension** 
    - Go to `Window` → `Extensions`
    - Search for `isaacsim.ros2.sim_control`
    - Enable the extension

### Step 2: Configure the Robot

Use the setup script in Isaac Sim's Script Editor to configure physics and ROS 2 integration:

1. Open Isaac Sim's Script Editor (`Window` → `Script Editor`)
2. Paste and run the following script:

```python
"""
Isaac Sim Setup Script for TidyBot++
Run this after importing the URDF into Isaac Sim.
"""

import omni.usd
from pxr import Usd, UsdPhysics, Sdf
import omni.graph.core as og
from pxr import UsdPhysics

# Configuration
ROBOT_PATH = "/World/tidybot"
BASE_JOINTS = ["joint_x", "joint_y", "joint_th"]
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]
GRIPPER_JOINTS = ["left_outer_knuckle_joint", "left_inner_knuckle_joint", "left_inner_finger_joint",
                  "right_outer_knuckle_joint", "right_inner_knuckle_joint", "right_inner_finger_joint"]

stage = omni.usd.get_context().get_stage()

# Find robot
robot_prim = stage.GetPrimAtPath(ROBOT_PATH)
if not robot_prim.IsValid():
    for prim in stage.Traverse():
        if prim.GetName() == "tidybot":
            ROBOT_PATH = str(prim.GetPath())
            robot_prim = prim
            break

print(f"Robot path: {ROBOT_PATH}")

if not robot_prim.IsValid():
    raise Exception("Robot not found! Check the path.")

# Find articulation root (first rigid body under robot)
artic_root_path = None
for child in robot_prim.GetAllChildren():
    if child.HasAPI(UsdPhysics.RigidBodyAPI):
        artic_root_path = str(child.GetPath())
        if not child.HasAPI(UsdPhysics.ArticulationRootAPI):
            UsdPhysics.ArticulationRootAPI.Apply(child)
        print(f"Articulation root: {artic_root_path}")
        break

if not artic_root_path:
    for name in ["joint_x_jointbody", "base_link", "base"]:
        path = f"{ROBOT_PATH}/{name}"
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            artic_root_path = path
            if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                UsdPhysics.ArticulationRootAPI.Apply(prim)
            print(f"Articulation root: {artic_root_path}")
            break

if not artic_root_path:
    raise Exception("Could not find articulation root!")

# Find joints path
joints_path = f"{ROBOT_PATH}/joints"
if not stage.GetPrimAtPath(joints_path).IsValid():
    joints_path = ROBOT_PATH
print(f"Joints path: {joints_path}")

# Configure joint drives
def configure_drive(joint_path, stiffness, damping, max_force, drive_type="angular"):
    prim = stage.GetPrimAtPath(joint_path)
    if not prim.IsValid():
        print(f"  WARNING: Joint not found: {joint_path}")
        return False
    drive = UsdPhysics.DriveAPI.Apply(prim, drive_type)
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateMaxForceAttr(max_force)
    print(f"  Configured: {joint_path} (stiffness={stiffness}, damping={damping}, maxForce={max_force})")
    return True

print("\nConfiguring base joints...")
# Base joints need high stiffness, moderate damping, and HIGH max force for responsive movement
# The base is heavy (~34 kg) so it needs significant force to move quickly
for name in BASE_JOINTS:
    drive_type = "linear" if name in ["joint_x", "joint_y"] else "angular"
    # Stiffness: 1e6 (high for position tracking)
    # Damping: 1e4 (moderate for smooth motion without oscillation)  
    # MaxForce: 1e6 (very high to allow fast movement)
    configure_drive(f"{joints_path}/{name}", 1e6, 1e4, 1e6, drive_type)

print("\nConfiguring arm joints...")
for name in ARM_JOINTS:
    # Arm joints: moderate stiffness and damping, high max force
    configure_drive(f"{joints_path}/{name}", 1e5, 1e3, 1e4, "angular")

print("\nConfiguring gripper joints...")
for name in GRIPPER_JOINTS:
    # Gripper joints: lower stiffness for compliant grasping
    configure_drive(f"{joints_path}/{name}", 1e4, 1e2, 1e3, "angular")

# Create ROS 2 Action Graph
print("\nCreating ROS 2 Action Graph...")
graph_path = "/World/ROS2_ActionGraph"
existing = stage.GetPrimAtPath(graph_path)
if existing.IsValid():
    stage.RemovePrim(graph_path)

og.Controller.edit(
    {"graph_path": graph_path, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublishClock.inputs:topicName", "/clock"),
            ("PublishJointState.inputs:topicName", "/joint_states"),
            ("SubscribeJointState.inputs:topicName", "/joint_command"),
            ("ArticulationController.inputs:robotPath", artic_root_path),
            ("PublishJointState.inputs:targetPrim", [Sdf.Path(artic_root_path)]),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
            ("SubscribeJointState.outputs:execOut", "ArticulationController.inputs:execIn"),
            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
            ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
        ],
    }
)

print("\n" + "="*50)
print("SETUP COMPLETE!")
print("="*50)
print(f"Articulation root: {artic_root_path}")
print("\nSave scene and press PLAY to start simulation.")
```

The script above does these things:
- Locates the robot and sets the **Articulation Root** on the first dynamic rigid body
- Configures **joint drives** with appropriate stiffness/damping for base, arm, and gripper
- Creates a **ROS 2 Action Graph** that publishes `/clock` and `/joint_states`, and subscribes to `/joint_command`

After executing the script, press the `Play` button or space to start the simulation in Isaac Sim

### Step 3: Launch ROS 2 Stack

In the TidyBot ROS 2 container:

```bash
# Launch the Isaac Sim integration stack
ros2 launch tidybot_description launch_isaac_sim.launch.py

# Verify communication
ros2 topic echo /joint_states  # Should show joint positions from Isaac Sim
ros2 topic echo /clock         # Should show simulation time
```

After the ROS2-Isaac Sim bridge is up, you can run the policies to control the robot in Isaac Sim

### Troubleshooting Isaac Sim

1. **No `/joint_states` messages:**
   - Ensure Isaac Sim is in PLAY mode
   - Check that ROS 2 Bridge extension is enabled
   - Verify FastDDS configuration in both containers

2. **Robot doesn't move:**
   - Check ArticulationRoot is on a dynamic body (not `world` link)
   - Verify joint drives are configured with sufficient stiffness
   - Check `/joint_command` topic is receiving messages

3. **Robot falls through ground:**
   - Ensure ground plane has collision enabled
   - Check that base joints have high stiffness (1e7)

4. **Physics instability / robot disappears:**
   - Reduce physics timestep
   - Check for conflicting ArticulationRoot definitions
   - Ensure no kinematic bodies are part of the articulation chain

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
- Gazebo Harmonic (for Gazebo simulation)
- NVIDIA Isaac Sim 5.1.0+ (for Isaac Sim simulation)
- RViz2

## 📚 Additional Resources

- [URDF Tutorials](https://wiki.ros.org/urdf)
- [XACRO Documentation](https://wiki.ros.org/xacro)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)
- [Gazebo ROS2 Integration](https://docs.ros.org/en/jazzy/p/ros_gz_sim)
- [ROS2 Control Framework](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)



