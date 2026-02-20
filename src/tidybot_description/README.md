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
│  │    • PublishJointState → /isaac_sim/joint_states                         │ │
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

### Step 1: Import Robot URDF into Isaac Sim and Prepare the Robot USD

1. **Start Isaac Sim** (via Docker or native installation)

2. **Import the URDF:**
   - Go to `File` → `Import`
   - Select `/workspace/src/tidybot_description/urdf/tidybot_isaac.urdf` (a pre-configured URDF for Isaac Sim)
   - **Important settings:**
     - ✅ Static Base (checked)
     - Output directory: `/tmp` or similar writable path
     - Leave other configurations as default
   - Click "Import"

3. **Verify the robot** appears in the stage at `/World/tidybot`
4. **Add inner knuckle joints** (for closed-loop gripper kinematics):
   - Switch to **Right** view and zoom into the gripper
   - Select `left_inner_knuckle` + `robotiq_arg2f_base_link` → Right-click → **Create > Physics > Joint > Revolute Joint**
   - Position joint center at the knuckle-pad connection (see image below)
   - Repeat for `right_inner_knuckle`
   - Name joints `left_inner_knuckle_joint` and `right_inner_knuckle_joint`
   - For both joints: **Property > Exclude From Articulation** ✅
   - Select both → **Property > Add > Physics > Angular Drive**

   <img width="1648" height="933" alt="Inner knuckle joint placement" src="https://github.com/user-attachments/assets/d6ddc068-3420-42ee-942e-c54f91ddc327" />

5. **Configure gripper joint drives** (Property > Physics > Angular Drive):
   | Joints | Stiffness | Damping | Max Force | Max Velocity |
   |--------|-----------|---------|-----------|--------------|
   | `finger_joint`, `right_outer_knuckle_joint` | 10000 | 100 | 180 | 130 |
   | `left_outer_finger_joint`, `right_outer_finger_joint` | 0.05 | 5000 | 180 | 130 |
   | `*_inner_finger_joint`, `*_inner_finger_knuckle_joint` | 0.00 | 5000 | 180 | 130 |

   **Configure base joint drives** (Property > Physics > Linear/Angular Drive):
   | Joints | Type | Stiffness | Damping | Max Force |
   |--------|------|-----------|---------|-----------|
   | `joint_x`, `joint_y` | Linear | 1e7 | 1e4 | 1e7 |
   | `joint_th` | Angular | 1e7 | 1e4 | 1e7 |

   **Configure arm joint drives** (Property > Physics > Angular Drive):
   | Joints | Stiffness | Damping | Max Force |
   |--------|-----------|---------|-----------|
   | `joint_1` through `joint_7` | 1e7 | 1e4 | 1e7 |

6. **Apply fingertip material**:
   - **Create > Physics > Physics Material > Rigid Body Material** → rename to `fingertip_material`
   - Set friction coefficients to `0.8`, Friction Combine Mode to `max`
   - Apply to `left_inner_finger` and `right_inner_finger` and choose `Strength` to `Stronger than Descendants`

7. **Add fingertip colliders**:
   - Select `left_inner_finger/collisions` → uncheck **Instanceable**
   - Navigate to `collisions/robotiq_arg2f_85_inner_finger/node_STL_BINARY_/mesh`
   - **Add > Physics > Collider**, set Rest Offset=`0.0`, Contact Offset=`0.005`
   - Repeat for `robotiq_arg2f_85_pad` mesh and for `right_inner_finger`

8. **Save the USD**: `File → Collect and Save As...` to a mounted directory

Now you have the USD file ready to be used in the simulation.

### Step 2: Create a Simulation Scene with the robot

1. Open Isaac Sim
2. In the stage, select `/World`, right-click and select `Create -> Xform`, rename it to `Robot`
3. Right-click the `Robot` xform, select `Add -> Reference`, select the USD file you just saved
4. Now the robot is in the stage under `/World/Robot/tidybot`

5. Load Simulation Control Extension 
    - Go to `Window` → `Extensions`
    - Search for `isaacsim.ros2.sim_control`
    - Enable the extension
    - (Do this if you did not enable the extension when launching Isaac Sim)

### Step 3: Configure the Joint Home Positions, Cameras and ROS 2 Integration

Use the setup script in Isaac Sim's Script Editor to configure physics and ROS 2 integration:

1. Open Isaac Sim's Script Editor (`Window` → `Script Editor`)
2. Paste and run the following script:

```python
"""
Isaac Sim Setup Script for TidyBot++
Run this after importing the URDF into Isaac Sim.

Gripper: Uses spring-loaded mechanism for the outer fingers.
Outer finger joints are revolute and driven by angular drives.
The isaac_sim_bridge.py applies gear ratios to command all joints together.
"""

import omni.usd
from pxr import Usd, UsdPhysics, Sdf, UsdGeom, Gf, PhysxSchema, UsdShade
import omni.graph.core as og
import math

# Configuration
ROBOT_PATH = "/World/Robot/tidybot"
BASE_JOINTS = ["joint_x", "joint_y", "joint_th"]
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]
GRIPPER_JOINTS = ["finger_joint", "left_inner_finger_knuckle_joint", "left_inner_finger_joint",
                  "right_outer_knuckle_joint", "right_inner_finger_knuckle_joint", "right_inner_finger_joint",
                  "left_outer_finger_joint", "right_outer_finger_joint"]
CAMERAS = {
    "arm_camera": {
        "parent_link": "bracelet_link/end_effector_link/arm_camera_link",
        "topic": "/tidybot/camera_wrist/color",
        "frame_id": "arm_camera_link",
        "width": 640,
        "height": 480,
        "horizontal_fov": 1.247,  # radians (~71.4 degrees)
        # Pose offset from link (camera optical frame relative to link)
        "position": Gf.Vec3d(0.0, -0.01, 0.03),
        "orientation": Gf.Quatf(0.0, -1.0, 0.0, 0.0),  # Quaternion (w,x,y,z)
    },
    "base_camera": {
        "parent_link": "base/base_camera_link",
        "topic": "/tidybot/camera_base/color",
        "frame_id": "base_camera_link",
        "width": 640,
        "height": 360,
        "horizontal_fov": 1.434,  # radians (~82.2 degrees)
        "position": Gf.Vec3d(0.0, 0.0, 0.0),
        "orientation": Gf.Quatf(0.5, 0.5, -0.5, -0.5),
    },
}

stage = omni.usd.get_context().get_stage()

# Helper function to get prim at path (handles string to Sdf.Path conversion)
def get_prim(path_str):
    return stage.GetPrimAtPath(Sdf.Path(path_str))

print("\nSetting initial joint positions...")

# Initial positions in RADIANS (from tidybot.ros2_control.xacro)
# Note: DriveAPI uses DEGREES, JointStateAPI uses RADIANS
INITIAL_JOINT_POSITIONS = {
    # Base joints (linear joints use meters)
    "joint_x": 0.0,        # meters
    "joint_y": 0.0,        # meters
    "joint_th": 0.0,       # radians
    # Arm joints (gen3_7dof) - radians
    "joint_1": 0.0,
    "joint_2": -0.35,
    "joint_3": 3.141522,
    "joint_4": -2.36,
    "joint_5": 0.0,
    "joint_6": -1.13,
    "joint_7": 1.574186,
    # Gripper - all joints start at 0 (open position)
    # The isaac_sim_bridge applies gear ratios when commanding
    "finger_joint": 0.0,
    "right_outer_knuckle_joint": 0.0,
    "left_inner_finger_joint": 0.0,
    "right_inner_finger_joint": 0.0,
    "left_inner_finger_knuckle_joint": 0.0,
    "right_inner_finger_knuckle_joint": 0.0,
}

# Find and configure each joint by traversing the stage
for prim in stage.Traverse():
    joint_name = prim.GetName()
    if joint_name not in INITIAL_JOINT_POSITIONS:
        continue
    
    pos = INITIAL_JOINT_POSITIONS[joint_name]
    
    # Determine if linear or angular joint
    if joint_name in ["joint_x", "joint_y"]:
        drive_type = "linear"
        state_type = "linear"
        pos_drive = pos  # Linear uses meters for both
        pos_state = pos
        unit = "m"
    else:
        drive_type = "angular"
        state_type = "angular"
        pos_drive = pos * 180.0 / math.pi  # DriveAPI uses degrees
        pos_state = pos * 180.0 / math.pi  # JointStateAPI also uses degrees in Isaac Sim
        unit = "°"
    
    # 1. Set drive target (where controller aims)
    drive_api = UsdPhysics.DriveAPI.Get(prim, drive_type)
    if drive_api:
        drive_api.GetTargetPositionAttr().Set(pos_drive)
    
    # 2. Set the physics body initial state (where joint STARTS on reset)
    state_api = PhysxSchema.JointStateAPI.Apply(prim, state_type)
    if state_api:
        state_api.CreatePositionAttr(pos_state)
        state_api.CreateVelocityAttr(0.0)
    
    print(f"  Set {joint_name}: start={pos_state:.2f}{unit}, target={pos_drive:.2f}{unit}")

print("Initial positions configured!")

# Create cameras under parent links
def create_camera(camera_name, config):
    """Create a camera prim under the parent link."""
    parent_path = f"{ROBOT_PATH}/{config['parent_link']}"
    parent_prim = get_prim(parent_path)
    if not parent_prim.IsValid():
        print(f"  WARNING: Parent link not found: {parent_path}")
        return None
    
    camera_path = f"{parent_path}/{camera_name}"
    
    # Remove existing camera if present
    existing = get_prim(camera_path)
    if existing.IsValid():
        stage.RemovePrim(Sdf.Path(camera_path))
    
    # Create camera prim
    camera = UsdGeom.Camera.Define(stage, Sdf.Path(camera_path))
    
    # Calculate focal length from horizontal FOV and horizontal aperture
    # Default horizontal aperture is 20.955mm (35mm film equivalent)
    h_aperture = 20.955
    focal_length = h_aperture / (2.0 * math.tan(config["horizontal_fov"] / 2.0))
    
    # Set camera properties
    camera.CreateFocalLengthAttr(focal_length)
    camera.CreateHorizontalApertureAttr(h_aperture)
    camera.CreateVerticalApertureAttr(h_aperture * config["height"] / config["width"])
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
    
    # Set transform (position and orientation)
    xformable = UsdGeom.Xformable(camera.GetPrim())
    xformable.ClearXformOpOrder()
    
    pos = config["position"]
    orient = config["orientation"]
    
    xformable.AddTranslateOp().Set(pos)
    xformable.AddOrientOp().Set(orient)
    
    print(f"  Created camera: {camera_path} (focal_length={focal_length:.2f}mm)")
    return camera_path

print("\nConfiguring cameras...")
camera_paths = {}
for cam_name, cam_config in CAMERAS.items():
    cam_path = create_camera(cam_name, cam_config)
    if cam_path:
        camera_paths[cam_name] = cam_path

# Create ROS 2 Action Graph
print("\nCreating ROS 2 Action Graph...")
graph_path = "/World/ROS2_ActionGraph"
existing = get_prim(graph_path)
if existing.IsValid():
    stage.RemovePrim(Sdf.Path(graph_path))

# Build node list - base nodes plus camera nodes
nodes_to_create = [
    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
    ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
    ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
]

# Add camera render product and publisher nodes
for cam_name in camera_paths:
    nodes_to_create.extend([
        (f"{cam_name}_RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        (f"{cam_name}_CameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
    ])

# Build values list
values_to_set = [
    ("PublishClock.inputs:topicName", "/clock"),
    ("PublishJointState.inputs:topicName", "/joint_states"),
    ("SubscribeJointState.inputs:topicName", "/joint_command"),
    ("ArticulationController.inputs:robotPath", artic_root_path),
    ("PublishJointState.inputs:targetPrim", [Sdf.Path(artic_root_path)]),
]

# Add camera values
for cam_name, cam_path in camera_paths.items():
    config = CAMERAS[cam_name]
    values_to_set.extend([
        (f"{cam_name}_RenderProduct.inputs:cameraPrim", [Sdf.Path(cam_path)]),
        (f"{cam_name}_RenderProduct.inputs:width", config["width"]),
        (f"{cam_name}_RenderProduct.inputs:height", config["height"]),
        (f"{cam_name}_CameraHelper.inputs:topicName", f"{config['topic']}/raw"),
        (f"{cam_name}_CameraHelper.inputs:frameId", config["frame_id"]),
        (f"{cam_name}_CameraHelper.inputs:type", "rgb"),
    ])

# Build connections list
connections = [
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
]

# Add camera connections
for cam_name in camera_paths:
    connections.extend([
        ("OnPlaybackTick.outputs:tick", f"{cam_name}_RenderProduct.inputs:execIn"),
        (f"{cam_name}_RenderProduct.outputs:execOut", f"{cam_name}_CameraHelper.inputs:execIn"),
        (f"{cam_name}_RenderProduct.outputs:renderProductPath", f"{cam_name}_CameraHelper.inputs:renderProductPath"),
    ])

og.Controller.edit(
    {"graph_path": graph_path, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: nodes_to_create,
        og.Controller.Keys.SET_VALUES: values_to_set,
        og.Controller.Keys.CONNECT: connections,
    }
)
```

The script above does these things:
- Sets **initial joint positions** (home pose) for the arm and gripper
- Creates **camera prims** under the arm and base camera links with proper FOV and resolution
- Creates a **ROS 2 Action Graph** that:
  - Publishes `/clock` and `/joint_states`
  - Subscribes to `/joint_command` for robot control
  - Publishes camera images to `/tidybot/camera_wrist/color/raw` and `/tidybot/camera_base/color/raw`

After executing, press **Play** to start the simulation. Or you can save the scene for later use.

### Step 4: Launch ROS 2 Stack

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

1. **No `/isaac_sim/joint_states` or `/joint_states` messages:**
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
- [Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/index.html)



