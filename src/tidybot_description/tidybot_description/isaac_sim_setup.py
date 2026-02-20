#!/usr/bin/env python3
"""
Isaac Sim Setup Script for TidyBot++

Configures the robot in Isaac Sim:
  - Sets initial joint positions (home pose)
  - Creates camera prims under arm and base camera links
  - Creates a ROS 2 Action Graph for /clock, /joint_states, /joint_command, and camera topics

Usage:
  Run via Isaac Sim's Script Editor, or from the command line:
    ./isaac-sim.sh --exec "/path/to/isaac_sim_setup.py"
"""

import omni.usd
from pxr import Usd, UsdPhysics, Sdf, UsdGeom, Gf, PhysxSchema, UsdShade
import omni.graph.core as og
import omni.kit.commands
import math

# Enable required extensions for contact sensors
import omni.kit.app
ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("isaacsim.sensors.physics", True)

# Configuration
ROBOT_PATH = "/World/Robot/tidybot"
BASE_JOINTS = ["joint_x", "joint_y", "joint_th"]
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]
GRIPPER_JOINTS = ["finger_joint", "left_inner_finger_knuckle_joint", "left_inner_finger_joint",
                  "right_outer_knuckle_joint", "right_inner_finger_knuckle_joint", "right_inner_finger_joint"]
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

# Note: Contact forces are handled in contact_force_publisher.py using PhysX Contact Report API
# No need to create separate contact sensor prims

stage = omni.usd.get_context().get_stage()

# Helper function to get prim at path (handles string to Sdf.Path conversion)
def get_prim(path_str):
    return stage.GetPrimAtPath(Sdf.Path(path_str))

# Articulation root path (where the ArticulationRootAPI is applied)
artic_root_path = f"{ROBOT_PATH}/root_joint"

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

# Note: Contact sensor XYZ forces are published via contact_force_publisher.py
# The OmniGraph IsaacReadContactSensor node only provides scalar magnitude, not XYZ components

og.Controller.edit(
    {"graph_path": graph_path, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: nodes_to_create,
        og.Controller.Keys.SET_VALUES: values_to_set,
        og.Controller.Keys.CONNECT: connections,
    }
)

print("\n[INFO] Isaac Sim setup complete!")
print("[INFO] Action graph created at: /World/ROS2_ActionGraph")
print("[INFO] Press Play to start simulation, or save the scene.")
