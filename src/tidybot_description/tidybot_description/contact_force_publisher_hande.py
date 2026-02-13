#!/usr/bin/env python3
"""
Contact Force & Drawer State Publisher for Isaac Sim (Robotiq Hand-E)

Publishes gripper contact forces and drawer handle state to ROS2:
  - /tidybot/contact/left_finger  (geometry_msgs/WrenchStamped) - total force (world frame)
  - /tidybot/contact/right_finger (geometry_msgs/WrenchStamped) - total force (world frame)
  - /tidybot/drawer/state         (std_msgs/Float64MultiArray)  - XYZ pos/vel/acc

Uses subscribe_contact_report_events which provides per-contact-point impulse
vectors. The impulse includes both normal and friction components. Converting
impulse to force via F = impulse / dt.

Usage:
  Run via Isaac Sim's Script Editor, or from the command line:
    ./isaac-sim.sh --exec "/path/to/contact_force_publisher_hande.py"

  Then press Play in Isaac Sim to start publishing.
"""

import omni.usd
from pxr import Usd, Sdf, UsdGeom, UsdPhysics, PhysxSchema, PhysicsSchemaTools
import omni.kit.app
import omni.physx
from omni.physx import get_physx_interface, get_physx_simulation_interface
import numpy as np
import carb

# Enable required extensions
ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)
ext_manager.set_extension_enabled_immediate("omni.physx", True)
ext_manager.set_extension_enabled_immediate("omni.debugdraw", True)

# ROS2 imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

# Configuration — override via Isaac Sim CLI flags:
#   --/app/tidybot/robot_path="/World/Robot/tidybot"
#   --/app/tidybot/drawer_joint_path="/World/some_cabinet/drawer/joint"
#   --/app/tidybot/drawer_handle_path="/World/some_cabinet/drawer_handle_top"
settings = carb.settings.get_settings()
ROBOT_PATH = settings.get("/app/tidybot/robot_path") or "/World/Robot/tidybot"
DRAWER_JOINT_PATH = settings.get("/app/tidybot/drawer_joint_path") or \
    "/World/sektion_cabinet_instanceable/drawer_top/drawer_handle_top_joint"
DRAWER_HANDLE_PATH = settings.get("/app/tidybot/drawer_handle_path") or \
    "/World/sektion_cabinet_instanceable/drawer_handle_top"
LEFT_PAD_PATH = f"{ROBOT_PATH}/hande_left_finger"
RIGHT_PAD_PATH = f"{ROBOT_PATH}/hande_right_finger"

print(f"[CONFIG] ROBOT_PATH: {ROBOT_PATH}")
print(f"[CONFIG] DRAWER_JOINT_PATH: {DRAWER_JOINT_PATH}")
print(f"[CONFIG] DRAWER_HANDLE_PATH: {DRAWER_HANDLE_PATH}")
print(f"[CONFIG] Left  finger: {LEFT_PAD_PATH}")
print(f"[CONFIG] Right finger: {RIGHT_PAD_PATH}")

# Initialize ROS2
if not rclpy.ok():
    rclpy.init()


class ContactForcePublisher(Node):
    """ROS2 node that publishes contact forces and drawer state from Isaac Sim."""

    def __init__(self):
        super().__init__('contact_force_publisher')

        # Contact force publishers (matching sensor_data_recorder.py topics)
        self.left_pub = self.create_publisher(
            WrenchStamped, '/tidybot/contact/left_finger', 10)
        self.right_pub = self.create_publisher(
            WrenchStamped, '/tidybot/contact/right_finger', 10)
        # Drawer state publisher
        self.drawer_pub = self.create_publisher(
            Float64MultiArray, '/tidybot/drawer/state', 10)

        self.get_logger().info('ContactForcePublisher initialized')

        # Contact data storage (set by callback, read by physics step)
        self.left_force = np.zeros(3)
        self.right_force = np.zeros(3)

        # Drawer state tracking
        self.drawer_pos = np.zeros(3)
        self.drawer_vel = np.zeros(3)
        self.drawer_acc = np.zeros(3)
        self._prev_drawer_pos = None
        self._prev_drawer_vel = None

    def update_drawer_state(self, current_pos, dt):
        """Update drawer position, velocity, and acceleration."""
        self.drawer_pos = np.array(current_pos)
        if self._prev_drawer_pos is not None and dt > 0:
            self.drawer_vel = (self.drawer_pos - self._prev_drawer_pos) / dt
        else:
            self.drawer_vel = np.zeros(3)
        if self._prev_drawer_vel is not None and dt > 0:
            self.drawer_acc = (self.drawer_vel - self._prev_drawer_vel) / dt
        else:
            self.drawer_acc = np.zeros(3)
        self._prev_drawer_pos = self.drawer_pos.copy()
        self._prev_drawer_vel = self.drawer_vel.copy()

    def publish_forces(self, sim_time: float):
        """Publish contact forces."""
        sec = int(sim_time)
        nsec = int((sim_time - sec) * 1e9)

        for pub, force in [
            (self.left_pub, self.left_force),
            (self.right_pub, self.right_force),
        ]:
            msg = WrenchStamped()
            msg.header.stamp.sec = sec
            msg.header.stamp.nanosec = nsec
            msg.header.frame_id = "world"
            msg.wrench.force.x = float(force[0])
            msg.wrench.force.y = float(force[1])
            msg.wrench.force.z = float(force[2])
            pub.publish(msg)

    def publish_drawer_state(self):
        """Publish drawer position, velocity, and acceleration."""
        msg = Float64MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='state', size=9, stride=9)
        ]
        msg.data = list(self.drawer_pos) + list(self.drawer_vel) + list(self.drawer_acc)
        self.drawer_pub.publish(msg)


# Create ROS2 node
ros_node = ContactForcePublisher()
stage = omni.usd.get_context().get_stage()

# ─────────────────────────────────────────────────────────────────────────────
# Enable contact reporting on fingertips and drawer handle
# ─────────────────────────────────────────────────────────────────────────────
for path in [LEFT_PAD_PATH, RIGHT_PAD_PATH, DRAWER_HANDLE_PATH]:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        if not prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(prim)
        contact_api = PhysxSchema.PhysxContactReportAPI(prim)
        contact_api.CreateThresholdAttr().Set(0.0)
        print(f"[INFO] Contact reporting enabled on: {path}")
    else:
        raise RuntimeError(f"Prim not found: {path}")

# ─────────────────────────────────────────────────────────────────────────────
# Contact report callback — called by PhysX each step with all contacts
#
# data.impulse contains the TOTAL contact impulse (normal + friction).
# We convert impulse (N·s) to force (N) by dividing by dt.
# ─────────────────────────────────────────────────────────────────────────────
_last_dt = [1.0 / 60.0]


def contact_report_callback(contact_headers, contact_data):
    """Process contact reports to extract total forces on fingertips."""
    dt = _last_dt[0]

    left_force = np.zeros(3)
    right_force = np.zeros(3)

    for header in contact_headers:
        actor0_path = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
        actor1_path = str(PhysicsSchemaTools.intToSdfPath(header.actor1))

        is_left = LEFT_PAD_PATH in actor0_path or LEFT_PAD_PATH in actor1_path
        is_right = RIGHT_PAD_PATH in actor0_path or RIGHT_PAD_PATH in actor1_path

        if not (is_left or is_right):
            continue

        for i in range(header.num_contact_data):
            data = contact_data[header.contact_data_offset + i]
            impulse = np.array([data.impulse[0], data.impulse[1], data.impulse[2]])

            # Convert impulse (N·s) to force (N)
            force = impulse / dt if dt > 0 else impulse

            if is_left:
                left_force += force
            if is_right:
                right_force += force

    ros_node.left_force = left_force
    ros_node.right_force = right_force


# Subscribe to contact reports
contact_sub = get_physx_simulation_interface().subscribe_contact_report_events(
    contact_report_callback)

# ─────────────────────────────────────────────────────────────────────────────
# Debug drawing
# ─────────────────────────────────────────────────────────────────────────────
from omni.debugdraw import get_debug_draw_interface
debug_draw = get_debug_draw_interface()
FORCE_SCALE = 0.01  # 1 N = 0.01m arrow length


def get_prim_world_position(prim_path):
    """Get the world-space position of a prim."""
    prim = stage.GetPrimAtPath(Sdf.Path(prim_path))
    if not prim.IsValid():
        return (0.0, 0.0, 0.0)
    xformable = UsdGeom.Xformable(prim)
    transform = xformable.ComputeLocalToWorldTransform(0)
    t = transform.ExtractTranslation()
    return (t[0], t[1], t[2])


def draw_force_vectors():
    """Draw force arrows at each finger (yellow = total contact force)."""
    YELLOW = 0xFFFFCC00

    for pad_path, force in [
        (LEFT_PAD_PATH, ros_node.left_force),
        (RIGHT_PAD_PATH, ros_node.right_force),
    ]:
        pos = get_prim_world_position(pad_path)
        origin = carb.Float3(pos[0], pos[1], pos[2])
        debug_draw.draw_line(
            origin, YELLOW, 3.0,
            carb.Float3(
                pos[0] + force[0] * FORCE_SCALE,
                pos[1] + force[1] * FORCE_SCALE,
                pos[2] + force[2] * FORCE_SCALE,
            ), YELLOW, 3.0)


# ─────────────────────────────────────────────────────────────────────────────
# Main physics step callback
# ─────────────────────────────────────────────────────────────────────────────
_sim_time = [0.0]


def on_physics_step(dt: float):
    """Called every physics step — publishes forces and drawer state."""
    _sim_time[0] += dt
    _last_dt[0] = dt

    # Update drawer state
    drawer_pos = get_prim_world_position(DRAWER_JOINT_PATH)
    ros_node.update_drawer_state(drawer_pos, dt)

    # ROS2 spin + publish
    rclpy.spin_once(ros_node, timeout_sec=0)
    ros_node.publish_forces(_sim_time[0])
    ros_node.publish_drawer_state()
    draw_force_vectors()


# Subscribe to physics events
physx_subs = get_physx_interface().subscribe_physics_step_events(on_physics_step)

print("[INFO] Contact Force & Drawer State Publisher registered.")
print("[INFO] Using subscribe_contact_report_events (impulse includes normal + friction)")
print("[INFO] Topics:")
print("[INFO]   /tidybot/contact/left_finger   (WrenchStamped, total force N, world frame)")
print("[INFO]   /tidybot/contact/right_finger   (WrenchStamped, total force N, world frame)")
print("[INFO]   /tidybot/drawer/state            (Float64MultiArray, pos/vel/acc)")
print("[INFO] Press Play to start publishing.")
