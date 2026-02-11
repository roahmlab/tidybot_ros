#!/usr/bin/env python3
"""
Contact Force & Drawer State Publisher for Isaac Sim

Publishes gripper contact forces and drawer handle state to ROS2:
  - /tidybot/contact/left_finger  (geometry_msgs/WrenchStamped) - XYZ contact impulse
  - /tidybot/contact/right_finger (geometry_msgs/WrenchStamped) - XYZ contact impulse
  - /tidybot/drawer/state         (std_msgs/Float64MultiArray)  - XYZ pos/vel/acc

Also draws debug force vectors (RGB = XYZ) at the left fingertip in the viewport.

Usage:
  Run via Isaac Sim's Script Editor, or from the command line:
    ./isaac-sim.sh --exec "/path/to/contact_force_publisher.py"

  Then press Play in Isaac Sim to start publishing.
"""

import omni.usd
from pxr import Usd, Sdf, UsdGeom, PhysicsSchemaTools
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
from std_msgs.msg import Header, Float64MultiArray, MultiArrayDimension

# Configuration — override via Isaac Sim CLI flags:
#   --/app/tidybot/robot_path="/World/Robot/tidybot"
#   --/app/tidybot/drawer_joint_path="/World/some_cabinet/drawer/joint"
settings = carb.settings.get_settings()
ROBOT_PATH = settings.get("/app/tidybot/robot_path") or "/World/Robot/tidybot"
DRAWER_JOINT_PATH = settings.get("/app/tidybot/drawer_joint_path") or \
    "/World/sektion_cabinet_instanceable/drawer_top/drawer_handle_top_joint"
LEFT_PAD_PATH = f"{ROBOT_PATH}/left_inner_finger"
RIGHT_PAD_PATH = f"{ROBOT_PATH}/right_inner_finger"

print(f"[CONFIG] ROBOT_PATH: {ROBOT_PATH}")
print(f"[CONFIG] DRAWER_JOINT_PATH: {DRAWER_JOINT_PATH}")

# Initialize ROS2
if not rclpy.ok():
    rclpy.init()

class ContactForcePublisher(Node):
    """ROS2 node that publishes contact forces and drawer state from Isaac Sim."""
    
    def __init__(self):
        super().__init__('contact_force_publisher')
        
        # Contact force publishers
        self.left_pub = self.create_publisher(
            WrenchStamped, '/tidybot/contact/left_finger', 10)
        self.right_pub = self.create_publisher(
            WrenchStamped, '/tidybot/contact/right_finger', 10)
        
        # Drawer state publisher: [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z]
        self.drawer_pub = self.create_publisher(
            Float64MultiArray, '/tidybot/drawer/state', 10)
        
        self.get_logger().info('ContactForcePublisher initialized')
        
        # Contact data storage
        self.left_force = [0.0, 0.0, 0.0]
        self.right_force = [0.0, 0.0, 0.0]
        
        # Drawer state tracking (for numerical differentiation)
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
        """Publish stored contact forces."""
        sec = int(sim_time)
        nsec = int((sim_time - sec) * 1e9)
        
        # Publish left finger force
        left_msg = WrenchStamped()
        left_msg.header.stamp.sec = sec
        left_msg.header.stamp.nanosec = nsec
        left_msg.header.frame_id = "left_inner_finger"
        left_msg.wrench.force.x = self.left_force[0]
        left_msg.wrench.force.y = self.left_force[1]
        left_msg.wrench.force.z = self.left_force[2]
        self.left_pub.publish(left_msg)
        
        # Publish right finger force
        right_msg = WrenchStamped()
        right_msg.header.stamp.sec = sec
        right_msg.header.stamp.nanosec = nsec
        right_msg.header.frame_id = "right_inner_finger"
        right_msg.wrench.force.x = self.right_force[0]
        right_msg.wrench.force.y = self.right_force[1]
        right_msg.wrench.force.z = self.right_force[2]
        self.right_pub.publish(right_msg)
    
    def publish_drawer_state(self):
        """Publish drawer position, velocity, and acceleration."""
        msg = Float64MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='state', size=9, stride=9)
        ]
        # Layout: [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z]
        msg.data = list(self.drawer_pos) + list(self.drawer_vel) + list(self.drawer_acc)
        self.drawer_pub.publish(msg)


# Create ROS2 node
ros_node = ContactForcePublisher()

# Get stage and physx interface
stage = omni.usd.get_context().get_stage()

# Contact report callback - receives all contacts in the scene
def contact_report_callback(contact_headers, contact_data):
    """Process contact reports to extract forces on fingertips."""
    left_path = LEFT_PAD_PATH
    right_path = RIGHT_PAD_PATH
    
    left_force = [0.0, 0.0, 0.0]
    right_force = [0.0, 0.0, 0.0]
    
    for header in contact_headers:
        # Decode integer path IDs to string paths (Isaac Sim 5.x API)
        actor0_path = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
        actor1_path = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
        
        # Check if this contact involves our fingertips
        if left_path in actor0_path or left_path in actor1_path:
            # Get contact points for this pair
            for i in range(header.num_contact_data):
                data = contact_data[header.contact_data_offset + i]
                left_force[0] += data.impulse[0]
                left_force[1] += data.impulse[1]
                left_force[2] += data.impulse[2]
        
        if right_path in actor0_path or right_path in actor1_path:
            for i in range(header.num_contact_data):
                data = contact_data[header.contact_data_offset + i]
                right_force[0] += data.impulse[0]
                right_force[1] += data.impulse[1]
                right_force[2] += data.impulse[2]
    
    # Store forces (impulse / dt gives force, but we'll use impulse as proxy)
    ros_node.left_force = left_force
    ros_node.right_force = right_force

# Physics step callback
_sim_time = [0.0]  # Mutable container to track time

# Debug draw for force visualization
from omni.debugdraw import get_debug_draw_interface
debug_draw = get_debug_draw_interface()
FORCE_SCALE = 0.01  # Scale factor: 1 N·s impulse = 0.01m arrow length

def get_prim_world_position(prim_path):
    """Get the world-space position of a prim."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return (0.0, 0.0, 0.0)
    xformable = UsdGeom.Xformable(prim)
    transform = xformable.ComputeLocalToWorldTransform(0)
    t = transform.ExtractTranslation()
    return (t[0], t[1], t[2])

def draw_force_vectors():
    """Draw RGB force arrows at the left finger position."""
    pos = get_prim_world_position(LEFT_PAD_PATH)
    fx, fy, fz = ros_node.left_force
    
    # Colors as packed 0xAARRGGBB integers
    RED = 0xFFCC1919
    GREEN = 0xFF19CC19
    BLUE = 0xFF4D4DFF
    
    origin = carb.Float3(pos[0], pos[1], pos[2])
    
    # Red arrow = X force
    debug_draw.draw_line(
        origin, RED, 3.0,
        carb.Float3(pos[0] + fx * FORCE_SCALE, pos[1], pos[2]), RED, 3.0
    )
    # Green arrow = Y force
    debug_draw.draw_line(
        origin, GREEN, 3.0,
        carb.Float3(pos[0], pos[1] + fy * FORCE_SCALE, pos[2]), GREEN, 3.0
    )
    # Blue arrow = Z force
    debug_draw.draw_line(
        origin, BLUE, 3.0,
        carb.Float3(pos[0], pos[1], pos[2] + fz * FORCE_SCALE), BLUE, 3.0
    )

def on_physics_step(dt: float):
    """Called every physics step."""
    _sim_time[0] += dt
    
    # Update drawer state
    drawer_pos = get_prim_world_position(DRAWER_JOINT_PATH)
    ros_node.update_drawer_state(drawer_pos, dt)
    
    rclpy.spin_once(ros_node, timeout_sec=0)
    ros_node.publish_forces(_sim_time[0])
    ros_node.publish_drawer_state()
    draw_force_vectors()

# Subscribe to physics events using omni.physx
physx_subs = get_physx_interface().subscribe_physics_step_events(on_physics_step)

# Enable contact reporting on the fingertips
from pxr import PhysxSchema
for path in [LEFT_PAD_PATH, RIGHT_PAD_PATH]:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        # Apply contact report API if not already applied
        if not prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(prim)
        contact_api = PhysxSchema.PhysxContactReportAPI(prim)
        contact_api.CreateThresholdAttr().Set(0.0)  # Report all contacts
        print(f"[INFO] Contact reporting enabled on: {path}")

# Subscribe to contact reports using the SIMULATION interface (not regular physx interface)
contact_sub = get_physx_simulation_interface().subscribe_contact_report_events(contact_report_callback)

print("[INFO] Contact Force & Drawer State Publisher registered. Press Play to start publishing.")
print("[INFO] Topics: /tidybot/contact/left_finger, /tidybot/contact/right_finger, /tidybot/drawer/state")
print("[INFO] Forces represent contact impulses (Newton-seconds per step)")
