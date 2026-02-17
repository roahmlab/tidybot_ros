#!/usr/bin/env python3
"""
Contact Force & Drawer State Publisher
Updates: 
- Separates Friction (Tangential) vs Normal (Perpendicular) forces.
- Visualizes Friction (Red), Normal (Blue), and NET FORCE (Green).
- Publishes Net Force X/Y/Z to ROS.
"""

import omni.usd
from pxr import Usd, Sdf, UsdGeom, UsdPhysics, PhysxSchema, PhysicsSchemaTools
import omni.kit.app
import omni.physx
from omni.physx import get_physx_interface, get_physx_simulation_interface
import numpy as np
import carb
from omni.debugdraw import get_debug_draw_interface

# ROS2 imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

# --- CONFIGURATION ---
ROBOT_PATH = "/World/Robot/tidybot" 
LEFT_PAD_PATH = f"{ROBOT_PATH}/hande_left_finger"
RIGHT_PAD_PATH = f"{ROBOT_PATH}/hande_right_finger"

DRAWER_JOINT_PATH = "/World/sektion_cabinet_instanceable/drawer_top/drawer_handle_top_joint"
DRAWER_HANDLE_PATH = "/World/sektion_cabinet_instanceable/drawer_top/drawer_handle_top_visual"

# Visualization Settings
FORCE_SCALE = 0.05
COLOR_YELLOW = 0xFF00FFFF # Total Force (Individual Fingers)
COLOR_RED    = 0xFF0000FF # Friction (Tangential/Shear)
COLOR_BLUE   = 0xFFFF0000 # Normal (Squeeze)
COLOR_GREEN  = 0xFF00FF00 # NET FORCE (Combined Pull)

# Initialize ROS2
if not rclpy.ok():
    rclpy.init()

class ContactForcePublisher(Node):
    def __init__(self):
        super().__init__('contact_force_publisher')
        
        # Publishers
        self.left_pub = self.create_publisher(WrenchStamped, '/tidybot/contact/left_finger', 10)
        self.right_pub = self.create_publisher(WrenchStamped, '/tidybot/contact/right_finger', 10)
        self.net_pub = self.create_publisher(WrenchStamped, '/tidybot/contact/net_force', 10) 
        self.drawer_pub = self.create_publisher(Float64MultiArray, '/tidybot/drawer/state', 10)

        # -- Data Storage --
        # Total Force (Friction + Normal)
        self.left_force = np.zeros(3)
        self.right_force = np.zeros(3)
        self.net_force = np.zeros(3) 

        # Components (For Debugging/Visualization)
        self.left_friction = np.zeros(3)
        self.left_normal = np.zeros(3)
        self.right_friction = np.zeros(3)
        self.right_normal = np.zeros(3)
        
        # Center of Pressure
        self.left_cop = np.zeros(3)
        self.right_cop = np.zeros(3)
        self.handle_center = np.zeros(3)

        # Drawer Kinematics
        self.drawer_pos = np.zeros(3)
        self.drawer_vel = np.zeros(3)
        self.drawer_acc = np.zeros(3)
        self._prev_drawer_pos = None
        self._prev_drawer_vel = None

    def update_drawer_state(self, current_pos, dt):
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

    def publish(self, sim_time):
        sec = int(sim_time)
        nsec = int((sim_time - sec) * 1e9)
        
        # 1. Publish Individual Wrenches
        for pub, force in [(self.left_pub, self.left_force), (self.right_pub, self.right_force)]:
            msg = WrenchStamped()
            msg.header.stamp.sec = sec
            msg.header.stamp.nanosec = nsec
            msg.header.frame_id = "world"
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = force
            pub.publish(msg)

        # 2. Publish NET FORCE (The Green Arrow)
        # This represents the actual vector sum acting on the handle
        self.net_force = self.left_force + self.right_force
        
        msg_net = WrenchStamped()
        msg_net.header.stamp.sec = sec
        msg_net.header.stamp.nanosec = nsec
        msg_net.header.frame_id = "world"
        msg_net.wrench.force.x, msg_net.wrench.force.y, msg_net.wrench.force.z = self.net_force
        self.net_pub.publish(msg_net)

        # 3. Publish Drawer State
        msg = Float64MultiArray()
        msg.layout.dim = [MultiArrayDimension(label='state', size=9, stride=9)]
        msg.data = list(self.drawer_pos) + list(self.drawer_vel) + list(self.drawer_acc)
        self.drawer_pub.publish(msg)

# Global Setup
ros_node = ContactForcePublisher()
stage = omni.usd.get_context().get_stage()
debug_draw = get_debug_draw_interface()

# ---------------------------------------------------------
# 1. Apply Contact API
# ---------------------------------------------------------
for path in [LEFT_PAD_PATH, RIGHT_PAD_PATH, DRAWER_HANDLE_PATH]:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        if not prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(prim)
        PhysxSchema.PhysxContactReportAPI(prim).CreateThresholdAttr().Set(0.0)
        print(f"[INFO] Contact reporting enabled on: {path}")
    else:
        print(f"[WARN] Prim not found: {path}")

# ---------------------------------------------------------
# 2. Contact Report Callback (Physics Thread)
# ---------------------------------------------------------
_last_dt = [1.0 / 60.0]

def contact_report_callback(contact_headers, contact_data):
    dt = _last_dt[0]
    if dt <= 0: return

    # Temporary Accumulators
    l_accum = {"total": np.zeros(3), "friction": np.zeros(3), "normal": np.zeros(3), "pos": np.zeros(3), "count": 0}
    r_accum = {"total": np.zeros(3), "friction": np.zeros(3), "normal": np.zeros(3), "pos": np.zeros(3), "count": 0}

    for header in contact_headers:
        # Identify Actors
        act0_path = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
        act1_path = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
        
        is_left_0 = LEFT_PAD_PATH in act0_path
        is_left_1 = LEFT_PAD_PATH in act1_path
        is_right_0 = RIGHT_PAD_PATH in act0_path
        is_right_1 = RIGHT_PAD_PATH in act1_path

        target_accum = None
        sign_flip = 1.0 

        if is_left_0:
            target_accum = l_accum
            sign_flip = 1.0
        elif is_left_1:
            target_accum = l_accum
            sign_flip = -1.0
        elif is_right_0:
            target_accum = r_accum
            sign_flip = 1.0
        elif is_right_1:
            target_accum = r_accum
            sign_flip = -1.0
        else:
            continue

        for i in range(header.num_contact_data):
            d = contact_data[header.contact_data_offset + i]
            
            # 1. Raw Data
            raw_impulse = np.array([d.impulse[0], d.impulse[1], d.impulse[2]])
            raw_normal = np.array([d.normal[0], d.normal[1], d.normal[2]])
            pos = np.array([d.position[0], d.position[1], d.position[2]])

            norm_len = np.linalg.norm(raw_normal)
            if norm_len > 1e-6:
                raw_normal /= norm_len

            # 2. Calculate Total Force on the Finger
            total_force = (raw_impulse / dt) * sign_flip
            
            # 3. Decompose
            normal_component_scalar = np.dot(total_force, raw_normal)
            force_normal_vec = normal_component_scalar * raw_normal
            force_friction_vec = total_force - force_normal_vec

            # 4. Accumulate
            target_accum["total"] += total_force
            target_accum["friction"] += force_friction_vec
            target_accum["normal"] += force_normal_vec
            target_accum["pos"] += pos
            target_accum["count"] += 1

    # Apply to Node
    ros_node.left_force = l_accum["total"]
    ros_node.left_friction = l_accum["friction"]
    ros_node.left_normal = l_accum["normal"]
    
    ros_node.right_force = r_accum["total"]
    ros_node.right_friction = r_accum["friction"]
    ros_node.right_normal = r_accum["normal"]

    if l_accum["count"] > 0: ros_node.left_cop = l_accum["pos"] / l_accum["count"]
    if r_accum["count"] > 0: ros_node.right_cop = r_accum["pos"] / r_accum["count"]
    
    # Calculate Center of Handle (Midpoint between fingers) for drawing the Net Force
    if l_accum["count"] > 0 and r_accum["count"] > 0:
        ros_node.handle_center = (ros_node.left_cop + ros_node.right_cop) / 2.0
    elif l_accum["count"] > 0:
        ros_node.handle_center = ros_node.left_cop
    elif r_accum["count"] > 0:
        ros_node.handle_center = ros_node.right_cop

contact_sub = get_physx_simulation_interface().subscribe_contact_report_events(contact_report_callback)

# ---------------------------------------------------------
# 3. Helpers
# ---------------------------------------------------------
def get_world_pos(path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid(): return np.zeros(3)
    return np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractTranslation())

def draw_arrow(start, vec, color):
    mag = np.linalg.norm(vec)
    if mag < 0.1: return 
    
    end = start + (vec * FORCE_SCALE)
    debug_draw.draw_line(
        carb.Float3(float(start[0]), float(start[1]), float(start[2])),
        color, 5.0,
        carb.Float3(float(end[0]), float(end[1]), float(end[2])), 
        color, 5.0
    )

# ---------------------------------------------------------
# 4. Main Loop
# ---------------------------------------------------------
_sim_time = [0.0]

def on_physics_step(dt: float):
    _sim_time[0] += dt
    _last_dt[0] = dt
    
    # 1. Update Drawer State
    drawer_pos = get_world_pos(DRAWER_JOINT_PATH)
    ros_node.update_drawer_state(drawer_pos, dt)
    
    # 2. ROS Spin & Publish
    rclpy.spin_once(ros_node, timeout_sec=0)
    ros_node.publish(_sim_time[0])
    
    # LEFT FINGER
    draw_arrow(ros_node.left_cop, ros_node.left_friction, COLOR_RED)  
    draw_arrow(ros_node.left_cop, ros_node.left_normal, COLOR_BLUE)   

    # RIGHT FINGER
    draw_arrow(ros_node.right_cop, ros_node.right_friction, COLOR_RED)
    draw_arrow(ros_node.right_cop, ros_node.right_normal, COLOR_BLUE)
    
    # NET FORCE (GREEN ARROW)
    # Visualizes the combined pull of both fingers
    draw_arrow(ros_node.handle_center, ros_node.net_force, COLOR_GREEN)

physx_sub = get_physx_interface().subscribe_physics_step_events(on_physics_step)

print("[INFO] Script Running.")
print("[INFO] RED Arrow   = Friction Force")
print("[INFO] BLUE Arrow  = Normal Force (Squeeze)")
print("[INFO] GREEN Arrow = NET FORCE (Resultant Pull)")
