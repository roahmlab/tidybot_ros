import os
import random
from pxr import Usd, UsdGeom, UsdPhysics, Gf
import omni.usd

def generate_procedural_doors(output_dir, num_doors=20):
    os.makedirs(output_dir, exist_ok=True)
    
    for i in range(num_doors):
        usd_path = os.path.join(output_dir, f"door_{i}.usd")
        if os.path.exists(usd_path):
            os.remove(usd_path)
            
        stage = Usd.Stage.CreateNew(usd_path)
        
        # Set Stage Up-Axis to Z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        
        # Only randomize the width for the canonical asset
        min_width = 0.12
        max_width = 0.25
        width = min_width + i * (max_width - min_width) / (num_doors - 1)
        height = 0.2
        thickness = 0.02
        handle_radius = 0.0075
        handle_gap = 0.025
        handle_edge_margin = handle_radius 
        clearance = 0.005 
        
        # Clean root path for the USD asset
        prim_path = "/Door"
        
        # Create canonical root transform (Articulation Root)
        root_xform = UsdGeom.Xform.Define(stage, prim_path)
        root_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
        root_xform.AddOrientOp().Set(Gf.Quatf(1.0)) 
        root_xform.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0)) 
        stage.SetDefaultPrim(root_xform.GetPrim())
        
        UsdPhysics.ArticulationRootAPI.Apply(root_xform.GetPrim())
        
        base_thickness = 0.04
        base_width = width 
        lower_limit, upper_limit = 0.0, 100.0
        
        # Calculate the original base center to use as a global offset
        orig_base_x = thickness/2 + base_thickness/2 + clearance
        orig_base_y = 0.0 
        orig_base_z = height/2
        
        # ==========================================
        # 1. Base Link (Xform) + Geometry Child
        # ==========================================
        base_path = f"{prim_path}/Base"
        base_xform = UsdGeom.Xform.Define(stage, base_path)
        base_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
        base_xform.AddOrientOp().Set(Gf.Quatf(1.0)) 
        
        UsdPhysics.RigidBodyAPI.Apply(base_xform.GetPrim())
        UsdPhysics.MassAPI.Apply(base_xform.GetPrim()).CreateMassAttr(10.0)

        base_geom = UsdGeom.Cube.Define(stage, f"{base_path}/Geometry")
        base_geom.CreateSizeAttr(1.0) 
        base_geom.AddScaleOp().Set(Gf.Vec3f(base_thickness, base_width, height + 0.1))
        UsdPhysics.CollisionAPI.Apply(base_geom.GetPrim())

        # Anchor Base to World
        world_anchor = UsdPhysics.FixedJoint.Define(stage, f"{base_path}/WorldAnchor")
        world_anchor.CreateBody1Rel().SetTargets([base_path])

        # ==========================================
        # 2. Door Link (Xform) + Geometry Child
        # ==========================================
        door_path = f"{prim_path}/DoorPanel"
        door_xform = UsdGeom.Xform.Define(stage, door_path)
        door_xform.AddTranslateOp().Set(Gf.Vec3d(-orig_base_x, -orig_base_y, 0.0))
        door_xform.AddOrientOp().Set(Gf.Quatf(1.0)) 
        
        UsdPhysics.RigidBodyAPI.Apply(door_xform.GetPrim())
        UsdPhysics.MassAPI.Apply(door_xform.GetPrim()).CreateMassAttr(1.0)

        door_geom = UsdGeom.Cube.Define(stage, f"{door_path}/Geometry")
        door_geom.CreateSizeAttr(1.0)
        door_geom.AddScaleOp().Set(Gf.Vec3f(thickness, width, height))
        UsdPhysics.CollisionAPI.Apply(door_geom.GetPrim())

        # ==========================================
        # 3. Hinge Joint (Metric unscaled offsets)
        # ==========================================
        hinge_path = f"{prim_path}/HingeJoint"
        hinge = UsdPhysics.RevoluteJoint.Define(stage, hinge_path)
        hinge.CreateBody0Rel().SetTargets([base_path])
        hinge.CreateBody1Rel().SetTargets([door_path])
        hinge.CreateAxisAttr("Z")
        
        hinge_local_x_base = - (thickness/2 + base_thickness/2 + clearance)
        hinge_local_y = -width / 2.0
        
        hinge.CreateLocalPos0Attr(Gf.Vec3f(hinge_local_x_base, hinge_local_y, 0))
        hinge.CreateLocalPos1Attr(Gf.Vec3f(0.0, hinge_local_y, 0))
        hinge.CreateLowerLimitAttr(lower_limit)
        hinge.CreateUpperLimitAttr(upper_limit)

        # ==========================================
        # 3.5 Hinge Tracking Site (Invisible Frame)
        # ==========================================
        site_path = f"{prim_path}/HingeOrigin"
        site_geom = UsdGeom.Sphere.Define(stage, site_path)
        site_geom.CreateRadiusAttr(0.001) # 1mm sphere
        site_geom.GetPrim().GetAttribute("visibility").Set("invisible") # Hide it
        
        # Position it exactly at the dynamically calculated hinge offset
        site_geom.AddTranslateOp().Set(Gf.Vec3d(hinge_local_x_base, hinge_local_y, 0.0))
        site_geom.AddOrientOp().Set(Gf.Quatf(1.0))
        
        # Apply physics APIs so Isaac Lab parses it as a valid tracked body
        UsdPhysics.RigidBodyAPI.Apply(site_geom.GetPrim())
        UsdPhysics.MassAPI.Apply(site_geom.GetPrim()).CreateMassAttr(1e-5) # Negligible mass
        
        # Weld it rigidly to the Base
        site_weld = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/HingeOriginWeld")
        site_weld.CreateBody0Rel().SetTargets([base_path])
        site_weld.CreateBody1Rel().SetTargets([site_path])
        site_weld.CreateLocalPos0Attr(Gf.Vec3f(hinge_local_x_base, hinge_local_y, 0.0))
        site_weld.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))

        # ==========================================
        # 4. Handle Link (Xform) + Geometry Child
        # ==========================================
        handle_x = - (thickness/2) - handle_gap - handle_radius
        handle_y = (width / 2.0) - handle_edge_margin
        
        handle_path = f"{prim_path}/Handle"
        handle_xform = UsdGeom.Xform.Define(stage, handle_path)
        handle_xform.AddTranslateOp().Set(Gf.Vec3d(handle_x - orig_base_x, handle_y - orig_base_y, 0.0))
        handle_xform.AddOrientOp().Set(Gf.Quatf(1.0)) 
        
        UsdPhysics.RigidBodyAPI.Apply(handle_xform.GetPrim())
        UsdPhysics.MassAPI.Apply(handle_xform.GetPrim()).CreateMassAttr(0.1)

        handle_geom = UsdGeom.Cylinder.Define(stage, f"{handle_path}/Geometry")
        handle_geom.CreateRadiusAttr(handle_radius)
        handle_geom.CreateHeightAttr(height * 0.95)
        handle_geom.CreateAxisAttr("Z")
        UsdPhysics.CollisionAPI.Apply(handle_geom.GetPrim())

        # ==========================================
        # 5. Handle Weld 
        # ==========================================
        handle_joint_path = f"{prim_path}/HandleWeld"
        handle_joint = UsdPhysics.FixedJoint.Define(stage, handle_joint_path)
        handle_joint.CreateBody0Rel().SetTargets([door_path])
        handle_joint.CreateBody1Rel().SetTargets([handle_path])
        
        # Local pos offsets remain identical 
        handle_joint.CreateLocalPos0Attr(Gf.Vec3f(handle_x, handle_y, 0))
        handle_joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))

        stage.GetRootLayer().Save()
        print(f"[{i+1}/{num_doors}] Generated canonical door at {usd_path} | Width: {width:.2f}")

# Execute
output_directory = "/workspace/tidybot_isaac/source/tidybot_isaac/tidybot_isaac/tasks/manager_based/open_door/assets"
generate_procedural_doors(output_directory, num_doors=20)