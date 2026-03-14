import argparse
from isaaclab.app import AppLauncher

# 1. PARSE ARGS AND LAUNCH APP FIRST!
parser = argparse.ArgumentParser(description="Test Gripper")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# =================================================================
# 2. NOW WE CAN SAFELY IMPORT EVERYTHING ELSE
# =================================================================
import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg

# Define the USD path (UPDATE THIS TO YOUR ACTUAL PATH)
TIDYBOT_HANDE_USD_PATH = "/workspace/src/tidybot_description/usd/Collected_tidybot_hande/tidybot.usd"

# 3. YOUR EXACT CONFIGURATION (with prim_path added)
TIDYBOT_HANDE_CFG = ArticulationCfg(
    prim_path="/World/Robot",  # REQUIRED: Tells IsaacLab where to spawn the robot
    spawn=sim_utils.UsdFileCfg(
        usd_path=TIDYBOT_HANDE_USD_PATH,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=1,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos={
            "joint_x": 0.0, "joint_y": 0.0, "joint_th": 0.0,
            "joint_1": 0.0, "joint_2": -0.35, "joint_3": 3.14,
            "joint_4": -2.36, "joint_5": 0.0, "joint_6": -1.13, "joint_7": 1.57,
            "hande_left_finger_joint": 0.025,
            "hande_right_finger_joint": 0.025,
        },
    ),
    actuators={
        "base": ImplicitActuatorCfg(
            joint_names_expr=["joint_x", "joint_y", "joint_th"],
            stiffness=1e7, damping=1e4, armature=1000.0,
        ),
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint_[1-7]"],
            stiffness={
                "joint_1": 400.0, "joint_2": 400.0, "joint_3": 300.0,
                "joint_4": 300.0, "joint_5": 300.0, "joint_6": 300.0, "joint_7": 300.0,
            },
            damping={
                "joint_1": 15.0, "joint_2": 15.0, "joint_3": 10.0,
                "joint_4": 10.0, "joint_5": 8.0, "joint_6": 8.0, "joint_7": 8.0,
            },
            armature={
                "joint_1": 0.3, "joint_2": 0.3, "joint_3": 0.3, 
                "joint_4": 0.3, "joint_5": 0.18, "joint_6": 0.18, "joint_7": 0.18
            },
            friction=1.0
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["hande_left_finger_joint", "hande_right_finger_joint"],
            effort_limit=12000.0,   
            stiffness=10000.0,
            damping=2500.0,         # Increased 2.5x to stretch the close time to ~0.8s
            armature=0.1,
        ),
    },
)

def main():
    # Initialize simulation context
    sim_cfg = sim_utils.SimulationCfg(dt=0.001)
    sim = sim_utils.SimulationContext(sim_cfg)

    # Spawn your robot using the config above
    robot = Articulation(cfg=TIDYBOT_HANDE_CFG)
    sim.reset()
    robot.reset()

    # Find the gripper joint indices
    left_finger_idx = robot.find_joints("hande_left_finger_joint")[0][0]
    right_finger_idx = robot.find_joints("hande_right_finger_joint")[0][0]
    
    # ASK ISAACLAB WHAT THE LIMITS ARE
    limits = robot.data.joint_pos_limits[0, left_finger_idx]
    print(f"HARD JOINT LIMITS -> Min: {limits[0].item():.4f}, Max: {limits[1].item():.4f}")
    # ==========================================================
    # WARM-UP PHASE: Force the gripper OPEN and let physics settle
    # ==========================================================
    open_target = torch.tensor([[0.025, 0.025]], device=sim.device)
    print("Warming up physics engine (Holding OPEN for 50 steps)...")
    
    for _ in range(1000):
        robot.set_joint_position_target(open_target, joint_ids=[left_finger_idx, right_finger_idx])
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim_cfg.dt)

    warmup_pos = robot.data.joint_pos[0, left_finger_idx].item()
    print(f"Actual physical position after warmup: {warmup_pos:.4f}")
    # ==========================================================

    # Target position for closed (0.0)
    close_target = torch.tensor([[0.0, 0.0]], device=sim.device)
    
    steps = 0
    is_closed = False

    print("Simulation started. Sending close command...")
    
    while simulation_app.is_running() and not is_closed:
        # Command the gripper to close
        robot.set_joint_position_target(close_target, joint_ids=[left_finger_idx, right_finger_idx])
        
        # Step the physics
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim_cfg.dt)
        steps += 1

        # Read current position
        current_pos = robot.data.joint_pos[0, left_finger_idx].item()
        
        # Check if it has reached the closed state (within a tiny tolerance)
        if current_pos < 0.001:
            is_closed = True
            
        # Failsafe so the script doesn't hang forever
        if steps > 2000: 
            print(f"Timeout: Gripper took too long to close! Stuck at pos: {current_pos:.4f}")
            break
            
    closing_time = steps * sim_cfg.dt
    print(f"Gripper fully closed in {steps} steps.")
    print(f"Total Closing Time: {closing_time:.3f} seconds.")

    simulation_app.close()

if __name__ == "__main__":
    main()