
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.sim.spawners.wrappers import MultiUsdFileCfg

##
# TidyBot Assets — Dual Gripper Configurations
##

# --- Robotiq Hand-E Gripper ---

TIDYBOT_HANDE_USD_PATH = "/workspace/src/tidybot_description/usd/Collected_tidybot_hande/tidybot.usd"

TIDYBOT_HANDE_CFG = ArticulationCfg(
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
            # Base (Prismatic x, y, Revolute yaw)
            "joint_x": 0.0,
            "joint_y": 0.0,
            "joint_th": 0.0,
            # Arm (7 DOF)
            "joint_1": 0.0,
            "joint_2": -0.35,  # Home position
            "joint_3": 3.14,
            "joint_4": -2.36,
            "joint_5": 0.0,
            "joint_6": -1.13,
            "joint_7": 1.57,
            # Hand-E gripper (prismatic, 0.0 = closed, 0.025 = open)
            "hande_left_finger_joint": 0.025,
            "hande_right_finger_joint": 0.025,
        },
    ),
    actuators={
        "base": ImplicitActuatorCfg(
            joint_names_expr=["joint_x", "joint_y", "joint_th"],
            stiffness=1e7,
            damping=1e4,
            armature=10000.0,
        ),
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint_[1-7]"],
            # Taken from tidybot_driver/arm_controller
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
            effort_limit={
                "joint_1": 39.0, "joint_2": 39.0, "joint_3": 39.0, "joint_4": 39.0,
                "joint_5": 9.0,  "joint_6": 9.0,  "joint_7": 9.0,
            },
            friction=1.0
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["hande_left_finger_joint", "hande_right_finger_joint"],
            effort_limit=12000.0,   
            stiffness=10000.0,      # Kept high for a strong, locked grip
            damping=2500.0,         # Increased 2.5x to stretch the close time to ~0.8s
            armature=0.1,
        ),
    },
)


# --- Robotiq 2F-85 Gripper ---

TIDYBOT_2F85_USD_PATH = "/workspace/src/tidybot_description/usd/Collected_tidybot_2f_85/tidybot.usd"

TIDYBOT_2F85_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=TIDYBOT_2F85_USD_PATH,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
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
            # Base (Prismatic x, y, Revolute yaw)
            "joint_x": 0.0,
            "joint_y": 0.0,
            "joint_th": 0.0,
            # Arm (7 DOF)
            "joint_1": 0.0,
            "joint_2": -0.35,  # Home position
            "joint_3": 3.14,
            "joint_4": -2.36,
            "joint_5": 0.0,
            "joint_6": -1.13,
            "joint_7": 1.57,
            # 2F-85 gripper (active joints: [0.0, 0.82], inner joints: [-0.82, 0.0])
            "finger_joint": 0.4,
            "right_outer_knuckle_joint": 0.4,
            "left_inner_finger_knuckle_joint": -0.4,
            "right_inner_finger_knuckle_joint": -0.4,
            "left_inner_finger_joint": -0.4,
            "right_inner_finger_joint": 0.4,
            "left_outer_finger_joint": 0.0,
            "right_outer_finger_joint": 0.0,
        },
    ),
    actuators={
        "base": ImplicitActuatorCfg(
            joint_names_expr=["joint_x", "joint_y", "joint_th"],
            stiffness=1e7,
            damping=1e4,
        ),
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint_[1-7]"],
            stiffness=1e7,
            damping=1e4,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["finger_joint", "right_outer_knuckle_joint"],
            stiffness=1e5,
            damping=100.0,
            effort_limit=180.0,   # Force limit (N)
            velocity_limit=2.27,  # Velocity limit (rad/s) ~130 deg/s
        ),
        "gripper_passive": ImplicitActuatorCfg(
            joint_names_expr=[".*_inner_finger_joint", ".*_inner_finger_knuckle_joint"],
            stiffness=0.0,
            damping=0.005,  # Impact absorption for 4-bar linkage
        ),
        "gripper_outer_passive": ImplicitActuatorCfg(
            joint_names_expr=[".*_outer_finger_joint"],
            stiffness=1.0,
            damping=5000.0,
        ),
    },
)


##
# Cabinet Asset
##

CABINET_USD_URL = "/workspace/tidybot_isaac/cabinet_locked.usd"

CABINET_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=CABINET_USD_URL,
        # Force the root of the cabinet to be fixed to the world
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(fix_root_link=True),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(1.3, 0.0, 0.39),  # Farther away (was 0.7m, requested 1.2m)
        rot=(0.0, 0.0, 0.0, 1.0),  # Original verified rotation
    ),
    actuators={
        "drawer": ImplicitActuatorCfg(
            joint_names_expr=["drawer_top_joint"],
            stiffness=0.0,
            damping=1.0,      # Viscous resistance (force scales with velocity)
            friction=1.0,     # Coulomb friction (constant resistance force in Newtons)
        ),
    },
)

##
# Door Asset
##
door_usd_paths = [f"/workspace/tidybot_isaac/source/tidybot_isaac/tidybot_isaac/tasks/manager_based/open_door/assets/door_{i}.usd" for i in range(20)]
DOOR_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Door",
    spawn=MultiUsdFileCfg(
        usd_path=door_usd_paths,
        random_choice=True,
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(fix_root_link=True),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(1.0, 0.0, 0.7),
        rot=(0.0, 0.0, 0.0, 1.0),
        joint_pos={"HingeJoint": 0.0},
    ),
    actuators={
        "hinge": ImplicitActuatorCfg(
            joint_names_expr=["HingeJoint"],
            stiffness=0.0, 
            damping=0.5, 
            friction=1.0,
            effort_limit=1.0,
        )
    }
)

RECONSTRUCTED_OVEN = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Door",
    spawn=sim_utils.UsdFileCfg(
        usd_path="/workspace/tidybot_isaac/source/tidybot_isaac/tidybot_isaac/tasks/manager_based/open_door/assets/reconstructed_oven.usd",
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(fix_root_link=True),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(1.0, 0.0, 0.7),
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos={"HingeJoint": 0.0}, 
    ),
    actuators={
        "hinge": ImplicitActuatorCfg(
            joint_names_expr=["HingeJoint"],
            stiffness=24.0,
            damping=1.0, 
            friction=0.05,
            effort_limit=5.0,
        ),
    },
)

DEBUG_DOOR_CFG = RigidObjectCfg(
    prim_path="{ENV_REGEX_NS}/Door",
    spawn=sim_utils.UsdFileCfg(
        usd_path="/workspace/tidybot_isaac/source/tidybot_isaac/tidybot_isaac/tasks/manager_based/open_door/assets/door_debug.usd",
    ),
    init_state=RigidObjectCfg.InitialStateCfg(
        pos=(1.0, 0.0, 0.7),
        rot=(0.0, 0.0, 0.0, 1.0),
    ),
)