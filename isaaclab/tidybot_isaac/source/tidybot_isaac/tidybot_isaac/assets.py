
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg

##
# TidyBot Asset
##

TIDYBOT_USD_PATH = "/workspace/src/tidybot_description/usd/Collected_tidybot/tidybot.usd"

TIDYBOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=TIDYBOT_USD_PATH,
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
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
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
            "joint_2": -0.35, # Home position
            "joint_3": 3.14,
            "joint_4": -2.36,
            "joint_5": 0.0,
            "joint_6": -1.13,
            "joint_7": 1.57,
            # Gripper
            "finger_joint": 0.4,
            "right_outer_knuckle_joint": 0.4,
            "left_inner_finger_knuckle_joint": 0.4,
            "right_inner_finger_knuckle_joint": 0.4,
            "left_inner_finger_joint": 0.4,
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
            damping=1.0,
        ),
        "gripper_passive": ImplicitActuatorCfg(
            joint_names_expr=[".*_inner_finger_joint", ".*_inner_finger_knuckle_joint"],
            stiffness=0.0, 
            damping=0.0,
        ),
    },
)

##
# Cabinet Asset
##

CABINET_USD_URL = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd"

CABINET_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=CABINET_USD_URL,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(1.5, 0.0, 0.39), # Place it in front of the robot
        rot=(0.0, 0.0, 0.0, 1.0), # Rotate 180 deg z
    ),
    actuators={
        "drawer": ImplicitActuatorCfg(
            joint_names_expr=["drawer_top_joint"], 
            stiffness=0.0,
            damping=1.0, # Some friction
        ),
    },
)
