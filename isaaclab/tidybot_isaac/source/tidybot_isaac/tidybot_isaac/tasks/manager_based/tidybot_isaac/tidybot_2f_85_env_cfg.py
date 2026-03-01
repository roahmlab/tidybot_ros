import math

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.sensors import TiledCameraCfg, ContactSensorCfg, FrameTransformerCfg, OffsetCfg
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

# Standard IsaacLab MDP imports for joint/action observation terms
import isaaclab.envs.mdp as standard_mdp
# Your custom MDP functions from the script above
from . import mdp as custom_mdp
from tidybot_isaac import assets

##
# Scene definition
##

FRAME_MARKER_SMALL_CFG = FRAME_MARKER_CFG.copy()
FRAME_MARKER_SMALL_CFG.markers["frame"].scale = (0.10, 0.10, 0.10)

FORCE_MARKER_CFG = VisualizationMarkersCfg(
    markers={
        "arrow": sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
            scale=(0.02, 0.02, 0.02),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
        )
    }
)

@configclass
class Tidybot2F85SceneCfg(InteractiveSceneCfg):
    """Configuration for the TidyBot drawer opening scene."""
    
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    robot: ArticulationCfg = assets.TIDYBOT_2F85_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    contact_forces_left = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/tidybot/left_inner_finger",
        update_period=0.0,
        history_length=6,
        debug_vis=True,
        track_friction_forces=True,
        filter_prim_paths_expr=["{ENV_REGEX_NS}/Cabinet/drawer_handle_top"],
        max_contact_data_count_per_prim=4,
    )

    contact_forces_right = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/tidybot/right_inner_finger",
        update_period=0.0,
        history_length=6,
        debug_vis=True,
        track_friction_forces=True,
        filter_prim_paths_expr=["{ENV_REGEX_NS}/Cabinet/drawer_handle_top"],
        max_contact_data_count_per_prim=16,
    )

    ee_frame = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Robot/tidybot/bracelet_link",
        visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(prim_path="/Visuals/EEFrameTransformer"),
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Robot/tidybot/bracelet_link",
                name="ee_tcp",
                offset=OffsetCfg(
                    pos=(0.0, 0.0, -0.1815),
                    rot=(0.0, 1.0, 0.0, 0.0),
                ),
            ),
        ],
    )

    cabinet: ArticulationCfg = assets.CABINET_CFG.replace(prim_path="{ENV_REGEX_NS}/Cabinet")
    
    cabinet_frame = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Cabinet/sektion",
        visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(prim_path="/Visuals/CabinetFrameTransformer"),
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Cabinet/drawer_handle_top",
                name="handle",
                offset=OffsetCfg(
                    pos=(0.305, 0.0, 0.01),
                    rot=(0.5, 0.5, -0.5, -0.5), 
                ),
            ),
        ],
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )


##
# MDP settings
##

@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    arm_pos = standard_mdp.RelativeJointPositionActionCfg(
        asset_name="robot",
        joint_names=["joint_[1-7]"],
        scale=0.02, 
    )

    # Replaced continuous mimic with discrete binary actions
    gripper = standard_mdp.BinaryJointPositionActionCfg(
        asset_name="robot",
        joint_names=["finger_joint", "right_outer_knuckle_joint"],
        open_command_expr={
            "finger_joint": 0.0,
            "right_outer_knuckle_joint": 0.0,
        },
        close_command_expr={
            "finger_joint": 0.82,
            "right_outer_knuckle_joint": 0.82,
        },
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        # 1. Arm State
        joint_pos = ObsTerm(func=standard_mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=standard_mdp.joint_vel_rel)
        
        # 2. Cabinet State
        cabinet_joint_pos = ObsTerm(
            func=standard_mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"])},
        )
        cabinet_joint_vel = ObsTerm(
            func=standard_mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"])},
        )
        
        # 3. Geometric Target (3D Vector directly from EE to Handle)
        rel_ee_drawer_distance = ObsTerm(func=custom_mdp.rel_ee_drawer_distance)
        
        # 4. Previous Action
        actions = ObsTerm(func=standard_mdp.last_action)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    reset_robot_base = EventTerm(
        func=standard_mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["joint_x", "joint_y", "joint_th"]),
            "position_range": (0.0, 0.0),
            "velocity_range": (0.0, 0.0),
        },
    )

    reset_robot_arm = EventTerm(
        func=standard_mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["joint_[1-7]"]),
            "position_range": (-0.5, 0.5), 
            "velocity_range": (0.0, 0.0),
        },
    )

    reset_cabinet = EventTerm(
        func=standard_mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("cabinet", joint_names=".*"),
            "position_range": (0.0, 0.0),
            "velocity_range": (0.0, 0.0),
        },
    )


@configclass
class RewardsCfg:
    """Reward terms mathematically balanced for standard RL paradigms."""
    
    approach = RewTerm(
        func=custom_mdp.approach_ee_handle,
        weight=2.0, 
    )

    grasp = RewTerm(
        func=custom_mdp.grasp_handle,
        weight=4.0,
        params={
            "robot_cfg": SceneEntityCfg("robot"),
            "gripper_joint_name": "finger_joint",
        },
    )
    
    drawer_progress = RewTerm(
        func=custom_mdp.open_drawer_bonus,
        weight=10.0,
        params={"cabinet_cfg": SceneEntityCfg("cabinet")},
    )
    
    action_rate = RewTerm(
        func=custom_mdp.action_rate_penalty,
        weight=0.005,
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=standard_mdp.time_out, time_out=True)
    
    success = DoneTerm(
        func=custom_mdp.drawer_opened,
        params={
            "threshold": 0.35,
            "asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"]),
        },
    )


##
# Environment configuration
##

@configclass
class Tidybot2F85EnvCfg(ManagerBasedRLEnvCfg):
    scene: Tidybot2F85SceneCfg = Tidybot2F85SceneCfg(num_envs=4096, env_spacing=4.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self) -> None:
        self.decimation = 4
        self.episode_length_s = 30
        self.viewer.eye = (3.0, 3.0, 2.0)
        self.viewer.lookat = (0.0, 0.0, 0.0)
        self.sim.dt = 1 / 240
        self.sim.substeps = 1
        self.sim.physx.solver_iterations = 8
        self.sim.physx.num_position_iterations = 8