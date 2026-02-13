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
from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
import isaaclab.sim as sim_utils

from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from . import mdp
from tidybot_isaac import assets

##
# Scene definition
##


# Define Marker Configs (Standalone)
FRAME_MARKER_SMALL_CFG = FRAME_MARKER_CFG.copy()
FRAME_MARKER_SMALL_CFG.markers["frame"].scale = (0.10, 0.10, 0.10)

FORCE_MARKER_CFG = VisualizationMarkersCfg(
    markers={
        "arrow": sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
            scale=(0.02, 0.02, 0.02),  # Scale down the arrow
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
        )
    }
)

@configclass
class TidybotIsaacSceneCfg(InteractiveSceneCfg):
    """Configuration for the TidyBot drawer opening scene."""
    
    # ... assets ...

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    # robot
    robot: ArticulationCfg = assets.TIDYBOT_HANDE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # Per-finger contact sensors with filter targeting drawer handle
    # (matches open_drawer_collect_data.py approach — friction_forces_w only)
    contact_forces_left = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/tidybot/hande_left_finger",
        update_period=0.0,
        history_length=6,
        debug_vis=True,
        track_friction_forces=True,
        filter_prim_paths_expr=["{ENV_REGEX_NS}/Cabinet/drawer_handle_top"],
        max_contact_data_count_per_prim=4,
    )

    contact_forces_right = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/tidybot/hande_right_finger",
        update_period=0.0,
        history_length=6,
        debug_vis=True,
        track_friction_forces=True,
        filter_prim_paths_expr=["{ENV_REGEX_NS}/Cabinet/drawer_handle_top"],
        max_contact_data_count_per_prim=16,
    )

    # End-effector Frame
    ee_frame = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Robot/tidybot/bracelet_link",
        # debug_vis=True,
        visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(prim_path="/Visuals/EEFrameTransformer"),
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Robot/tidybot/bracelet_link",
                name="ee_tcp",
                offset=OffsetCfg(
                    pos=(0.0, 0.0, -0.1815),
                    rot=(0.0, 1.0, 0.0, 0.0), # w, x, y, z
                ),
            ),
        ],
    )

    # cabinet
    cabinet: ArticulationCfg = assets.CABINET_CFG.replace(prim_path="{ENV_REGEX_NS}/Cabinet")
    
    # Cabinet Frame (Handle Target)
    cabinet_frame = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Cabinet/sektion",
        # debug_vis=True,
        visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(prim_path="/Visuals/CabinetFrameTransformer"),
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Cabinet/drawer_handle_top",
                name="handle", # simplified name
                offset=OffsetCfg(
                    pos=(0.305, 0.0, 0.01),
                    rot=(0.5, 0.5, -0.5, -0.5),  # align with end-effector frame
                ),
            ),
        ],
    )

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )
    
    # Cameras disabled for performance — uncomment to re-enable
    # wrist_camera = TiledCameraCfg(
    #     prim_path="{ENV_REGEX_NS}/Robot/tidybot/bracelet_link/end_effector_link/arm_camera_link/sensor",
    #     data_types=["rgb"],
    #     spawn=sim_utils.PinholeCameraCfg(
    #         focal_length=24.0,
    #         focus_distance=400.0,
    #         horizontal_aperture=20.955,
    #         clipping_range=(0.1, 100.0),
    #     ),
    #     offset=TiledCameraCfg.OffsetCfg(
    #         pos=(0.0, -0.01, 0.03),
    #         rot=(1.0, 0.0, 0.0, 0.0),
    #         convention="ros",
    #     ),
    #     width=640,
    #     height=480,
    # )
    #
    # base_camera = TiledCameraCfg(
    #     prim_path="{ENV_REGEX_NS}/Robot/tidybot/base/base_camera_link/sensor",
    #     data_types=["rgb"],
    #     spawn=sim_utils.PinholeCameraCfg(
    #         focal_length=24.0,
    #         focus_distance=400.0,
    #         horizontal_aperture=20.955,
    #         clipping_range=(0.1, 100.0),
    #     ),
    #     offset=TiledCameraCfg.OffsetCfg(
    #         pos=(0.0, 0.0, 0.0),
    #         rot=(0.5, -0.5, 0.5, -0.5),
    #         convention="ros",
    #     ),
    #     width=640,
    #     height=480,
    # )
    



##
# MDP settings
##


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    # Position control for base
    base_pos = mdp.JointPositionActionCfg(
        asset_name="robot", 
        joint_names=["joint_x", "joint_y", "joint_th"], 
        scale=0.5,
        use_default_offset=True, # Relative to initial spawn?
    )
    
    # Position control for arm
    arm_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["joint_[1-7]"],
        scale=0.1,
        use_default_offset=True,
    )

    # Position control for gripper (Continuous with mimic)
    # Hand-E: 2 prismatic joints, 0.025 = open, 0.0 = closed
    gripper = mdp.MimicGripperActionCfg(
        asset_name="robot",
        joint_names=["hande_left_finger_joint", "hande_right_finger_joint"],
        leader_joint_name="hande_left_finger_joint",
        mimic_multiplier={
            "hande_right_finger_joint": 1.0,
        },
        open_command_expr={"hande_left_finger_joint": 0.025},
        close_command_expr={"hande_left_finger_joint": 0.0},
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # Robot proprioception
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        
        # End-effector position (3D) - where the gripper is
        ee_pos = ObsTerm(
            func=mdp.end_effector_pos,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "ee_body_name": "hande_link",
            },
        )
        
        # EE to handle vector (3D) - CRITICAL: tells policy where to go
        ee_to_handle = ObsTerm(
            func=mdp.ee_to_handle_vector,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "cabinet_cfg": SceneEntityCfg("cabinet"),
                "ee_body_name": "hande_link",
                "handle_offset": (0.0, 0.3, 0.0),
            },
        )
        
        # Gripper state (1D) - how open/closed the gripper is
        gripper_pos = ObsTerm(
            func=mdp.gripper_open_amount,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "gripper_joint_name": "hande_left_finger_joint",
            },
        )
        
        # Drawer state (1D) - how open the drawer is
        drawer_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"])},
        )

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    # reset
    reset_robot = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "position_range": (0.0, 0.0), # Reset to default init_state
            "velocity_range": (0.0, 0.0),
        },
    )

    reset_cabinet = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("cabinet", joint_names=".*"),
            "position_range": (0.0, 0.0), # Reset closed
            "velocity_range": (0.0, 0.0),
        },
    )



@configclass
class RewardsCfg:
    """Reward terms for the MDP - Based on IsaacLab cabinet example."""

    # (1) Approach: Inverse-square reward for reaching handle (smoother gradient)
    approach = RewTerm(
        func=mdp.approach_ee_handle,
        weight=3.0,
        params={
            "threshold": 0.5,
            "robot_cfg": SceneEntityCfg("robot"),
            "cabinet_cfg": SceneEntityCfg("cabinet"),
            "ee_body_name": "hande_link",
        },
    )
    
    # (2) Grasp: Reward closing gripper when near handle
    grasp = RewTerm(
        func=mdp.grasp_handle,
        weight=15.0,
        params={
            "open_joint_pos": 0.025,  # Hand-E max opening (meters)
            "robot_cfg": SceneEntityCfg("robot"),
            "cabinet_cfg": SceneEntityCfg("cabinet"),
            "ee_body_name": "hande_link",
            "gripper_joint_name": "hande_left_finger_joint",
        },
    )
    
    # (3) Drawer progress: Direct reward for drawer position
    drawer_progress = RewTerm(
        func=mdp.open_drawer_bonus,
        weight=10.0,  # Strong weight - main objective
        params={
            "cabinet_cfg": SceneEntityCfg("cabinet"),
        },
    )
    
    # (4) Multi-stage bonus: Progressive rewards for opening stages
    multi_stage = RewTerm(
        func=mdp.multi_stage_open_drawer,
        weight=10.0,
        params={
            "cabinet_cfg": SceneEntityCfg("cabinet"),
        },
    )
    
    # (5) Action regularization: Small penalty for jerky actions
    action_rate = RewTerm(
        func=mdp.action_rate_penalty,
        weight=0.005,
    )
    
    # (6) Alignment: Align tool frame with handle frame
    align_handle = RewTerm(
        func=mdp.align_ee_handle,
        weight=2.0,
    )

    # (7) Penalty: Avoid closing gripper early
    early_close_penalty = RewTerm(
        func=mdp.avoid_early_close,
        weight=-0.5,
        params={
            "robot_cfg": SceneEntityCfg("robot"),
            "gripper_joint_name": "hande_left_finger_joint",
        },
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # (1) Time out
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    
    # (2) Success: Drawer opened
    success = DoneTerm(
        func=mdp.drawer_opened,
        params={
            "threshold": 0.35,
            "asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"]),
        },
    )


##
# Environment configuration
##


@configclass
class TidybotHandeIsaacEnvCfg(ManagerBasedRLEnvCfg):
    # Scene settings
    scene: TidybotIsaacSceneCfg = TidybotIsaacSceneCfg(num_envs=4096, env_spacing=4.0)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    # Post initialization
    def __post_init__(self) -> None:
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 5  # Extended from 5s for complex manipulation
        # viewer settings
        self.viewer.eye = (3.0, 3.0, 2.0)
        self.viewer.lookat = (0.0, 0.0, 0.0)
        # simulation settings
        self.sim.dt = 1 / 120