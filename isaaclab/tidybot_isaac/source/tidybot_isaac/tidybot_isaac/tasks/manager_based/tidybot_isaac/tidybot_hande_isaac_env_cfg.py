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
from isaaclab.utils.noise import GaussianNoiseCfg

# Standard IsaacLab MDP imports for joint/action observation terms
import isaaclab.envs.mdp as standard_mdp
# Custom MDP functions
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
class TidybotHandeIsaacSceneCfg(InteractiveSceneCfg):
    """Configuration for the TidyBot drawer opening scene using the Hand-E Gripper."""
    
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    # Robot Asset (Hand-E)
    robot: ArticulationCfg = assets.TIDYBOT_HANDE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

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
        scale=0.003, 
    )

    gripper = standard_mdp.BinaryJointPositionActionCfg(
        asset_name="robot",
        joint_names=["hande_left_finger_joint", "hande_right_finger_joint"],
        open_command_expr={
            "hande_left_finger_joint": 0.025,
            "hande_right_finger_joint": 0.025,
        },
        close_command_expr={
            "hande_left_finger_joint": 0.0,
            "hande_right_finger_joint": 0.0,
        },
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
        
        # Geometric Target
        rel_ee_drawer_distance = ObsTerm(
            func=custom_mdp.rel_ee_drawer_distance,
            noise=GaussianNoiseCfg(mean=0.0, std=0.005) 
        )

        # Previous Action
        actions = ObsTerm(func=standard_mdp.last_action)

        def __post_init__(self) -> None:
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events (Domain Randomizations)."""

    reset_robot_base = EventTerm(
        func=standard_mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["joint_x", "joint_y", "joint_th"]),
            "position_range": (0.0, 0.0),
            "velocity_range": (0.0, 0.0),
        },
    )

    # Domain randomization: Arm Joints
    reset_robot_arm = EventTerm(
        func=standard_mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["joint_[1-7]"]),
            "position_range": (-0.5, 0.5), 
            "velocity_range": (0.0, 0.0),
        },
    )
    
    reset_cabinet_root_position = EventTerm(
        func=standard_mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("cabinet"),  # Targets the root of the cabinet asset
            "pose_range": {
                "x": (-0.15, 0.05),  # +/- 5 cm forward/backward from x=1.3m
                "y": (-0.1, 0.1),  # +/- 5 cm left/right from y=0.0m
                "z": (0.0, 0.0),     # Keep height fixed at z=0.39m
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (-0.16, 0.16), # Optional: Slight rotation (+/- ~4.5 degrees)
            },
            "velocity_range": {
                # Ensure the cabinet isn't moving when spawned
                "x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0),
                "roll": (0.0, 0.0), "pitch": (0.0, 0.0), "yaw": (0.0, 0.0),
            },
        },
    )

    reset_cabinet_position = EventTerm(
        func=standard_mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"]),
            "position_range": (0.0, 0.0), 
            "velocity_range": (0.0, 0.0),
        },
    )

    # Domain randomization: Drawer track friction
    randomize_cabinet_friction = EventTerm(
        func=standard_mdp.randomize_joint_parameters,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"]),
            "friction_distribution_params": (0.1, 1.5), # Scales friction by 0.5x to 1.5x
            "operation": "scale",
            "distribution": "uniform",
        },
    )


@configclass
class RewardsCfg:
    """Reward terms mathematically balanced for standard RL paradigms."""
    
    approach = RewTerm(
        func=custom_mdp.approach_ee_handle,
        weight=1.0, 
    )

    alignment = RewTerm(
        func=custom_mdp.align_ee_handle,
        weight=2.0,
    )

    grasp = RewTerm(
        func=custom_mdp.grasp_handle,
        weight=2.0,
        params={
            "robot_cfg": SceneEntityCfg("robot"),
            "gripper_joint_name": "hande_left_finger_joint", # Explicitly reference the Hand-E joint
        },
    )
    
    drawer_progress = RewTerm(
        func=custom_mdp.open_drawer_bonus,
        weight=4.0,
        params={"cabinet_cfg": SceneEntityCfg("cabinet")},
    )
    
    action_rate = RewTerm(
        func=custom_mdp.action_rate_penalty,
        weight=0.1,
    )

    hold_open = RewTerm(
        func=custom_mdp.hold_open_bonus,
        weight=25.0, # 25 points EVERY timestep it holds the drawer open
        params={
            "threshold": 0.25, 
            "cabinet_cfg": SceneEntityCfg("cabinet")
        },
    )

@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=standard_mdp.time_out, time_out=True)
    
    # success = DoneTerm(
    #     func=custom_mdp.drawer_opened,
    #     params={
    #         "threshold": 0.25,
    #         "asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"]),
    #     },
    # )

##
# Environment configuration
##

@configclass
class TidybotHandeIsaacEnvCfg(ManagerBasedRLEnvCfg):
    scene: TidybotHandeIsaacSceneCfg = TidybotHandeIsaacSceneCfg(num_envs=4096, env_spacing=4.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self) -> None:
        self.decimation = 4
        self.episode_length_s = 25
        self.viewer.eye = (3.0, 3.0, 2.0)
        self.viewer.lookat = (0.0, 0.0, 0.0)
        self.sim.dt = 1 / 240
        self.sim.substeps = 1
        self.sim.physx.solver_iterations = 8
        self.sim.physx.num_position_iterations = 8
        self.action_delay_step = 1