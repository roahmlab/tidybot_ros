import math

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from . import mdp
from tidybot_isaac import assets

##
# Scene definition
##


@configclass
class TidybotIsaacSceneCfg(InteractiveSceneCfg):
    """Configuration for the TidyBot drawer opening scene."""

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    # robot
    robot: ArticulationCfg = assets.TIDYBOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # cabinet
    cabinet: ArticulationCfg = assets.CABINET_CFG.replace(prim_path="{ENV_REGEX_NS}/Cabinet")

    # lights
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
    gripper = mdp.MimicGripperActionCfg(
        asset_name="robot",
        joint_names=["finger_joint", "right_outer_knuckle_joint"],
        leader_joint_name="finger_joint",
        mimic_multiplier={
            "right_outer_knuckle_joint": 1.0,
        },
        open_command_expr={"finger_joint": 0.0},
        close_command_expr={"finger_joint": 0.82},
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        
        # Drawer state
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
    """Reward terms for the MDP."""

    # (1) Constant running reward
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    
    # (2) Penalize large joint velocities (action smoothness)
    # joint_vel_l2 = RewTerm(func=mdp.joint_vel_l2, weight=-0.001)

    # (3) Reward opening the drawer
    # We want the drawer_top_joint to move to a positive position
    drawer_opened = RewTerm(
        func=mdp.joint_pos_target_l2, # This punishes distance to target
        weight=1.0, # Positive weight means we want to minimize distance? No. 
                    # joint_pos_target_l2 returns -error^2. So maximize this.
        params={
            "asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"]), 
            "target": 0.5 # Target open position
        },
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # (1) Time out
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    
    # (2) Success?
    # Could define success if drawer positions > threshold


##
# Environment configuration
##


@configclass
class TidybotIsaacEnvCfg(ManagerBasedRLEnvCfg):
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
        self.episode_length_s = 5
        # viewer settings
        self.viewer.eye = (3.0, 3.0, 2.0)
        self.viewer.lookat = (0.0, 0.0, 0.0)
        # simulation settings
        self.sim.dt = 1 / 120
        self.sim.render_interval = self.decimation