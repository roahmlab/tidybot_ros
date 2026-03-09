import math
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg, ManagerBasedRLEnv
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
    """Configuration for the TidyBot door opening scene using the Hand-E Gripper."""

    ground: AssetBaseCfg = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(300.0, 300.0)),
    )

    # Robot Asset (Hand-E)
    robot: ArticulationCfg = assets.TIDYBOT_HANDE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    ee_frame = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Robot/tidybot/bracelet_link",
        visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(prim_path="/Visuals/EEFrameTransformer"),
        # debug_vis=True,
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Robot/tidybot/bracelet_link",
                name="ee_tcp",
                offset=OffsetCfg(
                    pos=(0.0, 0.0, -0.2015),
                    rot=(0.0, 1.0, 0.0, 0.0),
                ),
            ),
        ],
    )

    replicate_physics: bool = False # Allows randomly selected door assets
    door: ArticulationCfg = assets.DOOR_CFG.replace(prim_path="{ENV_REGEX_NS}/Door")
    
    handle_frame = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Door/Base", 
        visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(prim_path="/Visuals/HandleFrameTransformer"),
        # debug_vis=True,
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Door/Handle",
                name="handle"
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
        scale=0.01, 
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

def frame_transformer_pose_w(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """The pose (position and orientation) of a frame transformer's target in world frame."""
    source_entity = env.scene[asset_cfg.name]
    if asset_cfg.body_ids is None:
        raise ValueError(f"Could not resolve body_ids for '{asset_cfg.name}'. Check that 'ee_tcp' exists in your SceneCfg!")
        
    target_idx = asset_cfg.body_ids[0]
    pos = source_entity.data.target_pos_w[:, target_idx]
    quat = source_entity.data.target_quat_w[:, target_idx]
    
    return torch.cat([pos, quat], dim=-1)

@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    # ==========================================
    # ACTOR OBSERVATIONS
    # ==========================================
    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""
        # Robot Proprioception
        joint_pos = ObsTerm(
            func=standard_mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["joint_[1-7]"])}
        )
        
        joint_vel = ObsTerm(
            func=standard_mdp.joint_vel_rel,
            noise=GaussianNoiseCfg(mean=0.0, std=0.015), 
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["joint_[1-7]"])}
        )
        gripper_pos = ObsTerm(
            func=custom_mdp.gripper_open_amount,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "gripper_joint_name": "hande_left_finger_joint",
                "open_pos": 0.025,  
                "close_pos": 0.0,  
            },
        )
        
        # Explicit End-Effector Pose
        ee_position = ObsTerm(
            func=frame_transformer_pose_w,
            params={"asset_cfg": SceneEntityCfg("ee_frame", body_names="ee_tcp")}
        )

        # Geometric Task State
        rel_ee_handle_transform = ObsTerm(
            func=custom_mdp.rel_ee_handle_transform,
            noise=GaussianNoiseCfg(mean=0.0, std=0.01), # 1cm of vision noise
        )
        handle_initial_pos = ObsTerm(
            func=custom_mdp.handle_initial_position,
            noise=GaussianNoiseCfg(mean=0.0, std=0.01),
            params={"door_cfg": SceneEntityCfg("door", body_names="Handle")}
        )
        hinge_origin = ObsTerm(
            func=custom_mdp.hinge_origin_position,
            noise=GaussianNoiseCfg(mean=0.0, std=0.01),
            params={"door_cfg": SceneEntityCfg("door", body_names="Base")}
        )
        actions = ObsTerm(func=standard_mdp.last_action)

        def __post_init__(self) -> None:
            self.enable_corruption = True
            self.concatenate_terms = True

    # ==========================================
    # CRITIC OBSERVATIONS
    # ==========================================
    @configclass
    class CriticCfg(ObsGroup):
        """Privileged observations for the critic group."""
        
        joint_pos = ObsTerm(
            func=standard_mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["joint_[1-7]"])}
        )
        # Clean Proprioception (No noise)
        joint_vel_clean = ObsTerm(
            func=standard_mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["joint_[1-7]"])}
        )
        gripper_pos = ObsTerm(
            func=custom_mdp.gripper_open_amount,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "gripper_joint_name": "hande_left_finger_joint",
                "open_pos": 0.025,  
                "close_pos": 0.0,  
            },
        )
        
        ee_position = ObsTerm(
            func=frame_transformer_pose_w,
            params={"asset_cfg": SceneEntityCfg("ee_frame", body_names="ee_tcp")}
        )

        # Clean Geometric Task State
        rel_ee_handle_transform_clean = ObsTerm(
            func=custom_mdp.rel_ee_handle_transform,
        )
        handle_initial_pos_clean = ObsTerm(
            func=custom_mdp.handle_initial_position,
            params={"door_cfg": SceneEntityCfg("door", body_names="Handle")}
        )
        hinge_origin_clean = ObsTerm(
            func=custom_mdp.hinge_origin_position,
            params={"door_cfg": SceneEntityCfg("door", body_names="Base")}
        )

        # Privileged Data: Exact door state
        door_joint_pos = ObsTerm(
            func=standard_mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("door", joint_names=["HingeJoint"])}
        )
        door_joint_vel = ObsTerm(
            func=standard_mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("door", joint_names=["HingeJoint"])}
        )

        actions = ObsTerm(func=standard_mdp.last_action)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()

def reset_door_state(
    env: ManagerBasedRLEnv, 
    env_ids: torch.Tensor, 
    asset_cfg: SceneEntityCfg, 
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    z_range: tuple[float, float],
    yaw_range: tuple[float, float],
):
    door = env.scene[asset_cfg.name]
    num_resets = len(env_ids)

    # Grab default LOCAL state
    default_root_state = door.data.default_root_state[env_ids]
    default_pos_local = default_root_state[:, 0:3]

    # Generate the LOCAL random positional noise
    noise_x = torch.empty(num_resets, device=env.device).uniform_(*x_range)
    noise_y = torch.empty(num_resets, device=env.device).uniform_(*y_range)
    noise_z = torch.empty(num_resets, device=env.device).uniform_(*z_range)

    # Apply noise to create final LOCAL positions
    local_positions = default_pos_local.clone()
    local_positions[:, 0] += noise_x
    local_positions[:, 1] += noise_y
    local_positions[:, 2] += noise_z
    world_positions = local_positions + env.scene.env_origins[env_ids]

    # --- Choose  Door Configuration (Roll around X) ---
    # 0: Hinge Right  -> 0 deg
    # 1: Hinge Left   -> 180 deg
    # 2: Hinge Bottom -> -90 deg
    # 3: Hinge Top    -> +90 deg
    base_quats = torch.tensor([
        [1.0, 0.0, 0.0, 0.0],                
        [0.0, 1.0, 0.0, 0.0],                
        [0.7071068, -0.7071068, 0.0, 0.0],   
        [0.7071068, 0.7071068, 0.0, 0.0]     
    ], device=env.device)
    
    # Randomly select 1 of the 4 states per environment
    choices = torch.randint(0, 4, (num_resets,), device=env.device)
    state_quats = base_quats[choices]

    # --- Apply Random Yaw (Rotation around World Z) ---
    yaw_noise = torch.empty(num_resets, device=env.device).uniform_(*yaw_range)
    yaw_quats = torch.zeros((num_resets, 4), device=env.device)
    yaw_quats[:, 0] = torch.cos(yaw_noise / 2.0)  # w
    yaw_quats[:, 3] = torch.sin(yaw_noise / 2.0)  # z

    # Multiply Yaw Quat * State Quat (Vectorized Hamilton Product)
    w1, x1, y1, z1 = yaw_quats[:, 0], yaw_quats[:, 1], yaw_quats[:, 2], yaw_quats[:, 3]
    w2, x2, y2, z2 = state_quats[:, 0], state_quats[:, 1], state_quats[:, 2], state_quats[:, 3]
    
    final_quat = torch.stack([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dim=-1)
    
    final_quat = torch.nn.functional.normalize(final_quat, p=2.0, dim=-1)

    # --- Apply to the Simulator ---
    root_pose = torch.cat([world_positions, final_quat], dim=-1)
    door.write_root_pose_to_sim(root_pose, env_ids=env_ids)
    
    # Ensure the door is fully closed and stationary
    joint_pos = torch.zeros((num_resets, door.num_joints), device=env.device)
    joint_vel = torch.zeros((num_resets, door.num_joints), device=env.device)
    door.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)

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
    
    reset_door = EventTerm(
        func=reset_door_state,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("door"), 
            "x_range": (-0.1, 0.0),
            "y_range": (-0.1, 0.1), 
            "z_range": (-0.15, 0.10),
            "yaw_range": (-0.085, 0.085),
        },
    )

    # Randomize Coulomb Friction
    randomize_door_friction = EventTerm(
        func=standard_mdp.randomize_joint_parameters,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("door", joint_names=["HingeJoint"]),
            "friction_distribution_params": (0.1, 2.0), 
            "operation": "scale",
            "distribution": "uniform",
        },
    )

    # Randomize Actuator Damping
    randomize_door_damping = EventTerm(
        func=standard_mdp.randomize_actuator_gains,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("door", joint_names=["HingeJoint"]),
            "damping_distribution_params": (0.5, 2.0),
            "operation": "scale",
            "distribution": "uniform",
        },
    )

@configclass
class RewardsCfg:
    """Reward terms mathematically balanced for standard RL paradigms."""
    
    approach = RewTerm(
        func=custom_mdp.approach_ee_handle,
        weight=2.0, 
    )

    alignment = RewTerm(
        func=custom_mdp.align_ee_handle,
        weight=5.0,
    )

    grasp = RewTerm(
        func=custom_mdp.grasp_handle,
        weight=3.0,
        params={
            "robot_cfg": SceneEntityCfg("robot"),
            "gripper_joint_name": "hande_left_finger_joint", # Explicitly reference the Hand-E joint
        },
    )
    
    door_progress_ungated = RewTerm(
        func=custom_mdp.ungated_open_door,
        weight=0.03,
        params={
            "threshold": 1.25,
            "door_cfg": SceneEntityCfg("door"),
        },
    )

    door_progress_gated = RewTerm(
        func=custom_mdp.gated_open_door,
        weight=20.0,
        params={
            "threshold": 1.25,
            "door_cfg": SceneEntityCfg("door"),
            "robot_cfg": SceneEntityCfg("robot"),
            "gripper_joint_name": "hande_left_finger_joint"
        },
    )

    hold_open = RewTerm(
        func=custom_mdp.hold_open_bonus,
        weight=10.0, 
        params={
            "threshold": 1.25, 
            "door_cfg": SceneEntityCfg("door"),
            "robot_cfg": SceneEntityCfg("robot"),
            "gripper_joint_name": "hande_left_finger_joint",
        },
    )

    action_rate = RewTerm(
        func=custom_mdp.action_rate_penalty,
        weight=0.005, 
    )
    
    gripper_chatter = RewTerm(
        func=custom_mdp.gripper_chatter_penalty,
        weight=0.01
    )

    smooth_pull = RewTerm(
        func=custom_mdp.ee_velocity_penalty,
        params={
            "robot_cfg": SceneEntityCfg("robot"),
        },
        weight=0.5, 
    )

    unaligned_approach = RewTerm(
        func=custom_mdp.unaligned_approach_penalty,
        weight=0.01, 
    )

    rest_at_goal = RewTerm(
        func=custom_mdp.rest_at_goal_penalty,
        weight=0.001, 
        params={
            "threshold": 1.25, 
            "door_cfg": SceneEntityCfg("door"),
            "robot_cfg": SceneEntityCfg("robot"),
        },
    )

@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""
    time_out = DoneTerm(func=standard_mdp.time_out, time_out=True)

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
        self.decimation = 2
        self.episode_length_s = 8
        self.viewer.eye = (3.0, 6.0, 3.5)
        self.viewer.lookat = (-100.0, -200.0, -125.0)
        self.sim.dt = 1 / 120
        self.sim.substeps = 1
        self.sim.physx.solver_iterations = 8
        self.sim.physx.num_position_iterations = 4
        self.action_delay_step = 1