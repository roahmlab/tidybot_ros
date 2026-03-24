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
from isaaclab.envs.mdp.actions.binary_joint_actions import BinaryJointPositionAction
from isaaclab.envs.mdp.actions.actions_cfg import BinaryJointPositionActionCfg
# Custom MDP functions
from . import mdp as custom_mdp
from tidybot_isaac import assets

##
# Scene definition
##

FRAME_MARKER_SMALL_CFG = FRAME_MARKER_CFG.copy()
FRAME_MARKER_SMALL_CFG.markers["frame"].scale = (0.10, 0.10, 0.10)
FORCE_MARKER_CFG = VisualizationMarkersCfg(
    prim_path="/Visuals/RadialForceArrow",
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
    hinge_frame = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Door/Base",
        visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(prim_path="/Visuals/HingeFrameTransformer"),
        debug_vis=True,
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Door/HingeOrigin", # The specific frame to track
                name="hinge_origin"
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

class CooldownBinaryAction(BinaryJointPositionAction):
    def __init__(self, cfg: "CooldownBinaryActionCfg", env):
        super().__init__(cfg, env)
        self.cooldown_steps = int(cfg.cooldown_time / env.step_dt)
        self.cooldown_timers = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        self.committed_actions = torch.ones((env.num_envs, self.action_dim), dtype=torch.float, device=env.device)

    def process_actions(self, actions: torch.Tensor):
        desired_actions = torch.sign(actions)
        
        # Determine who is allowed to change their gripper state
        can_change = self.cooldown_timers <= 0
        wants_to_change = (desired_actions != self.committed_actions).any(dim=-1)
        
        # Envs that are legally flipping state this step
        flipping_envs = can_change & wants_to_change
        
        # Update committed actions for those allowed to change
        self.committed_actions = torch.where(
            flipping_envs.unsqueeze(1), 
            desired_actions, 
            self.committed_actions
        )
        
        # Update timers (Reset to max if flipping, otherwise subtract 1)
        self.cooldown_timers = torch.where(
            flipping_envs,
            torch.tensor(self.cooldown_steps, device=self.device),
            torch.clamp(self.cooldown_timers - 1, min=0)
        )

        # Pass the filtered actions to the original Isaac Lab handler
        super().process_actions(self.committed_actions)

    def reset(self, env_ids: torch.Tensor | None = None) -> None:
        super().reset(env_ids)
        if env_ids is None:
            self.cooldown_timers.fill_(0)
            self.committed_actions.fill_(1.0) # Assume Open
        else:
            self.cooldown_timers[env_ids] = 0
            self.committed_actions[env_ids] = 1.0
            
@configclass
class CooldownBinaryActionCfg(BinaryJointPositionActionCfg):
    """Configuration for a binary action with a hardware lockout timer."""
    class_type: type = CooldownBinaryAction
    cooldown_time: float = 0.8  # Lockout time in seconds
    
            
@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    arm_pos = standard_mdp.RelativeJointPositionActionCfg(
        asset_name="robot",
        joint_names=["joint_[1-7]"],
        scale=0.005, 
    )

    gripper = CooldownBinaryActionCfg(
        asset_name="robot",
        joint_names=["hande_left_finger_joint", "hande_right_finger_joint"],
        cooldown_time=0.8,
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
        gripper_cooldown = ObsTerm(
            func=custom_mdp.gripper_cooldown_state,
            params={"action_term_name": "gripper"}
        )

        # Geometric Task State
        rel_ee_handle_transform = ObsTerm(
            func=custom_mdp.rel_ee_handle_transform,
            noise=GaussianNoiseCfg(mean=0.0, std=0.005),
        )
        ee_to_hinge_vector = ObsTerm(
            func=custom_mdp.ee_to_hinge_in_ee_frame,
            noise=GaussianNoiseCfg(mean=0.0, std=0.002),
            params={"hinge_cfg": SceneEntityCfg("door", body_names="HingeOrigin")}
        )
        hinge_axis = ObsTerm(
            func=custom_mdp.hinge_axis_in_ee_frame,
            noise=GaussianNoiseCfg(mean=0.0, std=0.002),
            params={"door_cfg": SceneEntityCfg("door", body_names="HingeOrigin")} 
        )
        door_pos = ObsTerm(
            func=custom_mdp.door_position,
            noise=GaussianNoiseCfg(mean=0.0, std=0.002),
            params={"asset_cfg": SceneEntityCfg("door", joint_names=["HingeJoint"])}
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
        gripper_cooldown = ObsTerm(
            func=custom_mdp.gripper_cooldown_state,
            params={"action_term_name": "gripper"}
        )


        # Clean Geometric Task State
        rel_ee_handle_transform_clean = ObsTerm(
            func=custom_mdp.rel_ee_handle_transform,
        )
        ee_to_hinge_vector = ObsTerm(
            func=custom_mdp.ee_to_hinge_in_ee_frame,
            params={"hinge_cfg": SceneEntityCfg("door", body_names="HingeOrigin")}
        )
        hinge_axis = ObsTerm(
            func=custom_mdp.hinge_axis_in_ee_frame,
            params={"door_cfg": SceneEntityCfg("door", body_names="HingeOrigin")} 
        )
        door_pos = ObsTerm(
            func=custom_mdp.door_position,
            params={"asset_cfg": SceneEntityCfg("door", joint_names=["HingeJoint"])}
        )

        # Privileged Data: Exact door state
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
    env: "ManagerBasedRLEnv", 
    env_ids: torch.Tensor, 
    asset_cfg: "SceneEntityCfg", 
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    z_range: tuple[float, float],
    yaw_range: tuple[float, float],
    allowed_orientations: tuple[str, ...] = ("right", "left", "bottom", "top"),
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

    # --- Choose Door Configuration (Roll around X) ---
    quat_mapping = {
        "right": [1.0, 0.0, 0.0, 0.0],                # 0 deg
        "left": [0.0, 1.0, 0.0, 0.0],                 # 180 deg
        "bottom": [0.7071068, -0.7071068, 0.0, 0.0],  # -90 deg
        "top": [0.7071068, 0.7071068, 0.0, 0.0]       # +90 deg
    }
    
    selected_quats = [quat_mapping[k.lower()] for k in allowed_orientations if k.lower() in quat_mapping]
    if not selected_quats:
        raise ValueError(f"No valid orientations provided in {allowed_orientations}. Must be from {list(quat_mapping.keys())}")
        
    base_quats = torch.tensor(selected_quats, device=env.device)
    
    # Randomly select 1 of the allowed states per environment
    choices = torch.randint(0, len(base_quats), (num_resets,), device=env.device)
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
    
    reset_gripper = EventTerm(
        func=standard_mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg(
                "robot", 
                joint_names=["hande_left_finger_joint", "hande_right_finger_joint"]
            ),
            "position_range": (0.025, 0.025), 
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
            "allowed_orientations": ["right", "left", "bottom", "top"],
        },
    )

    randomize_door_mass = EventTerm(
        func=standard_mdp.randomize_rigid_body_mass,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("door", body_names="DoorPanel"), 
            "mass_distribution_params": (0.1, 2),
            "operation": "scale",
            "distribution": "uniform",
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
            "damping_distribution_params": (0.1, 2.0),
            "operation": "scale",
            "distribution": "uniform",
        },
    )

    randomize_door_stiffness = EventTerm(
        func=standard_mdp.randomize_actuator_gains,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("door", joint_names=["HingeJoint"]),
            "stiffness_distribution_params": (0.0, 10.0),
            "operation": "add", 
            "distribution": "uniform",
        },
    )

@configclass
class RewardsCfg:
    """Reward terms scaled down to stabilize the PPO Value Network."""
    
    approach = RewTerm(
        func=custom_mdp.approach_ee_handle,
        weight=0.2,   # Was 2.0
    )
    approach.log = True

    alignment = RewTerm(
        func=custom_mdp.align_ee_handle,
        weight=0.5,   # Was 5.0
    )

    grasp = RewTerm(
        func=custom_mdp.grasp_handle,
        weight=1.0,   # Was 3.0
        params={
            "robot_cfg": SceneEntityCfg("robot"),
            "gripper_joint_name": "hande_left_finger_joint", 
        },
    )

    door_progress_gated = RewTerm(
        func=custom_mdp.gated_open_door,
        weight=2.0,   # Was 20.0
        params={
            "threshold": 1.25,
            "door_cfg": SceneEntityCfg("door"),
            "robot_cfg": SceneEntityCfg("robot"),
            "gripper_joint_name": "hande_left_finger_joint"
        },
    )

    hold_open = RewTerm(
        func=custom_mdp.hold_open_bonus,
        weight=1.0,   # Was 10.0
        params={
            "threshold": 1.25, 
            "door_cfg": SceneEntityCfg("door"),
            "robot_cfg": SceneEntityCfg("robot"),
            "gripper_joint_name": "hande_left_finger_joint",
        },
    )

    action_rate = RewTerm(
        func=custom_mdp.action_rate_penalty,
        weight=0.001, # Was 0.005
    )

    action_l2 = RewTerm(
        func=custom_mdp.action_l2_penalty,
        weight=0.001,
    )

    gripper_chatter = RewTerm(
        func=custom_mdp.gripper_chatter_penalty,
        weight=0.005  # Was 0.05
    )

    illegal_gripper_flip = RewTerm(
        func=custom_mdp.illegal_gripper_command_penalty,
        weight=0.0025, 
        params={"action_term_name": "gripper"},
    )

    smooth_pull = RewTerm(
        func=custom_mdp.ee_velocity_penalty,
        params={
            "robot_cfg": SceneEntityCfg("robot"),
        },
        weight=0.2,  # Was 0.5
    )

    unaligned_approach = RewTerm(
        func=custom_mdp.unaligned_approach_penalty,
        weight=0.1, # Was 0.01
    )

    rest_at_goal = RewTerm(
        func=custom_mdp.rest_at_goal_penalty,
        weight=0.0005, # Was 0.005
        params={
            "threshold": 1.25, 
            "door_cfg": SceneEntityCfg("door"),
            "robot_cfg": SceneEntityCfg("robot"),
        },
    )

    track_energy_used = RewTerm(
        func=custom_mdp.log_cumulative_mechanical_work,
        weight=1e-10, # Keeps it purely diagnostic
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

    track_squared_torque_effort = RewTerm(
        func=custom_mdp.log_squared_torque_effort,
        weight=1e-10,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot", joint_names=["joint_[1-7]"]
            )
        }
    )

    track_success_time = RewTerm(
        func=custom_mdp.track_and_log_time_to_success,
        weight=1e-10,
        params={
            "threshold": 1.25, 
            "door_cfg": SceneEntityCfg("door", joint_names=["HingeJoint"]),
            "robot_cfg": SceneEntityCfg("robot"),
            "gripper_joint_name": "hande_left_finger_joint",
        }
    )
    
@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""
    time_out = DoneTerm(func=standard_mdp.time_out, time_out=True)
    success = DoneTerm(
        func=custom_mdp.success_at_timeout,
        time_out=True,
        params={
            "threshold": 1.25, 
            "door_cfg": SceneEntityCfg("door", joint_names=["HingeJoint"]),
            "robot_cfg": SceneEntityCfg("robot"),
            "gripper_joint_name": "hande_left_finger_joint",
        }
    )

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
        self.episode_length_s = 10
        self.viewer.eye = (3.0, 6.0, 3.5)
        self.viewer.lookat = (-100.0, -200.0, -125.0)
        self.sim.dt = 1 / 120
        self.sim.substeps = 1
        self.sim.physx.solver_iterations = 8
        self.sim.physx.num_position_iterations = 4
        self.action_delay_step = 1