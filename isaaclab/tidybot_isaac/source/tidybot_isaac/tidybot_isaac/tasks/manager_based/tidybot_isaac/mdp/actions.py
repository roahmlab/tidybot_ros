from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.envs.mdp import JointPositionAction, JointPositionActionCfg
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

class MimicGripperAction(JointPositionAction):
    """Action term that controls a leader joint and mimics others."""

    cfg: MimicGripperActionCfg

    def __init__(self, cfg: MimicGripperActionCfg, env: ManagerBasedRLEnv):
        # We want to control ALL gripper joints (leader + mimics) in the asset,
        # but only receive 1 action dimension from the agent.
        
        super().__init__(cfg, env)
        
        # Store mimic logic
        self.leader_name = cfg.leader_joint_name
        self.mimics = cfg.mimic_multiplier
        
        self._leader_idx = -1
        self._mimic_indices = {} # {idx: multiplier}
        
        # Iterate over the configured joint ids
        for i, joint_id in enumerate(self._joint_ids):
            jid = int(joint_id)
            name = self._asset.joint_names[jid]
            if name == self.leader_name:
                self._leader_idx = i
            elif name in self.mimics:
                self._mimic_indices[i] = self.mimics[name]
        
        if self._leader_idx == -1:
            raise ValueError(f"Leader joint {self.leader_name} not found in configured joint_names.")

    @property
    def action_dim(self) -> int:
        """The agent only controls the leader joint (1 Dimension)."""
        return 1

    def process_actions(self, actions: torch.Tensor):
        """Process the single-dim action into multi-joint targets."""
        # Get open/close values for leader
        open_val = self.cfg.open_command_expr[self.leader_name]
        close_val = self.cfg.close_command_expr[self.leader_name]
        
        # Clip actions to [0, 1]
        clamped_actions = torch.clamp(actions, 0.0, 1.0)
        
        # Interpolate to get leader target position
        leader_target = open_val + (close_val - open_val) * clamped_actions
        
        # Create full command tensor for all controlled joints
        num_envs = actions.shape[0]
        num_joints = len(self._joint_ids)
        
        # Store in our own buffer (not parent's _raw_actions which has wrong size)
        self._processed_actions = torch.zeros((num_envs, num_joints), device=actions.device)
        
        # Fill leader slot
        self._processed_actions[:, self._leader_idx] = leader_target[:, 0]
        
        # Fill mimic slots: pos_mimic = multiplier * pos_leader
        for idx, mult in self._mimic_indices.items():
            self._processed_actions[:, idx] = leader_target[:, 0] * mult

    def apply_actions(self):
        """Apply the processed joint positions to the asset."""
        self._asset.set_joint_position_target(self._processed_actions, joint_ids=self._joint_ids)


@configclass
class MimicGripperActionCfg(JointPositionActionCfg):
    """Configuration for the mimic gripper action term."""
    class_type = MimicGripperAction
    
    # Configuration for mimic joints: {joint_name: multiplier}
    mimic_multiplier: dict[str, float] = {
        "right_outer_knuckle_joint": 1.0,
        "left_inner_finger_joint": 1.0,
        "right_inner_finger_joint": 1.0,
        "left_inner_finger_knuckle_joint": 1.0,
        "right_inner_finger_knuckle_joint": -1.0,
    }
    # The leader joint name
    leader_joint_name: str = "finger_joint"
    # The joint name to read the observation from
    open_command_expr: dict[str, float] = {"finger_joint": 0.0}
    close_command_expr: dict[str, float] = {"finger_joint": 0.8}
