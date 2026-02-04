# Copyright (c) 2022-2026, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def drawer_opened(
    env: ManagerBasedRLEnv,
    threshold: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("cabinet"),
) -> torch.Tensor:
    """Terminate when the drawer is opened beyond a threshold.
    
    Returns:
        A boolean tensor indicating if the termination condition is met.
    """
    # Extract the used system (cabinet)
    asset: Articulation = env.scene[asset_cfg.name]
    
    # Get joint position
    # The joint index is found from the asset_cfg joint_names
    joint_ids = asset.find_joints(asset_cfg.joint_names)[0]
    joint_pos = asset.data.joint_pos[:, joint_ids]
    
    # Check if joint position > threshold (Open)
    # Drawer joint usually goes 0 -> limit
    return (joint_pos > threshold).squeeze(-1)
