# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

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

from xiaomibei.tasks.locomotion.xiaomibei import mdp

##
# Pre-defined configs
##

from xiaomibei.assets.go2.unitree import UNITREE_GO2_CFG

##
# Scene definition
##

STONE_TRACK_WIDTH = 1.0
STONE_TRACK_SIDE_LINE_WIDTH = 0.15
STONE_BLOCK_LENGTH = 0.30
STONE_BLOCK_HEIGHT = 0.05
STONE_BLOCK_GAP = 0.20
STONE_BLOCK_COUNT = 4
STONE_TRACK_LEAD_IN = 1.0
STONE_TRACK_LEAD_OUT = 1.0
STONE_SPAWN_X = 0.50
STONE_SPAWN_Y_RANGE = 0.10


# Stones are laid out along +X and span the 1.0 m track width along Y.
def _stone_center_x(index: int) -> float:
    return STONE_TRACK_LEAD_IN + STONE_BLOCK_LENGTH * 0.5 + index * (STONE_BLOCK_LENGTH + STONE_BLOCK_GAP)


STONE_TRACK_TOTAL_LENGTH = (
    STONE_TRACK_LEAD_IN
    + STONE_BLOCK_COUNT * STONE_BLOCK_LENGTH
    + (STONE_BLOCK_COUNT - 1) * STONE_BLOCK_GAP
    + STONE_TRACK_LEAD_OUT
)
STONE_TRACK_CENTER_X = STONE_TRACK_TOTAL_LENGTH * 0.5
SIDE_LINE_Y = STONE_TRACK_WIDTH * 0.5 + STONE_TRACK_SIDE_LINE_WIDTH * 0.5
STONE_PHYSICS_MATERIAL = sim_utils.RigidBodyMaterialCfg(static_friction=1.2, dynamic_friction=1.0, restitution=0.0)


@configclass
class Go2SceneCfg(InteractiveSceneCfg):
    """Configuration for a simple Go2 stone-track scene."""

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    lane_line_left = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/LaneLineLeft",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(STONE_TRACK_CENTER_X, SIDE_LINE_Y, 0.001)),
        spawn=sim_utils.CuboidCfg(
            size=(STONE_TRACK_TOTAL_LENGTH, STONE_TRACK_SIDE_LINE_WIDTH, 0.002),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 0.0), roughness=1.0),
        ),
    )

    lane_line_right = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/LaneLineRight",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(STONE_TRACK_CENTER_X, -SIDE_LINE_Y, 0.001)),
        spawn=sim_utils.CuboidCfg(
            size=(STONE_TRACK_TOTAL_LENGTH, STONE_TRACK_SIDE_LINE_WIDTH, 0.002),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 0.0), roughness=1.0),
        ),
    )

    stone_1 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Stone1",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(_stone_center_x(0), 0.0, STONE_BLOCK_HEIGHT * 0.5)),
        spawn=sim_utils.CuboidCfg(
            size=(STONE_BLOCK_LENGTH, STONE_TRACK_WIDTH, STONE_BLOCK_HEIGHT),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            physics_material=STONE_PHYSICS_MATERIAL,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.55, 0.55, 0.55), roughness=0.95),
        ),
    )

    stone_2 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Stone2",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(_stone_center_x(1), 0.0, STONE_BLOCK_HEIGHT * 0.5)),
        spawn=sim_utils.CuboidCfg(
            size=(STONE_BLOCK_LENGTH, STONE_TRACK_WIDTH, STONE_BLOCK_HEIGHT),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            physics_material=STONE_PHYSICS_MATERIAL,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.55, 0.55, 0.55), roughness=0.95),
        ),
    )

    stone_3 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Stone3",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(_stone_center_x(2), 0.0, STONE_BLOCK_HEIGHT * 0.5)),
        spawn=sim_utils.CuboidCfg(
            size=(STONE_BLOCK_LENGTH, STONE_TRACK_WIDTH, STONE_BLOCK_HEIGHT),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            physics_material=STONE_PHYSICS_MATERIAL,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.55, 0.55, 0.55), roughness=0.95),
        ),
    )

    stone_4 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Stone4",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(_stone_center_x(3), 0.0, STONE_BLOCK_HEIGHT * 0.5)),
        spawn=sim_utils.CuboidCfg(
            size=(STONE_BLOCK_LENGTH, STONE_TRACK_WIDTH, STONE_BLOCK_HEIGHT),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            physics_material=STONE_PHYSICS_MATERIAL,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.55, 0.55, 0.55), roughness=0.95),
        ),
    )

    # robot
    robot: ArticulationCfg = UNITREE_GO2_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=UNITREE_GO2_CFG.init_state.replace(pos=(STONE_SPAWN_X, 0.0, 0.4)),
    )

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )


##
# MDP settings
##

##
# MDP settings (用于纯测试的极简配置)
##

@configclass
class ActionsCfg:
    """极简动作配置：使用正则表达式 ".*" 匹配 Go2 的所有 12 个关节"""
    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=1.0, use_default_offset=True)

@configclass
class ObservationsCfg:
    """极简观察配置：只获取基座线速度，防止维度报错"""
    @configclass
    class PolicyCfg(ObsGroup):
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True
    policy: PolicyCfg = PolicyCfg()

@configclass
class EventCfg:
    """极简事件配置：重置时随机一下狗的位置和朝向"""
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            # 默认出生点位于第一块石板前 0.5m，重置时随机到石板前约 0.3-0.8m。
            "pose_range": {"x": (-0.2, 0.3), "y": (-STONE_SPAWN_Y_RANGE, STONE_SPAWN_Y_RANGE), "yaw": (-0.2, 0.2)},
            "velocity_range": {},
        },
    )

@configclass
class RewardsCfg:
    """极简奖励配置：活着就行"""
    alive = RewTerm(func=mdp.is_alive, weight=1.0)

@configclass
class TerminationsCfg:
    """极简终止配置：时间到了就重置"""
    time_out = DoneTerm(func=mdp.time_out, time_out=True)


##
# Environment configuration
##


@configclass
class Go2EnvCfg(ManagerBasedRLEnvCfg):
    # Scene settings
    scene: Go2SceneCfg = Go2SceneCfg(num_envs=4096, env_spacing=4.0)
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
        self.viewer.eye = (8.0, 0.0, 5.0)
        # simulation settings
        self.sim.dt = 1 / 120
        self.sim.render_interval = self.decimation