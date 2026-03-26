# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

gym.register(
    id="go2_stone",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.go2_stone:Go2EnvCfg",
        "rsl_rl_cfg_entry_point": "xiaomibei.tasks.locomotion.xiaomibei.agents.rsl_rl_ppo_cfg:PPORunnerCfg",
    },
)