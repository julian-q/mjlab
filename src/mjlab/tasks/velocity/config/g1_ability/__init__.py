import gymnasium as gym

gym.register(
  id="Mjlab-Velocity-Rough-Unitree-G1Ability",
  entry_point="mjlab.envs:ManagerBasedRlEnv",
  disable_env_checker=True,
  kwargs={
    "env_cfg_entry_point": f"{__name__}.rough_env_cfg:UnitreeG1AbilityRoughEnvCfg",
    "rl_cfg_entry_point": f"{__name__}.rl_cfg:UnitreeG1PPORunnerCfg",
  },
)

gym.register(
  id="Mjlab-Velocity-Rough-Unitree-G1Ability-Play",
  entry_point="mjlab.envs:ManagerBasedRlEnv",
  disable_env_checker=True,
  kwargs={
    "env_cfg_entry_point": f"{__name__}.rough_env_cfg:UnitreeG1AbilityRoughEnvCfg_PLAY",
    "rl_cfg_entry_point": f"{__name__}.rl_cfg:UnitreeG1PPORunnerCfg",
  },
)

gym.register(
  id="Mjlab-Velocity-Flat-Unitree-G1Ability",
  entry_point="mjlab.envs:ManagerBasedRlEnv",
  disable_env_checker=True,
  kwargs={
    "env_cfg_entry_point": f"{__name__}.flat_env_cfg:UnitreeG1AbilityFlatEnvCfg",
    "rl_cfg_entry_point": f"{__name__}.rl_cfg:UnitreeG1PPORunnerCfg",
  },
)

gym.register(
  id="Mjlab-Velocity-Flat-Unitree-G1Ability-Play",
  entry_point="mjlab.envs:ManagerBasedRlEnv",
  disable_env_checker=True,
  kwargs={
    "env_cfg_entry_point": f"{__name__}.flat_env_cfg:UnitreeG1AbilityFlatEnvCfg_PLAY",
    "rl_cfg_entry_point": f"{__name__}.rl_cfg:UnitreeG1PPORunnerCfg",
  },
)
