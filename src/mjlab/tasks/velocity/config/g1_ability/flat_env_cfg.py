from dataclasses import dataclass

from mjlab.asset_zoo.robots.unitree_g1.g1_constants import G1_ACTION_SCALE
from mjlab.tasks.velocity.config.g1_ability.rough_env_cfg import (
  UnitreeG1AbilityRoughEnvCfg,
)


@dataclass
class UnitreeG1AbilityFlatEnvCfg(UnitreeG1AbilityRoughEnvCfg):
  def __post_init__(self):
    super().__post_init__()

    assert self.scene.terrain is not None
    self.scene.terrain.terrain_type = "plane"
    self.scene.terrain.terrain_generator = None
    self.curriculum.terrain_levels = None

    # Exclude hand actuators from action space (only control body/legs/arms)
    controlled_joint_patterns = [
      ".*_hip_.*",           # Hip joints
      ".*_knee_.*",          # Knee joints
      ".*_ankle_.*",         # Ankle joints
      "waist_.*",            # Waist joints
      ".*_shoulder_.*",      # Shoulder joints
      ".*_elbow_.*",         # Elbow joints
      ".*_wrist_.*",         # Wrist joints (but not hand)
    ]
    self.actions.joint_pos.actuator_names = controlled_joint_patterns
    # Use G1 action scale (without hand joints) instead of G1_ABILITY_ACTION_SCALE
    self.actions.joint_pos.scale = G1_ACTION_SCALE

    # Exclude hand joints from pose reward (only penalize deviation for controlled joints)
    from mjlab.managers.scene_entity_config import SceneEntityCfg
    self.rewards.pose.params["asset_cfg"] = SceneEntityCfg(
      "robot",
      joint_names=controlled_joint_patterns
    )


@dataclass
class UnitreeG1AbilityFlatEnvCfg_PLAY(UnitreeG1AbilityFlatEnvCfg):
  def __post_init__(self):
    super().__post_init__()

    # Effectively infinite episode length.
    self.episode_length_s = int(1e9)

    self.observations.policy.enable_corruption = False
    self.events.push_robot = None

    self.commands.twist.ranges.lin_vel_x = (-1.5, 2.0)
    self.commands.twist.ranges.ang_vel_z = (-0.7, 0.7)
