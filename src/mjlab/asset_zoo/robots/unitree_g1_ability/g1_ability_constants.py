"""Unitree G1 with Ability Hands constants."""

from pathlib import Path

import mujoco

from mjlab import MJLAB_SRC_PATH
from mjlab.asset_zoo.robots.unitree_g1.g1_constants import (
  ACTUATOR_4010,
  ACTUATOR_5020,
  ACTUATOR_7520_14,
  ACTUATOR_7520_22,
  DAMPING_4010,
  DAMPING_5020,
  DAMPING_7520_14,
  DAMPING_7520_22,
  HOME_KEYFRAME,
  KNEES_BENT_KEYFRAME,
  STIFFNESS_4010,
  STIFFNESS_5020,
  STIFFNESS_7520_14,
  STIFFNESS_7520_22,
)
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import ActuatorCfg, CollisionCfg

##
# MJCF and assets.
##

G1_ABILITY_XML: Path = (
  MJLAB_SRC_PATH / "asset_zoo" / "robots" / "unitree_g1_ability" / "xmls" / "g1_ability.xml"
)
assert G1_ABILITY_XML.exists()


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, G1_ABILITY_XML.parent / "assets", meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(G1_ABILITY_XML))
  spec.assets = get_assets(spec.meshdir)
  return spec


##
# G1 Body Actuator config (reused from g1_constants).
##

G1_ACTUATOR_5020 = ActuatorCfg(
  joint_names_expr=[
    ".*_elbow_joint",
    ".*_shoulder_pitch_joint",
    ".*_shoulder_roll_joint",
    ".*_shoulder_yaw_joint",
    ".*_wrist_roll_joint",
  ],
  effort_limit=ACTUATOR_5020.effort_limit,
  armature=ACTUATOR_5020.reflected_inertia,
  stiffness=STIFFNESS_5020,
  damping=DAMPING_5020,
)
G1_ACTUATOR_7520_14 = ActuatorCfg(
  joint_names_expr=[".*_hip_pitch_joint", ".*_hip_yaw_joint", "waist_yaw_joint"],
  effort_limit=ACTUATOR_7520_14.effort_limit,
  armature=ACTUATOR_7520_14.reflected_inertia,
  stiffness=STIFFNESS_7520_14,
  damping=DAMPING_7520_14,
)
G1_ACTUATOR_7520_22 = ActuatorCfg(
  joint_names_expr=[".*_hip_roll_joint", ".*_knee_joint"],
  effort_limit=ACTUATOR_7520_22.effort_limit,
  armature=ACTUATOR_7520_22.reflected_inertia,
  stiffness=STIFFNESS_7520_22,
  damping=DAMPING_7520_22,
)
G1_ACTUATOR_4010 = ActuatorCfg(
  joint_names_expr=[".*_wrist_pitch_joint", ".*_wrist_yaw_joint"],
  effort_limit=ACTUATOR_4010.effort_limit,
  armature=ACTUATOR_4010.reflected_inertia,
  stiffness=STIFFNESS_4010,
  damping=DAMPING_4010,
)

# Waist pitch/roll and ankles are 4-bar linkages with 2 5020 actuators.
G1_ACTUATOR_WAIST = ActuatorCfg(
  joint_names_expr=["waist_pitch_joint", "waist_roll_joint"],
  effort_limit=ACTUATOR_5020.effort_limit * 2,
  armature=ACTUATOR_5020.reflected_inertia * 2,
  stiffness=STIFFNESS_5020 * 2,
  damping=DAMPING_5020 * 2,
)
G1_ACTUATOR_ANKLE = ActuatorCfg(
  joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
  effort_limit=ACTUATOR_5020.effort_limit * 2,
  armature=ACTUATOR_5020.reflected_inertia * 2,
  stiffness=STIFFNESS_5020 * 2,
  damping=DAMPING_5020 * 2,
)

##
# Ability Hand Actuator config.
##

# Hand parameters from ability-hand-api XML defaults:
# - joint damping: 0.01
# - joint frictionloss: 0.1
# - position kp: 1, kv: 0
#
# We use very low stiffness (kp=1) to allow compliant grasping.
# The hands are designed for dexterous manipulation with compliance.

HAND_ARMATURE = 0.001  # Low inertia for finger joints
HAND_STIFFNESS = 1.0   # Low stiffness for compliant grasping (from XML kp=1)
HAND_DAMPING = 0.01    # From XML joint damping

# Finger MCP (metacarpophalangeal) joints - Base of fingers
ABILITY_HAND_MCP = ActuatorCfg(
  joint_names_expr=[
    "index_mcp",
    "middle_mcp",
    "ring_mcp",
    "pinky_mcp",
    "l_index_mcp",
    "l_middle_mcp",
    "l_ring_mcp",
    "l_pinky_mcp",
  ],
  effort_limit=2.0,  # Reasonable torque for finger joints
  armature=HAND_ARMATURE,
  stiffness=HAND_STIFFNESS,
  damping=HAND_DAMPING,
)

# Finger PIP (proximal interphalangeal) joints - Middle of fingers
ABILITY_HAND_PIP = ActuatorCfg(
  joint_names_expr=[
    "index_pip",
    "middle_pip",
    "ring_pip",
    "pinky_pip",
    "l_index_pip",
    "l_middle_pip",
    "l_ring_pip",
    "l_pinky_pip",
  ],
  effort_limit=2.0,  # Reasonable torque for finger joints
  armature=HAND_ARMATURE,
  stiffness=HAND_STIFFNESS,
  damping=HAND_DAMPING,
)

# Thumb MCP (flexion) joints
ABILITY_HAND_THUMB_MCP = ActuatorCfg(
  joint_names_expr=[
    "thumb_mcp",
    "l_thumb_mcp",
  ],
  effort_limit=2.0,  # Reasonable torque for thumb
  armature=HAND_ARMATURE,
  stiffness=HAND_STIFFNESS,
  damping=HAND_DAMPING,
)

# Thumb CMC (carpometacarpal - rotation) joints
ABILITY_HAND_THUMB_CMC = ActuatorCfg(
  joint_names_expr=[
    "thumb_cmc",
    "l_thumb_cmc",
  ],
  effort_limit=2.0,  # Reasonable torque for thumb rotation
  armature=HAND_ARMATURE,
  stiffness=HAND_STIFFNESS,
  damping=HAND_DAMPING,
)

##
# Collision config.
##

# Extend G1's collision config to include hand collisions.
# Hand collision geoms use mesh-based collision from ability-hand-api.
FULL_COLLISION_WITH_HANDS = CollisionCfg(
  geom_names_expr=[".*_collision", ".*_coll"],
  condim={r"^(left|right)_foot[1-7]_collision$": 3, ".*": 1},
  priority={r"^(left|right)_foot[1-7]_collision$": 1},
  friction={r"^(left|right)_foot[1-7]_collision$": (0.6,)},
)

FEET_ONLY_COLLISION = CollisionCfg(
  geom_names_expr=[r"^(left|right)_foot[1-7]_collision$"],
  contype=0,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.6,),
)

##
# Keyframe config.
##

# Extend home keyframe with hand positions (all fingers extended)
HOME_KEYFRAME_WITH_HANDS = EntityCfg.InitialStateCfg(
  pos=HOME_KEYFRAME.pos,
  joint_pos={
    **HOME_KEYFRAME.joint_pos,
    # Right hand - fingers slightly curled
    "index_mcp": 0.0,
    "index_pip": 0.766,
    "middle_mcp": 0.0,
    "middle_pip": 0.766,
    "ring_mcp": 0.0,
    "ring_pip": 0.766,
    "pinky_mcp": 0.0,
    "pinky_pip": 0.766,
    "thumb_cmc": -1.0,
    "thumb_mcp": 0.5,
    # Left hand - fingers slightly curled
    "l_index_mcp": 0.0,
    "l_index_pip": 0.766,
    "l_middle_mcp": 0.0,
    "l_middle_pip": 0.766,
    "l_ring_mcp": 0.0,
    "l_ring_pip": 0.766,
    "l_pinky_mcp": 0.0,
    "l_pinky_pip": 0.766,
    "l_thumb_cmc": -1.0,
    "l_thumb_mcp": 0.5,
  },
  joint_vel=HOME_KEYFRAME.joint_vel,
)

KNEES_BENT_KEYFRAME_WITH_HANDS = EntityCfg.InitialStateCfg(
  pos=KNEES_BENT_KEYFRAME.pos,
  joint_pos={
    **KNEES_BENT_KEYFRAME.joint_pos,
    # Right hand - fingers slightly curled
    "index_mcp": 0.0,
    "index_pip": 0.766,
    "middle_mcp": 0.0,
    "middle_pip": 0.766,
    "ring_mcp": 0.0,
    "ring_pip": 0.766,
    "pinky_mcp": 0.0,
    "pinky_pip": 0.766,
    "thumb_cmc": -1.0,
    "thumb_mcp": 0.5,
    # Left hand - fingers slightly curled
    "l_index_mcp": 0.0,
    "l_index_pip": 0.766,
    "l_middle_mcp": 0.0,
    "l_middle_pip": 0.766,
    "l_ring_mcp": 0.0,
    "l_ring_pip": 0.766,
    "l_pinky_mcp": 0.0,
    "l_pinky_pip": 0.766,
    "l_thumb_cmc": -1.0,
    "l_thumb_mcp": 0.5,
  },
  joint_vel=KNEES_BENT_KEYFRAME.joint_vel,
)

##
# Final config.
##

G1_ABILITY_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    # G1 body actuators
    G1_ACTUATOR_5020,
    G1_ACTUATOR_7520_14,
    G1_ACTUATOR_7520_22,
    G1_ACTUATOR_4010,
    G1_ACTUATOR_WAIST,
    G1_ACTUATOR_ANKLE,
    # Ability Hand actuators
    ABILITY_HAND_MCP,
    ABILITY_HAND_PIP,
    ABILITY_HAND_THUMB_MCP,
    ABILITY_HAND_THUMB_CMC,
  ),
  soft_joint_pos_limit_factor=0.9,
)

G1_ABILITY_ROBOT_CFG = EntityCfg(
  init_state=KNEES_BENT_KEYFRAME_WITH_HANDS,
  collisions=(FULL_COLLISION_WITH_HANDS,),
  spec_fn=get_spec,
  articulation=G1_ABILITY_ARTICULATION,
)

G1_ABILITY_ACTION_SCALE: dict[str, float] = {}
for a in G1_ABILITY_ARTICULATION.actuators:
  e = a.effort_limit
  s = a.stiffness
  names = a.joint_names_expr
  if not isinstance(e, dict):
    e = {n: e for n in names}
  if not isinstance(s, dict):
    s = {n: s for n in names}
  for n in names:
    if n in e and n in s and s[n]:
      G1_ABILITY_ACTION_SCALE[n] = 0.25 * e[n] / s[n]

if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(G1_ABILITY_ROBOT_CFG)
  model = robot.spec.compile()

  print("\n=== G1 with Ability Hands ===")
  print(f"Total DOF: {model.nv}")
  print(f"Total actuators: {model.nu}")
  print("\nActuator names:")
  for i in range(model.nu):
    print(f"  - {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)}")

  viewer.launch(model)
