"""Explicit arm/hand joint-name subset (54 axes); replay adds two named head axes."""

LEFT_ARM_JOINT_NAMES = (
    "proximal_pitch_L_Joint",
    "proximal_roll_L_Joint",
    "proximal_yaw_L_Joint",
    "elbow_L_Joint",
    "wrist_yaw_L_Joint",
    "wrist_pitch_L_Joint",
    "wrist_roll_L_Joint",
)

RIGHT_ARM_JOINT_NAMES = tuple(name.replace("_L_", "_R_") for name in LEFT_ARM_JOINT_NAMES)

_HAND_SUFFIXES = (
    "thumb_cmc_flex",
    "thumb_cmc_abd",
    "thumb_mcp",
    "thumb_ip",
    "index_finger_mcp_flex",
    "index_finger_mcp_abd",
    "index_finger_pip",
    "index_finger_dip",
    "middle_finger_mcp_flex",
    "middle_finger_mcp_abd",
    "middle_finger_pip",
    "middle_finger_dip",
    "ring_finger_mcp_flex",
    "ring_finger_mcp_abd",
    "ring_finger_pip",
    "ring_finger_dip",
    "pinky_mcp_flex",
    "pinky_mcp_abd",
    "pinky_pip",
    "pinky_dip",
)

LEFT_HAND_JOINT_NAMES = tuple(f"l_{name}" for name in _HAND_SUFFIXES)
RIGHT_HAND_JOINT_NAMES = tuple(f"r_{name}" for name in _HAND_SUFFIXES)
ALL_JOINT_NAMES = (
    LEFT_ARM_JOINT_NAMES + LEFT_HAND_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES + RIGHT_HAND_JOINT_NAMES
)

assert len(LEFT_ARM_JOINT_NAMES) == len(RIGHT_ARM_JOINT_NAMES) == 7
assert len(LEFT_HAND_JOINT_NAMES) == len(RIGHT_HAND_JOINT_NAMES) == 20
assert len(ALL_JOINT_NAMES) == len(set(ALL_JOINT_NAMES)) == 54
