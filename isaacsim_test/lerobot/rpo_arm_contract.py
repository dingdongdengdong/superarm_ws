from __future__ import annotations

import math
from collections.abc import Iterable

ARM_JOINT_NAMES = [
    "right_arm_pitch_joint",
    "right_arm_roll_joint",
    "right_arm_yaw_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
]
AMAZINGHAND_MOTOR_JOINT_NAMES = [
    "finger1_motor1",
    "finger1_motor2",
    "finger2_motor1",
    "finger2_motor2",
    "finger3_motor1",
    "finger3_motor2",
    "finger4_motor1",
    "finger4_motor2",
]
GRASP_JOINT_NAME = "amazinghand_grasp"
JOINT_NAMES = [*ARM_JOINT_NAMES, GRASP_JOINT_NAME]
FEATURE_KEYS = [f"{name}.pos" for name in JOINT_NAMES]

ARM_JOINT_LIMITS: dict[str, tuple[float, float]] = {
    "right_arm_pitch_joint": (-1.57, 1.57),
    "right_arm_roll_joint": (-1.0, 0.25),
    "right_arm_yaw_joint": (-1.57, 1.57),
    "right_elbow_pitch_joint": (-0.6, 1.57),
    "right_elbow_yaw_joint": (-1.57, 1.57),
}
GRASP_LIMIT = (0.0, 1.0)
DEFAULT_MIDDLE_POS_DEG = [0.0] * 8
DEFAULT_OPEN_DEG = [-35.0, 35.0, -35.0, 35.0, -35.0, 35.0, -35.0, 35.0]
DEFAULT_CLOSE_DEG = [90.0, -90.0, 90.0, -90.0, 90.0, -90.0, 90.0, -90.0]


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))


def normalize_action(values: Iterable[float], *, joint_names: Iterable[str] = JOINT_NAMES) -> list[float]:
    names = list(joint_names)
    raw = [float(value) for value in values]
    if len(raw) < len(names):
        raw.extend([0.0] * (len(names) - len(raw)))
    raw = raw[: len(names)]

    normalized: list[float] = []
    for name, value in zip(names, raw, strict=True):
        if name == GRASP_JOINT_NAME:
            normalized.append(clamp(value, *GRASP_LIMIT))
        elif name in ARM_JOINT_LIMITS:
            normalized.append(clamp(value, *ARM_JOINT_LIMITS[name]))
        else:
            normalized.append(float(value))
    return normalized


def grasp_scalar_to_servo_targets(
    grasp: float,
    *,
    middle_pos_deg: Iterable[float] | None = None,
    servo_ids: Iterable[int] = range(1, 9),
) -> dict[int, float]:
    g = clamp(grasp, *GRASP_LIMIT)
    middle = list(DEFAULT_MIDDLE_POS_DEG if middle_pos_deg is None else middle_pos_deg)
    if len(middle) != 8:
        raise ValueError(f"middle_pos_deg must contain 8 values, got {len(middle)}")

    ids = list(servo_ids)
    if len(ids) != 8:
        raise ValueError(f"servo_ids must contain 8 values, got {len(ids)}")

    targets: dict[int, float] = {}
    for servo_id, open_deg, close_deg, offset_deg in zip(
        ids,
        DEFAULT_OPEN_DEG,
        DEFAULT_CLOSE_DEG,
        middle,
        strict=True,
    ):
        target_deg = ((1.0 - g) * open_deg) + (g * close_deg) + float(offset_deg)
        targets[int(servo_id)] = math.radians(target_deg)
    return targets
