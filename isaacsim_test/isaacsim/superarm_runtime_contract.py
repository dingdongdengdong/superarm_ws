"""Six-control SuperArm policy contract and physical AmazingHand expansion."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence

try:
    from .graspable_hand_urdf import (
        HAND_ACTUATED_JOINT_NAMES,
        fixed_hand_motion_library,
        resolve_fixed_hand_motion,
    )
except ImportError:  # Isaac standalone scripts import from this directory directly.
    from graspable_hand_urdf import (  # type: ignore[no-redef]
        HAND_ACTUATED_JOINT_NAMES,
        fixed_hand_motion_library,
        resolve_fixed_hand_motion,
    )


CANONICAL_ARM_JOINT_NAMES = [f"joint_rev_{index}" for index in range(1, 6)]
LOGICAL_HAND_MOTION_NAME = "amazinghand_motion"
LOGICAL_JOINT_NAMES = [*CANONICAL_ARM_JOINT_NAMES, LOGICAL_HAND_MOTION_NAME]
PHYSICAL_JOINT_NAMES = [*CANONICAL_ARM_JOINT_NAMES, *HAND_ACTUATED_JOINT_NAMES]
DEFAULT_ARM_LIMITS = {name: (-math.pi, math.pi) for name in CANONICAL_ARM_JOINT_NAMES}


@dataclass(frozen=True)
class LogicalCommandResolution:
    requested_logical_command: list[float]
    resolved_logical_command: list[float]
    physical_targets: dict[str, float]
    logical_joint_names: list[str]
    physical_joint_names: list[str]
    motion_name: str


def resolve_logical_command(
    values: Sequence[float],
    *,
    previous_motion_code: float | None = None,
    motion_hysteresis: float = 0.05,
    arm_limits: Mapping[str, tuple[float, float]] | None = None,
) -> LogicalCommandResolution:
    """Validate a 6D policy command and expand its hand motion to 13 joints."""
    if len(values) != len(LOGICAL_JOINT_NAMES):
        raise ValueError(
            f"SuperArm logical command must contain exactly 6 values, got {len(values)}"
        )
    requested = [float(value) for value in values]
    if not all(math.isfinite(value) for value in requested):
        raise ValueError(f"SuperArm logical command values must be finite, got {requested}")

    limits = dict(DEFAULT_ARM_LIMITS if arm_limits is None else arm_limits)
    arm_values = requested[: len(CANONICAL_ARM_JOINT_NAMES)]
    for joint_name, value in zip(CANONICAL_ARM_JOINT_NAMES, arm_values, strict=True):
        lower, upper = limits[joint_name]
        if value < lower or value > upper:
            raise ValueError(
                f"{joint_name} command {value} is outside [{lower}, {upper}]"
            )

    motion = resolve_fixed_hand_motion(
        requested[-1],
        previous_code=previous_motion_code,
        hysteresis=motion_hysteresis,
    )
    resolved = [*arm_values, float(motion["code"])]
    physical_targets = dict(
        zip(CANONICAL_ARM_JOINT_NAMES, arm_values, strict=True)
    )
    physical_targets.update(
        {name: float(motion["joint_targets"][name]) for name in HAND_ACTUATED_JOINT_NAMES}
    )
    return LogicalCommandResolution(
        requested_logical_command=requested,
        resolved_logical_command=resolved,
        physical_targets=physical_targets,
        logical_joint_names=list(LOGICAL_JOINT_NAMES),
        physical_joint_names=list(PHYSICAL_JOINT_NAMES),
        motion_name=str(motion["name"]),
    )


def infer_logical_state(physical_positions: Mapping[str, float]) -> list[float]:
    """Collapse 13D articulation readback into the canonical 6D policy state."""
    missing = [name for name in PHYSICAL_JOINT_NAMES if name not in physical_positions]
    if missing:
        raise ValueError(f"Physical SuperArm state is missing joints: {missing}")
    values = {name: float(physical_positions[name]) for name in PHYSICAL_JOINT_NAMES}
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError("Physical SuperArm state values must be finite")

    motion = min(
        fixed_hand_motion_library(),
        key=lambda candidate: sum(
            (values[name] - float(candidate["joint_targets"][name])) ** 2
            for name in HAND_ACTUATED_JOINT_NAMES
        ),
    )
    return [
        *(values[name] for name in CANONICAL_ARM_JOINT_NAMES),
        float(motion["code"]),
    ]
