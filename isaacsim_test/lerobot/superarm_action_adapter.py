"""Map SO101 leader actions into the six-control SuperArm policy contract."""

from __future__ import annotations

import math
from typing import Any

try:
    from isaacsim_test.isaacsim.graspable_hand_urdf import resolve_fixed_hand_motion
except ModuleNotFoundError:
    from graspable_hand_urdf import resolve_fixed_hand_motion


CANONICAL_MOTION_FEATURE = "amazinghand_motion.pos"


def map_so101_action_to_superarm(
    action: dict[str, float],
    *,
    arm_mapping: list[dict[str, Any]],
    arm_limits: dict[str, dict[str, float]],
    gripper_feature: str = "gripper.pos",
    previous_motion_code: float | None = None,
    motion_hysteresis: float = 0.05,
) -> dict[str, float]:
    """Convert SO101 degrees plus 0..100 gripper into five radians plus a motion code."""
    if len(arm_mapping) != 5:
        raise ValueError(f"SO101 arm mapping must contain exactly five entries, got {len(arm_mapping)}")

    mapped: dict[str, float] = {}
    for item in arm_mapping:
        source = str(item["source"])
        target = str(item["target"])
        if source not in action:
            raise ValueError(f"SO101 action is missing required feature {source!r}")
        degrees = float(action[source])
        if not math.isfinite(degrees):
            raise ValueError(f"SO101 feature {source!r} must be finite")
        radians = float(item.get("sign", 1.0)) * math.radians(degrees) + float(
            item.get("offset_rad", 0.0)
        )
        limit = arm_limits.get(target.removesuffix(".pos"))
        if limit:
            radians = max(float(limit["min"]), min(float(limit["max"]), radians))
        mapped[target] = radians

    if len(mapped) != 5:
        raise ValueError("SO101 mapping targets must be five unique canonical arm features")
    if gripper_feature not in action:
        raise ValueError(f"SO101 action is missing required feature {gripper_feature!r}")
    gripper = float(action[gripper_feature])
    if not math.isfinite(gripper):
        raise ValueError(f"SO101 feature {gripper_feature!r} must be finite")
    motion = resolve_fixed_hand_motion(
        gripper / 100.0,
        previous_code=previous_motion_code,
        hysteresis=motion_hysteresis,
    )
    mapped[CANONICAL_MOTION_FEATURE] = float(motion["code"])
    return mapped


class SO101ToSuperArmActionAdapter:
    """Stateful mapper that preserves hand-motion hysteresis across leader frames."""

    def __init__(
        self,
        *,
        arm_mapping: list[dict[str, Any]],
        arm_limits: dict[str, dict[str, float]],
        gripper_feature: str = "gripper.pos",
        motion_hysteresis: float = 0.05,
    ) -> None:
        self.arm_mapping = arm_mapping
        self.arm_limits = arm_limits
        self.gripper_feature = gripper_feature
        self.motion_hysteresis = motion_hysteresis
        self.previous_motion_code: float | None = None

    def __call__(self, action: dict[str, float]) -> dict[str, float]:
        mapped = map_so101_action_to_superarm(
            action,
            arm_mapping=self.arm_mapping,
            arm_limits=self.arm_limits,
            gripper_feature=self.gripper_feature,
            previous_motion_code=self.previous_motion_code,
            motion_hysteresis=self.motion_hysteresis,
        )
        self.previous_motion_code = mapped[CANONICAL_MOTION_FEATURE]
        return mapped
