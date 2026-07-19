from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from isaacsim_test.lerobot.superarm_action_adapter import (
    CANONICAL_MOTION_FEATURE,
    SO101ToSuperArmActionAdapter,
    map_so101_action_to_superarm,
)


ROOT = Path(__file__).resolve().parents[1]
CONFIG = yaml.safe_load(
    (ROOT / "isaacsim_test/lerobot/source_arm_amazinghand.yaml").read_text(encoding="utf-8")
)


def _action(gripper: float = 0.0) -> dict[str, float]:
    return {
        "shoulder_pan.pos": 90.0,
        "shoulder_lift.pos": -45.0,
        "elbow_flex.pos": 30.0,
        "wrist_flex.pos": -15.0,
        "wrist_roll.pos": 10.0,
        "gripper.pos": gripper,
    }


def _map(action: dict[str, float], previous: float | None = None) -> dict[str, float]:
    return map_so101_action_to_superarm(
        action,
        arm_mapping=CONFIG["so101_leader_mapping"],
        arm_limits=CONFIG["arm_limits"],
        gripper_feature=CONFIG["so101_gripper_feature"],
        previous_motion_code=previous,
        motion_hysteresis=CONFIG["motion_hysteresis"],
    )


def test_so101_degrees_map_to_five_radian_features() -> None:
    mapped = _map(_action())

    assert list(mapped) == [
        "joint_rev_1.pos",
        "joint_rev_2.pos",
        "joint_rev_3.pos",
        "joint_rev_4.pos",
        "joint_rev_5.pos",
        CANONICAL_MOTION_FEATURE,
    ]
    assert mapped["joint_rev_1.pos"] == pytest.approx(math.radians(90.0))
    assert mapped["joint_rev_2.pos"] == pytest.approx(math.radians(-45.0))
    assert mapped["joint_rev_3.pos"] == pytest.approx(math.radians(30.0))


@pytest.mark.parametrize(
    ("gripper", "expected"),
    [(0.0, 0.0), (26.0, 0.5), (74.0, 0.5), (76.0, 1.0), (100.0, 1.0)],
)
def test_so101_gripper_selects_only_fixed_motion_codes(gripper: float, expected: float) -> None:
    assert _map(_action(gripper))[CANONICAL_MOTION_FEATURE] == expected


def test_stateful_adapter_applies_hysteresis() -> None:
    adapter = SO101ToSuperArmActionAdapter(
        arm_mapping=CONFIG["so101_leader_mapping"],
        arm_limits=CONFIG["arm_limits"],
        gripper_feature=CONFIG["so101_gripper_feature"],
        motion_hysteresis=CONFIG["motion_hysteresis"],
    )

    assert adapter(_action(0.0))[CANONICAL_MOTION_FEATURE] == 0.0
    assert adapter(_action(29.0))[CANONICAL_MOTION_FEATURE] == 0.0
    assert adapter(_action(31.0))[CANONICAL_MOTION_FEATURE] == 0.5
    assert adapter(_action(71.0))[CANONICAL_MOTION_FEATURE] == 0.5
    assert adapter(_action(81.0))[CANONICAL_MOTION_FEATURE] == 1.0


def test_mapping_clamps_arm_and_rejects_invalid_input() -> None:
    action = _action()
    action["shoulder_pan.pos"] = 360.0
    assert _map(action)["joint_rev_1.pos"] == CONFIG["arm_limits"]["joint_rev_1"]["max"]

    with pytest.raises(ValueError, match="missing required feature"):
        _map({"gripper.pos": 0.0})
    bad = _action()
    bad["gripper.pos"] = float("nan")
    with pytest.raises(ValueError, match="must be finite"):
        _map(bad)
