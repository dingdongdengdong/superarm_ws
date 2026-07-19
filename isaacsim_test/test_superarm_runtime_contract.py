from __future__ import annotations

import math

import pytest

from isaacsim_test.isaacsim.graspable_hand_urdf import HAND_ACTUATED_JOINT_NAMES
from isaacsim_test.isaacsim.superarm_runtime_contract import (
    CANONICAL_ARM_JOINT_NAMES,
    LOGICAL_HAND_MOTION_NAME,
    infer_logical_state,
    resolve_logical_command,
)


def test_resolve_six_control_command_expands_fixed_motion_to_thirteen_joints() -> None:
    resolution = resolve_logical_command([0.1, -0.2, 0.3, -0.4, 0.5, 0.77])

    assert resolution.logical_joint_names == [
        *CANONICAL_ARM_JOINT_NAMES,
        LOGICAL_HAND_MOTION_NAME,
    ]
    assert resolution.resolved_logical_command == [0.1, -0.2, 0.3, -0.4, 0.5, 1.0]
    assert list(resolution.physical_targets) == [
        *CANONICAL_ARM_JOINT_NAMES,
        *HAND_ACTUATED_JOINT_NAMES,
    ]
    assert resolution.physical_targets["finger1_motor1"] == 0.95
    assert resolution.physical_targets["finger1_motor2"] == 1.1
    assert resolution.motion_name == "close"


@pytest.mark.parametrize(
    ("command", "match"),
    [
        ([0.0] * 5, "exactly 6"),
        ([0.0] * 7, "exactly 6"),
        ([0.0, 0.0, math.nan, 0.0, 0.0, 0.0], "finite"),
        ([0.0, 0.0, 0.0, 0.0, 4.0, 0.0], "outside"),
    ],
)
def test_resolve_rejects_invalid_policy_commands(command: list[float], match: str) -> None:
    with pytest.raises(ValueError, match=match):
        resolve_logical_command(command)


def test_resolve_preserves_hand_motion_hysteresis() -> None:
    resolution = resolve_logical_command(
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.72],
        previous_motion_code=0.5,
    )

    assert resolution.resolved_logical_command[-1] == 0.5
    assert resolution.motion_name == "half_close"


def test_infer_logical_state_keeps_policy_state_six_dimensional() -> None:
    resolution = resolve_logical_command([0.1, -0.2, 0.3, -0.4, 0.5, 0.51])

    logical = infer_logical_state(resolution.physical_targets)

    assert logical == [0.1, -0.2, 0.3, -0.4, 0.5, 0.5]
    assert len(logical) == 6
