#!/usr/bin/env python3
"""No-motion DM4340P HITL safety backend primitives.

This module is intentionally read-only. It models the Gate 2 hardware boundary
before any torque-enable, disable/enable, or motion command is allowed.
"""

from __future__ import annotations

import json
from enum import Enum
from pathlib import Path
from typing import Any, Iterable


class SafetyState(Enum):
    DISCONNECTED = "disconnected"
    READ_ONLY = "read_only"
    BLOCKED = "blocked"
    FAULT = "fault"


class MotorConfig:
    def __init__(
        self,
        *,
        label: str,
        joint_name: str,
        send_id: int | None,
        status_id: int | None,
        mode: str,
        confirmed: bool,
    ) -> None:
        self.label = label
        self.joint_name = joint_name
        self.send_id = send_id
        self.status_id = status_id
        self.mode = mode
        self.confirmed = confirmed


class ReadOnlyConfig:
    def __init__(self, *, channel: str, bitrate: int, motion_disabled: bool, motors: list[MotorConfig]) -> None:
        self.channel = channel
        self.bitrate = bitrate
        self.motion_disabled = motion_disabled
        self.motors = motors
        self.block_reasons = _block_reasons(motors, motion_disabled)
        self.can_complete_gate2 = not self.block_reasons

    @property
    def expected_status_ids(self) -> list[int]:
        return sorted(m.status_id for m in self.motors if m.confirmed and m.status_id is not None)

    def to_dict(self) -> dict[str, Any]:
        return {
            "channel": self.channel,
            "bitrate": self.bitrate,
            "motion_disabled": self.motion_disabled,
            "can_complete_gate2": self.can_complete_gate2,
            "block_reasons": self.block_reasons,
            "expected_status_ids": self.expected_status_ids,
            "motors": [
                {
                    "label": m.label,
                    "joint_name": m.joint_name,
                    "send_id": m.send_id,
                    "status_id": m.status_id,
                    "mode": m.mode,
                    "confirmed": m.confirmed,
                }
                for m in self.motors
            ],
        }


class IdComparison:
    def __init__(
        self,
        *,
        gate_state: str,
        expected_status_ids: list[int],
        observed_ids: list[int],
        unexpected_ids: list[int],
        missing_expected_ids: list[int],
        block_reasons: list[str],
    ) -> None:
        self.gate_state = gate_state
        self.expected_status_ids = expected_status_ids
        self.observed_ids = observed_ids
        self.unexpected_ids = unexpected_ids
        self.missing_expected_ids = missing_expected_ids
        self.block_reasons = block_reasons

    def to_dict(self) -> dict[str, Any]:
        return {
            "gate_state": self.gate_state,
            "expected_status_ids": self.expected_status_ids,
            "observed_ids": self.observed_ids,
            "unexpected_ids": self.unexpected_ids,
            "missing_expected_ids": self.missing_expected_ids,
            "block_reasons": self.block_reasons,
        }


def _parse_optional_id(raw: Any, field: str, index: int) -> int | None:
    if raw is None:
        return None
    if isinstance(raw, int):
        return raw
    raise ValueError(f"motor[{index}].{field} must be an integer or null")


def _load_motor(raw: dict[str, Any], index: int) -> MotorConfig:
    required = ["label", "joint_name", "send_id", "status_id", "mode", "confirmed"]
    missing = [key for key in required if key not in raw]
    if missing:
        raise ValueError(f"motor[{index}] missing required fields: {', '.join(missing)}")
    if not isinstance(raw["confirmed"], bool):
        raise ValueError(f"motor[{index}].confirmed must be boolean")
    return MotorConfig(
        label=str(raw["label"]),
        joint_name=str(raw["joint_name"]),
        send_id=_parse_optional_id(raw["send_id"], "send_id", index),
        status_id=_parse_optional_id(raw["status_id"], "status_id", index),
        mode=str(raw["mode"]),
        confirmed=raw["confirmed"],
    )


def _find_duplicates(values: Iterable[int]) -> list[int]:
    seen: set[int] = set()
    duplicates: set[int] = set()
    for value in values:
        if value in seen:
            duplicates.add(value)
        seen.add(value)
    return sorted(duplicates)


def _block_reasons(motors: list[MotorConfig], motion_disabled: bool) -> list[str]:
    reasons: list[str] = []
    if not motion_disabled:
        reasons.append("motion_disabled must be true for Gate 2 read-only backend")
    unconfirmed = [m.label for m in motors if not m.confirmed]
    if unconfirmed:
        reasons.append(f"unconfirmed motor IDs: {', '.join(unconfirmed)}")
    missing_status = [m.label for m in motors if m.confirmed and m.status_id is None]
    if missing_status:
        reasons.append(f"confirmed motors missing status_id: {', '.join(missing_status)}")
    return reasons


def load_read_only_config(path: str | Path) -> ReadOnlyConfig:
    payload = json.loads(Path(path).read_text())
    motors_raw = payload.get("motors")
    if not isinstance(motors_raw, list) or len(motors_raw) != 2:
        raise ValueError("read-only config must define exactly two motors")
    motors = [_load_motor(raw, i) for i, raw in enumerate(motors_raw)]

    confirmed_send_ids = [m.send_id for m in motors if m.confirmed and m.send_id is not None]
    confirmed_status_ids = [m.status_id for m in motors if m.confirmed and m.status_id is not None]
    duplicate_send_ids = _find_duplicates(confirmed_send_ids)
    duplicate_status_ids = _find_duplicates(confirmed_status_ids)
    if duplicate_send_ids:
        raise ValueError(f"duplicate send_id values: {duplicate_send_ids}")
    if duplicate_status_ids:
        raise ValueError(f"duplicate status_id values: {duplicate_status_ids}")

    return ReadOnlyConfig(
        channel=str(payload.get("channel", "/dev/ttyACM0")),
        bitrate=int(payload.get("bitrate", 1_000_000)),
        motion_disabled=bool(payload.get("motion_disabled", False)),
        motors=motors,
    )


def compare_observed_ids(config: ReadOnlyConfig, observed_ids: Iterable[int]) -> IdComparison:
    observed = sorted({int(value) for value in observed_ids})
    expected = config.expected_status_ids
    unexpected = sorted(set(observed) - set(expected)) if expected else []
    missing = sorted(set(expected) - set(observed)) if observed else expected.copy()
    reasons = list(config.block_reasons)
    if unexpected:
        reasons.append(f"unexpected observed status IDs: {unexpected}")
    if missing:
        reasons.append(f"missing expected status IDs: {missing}")
    gate_state = "pass" if not reasons and expected and observed else "blocked"
    return IdComparison(
        gate_state=gate_state,
        expected_status_ids=expected,
        observed_ids=observed,
        unexpected_ids=unexpected,
        missing_expected_ids=missing,
        block_reasons=reasons,
    )


class ReadOnlySafetyBackend:
    transmits_can_frames = False
    motor_enable_allowed = False
    motion_command_allowed = False

    def __init__(self, config: ReadOnlyConfig) -> None:
        self.config = config
        self.state = SafetyState.DISCONNECTED

    def connect_read_only(self) -> None:
        self.state = SafetyState.READ_ONLY

    def _blocked(self) -> None:
        raise PermissionError("blocked by read-only HITL gate")

    def send_frame(self, *_args: Any, **_kwargs: Any) -> None:
        self._blocked()

    def enable_motor(self, *_args: Any, **_kwargs: Any) -> None:
        self._blocked()

    def disable_motor(self, *_args: Any, **_kwargs: Any) -> None:
        self._blocked()

    def command_motion(self, *_args: Any, **_kwargs: Any) -> None:
        self._blocked()
