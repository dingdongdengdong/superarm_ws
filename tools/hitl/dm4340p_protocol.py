#!/usr/bin/env python3
"""Gated DM4340P protocol planning helpers for HITL milestones C-F.

This module can build documented system command frames, but it does not open CAN
or transmit frames. Gate runner scripts use these plans in dry-run mode unless a
future, explicit hardware safety gate changes that behavior.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from typing import Any

_SYSTEM_COMMAND_SUFFIX = {
    "enable": ("system_enable", 0xFC),
    "disable": ("system_disable", 0xFD),
    "set_zero": ("system_set_zero", 0xFE),
    "clear_error": ("system_clear_error", 0xFB),
}


def _load_backend_module():
    module_path = Path(__file__).with_name("read_only_backend.py")
    spec = importlib.util.spec_from_file_location("read_only_backend", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class ProtocolFrame:
    def __init__(self, *, arbitration_id: int, data: bytes, kind: str, transmits_if_executed: bool = True) -> None:
        self.arbitration_id = arbitration_id
        self.data = data
        self.kind = kind
        self.transmits_if_executed = transmits_if_executed

    @property
    def data_hex(self) -> str:
        return self.data.hex()

    def to_dict(self) -> dict[str, Any]:
        return {
            "arbitration_id": self.arbitration_id,
            "arbitration_id_hex": f"0x{self.arbitration_id:x}",
            "data_hex": self.data_hex,
            "kind": self.kind,
            "transmits_if_executed": self.transmits_if_executed,
        }


class GateConfig:
    def __init__(self, *, raw: Any, channel: str, bitrate: int, motors: list[Any], block_reasons: list[str]) -> None:
        self.raw = raw
        self.channel = channel
        self.bitrate = bitrate
        self.motors = motors
        self.block_reasons = block_reasons

    def motor_by_label(self, label: str) -> Any:
        for motor in self.motors:
            if motor.label == label:
                return motor
        labels = ", ".join(m.label for m in self.motors)
        raise ValueError(f"unknown motor label {label!r}; available labels: {labels}")


class GatePlan:
    def __init__(self, *, milestone: str, name: str, motor_label: str | None, execute_allowed: bool, transmitted: bool, block_reasons: list[str], frames: list[ProtocolFrame], relative_step_rad: float | None = None, checklist: list[str] | None = None) -> None:
        self.milestone = milestone
        self.name = name
        self.motor_label = motor_label
        self.execute_allowed = execute_allowed
        self.transmitted = transmitted
        self.block_reasons = block_reasons
        self.frames = frames
        self.relative_step_rad = relative_step_rad
        self.checklist = checklist

    def to_dict(self) -> dict[str, Any]:
        return {
            "milestone": self.milestone,
            "name": self.name,
            "motor_label": self.motor_label,
            "execute_allowed": self.execute_allowed,
            "transmitted": self.transmitted,
            "block_reasons": self.block_reasons,
            "relative_step_rad": self.relative_step_rad,
            "frames": [frame.to_dict() for frame in self.frames],
            "checklist": self.checklist or [],
        }


def build_system_frame(*, send_id: int, command: str) -> ProtocolFrame:
    if command not in _SYSTEM_COMMAND_SUFFIX:
        raise ValueError(f"unsupported system command: {command}")
    kind, suffix = _SYSTEM_COMMAND_SUFFIX[command]
    return ProtocolFrame(arbitration_id=int(send_id), data=bytes([0xFF] * 7 + [suffix]), kind=kind)


def load_gate_config(path: str | Path) -> GateConfig:
    backend = _load_backend_module()
    raw = backend.load_read_only_config(path)
    return GateConfig(
        raw=raw,
        channel=raw.channel,
        bitrate=raw.bitrate,
        motors=raw.motors,
        block_reasons=list(raw.block_reasons),
    )


def _reasons_for_motor(config: GateConfig, motor: Any) -> list[str]:
    reasons = list(config.block_reasons)
    if not motor.confirmed:
        reasons.append(f"motor {motor.label} is not confirmed")
    if motor.send_id is None:
        reasons.append(f"motor {motor.label} missing send_id")
    if motor.status_id is None:
        reasons.append(f"motor {motor.label} missing status_id")
    return reasons


def build_disable_proof_plan(config: GateConfig, *, motor_label: str) -> GatePlan:
    motor = config.motor_by_label(motor_label)
    send_id = motor.send_id if motor.send_id is not None else 0
    reasons = _reasons_for_motor(config, motor)
    return GatePlan(
        milestone="C",
        name="disable-proof-dry-run",
        motor_label=motor_label,
        execute_allowed=False,
        transmitted=False,
        block_reasons=reasons + ["hardware transmission is disabled in this implementation slice"],
        frames=[build_system_frame(send_id=send_id, command="disable")],
        checklist=[
            "operator confirms power cutoff reachable",
            "one unloaded motor only",
            "expected send/status IDs confirmed",
            "run actual transmit only in a future explicit safety gate",
        ],
    )


def build_enable_disable_plan(config: GateConfig, *, motor_label: str) -> GatePlan:
    motor = config.motor_by_label(motor_label)
    send_id = motor.send_id if motor.send_id is not None else 0
    reasons = _reasons_for_motor(config, motor)
    return GatePlan(
        milestone="D",
        name="enable-immediate-disable-dry-run",
        motor_label=motor_label,
        execute_allowed=False,
        transmitted=False,
        block_reasons=reasons + ["Gate C disable proof is not passed on real hardware"],
        frames=[build_system_frame(send_id=send_id, command="enable"), build_system_frame(send_id=send_id, command="disable")],
        checklist=[
            "Gate C real disable proof passed",
            "auto-disable timeout configured",
            "current/temperature/fault observation available",
            "one unloaded motor only",
        ],
    )


def build_tiny_motion_plan(config: GateConfig, *, motor_label: str, relative_step_rad: float) -> GatePlan:
    if abs(relative_step_rad) > 0.01:
        raise ValueError("max tiny relative step is 0.01 rad")
    motor = config.motor_by_label(motor_label)
    reasons = _reasons_for_motor(config, motor)
    reasons.extend([
        "Gate D enable-disable proof is not passed",
        "sign/zero/clamps are not confirmed",
        "motion frame packing is intentionally withheld until protocol and motor limits are confirmed",
    ])
    return GatePlan(
        milestone="E",
        name="tiny-single-motor-motion-plan-blocked",
        motor_label=motor_label,
        execute_allowed=False,
        transmitted=False,
        block_reasons=reasons,
        frames=[],
        relative_step_rad=relative_step_rad,
        checklist=[
            "one motor only; second motor disabled",
            "relative command 0.005-0.01 rad maximum",
            "sign, zero, p/v/t/kp/kd clamps confirmed",
            "disable after every attempt",
        ],
    )


def build_two_motor_parity_plan(config: GateConfig) -> GatePlan:
    reasons = list(config.block_reasons)
    reasons.append("two verified single-motor tiny-motion results are required before two-motor parity")
    reasons.append("sign, zero, limit, and emergency-disable evidence must exist for both motors")
    return GatePlan(
        milestone="F",
        name="two-motor-hardware-parity-plan-blocked",
        motor_label=None,
        execute_allowed=False,
        transmitted=False,
        block_reasons=reasons,
        frames=[],
        checklist=[
            "both motors independently identified",
            "both motors independently disabled/enabled/disabled safely",
            "both motors completed tiny single-motor sign checks",
            "no-op hold/read test before any coordinated movement",
            "emergency disable tested before dataset/policy work",
        ],
    )



def build_lelab_integration_plan(config: GateConfig) -> GatePlan:
    reasons = list(config.block_reasons)
    reasons.extend([
        "Milestone F two-motor parity is not passed",
        "LeLab UI must remain read-only/disabled until expected IDs, disable path, clamps, and emergency stop are verified",
        "existing IsaacSimRpoArmRobot SITL path must not be reused as a real CAN backend",
    ])
    return GatePlan(
        milestone="G",
        name="lelab-safe-backend-integration-plan-blocked",
        motor_label=None,
        execute_allowed=False,
        transmitted=False,
        block_reasons=reasons,
        frames=[],
        checklist=[
            "create separate hardware-safe Robot backend after F passes",
            "robot.connect verifies expected IDs and disable path before UI enables controls",
            "UI starts read-only/disabled",
            "requested target, clamped target, raw response, and disable result are logged",
            "start with no-op and tiny relative actions only after all gates pass",
        ],
    )


def build_policy_readiness_plan(config: GateConfig) -> GatePlan:
    reasons = list(config.block_reasons)
    reasons.extend([
        "Milestone G LeLab safe-backend integration is not passed",
        "calibration startup and recovery procedure are missing",
        "dataset/policy safety envelope is not validated on hardware",
        "dry-run policy replay must precede any real actuation",
    ])
    return GatePlan(
        milestone="H",
        name="controlled-data-policy-readiness-plan-blocked",
        motor_label=None,
        execute_allowed=False,
        transmitted=False,
        block_reasons=reasons,
        frames=[],
        checklist=[
            "repeatable calibration startup exists",
            "teleop speed/torque/current limits are documented and enforced",
            "recovery/disable procedure is practiced",
            "dataset logging captures clamps and raw responses",
            "policy replay dry-run passes before any real actuation",
        ],
    )

def plan_from_command(args: Any) -> GatePlan:
    config = load_gate_config(args.config)
    if args.command == "disable-proof":
        return build_disable_proof_plan(config, motor_label=args.motor_label)
    if args.command == "enable-disable-proof":
        return build_enable_disable_plan(config, motor_label=args.motor_label)
    if args.command == "tiny-motion-plan":
        return build_tiny_motion_plan(config, motor_label=args.motor_label, relative_step_rad=args.relative_step_rad)
    if args.command == "two-motor-parity-plan":
        return build_two_motor_parity_plan(config)
    if args.command == "lelab-integration-plan":
        return build_lelab_integration_plan(config)
    if args.command == "policy-readiness-plan":
        return build_policy_readiness_plan(config)
    raise ValueError(f"unsupported command: {args.command}")
