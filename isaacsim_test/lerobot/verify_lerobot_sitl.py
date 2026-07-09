#!/usr/bin/env python3
"""Verify Isaac Sim SITL by controlling the RoboParty V2 arm through LeRobot.

This script intentionally uses the local LeRobot robot shim instead of publishing
commands directly to ROS. It sends one target action through
IsaacSimRpoArmRobot.send_action(), then compares the observed state returned by
IsaacSimRpoArmRobot.capture_observation() against the intended target.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
LEROBOT_PATH = REPO_ROOT / "lerobot"
for path in (SCRIPT_DIR, LEROBOT_PATH):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)

from isaacsim_rpo_arm_robot import (  # noqa: E402
    ARM_JOINT_NAMES,
    JOINT_NAMES,
    SYNTHETIC_GRASP_NAME,
    IsaacSimRpoArmConfig,
    IsaacSimRpoArmRobot,
)

# Full 6D contract kept explicit for static/readability tests:
# right_arm_pitch_joint, right_arm_roll_joint, right_arm_yaw_joint,
# right_elbow_pitch_joint, right_elbow_yaw_joint, amazinghand_grasp.
# The fixed-hand branch uses the five arm joints only.
DEFAULT_FULL_TARGET_BY_JOINT = {
    "right_arm_pitch_joint": 0.2,
    "right_arm_roll_joint": 0.1,
    "right_arm_yaw_joint": -0.2,
    "right_elbow_pitch_joint": 0.3,
    "right_elbow_yaw_joint": 0.1,
    SYNTHETIC_GRASP_NAME: 0.5,
}
DEFAULT_ARM_ONLY_TARGET_BY_JOINT = {
    "right_arm_pitch_joint": 0.2,
    "right_arm_roll_joint": 0.1,
    "right_arm_yaw_joint": -0.2,
    "right_elbow_pitch_joint": 0.3,
    "right_elbow_yaw_joint": -0.15,
}
DEFAULT_TOLERANCE = 0.03
DEFAULT_CONFIG = SCRIPT_DIR / "rpo_arm_isaacsim.yaml"
DEFAULT_EVIDENCE = REPO_ROOT / "isaacsim_test/artifacts/lerobot_sitl_verify.json"


def _parse_vector(value: str) -> list[float]:
    parts = [part.strip() for part in value.split(",") if part.strip()]
    return [float(part) for part in parts]


def _load_config(path: Path) -> IsaacSimRpoArmConfig:
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    raw.pop("_type", None)
    return IsaacSimRpoArmConfig(**raw)


def _default_target_for_config(config: IsaacSimRpoArmConfig) -> list[float]:
    defaults = (
        DEFAULT_FULL_TARGET_BY_JOINT
        if SYNTHETIC_GRASP_NAME in config.joint_names
        else DEFAULT_ARM_ONLY_TARGET_BY_JOINT
    )
    target: list[float] = []
    for joint_name in config.joint_names:
        if joint_name == SYNTHETIC_GRASP_NAME and config.fixed_hand:
            target.append(float(config.fixed_grasp))
        elif joint_name in defaults:
            target.append(defaults[joint_name])
        else:
            target.append(0.0)
    return target


def _validate_config_joint_names(config: IsaacSimRpoArmConfig) -> None:
    joint_names = list(config.joint_names)
    if config.allow_custom_joint_names:
        if not joint_names:
            raise ValueError("Custom config joint_names must not be empty")
        duplicates = sorted({name for name in joint_names if joint_names.count(name) > 1})
        if duplicates:
            raise ValueError(f"Duplicate custom config joint_names: {duplicates}")
        return
    allowed = set(JOINT_NAMES)
    unknown = [name for name in joint_names if name not in allowed]
    if unknown:
        raise ValueError(f"Unexpected config joint_names {unknown}; expected subset/order of {JOINT_NAMES}")
    if joint_names[: len(ARM_JOINT_NAMES)] != ARM_JOINT_NAMES[: len(joint_names[: len(ARM_JOINT_NAMES)])]:
        raise ValueError(f"Unexpected arm joint order in config: {joint_names}")


def _as_list(value: Any) -> list[float]:
    return np.asarray(value, dtype=np.float32).reshape(-1).astype(float).tolist()


def _write_evidence(path: Path, evidence: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(evidence, indent=2, sort_keys=True), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--target", type=_parse_vector, default=None)
    parser.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE)
    parser.add_argument("--timeout-s", type=float, default=20.0)
    parser.add_argument("--period-s", type=float, default=0.1)
    parser.add_argument("--evidence", type=Path, default=DEFAULT_EVIDENCE)
    args = parser.parse_args()

    os.environ.setdefault("ROS_DOMAIN_ID", "42")
    config = _load_config(args.config)
    _validate_config_joint_names(config)

    target_values = args.target if args.target is not None else _default_target_for_config(config)
    if len(target_values) != len(config.joint_names):
        raise ValueError(
            f"Target length {len(target_values)} does not match config joint count "
            f"{len(config.joint_names)} for {list(config.joint_names)}"
        )
    target = np.asarray(target_values, dtype=np.float32)
    tolerance = np.full(target.shape, float(args.tolerance), dtype=np.float32)
    robot = IsaacSimRpoArmRobot(config)
    deadline = time.time() + args.timeout_s
    best_observed: list[float] | None = None
    best_error: list[float] | None = None

    try:
        robot.connect()
        while time.time() < deadline:
            sent = robot.send_action(target)
            observation = robot.capture_observation()["observation.state"]
            observed = np.asarray(observation, dtype=np.float32).reshape(-1)
            error = np.abs(observed - target)
            best_observed = _as_list(observed)
            best_error = _as_list(error)
            if observed.shape == target.shape and bool(np.all(error <= tolerance)):
                evidence = {
                    "passed": True,
                    "joint_names": list(config.joint_names),
                    "fixed_hand": bool(config.fixed_hand),
                    "fixed_grasp": float(config.fixed_grasp),
                    "target": _as_list(target),
                    "sent_action": _as_list(sent),
                    "observed": best_observed,
                    "absolute_error": best_error,
                    "tolerance": _as_list(tolerance),
                    "config": str(args.config),
                }
                _write_evidence(args.evidence, evidence)
                print(json.dumps(evidence, indent=2, sort_keys=True))
                return 0
            time.sleep(args.period_s)

        evidence = {
            "passed": False,
            "joint_names": list(config.joint_names),
            "fixed_hand": bool(config.fixed_hand),
            "fixed_grasp": float(config.fixed_grasp),
            "target": _as_list(target),
            "observed": best_observed,
            "absolute_error": best_error,
            "tolerance": _as_list(tolerance),
            "config": str(args.config),
            "reason": f"target not reached within {args.timeout_s}s",
        }
        _write_evidence(args.evidence, evidence)
        print(json.dumps(evidence, indent=2, sort_keys=True), file=sys.stderr)
        return 1
    finally:
        robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
