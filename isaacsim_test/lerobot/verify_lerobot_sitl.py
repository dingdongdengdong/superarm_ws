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

from isaacsim_rpo_arm_robot import IsaacSimRpoArmConfig, IsaacSimRpoArmRobot  # noqa: E402
from rpo_arm_contract import JOINT_NAMES, normalize_action  # noqa: E402

DEFAULT_TARGET = [0.2, 0.1, -0.2, 0.3, 0.1, 0.5]
DEFAULT_TOLERANCE = 0.03
DEFAULT_CONFIG = SCRIPT_DIR / "rpo_arm_isaacsim.yaml"
DEFAULT_EVIDENCE = REPO_ROOT / "isaacsim_test/artifacts/lerobot_sitl_verify.json"


def _parse_vector(value: str) -> list[float]:
    parts = [part.strip() for part in value.split(",") if part.strip()]
    vector = [float(part) for part in parts]
    if len(vector) != len(JOINT_NAMES):
        raise argparse.ArgumentTypeError(
            f"expected {len(JOINT_NAMES)} comma-separated values, got {len(vector)}"
        )
    return vector


def _load_config(path: Path) -> IsaacSimRpoArmConfig:
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    raw.pop("_type", None)
    return IsaacSimRpoArmConfig(**raw)


def _as_list(value: Any) -> list[float]:
    return np.asarray(value, dtype=np.float32).reshape(-1).astype(float).tolist()


def _write_evidence(path: Path, evidence: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(evidence, indent=2, sort_keys=True), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--target", type=_parse_vector, default=list(DEFAULT_TARGET))
    parser.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE)
    parser.add_argument("--timeout-s", type=float, default=20.0)
    parser.add_argument("--period-s", type=float, default=0.1)
    parser.add_argument("--evidence", type=Path, default=DEFAULT_EVIDENCE)
    args = parser.parse_args()

    os.environ.setdefault("ROS_DOMAIN_ID", "42")
    config = _load_config(args.config)
    if list(config.joint_names) != JOINT_NAMES:
        raise ValueError(f"Unexpected config joint_names: {config.joint_names}")

    target = np.asarray(normalize_action(args.target), dtype=np.float32)
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
                    "joint_names": JOINT_NAMES,
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
            "joint_names": JOINT_NAMES,
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
