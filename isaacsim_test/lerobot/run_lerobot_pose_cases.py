#!/usr/bin/env python3
"""Send multiple LeRobot arm pose test cases to Isaac Sim and verify readback.

This intentionally sends one LeRobot action per case, then polls observation.
That gives the Isaac scene one distinct ROS command per screenshot.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime, timezone
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

DEFAULT_CONFIG = SCRIPT_DIR / "source_arm_isaacsim_arm_only.yaml"
DEFAULT_EVIDENCE = REPO_ROOT / "isaacsim_test/artifacts/lerobot_pose_cases.json"
DEFAULT_CASES = [
    {"name": "home_zero", "target": [0.0, 0.0, 0.0, 0.0, 0.0]},
    {"name": "positive_reach", "target": [0.25, -0.20, 0.30, -0.35, 0.20]},
    {"name": "negative_reach", "target": [-0.25, 0.20, -0.30, 0.35, -0.20]},
    {"name": "mixed_elbow", "target": [0.40, 0.10, 0.15, -0.45, 0.30]},
]


def _load_config(path: Path) -> IsaacSimRpoArmConfig:
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    raw.pop("_type", None)
    return IsaacSimRpoArmConfig(**raw)


def _load_cases(path: Path | None) -> list[dict[str, Any]]:
    if path is None:
        return list(DEFAULT_CASES)
    raw = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(raw, dict):
        raw = raw.get("cases", [])
    if not isinstance(raw, list) or not raw:
        raise ValueError("Pose cases JSON must be a non-empty list or {'cases': [...]} object")
    cases: list[dict[str, Any]] = []
    for idx, item in enumerate(raw, start=1):
        if not isinstance(item, dict):
            raise ValueError(f"Case #{idx} must be an object")
        name = str(item.get("name") or f"case_{idx:03d}")
        target = [float(v) for v in item["target"]]
        cases.append({"name": name, "target": target})
    return cases


def _as_list(value: Any) -> list[float]:
    return np.asarray(value, dtype=np.float32).reshape(-1).astype(float).tolist()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--cases", type=Path, default=None)
    parser.add_argument("--evidence", type=Path, default=DEFAULT_EVIDENCE)
    parser.add_argument("--tolerance", type=float, default=0.03)
    parser.add_argument("--case-timeout-s", type=float, default=10.0)
    parser.add_argument("--period-s", type=float, default=0.1)
    parser.add_argument("--settle-s", type=float, default=2.5)
    args = parser.parse_args()

    os.environ.setdefault("ROS_DOMAIN_ID", "42")
    config = _load_config(args.config)
    cases = _load_cases(args.cases)
    joint_names = list(config.joint_names)
    tolerance = np.full((len(joint_names),), float(args.tolerance), dtype=np.float32)

    for case in cases:
        if len(case["target"]) != len(joint_names):
            raise ValueError(
                f"Case {case['name']} target length {len(case['target'])} does not match "
                f"joint count {len(joint_names)}"
            )

    robot = IsaacSimRpoArmRobot(config)
    results: list[dict[str, Any]] = []
    overall_passed = True
    try:
        robot.connect()
        for index, case in enumerate(cases, start=1):
            target = np.asarray(case["target"], dtype=np.float32)
            sent = robot.send_action(target)
            deadline = time.time() + args.case_timeout_s
            observed = None
            error = None
            passed = False
            while time.time() < deadline:
                obs = robot.capture_observation()["observation.state"]
                observed = np.asarray(obs, dtype=np.float32).reshape(-1)
                error = np.abs(observed - target)
                if observed.shape == target.shape and bool(np.all(error <= tolerance)):
                    passed = True
                    break
                time.sleep(args.period_s)
            time.sleep(max(args.settle_s, 0.0))
            if not passed:
                overall_passed = False
            results.append(
                {
                    "index": index,
                    "name": case["name"],
                    "joint_names": joint_names,
                    "target": _as_list(target),
                    "sent_action": _as_list(sent),
                    "observed": _as_list(observed if observed is not None else []),
                    "absolute_error": _as_list(error if error is not None else []),
                    "tolerance": _as_list(tolerance),
                    "passed": passed,
                }
            )
            print(json.dumps(results[-1], indent=2, sort_keys=True), flush=True)
    finally:
        robot.disconnect()

    evidence = {
        "passed": overall_passed,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "config": str(args.config),
        "joint_names": joint_names,
        "case_count": len(cases),
        "cases": results,
    }
    args.evidence.parent.mkdir(parents=True, exist_ok=True)
    args.evidence.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(evidence, indent=2, sort_keys=True), flush=True)
    return 0 if overall_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
