#!/usr/bin/env python3
"""Dry-run gate runner for DM4340P HITL milestones C-F.

This CLI intentionally does not transmit CAN frames. It prints the frames and
blockers needed for the next explicit safety gate.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path


def _load_protocol_module():
    module_path = Path(__file__).with_name("dm4340p_protocol.py")
    spec = importlib.util.spec_from_file_location("dm4340p_protocol", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Dry-run DM4340P HITL gate planner; transmits no CAN frames.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    def add_common(subparser: argparse.ArgumentParser, *, motor: bool = True) -> None:
        subparser.add_argument("--config", default="configs/hitl/dm4340p_x2_read_only.json")
        subparser.add_argument("--json", action="store_true")
        if motor:
            subparser.add_argument("--motor-label", required=True)

    add_common(subparsers.add_parser("disable-proof", help="Milestone C dry-run disable proof plan"))
    add_common(subparsers.add_parser("enable-disable-proof", help="Milestone D dry-run enable then immediate disable plan"))
    tiny = subparsers.add_parser("tiny-motion-plan", help="Milestone E blocked tiny-motion plan")
    add_common(tiny)
    tiny.add_argument("--relative-step-rad", type=float, default=0.005)
    parity = subparsers.add_parser("two-motor-parity-plan", help="Milestone F blocked two-motor parity plan")
    add_common(parity, motor=False)
    lelab = subparsers.add_parser("lelab-integration-plan", help="Milestone G blocked LeLab safe-backend plan")
    add_common(lelab, motor=False)
    policy = subparsers.add_parser("policy-readiness-plan", help="Milestone H blocked data/policy readiness plan")
    add_common(policy, motor=False)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    protocol = _load_protocol_module()
    plan = protocol.plan_from_command(args)
    payload = plan.to_dict()
    if args.json:
        print(json.dumps(payload, sort_keys=True))
    else:
        print(json.dumps(payload, indent=2, sort_keys=True))
        print("Safety: dry-run only; no CAN transmit, no torque enable, no motion command.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
