#!/usr/bin/env python3
"""Generate Isaac Sim 5.1 LeLab SuperArm verification artifacts.

Creates a timestamped folder with logs, six 5DOF+grasp command cases, LeLab SuperArm HTML snapshot, and PNG evidence of the control/case matrix.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from isaacsim_test.lerobot.lelab_isaacsim51_control_contract import (  # noqa: E402
    create_timestamped_artifact_root,
    write_contract_artifacts,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--base-dir",
        type=Path,
        default=Path("isaacsim_test/artifacts"),
        help="Parent directory for the timestamped verification folder.",
    )
    parser.add_argument(
        "--timestamp",
        default=None,
        help="UTC timestamp folder suffix, e.g. 20260706T051500Z. Defaults to now.",
    )
    parser.add_argument(
        "--version-file",
        type=Path,
        default=Path("/workspace/isaacsim/VERSION"),
        help="Isaac Sim VERSION file to check for 5.1 compatibility.",
    )
    args = parser.parse_args()

    root = create_timestamped_artifact_root(args.base_dir, args.timestamp)
    report = write_contract_artifacts(root, args.version_file)
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    return 0 if report["status"] == "done" else 2


if __name__ == "__main__":
    raise SystemExit(main())
