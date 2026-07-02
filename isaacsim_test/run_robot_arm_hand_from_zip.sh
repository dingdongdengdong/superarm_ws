#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
SUPERARM_WS_PATH=${SUPERARM_WS_PATH:-$(cd -- "$SCRIPT_DIR/.." && pwd)}
OUTPUT_ROOT=${ROBOT_ARM_HAND_OUTPUT_ROOT:-/workspace/superarm_ws/isaacsim_test/outputs/robot_arm_hand_from_zip}
SCREENSHOT_ROOT=${ROBOT_ARM_HAND_SCREENSHOT_OUTPUT_DIR:-/workspace/superarm_ws/isaacsim_test/artifacts/robot_arm_hand_from_zip}
REPORT_PATH=${ROBOT_ARM_HAND_REPORT_PATH:-$OUTPUT_ROOT/robot_arm_hand_connected_report.json}
LOG_STAMP=${ROBOT_ARM_HAND_LOG_STAMP:-$(date -u +%Y%m%dT%H%M%SZ)}
RUNTIME_LOG_CONTAINER=${ROBOT_ARM_HAND_RUNTIME_LOG_PATH:-/workspace/superarm_ws/isaacsim_test/artifacts/runtime_logs/robot_arm_hand_from_zip_${LOG_STAMP}.log}
RUNTIME_LOG_HOST="$SUPERARM_WS_PATH/${RUNTIME_LOG_CONTAINER#/workspace/superarm_ws/}"

cd "$SCRIPT_DIR"
mkdir -p "$(dirname "$RUNTIME_LOG_HOST")"

set -o pipefail
SUPERARM_WS_PATH="$SUPERARM_WS_PATH" docker compose run --rm --no-deps \
  -e HEADLESS="${HEADLESS:-1}" \
  -e ROBOT_ARM_HAND_ZIP_SOURCE="${ROBOT_ARM_HAND_ZIP_SOURCE:-/workspace/superarm_ws/robot_arm_hand_package.zip}" \
  -e ROBOT_ARM_HAND_INPUT_ROOT="${ROBOT_ARM_HAND_INPUT_ROOT:-/workspace/superarm_ws/isaacsim_test/inputs/robot_arm_hand_package}" \
  -e ROBOT_ARM_HAND_OUTPUT_ROOT="$OUTPUT_ROOT" \
  -e ROBOT_ARM_HAND_CONNECTED_USD_PATH="${ROBOT_ARM_HAND_CONNECTED_USD_PATH:-$OUTPUT_ROOT/robot_arm_hand_connected.usd}" \
  -e ROBOT_ARM_HAND_REPORT_PATH="$REPORT_PATH" \
  -e ROBOT_ARM_HAND_SCREENSHOT_OUTPUT_DIR="$SCREENSHOT_ROOT" \
  -e ROBOT_ARM_HAND_SETTLE_STEPS="${ROBOT_ARM_HAND_SETTLE_STEPS:-20}" \
  -e ROBOT_ARM_HAND_GRASP_STEPS="${ROBOT_ARM_HAND_GRASP_STEPS:-90}" \
  -e ROBOT_ARM_HAND_FINGER_MOTION_STEPS="${ROBOT_ARM_HAND_FINGER_MOTION_STEPS:-45}" \
  isaac-sim-51 \
  "exec /isaac-sim/python.sh /workspace/isaacsim/robot_arm_hand_from_zip.py --mode convert" \
  2>&1 | tee "$RUNTIME_LOG_HOST"

SUPERARM_WS_PATH="$SUPERARM_WS_PATH" docker compose run --rm --no-deps \
  -e HEADLESS="${HEADLESS:-1}" \
  -e ROBOT_ARM_HAND_ZIP_SOURCE="${ROBOT_ARM_HAND_ZIP_SOURCE:-/workspace/superarm_ws/robot_arm_hand_package.zip}" \
  -e ROBOT_ARM_HAND_INPUT_ROOT="${ROBOT_ARM_HAND_INPUT_ROOT:-/workspace/superarm_ws/isaacsim_test/inputs/robot_arm_hand_package}" \
  -e ROBOT_ARM_HAND_OUTPUT_ROOT="$OUTPUT_ROOT" \
  -e ROBOT_ARM_HAND_CONNECTED_USD_PATH="${ROBOT_ARM_HAND_CONNECTED_USD_PATH:-$OUTPUT_ROOT/robot_arm_hand_connected.usd}" \
  -e ROBOT_ARM_HAND_REPORT_PATH="$REPORT_PATH" \
  -e ROBOT_ARM_HAND_SCREENSHOT_OUTPUT_DIR="$SCREENSHOT_ROOT" \
  -e ROBOT_ARM_HAND_SETTLE_STEPS="${ROBOT_ARM_HAND_SETTLE_STEPS:-20}" \
  -e ROBOT_ARM_HAND_GRASP_STEPS="${ROBOT_ARM_HAND_GRASP_STEPS:-90}" \
  -e ROBOT_ARM_HAND_FINGER_MOTION_STEPS="${ROBOT_ARM_HAND_FINGER_MOTION_STEPS:-45}" \
  isaac-sim-51 \
  "exec /isaac-sim/python.sh /workspace/isaacsim/robot_arm_hand_from_zip.py --mode runtime" \
  2>&1 | tee -a "$RUNTIME_LOG_HOST"

python3 - "$SUPERARM_WS_PATH" "$REPORT_PATH" "$RUNTIME_LOG_CONTAINER" <<'PY'
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
report_path = sys.argv[2]
runtime_log = sys.argv[3]
if report_path.startswith("/workspace/superarm_ws/"):
    report = root / report_path.removeprefix("/workspace/superarm_ws/")
else:
    report = Path(report_path)
payload = json.loads(report.read_text(encoding="utf-8"))
payload.setdefault("runtime_validation", {})["isaac_sim_log"] = runtime_log.removeprefix(
    "/workspace/superarm_ws/"
)
try:
    report.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
except PermissionError as exc:
    print(f"[robot-arm-hand-runner] WARNING: could not update root-owned report: {exc}")
else:
    print(f"[robot-arm-hand-runner] Report updated: {report}")
PY
