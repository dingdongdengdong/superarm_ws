#!/usr/bin/env bash
set -euo pipefail

# Run multiple LeRobot pose cases against local /workspace/isaacsim and capture
# one live Isaac camera screenshot per commanded pose.

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
ISAAC_ROOT=${ISAAC_ROOT:-/workspace/isaacsim}
RUN_ID=${RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}
RUN_DIR=${RUN_DIR:-$REPO_ROOT/isaacsim_test/artifacts/source_arm_lerobot_pose_cases_$RUN_ID}
LOG_DIR=$RUN_DIR/logs
DATA_DIR=$RUN_DIR/data
SCREENSHOT_DIR=$RUN_DIR/screenshots
REPORT_JSON=$RUN_DIR/report.json
REPORT_MD=$RUN_DIR/report.md
URDF_PATH=${URDF_PATH:-$REPO_ROOT/isaacsim_test/outputs/robot_arm_hand_from_zip_local_drive/robot_arm_hand_sanitized.urdf}
CONFIG_PATH=${CONFIG_PATH:-$REPO_ROOT/isaacsim_test/lerobot/source_arm_isaacsim_arm_only.yaml}
CASES_PATH=${CASES_PATH:-}
POSE_EVIDENCE=$DATA_DIR/lerobot_pose_cases.json
COMMAND_EVIDENCE_DIR=$DATA_DIR/isaac_command_evidence
ISAAC_LOG=$LOG_DIR/isaac_pose_cases.log
POSE_LOG=$LOG_DIR/lerobot_pose_cases.log
ORCH_LOG=$LOG_DIR/orchestration.log
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
EXPECTED_CASES=${EXPECTED_CASES:-4}
isaac_pid=""
pose_status="not_run"
sim_status="not_run"
report_status="not_done"
report_reason="initialized"

write_report() {
  local exit_status=${1:-0}
  REPORT_EXIT_STATUS=$exit_status \
  REPORT_STATUS=$report_status \
  REPORT_REASON=$report_reason \
  RUN_ID=$RUN_ID \
  RUN_DIR=$RUN_DIR \
  LOG_DIR=$LOG_DIR \
  DATA_DIR=$DATA_DIR \
  SCREENSHOT_DIR=$SCREENSHOT_DIR \
  URDF_PATH=$URDF_PATH \
  CONFIG_PATH=$CONFIG_PATH \
  POSE_EVIDENCE=$POSE_EVIDENCE \
  ISAAC_LOG=$ISAAC_LOG \
  POSE_LOG=$POSE_LOG \
  COMMAND_EVIDENCE_DIR=$COMMAND_EVIDENCE_DIR \
  REPORT_JSON=$REPORT_JSON \
  REPORT_MD=$REPORT_MD \
  CONTACT_SHEET=$SCREENSHOT_DIR/contact_sheet.png \
  POSE_STATUS=$pose_status \
  SIM_STATUS=$sim_status \
  EXPECTED_CASES=$EXPECTED_CASES \
  python3 - <<'PY'
import json
import os
from datetime import datetime, timezone
from pathlib import Path

screenshots = sorted(Path(os.environ["SCREENSHOT_DIR"]).glob("command_*.png"))
manifest = Path(os.environ["SCREENSHOT_DIR"]) / "manifest.json"
pose_evidence = Path(os.environ["POSE_EVIDENCE"])
case_passed = None
case_count = 0
if pose_evidence.is_file():
    data = json.loads(pose_evidence.read_text(encoding="utf-8"))
    case_passed = bool(data.get("passed"))
    case_count = int(data.get("case_count", 0))
report = {
    "status": os.environ["REPORT_STATUS"],
    "reason": os.environ["REPORT_REASON"],
    "exit_status": int(os.environ["REPORT_EXIT_STATUS"]),
    "generated_at_utc": datetime.now(timezone.utc).isoformat(),
    "run_id": os.environ["RUN_ID"],
    "run_dir": os.environ["RUN_DIR"],
    "logs_dir": os.environ["LOG_DIR"],
    "data_dir": os.environ["DATA_DIR"],
    "screenshots_dir": os.environ["SCREENSHOT_DIR"],
    "urdf_path": os.environ["URDF_PATH"],
    "config": os.environ["CONFIG_PATH"],
    "checks": {
        "pose_runner_status": os.environ["POSE_STATUS"],
        "isaac_sim_status": os.environ["SIM_STATUS"],
        "pose_cases_passed": case_passed,
        "pose_case_count": case_count,
        "expected_screenshot_count": int(os.environ["EXPECTED_CASES"]),
        "fresh_screenshot_count": len([p for p in screenshots if p.stat().st_size > 0]),
        "manifest_exists": manifest.is_file() and manifest.stat().st_size > 0,
    },
    "paths": {
        "isaac_log": os.environ["ISAAC_LOG"],
        "pose_log": os.environ["POSE_LOG"],
        "pose_evidence": os.environ["POSE_EVIDENCE"],
        "command_evidence_dir": os.environ["COMMAND_EVIDENCE_DIR"],
        "screenshot_manifest": str(manifest),
        "contact_sheet": os.environ["CONTACT_SHEET"],
        "screenshots": [str(p) for p in screenshots],
    },
}
Path(os.environ["REPORT_JSON"]).write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
Path(os.environ["REPORT_MD"]).write_text(
    "# Source Arm LeRobot Pose Capture Report\n\n"
    f"- Status: `{report['status']}`\n"
    f"- Reason: {report['reason']}\n"
    f"- Run directory: `{report['run_dir']}`\n"
    f"- Pose cases passed: `{report['checks']['pose_cases_passed']}`\n"
    f"- Screenshots: {report['checks']['fresh_screenshot_count']}/{report['checks']['expected_screenshot_count']}\n"
    f"- Screenshots dir: `{report['screenshots_dir']}`\n"
    f"- Pose evidence: `{report['paths']['pose_evidence']}`\n",
    encoding="utf-8",
)
print(json.dumps(report, indent=2, sort_keys=True))
PY
}

cleanup() {
  local exit_status=$?
  if [[ -n "${isaac_pid:-}" ]] && kill -0 "$isaac_pid" >/dev/null 2>&1; then
    kill "$isaac_pid" >/dev/null 2>&1 || true
    wait "$isaac_pid" >/dev/null 2>&1 || true
  fi
  if [[ -d "$RUN_DIR" ]]; then
    write_report "$exit_status" > /dev/null || true
  fi
}
trap cleanup EXIT

mkdir -p "$LOG_DIR" "$DATA_DIR" "$SCREENSHOT_DIR" "$COMMAND_EVIDENCE_DIR"
cat > "$DATA_DIR/pose_cases_used.json" <<'JSON'
{
  "cases": [
    {"name": "home_zero", "target": [0.0, 0.0, 0.0, 0.0, 0.0]},
    {"name": "positive_reach", "target": [0.25, -0.20, 0.30, -0.35, 0.20]},
    {"name": "negative_reach", "target": [-0.25, 0.20, -0.30, 0.35, -0.20]},
    {"name": "mixed_elbow", "target": [0.40, 0.10, 0.15, -0.45, 0.30]}
  ]
}
JSON
if [[ -z "$CASES_PATH" ]]; then
  CASES_PATH=$DATA_DIR/pose_cases_used.json
fi

if [[ ! -x "$ISAAC_ROOT/python.sh" ]]; then
  report_reason="Isaac Sim python.sh not found or not executable: $ISAAC_ROOT/python.sh"
  echo "$report_reason" >&2
  exit 2
fi
if [[ ! -s "$URDF_PATH" ]]; then
  echo "Preparing source arm URDF from tracked robot_arm_hand_package.zip" | tee -a "$ORCH_LOG"
  python3 "$REPO_ROOT/isaacsim_test/isaacsim/robot_arm_hand_from_zip.py" \
    --mode prepare \
    --zip "$REPO_ROOT/robot_arm_hand_package.zip" \
    --input-root "$REPO_ROOT/isaacsim_test/inputs/robot_arm_hand_package_local_drive" \
    --output-root "$REPO_ROOT/isaacsim_test/outputs/robot_arm_hand_from_zip_local_drive" \
    --report "$DATA_DIR/source_arm_prepare_report.json" \
    > "$LOG_DIR/source_arm_prepare.stdout.log" \
    2> "$LOG_DIR/source_arm_prepare.stderr.log"
fi
if [[ ! -s "$URDF_PATH" ]]; then
  report_reason="URDF not found after prepare: $URDF_PATH"
  echo "$report_reason" >&2
  exit 2
fi

(
  cd "$REPO_ROOT"
  export ROS_DOMAIN_ID
  export ROS_DISTRO=humble
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  export PYTHONUNBUFFERED=1
  export HEADLESS=1
  export SCREENSHOT_AFTER_COMMAND=1
  export SCREENSHOT_EACH_COMMAND=1
  export SCREENSHOT_EXIT_AFTER_COMMAND_COUNT=$EXPECTED_CASES
  export SCREENSHOT_ON_STARTUP=0
  export EXIT_AFTER_SCREENSHOT=1
  export SCREENSHOT_SEQUENCE_DIR="$SCREENSHOT_DIR"
  export SIMREADY_USD_PATH=/does/not/exist.usd
  export RPO_ARM_URDF_PATH="$URDF_PATH"
  export JOINT_NAMES=joint_rev_1,joint_rev_2,joint_rev_3,joint_rev_4,joint_rev_5
  export NUM_JOINTS=5
  export COMMAND_EVIDENCE_PATH="$DATA_DIR/isaac_last_command_evidence.json"
  export COMMAND_EVIDENCE_DIR="$COMMAND_EVIDENCE_DIR"
  export LD_LIBRARY_PATH="$ISAAC_ROOT/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$ISAAC_ROOT/exts/isaacsim.ros2.bridge/humble/rclpy:${PYTHONPATH:-}"
  "$ISAAC_ROOT/python.sh" "$REPO_ROOT/isaacsim_test/isaacsim/setup_rpo_arm_scene.py"
) >"$ISAAC_LOG" 2>&1 &
isaac_pid=$!

deadline=$((SECONDS + 150))
while (( SECONDS < deadline )); do
  if grep -q "Simulation running" "$ISAAC_LOG" 2>/dev/null; then
    echo "Isaac ready" | tee "$ORCH_LOG"
    break
  fi
  if grep -q "ERROR:" "$ISAAC_LOG" 2>/dev/null; then
    echo "Isaac error before ready" | tee "$ORCH_LOG"
    break
  fi
  if ! kill -0 "$isaac_pid" >/dev/null 2>&1; then
    echo "Isaac exited before ready" | tee "$ORCH_LOG"
    break
  fi
  sleep 2
done
if ! grep -q "Simulation running" "$ISAAC_LOG" 2>/dev/null; then
  report_reason="Isaac did not reach Simulation running; see $ISAAC_LOG"
  exit 3
fi

(
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  export ROS_DOMAIN_ID
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  export PYTHONPATH="$REPO_ROOT/isaacsim_test/lerobot:${PYTHONPATH:-}"
  python3 "$REPO_ROOT/isaacsim_test/lerobot/run_lerobot_pose_cases.py" \
    --config "$CONFIG_PATH" \
    --cases "$CASES_PATH" \
    --evidence "$POSE_EVIDENCE" \
    --case-timeout-s 20 \
    --settle-s 3.0
) >"$POSE_LOG" 2>&1
pose_status=$?

sim_status=0
wait "$isaac_pid" || sim_status=$?
isaac_pid=""

actual_count=$(find "$SCREENSHOT_DIR" -maxdepth 1 -type f -name 'command_*.png' -size +0c | wc -l | tr -d ' ')
if [[ "$pose_status" -ne 0 ]]; then
  report_reason="LeRobot pose-case runner failed with status $pose_status; see $POSE_LOG"
  exit "$pose_status"
fi
if [[ "$sim_status" -ne 0 ]]; then
  report_reason="Isaac Sim exited with status $sim_status; see $ISAAC_LOG"
  exit "$sim_status"
fi
if [[ "$actual_count" != "$EXPECTED_CASES" ]]; then
  report_reason="Expected $EXPECTED_CASES screenshots, found $actual_count in $SCREENSHOT_DIR"
  exit 4
fi
if [[ ! -s "$SCREENSHOT_DIR/manifest.json" ]]; then
  report_reason="Missing screenshot manifest: $SCREENSHOT_DIR/manifest.json"
  exit 4
fi
python3 - <<PY
from pathlib import Path
try:
    from PIL import Image, ImageDraw
except Exception:
    raise SystemExit(0)
shot_dir = Path("$SCREENSHOT_DIR")
imgs = sorted(shot_dir.glob("command_*.png"))
if not imgs:
    raise SystemExit(0)
thumbs = []
for img in imgs:
    im = Image.open(img).convert("RGB")
    im.thumbnail((640, 360))
    canvas = Image.new("RGB", (640, 400), "white")
    canvas.paste(im, (0, 30))
    ImageDraw.Draw(canvas).text((10, 8), img.name, fill=(0, 0, 0))
    thumbs.append(canvas)
cols = 2
rows = (len(thumbs) + cols - 1) // cols
out = Image.new("RGB", (cols * 640, rows * 400), "white")
for i, thumb in enumerate(thumbs):
    out.paste(thumb, ((i % cols) * 640, (i // cols) * 400))
out.save(shot_dir / "contact_sheet.png")
PY

report_status="done"
report_reason="LeRobot drove $EXPECTED_CASES source-arm pose cases and captured one live screenshot per case"
write_report 0
printf 'Source arm LeRobot pose capture artifacts:\n%s\n' "$RUN_DIR"
