#!/usr/bin/env bash
set -euo pipefail

# Run the arm-only LeRobot/Isaac Sim verification for this branch.
# The AmazingHand is fixed: LeRobot exposes only the five arm joints.
# Artifacts are always written under a UTC timestamped directory:
#   isaacsim_test/artifacts/arm_fixed_hand_lerobot_<YYYYmmddTHHMMSSZ>/
#     logs/
#     screenshots/
#     data/
#     report.json
#     report.md

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
RUN_ID=${RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}
RUN_DIR_NAME=${RUN_DIR_NAME:-arm_fixed_hand_lerobot_$RUN_ID}
ARTIFACT_ROOT_HOST=${ARTIFACT_ROOT_HOST:-$REPO_ROOT/isaacsim_test/artifacts}
RUN_DIR_HOST=${RUN_DIR_HOST:-$ARTIFACT_ROOT_HOST/$RUN_DIR_NAME}
RUN_DIR_CONTAINER=${RUN_DIR_CONTAINER:-/workspace/superarm_ws/isaacsim_test/artifacts/$RUN_DIR_NAME}
LOG_DIR_HOST=$RUN_DIR_HOST/logs
SCREENSHOT_DIR_HOST=$RUN_DIR_HOST/screenshots
DATA_DIR_HOST=$RUN_DIR_HOST/data
DATA_DIR_CONTAINER=$RUN_DIR_CONTAINER/data
SCREENSHOT_DIR_CONTAINER=$RUN_DIR_CONTAINER/screenshots
REPORT_JSON_HOST=$RUN_DIR_HOST/report.json
REPORT_MD_HOST=$RUN_DIR_HOST/report.md
ISAAC_LOG_HOST=$LOG_DIR_HOST/isaac_sim.log
VERIFY_LOG_HOST=$LOG_DIR_HOST/lerobot_verify.log
VERIFY_EVIDENCE_CONTAINER=$DATA_DIR_CONTAINER/arm_fixed_hand_lerobot_verify.json
VERIFY_EVIDENCE_HOST=$DATA_DIR_HOST/arm_fixed_hand_lerobot_verify.json
SCREENSHOT_PATH_CONTAINER=$SCREENSHOT_DIR_CONTAINER/arm_fixed_hand_after_command.png
SCREENSHOT_PATH_HOST=$SCREENSHOT_DIR_HOST/arm_fixed_hand_after_command.png
CONFIG_CONTAINER=/workspace/superarm_ws/isaacsim_test/lerobot/rpo_arm_isaacsim_arm_only.yaml
SUPERARM_WS_PATH=${SUPERARM_WS_PATH:-$REPO_ROOT}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
ISAAC_STARTUP_WAIT_S=${ISAAC_STARTUP_WAIT_S:-35}
TARGET=${TARGET:-}
isaac_pid=""
verify_status="not_run"
sim_status="not_run"
report_status="not_done"
report_reason="initialized"

write_report() {
  local exit_status=${1:-0}
  REPORT_EXIT_STATUS=$exit_status \
  REPORT_STATUS=$report_status \
  REPORT_REASON=$report_reason \
  RUN_ID=$RUN_ID \
  RUN_DIR_HOST=$RUN_DIR_HOST \
  LOG_DIR_HOST=$LOG_DIR_HOST \
  SCREENSHOT_DIR_HOST=$SCREENSHOT_DIR_HOST \
  DATA_DIR_HOST=$DATA_DIR_HOST \
  VERIFY_EVIDENCE_HOST=$VERIFY_EVIDENCE_HOST \
  VERIFY_LOG_HOST=$VERIFY_LOG_HOST \
  ISAAC_LOG_HOST=$ISAAC_LOG_HOST \
  SCREENSHOT_PATH_HOST=$SCREENSHOT_PATH_HOST \
  CONFIG_HOST=$REPO_ROOT/isaacsim_test/lerobot/rpo_arm_isaacsim_arm_only.yaml \
  REPORT_JSON_HOST=$REPORT_JSON_HOST \
  REPORT_MD_HOST=$REPORT_MD_HOST \
  python3 - <<'PY'
import json
import os
from datetime import datetime, timezone
from pathlib import Path

report = {
    "status": os.environ["REPORT_STATUS"],
    "reason": os.environ["REPORT_REASON"],
    "exit_status": int(os.environ["REPORT_EXIT_STATUS"]),
    "generated_at_utc": datetime.now(timezone.utc).isoformat(),
    "run_id": os.environ["RUN_ID"],
    "run_dir": os.environ["RUN_DIR_HOST"],
    "logs_dir": os.environ["LOG_DIR_HOST"],
    "screenshots_dir": os.environ["SCREENSHOT_DIR_HOST"],
    "data_dir": os.environ["DATA_DIR_HOST"],
    "config": os.environ["CONFIG_HOST"],
    "hand_contract": "fixed; LeRobot action/state exposes only five arm joints",
    "paths": {
        "isaac_log": os.environ["ISAAC_LOG_HOST"],
        "lerobot_verify_log": os.environ["VERIFY_LOG_HOST"],
        "lerobot_verify_evidence": os.environ["VERIFY_EVIDENCE_HOST"],
        "screenshot": os.environ["SCREENSHOT_PATH_HOST"],
    },
    "checks": {
        "lerobot_verify_status": os.environ.get("VERIFY_STATUS", "unknown"),
        "isaac_sim_status": os.environ.get("SIM_STATUS", "unknown"),
        "screenshot_exists": Path(os.environ["SCREENSHOT_PATH_HOST"]).is_file()
        and Path(os.environ["SCREENSHOT_PATH_HOST"]).stat().st_size > 0,
    },
}
report_path = Path(os.environ["REPORT_JSON_HOST"])
report_path.parent.mkdir(parents=True, exist_ok=True)
report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
Path(os.environ["REPORT_MD_HOST"]).write_text(
    "# Arm Fixed-Hand LeRobot Capture Report\n\n"
    f"- Status: `{report['status']}`\n"
    f"- Reason: {report['reason']}\n"
    f"- Run directory: `{report['run_dir']}`\n"
    f"- Logs: `{report['logs_dir']}`\n"
    f"- Screenshots: `{report['screenshots_dir']}`\n"
    f"- Data: `{report['data_dir']}`\n"
    f"- Config: `{report['config']}`\n"
    f"- Hand contract: {report['hand_contract']}\n"
    f"- Screenshot exists: `{report['checks']['screenshot_exists']}`\n",
    encoding="utf-8",
)
PY
}

cleanup() {
  local exit_status=$?
  if [[ -n "${isaac_pid:-}" ]] && kill -0 "$isaac_pid" >/dev/null 2>&1; then
    (cd "$SCRIPT_DIR" && docker compose stop isaac-sim-51 >/dev/null 2>&1) || true
    wait "$isaac_pid" >/dev/null 2>&1 || true
  fi
  if [[ -d "$RUN_DIR_HOST" ]]; then
    VERIFY_STATUS=$verify_status SIM_STATUS=$sim_status write_report "$exit_status" || true
  fi
}
trap cleanup EXIT

mkdir -p "$LOG_DIR_HOST" "$SCREENSHOT_DIR_HOST" "$DATA_DIR_HOST"
cat > "$SCREENSHOT_DIR_HOST/README.md" <<'EOF'
This directory is reserved for live Isaac Sim viewport screenshots from this run.
For this fixed-hand branch, screenshots should show arm motion only; AmazingHand stays fixed.
EOF

if ! command -v docker >/dev/null 2>&1; then
  report_reason="docker is required for the Isaac/LeRobot container verification"
  echo "$report_reason" >&2
  exit 2
fi
if ! docker compose version >/dev/null 2>&1; then
  report_reason="docker compose plugin is required"
  echo "$report_reason" >&2
  exit 2
fi
if ! docker info >/dev/null 2>&1; then
  report_reason="docker daemon is not reachable"
  echo "$report_reason" >&2
  exit 2
fi

export SUPERARM_WS_PATH ROS_DOMAIN_ID
export NUM_JOINTS=5
export JOINT_NAMES=right_arm_pitch_joint,right_arm_roll_joint,right_arm_yaw_joint,right_elbow_pitch_joint,right_elbow_yaw_joint
export SCREENSHOT_AFTER_COMMAND=1
export SCREENSHOT_ON_STARTUP=0
export SCREENSHOT_PATH=$SCREENSHOT_PATH_CONTAINER
export EXIT_AFTER_SCREENSHOT=1

pushd "$SCRIPT_DIR" >/dev/null
docker compose up --remove-orphans --no-log-prefix isaac-sim-51 >"$ISAAC_LOG_HOST" 2>&1 &
isaac_pid=$!
sleep "$ISAAC_STARTUP_WAIT_S"

verify_cmd=(
  source /opt/ros/humble/setup.bash '&&'
  export PYTHONPATH=/workspace/superarm_ws/isaacsim_test/lerobot:\${PYTHONPATH:-} '&&'
  python3 /workspace/superarm_ws/isaacsim_test/lerobot/verify_lerobot_sitl.py
  --config "$CONFIG_CONTAINER"
  --evidence "$VERIFY_EVIDENCE_CONTAINER"
)
if [[ -n "$TARGET" ]]; then
  verify_cmd+=(--target "$TARGET")
fi

set +e
docker compose run --rm --no-deps --entrypoint /bin/bash lerobot -lc "${verify_cmd[*]}" >"$VERIFY_LOG_HOST" 2>&1
verify_status=$?
set -e

sim_status=0
wait "$isaac_pid" || sim_status=$?
isaac_pid=""
popd >/dev/null

if [[ "$verify_status" -ne 0 ]]; then
  report_reason="LeRobot fixed-hand arm verifier failed with exit status $verify_status; see $VERIFY_LOG_HOST"
  exit "$verify_status"
fi
if [[ "$sim_status" -ne 0 ]]; then
  report_reason="Isaac Sim exited with status $sim_status; see $ISAAC_LOG_HOST"
  exit "$sim_status"
fi
if [[ ! -s "$SCREENSHOT_PATH_HOST" ]]; then
  report_reason="Verifier passed but no non-empty screenshot was written at $SCREENSHOT_PATH_HOST"
  exit 4
fi

report_status="done"
report_reason="LeRobot moved five arm joints with fixed hand and captured a live Isaac screenshot"
VERIFY_STATUS=$verify_status SIM_STATUS=$sim_status write_report 0
printf 'Arm fixed-hand LeRobot run artifacts:\n%s\n' "$RUN_DIR_HOST"
