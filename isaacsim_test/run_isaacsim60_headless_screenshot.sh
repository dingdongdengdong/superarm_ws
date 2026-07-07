#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

: "${SUPERARM_WS_PATH:=$(cd .. && pwd)}"
export SUPERARM_WS_PATH
export HEADLESS="${HEADLESS:-1}"
export SCREENSHOT_ON_STARTUP="${SCREENSHOT_ON_STARTUP:-1}"
export SCREENSHOT_AFTER_COMMAND="${SCREENSHOT_AFTER_COMMAND:-0}"
export SCREENSHOT_PATH="${ISAAC_SIM_60_SCREENSHOT_PATH:-/workspace/superarm_ws/isaacsim_test/artifacts/echo_full_simready_startup_isaacsim60.png}"
export EXIT_AFTER_SCREENSHOT="${EXIT_AFTER_SCREENSHOT:-1}"

mkdir -p artifacts
log_path="artifacts/isaac-sim-60-headless-screenshot.log"
status_path="artifacts/isaacsim60_headless_status.json"

set +e
docker compose up --no-deps --abort-on-container-exit isaac-sim-60 2>&1 | tee "$log_path"
status=${PIPESTATUS[0]}
set -e

printf '{"status":%s,"screenshot_path":"%s","log_path":"%s"}\n' \
  "$status" "$SCREENSHOT_PATH" "$log_path" > "$status_path"

exit "$status"
