#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LEROBOT_PATH="${LEROBOT_PATH:-${WS_DIR}/lerobot}"
PHONE_PORT="${PHONE_PORT:-8766}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
NUM_JOINTS="${NUM_JOINTS:-7}"    # Franka has 7 DOF; update if using a 6-DOF arm

# Recording settings — override via environment or .env
REPO_ID="${REPO_ID:-local/openarm_isaacsim_v1}"
NUM_EPISODES="${NUM_EPISODES:-10}"
EPISODE_TIME_S="${EPISODE_TIME_S:-30}"
PUSH_TO_HUB="${PUSH_TO_HUB:-false}"
FPS="${FPS:-30}"

export ROS_DOMAIN_ID
export PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH:-}"

LOCAL_IP=$(hostname -I | awk '{print $1}')

echo "=========================================="
echo "  Isaac Sim OpenArm Smartphone Teleop"
echo "  Teleop UI : http://${LOCAL_IP}:${PHONE_PORT}"
echo "  (phone must be on the same WiFi/LAN)"
echo "  Recording : ${REPO_ID}"
echo "              ${NUM_EPISODES} episodes x ${EPISODE_TIME_S}s @ ${FPS}fps"
echo "=========================================="
echo ""

# --- Start phone web server ---
python3 "${SCRIPT_DIR}/phone_teleop_server.py" \
    --port "${PHONE_PORT}" \
    --num-joints "${NUM_JOINTS}" \
    --topic "/leader/joint_commands" \
    --ros-domain-id "${ROS_DOMAIN_ID}" &
SERVER_PID=$!

cleanup() {
    echo "[run_smartphone_teleop] Shutting down..."
    kill "${SERVER_PID}" 2>/dev/null || true
    kill "${RECORD_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# --- Wait for Isaac Sim to publish joint states ---
echo "Waiting for Isaac Sim (/follower/joint_states) ..."
TIMEOUT=60
ELAPSED=0
until ros2 topic list 2>/dev/null | grep -q '/follower/joint_states'; do
    if [ "${ELAPSED}" -ge "${TIMEOUT}" ]; then
        echo "ERROR: /follower/joint_states not found after ${TIMEOUT}s."
        echo "  → Is isaac-sim-51 running?  docker compose up isaac-sim-51"
        echo "  → ROS_DOMAIN_ID=${ROS_DOMAIN_ID} — must match on both sides."
        exit 1
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
done
echo "Isaac Sim ready."
echo ""

# --- Auto-start recording via LeRobot control_robot.py ---
echo "Starting LeRobot recorder..."
cd "${LEROBOT_PATH}"
python lerobot/scripts/control_robot.py \
    --robot.type=isaacsim_openarm \
    --control.type=record \
    --control.repo_id="${REPO_ID}" \
    --control.num_episodes="${NUM_EPISODES}" \
    --control.episode_time_s="${EPISODE_TIME_S}" \
    --control.fps="${FPS}" \
    --control.push_to_hub="${PUSH_TO_HUB}" \
    --control.play_sounds=false &
RECORD_PID=$!

echo ""
echo "=========================================="
echo "  LIVE — open on your phone:"
echo "  http://${LOCAL_IP}:${PHONE_PORT}"
echo ""
echo "  Move sliders → arm moves in Isaac Sim"
echo "  Recording auto-saves every episode."
echo "  Episodes saved to: ${REPO_ID}"
echo "=========================================="

wait "${RECORD_PID}"
