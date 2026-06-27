#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PHONE_PORT="${PHONE_PORT:-8766}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
NUM_JOINTS="${NUM_JOINTS:-6}"

export ROS_DOMAIN_ID
export PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH:-}"

LOCAL_IP=$(hostname -I | awk '{print $1}')

echo "=========================================="
echo "  Isaac Sim OpenArm Smartphone Teleop"
echo "  Open on your phone: http://${LOCAL_IP}:${PHONE_PORT}"
echo "  (phone must be on the same WiFi network)"
echo "=========================================="
echo ""

# Start the phone web server in background
python3 "${SCRIPT_DIR}/phone_teleop_server.py" \
    --port "${PHONE_PORT}" \
    --num-joints "${NUM_JOINTS}" \
    --topic "/leader/joint_commands" \
    --ros-domain-id "${ROS_DOMAIN_ID}" &
SERVER_PID=$!

cleanup() {
    echo ""
    echo "[run_smartphone_teleop] Stopping phone server (PID ${SERVER_PID})..."
    kill "${SERVER_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Wait for Isaac Sim to publish joint states
echo "Waiting for /follower/joint_states from Isaac Sim (up to 30 s)..."
TIMEOUT=30
ELAPSED=0
until ros2 topic list 2>/dev/null | grep -q '/follower/joint_states'; do
    if [ "${ELAPSED}" -ge "${TIMEOUT}" ]; then
        echo ""
        echo "ERROR: /follower/joint_states not found after ${TIMEOUT}s."
        echo "  → Start Isaac Sim first:  docker compose up isaac-sim-45"
        echo "  → Check ROS_DOMAIN_ID matches in both containers (currently: ${ROS_DOMAIN_ID})"
        exit 1
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
done

echo "Isaac Sim topics found. Teleoperation is active."
echo ""
echo "Move sliders on your phone to control the arm in Isaac Sim."
echo ""
echo "To RECORD episodes, open a new terminal and run:"
echo "  docker exec isaacsim-test-lerobot bash -c \\"
echo "    'source /opt/ros/humble/setup.bash && \\"
echo "     cd /workspace/superarm_ws/lerobot && \\"
echo "     python lerobot/scripts/control_robot.py \\"
echo "       --robot.type=isaacsim_openarm \\"
echo "       --control.type=record \\"
echo "       --control.repo_id=YOUR_HF_USER/openarm_isaacsim_v1 \\"
echo "       --control.fps=30 \\"
echo "       --control.num_episodes=5'"
echo ""

wait "${SERVER_PID}"
