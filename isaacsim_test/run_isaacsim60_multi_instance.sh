#!/usr/bin/env bash
set -euo pipefail

# Isaac Sim 6.0 isolated multi-instance runner.
#
# Defaults intentionally avoid the standard 5.x/6.x WebRTC ports so this can run
# beside an already-open Isaac Sim session:
#   signal TCP : 49200
#   stream UDP : 48100
#   web viewer : 8211 (reserved for the official Docker Compose viewer)
#
# Commands:
#   run-hand  Convert and runtime-check AmazingHand in disposable 6.0 containers.
#   start-ui  Start a persistent 6.0 streaming container with custom ports.
#   stop-ui   Stop the persistent streaming container for this instance.
#   status    Show containers and port listeners for this instance.
#   logs      Follow persistent streaming container logs.

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
SUPERARM_WS_PATH=${SUPERARM_WS_PATH:-$(cd -- "$SCRIPT_DIR/.." && pwd)}
ACTION=${1:-run-hand}
INSTANCE=${ISAACSIM60_INSTANCE:-amazinghand}
IMAGE=${ISAAC_SIM_IMAGE:-nvcr.io/nvidia/isaac-sim:6.0.0}
GPU_DEVICE=${GPU_DEVICE:-all}
if [[ "$GPU_DEVICE" == "all" ]]; then
  DOCKER_GPUS=${DOCKER_GPUS:-all}
else
  DOCKER_GPUS=${DOCKER_GPUS:-device=$GPU_DEVICE}
fi
ISAACSIM_HOST=${ISAACSIM_HOST:-127.0.0.1}
ISAACSIM_SIGNAL_PORT=${ISAACSIM_SIGNAL_PORT:-49200}
ISAACSIM_STREAM_PORT=${ISAACSIM_STREAM_PORT:-48100}
WEB_VIEWER_PORT=${WEB_VIEWER_PORT:-8211}
ISAAC_SIM_DATA=${ISAAC_SIM_DATA:-$HOME/docker/isaac-sim-6-${INSTANCE}}
ISAACSIM_HUB_CACHE_PATH=${ISAACSIM_HUB_CACHE_PATH:-$HOME/.cache/ov/hub}
CONTAINER_NAME=${ISAACSIM60_CONTAINER_NAME:-isaacsim6-${INSTANCE}}
RUN_ID=${ROBOT_ARM_HAND_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)_isaacsim60_${INSTANCE}}
OUTPUT_ROOT=${ROBOT_ARM_HAND_OUTPUT_ROOT:-/workspace/superarm_ws/isaacsim_test/outputs/robot_arm_hand_graspable_${RUN_ID}}
SCREENSHOT_ROOT=${ROBOT_ARM_HAND_SCREENSHOT_OUTPUT_DIR:-/workspace/superarm_ws/isaacsim_test/artifacts/robot_arm_hand_graspable_${RUN_ID}}
REPORT_PATH=${ROBOT_ARM_HAND_REPORT_PATH:-$OUTPUT_ROOT/robot_arm_hand_connected_report.json}
LOG_DIR_HOST=${ROBOT_ARM_HAND_LOG_DIR_HOST:-$SUPERARM_WS_PATH/.omx/reports}
LOG_PATH_HOST=${ROBOT_ARM_HAND_RUNTIME_LOG_PATH:-$LOG_DIR_HOST/${RUN_ID}.log}

mkdir -p \
  "$ISAAC_SIM_DATA/cache/main" \
  "$ISAAC_SIM_DATA/cache/computecache" \
  "$ISAAC_SIM_DATA/config" \
  "$ISAAC_SIM_DATA/data" \
  "$ISAAC_SIM_DATA/logs" \
  "$ISAAC_SIM_DATA/pkg" \
  "$ISAACSIM_HUB_CACHE_PATH" \
  "$LOG_DIR_HOST" \
  "$SUPERARM_WS_PATH/${OUTPUT_ROOT#/workspace/superarm_ws/}" \
  "$SUPERARM_WS_PATH/${SCREENSHOT_ROOT#/workspace/superarm_ws/}"
chmod -R u+rwX,go+rwX "$ISAAC_SIM_DATA" "$ISAACSIM_HUB_CACHE_PATH" 2>/dev/null || true

print_config() {
  cat <<EOF
Isaac Sim 6.0 multi-instance config
  action:              $ACTION
  instance:            $INSTANCE
  image:               $IMAGE
  container:           $CONTAINER_NAME
  gpu:                 $GPU_DEVICE
  docker --gpus:       $DOCKER_GPUS
  host:                $ISAACSIM_HOST
  signal port (tcp):   $ISAACSIM_SIGNAL_PORT
  stream port (udp):   $ISAACSIM_STREAM_PORT
  web viewer port:     $WEB_VIEWER_PORT (reserved for official compose web-viewer)
  data root:           $ISAAC_SIM_DATA
  hub cache:           $ISAACSIM_HUB_CACHE_PATH
  output root:         $OUTPUT_ROOT
  screenshot root:     $SCREENSHOT_ROOT
  log:                 $LOG_PATH_HOST
EOF
}

port_status() {
  if command -v ss >/dev/null 2>&1; then
    ss -ltnup 2>/dev/null | grep -E ":(${ISAACSIM_SIGNAL_PORT}|${ISAACSIM_STREAM_PORT}|${WEB_VIEWER_PORT})\\b" || true
  else
    netstat -ltnup 2>/dev/null | grep -E ":(${ISAACSIM_SIGNAL_PORT}|${ISAACSIM_STREAM_PORT}|${WEB_VIEWER_PORT})\\b" || true
  fi
}

run_python_phase() {
  local phase=$1
  docker run --rm --gpus "$DOCKER_GPUS" --network host --ipc host \
    --name "${CONTAINER_NAME}-${phase}" \
    --user 0:0 \
    -e ACCEPT_EULA=Y \
    -e PRIVACY_CONSENT=Y \
    -e HEADLESS=1 \
    -e PYTHONUNBUFFERED=1 \
    -e ROBOT_ARM_HAND_ISAAC_FAST_CLOSE=1 \
    -e ISAACSIM_HOST="$ISAACSIM_HOST" \
    -e ISAACSIM_SIGNAL_PORT="$ISAACSIM_SIGNAL_PORT" \
    -e ISAACSIM_STREAM_PORT="$ISAACSIM_STREAM_PORT" \
    -e ROBOT_ARM_HAND_ZIP_SOURCE="${ROBOT_ARM_HAND_ZIP_SOURCE:-/workspace/superarm_ws/robot_arm_hand_package.zip}" \
    -e ROBOT_ARM_HAND_INPUT_ROOT="${ROBOT_ARM_HAND_INPUT_ROOT:-/workspace/superarm_ws/isaacsim_test/inputs/robot_arm_hand_package}" \
    -e ROBOT_ARM_HAND_OUTPUT_ROOT="$OUTPUT_ROOT" \
    -e ROBOT_ARM_HAND_CONNECTED_USD_PATH="${ROBOT_ARM_HAND_CONNECTED_USD_PATH:-$OUTPUT_ROOT/robot_arm_hand_connected.usd}" \
    -e ROBOT_ARM_HAND_REPORT_PATH="$REPORT_PATH" \
    -e ROBOT_ARM_HAND_SCREENSHOT_OUTPUT_DIR="$SCREENSHOT_ROOT" \
    -e ROBOT_ARM_HAND_SETTLE_STEPS="${ROBOT_ARM_HAND_SETTLE_STEPS:-20}" \
    -e ROBOT_ARM_HAND_GRASP_STEPS="${ROBOT_ARM_HAND_GRASP_STEPS:-90}" \
    -e ROBOT_ARM_HAND_FINGER_MOTION_STEPS="${ROBOT_ARM_HAND_FINGER_MOTION_STEPS:-45}" \
    -e ROBOT_ARM_HAND_LIFT_RETAIN_STEPS="${ROBOT_ARM_HAND_LIFT_RETAIN_STEPS:-75}" \
    -e ROBOT_ARM_HAND_SHOW_CONTACT_PROXIES="${ROBOT_ARM_HAND_SHOW_CONTACT_PROXIES:-0}" \
    -v "$SUPERARM_WS_PATH:/workspace/superarm_ws:rw" \
    -v "$SCRIPT_DIR/isaacsim:/workspace/isaacsim:ro" \
    -v "$ISAAC_SIM_DATA/cache/main:/isaac-sim/.cache:rw" \
    -v "$ISAAC_SIM_DATA/cache/computecache:/isaac-sim/.nv/ComputeCache:rw" \
    -v "$ISAAC_SIM_DATA/logs:/isaac-sim/.nvidia-omniverse/logs:rw" \
    -v "$ISAAC_SIM_DATA/config:/isaac-sim/.nvidia-omniverse/config:rw" \
    -v "$ISAAC_SIM_DATA/data:/isaac-sim/.local/share/ov/data:rw" \
    -v "$ISAAC_SIM_DATA/pkg:/isaac-sim/.local/share/ov/pkg:rw" \
    -v "$ISAACSIM_HUB_CACHE_PATH:/var/cache/hub:rw" \
    --entrypoint /bin/bash \
    "$IMAGE" \
    -lc "cd /workspace/superarm_ws && exec /isaac-sim/python.sh /workspace/isaacsim/robot_arm_hand_from_zip.py --mode ${phase} --headless"
}

case "$ACTION" in
  run-hand)
    print_config | tee "$LOG_PATH_HOST"
    {
      echo "[run-hand] phase=convert"
      run_python_phase convert
      echo "[run-hand] phase=runtime"
      run_python_phase runtime
      echo "[run-hand] report candidates"
      find "$SUPERARM_WS_PATH/${OUTPUT_ROOT#/workspace/superarm_ws/}" -maxdepth 2 -type f -name '*report*.json' -printf '%p %s bytes\n' | sort || true
    } 2>&1 | tee -a "$LOG_PATH_HOST"
    ;;
  start-ui)
    print_config
    docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
    docker run -d --gpus "$DOCKER_GPUS" --network host --ipc host \
      --name "$CONTAINER_NAME" \
      -e ACCEPT_EULA=Y \
      -e PRIVACY_CONSENT=Y \
      -e NVIDIA_VISIBLE_DEVICES="$GPU_DEVICE" \
      -e ISAACSIM_HOST="$ISAACSIM_HOST" \
      -e ISAACSIM_SIGNAL_PORT="$ISAACSIM_SIGNAL_PORT" \
      -e ISAACSIM_STREAM_PORT="$ISAACSIM_STREAM_PORT" \
      -v "$ISAAC_SIM_DATA/cache/main:/isaac-sim/.cache:rw" \
      -v "$ISAAC_SIM_DATA/cache/computecache:/isaac-sim/.nv/ComputeCache:rw" \
      -v "$ISAAC_SIM_DATA/logs:/isaac-sim/.nvidia-omniverse/logs:rw" \
      -v "$ISAAC_SIM_DATA/config:/isaac-sim/.nvidia-omniverse/config:rw" \
      -v "$ISAAC_SIM_DATA/data:/isaac-sim/.local/share/ov/data:rw" \
      -v "$ISAAC_SIM_DATA/pkg:/isaac-sim/.local/share/ov/pkg:rw" \
      -v "$ISAACSIM_HUB_CACHE_PATH:/var/cache/hub:rw" \
      "$IMAGE"
    echo "Started $CONTAINER_NAME. Native/WebRTC signal=$ISAACSIM_SIGNAL_PORT stream=$ISAACSIM_STREAM_PORT."
    echo "For browser web-viewer, use NVIDIA's official Docker Compose web-viewer with WEB_VIEWER_PORT=$WEB_VIEWER_PORT."
    ;;
  stop-ui)
    docker rm -f "$CONTAINER_NAME"
    ;;
  status)
    print_config
    echo
    docker ps --format '{{.ID}} {{.Image}} {{.Names}} {{.Status}} {{.Ports}}' | grep -E "($CONTAINER_NAME|isaacsim6-)" || true
    echo
    port_status
    ;;
  logs)
    docker logs -f "$CONTAINER_NAME"
    ;;
  *)
    echo "Usage: $0 {run-hand|start-ui|stop-ui|status|logs}" >&2
    exit 64
    ;;
esac
