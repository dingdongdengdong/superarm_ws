#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
SUPERARM_WS_PATH=${SUPERARM_WS_PATH:-$(cd -- "$SCRIPT_DIR/.." && pwd)}
OUTPUT_DIR=${MOTION_SCREENSHOT_OUTPUT_DIR:-/workspace/superarm_ws/isaacsim_test/artifacts/simready_motion_cases}
SETTLE_STEPS=${MOTION_SCREENSHOT_SETTLE_STEPS:-30}
PHYSICAL_URDF=${PHYSICAL_ROBOT_URDF_PATH:-/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/sitl/roboto_v2_right_arm_amazinghand_full.urdf}
CUSTOM_VISUAL_USD=${CUSTOM_VISUAL_USD_PATH:-/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/sitl/echo_full_lerobot_articulation.usda}
CUSTOM_VISUAL_PRIM=${CUSTOM_VISUAL_PRIM_PATH:-/World/echo_full_visual}
REPORT_PATH=${SIMREADY_ARTICULATION_REPORT_PATH:-/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/sitl/echo_full_lerobot_articulation_report.json}
CONTACT_SHEET_PATH=${MOTION_SCREENSHOT_CONTACT_SHEET_PATH:-/workspace/superarm_ws/isaacsim_test/artifacts/simready_motion_cases_contact_sheet.png}
LOG_STAMP=${MOTION_SCREENSHOT_LOG_STAMP:-$(date -u +%Y%m%dT%H%M%SZ)}
RUNTIME_LOG_CONTAINER=${ISAAC_SIM_RUNTIME_LOG_PATH:-/workspace/superarm_ws/isaacsim_test/artifacts/runtime_logs/direct_urdf_motion_${LOG_STAMP}.log}
RUNTIME_LOG_HOST="$SUPERARM_WS_PATH/${RUNTIME_LOG_CONTAINER#/workspace/superarm_ws/}"

container_to_host() {
  local path="$1"
  if [[ "$path" == /workspace/superarm_ws/* ]]; then
    printf '%s/%s' "$SUPERARM_WS_PATH" "${path#/workspace/superarm_ws/}"
  else
    printf '%s' "$path"
  fi
}

cd "$SCRIPT_DIR"
mkdir -p artifacts "$(dirname "$RUNTIME_LOG_HOST")"

set -o pipefail
SUPERARM_WS_PATH="$SUPERARM_WS_PATH" docker compose run --rm --no-deps \
  -e HEADLESS=${HEADLESS:-1} \
  -e USE_SIMREADY_USD=0 \
  -e LOAD_CUSTOM_VISUAL_USD=${LOAD_CUSTOM_VISUAL_USD:-1} \
  -e CUSTOM_VISUAL_USD_PATH="$CUSTOM_VISUAL_USD" \
  -e CUSTOM_VISUAL_PRIM_PATH="$CUSTOM_VISUAL_PRIM" \
  -e CUSTOM_VISUAL_FOLLOW_LINK="${CUSTOM_VISUAL_FOLLOW_LINK:-}" \
  -e CUSTOM_VISUAL_FOLLOW_XYZ="${CUSTOM_VISUAL_FOLLOW_XYZ:-0 0 0}" \
  -e PHYSICAL_ROBOT_URDF_PATH="$PHYSICAL_URDF" \
  -e SIMREADY_ARTICULATION_REPORT_PATH="$REPORT_PATH" \
  -e ISAAC_SIM_RUNTIME_LOG_PATH="$RUNTIME_LOG_CONTAINER" \
  -e MOTION_SCREENSHOT_CONTACT_SHEET_PATH="$CONTACT_SHEET_PATH" \
  -e SCREENSHOT_ON_STARTUP=0 \
  -e SCREENSHOT_AFTER_COMMAND=0 \
  -e EXIT_AFTER_SCREENSHOT=0 \
  -e MOTION_SCREENSHOT_CASES_PATH=/workspace/superarm_ws/isaacsim_test/simready_motion_cases.json \
  -e MOTION_SCREENSHOT_CASES_JSON= \
  -e MOTION_SCREENSHOT_OUTPUT_DIR="$OUTPUT_DIR" \
  -e MOTION_SCREENSHOT_SETTLE_STEPS="$SETTLE_STEPS" \
  -e MOTION_SCREENSHOT_KINEMATIC_CAPTURE=${MOTION_SCREENSHOT_KINEMATIC_CAPTURE:-0} \
  -e EXIT_AFTER_MOTION_SCREENSHOTS=1 \
  isaac-sim-51 2>&1 | tee "$RUNTIME_LOG_HOST"

HOST_OUTPUT_DIR=$(container_to_host "$OUTPUT_DIR")
HOST_CONTACT_SHEET=$(container_to_host "$CONTACT_SHEET_PATH")
HOST_REPORT_PATH=$(container_to_host "$REPORT_PATH")
python3 - "$HOST_OUTPUT_DIR" "$HOST_CONTACT_SHEET" "$HOST_REPORT_PATH" "$CONTACT_SHEET_PATH" <<'PY'
import json
import sys
from pathlib import Path

from PIL import Image, ImageDraw

image_dir = Path(sys.argv[1])
contact_sheet = Path(sys.argv[2])
report_path = Path(sys.argv[3])
contact_sheet_container = sys.argv[4]
images = sorted(image_dir.glob("*.png"))
if not images:
    raise RuntimeError(f"No motion screenshot PNGs found in {image_dir}")

thumb_w, thumb_h = 480, 270
label_h = 28
sheet = Image.new("RGB", (thumb_w * 2, (thumb_h + label_h) * 2), "white")
draw = ImageDraw.Draw(sheet)
for index, image_path in enumerate(images[:4]):
    image = Image.open(image_path).convert("RGB")
    image.thumbnail((thumb_w, thumb_h))
    col, row = index % 2, index // 2
    x = col * thumb_w + (thumb_w - image.width) // 2
    y = row * (thumb_h + label_h)
    sheet.paste(image, (x, y))
    draw.text((col * thumb_w + 8, y + thumb_h + 6), image_path.name, fill=(0, 0, 0))

contact_sheet.parent.mkdir(parents=True, exist_ok=True)
sheet.save(contact_sheet)
print(f"[motion-runner] Contact sheet saved: {contact_sheet}")

if report_path.is_file():
    report = json.loads(report_path.read_text(encoding="utf-8"))
    runtime_validation = report.setdefault("runtime_validation", {})
    if isinstance(runtime_validation, dict):
        runtime_validation["contact_sheet"] = contact_sheet_container.replace(
            "/workspace/superarm_ws/", ""
        )
        try:
            report_path.write_text(
                json.dumps(report, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        except PermissionError as exc:
            print(
                "[motion-runner] WARNING: could not update runtime report contact sheet "
                f"for {report_path}: {exc}"
            )
        else:
            print(f"[motion-runner] Runtime report contact sheet updated: {report_path}")
PY
