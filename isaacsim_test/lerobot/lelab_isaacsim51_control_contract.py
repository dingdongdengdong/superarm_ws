"""Isaac Sim 5.1 LeLab SuperArm contract and artifact helpers.

This module is intentionally dependency-light so the 5DOF + grasp LeLab control contract can
be verified on the ROS2/LeLab host before a full Isaac Sim process is started.
Live screenshots remain separate evidence; the PNG generated here documents the
LeLab SuperArm server/case matrix used to drive the sim.
"""
from __future__ import annotations

import json
import re
import struct
import zlib
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REQUIRED_ISAACSIM_MAJOR_MINOR = "5.1"
DEFAULT_CONFIG_PATH = "/workspaces/superarm_ws/isaacsim_test/lerobot/rpo_arm_isaacsim.yaml"
DEFAULT_ARTIFACT_PREFIX = "lelab_isaacsim51_control"


@dataclass(frozen=True)
class ControlField:
    name: str
    label: str
    minimum: float
    maximum: float
    step: float
    default: float = 0.0


@dataclass(frozen=True)
class ControlCase:
    name: str
    target: tuple[float, float, float, float, float, float]
    purpose: str


CONTROL_FIELDS: tuple[ControlField, ...] = (
    ControlField("right_arm_pitch_joint", "Arm pitch", -1.57, 1.57, 0.01),
    ControlField("right_arm_roll_joint", "Arm roll", -1.57, 1.57, 0.01),
    ControlField("right_arm_yaw_joint", "Arm yaw", -1.57, 1.57, 0.01),
    ControlField("right_elbow_pitch_joint", "Elbow pitch", -1.57, 1.57, 0.01),
    ControlField("right_elbow_yaw_joint", "Elbow yaw", -1.57, 1.57, 0.01),
    ControlField("amazinghand_grasp", "AmazingHand grasp", 0.0, 1.0, 0.01),
)

FIVE_DOF_GRASP_TEST_CASES: tuple[ControlCase, ...] = (
    ControlCase("pitch_positive", (0.35, 0.0, 0.0, 0.0, 0.0, 0.0), "exercise right arm pitch DOF"),
    ControlCase("roll_negative", (0.0, -0.35, 0.0, 0.0, 0.0, 0.0), "exercise right arm roll DOF"),
    ControlCase("yaw_positive", (0.0, 0.0, 0.35, 0.0, 0.0, 0.0), "exercise right arm yaw DOF"),
    ControlCase("elbow_pitch_positive", (0.0, 0.0, 0.0, 0.35, 0.0, 0.0), "exercise right elbow pitch DOF"),
    ControlCase("elbow_yaw_negative", (0.0, 0.0, 0.0, 0.0, -0.35, 0.0), "exercise right elbow yaw DOF"),
    ControlCase("grasp_close", (0.0, 0.0, 0.0, 0.0, 0.0, 1.0), "exercise AmazingHand grasp scalar"),
)


def _utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _parse_major_minor(version: str) -> str | None:
    match = re.search(r"(\d+)\.(\d+)", version)
    if not match:
        return None
    return f"{match.group(1)}.{match.group(2)}"


def detect_isaacsim_compatibility(version_file: Path = Path("/workspace/isaacsim/VERSION")) -> dict[str, Any]:
    detected_version = "missing"
    if version_file.is_file():
        detected_version = version_file.read_text(encoding="utf-8").strip()
    detected_major_minor = _parse_major_minor(detected_version)
    return {
        "required_major_minor": REQUIRED_ISAACSIM_MAJOR_MINOR,
        "detected_version": detected_version,
        "detected_major_minor": detected_major_minor,
        "compatible": detected_major_minor == REQUIRED_ISAACSIM_MAJOR_MINOR,
        "compatibility_note": (
            "Isaac Sim 5.1 uses the same 6-field LeRobot/ROS Float64MultiArray contract "
            "as the earlier 6.0-oriented notes for this project; this check gates the local "
            "runtime to the installed 5.1 version."
        ),
    }


def _json_for_js(value: Any) -> str:
    return json.dumps(value, separators=(",", ":"))


def render_lelab_superarm_html_snapshot(config_path: str = DEFAULT_CONFIG_PATH) -> str:
    fields = [asdict(field) for field in CONTROL_FIELDS]
    cases = [asdict(case) | {"target": list(case.target)} for case in FIVE_DOF_GRASP_TEST_CASES]
    buttons = "\n".join(
        f'<button onclick="preset({ _json_for_js(list(case.target)) })">{case.name}</button>'
        for case in FIVE_DOF_GRASP_TEST_CASES
    )
    slider_rows = "\n".join(
        f'<div class="row"><label>{field.name}</label>'
        f'<input id="j{idx}" class="range-control" type="range" min="{field.minimum}" max="{field.maximum}" '
        f'step="{field.step}" value="{field.default}" oninput="v{idx}.textContent=this.value">'
        f'<span id="v{idx}">{field.default}</span></div>'
        for idx, field in enumerate(CONTROL_FIELDS)
    )
    return f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <title>LeLab SuperArm Isaac Sim 5.1 Control</title>
  <style>
    body {{ font-family: system-ui, sans-serif; margin: 2rem; max-width: 1100px; }}
    .row {{ display: grid; grid-template-columns: 180px 1fr 90px; gap: 1rem; align-items: center; margin: .7rem 0; }}
    input.range-control {{ width: 100%; }}
    button {{ padding: .55rem .8rem; margin: .25rem; }}
    pre {{ background: #111; color: #eee; padding: 1rem; overflow: auto; }}
  </style>
</head>
<body>
  <h1>LeLab SuperArm Isaac Sim 5.1 Control</h1>
  <p>Backend: <code>isaacsim_rpo_arm</code>; config: <code>{config_path}</code></p>
  <p>Control contract: five arm DOF + one normalized AmazingHand grasp scalar.</p>
  <button onclick="connectArm()">Connect Isaac Sim Arm</button>
  <button onclick="sendAction()">Send Slider Action</button>
  {buttons}
  <button onclick="stopArm()">Stop</button>
  <div id="sliders">
  {slider_rows}
  </div>
  <h2>Status</h2>
  <pre id="out">Not connected</pre>
<script>
const fields = {_json_for_js(fields)};
const cases = {_json_for_js(cases)};
function values() {{ return fields.map((_, i) => parseFloat(document.getElementById(`j${{i}}`).value)); }}
function log(obj) {{ document.getElementById("out").textContent = JSON.stringify(obj, null, 2); }}
async function post(path, body) {{ const r = await fetch(path, {{method:"POST", headers:{{"content-type":"application/json"}}, body: JSON.stringify(body)}}); const j = await r.json(); log(j); return j; }}
async function connectArm() {{ return post('/move-arm', {{robot_backend:'isaacsim_rpo_arm', leader_port:'unused', follower_port:'unused', leader_config:'unused', follower_config:'{config_path}', superarm_ws_path:'/workspaces/superarm_ws'}}); }}
async function sendAction() {{ return post('/send-joint-action', {{action: values()}}); }}
async function preset(vals) {{ vals.forEach((v,i)=>{{document.getElementById(`j${{i}}`).value=v; document.getElementById(`v${{i}}`).textContent=v;}}); return post('/send-joint-action', {{action: vals}}); }}
async function stopArm() {{ return post('/stop-teleoperation', {{}}); }}
</script>
</body>
</html>
"""


def create_timestamped_artifact_root(base_dir: Path, now_utc: str | None = None) -> Path:
    stamp = now_utc or _utc_stamp()
    if not re.fullmatch(r"\d{8}T\d{6}Z", stamp):
        raise ValueError("Artifact timestamp must use UTC date-time format YYYYMMDDTHHMMSSZ")
    root = base_dir / f"{DEFAULT_ARTIFACT_PREFIX}_{stamp}"
    (root / "logs").mkdir(parents=True, exist_ok=True)
    (root / "data").mkdir(parents=True, exist_ok=True)
    (root / "screenshots").mkdir(parents=True, exist_ok=True)
    return root


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    return struct.pack("!I", len(payload)) + kind + payload + struct.pack("!I", zlib.crc32(kind + payload) & 0xFFFFFFFF)


def _write_png(path: Path, width: int, height: int, rgb_rows: list[bytes]) -> None:
    raw = b"".join(b"\x00" + row for row in rgb_rows)
    png = b"\x89PNG\r\n\x1a\n"
    png += _png_chunk(b"IHDR", struct.pack("!IIBBBBB", width, height, 8, 2, 0, 0, 0))
    png += _png_chunk(b"IDAT", zlib.compress(raw, 9))
    png += _png_chunk(b"IEND", b"")
    path.write_bytes(png)


def write_lelab_superarm_png(path: Path) -> None:
    """Write a simple PNG matrix proving six LeLab controls and six cases were generated."""
    width, height = 960, 540
    bg = (245, 247, 250)
    grid = (210, 220, 230)
    colors = [
        (46, 134, 193),
        (142, 68, 173),
        (39, 174, 96),
        (230, 126, 34),
        (192, 57, 43),
        (44, 62, 80),
    ]
    pixels = [[bg for _ in range(width)] for _ in range(height)]

    def rect(x0: int, y0: int, x1: int, y1: int, color: tuple[int, int, int]) -> None:
        for y in range(max(y0, 0), min(y1, height)):
            row = pixels[y]
            for x in range(max(x0, 0), min(x1, width)):
                row[x] = color

    # Header stripes: 6 controls.
    margin_x = 80
    top = 72
    cell_w = 120
    for i, color in enumerate(colors):
        x = margin_x + i * (cell_w + 12)
        rect(x, 30, x + cell_w, 56, color)
        rect(x, top, x + cell_w, top + 360, (255, 255, 255))
        rect(x, top, x + 2, top + 360, grid)
        rect(x + cell_w - 2, top, x + cell_w, top + 360, grid)

    # Rows: 6 one-axis test cases. Fill the active cell with proportional bars.
    row_h = 52
    for case_i, case in enumerate(FIVE_DOF_GRASP_TEST_CASES):
        y = top + 20 + case_i * row_h
        rect(25, y, 62, y + 28, colors[case_i])
        for field_i, value in enumerate(case.target):
            x = margin_x + field_i * (cell_w + 12) + 12
            rect(x, y, x + cell_w - 24, y + 28, (232, 238, 244))
            if abs(value) > 1e-9:
                field = CONTROL_FIELDS[field_i]
                span = field.maximum - field.minimum
                normalized = 0.0 if span == 0 else (value - field.minimum) / span
                bar_w = max(6, int((cell_w - 24) * max(0.0, min(1.0, normalized))))
                rect(x, y, x + bar_w, y + 28, colors[field_i])

    # Footer blocks encode compatibility and artifact semantics.
    rect(80, 465, 370, 500, (30, 130, 76))
    rect(405, 465, 690, 500, (35, 91, 148))
    rect(725, 465, 920, 500, (125, 60, 152))

    rows = [bytes(channel for pixel in row for channel in pixel) for row in pixels]
    path.parent.mkdir(parents=True, exist_ok=True)
    _write_png(path, width, height, rows)


def write_contract_artifacts(root: Path, version_file: Path = Path("/workspace/isaacsim/VERSION")) -> dict[str, Any]:
    root.mkdir(parents=True, exist_ok=True)
    for subdir in ("logs", "data", "screenshots"):
        (root / subdir).mkdir(parents=True, exist_ok=True)

    compatibility = detect_isaacsim_compatibility(version_file)
    cases_payload = {
        "control_fields": [asdict(field) for field in CONTROL_FIELDS],
        "cases": [asdict(case) | {"target": list(case.target)} for case in FIVE_DOF_GRASP_TEST_CASES],
    }
    cases_path = root / "data" / "five_dof_grasp_cases.json"
    cases_path.write_text(json.dumps(cases_payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    html_path = root / "lelab_superarm_control.html"
    html_path.write_text(render_lelab_superarm_html_snapshot(), encoding="utf-8")

    png_path = root / "screenshots" / "lelab_superarm_control_verification.png"
    write_lelab_superarm_png(png_path)

    log_path = root / "logs" / "lelab_superarm_contract.log"
    log_path.write_text(
        "Isaac Sim 5.1 LeLab SuperArm contract verification\n"
        f"compatible={compatibility['compatible']}\n"
        f"detected_version={compatibility['detected_version']}\n"
        f"control_count={len(CONTROL_FIELDS)}\n"
        f"test_case_count={len(FIVE_DOF_GRASP_TEST_CASES)}\n"
        f"lelab_superarm_png_evidence_path={png_path}\n",
        encoding="utf-8",
    )

    report = {
        "status": "done" if compatibility["compatible"] and png_path.stat().st_size > 0 else "failed",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "artifact_root": str(root),
        "compatibility": compatibility,
        "control_count": len(CONTROL_FIELDS),
        "test_case_count": len(FIVE_DOF_GRASP_TEST_CASES),
        "cases_path": str(cases_path),
        "lelab_superarm_html_path": str(html_path),
        "lelab_superarm_png_evidence_path": str(png_path),
        "logs_dir": str(root / "logs"),
        "note": (
            "This verifies the LeLab SuperArm contract and 6 command vectors. "
            "Use live Isaac command/readback evidence to prove physical/sim motion."
        ),
    }
    report_path = root / "report.json"
    report["report_path"] = str(report_path)
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report
