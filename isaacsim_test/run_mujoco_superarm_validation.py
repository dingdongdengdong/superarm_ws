"""Create numeric and close-up visual evidence for the combined MuJoCo model."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from isaacsim_test.mujoco_models.generate_superarm_amazinghand import (  # noqa: E402
    ARM_JOINT_NAMES,
    HAND_ACTUATOR_NAMES,
    generate_combined_model,
)

def _timestamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _step(model, data, count: int = 1200) -> None:
    import mujoco

    for _ in range(count):
        mujoco.mj_step(model, data)


def _jpeg_stats(path: Path) -> dict[str, float | int]:
    image = np.asarray(Image.open(path).convert("RGB"))
    return {
        "width": int(image.shape[1]),
        "height": int(image.shape[0]),
        "min": int(image.min()),
        "max": int(image.max()),
        "stddev": float(image.std()),
        "nonblank": bool(image.std() > 2.0 and image.max() > image.min()),
    }


def run(output_root: Path) -> dict:
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco

    output_root.mkdir(parents=True, exist_ok=True)
    model_report = generate_combined_model(
        workspace_root=ROOT,
        output_dir=output_root / "model",
    )
    model = mujoco.MjModel.from_xml_path(model_report["model_path"])
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=480, width=640)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    hand_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "r_wrist_interface")

    actuator_ids = {
        name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in [*ARM_JOINT_NAMES, *HAND_ACTUATOR_NAMES]
    }
    qpos_addresses = {}
    for name in [*ARM_JOINT_NAMES, *HAND_ACTUATOR_NAMES]:
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        qpos_addresses[name] = int(model.jnt_qposadr[joint])

    commands: list[dict] = []
    for name in ARM_JOINT_NAMES:
        mujoco.mj_resetData(model, data)
        data.ctrl[:] = 0.0
        data.ctrl[actuator_ids[name]] = 0.25
        _step(model, data)
        readback = float(data.qpos[qpos_addresses[name]])
        commands.append(
            {
                "subsystem": "arm",
                "name": name,
                "target_rad": 0.25,
                "readback_rad": readback,
                "finite": bool(np.isfinite(data.qpos).all()),
                "passed": bool(abs(readback - 0.25) < 0.08),
            }
        )

    render_records: list[dict] = []
    for finger in range(1, 5):
        pair = [f"finger{finger}_motor1", f"finger{finger}_motor2"]
        focus_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, pair[0])
        focus_body = int(model.jnt_bodyid[focus_joint])
        mujoco.mj_resetData(model, data)
        for index in range(1, 5):
            data.ctrl[actuator_ids[f"finger{index}_motor1"]] = 0.05
            data.ctrl[actuator_ids[f"finger{index}_motor2"]] = 0.02
        _step(model, data, 600)
        camera.lookat[:] = (data.xpos[hand_body] + data.xpos[focus_body]) / 2.0
        camera.distance = 0.22
        camera.azimuth = 135
        camera.elevation = -15
        renderer.update_scene(data, camera=camera)
        before_path = output_root / f"finger{finger}_before_open.jpg"
        Image.fromarray(renderer.render()).save(before_path, quality=90)

        raw_target = [0.50, 0.56]
        for name, value in zip(pair, raw_target, strict=True):
            data.ctrl[actuator_ids[name]] = value
        _step(model, data, 800)
        raw_readback = [float(data.qpos[qpos_addresses[name]]) for name in pair]
        commands.append(
            {
                "subsystem": "hand",
                "finger": finger,
                "mode": "raw",
                "actuators": pair,
                "target_rad": raw_target,
                "readback_rad": raw_readback,
                "finite": bool(np.isfinite(data.qpos).all()),
            }
        )

        close_target = [0.95, 1.10]
        for name, value in zip(pair, close_target, strict=True):
            data.ctrl[actuator_ids[name]] = value
        _step(model, data, 1200)
        close_readback = [float(data.qpos[qpos_addresses[name]]) for name in pair]
        commands.append(
            {
                "subsystem": "hand",
                "finger": finger,
                "mode": "close",
                "actuators": pair,
                "target_rad": close_target,
                "readback_rad": close_readback,
                "finite": bool(np.isfinite(data.qpos).all()),
            }
        )
        camera.lookat[:] = (data.xpos[hand_body] + data.xpos[focus_body]) / 2.0
        renderer.update_scene(data, camera=camera)
        after_path = output_root / f"finger{finger}_after_close.jpg"
        Image.fromarray(renderer.render()).save(after_path, quality=90)
        render_records.append(
            {
                "finger": finger,
                "before": before_path.name,
                "after": after_path.name,
                "before_stats": _jpeg_stats(before_path),
                "after_stats": _jpeg_stats(after_path),
                "pixel_mean_absolute_difference": float(
                    np.mean(
                        np.abs(
                            np.asarray(Image.open(before_path), dtype=np.float32)
                            - np.asarray(Image.open(after_path), dtype=np.float32)
                        )
                    )
                ),
            }
        )

    contact_sheet = Image.new("RGB", (1280, 2032), (18, 24, 32))
    draw = ImageDraw.Draw(contact_sheet)
    for row in range(4):
        for column, mode in enumerate(("before_open", "after_close")):
            image = Image.open(output_root / f"finger{row + 1}_{mode}.jpg").convert("RGB")
            y = row * 508 + 28
            contact_sheet.paste(image, (column * 640, y))
            draw.text(
                (column * 640 + 8, y - 22),
                f"Finger {row + 1} {mode.replace('_', ' ')}",
                fill=(220, 235, 245),
            )
    contact_sheet_path = output_root / "closeup_contact_sheet.jpg"
    contact_sheet.save(contact_sheet_path, quality=92)

    # Exercise rendering throughput independently of the real-time physics
    # worker. The service itself caps publication at 15 FPS and drops old frames.
    frames = 30
    start = time.monotonic()
    for _ in range(frames):
        _step(model, data, 1)
        renderer.update_scene(data, camera=camera)
        renderer.render()
    render_elapsed = time.monotonic() - start
    renderer.close()

    report = {
        "status": "PASS"
        if all(item.get("finite", True) for item in commands)
        and all(item["before_stats"]["nonblank"] and item["after_stats"]["nonblank"] for item in render_records)
        else "FAIL",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "runtime": {"mujoco_version": mujoco.__version__, "render_backend": os.environ["MUJOCO_GL"]},
        "model": {
            "actuator_count": model.nu,
            "equality_count": model.neq,
            "mesh_count": model.nmesh,
            "qpos_count": model.nq,
            "actuator_order": [
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
                for index in range(model.nu)
            ],
            "manifest": str(Path(model_report["manifest_path"]).relative_to(output_root)),
        },
        "commands": commands,
        "renders": render_records,
        "closeup_contact_sheet": contact_sheet_path.name,
        "render_throughput_fps": frames / render_elapsed,
    }
    (output_root / "command_and_render_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=ROOT / "isaacsim_test" / "artifacts" / f"mujoco_superarm_amazinghand_{_timestamp()}",
    )
    args = parser.parse_args()
    report = run(args.output_root)
    print(json.dumps({"output_root": str(args.output_root), **report}, indent=2))


if __name__ == "__main__":
    main()
