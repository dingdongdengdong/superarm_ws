#!/usr/bin/env python3
"""Headless visual + physics-drive diagnostics for the controllable SimReady arm."""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

DEFAULT_PHYSICAL_PROPERTIES_JSON = Path("isaacsim_test/artifacts/simready_controllable_physical_properties.json")
EXPECTED_ARM_JOINTS = [
    "right_arm_pitch_joint",
    "right_arm_roll_joint",
    "right_arm_yaw_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
]
AMAZINGHAND_VISUAL_COMPONENT = "tn__AmazingHand_righthandv201_lQjM"


def _save_png(path: Path, rgba) -> None:
    import numpy as np
    from PIL import Image

    arr = np.asarray(rgba)
    if arr.dtype != np.uint8:
        arr = np.clip(arr * 255.0 if arr.max(initial=0) <= 1.0 else arr, 0, 255).astype("uint8")
    if arr.shape[-1] == 4:
        img = Image.fromarray(arr, mode="RGBA")
    else:
        img = Image.fromarray(arr[..., :3], mode="RGB")
    path.parent.mkdir(parents=True, exist_ok=True)
    img.save(path)


def _make_contact_sheet(image_paths: list[Path], labels: list[str], out_path: Path) -> dict:
    from PIL import Image, ImageDraw, ImageFont

    images = [Image.open(p).convert("RGB") for p in image_paths]
    thumb_w = min(640, max(1, images[0].width))
    thumbs = []
    for img in images:
        h = int(img.height * (thumb_w / img.width))
        thumbs.append(img.resize((thumb_w, h)))
    label_h = 34
    sheet = Image.new("RGB", (thumb_w * len(thumbs), thumbs[0].height + label_h), (20, 20, 20))
    draw = ImageDraw.Draw(sheet)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 18)
    except Exception:
        font = ImageFont.load_default()
    for i, (thumb, label) in enumerate(zip(thumbs, labels, strict=True)):
        x = i * thumb_w
        sheet.paste(thumb, (x, label_h))
        draw.text((x + 10, 7), label, fill=(255, 255, 255), font=font)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out_path)
    return {"contact_sheet": str(out_path), "inputs": [str(p) for p in image_paths], "labels": labels}


def _load_json(path: Path) -> dict:
    if not path.is_file():
        return {"status": "missing", "path": str(path)}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:  # pragma: no cover - defensive runtime report path
        return {"status": "invalid_json", "path": str(path), "exception": repr(exc)}
    payload.setdefault("status", "loaded")
    payload.setdefault("path", str(path))
    return payload


def _summarize_provenance(payload: dict) -> dict:
    link_props = payload.get("link_properties") or {}
    rows = payload.get("per_link_audit_table") or []
    counts: dict[str, int] = {"derived": 0, "inferred": 0, "missing": 0, "runtime_tuned": 0}
    for props in link_props.values():
        for field in ("mass", "center_of_mass", "diagonal_inertia", "collider"):
            value = props.get(field, {})
            provenance = value.get("provenance")
            if provenance in counts:
                counts[provenance] += 1
    runtime_policy = (payload.get("controller_properties") or {}).get("runtime_override_policy") or {}
    if runtime_policy.get("provenance") == "runtime_tuned":
        counts["runtime_tuned"] += 1
    return {
        "schema_version": payload.get("schema_version"),
        "claim_boundary": payload.get("claim_boundary"),
        "source_asset_hash": payload.get("source_asset_hash"),
        "source_asset_hash_scope": payload.get("source_asset_hash_scope"),
        "per_link_audit_table": rows,
        "provenance_counts": counts,
        "controller_properties": payload.get("controller_properties"),
        "status": payload.get("status", "loaded"),
    }


def _is_physics_schema_token(token: object) -> bool:
    text = str(token)
    return "Physics" in text or "Physx" in text


def _visual_physics_findings(stage, root_path: str, *, source_visuals_only: bool = False) -> dict:
    root = stage.GetPrimAtPath(root_path)
    findings: list[dict] = []
    if not root or not root.IsValid():
        return {
            "root_path": root_path,
            "root_valid": False,
            "source_visuals_only": source_visuals_only,
            "physics_schema_count": 0,
            "physics_type_count": 0,
            "finding_count": 0,
            "findings": findings,
            "status": "missing_root",
        }

    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(root_path.rstrip("/") + "/") and path != root_path:
            continue
        if source_visuals_only and "/source_visuals" not in path:
            continue
        applied = [schema for schema in prim.GetAppliedSchemas() if _is_physics_schema_token(schema)]
        type_name = prim.GetTypeName()
        physics_type = type_name if type_name and _is_physics_schema_token(type_name) else None
        if applied or physics_type:
            findings.append(
                {
                    "prim_path": path,
                    "applied_physics_schemas": applied,
                    "physics_type": physics_type,
                }
            )

    physics_schema_count = sum(len(item["applied_physics_schemas"]) for item in findings)
    physics_type_count = sum(1 for item in findings if item["physics_type"])
    return {
        "root_path": root_path,
        "root_valid": True,
        "source_visuals_only": source_visuals_only,
        "physics_schema_count": physics_schema_count,
        "physics_type_count": physics_type_count,
        "finding_count": len(findings),
        "findings": findings[:50],
        "truncated": len(findings) > 50,
        "status": "passed" if not findings else "failed",
    }


def _trajectory_sanity(trajectory: list[dict], expected: list[str]) -> dict:
    if not trajectory:
        return {
            "finite": False,
            "bad_samples": [],
            "sample_count": 0,
            "max_abs_sample_rad": 0.0,
            "explosion_threshold_rad": 4.0 * math.pi,
            "within_explosion_threshold": False,
            "status": "failed",
            "reason": "empty_trajectory",
        }
    values: list[float] = []
    bad_samples: list[dict] = []
    for sample in trajectory:
        readback = sample.get("readback_by_name", {})
        for joint in expected:
            value = readback.get(joint)
            if value is None or not math.isfinite(float(value)):
                bad_samples.append({"frame": sample.get("frame"), "joint": joint, "value": value})
            else:
                values.append(float(value))
    max_abs = max((abs(v) for v in values), default=0.0)
    explosion_threshold_rad = 4.0 * math.pi
    return {
        "finite": not bad_samples,
        "bad_samples": bad_samples,
        "sample_count": len(trajectory),
        "max_abs_sample_rad": max_abs,
        "explosion_threshold_rad": explosion_threshold_rad,
        "within_explosion_threshold": max_abs <= explosion_threshold_rad,
        "status": "passed" if not bad_samples and max_abs <= explosion_threshold_rad else "failed",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--usd", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--resolution", default="1280x720")
    parser.add_argument("--physical-properties-json", type=Path, default=DEFAULT_PHYSICAL_PROPERTIES_JSON)
    parser.add_argument("--thread-count", type=int, default=int(os.environ.get("ISAACSIM_THREAD_COUNT", "4")))
    args = parser.parse_args()

    width, height = (int(v) for v in args.resolution.lower().split("x", 1))
    args.out_dir.mkdir(parents=True, exist_ok=True)
    report_path = args.out_dir / "viewport_physics_report.json"
    physical_payload = _load_json(args.physical_properties_json)
    provenance_summary = _summarize_provenance(physical_payload)
    thread_limit_args = [
        f"--/plugins/carb.tasking.plugin/threadCount={args.thread_count}",
        f"--/plugins/omni.tbb.globalcontrol/maxThreadCount={args.thread_count}",
    ]

    physical_properties_loaded = (
        provenance_summary.get("status") == "loaded"
        and provenance_summary.get("schema_version") == 1
        and bool(provenance_summary.get("claim_boundary"))
    )

    report: dict = {
        "status": "started",
        "runtime_mode": "standalone_headless_camera_render",
        "usd": str(args.usd),
        "out_dir": str(args.out_dir),
        "resolution": [width, height],
        "command": sys.argv,
        "physical_properties_json": str(args.physical_properties_json),
        "physical_properties_loaded": physical_properties_loaded,
        "physical_properties_summary": provenance_summary,
        "requested_thread_limit_args": thread_limit_args,
        "thread_limit_note": "Isaac/Kit launchers may append their own thread args after these; startup logs remain the source of truth.",
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    simulation_app = None
    try:
        from isaacsim import SimulationApp

        simulation_app = SimulationApp(
            {
                "headless": args.headless,
                "width": width,
                "height": height,
                "extra_args": thread_limit_args,
            }
        )

        import numpy as np
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import Articulation
        from isaacsim.core.utils.rotations import rot_matrix_to_quat
        from isaacsim.sensors.camera import Camera
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

        stage = omni.usd.get_context().get_stage()
        root_path = "/World/echo_full_simready"
        root = stage.DefinePrim(root_path, "Xform")
        root.GetReferences().AddReference(str(args.usd.resolve()))
        articulation_path = root_path + "/ControlRig"
        wrist_device_visual_path = (
            articulation_path
            + "/Links/wrist_fixed_hand_mount_link/source_visuals/"
            + AMAZINGHAND_VISUAL_COMPONENT
        )
        target_prims = [
            root_path,
            articulation_path,
            articulation_path + "/Links/forearm_link",
            articulation_path + "/Links/wrist_fixed_hand_mount_link",
            wrist_device_visual_path,
        ]

        marker_specs = [
            ("base_link", (1.0, 0.1, 0.1)),
            ("shoulder_pitch_link", (1.0, 0.6, 0.0)),
            ("shoulder_roll_link", (1.0, 1.0, 0.0)),
            ("upper_arm_link", (0.1, 1.0, 0.1)),
            ("forearm_link", (0.0, 0.7, 1.0)),
            ("wrist_fixed_hand_mount_link", (0.8, 0.2, 1.0)),
        ]
        marker_paths = []
        for link_name, color in marker_specs:
            marker_path = Sdf.Path(f"{articulation_path}/Links/{link_name}/viewport_debug_marker")
            marker = UsdGeom.Cube.Define(stage, marker_path)
            marker.CreateSizeAttr(1.0)
            marker.AddTranslateOp().Set(Gf.Vec3f(0.18, 0.0, 0.0))
            marker.AddScaleOp().Set(Gf.Vec3f(0.045, 0.045, 0.045))
            marker.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            marker_paths.append(str(marker_path))

        for light_path, intensity in (
            ("/World/ViewportDebugDomeLight", 900.0),
            ("/World/ViewportDebugDistantLight", 3500.0),
        ):
            if light_path.endswith("DomeLight"):
                light = UsdLux.DomeLight.Define(stage, light_path)
            else:
                light = UsdLux.DistantLight.Define(stage, light_path)
                light.CreateAngleAttr(0.5)
            light.CreateIntensityAttr(intensity)

        world = World(stage_units_in_meters=1.0)
        world.scene.add_default_ground_plane()
        art = Articulation(articulation_path)
        for _ in range(30):
            simulation_app.update()
        world.reset()
        for _ in range(20):
            world.step(render=True)
        art.initialize()

        dof_names = list(art.dof_names)
        expected = EXPECTED_ARM_JOINTS
        missing = [name for name in expected if name not in dof_names]
        if missing:
            report.update({"status": "failed_missing_joints", "dof_names": dof_names, "missing": missing})
            report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            print(json.dumps(report, indent=2, sort_keys=True), flush=True)
            return 2

        runtime_policy = (physical_payload.get("controller_properties") or {}).get("runtime_override_policy") or {}
        controller_settings = {
            "kp": float(runtime_policy.get("kp", 50000.0)),
            "kd": float(runtime_policy.get("kd", 5000.0)),
            "max_effort": float(runtime_policy.get("max_effort", 5000.0)),
        }
        controller_provenance = {
            "authored_drive": (physical_payload.get("controller_properties") or {}).get("authored_drive"),
            "runtime_override_policy": runtime_policy,
            "runtime_override_applied": True,
            "threshold_motion_requires_tuning": "evaluated_by_this_run; disclosed as runtime_tuned when applied",
        }
        art.set_gains(
            kps=np.array([[controller_settings["kp"]] * int(art.num_dof)], dtype=np.float32),
            kds=np.array([[controller_settings["kd"]] * int(art.num_dof)], dtype=np.float32),
        )
        art.set_max_efforts(values=np.array([[controller_settings["max_effort"]] * int(art.num_dof)], dtype=np.float32))

        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        root_bbox = bbox_cache.ComputeWorldBound(stage.GetPrimAtPath(root_path)).ComputeAlignedBox()
        root_min = np.array(root_bbox.GetMin(), dtype=float)
        root_max = np.array(root_bbox.GetMax(), dtype=float)
        root_center = (root_min + root_max) * 0.5
        root_size = root_max - root_min
        if not np.all(np.isfinite(root_center)) or float(np.max(root_size)) <= 1e-6:
            root_center = np.array([-0.1, 0.15, 0.45], dtype=float)
            root_size = np.array([0.8, 0.8, 0.9], dtype=float)
        camera_path = "/World/ViewportPhysicsDebugCamera"
        view_direction = np.array([2.8, -4.2, 1.3], dtype=float)
        view_direction /= max(float(np.linalg.norm(view_direction)), 1e-9)
        camera_distance = max(5.0, float(np.max(root_size)) * 7.5)
        camera_target = root_center + np.array([0.0, 0.0, -0.10], dtype=float)
        camera_position = camera_target + view_direction * camera_distance

        def look_at_world_quat(pos: np.ndarray, target: np.ndarray) -> np.ndarray:
            forward = target - pos
            norm = np.linalg.norm(forward)
            if not math.isfinite(float(norm)) or norm < 1e-9:
                return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
            x_axis = forward / norm
            up_hint = np.array([0.0, 0.0, 1.0], dtype=float)
            if abs(float(np.dot(x_axis, up_hint))) > 0.98:
                up_hint = np.array([0.0, 1.0, 0.0], dtype=float)
            y_axis = np.cross(up_hint, x_axis)
            y_axis /= max(float(np.linalg.norm(y_axis)), 1e-9)
            z_axis = np.cross(x_axis, y_axis)
            z_axis /= max(float(np.linalg.norm(z_axis)), 1e-9)
            return np.asarray(rot_matrix_to_quat(np.column_stack([x_axis, y_axis, z_axis])), dtype=float)

        camera = Camera(
            prim_path=camera_path,
            position=camera_position,
            orientation=look_at_world_quat(camera_position, camera_target),
            resolution=(width, height),
        )
        camera.initialize()
        camera.set_focal_length(7.0)

        def capture(label: str) -> Path:
            last_shape = None
            rgba = None
            for attempt in range(1, 61):
                world.step(render=True)
                rgba = camera.get_rgba()
                last_shape = None if rgba is None else list(np.asarray(rgba).shape)
                if rgba is not None and np.asarray(rgba).size > 0:
                    # Let the headless RTX camera settle so proof frames are readable,
                    # especially the initial rest frame after camera creation.
                    for _ in range(12):
                        world.step(render=True)
                    rgba = camera.get_rgba()
                    path = args.out_dir / f"{label}.png"
                    _save_png(path, rgba)
                    return path
            raise RuntimeError(
                f"Camera returned no RGBA data for {label} after 60 rendered steps; last_shape={last_shape}"
            )

        start_positions = np.asarray(art.get_joint_positions(), dtype=np.float32).reshape(-1)
        before_image = capture("frame_000_before_rest")

        target_by_name = {
            "right_arm_pitch_joint": 0.45,
            "right_arm_roll_joint": -0.35,
            "right_arm_yaw_joint": 0.32,
            "right_elbow_pitch_joint": 0.55,
            "right_elbow_yaw_joint": -0.28,
        }
        target_positions = start_positions.copy()
        for name, value in target_by_name.items():
            target_positions[dof_names.index(name)] = value

        trajectory = []
        capture_points = {80: "frame_080_mid_drive", 240: "frame_240_after_drive"}
        captures = [before_image]
        for frame in range(1, 241):
            art.set_joint_position_targets(np.array([target_positions], dtype=np.float32))
            world.step(render=True)
            if frame % 10 == 0 or frame in capture_points:
                current = np.asarray(art.get_joint_positions(), dtype=np.float32).reshape(-1)
                trajectory.append(
                    {
                        "frame": frame,
                        "readback_by_name": {name: float(current[dof_names.index(name)]) for name in expected},
                    }
                )
            if frame in capture_points:
                captures.append(capture(capture_points[frame]))

        readback = np.asarray(art.get_joint_positions(), dtype=np.float32).reshape(-1)
        readback_by_name = {name: float(readback[dof_names.index(name)]) for name in expected}
        max_abs_error = max(abs(readback_by_name[name] - target_by_name[name]) for name in expected)
        moved_delta_by_name = {
            name: float(readback[dof_names.index(name)] - start_positions[dof_names.index(name)]) for name in expected
        }
        motion_threshold_rad = 5e-2
        per_joint_motion_pass = {name: abs(delta) >= motion_threshold_rad for name, delta in moved_delta_by_name.items()}
        all_expected_joints_moved = all(per_joint_motion_pass.values())
        min_abs_motion = min(abs(v) for v in moved_delta_by_name.values())
        trajectory_sanity = _trajectory_sanity(trajectory, expected)

        stage_for_inspect = Usd.Stage.Open(str(args.usd))
        has_grasp_joint = bool(stage_for_inspect.GetPrimAtPath("/echo_full_controllable/ControlRig/Joints/amazinghand_grasp"))
        hand_path = (
            "/echo_full_controllable/ControlRig/Links/wrist_fixed_hand_mount_link/source_visuals/"
            + AMAZINGHAND_VISUAL_COMPONENT
        )
        hand_prim = stage_for_inspect.GetPrimAtPath(hand_path)
        fixed_hand_path = (
            "/echo_full_controllable/VisualFixed/tn____xaZ2Ve2pr5yw2tw0WSflDbf1/"
            "tn__v341_a4a4XfWAou7zcLveO/" + AMAZINGHAND_VISUAL_COMPONENT
        )
        fixed_hand_prim = stage_for_inspect.GetPrimAtPath(fixed_hand_path)
        fixed_hand_visibility = (
            fixed_hand_prim.GetAttribute("visibility").Get()
            if fixed_hand_prim and fixed_hand_prim.IsValid() and fixed_hand_prim.GetAttribute("visibility")
            else None
        )
        hand_visual_physics_scan = _visual_physics_findings(stage_for_inspect, hand_path)
        source_visuals_physics_scan = _visual_physics_findings(
            stage_for_inspect,
            "/echo_full_controllable/ControlRig/Links",
            source_visuals_only=True,
        )
        visual_only_physics_clean = (
            hand_visual_physics_scan["status"] == "passed"
            and source_visuals_physics_scan["status"] == "passed"
        )

        contact = _make_contact_sheet(
            captures,
            ["rest", "mid physics drive", "after physics drive"],
            args.out_dir / "before_mid_after_contact_sheet.png",
        )
        from PIL import Image, ImageChops

        before = Image.open(captures[0]).convert("RGB")
        after = Image.open(captures[-1]).convert("RGB")
        diff = ImageChops.difference(before, after)
        bbox = diff.getbbox()
        diff_path = args.out_dir / "before_after_pixel_diff.png"
        diff.save(diff_path)

        tracking_error_note = "reported for tuning only; pass/fail here checks visible target-driven motion, not precision tracking"
        status = (
            "passed"
            if (
                physical_properties_loaded
                and all_expected_joints_moved
                and trajectory_sanity["status"] == "passed"
                and not has_grasp_joint
                and bool(hand_prim and hand_prim.IsValid())
                and fixed_hand_visibility == "invisible"
                and visual_only_physics_clean
                and bbox is not None
            )
            else "failed_visual_or_physics_contract"
        )
        report.update(
            {
                "status": status,
                "runtime_mode": "standalone_headless_camera_render",
                "capture_method": "isaacsim.sensors.camera.Camera.get_rgba",
                "physics_control_method": "Articulation.set_joint_position_targets + World.step(render=True) with runtime controller gains/max efforts",
                "controller_settings": controller_settings,
                "controller_provenance": controller_provenance,
                "camera_path": camera_path,
                "camera_profile": "auto_framed_fullbody_wrist_device_context",
                "camera_position": camera_position.astype(float).tolist(),
                "camera_target": camera_target.astype(float).tolist(),
                "camera_focal_length_mm": 7.0,
                "root_bounds_world": {
                    "min": root_min.astype(float).tolist(),
                    "max": root_max.astype(float).tolist(),
                    "center": root_center.astype(float).tolist(),
                    "size": root_size.astype(float).tolist(),
                },
                "target_prims": target_prims,
                "debug_marker_prims": marker_paths,
                "articulation_path": articulation_path,
                "num_dof": int(art.num_dof),
                "dof_names": dof_names,
                "expected_dof_names": expected,
                "target_by_name": target_by_name,
                "start_by_name": {name: float(start_positions[dof_names.index(name)]) for name in expected},
                "readback_by_name": readback_by_name,
                "moved_delta_by_name": moved_delta_by_name,
                "per_joint_motion_pass": per_joint_motion_pass,
                "all_expected_joints_moved": all_expected_joints_moved,
                "max_abs_error": float(max_abs_error),
                "tracking_error_note": tracking_error_note,
                "min_abs_motion": float(min_abs_motion),
                "motion_threshold_rad": motion_threshold_rad,
                "trajectory_sanity": trajectory_sanity,
                "has_amazinghand_grasp_joint": has_grasp_joint,
                "amazinghand_visual_prim_valid": bool(hand_prim and hand_prim.IsValid()),
                "amazinghand_visual_path": hand_path,
                "fixed_amazinghand_visual_path": fixed_hand_path,
                "fixed_amazinghand_visibility": fixed_hand_visibility,
                "visual_only_physics_clean": visual_only_physics_clean,
                "hand_visual_physics_scan": hand_visual_physics_scan,
                "source_visuals_physics_scan": source_visuals_physics_scan,
                "screenshots": [str(p) for p in captures],
                "pixel_diff": str(diff_path),
                "pixel_diff_nonempty": bbox is not None,
                **contact,
                "trajectory_samples": trajectory,
                "finished_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            }
        )
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2, sort_keys=True), flush=True)
        return 0 if report["status"] == "passed" else 3
    except Exception as exc:
        report.update(
            {
                "status": "exception",
                "exception": repr(exc),
                "exception_type": type(exc).__name__,
                "finished_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            }
        )
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2, sort_keys=True), flush=True)
        return 1
    finally:
        if simulation_app is not None:
            simulation_app.close()


if __name__ == "__main__":
    raise SystemExit(main())
