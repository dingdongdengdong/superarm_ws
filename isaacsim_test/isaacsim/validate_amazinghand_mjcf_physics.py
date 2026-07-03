"""Validate standalone AmazingHand MJCF import and physics behavior in Isaac Sim."""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import shutil
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from isaacsim import SimulationApp


CONTAINER_ROOT = "/workspace/superarm_ws"
HOST_ROOT = (
    Path(CONTAINER_ROOT)
    if Path(CONTAINER_ROOT).is_dir()
    else Path(__file__).resolve().parents[2]
)
DEFAULT_MJCF = (
    f"{CONTAINER_ROOT}/AmazingHand/Demo/AHSimulation/AHSimulation/AH_Right/mjcf/robot.xml"
)
DEFAULT_OUTPUT_ROOT = (
    f"{CONTAINER_ROOT}/isaacsim_test/outputs/simready/echo_full/sitl/"
    "amazinghand_single_mjcf_physics"
)
HAND_PRIM_PATH = "/World/AmazingHandRightStandalone"


def _host_path(path: str | Path) -> Path:
    raw = str(path)
    if raw.startswith(CONTAINER_ROOT + "/"):
        candidate = HOST_ROOT / raw[len(CONTAINER_ROOT) + 1 :]
        if candidate.exists() or not Path(raw).exists():
            return candidate
    return Path(raw)


def _container_path(path: str | Path) -> str:
    host = _host_path(path).resolve()
    try:
        rel = host.relative_to(HOST_ROOT.resolve())
    except ValueError:
        return str(path).replace("\\", "/")
    return f"{CONTAINER_ROOT}/{rel.as_posix()}"


def _repo_relative_path(path: str | Path) -> str:
    normalized = str(path).replace("\\", "/")
    prefix = CONTAINER_ROOT + "/"
    if normalized.startswith(prefix):
        return normalized[len(prefix) :]
    host = _host_path(path)
    try:
        return host.resolve().relative_to(HOST_ROOT.resolve()).as_posix()
    except ValueError:
        return normalized


def _write_json(path: str | Path, payload: dict[str, Any]) -> None:
    output = _host_path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _set_if_available(config: Any, method_name: str, value: Any) -> None:
    method = getattr(config, method_name, None)
    if callable(method):
        method(value)


def analyze_mjcf(mjcf_path: str | Path) -> dict[str, Any]:
    source = _host_path(mjcf_path)
    asset_root = source.parent / "assets"
    root = ET.parse(source).getroot()
    worldbody = root.find("worldbody")
    root_body = None
    if worldbody is not None and worldbody.find("body") is not None:
        root_body = worldbody.find("body").attrib.get("name")
    mesh_files = [
        mesh.attrib.get("file", "")
        for mesh in root.findall("./asset/mesh")
        if mesh.attrib.get("file")
    ]
    missing_meshes = [
        str(asset_root / mesh_file)
        for mesh_file in mesh_files
        if not (asset_root / mesh_file).is_file()
    ]
    motor_joints = [
        actuator.attrib.get("joint", "")
        for actuator in root.findall("./actuator/position")
        if actuator.attrib.get("joint")
    ]
    return {
        "mjcf_path": _repo_relative_path(source),
        "asset_root": _repo_relative_path(asset_root),
        "root_body": root_body,
        "top_level_default_count": len(root.findall("default")),
        "mesh_count": len(mesh_files),
        "missing_meshes": missing_meshes,
        "ball_joint_count": len(root.findall(".//joint[@type='ball']")),
        "hinge_joint_count": len(root.findall(".//joint[@type='hinge']")),
        "position_actuator_count": len(motor_joints),
        "position_actuator_joints": motor_joints,
        "equality_connect_count": len(root.findall("./equality/connect")),
        "has_closed_loop_constraints": bool(root.findall("./equality/connect")),
        "status": "PASS" if root_body and not missing_meshes else "FAIL",
    }


def sanitize_mjcf_for_isaac(source_mjcf: str | Path, output_mjcf: str | Path) -> dict[str, Any]:
    source = _host_path(source_mjcf)
    output = _host_path(output_mjcf)
    tree = ET.parse(source)
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.attrib["meshdir"] = str((source.parent / "assets").resolve())

    defaults = root.findall("default")
    if len(defaults) > 1:
        first_index = list(root).index(defaults[0])
        merged = ET.Element("default")
        for default in defaults:
            for child in list(default):
                default.remove(child)
                merged.append(child)
            root.remove(default)
        root.insert(first_index, merged)

    mesh_names_added = 0
    for mesh in root.findall("./asset/mesh"):
        if not mesh.attrib.get("name") and mesh.attrib.get("file"):
            mesh.attrib["name"] = Path(mesh.attrib["file"]).stem
            mesh_names_added += 1

    equality_connect_names_added = 0
    for index, connect in enumerate(root.findall("./equality/connect"), start=1):
        if not connect.attrib.get("name"):
            connect.attrib["name"] = f"closing_connect_{index:02d}"
            equality_connect_names_added += 1

    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    output.write_text(output.read_text(encoding="utf-8") + "\n", encoding="utf-8")
    sanitized_root = ET.parse(output).getroot()
    return {
        "input_mjcf": _repo_relative_path(source),
        "output_mjcf": _repo_relative_path(output),
        "meshdir": compiler.attrib["meshdir"],
        "top_level_defaults_before": len(defaults),
        "top_level_defaults_after": len(sanitized_root.findall("default")),
        "mesh_names_added": mesh_names_added,
        "equality_connect_names_added": equality_connect_names_added,
        "equality_connect_count": len(sanitized_root.findall("./equality/connect")),
        "status": "PASS" if len(sanitized_root.findall("default")) == 1 else "FAIL",
    }


def _import_mjcf_to_usd(mjcf_path: str | Path, usd_path: str | Path, fix_base: bool) -> dict[str, Any]:
    import omni.kit.commands
    import omni.usd

    context = omni.usd.get_context()
    context.new_stage()
    status, import_config = omni.kit.commands.execute("MJCFCreateImportConfig")
    if not status:
        raise RuntimeError("MJCFCreateImportConfig failed")

    _set_if_available(import_config, "set_fix_base", fix_base)
    _set_if_available(import_config, "set_import_inertia_tensor", True)
    _set_if_available(import_config, "set_make_default_prim", True)

    output = _host_path(usd_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    status, imported_prim_path = omni.kit.commands.execute(
        "MJCFCreateAsset",
        mjcf_path=_container_path(mjcf_path),
        import_config=import_config,
        prim_path=HAND_PRIM_PATH,
        dest_path=_container_path(output),
    )
    if not status:
        raise RuntimeError(f"MJCFCreateAsset failed for {_repo_relative_path(mjcf_path)}")
    return {
        "status": "PASS",
        "input_mjcf": _repo_relative_path(mjcf_path),
        "output_usd": _repo_relative_path(output),
        "imported_prim_path": imported_prim_path or HAND_PRIM_PATH,
        "fix_base": fix_base,
    }


def _inspect_stage(stage: Any, root_prim_path: str = HAND_PRIM_PATH) -> dict[str, Any]:
    from pxr import Usd, UsdGeom, UsdPhysics

    prim_count = 0
    mesh_count = 0
    xform_count = 0
    rigid_body_paths: list[str] = []
    joint_paths: list[str] = []
    articulation_roots: list[str] = []
    for prim in stage.Traverse():
        prim_count += 1
        if prim.IsA(UsdGeom.Mesh):
            mesh_count += 1
        if prim.IsA(UsdGeom.Xform):
            xform_count += 1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_body_paths.append(str(prim.GetPath()))
        if prim.IsA(UsdPhysics.Joint):
            joint_paths.append(str(prim.GetPath()))
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            articulation_roots.append(str(prim.GetPath()))

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=True,
    )
    root_prim = stage.GetPrimAtPath(root_prim_path)
    bbox = (
        bbox_cache.ComputeWorldBound(root_prim).ComputeAlignedBox()
        if root_prim.IsValid()
        else None
    )
    bbox_size = None
    if bbox:
        size = bbox.GetMax() - bbox.GetMin()
        bbox_size = [float(size[0]), float(size[1]), float(size[2])]
    return {
        "prim_count": prim_count,
        "mesh_count": mesh_count,
        "xform_count": xform_count,
        "rigid_body_count": len(rigid_body_paths),
        "rigid_body_paths_sample": rigid_body_paths[:20],
        "joint_count": len(joint_paths),
        "joint_paths_sample": joint_paths[:20],
        "articulation_roots": articulation_roots,
        "bbox_size_m": bbox_size,
        "contains_imported_hand": mesh_count > 0 and (joint_paths or rigid_body_paths),
    }


def _body_translations(stage: Any) -> dict[str, list[float]]:
    from pxr import UsdGeom, UsdPhysics

    cache = UsdGeom.XformCache()
    poses: dict[str, list[float]] = {}
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            transform = cache.GetLocalToWorldTransform(prim)
            translation = transform.ExtractTranslation()
            poses[str(prim.GetPath())] = [float(translation[0]), float(translation[1]), float(translation[2])]
    return poses


def _max_body_displacement(before: dict[str, list[float]], after: dict[str, list[float]]) -> float | None:
    import math

    common = set(before) & set(after)
    if not common:
        return None
    return max(
        math.dist(before[path], after[path])
        for path in common
    )


def _frame_camera(stage: Any, root_prim_path: str) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    from pxr import Gf, Usd, UsdGeom

    prim = stage.GetPrimAtPath(root_prim_path)
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=True,
    )
    bbox = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
    center = (bbox.GetMin() + bbox.GetMax()) * 0.5
    size = bbox.GetMax() - bbox.GetMin()
    radius = max(float(size[0]), float(size[1]), float(size[2]), 0.2)
    eye = Gf.Vec3d(
        float(center[0]) + radius * 1.8,
        float(center[1]) - radius * 2.0,
        float(center[2]) + radius * 1.2,
    )
    target = Gf.Vec3d(float(center[0]), float(center[1]), float(center[2]))
    return tuple(float(v) for v in eye), tuple(float(v) for v in target)


def _capture_replicator(path: str | Path, root_prim_path: str) -> None:
    import omni.replicator.core as rep
    import omni.usd
    from pxr import UsdLux

    stage = omni.usd.get_context().get_stage()
    eye, target = _frame_camera(stage, root_prim_path)
    if not stage.GetPrimAtPath("/World/AmazingHandStandaloneCaptureLight").IsValid():
        UsdLux.DomeLight.Define(stage, "/World/AmazingHandStandaloneCaptureLight").CreateIntensityAttr(600.0)
    output_dir = tempfile.mkdtemp(prefix=Path(path).name + ".")
    try:
        with rep.new_layer():
            camera = rep.create.camera(position=eye, look_at=target, focal_length=45)
            render_product = rep.create.render_product(camera, (1280, 720))
            writer = rep.WriterRegistry.get("BasicWriter")
            writer.initialize(output_dir=output_dir, rgb=True)
            writer.attach([render_product])
            for _ in range(8):
                rep.orchestrator.step(rt_subframes=8)
            writer.detach()
        frames = sorted(Path(output_dir).glob("rgb*.png"))
        if not frames:
            raise RuntimeError(f"Replicator did not write RGB frames in {output_dir}")
        shutil.copyfile(frames[-1], _host_path(path))
    finally:
        shutil.rmtree(output_dir, ignore_errors=True)


def _capture_viewport(path: str | Path, root_prim_path: str) -> None:
    import omni.usd
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

    stage = omni.usd.get_context().get_stage()
    eye, target = _frame_camera(stage, root_prim_path)
    set_camera_view(eye=eye, target=target, camera_prim_path="/OmniverseKit_Persp")
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("No active viewport available")

    async def _capture() -> None:
        capture = capture_viewport_to_file(viewport, file_path=_container_path(path))
        await asyncio.wait_for(capture.wait_for_result(), timeout=15.0)

    asyncio.get_event_loop().run_until_complete(_capture())


def _capture_screenshot(path: str | Path, root_prim_path: str) -> dict[str, Any]:
    output = _host_path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    errors: list[str] = []
    for label, method in (("viewport", _capture_viewport), ("replicator", _capture_replicator)):
        try:
            method(output, root_prim_path)
            deadline = time.time() + 15.0
            while time.time() < deadline:
                if output.is_file() and output.stat().st_size > 0:
                    return {
                        "status": "PASS",
                        "path": _repo_relative_path(output),
                        "size_bytes": output.stat().st_size,
                        "method": label,
                    }
                time.sleep(0.05)
            raise TimeoutError(f"{label} did not create a non-empty screenshot")
        except Exception as exc:
            errors.append(f"{label}: {type(exc).__name__}: {exc}")
    return {
        "status": "FAIL",
        "path": _repo_relative_path(output),
        "errors": errors,
    }


def _make_contact_sheet(image_paths: list[Path], output_path: Path) -> dict[str, Any]:
    from PIL import Image, ImageDraw

    if not image_paths:
        return {"status": "SKIPPED", "reason": "no screenshots"}
    thumb_w, thumb_h = 640, 360
    label_h = 30
    sheet = Image.new("RGB", (thumb_w, (thumb_h + label_h) * len(image_paths)), "white")
    draw = ImageDraw.Draw(sheet)
    for index, image_path in enumerate(image_paths):
        image = Image.open(image_path).convert("RGB")
        image.thumbnail((thumb_w, thumb_h))
        x = (thumb_w - image.width) // 2
        y = index * (thumb_h + label_h)
        sheet.paste(image, (x, y))
        draw.text((8, y + thumb_h + 7), image_path.name, fill=(0, 0, 0))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(output_path)
    return {
        "status": "PASS",
        "path": _repo_relative_path(output_path),
        "size_bytes": output_path.stat().st_size,
    }


def _attempt_imports(args: argparse.Namespace) -> tuple[dict[str, Any] | None, list[dict[str, Any]], dict[str, Any] | None]:
    attempts: list[dict[str, Any]] = []
    sanitized_report = None
    candidates = [("original", _host_path(args.mjcf))]
    if not args.no_sanitize:
        sanitized_report = sanitize_mjcf_for_isaac(args.mjcf, args.sanitized_mjcf)
        candidates.append(("sanitized", _host_path(args.sanitized_mjcf)))

    for label, mjcf_path in candidates:
        usd_path = _host_path(args.usd).with_name(f"{Path(args.usd).stem}_{label}.usd")
        attempt: dict[str, Any] = {
            "label": label,
            "input_mjcf": _repo_relative_path(mjcf_path),
            "output_usd": _repo_relative_path(usd_path),
            "fix_base": args.fix_base,
        }
        try:
            import_report = _import_mjcf_to_usd(mjcf_path, usd_path, args.fix_base)
            import omni.usd

            context = omni.usd.get_context()
            if not context.open_stage(_container_path(usd_path)):
                raise RuntimeError(f"Could not reopen imported USD: {_repo_relative_path(usd_path)}")
            stage = context.get_stage()
            stage_report = _inspect_stage(stage, import_report["imported_prim_path"])
            attempt.update(import_report)
            attempt["stage_inspection"] = stage_report
            attempt["status"] = "PASS" if stage_report["contains_imported_hand"] else "EMPTY_IMPORT"
            attempts.append(attempt)
            if attempt["status"] == "PASS":
                return attempt, attempts, sanitized_report
        except Exception as exc:
            attempt["status"] = "FAIL"
            attempt["reason"] = f"{type(exc).__name__}: {exc}"
            if usd_path.exists():
                attempt["partial_usd_size_bytes"] = usd_path.stat().st_size
            attempts.append(attempt)
    return None, attempts, sanitized_report


def run_validation(args: argparse.Namespace) -> dict[str, Any]:
    app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})
    try:
        import omni.timeline
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.asset.importer.mjcf")
        enable_extension("isaacsim.test.utils")
        enable_extension("omni.kit.renderer.capture")
        app.update()

        mjcf_report = analyze_mjcf(args.mjcf)
        selected_import, import_attempts, sanitized_report = _attempt_imports(args)
        report: dict[str, Any] = {
            "status": "IMPORT_BLOCKED",
            "mjcf_source": mjcf_report,
            "sanitization": sanitized_report,
            "import_attempts": import_attempts,
            "physics_validation": None,
            "evidence_summary": (
                "Standalone AmazingHand physics was not run because Isaac Sim did not "
                "produce a non-empty imported hand USD from the MJCF."
            ),
        }
        if selected_import is None:
            _write_json(args.report, report)
            return report

        context = omni.usd.get_context()
        usd_path = selected_import["output_usd"]
        if not context.open_stage(_container_path(usd_path)):
            raise RuntimeError(f"Could not open selected hand USD: {usd_path}")
        app.update()
        stage = context.get_stage()
        root_prim_path = selected_import["imported_prim_path"]

        world = World(stage_units_in_meters=1.0)
        if args.add_ground:
            world.scene.add_default_ground_plane()
        world.reset()
        timeline = omni.timeline.get_timeline_interface()

        screenshot_root = _host_path(args.screenshot_root)
        before_path = screenshot_root / "01_before_physics.png"
        before_capture = _capture_screenshot(before_path, root_prim_path)
        before_stage = _inspect_stage(stage, root_prim_path)
        before_poses = _body_translations(stage)

        timeline.play()
        for _ in range(args.steps):
            world.step(render=True)
        app.update()
        after_poses = _body_translations(stage)
        after_stage = _inspect_stage(stage, root_prim_path)
        after_path = screenshot_root / "02_after_physics.png"
        after_capture = _capture_screenshot(after_path, root_prim_path)
        timeline.stop()

        screenshots = [
            _host_path(item["path"])
            for item in (before_capture, after_capture)
            if item.get("status") == "PASS"
        ]
        contact_sheet = _make_contact_sheet(screenshots, screenshot_root / "contact_sheet.png")
        max_displacement = _max_body_displacement(before_poses, after_poses)
        bbox_before = before_stage.get("bbox_size_m") or [0.0, 0.0, 0.0]
        bbox_after = after_stage.get("bbox_size_m") or [0.0, 0.0, 0.0]
        bbox_growth = [
            float(after) - float(before)
            for before, after in zip(bbox_before, bbox_after, strict=False)
        ]
        separated = (
            max_displacement is not None and max_displacement > args.separation_threshold
        ) or any(abs(value) > args.separation_threshold for value in bbox_growth)
        physics_status = "FAIL_SEPARATED" if separated else "PASS"

        report.update(
            {
                "status": physics_status,
                "selected_import": selected_import,
                "physics_validation": {
                    "status": physics_status,
                    "fix_base": args.fix_base,
                    "steps": args.steps,
                    "add_ground": args.add_ground,
                    "separation_threshold_m": args.separation_threshold,
                    "before_stage": before_stage,
                    "after_stage": after_stage,
                    "max_rigid_body_displacement_m": max_displacement,
                    "bbox_growth_m": bbox_growth,
                    "before_screenshot": before_capture,
                    "after_screenshot": after_capture,
                    "contact_sheet": contact_sheet,
                },
                "evidence_summary": (
                    "Isaac Sim imported the standalone AmazingHand MJCF to a non-empty USD, "
                    "captured the hand before physics, stepped physics, captured it again, "
                    "and compared rigid-body displacement plus bbox growth."
                ),
            }
        )
        _write_json(args.report, report)
        return report
    finally:
        app.close()


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mjcf", default=os.environ.get("AMAZINGHAND_MJCF_PATH", DEFAULT_MJCF))
    parser.add_argument(
        "--output-root",
        default=os.environ.get("AMAZINGHAND_SINGLE_OUTPUT_ROOT", DEFAULT_OUTPUT_ROOT),
    )
    parser.add_argument("--steps", type=int, default=int(os.environ.get("AMAZINGHAND_SINGLE_STEPS", "120")))
    parser.add_argument(
        "--separation-threshold",
        type=float,
        default=float(os.environ.get("AMAZINGHAND_SINGLE_SEPARATION_THRESHOLD_M", "0.05")),
    )
    parser.add_argument("--no-sanitize", action="store_true")
    parser.add_argument(
        "--fix-base",
        action="store_true",
        default=os.environ.get("AMAZINGHAND_SINGLE_FIX_BASE", "1").strip() != "0",
    )
    parser.add_argument(
        "--no-ground",
        dest="add_ground",
        action="store_false",
        default=os.environ.get("AMAZINGHAND_SINGLE_ADD_GROUND", "1").strip() != "0",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        default=os.environ.get("HEADLESS", "1").strip() != "0",
    )
    args = parser.parse_args()
    output_root = _host_path(args.output_root)
    args.usd = str(output_root / "amazinghand_right_single_mjcf.usd")
    args.sanitized_mjcf = str(output_root / "amazinghand_right_single_isaac_sanitized.xml")
    args.screenshot_root = str(output_root / "screenshots")
    args.report = str(output_root / "amazinghand_right_single_mjcf_physics_report.json")
    return args


def main() -> None:
    args = _build_arg_parser()
    report = run_validation(args)
    _write_json(args.report, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    if report["status"] in {"PASS", "FAIL_SEPARATED", "IMPORT_BLOCKED"}:
        return
    sys.exit(1)


if __name__ == "__main__":
    main()
