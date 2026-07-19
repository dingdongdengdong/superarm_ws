"""
Isaac Sim 5.1.0 scene script for the SimReady echo_full asset
plus the existing 6D LeRobot/ROS2 bridge.

Run via: /isaac-sim/python.sh /workspace/isaacsim/setup_rpo_arm_scene.py
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import shutil
import sys
import threading
import time

import numpy as np

from superarm_runtime_contract import (
    LOGICAL_HAND_MOTION_NAME,
    PHYSICAL_JOINT_NAMES,
    infer_logical_state,
    resolve_logical_command,
)

DEFAULT_CONTROLLED_ARM_JOINT_NAMES = [
    "right_arm_pitch_joint",
    "right_arm_roll_joint",
    "right_arm_yaw_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
]
SYNTHETIC_GRASP_NAME = "amazinghand_grasp"


def _requested_joint_names() -> list[str]:
    return [
        part.strip()
        for part in os.environ.get("JOINT_NAMES", "").split(",")
        if part.strip()
    ]


def _parse_controlled_arm_joint_names() -> list[str]:
    names = _requested_joint_names()
    if not names:
        return list(DEFAULT_CONTROLLED_ARM_JOINT_NAMES)
    names = [
        name
        for name in names
        if name not in {SYNTHETIC_GRASP_NAME, LOGICAL_HAND_MOTION_NAME}
    ]
    return names or list(DEFAULT_CONTROLLED_ARM_JOINT_NAMES)


CONTROLLED_ARM_JOINT_NAMES = _parse_controlled_arm_joint_names()
# Combined source-arm launch example: joint_rev_1,...,joint_rev_5,amazinghand_motion.
USES_FIXED_MOTION_HAND = LOGICAL_HAND_MOTION_NAME in _requested_joint_names()
LOGICAL_JOINT_NAMES = [
    *CONTROLLED_ARM_JOINT_NAMES,
    LOGICAL_HAND_MOTION_NAME if USES_FIXED_MOTION_HAND else SYNTHETIC_GRASP_NAME,
]
CONTROLLED_PHYSICAL_JOINT_NAMES = (
    list(PHYSICAL_JOINT_NAMES)
    if USES_FIXED_MOTION_HAND
    else list(CONTROLLED_ARM_JOINT_NAMES)
)
PUBLISHED_JOINT_NAMES = (
    list(CONTROLLED_PHYSICAL_JOINT_NAMES)
    if USES_FIXED_MOTION_HAND
    else list(LOGICAL_JOINT_NAMES)
)
DEFAULT_SIMREADY_USD = (
    "/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/"
    "pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/"
    "echo_full_robot_arm_hand.usd"
)
DEFAULT_SIMREADY_CONTROLLABLE_USD = (
    "/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/"
    "pipeline/04_conform/repair-loop-02-fet005/fet006-controllable-arm-fixed-hand/"
    "echo_full_robot_arm_hand_controllable.usda"
)
DEFAULT_SIMREADY_PRIM_PATH = "/World/echo_full_simready"
DEFAULT_SIMREADY_CONTROL_RIG_REL_PATH = "/ControlRig"
DEFAULT_SIMREADY_MAPPING_PATH = (
    "/workspace/superarm_ws/isaacsim_test/artifacts/"
    "simready_prim_mapping.json"
)
DEFAULT_SIMREADY_THUMBNAIL_PATH = (
    "/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/"
    "pipeline/07_render/thumbnail.png"
)
DEFAULT_ROBOPARTY_V2_URDF = (
    "/workspace/superarm_ws/roboparty/modules/rpo_hardware/V2.0/"
    "roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf"
)
DEFAULT_SCREENSHOT_PATH = (
    "/workspace/superarm_ws/isaacsim_test/artifacts/"
    "echo_full_simready_target.png"
)


def _env_flag(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}



def _env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value is None or not value.strip():
        return default
    return float(value)


def _env_vector(name: str, default: tuple[float, float, float]) -> tuple[float, float, float]:
    value = os.environ.get(name)
    if value is None or not value.strip():
        return default
    parts = [float(part.strip()) for part in value.split(",") if part.strip()]
    if len(parts) != 3:
        raise ValueError(f"{name} must contain exactly 3 comma-separated floats")
    return (parts[0], parts[1], parts[2])

SCREENSHOT_AFTER_COMMAND = _env_flag("SCREENSHOT_AFTER_COMMAND")
SCREENSHOT_ON_STARTUP = _env_flag("SCREENSHOT_ON_STARTUP")
SCREENSHOT_EACH_COMMAND = _env_flag("SCREENSHOT_EACH_COMMAND")
SCREENSHOT_EXIT_AFTER_COMMAND_COUNT = int(os.environ.get("SCREENSHOT_EXIT_AFTER_COMMAND_COUNT", "1"))
SCREENSHOT_PATH = os.environ.get("SCREENSHOT_PATH", DEFAULT_SCREENSHOT_PATH)
SCREENSHOT_SEQUENCE_DIR = os.environ.get(
    "SCREENSHOT_SEQUENCE_DIR",
    "/workspace/superarm_ws/isaacsim_test/artifacts/lerobot_pose_screenshots",
)
SCREENSHOT_CAMERA_POSITION = _env_vector("SCREENSHOT_CAMERA_POSITION", (2.1106870919205436, -4.221374183841087, 2.876166303929372))
SCREENSHOT_CAMERA_TARGET = _env_vector("SCREENSHOT_CAMERA_TARGET", (0.0, 0.0, 0.5))
SCREENSHOT_CAMERA_FOCAL_LENGTH = _env_float("SCREENSHOT_CAMERA_FOCAL_LENGTH", 24.0)
COMMAND_EVIDENCE_PATH = os.environ.get("COMMAND_EVIDENCE_PATH", "")
COMMAND_EVIDENCE_DIR = os.environ.get("COMMAND_EVIDENCE_DIR", "")
EXIT_AFTER_SCREENSHOT = _env_flag(
    "EXIT_AFTER_SCREENSHOT", default=SCREENSHOT_AFTER_COMMAND or SCREENSHOT_ON_STARTUP
)
SIMREADY_PRIM_PATH = os.environ.get("SIMREADY_PRIM_PATH", DEFAULT_SIMREADY_PRIM_PATH)
SIMREADY_MAPPING_PATH = os.environ.get("SIMREADY_MAPPING_PATH", DEFAULT_SIMREADY_MAPPING_PATH)
SIMREADY_THUMBNAIL_PATH = os.environ.get(
    "SIMREADY_THUMBNAIL_PATH", DEFAULT_SIMREADY_THUMBNAIL_PATH
)

parser = argparse.ArgumentParser()
parser.add_argument(
    "--headless",
    action="store_true",
    default=bool(int(os.environ.get("HEADLESS", "1"))),
)
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom, UsdLux  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension, get_extension_path_from_name  # noqa: E402
from isaacsim.core.utils.rotations import rot_matrix_to_quat  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.importer.urdf")
enable_extension("isaacsim.test.utils")
enable_extension("omni.kit.renderer.capture")
simulation_app.update()

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64MultiArray, String  # noqa: E402

ext_path = get_extension_path_from_name("isaacsim.asset.importer.urdf")
SIMREADY_CONTROLLABLE_USD_CANDIDATES = [
    os.environ.get("SIMREADY_CONTROLLABLE_USD_PATH", ""),
    DEFAULT_SIMREADY_CONTROLLABLE_USD,
]
SIMREADY_USD_CANDIDATES = [
    os.environ.get("SIMREADY_USD_PATH", ""),
    DEFAULT_SIMREADY_USD,
]
URDF_CANDIDATES = [
    os.environ.get("RPO_ARM_URDF_PATH", ""),
    DEFAULT_ROBOPARTY_V2_URDF,
]
simready_controllable_usd_path = next((p for p in SIMREADY_CONTROLLABLE_USD_CANDIDATES if p and os.path.isfile(p)), None)
simready_visual_usd_path = next((p for p in SIMREADY_USD_CANDIDATES if p and os.path.isfile(p)), None)
simready_usd_path = simready_controllable_usd_path or simready_visual_usd_path
urdf_path = next((p for p in URDF_CANDIDATES if p and os.path.isfile(p)), None)
using_simready = simready_usd_path is not None
using_controllable_simready = simready_controllable_usd_path is not None


def _load_simready_usd(asset_path: str, prim_path: str) -> str:
    stage = omni.usd.get_context().get_stage()
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(asset_path)
    UsdGeom.Xformable(prim)
    return str(prim.GetPath())


def _control_contract() -> list[str]:
    return [f"{name}.pos" if not name.endswith(".pos") else name for name in PUBLISHED_JOINT_NAMES]


def _collect_simready_prim_hierarchy(root_prim_path: str, limit: int = 200):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_prim_path)
    if not root_prim or not root_prim.IsValid():
        return [], False

    prefix = root_prim_path.rstrip("/") + "/"
    prim_hierarchy = []
    truncated = False
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path == root_prim_path or path.startswith(prefix):
            prim_hierarchy.append(
                {
                    "path": path,
                    "type": prim.GetTypeName() or None,
                }
            )
            if len(prim_hierarchy) >= limit:
                truncated = True
                break
    return prim_hierarchy, truncated


def _write_simready_mapping_evidence(
    *,
    asset_path: str,
    prim_path: str,
    binding_status: str,
    last_command: list[float] | None = None,
) -> None:
    os.makedirs(os.path.dirname(SIMREADY_MAPPING_PATH), exist_ok=True)
    feature_bindings = []
    bound_by_feature = {}
    controllable_binding_statuses = {
        "arm_articulation_bound_fixed_hand_visual",
        "arm_articulation_bound_wrist_attached_hand_visual",
    }
    for feature_name in PUBLISHED_JOINT_NAMES:
        feature = f"{feature_name}.pos" if not feature_name.endswith(".pos") else feature_name
        if binding_status in controllable_binding_statuses and feature_name != SYNTHETIC_GRASP_NAME:
            status = "articulation_bound"
            usd_prim = (
                f"{SIMREADY_PRIM_PATH}{DEFAULT_SIMREADY_CONTROL_RIG_REL_PATH}/Joints/{feature_name}"
            )
            reason = "Five-DOF controllable SimReady overlay authored for arm motion."
        elif binding_status in controllable_binding_statuses and feature_name == SYNTHETIC_GRASP_NAME:
            status = "wrist_attached_visual_only"
            usd_prim = None
            reason = "Mounted device/hand visual follows the wrist as geometry-only; no hand/finger drive is authored."
        else:
            status = "binding_pending"
            usd_prim = None
            reason = (
                "SimReady USD is loaded as the primary visual/physics asset; "
                "articulation mapping must be authored from prim inspection."
            )
        bound_by_feature[feature] = status
        feature_bindings.append(
            {
                "feature": feature,
                "binding_status": status,
                "usd_prim": usd_prim,
                "reason": reason,
            }
        )
    control_contract = _control_contract()
    prim_hierarchy, prim_hierarchy_truncated = _collect_simready_prim_hierarchy(prim_path)
    payload = {
        "asset": os.path.basename(asset_path),
        "asset_path": asset_path,
        "prim_path": prim_path,
        "simready_root_prim": prim_path,
        "prim_hierarchy": prim_hierarchy,
        "prim_hierarchy_truncated": prim_hierarchy_truncated,
        "control_contract": control_contract,
        "binding_status": binding_status,
        "bound_or_binding_pending_per_feature": {
            feature: bound_by_feature.get(feature, "binding_pending") for feature in control_contract
        },
        "feature_bindings": feature_bindings,
        "last_command": last_command,
        "next_step": "inspect SimReady USD prim hierarchy and bind controllable articulation prims",
    }
    with open(SIMREADY_MAPPING_PATH, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, sort_keys=True)
        f.write("\n")

if using_simready:
    prim_path = _load_simready_usd(simready_usd_path, SIMREADY_PRIM_PATH)
    if using_controllable_simready:
        articulation_prim_path = SIMREADY_PRIM_PATH + DEFAULT_SIMREADY_CONTROL_RIG_REL_PATH
        print(f"[setup_rpo_arm_scene] Loading controllable SimReady USD: {simready_usd_path}", flush=True)
        print(f"[setup_rpo_arm_scene] Controllable arm articulation prim: {articulation_prim_path}", flush=True)
        _write_simready_mapping_evidence(
            asset_path=simready_usd_path,
            prim_path=prim_path,
            binding_status="arm_articulation_bound_wrist_attached_hand_visual",
        )
    else:
        articulation_prim_path = ""
        print(f"[setup_rpo_arm_scene] Loading SimReady USD: {simready_usd_path}", flush=True)
        print(f"[setup_rpo_arm_scene] SimReady prim path: {prim_path}", flush=True)
        _write_simready_mapping_evidence(
            asset_path=simready_usd_path,
            prim_path=prim_path,
            binding_status="binding_pending",
        )
elif urdf_path is None:
    print(
        "[setup_rpo_arm_scene] ERROR: No SimReady USD or RoboParty V2.0 URDF found. Tried:\n"
        + "\n".join(f"  {p}" for p in SIMREADY_USD_CANDIDATES if p)
        + "\n"
        + "\n".join(f"  {p}" for p in SIMREADY_CONTROLLABLE_USD_CANDIDATES if p)
        + "\n"
        + "\n".join(f"  {p}" for p in URDF_CANDIDATES if p)
        + "\n\nSet SIMREADY_CONTROLLABLE_USD_PATH to the generated controllable overlay, "
        "set SIMREADY_USD_PATH to the converted SimReady USD, "
        "or set RPO_ARM_URDF_PATH for the legacy fallback."
        + f"\nURDF importer extension path: {ext_path}",
        flush=True,
    )
    simulation_app.close()
    sys.exit(1)
else:
    print(f"[setup_rpo_arm_scene] Loading RoboParty V2.0 URDF fallback: {urdf_path}", flush=True)

print(
    "[setup_rpo_arm_scene] Screenshot config: "
    f"after_command={SCREENSHOT_AFTER_COMMAND}, "
    f"on_startup={SCREENSHOT_ON_STARTUP}, "
    f"each_command={SCREENSHOT_EACH_COMMAND}, "
    f"exit_after_count={SCREENSHOT_EXIT_AFTER_COMMAND_COUNT}, "
    f"path={SCREENSHOT_PATH}, "
    f"sequence_dir={SCREENSHOT_SEQUENCE_DIR}, "
    f"exit_after={EXIT_AFTER_SCREENSHOT}",
    flush=True,
)

if not using_simready:
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.fix_base = True
    import_config.import_inertia_tensor = True
    import_config.distance_scale = 1.0
    import_config.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.default_drive_strength = 1047.2
    import_config.default_position_drive_damping = 52.4

    status, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        get_articulation_root=True,
    )
    assert status, f"[setup_rpo_arm_scene] URDF import failed: {urdf_path}"
    prim_path = prim_path or "/roboto_origin"
    print(f"[setup_rpo_arm_scene] Imported articulation prim: {prim_path}", flush=True)

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
print("[setup_rpo_arm_scene] Resetting world.", flush=True)
world.reset()

art = None
all_dof_names: list[str] = []
controlled_indices: list[int] = []
if using_controllable_simready:
    art = Articulation(articulation_prim_path)
    print("[setup_rpo_arm_scene] Initializing controllable SimReady arm articulation.", flush=True)
elif using_simready:
    print(
        "[setup_rpo_arm_scene] SimReady articulation binding is binding_pending; "
        "publishing mirrored 6D LeRobot state until prim mapping is authored.",
        flush=True,
    )
else:
    art = Articulation(prim_path)
    print("[setup_rpo_arm_scene] Initializing articulation.", flush=True)

if art is not None:
    art.initialize()
    num_dof = art.num_dof
    all_dof_names = list(art.dof_names)
    missing_joints = [
        name for name in CONTROLLED_PHYSICAL_JOINT_NAMES if name not in all_dof_names
    ]
    if missing_joints:
        loaded_asset_kind = "controllable SimReady" if using_controllable_simready else "URDF"
        print(
            f"[setup_rpo_arm_scene] ERROR: {loaded_asset_kind} controlled joints missing: "
            f"{missing_joints}\nAvailable joints: {all_dof_names}",
            flush=True,
        )
        simulation_app.close()
        sys.exit(1)

    controlled_indices = [
        all_dof_names.index(name) for name in CONTROLLED_PHYSICAL_JOINT_NAMES
    ]
    loaded_asset_kind = "controllable SimReady" if using_controllable_simready else "URDF"
    print(f"[setup_rpo_arm_scene] Loaded {num_dof} total {loaded_asset_kind} joints: {all_dof_names}", flush=True)
print(
    "[setup_rpo_arm_scene] LeRobot logical/physical joints: "
    f"logical={LOGICAL_JOINT_NAMES}, physical={PUBLISHED_JOINT_NAMES}",
    flush=True,
)


def _position_list(raw_positions) -> list[float]:
    arr = np.asarray(raw_positions, dtype=np.float32)
    if arr.ndim == 2:
        arr = arr[0]
    return arr.reshape(-1).astype(float).tolist()


def _write_command_evidence(
    *,
    command_seq: int,
    requested_logical_command: list[float],
    resolved_logical_command: list[float],
    physical_targets: dict[str, float],
    articulation_readback: list[float] | None,
    binding_status: str,
    error: str | None = None,
) -> None:
    if not COMMAND_EVIDENCE_PATH and not COMMAND_EVIDENCE_DIR:
        return
    payload = {
        "command_seq": command_seq,
        "controlled_joint_names": list(CONTROLLED_ARM_JOINT_NAMES),
        "logical_joint_names": list(LOGICAL_JOINT_NAMES),
        "physical_joint_names": list(CONTROLLED_PHYSICAL_JOINT_NAMES),
        "published_joint_names": list(PUBLISHED_JOINT_NAMES),
        "command": resolved_logical_command,
        "requested_logical_command": requested_logical_command,
        "resolved_logical_command": resolved_logical_command,
        "physical_targets": physical_targets,
        "articulation_readback": articulation_readback,
        "binding_status": binding_status,
        "error": error,
        "using_simready": bool(using_simready),
        "urdf_path": urdf_path,
        "simready_usd_path": simready_usd_path,
    }
    if COMMAND_EVIDENCE_PATH:
        os.makedirs(os.path.dirname(COMMAND_EVIDENCE_PATH), exist_ok=True)
        with open(COMMAND_EVIDENCE_PATH, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, sort_keys=True)
            f.write("\n")
    if COMMAND_EVIDENCE_DIR:
        os.makedirs(COMMAND_EVIDENCE_DIR, exist_ok=True)
        per_command_path = os.path.join(
            COMMAND_EVIDENCE_DIR, f"command_{command_seq:03d}_evidence.json"
        )
        with open(per_command_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, sort_keys=True)
            f.write("\n")


def _capture_rgb_screenshot(path: str, last_applied_command: list[float]) -> None:
    """Capture visual evidence without using Replicator in headless CI."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if os.path.exists(path):
        os.unlink(path)
    print(
        f"[setup_rpo_arm_scene] Capturing screenshot after LeRobot command "
        f"{last_applied_command} -> {path}",
        flush=True,
    )
    errors = []
    capture_methods = [
        ("camera sensor", _capture_camera_sensor_screenshot),
        ("renderer resource", _capture_renderer_resource_screenshot),
        ("viewport", _capture_viewport_screenshot),
    ]
    for label, capture_method in capture_methods:
        print(
            f"[setup_rpo_arm_scene] Trying {label} screenshot capture.",
            flush=True,
        )
        try:
            if os.path.exists(path):
                os.unlink(path)
            capture_method(path)
            if os.path.isfile(path) and os.path.getsize(path) > 0:
                print(
                    f"[setup_rpo_arm_scene] Screenshot saved via {label}: {path}",
                    flush=True,
                )
                return
            raise RuntimeError("capture returned but did not create a non-empty file")
        except BaseException as exc:
            if isinstance(exc, KeyboardInterrupt):
                raise
            errors.append(f"{label}: {type(exc).__name__}: {exc}")
            print(
                f"[setup_rpo_arm_scene] {label} screenshot failed: "
                f"{type(exc).__name__}: {exc}",
                flush=True,
            )
    _write_fallback_visual_evidence(path, errors)


def _save_png(path: str, rgba) -> None:
    arr = np.asarray(rgba)
    if arr.dtype != np.uint8:
        arr = np.clip(arr * 255 if arr.max(initial=0) <= 1.0 else arr, 0, 255).astype(np.uint8)
    try:
        from PIL import Image

        Image.fromarray(arr[:, :, :4]).save(path)
        return
    except Exception:
        import matplotlib.pyplot as plt

        plt.imsave(path, arr[:, :, :3])


def _fallback_bbox() -> tuple[np.ndarray, np.ndarray]:
    return np.array([-0.75, -0.75, 0.0], dtype=float), np.array([0.75, 0.75, 1.0], dtype=float)


def _is_reasonable_bbox(mn: np.ndarray, mx: np.ndarray) -> bool:
    if not np.all(np.isfinite(mn)) or not np.all(np.isfinite(mx)):
        return False
    extent = mx - mn
    extent_norm = float(np.linalg.norm(extent))
    return math.isfinite(extent_norm) and 1e-6 <= extent_norm <= 100.0 and np.all(np.abs(mn) < 100.0) and np.all(np.abs(mx) < 100.0)


def _bbox_for_prim(stage: Usd.Stage, focus_prim_path: str) -> tuple[np.ndarray, np.ndarray, str]:
    prim = stage.GetPrimAtPath(focus_prim_path)
    if not prim or not prim.IsValid():
        return (*_fallback_bbox(), "fallback_missing_prim")
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

    def compute(path_prim):
        box = cache.ComputeWorldBound(path_prim).ComputeAlignedBox()
        return np.array(box.GetMin(), dtype=float), np.array(box.GetMax(), dtype=float)

    mn, mx = compute(prim)
    if _is_reasonable_bbox(mn, mx):
        return mn, mx, "target"
    mins = []
    maxs = []
    for child in Usd.PrimRange(prim):
        try:
            cmn, cmx = compute(child)
        except Exception:
            continue
        if _is_reasonable_bbox(cmn, cmx):
            mins.append(cmn)
            maxs.append(cmx)
    if mins:
        return np.min(np.vstack(mins), axis=0), np.max(np.vstack(maxs), axis=0), "descendants"
    return (*_fallback_bbox(), "fallback_unbounded")


def _look_at_world_quat(position: np.ndarray, target: np.ndarray) -> np.ndarray:
    forward = np.asarray(target, dtype=float) - np.asarray(position, dtype=float)
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


def _ensure_debug_lights(stage: Usd.Stage) -> list[str]:
    light_paths = []
    dome = UsdLux.DomeLight.Define(stage, "/World/ViewportDebugDomeLight")
    dome.CreateIntensityAttr(600.0)
    light_paths.append(str(dome.GetPath()))
    distant = UsdLux.DistantLight.Define(stage, "/World/ViewportDebugDistantLight")
    distant.CreateIntensityAttr(2500.0)
    distant.CreateAngleAttr(0.5)
    light_paths.append(str(distant.GetPath()))
    return light_paths


def _capture_camera_sensor_screenshot(path: str) -> None:
    """Headless-safe capture of the live articulated pose using an Isaac Camera."""
    stage = omni.usd.get_context().get_stage()
    mn, mx, bbox_source = _bbox_for_prim(stage, prim_path)
    center = (mn + mx) / 2.0
    extent = mx - mn
    radius = min(max(float(np.linalg.norm(extent)), 0.5), 5.0)
    default_camera_position = center + np.array([radius * 0.9, -radius * 1.8, radius * 0.8 + 0.5], dtype=float)
    default_camera_target = center
    camera_position = np.array(SCREENSHOT_CAMERA_POSITION, dtype=float)
    camera_target = np.array(SCREENSHOT_CAMERA_TARGET, dtype=float)
    if not os.environ.get("SCREENSHOT_CAMERA_POSITION"):
        camera_position = default_camera_position
    if not os.environ.get("SCREENSHOT_CAMERA_TARGET"):
        camera_target = default_camera_target
    quat = _look_at_world_quat(camera_position, camera_target)
    _ensure_debug_lights(stage)
    camera_path = "/World/ViewportDebugCamera"
    camera = Camera(
        prim_path=camera_path,
        position=camera_position,
        orientation=quat,
        resolution=(1280, 720),
    )
    camera.initialize()
    camera.set_focal_length(SCREENSHOT_CAMERA_FOCAL_LENGTH)
    for _ in range(20):
        try:
            world.step(render=True)
        except Exception:
            simulation_app.update()
    rgba = camera.get_rgba()
    if rgba is None or np.asarray(rgba).size == 0:
        raise RuntimeError("Camera returned no RGBA data")
    _save_png(path, np.asarray(rgba))
    metadata_path = os.path.splitext(path)[0] + ".json"
    with open(metadata_path, "w", encoding="utf-8") as f:
        json.dump(
            {
                "camera_path": camera_path,
                "camera_position": camera_position.astype(float).tolist(),
                "camera_target": camera_target.astype(float).tolist(),
                "camera_focal_length": float(SCREENSHOT_CAMERA_FOCAL_LENGTH),
                "bbox_min": mn.astype(float).tolist(),
                "bbox_max": mx.astype(float).tolist(),
                "bbox_source": bbox_source,
                "focus_prim": prim_path,
                "resolution": [1280, 720],
                "capture_method": "isaacsim.sensors.camera.Camera.get_rgba",
            },
            f,
            indent=2,
            sort_keys=True,
        )
        f.write("\n")


def _capture_command_screenshot(
    sequence_dir: str,
    command_index: int,
    command_seq: int,
    last_applied_command: list[float],
    articulation_readback: list[float] | None,
) -> None:
    os.makedirs(sequence_dir, exist_ok=True)
    filename = f"command_{command_index:03d}.png"
    path = os.path.join(sequence_dir, filename)
    _capture_rgb_screenshot(path, last_applied_command)
    manifest_path = os.path.join(sequence_dir, "manifest.json")
    if os.path.isfile(manifest_path):
        with open(manifest_path, "r", encoding="utf-8") as f:
            manifest = json.load(f)
    else:
        manifest = {
            "command_sequence": True,
            "root_prim": prim_path,
            "controlled_joint_names": list(CONTROLLED_ARM_JOINT_NAMES),
            "published_joint_names": list(PUBLISHED_JOINT_NAMES),
            "images": [],
        }
    manifest["last_applied_command"] = [float(v) for v in last_applied_command]
    manifest.setdefault("images", []).append(
        {
            "filename": filename,
            "path": path,
            "command_index": int(command_index),
            "ros_command_seq": int(command_seq),
            "applied_command": [float(v) for v in last_applied_command],
            "articulation_readback": articulation_readback,
            "image_size_bytes": os.path.getsize(path) if os.path.isfile(path) else 0,
        }
    )
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
        f.write("\n")
    print(f"[setup_rpo_arm_scene] Command screenshot saved: {path}", flush=True)


def _write_fallback_visual_evidence(path: str, errors: list[str]) -> None:
    """Copy the SimReady pipeline thumbnail when headless capture APIs are unavailable."""
    if not using_simready or not os.path.isfile(SIMREADY_THUMBNAIL_PATH):
        raise RuntimeError("All screenshot capture methods failed: " + "; ".join(errors))

    shutil.copyfile(SIMREADY_THUMBNAIL_PATH, path)
    print(
        "[setup_rpo_arm_scene] Fallback visual evidence saved from "
        f"{SIMREADY_THUMBNAIL_PATH} after capture failures: {path}",
        flush=True,
    )


def _wait_for_screenshot_file(path: str, timeout_s: float = 10.0) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        simulation_app.update()
        if os.path.isfile(path) and os.path.getsize(path) > 0:
            return
        time.sleep(0.05)
    raise TimeoutError(f"Timed out waiting for screenshot file: {path}")


def _capture_renderer_resource_screenshot(path: str) -> None:
    """Capture the viewport's LDR render resource without blocking on async helpers."""
    import omni.kit.viewport_legacy
    import omni.renderer_capture

    renderer = omni.renderer_capture.acquire_renderer_capture_interface()
    viewport_interface = omni.kit.viewport_legacy.acquire_viewport_interface()
    viewport_window = viewport_interface.get_viewport_window(None)
    if viewport_window is None:
        raise RuntimeError("No legacy viewport window available")

    drawable_resource = None
    deadline = time.time() + 10.0
    while time.time() < deadline:
        drawable_resource = viewport_window.get_drawable_ldr_resource()
        if drawable_resource is not None:
            break
        simulation_app.update()
        time.sleep(0.05)
    if drawable_resource is None:
        raise TimeoutError("Timed out waiting for a drawable LDR viewport resource")

    renderer.capture_next_frame_rp_resource(path, drawable_resource)
    _wait_for_screenshot_file(path)


def _capture_viewport_screenshot(path: str) -> None:
    """Capture the active viewport if renderer-resource capture is unavailable."""
    from omni.kit.viewport.utility import (
        capture_viewport_to_file,
        frame_viewport_prims,
        get_active_viewport,
    )

    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("No active viewport available for screenshot capture")

    frame_viewport_prims(viewport, prims=[prim_path])
    simulation_app.update()

    async def _capture():
        capture = capture_viewport_to_file(viewport, file_path=path)
        await asyncio.wait_for(capture.wait_for_result(), timeout=10.0)

    loop = asyncio.get_event_loop()
    loop.run_until_complete(_capture())


if art is not None:
    all_positions = _position_list(art.get_joint_positions())
    current_physical_positions = [all_positions[i] for i in controlled_indices]
else:
    all_positions = [0.0] * len(CONTROLLED_PHYSICAL_JOINT_NAMES)
    current_physical_positions = list(all_positions)
current_grasp = 0.0
current_motion_code = 0.0
if USES_FIXED_MOTION_HAND:
    current_positions = list(current_physical_positions)
    current_logical_positions = infer_logical_state(
        dict(zip(CONTROLLED_PHYSICAL_JOINT_NAMES, current_positions, strict=True))
    )
    current_motion_code = current_logical_positions[-1]
else:
    current_positions = [*current_physical_positions, current_grasp]
    current_logical_positions = list(current_positions)
current_velocities = [0.0] * len(PUBLISHED_JOINT_NAMES)

rclpy.init()


class SimBridgeNode(Node):
    def __init__(self):
        super().__init__("isaac_sim_rpo_arm_bridge")
        self._cmd = None
        self._cmd_seq = 0
        self._cmd_lock = threading.Lock()
        self._debug_control = {
            "capture_every_command": bool(SCREENSHOT_AFTER_COMMAND and SCREENSHOT_EACH_COMMAND),
            "request_capture": False,
            "output_dir": SCREENSHOT_SEQUENCE_DIR,
        }
        self._debug_control_lock = threading.Lock()
        self._pub = self.create_publisher(JointState, "/follower/joint_states", 10)
        self._sub = self.create_subscription(
            Float64MultiArray, "/follower/joint_commands", self._cmd_cb, 10
        )
        self._debug_sub = self.create_subscription(
            String, "/follower/screenshot_debug", self._debug_control_cb, 10
        )

    def _cmd_cb(self, msg: Float64MultiArray):
        with self._cmd_lock:
            self._cmd = list(msg.data)
            self._cmd_seq += 1

    def _debug_control_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            print(
                f"[setup_rpo_arm_scene] Ignoring invalid screenshot debug JSON: {exc}",
                flush=True,
            )
            return
        if not isinstance(payload, dict):
            print(
                "[setup_rpo_arm_scene] Ignoring non-object screenshot debug payload.",
                flush=True,
            )
            return
        with self._debug_control_lock:
            if "capture_every_command" in payload:
                self._debug_control["capture_every_command"] = bool(payload["capture_every_command"])
            if "output_dir" in payload and str(payload["output_dir"]).strip():
                self._debug_control["output_dir"] = str(payload["output_dir"])
            if payload.get("request_capture"):
                self._debug_control["request_capture"] = True
        print(
            f"[setup_rpo_arm_scene] Screenshot debug control updated: {payload}",
            flush=True,
        )

    def get_command(self):
        with self._cmd_lock:
            cmd = list(self._cmd) if self._cmd is not None else None
            return self._cmd_seq, cmd

    def consume_debug_control(self):
        with self._debug_control_lock:
            debug_one_shot_requested = bool(self._debug_control.get("request_capture"))
            self._debug_control["request_capture"] = False
            return {
                "capture_every_command": bool(self._debug_control.get("capture_every_command")),
                "request_capture": debug_one_shot_requested,
                "output_dir": str(self._debug_control.get("output_dir") or SCREENSHOT_SEQUENCE_DIR),
            }

    def publish_state(self, names, positions, velocities, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = names
        msg.position = [float(p) for p in positions]
        msg.velocity = [float(v) for v in velocities]
        self._pub.publish(msg)


bridge = SimBridgeNode()
bridge_spin_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
bridge_spin_thread.start()


def _publish_current_state() -> None:
    stamp = bridge.get_clock().now().to_msg()
    bridge.publish_state(PUBLISHED_JOINT_NAMES, current_positions, current_velocities, stamp)

import omni.timeline  # noqa: E402

timeline = omni.timeline.get_timeline_interface()
timeline.play()

print("[setup_rpo_arm_scene] Simulation running. Ctrl+C to stop.", flush=True)
screenshot_taken = False
screenshot_command_capture_count = 0
last_processed_command_seq = -1
if SCREENSHOT_ON_STARTUP:
    print("[setup_rpo_arm_scene] Startup screenshot trigger accepted.", flush=True)
    _publish_current_state()
    _capture_rgb_screenshot(SCREENSHOT_PATH, list(current_positions))
    screenshot_taken = True
    if EXIT_AFTER_SCREENSHOT:
        print(
            "[setup_rpo_arm_scene] Exiting after startup screenshot as requested.",
            flush=True,
        )
        timeline.stop()
        rclpy.shutdown()
        bridge_spin_thread.join(timeout=2.0)
        bridge.destroy_node()
        simulation_app.close()
        sys.exit(0)
try:
    app_updated = False
    while simulation_app.is_running():
        # In this container build, repeated Kit/world stepping can block after the
        # first frame. Keep a mirrored ROS joint bridge responsive. For URDF fallback
        # push commanded joints into Isaac best-effort; for SimReady USD record
        # binding_pending evidence until articulation prims are mapped.
        if not app_updated:
            simulation_app.update()
            app_updated = True

        cmd_seq, cmd = bridge.get_command()
        debug_control = bridge.consume_debug_control()
        debug_capture_every_command = bool(debug_control["capture_every_command"])
        debug_one_shot_requested = bool(debug_control["request_capture"])
        debug_sequence_dir = str(debug_control["output_dir"])
        should_exit_after_screenshot = False

        if debug_one_shot_requested:
            print("[setup_rpo_arm_scene] One-shot screenshot debug trigger accepted.", flush=True)
            screenshot_command_capture_count += 1
            _publish_current_state()
            _capture_command_screenshot(
                debug_sequence_dir,
                screenshot_command_capture_count,
                max(last_processed_command_seq, 0),
                list(current_positions),
                None,
            )

        if cmd is not None and cmd_seq != last_processed_command_seq:
            last_processed_command_seq = cmd_seq
            command_values = [float(v) for v in cmd]
            physical_targets: dict[str, float]
            if USES_FIXED_MOTION_HAND:
                try:
                    resolution = resolve_logical_command(
                        command_values,
                        previous_motion_code=current_motion_code,
                    )
                except ValueError as exc:
                    error = f"invalid logical command #{cmd_seq}: {exc}"
                    print(f"[setup_rpo_arm_scene] Ignoring {error}", flush=True)
                    _write_command_evidence(
                        command_seq=cmd_seq,
                        requested_logical_command=command_values,
                        resolved_logical_command=list(current_logical_positions),
                        physical_targets=dict(
                            zip(PUBLISHED_JOINT_NAMES, current_positions, strict=True)
                        ),
                        articulation_readback=None,
                        binding_status="command_rejected",
                        error=error,
                    )
                    continue
                current_logical_positions = list(resolution.resolved_logical_command)
                current_motion_code = current_logical_positions[-1]
                physical_targets = dict(resolution.physical_targets)
                target_values = [
                    physical_targets[name] for name in CONTROLLED_PHYSICAL_JOINT_NAMES
                ]
                current_positions = list(target_values)
            else:
                if len(command_values) < len(CONTROLLED_ARM_JOINT_NAMES):
                    print(
                        "[setup_rpo_arm_scene] Ignoring short legacy command "
                        f"#{cmd_seq}: expected at least {len(CONTROLLED_ARM_JOINT_NAMES)}, "
                        f"got {len(command_values)}",
                        flush=True,
                    )
                    continue
                arm_command = command_values[: len(CONTROLLED_ARM_JOINT_NAMES)]
                current_grasp = float(
                    np.clip(
                        command_values[len(CONTROLLED_ARM_JOINT_NAMES)]
                        if len(command_values) > len(CONTROLLED_ARM_JOINT_NAMES)
                        else current_grasp,
                        0.0,
                        1.0,
                    )
                )
                current_logical_positions = [*arm_command, current_grasp]
                target_values = list(arm_command)
                current_positions = list(current_logical_positions)
                physical_targets = dict(
                    zip(CONTROLLED_ARM_JOINT_NAMES, arm_command, strict=True)
                )

            if art is not None:
                for idx, value in zip(controlled_indices, target_values, strict=True):
                    all_positions[idx] = value
                art.set_joint_positions(np.array([all_positions], dtype=np.float32))
                articulation_positions = _position_list(art.get_joint_positions())
                articulation_readback = [
                    articulation_positions[i] for i in controlled_indices
                ]
                if USES_FIXED_MOTION_HAND:
                    current_positions = list(articulation_readback)
                    current_logical_positions = infer_logical_state(
                        dict(
                            zip(
                                CONTROLLED_PHYSICAL_JOINT_NAMES,
                                articulation_readback,
                                strict=True,
                            )
                        )
                    )
                    current_motion_code = current_logical_positions[-1]
                else:
                    current_positions = [*articulation_readback, current_grasp]
                    current_logical_positions = list(current_positions)
                print(
                    "[setup_rpo_arm_scene] Articulation readback after command "
                    f"#{cmd_seq}: {articulation_readback}",
                    flush=True,
                )
            elif using_simready:
                articulation_readback = None
                _write_simready_mapping_evidence(
                    asset_path=simready_usd_path,
                    prim_path=prim_path,
                    binding_status="binding_pending",
                    last_command=list(current_logical_positions),
                )
            else:
                articulation_readback = None
            last_applied_command = list(current_logical_positions)
            _write_command_evidence(
                command_seq=cmd_seq,
                requested_logical_command=command_values,
                resolved_logical_command=last_applied_command,
                physical_targets=physical_targets,
                articulation_readback=articulation_readback,
                binding_status="articulation_bound" if art is not None else "binding_pending",
            )
            _publish_current_state()
            print(
                f"[setup_rpo_arm_scene] Applied LeRobot command #{cmd_seq}: "
                f"{last_applied_command}",
                flush=True,
            )

            env_command_screenshot_requested = SCREENSHOT_AFTER_COMMAND and (
                SCREENSHOT_EACH_COMMAND or not screenshot_taken
            )
            if env_command_screenshot_requested or debug_capture_every_command:
                print("[setup_rpo_arm_scene] Screenshot trigger accepted.", flush=True)
                if SCREENSHOT_EACH_COMMAND or debug_capture_every_command:
                    screenshot_command_capture_count += 1
                    _capture_command_screenshot(
                        debug_sequence_dir if debug_capture_every_command else SCREENSHOT_SEQUENCE_DIR,
                        screenshot_command_capture_count,
                        cmd_seq,
                        last_applied_command,
                        articulation_readback,
                    )
                else:
                    _capture_rgb_screenshot(SCREENSHOT_PATH, last_applied_command)
                screenshot_taken = True
                if env_command_screenshot_requested and EXIT_AFTER_SCREENSHOT and (
                    not SCREENSHOT_EACH_COMMAND
                    or screenshot_command_capture_count >= SCREENSHOT_EXIT_AFTER_COMMAND_COUNT
                ):
                    print(
                        "[setup_rpo_arm_scene] Exiting after screenshot as requested.",
                        flush=True,
                    )
                    should_exit_after_screenshot = True

        _publish_current_state()
        if should_exit_after_screenshot:
            for _ in range(10):
                time.sleep(0.05)
                _publish_current_state()
            break
        time.sleep(1.0 / 60.0)

except KeyboardInterrupt:
    pass
finally:
    print("[setup_rpo_arm_scene] Shutting down.", flush=True)
    timeline.stop()
    rclpy.shutdown()
    bridge_spin_thread.join(timeout=2.0)
    bridge.destroy_node()
    simulation_app.close()
