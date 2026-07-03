"""
Isaac Sim 5.1.0 physical scene script.

By default this script loads the custom SimReady USD assembly as the visual
robot/frame and imports the generated physical URDF for the movable
Roboto V2 right arm + AmazingHand hand. Set USE_SIMREADY_USD=1 only to debug
the optional SimReady USD visual/provenance path as the primary scene asset.

Run via: /isaac-sim/python.sh /workspace/isaacsim/setup_rpo_arm_scene.py
"""
from __future__ import annotations

import argparse
import asyncio
import glob
import json
import os
import shutil
import sys
import threading
import time
import tempfile

import numpy as np

for contract_dir in (
    "/workspace/superarm_ws/isaacsim_test/lerobot",
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "lerobot")),
):
    if os.path.isdir(contract_dir) and contract_dir not in sys.path:
        sys.path.insert(0, contract_dir)

from rpo_arm_contract import (  # noqa: E402
    AMAZINGHAND_MOTOR_JOINT_NAMES,
    ARM_JOINT_NAMES,
    DEFAULT_MIDDLE_POS_DEG,
    GRASP_JOINT_NAME,
    JOINT_NAMES,
    grasp_scalar_to_servo_targets,
    normalize_action,
)

CONTROLLED_ARM_JOINT_NAMES = list(ARM_JOINT_NAMES)
SYNTHETIC_GRASP_NAME = GRASP_JOINT_NAME
PUBLISHED_JOINT_NAMES = list(JOINT_NAMES)
DEFAULT_SIMREADY_USD = (
    "/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/"
    "pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/"
    "echo_full_robot_arm_hand.usd"
)
DEFAULT_SIMREADY_ARTICULATION_USD = "/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/sitl/echo_full_lerobot_articulation.usda"
DEFAULT_CUSTOM_VISUAL_USD = DEFAULT_SIMREADY_ARTICULATION_USD
DEFAULT_ARTICULATION_REPORT = "/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/sitl/echo_full_lerobot_articulation_report.json"
DEFAULT_PHYSICAL_ROBOT_URDF = "/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/sitl/roboto_v2_right_arm_amazinghand_full.urdf"
DEFAULT_SIMREADY_PRIM_PATH = "/World/echo_full_simready"
DEFAULT_CUSTOM_VISUAL_PRIM_PATH = "/World/echo_full_visual"
DEFAULT_SIMREADY_MAPPING_PATH = (
    "/workspace/superarm_ws/isaacsim_test/artifacts/"
    "simready_prim_mapping.json"
)
DEFAULT_SIMREADY_THUMBNAIL_PATH = (
    "/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/"
    "pipeline/07_render/thumbnail.png"
)
DEFAULT_SCREENSHOT_PATH = (
    "/workspace/superarm_ws/isaacsim_test/artifacts/"
    "echo_full_simready_target.png"
)
DEFAULT_MOTION_SCREENSHOT_OUTPUT_DIR = (
    "/workspace/superarm_ws/isaacsim_test/artifacts/"
    "simready_motion_cases"
)
# BBoxCache on the manifest wrapper can report only a small referenced sub-tree
# in headless Replicator. Clamp the camera radius while keeping the measured
# bbox center so motion screenshots show the wrist/hand connection instead of a
# far top-down frame or an empty fixed-center crop.
DEFAULT_CUSTOM_VISUAL_CAPTURE_RADIUS = 0.28
MIN_CUSTOM_VISUAL_CAPTURE_RADIUS = 0.35
WRIST_ATTACHMENT_PARENT_LINK = "right_elbow_yaw_link"
WRIST_ATTACHMENT_CHILD_LINK = "r_wrist_interface"
WRIST_ATTACHMENT_ORIGIN_XYZ = (0.13825, 0.0, 0.0)
WRIST_ATTACHMENT_MAX_GAP_M = 0.005


def _env_flag(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _env_int(name: str, default: int) -> int:
    value = os.environ.get(name)
    if value is None or not value.strip():
        return default
    try:
        return int(value)
    except ValueError:
        print(
            f"[setup_rpo_arm_scene] WARNING: invalid integer for {name}: {value!r}; "
            f"using {default}",
            flush=True,
        )
        return default


def _env_float_triplet(name: str, default: tuple[float, float, float]) -> tuple[float, float, float]:
    value = os.environ.get(name)
    if value is None or not value.strip():
        return default
    try:
        parts = [float(part) for part in value.replace(",", " ").split()]
    except ValueError:
        parts = []
    if len(parts) != 3:
        print(
            f"[setup_rpo_arm_scene] WARNING: invalid xyz triplet for {name}: {value!r}; "
            f"using {default}",
            flush=True,
        )
        return default
    return (parts[0], parts[1], parts[2])


SCREENSHOT_AFTER_COMMAND = _env_flag("SCREENSHOT_AFTER_COMMAND")
SCREENSHOT_ON_STARTUP = _env_flag("SCREENSHOT_ON_STARTUP")
SCREENSHOT_PATH = os.environ.get("SCREENSHOT_PATH", DEFAULT_SCREENSHOT_PATH)
EXIT_AFTER_SCREENSHOT = _env_flag(
    "EXIT_AFTER_SCREENSHOT", default=SCREENSHOT_AFTER_COMMAND or SCREENSHOT_ON_STARTUP
)
MOTION_SCREENSHOT_CASES_PATH = os.environ.get("MOTION_SCREENSHOT_CASES_PATH", "")
MOTION_SCREENSHOT_CASES_JSON = os.environ.get("MOTION_SCREENSHOT_CASES_JSON", "")
MOTION_SCREENSHOT_OUTPUT_DIR = os.environ.get(
    "MOTION_SCREENSHOT_OUTPUT_DIR", DEFAULT_MOTION_SCREENSHOT_OUTPUT_DIR
)
MOTION_SCREENSHOT_KINEMATIC_CAPTURE = _env_flag("MOTION_SCREENSHOT_KINEMATIC_CAPTURE")
MOTION_SCREENSHOT_SETTLE_STEPS = max(0, _env_int("MOTION_SCREENSHOT_SETTLE_STEPS", 30))
EXIT_AFTER_MOTION_SCREENSHOTS = _env_flag(
    "EXIT_AFTER_MOTION_SCREENSHOTS",
    default=bool(MOTION_SCREENSHOT_CASES_PATH.strip() or MOTION_SCREENSHOT_CASES_JSON.strip()),
)
SIMREADY_PRIM_PATH = os.environ.get("SIMREADY_PRIM_PATH", DEFAULT_SIMREADY_PRIM_PATH)
SIMREADY_MAPPING_PATH = os.environ.get("SIMREADY_MAPPING_PATH", DEFAULT_SIMREADY_MAPPING_PATH)
SIMREADY_THUMBNAIL_PATH = os.environ.get(
    "SIMREADY_THUMBNAIL_PATH", DEFAULT_SIMREADY_THUMBNAIL_PATH
)
LOAD_CUSTOM_VISUAL_USD = _env_flag("LOAD_CUSTOM_VISUAL_USD", default=True)
CUSTOM_VISUAL_USD_PATH = os.environ.get("CUSTOM_VISUAL_USD_PATH", DEFAULT_CUSTOM_VISUAL_USD)
CUSTOM_VISUAL_PRIM_PATH = os.environ.get(
    "CUSTOM_VISUAL_PRIM_PATH", DEFAULT_CUSTOM_VISUAL_PRIM_PATH
)
CUSTOM_VISUAL_FOLLOW_LINK = os.environ.get("CUSTOM_VISUAL_FOLLOW_LINK", "").strip()
CUSTOM_VISUAL_FOLLOW_XYZ = _env_float_triplet("CUSTOM_VISUAL_FOLLOW_XYZ", (0.0, 0.0, 0.0))

parser = argparse.ArgumentParser()
parser.add_argument(
    "--headless",
    action="store_true",
    default=bool(int(os.environ.get("HEADLESS", "1"))),
)
args, _ = parser.parse_known_args()
CONTINUOUS_APP_UPDATE = _env_flag("CONTINUOUS_APP_UPDATE", default=not args.headless)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension, get_extension_path_from_name  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.importer.urdf")
enable_extension("isaacsim.test.utils")
enable_extension("omni.kit.renderer.capture")
simulation_app.update()

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

ext_path = get_extension_path_from_name("isaacsim.asset.importer.urdf")
USE_SIMREADY_USD = _env_flag("USE_SIMREADY_USD", default=False)
USE_SIMREADY_ARTICULATION_USD = _env_flag("USE_SIMREADY_ARTICULATION_USD", default=False)
ECHO_FULL_ARM_ONLY = _env_flag("ECHO_FULL_ARM_ONLY", default=False)
PUBLISHED_JOINT_NAMES = (
    list(CONTROLLED_ARM_JOINT_NAMES) if ECHO_FULL_ARM_ONLY else list(JOINT_NAMES)
)
SIMREADY_ARTICULATION_USD_CANDIDATES = (
    [
        os.environ.get("SIMREADY_ARTICULATION_USD_PATH", ""),
        DEFAULT_SIMREADY_ARTICULATION_USD,
    ]
    if USE_SIMREADY_USD and USE_SIMREADY_ARTICULATION_USD
    else []
)
SIMREADY_USD_CANDIDATES = (
    [
        os.environ.get("SIMREADY_USD_PATH", ""),
        DEFAULT_SIMREADY_USD,
    ]
    if USE_SIMREADY_USD
    else []
)
URDF_CANDIDATES = [
    os.environ.get("PHYSICAL_ROBOT_URDF_PATH", DEFAULT_PHYSICAL_ROBOT_URDF),
    DEFAULT_PHYSICAL_ROBOT_URDF,
]
simready_articulation_usd_path = next(
    (p for p in SIMREADY_ARTICULATION_USD_CANDIDATES if p and os.path.isfile(p)),
    None,
)
simready_usd_path = simready_articulation_usd_path or next(
    (p for p in SIMREADY_USD_CANDIDATES if p and os.path.isfile(p)),
    None,
)
urdf_path = next((p for p in URDF_CANDIDATES if p and os.path.isfile(p)), None)
using_articulated_simready = simready_articulation_usd_path is not None
using_simready = simready_usd_path is not None


def _load_simready_usd(asset_path: str, prim_path: str) -> str:
    stage = omni.usd.get_context().get_stage()
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(asset_path)
    UsdGeom.Xformable(prim)
    return str(prim.GetPath())


def _load_custom_visual_usd(asset_path: str, prim_path: str) -> str | None:
    """Reference the user's custom robot/frame visual USD beside the physical URDF."""
    if not LOAD_CUSTOM_VISUAL_USD:
        print("[setup_rpo_arm_scene] Custom visual USD loading disabled.", flush=True)
        return None
    if not asset_path or not os.path.isfile(asset_path):
        print(
            f"[setup_rpo_arm_scene] WARNING: custom visual USD not found: {asset_path}",
            flush=True,
        )
        return None

    loaded_prim_path = _load_simready_usd(asset_path, prim_path)
    print(
        f"[setup_rpo_arm_scene] Loading custom visual USD: {asset_path} -> {loaded_prim_path}",
        flush=True,
    )
    return loaded_prim_path


def _control_contract() -> list[str]:
    return [f"{name}.pos" if not name.endswith(".pos") else name for name in PUBLISHED_JOINT_NAMES]


def _repo_relative_path(path: str) -> str:
    prefix = "/workspace/superarm_ws/"
    normalized = path.replace("\\", "/")
    if normalized.startswith(prefix):
        return normalized[len(prefix) :]
    return normalized


def _imported_articulation_root_path() -> str:
    if prim_path.endswith("/root_joint"):
        return prim_path.rsplit("/", maxsplit=1)[0]
    return prim_path


def _compute_wrist_attachment_runtime_validation() -> dict[str, object]:
    """Measure the fixed arm-end to hand-root connection in the live USD stage."""
    if ECHO_FULL_ARM_ONLY:
        return {
            "status": "SKIPPED",
            "reason": "Arm-only package intentionally excludes AmazingHand/wrist-interface links.",
        }
    if using_simready:
        return {
            "status": "SKIPPED",
            "reason": "Runtime wrist-gap check applies to the direct imported physical URDF.",
        }

    stage = omni.usd.get_context().get_stage()
    root_path = _imported_articulation_root_path()
    parent_path = f"{root_path}/{WRIST_ATTACHMENT_PARENT_LINK}"
    child_path = f"{root_path}/{WRIST_ATTACHMENT_CHILD_LINK}"
    parent_prim = stage.GetPrimAtPath(parent_path)
    child_prim = stage.GetPrimAtPath(child_path)
    if not parent_prim or not parent_prim.IsValid() or not child_prim or not child_prim.IsValid():
        return {
            "status": "FAIL",
            "articulation_root_path": root_path,
            "parent_link_path": parent_path,
            "child_link_path": child_path,
            "reason": "Could not find imported wrist attachment parent/child link prims.",
        }

    parent_world = UsdGeom.Xformable(parent_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    child_world = UsdGeom.Xformable(child_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    expected_child_world = parent_world.Transform(Gf.Vec3d(*WRIST_ATTACHMENT_ORIGIN_XYZ))
    actual_child_world = child_world.ExtractTranslation()
    gap_m = float((actual_child_world - expected_child_world).GetLength())
    status = "PASS" if gap_m <= WRIST_ATTACHMENT_MAX_GAP_M else "FAIL"
    payload = {
        "status": status,
        "articulation_root_path": root_path,
        "parent_link": WRIST_ATTACHMENT_PARENT_LINK,
        "parent_link_path": parent_path,
        "child_link": WRIST_ATTACHMENT_CHILD_LINK,
        "child_link_path": child_path,
        "expected_parent_local_origin_xyz": list(WRIST_ATTACHMENT_ORIGIN_XYZ),
        "expected_child_world_xyz": [float(value) for value in expected_child_world],
        "actual_child_world_xyz": [float(value) for value in actual_child_world],
        "gap_m": gap_m,
        "max_allowed_gap_m": WRIST_ATTACHMENT_MAX_GAP_M,
    }
    print(
        "[setup_rpo_arm_scene] Wrist attachment runtime validation: "
        f"status={status}, gap_m={gap_m:.6f}, parent={parent_path}, child={child_path}",
        flush=True,
    )
    return payload


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
    for feature_name in PUBLISHED_JOINT_NAMES:
        feature = f"{feature_name}.pos" if not feature_name.endswith(".pos") else feature_name
        if binding_status == "bound" and feature_name in CONTROLLED_ARM_JOINT_NAMES:
            status = "bound"
            usd_prim = f"{prim_path}/lerobot_joints/{feature_name}"
            reason = "Bound to generated SimReady USD PhysicsRevoluteJoint articulation."
        elif binding_status == "bound" and feature_name == SYNTHETIC_GRASP_NAME:
            status = "synthetic"
            usd_prim = None
            reason = "AmazingHand grasp remains a normalized synthetic LeRobot channel."
        else:
            status = "binding_pending"
            usd_prim = None
            reason = (
                "SimReady USD is loaded as the primary visual/physics asset; "
                "articulation mapping must be authored from prim inspection."
            )
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
    feature_statuses = {item["feature"]: item["binding_status"] for item in feature_bindings}
    payload = {
        "asset": os.path.basename(asset_path),
        "asset_path": asset_path,
        "prim_path": prim_path,
        "simready_root_prim": prim_path,
        "prim_hierarchy": prim_hierarchy,
        "prim_hierarchy_truncated": prim_hierarchy_truncated,
        "control_contract": control_contract,
        "binding_status": binding_status,
        "bound_or_binding_pending_per_feature": feature_statuses,
        "feature_bindings": feature_bindings,
        "last_command": last_command,
        "next_step": (
            "drive bound SimReady articulation joints"
            if binding_status == "bound"
            else "inspect SimReady USD prim hierarchy and bind controllable articulation prims"
        ),
    }
    with open(SIMREADY_MAPPING_PATH, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, sort_keys=True)
        f.write("\n")

custom_visual_prim_path: str | None = None
custom_visual_follow_target_path: str | None = None

if using_simready:
    prim_path = _load_simready_usd(simready_usd_path, SIMREADY_PRIM_PATH)
    if using_articulated_simready:
        print(
            f"[setup_rpo_arm_scene] Loading articulated SimReady USD: {simready_usd_path}",
            flush=True,
        )
    else:
        print(f"[setup_rpo_arm_scene] Loading SimReady USD: {simready_usd_path}", flush=True)
    print(f"[setup_rpo_arm_scene] SimReady prim path: {prim_path}", flush=True)
    _write_simready_mapping_evidence(
        asset_path=simready_usd_path,
        prim_path=prim_path,
        binding_status="bound" if using_articulated_simready else "binding_pending",
    )
elif urdf_path is None:
    print(
        "[setup_rpo_arm_scene] ERROR: No physical robot URDF found. Tried:\n"
        + "\n".join(f"  {p}" for p in SIMREADY_ARTICULATION_USD_CANDIDATES if p)
        + "\n"
        + "\n".join(f"  {p}" for p in SIMREADY_USD_CANDIDATES if p)
        + "\n"
        + "\n".join(f"  {p}" for p in URDF_CANDIDATES if p)
        + "\n\nGenerate the direct physical arm+hand URDF artifact "
        "or set PHYSICAL_ROBOT_URDF_PATH to an equivalent custom robot URDF. "
        "Set USE_SIMREADY_USD=1 only for SimReady visual/provenance diagnostics."
        + f"\nURDF importer extension path: {ext_path}",
        flush=True,
    )
    simulation_app.close()
    sys.exit(1)
else:
    custom_visual_prim_path = _load_custom_visual_usd(CUSTOM_VISUAL_USD_PATH, CUSTOM_VISUAL_PRIM_PATH)
    print(f"[setup_rpo_arm_scene] Loading physical robot URDF: {urdf_path}", flush=True)

print(
    "[setup_rpo_arm_scene] Screenshot config: "
    f"after_command={SCREENSHOT_AFTER_COMMAND}, "
    f"on_startup={SCREENSHOT_ON_STARTUP}, "
    f"path={SCREENSHOT_PATH}, "
    f"exit_after={EXIT_AFTER_SCREENSHOT}, "
    f"motion_cases_path={MOTION_SCREENSHOT_CASES_PATH or '<unset>'}, "
    f"motion_cases_json={'set' if MOTION_SCREENSHOT_CASES_JSON.strip() else '<unset>'}, "
    f"motion_output_dir={MOTION_SCREENSHOT_OUTPUT_DIR}, "
    f"motion_settle_steps={MOTION_SCREENSHOT_SETTLE_STEPS}, "
    f"motion_kinematic_capture={MOTION_SCREENSHOT_KINEMATIC_CAPTURE}, "
    f"exit_after_motion={EXIT_AFTER_MOTION_SCREENSHOTS}, "
    f"load_custom_visual_usd={LOAD_CUSTOM_VISUAL_USD}, "
    f"custom_visual_usd_path={CUSTOM_VISUAL_USD_PATH}, "
    f"custom_visual_prim_path={CUSTOM_VISUAL_PRIM_PATH}, "
    f"custom_visual_follow_link={CUSTOM_VISUAL_FOLLOW_LINK or '<unset>'}, "
    f"custom_visual_follow_xyz={CUSTOM_VISUAL_FOLLOW_XYZ}",
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
hand_motor_joint_names: list[str] = []
hand_motor_indices: list[int] = []
missing_hand_motor_joints: list[str] = []
if using_articulated_simready:
    art = Articulation(prim_path)
    print("[setup_rpo_arm_scene] SimReady articulation binding is bound.", flush=True)
    print("[setup_rpo_arm_scene] Initializing SimReady articulation.", flush=True)
    art.initialize()

    num_dof = art.num_dof
    all_dof_names = list(art.dof_names)
    missing_joints = [name for name in CONTROLLED_ARM_JOINT_NAMES if name not in all_dof_names]
    if missing_joints:
        print(
            "[setup_rpo_arm_scene] ERROR: Generated SimReady articulation joints missing: "
            f"{missing_joints}\nAvailable joints: {all_dof_names}",
            flush=True,
        )
        simulation_app.close()
        sys.exit(1)

    controlled_indices = [all_dof_names.index(name) for name in CONTROLLED_ARM_JOINT_NAMES]
    if not ECHO_FULL_ARM_ONLY:
        missing_hand_motor_joints = [
            name for name in AMAZINGHAND_MOTOR_JOINT_NAMES if name not in all_dof_names
        ]
        hand_motor_indices = [
            all_dof_names.index(name)
            for name in AMAZINGHAND_MOTOR_JOINT_NAMES
            if name in all_dof_names
        ]
        hand_motor_joint_names = [
            name for name in AMAZINGHAND_MOTOR_JOINT_NAMES if name in all_dof_names
        ]
    print(
        f"[setup_rpo_arm_scene] Loaded {num_dof} total SimReady articulation joints: "
        f"{all_dof_names}",
        flush=True,
    )
    set_camera_view(
        eye=np.array([1.1, -1.8, 0.9]),
        target=np.array([0.0, 0.0, 0.35]),
        camera_prim_path="/OmniverseKit_Persp",
    )
elif using_simready:
    print(
        "[setup_rpo_arm_scene] SimReady articulation binding is binding_pending; "
        "publishing mirrored 6D LeRobot state until prim mapping is authored.",
        flush=True,
    )
else:
    art = Articulation(prim_path)
    print("[setup_rpo_arm_scene] Initializing articulation.", flush=True)
    art.initialize()

    num_dof = art.num_dof
    all_dof_names = list(art.dof_names)
    missing_joints = [name for name in CONTROLLED_ARM_JOINT_NAMES if name not in all_dof_names]
    if missing_joints:
        print(
            "[setup_rpo_arm_scene] ERROR: Required right-arm joints missing from imported URDF: "
            f"{missing_joints}\nAvailable joints: {all_dof_names}",
            flush=True,
        )
        simulation_app.close()
        sys.exit(1)

    controlled_indices = [all_dof_names.index(name) for name in CONTROLLED_ARM_JOINT_NAMES]
    if not ECHO_FULL_ARM_ONLY:
        missing_hand_motor_joints = [
            name for name in AMAZINGHAND_MOTOR_JOINT_NAMES if name not in all_dof_names
        ]
        hand_motor_indices = [
            all_dof_names.index(name)
            for name in AMAZINGHAND_MOTOR_JOINT_NAMES
            if name in all_dof_names
        ]
        hand_motor_joint_names = [
            name for name in AMAZINGHAND_MOTOR_JOINT_NAMES if name in all_dof_names
        ]
    print(f"[setup_rpo_arm_scene] Loaded {num_dof} total URDF joints: {all_dof_names}", flush=True)
    set_camera_view(
        eye=np.array([1.1, -1.8, 0.9]),
        target=np.array([0.0, 0.0, 0.35]),
        camera_prim_path="/OmniverseKit_Persp",
    )
print(
    "[setup_rpo_arm_scene] Controlled LeRobot joints: "
    f"{PUBLISHED_JOINT_NAMES}",
    flush=True,
)
if hand_motor_indices:
    print(
        "[setup_rpo_arm_scene] AmazingHand motor joints commanded from "
        f"{SYNTHETIC_GRASP_NAME}: {AMAZINGHAND_MOTOR_JOINT_NAMES}",
        flush=True,
    )
if missing_hand_motor_joints:
    print(
        "[setup_rpo_arm_scene] WARNING: AmazingHand motor joints missing from "
        f"loaded articulation: {missing_hand_motor_joints}",
        flush=True,
    )


def _find_descendant_prim_by_name(root_prim_path: str, prim_name: str) -> Usd.Prim | None:
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_prim_path)
    if not root_prim or not root_prim.IsValid():
        return None
    root_prefix = root_prim_path.rstrip("/") + "/"
    for candidate in stage.Traverse():
        candidate_path = str(candidate.GetPath())
        if candidate_path == root_prim_path or candidate_path.startswith(root_prefix):
            if candidate.GetName() == prim_name:
                return candidate
    return None


def _resolve_custom_visual_follow_target() -> str | None:
    if not custom_visual_prim_path or not CUSTOM_VISUAL_FOLLOW_LINK:
        return None
    search_root_path = prim_path.rsplit("/", maxsplit=1)[0] if "/" in prim_path.strip("/") else prim_path
    target_prim = _find_descendant_prim_by_name(search_root_path, CUSTOM_VISUAL_FOLLOW_LINK)
    if target_prim is None or not target_prim.IsValid():
        print(
            "[setup_rpo_arm_scene] WARNING: custom visual follow target link not found: "
            f"{CUSTOM_VISUAL_FOLLOW_LINK}",
            flush=True,
        )
        return None
    resolved_path = str(target_prim.GetPath())
    print(
        "[setup_rpo_arm_scene] Custom visual will follow link "
        f"{CUSTOM_VISUAL_FOLLOW_LINK}: {resolved_path}",
        flush=True,
    )
    return resolved_path


def _sync_custom_visual_to_follow_link() -> None:
    if not custom_visual_prim_path or not custom_visual_follow_target_path:
        return

    stage = omni.usd.get_context().get_stage()
    visual_prim = stage.GetPrimAtPath(custom_visual_prim_path)
    target_prim = stage.GetPrimAtPath(custom_visual_follow_target_path)
    if not visual_prim or not visual_prim.IsValid() or not target_prim or not target_prim.IsValid():
        return

    target_world = UsdGeom.Xformable(target_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    offset = Gf.Matrix4d().SetTranslate(Gf.Vec3d(*CUSTOM_VISUAL_FOLLOW_XYZ))
    xformable = UsdGeom.Xformable(visual_prim)
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp().Set(offset * target_world)
    simulation_app.update()


custom_visual_follow_target_path = _resolve_custom_visual_follow_target()


def _position_list(raw_positions) -> list[float]:
    arr = np.asarray(raw_positions, dtype=np.float32)
    if arr.ndim == 2:
        arr = arr[0]
    return arr.reshape(-1).astype(float).tolist()


def _sanitize_motion_case_name(raw_name: object, index: int) -> str:
    name = str(raw_name or f"case_{index:02d}").strip().lower()
    safe = "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in name)
    safe = "_".join(part for part in safe.split("_") if part)
    return safe or f"case_{index:02d}"


def _motion_case_command(case_payload: dict, index: int) -> list[float]:
    positions = case_payload.get("positions", case_payload.get("command"))
    if isinstance(positions, dict):
        values = [positions.get(joint_name, 0.0) for joint_name in PUBLISHED_JOINT_NAMES]
    elif isinstance(positions, list):
        values = list(positions)
    else:
        raise ValueError(
            f"Motion screenshot case #{index} must contain positions as a list or joint map"
        )

    if len(values) < len(CONTROLLED_ARM_JOINT_NAMES):
        raise ValueError(
            f"Motion screenshot case #{index} has {len(values)} values; "
            f"expected at least {len(CONTROLLED_ARM_JOINT_NAMES)} arm joints"
        )
    if len(values) == len(CONTROLLED_ARM_JOINT_NAMES) and not ECHO_FULL_ARM_ONLY:
        values.append(0.0)
    return [float(value) for value in values[: len(PUBLISHED_JOINT_NAMES)]]


def _parse_motion_screenshot_cases() -> list[dict]:
    raw_cases = MOTION_SCREENSHOT_CASES_JSON.strip()
    source = "MOTION_SCREENSHOT_CASES_JSON"
    if not raw_cases and MOTION_SCREENSHOT_CASES_PATH.strip():
        source = MOTION_SCREENSHOT_CASES_PATH
        with open(MOTION_SCREENSHOT_CASES_PATH, "r", encoding="utf-8") as f:
            raw_cases = f.read().strip()
    if not raw_cases:
        return []

    payload = json.loads(raw_cases)
    if isinstance(payload, dict):
        payload = payload.get("cases", [])
    if not isinstance(payload, list):
        raise ValueError(f"Motion screenshot cases from {source} must be a list or contain cases[]")

    cases = []
    seen_names = set()
    for index, case_payload in enumerate(payload, start=1):
        if not isinstance(case_payload, dict):
            raise ValueError(f"Motion screenshot case #{index} must be an object")
        name = _sanitize_motion_case_name(case_payload.get("name"), index)
        if name in seen_names:
            name = f"{name}_{index:02d}"
        seen_names.add(name)
        cases.append(
            {
                "name": name,
                "command": _motion_case_command(case_payload, index),
            }
        )
    return cases


def _capture_rgb_screenshot(path: str, last_applied_command: list[float]) -> None:
    """Capture visual evidence without using Replicator in headless CI."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    print(
        f"[setup_rpo_arm_scene] Capturing screenshot after LeRobot command "
        f"{last_applied_command} -> {path}",
        flush=True,
    )
    errors = []
    capture_methods = [
        ("renderer resource", _capture_renderer_resource_screenshot),
        ("viewport", _capture_viewport_screenshot),
        ("replicator", _capture_replicator_screenshot),
    ]
    for label, capture_method in capture_methods:
        print(
            f"[setup_rpo_arm_scene] Trying {label} screenshot capture.",
            flush=True,
        )
        try:
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


def _write_runtime_validation_report(motion_results: list[dict[str, object]]) -> None:
    """Append direct-URDF runtime evidence to the generated articulation report."""
    report_path = os.environ.get("SIMREADY_ARTICULATION_REPORT_PATH", DEFAULT_ARTICULATION_REPORT)
    report = {}
    if os.path.isfile(report_path):
        try:
            with open(report_path, "r", encoding="utf-8") as f:
                loaded_report = json.load(f)
            if isinstance(loaded_report, dict):
                report = loaded_report
        except (OSError, json.JSONDecodeError) as exc:
            print(
                f"[setup_rpo_arm_scene] WARNING: could not read articulation report "
                f"{report_path}: {type(exc).__name__}: {exc}",
                flush=True,
            )

    runtime_log_path = os.environ.get("ISAAC_SIM_RUNTIME_LOG_PATH", "").strip()
    contact_sheet_path = os.environ.get(
        "MOTION_SCREENSHOT_CONTACT_SHEET_PATH",
        "/workspace/superarm_ws/isaacsim_test/artifacts/simready_motion_cases_contact_sheet.png",
    ).strip()
    wrist_attachment_runtime_validation = _compute_wrist_attachment_runtime_validation()
    runtime_status = (
        "PASS"
        if wrist_attachment_runtime_validation.get("status") in {"PASS", "SKIPPED"}
        else "FAIL"
    )
    controlled_dofs_moved = list(CONTROLLED_ARM_JOINT_NAMES)
    if not ECHO_FULL_ARM_ONLY:
        controlled_dofs_moved.extend(
            name
            for name in AMAZINGHAND_MOTOR_JOINT_NAMES
            if name not in missing_hand_motor_joints
        )
    hand_motor_control_status = (
        "SKIPPED" if ECHO_FULL_ARM_ONLY else "PASS" if not missing_hand_motor_joints else "WARN"
    )
    urdf_constraint_fidelity = (
        {
            "status": "NOT_APPLICABLE",
            "mjcf_constraints_preserved": None,
            "omitted_mjcf_features": [],
            "note": (
                "Arm-only runtime intentionally excludes AmazingHand, so MJCF "
                "finger equality/connect constraints are outside this test scope."
            ),
        }
        if ECHO_FULL_ARM_ONLY
        else {
            "status": "LOSSY_MJCF_CONVERSION",
            "mjcf_constraints_preserved": False,
            "omitted_mjcf_features": ["equality/connect"],
            "note": (
                "AmazingHand is loaded through URDFParseAndImportFile for this scene. "
                "The upstream MJCF equality/connect constraints are not represented in URDF."
            ),
        }
    )
    evidence_summary = (
        "Isaac Sim imported the arm-only Roboto V2 URDF, controlled the five "
        "right-arm DOFs, applied the requested motion cases, saved non-empty "
        "screenshots, and skipped wrist/hand checks because AmazingHand and "
        "finger links are intentionally absent."
        if ECHO_FULL_ARM_ONLY
        else (
            "Isaac Sim loaded the configured visual context, imported the generated "
            "physical arm+hand URDF, controlled the five Roboto V2 right-arm DOFs "
            "and available AmazingHand motor DOFs from amazinghand_grasp, applied "
            "LeRobot motion cases, saved non-empty screenshots, and measured the "
            "right_elbow_yaw_link endpoint to r_wrist_interface runtime wrist gap. "
            "The URDF path remains a lossy MJCF conversion and does not preserve "
            "AmazingHand equality/connect constraints."
        )
    )
    runtime_validation = {
        "status": runtime_status,
        "loader": "URDFParseAndImportFile" if not using_simready else "SimReady USD",
        "physical_robot_urdf_path": _repo_relative_path(urdf_path or ""),
        "custom_visual_usd_path": _repo_relative_path(CUSTOM_VISUAL_USD_PATH),
        "custom_visual_prim_path": custom_visual_prim_path,
        "loaded_dof_count": len(all_dof_names),
        "loaded_dof_names": list(all_dof_names),
        "controlled_dofs_moved": controlled_dofs_moved,
        "arm_dofs_commanded": list(CONTROLLED_ARM_JOINT_NAMES),
        "hand_motor_dofs_commanded": list(hand_motor_joint_names),
        "missing_hand_motor_dofs": list(missing_hand_motor_joints),
        "hand_motor_control_status": hand_motor_control_status,
        "urdf_constraint_fidelity": urdf_constraint_fidelity,
        "motion_case_count": len(motion_results),
        "motion_cases": motion_results,
        "screenshot_output_dir": _repo_relative_path(MOTION_SCREENSHOT_OUTPUT_DIR),
        "wrist_attachment_runtime_validation": wrist_attachment_runtime_validation,
        "evidence_summary": evidence_summary,
    }
    if runtime_log_path:
        runtime_validation["isaac_sim_log"] = _repo_relative_path(runtime_log_path)
    if contact_sheet_path and os.path.isfile(contact_sheet_path):
        runtime_validation["contact_sheet"] = _repo_relative_path(contact_sheet_path)

    report["runtime_validation"] = runtime_validation
    os.makedirs(os.path.dirname(report_path), exist_ok=True)
    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, sort_keys=True)
        f.write("\n")
    print(
        f"[setup_rpo_arm_scene] Runtime validation report updated: {report_path}",
        flush=True,
    )
    if runtime_status != "PASS":
        raise RuntimeError(
            "Runtime wrist attachment validation failed: "
            f"{wrist_attachment_runtime_validation}"
        )


def _capture_prim_paths() -> list[str]:
    if custom_visual_prim_path and custom_visual_follow_target_path:
        return [prim_path, custom_visual_prim_path]
    return [custom_visual_prim_path or prim_path]


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

    frame_viewport_prims(viewport, prims=_capture_prim_paths())
    simulation_app.update()

    async def _capture():
        capture = capture_viewport_to_file(viewport, file_path=path)
        await asyncio.wait_for(capture.wait_for_result(), timeout=10.0)

    loop = asyncio.get_event_loop()
    loop.run_until_complete(_capture())


def _capture_replicator_screenshot(path: str) -> None:
    """Render the current scene with Replicator when viewport capture is unavailable."""
    import omni.replicator.core as rep
    from pxr import Gf, Usd, UsdLux

    stage = omni.usd.get_context().get_stage()
    capture_prim_paths = _capture_prim_paths()

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=True,
    )
    bbox_min = None
    bbox_max = None
    for capture_prim_path in capture_prim_paths:
        if capture_prim_path.startswith("/World/"):
            root_path = capture_prim_path
        else:
            root_path = "/" + capture_prim_path.strip("/").split("/", maxsplit=1)[0]
        if not stage.GetPrimAtPath(root_path).IsValid():
            root_path = capture_prim_path
        bbox = bbox_cache.ComputeWorldBound(stage.GetPrimAtPath(root_path)).ComputeAlignedBox()
        prim_min = bbox.GetMin()
        prim_max = bbox.GetMax()
        if bbox_min is None or bbox_max is None:
            bbox_min = Gf.Vec3d(prim_min)
            bbox_max = Gf.Vec3d(prim_max)
        else:
            bbox_min = Gf.Vec3d(
                min(float(bbox_min[0]), float(prim_min[0])),
                min(float(bbox_min[1]), float(prim_min[1])),
                min(float(bbox_min[2]), float(prim_min[2])),
            )
            bbox_max = Gf.Vec3d(
                max(float(bbox_max[0]), float(prim_max[0])),
                max(float(bbox_max[1]), float(prim_max[1])),
                max(float(bbox_max[2]), float(prim_max[2])),
            )
    if bbox_min is None or bbox_max is None:
        raise RuntimeError("No valid prims available for screenshot framing")
    center = (bbox_min + bbox_max) * 0.5
    size = bbox_max - bbox_min
    radius = max(float(size[0]), float(size[1]), float(size[2]), 0.2)
    if (
        custom_visual_prim_path
        and capture_prim_paths == [custom_visual_prim_path]
        and radius < MIN_CUSTOM_VISUAL_CAPTURE_RADIUS
    ):
        radius = DEFAULT_CUSTOM_VISUAL_CAPTURE_RADIUS
        print(
            "[setup_rpo_arm_scene] Using custom visual camera radius clamp for "
            f"{custom_visual_prim_path}: center={tuple(float(v) for v in center)}, radius={radius}",
            flush=True,
        )

    dome_path = "/World/ScreenshotDomeLight"
    if not stage.GetPrimAtPath(dome_path).IsValid():
        UsdLux.DomeLight.Define(stage, dome_path).CreateIntensityAttr(450.0)

    camera_pos = Gf.Vec3d(
        float(center[0]) + radius * 1.6,
        float(center[1]) - radius * 2.1,
        float(center[2]) + radius * 1.25,
    )
    camera_target = Gf.Vec3d(
        float(center[0]),
        float(center[1]),
        float(center[2]) + radius * 0.05,
    )
    focal_length = 35

    output_dir = tempfile.mkdtemp(
        prefix=os.path.basename(path) + ".",
        dir=os.path.dirname(path),
    )
    try:
        with rep.new_layer():
            camera = rep.create.camera(
                position=tuple(float(v) for v in camera_pos),
                look_at=tuple(float(v) for v in camera_target),
                focal_length=focal_length,
            )
            render_product = rep.create.render_product(camera, (1280, 720))
            writer = rep.WriterRegistry.get("BasicWriter")
            writer.initialize(output_dir=output_dir, rgb=True)
            writer.attach([render_product])
            for _ in range(8):
                rep.orchestrator.step(rt_subframes=8)
            writer.detach()

        frames = sorted(glob.glob(os.path.join(output_dir, "rgb*.png")))
        if not frames:
            raise RuntimeError(f"Replicator did not write an RGB frame in {output_dir}")
        shutil.copyfile(frames[-1], path)
    finally:
        shutil.rmtree(output_dir, ignore_errors=True)

    _wait_for_screenshot_file(path)


if art is not None:
    all_positions = _position_list(art.get_joint_positions())
    current_arm_positions = [all_positions[i] for i in controlled_indices]
else:
    all_positions = [0.0] * len(CONTROLLED_ARM_JOINT_NAMES)
    current_arm_positions = list(all_positions)
current_grasp = 0.0
current_positions = (
    list(current_arm_positions)
    if ECHO_FULL_ARM_ONLY
    else [*current_arm_positions, current_grasp]
)
current_velocities = [0.0] * len(PUBLISHED_JOINT_NAMES)
last_grasp_servo_targets = (
    {}
    if ECHO_FULL_ARM_ONLY
    else grasp_scalar_to_servo_targets(
        current_grasp,
        middle_pos_deg=DEFAULT_MIDDLE_POS_DEG,
    )
)

rclpy.init()


class SimBridgeNode(Node):
    def __init__(self):
        super().__init__("isaac_sim_rpo_arm_bridge")
        self._cmd = None
        self._cmd_seq = 0
        self._cmd_lock = threading.Lock()
        self._pub = self.create_publisher(JointState, "/follower/joint_states", 10)
        self._sub = self.create_subscription(
            Float64MultiArray, "/follower/joint_commands", self._cmd_cb, 10
        )

    def _cmd_cb(self, msg: Float64MultiArray):
        with self._cmd_lock:
            self._cmd = list(msg.data)
            self._cmd_seq += 1

    def get_command(self):
        with self._cmd_lock:
            cmd = list(self._cmd) if self._cmd is not None else None
            return self._cmd_seq, cmd

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


def _step_running_physics(step_count: int) -> None:
    if step_count <= 0:
        simulation_app.update()
        return
    for _ in range(step_count):
        try:
            world.step(render=True)
        except Exception as exc:
            print(
                f"[setup_rpo_arm_scene] world.step(render=True) failed; "
                f"using SimulationApp update: {type(exc).__name__}: {exc}",
                flush=True,
            )
            simulation_app.update()


def _ensure_articulation_initialized() -> None:
    global art, all_positions

    if art is None:
        return
    try:
        physics_handle_valid = art.is_physics_handle_valid()
    except AttributeError as exc:
        # Replicator capture can invalidate Isaac's private _physics_view handle;
        # recreate the Articulation before applying the next physical-scene motion case.
        if "_physics_view" not in str(exc):
            print(
                f"[setup_rpo_arm_scene] Articulation handle check raised "
                f"{type(exc).__name__}: {exc}",
                flush=True,
            )
        physics_handle_valid = False
    except Exception as exc:
        print(
            f"[setup_rpo_arm_scene] Articulation handle check failed: "
            f"{type(exc).__name__}: {exc}",
            flush=True,
        )
        physics_handle_valid = False

    if not physics_handle_valid:
        print(
            "[setup_rpo_arm_scene] Reinitializing articulation physics handle before motion.",
            flush=True,
        )
        world.reset()
        art = Articulation(prim_path)
        art.initialize()
        refreshed_positions = _position_list(art.get_joint_positions())
        if len(refreshed_positions) == len(all_positions):
            all_positions = refreshed_positions


def _apply_lerobot_command(command_values: list[float], source_label: str, settle_steps: int = 1) -> list[float]:
    global current_grasp, current_positions, last_grasp_servo_targets

    normalized_command = normalize_action(
        command_values,
        joint_names=PUBLISHED_JOINT_NAMES,
    )
    arm_command = normalized_command[: len(CONTROLLED_ARM_JOINT_NAMES)]
    if ECHO_FULL_ARM_ONLY:
        current_grasp = 0.0
        last_grasp_servo_targets = {}
        hand_motor_command = []
        current_positions = list(arm_command)
    else:
        current_grasp = normalized_command[len(CONTROLLED_ARM_JOINT_NAMES)]
        last_grasp_servo_targets = grasp_scalar_to_servo_targets(
            current_grasp,
            middle_pos_deg=DEFAULT_MIDDLE_POS_DEG,
        )
        hand_motor_command = [
            last_grasp_servo_targets[servo_id]
            for servo_id in range(1, len(AMAZINGHAND_MOTOR_JOINT_NAMES) + 1)
        ]
        current_positions = [*arm_command, current_grasp]

    if art is not None:
        _ensure_articulation_initialized()
        for idx, value in zip(controlled_indices, arm_command, strict=True):
            all_positions[idx] = value
        commanded_hand_motor_values: list[float] = []
        if not ECHO_FULL_ARM_ONLY:
            hand_motor_targets_by_name = dict(
                zip(AMAZINGHAND_MOTOR_JOINT_NAMES, hand_motor_command, strict=True)
            )
            commanded_hand_motor_values = [
                hand_motor_targets_by_name[name] for name in hand_motor_joint_names
            ]
            for idx, value in zip(hand_motor_indices, commanded_hand_motor_values, strict=True):
                all_positions[idx] = value
        art.set_joint_positions(np.array([all_positions], dtype=np.float32))
        target_indices = [*controlled_indices, *hand_motor_indices]
        target_values = [
            *arm_command,
            *commanded_hand_motor_values,
        ]
        if target_indices:
            art.set_joint_position_targets(
                np.array(target_values, dtype=np.float32),
                joint_indices=np.array(target_indices, dtype=np.int32),
            )
        if MOTION_SCREENSHOT_KINEMATIC_CAPTURE:
            print(
                "[setup_rpo_arm_scene] Kinematic capture mode: joint positions "
                "were applied without settling full physics.",
                flush=True,
            )
            _step_running_physics(0)
        else:
            _step_running_physics(settle_steps)
    elif using_simready:
        _write_simready_mapping_evidence(
            asset_path=simready_usd_path,
            prim_path=prim_path,
            binding_status="binding_pending",
            last_command=list(current_positions),
        )
        _step_running_physics(settle_steps)

    _sync_custom_visual_to_follow_link()
    _publish_current_state()
    applied = list(current_positions)
    if ECHO_FULL_ARM_ONLY:
        print(
            f"[setup_rpo_arm_scene] Applied arm-only command ({source_label}): {applied}",
            flush=True,
        )
    else:
        print(
            f"[setup_rpo_arm_scene] Applied LeRobot command ({source_label}): "
            f"{applied}; AmazingHand scalar servo targets(rad)={last_grasp_servo_targets}",
            flush=True,
        )
    return applied


def _run_motion_screenshot_cases() -> list[dict[str, object]]:
    cases = _parse_motion_screenshot_cases()
    if not cases:
        return []
    if art is None:
        raise RuntimeError(
            "Motion screenshot cases require a bound physical articulation; "
            "generate the direct physical URDF or provide PHYSICAL_ROBOT_URDF_PATH."
        )

    os.makedirs(MOTION_SCREENSHOT_OUTPUT_DIR, exist_ok=True)
    print(
        f"[setup_rpo_arm_scene] Running {len(cases)} Motion screenshot case(s) "
        f"while the physics timeline is playing.",
        flush=True,
    )
    if MOTION_SCREENSHOT_KINEMATIC_CAPTURE:
        print(
            "[setup_rpo_arm_scene] Kinematic capture mode enabled for motion screenshots.",
            flush=True,
        )
    results: list[dict[str, object]] = []
    for index, case in enumerate(cases, start=1):
        case_name = case["name"]
        command = case["command"]
        image_path = os.path.join(MOTION_SCREENSHOT_OUTPUT_DIR, f"{index:02d}_{case_name}.png")
        print(
            f"[setup_rpo_arm_scene] Motion screenshot case {index}/{len(cases)}: "
            f"{case_name} -> {command}",
            flush=True,
        )
        applied = _apply_lerobot_command(
            command,
            source_label=f"motion screenshot case {case_name}",
            settle_steps=0 if MOTION_SCREENSHOT_KINEMATIC_CAPTURE else MOTION_SCREENSHOT_SETTLE_STEPS,
        )
        _capture_rgb_screenshot(image_path, applied)
        commanded_hand_motors = {
            name: float(last_grasp_servo_targets[servo_id])
            for servo_id, name in enumerate(AMAZINGHAND_MOTOR_JOINT_NAMES, start=1)
            if name in hand_motor_joint_names
        }
        results.append(
            {
                "index": index,
                "name": case_name,
                "command": command,
                "applied_command": applied,
                "grasp_servo_targets_rad": {
                    str(servo_id): float(target)
                    for servo_id, target in last_grasp_servo_targets.items()
                },
                "commanded_hand_motor_targets_rad": commanded_hand_motors,
                "screenshot": _repo_relative_path(image_path),
                "size_bytes": os.path.getsize(image_path),
            }
        )
    return results


import omni.timeline  # noqa: E402

timeline = omni.timeline.get_timeline_interface()
timeline.play()

print("[setup_rpo_arm_scene] Simulation running. Ctrl+C to stop.", flush=True)
screenshot_taken = False
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
    motion_screenshot_cases_ran = _run_motion_screenshot_cases()
    if motion_screenshot_cases_ran:
        _write_runtime_validation_report(motion_screenshot_cases_ran)
except Exception as exc:
    print(
        f"[setup_rpo_arm_scene] ERROR: Motion screenshot cases failed: "
        f"{type(exc).__name__}: {exc}",
        flush=True,
    )
    timeline.stop()
    rclpy.shutdown()
    bridge_spin_thread.join(timeout=2.0)
    bridge.destroy_node()
    simulation_app.close()
    sys.exit(1)

if motion_screenshot_cases_ran and EXIT_AFTER_MOTION_SCREENSHOTS:
    print(
        "[setup_rpo_arm_scene] Exiting after motion screenshot cases as requested.",
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
        # Headed Sunshine sessions need continuous Kit updates to keep the X11
        # window responsive; headless runs can keep the conservative one-update
        # path unless CONTINUOUS_APP_UPDATE is explicitly enabled.
        if CONTINUOUS_APP_UPDATE or not app_updated:
            simulation_app.update()
            app_updated = True

        cmd_seq, cmd = bridge.get_command()
        should_exit_after_screenshot = False
        if (
            cmd is not None
            and cmd_seq != last_processed_command_seq
            and len(cmd) >= len(CONTROLLED_ARM_JOINT_NAMES)
        ):
            last_processed_command_seq = cmd_seq
            command_values = [float(v) for v in cmd]
            last_applied_command = _apply_lerobot_command(
                command_values,
                source_label=f"ROS command #{cmd_seq}",
                settle_steps=1,
            )

            if SCREENSHOT_AFTER_COMMAND and not screenshot_taken:
                print("[setup_rpo_arm_scene] Screenshot trigger accepted.", flush=True)
                _capture_rgb_screenshot(SCREENSHOT_PATH, last_applied_command)
                screenshot_taken = True
                if EXIT_AFTER_SCREENSHOT:
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
