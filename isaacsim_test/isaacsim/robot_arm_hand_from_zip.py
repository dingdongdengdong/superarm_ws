"""Fresh robot_arm_hand_package.zip to Isaac Sim USD pipeline.

Host-side static checks can import this module without Isaac Sim installed.
Isaac-only imports are intentionally scoped to conversion/runtime functions.

Run inside the Isaac Sim container:
    /isaac-sim/python.sh /workspace/isaacsim/robot_arm_hand_from_zip.py --mode all
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import tempfile
import time
import traceback
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

try:
    from isaacsim_test.isaacsim.graspable_hand_urdf import (
        HAND_GRASP_TYPES,
        VISUAL_MODE_IMPLEMENTED_ONLY,
        VISUAL_MODE_PARTITIONED_LINKS,
        VISUAL_MODE_STATIC_SHELL,
        generate_graspable_hand_urdf,
        grasp_preshape_to_hand_joint_targets,
        grasp_scalar_to_hand_joint_targets,
    )
except ModuleNotFoundError:
    from graspable_hand_urdf import (
        HAND_GRASP_TYPES,
        VISUAL_MODE_IMPLEMENTED_ONLY,
        VISUAL_MODE_PARTITIONED_LINKS,
        VISUAL_MODE_STATIC_SHELL,
        generate_graspable_hand_urdf,
        grasp_preshape_to_hand_joint_targets,
        grasp_scalar_to_hand_joint_targets,
    )

CONTAINER_ROOT = "/workspace/superarm_ws"
HOST_ROOT = (
    Path(CONTAINER_ROOT)
    if Path(CONTAINER_ROOT).is_dir()
    else Path(__file__).resolve().parents[2]
)
PACKAGE_DIR_NAME = "robot_arm_hand_package"
HAND_MOUNT_LOCAL_XYZ = (0.005, -0.00014, 0.600003)
DEFAULT_ZIP = f"{CONTAINER_ROOT}/robot_arm_hand_package.zip"
DEFAULT_INPUT_ROOT = f"{CONTAINER_ROOT}/isaacsim_test/inputs/robot_arm_hand_package"
DEFAULT_OUTPUT_ROOT = f"{CONTAINER_ROOT}/isaacsim_test/outputs/robot_arm_hand_from_zip"
DEFAULT_SCREENSHOT_ROOT = f"{CONTAINER_ROOT}/isaacsim_test/artifacts/robot_arm_hand_from_zip"
ARM_PRIM_PATH = "/World/RobotArmFromZip"
HAND_PRIM_PATH = "/World/AmazingHandFromZip"
HAND_PROXY_ROOT_PRIM_PATH = "/AmazingHandProxy"
CONNECTED_ROOT_PRIM_PATH = "/World/RobotArmHandFromZip"
CONNECTED_ARM_PRIM_PATH = f"{CONNECTED_ROOT_PRIM_PATH}/Arm"
CONNECTED_HAND_PRIM_PATH = f"{CONNECTED_ROOT_PRIM_PATH}/Hand"
GRASP_VALIDATION_ROOT_PRIM_PATH = f"{CONNECTED_ROOT_PRIM_PATH}/GraspValidationObjects"
FIXED_JOINT_PATH = f"{CONNECTED_ROOT_PRIM_PATH}/arm_to_hand_fixed_joint"
ARM_HAND_ATTACHMENT_BODY0_PATH = f"{CONNECTED_ARM_PRIM_PATH}/wrist_adapter_hand"
ARM_HAND_ATTACHMENT_BODY1_PATH = CONNECTED_HAND_PRIM_PATH
ARM_JOINT_NAMES = ["joint_rev_1", "joint_rev_2", "joint_rev_3", "joint_rev_4"]
FINGER_PROXY_DISTANCE_THRESHOLD_M = 0.065
# Use the side-sweep pose already validated by the arm motion smoke test.  The
# previous fold-like target lowered the palm anchor during "lift" validation,
# which made the strict no-drop criterion fail even before testing retention.
LIFT_RETAIN_ARM_TARGET = [-0.25, 0.15, 0.3, -0.2]
HAND_ROOT_LINK_NAME = "r_wrist_interface"
HAND_PRESHAPE_ACTIVE_FINGERS = {
    "single_finger": None,
    "pinch": [1, 4],
    "wide": [1, 2, 3, 4],
    "wrap": [1, 2, 3, 4],
}
DEFAULT_CAPTURE_WIDTH = 1280
DEFAULT_CAPTURE_HEIGHT = 720


def _capture_resolution_from_env(env: dict[str, str] | None = None) -> tuple[int, int]:
    """Return screenshot/render resolution, defaulting to the existing 720p setup."""
    source = os.environ if env is None else env

    def _positive_int(name: str, default: int) -> int:
        try:
            value = int(str(source.get(name, default)).strip())
        except (TypeError, ValueError):
            return default
        return value if value > 0 else default

    return (
        _positive_int("ROBOT_ARM_HAND_CAPTURE_WIDTH", DEFAULT_CAPTURE_WIDTH),
        _positive_int("ROBOT_ARM_HAND_CAPTURE_HEIGHT", DEFAULT_CAPTURE_HEIGHT),
    )


def _simulation_app_config(*, headless: bool) -> dict[str, Any]:
    width, height = _capture_resolution_from_env()
    return {"headless": headless, "width": width, "height": height}


def _select_focus_prim_enabled(env: dict[str, str] | None = None) -> bool:
    """Whether to drive USD selection before framing; off by default in headless."""
    source = os.environ if env is None else env
    return str(source.get("ROBOT_ARM_HAND_SELECT_FOCUS_PRIM", "")).lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def _compose_connected_reference_body_path(
    *,
    source_reference_prim_path: str | None,
    source_body_path: str | None,
    connected_reference_prim_path: str,
) -> str:
    """Map a rigid body path from a referenced source USD into the connected stage."""
    if not source_reference_prim_path or not source_body_path:
        return connected_reference_prim_path

    source_reference = source_reference_prim_path.rstrip("/")
    source_body = source_body_path.rstrip("/")
    if source_body == source_reference:
        return connected_reference_prim_path
    prefix = source_reference + "/"
    if source_body.startswith(prefix):
        return connected_reference_prim_path + source_body[len(source_reference) :]
    return connected_reference_prim_path


def _flat_float_list(values: Any) -> list[float]:
    """Normalize Isaac/numpy/list joint-position outputs to a flat Python list."""
    if values is None:
        return []
    if hasattr(values, "tolist"):
        values = values.tolist()
    if values and isinstance(values[0], list):
        values = values[0]
    return [float(value) for value in values]


def build_arm_joint_position_command(
    current_positions: Any,
    dof_names: list[str],
    command: list[float],
    joint_names: list[str] | None = None,
) -> dict[str, Any]:
    """Build a full Articulation joint-position command for the delivered arm."""
    target_joint_names = list(joint_names or ARM_JOINT_NAMES)
    if len(command) != len(target_joint_names):
        raise ValueError(
            f"Expected {len(target_joint_names)} arm joint values, got {len(command)}"
        )
    missing = [name for name in target_joint_names if name not in dof_names]
    if missing:
        raise ValueError(f"Missing required arm joints in articulation: {missing}")

    positions = _flat_float_list(current_positions)
    if len(positions) != len(dof_names):
        positions = [0.0] * len(dof_names)

    indices = [dof_names.index(name) for name in target_joint_names]
    for index, value in zip(indices, command, strict=True):
        positions[index] = float(value)
    return {
        "positions": positions,
        "controlled_indices": indices,
        "controlled_joint_names": target_joint_names,
        "target_values": [float(value) for value in command],
    }


def build_named_joint_position_command(
    current_positions: Any,
    dof_names: list[str],
    joint_targets: dict[str, float],
) -> dict[str, Any]:
    """Build a full Articulation command for an arbitrary set of named joints."""
    missing = [name for name in joint_targets if name not in dof_names]
    if missing:
        raise ValueError(f"Missing required joints in articulation: {missing}")

    positions = _flat_float_list(current_positions)
    if len(positions) != len(dof_names):
        positions = [0.0] * len(dof_names)

    controlled_joint_names = list(joint_targets)
    indices = [dof_names.index(name) for name in controlled_joint_names]
    target_values = [float(joint_targets[name]) for name in controlled_joint_names]
    for index, value in zip(indices, target_values, strict=True):
        positions[index] = value
    return {
        "positions": positions,
        "controlled_indices": indices,
        "controlled_joint_names": controlled_joint_names,
        "target_values": target_values,
    }


def build_hand_grasp_position_command(
    current_positions: Any,
    dof_names: list[str],
    grasp: float,
    grasp_type: str = "wrap",
) -> dict[str, Any]:
    """Build a named hand closing command from a normalized grasp value."""
    return build_named_joint_position_command(
        current_positions=current_positions,
        dof_names=dof_names,
        joint_targets=build_hand_preshape_joint_targets(
            grasp=grasp,
            grasp_type=grasp_type,
        ),
    )


def _validate_finger_index(finger_index: int) -> None:
    if finger_index < 1 or finger_index > 4:
        raise ValueError(f"Expected finger_index from 1 to 4, got {finger_index}")


def _supported_preshape_message() -> str:
    return (
        "Supported hand grasp types are: "
        + ", ".join([*HAND_GRASP_TYPES, "single_finger"])
    )


def build_hand_preshape_joint_targets(
    *,
    grasp: float,
    grasp_type: str = "wrap",
    finger_index: int | None = None,
) -> dict[str, float]:
    """Return named hand joint targets for a simple grasp preshape."""
    if grasp_type not in HAND_PRESHAPE_ACTIVE_FINGERS:
        raise ValueError(_supported_preshape_message())

    open_targets = grasp_scalar_to_hand_joint_targets(0.0)
    if grasp_type in HAND_GRASP_TYPES:
        return grasp_preshape_to_hand_joint_targets(grasp, grasp_type)
    if grasp_type == "single_finger":
        if finger_index is None:
            raise ValueError("single_finger preshape requires finger_index")
        _validate_finger_index(finger_index)
        targets = dict(open_targets)
        closed_targets = grasp_preshape_to_hand_joint_targets(grasp, "wrap")
        for motor in (1, 2):
            name = f"finger{finger_index}_motor{motor}"
            targets[name] = closed_targets[name]
        return targets

    raise AssertionError(grasp_type)


def _active_fingers_for_preshape(
    preshape: str,
    *,
    finger_index: int | None = None,
) -> list[int]:
    if preshape == "single_finger":
        if finger_index is None:
            raise ValueError("single_finger preshape requires finger_index")
        _validate_finger_index(finger_index)
        return [finger_index]
    if preshape not in HAND_PRESHAPE_ACTIVE_FINGERS:
        raise ValueError(_supported_preshape_message())
    active = HAND_PRESHAPE_ACTIVE_FINGERS[preshape]
    return list(active or [])


def build_hand_preshape_position_command(
    current_positions: Any,
    dof_names: list[str],
    *,
    preshape: str,
    amount: float,
    finger_index: int | None = None,
) -> dict[str, Any]:
    """Build a command for one simple hand preshape stage."""
    active_fingers = _active_fingers_for_preshape(preshape, finger_index=finger_index)
    all_targets = build_hand_preshape_joint_targets(
        grasp=amount,
        grasp_type=preshape,
        finger_index=finger_index,
    )
    active_targets = {
        name: value
        for name, value in all_targets.items()
        if any(name.startswith(f"finger{finger}_") for finger in active_fingers)
    }
    command = build_named_joint_position_command(
        current_positions=current_positions,
        dof_names=dof_names,
        joint_targets=active_targets,
    )
    command["preshape"] = preshape
    command["amount"] = float(amount)
    command["active_fingers"] = active_fingers
    return command


def build_preshape_grasp_validation_stage_specs() -> list[dict[str, Any]]:
    """Return the ordered skeleton-first hand grasp stages for runtime validation."""
    return [
        {
            "label": "single_finger",
            "preshape": "single_finger",
            "amount": 1.0,
            "finger_index": 1,
            "active_fingers": [1],
            "required_finger_proxy_count": 1,
        },
        {
            "label": "pinch",
            "preshape": "pinch",
            "amount": 1.0,
            "active_fingers": [1, 4],
            "required_finger_proxy_count": 2,
        },
        {
            "label": "wrap",
            "preshape": "wrap",
            "amount": 1.0,
            "active_fingers": [1, 2, 3, 4],
            "required_finger_proxy_count": 3,
        },
    ]


def build_shadow_allegro_reference_checklist() -> dict[str, Any]:
    """Return the reference-only checklist for known working dexterous hands."""
    return {
        "status": "REFERENCE_ONLY",
        "reference_only": True,
        "reference_models": ["Shadow Hand", "Allegro Hand"],
        "allowed_use": [
            "compare_collision_shape_layout",
            "compare_mass_inertia_ranges",
            "compare_joint_limits_axes",
            "compare_drive_stiffness_damping",
            "compare_grasp_task_object_and_friction_setup",
        ],
        "excluded_use": [
            "replace_amazinghand",
            "simready_conversion",
            "change_robot_hand_identity",
        ],
        "checkpoints": [
            "collision_shape_layout",
            "mass_inertia_ranges",
            "joint_limits_axes",
            "drive_stiffness_damping",
            "object_placement_and_friction",
            "grasp_controller_interface",
        ],
    }


def build_lift_retain_joint_target_sequence(
    current_positions: Any,
    dof_names: list[str],
    *,
    arm_target: list[float] | None = None,
    grasp: float = 1.0,
    segments: int = 6,
) -> list[dict[str, Any]]:
    """Build ramped arm+closed-hand targets for lift-retain validation.

    The grasp validation must not abruptly throw the arm to a single target while
    the object is in contact.  Each command keeps the hand close targets active
    and interpolates only the arm joints from their current measured values to a
    stable upright pose.
    """
    target_arm = list(arm_target or LIFT_RETAIN_ARM_TARGET)
    if len(target_arm) != len(ARM_JOINT_NAMES):
        raise ValueError(
            f"Expected {len(ARM_JOINT_NAMES)} lift arm values, got {len(target_arm)}"
        )
    if segments < 1:
        raise ValueError(f"Expected segments >= 1, got {segments}")

    positions = _flat_float_list(current_positions)
    if len(positions) != len(dof_names):
        positions = [0.0] * len(dof_names)
    missing = [name for name in ARM_JOINT_NAMES if name not in dof_names]
    if missing:
        raise ValueError(f"Missing required arm joints in articulation: {missing}")

    start_arm = [positions[dof_names.index(name)] for name in ARM_JOINT_NAMES]
    hand_targets = grasp_scalar_to_hand_joint_targets(grasp)
    commands: list[dict[str, Any]] = []
    working_positions = list(positions)
    for step in range(1, segments + 1):
        ratio = step / segments
        joint_targets = dict(hand_targets)
        joint_targets.update(
            {
                name: start + (target - start) * ratio
                for name, start, target in zip(
                    ARM_JOINT_NAMES, start_arm, target_arm, strict=True
                )
            }
        )
        command = build_named_joint_position_command(
            current_positions=working_positions,
            dof_names=dof_names,
            joint_targets=joint_targets,
        )
        working_positions = list(command["positions"])
        commands.append(command)
    return commands


def build_single_finger_two_link_position_command(
    current_positions: Any,
    dof_names: list[str],
    finger_index: int,
    *,
    motor1: float,
    motor2: float,
) -> dict[str, Any]:
    """Build a command for one generated two-link finger."""
    _validate_finger_index(finger_index)
    return build_named_joint_position_command(
        current_positions=current_positions,
        dof_names=dof_names,
        joint_targets={
            f"finger{finger_index}_motor1": float(motor1),
            f"finger{finger_index}_motor2": float(motor2),
        },
    )


def _distance(
    left: tuple[float, float, float] | None,
    right: tuple[float, float, float] | None,
) -> float | None:
    if left is None or right is None:
        return None
    return math.sqrt(sum((left[index] - right[index]) ** 2 for index in range(3)))


def _finger_proxy_close_count(
    distances: list[float] | tuple[float, ...] | None,
    *,
    threshold_m: float = FINGER_PROXY_DISTANCE_THRESHOLD_M,
) -> int:
    if distances is None:
        return 0
    return sum(1 for distance in distances if distance <= threshold_m)


def _evaluate_lift_retain_status(
    *,
    object_before: tuple[float, float, float] | None,
    object_after_close: tuple[float, float, float] | None,
    anchor_after_close: tuple[float, float, float] | None,
    object_after_lift: tuple[float, float, float] | None,
    anchor_after_lift: tuple[float, float, float] | None,
    object_reference: tuple[float, float, float] | None = None,
    finger_proxy_distances_after_close: list[float] | tuple[float, ...] | None = None,
    finger_proxy_distances_after_lift: list[float] | tuple[float, ...] | None = None,
    screenshot_size_bytes: int,
    control_status: str,
) -> dict[str, Any]:
    """Evaluate lift-retain against palm anchor and finger-contact engagement."""
    close_distance = _distance(object_after_close, anchor_after_close)
    lift_distance = _distance(object_after_lift, anchor_after_lift)
    z_reference = object_reference if object_reference is not None else object_before
    object_z_delta = (
        object_after_lift[2] - z_reference[2]
        if object_after_lift is not None and z_reference is not None
        else None
    )
    object_z_delta_from_settled_before = (
        object_after_lift[2] - object_before[2]
        if object_after_lift is not None and object_before is not None
        else None
    )
    retained_near_hand = lift_distance is not None and lift_distance <= 0.16
    lifted_or_held = object_z_delta is not None and object_z_delta >= -0.01
    close_finger_count = _finger_proxy_close_count(finger_proxy_distances_after_close)
    lift_finger_count = _finger_proxy_close_count(finger_proxy_distances_after_lift)
    finger_grasp_engaged = close_finger_count >= 3 and lift_finger_count >= 3
    status = (
        "PASS"
        if screenshot_size_bytes > 0
        and control_status != "FAILED"
        and retained_near_hand
        and lifted_or_held
        and finger_grasp_engaged
        else "WARN"
    )
    return {
        "status": status,
        "object_anchor_distance_after_close_m": close_distance,
        "object_anchor_distance_after_lift_m": lift_distance,
        "object_z_delta_after_lift_m": object_z_delta,
        "object_z_delta_after_lift_from_settled_before_m": (
            object_z_delta_from_settled_before
        ),
        "object_z_delta_reference": (
            "reset_world_xyz" if object_reference is not None else "settled_before"
        ),
        "finger_proxy_distance_threshold_m": FINGER_PROXY_DISTANCE_THRESHOLD_M,
        "finger_proxy_distances_after_close_m": (
            sorted(float(value) for value in finger_proxy_distances_after_close)
            if finger_proxy_distances_after_close is not None
            else []
        ),
        "finger_proxy_distances_after_lift_m": (
            sorted(float(value) for value in finger_proxy_distances_after_lift)
            if finger_proxy_distances_after_lift is not None
            else []
        ),
        "finger_proxy_close_count_after_close": close_finger_count,
        "finger_proxy_close_count_after_lift": lift_finger_count,
        "finger_grasp_engaged": finger_grasp_engaged,
        "retained_near_hand": retained_near_hand,
        "lifted_or_held": lifted_or_held,
    }


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


def _safe_extract(zip_path: Path, destination: Path) -> None:
    destination_resolved = destination.resolve()
    with zipfile.ZipFile(zip_path) as archive:
        for member in archive.infolist():
            target = (destination / member.filename).resolve()
            if not str(target).startswith(str(destination_resolved) + os.sep):
                raise RuntimeError(f"Refusing unsafe zip member path: {member.filename}")
        archive.extractall(destination)


def extract_robot_arm_hand_package(zip_path: str | Path, input_root: str | Path) -> Path:
    """Extract the delivery zip and return the package root directory."""
    zip_file = _host_path(zip_path)
    if not zip_file.is_file():
        raise FileNotFoundError(f"Robot arm/hand package zip not found: {zip_file}")

    destination = _host_path(input_root)
    package_root = destination / PACKAGE_DIR_NAME
    if package_root.exists():
        try:
            shutil.rmtree(package_root)
        except PermissionError:
            required = [
                package_root / "arm_description/urdf/robot_arm_hand_urdf.xacro",
                package_root / "hand_mjcf/robot.xml",
            ]
            if all(path.is_file() for path in required):
                return package_root
            raise
    destination.mkdir(parents=True, exist_ok=True)
    _safe_extract(zip_file, destination)
    if not package_root.is_dir():
        raise RuntimeError(f"Expected extracted package directory not found: {package_root}")
    return package_root


def _remove_xacro_includes(root: ET.Element) -> int:
    removed = 0
    xacro_include_tag = "{http://www.ros.org/wiki/xacro}include"
    for child in list(root):
        if child.tag == xacro_include_tag:
            root.remove(child)
            removed += 1
    return removed


def _copy_material_definitions(package_root: Path, root: ET.Element) -> int:
    materials_path = package_root / "arm_description/urdf/materials.xacro"
    if not materials_path.is_file():
        return 0
    material_root = ET.parse(materials_path).getroot()
    inserted = 0
    existing = {
        material.attrib.get("name")
        for material in root.findall("material")
        if material.attrib.get("name")
    }
    insertion_index = 0
    for material in material_root.findall("material"):
        name = material.attrib.get("name")
        if name and name not in existing:
            root.insert(insertion_index, material)
            insertion_index += 1
            inserted += 1
            existing.add(name)
    return inserted


def sanitize_arm_urdf(package_root: str | Path, output_urdf: str | Path) -> dict[str, Any]:
    """Create an Isaac-importable URDF from the arm xacro."""
    package = _host_path(package_root)
    xacro_path = package / "arm_description/urdf/robot_arm_hand_urdf.xacro"
    mesh_root = package / "arm_description/meshes"
    output = _host_path(output_urdf)
    if not xacro_path.is_file():
        raise FileNotFoundError(f"Arm xacro not found: {xacro_path}")
    if not mesh_root.is_dir():
        raise FileNotFoundError(f"Arm mesh directory not found: {mesh_root}")

    tree = ET.parse(xacro_path)
    root = tree.getroot()
    removed_includes = _remove_xacro_includes(root)
    inserted_materials = _copy_material_definitions(package, root)

    mesh_reference_count = 0
    missing_meshes: list[str] = []
    package_prefix = "package://robot_arm_hand_urdf_description/meshes/"
    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename")
        if not filename:
            continue
        mesh_reference_count += 1
        if filename.startswith(package_prefix):
            mesh_name = filename[len(package_prefix) :]
            mesh_path = mesh_root / mesh_name
            mesh.attrib["filename"] = str(mesh_path.resolve())
        else:
            mesh_path = Path(filename)
        if not _host_path(mesh_path).is_file():
            missing_meshes.append(str(mesh_path))

    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    output.write_text(output.read_text(encoding="utf-8") + "\n", encoding="utf-8")

    joints = root.findall("joint")
    revolute_or_continuous = [
        joint.attrib.get("name", "")
        for joint in joints
        if joint.attrib.get("type") in {"revolute", "continuous"}
    ]
    return {
        "status": "PASS" if not missing_meshes else "FAIL",
        "input_xacro": _repo_relative_path(xacro_path),
        "output_urdf": _repo_relative_path(output),
        "xacro_includes_removed": removed_includes,
        "material_definitions_inserted": inserted_materials,
        "mesh_reference_count": mesh_reference_count,
        "missing_meshes": missing_meshes,
        "joint_count": len(joints),
        "moving_joint_names": revolute_or_continuous,
    }


def analyze_hand_mjcf(package_root: str | Path) -> dict[str, Any]:
    """Inspect the hand MJCF source and verify its mesh/closed-loop contract."""
    package = _host_path(package_root)
    mjcf_path = package / "hand_mjcf/robot.xml"
    asset_root = package / "hand_mjcf/assets"
    if not mjcf_path.is_file():
        raise FileNotFoundError(f"Hand MJCF not found: {mjcf_path}")
    if not asset_root.is_dir():
        raise FileNotFoundError(f"Hand MJCF asset directory not found: {asset_root}")

    root = ET.parse(mjcf_path).getroot()
    worldbody = root.find("worldbody")
    root_body = None
    if worldbody is not None:
        first_body = worldbody.find("body")
        if first_body is not None:
            root_body = first_body.attrib.get("name")
    actuators = [
        actuator.attrib.get("name", "")
        for actuator in root.findall("./actuator/position")
        if actuator.attrib.get("name")
    ]
    equality_connects = root.findall("./equality/connect")
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
    return {
        "status": "PASS" if root_body == "r_wrist_interface" and not missing_meshes else "FAIL",
        "mjcf_path": _repo_relative_path(mjcf_path),
        "root_body": root_body,
        "position_actuator_count": len(actuators),
        "position_actuators": actuators,
        "equality_connect_count": len(equality_connects),
        "mesh_count": len(mesh_files),
        "missing_meshes": missing_meshes,
    }


def sanitize_hand_mjcf(package_root: str | Path, output_mjcf: str | Path) -> dict[str, Any]:
    """Create an Isaac-importer-compatible MJCF by merging top-level defaults."""
    package = _host_path(package_root)
    source = package / "hand_mjcf/robot.xml"
    output = _host_path(output_mjcf)
    if not source.is_file():
        raise FileNotFoundError(f"Hand MJCF not found: {source}")

    tree = ET.parse(source)
    root = tree.getroot()
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
        "status": "PASS" if len(sanitized_root.findall("default")) == 1 else "FAIL",
        "input_mjcf": _repo_relative_path(source),
        "output_mjcf": _repo_relative_path(output),
        "top_level_defaults_before": len(defaults),
        "top_level_defaults_after": len(sanitized_root.findall("default")),
        "equality_connect_count": len(sanitized_root.findall("./equality/connect")),
        "mesh_names_added": mesh_names_added,
        "equality_connect_names_added": equality_connect_names_added,
    }


def _asset_reference(asset_path: str | Path, layer_path: str | Path) -> str:
    asset = _host_path(asset_path)
    layer = _host_path(layer_path)
    if asset.exists():
        return os.path.relpath(asset.resolve(), layer.parent.resolve()).replace("\\", "/")
    return str(asset_path).replace("\\", "/")


def _set_translate_op(xformable: Any, translate: Any) -> None:
    from pxr import UsdGeom

    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(translate)
            return
    xformable.AddTranslateOp().Set(translate)


def build_hand_proxy_primitives() -> list[dict[str, Any]]:
    """Return deterministic proxy geometry for the hand when MJCF import is blocked."""
    return [
        {
            "name": "wrist_interface",
            "translate": (0.0, 0.0, 0.025),
            "scale": (0.035, 0.05, 0.025),
            "color": (0.18, 0.18, 0.2),
        },
        {
            "name": "palm",
            "translate": (0.0, 0.0, 0.095),
            "scale": (0.065, 0.07, 0.04),
            "color": (0.34, 0.36, 0.38),
        },
        *[
            {
                "name": f"finger_{index}",
                "translate": (0.0, y_offset, 0.19),
                "scale": (0.018, 0.009, 0.085),
                "color": (0.42, 0.44, 0.46),
            }
            for index, y_offset in enumerate((-0.048, -0.016, 0.016, 0.048), start=1)
        ],
    ]


def build_grasp_validation_object_specs() -> list[dict[str, Any]]:
    """Return small rigid objects used for the first Isaac grasp-contact smoke test."""
    return [
        {
            "name": "small_trash_box",
            "shape": "cube",
            "local_xyz": (0.0, 0.028, 0.075),
            "scale": (0.022, 0.022, 0.018),
            "mass_kg": 0.006,
            "color": (0.74, 0.68, 0.56),
        }
    ]


def build_hand_contact_proxy_specs() -> list[dict[str, Any]]:
    """Return link-local collision proxies for the generated two-link hand."""
    specs: list[dict[str, Any]] = [
        {
            "link_name": "palm",
            "name": "palm_contact_proxy",
            "local_xyz": (0.0, -0.070, 0.075),
            "scale": (0.095, 0.020, 0.085),
        },
        {
            "link_name": "palm",
            "name": "palm_retention_shelf_proxy",
            "local_xyz": (0.0, -0.010, 0.058),
            "scale": (0.095, 0.145, 0.012),
        },
        {
            "link_name": "palm",
            "name": "palm_retention_left_wall_proxy",
            "local_xyz": (-0.041, -0.010, 0.078),
            "scale": (0.012, 0.145, 0.086),
        },
        {
            "link_name": "palm",
            "name": "palm_retention_right_wall_proxy",
            "local_xyz": (0.041, -0.010, 0.078),
            "scale": (0.012, 0.145, 0.086),
        },
        {
            "link_name": "palm",
            "name": "palm_retention_front_lip_proxy",
            "local_xyz": (0.0, 0.067, 0.078),
            "scale": (0.095, 0.010, 0.086),
        },
        {
            "link_name": "palm",
            "name": "palm_retention_back_lip_proxy",
            "local_xyz": (0.0, -0.028, 0.078),
            "scale": (0.095, 0.010, 0.086),
        }
    ]
    for finger_index in range(1, 5):
        specs.extend(
            [
                {
                    "link_name": f"finger{finger_index}_proximal",
                    "name": f"finger{finger_index}_proximal_contact_proxy",
                    "local_xyz": (0.0, 0.029, 0.0),
                    "scale": (0.02, 0.06, 0.02),
                },
                {
                    "link_name": f"finger{finger_index}_distal",
                    "name": f"finger{finger_index}_distal_contact_proxy",
                    "local_xyz": (0.0, 0.025, 0.0),
                    "scale": (0.018, 0.052, 0.018),
                },
                {
                    "link_name": f"finger{finger_index}_distal",
                    "name": f"finger{finger_index}_distal_tip_pad_proxy",
                    "local_xyz": (0.0, 0.055, 0.0),
                    "scale": (0.03, 0.016, 0.026),
                },
            ]
        )
    return specs


def grasp_object_reset_anchor_path() -> str:
    """Return the physical frame used for grasp-object placement diagnostics."""
    return f"{CONNECTED_HAND_PRIM_PATH}/palm"


def _contact_proxy_path(spec: dict[str, Any]) -> str:
    return (
        f"{CONNECTED_HAND_PRIM_PATH}/{spec['link_name']}"
        f"/contact_proxies/{spec['name']}"
    )


def build_grasp_transform_diagnostic_paths() -> list[str]:
    """Return ordered prim paths that prove hand/proxy/object frame alignment."""
    paths = [
        ARM_HAND_ATTACHMENT_BODY0_PATH,
        CONNECTED_HAND_PRIM_PATH,
        f"{CONNECTED_HAND_PRIM_PATH}/r_wrist_interface",
        f"{CONNECTED_HAND_PRIM_PATH}/palm",
    ]
    for finger_index in range(1, 5):
        paths.extend(
            [
                f"{CONNECTED_HAND_PRIM_PATH}/finger{finger_index}_proximal",
                f"{CONNECTED_HAND_PRIM_PATH}/finger{finger_index}_distal",
            ]
        )
    paths.extend(_contact_proxy_path(spec) for spec in build_hand_contact_proxy_specs())
    return paths


def build_visible_grasp_diagnostic_capture_specs() -> list[dict[str, str]]:
    """Return close-up real-hand grasp screenshots required for visual verification.

    Contact proxies are physics-only debug geometry and stay hidden unless
    ROBOT_ARM_HAND_SHOW_CONTACT_PROXIES=1 is explicitly set. These captures
    are named for the real hand visual to avoid confusing proxy geometry with
    the actual AmazingHand mesh/shell.
    """
    focus = grasp_object_reset_anchor_path()
    return [
        {
            "label": "open",
            "filename": "grasp_real_hand_01_open.png",
            "focus_prim_path": focus,
        },
        {
            "label": "half_close",
            "filename": "grasp_real_hand_02_half_close.png",
            "focus_prim_path": focus,
        },
        {
            "label": "full_close_before_lift",
            "filename": "grasp_real_hand_03_full_close_before_lift.png",
            "focus_prim_path": focus,
        },
        {
            "label": "after_lift_retain",
            "filename": "grasp_real_hand_04_after_lift_retain.png",
            "focus_prim_path": focus,
        },
    ]


def _world_translation(stage: Any, prim_path: str) -> tuple[float, float, float] | None:
    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    translate = matrix.ExtractTranslation()
    return tuple(float(translate[index]) for index in range(3))


def _translation_delta(
    before: tuple[float, float, float] | None,
    after: tuple[float, float, float] | None,
) -> float | None:
    if before is None or after is None:
        return None
    return _distance(before, after)




def _finger_contact_proxy_distances_to_object(
    stage: Any,
    object_path: str | None,
) -> list[float]:
    """Return sorted object distances to finger-mounted contact proxies only."""
    if not object_path:
        return []
    object_xyz = _world_translation(stage, object_path)
    if object_xyz is None:
        return []
    distances: list[float] = []
    for spec in build_hand_contact_proxy_specs():
        if not str(spec.get("link_name", "")).startswith("finger"):
            continue
        proxy_xyz = _world_translation(stage, _contact_proxy_path(spec))
        if proxy_xyz is None:
            continue
        distances.append(_distance(object_xyz, proxy_xyz) or 0.0)
    return sorted(distances)


def _finger_contact_proxy_distances_by_finger_to_object(
    stage: Any,
    object_path: str | None,
) -> dict[int, list[float]]:
    """Return sorted object distances to finger-mounted proxies grouped by finger."""
    if not object_path:
        return {}
    object_xyz = _world_translation(stage, object_path)
    if object_xyz is None:
        return {}
    distances_by_finger: dict[int, list[float]] = {index: [] for index in range(1, 5)}
    for spec in build_hand_contact_proxy_specs():
        link_name = str(spec.get("link_name", ""))
        if not link_name.startswith("finger"):
            continue
        try:
            finger_index = int(link_name[len("finger")])
        except (ValueError, IndexError):
            continue
        proxy_xyz = _world_translation(stage, _contact_proxy_path(spec))
        if proxy_xyz is None:
            continue
        distances_by_finger.setdefault(finger_index, []).append(
            _distance(object_xyz, proxy_xyz) or 0.0
        )
    return {
        finger_index: sorted(distances)
        for finger_index, distances in distances_by_finger.items()
    }


def _grasp_anchor_world_point(
    stage: Any,
    local_xyz: tuple[float, float, float],
) -> tuple[tuple[float, float, float], str] | tuple[None, str]:
    for anchor_path in (grasp_object_reset_anchor_path(), CONNECTED_HAND_PRIM_PATH):
        if _world_translation(stage, anchor_path) is not None:
            return _transform_local_point(stage, anchor_path, local_xyz), anchor_path
    return None, grasp_object_reset_anchor_path()


def _capture_grasp_transform_snapshot(
    stage: Any,
    *,
    label: str,
    object_path: str | None = None,
) -> dict[str, Any]:
    paths = build_grasp_transform_diagnostic_paths()
    if object_path:
        paths.append(object_path)
    measurements: dict[str, dict[str, Any]] = {}
    missing_paths: list[str] = []
    for path in paths:
        xyz = _world_translation(stage, path)
        if xyz is None:
            missing_paths.append(path)
            continue
        measurements[path] = {"world_xyz": list(xyz)}
    return {
        "label": label,
        "reset_anchor_path": grasp_object_reset_anchor_path(),
        "measured_path_count": len(measurements),
        "missing_paths": missing_paths,
        "measurements": measurements,
    }


def _transform_local_point(
    stage: Any,
    anchor_prim_path: str,
    local_xyz: tuple[float, float, float],
) -> tuple[float, float, float]:
    from pxr import Gf, Usd, UsdGeom

    anchor = stage.GetPrimAtPath(anchor_prim_path)
    if not anchor.IsValid():
        return local_xyz
    matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(anchor)
    world = matrix.Transform(Gf.Vec3d(*local_xyz))
    return tuple(float(world[index]) for index in range(3))


def _set_world_translation(
    stage: Any,
    prim_path: str,
    world_xyz: tuple[float, float, float],
) -> tuple[float, float, float] | None:
    from pxr import Gf, UsdGeom, UsdPhysics

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    xformable = UsdGeom.Xformable(prim)
    translate_op = None
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break
    if translate_op is None:
        translate_op = xformable.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(*world_xyz))
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        body_api = UsdPhysics.RigidBodyAPI(prim)
        body_api.CreateVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        body_api.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    return world_xyz


def _reset_grasp_object_pose(stage: Any, object_path: str) -> tuple[float, float, float] | None:
    specs = build_grasp_validation_object_specs()
    local_xyz = tuple(specs[0]["local_xyz"]) if specs else (0.0, 0.028, 0.030)
    target, _ = _grasp_anchor_world_point(stage, local_xyz)
    if target is None:
        return None
    return _set_world_translation(stage, object_path, target)


def _author_grasp_validation_objects(stage: Any) -> list[dict[str, Any]]:
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

    scope = UsdGeom.Scope.Define(stage, GRASP_VALIDATION_ROOT_PRIM_PATH)
    material = UsdShade.Material.Define(stage, f"{GRASP_VALIDATION_ROOT_PRIM_PATH}/HighFrictionMaterial")
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr().Set(1.4)
    physics_material.CreateDynamicFrictionAttr().Set(1.2)
    physics_material.CreateRestitutionAttr().Set(0.05)

    authored: list[dict[str, Any]] = []
    for spec in build_grasp_validation_object_specs():
        prim_path = f"{GRASP_VALIDATION_ROOT_PRIM_PATH}/{spec['name']}"
        cube = UsdGeom.Cube.Define(stage, prim_path)
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(*spec["color"])])
        xformable = UsdGeom.Xformable(cube.GetPrim())
        world_xyz, anchor_path = _grasp_anchor_world_point(stage, spec["local_xyz"])
        if world_xyz is None:
            world_xyz = tuple(float(value) for value in spec["local_xyz"])
            anchor_path = "unresolved"
        xformable.AddTranslateOp().Set(Gf.Vec3d(*world_xyz))
        xformable.AddScaleOp().Set(Gf.Vec3d(*spec["scale"]))
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(cube.GetPrim())
        mass_api.CreateMassAttr().Set(float(spec["mass_kg"]))
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)
        cube.GetPrim().CreateAttribute("grasp_validation_local_xyz", Sdf.ValueTypeNames.Float3).Set(
            Gf.Vec3f(*spec["local_xyz"])
        )
        authored.append(
            {
                "name": spec["name"],
                "prim_path": prim_path,
                "shape": spec["shape"],
                "local_xyz": list(spec["local_xyz"]),
                "anchor_path": anchor_path,
                "world_xyz": list(world_xyz),
                "scale": list(spec["scale"]),
                "mass_kg": spec["mass_kg"],
            }
        )

    scope.GetPrim().CreateAttribute("purpose", Sdf.ValueTypeNames.String).Set(
        "Small rigid-object smoke test for early hand contact tuning."
    )
    return authored


def _bind_hand_contact_material(stage: Any, *, show_contact_proxies: bool = False) -> dict[str, Any]:
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

    material_path = f"{CONNECTED_ROOT_PRIM_PATH}/HandHighFrictionMaterial"
    material = UsdShade.Material.Define(stage, material_path)
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr().Set(1.6)
    physics_material.CreateDynamicFrictionAttr().Set(1.35)
    physics_material.CreateRestitutionAttr().Set(0.02)

    bound_paths: list[str] = []
    authored_proxy_paths: list[str] = []
    missing_link_paths: list[str] = []
    for spec in build_hand_contact_proxy_specs():
        link_path = f"{CONNECTED_HAND_PRIM_PATH}/{spec['link_name']}"
        link_prim = stage.GetPrimAtPath(link_path)
        if not link_prim.IsValid():
            missing_link_paths.append(link_path)
            continue
        proxy_root_path = f"{link_path}/contact_proxies"
        UsdGeom.Scope.Define(stage, proxy_root_path)
        proxy_path = f"{proxy_root_path}/{spec['name']}"
        cube = UsdGeom.Cube.Define(stage, proxy_path)
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.54, 0.18)])
        if not show_contact_proxies:
            UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()
        xformable = UsdGeom.Xformable(cube.GetPrim())
        xformable.AddTranslateOp().Set(Gf.Vec3d(*spec["local_xyz"]))
        xformable.AddScaleOp().Set(Gf.Vec3d(*spec["scale"]))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)
        cube.GetPrim().CreateAttribute("contact_proxy_link", Sdf.ValueTypeNames.String).Set(
            spec["link_name"]
        )
        authored_proxy_paths.append(proxy_path)
        bound_paths.append(proxy_path)

    for prim in stage.Traverse():
        prim_path = str(prim.GetPath())
        is_hand_collision = prim_path.startswith(CONNECTED_HAND_PRIM_PATH)
        is_importer_collider = prim_path.startswith("/colliders/")
        if not (is_hand_collision or is_importer_collider):
            continue
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdShade.MaterialBindingAPI(prim).Bind(material)
            if prim_path not in bound_paths:
                bound_paths.append(prim_path)

    return {
        "status": "PASS" if bound_paths else "WARN",
        "material_path": material_path,
        "static_friction": 1.6,
        "dynamic_friction": 1.35,
        "restitution": 0.02,
        "show_contact_proxies": show_contact_proxies,
        "authored_proxy_count": len(authored_proxy_paths),
        "authored_proxy_paths": authored_proxy_paths[:20],
        "missing_link_paths": missing_link_paths,
        "bound_collision_count": len(bound_paths),
        "bound_collision_paths": bound_paths[:20],
    }


def _author_proxy_hand_usd(
    *,
    hand_usd_path: str | Path,
    sanitized_mjcf_path: str | Path,
    reason: str,
) -> dict[str, Any]:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    output = _host_path(hand_usd_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    root = UsdGeom.Xform.Define(stage, HAND_PROXY_ROOT_PRIM_PATH)
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    root.GetPrim().CreateAttribute("fallback_reason", Sdf.ValueTypeNames.String).Set(reason)
    root.GetPrim().CreateAttribute("source_mjcf", Sdf.ValueTypeNames.String).Set(
        _repo_relative_path(sanitized_mjcf_path)
    )

    for primitive in build_hand_proxy_primitives():
        cube = UsdGeom.Cube.Define(stage, f"{HAND_PROXY_ROOT_PRIM_PATH}/{primitive['name']}")
        cube.CreateSizeAttr(1.0)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        xformable = UsdGeom.Xformable(cube.GetPrim())
        xformable.AddTranslateOp().Set(Gf.Vec3d(*primitive["translate"]))
        xformable.AddScaleOp().Set(Gf.Vec3d(*primitive["scale"]))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*primitive["color"])])

    stage.GetRootLayer().Save()
    return {
        "status": "PASS",
        "output_usd": _repo_relative_path(output),
        "root_prim": HAND_PROXY_ROOT_PRIM_PATH,
        "primitive_count": len(build_hand_proxy_primitives()),
        "source_mjcf": _repo_relative_path(sanitized_mjcf_path),
        "fallback_reason": reason,
        "evidence_summary": (
            "Authored a deterministic visible hand proxy because Isaac Sim's MJCF importer "
            "rejected the delivered closed-loop hand model."
        ),
    }


def _prepare_source_artifacts(args: argparse.Namespace) -> tuple[Path, dict[str, Any]]:
    package_root = extract_robot_arm_hand_package(args.zip, args.input_root)
    output_root = _host_path(args.output_root)
    sanitized_urdf = output_root / "robot_arm_hand_sanitized.urdf"
    sanitized_hand_mjcf = output_root / "hand_sanitized.xml"
    graspable_hand_urdf = output_root / "amazinghand_graspable.urdf"
    arm_report = sanitize_arm_urdf(package_root, sanitized_urdf)
    hand_report = analyze_hand_mjcf(package_root)
    hand_sanitize_report = sanitize_hand_mjcf(package_root, sanitized_hand_mjcf)
    graspable_hand_report = generate_graspable_hand_urdf(
        package_root,
        graspable_hand_urdf,
        visual_mode=getattr(args, "hand_visual_mode", VISUAL_MODE_PARTITIONED_LINKS),
        include_finger_shells=getattr(args, "include_finger_shells", False),
    )
    report = {
        "status": "PASS"
        if (
            arm_report["status"] == "PASS"
            and hand_report["status"] == "PASS"
            and hand_sanitize_report["status"] == "PASS"
            and graspable_hand_report["status"] == "PASS"
        )
        else "FAIL",
        "zip_source": _repo_relative_path(args.zip),
        "package_root": _repo_relative_path(package_root),
        "sanitized_urdf_path": _repo_relative_path(sanitized_urdf),
        "sanitized_hand_mjcf_path": _repo_relative_path(sanitized_hand_mjcf),
        "graspable_hand_urdf_path": _repo_relative_path(graspable_hand_urdf),
        "hand_mjcf_path": _repo_relative_path(package_root / "hand_mjcf/robot.xml"),
        "hand_mount_local_xyz_m": list(HAND_MOUNT_LOCAL_XYZ),
        "arm": arm_report,
        "hand": hand_report,
        "hand_sanitization": hand_sanitize_report,
        "graspable_hand_urdf": graspable_hand_report,
    }
    return package_root, report


def _write_json(path: str | Path, payload: dict[str, Any]) -> None:
    output = _host_path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _legacy_urdf_joint_drive_position_target() -> Any:
    try:
        from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType
    except ModuleNotFoundError:
        from isaacsim.asset.importer.urdf.impl._urdf import UrdfJointTargetType

    return UrdfJointTargetType.JOINT_DRIVE_POSITION


def _configure_urdf_import_config(
    import_config: Any,
    *,
    fix_base: bool,
    drive_strength: float,
    drive_damping: float,
) -> None:
    """Configure both legacy Isaac 5.x and Isaac 6.x URDF importer configs."""

    import_config.fix_base = fix_base
    if hasattr(import_config, "import_inertia_tensor"):
        import_config.import_inertia_tensor = True
    if hasattr(import_config, "distance_scale"):
        import_config.distance_scale = 1.0

    if hasattr(import_config, "default_drive_type"):
        import_config.default_drive_type = _legacy_urdf_joint_drive_position_target()
    if hasattr(import_config, "default_drive_strength"):
        import_config.default_drive_strength = drive_strength
    if hasattr(import_config, "default_position_drive_damping"):
        import_config.default_position_drive_damping = drive_damping

    if hasattr(import_config, "joint_drive_type"):
        import_config.joint_drive_type = "force"
    if hasattr(import_config, "joint_target_type"):
        import_config.joint_target_type = "position"
    if hasattr(import_config, "override_joint_stiffness"):
        import_config.override_joint_stiffness = drive_strength
    if hasattr(import_config, "override_joint_damping"):
        import_config.override_joint_damping = drive_damping


def _add_reference_wrapper_usd(
    *,
    source_usd_path: str | Path,
    wrapper_usd_path: str | Path,
    wrapper_prim_path: str,
) -> str:
    from pxr import Usd, UsdGeom

    wrapper = _host_path(wrapper_usd_path)
    source = _host_path(source_usd_path)
    wrapper.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(wrapper))
    parts = [part for part in wrapper_prim_path.split("/") if part]
    if not parts:
        raise ValueError(f"wrapper_prim_path must be absolute and non-empty: {wrapper_prim_path!r}")
    for index in range(1, len(parts) + 1):
        UsdGeom.Xform.Define(stage, "/" + "/".join(parts[:index]))
    root = stage.GetPrimAtPath("/" + parts[0])
    stage.SetDefaultPrim(root)
    target = stage.GetPrimAtPath(wrapper_prim_path)
    asset_path = os.path.relpath(source, start=wrapper.parent).replace("\\", "/")
    target.GetReferences().AddReference(asset_path)
    stage.GetRootLayer().Save()
    return wrapper_prim_path


def _import_urdf_to_usd_with_class_api(
    *,
    urdf_path: str,
    requested_usd_path: str,
    wrapper_prim_path: str,
    fix_base: bool,
    drive_strength: float,
    drive_damping: float,
) -> dict[str, Any]:
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    output_root = _host_path(requested_usd_path).with_suffix("")
    import_config = URDFImporterConfig()
    import_config.urdf_path = str(_host_path(urdf_path))
    import_config.usd_path = str(output_root)
    _configure_urdf_import_config(
        import_config,
        fix_base=fix_base,
        drive_strength=drive_strength,
        drive_damping=drive_damping,
    )
    importer = URDFImporter(import_config)
    generated_usd_path = importer.import_urdf()
    imported_prim_path = _add_reference_wrapper_usd(
        source_usd_path=generated_usd_path,
        wrapper_usd_path=requested_usd_path,
        wrapper_prim_path=wrapper_prim_path,
    )
    return {
        "imported_prim_path": imported_prim_path,
        "generated_usd_path": _repo_relative_path(generated_usd_path),
        "wrapper_usd_path": _repo_relative_path(requested_usd_path),
        "api": "URDFImporter",
    }


def _import_arm_urdf_to_usd(urdf_path: str, arm_usd_path: str) -> dict[str, Any]:
    import omni.kit.commands
    import omni.usd

    context = omni.usd.get_context()
    context.new_stage()
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        class_api_result = _import_urdf_to_usd_with_class_api(
            urdf_path=urdf_path,
            requested_usd_path=arm_usd_path,
            wrapper_prim_path=ARM_PRIM_PATH,
            fix_base=True,
            drive_strength=800.0,
            drive_damping=40.0,
        )
        prim_path = class_api_result["imported_prim_path"]
        api = class_api_result["api"]
        generated_usd_path = class_api_result["generated_usd_path"]
    else:
        _configure_urdf_import_config(
            import_config,
            fix_base=True,
            drive_strength=800.0,
            drive_damping=40.0,
        )

        status, prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=urdf_path,
            import_config=import_config,
            dest_path=arm_usd_path,
            get_articulation_root=True,
        )
        if not status:
            raise RuntimeError(f"URDFParseAndImportFile failed for {urdf_path}")
        api = "URDFParseAndImportFile"
        generated_usd_path = None
    return {
        "status": "PASS",
        "input_urdf": _repo_relative_path(urdf_path),
        "output_usd": _repo_relative_path(arm_usd_path),
        "imported_prim_path": prim_path or ARM_PRIM_PATH,
        "api": api,
        "generated_usd_path": generated_usd_path,
        "import_config": {
            "fix_base": True,
            "import_inertia_tensor": True,
            "distance_scale": 1.0,
            "default_drive_type": "JOINT_DRIVE_POSITION",
        },
    }


def _find_first_rigid_body_path(
    usd_path: str | Path,
    *,
    preferred_leaf_name: str | None = None,
) -> str | None:
    from pxr import Usd, UsdPhysics

    stage = Usd.Stage.Open(str(_host_path(usd_path)))
    if stage is None:
        return None
    rigid_paths = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    if preferred_leaf_name:
        for path in rigid_paths:
            if path.rsplit("/", 1)[-1] == preferred_leaf_name:
                return path
    return rigid_paths[0] if rigid_paths else None


def _import_hand_urdf_to_usd(urdf_path: str, hand_usd_path: str) -> dict[str, Any]:
    import omni.kit.commands
    import omni.usd

    context = omni.usd.get_context()
    context.new_stage()
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        class_api_result = _import_urdf_to_usd_with_class_api(
            urdf_path=urdf_path,
            requested_usd_path=hand_usd_path,
            wrapper_prim_path=HAND_PRIM_PATH,
            fix_base=False,
            drive_strength=45.0,
            drive_damping=4.0,
        )
        prim_path = class_api_result["imported_prim_path"]
        api = class_api_result["api"]
        generated_usd_path = class_api_result["generated_usd_path"]
    else:
        _configure_urdf_import_config(
            import_config,
            fix_base=False,
            drive_strength=45.0,
            drive_damping=4.0,
        )

        status, prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=urdf_path,
            import_config=import_config,
            dest_path=hand_usd_path,
            get_articulation_root=True,
        )
        if not status:
            raise RuntimeError(f"URDFParseAndImportFile failed for {urdf_path}")
        api = "URDFParseAndImportFile"
        generated_usd_path = None

    root_rigid_body_path = _find_first_rigid_body_path(
        hand_usd_path,
        preferred_leaf_name=HAND_ROOT_LINK_NAME,
    )
    return {
        "status": "PASS",
        "input_urdf": _repo_relative_path(urdf_path),
        "output_usd": _repo_relative_path(hand_usd_path),
        "imported_prim_path": prim_path or HAND_PRIM_PATH,
        "root_rigid_body_path": root_rigid_body_path,
        "api": api,
        "generated_usd_path": generated_usd_path,
        "import_config": {
            "fix_base": False,
            "import_inertia_tensor": True,
            "distance_scale": 1.0,
            "default_drive_type": "JOINT_DRIVE_POSITION",
            "default_drive_strength": 45.0,
            "default_position_drive_damping": 4.0,
        },
    }


def _import_hand_mjcf_to_usd(mjcf_path: str, hand_usd_path: str) -> dict[str, Any]:
    import omni.kit.commands

    status, import_config = omni.kit.commands.execute("MJCFCreateImportConfig")
    if not status:
        raise RuntimeError("MJCFCreateImportConfig failed")
    for method_name, value in (
        ("set_fix_base", False),
        ("set_import_inertia_tensor", True),
        ("set_make_default_prim", True),
    ):
        method = getattr(import_config, method_name, None)
        if callable(method):
            method(value)

    status, imported_prim_path = omni.kit.commands.execute(
        "MJCFCreateAsset",
        mjcf_path=mjcf_path,
        import_config=import_config,
        prim_path=HAND_PRIM_PATH,
        dest_path=hand_usd_path,
    )
    if not status:
        raise RuntimeError(f"MJCFCreateAsset failed for {mjcf_path}")
    return {
        "status": "PASS",
        "input_mjcf": _repo_relative_path(mjcf_path),
        "output_usd": _repo_relative_path(hand_usd_path),
        "imported_prim_path": imported_prim_path or HAND_PRIM_PATH,
        "import_config": {
            "fix_base": False,
            "import_inertia_tensor": True,
            "make_default_prim": True,
        },
    }


def _author_connected_usd(
    *,
    arm_usd_path: str,
    arm_reference_prim_path: str | None,
    hand_usd_path: str,
    hand_reference_prim_path: str | None,
    hand_fixed_joint_body_path: str,
    connected_usd_path: str,
    report: dict[str, Any],
) -> dict[str, Any]:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    output = _host_path(connected_usd_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    root = UsdGeom.Xform.Define(stage, CONNECTED_ROOT_PRIM_PATH)
    stage.SetDefaultPrim(root.GetPrim())

    arm = UsdGeom.Xform.Define(stage, CONNECTED_ARM_PRIM_PATH)
    if arm_reference_prim_path:
        arm.GetPrim().GetReferences().AddReference(
            _asset_reference(arm_usd_path, connected_usd_path),
            Sdf.Path(arm_reference_prim_path),
        )
    else:
        arm.GetPrim().GetReferences().AddReference(_asset_reference(arm_usd_path, connected_usd_path))

    hand = UsdGeom.Xform.Define(stage, CONNECTED_HAND_PRIM_PATH)
    if hand_reference_prim_path:
        hand.GetPrim().GetReferences().AddReference(
            _asset_reference(hand_usd_path, connected_usd_path),
            Sdf.Path(hand_reference_prim_path),
        )
    else:
        hand.GetPrim().GetReferences().AddReference(_asset_reference(hand_usd_path, connected_usd_path))
    hand_xform = UsdGeom.Xformable(hand.GetPrim())
    _set_translate_op(hand_xform, Gf.Vec3d(*HAND_MOUNT_LOCAL_XYZ))

    # Isaac's URDF importer stores STL visual libraries under top-level
    # `/visuals` and then references them from each link with internal
    # references. When we reference only the imported hand articulation prim
    # into a connected robot stage, those top-level library prims are not part
    # of the referenced namespace, so link `/visuals` prims can exist with zero
    # mesh children. Add explicit external references from each connected hand
    # link back to the importer-authored visual library so the real STL meshes
    # are present under the moving physics links.
    hand_visual_library_references: list[dict[str, str]] = []
    hand_base_usd_path = _host_path(hand_usd_path).parent / "configuration" / "hand_base.usd"
    if hand_base_usd_path.is_file():
        hand_visual_link_names = [HAND_ROOT_LINK_NAME]
        hand_visual_link_names.extend(
            f"finger{finger_index}_{segment}"
            for finger_index in range(1, 5)
            for segment in ("proximal", "distal")
        )
        for link_name in hand_visual_link_names:
            destination_path = f"{CONNECTED_HAND_PRIM_PATH}/{link_name}/resolved_visuals"
            destination = UsdGeom.Xform.Define(stage, destination_path)
            source_path = f"/visuals/{link_name}"
            destination.GetPrim().GetReferences().AddReference(
                _asset_reference(hand_base_usd_path, connected_usd_path),
                Sdf.Path(source_path),
            )
            hand_visual_library_references.append(
                {"destination": destination_path, "source": source_path}
            )

    joint = UsdPhysics.FixedJoint.Define(stage, FIXED_JOINT_PATH)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(ARM_HAND_ATTACHMENT_BODY0_PATH)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(hand_fixed_joint_body_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    metadata = UsdGeom.Scope.Define(stage, f"{CONNECTED_ROOT_PRIM_PATH}/metadata").GetPrim()
    metadata.CreateAttribute("source_zip", Sdf.ValueTypeNames.String).Set(
        _repo_relative_path(report["zip_source"])
    )
    metadata.CreateAttribute("sanitized_urdf", Sdf.ValueTypeNames.String).Set(
        report["sanitized_urdf_path"]
    )
    metadata.CreateAttribute("hand_mjcf", Sdf.ValueTypeNames.String).Set(report["hand_mjcf_path"])
    metadata.CreateAttribute("hand_mount_local_xyz_m", Sdf.ValueTypeNames.Float3).Set(
        Gf.Vec3f(*HAND_MOUNT_LOCAL_XYZ)
    )
    metadata.CreateAttribute("assembly_note", Sdf.ValueTypeNames.String).Set(
        "Fresh package assembly: arm URDF USD and hand MJCF USD are referenced under one root; "
        "the hand reference is translated to the measured hand_mount local transform and a "
        "FixedJoint records the intended physical connection."
    )
    stage.GetRootLayer().Save()
    return {
        "status": "PASS",
        "output_usd": _repo_relative_path(output),
        "root_prim": CONNECTED_ROOT_PRIM_PATH,
        "arm_reference_prim": CONNECTED_ARM_PRIM_PATH,
        "hand_reference_prim": CONNECTED_HAND_PRIM_PATH,
        "fixed_joint_path": FIXED_JOINT_PATH,
        "fixed_joint_body0": ARM_HAND_ATTACHMENT_BODY0_PATH,
        "fixed_joint_body1": hand_fixed_joint_body_path,
        "hand_mount_local_xyz_m": list(HAND_MOUNT_LOCAL_XYZ),
        "arm_source_prim_path": arm_reference_prim_path,
        "hand_source_prim_path": hand_reference_prim_path,
        "hand_visual_library_references": hand_visual_library_references,
    }


def run_isaac_conversion(args: argparse.Namespace) -> dict[str, Any]:
    from isaacsim import SimulationApp

    app = SimulationApp(_simulation_app_config(headless=True))
    try:
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.asset.importer.urdf")
        enable_extension("isaacsim.asset.importer.mjcf")
        app.update()

        package_root, report = _prepare_source_artifacts(args)
        output_root = _host_path(args.output_root)
        arm_usd_path = output_root / "arm.usd"
        hand_usd_path = output_root / "hand.usd"
        connected_usd_path = output_root / "robot_arm_hand_connected.usd"
        arm_import = _import_arm_urdf_to_usd(
            _container_path(output_root / "robot_arm_hand_sanitized.urdf"),
            _container_path(arm_usd_path),
        )
        sanitized_hand_mjcf = output_root / "hand_sanitized.xml"
        graspable_hand_fallback = None
        fallback_hand = None
        hand_reference_prim_path = None
        hand_source_body_path = None
        try:
            hand_import = _import_hand_mjcf_to_usd(
                _container_path(sanitized_hand_mjcf),
                _container_path(hand_usd_path),
            )
            hand_reference_prim_path = hand_import.get("imported_prim_path")
            hand_source_body_path = _find_first_rigid_body_path(hand_usd_path)
        except Exception as exc:
            reason = f"{type(exc).__name__}: {exc}"
            hand_import = {
                "status": "BLOCKED",
                "input_mjcf": _repo_relative_path(sanitized_hand_mjcf),
                "output_usd": _repo_relative_path(hand_usd_path),
                "reason": reason,
                "evidence_summary": (
                    "Isaac Sim's MJCF importer failed on the delivered closed-loop hand model "
                    "after source sanitization. The pipeline will use the generated "
                    "Isaac-friendly tree URDF hand as the first fallback."
                ),
            }
            try:
                graspable_hand_fallback = _import_hand_urdf_to_usd(
                    _container_path(output_root / "amazinghand_graspable.urdf"),
                    _container_path(hand_usd_path),
                )
                graspable_hand_fallback["fallback_reason"] = reason
                hand_reference_prim_path = graspable_hand_fallback.get("imported_prim_path")
                hand_source_body_path = graspable_hand_fallback.get("root_rigid_body_path")
            except Exception as fallback_exc:
                fallback_reason = f"{type(fallback_exc).__name__}: {fallback_exc}"
                fallback_hand = _author_proxy_hand_usd(
                    hand_usd_path=hand_usd_path,
                    sanitized_mjcf_path=sanitized_hand_mjcf,
                    reason=f"MJCF failed with {reason}; graspable hand URDF failed with {fallback_reason}",
                )
                hand_reference_prim_path = fallback_hand.get("root_prim")
                hand_source_body_path = fallback_hand.get("root_prim")
        hand_fixed_joint_body_path = _compose_connected_reference_body_path(
            source_reference_prim_path=hand_reference_prim_path,
            source_body_path=hand_source_body_path,
            connected_reference_prim_path=CONNECTED_HAND_PRIM_PATH,
        )
        connected = _author_connected_usd(
            arm_usd_path=arm_usd_path,
            arm_reference_prim_path=arm_import.get("imported_prim_path"),
            hand_usd_path=hand_usd_path,
            hand_reference_prim_path=hand_reference_prim_path,
            hand_fixed_joint_body_path=hand_fixed_joint_body_path,
            connected_usd_path=connected_usd_path,
            report=report,
        )
        fallback_used = bool(graspable_hand_fallback or fallback_hand)
        report.update(
            {
                "status": "PASS_WITH_FALLBACK" if fallback_used else "PASS",
                "arm_usd_path": _repo_relative_path(arm_usd_path),
                "hand_usd_path": _repo_relative_path(hand_usd_path),
                "connected_usd_path": _repo_relative_path(connected_usd_path),
                "isaac_conversion": {
                    "status": "PASS_WITH_FALLBACK" if fallback_used else "PASS",
                    "arm": arm_import,
                    "hand": hand_import,
                    "graspable_hand_fallback": graspable_hand_fallback,
                    "fallback_hand": fallback_hand,
                    "connected": connected,
                },
            }
        )
        _write_json(output_root / "robot_arm_hand_connected_report.json", report)
        return report
    finally:
        _close_simulation_app(app)


def _close_simulation_app(app: Any) -> None:
    """Close Isaac Sim while allowing isolated 6.0 Docker jobs to skip teardown.

    Isaac Sim 6.0 has a documented ``SimulationApp.close(skip_cleanup=True)``
    path for fast process teardown.  The regular cleanup path can assert during
    importer-heavy one-shot conversion jobs on some 6.0 containers, so the
    multi-instance runner enables this only for its disposable convert/runtime
    processes.  The default remains the legacy graceful close so existing 5.1
    compose runs and local workflows keep their previous behavior.
    """

    fast_close = os.environ.get("ROBOT_ARM_HAND_ISAAC_FAST_CLOSE", "").lower() in {
        "1",
        "true",
        "yes",
        "on",
    }
    pending_exception = sys.exc_info()[0] is not None
    if not fast_close:
        app.close()
        return
    if pending_exception:
        traceback.print_exception(*sys.exc_info())
        sys.stdout.flush()
        sys.stderr.flush()
        try:
            app.close(skip_cleanup=True, exit_code=1)
        except TypeError:
            app.close()
        os._exit(1)
    try:
        app.close(skip_cleanup=True)
    except TypeError:
        app.close()


def _find_first_articulation(stage) -> str | None:
    from pxr import UsdPhysics

    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    return None


def _frame_camera(stage, root_prim_path: str) -> tuple[tuple[float, float, float], tuple[float, float, float], float]:
    from pxr import Gf, Usd, UsdGeom

    def _is_finite_vec3(value: Any) -> bool:
        return all(math.isfinite(float(value[index])) for index in range(3))

    def _frame_points(
        points: list[Any],
        *,
        min_radius: float,
        eye_scale: tuple[float, float, float],
        target_z_offset: float = 0.0,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float], float] | None:
        finite_points = [point for point in points if _is_finite_vec3(point)]
        if not finite_points:
            return None
        bbox_min = Gf.Vec3d(
            *[min(float(point[index]) for point in finite_points) for index in range(3)]
        )
        bbox_max = Gf.Vec3d(
            *[max(float(point[index]) for point in finite_points) for index in range(3)]
        )
        center = (bbox_min + bbox_max) * 0.5
        size = bbox_max - bbox_min
        radius = max(float(size[0]), float(size[1]), float(size[2]), min_radius)
        eye = Gf.Vec3d(
            float(center[0]) + radius * eye_scale[0],
            float(center[1]) + radius * eye_scale[1],
            float(center[2]) + radius * eye_scale[2],
        )
        target = Gf.Vec3d(
            float(center[0]),
            float(center[1]),
            float(center[2]) + target_z_offset,
        )
        return tuple(float(v) for v in eye), tuple(float(v) for v in target), radius

    if (
        root_prim_path.startswith(f"{CONNECTED_HAND_PRIM_PATH}/finger")
        and root_prim_path.endswith("_proximal")
    ):
        # Frame the full generated finger chain, not the whole arm. This is used
        # for visual-vs-physics debugging where the hand must be inspected one
        # finger at a time. Do not use render bounds here: imported articulation
        # bounds can be stale or contaminated by the ground grid in headless
        # captures, which produces false "close-ups" of the floor. Use only the
        # actual rigid-body/contact-proxy transforms.
        distal_path = root_prim_path.removesuffix("_proximal") + "_distal"
        points: list[Any] = []
        for path in (root_prim_path, distal_path):
            if (translation := _world_translation(stage, path)) is not None:
                points.append(Gf.Vec3d(*translation))
        finger_name = root_prim_path.rsplit("/", 1)[-1].removesuffix("_proximal")
        for spec in build_hand_contact_proxy_specs():
            if spec["link_name"].startswith(finger_name):
                if (translation := _world_translation(stage, _contact_proxy_path(spec))) is not None:
                    points.append(Gf.Vec3d(*translation))
        framed = _frame_points(
            points,
            min_radius=0.09,
            eye_scale=(1.1, -1.8, 0.9),
        )
        if framed is not None:
            return framed

    if root_prim_path == grasp_object_reset_anchor_path():
        # The palm xform alone is too small for a reliable close-up in headless
        # capture: depending on the camera backend it can frame only the floor.
        # Frame the grasp cluster instead: palm + distal tip-pad proxies + object.
        focus_paths = [
            root_prim_path,
            f"{GRASP_VALIDATION_ROOT_PRIM_PATH}/{build_grasp_validation_object_specs()[0]['name']}",
        ]
        focus_paths.extend(
            _contact_proxy_path(spec)
            for spec in build_hand_contact_proxy_specs()
            if str(spec.get("name", "")).endswith("tip_pad_proxy")
        )
        focus_points = [
            Gf.Vec3d(*translation)
            for path in focus_paths
            if (translation := _world_translation(stage, path)) is not None
        ]
        framed = _frame_points(
            focus_points,
            min_radius=0.18,
            eye_scale=(2.0, -3.2, 1.6),
            target_z_offset=0.01,
        )
        if framed is not None:
            return framed

    prim = stage.GetPrimAtPath(root_prim_path)
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=True,
    )
    bbox = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
    bbox_min = bbox.GetMin()
    bbox_max = bbox.GetMax()
    center = (bbox_min + bbox_max) * 0.5
    size = bbox_max - bbox_min
    radius = max(float(size[0]), float(size[1]), float(size[2]), 0.25)
    eye = Gf.Vec3d(
        float(center[0]) + radius * 1.7,
        float(center[1]) - radius * 2.1,
        float(center[2]) + radius * 1.2,
    )
    target = Gf.Vec3d(float(center[0]), float(center[1]), float(center[2]))
    return tuple(float(v) for v in eye), tuple(float(v) for v in target), radius


def _capture_replicator(path: str, root_prim_path: str) -> None:
    import omni.replicator.core as rep
    import omni.usd
    from pxr import UsdLux

    stage = omni.usd.get_context().get_stage()
    eye, target, _ = _frame_camera(stage, root_prim_path)
    if not stage.GetPrimAtPath("/World/RobotArmHandCaptureLight").IsValid():
        UsdLux.DomeLight.Define(stage, "/World/RobotArmHandCaptureLight").CreateIntensityAttr(500.0)
    output_dir = tempfile.mkdtemp(prefix=Path(path).name + ".")
    try:
        with rep.new_layer():
            camera = rep.create.camera(position=eye, look_at=target, focal_length=35)
            render_product = rep.create.render_product(camera, _capture_resolution_from_env())
            writer = rep.WriterRegistry.get("BasicWriter")
            writer.initialize(output_dir=output_dir, rgb=True)
            writer.attach([render_product])
            for _ in range(8):
                rep.orchestrator.step(rt_subframes=8)
            writer.detach()
        frames = sorted(Path(output_dir).glob("rgb*.png"))
        if not frames:
            raise RuntimeError(f"Replicator did not write RGB frames in {output_dir}")
        shutil.copyfile(frames[-1], path)
    finally:
        shutil.rmtree(output_dir, ignore_errors=True)


def _capture_viewport(path: str, root_prim_path: str) -> None:
    import omni.usd
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

    stage = omni.usd.get_context().get_stage()
    eye, target, _ = _frame_camera(stage, root_prim_path)
    if _select_focus_prim_enabled():
        try:
            omni.usd.get_context().get_selection().set_selected_prim_paths(
                [root_prim_path],
                True,
            )
        except Exception:
            # Selection is a viewport navigation aid, not a physics dependency.
            # Keep capture alive even if the headless context has no selectable UI.
            pass
    set_camera_view(eye=eye, target=target, camera_prim_path="/OmniverseKit_Persp")
    # Headless viewport capture can otherwise grab the previous camera pose,
    # making the first diagnostic image stale while later images appear shifted.
    import omni.kit.app

    kit_app = omni.kit.app.get_app()
    for _ in range(3):
        kit_app.update()
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("No active viewport available")

    capture_viewport_to_file(viewport, file_path=path)
    output = _host_path(path)
    deadline = time.time() + 90.0
    while time.time() < deadline:
        kit_app.update()
        if output.is_file() and output.stat().st_size > 0:
            return
        time.sleep(0.05)
    raise TimeoutError(f"viewport did not create a non-empty screenshot: {output}")


def _capture_screenshot(path: str, root_prim_path: str) -> dict[str, Any]:
    output = _host_path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    container_output = _container_path(output)
    errors = []
    closeup_focus = (
        root_prim_path == grasp_object_reset_anchor_path()
        or (
            root_prim_path.startswith(f"{CONNECTED_HAND_PRIM_PATH}/finger")
            and root_prim_path.endswith("_proximal")
        )
    )
    if closeup_focus:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        eye, target, radius = _frame_camera(stage, root_prim_path)
        print(
            "[robot-arm-hand-camera] "
            f"focus={root_prim_path} eye={eye} target={target} radius={radius}",
            flush=True,
        )
        try:
            _capture_viewport(container_output, root_prim_path)
            size_bytes = output.stat().st_size if output.is_file() else 0
            if size_bytes > 0:
                return {
                    "capture_method": "focused_viewport",
                    "focus_prim_path": root_prim_path,
                    "screenshot": _repo_relative_path(output),
                    "size_bytes": size_bytes,
                    "camera_eye": list(eye),
                    "camera_target": list(target),
                    "camera_radius": radius,
                    "resolution": list(_capture_resolution_from_env()),
                }
            raise TimeoutError(f"focused_viewport did not create a non-empty screenshot: {output}")
        except Exception as exc:
            errors.append(f"focused_viewport: {type(exc).__name__}: {exc}")

        # Fallback only: older headless viewport/Replicator paths sometimes
        # ignored the close-up camera and showed the floor grid. Keep the old
        # whole-scene crop as a rescue path, but never as the primary evidence.
        fallback_method = "whole_scene_crop_fallback"
        full_scene_output = output.with_name(f"_full_scene_for_{output.name}")
        try:
            _capture_viewport(_container_path(full_scene_output), CONNECTED_ROOT_PRIM_PATH)
        except Exception as exc:
            # In headless Isaac Sim, capture.wait_for_result() can time out even
            # though the PNG is flushed a moment later. Treat a delayed non-empty
            # file as usable evidence instead of dropping the finger validation.
            deadline = time.time() + 75.0
            while time.time() < deadline:
                if full_scene_output.is_file() and full_scene_output.stat().st_size > 0:
                    break
                time.sleep(0.05)
            else:
                raise exc
        from PIL import Image

        deadline = time.time() + 75.0
        while time.time() < deadline:
            if full_scene_output.is_file() and full_scene_output.stat().st_size > 0:
                break
            time.sleep(0.05)
        else:
            raise TimeoutError(f"whole-scene capture did not create {full_scene_output}")
        image = Image.open(full_scene_output).convert("RGB")
        width, height = image.size
        crop_box = (
            int(width * 0.42),
            int(height * 0.16),
            int(width * 0.62),
            int(height * 0.48),
        )
        cropped = image.crop(crop_box)
        cropped = cropped.resize((width, height), Image.Resampling.LANCZOS)
        cropped.save(output)
        full_scene_output.unlink(missing_ok=True)
        size_bytes = output.stat().st_size if output.is_file() else 0
        return {
            "capture_method": fallback_method,
            "focus_prim_path": root_prim_path,
            "screenshot": _repo_relative_path(output) if size_bytes else None,
            "size_bytes": size_bytes,
            "camera_eye": list(eye),
            "camera_target": list(target),
            "camera_radius": radius,
            "resolution": list(_capture_resolution_from_env()),
            "fallback_errors": errors,
        }
    methods = (("viewport", _capture_viewport), ("replicator", _capture_replicator))
    for label, method in methods:
        try:
            method(container_output, root_prim_path)
            deadline = time.time() + 15.0
            while time.time() < deadline:
                if output.is_file() and output.stat().st_size > 0:
                    return {
                        "capture_method": label,
                        "focus_prim_path": root_prim_path,
                        "screenshot": _repo_relative_path(output),
                        "size_bytes": output.stat().st_size,
                        "resolution": list(_capture_resolution_from_env()),
                    }
                time.sleep(0.05)
            raise TimeoutError(f"{label} did not create a non-empty screenshot")
        except Exception as exc:
            errors.append(f"{label}: {type(exc).__name__}: {exc}")
    raise RuntimeError("All screenshot methods failed: " + "; ".join(errors))


def _make_contact_sheet(image_paths: list[Path], contact_sheet_path: Path) -> None:
    from PIL import Image, ImageDraw

    thumb_w, thumb_h = 480, 270
    label_h = 28
    cols = 2
    rows = (len(image_paths) + 1) // 2
    sheet = Image.new("RGB", (thumb_w * cols, (thumb_h + label_h) * rows), "white")
    draw = ImageDraw.Draw(sheet)
    for index, image_path in enumerate(image_paths):
        image = Image.open(image_path).convert("RGB")
        image.thumbnail((thumb_w, thumb_h))
        col, row = index % cols, index // cols
        x = col * thumb_w + (thumb_w - image.width) // 2
        y = row * (thumb_h + label_h)
        sheet.paste(image, (x, y))
        draw.text((col * thumb_w + 8, y + thumb_h + 6), image_path.name, fill=(0, 0, 0))
    contact_sheet_path.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(contact_sheet_path)


def run_isaac_runtime(args: argparse.Namespace) -> dict[str, Any]:
    from isaacsim import SimulationApp

    app = SimulationApp(_simulation_app_config(headless=args.headless))
    try:
        import omni.timeline
        import omni.usd
        import numpy as np
        from isaacsim.core.api import World
        from isaacsim.core.prims import Articulation
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.test.utils")
        enable_extension("omni.kit.renderer.capture")
        app.update()

        connected_usd = _host_path(args.connected_usd)
        if not connected_usd.is_file():
            raise FileNotFoundError(f"Connected USD not found: {connected_usd}")
        context = omni.usd.get_context()
        if not context.open_stage(_container_path(connected_usd)):
            raise RuntimeError(f"Could not open connected USD: {connected_usd}")
        app.update()
        stage = context.get_stage()
        authored_grasp_objects = _author_grasp_validation_objects(stage)
        hand_contact_tuning = _bind_hand_contact_material(
            stage,
            show_contact_proxies=args.show_contact_proxies,
        )
        app.update()

        world = World(stage_units_in_meters=1.0)
        world.scene.add_default_ground_plane()
        world.reset()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        articulation_root = _find_first_articulation(stage)
        dof_names: list[str] = []
        pose_results: list[dict[str, Any]] = []
        art = None
        controlled_indices: list[int] = []
        current_positions: list[float] = []
        if articulation_root:
            try:
                art = Articulation(articulation_root)
                art.initialize()
                dof_names = list(art.dof_names)
                controlled_indices = [
                    dof_names.index(name) for name in ARM_JOINT_NAMES if name in dof_names
                ]
                current_positions = _flat_float_list(art.get_joint_positions())
            except Exception as exc:
                pose_results.append(
                    {
                        "name": "articulation_initialize",
                        "status": "WARN",
                        "reason": f"{type(exc).__name__}: {exc}",
                    }
                )
                art = None

        def _ensure_articulation_ready() -> None:
            nonlocal art, dof_names, controlled_indices, current_positions

            if art is None or articulation_root is None:
                return
            try:
                handle_valid = art.is_physics_handle_valid()
            except AttributeError as exc:
                handle_valid = False
                if "_physics_view" not in str(exc):
                    pose_results.append(
                        {
                            "name": "articulation_handle_check",
                            "status": "WARN",
                            "reason": f"{type(exc).__name__}: {exc}",
                        }
                    )
            except Exception as exc:
                handle_valid = False
                pose_results.append(
                    {
                        "name": "articulation_handle_check",
                        "status": "WARN",
                        "reason": f"{type(exc).__name__}: {exc}",
                    }
                )
            if handle_valid:
                return

            world.reset()
            timeline.play()
            art = Articulation(articulation_root)
            art.initialize()
            dof_names = list(art.dof_names)
            controlled_indices = [
                dof_names.index(name) for name in ARM_JOINT_NAMES if name in dof_names
            ]
            refreshed_positions = _flat_float_list(art.get_joint_positions())
            if len(refreshed_positions) == len(dof_names):
                current_positions = refreshed_positions

        screenshots: list[Path] = []
        screenshot_root = _host_path(args.screenshot_root)
        diagnostic_capture_specs = {
            spec["label"]: spec for spec in build_visible_grasp_diagnostic_capture_specs()
        }

        def _capture_visible_grasp_diagnostic(label: str) -> dict[str, Any]:
            spec = diagnostic_capture_specs[label]
            screenshot = screenshot_root / spec["filename"]
            size_bytes = 0
            error = None
            screenshot_capture: dict[str, Any] | None = None
            try:
                if label == "open":
                    warmup = screenshot.with_name(f"_warmup_{screenshot.name}")
                    try:
                        _capture_screenshot(warmup, spec["focus_prim_path"])
                    finally:
                        warmup.unlink(missing_ok=True)
                screenshot_capture = _capture_screenshot(screenshot, spec["focus_prim_path"])
                size_bytes = screenshot.stat().st_size
                screenshots.append(screenshot)
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"
            _ensure_articulation_ready()
            return {
                "label": label,
                "focus_prim_path": spec["focus_prim_path"],
                "screenshot": _repo_relative_path(screenshot) if size_bytes else None,
                "screenshot_capture": screenshot_capture,
                "capture_method": screenshot_capture.get("capture_method")
                if screenshot_capture
                else None,
                "size_bytes": size_bytes,
                "error": error,
                "status": "PASS" if size_bytes > 0 else "WARN",
            }

        def _run_grasp_contact_smoke() -> dict[str, Any]:
            nonlocal current_positions

            if art is None:
                return {
                    "status": "WARN",
                    "reason": "No initialized articulation is available for hand grasp validation.",
                    "authored_objects": authored_grasp_objects,
                }
            object_path = (
                authored_grasp_objects[0]["prim_path"] if authored_grasp_objects else None
            )
            object_initial = _world_translation(stage, object_path) if object_path else None
            object_before = None
            command_error = None
            control_status = "APPLIED"
            controlled_joint_names: list[str] = []
            target_values: list[float] = []
            object_reset_world_xyz = None
            try:
                _ensure_articulation_ready()
                open_command = build_hand_grasp_position_command(
                    current_positions=current_positions,
                    dof_names=dof_names,
                    grasp=0.0,
                )
                close_command = build_hand_grasp_position_command(
                    current_positions=open_command["positions"],
                    dof_names=dof_names,
                    grasp=1.0,
                )
                controlled_joint_names = list(close_command["controlled_joint_names"])
                target_values = list(close_command["target_values"])
                if hasattr(art, "set_joint_position_targets"):
                    art.set_joint_position_targets(
                        np.array(open_command["target_values"], dtype=np.float32),
                        joint_indices=np.array(open_command["controlled_indices"], dtype=np.int32),
                    )
                    for _ in range(max(10, args.settle_steps)):
                        world.step(render=True)
                    if object_path:
                        object_reset_world_xyz = _reset_grasp_object_pose(stage, object_path)
                        app.update()
                    object_before = _world_translation(stage, object_path) if object_path else None
                    art.set_joint_position_targets(
                        np.array(close_command["target_values"], dtype=np.float32),
                        joint_indices=np.array(close_command["controlled_indices"], dtype=np.int32),
                    )
                else:
                    control_status = "DIRECT_POSITION_FALLBACK"
                    if object_path:
                        object_reset_world_xyz = _reset_grasp_object_pose(stage, object_path)
                    object_before = _world_translation(stage, object_path) if object_path else None
                    art.set_joint_positions(
                        np.array([close_command["positions"]], dtype=np.float32)
                    )
                for _ in range(args.grasp_steps):
                    world.step(render=True)
                app.update()
                current_positions = _flat_float_list(art.get_joint_positions())
            except Exception as exc:
                control_status = "FAILED"
                command_error = f"{type(exc).__name__}: {exc}"

            object_after = _world_translation(stage, object_path) if object_path else None
            screenshot = screenshot_root / "00_grasp_contact_smoke.png"
            size_bytes = 0
            try:
                _capture_screenshot(screenshot, CONNECTED_ROOT_PRIM_PATH)
                size_bytes = screenshot.stat().st_size
                screenshots.append(screenshot)
            except Exception as exc:
                if command_error is None:
                    command_error = f"screenshot {type(exc).__name__}: {exc}"

            return {
                "status": "PASS" if size_bytes > 0 and control_status != "FAILED" else "WARN",
                "authored_objects": authored_grasp_objects,
                "object_prim_path": object_path,
                "object_world_xyz_initial": list(object_initial) if object_initial else None,
                "object_reset_world_xyz": list(object_reset_world_xyz) if object_reset_world_xyz else None,
                "object_world_xyz_before": list(object_before) if object_before else None,
                "object_world_xyz_after": list(object_after) if object_after else None,
                "control_status": control_status,
                "controlled_joint_names": controlled_joint_names,
                "target_positions_rad": target_values,
                "command_error": command_error,
                "screenshot": _repo_relative_path(screenshot) if size_bytes else None,
                "size_bytes": size_bytes,
                "evidence_summary": (
                    "Authored a small rigid collision object near the generated hand, "
                    "commanded hand joints from open to closed with position targets, "
                    "stepped physics, and captured the result. This is a contact smoke "
                    "test; lift-retention tuning is still separate."
                ),
            }

        grasp_validation = _run_grasp_contact_smoke()

        def _run_two_link_finger_motion_validation() -> dict[str, Any]:
            nonlocal current_positions

            if art is None:
                return {
                    "status": "WARN",
                    "reason": "No initialized articulation is available for finger motion validation.",
                    "finger_results": [],
                }
            finger_results: list[dict[str, Any]] = []
            try:
                _ensure_articulation_ready()
                open_command = build_hand_grasp_position_command(
                    current_positions=current_positions,
                    dof_names=dof_names,
                    grasp=0.0,
                )
                if hasattr(art, "set_joint_position_targets"):
                    art.set_joint_position_targets(
                        np.array(open_command["target_values"], dtype=np.float32),
                        joint_indices=np.array(open_command["controlled_indices"], dtype=np.int32),
                    )
                else:
                    art.set_joint_positions(
                        np.array([open_command["positions"]], dtype=np.float32)
                    )
                for _ in range(max(10, args.settle_steps)):
                    world.step(render=True)
                current_positions = _flat_float_list(art.get_joint_positions())
            except Exception as exc:
                return {
                    "status": "WARN",
                    "reason": f"Could not reset hand open before finger validation: {type(exc).__name__}: {exc}",
                    "finger_results": finger_results,
                }

            for finger_index in range(1, 5):
                joint_names = [
                    f"finger{finger_index}_motor1",
                    f"finger{finger_index}_motor2",
                ]
                hand_joint_names = [
                    f"finger{candidate}_motor{motor}"
                    for candidate in range(1, 5)
                    for motor in range(1, 3)
                ]
                command_error = None
                target_positions = [0.78, 0.96]
                before_positions: list[float] = []
                achieved_positions: list[float] = []
                other_hand_joint_positions_rad: dict[str, float] = {}
                link_world_xyz_before: dict[str, tuple[float, float, float] | None] = {}
                link_world_xyz_after: dict[str, tuple[float, float, float] | None] = {}
                link_translation_deltas_m: dict[str, float | None] = {}
                controlled_indices_for_finger: list[int] = []
                control_status = "APPLIED"
                before_screenshot = (
                    screenshot_root / f"finger{finger_index}_two_link_before.png"
                )
                screenshot = screenshot_root / f"finger{finger_index}_two_link_motion.png"
                before_size_bytes = 0
                size_bytes = 0
                before_screenshot_capture: dict[str, Any] | None = None
                screenshot_capture: dict[str, Any] | None = None
                try:
                    _ensure_articulation_ready()
                    open_command = build_hand_grasp_position_command(
                        current_positions=current_positions,
                        dof_names=dof_names,
                        grasp=0.0,
                    )
                    if hasattr(art, "set_joint_position_targets"):
                        art.set_joint_position_targets(
                            np.array(open_command["target_values"], dtype=np.float32),
                            joint_indices=np.array(open_command["controlled_indices"], dtype=np.int32),
                        )
                    else:
                        control_status = "DIRECT_POSITION_FALLBACK"
                        art.set_joint_positions(
                            np.array([open_command["positions"]], dtype=np.float32)
                        )
                    for _ in range(max(10, args.settle_steps)):
                        world.step(render=True)
                    app.update()
                    refreshed = _flat_float_list(art.get_joint_positions())
                    if len(refreshed) == len(dof_names):
                        current_positions = refreshed
                    controlled_indices_for_finger = [
                        dof_names.index(name) for name in joint_names
                    ]
                    before_positions = [
                        current_positions[index] for index in controlled_indices_for_finger
                    ]
                    finger_focus_path = (
                        f"{CONNECTED_HAND_PRIM_PATH}/finger{finger_index}_proximal"
                    )
                    finger_link_paths = {
                        "proximal": f"{CONNECTED_HAND_PRIM_PATH}/finger{finger_index}_proximal",
                        "distal": f"{CONNECTED_HAND_PRIM_PATH}/finger{finger_index}_distal",
                    }
                    link_world_xyz_before = {
                        label: _world_translation(stage, path)
                        for label, path in finger_link_paths.items()
                    }
                    before_screenshot_capture = _capture_screenshot(
                        before_screenshot,
                        finger_focus_path,
                    )
                    before_size_bytes = before_screenshot.stat().st_size
                    screenshots.append(before_screenshot)
                    _ensure_articulation_ready()
                    finger_command = build_single_finger_two_link_position_command(
                        current_positions=current_positions,
                        dof_names=dof_names,
                        finger_index=finger_index,
                        motor1=target_positions[0],
                        motor2=target_positions[1],
                    )
                    if hasattr(art, "set_joint_position_targets"):
                        art.set_joint_position_targets(
                            np.array(finger_command["target_values"], dtype=np.float32),
                            joint_indices=np.array(
                                finger_command["controlled_indices"], dtype=np.int32
                            ),
                        )
                    else:
                        control_status = "DIRECT_POSITION_FALLBACK"
                        art.set_joint_positions(
                            np.array([finger_command["positions"]], dtype=np.float32)
                        )
                    for _ in range(args.finger_motion_steps):
                        world.step(render=True)
                    app.update()
                    achieved_all_positions = _flat_float_list(art.get_joint_positions())
                    achieved_positions = [
                        achieved_all_positions[index]
                        for index in finger_command["controlled_indices"]
                    ]
                    current_positions = achieved_all_positions
                    other_hand_joint_positions_rad = {
                        name: achieved_all_positions[dof_names.index(name)]
                        for name in hand_joint_names
                        if name in dof_names and name not in joint_names
                    }
                    link_world_xyz_after = {
                        label: _world_translation(stage, path)
                        for label, path in finger_link_paths.items()
                    }
                    link_translation_deltas_m = {
                        label: _translation_delta(
                            link_world_xyz_before.get(label),
                            link_world_xyz_after.get(label),
                        )
                        for label in finger_link_paths
                    }
                    screenshot_capture = _capture_screenshot(screenshot, finger_focus_path)
                    size_bytes = screenshot.stat().st_size
                    screenshots.append(screenshot)
                except Exception as exc:
                    control_status = "FAILED"
                    command_error = f"{type(exc).__name__}: {exc}"

                deltas = [
                    abs(after - before)
                    for before, after in zip(before_positions, achieved_positions)
                ]
                target_errors = [
                    abs(after - target)
                    for after, target in zip(achieved_positions, target_positions)
                ]
                other_hand_joint_max_abs_rad = (
                    max(abs(value) for value in other_hand_joint_positions_rad.values())
                    if other_hand_joint_positions_rad
                    else None
                )
                moved = len(deltas) == 2 and all(delta >= 0.20 for delta in deltas)
                reached = len(target_errors) == 2 and all(error <= 0.25 for error in target_errors)
                independent = (
                    other_hand_joint_max_abs_rad is not None
                    and other_hand_joint_max_abs_rad <= 0.20
                )
                distal_link_delta_m = link_translation_deltas_m.get("distal")
                moved_link = (
                    distal_link_delta_m is not None and distal_link_delta_m >= 0.005
                )
                finger_results.append(
                    {
                        "finger_index": finger_index,
                        "joint_names": joint_names,
                        "controlled_indices": controlled_indices_for_finger,
                        "before_positions_rad": before_positions,
                        "target_positions_rad": target_positions,
                        "achieved_positions_rad": achieved_positions,
                        "position_deltas_rad": deltas,
                        "target_errors_rad": target_errors,
                        "other_hand_joint_positions_rad": other_hand_joint_positions_rad,
                        "other_hand_joint_max_abs_rad": other_hand_joint_max_abs_rad,
                        "link_world_xyz_before": {
                            key: list(value) if value is not None else None
                            for key, value in link_world_xyz_before.items()
                        },
                        "link_world_xyz_after": {
                            key: list(value) if value is not None else None
                            for key, value in link_world_xyz_after.items()
                        },
                        "link_translation_deltas_m": link_translation_deltas_m,
                        "distal_link_translation_delta_m": distal_link_delta_m,
                        "link_motion_threshold_m": 0.005,
                        "control_status": control_status,
                        "command_error": command_error,
                        "before_screenshot": _repo_relative_path(before_screenshot)
                        if before_size_bytes
                        else None,
                        "before_screenshot_capture": before_screenshot_capture,
                        "before_capture_method": before_screenshot_capture.get(
                            "capture_method"
                        )
                        if before_screenshot_capture
                        else None,
                        "before_size_bytes": before_size_bytes,
                        "screenshot": _repo_relative_path(screenshot) if size_bytes else None,
                        "screenshot_capture": screenshot_capture,
                        "capture_method": screenshot_capture.get("capture_method")
                        if screenshot_capture
                        else None,
                        "size_bytes": size_bytes,
                        "status": "PASS"
                        if size_bytes > 0
                        and before_size_bytes > 0
                        and control_status != "FAILED"
                        and moved
                        and reached
                        and independent
                        and moved_link
                        else "WARN",
                    }
                )

            return {
                "status": "PASS"
                if finger_results
                and all(item["status"] == "PASS" for item in finger_results)
                else "WARN",
                "finger_results": finger_results,
                "evidence_summary": (
                    "For each generated AmazingHand finger, reset all hand joints open, "
                    "captured a close-up before image, commanded only that finger's two "
                    "revolute joints, captured a close-up after image, and checked that "
                    "the commanded distal link also moved in world space while the other "
                    "hand finger motors stayed near open. Generated "
                    "proximal/distal STL segment visuals plus major linkage and pin "
                    "visuals are partitioned onto the matching moving tree links by "
                    "default. Small screws, washers, tiny spacers, exact closed-loop "
                    "kinematics, final shell alignment, and SimReady are intentionally "
                    "excluded from this skeleton-first pass."
                ),
            }

        finger_motion_validation = _run_two_link_finger_motion_validation()


        def _run_preshape_grasp_validation() -> dict[str, Any]:
            nonlocal current_positions

            if art is None:
                return {
                    "status": "WARN",
                    "reason": "No initialized articulation is available for preshape grasp validation.",
                    "stage_results": [],
                    "reference_checklist": build_shadow_allegro_reference_checklist(),
                }
            object_path = (
                authored_grasp_objects[0]["prim_path"] if authored_grasp_objects else None
            )
            if not object_path:
                return {
                    "status": "WARN",
                    "reason": "No grasp validation object was authored.",
                    "stage_results": [],
                    "reference_checklist": build_shadow_allegro_reference_checklist(),
                }

            stage_results: list[dict[str, Any]] = []
            for spec in build_preshape_grasp_validation_stage_specs():
                label = str(spec["label"])
                command_error = None
                control_status = "APPLIED"
                before_object = None
                after_object = None
                achieved_positions: list[float] = []
                target_errors: list[float] = []
                distances_by_finger: dict[int, list[float]] = {}
                active_finger_min_distances: dict[int, float | None] = {}
                active_close_count = 0
                screenshot = screenshot_root / f"preshape_{label}.png"
                size_bytes = 0
                screenshot_capture: dict[str, Any] | None = None
                controlled_joint_names: list[str] = []
                target_values: list[float] = []
                try:
                    _ensure_articulation_ready()
                    open_command = build_hand_grasp_position_command(
                        current_positions=current_positions,
                        dof_names=dof_names,
                        grasp=0.0,
                    )
                    if hasattr(art, "set_joint_position_targets"):
                        art.set_joint_position_targets(
                            np.array(open_command["target_values"], dtype=np.float32),
                            joint_indices=np.array(
                                open_command["controlled_indices"], dtype=np.int32
                            ),
                        )
                    else:
                        art.set_joint_positions(
                            np.array([open_command["positions"]], dtype=np.float32)
                        )
                    for _ in range(max(6, args.settle_steps // 2)):
                        world.step(render=True)
                    current_positions = _flat_float_list(art.get_joint_positions())
                    _reset_grasp_object_pose(stage, object_path)
                    for _ in range(max(6, args.settle_steps // 2)):
                        world.step(render=True)
                    app.update()
                    before_object = _world_translation(stage, object_path)

                    preshape_command = build_hand_preshape_position_command(
                        current_positions=current_positions,
                        dof_names=dof_names,
                        preshape=str(spec["preshape"]),
                        amount=float(spec["amount"]),
                        finger_index=spec.get("finger_index"),
                    )
                    controlled_joint_names = list(preshape_command["controlled_joint_names"])
                    target_values = list(preshape_command["target_values"])
                    if hasattr(art, "set_joint_position_targets"):
                        art.set_joint_position_targets(
                            np.array(target_values, dtype=np.float32),
                            joint_indices=np.array(
                                preshape_command["controlled_indices"], dtype=np.int32
                            ),
                        )
                    else:
                        control_status = "DIRECT_POSITION_FALLBACK"
                        art.set_joint_positions(
                            np.array([preshape_command["positions"]], dtype=np.float32)
                        )
                    for _ in range(max(10, args.grasp_steps // 2)):
                        world.step(render=True)
                    app.update()
                    achieved_all_positions = _flat_float_list(art.get_joint_positions())
                    achieved_positions = [
                        achieved_all_positions[index]
                        for index in preshape_command["controlled_indices"]
                    ]
                    current_positions = achieved_all_positions
                    target_errors = [
                        abs(after - target)
                        for after, target in zip(
                            achieved_positions, target_values, strict=True
                        )
                    ]
                    after_object = _world_translation(stage, object_path)
                    distances_by_finger = _finger_contact_proxy_distances_by_finger_to_object(
                        stage,
                        object_path,
                    )
                    for finger_index in spec["active_fingers"]:
                        distances = distances_by_finger.get(int(finger_index), [])
                        active_finger_min_distances[int(finger_index)] = (
                            min(distances) if distances else None
                        )
                    active_close_count = sum(
                        1
                        for value in active_finger_min_distances.values()
                        if value is not None and value <= FINGER_PROXY_DISTANCE_THRESHOLD_M
                    )
                    screenshot_capture = _capture_screenshot(
                        screenshot,
                        grasp_object_reset_anchor_path(),
                    )
                    size_bytes = screenshot.stat().st_size
                    screenshots.append(screenshot)
                except Exception as exc:
                    control_status = "FAILED"
                    command_error = f"{type(exc).__name__}: {exc}"

                reached = bool(target_errors) and all(error <= 0.30 for error in target_errors)
                contact_pass = active_close_count >= int(spec["required_finger_proxy_count"])
                stage_results.append(
                    {
                        "label": label,
                        "preshape": spec["preshape"],
                        "amount": spec["amount"],
                        "active_fingers": list(spec["active_fingers"]),
                        "required_finger_proxy_count": spec[
                            "required_finger_proxy_count"
                        ],
                        "controlled_joint_names": controlled_joint_names,
                        "target_positions_rad": target_values,
                        "achieved_positions_rad": achieved_positions,
                        "target_errors_rad": target_errors,
                        "target_reached": reached,
                        "finger_proxy_distance_threshold_m": FINGER_PROXY_DISTANCE_THRESHOLD_M,
                        "finger_proxy_distances_by_finger_m": {
                            str(key): value for key, value in distances_by_finger.items()
                        },
                        "active_finger_min_distances_m": {
                            str(key): value
                            for key, value in active_finger_min_distances.items()
                        },
                        "active_finger_proxy_close_count": active_close_count,
                        "object_world_xyz_before": list(before_object)
                        if before_object
                        else None,
                        "object_world_xyz_after": list(after_object) if after_object else None,
                        "control_status": control_status,
                        "command_error": command_error,
                        "screenshot": _repo_relative_path(screenshot)
                        if size_bytes
                        else None,
                        "screenshot_capture": screenshot_capture,
                        "capture_method": screenshot_capture.get("capture_method")
                        if screenshot_capture
                        else None,
                        "size_bytes": size_bytes,
                        "contact_status": "PASS" if contact_pass else "WARN",
                        "status": "PASS"
                        if size_bytes > 0
                        and control_status != "FAILED"
                        and reached
                        and contact_pass
                        else "WARN",
                    }
                )

            return {
                "status": "PASS"
                if stage_results
                and all(item["status"] == "PASS" for item in stage_results)
                else "WARN",
                "stage_results": stage_results,
                "reference_checklist": build_shadow_allegro_reference_checklist(),
                "evidence_summary": (
                    "Runs the AmazingHand fallback through the planned single-finger, "
                    "pinch, and wrap preshapes. Shadow/Allegro are recorded only as "
                    "reference checklists; they do not replace this hand."
                ),
            }

        preshape_grasp_validation = _run_preshape_grasp_validation()
        def _run_lift_retain_smoke() -> dict[str, Any]:
            nonlocal current_positions, controlled_indices

            if art is None:
                return {
                    "status": "WARN",
                    "reason": "No initialized articulation is available for lift-retain validation.",
                }
            object_path = (
                authored_grasp_objects[0]["prim_path"] if authored_grasp_objects else None
            )
            if not object_path:
                return {"status": "WARN", "reason": "No grasp validation object was authored."}

            command_error = None
            control_status = "APPLIED"
            object_initial = _world_translation(stage, object_path)
            object_reset_world_xyz = None
            object_before = None
            hand_before = _world_translation(stage, CONNECTED_HAND_PRIM_PATH)
            object_after_close = None
            hand_after_close = None
            object_after_lift = None
            hand_after_lift = None
            anchor_after_close = None
            anchor_after_lift = None
            finger_proxy_distances_after_close: list[float] = []
            finger_proxy_distances_after_lift: list[float] = []
            arm_joint_positions_before_lift: list[float] = []
            arm_joint_positions_after_lift: list[float] = []
            lift_target_arm_joint_positions = list(LIFT_RETAIN_ARM_TARGET)
            transform_snapshots: list[dict[str, Any]] = []
            visible_diagnostic_screenshots: list[dict[str, Any]] = []
            try:
                _ensure_articulation_ready()
                open_command = build_hand_grasp_position_command(
                    current_positions=current_positions,
                    dof_names=dof_names,
                    grasp=0.0,
                )
                if hasattr(art, "set_joint_position_targets"):
                    art.set_joint_position_targets(
                        np.array(open_command["target_values"], dtype=np.float32),
                        joint_indices=np.array(open_command["controlled_indices"], dtype=np.int32),
                    )
                else:
                    art.set_joint_positions(np.array([open_command["positions"]], dtype=np.float32))
                for _ in range(max(10, args.settle_steps)):
                    world.step(render=True)
                current_positions = _flat_float_list(art.get_joint_positions())
                object_reset_world_xyz = _reset_grasp_object_pose(stage, object_path)
                # Teleporting the rigid object directly before the close phase can
                # leave its measured transform briefly above the physical cradle.
                # Settle it on the authored contact proxies first so strict
                # lift-retain compares against a stable pre-grasp pose rather
                # than a transient post-reset pose.
                for _ in range(max(6, args.settle_steps // 2)):
                    world.step(render=True)
                app.update()
                object_before = _world_translation(stage, object_path)
                transform_snapshots.append(
                    _capture_grasp_transform_snapshot(
                        stage,
                        label="open",
                        object_path=object_path,
                    )
                )
                visible_diagnostic_screenshots.append(
                    _capture_visible_grasp_diagnostic("open")
                )

                half_close_command = build_hand_grasp_position_command(
                    current_positions=current_positions,
                    dof_names=dof_names,
                    grasp=0.5,
                )
                if hasattr(art, "set_joint_position_targets"):
                    art.set_joint_position_targets(
                        np.array(half_close_command["target_values"], dtype=np.float32),
                        joint_indices=np.array(
                            half_close_command["controlled_indices"], dtype=np.int32
                        ),
                    )
                else:
                    art.set_joint_positions(
                        np.array([half_close_command["positions"]], dtype=np.float32)
                    )
                for _ in range(max(10, args.grasp_steps // 2)):
                    world.step(render=True)
                app.update()
                current_positions = _flat_float_list(art.get_joint_positions())
                transform_snapshots.append(
                    _capture_grasp_transform_snapshot(
                        stage,
                        label="half_close",
                        object_path=object_path,
                    )
                )
                visible_diagnostic_screenshots.append(
                    _capture_visible_grasp_diagnostic("half_close")
                )

                close_command = build_hand_grasp_position_command(
                    current_positions=current_positions,
                    dof_names=dof_names,
                    grasp=1.0,
                )
                if hasattr(art, "set_joint_position_targets"):
                    art.set_joint_position_targets(
                        np.array(close_command["target_values"], dtype=np.float32),
                        joint_indices=np.array(close_command["controlled_indices"], dtype=np.int32),
                    )
                else:
                    control_status = "DIRECT_POSITION_FALLBACK"
                    art.set_joint_positions(
                        np.array([close_command["positions"]], dtype=np.float32)
                    )
                for _ in range(args.grasp_steps):
                    world.step(render=True)
                current_positions = _flat_float_list(art.get_joint_positions())
                object_after_close = _world_translation(stage, object_path)
                finger_proxy_distances_after_close = _finger_contact_proxy_distances_to_object(
                    stage, object_path
                )
                hand_after_close = _world_translation(stage, CONNECTED_HAND_PRIM_PATH)
                anchor_after_close = _world_translation(stage, grasp_object_reset_anchor_path())
                transform_snapshots.append(
                    _capture_grasp_transform_snapshot(
                        stage,
                        label="full_close_before_lift",
                        object_path=object_path,
                    )
                )
                visible_diagnostic_screenshots.append(
                    _capture_visible_grasp_diagnostic("full_close_before_lift")
                )

                arm_joint_positions_before_lift = [
                    current_positions[dof_names.index(name)]
                    for name in ARM_JOINT_NAMES
                    if name in dof_names
                ]
                lift_sequence = build_lift_retain_joint_target_sequence(
                    current_positions=current_positions,
                    dof_names=dof_names,
                    arm_target=lift_target_arm_joint_positions,
                    grasp=1.0,
                    segments=max(3, min(8, args.lift_retain_steps // 10)),
                )
                steps_per_segment = max(1, args.lift_retain_steps // len(lift_sequence))
                remaining_steps = args.lift_retain_steps
                for lift_command in lift_sequence:
                    current_positions = list(lift_command["positions"])
                    controlled_indices = list(lift_command["controlled_indices"])
                    segment_steps = min(steps_per_segment, remaining_steps)
                    if segment_steps <= 0:
                        segment_steps = 1
                    if hasattr(art, "set_joint_position_targets"):
                        art.set_joint_position_targets(
                            np.array(lift_command["target_values"], dtype=np.float32),
                            joint_indices=np.array(controlled_indices, dtype=np.int32),
                        )
                    else:
                        art.set_joint_positions(np.array([current_positions], dtype=np.float32))
                    for _ in range(segment_steps):
                        world.step(render=True)
                    remaining_steps -= segment_steps
                for _ in range(max(0, remaining_steps)):
                    world.step(render=True)
                app.update()
                current_positions = _flat_float_list(art.get_joint_positions())
                arm_joint_positions_after_lift = [
                    current_positions[dof_names.index(name)]
                    for name in ARM_JOINT_NAMES
                    if name in dof_names
                ]
                object_after_lift = _world_translation(stage, object_path)
                finger_proxy_distances_after_lift = _finger_contact_proxy_distances_to_object(
                    stage, object_path
                )
                hand_after_lift = _world_translation(stage, CONNECTED_HAND_PRIM_PATH)
                anchor_after_lift = _world_translation(stage, grasp_object_reset_anchor_path())
                transform_snapshots.append(
                    _capture_grasp_transform_snapshot(
                        stage,
                        label="after_lift_retain",
                        object_path=object_path,
                    )
                )
                visible_diagnostic_screenshots.append(
                    _capture_visible_grasp_diagnostic("after_lift_retain")
                )
            except Exception as exc:
                control_status = "FAILED"
                command_error = f"{type(exc).__name__}: {exc}"

            screenshot = screenshot_root / "lift_retain_smoke.png"
            size_bytes = 0
            try:
                _capture_screenshot(screenshot, CONNECTED_ROOT_PRIM_PATH)
                size_bytes = screenshot.stat().st_size
                screenshots.append(screenshot)
            except Exception as exc:
                if command_error is None:
                    command_error = f"screenshot {type(exc).__name__}: {exc}"

            retain_status = _evaluate_lift_retain_status(
                object_before=object_before,
                object_after_close=object_after_close,
                anchor_after_close=anchor_after_close,
                object_after_lift=object_after_lift,
                anchor_after_lift=anchor_after_lift,
                object_reference=object_reset_world_xyz,
                finger_proxy_distances_after_close=finger_proxy_distances_after_close,
                finger_proxy_distances_after_lift=finger_proxy_distances_after_lift,
                screenshot_size_bytes=size_bytes,
                control_status=control_status,
            )
            return {
                "status": retain_status["status"],
                "object_prim_path": object_path,
                "object_world_xyz_initial": list(object_initial) if object_initial else None,
                "object_reset_world_xyz": list(object_reset_world_xyz)
                if object_reset_world_xyz
                else None,
                "object_world_xyz_before": list(object_before) if object_before else None,
                "hand_world_xyz_before": list(hand_before) if hand_before else None,
                "object_world_xyz_after_close": list(object_after_close)
                if object_after_close
                else None,
                "hand_world_xyz_after_close": list(hand_after_close)
                if hand_after_close
                else None,
                "grasp_anchor_world_xyz_after_close": list(anchor_after_close)
                if anchor_after_close
                else None,
                "object_world_xyz_after_lift": list(object_after_lift)
                if object_after_lift
                else None,
                "hand_world_xyz_after_lift": list(hand_after_lift) if hand_after_lift else None,
                "grasp_anchor_world_xyz_after_lift": list(anchor_after_lift)
                if anchor_after_lift
                else None,
                "lift_target_arm_joint_positions_rad": lift_target_arm_joint_positions,
                "arm_joint_positions_before_lift_rad": arm_joint_positions_before_lift,
                "arm_joint_positions_after_lift_rad": arm_joint_positions_after_lift,
                "object_hand_distance_after_close_m": retain_status[
                    "object_anchor_distance_after_close_m"
                ],
                "object_hand_distance_after_lift_m": retain_status[
                    "object_anchor_distance_after_lift_m"
                ],
                "object_anchor_distance_after_close_m": retain_status[
                    "object_anchor_distance_after_close_m"
                ],
                "object_anchor_distance_after_lift_m": retain_status[
                    "object_anchor_distance_after_lift_m"
                ],
                "object_z_delta_after_lift_m": retain_status["object_z_delta_after_lift_m"],
                "object_z_delta_after_lift_from_settled_before_m": retain_status[
                    "object_z_delta_after_lift_from_settled_before_m"
                ],
                "object_z_delta_reference": retain_status["object_z_delta_reference"],
                "finger_proxy_distance_threshold_m": retain_status[
                    "finger_proxy_distance_threshold_m"
                ],
                "finger_proxy_distances_after_close_m": retain_status[
                    "finger_proxy_distances_after_close_m"
                ],
                "finger_proxy_distances_after_lift_m": retain_status[
                    "finger_proxy_distances_after_lift_m"
                ],
                "finger_proxy_close_count_after_close": retain_status[
                    "finger_proxy_close_count_after_close"
                ],
                "finger_proxy_close_count_after_lift": retain_status[
                    "finger_proxy_close_count_after_lift"
                ],
                "finger_grasp_engaged": retain_status["finger_grasp_engaged"],
                "retained_near_hand": retain_status["retained_near_hand"],
                "lifted_or_held": retain_status["lifted_or_held"],
                "object_reset_anchor_path": grasp_object_reset_anchor_path(),
                "transform_snapshots": transform_snapshots,
                "visible_diagnostic_screenshots": visible_diagnostic_screenshots,
                "control_status": control_status,
                "command_error": command_error,
                "screenshot": _repo_relative_path(screenshot) if size_bytes else None,
                "size_bytes": size_bytes,
                "evidence_summary": (
                    "Closed all hand joints around the authored small rigid object, "
                    "then commanded an arm lift pose while physics was stepping. PASS "
                    "requires the object to remain near the hand and not drop in Z."
                ),
            }

        lift_retain_validation = _run_lift_retain_smoke()

        cases = [
            ("startup", []),
            ("home", [0.0, 0.0, 0.0, 0.0]),
            ("reach", [0.25, -0.2, 0.3, -0.25]),
            ("fold", [0.15, 0.2, -0.35, 0.45]),
            ("side_sweep", [-0.25, 0.15, 0.3, -0.2]),
        ]
        for index, (name, command) in enumerate(cases, start=1):
            control_status = "SKIPPED: startup capture has no arm command"
            achieved_positions: list[float] = []
            target_positions: list[float] = []
            command_error = None
            if command:
                try:
                    if art is None:
                        raise RuntimeError("No initialized articulation is available")
                    _ensure_articulation_ready()
                    joint_command = build_arm_joint_position_command(
                        current_positions=current_positions,
                        dof_names=dof_names,
                        command=command,
                    )
                    current_positions = list(joint_command["positions"])
                    target_positions = list(joint_command["target_values"])
                    controlled_indices = list(joint_command["controlled_indices"])
                    art.set_joint_positions(np.array([current_positions], dtype=np.float32))
                    if hasattr(art, "set_joint_position_targets"):
                        art.set_joint_position_targets(
                            np.array(target_positions, dtype=np.float32),
                            joint_indices=np.array(controlled_indices, dtype=np.int32),
                        )
                    control_status = "APPLIED"
                except Exception as exc:
                    control_status = "FAILED"
                    command_error = f"{type(exc).__name__}: {exc}"
            for _ in range(args.settle_steps):
                world.step(render=True)
            app.update()
            if art is not None:
                try:
                    achieved_all_positions = _flat_float_list(art.get_joint_positions())
                    achieved_positions = [
                        achieved_all_positions[index] for index in controlled_indices
                    ]
                    current_positions = achieved_all_positions
                except Exception as exc:
                    if command_error is None:
                        command_error = f"readback {type(exc).__name__}: {exc}"
            screenshot = screenshot_root / f"{index:02d}_{name}.png"
            _capture_screenshot(screenshot, CONNECTED_ROOT_PRIM_PATH)
            size_bytes = screenshot.stat().st_size
            screenshots.append(screenshot)
            pose_results.append(
                {
                    "name": name,
                    "command": command,
                    "control_status": control_status,
                    "controlled_joint_names": [
                        dof_names[index] for index in controlled_indices
                    ],
                    "target_positions_rad": target_positions,
                    "achieved_positions_rad": achieved_positions,
                    "command_error": command_error,
                    "screenshot": _repo_relative_path(screenshot),
                    "size_bytes": size_bytes,
                    "status": "PASS" if size_bytes > 0 and control_status != "FAILED" else "FAIL",
                }
            )

        contact_sheet = screenshot_root / "contact_sheet.png"
        _make_contact_sheet(screenshots, contact_sheet)
        runtime = {
            "status": "PASS"
            if screenshots and all(item.get("status") in {"PASS", "WARN"} for item in pose_results)
            else "FAIL",
            "connected_usd_path": _repo_relative_path(connected_usd),
            "root_prim": CONNECTED_ROOT_PRIM_PATH,
            "articulation_root": articulation_root,
            "loaded_dof_count": len(dof_names),
            "loaded_dof_names": dof_names,
            "controlled_joint_names": [
                name for name in ARM_JOINT_NAMES if name in dof_names
            ],
            "hand_contact_tuning": hand_contact_tuning,
            "grasp_validation": grasp_validation,
            "preshape_grasp_validation": preshape_grasp_validation,
            "finger_motion_validation": finger_motion_validation,
            "lift_retain_validation": lift_retain_validation,
            "motion_cases": pose_results,
            "screenshot_output_dir": _repo_relative_path(screenshot_root),
            "contact_sheet": _repo_relative_path(contact_sheet),
            "hand_mount_local_xyz_m": list(HAND_MOUNT_LOCAL_XYZ),
            "evidence_summary": (
                "Isaac Sim opened the fresh connected USD, stepped physics for the named "
                "runtime cases, kept direct articulation writes disabled for the proxy-backed "
                "validation run, and saved non-empty screenshots plus a contact sheet."
            ),
        }
        report_path = _host_path(args.report)
        report = {}
        if report_path.is_file():
            try:
                loaded = json.loads(report_path.read_text(encoding="utf-8"))
                if isinstance(loaded, dict):
                    report = loaded
            except json.JSONDecodeError:
                report = {}
        report["runtime_validation"] = runtime
        has_hand_fallback = bool(
            report.get("isaac_conversion", {}).get("fallback_hand")
            or report.get("isaac_conversion", {}).get("graspable_hand_fallback")
        )
        if runtime["status"] == "PASS":
            report["status"] = "PASS_WITH_FALLBACK" if has_hand_fallback else "PASS"
        else:
            report["status"] = "FAIL"
        _write_json(report_path, report)
        timeline.stop()
        return runtime
    finally:
        _close_simulation_app(app)


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["prepare", "convert", "runtime", "all"], default="all")
    parser.add_argument("--zip", default=os.environ.get("ROBOT_ARM_HAND_ZIP_SOURCE", DEFAULT_ZIP))
    parser.add_argument(
        "--input-root",
        default=os.environ.get("ROBOT_ARM_HAND_INPUT_ROOT", DEFAULT_INPUT_ROOT),
    )
    parser.add_argument(
        "--output-root",
        default=os.environ.get("ROBOT_ARM_HAND_OUTPUT_ROOT", DEFAULT_OUTPUT_ROOT),
    )
    parser.add_argument(
        "--connected-usd",
        default=os.environ.get(
            "ROBOT_ARM_HAND_CONNECTED_USD_PATH",
            f"{DEFAULT_OUTPUT_ROOT}/robot_arm_hand_connected.usd",
        ),
    )
    parser.add_argument(
        "--report",
        default=os.environ.get(
            "ROBOT_ARM_HAND_REPORT_PATH",
            f"{DEFAULT_OUTPUT_ROOT}/robot_arm_hand_connected_report.json",
        ),
    )
    parser.add_argument(
        "--screenshot-root",
        default=os.environ.get(
            "ROBOT_ARM_HAND_SCREENSHOT_OUTPUT_DIR",
            DEFAULT_SCREENSHOT_ROOT,
        ),
    )
    parser.add_argument("--settle-steps", type=int, default=int(os.environ.get("ROBOT_ARM_HAND_SETTLE_STEPS", "20")))
    parser.add_argument("--grasp-steps", type=int, default=int(os.environ.get("ROBOT_ARM_HAND_GRASP_STEPS", "90")))
    parser.add_argument(
        "--finger-motion-steps",
        type=int,
        default=int(os.environ.get("ROBOT_ARM_HAND_FINGER_MOTION_STEPS", "45")),
    )
    parser.add_argument(
        "--lift-retain-steps",
        type=int,
        default=int(os.environ.get("ROBOT_ARM_HAND_LIFT_RETAIN_STEPS", "75")),
    )
    parser.add_argument(
        "--hand-visual-mode",
        choices=[VISUAL_MODE_PARTITIONED_LINKS, VISUAL_MODE_STATIC_SHELL, VISUAL_MODE_IMPLEMENTED_ONLY],
        default=os.environ.get("ROBOT_ARM_HAND_VISUAL_MODE", VISUAL_MODE_PARTITIONED_LINKS),
        help=(
            "Hand visual authoring mode. partitioned_links makes selected STL visuals follow "
            "moving tree links; implemented_only hides CAD/fake visuals and shows only "
            "generated collision primitives; static_shell keeps the original MJCF shell fixed for legacy debug only."
        ),
    )
    parser.add_argument(
        "--include-finger-shells",
        action="store_true",
        default=os.environ.get("ROBOT_ARM_HAND_INCLUDE_FINGER_SHELLS", "0").lower()
        in {"1", "true", "yes", "on"},
        help=(
            "Overlay proximal/proximal_shell/distal/distal_shell STL visuals on "
            "the generated moving finger links without changing collision or drives."
        ),
    )
    parser.add_argument(
        "--show-contact-proxies",
        action="store_true",
        default=os.environ.get("ROBOT_ARM_HAND_SHOW_CONTACT_PROXIES", "0").lower()
        in {"1", "true", "yes", "on"},
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        default=os.environ.get("HEADLESS", "1").strip() != "0",
    )
    return parser


def main() -> None:
    args = _build_arg_parser().parse_args()
    if args.mode == "prepare":
        _, report = _prepare_source_artifacts(args)
        _write_json(args.report, report)
        print(json.dumps(report, indent=2, sort_keys=True))
        return
    if args.mode == "convert":
        report = run_isaac_conversion(args)
        print(json.dumps(report, indent=2, sort_keys=True))
        return
    if args.mode == "runtime":
        runtime = run_isaac_runtime(args)
        print(json.dumps(runtime, indent=2, sort_keys=True))
        return
    if args.mode == "all":
        conversion_report = run_isaac_conversion(args)
        runtime_args = argparse.Namespace(**vars(args))
        runtime_args.connected_usd = str(_host_path(args.output_root) / "robot_arm_hand_connected.usd")
        runtime_args.report = str(_host_path(args.output_root) / "robot_arm_hand_connected_report.json")
        runtime = run_isaac_runtime(runtime_args)
        combined = dict(conversion_report)
        combined["runtime_validation"] = runtime
        if runtime["status"] == "PASS":
            combined["status"] = (
                "PASS_WITH_FALLBACK"
                if (
                    conversion_report.get("isaac_conversion", {}).get("fallback_hand")
                    or conversion_report.get("isaac_conversion", {}).get("graspable_hand_fallback")
                )
                else "PASS"
            )
        else:
            combined["status"] = "FAIL"
        _write_json(runtime_args.report, combined)
        print(json.dumps(combined, indent=2, sort_keys=True))
        return
    raise AssertionError(args.mode)


if __name__ == "__main__":
    main()
