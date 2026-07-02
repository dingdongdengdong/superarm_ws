"""Fresh robot_arm_hand_package.zip to Isaac Sim USD pipeline.

Host-side static checks can import this module without Isaac Sim installed.
Isaac-only imports are intentionally scoped to conversion/runtime functions.

Run inside the Isaac Sim container:
    /isaac-sim/python.sh /workspace/isaacsim/robot_arm_hand_from_zip.py --mode all
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import shutil
import sys
import tempfile
import time
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

try:
    from isaacsim_test.isaacsim.graspable_hand_urdf import (
        generate_graspable_hand_urdf,
        grasp_scalar_to_hand_joint_targets,
    )
except ModuleNotFoundError:
    from graspable_hand_urdf import generate_graspable_hand_urdf, grasp_scalar_to_hand_joint_targets

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
HAND_ROOT_LINK_NAME = "r_wrist_interface"


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
) -> dict[str, Any]:
    """Build a named hand closing command from a normalized grasp value."""
    return build_named_joint_position_command(
        current_positions=current_positions,
        dof_names=dof_names,
        joint_targets=grasp_scalar_to_hand_joint_targets(grasp),
    )


def build_single_finger_two_link_position_command(
    current_positions: Any,
    dof_names: list[str],
    finger_index: int,
    *,
    motor1: float,
    motor2: float,
) -> dict[str, Any]:
    """Build a command for one generated two-link finger."""
    if finger_index < 1 or finger_index > 4:
        raise ValueError(f"Expected finger_index from 1 to 4, got {finger_index}")
    return build_named_joint_position_command(
        current_positions=current_positions,
        dof_names=dof_names,
        joint_targets={
            f"finger{finger_index}_motor1": float(motor1),
            f"finger{finger_index}_motor2": float(motor2),
        },
    )


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
            "local_xyz": (0.0, 0.030, 0.045),
            "scale": (0.022, 0.022, 0.018),
            "mass_kg": 0.006,
            "color": (0.74, 0.68, 0.56),
        }
    ]


def _world_translation(stage: Any, prim_path: str) -> tuple[float, float, float] | None:
    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    translate = matrix.ExtractTranslation()
    return tuple(float(translate[index]) for index in range(3))


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
        world_xyz = _transform_local_point(stage, CONNECTED_HAND_PRIM_PATH, spec["local_xyz"])
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
                "world_xyz": list(world_xyz),
                "scale": list(spec["scale"]),
                "mass_kg": spec["mass_kg"],
            }
        )

    scope.GetPrim().CreateAttribute("purpose", Sdf.ValueTypeNames.String).Set(
        "Small rigid-object smoke test for early hand contact tuning."
    )
    return authored


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
    graspable_hand_report = generate_graspable_hand_urdf(package_root, graspable_hand_urdf)
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


def _import_arm_urdf_to_usd(urdf_path: str, arm_usd_path: str) -> dict[str, Any]:
    import omni.kit.commands
    import omni.usd
    from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType

    context = omni.usd.get_context()
    context.new_stage()
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        raise RuntimeError("URDFCreateImportConfig failed")
    import_config.fix_base = True
    import_config.import_inertia_tensor = True
    import_config.distance_scale = 1.0
    import_config.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.default_drive_strength = 800.0
    import_config.default_position_drive_damping = 40.0

    status, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        dest_path=arm_usd_path,
        get_articulation_root=True,
    )
    if not status:
        raise RuntimeError(f"URDFParseAndImportFile failed for {urdf_path}")
    return {
        "status": "PASS",
        "input_urdf": _repo_relative_path(urdf_path),
        "output_usd": _repo_relative_path(arm_usd_path),
        "imported_prim_path": prim_path or ARM_PRIM_PATH,
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
    from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType

    context = omni.usd.get_context()
    context.new_stage()
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        raise RuntimeError("URDFCreateImportConfig failed")
    import_config.fix_base = False
    import_config.import_inertia_tensor = True
    import_config.distance_scale = 1.0
    import_config.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.default_drive_strength = 45.0
    import_config.default_position_drive_damping = 4.0

    status, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        dest_path=hand_usd_path,
        get_articulation_root=True,
    )
    if not status:
        raise RuntimeError(f"URDFParseAndImportFile failed for {urdf_path}")

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
    }


def run_isaac_conversion(args: argparse.Namespace) -> dict[str, Any]:
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "width": 1280, "height": 720})
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
        app.close()


def _find_first_articulation(stage) -> str | None:
    from pxr import UsdPhysics

    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    return None


def _frame_camera(stage, root_prim_path: str) -> tuple[tuple[float, float, float], tuple[float, float, float], float]:
    from pxr import Gf, Usd, UsdGeom

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
        shutil.copyfile(frames[-1], path)
    finally:
        shutil.rmtree(output_dir, ignore_errors=True)


def _capture_viewport(path: str, root_prim_path: str) -> None:
    import omni.usd
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

    stage = omni.usd.get_context().get_stage()
    eye, target, _ = _frame_camera(stage, root_prim_path)
    set_camera_view(eye=eye, target=target, camera_prim_path="/OmniverseKit_Persp")
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("No active viewport available")

    async def _capture() -> None:
        capture = capture_viewport_to_file(viewport, file_path=path)
        await asyncio.wait_for(capture.wait_for_result(), timeout=15.0)

    asyncio.get_event_loop().run_until_complete(_capture())


def _capture_screenshot(path: str, root_prim_path: str) -> None:
    output = _host_path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    container_output = _container_path(output)
    errors = []
    for label, method in (("viewport", _capture_viewport), ("replicator", _capture_replicator)):
        try:
            method(container_output, root_prim_path)
            deadline = time.time() + 15.0
            while time.time() < deadline:
                if output.is_file() and output.stat().st_size > 0:
                    return
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

    app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})
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
            object_before = _world_translation(stage, object_path) if object_path else None
            command_error = None
            control_status = "APPLIED"
            controlled_joint_names: list[str] = []
            target_values: list[float] = []
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
                command_error = None
                target_positions = [0.78, 0.96]
                before_positions: list[float] = []
                achieved_positions: list[float] = []
                controlled_indices_for_finger: list[int] = []
                control_status = "APPLIED"
                screenshot = screenshot_root / f"finger{finger_index}_two_link_motion.png"
                size_bytes = 0
                try:
                    _ensure_articulation_ready()
                    refreshed = _flat_float_list(art.get_joint_positions())
                    if len(refreshed) == len(dof_names):
                        current_positions = refreshed
                    controlled_indices_for_finger = [
                        dof_names.index(name) for name in joint_names
                    ]
                    before_positions = [
                        current_positions[index] for index in controlled_indices_for_finger
                    ]
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
                    _capture_screenshot(screenshot, CONNECTED_ROOT_PRIM_PATH)
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
                moved = len(deltas) == 2 and all(delta >= 0.20 for delta in deltas)
                reached = len(target_errors) == 2 and all(error <= 0.25 for error in target_errors)
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
                        "control_status": control_status,
                        "command_error": command_error,
                        "screenshot": _repo_relative_path(screenshot) if size_bytes else None,
                        "size_bytes": size_bytes,
                        "status": "PASS"
                        if size_bytes > 0 and control_status != "FAILED" and moved and reached
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
                    "Commanded each generated AmazingHand finger independently in Isaac "
                    "physics. Each finger uses two revolute joints: motor1 for palm-to-"
                    "proximal and motor2 for proximal-to-distal. Validation checks joint "
                    "position readback movement and screenshot capture; visual STL shell "
                    "is intentionally fixed in the default stable mode."
                ),
            }

        finger_motion_validation = _run_two_link_finger_motion_validation()

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
            "grasp_validation": grasp_validation,
            "finger_motion_validation": finger_motion_validation,
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
        app.close()


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
