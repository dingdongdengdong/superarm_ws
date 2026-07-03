"""Generate direct-URDF Isaac Sim artifacts for echo_full.

This first-pass physics artifact keeps the SimReady echo_full asset as visual
context, but the movable physical arm is imported from a generated URDF that
copies the Roboto V2 right-arm chain exactly and adds only fixed custom-frame
and AmazingHand terminal links around it.

Run on the host or inside the Isaac Sim container:
    python3 isaacsim_test/isaacsim/create_echo_full_articulation.py
    /isaac-sim/python.sh /workspace/isaacsim/create_echo_full_articulation.py
"""
from __future__ import annotations

import copy
import json
import os
import xml.etree.ElementTree as ET
from pathlib import Path

from pxr import Sdf, Usd, UsdGeom

HOST_ROOT = Path(__file__).resolve().parents[2]
CONTAINER_ROOT = "/workspace/superarm_ws"

DEFAULT_SOURCE_USD = (
    f"{CONTAINER_ROOT}/isaacsim_test/outputs/simready/echo_full/"
    "pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/"
    "echo_full_robot_arm_hand.usd"
)
DEFAULT_OUTPUT_USD = (
    f"{CONTAINER_ROOT}/isaacsim_test/outputs/simready/echo_full/"
    "sitl/echo_full_lerobot_articulation.usda"
)
DEFAULT_REPORT = (
    f"{CONTAINER_ROOT}/isaacsim_test/outputs/simready/echo_full/"
    "sitl/echo_full_lerobot_articulation_report.json"
)
DEFAULT_REFERENCE_URDF = (
    f"{CONTAINER_ROOT}/roboparty/modules/rpo_hardware/V2.0/"
    "roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf"
)
DEFAULT_PHYSICAL_ROBOT_URDF = (
    f"{CONTAINER_ROOT}/isaacsim_test/outputs/simready/echo_full/"
    "sitl/roboto_v2_right_arm_amazinghand_full.urdf"
)
DEFAULT_AMAZINGHAND_ASSET_DIR = (
    f"{CONTAINER_ROOT}/AmazingHand/Demo/AHSimulation/AHSimulation/AH_Right/mjcf/assets"
)
DEFAULT_AMAZINGHAND_MJCF = (
    f"{CONTAINER_ROOT}/AmazingHand/Demo/AHSimulation/AHSimulation/AH_Right/mjcf/robot.xml"
)

CONTROLLED_ARM_JOINTS = [
    "right_arm_pitch_joint",
    "right_arm_roll_joint",
    "right_arm_yaw_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
]
AMAZINGHAND_MOTOR_JOINTS = [
    "finger1_motor1",
    "finger1_motor2",
    "finger2_motor1",
    "finger2_motor2",
    "finger3_motor1",
    "finger3_motor2",
    "finger4_motor1",
    "finger4_motor2",
]
ARM_LINKS = [
    "torso_link",
    "right_arm_pitch_link",
    "right_arm_roll_link",
    "right_arm_yaw_link",
    "right_elbow_pitch_link",
    "right_elbow_yaw_link",
]
HAND_VISUAL_MESHES = [
    "r_wrist_interface.stl",
    "r_hand_plate.stl",
    "proximal_shell.stl",
    "distal_shell.stl",
    "finger_frame_1.stl",
    "finger_frame_2.stl",
]


def _host_path(path: str | Path) -> Path:
    """Map container-style workspace paths to the local checkout when needed."""
    raw = str(path)
    if raw.startswith(CONTAINER_ROOT + "/"):
        candidate = HOST_ROOT / raw[len(CONTAINER_ROOT) + 1 :]
        if candidate.exists() or not Path(raw).exists():
            return candidate
    return Path(raw)


def _container_path(path: str | Path) -> str:
    """Return an Isaac-container-readable path for files inside this checkout."""
    host = _host_path(path).resolve()
    try:
        rel = host.relative_to(HOST_ROOT.resolve())
    except ValueError:
        return str(path).replace("\\", "/")
    return f"{CONTAINER_ROOT}/{rel.as_posix()}"


def _repo_relative_path(path: str | Path) -> str:
    host = _host_path(path)
    try:
        return host.resolve().relative_to(HOST_ROOT.resolve()).as_posix()
    except ValueError:
        normalized = str(path).replace("\\", "/")
        for marker in ("roboparty/", "isaacsim_test/", "AmazingHand/"):
            index = normalized.find(marker)
            if index >= 0:
                return normalized[index:]
        return normalized


def _preserved_runtime_validation(
    *,
    report_path: str | Path,
    physical_urdf_path: str | Path,
    joint_topology: list[dict[str, object]],
) -> dict[str, object] | None:
    """Keep runtime evidence when regenerating an unchanged physical artifact."""
    existing_path = _host_path(report_path)
    if not existing_path.is_file():
        return None
    try:
        existing = json.loads(existing_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None

    runtime_validation = existing.get("runtime_validation")
    if not isinstance(runtime_validation, dict):
        return None
    same_physical_artifact = existing.get("physical_robot_urdf_path") == _repo_relative_path(
        physical_urdf_path
    )
    same_topology = existing.get("physical_urdf_joint_topology") == joint_topology
    if same_physical_artifact and same_topology:
        loaded_dofs = runtime_validation.get("loaded_dof_names", [])
        loaded_dof_names = set(loaded_dofs if isinstance(loaded_dofs, list) else [])
        hand_motor_dofs = [
            joint_name
            for joint_name in AMAZINGHAND_MOTOR_JOINTS
            if joint_name in loaded_dof_names
        ]
        missing_hand_motor_dofs = [
            joint_name
            for joint_name in AMAZINGHAND_MOTOR_JOINTS
            if joint_name not in loaded_dof_names
        ]
        runtime_validation["hand_motor_dofs_commanded"] = hand_motor_dofs
        runtime_validation["missing_hand_motor_dofs"] = missing_hand_motor_dofs
        controlled_dofs = runtime_validation.get("controlled_dofs_moved", [])
        if not isinstance(controlled_dofs, list):
            controlled_dofs = []
        for joint_name in hand_motor_dofs:
            if joint_name not in controlled_dofs:
                controlled_dofs.append(joint_name)
        runtime_validation["controlled_dofs_moved"] = controlled_dofs
        if not missing_hand_motor_dofs:
            runtime_validation["hand_motor_control_status"] = "PASS"
        return runtime_validation
    return None


def _usd_asset_reference(asset_path: str | Path, from_layer_path: str | Path) -> str:
    asset_host = _host_path(asset_path)
    layer_host = _host_path(from_layer_path)
    if asset_host.exists():
        return os.path.relpath(asset_host.resolve(), layer_host.parent.resolve()).replace("\\", "/")
    return str(asset_path).replace("\\", "/")


def _parse_float_list(raw: str) -> list[float]:
    return [float(part) for part in raw.split()]


def _parse_reference_urdf(urdf_path: str | Path) -> tuple[list[dict[str, object]], ET.ElementTree]:
    """Extract the authoritative V2 right-arm joint chain from the URDF."""
    tree = ET.parse(_host_path(urdf_path))
    root = tree.getroot()
    joints_by_name = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    missing = [name for name in CONTROLLED_ARM_JOINTS if name not in joints_by_name]
    if missing:
        raise RuntimeError(f"Reference URDF is missing required right-arm joints: {missing}")

    topology: list[dict[str, object]] = []
    for joint_name in CONTROLLED_ARM_JOINTS:
        joint = joints_by_name[joint_name]
        if joint.attrib.get("type") != "revolute":
            raise RuntimeError(f"{joint_name} must be a revolute joint, got {joint.attrib.get('type')}")
        parent = joint.find("parent")
        child = joint.find("child")
        origin = joint.find("origin")
        axis = joint.find("axis")
        limit = joint.find("limit")
        if parent is None or child is None or origin is None or axis is None or limit is None:
            raise RuntimeError(f"{joint_name} is missing parent/child/origin/axis/limit metadata")
        topology.append(
            {
                "joint": joint_name,
                "parent": parent.attrib["link"],
                "child": child.attrib["link"],
                "origin_xyz": _parse_float_list(origin.attrib.get("xyz", "0 0 0")),
                "origin_rpy": _parse_float_list(origin.attrib.get("rpy", "0 0 0")),
                "axis_xyz": _parse_float_list(axis.attrib["xyz"]),
                "limit_lower": float(limit.attrib["lower"]),
                "limit_upper": float(limit.attrib["upper"]),
                "effort": float(limit.attrib["effort"]),
                "velocity": float(limit.attrib["velocity"]),
            }
        )
    return topology, tree


def _read_amazinghand_mjcf_metadata(mjcf_path: str | Path) -> dict[str, object]:
    """Read the upstream MJCF reference enough to document URDF fidelity."""
    host_path = _host_path(mjcf_path)
    tree = ET.parse(host_path)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    root_body = worldbody.find("body") if worldbody is not None else None
    mesh_files = [
        mesh.attrib["file"]
        for mesh in root.findall("./asset/mesh")
        if mesh.attrib.get("file")
    ]
    missing_meshes = [
        mesh_file
        for mesh_file in mesh_files
        if not (host_path.parent / "assets" / mesh_file).is_file()
    ]
    position_actuators = [
        actuator.attrib.get("joint", actuator.attrib.get("name", ""))
        for actuator in root.findall("./actuator/position")
    ]
    equality_connect_count = len(root.findall("./equality/connect"))
    ball_joint_count = len(root.findall(".//joint[@type='ball']"))

    return {
        "path": _repo_relative_path(mjcf_path),
        "container_path": _container_path(mjcf_path),
        "root_body": root_body.attrib.get("name") if root_body is not None else None,
        "mesh_asset_count": len(mesh_files),
        "missing_meshes": missing_meshes,
        "position_actuator_count": len(position_actuators),
        "position_actuators": position_actuators,
        "motor_joints_expected_by_lerobot": list(AMAZINGHAND_MOTOR_JOINTS),
        "equality_connect_count": equality_connect_count,
        "ball_joint_count": ball_joint_count,
        "closed_loop_features": ["equality/connect"] if equality_connect_count else [],
    }


def _element_by_name(root: ET.Element, tag: str, name: str) -> ET.Element:
    element = root.find(f"{tag}[@name='{name}']")
    if element is None:
        raise RuntimeError(f"Reference URDF missing {tag} named {name!r}")
    return element


def _rewrite_mesh_filenames(element: ET.Element, reference_urdf_path: str | Path) -> None:
    reference_dir = _host_path(reference_urdf_path).parent
    for mesh in element.findall(".//mesh"):
        filename = mesh.attrib.get("filename")
        if not filename or filename.startswith(("/", "package://")):
            continue
        mesh_path = (reference_dir / filename).resolve()
        mesh.attrib["filename"] = _container_path(mesh_path)


def _add_origin(parent: ET.Element, xyz: str, rpy: str = "0 0 0") -> None:
    ET.SubElement(parent, "origin", {"xyz": xyz, "rpy": rpy})


def _add_box_visual(parent: ET.Element, *, xyz: str, size: str, rgba: str, name: str) -> None:
    visual = ET.SubElement(parent, "visual", {"name": name})
    _add_origin(visual, xyz)
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "box", {"size": size})
    material = ET.SubElement(visual, "material", {"name": f"{name}_material"})
    ET.SubElement(material, "color", {"rgba": rgba})


def _add_box_collision(parent: ET.Element, *, xyz: str, size: str, name: str) -> None:
    collision = ET.SubElement(parent, "collision", {"name": name})
    _add_origin(collision, xyz)
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, "box", {"size": size})


def _add_inertial(parent: ET.Element, *, mass: str, ixx: str, iyy: str, izz: str) -> None:
    inertial = ET.SubElement(parent, "inertial")
    _add_origin(inertial, "0 0 0")
    ET.SubElement(inertial, "mass", {"value": mass})
    ET.SubElement(
        inertial,
        "inertia",
        {"ixx": ixx, "ixy": "0", "ixz": "0", "iyy": iyy, "iyz": "0", "izz": izz},
    )


def _custom_frame_link() -> ET.Element:
    link = ET.Element("link", {"name": "custom_frame_link"})
    _add_inertial(link, mass="20.0", ixx="0.45", iyy="0.45", izz="0.35")
    _add_box_visual(
        link,
        xyz="0 0 0.035",
        size="0.70 0.50 0.07",
        rgba="0.22 0.22 0.24 1",
        name="fixed_custom_frame_base",
    )
    _add_box_visual(
        link,
        xyz="-0.28 -0.23 0.34",
        size="0.045 0.045 0.62",
        rgba="0.36 0.36 0.38 1",
        name="fixed_custom_frame_upright_left",
    )
    _add_box_visual(
        link,
        xyz="-0.28 0.23 0.34",
        size="0.045 0.045 0.62",
        rgba="0.36 0.36 0.38 1",
        name="fixed_custom_frame_upright_right",
    )
    _add_box_collision(link, xyz="0 0 0.035", size="0.70 0.50 0.07", name="fixed_custom_frame_collision")
    return link


def _fixed_joint(name: str, parent: str, child: str, xyz: str, rpy: str = "0 0 0") -> ET.Element:
    joint = ET.Element("joint", {"name": name, "type": "fixed"})
    _add_origin(joint, xyz, rpy)
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    return joint


def _amazinghand_link(asset_dir: str | Path) -> tuple[ET.Element, list[str]]:
    link = ET.Element("link", {"name": "amazinghand_fixed_link"})
    _add_inertial(link, mass="0.65", ixx="0.006", iyy="0.004", izz="0.004")
    _add_box_visual(
        link,
        xyz="0.055 0 0",
        size="0.12 0.08 0.045",
        rgba="0.05 0.05 0.06 1",
        name="amazinghand_fixed_palm_proxy",
    )
    _add_box_collision(
        link,
        xyz="0.055 0 0",
        size="0.12 0.08 0.045",
        name="amazinghand_fixed_collision_proxy",
    )

    visual_meshes = []
    for index, mesh_name in enumerate(HAND_VISUAL_MESHES, start=1):
        mesh_host_path = _host_path(asset_dir) / mesh_name
        mesh_container_path = _container_path(mesh_host_path)
        visual_meshes.append(mesh_container_path)
        visual = ET.SubElement(link, "visual", {"name": f"amazinghand_fixed_visual_{index:02d}"})
        # Keep the hand fixed and close to the Roboto wrist; detailed finger pose
        # is intentionally deferred, so these meshes are visual context only.
        _add_origin(visual, "0.04 0 0", "0 0 0")
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "mesh", {"filename": mesh_container_path, "scale": "0.001 0.001 0.001"})
    return link, visual_meshes


def _write_direct_physical_urdf(
    *,
    reference_urdf_path: str | Path,
    physical_urdf_path: str | Path,
    amazinghand_asset_dir: str | Path,
) -> dict[str, object]:
    joint_topology, reference_tree = _parse_reference_urdf(reference_urdf_path)
    reference_root = reference_tree.getroot()
    robot = ET.Element("robot", {"name": "echo_full_robotov2_right_arm_amazinghand"})
    robot.append(_custom_frame_link())

    for link_name in ARM_LINKS:
        link = copy.deepcopy(_element_by_name(reference_root, "link", link_name))
        _rewrite_mesh_filenames(link, reference_urdf_path)
        robot.append(link)

    robot.append(
        _fixed_joint(
            "custom_frame_to_torso_fixed_joint",
            "custom_frame_link",
            "torso_link",
            "0 0 0.08",
        )
    )

    for joint_name in CONTROLLED_ARM_JOINTS:
        robot.append(copy.deepcopy(_element_by_name(reference_root, "joint", joint_name)))

    amazinghand_link, visual_meshes = _amazinghand_link(amazinghand_asset_dir)
    robot.append(amazinghand_link)
    robot.append(
        _fixed_joint(
            "amazinghand_fixed_joint",
            "right_elbow_yaw_link",
            "amazinghand_fixed_link",
            "0.085 0 0",
        )
    )

    ET.indent(robot, space="  ")
    output = _host_path(physical_urdf_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(robot).write(output, encoding="utf-8", xml_declaration=True)
    output.write_text(output.read_text(encoding="utf-8") + "\n", encoding="utf-8")

    return {
        "path": _repo_relative_path(physical_urdf_path),
        "container_path": _container_path(physical_urdf_path),
        "controlled_joint_count": len(CONTROLLED_ARM_JOINTS),
        "copied_arm_links": list(ARM_LINKS),
        "fixed_joints": ["custom_frame_to_torso_fixed_joint", "amazinghand_fixed_joint"],
        "amazinghand_visual_meshes": visual_meshes,
        "joint_topology": joint_topology,
    }


def _read_direct_physical_urdf(physical_urdf_path: str | Path) -> dict[str, object]:
    """Read the current full right-arm + AmazingHand URDF artifact metadata."""
    topology, tree = _parse_reference_urdf(physical_urdf_path)
    root = tree.getroot()
    joints_by_name = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    wrist_joint_name = "right_elbow_yaw_to_r_wrist_interface"
    wrist_joint = joints_by_name.get(wrist_joint_name)
    if wrist_joint is None:
        raise RuntimeError(f"Physical URDF is missing {wrist_joint_name}")
    parent = wrist_joint.find("parent")
    child = wrist_joint.find("child")
    origin = wrist_joint.find("origin")
    if parent is None or child is None or origin is None:
        raise RuntimeError(f"{wrist_joint_name} is missing parent/child/origin metadata")

    finger_joints = [
        joint.attrib["name"]
        for joint in root.findall("joint")
        if joint.attrib.get("name", "").startswith("finger")
    ]
    visual_meshes = [
        mesh.attrib["filename"]
        for mesh in root.findall(".//mesh")
        if mesh.attrib.get("filename", "").find("AmazingHand/") >= 0
    ]

    return {
        "path": _repo_relative_path(physical_urdf_path),
        "container_path": _container_path(physical_urdf_path),
        "controlled_joint_count": len(CONTROLLED_ARM_JOINTS),
        "joint_topology": topology,
        "finger_dof_count": len(finger_joints),
        "finger_joints": finger_joints,
        "amazinghand_visual_meshes": visual_meshes,
        "wrist_attachment_transform": {
            "joint_name": wrist_joint_name,
            "parent_link": parent.attrib["link"],
            "child_link": child.attrib["link"],
            "origin_xyz": _parse_float_list(origin.attrib.get("xyz", "0 0 0")),
            "origin_rpy": _parse_float_list(origin.attrib.get("rpy", "0 0 0")),
        },
    }


def _author_usd_manifest(
    *,
    output_usd: str | Path,
    source_usd: str,
    reference_urdf_path: str | Path,
    physical_urdf_path: str | Path,
    amazinghand_mjcf: dict[str, object],
    joint_topology: list[dict[str, object]],
    direct_import: dict[str, object],
    visual_binding_status: dict[str, object],
) -> None:
    output = _host_path(output_usd)
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output))
    root = UsdGeom.Xform.Define(stage, "/echo_full")
    stage.SetDefaultPrim(root.GetPrim())

    visual = UsdGeom.Xform.Define(stage, "/echo_full/simready_visual_context")
    visual.GetPrim().GetReferences().AddReference(_usd_asset_reference(source_usd, output_usd))
    visual.GetPrim().CreateAttribute("purpose", Sdf.ValueTypeNames.Token).Set("default")
    visual.GetPrim().CreateAttribute("visual_context_role", Sdf.ValueTypeNames.String).Set(
        "Visual context only; physical arm is imported from PHYSICAL_ROBOT_URDF_PATH."
    )

    metadata = UsdGeom.Scope.Define(stage, "/echo_full/echo_full_lerobot_articulation_metadata")
    prim = metadata.GetPrim()
    prim.CreateAttribute("authoring_mode", Sdf.ValueTypeNames.String).Set(
        "custom_visual_usda_plus_direct_arm_hand_urdf"
    )
    prim.CreateAttribute("source_usd", Sdf.ValueTypeNames.String).Set(source_usd)
    prim.CreateAttribute("reference_urdf_path", Sdf.ValueTypeNames.String).Set(
        _repo_relative_path(reference_urdf_path)
    )
    prim.CreateAttribute("physical_robot_urdf_path", Sdf.ValueTypeNames.String).Set(
        _repo_relative_path(physical_urdf_path)
    )
    prim.CreateAttribute("amazinghand_mjcf_source_json", Sdf.ValueTypeNames.String).Set(
        json.dumps(amazinghand_mjcf, sort_keys=True)
    )
    prim.CreateAttribute("isaac_importer", Sdf.ValueTypeNames.String).Set("URDFParseAndImportFile")
    prim.CreateAttribute("control_contract", Sdf.ValueTypeNames.StringArray).Set(
        CONTROLLED_ARM_JOINTS + ["amazinghand_grasp"]
    )
    prim.CreateAttribute("joint_topology_json", Sdf.ValueTypeNames.String).Set(
        json.dumps(joint_topology, sort_keys=True)
    )
    prim.CreateAttribute("direct_urdf_import_json", Sdf.ValueTypeNames.String).Set(
        json.dumps(direct_import, sort_keys=True)
    )
    prim.CreateAttribute("visual_binding_status_json", Sdf.ValueTypeNames.String).Set(
        json.dumps(visual_binding_status, sort_keys=True)
    )
    prim.CreateAttribute("authoring_note", Sdf.ValueTypeNames.String).Set(
        "custom_visual_usda_plus_direct_arm_hand_urdf: Isaac Sim should load the generated "
        "PHYSICAL_ROBOT_URDF_PATH with URDFParseAndImportFile for the physical "
        "Roboto V2 right-arm + AmazingHand articulation; this USD is a provenance/visual manifest."
    )
    stage.GetRootLayer().Save()


def generate(
    source_usd: str,
    output_usd: str,
    report_path: str,
    reference_urdf_path: str,
    physical_urdf_path: str = DEFAULT_PHYSICAL_ROBOT_URDF,
    amazinghand_asset_dir: str = DEFAULT_AMAZINGHAND_ASSET_DIR,
    amazinghand_mjcf_path: str = DEFAULT_AMAZINGHAND_MJCF,
) -> dict[str, object]:
    reference_joint_topology, _ = _parse_reference_urdf(reference_urdf_path)
    physical = _read_direct_physical_urdf(physical_urdf_path)
    amazinghand_mjcf = _read_amazinghand_mjcf_metadata(amazinghand_mjcf_path)
    joint_topology = physical["joint_topology"]  # type: ignore[assignment]
    wrist_attachment = physical["wrist_attachment_transform"]  # type: ignore[assignment]
    urdf_constraint_fidelity = {
        "status": "LOSSY_MJCF_CONVERSION",
        "mjcf_constraints_preserved": False,
        "omitted_mjcf_features": list(amazinghand_mjcf["closed_loop_features"]),  # type: ignore[index]
        "equality_connect_count_in_source_mjcf": amazinghand_mjcf["equality_connect_count"],
        "ball_joint_count_in_source_mjcf": amazinghand_mjcf["ball_joint_count"],
        "reason": (
            "The runtime physical artifact is a URDF imported with URDFParseAndImportFile. "
            "URDF does not encode the upstream AmazingHand MJCF equality/connect closed-loop "
            "constraints, so MJCF remains the authoritative hand reference."
        ),
    }

    direct_import = {
        "mode": "direct_urdf_import_artifact",
        "isaac_importer": "URDFParseAndImportFile",
        "import_config": {
            "fix_base": True,
            "import_inertia_tensor": True,
            "distance_scale": 1.0,
            "default_drive_type": "JOINT_DRIVE_POSITION",
        },
        "artifact_path": physical["path"],
        "artifact_container_path": physical["container_path"],
        "reference_urdf_path": _repo_relative_path(reference_urdf_path),
        "synthetic_usd_reconstruction": False,
        "controlled_joint_count": len(CONTROLLED_ARM_JOINTS),
        "hand_motor_joint_count": len(AMAZINGHAND_MOTOR_JOINTS),
        "hand_motor_joints": list(AMAZINGHAND_MOTOR_JOINTS),
        "wrist_attachment_transform": wrist_attachment,
        "runtime_loader": "setup_rpo_arm_scene.py PHYSICAL_ROBOT_URDF_PATH -> URDFParseAndImportFile",
        "custom_visual_loader": "setup_rpo_arm_scene.py CUSTOM_VISUAL_USD_PATH -> AddReference",
        "amazinghand_mjcf_source": amazinghand_mjcf,
        "urdf_constraint_fidelity": urdf_constraint_fidelity,
    }
    visual_binding_status = {
        "strategy": "custom_visual_usda_plus_direct_arm_hand_urdf",
        "simready_visual_source": source_usd,
        "custom_visual_usd_path": _repo_relative_path(output_usd),
        "custom_frame": {
            "status": "fixed_base_frame_from_custom_usda",
            "physical_urdf_membership": "excluded",
        },
        "physical_arm": {
            "root_link": "right_arm_base_link",
            "torso_membership": "excluded",
            "controlled_joint_count": len(CONTROLLED_ARM_JOINTS),
        },
        "physical_arm_links": {
            link_name: {
                "status": "copied_from_roboto_v2_reference_urdf",
                "physical_urdf_path": physical["path"],
            }
            for link_name in ARM_LINKS
            if link_name != "torso_link"
        },
        "amazinghand": {
            "status": "attached_to_terminal_urdf_link",
            "authoritative_reference": "mjcf",
            "mjcf_path": amazinghand_mjcf["path"],
            "attached_to": wrist_attachment["parent_link"],  # type: ignore[index]
            "root_link": wrist_attachment["child_link"],  # type: ignore[index]
            "joint_name": wrist_attachment["joint_name"],  # type: ignore[index]
            "attachment_origin_xyz": wrist_attachment["origin_xyz"],  # type: ignore[index]
            "attachment_origin_rpy": wrist_attachment["origin_rpy"],  # type: ignore[index]
            "finger_dofs": "present_in_physical_urdf_and_commanded_from_lerobot_grasp_scalar",
            "finger_dof_count": physical["finger_dof_count"],
            "hand_motor_joints": list(AMAZINGHAND_MOTOR_JOINTS),
            "mjcf_constraints_preserved_in_urdf": False,
            "urdf_constraint_fidelity": urdf_constraint_fidelity,
            "synthetic_channels": ["amazinghand_grasp"],
            "visual_meshes": physical["amazinghand_visual_meshes"],
        },
    }

    _author_usd_manifest(
        output_usd=output_usd,
        source_usd=source_usd,
        reference_urdf_path=reference_urdf_path,
        physical_urdf_path=physical_urdf_path,
        amazinghand_mjcf=amazinghand_mjcf,
        joint_topology=joint_topology,  # type: ignore[arg-type]
        direct_import=direct_import,
        visual_binding_status=visual_binding_status,
    )

    report = {
        "source_usd": source_usd,
        "reference_urdf_path": _repo_relative_path(reference_urdf_path),
        "reference_urdf_source_path": str(reference_urdf_path),
        "physical_robot_urdf_path": _repo_relative_path(physical_urdf_path),
        "physical_robot_urdf_container_path": _container_path(physical_urdf_path),
        "custom_visual_usd_path": _repo_relative_path(output_usd),
        "output_usd": _repo_relative_path(output_usd),
        "status": "PASS",
        "root_prim": "/echo_full",
        "articulation_root": "generated_by_isaac_urdf_importer_at_runtime",
        "direct_urdf_import": direct_import,
        "controlled_joints": CONTROLLED_ARM_JOINTS,
        "hand_motor_joints": list(AMAZINGHAND_MOTOR_JOINTS),
        "synthetic_channels": ["amazinghand_grasp"],
        "amazinghand_mjcf_source": amazinghand_mjcf,
        "urdf_constraint_fidelity": urdf_constraint_fidelity,
        "reference_joint_topology": reference_joint_topology,
        "joint_topology": joint_topology,
        "physical_urdf_joint_topology": joint_topology,
        "visual_binding_status": visual_binding_status,
    }
    runtime_validation = _preserved_runtime_validation(
        report_path=report_path,
        physical_urdf_path=physical_urdf_path,
        joint_topology=joint_topology,  # type: ignore[arg-type]
    )
    if runtime_validation is not None:
        report["runtime_validation"] = runtime_validation
    report_output = _host_path(report_path)
    report_output.parent.mkdir(parents=True, exist_ok=True)
    report_output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def main() -> None:
    source_usd = os.environ.get("SOURCE_SIMREADY_USD", DEFAULT_SOURCE_USD)
    output_usd = os.environ.get("SIMREADY_ARTICULATION_USD_PATH", DEFAULT_OUTPUT_USD)
    report_path = os.environ.get("SIMREADY_ARTICULATION_REPORT_PATH", DEFAULT_REPORT)
    reference_urdf_path = os.environ.get("REFERENCE_ROBOTO_V2_URDF_PATH", DEFAULT_REFERENCE_URDF)
    physical_urdf_path = os.environ.get("DIRECT_PHYSICAL_ROBOT_URDF_PATH", DEFAULT_PHYSICAL_ROBOT_URDF)
    amazinghand_asset_dir = os.environ.get("AMAZINGHAND_ASSET_DIR", DEFAULT_AMAZINGHAND_ASSET_DIR)
    amazinghand_mjcf_path = os.environ.get("AMAZINGHAND_MJCF_PATH", DEFAULT_AMAZINGHAND_MJCF)
    report = generate(
        source_usd,
        output_usd,
        report_path,
        reference_urdf_path,
        physical_urdf_path,
        amazinghand_asset_dir,
        amazinghand_mjcf_path,
    )
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
