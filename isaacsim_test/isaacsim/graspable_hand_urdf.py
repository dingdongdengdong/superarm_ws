"""Generate an Isaac-friendly tree-articulation AmazingHand URDF.

The upstream AmazingHand MJCF uses closed-loop equality/connect constraints.
Those constraints are useful for MuJoCo but brittle in Isaac's MJCF importer.
This module builds a conservative open-chain hand for Isaac: original STL files
remain visual assets, while primitive collision geometry carries contact.
"""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


HAND_ACTUATED_JOINT_NAMES = [
    "finger1_motor1",
    "finger1_motor2",
    "finger2_motor1",
    "finger2_motor2",
    "finger3_motor1",
    "finger3_motor2",
    "finger4_motor1",
    "finger4_motor2",
]

_FINGER_LAYOUTS = {
    1: {
        "role": "index",
        "base_xyz": (-0.030, -0.030, 0.020),
        "axis": (1.0, 0.0, 0.0),
    },
    2: {
        "role": "middle",
        "base_xyz": (0.0, -0.033, 0.020),
        "axis": (1.0, 0.0, 0.0),
    },
    3: {
        "role": "ring",
        "base_xyz": (0.030, -0.030, 0.020),
        "axis": (1.0, 0.0, 0.0),
    },
    4: {
        "role": "thumb",
        "base_xyz": (0.044, -0.006, -0.006),
        "axis": (0.0, 0.0, 1.0),
    },
}

_VISUAL_MESH_FILES = [
    "bushing_0608_04.stl",
    "r_wrist_interface.stl",
    "r_hand_plate.stl",
    "finger_frame_1.stl",
    "finger_frame_2.stl",
    "scs0009.stl",
    "proximal.stl",
    "proximal_shell.stl",
    "custom_servo_horn.stl",
    "distal.stl",
    "distal_shell.stl",
    "gimbal.stl",
    "link.stl",
    "m2_rod_l18.stl",
    "parallel_pin_2_x_10__fee063fca0c8b40e46bbc4ffff61d999.stl",
    "parallel_pin_2_x_16__da4b7ddbe9d803fe3fbc70f2e822b99b.stl",
    "ph_pan_head_screw_m2x0_40_x_10__2803432263e518bbd16bccbbef8784ed.stl",
    "plain_washer_large_grade_a_m2_5__9a369f0dc77bf9c598cdf3fb468977e5.stl",
    "rotule_ball.stl",
    "rotule_lever.stl",
    "spacer.stl",
    "std00333_plast_tcb_torx_2_5x8__configuration_copy_of_default.stl",
    "std00447_thermoplastique_m2_5x6__configuration_default.stl",
]
_DISTAL_VISUAL_MESHES = {
    "distal",
    "distal_shell",
    "parallel_pin_2_x_10__fee063fca0c8b40e46bbc4ffff61d999",
}
_PROXIMAL_VISUAL_MESHES = {
    "proximal",
    "proximal_shell",
    "parallel_pin_2_x_16__da4b7ddbe9d803fe3fbc70f2e822b99b",
}
_MJCF_ACTUATED_BODY_TO_LINK_PREFIX = {
    "custom_servo_horn": "finger1",
    "rotule_ball_2": "finger1",
    "custom_servo_horn_2": "finger2",
    "rotule_ball_4": "finger2",
    "custom_servo_horn_3": "finger3",
    "rotule_ball_6": "finger3",
    "custom_servo_horn_4": "finger4",
    "rotule_ball_8": "finger4",
}


def _format_float(value: float) -> str:
    if abs(value) < 1e-12:
        value = 0.0
    return f"{value:.8g}"


def _format_floats(values: tuple[float, float, float]) -> str:
    return " ".join(_format_float(value) for value in values)


def _parse_vec3(raw: str | None, default: tuple[float, float, float]) -> tuple[float, float, float]:
    if not raw:
        return default
    values = [float(item) for item in raw.split()]
    if len(values) != 3:
        raise ValueError(f"Expected 3 floats, got {raw!r}")
    return values[0], values[1], values[2]


def _parse_quat(raw: str | None) -> tuple[float, float, float, float]:
    if not raw:
        return 1.0, 0.0, 0.0, 0.0
    values = [float(item) for item in raw.split()]
    if len(values) != 4:
        raise ValueError(f"Expected MJCF wxyz quaternion, got {raw!r}")
    return _normalize_quat((values[0], values[1], values[2], values[3]))


def _normalize_quat(quat: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in quat))
    if norm <= 1e-12:
        return 1.0, 0.0, 0.0, 0.0
    return tuple(value / norm for value in quat)  # type: ignore[return-value]


def _quat_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return _normalize_quat(
        (
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        )
    )


def _quat_rotate(
    quat: tuple[float, float, float, float],
    vec: tuple[float, float, float],
) -> tuple[float, float, float]:
    w, x, y, z = quat
    vx, vy, vz = vec
    # q * v * q^-1, expanded to avoid temporary quaternion allocations.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def _compose_transform(
    parent_xyz: tuple[float, float, float],
    parent_quat: tuple[float, float, float, float],
    local_xyz: tuple[float, float, float],
    local_quat: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    rotated = _quat_rotate(parent_quat, local_xyz)
    return (
        (
            parent_xyz[0] + rotated[0],
            parent_xyz[1] + rotated[1],
            parent_xyz[2] + rotated[2],
        ),
        _quat_multiply(parent_quat, local_quat),
    )


def _quat_to_rpy(quat: tuple[float, float, float, float]) -> tuple[float, float, float]:
    w, x, y, z = _normalize_quat(quat)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _add_origin(
    parent: ET.Element,
    *,
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    ET.SubElement(parent, "origin", {"xyz": _format_floats(xyz), "rpy": _format_floats(rpy)})


def _add_inertial(
    link: ET.Element,
    *,
    mass: float,
    inertia: tuple[float, float, float],
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    inertial = ET.SubElement(link, "inertial")
    _add_origin(inertial, xyz=xyz)
    ET.SubElement(inertial, "mass", {"value": _format_float(mass)})
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": _format_float(inertia[0]),
            "ixy": "0",
            "ixz": "0",
            "iyy": _format_float(inertia[1]),
            "iyz": "0",
            "izz": _format_float(inertia[2]),
        },
    )


def _add_visual(
    link: ET.Element,
    *,
    name: str,
    mesh_path: Path,
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    visual = ET.SubElement(link, "visual", {"name": name})
    _add_origin(visual, xyz=xyz, rpy=rpy)
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "mesh", {"filename": str(mesh_path.resolve())})


def _collect_mjcf_visual_geoms(package_root: Path) -> list[dict[str, Any]]:
    mjcf_path = package_root / "hand_mjcf" / "robot.xml"
    asset_root = package_root / "hand_mjcf" / "assets"
    root = ET.parse(mjcf_path).getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        return []

    visuals: list[dict[str, Any]] = []

    def walk_body(
        body: ET.Element,
        parent_xyz: tuple[float, float, float],
        parent_quat: tuple[float, float, float, float],
        parent_chain: tuple[str, ...],
    ) -> None:
        body_xyz, body_quat = _compose_transform(
            parent_xyz,
            parent_quat,
            _parse_vec3(body.attrib.get("pos"), (0.0, 0.0, 0.0)),
            _parse_quat(body.attrib.get("quat")),
        )
        body_name = body.attrib.get("name", "body")
        body_chain = (*parent_chain, body_name)
        for geom_index, geom in enumerate(body.findall("geom")):
            mesh_name = geom.attrib.get("mesh")
            if not mesh_name:
                continue
            mesh_path = asset_root / f"{mesh_name}.stl"
            geom_xyz, geom_quat = _compose_transform(
                body_xyz,
                body_quat,
                _parse_vec3(geom.attrib.get("pos"), (0.0, 0.0, 0.0)),
                _parse_quat(geom.attrib.get("quat")),
            )
            visuals.append(
                {
                    "name": f"mjcf_{len(visuals):03d}_{body_name}_{mesh_name}_{geom_index}",
                    "body_name": body_name,
                    "body_chain": body_chain,
                    "mesh_name": mesh_name,
                    "mesh_path": mesh_path,
                    "xyz": geom_xyz,
                    "rpy": _quat_to_rpy(geom_quat),
                }
            )
        for child in body.findall("body"):
            walk_body(child, body_xyz, body_quat, body_chain)

    for body in worldbody.findall("body"):
        walk_body(body, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), ())
    return visuals


def _finger_link_initial_xyz() -> dict[str, tuple[float, float, float]]:
    link_xyz = {
        "r_wrist_interface": (0.0, 0.0, 0.0),
        "palm": (0.0, 0.0, 0.0),
    }
    for finger_index, layout in sorted(_FINGER_LAYOUTS.items()):
        base_xyz = layout["base_xyz"]
        link_xyz[f"finger{finger_index}_proximal"] = base_xyz
        link_xyz[f"finger{finger_index}_distal"] = (
            base_xyz[0],
            base_xyz[1] + 0.058,
            base_xyz[2],
        )
    return link_xyz


def _classify_mjcf_visual_link(visual: dict[str, Any]) -> str:
    mesh_name = visual["mesh_name"]
    body_chain = visual["body_chain"]
    finger_prefix = None
    for body_name in body_chain:
        if body_name in _MJCF_ACTUATED_BODY_TO_LINK_PREFIX:
            finger_prefix = _MJCF_ACTUATED_BODY_TO_LINK_PREFIX[body_name]
            break
    if finger_prefix is None:
        return "r_wrist_interface"
    if mesh_name in _DISTAL_VISUAL_MESHES:
        return f"{finger_prefix}_distal"
    if mesh_name in _PROXIMAL_VISUAL_MESHES:
        return f"{finger_prefix}_proximal"
    return f"{finger_prefix}_proximal"


def _add_mjcf_visuals_to_tree_links(
    links: dict[str, ET.Element],
    *,
    package_root: Path,
) -> dict[str, Any]:
    visual_geoms = _collect_mjcf_visual_geoms(package_root)
    link_initial_xyz = _finger_link_initial_xyz()
    link_visual_counts = {link_name: 0 for link_name in links}
    missing_meshes: list[str] = []
    for visual in visual_geoms:
        mesh_path = Path(visual["mesh_path"])
        if not mesh_path.is_file():
            missing_meshes.append(str(mesh_path))
            continue
        link_name = _classify_mjcf_visual_link(visual)
        link_origin = link_initial_xyz[link_name]
        local_xyz = (
            visual["xyz"][0] - link_origin[0],
            visual["xyz"][1] - link_origin[1],
            visual["xyz"][2] - link_origin[2],
        )
        _add_visual(
            links[link_name],
            name=visual["name"],
            mesh_path=mesh_path,
            xyz=local_xyz,
            rpy=visual["rpy"],
        )
        link_visual_counts[link_name] += 1
    return {
        "visual_attachment_mode": "mjcf_visuals_partitioned_to_tree_links",
        "mjcf_visual_geom_count": len(visual_geoms),
        "missing_mjcf_visual_meshes": missing_meshes,
        "link_visual_counts": {
            link_name: count
            for link_name, count in sorted(link_visual_counts.items())
            if count
        },
    }


def _add_box_collision(
    link: ET.Element,
    *,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
) -> None:
    collision = ET.SubElement(link, "collision", {"name": name})
    _add_origin(collision, xyz=xyz)
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, "box", {"size": _format_floats(size)})


def _add_fixed_joint(
    robot: ET.Element,
    *,
    name: str,
    parent: str,
    child: str,
    xyz: tuple[float, float, float],
) -> None:
    joint = ET.SubElement(robot, "joint", {"name": name, "type": "fixed"})
    _add_origin(joint, xyz=xyz)
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})


def _add_revolute_joint(
    robot: ET.Element,
    *,
    name: str,
    parent: str,
    child: str,
    xyz: tuple[float, float, float],
    axis: tuple[float, float, float],
    limit: tuple[float, float],
) -> None:
    joint = ET.SubElement(robot, "joint", {"name": name, "type": "revolute"})
    _add_origin(joint, xyz=xyz)
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    ET.SubElement(joint, "axis", {"xyz": _format_floats(axis)})
    ET.SubElement(
        joint,
        "limit",
        {
            "lower": _format_float(limit[0]),
            "upper": _format_float(limit[1]),
            "effort": "2.0",
            "velocity": "3.0",
        },
    )
    ET.SubElement(joint, "dynamics", {"damping": "0.08", "friction": "0.02"})


def _add_link(
    robot: ET.Element,
    name: str,
    *,
    mass: float,
    inertia: tuple[float, float, float],
) -> ET.Element:
    link = ET.SubElement(robot, "link", {"name": name})
    _add_inertial(link, mass=mass, inertia=inertia)
    return link


def build_graspable_hand_model_spec() -> dict[str, Any]:
    """Return the deterministic tree-hand contract used by the URDF generator."""
    return {
        "robot_name": "amazinghand_graspable",
        "root_link": "r_wrist_interface",
        "finger_count": 4,
        "finger_roles": {
            f"finger{index}": layout["role"]
            for index, layout in sorted(_FINGER_LAYOUTS.items())
        },
        "excluded_human_finger": "pinky",
        "actuated_joint_names": list(HAND_ACTUATED_JOINT_NAMES),
        "visual_mesh_files": list(_VISUAL_MESH_FILES),
        "collision_primitive_count": 13,
        "equality_constraint_count": 0,
        "notes": [
            "STL files are used only as visual geometry.",
            "Primitive box collisions are used for stable Isaac contact.",
            "The model is a tree articulation with four two-joint fingers.",
        ],
    }


def grasp_scalar_to_hand_joint_targets(grasp: float) -> dict[str, float]:
    """Map a normalized grasp command to the eight Isaac-friendly hand joints."""
    closedness = max(0.0, min(1.0, float(grasp)))
    targets: dict[str, float] = {}
    for finger_index in range(1, 5):
        targets[f"finger{finger_index}_motor1"] = 0.05 + closedness * 0.90
        targets[f"finger{finger_index}_motor2"] = 0.02 + closedness * 1.08
    return targets


def _mesh(asset_root: Path, filename: str) -> Path:
    return asset_root / filename


def generate_graspable_hand_urdf(
    package_root: str | Path,
    output_urdf: str | Path,
    *,
    robot_name: str = "amazinghand_graspable",
) -> dict[str, Any]:
    """Generate a simplified tree hand URDF that Isaac can import as an articulation."""
    package = Path(package_root)
    output = Path(output_urdf)
    asset_root = package / "hand_mjcf" / "assets"
    if not asset_root.is_dir():
        raise FileNotFoundError(f"Hand asset directory not found: {asset_root}")

    spec = build_graspable_hand_model_spec()
    missing_meshes = [
        str(_mesh(asset_root, filename))
        for filename in spec["visual_mesh_files"]
        if not _mesh(asset_root, filename).is_file()
    ]

    robot = ET.Element("robot", {"name": robot_name})
    robot.append(
        ET.Comment(
            "Isaac tree articulation generated from AmazingHand visual assets; "
            "closed-loop MJCF constraints are intentionally not used."
        )
    )

    wrist = _add_link(
        robot,
        "r_wrist_interface",
        mass=0.055,
        inertia=(2.0e-5, 2.0e-5, 2.0e-5),
    )
    links = {"r_wrist_interface": wrist}

    palm = _add_link(
        robot,
        "palm",
        mass=0.14,
        inertia=(8.0e-5, 8.0e-5, 1.0e-4),
    )
    links["palm"] = palm
    _add_box_collision(
        palm,
        name="palm_contact_box",
        size=(0.09, 0.085, 0.035),
        xyz=(0.0, -0.038, 0.0),
    )
    _add_fixed_joint(
        robot,
        name="wrist_to_palm",
        parent="r_wrist_interface",
        child="palm",
        xyz=(0.0, 0.0, 0.0),
    )

    for finger_index in range(1, 5):
        proximal_name = f"finger{finger_index}_proximal"
        distal_name = f"finger{finger_index}_distal"
        layout = _FINGER_LAYOUTS[finger_index]
        proximal = _add_link(
            robot,
            proximal_name,
            mass=0.028,
            inertia=(1.1e-5, 1.1e-5, 3.0e-6),
        )
        links[proximal_name] = proximal
        _add_box_collision(
            proximal,
            name=f"{proximal_name}_contact_box",
            size=(0.018, 0.058, 0.018),
            xyz=(0.0, 0.029, 0.0),
        )

        distal = _add_link(
            robot,
            distal_name,
            mass=0.022,
            inertia=(8.0e-6, 8.0e-6, 2.0e-6),
        )
        links[distal_name] = distal
        _add_box_collision(
            distal,
            name=f"{distal_name}_contact_box",
            size=(0.016, 0.050, 0.016),
            xyz=(0.0, 0.025, 0.0),
        )
        _add_box_collision(
            distal,
            name=f"{distal_name}_tip_pad_contact_box",
            size=(0.026, 0.014, 0.022),
            xyz=(0.0, 0.055, 0.0),
        )

        _add_revolute_joint(
            robot,
            name=f"finger{finger_index}_motor1",
            parent="palm",
            child=proximal_name,
            xyz=layout["base_xyz"],
            axis=layout["axis"],
            limit=(-0.05, 1.05),
        )
        _add_revolute_joint(
            robot,
            name=f"finger{finger_index}_motor2",
            parent=proximal_name,
            child=distal_name,
            xyz=(0.0, 0.058, 0.0),
            axis=layout["axis"],
            limit=(0.0, 1.2),
        )

    visual_shell_report = _add_mjcf_visuals_to_tree_links(links, package_root=package)

    ET.indent(robot, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(robot).write(output, encoding="utf-8", xml_declaration=True)
    output.write_text(output.read_text(encoding="utf-8") + "\n", encoding="utf-8")

    link_count = len(robot.findall("link"))
    joints = robot.findall("joint")
    collisions = robot.findall(".//collision")
    report = {
        "status": "PASS" if not missing_meshes else "FAIL",
        "package_root": package.as_posix(),
        "output_urdf": output.as_posix(),
        "robot_name": robot_name,
        "root_link": "r_wrist_interface",
        "link_count": link_count,
        "joint_count": len(joints),
        "actuated_joint_names": list(HAND_ACTUATED_JOINT_NAMES),
        "visual_mesh_files": list(spec["visual_mesh_files"]),
        "missing_visual_meshes": missing_meshes + visual_shell_report["missing_mjcf_visual_meshes"],
        "mjcf_visual_shell": visual_shell_report,
        "collision_primitive_count": len(collisions),
        "equality_constraint_count": 0,
    }
    (output.parent / "amazinghand_graspable_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return report
