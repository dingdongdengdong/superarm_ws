"""Generate an Isaac-friendly tree-articulation AmazingHand URDF.

The upstream AmazingHand MJCF uses closed-loop equality/connect constraints.
Those constraints are useful for MuJoCo but brittle in Isaac's MJCF importer.
This module builds a conservative open-chain hand for Isaac: original STL files
remain visual assets, while primitive collision geometry carries contact.
"""

from __future__ import annotations

import json
import math
import struct
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
HAND_GRASP_TYPES = ("wrap", "pinch", "wide")
AMAZINGHAND_SERVO_MODEL = "SCS0009"
AMAZINGHAND_DEFAULT_SPEED = 6

VISUAL_MODE_STATIC_SHELL = "static_shell"
VISUAL_MODE_PARTITIONED_LINKS = "partitioned_links"
VISUAL_MODE_IMPLEMENTED_ONLY = "implemented_only"

_FINGER_LAYOUTS = {
    1: {
        "role": "index",
        "base_xyz": (-0.00505, 0.03055, 0.06980),
        "mjcf_anchor_body": "custom_servo_horn",
        "mjcf_proximal_xyz": (-0.01150, 0.02475, 0.10220),
        "axis": (1.0, 0.0, 0.0),
    },
    2: {
        "role": "middle",
        "base_xyz": (-0.00505, 0.00110, 0.06456),
        "mjcf_anchor_body": "custom_servo_horn_2",
        "mjcf_proximal_xyz": (-0.01150, -0.00805, 0.09618),
        "axis": (1.0, 0.0, 0.0),
    },
    3: {
        "role": "ring",
        "base_xyz": (-0.00505, -0.02705, 0.05505),
        "mjcf_anchor_body": "custom_servo_horn_3",
        "mjcf_proximal_xyz": (-0.01150, -0.04026, 0.08520),
        "axis": (1.0, 0.0, 0.0),
    },
    4: {
        "role": "thumb",
        "base_xyz": (-0.00030, 0.00773, 0.03615),
        "mjcf_anchor_body": "custom_servo_horn_4",
        "mjcf_proximal_xyz": (0.02816, 0.02426, 0.02970),
        "axis": (0.0, 0.0, 1.0),
    },
}
_AMAZINGHAND_MOTOR_CONFIG = {
    "finger1_motor1": {
        "finger_index": 1,
        "motor_index": 1,
        "servo_id": 1,
        "offset_rad": math.radians(7.0),
        "invert": False,
    },
    "finger1_motor2": {
        "finger_index": 1,
        "motor_index": 2,
        "servo_id": 2,
        "offset_rad": math.radians(5.0),
        "invert": False,
    },
    "finger2_motor1": {
        "finger_index": 2,
        "motor_index": 1,
        "servo_id": 3,
        "offset_rad": 0.0,
        "invert": False,
    },
    "finger2_motor2": {
        "finger_index": 2,
        "motor_index": 2,
        "servo_id": 4,
        "offset_rad": math.radians(7.0),
        "invert": False,
    },
    "finger3_motor1": {
        "finger_index": 3,
        "motor_index": 1,
        "servo_id": 5,
        "offset_rad": math.radians(5.0),
        "invert": False,
    },
    "finger3_motor2": {
        "finger_index": 3,
        "motor_index": 2,
        "servo_id": 6,
        "offset_rad": math.radians(7.0),
        "invert": False,
    },
    "finger4_motor1": {
        "finger_index": 4,
        "motor_index": 1,
        "servo_id": 7,
        "offset_rad": 0.0,
        "invert": False,
    },
    "finger4_motor2": {
        "finger_index": 4,
        "motor_index": 2,
        "servo_id": 8,
        "offset_rad": math.radians(7.0),
        "invert": False,
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
}
_PROXIMAL_VISUAL_MESHES = {
    "proximal",
    "proximal_shell",
}
_MAJOR_PROXIMAL_FOLLOWER_MESHES = {
    "custom_servo_horn",
    "gimbal",
    "m2_rod_l18",
    "rotule_ball",
    "rotule_lever",
    "parallel_pin_2_x_16__da4b7ddbe9d803fe3fbc70f2e822b99b",
}
_MAJOR_DISTAL_FOLLOWER_MESHES = {
    "link",
}
_MAJOR_PARALLEL_PIN_10_MESH = "parallel_pin_2_x_10__fee063fca0c8b40e46bbc4ffff61d999"
_SKELETON_FIRST_FOLLOWER_MESHES = (
    _MAJOR_PROXIMAL_FOLLOWER_MESHES
    | _MAJOR_DISTAL_FOLLOWER_MESHES
    | {_MAJOR_PARALLEL_PIN_10_MESH}
)
_OMITTED_SHELL_VISUAL_MESHES = _PROXIMAL_VISUAL_MESHES | _DISTAL_VISUAL_MESHES
_OMITTED_DETAIL_VISUAL_MESHES = {
    "ph_pan_head_screw_m2x0_40_x_10__2803432263e518bbd16bccbbef8784ed",
    "plain_washer_large_grade_a_m2_5__9a369f0dc77bf9c598cdf3fb468977e5",
    "spacer",
    "std00333_plast_tcb_torx_2_5x8__configuration_copy_of_default",
    "std00447_thermoplastique_m2_5x6__configuration_default",
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


def _add_box_visual(
    link: ET.Element,
    *,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    color: tuple[float, float, float, float],
) -> None:
    visual = ET.SubElement(link, "visual", {"name": name})
    _add_origin(visual, xyz=xyz)
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "box", {"size": _format_floats(size)})
    material = ET.SubElement(visual, "material", {"name": f"{name}_material"})
    ET.SubElement(material, "color", {"rgba": " ".join(_format_float(value) for value in color)})


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
        body_mesh_names = tuple(
            geom.attrib["mesh"] for geom in body.findall("geom") if geom.attrib.get("mesh")
        )
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
                    "body_mesh_names": body_mesh_names,
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


def _stl_bounds(mesh_path: Path) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    data = mesh_path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"STL file is too small: {mesh_path}")
    triangle_count = struct.unpack("<I", data[80:84])[0]
    expected_size = 84 + triangle_count * 50
    if len(data) < expected_size:
        raise ValueError(f"Binary STL is truncated: {mesh_path}")
    mins = [math.inf, math.inf, math.inf]
    maxs = [-math.inf, -math.inf, -math.inf]
    offset = 84
    for _ in range(triangle_count):
        values = struct.unpack("<12fH", data[offset : offset + 50])
        coords = values[3:12]
        for coord_index in range(0, 9, 3):
            for axis in range(3):
                value = float(coords[coord_index + axis])
                mins[axis] = min(mins[axis], value)
                maxs[axis] = max(maxs[axis], value)
        offset += 50
    return (mins[0], mins[1], mins[2]), (maxs[0], maxs[1], maxs[2])


def _tree_aligned_segment_visual_origin(
    mesh_path: Path,
) -> tuple[float, float, float]:
    """Return a conservative link-local placement for generated finger visuals.

    The MJCF hand is a closed-loop linkage, but the generated Isaac hand is a
    simple two-link tree. Using the MJCF world transform on a simplified tree
    link rotates the STL around the wrong pivot and makes finger parts appear
    detached/floating. The proximal/distal STL files already use finger-local
    coordinates, so align their mesh bounds to the generated link frame instead.
    """
    bbox_min, bbox_max = _stl_bounds(mesh_path)
    center_x = (bbox_min[0] + bbox_max[0]) * 0.5
    center_z = (bbox_min[2] + bbox_max[2]) * 0.5
    return (-center_x, -bbox_min[1], -center_z)


def _finger_link_initial_xyz() -> dict[str, tuple[float, float, float]]:
    link_xyz = {
        "r_wrist_interface": (0.0, 0.0, 0.0),
        "palm": (0.0, 0.0, 0.0),
    }
    for finger_index, layout in sorted(_FINGER_LAYOUTS.items()):
        base_xyz = layout["base_xyz"]
        link_xyz[f"finger{finger_index}_base"] = base_xyz
        link_xyz[f"finger{finger_index}_proximal"] = base_xyz
        link_xyz[f"finger{finger_index}_distal"] = (
            base_xyz[0],
            base_xyz[1] + 0.058,
            base_xyz[2],
        )
    return link_xyz


def _body_chain_contains_prefix(body_chain: tuple[str, ...], prefix: str) -> bool:
    return any(body_name == prefix or body_name.startswith(f"{prefix}_") for body_name in body_chain)


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
    if mesh_name in _MAJOR_DISTAL_FOLLOWER_MESHES:
        return f"{finger_prefix}_distal"
    if mesh_name == _MAJOR_PARALLEL_PIN_10_MESH:
        if _body_chain_contains_prefix(body_chain, _MAJOR_PARALLEL_PIN_10_MESH):
            return f"{finger_prefix}_distal"
        return f"{finger_prefix}_proximal"
    if mesh_name in _MAJOR_PROXIMAL_FOLLOWER_MESHES:
        return f"{finger_prefix}_proximal"
    # Small screws, washers, spacers, bushings and exact closed-loop detail are
    # intentionally outside the skeleton-first pass. Keep them on the wrist so
    # they do not imply a verified moving linkage.
    return "r_wrist_interface"


def _raw_link_local_visual_origin(
    visual: dict[str, Any],
    link_initial_xyz: dict[str, tuple[float, float, float]],
    link_name: str,
) -> tuple[float, float, float]:
    link_origin = link_initial_xyz[link_name]
    return (
        visual["xyz"][0] - link_origin[0],
        visual["xyz"][1] - link_origin[1],
        visual["xyz"][2] - link_origin[2],
    )


def _moving_link_visual_alignment_offsets(
    visual_geoms: list[dict[str, Any]],
    link_initial_xyz: dict[str, tuple[float, float, float]],
) -> tuple[dict[str, tuple[float, float, float]], dict[str, str]]:
    """Return per-generated-link corrections from MJCF world space to tree space.

    The generated Isaac hand has simple two-joint links whose local frames do
    not match the original closed-loop MJCF body graph. Proximal/distal segment
    meshes are the best available anchors because their STL bounds already fit
    the generated tree links. Apply the same anchor correction to nearby major
    linkage/pin visuals so they move with the finger without floating far above
    the simplified segment.
    """
    offsets: dict[str, tuple[float, float, float]] = {}
    anchor_meshes: dict[str, str] = {}
    for visual in visual_geoms:
        mesh_name = visual["mesh_name"]
        if mesh_name not in {"proximal", "distal"}:
            continue
        link_name = _classify_mjcf_visual_link(visual)
        if not (link_name.endswith("_proximal") or link_name.endswith("_distal")):
            continue
        mesh_path = Path(visual["mesh_path"])
        if not mesh_path.is_file():
            continue
        raw_local = _raw_link_local_visual_origin(visual, link_initial_xyz, link_name)
        tree_local = _tree_aligned_segment_visual_origin(mesh_path)
        offsets[link_name] = (
            tree_local[0] - raw_local[0],
            tree_local[1] - raw_local[1],
            tree_local[2] - raw_local[2],
        )
        anchor_meshes[link_name] = mesh_name
    return offsets, anchor_meshes


def _add_mjcf_visuals_to_tree_links(
    links: dict[str, ET.Element],
    *,
    package_root: Path,
    include_finger_shells: bool = False,
) -> dict[str, Any]:
    visual_geoms = _collect_mjcf_visual_geoms(package_root)
    link_initial_xyz = _finger_link_initial_xyz()
    link_alignment_offsets, link_alignment_anchor_meshes = _moving_link_visual_alignment_offsets(
        visual_geoms,
        link_initial_xyz,
    )
    link_visual_counts = {link_name: 0 for link_name in links}
    moving_skeleton_visual_counts = {link_name: 0 for link_name in links}
    wrist_fixed_major_visuals: list[str] = []
    omitted_shell_visuals: list[str] = []
    moving_shell_visual_counts = {link_name: 0 for link_name in links}
    omitted_detail_visuals: list[str] = []
    missing_meshes: list[str] = []
    for visual in visual_geoms:
        if (
            visual["mesh_name"] in _OMITTED_SHELL_VISUAL_MESHES
            and not include_finger_shells
        ):
            omitted_shell_visuals.append(visual["name"])
            continue
        if visual["mesh_name"] in _OMITTED_DETAIL_VISUAL_MESHES:
            omitted_detail_visuals.append(visual["name"])
            continue
        mesh_path = Path(visual["mesh_path"])
        if not mesh_path.is_file():
            missing_meshes.append(str(mesh_path))
            continue
        link_name = _classify_mjcf_visual_link(visual)
        if (
            link_name.endswith("_proximal")
            or link_name.endswith("_distal")
        ) and visual["mesh_name"] in (_PROXIMAL_VISUAL_MESHES | _DISTAL_VISUAL_MESHES):
            local_xyz = _tree_aligned_segment_visual_origin(mesh_path)
            local_rpy = (0.0, 0.0, 0.0)
        elif (
            (link_name.endswith("_proximal") or link_name.endswith("_distal"))
            and visual["mesh_name"] in _SKELETON_FIRST_FOLLOWER_MESHES
            and link_name in link_alignment_offsets
        ):
            raw_local = _raw_link_local_visual_origin(visual, link_initial_xyz, link_name)
            alignment_offset = link_alignment_offsets[link_name]
            local_xyz = (
                raw_local[0] + alignment_offset[0],
                raw_local[1] + alignment_offset[1],
                raw_local[2] + alignment_offset[2],
            )
            local_rpy = visual["rpy"]
        else:
            local_xyz = _raw_link_local_visual_origin(visual, link_initial_xyz, link_name)
            local_rpy = visual["rpy"]
        _add_visual(
            links[link_name],
            name=visual["name"],
            mesh_path=mesh_path,
            xyz=local_xyz,
            rpy=local_rpy,
        )
        link_visual_counts[link_name] += 1
        if visual["mesh_name"] in _SKELETON_FIRST_FOLLOWER_MESHES:
            if link_name == "r_wrist_interface":
                wrist_fixed_major_visuals.append(visual["name"])
            else:
                moving_skeleton_visual_counts[link_name] += 1
        if visual["mesh_name"] in _OMITTED_SHELL_VISUAL_MESHES and link_name != "r_wrist_interface":
            moving_shell_visual_counts[link_name] += 1
    skeleton_first_exclusions = [
        "small_screws",
        "washers",
        "tiny_spacers",
        "exact_closed_loop",
    ]
    if not include_finger_shells:
        skeleton_first_exclusions.extend(
            [
                "outer_shell_visuals_hidden_for_joint_debug",
                "shell_alignment_finalization",
            ]
        )
    return {
        "visual_attachment_mode": "mjcf_visuals_partitioned_to_tree_links",
        "skeleton_first_policy": "major_linkage_and_pin_visuals_follow_generated_finger_links",
        "skeleton_first_exclusions": skeleton_first_exclusions,
        "finger_shell_visuals_enabled": include_finger_shells,
        "finger_shell_alignment_policy": (
            "proximal_and_distal_shell_visuals_follow_generated_finger_links"
            if include_finger_shells
            else "finger_shell_visuals_hidden_until_shell_alignment_pass"
        ),
        "omitted_shell_visual_count": len(omitted_shell_visuals),
        "omitted_shell_visuals": omitted_shell_visuals,
        "moving_shell_visual_count": sum(moving_shell_visual_counts.values()),
        "moving_shell_visual_counts": {
            link_name: count
            for link_name, count in sorted(moving_shell_visual_counts.items())
            if count
        },
        "omitted_detail_visual_count": len(omitted_detail_visuals),
        "omitted_detail_visuals": omitted_detail_visuals,
        "mjcf_visual_geom_count": len(visual_geoms),
        "missing_mjcf_visual_meshes": missing_meshes,
        "link_visual_counts": {
            link_name: count
            for link_name, count in sorted(link_visual_counts.items())
            if count
        },
        "moving_skeleton_visual_counts": {
            link_name: count
            for link_name, count in sorted(moving_skeleton_visual_counts.items())
            if count
        },
        "tree_alignment_anchor_meshes": dict(sorted(link_alignment_anchor_meshes.items())),
        "wrist_fixed_major_visuals": wrist_fixed_major_visuals,
    }


def _add_mjcf_visual_shell(
    robot: ET.Element,
    *,
    package_root: Path,
) -> dict[str, Any]:
    visual_shell = _add_link(
        robot,
        "amazinghand_visual_shell",
        mass=0.001,
        inertia=(1.0e-8, 1.0e-8, 1.0e-8),
    )
    _add_fixed_joint(
        robot,
        name="wrist_to_amazinghand_visual_shell",
        parent="r_wrist_interface",
        child="amazinghand_visual_shell",
        xyz=(0.0, 0.0, 0.0),
    )

    visual_geoms = _collect_mjcf_visual_geoms(package_root)
    missing_meshes: list[str] = []
    for visual in visual_geoms:
        mesh_path = Path(visual["mesh_path"])
        if not mesh_path.is_file():
            missing_meshes.append(str(mesh_path))
            continue
        _add_visual(
            visual_shell,
            name=visual["name"],
            mesh_path=mesh_path,
            xyz=visual["xyz"],
            rpy=visual["rpy"],
        )
    return {
        "visual_attachment_mode": "mjcf_static_visual_shell",
        "mjcf_visual_geom_count": len(visual_geoms),
        "missing_mjcf_visual_meshes": missing_meshes,
        "link_visual_counts": {"amazinghand_visual_shell": len(visual_geoms) - len(missing_meshes)},
        "motion_caveat": (
            "Original AmazingHand MJCF visual assembly is fixed to the wrist. "
            "Primitive collision fingers still move for contact; this avoids "
            "tearing closed-loop visual parts around approximate URDF pivots."
        ),
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
        "finger_base_frame_count": 4,
        "finger_base_anchor_policy": "fixed finger_base frames use MJCF custom_servo_horn world positions as palm-local motor-frame anchors",
        "finger_roles": {
            f"finger{index}": layout["role"]
            for index, layout in sorted(_FINGER_LAYOUTS.items())
        },
        "finger_base_layouts": {
            f"finger{index}": {
                "role": layout["role"],
                "base_xyz": list(layout["base_xyz"]),
                "mjcf_anchor_body": layout["mjcf_anchor_body"],
                "mjcf_proximal_xyz": list(layout["mjcf_proximal_xyz"]),
            }
            for index, layout in sorted(_FINGER_LAYOUTS.items())
        },
        "excluded_human_finger": "pinky",
        "actuated_joint_names": list(HAND_ACTUATED_JOINT_NAMES),
        "motor_contract": build_amazinghand_motor_contract(),
        "grasp_types": list(HAND_GRASP_TYPES),
        "default_grasp_type": "wrap",
        "visual_mesh_files": list(_VISUAL_MESH_FILES),
        "collision_primitive_count": 13,
        "equality_constraint_count": 0,
        "notes": [
            "STL files are used only as visual geometry.",
            "Primitive box collisions are used for stable Isaac contact.",
            "The model is a tree articulation with four fixed finger-base frames and four two-joint fingers.",
            "Finger base frames are aligned to MJCF custom_servo_horn anchor positions for motor-frame debugging.",
            "Default visual mode partitions MJCF visuals onto moving tree links so grasp motion is visible.",
            "Static shell mode is available only as a legacy debug fallback.",
        ],
        "default_visual_mode": VISUAL_MODE_PARTITIONED_LINKS,
        "available_visual_modes": [
            VISUAL_MODE_PARTITIONED_LINKS,
            VISUAL_MODE_STATIC_SHELL,
            VISUAL_MODE_IMPLEMENTED_ONLY,
        ],
    }


def build_amazinghand_motor_contract() -> dict[str, Any]:
    """Return the AmazingHand SCS0009 servo contract mirrored from r_hand.toml."""
    motors = {
        joint_name: {
            **config,
            "model": AMAZINGHAND_SERVO_MODEL,
            "joint_name": joint_name,
        }
        for joint_name, config in _AMAZINGHAND_MOTOR_CONFIG.items()
    }
    return {
        "source": "AmazingHand/Demo/AHControl/config/r_hand.toml",
        "servo_model": AMAZINGHAND_SERVO_MODEL,
        "default_speed": AMAZINGHAND_DEFAULT_SPEED,
        "servo_ids": [motors[name]["servo_id"] for name in HAND_ACTUATED_JOINT_NAMES],
        "joint_to_servo_id": {
            name: motors[name]["servo_id"] for name in HAND_ACTUATED_JOINT_NAMES
        },
        "motors": motors,
        "servo_command_policy": (
            "Apply the r_hand.toml offset in radians to each generated motor target; "
            "invert the final target only when the upstream config marks that motor inverted."
        ),
    }


def _grasp_type_motor_scales(
    grasp_type: str,
) -> tuple[dict[int, float], dict[int, float]]:
    if grasp_type == "wrap":
        return (
            {1: 1.0, 2: 1.0, 3: 1.0, 4: 1.0},
            {1: 1.0, 2: 1.0, 3: 1.0, 4: 1.0},
        )
    if grasp_type == "pinch":
        return (
            {1: 1.0, 2: 0.45, 3: 0.45, 4: 1.0},
            {1: 1.0, 2: 0.40, 3: 0.40, 4: 1.0},
        )
    return (
        {1: 0.62, 2: 0.62, 3: 0.62, 4: 0.62},
        {1: 0.58, 2: 0.58, 3: 0.58, 4: 0.58},
    )


def grasp_scalar_to_hand_joint_targets(grasp: float) -> dict[str, float]:
    """Map a normalized grasp command to the eight Isaac-friendly hand joints."""
    return grasp_preshape_to_hand_joint_targets(grasp, "wrap")


def grasp_preshape_to_hand_joint_targets(
    grasp_amount: float,
    grasp_type: str = "wrap",
) -> dict[str, float]:
    """Map a normalized preshape command to the eight generated hand joints.

    The generated Isaac hand intentionally starts with low-dimensional control:
    one grasp amount plus a named preshape.  ``wrap`` is the legacy scalar
    behavior, while ``pinch`` and ``wide`` provide safer intermediate policies
    before exposing raw per-servo targets.
    """
    if grasp_type not in HAND_GRASP_TYPES:
        raise ValueError(
            f"Unsupported grasp_type {grasp_type!r}; supported grasp types: "
            f"{', '.join(HAND_GRASP_TYPES)}"
        )
    closedness = max(0.0, min(1.0, float(grasp_amount)))
    motor1_scale, motor2_scale = _grasp_type_motor_scales(grasp_type)
    targets: dict[str, float] = {}
    for finger_index in range(1, 5):
        targets[f"finger{finger_index}_motor1"] = (
            0.05 + closedness * 0.90 * motor1_scale[finger_index]
        )
        targets[f"finger{finger_index}_motor2"] = (
            0.02 + closedness * 1.08 * motor2_scale[finger_index]
        )
    return targets


def grasp_preshape_to_servo_targets(
    grasp_amount: float,
    grasp_type: str = "wrap",
) -> dict[int, float]:
    """Map a normalized preshape to real AmazingHand SCS0009 servo targets.

    The generated Isaac hand uses positive revolute joints for stable tree
    articulation.  The hardware hand uses paired SCS0009 servos where motor1
    and motor2 close in opposite angular directions.  This helper preserves the
    real command convention from the upstream Python examples while keeping the
    generated joint targets available for simulation.
    """
    if grasp_type not in HAND_GRASP_TYPES:
        raise ValueError(
            f"Unsupported grasp_type {grasp_type!r}; supported grasp types: "
            f"{', '.join(HAND_GRASP_TYPES)}"
        )
    closedness = max(0.0, min(1.0, float(grasp_amount)))
    motor1_scale, motor2_scale = _grasp_type_motor_scales(grasp_type)
    targets: dict[int, float] = {}
    for joint_name in HAND_ACTUATED_JOINT_NAMES:
        config = _AMAZINGHAND_MOTOR_CONFIG[joint_name]
        finger_index = int(config["finger_index"])
        motor_index = int(config["motor_index"])
        scale = motor1_scale[finger_index] if motor_index == 1 else motor2_scale[finger_index]
        open_deg = -30.0 if motor_index == 1 else 30.0
        closed_deg = 90.0 * scale if motor_index == 1 else -90.0 * scale
        relative_rad = math.radians(open_deg + closedness * (closed_deg - open_deg))
        servo_target = relative_rad + float(config["offset_rad"])
        if config["invert"]:
            servo_target = -servo_target
        targets[int(config["servo_id"])] = servo_target
    return targets


def _mesh(asset_root: Path, filename: str) -> Path:
    return asset_root / filename


def generate_graspable_hand_urdf(
    package_root: str | Path,
    output_urdf: str | Path,
    *,
    robot_name: str = "amazinghand_graspable",
    visual_mode: str = VISUAL_MODE_PARTITIONED_LINKS,
    include_finger_shells: bool = False,
) -> dict[str, Any]:
    """Generate a simplified tree hand URDF that Isaac can import as an articulation."""
    package = Path(package_root)
    output = Path(output_urdf)
    asset_root = package / "hand_mjcf" / "assets"
    if not asset_root.is_dir():
        raise FileNotFoundError(f"Hand asset directory not found: {asset_root}")
    supported_visual_modes = {
        VISUAL_MODE_STATIC_SHELL,
        VISUAL_MODE_PARTITIONED_LINKS,
        VISUAL_MODE_IMPLEMENTED_ONLY,
    }
    if visual_mode not in supported_visual_modes:
        raise ValueError(
            f"Unsupported visual_mode {visual_mode!r}; expected one of "
            f"{sorted(supported_visual_modes)!r}"
        )

    implemented_only_visuals = visual_mode == VISUAL_MODE_IMPLEMENTED_ONLY
    implemented_debug_visual_count = 0

    def add_implemented_box_visual(
        link: ET.Element,
        *,
        name: str,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        color: tuple[float, float, float, float],
    ) -> None:
        nonlocal implemented_debug_visual_count
        if not implemented_only_visuals:
            return
        _add_box_visual(
            link,
            name=name,
            size=size,
            xyz=xyz,
            color=color,
        )
        implemented_debug_visual_count += 1

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
    add_implemented_box_visual(
        palm,
        name="palm_contact_box_implemented_visual",
        size=(0.09, 0.085, 0.035),
        xyz=(0.0, -0.038, 0.0),
        color=(0.18, 0.42, 0.95, 0.55),
    )
    _add_fixed_joint(
        robot,
        name="wrist_to_palm",
        parent="r_wrist_interface",
        child="palm",
        xyz=(0.0, 0.0, 0.0),
    )

    for finger_index in range(1, 5):
        base_name = f"finger{finger_index}_base"
        proximal_name = f"finger{finger_index}_proximal"
        distal_name = f"finger{finger_index}_distal"
        layout = _FINGER_LAYOUTS[finger_index]
        base = _add_link(
            robot,
            base_name,
            mass=0.006,
            inertia=(1.0e-6, 1.0e-6, 1.0e-6),
        )
        links[base_name] = base
        add_implemented_box_visual(
            base,
            name=f"{base_name}_motor_mount_frame_implemented_visual",
            size=(0.030, 0.024, 0.024),
            xyz=(0.0, 0.0, 0.0),
            color=(0.25, 0.65, 1.0, 0.55),
        )
        _add_fixed_joint(
            robot,
            name=f"palm_to_{base_name}",
            parent="palm",
            child=base_name,
            xyz=layout["base_xyz"],
        )

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
        add_implemented_box_visual(
            proximal,
            name=f"{proximal_name}_contact_box_implemented_visual",
            size=(0.018, 0.058, 0.018),
            xyz=(0.0, 0.029, 0.0),
            color=(0.95, 0.54, 0.18, 0.62),
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
        add_implemented_box_visual(
            distal,
            name=f"{distal_name}_contact_box_implemented_visual",
            size=(0.016, 0.050, 0.016),
            xyz=(0.0, 0.025, 0.0),
            color=(0.95, 0.54, 0.18, 0.62),
        )
        _add_box_collision(
            distal,
            name=f"{distal_name}_tip_pad_contact_box",
            size=(0.026, 0.014, 0.022),
            xyz=(0.0, 0.055, 0.0),
        )
        add_implemented_box_visual(
            distal,
            name=f"{distal_name}_tip_pad_contact_box_implemented_visual",
            size=(0.026, 0.014, 0.022),
            xyz=(0.0, 0.055, 0.0),
            color=(0.1, 0.85, 0.35, 0.72),
        )

        _add_revolute_joint(
            robot,
            name=f"finger{finger_index}_motor1",
            parent=base_name,
            child=proximal_name,
            xyz=(0.0, 0.0, 0.0),
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

    if visual_mode == VISUAL_MODE_STATIC_SHELL:
        visual_shell_report = _add_mjcf_visual_shell(robot, package_root=package)
    elif visual_mode == VISUAL_MODE_IMPLEMENTED_ONLY:
        visual_shell_report = {
            "visual_attachment_mode": "implemented_collision_primitives_only",
            "skeleton_first_policy": "hide_mjcf_cad_visuals_show_only_generated_collision_primitives",
            "skeleton_first_exclusions": [
                "all_mjcf_cad_visuals_hidden_for_joint_debug",
                "outer_shell_visuals_hidden_for_joint_debug",
                "small_screws",
                "washers",
                "tiny_spacers",
                "exact_closed_loop",
            ],
            "finger_shell_visuals_enabled": False,
            "finger_shell_alignment_policy": "finger_shell_visuals_hidden_until_shell_alignment_pass",
            "omitted_shell_visual_count": 16,
            "omitted_shell_visuals": [],
            "moving_shell_visual_count": 0,
            "moving_shell_visual_counts": {},
            "omitted_detail_visual_count": 0,
            "omitted_detail_visuals": [],
            "mjcf_visual_geom_count": len(_collect_mjcf_visual_geoms(package)),
            "missing_mjcf_visual_meshes": [],
            "link_visual_counts": {
                link.attrib["name"]: len(link.findall("visual"))
                for link in links.values()
                if link.findall("visual")
            },
            "implemented_debug_visual_count": implemented_debug_visual_count,
            "implemented_debug_visual_policy": "translucent box visuals for authored collision primitives plus fixed finger-base motor frames",
            "moving_skeleton_visual_counts": {},
            "tree_alignment_anchor_meshes": {},
            "wrist_fixed_major_visuals": [],
            "finger_base_anchor_policy": spec["finger_base_anchor_policy"],
            "finger_base_layouts": spec["finger_base_layouts"],
        }
    else:
        visual_shell_report = _add_mjcf_visuals_to_tree_links(
            links,
            package_root=package,
            include_finger_shells=include_finger_shells,
        )

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
        "motor_contract": spec["motor_contract"],
        "visual_mode": visual_mode,
        "finger_base_anchor_policy": spec["finger_base_anchor_policy"],
        "finger_base_layouts": spec["finger_base_layouts"],
        "finger_shell_visuals_enabled": (
            include_finger_shells and visual_mode == VISUAL_MODE_PARTITIONED_LINKS
        ),
        "implemented_debug_visual_count": implemented_debug_visual_count,
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
