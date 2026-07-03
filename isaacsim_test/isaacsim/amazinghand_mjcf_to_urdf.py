"""Convert the upstream AmazingHand MJCF export into a standalone URDF."""
from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable


def _parse_float_list(raw: str | None, default: Iterable[float]) -> list[float]:
    if not raw:
        return list(default)
    return [float(part) for part in raw.split()]


def _format_float(value: float) -> str:
    if abs(value) < 1e-15:
        value = 0.0
    return f"{value:.16g}"


def _format_floats(values: Iterable[float]) -> str:
    return " ".join(_format_float(value) for value in values)


def _quat_to_rpy(quat: list[float]) -> list[float]:
    w, x, y, z = quat

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
    return [roll, pitch, yaw]


def _origin(parent: ET.Element, *, xyz: str | None = None, quat: str | None = None) -> None:
    pos = _parse_float_list(xyz, [0.0, 0.0, 0.0])
    rpy = _quat_to_rpy(_parse_float_list(quat, [1.0, 0.0, 0.0, 0.0]))
    ET.SubElement(parent, "origin", {"xyz": _format_floats(pos), "rpy": _format_floats(rpy)})


def _sanitize_name(raw: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", raw).strip("_")
    return cleaned or "unnamed"


def _unique_name(raw: str, used: set[str]) -> str:
    candidate = _sanitize_name(raw)
    if candidate not in used:
        used.add(candidate)
        return candidate
    suffix = 2
    while f"{candidate}_{suffix}" in used:
        suffix += 1
    unique = f"{candidate}_{suffix}"
    used.add(unique)
    return unique


def _add_inertial(link: ET.Element, inertial_source: ET.Element | None) -> None:
    inertial = ET.SubElement(link, "inertial")
    if inertial_source is None:
        ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(inertial, "mass", {"value": "1e-09"})
        ET.SubElement(
            inertial,
            "inertia",
            {"ixx": "1e-09", "ixy": "0", "ixz": "0", "iyy": "1e-09", "iyz": "0", "izz": "1e-09"},
        )
        return

    _origin(inertial, xyz=inertial_source.attrib.get("pos"), quat=inertial_source.attrib.get("quat"))
    ET.SubElement(inertial, "mass", {"value": inertial_source.attrib.get("mass", "1e-09")})
    full = _parse_float_list(inertial_source.attrib.get("fullinertia"), [1e-9, 1e-9, 1e-9, 0.0, 0.0, 0.0])
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": _format_float(full[0]),
            "iyy": _format_float(full[1]),
            "izz": _format_float(full[2]),
            "ixy": _format_float(full[3]),
            "ixz": _format_float(full[4]),
            "iyz": _format_float(full[5]),
        },
    )


def _add_dummy_link(robot: ET.Element, name: str) -> None:
    link = ET.SubElement(robot, "link", {"name": name})
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": "1e-06"})
    ET.SubElement(
        inertial,
        "inertia",
        {"ixx": "1e-09", "ixy": "0", "ixz": "0", "iyy": "1e-09", "iyz": "0", "izz": "1e-09"},
    )


def _add_visuals(
    link: ET.Element,
    body: ET.Element,
    *,
    mesh_files: dict[str, str],
    visual_counts: dict[str, int],
) -> int:
    count = 0
    link_name = link.attrib["name"]
    for geom in body.findall("geom"):
        if geom.attrib.get("type") != "mesh" or "mesh" not in geom.attrib:
            continue
        mesh_name = geom.attrib["mesh"]
        if mesh_name not in mesh_files:
            raise RuntimeError(f"Missing mesh asset declaration for {mesh_name!r}")
        visual_counts[link_name] = visual_counts.get(link_name, 0) + 1
        visual = ET.SubElement(
            link,
            "visual",
            {"name": f"{link_name}_visual_{visual_counts[link_name]:02d}"},
        )
        _origin(visual, xyz=geom.attrib.get("pos"), quat=geom.attrib.get("quat"))
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "mesh", {"filename": f"assets/{mesh_files[mesh_name]}"})
        material_name = geom.attrib.get("material")
        if material_name:
            ET.SubElement(visual, "material", {"name": material_name})
        count += 1
    return count


def _add_limit(joint: ET.Element, source: ET.Element, *, default_range: tuple[float, float]) -> None:
    lower, upper = default_range
    if "range" in source.attrib:
        lower, upper = [float(part) for part in source.attrib["range"].split()]
    ET.SubElement(
        joint,
        "limit",
        {
            "lower": _format_float(lower),
            "upper": _format_float(upper),
            "effort": "1.0",
            "velocity": "3.14159",
        },
    )


def _add_revolute_joint(
    robot: ET.Element,
    *,
    name: str,
    parent: str,
    child: str,
    pos: str | None,
    quat: str | None,
    axis: str,
    source: ET.Element,
) -> None:
    joint = ET.SubElement(robot, "joint", {"name": name, "type": "revolute"})
    _origin(joint, xyz=pos, quat=quat)
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    ET.SubElement(joint, "axis", {"xyz": _format_floats(_parse_float_list(axis, [0.0, 0.0, 1.0]))})
    _add_limit(joint, source, default_range=(-math.pi, math.pi))


def _add_ball_joint_chain(
    robot: ET.Element,
    *,
    name: str,
    parent: str,
    child: str,
    pos: str | None,
    quat: str | None,
) -> list[str]:
    x_link = f"{child}_{name}_ball_x_link"
    y_link = f"{child}_{name}_ball_y_link"
    _add_dummy_link(robot, x_link)
    _add_dummy_link(robot, y_link)

    dummy_source = ET.Element("joint")
    joints = [
        (f"{name}_x", parent, x_link, "1 0 0", pos, quat),
        (f"{name}_y", x_link, y_link, "0 1 0", None, None),
        (f"{name}_z", y_link, child, "0 0 1", None, None),
    ]
    for joint_name, joint_parent, joint_child, axis, joint_pos, joint_quat in joints:
        _add_revolute_joint(
            robot,
            name=joint_name,
            parent=joint_parent,
            child=joint_child,
            pos=joint_pos,
            quat=joint_quat,
            axis=axis,
            source=dummy_source,
        )
    return [joint_name for joint_name, *_ in joints]


def _copy_assets(mjcf_path: Path, urdf_path: Path, mesh_files: dict[str, str]) -> None:
    source_dir = mjcf_path.parent / "assets"
    output_dir = urdf_path.parent / "assets"
    output_dir.mkdir(parents=True, exist_ok=True)
    for filename in sorted(set(mesh_files.values())):
        source = source_dir / filename
        if not source.is_file():
            raise FileNotFoundError(source)
        shutil.copy2(source, output_dir / filename)


def convert_amazinghand_mjcf_to_urdf(
    *,
    mjcf_path: str | Path,
    urdf_path: str | Path,
    robot_name: str,
) -> dict[str, object]:
    """Convert an AmazingHand `robot.xml` MJCF file into a standalone URDF.

    MJCF equality/connect constraints are not representable in URDF. They are
    reported in the returned metadata and as a URDF comment for downstream review.
    """

    mjcf = Path(mjcf_path)
    urdf = Path(urdf_path)
    root = ET.parse(mjcf).getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError(f"Missing worldbody in {mjcf}")
    root_bodies = worldbody.findall("body")
    if len(root_bodies) != 1:
        raise RuntimeError(f"Expected one root body in {mjcf}, found {len(root_bodies)}")

    asset = root.find("asset")
    mesh_files: dict[str, str] = {}
    material_rgba: dict[str, str] = {}
    if asset is not None:
        for mesh in asset.findall("mesh"):
            filename = mesh.attrib["file"]
            mesh_files[mesh.attrib.get("name", Path(filename).stem)] = filename
        for material in asset.findall("material"):
            if "name" in material.attrib and "rgba" in material.attrib:
                material_rgba[material.attrib["name"]] = material.attrib["rgba"]

    _copy_assets(mjcf, urdf, mesh_files)

    robot = ET.Element("robot", {"name": robot_name})
    robot.append(ET.Comment(f"Converted from {mjcf.as_posix()}"))

    equality_constraints = root.findall(".//equality/connect")
    if equality_constraints:
        robot.append(
            ET.Comment(
                "MJCF equality/connect constraints cannot be represented directly in URDF; "
                f"{len(equality_constraints)} constraints were omitted."
            )
        )

    for material_name, rgba in sorted(material_rgba.items()):
        material = ET.SubElement(robot, "material", {"name": material_name})
        ET.SubElement(material, "color", {"rgba": rgba})

    used_link_names: set[str] = set()
    used_joint_names: set[str] = set()
    visual_counts: dict[str, int] = {}
    joint_counts = {"hinge": 0, "ball": 0, "expanded_ball_revolute": 0}
    visual_count = 0

    def add_body(body: ET.Element, parent_link_name: str | None) -> None:
        nonlocal visual_count

        source_body_name = body.attrib["name"]
        body_link_name = _unique_name(source_body_name, used_link_names)

        source_joint = body.find("joint")
        if parent_link_name is None:
            link_parent_for_children = body_link_name
        elif source_joint is None:
            joint_name = _unique_name(f"{parent_link_name}_to_{body_link_name}", used_joint_names)
            fixed = ET.SubElement(robot, "joint", {"name": joint_name, "type": "fixed"})
            _origin(fixed, xyz=body.attrib.get("pos"), quat=body.attrib.get("quat"))
            ET.SubElement(fixed, "parent", {"link": parent_link_name})
            ET.SubElement(fixed, "child", {"link": body_link_name})
            link_parent_for_children = body_link_name
        else:
            joint_type = source_joint.attrib.get("type", "hinge")
            source_name = source_joint.attrib.get("name", f"{parent_link_name}_to_{body_link_name}")
            joint_name = _unique_name(source_name, used_joint_names)
            if joint_type == "ball":
                expanded = _add_ball_joint_chain(
                    robot,
                    name=joint_name,
                    parent=parent_link_name,
                    child=body_link_name,
                    pos=body.attrib.get("pos"),
                    quat=body.attrib.get("quat"),
                )
                joint_counts["ball"] += 1
                joint_counts["expanded_ball_revolute"] += len(expanded)
            elif joint_type in ("hinge", "revolute"):
                _add_revolute_joint(
                    robot,
                    name=joint_name,
                    parent=parent_link_name,
                    child=body_link_name,
                    pos=body.attrib.get("pos"),
                    quat=body.attrib.get("quat"),
                    axis=source_joint.attrib.get("axis", "0 0 1"),
                    source=source_joint,
                )
                joint_counts["hinge"] += 1
            else:
                raise RuntimeError(f"Unsupported MJCF joint type {joint_type!r} on {source_body_name}")
            link_parent_for_children = body_link_name

        link = ET.SubElement(robot, "link", {"name": body_link_name})
        _add_inertial(link, body.find("inertial"))
        visual_count += _add_visuals(link, body, mesh_files=mesh_files, visual_counts=visual_counts)

        for child_body in body.findall("body"):
            add_body(child_body, link_parent_for_children)

    add_body(root_bodies[0], None)

    ET.indent(robot, space="  ")
    urdf.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(robot).write(urdf, encoding="utf-8", xml_declaration=True)
    urdf.write_text(urdf.read_text(encoding="utf-8") + "\n", encoding="utf-8")

    return {
        "mjcf_path": mjcf.as_posix(),
        "urdf_path": urdf.as_posix(),
        "robot_name": robot_name,
        "root_link": root_bodies[0].attrib["name"],
        "body_count": len(root.findall(".//body")),
        "link_count": len(robot.findall("link")),
        "visual_count": visual_count,
        "mesh_asset_count": len(mesh_files),
        "material_count": len(material_rgba),
        "hinge_joint_count": joint_counts["hinge"],
        "ball_joint_count": joint_counts["ball"],
        "expanded_ball_revolute_joint_count": joint_counts["expanded_ball_revolute"],
        "equality_constraint_count": len(equality_constraints),
        "unsupported_features": ["mjcf_equality_connect"] if equality_constraints else [],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mjcf", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--robot-name", required=True)
    args = parser.parse_args()

    report = convert_amazinghand_mjcf_to_urdf(
        mjcf_path=args.mjcf,
        urdf_path=args.urdf,
        robot_name=args.robot_name,
    )
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
