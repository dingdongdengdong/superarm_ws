"""Generate the portable source-arm + official AmazingHand MuJoCo model.

The source arm is regenerated from ``robot_arm_hand_package.zip`` with the
existing sanitizer, compiled by MuJoCo's URDF importer, and merged with the
official AmazingHand MJCF. The hand body, all eight position actuators, and all
20 closed-loop equality constraints are copied without simplification.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from isaacsim_test.isaacsim.robot_arm_hand_from_zip import (
    ARM_JOINT_NAMES,
    extract_robot_arm_hand_package,
    sanitize_arm_urdf,
)

HAND_ACTUATOR_NAMES = [
    f"finger{finger}_motor{motor}"
    for finger in range(1, 5)
    for motor in range(1, 3)
]
ACTUATOR_ORDER = [*ARM_JOINT_NAMES, *HAND_ACTUATOR_NAMES]
ARM_CONTROL_RANGE = (-1.57, 1.57)

# The fixed source-arm CAD chain resolves the AmazingHand interface at this
# transform in arm_link3b coordinates. This is the transform used by the
# planning prototype that compiled the complete closed-loop model.
HAND_ATTACHMENT_BODY = "arm_link3b"
HAND_ATTACHMENT_LOCAL_POS = (0.0, -0.025, 0.186753)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _copy_meshes(source: ET.Element | None, source_root: Path, target_root: Path) -> int:
    if source is None:
        return 0
    target_root.mkdir(parents=True, exist_ok=True)
    count = 0
    for mesh in source.findall("mesh"):
        mesh_file = mesh.get("file")
        if not mesh_file:
            continue
        source_path = Path(mesh_file)
        if not source_path.is_absolute():
            source_path = source_root / source_path
        if not source_path.is_file():
            raise FileNotFoundError(f"Mesh referenced by model is missing: {source_path}")
        target_path = target_root / source_path.name
        if target_path.exists() and _sha256(target_path) != _sha256(source_path):
            raise RuntimeError(f"Conflicting mesh basename: {source_path.name}")
        shutil.copy2(source_path, target_path)
        mesh.set("file", target_path.relative_to(target_root.parent.parent).as_posix())
        count += 1
    return count


def _find_body(root: ET.Element, name: str) -> ET.Element:
    for body in root.iter("body"):
        if body.get("name") == name:
            return body
    raise RuntimeError(f"Required MuJoCo body is missing: {name}")


def generate_combined_model(
    *,
    workspace_root: str | Path,
    output_dir: str | Path,
) -> dict[str, Any]:
    """Generate the combined model and return its reproducibility manifest."""
    try:
        import mujoco
    except ImportError as exc:  # pragma: no cover - exercised by capability API
        raise RuntimeError("mujoco==3.10.0 is required to generate the model") from exc

    workspace = Path(workspace_root).resolve()
    output = Path(output_dir).resolve()
    zip_path = workspace / "robot_arm_hand_package.zip"
    if not zip_path.is_file():
        raise FileNotFoundError(f"Source package is missing: {zip_path}")

    output.mkdir(parents=True, exist_ok=True)
    source_dir = output / "source"
    input_dir = output / "generated_inputs"
    assets_dir = output / "assets"
    for directory in (source_dir, input_dir, assets_dir):
        if directory.exists():
            shutil.rmtree(directory)
        directory.mkdir(parents=True, exist_ok=True)

    package_root = extract_robot_arm_hand_package(zip_path, input_dir)
    arm_urdf = source_dir / "source_arm_sanitized.urdf"
    arm_report = sanitize_arm_urdf(package_root, arm_urdf)
    hand_xml = package_root / "hand_mjcf" / "robot.xml"

    with tempfile.TemporaryDirectory(prefix="superarm-mujoco-") as temp:
        arm_model = mujoco.MjModel.from_xml_path(str(arm_urdf))
        arm_mjcf_path = Path(temp) / "source_arm_imported.xml"
        mujoco.mj_saveLastXML(str(arm_mjcf_path), arm_model)
        arm_root = ET.parse(arm_mjcf_path).getroot()

    hand_root = ET.parse(hand_xml).getroot()
    combined = ET.Element("mujoco", {"model": "source_arm_amazinghand"})
    ET.SubElement(combined, "compiler", {"angle": "radian", "autolimits": "true"})
    ET.SubElement(combined, "option", {"timestep": "0.002", "integrator": "implicitfast"})
    ET.SubElement(
        combined,
        "visual",
    )

    for default in hand_root.findall("default"):
        combined.append(copy.deepcopy(default))

    asset = ET.SubElement(combined, "asset")
    arm_asset = copy.deepcopy(arm_root.find("asset"))
    hand_asset = copy.deepcopy(hand_root.find("asset"))
    arm_mesh_count = _copy_meshes(arm_asset, Path("/"), assets_dir / "arm")
    hand_mesh_count = _copy_meshes(
        hand_asset,
        package_root / "hand_mjcf" / "assets",
        assets_dir / "hand",
    )
    for source_asset in (arm_asset, hand_asset):
        if source_asset is not None:
            for child in source_asset:
                asset.append(child)

    world = ET.SubElement(combined, "worldbody")
    ET.SubElement(world, "light", {"pos": "0 0 3.5", "dir": "0 0 -1", "directional": "true"})
    ET.SubElement(
        world,
        "geom",
        {
            "name": "floor",
            "size": "0 0 0.05",
            "pos": "0 0 -0.02",
            "type": "plane",
            "rgba": ".12 .16 .20 1",
        },
    )
    arm_world = arm_root.find("worldbody")
    if arm_world is None:
        raise RuntimeError("MuJoCo URDF importer did not emit a worldbody")
    for child in arm_world:
        world.append(copy.deepcopy(child))

    hand_world = hand_root.find("worldbody")
    if hand_world is None:
        raise RuntimeError("Official AmazingHand MJCF has no worldbody")
    official_hand_body = next(
        (
            child
            for child in hand_world
            if child.tag == "body" and child.get("name") == "r_wrist_interface"
        ),
        None,
    )
    if official_hand_body is None:
        raise RuntimeError("Official AmazingHand r_wrist_interface body is missing")
    attached_hand = copy.deepcopy(official_hand_body)
    attached_hand.set("pos", " ".join(str(value) for value in HAND_ATTACHMENT_LOCAL_POS))
    _find_body(world, HAND_ATTACHMENT_BODY).append(attached_hand)

    for section_name in ("contact", "equality"):
        section = hand_root.find(section_name)
        if section is not None:
            combined.append(copy.deepcopy(section))

    actuator = ET.SubElement(combined, "actuator")
    for joint_name in ARM_JOINT_NAMES:
        ET.SubElement(
            actuator,
            "position",
            {
                "name": joint_name,
                "joint": joint_name,
                "kp": "50",
                "dampratio": "1",
                "ctrlrange": f"{ARM_CONTROL_RANGE[0]} {ARM_CONTROL_RANGE[1]}",
            },
        )
    official_actuators = hand_root.find("actuator")
    if official_actuators is None:
        raise RuntimeError("Official AmazingHand actuator section is missing")
    for child in official_actuators:
        actuator.append(copy.deepcopy(child))

    ET.indent(combined)
    model_path = output / "superarm_amazinghand.xml"
    ET.ElementTree(combined).write(model_path, encoding="utf-8", xml_declaration=True)

    model = mujoco.MjModel.from_xml_path(str(model_path))
    actuator_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
        for index in range(model.nu)
    ]
    if actuator_names != ACTUATOR_ORDER:
        raise RuntimeError(f"Unexpected actuator order: {actuator_names}")
    if model.neq != 20:
        raise RuntimeError(f"Expected 20 hand equalities, got {model.neq}")

    manifest = {
        "schema_version": 1,
        "generator": Path(__file__).relative_to(workspace).as_posix(),
        "source_zip": zip_path.relative_to(workspace).as_posix(),
        "source_zip_sha256": _sha256(zip_path),
        "sanitized_arm_urdf": arm_urdf.relative_to(output).as_posix(),
        "sanitized_arm_urdf_sha256": _sha256(arm_urdf),
        "official_hand_mjcf": "robot_arm_hand_package/hand_mjcf/robot.xml",
        "model": model_path.name,
        "model_sha256": _sha256(model_path),
        "arm_sanitizer_report": arm_report,
        "hand_attachment_body": HAND_ATTACHMENT_BODY,
        "hand_attachment_local_pos": list(HAND_ATTACHMENT_LOCAL_POS),
        "actuator_order": actuator_names,
        "arm_joint_limit_rad": list(ARM_CONTROL_RANGE),
        "actuator_count": model.nu,
        "equality_count": model.neq,
        "mesh_count": model.nmesh,
        "arm_mesh_count": arm_mesh_count,
        "hand_mesh_count": hand_mesh_count,
        "model_timestep_s": model.opt.timestep,
    }
    manifest_path = output / "generated_inputs_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return {**manifest, "model_path": str(model_path), "manifest_path": str(manifest_path)}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace-root",
        default=str(Path(__file__).resolve().parents[2]),
    )
    parser.add_argument(
        "--output-dir",
        default=str(Path(__file__).resolve().parent / "generated"),
    )
    args = parser.parse_args()
    result = generate_combined_model(
        workspace_root=args.workspace_root,
        output_dir=args.output_dir,
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()

