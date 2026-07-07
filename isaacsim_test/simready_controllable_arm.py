"""Author a non-destructive controllable arm overlay for the SimReady echo_full USD.

The source SimReady USD is preserved as a fixed visual reference. A lightweight
five-DOF UsdPhysics articulation is authored beside it so Isaac Sim/LeRobot can
bind real arm joint commands while AmazingHand remains visual-only.

Physical values authored into the derived control rig are accompanied by a
provenance report. Source USD density/bounds are used when available; incomplete
CAD/USD mass, COM, or inertia values are explicitly labeled as inferred fallback
rather than hidden constants.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import time
from pathlib import Path
from typing import Any

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

ARM_JOINT_NAMES: tuple[str, ...] = (
    "right_arm_pitch_joint",
    "right_arm_roll_joint",
    "right_arm_yaw_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
)
FIXED_HAND_FEATURE = "amazinghand_grasp"
ROOT_PRIM_PATH = "/echo_full_controllable"
VISUAL_FIXED_PRIM_PATH = f"{ROOT_PRIM_PATH}/VisualFixed"
CONTROL_RIG_PRIM_PATH = f"{ROOT_PRIM_PATH}/ControlRig"
DEFAULT_SOURCE_USD = Path(
    "isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/"
    "fet005-grasp/echo_full_robot_arm_hand.usd"
)
DEFAULT_OUTPUT_USD = Path(
    "isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/"
    "fet006-controllable-arm-fixed-hand/echo_full_robot_arm_hand_controllable.usda"
)
DEFAULT_MAPPING_JSON = Path("isaacsim_test/artifacts/simready_controllable_prim_mapping.json")
DEFAULT_PHYSICAL_PROPERTIES_JSON = Path("isaacsim_test/artifacts/simready_controllable_physical_properties.json")

CLAIM_BOUNDARY = (
    "CAD/USD-grounded, provenance-disclosed stepped physics motion; not fully measured "
    "production dynamics when values are inferred or runtime-tuned."
)
MASS_FILL_RATIO = 0.35
MIN_COLLIDER_DIMENSION_M = 0.03
MIN_MASS_KG = 0.05
SOURCE_VISUAL_METER_NORMALIZATION = 0.001

# Control-rig link layout. These positions define the authored five-DOF
# kinematic chain. They are not treated as CAD-measured physical truth; if a
# value is used because source data is absent it is reported as inferred/manual
# fallback in the physical-properties provenance JSON.
_LINK_LAYOUTS: tuple[dict[str, Any], ...] = (
    {"name": "base_link", "translate": (0.0, 0.0, 0.06), "fallback_dimensions": (0.18, 0.18, 0.12), "fallback_mass": 8.0},
    {"name": "shoulder_pitch_link", "translate": (0.0, 0.0, 0.28), "fallback_dimensions": (0.14, 0.14, 0.35), "fallback_mass": 3.0},
    {"name": "shoulder_roll_link", "translate": (0.0, 0.0, 0.58), "fallback_dimensions": (0.12, 0.12, 0.28), "fallback_mass": 2.5},
    {"name": "upper_arm_link", "translate": (0.0, 0.0, 0.88), "fallback_dimensions": (0.10, 0.10, 0.42), "fallback_mass": 2.0},
    {"name": "forearm_link", "translate": (0.0, 0.0, 1.22), "fallback_dimensions": (0.09, 0.09, 0.34), "fallback_mass": 1.6},
    {"name": "wrist_fixed_hand_mount_link", "translate": (0.0, 0.0, 1.48), "fallback_dimensions": (0.08, 0.08, 0.18), "fallback_mass": 1.0},
)
_LINK_LAYOUT_BY_NAME = {str(spec["name"]): spec for spec in _LINK_LAYOUTS}

# Joint origin positions in the control-rig frame. Axes are selected to provide
# the requested pitch/roll/yaw semantics for the arm-side command interface.
_JOINT_SPECS: tuple[tuple[str, str, str, tuple[float, float, float]], ...] = (
    ("right_arm_pitch_joint", "base_link", "shoulder_pitch_link", "Y", (0.0, 0.0, 0.16)),
    ("right_arm_roll_joint", "shoulder_pitch_link", "shoulder_roll_link", "X", (0.0, 0.0, 0.44)),
    ("right_arm_yaw_joint", "shoulder_roll_link", "upper_arm_link", "Z", (0.0, 0.0, 0.70)),
    ("right_elbow_pitch_joint", "upper_arm_link", "forearm_link", "Y", (0.0, 0.0, 1.06)),
    ("right_elbow_yaw_joint", "forearm_link", "wrist_fixed_hand_mount_link", "Z", (0.0, 0.0, 1.36)),
)

_AUTHORED_DRIVE_SETTINGS = {
    "type": "force",
    "target_position": 0.0,
    "target_velocity": 0.0,
    "stiffness": 450.0,
    "damping": 45.0,
    "max_force": 250.0,
}
_RUNTIME_CONTROLLER_OVERRIDE_POLICY = {
    "kp": 50000.0,
    "kd": 5000.0,
    "max_effort": 5000.0,
    "provenance": "runtime_tuned",
    "reason": "Viewport physics proof may raise runtime controller authority without writing it back to the USD.",
}

_SOURCE_ROBOT_ROOT = "/echo_full/tn____xaZ2Ve2pr5yw2tw0WSflDbf1/tn__v341_a4a4XfWAou7zcLveO"
_FIXED_VISUAL_ROBOT_ROOT = (
    f"{VISUAL_FIXED_PRIM_PATH}/tn____xaZ2Ve2pr5yw2tw0WSflDbf1/tn__v341_a4a4XfWAou7zcLveO"
)
_AMAZINGHAND_VISUAL_COMPONENT = "tn__AmazingHand_righthandv201_lQjM"
_WRIST_ATTACHED_HAND_VISUAL_BINDING = ("wrist_fixed_hand_mount_link", _AMAZINGHAND_VISUAL_COMPONENT)
# Existing CAD arm components are hidden in the fixed visual reference and
# re-referenced under the driven links below. AmazingHand is intentionally kept
# out of this mass/collider provenance list: it is attached separately as
# wrist-mounted visual-only geometry with no hand/finger physics.
_MOVING_ARM_VISUAL_BINDINGS: tuple[tuple[str, str], ...] = (
    ("shoulder_pitch_link", "tn__RB000160_02_v11_xFsDiuuIbc3"),
    ("shoulder_pitch_link", "tn__DM4340rau35053Dv11_qI7BGHKrKpgzLslb2"),
    ("shoulder_pitch_link", "tn__RB000179_01_5v11_zGoEffzEbog4"),
    ("shoulder_roll_link", "tn__DM4340rau35053Dv12_qI7BGHKrKpgzLslb2"),
    ("shoulder_roll_link", "tn__RB000153_02_2v11_zGoEffzEbog4"),
    ("upper_arm_link", "tn__DM4340rau35053Dv13_qI7BGHKrKpgzLslb2"),
    ("upper_arm_link", "tn__RB000154_02_3v11_zGoEffzEbog4"),
    ("upper_arm_link", "tn__RB000155_01_4v11_zGoEffzEbog4"),
    ("forearm_link", "tn__RB000152_02_1v11_zGoEffzEbog4"),
    ("forearm_link", "tn__DM4340rau35053Dv14_qI7BGHKrKpgzLslb2"),
    ("forearm_link", "tn__RB000153_02_2v12_zGoEffzEbog4"),
    ("wrist_fixed_hand_mount_link", "tn__DM4340rau35053Dv15_qI7BGHKrKpgzLslb2"),
    ("wrist_fixed_hand_mount_link", "tn__RB000154_02_3v12_zGoEffzEbog4"),
    ("wrist_fixed_hand_mount_link", "tn__RB000155_01_4v12_zGoEffzEbog4"),
    ("wrist_fixed_hand_mount_link", "tn___v31_b4a4iZg9jXSVlF4rxHjo7"),
    ("wrist_fixed_hand_mount_link", "tn___v11_b4a4iZg9jXSVlF4rxHjo7"),
    ("wrist_fixed_hand_mount_link", "tn__RealSense_D435v11_WIkF"),
)


def _is_physics_schema_token(token: object) -> bool:
    text = str(token)
    return "Physics" in text or "Physx" in text


def _strip_physics_from_visual_subtree(stage: Usd.Stage, root_path: Sdf.Path | str) -> dict[str, Any]:
    """Remove referenced physics schemas/types from a subtree that is only visual.

    Referencing source CAD/USD subtrees directly keeps visual fidelity, but those
    referenced mesh prims can carry Physics* APIs from the source. Stronger
    overlay opinions delete those APIs and deactivate typed physics helper prims
    so the composed subtree is geometry-only without editing the source USD.
    """
    root_sdf_path = Sdf.Path(str(root_path))
    root = stage.GetPrimAtPath(root_sdf_path)
    summary: dict[str, Any] = {
        "root_path": str(root_sdf_path),
        "root_valid": bool(root and root.IsValid()),
        "removed_api_schema_count": 0,
        "deactivated_physics_type_count": 0,
        "prim_count_with_removed_api_schemas": 0,
        "prim_count_with_deactivated_physics_type": 0,
        "examples": [],
    }
    if not root or not root.IsValid():
        return summary

    # Snapshot paths before authoring overrides/deactivation so traversal is not
    # affected by stronger opinions added below.
    prim_paths = [prim.GetPath() for prim in Usd.PrimRange(root)]
    for prim_path in prim_paths:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            continue

        physics_schemas = [schema for schema in prim.GetAppliedSchemas() if _is_physics_schema_token(schema)]
        type_name = prim.GetTypeName()
        physics_typed = bool(type_name and _is_physics_schema_token(type_name))
        if not physics_schemas and not physics_typed:
            continue

        override = stage.OverridePrim(prim_path)
        if physics_schemas:
            summary["prim_count_with_removed_api_schemas"] += 1
            summary["removed_api_schema_count"] += len(physics_schemas)
            for schema in physics_schemas:
                override.RemoveAppliedSchema(schema)
        if physics_typed:
            summary["prim_count_with_deactivated_physics_type"] += 1
            summary["deactivated_physics_type_count"] += 1
            override.SetActive(False)
        if len(summary["examples"]) < 12:
            summary["examples"].append(
                {
                    "prim_path": str(prim_path),
                    "removed_api_schemas": physics_schemas,
                    "deactivated_type": type_name if physics_typed else None,
                }
            )

    return summary


def _merge_visual_strip_summaries(summaries: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "policy": (
            "delete Physics*/Physx* applied API schemas and deactivate Physics*/Physx* typed prims "
            "inside referenced visual-only subtrees in the derived overlay"
        ),
        "source_usd_mutated": False,
        "subtree_count": len(summaries),
        "removed_api_schema_count": sum(int(item.get("removed_api_schema_count", 0)) for item in summaries),
        "deactivated_physics_type_count": sum(int(item.get("deactivated_physics_type_count", 0)) for item in summaries),
        "prim_count_with_removed_api_schemas": sum(
            int(item.get("prim_count_with_removed_api_schemas", 0)) for item in summaries
        ),
        "prim_count_with_deactivated_physics_type": sum(
            int(item.get("prim_count_with_deactivated_physics_type", 0)) for item in summaries
        ),
        "subtrees": summaries,
    }


def _vec3(value: tuple[float, float, float] | list[float]) -> Gf.Vec3f:
    return Gf.Vec3f(float(value[0]), float(value[1]), float(value[2]))


def _json_vec(value: Any) -> list[float]:
    return [float(value[0]), float(value[1]), float(value[2])]


def _is_finite_number(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _finite_vec(value: Any) -> bool:
    return value is not None and len(value) == 3 and all(_is_finite_number(v) for v in value)


def _clamp_dimensions(dimensions: tuple[float, float, float] | list[float]) -> tuple[float, float, float]:
    return tuple(max(float(v), MIN_COLLIDER_DIMENSION_M) for v in dimensions)  # type: ignore[return-value]


def _box_inertia(mass: float, dimensions: tuple[float, float, float] | list[float]) -> tuple[float, float, float]:
    dx, dy, dz = _clamp_dimensions(dimensions)
    return (
        max((mass / 12.0) * (dy * dy + dz * dz), 1e-6),
        max((mass / 12.0) * (dx * dx + dz * dz), 1e-6),
        max((mass / 12.0) * (dx * dx + dy * dy), 1e-6),
    )


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _component_bindings_by_link() -> dict[str, list[str]]:
    by_link: dict[str, list[str]] = {str(spec["name"]): [] for spec in _LINK_LAYOUTS}
    for link_name, component_name in _MOVING_ARM_VISUAL_BINDINGS:
        by_link.setdefault(link_name, []).append(component_name)
    return by_link


def _density_sources(prim: Usd.Prim) -> list[dict[str, Any]]:
    sources: list[dict[str, Any]] = []
    for desc in Usd.PrimRange(prim):
        attr = desc.GetAttribute("physics:density")
        density = attr.Get() if attr else None
        if density is not None and _is_finite_number(density) and float(density) > 0.0:
            sources.append(
                {
                    "prim_path": str(desc.GetPath()),
                    "density": float(density),
                    "unit": "kg/m^3",
                    "provenance": "derived",
                    "source": "source_usd_physics_density",
                }
            )
    return sources


def _valid_source_mass_properties(prim: Usd.Prim) -> dict[str, Any] | None:
    """Return source-authored mass properties only when all values are finite/nonzero."""
    mass_values: list[float] = []
    inertia_values: list[tuple[float, float, float]] = []
    com_values: list[tuple[float, float, float]] = []
    for desc in Usd.PrimRange(prim):
        mass_attr = desc.GetAttribute("physics:mass")
        inertia_attr = desc.GetAttribute("physics:diagonalInertia")
        com_attr = desc.GetAttribute("physics:centerOfMass")
        mass = mass_attr.Get() if mass_attr else None
        inertia = inertia_attr.Get() if inertia_attr else None
        com = com_attr.Get() if com_attr else None
        if mass is not None and _is_finite_number(mass) and float(mass) > 0 and _finite_vec(inertia) and _finite_vec(com):
            inertia_tuple = tuple(float(v) for v in inertia)
            com_tuple = tuple(float(v) for v in com)
            if all(v > 0 for v in inertia_tuple):
                mass_values.append(float(mass))
                inertia_values.append(inertia_tuple)  # type: ignore[arg-type]
                com_values.append(com_tuple)  # type: ignore[arg-type]
    if not mass_values:
        return None
    total_mass = sum(mass_values)
    weighted_com = tuple(
        sum(com[i] * mass for com, mass in zip(com_values, mass_values)) / total_mass for i in range(3)
    )
    summed_inertia = tuple(sum(inertia[i] for inertia in inertia_values) for i in range(3))
    return {"mass": total_mass, "center_of_mass_world": weighted_com, "diagonal_inertia": summed_inertia}


def _component_evidence(stage: Usd.Stage, bbox_cache: UsdGeom.BBoxCache, component_name: str) -> dict[str, Any]:
    source_path = Sdf.Path(f"{_SOURCE_ROBOT_ROOT}/{component_name}")
    prim = stage.GetPrimAtPath(source_path)
    evidence: dict[str, Any] = {
        "component_name": component_name,
        "source_prim": str(source_path),
        "exists": bool(prim and prim.IsValid()),
    }
    if not prim or not prim.IsValid():
        evidence["fallback_reason"] = "source_component_prim_missing"
        return evidence

    aligned = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
    if aligned.IsEmpty():
        evidence["fallback_reason"] = "source_component_bounds_empty"
    else:
        min_v = _json_vec(aligned.GetMin())
        max_v = _json_vec(aligned.GetMax())
        size_v = _json_vec(aligned.GetSize())
        mid_v = _json_vec(aligned.GetMidpoint())
        evidence.update(
            {
                "bounds": {
                    "min": min_v,
                    "max": max_v,
                    "size": size_v,
                    "midpoint": mid_v,
                    "unit": "m",
                    "provenance": "derived",
                    "source": "source_usd_world_bounds",
                }
            }
        )

    densities = _density_sources(prim)
    evidence["density_sources"] = densities
    valid_mass = _valid_source_mass_properties(prim)
    if valid_mass:
        evidence["valid_source_mass_properties"] = valid_mass
    else:
        evidence["invalid_source_mass_properties"] = "source mass/inertia/COM absent, zero, or non-finite"
    return evidence


def _weighted_average(points: list[list[float]], weights: list[float]) -> list[float]:
    total = sum(weights)
    if total <= 0:
        return [0.0, 0.0, 0.0]
    return [sum(point[i] * weight for point, weight in zip(points, weights)) / total for i in range(3)]


def _union_bounds(component_evidence: list[dict[str, Any]]) -> dict[str, list[float]] | None:
    valid_bounds = [item.get("bounds") for item in component_evidence if item.get("bounds")]
    if not valid_bounds:
        return None
    min_v = [min(bounds["min"][i] for bounds in valid_bounds) for i in range(3)]
    max_v = [max(bounds["max"][i] for bounds in valid_bounds) for i in range(3)]
    size_v = [max_v[i] - min_v[i] for i in range(3)]
    midpoint = [(min_v[i] + max_v[i]) / 2.0 for i in range(3)]
    return {"min": min_v, "max": max_v, "size": size_v, "midpoint": midpoint}


def _derive_link_properties(
    *,
    source_stage: Usd.Stage,
    bbox_cache: UsdGeom.BBoxCache,
    link_name: str,
    component_names: list[str],
) -> dict[str, Any]:
    layout = _LINK_LAYOUT_BY_NAME[link_name]
    link_translate = tuple(float(v) for v in layout["translate"])
    fallback_dimensions = tuple(float(v) for v in layout["fallback_dimensions"])
    fallback_mass = float(layout["fallback_mass"])
    component_items = [_component_evidence(source_stage, bbox_cache, name) for name in component_names]
    source_prims = [item["source_prim"] for item in component_items if item.get("exists")]
    density_sources = [density for item in component_items for density in item.get("density_sources", [])]
    union = _union_bounds(component_items)

    source_mass_items = [item.get("valid_source_mass_properties") for item in component_items if item.get("valid_source_mass_properties")]
    fallback_reasons: list[str] = []

    if source_mass_items:
        mass = sum(float(item["mass"]) for item in source_mass_items)
        world_com = _weighted_average(
            [list(item["center_of_mass_world"]) for item in source_mass_items],
            [float(item["mass"]) for item in source_mass_items],
        )
        dimensions = _clamp_dimensions(union["size"] if union else fallback_dimensions)
        diagonal_inertia = tuple(
            sum(float(item["diagonal_inertia"][i]) for item in source_mass_items) for i in range(3)
        )
        mass_method = "source_usd_valid_mass_properties"
        mass_provenance = "derived"
        com_provenance = "derived"
        inertia_provenance = "derived"
    elif union and density_sources:
        component_masses: list[float] = []
        component_midpoints: list[list[float]] = []
        for item in component_items:
            bounds = item.get("bounds")
            densities = item.get("density_sources", [])
            if not bounds or not densities:
                continue
            dimensions_i = _clamp_dimensions(bounds["size"])
            volume = dimensions_i[0] * dimensions_i[1] * dimensions_i[2]
            density = sum(float(src["density"]) for src in densities) / len(densities)
            component_mass = max(volume * density * MASS_FILL_RATIO, MIN_MASS_KG / max(len(component_items), 1))
            component_masses.append(component_mass)
            component_midpoints.append(bounds["midpoint"])
        mass = max(sum(component_masses), MIN_MASS_KG)
        world_com = _weighted_average(component_midpoints, component_masses) if component_masses else union["midpoint"]
        dimensions = _clamp_dimensions(union["size"])
        diagonal_inertia = _box_inertia(mass, dimensions)
        mass_method = "inferred_from_usd_density_bounds_and_fill_ratio"
        mass_provenance = "inferred"
        com_provenance = "inferred"
        inertia_provenance = "inferred"
        fallback_reasons.append(
            "source USD provides density/bounds but component mass, centerOfMass, or diagonalInertia is zero/non-finite"
        )
    else:
        dimensions = fallback_dimensions
        mass = fallback_mass
        world_com = [link_translate[0], link_translate[1], link_translate[2]]
        diagonal_inertia = _box_inertia(mass, dimensions)
        mass_method = "inferred_manual_fallback_layout"
        mass_provenance = "inferred"
        com_provenance = "inferred"
        inertia_provenance = "inferred"
        fallback_reasons.append("no usable source component bounds/density for this control-rig link")

    collider_center_world = union["midpoint"] if union else list(world_com)
    collider_center_local = [float(collider_center_world[i]) - link_translate[i] for i in range(3)]
    center_of_mass_local = [float(world_com[i]) - link_translate[i] for i in range(3)]
    dimensions = _clamp_dimensions(dimensions)

    return {
        "link_name": link_name,
        "translate": {
            "value": list(link_translate),
            "unit": "m",
            "frame": "control_rig_parent",
            "provenance": "inferred",
            "method": "control_rig_layout_preserved_from_existing_overlay",
        },
        "component_source_prims": source_prims,
        "component_evidence": component_items,
        "density_sources": density_sources,
        "source_bounds_union": (
            {"min": union["min"], "max": union["max"], "size": union["size"], "midpoint": union["midpoint"], "unit": "m", "provenance": "derived"}
            if union
            else None
        ),
        "mass": {
            "value": float(mass),
            "unit": "kg",
            "provenance": mass_provenance,
            "method": mass_method,
        },
        "center_of_mass": {
            "value": center_of_mass_local,
            "unit": "m",
            "frame": "link_local",
            "provenance": com_provenance,
            "method": mass_method if com_provenance == "inferred" else "source_usd_valid_mass_properties",
        },
        "diagonal_inertia": {
            "value": [float(v) for v in diagonal_inertia],
            "unit": "kg*m^2",
            "provenance": inertia_provenance,
            "method": "box_approximation_from_mass_and_collider_dimensions" if inertia_provenance == "inferred" else "source_usd_valid_mass_properties",
        },
        "collider": {
            "type": "box",
            "dimensions": list(dimensions),
            "center": collider_center_local,
            "unit": "m",
            "frame": "link_local",
            "provenance": "inferred" if union else "inferred",
            "method": "source_usd_bounds_union_simplified_box" if union else "manual_fallback_box",
            "fallback_reason": None if union else "no source bounds available for simplified collider",
        },
        "fallback_reasons": fallback_reasons,
    }


def build_physical_property_provenance(source_usd: str | Path, output_usd: str | Path) -> dict[str, Any]:
    """Build CAD/USD-grounded plus disclosed-fallback physical-property provenance."""
    source_path = Path(source_usd)
    output_path = Path(output_usd)
    if not source_path.is_file():
        raise FileNotFoundError(source_path)
    source_stage = Usd.Stage.Open(str(source_path))
    if not source_stage:
        raise RuntimeError(f"Unable to open source USD: {source_path}")
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )
    by_link = _component_bindings_by_link()
    link_properties = {
        str(spec["name"]): _derive_link_properties(
            source_stage=source_stage,
            bbox_cache=bbox_cache,
            link_name=str(spec["name"]),
            component_names=by_link.get(str(spec["name"]), []),
        )
        for spec in _LINK_LAYOUTS
    }
    joint_properties: dict[str, Any] = {}
    for joint_name, body0, body1, axis, origin in _JOINT_SPECS:
        joint_properties[joint_name] = {
            "parent_link": body0,
            "child_link": body1,
            "axis": {"value": axis, "provenance": "inferred", "method": "semantic_arm_contract_pitch_roll_yaw"},
            "origin": {"value": list(origin), "unit": "m", "frame": "control_rig_parent", "provenance": "inferred", "method": "control_rig_layout"},
            "limits": {"lower": -90.0, "upper": 90.0, "unit": "deg", "provenance": "inferred", "method": "safe_initial_articulation_range"},
            "drive": {
                "type": {"value": _AUTHORED_DRIVE_SETTINGS["type"], "provenance": "inferred", "method": "initial_force_drive_contract"},
                "stiffness": {"value": _AUTHORED_DRIVE_SETTINGS["stiffness"], "provenance": "inferred", "method": "initial_controller_tuning"},
                "damping": {"value": _AUTHORED_DRIVE_SETTINGS["damping"], "provenance": "inferred", "method": "initial_controller_tuning"},
                "max_force": {"value": _AUTHORED_DRIVE_SETTINGS["max_force"], "unit": "N*m", "provenance": "inferred", "method": "initial_controller_tuning"},
            },
        }

    audit_rows = []
    for link_name, props in link_properties.items():
        audit_rows.append(
            {
                "link_name": link_name,
                "component_source_prims": props["component_source_prims"],
                "density_source_count": len(props["density_sources"]),
                "mass_provenance": props["mass"]["provenance"],
                "mass_method": props["mass"]["method"],
                "collider_method": props["collider"]["method"],
                "fallback_reasons": props["fallback_reasons"],
                "controller_status": "authored_drive_in_usd_runtime_override_disclosed_separately",
            }
        )

    return {
        "schema_version": 1,
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "claim_boundary": CLAIM_BOUNDARY,
        "source_usd": str(source_path),
        "output_usd": str(output_path),
        "source_asset_hash": _sha256_file(source_path),
        "source_asset_hash_scope": "sha256_source_usd_file_bytes",
        "resolved_asset_graph_hash": None,
        "resolved_asset_graph_hash_reason": "not_computed_without_additional_dependency_or_asset_graph_policy",
        "units": {"length": "m", "mass": "kg", "inertia": "kg*m^2", "density": "kg/m^3"},
        "link_properties": link_properties,
        "joint_properties": joint_properties,
        "controller_properties": {
            "authored_drive": {
                "settings": _AUTHORED_DRIVE_SETTINGS,
                "provenance": "inferred",
                "method": "initial_usd_drive_tuning_preserved_and_disclosed",
            },
            "runtime_override_policy": _RUNTIME_CONTROLLER_OVERRIDE_POLICY,
        },
        "visual_only_physics_policy": {
            "policy": (
                "Referenced fixed and moving visual subtrees are stripped of Physics*/Physx* applied "
                "API schemas, and typed Physics*/Physx* helper prims are deactivated in the derived overlay."
            ),
            "reason": "The source asset may contain collision/rigid-body/mass schemas, but these visual references are not part of the control-rig physics model.",
            "source_usd_mutated": False,
        },
        "visual_only_features": [f"{FIXED_HAND_FEATURE}.pos"],
        "wrist_attached_visual_features": [f"{FIXED_HAND_FEATURE}.pos"],
        "wrist_attached_visual_component_count": 1,
        "wrist_attached_visual_components": [_AMAZINGHAND_VISUAL_COMPONENT],
        "source_visual_meter_normalization": SOURCE_VISUAL_METER_NORMALIZATION,
        "out_of_scope": ["hand_physics", "lerobot_ros_ui", "object_grasp_task"],
        "per_link_audit_table": audit_rows,
    }


def write_physical_property_provenance(
    source_usd: str | Path,
    output_usd: str | Path,
    physical_properties_json: str | Path,
) -> dict[str, Any]:
    payload = build_physical_property_provenance(source_usd, output_usd)
    path = Path(physical_properties_json)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload


def _define_link(stage: Usd.Stage, name: str, properties: dict[str, Any]) -> Sdf.Path:
    link_path = Sdf.Path(f"{CONTROL_RIG_PRIM_PATH}/Links/{name}")
    translate = properties["translate"]["value"]
    link = UsdGeom.Xform.Define(stage, link_path)
    link.AddTranslateOp().Set(_vec3(translate))
    link.GetPrim().CreateAttribute("superarm:physicalProvenance", Sdf.ValueTypeNames.String).Set(properties["mass"]["method"])
    UsdPhysics.RigidBodyAPI.Apply(link.GetPrim()).CreateRigidBodyEnabledAttr(True)
    mass_api = UsdPhysics.MassAPI.Apply(link.GetPrim())
    mass_api.CreateMassAttr(float(properties["mass"]["value"]))
    mass_api.CreateCenterOfMassAttr(_vec3(properties["center_of_mass"]["value"]))
    mass_api.CreateDiagonalInertiaAttr(_vec3(properties["diagonal_inertia"]["value"]))

    collider = properties["collider"]
    cube = UsdGeom.Cube.Define(stage, link_path.AppendChild("collision_visual"))
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(_vec3(collider["center"]))
    cube.AddScaleOp().Set(_vec3(collider["dimensions"]))
    cube.GetPrim().CreateAttribute("superarm:colliderProvenance", Sdf.ValueTypeNames.String).Set(collider["method"])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
    return link_path


def _hide_fixed_arm_visual(stage: Usd.Stage, component_name: str) -> None:
    prim = stage.OverridePrim(Sdf.Path(f"{_FIXED_VISUAL_ROBOT_ROOT}/{component_name}"))
    UsdGeom.Imageable(prim).CreateVisibilityAttr(UsdGeom.Tokens.invisible)


def _add_moving_visual_reference(
    stage: Usd.Stage,
    *,
    source_path: Path,
    output_path: Path,
    link_name: str,
    component_name: str,
    link_translate: tuple[float, float, float],
) -> dict[str, Any]:
    link_path = Sdf.Path(f"{CONTROL_RIG_PRIM_PATH}/Links/{link_name}")
    visual_root = UsdGeom.Xform.Define(stage, link_path.AppendChild("source_visuals"))
    if not visual_root.GetOrderedXformOps():
        visual_root.AddTranslateOp().Set(_vec3((-link_translate[0], -link_translate[1], -link_translate[2])))
        visual_root.AddScaleOp().Set(
            Gf.Vec3f(
                SOURCE_VISUAL_METER_NORMALIZATION,
                SOURCE_VISUAL_METER_NORMALIZATION,
                SOURCE_VISUAL_METER_NORMALIZATION,
            )
        )
    ref_prim = UsdGeom.Xform.Define(stage, visual_root.GetPath().AppendChild(component_name))
    reference_path = os.path.relpath(source_path.resolve(), output_path.parent.resolve())
    ref_prim.GetPrim().GetReferences().AddReference(reference_path, Sdf.Path(f"{_SOURCE_ROBOT_ROOT}/{component_name}"))
    return _strip_physics_from_visual_subtree(stage, ref_prim.GetPath())


def _define_revolute_joint(
    stage: Usd.Stage,
    name: str,
    body0: Sdf.Path,
    body1: Sdf.Path,
    axis: str,
    origin: tuple[float, float, float],
    body0_translate: tuple[float, float, float] | list[float],
    body1_translate: tuple[float, float, float] | list[float],
) -> Sdf.Path:
    joint_path = Sdf.Path(f"{CONTROL_RIG_PRIM_PATH}/Joints/{name}")
    joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([body0])
    joint.CreateBody1Rel().SetTargets([body1])
    joint.CreateAxisAttr(axis)
    # USD physics joint anchors are local to each rigid body, not world-space.
    joint.CreateLocalPos0Attr(
        _vec3(
            (
                origin[0] - float(body0_translate[0]),
                origin[1] - float(body0_translate[1]),
                origin[2] - float(body0_translate[2]),
            )
        )
    )
    joint.CreateLocalPos1Attr(
        _vec3(
            (
                origin[0] - float(body1_translate[0]),
                origin[1] - float(body1_translate[1]),
                origin[2] - float(body1_translate[2]),
            )
        )
    )
    joint.CreateLowerLimitAttr(-90.0)
    joint.CreateUpperLimitAttr(90.0)
    joint.CreateCollisionEnabledAttr(False)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr(str(_AUTHORED_DRIVE_SETTINGS["type"]))
    drive.CreateTargetPositionAttr(float(_AUTHORED_DRIVE_SETTINGS["target_position"]))
    drive.CreateTargetVelocityAttr(float(_AUTHORED_DRIVE_SETTINGS["target_velocity"]))
    drive.CreateStiffnessAttr(float(_AUTHORED_DRIVE_SETTINGS["stiffness"]))
    drive.CreateDampingAttr(float(_AUTHORED_DRIVE_SETTINGS["damping"]))
    drive.CreateMaxForceAttr(float(_AUTHORED_DRIVE_SETTINGS["max_force"]))
    return joint_path


def _write_mapping(
    mapping_json: Path,
    source_usd: Path,
    output_usd: Path,
    joint_paths: dict[str, str],
    physical_properties_json: Path,
) -> dict[str, Any]:
    control_features = [f"{name}.pos" for name in ARM_JOINT_NAMES] + [f"{FIXED_HAND_FEATURE}.pos"]
    per_feature = {f"{name}.pos": "articulation_bound" for name in ARM_JOINT_NAMES}
    per_feature[f"{FIXED_HAND_FEATURE}.pos"] = "wrist_attached_visual_only"
    feature_bindings: list[dict[str, Any]] = [
        {
            "feature": f"{name}.pos",
            "binding_status": "articulation_bound",
            "usd_joint_prim": joint_paths[name],
            "drive_api": "angular",
        }
        for name in ARM_JOINT_NAMES
    ]
    feature_bindings.append(
        {
            "feature": f"{FIXED_HAND_FEATURE}.pos",
            "binding_status": "wrist_attached_visual_only",
            "usd_joint_prim": None,
            "visual_prim": (
                f"{CONTROL_RIG_PRIM_PATH}/Links/wrist_fixed_hand_mount_link/source_visuals/"
                f"{_AMAZINGHAND_VISUAL_COMPONENT}"
            ),
            "reason": "AmazingHand is rigidly attached to the moving wrist link as visual-only geometry; no hand/finger drive or physics is authored.",
        }
    )
    payload = {
        "binding_status": "arm_articulation_bound_wrist_attached_hand_visual",
        "claim_boundary": CLAIM_BOUNDARY,
        "source_usd": str(source_usd),
        "output_usd": str(output_usd),
        "physical_properties_json": str(physical_properties_json),
        "root_prim": ROOT_PRIM_PATH,
        "visual_fixed_prim": VISUAL_FIXED_PRIM_PATH,
        "articulation_root": CONTROL_RIG_PRIM_PATH,
        "control_contract": control_features,
        "arm_joint_names": list(ARM_JOINT_NAMES),
        "visual_only_features": [f"{FIXED_HAND_FEATURE}.pos"],
        "wrist_attached_visual_features": [f"{FIXED_HAND_FEATURE}.pos"],
        "wrist_attached_visual_component_count": 1,
        "wrist_attached_visual_components": [_AMAZINGHAND_VISUAL_COMPONENT],
        "source_visual_meter_normalization": SOURCE_VISUAL_METER_NORMALIZATION,
        "bound_or_binding_pending_per_feature": per_feature,
        "feature_bindings": feature_bindings,
        "moving_visual_component_count": len(_MOVING_ARM_VISUAL_BINDINGS),
        "moving_visual_components": [component for _, component in _MOVING_ARM_VISUAL_BINDINGS],
        "next_step": "Run strict Isaac Sim target-driven physics validation against the authored ControlRig articulation.",
    }
    mapping_json.parent.mkdir(parents=True, exist_ok=True)
    mapping_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload


def create_controllable_arm_usd(
    source_usd: str | Path,
    output_usd: str | Path,
    mapping_json: str | Path,
    physical_properties_json: str | Path = DEFAULT_PHYSICAL_PROPERTIES_JSON,
) -> dict[str, Any]:
    source_path = Path(source_usd)
    output_path = Path(output_usd)
    mapping_path = Path(mapping_json)
    physical_path = Path(physical_properties_json)
    if not source_path.is_file():
        raise FileNotFoundError(source_path)
    if source_path.resolve() == output_path.resolve():
        raise ValueError("Refusing to overwrite source SimReady USD")

    physical_properties = build_physical_property_provenance(source_path, output_path)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    root = UsdGeom.Xform.Define(stage, ROOT_PRIM_PATH)
    stage.SetDefaultPrim(root.GetPrim())

    visual = UsdGeom.Xform.Define(stage, VISUAL_FIXED_PRIM_PATH)
    reference_path = os.path.relpath(source_path.resolve(), output_path.parent.resolve())
    visual.GetPrim().GetReferences().AddReference(reference_path)
    visual.GetPrim().CreateAttribute("superarm:role", Sdf.ValueTypeNames.String).Set("fixed_full_visual_reference")
    visual_strip_summaries = [_strip_physics_from_visual_subtree(stage, Sdf.Path(VISUAL_FIXED_PRIM_PATH))]

    control_rig = UsdGeom.Xform.Define(stage, CONTROL_RIG_PRIM_PATH)
    UsdPhysics.ArticulationRootAPI.Apply(control_rig.GetPrim())
    control_rig.GetPrim().CreateAttribute("superarm:role", Sdf.ValueTypeNames.String).Set("five_dof_arm_control_rig")
    control_rig.GetPrim().CreateAttribute("superarm:physicalPropertiesJson", Sdf.ValueTypeNames.String).Set(str(physical_path))
    UsdPhysics.Scene.Define(stage, f"{ROOT_PRIM_PATH}/PhysicsScene")

    link_paths: dict[str, Sdf.Path] = {}
    link_translates: dict[str, list[float]] = {}
    for spec in _LINK_LAYOUTS:
        name = str(spec["name"])
        link_properties = physical_properties["link_properties"][name]
        link_paths[name] = _define_link(stage, name, link_properties)
        link_translates[name] = link_properties["translate"]["value"]

    for link_name, component_name in _MOVING_ARM_VISUAL_BINDINGS:
        _hide_fixed_arm_visual(stage, component_name)
        visual_strip_summaries.append(
            _add_moving_visual_reference(
                stage,
                source_path=source_path,
                output_path=output_path,
                link_name=link_name,
                component_name=component_name,
                link_translate=tuple(link_translates[link_name]),
            )
        )

    hand_link_name, hand_component_name = _WRIST_ATTACHED_HAND_VISUAL_BINDING
    _hide_fixed_arm_visual(stage, hand_component_name)
    visual_strip_summaries.append(
        _add_moving_visual_reference(
            stage,
            source_path=source_path,
            output_path=output_path,
            link_name=hand_link_name,
            component_name=hand_component_name,
            link_translate=tuple(link_translates[hand_link_name]),
        )
    )

    joint_paths: dict[str, str] = {}
    for joint_name, body0, body1, axis, origin in _JOINT_SPECS:
        joint_paths[joint_name] = str(
            _define_revolute_joint(
                stage,
                joint_name,
                link_paths[body0],
                link_paths[body1],
                axis,
                origin,
                link_translates[body0],
                link_translates[body1],
            )
        )

    visual_strip_summary = _merge_visual_strip_summaries(visual_strip_summaries)
    physical_properties["visual_only_physics_stripping"] = visual_strip_summary

    stage.GetRootLayer().Save()
    physical_path.parent.mkdir(parents=True, exist_ok=True)
    physical_path.write_text(json.dumps(physical_properties, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    mapping = _write_mapping(mapping_path, source_path, output_path, joint_paths, physical_path)
    mapping["visual_only_physics_stripping"] = {
        "removed_api_schema_count": visual_strip_summary["removed_api_schema_count"],
        "deactivated_physics_type_count": visual_strip_summary["deactivated_physics_type_count"],
        "source_usd_mutated": False,
    }
    mapping_path.write_text(json.dumps(mapping, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return {
        "status": "created",
        "source_usd": str(source_path),
        "output_usd": str(output_path),
        "mapping_json": str(mapping_path),
        "physical_properties_json": str(physical_path),
        "visual_only_physics_stripping": mapping["visual_only_physics_stripping"],
        "articulation_root": CONTROL_RIG_PRIM_PATH,
        "arm_joint_count": len(ARM_JOINT_NAMES),
        "visual_only_features": mapping["visual_only_features"],
        "wrist_attached_visual_features": mapping["wrist_attached_visual_features"],
        "claim_boundary": CLAIM_BOUNDARY,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-usd", type=Path, default=DEFAULT_SOURCE_USD)
    parser.add_argument("--output-usd", type=Path, default=DEFAULT_OUTPUT_USD)
    parser.add_argument("--mapping-json", type=Path, default=DEFAULT_MAPPING_JSON)
    parser.add_argument("--physical-properties-json", type=Path, default=DEFAULT_PHYSICAL_PROPERTIES_JSON)
    args = parser.parse_args()
    report = create_controllable_arm_usd(
        args.source_usd,
        args.output_usd,
        args.mapping_json,
        args.physical_properties_json,
    )
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
