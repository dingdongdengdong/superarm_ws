from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

from pxr import Usd, UsdGeom, UsdPhysics

SOURCE_USD = Path(
    "isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/"
    "fet005-grasp/echo_full_robot_arm_hand.usd"
)


def _finite_vec(value: object) -> bool:
    return value is not None and len(value) == 3 and all(math.isfinite(float(v)) for v in value)  # type: ignore[arg-type]


def _is_physics_schema_token(token: object) -> bool:
    text = str(token)
    return "Physics" in text or "Physx" in text


def _physics_schema_findings(stage: Usd.Stage, root_path: str, *, source_visuals_only: bool = False) -> list[dict[str, object]]:
    findings: list[dict[str, object]] = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(root_path.rstrip("/") + "/") and path != root_path:
            continue
        if source_visuals_only and "/source_visuals" not in path:
            continue
        applied = [schema for schema in prim.GetAppliedSchemas() if _is_physics_schema_token(schema)]
        type_name = prim.GetTypeName()
        physics_type = type_name if type_name and _is_physics_schema_token(type_name) else None
        if applied or physics_type:
            findings.append(
                {
                    "prim_path": path,
                    "applied_physics_schemas": applied,
                    "physics_type": physics_type,
                }
            )
    return findings


class SimReadyControllableArmOverlayTest(unittest.TestCase):
    def test_authors_five_dof_control_rig_and_wrist_attached_visual_hand_mapping(self) -> None:
        from isaacsim_test.simready_controllable_arm import (
            ARM_JOINT_NAMES,
            FIXED_HAND_FEATURE,
            CONTROL_RIG_PRIM_PATH,
            create_controllable_arm_usd,
        )

        with tempfile.TemporaryDirectory() as tmp:
            output_usd = Path(tmp) / "echo_full_robot_arm_hand_controllable.usda"
            mapping_json = Path(tmp) / "simready_controllable_prim_mapping.json"
            physical_json = Path(tmp) / "simready_controllable_physical_properties.json"

            report = create_controllable_arm_usd(SOURCE_USD, output_usd, mapping_json, physical_json)

            self.assertEqual(report["status"], "created")
            self.assertEqual(report["source_usd"], str(SOURCE_USD))
            self.assertEqual(report["output_usd"], str(output_usd))
            self.assertEqual(report["physical_properties_json"], str(physical_json))
            self.assertTrue(output_usd.is_file())
            self.assertTrue(mapping_json.is_file())
            self.assertTrue(physical_json.is_file())

            stage = Usd.Stage.Open(str(output_usd))
            self.assertTrue(stage)
            self.assertEqual(str(stage.GetDefaultPrim().GetPath()), "/echo_full_controllable")

            control_rig = stage.GetPrimAtPath(CONTROL_RIG_PRIM_PATH)
            self.assertTrue(control_rig.IsValid())
            self.assertTrue(control_rig.HasAPI(UsdPhysics.ArticulationRootAPI))

            visual_fixed = stage.GetPrimAtPath("/echo_full_controllable/VisualFixed")
            self.assertTrue(visual_fixed.IsValid())
            refs = visual_fixed.GetMetadata("references")
            self.assertIn("echo_full_robot_arm_hand.usd", str(refs))

            joint_prims = []
            for joint_name in ARM_JOINT_NAMES:
                prim = stage.GetPrimAtPath(f"{CONTROL_RIG_PRIM_PATH}/Joints/{joint_name}")
                self.assertTrue(prim.IsValid(), joint_name)
                self.assertEqual(prim.GetTypeName(), "PhysicsRevoluteJoint")
                joint_prims.append(prim)
                joint = UsdPhysics.RevoluteJoint(prim)
                local_pos0 = tuple(round(float(v), 6) for v in joint.GetLocalPos0Attr().Get())
                local_pos1 = tuple(round(float(v), 6) for v in joint.GetLocalPos1Attr().Get())
                self.assertNotEqual(
                    local_pos0,
                    (0.0, 0.0, 0.0),
                    f"{joint_name} body0 anchor should be local to parent body",
                )
                self.assertNotEqual(
                    local_pos1,
                    (0.0, 0.0, 0.0),
                    f"{joint_name} body1 anchor should be local to child body",
                )
                drive = UsdPhysics.DriveAPI.Get(prim, "angular")
                self.assertTrue(drive)
                self.assertEqual(drive.GetTypeAttr().Get(), "force")
                self.assertGreater(drive.GetStiffnessAttr().Get(), 0.0)
                self.assertGreater(drive.GetDampingAttr().Get(), 0.0)
                self.assertGreater(drive.GetMaxForceAttr().Get(), 0.0)

            self.assertEqual(len(joint_prims), 5)
            self.assertFalse(stage.GetPrimAtPath(f"{CONTROL_RIG_PRIM_PATH}/Joints/{FIXED_HAND_FEATURE}").IsValid())

            hidden_arm_prim = stage.GetPrimAtPath(
                "/echo_full_controllable/VisualFixed/"
                "tn____xaZ2Ve2pr5yw2tw0WSflDbf1/tn__v341_a4a4XfWAou7zcLveO/"
                "tn__DM4340rau35053Dv11_qI7BGHKrKpgzLslb2"
            )
            self.assertEqual(hidden_arm_prim.GetAttribute("visibility").Get(), "invisible")

            fixed_hand_prim = stage.GetPrimAtPath(
                "/echo_full_controllable/VisualFixed/"
                "tn____xaZ2Ve2pr5yw2tw0WSflDbf1/tn__v341_a4a4XfWAou7zcLveO/"
                "tn__AmazingHand_righthandv201_lQjM"
            )
            self.assertTrue(fixed_hand_prim.IsValid())
            self.assertEqual(fixed_hand_prim.GetAttribute("visibility").Get(), "invisible")
            self.assertEqual(
                _physics_schema_findings(stage, str(fixed_hand_prim.GetPath())),
                [],
                "Hidden AmazingHand fixed visual subtree must not carry active Physics*/Physx* APIs or typed physics prims",
            )

            wrist_hand_visual = stage.GetPrimAtPath(
                f"{CONTROL_RIG_PRIM_PATH}/Links/wrist_fixed_hand_mount_link/source_visuals/"
                "tn__AmazingHand_righthandv201_lQjM"
            )
            self.assertTrue(wrist_hand_visual.IsValid())
            self.assertIn("echo_full_robot_arm_hand.usd", str(wrist_hand_visual.GetMetadata("references")))
            self.assertEqual(
                _physics_schema_findings(stage, str(wrist_hand_visual.GetPath())),
                [],
                "Wrist-attached AmazingHand visual must remain geometry-only with no hand/finger physics",
            )

            moving_visual = stage.GetPrimAtPath(
                f"{CONTROL_RIG_PRIM_PATH}/Links/shoulder_pitch_link/source_visuals/"
                "tn__DM4340rau35053Dv11_qI7BGHKrKpgzLslb2"
            )
            self.assertTrue(moving_visual.IsValid())
            self.assertIn("echo_full_robot_arm_hand.usd", str(moving_visual.GetMetadata("references")))
            bounds_cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
                useExtentsHint=True,
            )
            moving_visuals_root = stage.GetPrimAtPath(f"{CONTROL_RIG_PRIM_PATH}/Links")
            moving_bounds = bounds_cache.ComputeWorldBound(moving_visuals_root).ComputeAlignedBox()
            self.assertLess(
                max(float(v) for v in moving_bounds.GetSize()),
                2.0,
                "Moving source visuals must preserve source meter normalization instead of appearing 1000x too large",
            )
            self.assertEqual(
                _physics_schema_findings(stage, f"{CONTROL_RIG_PRIM_PATH}/Links", source_visuals_only=True),
                [],
                "Moving source visuals must be geometry-only; physics belongs only to authored ControlRig links/colliders",
            )

            allowed_link_physics: dict[str, set[str]] = {
                f"{CONTROL_RIG_PRIM_PATH}/Links/{name}": {"PhysicsRigidBodyAPI", "PhysicsMassAPI"}
                for name in (
                    "base_link",
                    "shoulder_pitch_link",
                    "shoulder_roll_link",
                    "upper_arm_link",
                    "forearm_link",
                    "wrist_fixed_hand_mount_link",
                )
            }
            allowed_link_physics.update(
                {
                    f"{CONTROL_RIG_PRIM_PATH}/Links/{name}/collision_visual": {"PhysicsCollisionAPI"}
                    for name in (
                        "base_link",
                        "shoulder_pitch_link",
                        "shoulder_roll_link",
                        "upper_arm_link",
                        "forearm_link",
                        "wrist_fixed_hand_mount_link",
                    )
                }
            )
            unexpected_physics = []
            for finding in _physics_schema_findings(stage, f"{CONTROL_RIG_PRIM_PATH}/Links"):
                prim_path = str(finding["prim_path"])
                applied = set(finding["applied_physics_schemas"])  # type: ignore[arg-type]
                expected = allowed_link_physics.get(prim_path)
                if expected != applied or finding["physics_type"]:
                    unexpected_physics.append(finding)
            self.assertEqual(unexpected_physics, [])
            self.assertEqual(
                set(allowed_link_physics),
                {str(finding["prim_path"]) for finding in _physics_schema_findings(stage, f"{CONTROL_RIG_PRIM_PATH}/Links")},
            )

            mapping = json.loads(mapping_json.read_text(encoding="utf-8"))
            self.assertEqual(mapping["binding_status"], "arm_articulation_bound_wrist_attached_hand_visual")
            self.assertEqual(mapping["articulation_root"], CONTROL_RIG_PRIM_PATH)
            self.assertEqual(mapping["physical_properties_json"], str(physical_json))
            self.assertGreater(mapping["visual_only_physics_stripping"]["removed_api_schema_count"], 0)
            self.assertFalse(mapping["visual_only_physics_stripping"]["source_usd_mutated"])
            self.assertIn("CAD/USD-grounded", mapping["claim_boundary"])
            self.assertEqual(mapping["visual_only_features"], [f"{FIXED_HAND_FEATURE}.pos"])
            self.assertEqual(mapping["wrist_attached_visual_features"], [f"{FIXED_HAND_FEATURE}.pos"])
            per_feature = mapping["bound_or_binding_pending_per_feature"]
            for joint_name in ARM_JOINT_NAMES:
                self.assertEqual(per_feature[f"{joint_name}.pos"], "articulation_bound")
            self.assertEqual(per_feature[f"{FIXED_HAND_FEATURE}.pos"], "wrist_attached_visual_only")

    def test_physical_properties_provenance_schema_and_usd_mass_values(self) -> None:
        from isaacsim_test.simready_controllable_arm import (
            ARM_JOINT_NAMES,
            CLAIM_BOUNDARY,
            CONTROL_RIG_PRIM_PATH,
            FIXED_HAND_FEATURE,
            create_controllable_arm_usd,
        )

        with tempfile.TemporaryDirectory() as tmp:
            output_usd = Path(tmp) / "echo_full_robot_arm_hand_controllable.usda"
            mapping_json = Path(tmp) / "simready_controllable_prim_mapping.json"
            physical_json = Path(tmp) / "simready_controllable_physical_properties.json"

            create_controllable_arm_usd(SOURCE_USD, output_usd, mapping_json, physical_json)
            provenance = json.loads(physical_json.read_text(encoding="utf-8"))
            stage = Usd.Stage.Open(str(output_usd))
            self.assertTrue(stage)

            self.assertEqual(provenance["schema_version"], 1)
            self.assertEqual(provenance["claim_boundary"], CLAIM_BOUNDARY)
            self.assertEqual(provenance["source_asset_hash_scope"], "sha256_source_usd_file_bytes")
            self.assertRegex(provenance["source_asset_hash"], r"^[0-9a-f]{64}$")
            self.assertIn("link_properties", provenance)
            self.assertIn("joint_properties", provenance)
            self.assertIn("controller_properties", provenance)
            self.assertEqual(provenance["visual_only_features"], [f"{FIXED_HAND_FEATURE}.pos"])
            self.assertEqual(provenance["wrist_attached_visual_features"], [f"{FIXED_HAND_FEATURE}.pos"])

            self.assertEqual(set(provenance["joint_properties"]), set(ARM_JOINT_NAMES))
            for joint_name, joint_props in provenance["joint_properties"].items():
                self.assertIn(joint_props["axis"]["provenance"], {"derived", "inferred", "missing"})
                drive = joint_props["drive"]
                for key in ("stiffness", "damping", "max_force"):
                    self.assertGreater(drive[key]["value"], 0.0, (joint_name, key))
                    self.assertIn(drive[key]["provenance"], {"derived", "inferred", "missing", "runtime_tuned"})

            controller = provenance["controller_properties"]
            self.assertEqual(controller["runtime_override_policy"]["provenance"], "runtime_tuned")
            for key in ("kp", "kd", "max_effort"):
                self.assertGreater(controller["runtime_override_policy"][key], 0.0)
            self.assertFalse(provenance["visual_only_physics_policy"]["source_usd_mutated"])
            self.assertGreater(provenance["visual_only_physics_stripping"]["removed_api_schema_count"], 0)

            link_properties = provenance["link_properties"]
            self.assertEqual(
                set(link_properties),
                {
                    "base_link",
                    "shoulder_pitch_link",
                    "shoulder_roll_link",
                    "upper_arm_link",
                    "forearm_link",
                    "wrist_fixed_hand_mount_link",
                },
            )
            moving_component_total = 0
            for link_name, props in link_properties.items():
                for field in ("mass", "center_of_mass", "diagonal_inertia"):
                    self.assertIn(props[field]["provenance"], {"derived", "inferred", "missing"}, (link_name, field))
                    self.assertIn("method", props[field], (link_name, field))
                self.assertGreater(props["mass"]["value"], 0.0, link_name)
                self.assertTrue(_finite_vec(props["center_of_mass"]["value"]), link_name)
                self.assertTrue(_finite_vec(props["diagonal_inertia"]["value"]), link_name)
                self.assertTrue(all(float(v) > 0.0 for v in props["diagonal_inertia"]["value"]), link_name)
                collider = props["collider"]
                self.assertEqual(collider["type"], "box")
                self.assertIn(collider["provenance"], {"derived", "inferred", "missing"})
                self.assertTrue(_finite_vec(collider["dimensions"]), link_name)
                self.assertTrue(all(float(v) > 0.0 for v in collider["dimensions"]), link_name)
                self.assertTrue(_finite_vec(collider["center"]), link_name)
                moving_component_total += len(props["component_source_prims"])

                prim = stage.GetPrimAtPath(f"{CONTROL_RIG_PRIM_PATH}/Links/{link_name}")
                self.assertTrue(prim.IsValid(), link_name)
                self.assertTrue(prim.HasAPI(UsdPhysics.RigidBodyAPI), link_name)
                mass_api = UsdPhysics.MassAPI(prim)
                self.assertGreater(mass_api.GetMassAttr().Get(), 0.0, link_name)
                self.assertTrue(_finite_vec(mass_api.GetCenterOfMassAttr().Get()), link_name)
                self.assertTrue(_finite_vec(mass_api.GetDiagonalInertiaAttr().Get()), link_name)
                self.assertTrue(all(float(v) > 0.0 for v in mass_api.GetDiagonalInertiaAttr().Get()), link_name)
                collider_prim = stage.GetPrimAtPath(f"{CONTROL_RIG_PRIM_PATH}/Links/{link_name}/collision_visual")
                self.assertTrue(collider_prim.HasAPI(UsdPhysics.CollisionAPI), link_name)

            self.assertEqual(moving_component_total, 17)
            self.assertEqual(provenance["wrist_attached_visual_component_count"], 1)
            audit_rows = provenance["per_link_audit_table"]
            self.assertEqual(len(audit_rows), len(link_properties))
            for row in audit_rows:
                self.assertIn("mass_provenance", row)
                self.assertIn("collider_method", row)
                self.assertIn("controller_status", row)


    def test_viewport_report_helpers_gate_missing_provenance_and_bad_trajectory(self) -> None:
        from isaacsim_test.run_simready_controllable_arm_viewport_debug import (
            _summarize_provenance,
            _trajectory_sanity,
        )

        missing_summary = _summarize_provenance({"status": "missing", "path": "missing.json"})
        self.assertEqual(missing_summary["status"], "missing")
        self.assertIsNone(missing_summary["schema_version"])

        good = _trajectory_sanity(
            [{"frame": 10, "readback_by_name": {"joint_a": 0.1, "joint_b": -0.2}}],
            ["joint_a", "joint_b"],
        )
        self.assertEqual(good["status"], "passed")
        self.assertTrue(good["finite"])
        self.assertEqual(good["sample_count"], 1)

        empty = _trajectory_sanity([], ["joint_a", "joint_b"])
        self.assertEqual(empty["status"], "failed")
        self.assertEqual(empty["reason"], "empty_trajectory")
        self.assertEqual(empty["sample_count"], 0)

        bad = _trajectory_sanity(
            [{"frame": 10, "readback_by_name": {"joint_a": float("nan"), "joint_b": 0.0}}],
            ["joint_a", "joint_b"],
        )
        self.assertEqual(bad["status"], "failed")
        self.assertFalse(bad["finite"])

    def test_generator_never_overwrites_source_usd(self) -> None:
        from isaacsim_test.simready_controllable_arm import create_controllable_arm_usd

        with tempfile.TemporaryDirectory() as tmp:
            mapping_json = Path(tmp) / "mapping.json"
            physical_json = Path(tmp) / "physical.json"
            with self.assertRaisesRegex(ValueError, "Refusing to overwrite source"):
                create_controllable_arm_usd(SOURCE_USD, SOURCE_USD, mapping_json, physical_json)


if __name__ == "__main__":
    unittest.main()
