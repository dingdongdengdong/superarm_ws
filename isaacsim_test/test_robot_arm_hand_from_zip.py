"""Static checks for the fresh robot_arm_hand_package.zip Isaac pipeline.

Run with:
    python3 isaacsim_test/test_robot_arm_hand_from_zip.py
"""

from __future__ import annotations

import tempfile
import sys
import unittest
import xml.etree.ElementTree as ET
import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from isaacsim_test.isaacsim.graspable_hand_urdf import HAND_ACTUATED_JOINT_NAMES
from isaacsim_test.isaacsim.robot_arm_hand_from_zip import (
    ARM_HAND_ATTACHMENT_BODY0_PATH,
    ARM_HAND_ATTACHMENT_BODY1_PATH,
    CONNECTED_HAND_PRIM_PATH,
    HAND_MOUNT_LOCAL_XYZ,
    _compose_connected_reference_body_path,
    _prepare_source_artifacts,
    _author_connected_usd,
    _author_proxy_hand_usd,
    analyze_hand_mjcf,
    build_arm_joint_position_command,
    build_grasp_validation_object_specs,
    build_hand_grasp_position_command,
    build_hand_proxy_primitives,
    build_named_joint_position_command,
    extract_robot_arm_hand_package,
    sanitize_arm_urdf,
    sanitize_hand_mjcf,
)

ZIP_PATH = ROOT / "robot_arm_hand_package.zip"


class RobotArmHandFromZipTests(unittest.TestCase):
    def test_sanitize_arm_urdf_rewrites_meshes_and_removes_unsupported_includes(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            package_root = extract_robot_arm_hand_package(ZIP_PATH, Path(tmp) / "inputs")
            output_urdf = Path(tmp) / "robot_arm_hand_sanitized.urdf"

            report = sanitize_arm_urdf(package_root, output_urdf)

            self.assertEqual(report["status"], "PASS")
            self.assertEqual(report["xacro_includes_removed"], 3)
            self.assertGreaterEqual(report["mesh_reference_count"], 30)
            self.assertEqual(report["missing_meshes"], [])
            self.assertTrue(output_urdf.is_file())

            text = output_urdf.read_text(encoding="utf-8")
            self.assertNotIn("xacro:include", text)
            self.assertNotIn("package://", text)
            self.assertNotIn("<gazebo", text)
            self.assertNotIn("<transmission", text)

            root = ET.parse(output_urdf).getroot()
            mesh_paths = [
                mesh.attrib["filename"]
                for mesh in root.findall(".//mesh")
                if "filename" in mesh.attrib
            ]
            self.assertTrue(mesh_paths)
            for mesh_path in mesh_paths:
                self.assertTrue(Path(mesh_path).is_file(), mesh_path)

    def test_hand_mjcf_analysis_records_closed_loop_and_actuators(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            package_root = extract_robot_arm_hand_package(ZIP_PATH, Path(tmp) / "inputs")

            report = analyze_hand_mjcf(package_root)

            self.assertEqual(report["status"], "PASS")
            self.assertEqual(report["root_body"], "r_wrist_interface")
            self.assertEqual(report["position_actuator_count"], 8)
            self.assertEqual(report["equality_connect_count"], 20)
            self.assertEqual(report["missing_meshes"], [])
            self.assertIn("finger1_motor1", report["position_actuators"])
            self.assertIn("finger4_motor2", report["position_actuators"])

    def test_sanitize_hand_mjcf_merges_top_level_defaults_for_isaac_importer(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            package_root = extract_robot_arm_hand_package(ZIP_PATH, Path(tmp) / "inputs")
            output_mjcf = Path(tmp) / "hand_sanitized.xml"

            report = sanitize_hand_mjcf(package_root, output_mjcf)

            self.assertEqual(report["status"], "PASS")
            self.assertEqual(report["top_level_defaults_before"], 3)
            self.assertEqual(report["top_level_defaults_after"], 1)
            self.assertEqual(report["equality_connect_count"], 20)
            self.assertEqual(report["mesh_names_added"], 23)
            self.assertEqual(report["equality_connect_names_added"], 20)
            self.assertTrue(output_mjcf.is_file())

            root = ET.parse(output_mjcf).getroot()
            self.assertEqual(len(root.findall("default")), 1)
            self.assertEqual(len(root.findall("./equality/connect")), 20)
            for connect in root.findall("./equality/connect"):
                self.assertTrue(connect.attrib.get("name"))
            for mesh in root.findall("./asset/mesh"):
                self.assertTrue(mesh.attrib.get("name"))

    def test_zip_connection_contract_uses_measured_hand_mount_transform(self) -> None:
        self.assertEqual(HAND_MOUNT_LOCAL_XYZ, (0.005, -0.00014, 0.600003))

    def test_hand_proxy_spec_preserves_visible_wrist_palm_and_four_fingers(self) -> None:
        primitives = build_hand_proxy_primitives()

        self.assertEqual(len(primitives), 6)
        self.assertEqual(primitives[0]["name"], "wrist_interface")
        self.assertEqual(primitives[1]["name"], "palm")
        finger_names = [item["name"] for item in primitives if item["name"].startswith("finger_")]
        self.assertEqual(finger_names, ["finger_1", "finger_2", "finger_3", "finger_4"])
        for primitive in primitives:
            self.assertEqual(len(primitive["translate"]), 3)
            self.assertEqual(len(primitive["scale"]), 3)

    def test_arm_joint_command_updates_named_dofs_without_disturbing_others(self) -> None:
        result = build_arm_joint_position_command(
            current_positions=[10.0, 0.0, 0.0, 0.0, 20.0],
            dof_names=["fixed_a", "joint_rev_1", "joint_rev_2", "joint_rev_3", "joint_rev_4"],
            command=[0.1, -0.2, 0.3, -0.4],
        )

        self.assertEqual(result["controlled_indices"], [1, 2, 3, 4])
        self.assertEqual(result["controlled_joint_names"], [
            "joint_rev_1",
            "joint_rev_2",
            "joint_rev_3",
            "joint_rev_4",
        ])
        self.assertEqual(result["positions"], [10.0, 0.1, -0.2, 0.3, -0.4])

    def test_named_joint_command_updates_only_requested_joints(self) -> None:
        result = build_named_joint_position_command(
            current_positions=[0.0, 1.0, 2.0, 3.0],
            dof_names=["a", "finger1_motor1", "b", "finger1_motor2"],
            joint_targets={"finger1_motor1": 0.5, "finger1_motor2": 0.75},
        )

        self.assertEqual(result["controlled_indices"], [1, 3])
        self.assertEqual(result["controlled_joint_names"], ["finger1_motor1", "finger1_motor2"])
        self.assertEqual(result["target_values"], [0.5, 0.75])
        self.assertEqual(result["positions"], [0.0, 0.5, 2.0, 0.75])

    def test_hand_grasp_command_targets_all_generated_hand_dofs(self) -> None:
        dof_names = ["joint_rev_1", *HAND_ACTUATED_JOINT_NAMES, "joint_rev_4"]

        result = build_hand_grasp_position_command(
            current_positions=[0.0] * len(dof_names),
            dof_names=dof_names,
            grasp=1.0,
        )

        self.assertEqual(result["controlled_joint_names"], HAND_ACTUATED_JOINT_NAMES)
        self.assertEqual(result["controlled_indices"], list(range(1, 9)))
        self.assertTrue(all(value > 0.9 for value in result["target_values"][::2]))
        self.assertTrue(all(value > 1.0 for value in result["target_values"][1::2]))

    def test_grasp_validation_object_specs_are_small_rigid_trash_like_objects(self) -> None:
        specs = build_grasp_validation_object_specs()

        self.assertEqual([spec["name"] for spec in specs], ["small_trash_box"])
        spec = specs[0]
        self.assertEqual(spec["shape"], "cube")
        self.assertLessEqual(max(spec["scale"]), 0.03)
        self.assertGreater(spec["mass_kg"], 0.0)
        self.assertLess(spec["mass_kg"], 0.05)
        self.assertEqual(len(spec["local_xyz"]), 3)

    def test_proxy_hand_authors_physics_body_and_collision_shapes(self) -> None:
        from pxr import Usd, UsdPhysics

        with tempfile.TemporaryDirectory() as tmp:
            output_usd = Path(tmp) / "hand_proxy.usd"
            source_mjcf = Path(tmp) / "hand_sanitized.xml"
            source_mjcf.write_text("<mujoco />\n", encoding="utf-8")

            _author_proxy_hand_usd(
                hand_usd_path=output_usd,
                sanitized_mjcf_path=source_mjcf,
                reason="test fallback",
            )

            stage = Usd.Stage.Open(str(output_usd))
            root = stage.GetPrimAtPath("/AmazingHandProxy")
            self.assertTrue(root.HasAPI(UsdPhysics.RigidBodyAPI))
            for primitive in build_hand_proxy_primitives():
                child = stage.GetPrimAtPath(f"/AmazingHandProxy/{primitive['name']}")
                self.assertTrue(child.HasAPI(UsdPhysics.CollisionAPI), child.GetPath())

    def test_connected_joint_targets_real_rigid_body_paths(self) -> None:
        self.assertEqual(ARM_HAND_ATTACHMENT_BODY0_PATH, "/World/RobotArmHandFromZip/Arm/wrist_adapter_hand")
        self.assertEqual(ARM_HAND_ATTACHMENT_BODY1_PATH, CONNECTED_HAND_PRIM_PATH)

    def test_prepare_source_artifacts_generates_graspable_hand_urdf(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            args = argparse.Namespace(
                zip=str(ZIP_PATH),
                input_root=str(Path(tmp) / "inputs"),
                output_root=str(Path(tmp) / "outputs"),
            )

            _, report = _prepare_source_artifacts(args)

            generated = Path(report["graspable_hand_urdf_path"])
            self.assertTrue(generated.is_file())
            self.assertEqual(report["graspable_hand_urdf"]["status"], "PASS")
            self.assertEqual(
                report["graspable_hand_urdf"]["actuated_joint_names"],
                HAND_ACTUATED_JOINT_NAMES,
            )

    def test_reference_body_path_maps_imported_child_body_into_connected_hand(self) -> None:
        self.assertEqual(
            _compose_connected_reference_body_path(
                source_reference_prim_path="/amazinghand_graspable",
                source_body_path="/amazinghand_graspable/r_wrist_interface",
                connected_reference_prim_path=CONNECTED_HAND_PRIM_PATH,
            ),
            f"{CONNECTED_HAND_PRIM_PATH}/r_wrist_interface",
        )
        self.assertEqual(
            _compose_connected_reference_body_path(
                source_reference_prim_path="/AmazingHandProxy",
                source_body_path="/AmazingHandProxy",
                connected_reference_prim_path=CONNECTED_HAND_PRIM_PATH,
            ),
            CONNECTED_HAND_PRIM_PATH,
        )

    def test_connected_usd_reuses_existing_hand_translate_op_from_reference(self) -> None:
        from pxr import Gf, Usd, UsdGeom, UsdPhysics

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            arm_usd = tmp_path / "arm.usd"
            hand_usd = tmp_path / "hand.usd"
            connected_usd = tmp_path / "connected.usd"

            arm_stage = Usd.Stage.CreateNew(str(arm_usd))
            arm_root = UsdGeom.Xform.Define(arm_stage, "/ArmRoot")
            UsdPhysics.RigidBodyAPI.Apply(
                UsdGeom.Xform.Define(arm_stage, "/ArmRoot/wrist_adapter_hand").GetPrim()
            )
            arm_stage.SetDefaultPrim(arm_root.GetPrim())
            arm_stage.GetRootLayer().Save()

            hand_stage = Usd.Stage.CreateNew(str(hand_usd))
            hand_root = UsdGeom.Xform.Define(hand_stage, "/HandRoot")
            UsdGeom.Xformable(hand_root.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(1.0, 2.0, 3.0))
            UsdPhysics.RigidBodyAPI.Apply(hand_root.GetPrim())
            hand_stage.SetDefaultPrim(hand_root.GetPrim())
            hand_stage.GetRootLayer().Save()

            result = _author_connected_usd(
                arm_usd_path=arm_usd,
                arm_reference_prim_path="/ArmRoot",
                hand_usd_path=hand_usd,
                hand_reference_prim_path="/HandRoot",
                hand_fixed_joint_body_path=CONNECTED_HAND_PRIM_PATH,
                connected_usd_path=connected_usd,
                report={
                    "zip_source": "robot_arm_hand_package.zip",
                    "sanitized_urdf_path": "arm.urdf",
                    "hand_mjcf_path": "hand.xml",
                },
            )

            self.assertEqual(result["status"], "PASS")
            stage = Usd.Stage.Open(str(connected_usd))
            self.assertTrue(stage.GetPrimAtPath(CONNECTED_HAND_PRIM_PATH).IsValid())
            joint = UsdPhysics.FixedJoint.Get(stage, "/World/RobotArmHandFromZip/arm_to_hand_fixed_joint")
            self.assertEqual(
                [str(path) for path in joint.GetBody1Rel().GetTargets()],
                [CONNECTED_HAND_PRIM_PATH],
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
