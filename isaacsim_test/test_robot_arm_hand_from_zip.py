"""Static checks for the fresh robot_arm_hand_package.zip Isaac pipeline.

Run with:
    python3 isaacsim_test/test_robot_arm_hand_from_zip.py
"""

from __future__ import annotations

import inspect
import os
import tempfile
import sys
import unittest
from unittest import mock
import xml.etree.ElementTree as ET
import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from isaacsim_test.isaacsim.graspable_hand_urdf import HAND_ACTUATED_JOINT_NAMES
from isaacsim_test.isaacsim import robot_arm_hand_from_zip as rah
from isaacsim_test.isaacsim.robot_arm_hand_from_zip import (
    ARM_HAND_ATTACHMENT_BODY0_PATH,
    ARM_HAND_ATTACHMENT_BODY1_PATH,
    CONNECTED_HAND_PRIM_PATH,
    HAND_MOUNT_LOCAL_XYZ,
    LIFT_RETAIN_ARM_TARGET,
    _compose_connected_reference_body_path,
    _prepare_source_artifacts,
    _author_connected_usd,
    _author_proxy_hand_usd,
    analyze_hand_mjcf,
    build_arm_joint_position_command,
    build_grasp_validation_object_specs,
    build_grasp_transform_diagnostic_paths,
    build_hand_contact_proxy_specs,
    build_hand_grasp_position_command,
    build_lift_retain_joint_target_sequence,
    build_visible_grasp_diagnostic_capture_specs,
    build_hand_proxy_primitives,
    grasp_object_reset_anchor_path,
    build_named_joint_position_command,
    build_single_finger_two_link_position_command,
    extract_robot_arm_hand_package,
    _add_reference_wrapper_usd,
    _evaluate_lift_retain_status,
    sanitize_arm_urdf,
    sanitize_hand_mjcf,
)

ZIP_PATH = ROOT / "robot_arm_hand_package.zip"


class RobotArmHandFromZipTests(unittest.TestCase):
    def _hand_grasp_command(
        self,
        *,
        dof_names: list[str],
        grasp: float,
        grasp_type: str = "wrap",
    ) -> dict[str, object]:
        self.assertIn(
            "grasp_type",
            inspect.signature(build_hand_grasp_position_command).parameters,
            "build_hand_grasp_position_command must accept grasp_type for preshape control",
        )
        return build_hand_grasp_position_command(
            current_positions=[0.0] * len(dof_names),
            dof_names=dof_names,
            grasp=grasp,
            grasp_type=grasp_type,
        )

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

    def test_hand_grasp_command_defaults_to_wrap_preshape(self) -> None:
        dof_names = ["joint_rev_1", *HAND_ACTUATED_JOINT_NAMES, "joint_rev_4"]

        default_result = build_hand_grasp_position_command(
            current_positions=[0.0] * len(dof_names),
            dof_names=dof_names,
            grasp=1.0,
        )
        wrap_result = self._hand_grasp_command(
            dof_names=dof_names,
            grasp=1.0,
            grasp_type="wrap",
        )

        self.assertEqual(default_result["target_values"], wrap_result["target_values"])

    def test_hand_grasp_command_threads_pinch_preshape_targets(self) -> None:
        dof_names = ["joint_rev_1", *HAND_ACTUATED_JOINT_NAMES, "joint_rev_4"]

        result = self._hand_grasp_command(
            dof_names=dof_names,
            grasp=1.0,
            grasp_type="pinch",
        )
        targets = dict(zip(result["controlled_joint_names"], result["target_values"]))

        self.assertGreater(targets["finger1_motor1"], targets["finger2_motor1"])
        self.assertGreater(targets["finger4_motor1"], targets["finger3_motor1"])

    def test_hand_grasp_command_reports_servo_targets_and_motor_contract(self) -> None:
        dof_names = ["joint_rev_1", *HAND_ACTUATED_JOINT_NAMES, "joint_rev_4"]

        result = self._hand_grasp_command(
            dof_names=dof_names,
            grasp=1.0,
            grasp_type="wrap",
        )

        self.assertEqual(result["controlled_servo_ids"], list(range(1, 9)))
        self.assertEqual(result["motor_contract"]["servo_model"], "SCS0009")
        self.assertEqual(result["joint_to_servo_id"]["finger1_motor1"], 1)
        self.assertAlmostEqual(
            result["servo_targets_rad"][1],
            result["motor_targets"]["finger1_motor1"]["servo_target_rad"],
        )
        self.assertGreater(result["servo_targets_rad"][1], 1.0)
        self.assertLess(result["servo_targets_rad"][2], -1.0)

    def test_hand_grasp_command_rejects_invalid_preshape(self) -> None:
        dof_names = ["joint_rev_1", *HAND_ACTUATED_JOINT_NAMES, "joint_rev_4"]

        with self.assertRaisesRegex(ValueError, "wrap.*pinch.*wide"):
            self._hand_grasp_command(
                dof_names=dof_names,
                grasp=0.5,
                grasp_type="claw",
            )

    def test_single_finger_two_link_command_targets_only_one_motor_pair(self) -> None:
        dof_names = ["joint_rev_1", *HAND_ACTUATED_JOINT_NAMES, "joint_rev_4"]
        current = [0.1] * len(dof_names)

        result = build_single_finger_two_link_position_command(
            current_positions=current,
            dof_names=dof_names,
            finger_index=3,
            motor1=0.78,
            motor2=0.96,
        )

        self.assertEqual(
            result["controlled_joint_names"],
            ["finger3_motor1", "finger3_motor2"],
        )
        self.assertEqual(
            result["controlled_indices"],
            [dof_names.index("finger3_motor1"), dof_names.index("finger3_motor2")],
        )
        self.assertEqual(result["target_values"], [0.78, 0.96])
        for index, name in enumerate(dof_names):
            if name == "finger3_motor1":
                self.assertEqual(result["positions"][index], 0.78)
            elif name == "finger3_motor2":
                self.assertEqual(result["positions"][index], 0.96)
            else:
                self.assertEqual(result["positions"][index], 0.1)

        with self.assertRaises(ValueError):
            build_single_finger_two_link_position_command(
                current_positions=current,
                dof_names=dof_names,
                finger_index=5,
                motor1=0.0,
                motor2=0.0,
            )

    def test_finger_motor_frame_report_carries_servo_pair_offsets_and_anchor(self) -> None:
        self.assertTrue(hasattr(rah, "build_finger_motor_frame_report"))

        report = rah.build_finger_motor_frame_report(
            finger_index=2,
            target_positions_rad=[0.78, 0.96],
            achieved_positions_rad=[0.77, 0.95],
            link_translation_deltas_m={"proximal": 0.001, "distal": 0.041},
        )

        self.assertEqual(report["finger_index"], 2)
        self.assertEqual(report["servo_ids"], [3, 4])
        self.assertEqual(report["mjcf_anchor_body"], "custom_servo_horn_2")
        self.assertEqual(report["base_xyz_m"], [-0.00505, 0.0011, 0.06456])
        self.assertEqual(
            [motor["joint_name"] for motor in report["motors"]],
            ["finger2_motor1", "finger2_motor2"],
        )
        self.assertEqual(report["motors"][0]["servo_id"], 3)
        self.assertEqual(report["motors"][0]["target_rad"], 0.78)
        self.assertEqual(report["motors"][1]["achieved_rad"], 0.95)
        self.assertEqual(report["link_translation_deltas_m"]["distal"], 0.041)

    def test_hand_preshape_command_supports_single_finger_pinch_and_wrap(self) -> None:
        self.assertTrue(hasattr(rah, "build_hand_preshape_position_command"))
        dof_names = ["joint_rev_1", *HAND_ACTUATED_JOINT_NAMES, "joint_rev_4"]
        current = [0.1] * len(dof_names)

        single = rah.build_hand_preshape_position_command(
            current_positions=current,
            dof_names=dof_names,
            preshape="single_finger",
            amount=1.0,
            finger_index=3,
        )
        self.assertEqual(
            single["controlled_joint_names"],
            ["finger3_motor1", "finger3_motor2"],
        )
        self.assertEqual(single["preshape"], "single_finger")
        self.assertEqual(single["active_fingers"], [3])
        self.assertEqual(single["controlled_servo_ids"], [5, 6])
        self.assertEqual(set(single["servo_targets_rad"]), {5, 6})

        pinch = rah.build_hand_preshape_position_command(
            current_positions=current,
            dof_names=dof_names,
            preshape="pinch",
            amount=1.0,
        )
        self.assertEqual(
            pinch["controlled_joint_names"],
            ["finger1_motor1", "finger1_motor2", "finger4_motor1", "finger4_motor2"],
        )
        self.assertEqual(pinch["active_fingers"], [1, 4])
        self.assertTrue(all(value > 0.9 for value in pinch["target_values"]))

        wrap = rah.build_hand_preshape_position_command(
            current_positions=current,
            dof_names=dof_names,
            preshape="wrap",
            amount=0.5,
        )
        self.assertEqual(wrap["controlled_joint_names"], HAND_ACTUATED_JOINT_NAMES)
        self.assertEqual(wrap["active_fingers"], [1, 2, 3, 4])
        self.assertEqual(len(wrap["target_values"]), 8)

        with self.assertRaises(ValueError):
            rah.build_hand_preshape_position_command(
                current_positions=current,
                dof_names=dof_names,
                preshape="single_finger",
                amount=1.0,
            )

    def test_preshape_grasp_stage_specs_are_ordered_from_single_to_wrap(self) -> None:
        self.assertTrue(hasattr(rah, "build_preshape_grasp_validation_stage_specs"))
        specs = rah.build_preshape_grasp_validation_stage_specs()

        self.assertEqual([spec["label"] for spec in specs], ["single_finger", "pinch", "wrap"])
        self.assertEqual(specs[0]["active_fingers"], [1])
        self.assertEqual(specs[1]["active_fingers"], [1, 4])
        self.assertEqual(specs[2]["active_fingers"], [1, 2, 3, 4])
        self.assertEqual([spec["required_finger_proxy_count"] for spec in specs], [1, 2, 3])
        self.assertTrue(all(spec["amount"] == 1.0 for spec in specs))

    def test_shadow_allegro_reference_checklist_is_reference_only(self) -> None:
        self.assertTrue(hasattr(rah, "build_shadow_allegro_reference_checklist"))
        checklist = rah.build_shadow_allegro_reference_checklist()

        self.assertTrue(checklist["reference_only"])
        self.assertEqual(checklist["status"], "REFERENCE_ONLY")
        self.assertIn("Shadow Hand", checklist["reference_models"])
        self.assertIn("Allegro Hand", checklist["reference_models"])
        self.assertIn("collision_shape_layout", checklist["checkpoints"])
        self.assertIn("drive_stiffness_damping", checklist["checkpoints"])
        self.assertNotIn("replace_amazinghand", checklist["allowed_use"])

    def test_grasp_validation_object_specs_are_small_rigid_trash_like_objects(self) -> None:
        specs = build_grasp_validation_object_specs()

        self.assertEqual([spec["name"] for spec in specs], ["small_trash_box"])
        spec = specs[0]
        self.assertEqual(spec["shape"], "cube")
        self.assertLessEqual(max(spec["scale"]), 0.03)
        self.assertGreater(spec["mass_kg"], 0.0)
        self.assertLess(spec["mass_kg"], 0.05)
        self.assertEqual(len(spec["local_xyz"]), 3)
        self.assertGreaterEqual(spec["local_xyz"][2], 0.070)

    def test_hand_contact_proxy_specs_cover_palm_and_two_link_fingers(self) -> None:
        specs = build_hand_contact_proxy_specs()

        self.assertEqual(len(specs), 18)
        self.assertEqual(specs[0]["link_name"], "palm")
        self.assertEqual(specs[1]["name"], "palm_retention_shelf_proxy")
        self.assertEqual(
            {spec["name"] for spec in specs if spec["link_name"] == "palm"},
            {
                "palm_contact_proxy",
                "palm_retention_shelf_proxy",
                "palm_retention_left_wall_proxy",
                "palm_retention_right_wall_proxy",
                "palm_retention_front_lip_proxy",
                "palm_retention_back_lip_proxy",
            },
        )
        for finger_index in range(1, 5):
            names_for_finger = {
                spec["name"]
                for spec in specs
                if spec["link_name"].startswith(f"finger{finger_index}_")
            }
            self.assertEqual(
                names_for_finger,
                {
                    f"finger{finger_index}_proximal_contact_proxy",
                    f"finger{finger_index}_distal_contact_proxy",
                    f"finger{finger_index}_distal_tip_pad_proxy",
                },
            )
        for spec in specs:
            self.assertEqual(len(spec["local_xyz"]), 3)
            self.assertEqual(len(spec["scale"]), 3)
            self.assertTrue(all(value > 0.0 for value in spec["scale"]))

    def test_palm_contact_proxy_is_thin_backstop_behind_grasp_object(self) -> None:
        object_spec = build_grasp_validation_object_specs()[0]
        palm_spec = build_hand_contact_proxy_specs()[0]

        object_back_y = object_spec["local_xyz"][1] - object_spec["scale"][1] / 2.0
        palm_front_y = palm_spec["local_xyz"][1] + palm_spec["scale"][1] / 2.0
        self.assertLessEqual(palm_front_y, object_back_y)
        self.assertLessEqual(palm_spec["scale"][1], 0.035)
        self.assertGreaterEqual(
            palm_spec["local_xyz"][2] + palm_spec["scale"][2] / 2.0,
            object_spec["local_xyz"][2] + object_spec["scale"][2] / 2.0,
        )

    def test_preshape_object_reset_stability_flags_far_settled_objects(self) -> None:
        self.assertTrue(
            hasattr(rah, "_evaluate_preshape_object_reset_stability"),
            "preshape validation must report when the reset object drifts away before command",
        )

        stable = rah._evaluate_preshape_object_reset_stability(  # type: ignore[attr-defined]
            object_reset_world_xyz=(0.0, 0.0, 0.0),
            settled_object_world_xyz=(0.01, 0.02, 0.03),
        )
        self.assertEqual(stable["object_reset_status"], "PASS")
        self.assertTrue(stable["object_reset_stable"])
        self.assertLess(stable["object_reset_drift_m"], stable["object_reset_max_drift_m"])

        escaped = rah._evaluate_preshape_object_reset_stability(  # type: ignore[attr-defined]
            object_reset_world_xyz=(-0.035, 0.311, 0.673),
            settled_object_world_xyz=(-0.609, 1.182, 0.099),
        )
        self.assertEqual(escaped["object_reset_status"], "WARN")
        self.assertFalse(escaped["object_reset_stable"])
        self.assertGreater(escaped["object_reset_drift_m"], 1.0)

    def test_target_error_threshold_flags_wrap_finger3_underreach(self) -> None:
        self.assertTrue(
            hasattr(rah, "_target_errors_within_threshold"),
            "runtime validation should use one explicit target-error threshold helper",
        )

        self.assertTrue(
            rah._target_errors_within_threshold(  # type: ignore[attr-defined]
                [0.0017, 0.0013, 0.1540],
                threshold_rad=0.30,
            )
        )
        self.assertFalse(
            rah._target_errors_within_threshold(  # type: ignore[attr-defined]
                [0.0017, 0.0013, 0.1540, 0.3487],
                threshold_rad=0.30,
            )
        )

    def test_single_finger_preshape_uses_finger_local_object_reset(self) -> None:
        single = rah.build_preshape_grasp_validation_stage_specs()[0]

        self.assertEqual(single["label"], "single_finger")
        self.assertEqual(
            single["object_reset_anchor_path"],
            f"{CONNECTED_HAND_PRIM_PATH}/finger1_proximal",
        )
        self.assertEqual(tuple(single["object_reset_local_xyz"]), (0.0, 0.055, 0.010))
        self.assertLessEqual(rah.PRESHAPE_OBJECT_RESET_MAX_DRIFT_M, 0.06)

    def test_repair_urdf_import_visual_library_adds_empty_missing_sources(self) -> None:
        from pxr import Usd, UsdGeom

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            hand = tmp_path / "hand.usd"
            config = tmp_path / "configuration"
            config.mkdir()
            base = config / "hand_base.usd"

            hand_stage = Usd.Stage.CreateNew(str(hand))
            UsdGeom.Xform.Define(hand_stage, "/amazinghand_graspable")
            hand_stage.GetRootLayer().Save()

            base_stage = Usd.Stage.CreateNew(str(base))
            UsdGeom.Xform.Define(base_stage, "/amazinghand_graspable/palm/visuals")
            UsdGeom.Xform.Define(base_stage, "/amazinghand_graspable/finger1_base/visuals")
            existing = UsdGeom.Xform.Define(base_stage, "/visuals/r_wrist_interface").GetPrim()
            UsdGeom.Xform.Define(base_stage, f"{existing.GetPath()}/mesh")
            base_stage.GetRootLayer().Save()

            report = rah._repair_urdf_import_visual_library(hand)

            self.assertEqual(report["status"], "PASS")
            self.assertIn("/visuals/palm", report["created_placeholder_paths"])
            self.assertIn("/visuals/finger1_base", report["created_placeholder_paths"])
            repaired_stage = Usd.Stage.Open(str(base))
            self.assertTrue(repaired_stage.GetPrimAtPath("/visuals/palm").IsValid())
            self.assertTrue(repaired_stage.GetPrimAtPath("/visuals/finger1_base").IsValid())
            self.assertTrue(repaired_stage.GetPrimAtPath("/visuals/r_wrist_interface/mesh").IsValid())
            repaired_hand_stage = Usd.Stage.Open(str(hand))
            self.assertTrue(repaired_hand_stage.GetPrimAtPath("/visuals/palm").IsValid())

    def test_write_image_log_records_screenshots_and_contact_sheet(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            screenshot_root = Path(tmp)
            image_a = screenshot_root / "00_start.png"
            image_b = screenshot_root / "01_motion.png"
            contact_sheet = screenshot_root / "contact_sheet.png"
            image_a.write_bytes(b"a" * 3)
            image_b.write_bytes(b"b" * 4)
            contact_sheet.write_bytes(b"c" * 5)

            log_path = rah._write_image_log(
                screenshot_root=screenshot_root,
                image_paths=[image_a, image_b],
                contact_sheet_path=contact_sheet,
            )

            text = log_path.read_text(encoding="utf-8")
            self.assertIn("# Image Log", text)
            self.assertIn("- Image count: 3", text)
            self.assertIn("`00_start.png` - 3 bytes", text)
            self.assertIn("`01_motion.png` - 4 bytes", text)
            self.assertIn("`contact_sheet.png` - 5 bytes", text)

    def test_palm_retention_shelf_sits_under_grasp_object_for_lift_retain(self) -> None:
        object_spec = build_grasp_validation_object_specs()[0]
        shelf_spec = next(
            spec
            for spec in build_hand_contact_proxy_specs()
            if spec["name"] == "palm_retention_shelf_proxy"
        )

        object_bottom_z = object_spec["local_xyz"][2] - object_spec["scale"][2] / 2.0
        shelf_top_z = shelf_spec["local_xyz"][2] + shelf_spec["scale"][2] / 2.0
        self.assertLessEqual(shelf_top_z, object_bottom_z)
        self.assertLess(object_bottom_z - shelf_top_z, 0.012)
        self.assertGreater(object_spec["local_xyz"][2], 0.070)
        self.assertGreaterEqual(shelf_spec["scale"][0], object_spec["scale"][0] * 2.0)
        self.assertGreaterEqual(shelf_spec["scale"][1], object_spec["scale"][1] * 2.0)


    def test_palm_retention_cradle_surrounds_grasp_object_without_initial_overlap(self) -> None:
        object_spec = build_grasp_validation_object_specs()[0]
        palm_specs = {
            spec["name"]: spec
            for spec in build_hand_contact_proxy_specs()
            if spec["link_name"] == "palm"
        }

        object_half_x = object_spec["scale"][0] / 2.0
        object_half_y = object_spec["scale"][1] / 2.0
        object_top_z = object_spec["local_xyz"][2] + object_spec["scale"][2] / 2.0

        left_wall = palm_specs["palm_retention_left_wall_proxy"]
        right_wall = palm_specs["palm_retention_right_wall_proxy"]
        front_lip = palm_specs["palm_retention_front_lip_proxy"]
        back_lip = palm_specs["palm_retention_back_lip_proxy"]
        shelf = palm_specs["palm_retention_shelf_proxy"]

        left_inner_x = left_wall["local_xyz"][0] + left_wall["scale"][0] / 2.0
        right_inner_x = right_wall["local_xyz"][0] - right_wall["scale"][0] / 2.0
        front_inner_y = front_lip["local_xyz"][1] - front_lip["scale"][1] / 2.0
        back_inner_y = back_lip["local_xyz"][1] + back_lip["scale"][1] / 2.0
        shelf_back_y = shelf["local_xyz"][1] - shelf["scale"][1] / 2.0
        shelf_front_y = shelf["local_xyz"][1] + shelf["scale"][1] / 2.0

        self.assertLess(left_inner_x, -object_half_x)
        self.assertGreater(right_inner_x, object_half_x)
        self.assertGreater(front_inner_y, object_spec["local_xyz"][1] + object_half_y)
        self.assertLess(back_inner_y, object_spec["local_xyz"][1] - object_half_y)
        self.assertLess(
            object_spec["local_xyz"][1] - object_half_y - back_inner_y,
            0.045,
        )
        self.assertLessEqual(shelf_back_y, -0.060)
        self.assertGreaterEqual(shelf_front_y, object_spec["local_xyz"][1] + object_half_y)
        for wall in (left_wall, right_wall, front_lip, back_lip):
            self.assertGreaterEqual(
                wall["local_xyz"][2] + wall["scale"][2] / 2.0,
                object_top_z,
            )

    def test_grasp_diagnostics_use_palm_and_proxy_frame_not_top_level_hand_only(self) -> None:
        self.assertEqual(grasp_object_reset_anchor_path(), f"{CONNECTED_HAND_PRIM_PATH}/palm")

        paths = build_grasp_transform_diagnostic_paths()

        self.assertIn(ARM_HAND_ATTACHMENT_BODY0_PATH, paths)
        self.assertIn(f"{CONNECTED_HAND_PRIM_PATH}/r_wrist_interface", paths)
        self.assertIn(f"{CONNECTED_HAND_PRIM_PATH}/palm", paths)
        self.assertIn(
            f"{CONNECTED_HAND_PRIM_PATH}/finger1_distal/contact_proxies/finger1_distal_tip_pad_proxy",
            paths,
        )
        self.assertIn(
            f"{CONNECTED_HAND_PRIM_PATH}/finger4_distal/contact_proxies/finger4_distal_tip_pad_proxy",
            paths,
        )
        self.assertNotEqual(paths[0], CONNECTED_HAND_PRIM_PATH)

    def test_viewport_capture_can_optionally_select_focus_prim_before_camera_framing(self) -> None:
        old_value = os.environ.pop("ROBOT_ARM_HAND_SELECT_FOCUS_PRIM", None)
        try:
            self.assertFalse(rah._select_focus_prim_enabled())
            os.environ["ROBOT_ARM_HAND_SELECT_FOCUS_PRIM"] = "1"
            self.assertTrue(rah._select_focus_prim_enabled())
        finally:
            if old_value is None:
                os.environ.pop("ROBOT_ARM_HAND_SELECT_FOCUS_PRIM", None)
            else:
                os.environ["ROBOT_ARM_HAND_SELECT_FOCUS_PRIM"] = old_value

        source = (ROOT / "isaacsim_test/isaacsim/robot_arm_hand_from_zip.py").read_text(
            encoding="utf-8"
        )

        viewport_index = source.index("def _capture_viewport")
        set_camera_index = source.index("set_camera_view(eye=eye", viewport_index)
        select_index = source.index("set_selected_prim_paths", viewport_index)

        self.assertLess(select_index, set_camera_index)

    def test_visible_grasp_diagnostic_capture_specs_are_closeup_and_stage_labeled(self) -> None:
        specs = build_visible_grasp_diagnostic_capture_specs()

        self.assertEqual(
            [spec["label"] for spec in specs],
            ["open", "half_close", "full_close_before_lift", "after_lift_retain"],
        )
        self.assertTrue(all(spec["focus_prim_path"].endswith("/Hand/palm") for spec in specs))
        self.assertTrue(all("real_hand" in spec["filename"] for spec in specs))
        self.assertFalse(any("visible_proxies" in spec["filename"] for spec in specs))


    def test_closeup_capture_prefers_focused_viewport_before_crop_fallback(self) -> None:
        source = (ROOT / "isaacsim_test/isaacsim/robot_arm_hand_from_zip.py").read_text(
            encoding="utf-8"
        )

        closeup_index = source.index("if closeup_focus:")
        fallback_index = source.index("whole_scene_crop_fallback", closeup_index)
        focused_index = source.index("focused_viewport", closeup_index)
        connected_root_call = "_capture_viewport(_container_path(full_scene_output), CONNECTED_ROOT_PRIM_PATH)"

        self.assertLess(focused_index, fallback_index)
        self.assertNotIn(connected_root_call, source[closeup_index:fallback_index])

    def test_capture_resolution_comes_from_environment_with_720p_default(self) -> None:
        old_width = os.environ.pop("ROBOT_ARM_HAND_CAPTURE_WIDTH", None)
        old_height = os.environ.pop("ROBOT_ARM_HAND_CAPTURE_HEIGHT", None)
        try:
            self.assertEqual(rah._capture_resolution_from_env(), (1280, 720))

            os.environ["ROBOT_ARM_HAND_CAPTURE_WIDTH"] = "1920"
            os.environ["ROBOT_ARM_HAND_CAPTURE_HEIGHT"] = "1080"
            self.assertEqual(rah._capture_resolution_from_env(), (1920, 1080))

            os.environ["ROBOT_ARM_HAND_CAPTURE_WIDTH"] = "bad"
            os.environ["ROBOT_ARM_HAND_CAPTURE_HEIGHT"] = "0"
            self.assertEqual(rah._capture_resolution_from_env(), (1280, 720))
        finally:
            if old_width is None:
                os.environ.pop("ROBOT_ARM_HAND_CAPTURE_WIDTH", None)
            else:
                os.environ["ROBOT_ARM_HAND_CAPTURE_WIDTH"] = old_width
            if old_height is None:
                os.environ.pop("ROBOT_ARM_HAND_CAPTURE_HEIGHT", None)
            else:
                os.environ["ROBOT_ARM_HAND_CAPTURE_HEIGHT"] = old_height

    def test_runtime_report_records_screenshot_capture_method(self) -> None:
        source = (ROOT / "isaacsim_test/isaacsim/robot_arm_hand_from_zip.py").read_text(
            encoding="utf-8"
        )

        self.assertIn('"screenshot_capture": screenshot_capture', source)
        self.assertIn('"before_screenshot_capture": before_screenshot_capture', source)
        self.assertIn('"capture_method"', source)

    def test_lift_retain_resets_then_settles_object_before_baseline_measurement(self) -> None:
        source = (ROOT / "isaacsim_test/isaacsim/robot_arm_hand_from_zip.py").read_text(
            encoding="utf-8"
        )
        settle_index = source.index("args.settle_steps // 2")
        reset_index = source.rindex("object_reset_world_xyz = _reset_grasp_object_pose", 0, settle_index)
        before_index = source.index("object_before = _world_translation", settle_index)

        self.assertLess(reset_index, settle_index)
        self.assertLess(settle_index, before_index)

    def test_runtime_report_includes_preshape_grasp_validation_before_lift(self) -> None:
        source = (ROOT / "isaacsim_test/isaacsim/robot_arm_hand_from_zip.py").read_text(
            encoding="utf-8"
        )

        preshape_marker = "preshape_grasp_validation = _run_preshape_grasp_validation()"
        self.assertIn(preshape_marker, source)
        preshape_index = source.index(preshape_marker)
        lift_index = source.index("lift_retain_validation = _run_lift_retain_smoke()")
        report_key_index = source.index('"preshape_grasp_validation": preshape_grasp_validation')

        self.assertLess(preshape_index, lift_index)
        self.assertGreater(report_key_index, preshape_index)

    def test_simulation_app_close_defaults_to_graceful_and_fast_close_is_opt_in(self) -> None:
        class ModernApp:
            def __init__(self) -> None:
                self.calls: list[dict[str, object]] = []

            def close(self, **kwargs: object) -> None:
                self.calls.append(kwargs)

        class LegacyApp:
            def __init__(self) -> None:
                self.calls = 0

            def close(self) -> None:
                self.calls += 1

        old_value = os.environ.pop("ROBOT_ARM_HAND_ISAAC_FAST_CLOSE", None)
        try:
            modern = ModernApp()
            rah._close_simulation_app(modern)
            self.assertEqual(modern.calls, [{}])

            os.environ["ROBOT_ARM_HAND_ISAAC_FAST_CLOSE"] = "1"
            modern_fast = ModernApp()
            rah._close_simulation_app(modern_fast)
            self.assertEqual(modern_fast.calls, [{"skip_cleanup": True}])

            legacy = LegacyApp()
            rah._close_simulation_app(legacy)
            self.assertEqual(legacy.calls, 1)

            with (
                self.assertRaises(RuntimeError),
                mock.patch.object(rah.os, "_exit") as fake_exit,
                mock.patch.object(rah.traceback, "print_exception") as fake_print,
            ):
                try:
                    raise RuntimeError("boom")
                finally:
                    rah._close_simulation_app(modern_fast)
            fake_exit.assert_called_once_with(1)
            fake_print.assert_called_once()
        finally:
            if old_value is None:
                os.environ.pop("ROBOT_ARM_HAND_ISAAC_FAST_CLOSE", None)
            else:
                os.environ["ROBOT_ARM_HAND_ISAAC_FAST_CLOSE"] = old_value

    def test_isaacsim60_multi_instance_runner_uses_isolated_ports_and_fast_close(self) -> None:
        script = (ROOT / "isaacsim_test/run_isaacsim60_multi_instance.sh").read_text(
            encoding="utf-8"
        )

        self.assertIn("nvcr.io/nvidia/isaac-sim:6.0.0", script)
        self.assertIn("ISAACSIM_SIGNAL_PORT=${ISAACSIM_SIGNAL_PORT:-49200}", script)
        self.assertIn("ISAACSIM_STREAM_PORT=${ISAACSIM_STREAM_PORT:-48100}", script)
        self.assertIn("WEB_VIEWER_PORT=${WEB_VIEWER_PORT:-8211}", script)
        self.assertIn("ROBOT_ARM_HAND_ISAAC_FAST_CLOSE=1", script)
        self.assertIn("--user 0:0", script)
        self.assertIn("robot_arm_hand_from_zip.py --mode ${phase}", script)

    def test_robot_arm_hand_runner_defaults_to_kst_timestamped_result_dirs(self) -> None:
        script = (ROOT / "isaacsim_test/run_robot_arm_hand_from_zip.sh").read_text(
            encoding="utf-8"
        )

        self.assertIn("RUN_STAMP=${ROBOT_ARM_HAND_RUN_STAMP:-$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S_KST)}", script)
        self.assertIn("RUN_NAME=${ROBOT_ARM_HAND_RUN_NAME:-robot_arm_hand_from_zip}", script)
        self.assertIn("robot_arm_hand_graspable_${RUN_STAMP}_${RUN_NAME}", script)
        self.assertIn("visual_verification_${RUN_STAMP}_${RUN_NAME}", script)
        self.assertIn("LOG_STAMP=${ROBOT_ARM_HAND_LOG_STAMP:-$RUN_STAMP}", script)
        self.assertEqual(
            script.count(
                'ROBOT_ARM_HAND_INCLUDE_FINGER_SHELLS="${ROBOT_ARM_HAND_INCLUDE_FINGER_SHELLS:-0}"'
            ),
            2,
        )
        self.assertEqual(
            script.count(
                'ROBOT_ARM_HAND_VISUAL_MODE="${ROBOT_ARM_HAND_VISUAL_MODE:-partitioned_links}"'
            ),
            2,
        )
        self.assertIn("fix_artifact_ownership() {", script)
        self.assertIn("uid=$(id -u)", script)
        self.assertIn("gid=$(id -g)", script)
        self.assertIn("chown -R $uid:$gid", script)
        self.assertLess(
            script.index('"exec /isaac-sim/python.sh /workspace/isaacsim/robot_arm_hand_from_zip.py --mode runtime"'),
            script.index("fix_artifact_ownership\n\npython3 -"),
        )

    def test_urdf_import_config_supports_legacy_and_isaacsim60_fields(self) -> None:
        class LegacyConfig:
            def __init__(self) -> None:
                self.fix_base = None
                self.import_inertia_tensor = False
                self.distance_scale = 0.0
                self.default_drive_type = None
                self.default_drive_strength = 0.0
                self.default_position_drive_damping = 0.0

        class Isaac60Config:
            def __init__(self) -> None:
                self.fix_base = None
                self.joint_drive_type = None
                self.joint_target_type = None
                self.override_joint_stiffness = None
                self.override_joint_damping = None

        legacy = LegacyConfig()
        with mock.patch.object(
            rah,
            "_legacy_urdf_joint_drive_position_target",
            return_value="LEGACY_POSITION",
        ):
            rah._configure_urdf_import_config(
                legacy,
                fix_base=True,
                drive_strength=800.0,
                drive_damping=40.0,
            )
        self.assertTrue(legacy.fix_base)
        self.assertTrue(legacy.import_inertia_tensor)
        self.assertEqual(legacy.distance_scale, 1.0)
        self.assertEqual(legacy.default_drive_type, "LEGACY_POSITION")
        self.assertEqual(legacy.default_drive_strength, 800.0)
        self.assertEqual(legacy.default_position_drive_damping, 40.0)

        isaac60 = Isaac60Config()
        rah._configure_urdf_import_config(
            isaac60,
            fix_base=False,
            drive_strength=45.0,
            drive_damping=4.0,
        )
        self.assertFalse(isaac60.fix_base)
        self.assertEqual(isaac60.joint_drive_type, "force")
        self.assertEqual(isaac60.joint_target_type, "position")
        self.assertEqual(isaac60.override_joint_stiffness, 45.0)
        self.assertEqual(isaac60.override_joint_damping, 4.0)

    def test_isaacsim60_urdf_class_api_wrapper_references_generated_usd(self) -> None:
        from pxr import Usd, UsdGeom

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            generated = tmp_path / "generated.usd"
            generated_stage = Usd.Stage.CreateNew(str(generated))
            generated_root = UsdGeom.Xform.Define(generated_stage, "/generated_robot")
            UsdGeom.Xform.Define(generated_stage, "/generated_robot/link")
            generated_stage.SetDefaultPrim(generated_root.GetPrim())
            generated_stage.GetRootLayer().Save()

            wrapper = tmp_path / "arm.usd"
            prim_path = _add_reference_wrapper_usd(
                source_usd_path=generated,
                wrapper_usd_path=wrapper,
                wrapper_prim_path="/World/RobotArmFromZip",
            )

            self.assertEqual(prim_path, "/World/RobotArmFromZip")
            stage = Usd.Stage.Open(str(wrapper))
            self.assertTrue(stage.GetPrimAtPath("/World/RobotArmFromZip/link").IsValid())
            self.assertEqual(stage.GetDefaultPrim().GetPath().pathString, "/World")

    def test_lift_retain_sequence_ramps_arm_while_holding_hand_closed(self) -> None:
        dof_names = [
            "joint_rev_1",
            "joint_rev_2",
            "joint_rev_3",
            "joint_rev_4",
            "finger1_motor1",
            "finger2_motor1",
            "finger3_motor1",
            "finger4_motor1",
            "finger1_motor2",
            "finger2_motor2",
            "finger3_motor2",
            "finger4_motor2",
        ]
        sequence = build_lift_retain_joint_target_sequence(
            [0.0] * len(dof_names),
            dof_names,
            grasp=1.0,
            segments=3,
        )

        expected_lift_target = [-0.25, 0.15, 0.3, -0.2]
        self.assertEqual(LIFT_RETAIN_ARM_TARGET, expected_lift_target)
        self.assertEqual(len(sequence), 3)
        self.assertAlmostEqual(sequence[0]["positions"][0], -0.25 / 3.0)
        self.assertAlmostEqual(sequence[-1]["positions"][0], -0.25)
        self.assertAlmostEqual(sequence[-1]["positions"][1], 0.15)
        self.assertAlmostEqual(sequence[-1]["positions"][2], 0.3)
        self.assertAlmostEqual(sequence[-1]["positions"][3], -0.2)
        for joint_name in HAND_ACTUATED_JOINT_NAMES:
            self.assertIn(joint_name, sequence[-1]["controlled_joint_names"])

    def test_lift_retain_status_uses_grasp_anchor_not_top_level_hand(self) -> None:
        result = _evaluate_lift_retain_status(
            object_before=(0.0, 0.0, 0.60),
            object_after_close=(0.01, 0.02, 0.61),
            anchor_after_close=(0.0, 0.0, 0.60),
            object_after_lift=(0.02, 0.03, 0.66),
            anchor_after_lift=(0.0, 0.0, 0.65),
            object_reference=(0.0, 0.0, 0.64),
            finger_proxy_distances_after_close=[0.03, 0.04, 0.045],
            finger_proxy_distances_after_lift=[0.035, 0.04, 0.05],
            screenshot_size_bytes=123,
            control_status="APPLIED",
        )

        self.assertEqual(result["status"], "PASS")
        self.assertTrue(result["retained_near_hand"])
        self.assertTrue(result["lifted_or_held"])
        self.assertLess(result["object_anchor_distance_after_lift_m"], 0.05)
        self.assertEqual(result["object_z_delta_reference"], "reset_world_xyz")
        self.assertAlmostEqual(result["object_z_delta_after_lift_m"], 0.02)
        self.assertAlmostEqual(
            result["object_z_delta_after_lift_from_settled_before_m"],
            0.06,
        )


    def test_lift_retain_status_can_use_reset_pose_as_strict_z_reference(self) -> None:
        result = _evaluate_lift_retain_status(
            object_before=(0.0, 0.0, 0.64),
            object_after_close=(0.0, 0.0, 0.64),
            anchor_after_close=(0.0, 0.0, 0.60),
            object_after_lift=(0.03, 0.02, 0.628),
            anchor_after_lift=(0.0, 0.0, 0.60),
            object_reference=(0.0, 0.0, 0.627),
            finger_proxy_distances_after_close=[0.03, 0.04, 0.05],
            finger_proxy_distances_after_lift=[0.03, 0.04, 0.05],
            screenshot_size_bytes=123,
            control_status="APPLIED",
        )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["object_z_delta_reference"], "reset_world_xyz")
        self.assertAlmostEqual(result["object_z_delta_after_lift_m"], 0.001)
        self.assertAlmostEqual(
            result["object_z_delta_after_lift_from_settled_before_m"],
            -0.012,
        )

    def test_lift_retain_status_accepts_shell_overlay_finger_contact_tolerance(self) -> None:
        result = _evaluate_lift_retain_status(
            object_before=(0.0, 0.0, 0.641),
            object_after_close=(-0.034, 0.232, 0.634),
            anchor_after_close=(-0.035, 0.283, 0.598),
            object_after_lift=(-0.087, 0.284, 0.627),
            anchor_after_lift=(-0.043, 0.309, 0.594),
            object_reference=(-0.035, 0.311, 0.628),
            finger_proxy_distances_after_close=[0.035, 0.048, 0.049],
            finger_proxy_distances_after_lift=[0.030, 0.034, 0.060],
            screenshot_size_bytes=123,
            control_status="APPLIED",
        )

        self.assertEqual(result["status"], "PASS")
        self.assertTrue(result["finger_grasp_engaged"])
        self.assertEqual(result["finger_proxy_close_count_after_lift"], 3)
        self.assertEqual(
            result["finger_proxy_distance_threshold_m"],
            rah.FINGER_PROXY_DISTANCE_THRESHOLD_M,
        )


    def test_lift_retain_status_requires_finger_proxy_engagement_not_only_palm_cradle(self) -> None:
        result = _evaluate_lift_retain_status(
            object_before=(0.0, 0.0, 0.60),
            object_after_close=(0.01, 0.02, 0.61),
            anchor_after_close=(0.0, 0.0, 0.60),
            object_after_lift=(0.02, 0.03, 0.66),
            anchor_after_lift=(0.0, 0.0, 0.65),
            object_reference=(0.0, 0.0, 0.64),
            finger_proxy_distances_after_close=[0.12, 0.13, 0.14],
            finger_proxy_distances_after_lift=[0.12, 0.13, 0.14],
            screenshot_size_bytes=123,
            control_status="APPLIED",
        )

        self.assertEqual(result["status"], "WARN")
        self.assertFalse(result["finger_grasp_engaged"])
        self.assertEqual(result["finger_proxy_close_count_after_lift"], 0)

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
                hand_visual_mode="implemented_only",
                include_finger_shells=True,
            )

            _, report = _prepare_source_artifacts(args)

            generated = Path(report["graspable_hand_urdf_path"])
            self.assertTrue(generated.is_file())
            self.assertEqual(report["graspable_hand_urdf"]["status"], "PASS")
            self.assertEqual(
                report["graspable_hand_urdf"]["actuated_joint_names"],
                HAND_ACTUATED_JOINT_NAMES,
            )
            self.assertEqual(report["graspable_hand_urdf"]["visual_mode"], "implemented_only")
            self.assertFalse(report["graspable_hand_urdf"]["finger_shell_visuals_enabled"])
            self.assertEqual(report["graspable_hand_urdf"]["implemented_debug_visual_count"], 17)

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
