from __future__ import annotations

import tempfile
import sys
import unittest
import math
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from isaacsim_test.isaacsim.robot_arm_hand_from_zip import (
    extract_robot_arm_hand_package,
)
import isaacsim_test.isaacsim.graspable_hand_urdf as graspable_hand_urdf
from isaacsim_test.isaacsim.graspable_hand_urdf import (
    HAND_ACTUATED_JOINT_NAMES,
    VISUAL_MODE_IMPLEMENTED_ONLY,
    VISUAL_MODE_PARTITIONED_LINKS,
    VISUAL_MODE_STATIC_SHELL,
    build_graspable_hand_model_spec,
    generate_graspable_hand_urdf,
    grasp_scalar_to_hand_joint_targets,
)

ZIP_PATH = ROOT / "robot_arm_hand_package.zip"


class GraspableHandUrdfTests(unittest.TestCase):
    def _preshape_targets(self, grasp_amount: float, grasp_type: str = "wrap") -> dict[str, float]:
        self.assertTrue(
            hasattr(graspable_hand_urdf, "grasp_preshape_to_hand_joint_targets"),
            "grasp_preshape_to_hand_joint_targets must exist for reference-hand preshapes",
        )
        return graspable_hand_urdf.grasp_preshape_to_hand_joint_targets(  # type: ignore[attr-defined]
            grasp_amount,
            grasp_type,
        )

    def test_spec_preserves_eight_actuated_tree_joints_and_visual_assets(self) -> None:
        spec = build_graspable_hand_model_spec()

        self.assertEqual(
            HAND_ACTUATED_JOINT_NAMES,
            [
                "finger1_motor1",
                "finger1_motor2",
                "finger2_motor1",
                "finger2_motor2",
                "finger3_motor1",
                "finger3_motor2",
                "finger4_motor1",
                "finger4_motor2",
            ],
        )
        self.assertEqual(spec["root_link"], "r_wrist_interface")
        self.assertEqual(spec["actuated_joint_names"], HAND_ACTUATED_JOINT_NAMES)
        self.assertEqual(spec["equality_constraint_count"], 0)
        self.assertEqual(spec["finger_count"], 4)
        self.assertEqual(spec["finger_base_frame_count"], 4)
        self.assertEqual(spec["excluded_human_finger"], "pinky")
        self.assertEqual(
            spec["finger_roles"],
            {
                "finger1": "index",
                "finger2": "middle",
                "finger3": "ring",
                "finger4": "thumb",
            },
        )
        self.assertEqual(spec["collision_primitive_count"], 13)
        self.assertIn("r_wrist_interface.stl", spec["visual_mesh_files"])
        self.assertIn("r_hand_plate.stl", spec["visual_mesh_files"])
        self.assertIn("proximal.stl", spec["visual_mesh_files"])
        self.assertIn("proximal_shell.stl", spec["visual_mesh_files"])
        self.assertIn("distal.stl", spec["visual_mesh_files"])
        self.assertIn("distal_shell.stl", spec["visual_mesh_files"])
        self.assertEqual(spec["default_visual_mode"], VISUAL_MODE_PARTITIONED_LINKS)
        self.assertIn(VISUAL_MODE_STATIC_SHELL, spec["available_visual_modes"])
        self.assertIn(VISUAL_MODE_IMPLEMENTED_ONLY, spec["available_visual_modes"])

    def test_motor_contract_matches_amazinghand_scs0009_config(self) -> None:
        self.assertTrue(hasattr(graspable_hand_urdf, "build_amazinghand_motor_contract"))

        contract = graspable_hand_urdf.build_amazinghand_motor_contract()  # type: ignore[attr-defined]

        self.assertEqual(contract["servo_model"], "SCS0009")
        self.assertEqual(contract["servo_ids"], [1, 2, 3, 4, 5, 6, 7, 8])
        self.assertEqual(
            contract["joint_to_servo_id"],
            {
                "finger1_motor1": 1,
                "finger1_motor2": 2,
                "finger2_motor1": 3,
                "finger2_motor2": 4,
                "finger3_motor1": 5,
                "finger3_motor2": 6,
                "finger4_motor1": 7,
                "finger4_motor2": 8,
            },
        )
        self.assertAlmostEqual(
            contract["motors"]["finger1_motor1"]["offset_rad"],
            math.radians(7.0),
        )
        self.assertAlmostEqual(
            contract["motors"]["finger4_motor2"]["offset_rad"],
            math.radians(7.0),
        )
        self.assertFalse(contract["motors"]["finger4_motor2"]["invert"])
        spec = build_graspable_hand_model_spec()
        self.assertEqual(spec["motor_contract"]["servo_model"], "SCS0009")
        self.assertEqual(spec["motor_contract"]["servo_ids"], list(range(1, 9)))

    def test_servo_targets_apply_offsets_and_opposed_pair_directions(self) -> None:
        self.assertTrue(
            hasattr(graspable_hand_urdf, "grasp_preshape_to_servo_targets")
        )

        open_targets = graspable_hand_urdf.grasp_preshape_to_servo_targets(  # type: ignore[attr-defined]
            0.0,
            "wrap",
        )
        closed_targets = graspable_hand_urdf.grasp_preshape_to_servo_targets(  # type: ignore[attr-defined]
            1.0,
            "wrap",
        )
        pinch_targets = graspable_hand_urdf.grasp_preshape_to_servo_targets(  # type: ignore[attr-defined]
            1.0,
            "pinch",
        )

        self.assertAlmostEqual(open_targets[1], math.radians(7.0 - 30.0))
        self.assertAlmostEqual(open_targets[2], math.radians(5.0 + 30.0))
        self.assertAlmostEqual(closed_targets[1], math.radians(7.0 + 90.0))
        self.assertAlmostEqual(closed_targets[2], math.radians(5.0 - 90.0))
        self.assertGreater(pinch_targets[1], pinch_targets[3])
        self.assertLess(pinch_targets[2], pinch_targets[4])

    def test_grasp_scalar_targets_clamp_and_close_monotonically(self) -> None:
        open_targets = grasp_scalar_to_hand_joint_targets(0.0)
        closed_targets = grasp_scalar_to_hand_joint_targets(1.0)
        over_closed_targets = grasp_scalar_to_hand_joint_targets(2.0)
        under_open_targets = grasp_scalar_to_hand_joint_targets(-1.0)

        self.assertEqual(set(open_targets), set(HAND_ACTUATED_JOINT_NAMES))
        self.assertEqual(under_open_targets, open_targets)
        self.assertEqual(over_closed_targets, closed_targets)
        for joint_name in HAND_ACTUATED_JOINT_NAMES:
            self.assertLessEqual(open_targets[joint_name], closed_targets[joint_name])
            self.assertGreaterEqual(closed_targets[joint_name], 0.6)
            self.assertLessEqual(closed_targets[joint_name], 1.25)

    def test_wrap_preshape_matches_legacy_scalar_targets(self) -> None:
        for amount in (0.0, 0.35, 1.0):
            self.assertEqual(
                self._preshape_targets(amount, "wrap"),
                grasp_scalar_to_hand_joint_targets(amount),
            )

    def test_preshape_targets_clamp_amount_and_keep_valid_joint_names(self) -> None:
        open_targets = self._preshape_targets(-1.0, "wide")
        closed_targets = self._preshape_targets(2.0, "wide")

        self.assertEqual(set(open_targets), set(HAND_ACTUATED_JOINT_NAMES))
        self.assertEqual(set(closed_targets), set(HAND_ACTUATED_JOINT_NAMES))
        for joint_name in HAND_ACTUATED_JOINT_NAMES:
            self.assertLessEqual(open_targets[joint_name], closed_targets[joint_name])
            self.assertGreaterEqual(open_targets[joint_name], 0.0)
            self.assertLessEqual(closed_targets[joint_name], 1.25)

    def test_pinch_preshape_prioritizes_thumb_and_index(self) -> None:
        targets = self._preshape_targets(1.0, "pinch")

        self.assertGreater(targets["finger1_motor1"], targets["finger2_motor1"])
        self.assertGreater(targets["finger1_motor2"], targets["finger2_motor2"])
        self.assertGreater(targets["finger4_motor1"], targets["finger3_motor1"])
        self.assertGreater(targets["finger4_motor2"], targets["finger3_motor2"])

    def test_wide_preshape_is_more_open_than_wrap_at_same_amount(self) -> None:
        wide_targets = self._preshape_targets(1.0, "wide")
        wrap_targets = self._preshape_targets(1.0, "wrap")

        for joint_name in HAND_ACTUATED_JOINT_NAMES:
            self.assertLess(wide_targets[joint_name], wrap_targets[joint_name])

    def test_invalid_preshape_type_lists_supported_options(self) -> None:
        with self.assertRaisesRegex(ValueError, "wrap.*pinch.*wide"):
            self._preshape_targets(0.5, "claw")

    def test_generate_urdf_writes_tree_hand_with_visuals_collisions_and_inertials(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            package_root = extract_robot_arm_hand_package(ZIP_PATH, Path(tmp) / "inputs")
            output_urdf = Path(tmp) / "amazinghand_graspable.urdf"

            report = generate_graspable_hand_urdf(package_root, output_urdf)

            self.assertEqual(report["status"], "PASS")
            self.assertEqual(report["root_link"], "r_wrist_interface")
            self.assertEqual(report["actuated_joint_names"], HAND_ACTUATED_JOINT_NAMES)
            self.assertEqual(report["visual_mode"], VISUAL_MODE_PARTITIONED_LINKS)
            self.assertEqual(report["missing_visual_meshes"], [])
            self.assertEqual(report["collision_primitive_count"], 13)
            self.assertTrue(output_urdf.is_file())
            self.assertEqual(
                report["mjcf_visual_shell"]["visual_attachment_mode"],
                "mjcf_visuals_partitioned_to_tree_links",
            )
            self.assertEqual(report["mjcf_visual_shell"]["mjcf_visual_geom_count"], 162)
            self.assertEqual(report["mjcf_visual_shell"]["missing_mjcf_visual_meshes"], [])
            counts = report["mjcf_visual_shell"]["link_visual_counts"]
            self.assertGreaterEqual(counts.get("finger1_proximal", 0), 16)
            self.assertGreaterEqual(counts.get("finger1_distal", 0), 2)
            self.assertGreaterEqual(counts.get("finger2_proximal", 0), 16)
            self.assertGreaterEqual(counts.get("finger2_distal", 0), 2)
            self.assertLess(counts.get("r_wrist_interface", 0), 146)
            self.assertEqual(
                report["mjcf_visual_shell"]["skeleton_first_policy"],
                "major_linkage_and_pin_visuals_follow_generated_finger_links",
            )

            root = ET.parse(output_urdf).getroot()
            links = {link.attrib["name"]: link for link in root.findall("link")}
            joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}

            self.assertIn("r_wrist_interface", links)
            self.assertNotIn("amazinghand_visual_shell", links)
            self.assertIn("palm", links)
            self.assertEqual(len([name for name in links if name.endswith("_base")]), 4)
            self.assertEqual(len([name for name in links if name.endswith("_proximal")]), 4)
            self.assertEqual(len([name for name in links if name.endswith("_distal")]), 4)
            self.assertEqual(len(joints), 13)
            self.assertEqual(joints["wrist_to_palm"].attrib["type"], "fixed")
            self.assertNotIn("wrist_to_amazinghand_visual_shell", joints)
            expected_base_xyz = {
                1: (-0.00505, 0.03055, 0.06980),
                2: (-0.00505, 0.00110, 0.06456),
                3: (-0.00505, -0.02705, 0.05505),
                4: (-0.00030, 0.00773, 0.03615),
            }
            self.assertEqual(
                report["finger_base_anchor_policy"],
                "fixed finger_base frames use MJCF custom_servo_horn world positions as palm-local motor-frame anchors",
            )
            self.assertEqual(
                report["finger_base_layouts"]["finger1"]["mjcf_anchor_body"],
                "custom_servo_horn",
            )
            for finger_index in range(1, 5):
                motor1 = joints[f"finger{finger_index}_motor1"]
                motor2 = joints[f"finger{finger_index}_motor2"]
                palm_to_base = joints[f"palm_to_finger{finger_index}_base"]
                origin = palm_to_base.find("origin")
                self.assertIsNotNone(origin)
                actual_base_xyz = tuple(
                    round(float(value), 5)
                    for value in origin.attrib["xyz"].split()
                )
                self.assertEqual(actual_base_xyz, expected_base_xyz[finger_index])
                self.assertEqual(
                    joints[f"palm_to_finger{finger_index}_base"].find("parent").attrib["link"],  # type: ignore[union-attr]
                    "palm",
                )
                self.assertEqual(
                    joints[f"palm_to_finger{finger_index}_base"].find("child").attrib["link"],  # type: ignore[union-attr]
                    f"finger{finger_index}_base",
                )
                self.assertEqual(
                    motor1.find("parent").attrib["link"],  # type: ignore[union-attr]
                    f"finger{finger_index}_base",
                )
                self.assertEqual(
                    motor1.find("child").attrib["link"],  # type: ignore[union-attr]
                    f"finger{finger_index}_proximal",
                )
                self.assertEqual(
                    motor2.find("parent").attrib["link"],  # type: ignore[union-attr]
                    f"finger{finger_index}_proximal",
                )
                self.assertEqual(
                    motor2.find("child").attrib["link"],  # type: ignore[union-attr]
                    f"finger{finger_index}_distal",
                )
            for joint_name in HAND_ACTUATED_JOINT_NAMES:
                joint = joints[joint_name]
                self.assertEqual(joint.attrib["type"], "revolute")
                self.assertIsNotNone(joint.find("limit"))
                self.assertIsNotNone(joint.find("axis"))

            visuals = root.findall(".//visual")
            visual_mesh_names = {
                Path(mesh.attrib["filename"]).name
                for mesh in root.findall(".//visual/geometry/mesh")
                if "filename" in mesh.attrib
            }
            finger1_proximal_mesh_names = {
                Path(mesh.attrib["filename"]).name
                for mesh in root.findall(".//link[@name='finger1_proximal']//visual/geometry/mesh")
                if "filename" in mesh.attrib
            }
            finger1_distal_mesh_names = {
                Path(mesh.attrib["filename"]).name
                for mesh in root.findall(".//link[@name='finger1_distal']//visual/geometry/mesh")
                if "filename" in mesh.attrib
            }
            finger1_major_visual_origins = []
            for link_name in ("finger1_proximal", "finger1_distal"):
                for visual in root.findall(f".//link[@name='{link_name}']/visual"):
                    mesh = visual.find("geometry/mesh")
                    origin = visual.find("origin")
                    if mesh is None or origin is None:
                        continue
                    mesh_name = Path(mesh.attrib["filename"]).name
                    if mesh_name in {
                        "custom_servo_horn.stl",
                        "gimbal.stl",
                        "link.stl",
                        "m2_rod_l18.stl",
                        "parallel_pin_2_x_10__fee063fca0c8b40e46bbc4ffff61d999.stl",
                        "parallel_pin_2_x_16__da4b7ddbe9d803fe3fbc70f2e822b99b.stl",
                        "rotule_ball.stl",
                        "rotule_lever.stl",
                    }:
                        finger1_major_visual_origins.append(
                            [float(value) for value in origin.attrib["xyz"].split()]
                        )
            self.assertGreaterEqual(len(visuals), 90)
            self.assertIn("r_wrist_interface.stl", visual_mesh_names)
            self.assertIn("r_hand_plate.stl", visual_mesh_names)
            self.assertIn("finger_frame_1.stl", visual_mesh_names)
            self.assertIn("scs0009.stl", visual_mesh_names)
            self.assertNotIn("proximal.stl", visual_mesh_names)
            self.assertNotIn("proximal_shell.stl", visual_mesh_names)
            self.assertNotIn("distal.stl", visual_mesh_names)
            self.assertNotIn("distal_shell.stl", visual_mesh_names)
            self.assertEqual(
                report["mjcf_visual_shell"]["omitted_shell_visual_count"],
                16,
            )
            self.assertNotIn(
                "ph_pan_head_screw_m2x0_40_x_10__2803432263e518bbd16bccbbef8784ed.stl",
                visual_mesh_names,
            )
            self.assertNotIn(
                "plain_washer_large_grade_a_m2_5__9a369f0dc77bf9c598cdf3fb468977e5.stl",
                visual_mesh_names,
            )
            self.assertNotIn("spacer.stl", visual_mesh_names)
            self.assertEqual(
                report["mjcf_visual_shell"]["omitted_detail_visual_count"],
                48,
            )
            self.assertIn("custom_servo_horn.stl", finger1_proximal_mesh_names)
            self.assertIn(
                "parallel_pin_2_x_16__da4b7ddbe9d803fe3fbc70f2e822b99b.stl",
                finger1_proximal_mesh_names,
            )
            self.assertIn("gimbal.stl", finger1_proximal_mesh_names)
            self.assertIn("link.stl", finger1_distal_mesh_names)
            self.assertIn(
                "parallel_pin_2_x_10__fee063fca0c8b40e46bbc4ffff61d999.stl",
                finger1_distal_mesh_names,
            )
            self.assertTrue(finger1_major_visual_origins)
            self.assertLess(
                max(abs(origin[2]) for origin in finger1_major_visual_origins),
                0.06,
            )
            proximal_visual_origin = root.find(
                ".//link[@name='finger1_proximal']/visual/origin"
            )
            distal_visual_origin = root.find(
                ".//link[@name='finger1_distal']/visual/origin"
            )
            self.assertIsNotNone(proximal_visual_origin)
            self.assertIsNotNone(distal_visual_origin)

            proximal_box = root.find(".//collision[@name='finger1_proximal_contact_box']/geometry/box")
            distal_box = root.find(".//collision[@name='finger1_distal_contact_box']/geometry/box")
            self.assertIsNotNone(proximal_box)
            self.assertIsNotNone(distal_box)
            proximal_size = [float(item) for item in proximal_box.attrib["size"].split()]  # type: ignore[union-attr]
            distal_size = [float(item) for item in distal_box.attrib["size"].split()]  # type: ignore[union-attr]
            self.assertGreater(proximal_size[1], proximal_size[2])
            self.assertGreater(distal_size[1], distal_size[2])

            mesh_paths = [
                Path(mesh.attrib["filename"])
                for mesh in root.findall(".//mesh")
                if "filename" in mesh.attrib
            ]
            self.assertTrue(mesh_paths)
            for mesh_path in mesh_paths:
                self.assertTrue(mesh_path.is_file(), mesh_path)

            collisions = root.findall(".//collision")
            inertials = root.findall(".//inertial")
            self.assertEqual(len(collisions), 13)
            self.assertEqual(len(inertials), len(links))
            self.assertNotIn("equality", output_urdf.read_text(encoding="utf-8").lower())

    def test_generate_urdf_can_write_legacy_static_visual_shell(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            package_root = extract_robot_arm_hand_package(ZIP_PATH, Path(tmp) / "inputs")
            output_urdf = Path(tmp) / "amazinghand_graspable_static.urdf"

            report = generate_graspable_hand_urdf(
                package_root,
                output_urdf,
                visual_mode=VISUAL_MODE_STATIC_SHELL,
            )

            self.assertEqual(report["status"], "PASS")
            self.assertEqual(report["visual_mode"], VISUAL_MODE_STATIC_SHELL)
            self.assertEqual(
                report["mjcf_visual_shell"]["visual_attachment_mode"],
                "mjcf_static_visual_shell",
            )
            self.assertEqual(
                report["mjcf_visual_shell"]["link_visual_counts"]["amazinghand_visual_shell"],
                162,
            )
            root = ET.parse(output_urdf).getroot()
            links = {link.attrib["name"]: link for link in root.findall("link")}
            joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
            self.assertIn("amazinghand_visual_shell", links)
            self.assertEqual(joints["wrist_to_amazinghand_visual_shell"].attrib["type"], "fixed")

    def test_generate_urdf_can_write_experimental_partitioned_moving_visuals(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            package_root = extract_robot_arm_hand_package(ZIP_PATH, Path(tmp) / "inputs")
            output_urdf = Path(tmp) / "amazinghand_graspable_partitioned.urdf"

            report = generate_graspable_hand_urdf(
                package_root,
                output_urdf,
                visual_mode=VISUAL_MODE_PARTITIONED_LINKS,
            )

            self.assertEqual(report["status"], "PASS")
            self.assertEqual(report["visual_mode"], VISUAL_MODE_PARTITIONED_LINKS)
            self.assertEqual(
                report["mjcf_visual_shell"]["visual_attachment_mode"],
                "mjcf_visuals_partitioned_to_tree_links",
            )
            self.assertEqual(report["mjcf_visual_shell"]["mjcf_visual_geom_count"], 162)
            self.assertEqual(report["mjcf_visual_shell"]["missing_mjcf_visual_meshes"], [])
            self.assertEqual(
                report["mjcf_visual_shell"]["skeleton_first_policy"],
                "major_linkage_and_pin_visuals_follow_generated_finger_links",
            )
            self.assertLess(report["mjcf_visual_shell"]["link_visual_counts"]["r_wrist_interface"], 146)
            self.assertGreaterEqual(report["mjcf_visual_shell"]["link_visual_counts"]["finger1_proximal"], 16)
            self.assertGreaterEqual(report["mjcf_visual_shell"]["link_visual_counts"]["finger1_distal"], 2)
            self.assertEqual(report["mjcf_visual_shell"]["omitted_shell_visual_count"], 16)

            root = ET.parse(output_urdf).getroot()
            links = {link.attrib["name"]: link for link in root.findall("link")}
            joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
            self.assertNotIn("amazinghand_visual_shell", links)
            self.assertEqual(len(joints), 13)
            self.assertIsNotNone(root.find(".//link[@name='finger1_proximal']/visual/origin"))
            self.assertIsNotNone(root.find(".//link[@name='finger1_distal']/visual/origin"))

    def test_generate_urdf_can_hide_cad_visuals_and_show_only_implemented_primitives(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            package_root = extract_robot_arm_hand_package(ZIP_PATH, Path(tmp) / "inputs")
            output_urdf = Path(tmp) / "amazinghand_graspable_implemented_only.urdf"

            report = generate_graspable_hand_urdf(
                package_root,
                output_urdf,
                visual_mode=VISUAL_MODE_IMPLEMENTED_ONLY,
                include_finger_shells=True,
            )

            self.assertEqual(report["status"], "PASS")
            self.assertEqual(report["visual_mode"], VISUAL_MODE_IMPLEMENTED_ONLY)
            self.assertFalse(report["finger_shell_visuals_enabled"])
            self.assertEqual(report["collision_primitive_count"], 13)
            self.assertEqual(report["implemented_debug_visual_count"], 17)
            visual_report = report["mjcf_visual_shell"]
            self.assertEqual(
                visual_report["visual_attachment_mode"],
                "implemented_collision_primitives_only",
            )
            self.assertEqual(visual_report["implemented_debug_visual_count"], 17)
            self.assertEqual(visual_report["moving_shell_visual_count"], 0)
            self.assertEqual(visual_report["wrist_fixed_major_visuals"], [])

            root = ET.parse(output_urdf).getroot()
            self.assertEqual(len(root.findall(".//collision")), 13)
            self.assertEqual(len(root.findall(".//visual/geometry/box")), 17)
            self.assertEqual(len(root.findall(".//visual/geometry/mesh")), 0)
            self.assertEqual(len(root.findall("joint")), 13)
            visual_names = {
                visual.attrib["name"]
                for visual in root.findall(".//visual")
                if "name" in visual.attrib
            }
            self.assertIn("palm_contact_box_implemented_visual", visual_names)
            self.assertIn("finger1_base_motor_mount_frame_implemented_visual", visual_names)
            self.assertIn("finger1_proximal_contact_box_implemented_visual", visual_names)
            self.assertIn("finger1_distal_tip_pad_contact_box_implemented_visual", visual_names)

    def test_generate_urdf_can_overlay_finger_shells_without_changing_physics(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            package_root = extract_robot_arm_hand_package(ZIP_PATH, Path(tmp) / "inputs")
            output_urdf = Path(tmp) / "amazinghand_graspable_shell_overlay.urdf"

            report = generate_graspable_hand_urdf(
                package_root,
                output_urdf,
                visual_mode=VISUAL_MODE_PARTITIONED_LINKS,
                include_finger_shells=True,
            )

            self.assertEqual(report["status"], "PASS")
            self.assertTrue(report["finger_shell_visuals_enabled"])
            self.assertEqual(report["collision_primitive_count"], 13)
            shell_report = report["mjcf_visual_shell"]
            self.assertTrue(shell_report["finger_shell_visuals_enabled"])
            self.assertEqual(shell_report["omitted_shell_visual_count"], 0)
            self.assertEqual(shell_report["moving_shell_visual_count"], 16)
            self.assertEqual(
                shell_report["finger_shell_alignment_policy"],
                "proximal_and_distal_shell_visuals_follow_generated_finger_links",
            )
            self.assertEqual(shell_report["moving_shell_visual_counts"]["finger1_proximal"], 2)
            self.assertEqual(shell_report["moving_shell_visual_counts"]["finger1_distal"], 2)

            root = ET.parse(output_urdf).getroot()
            visual_mesh_names = {
                Path(mesh.attrib["filename"]).name
                for mesh in root.findall(".//visual/geometry/mesh")
                if "filename" in mesh.attrib
            }
            self.assertIn("proximal.stl", visual_mesh_names)
            self.assertIn("proximal_shell.stl", visual_mesh_names)
            self.assertIn("distal.stl", visual_mesh_names)
            self.assertIn("distal_shell.stl", visual_mesh_names)
            self.assertEqual(len(root.findall(".//collision")), 13)
            self.assertEqual(len(root.findall("joint")), 13)


if __name__ == "__main__":
    unittest.main(verbosity=2)
