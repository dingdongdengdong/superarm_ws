from __future__ import annotations

import tempfile
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from isaacsim_test.isaacsim.robot_arm_hand_from_zip import (
    extract_robot_arm_hand_package,
)
from isaacsim_test.isaacsim.graspable_hand_urdf import (
    HAND_ACTUATED_JOINT_NAMES,
    build_graspable_hand_model_spec,
    generate_graspable_hand_urdf,
    grasp_scalar_to_hand_joint_targets,
)

ZIP_PATH = ROOT / "robot_arm_hand_package.zip"


class GraspableHandUrdfTests(unittest.TestCase):
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

    def test_generate_urdf_writes_tree_hand_with_visuals_collisions_and_inertials(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            package_root = extract_robot_arm_hand_package(ZIP_PATH, Path(tmp) / "inputs")
            output_urdf = Path(tmp) / "amazinghand_graspable.urdf"

            report = generate_graspable_hand_urdf(package_root, output_urdf)

            self.assertEqual(report["status"], "PASS")
            self.assertEqual(report["root_link"], "r_wrist_interface")
            self.assertEqual(report["actuated_joint_names"], HAND_ACTUATED_JOINT_NAMES)
            self.assertEqual(report["missing_visual_meshes"], [])
            self.assertEqual(report["collision_primitive_count"], 13)
            self.assertTrue(output_urdf.is_file())

            root = ET.parse(output_urdf).getroot()
            links = {link.attrib["name"]: link for link in root.findall("link")}
            joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}

            self.assertIn("r_wrist_interface", links)
            self.assertIn("amazinghand_visual_shell", links)
            self.assertIn("palm", links)
            self.assertEqual(len([name for name in links if name.endswith("_proximal")]), 4)
            self.assertEqual(len([name for name in links if name.endswith("_distal")]), 4)
            self.assertEqual(len(joints), 10)
            self.assertEqual(joints["wrist_to_amazinghand_visual_shell"].attrib["type"], "fixed")
            self.assertEqual(joints["wrist_to_palm"].attrib["type"], "fixed")
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
            self.assertGreaterEqual(len(visuals), 160)
            self.assertIn("r_wrist_interface.stl", visual_mesh_names)
            self.assertIn("r_hand_plate.stl", visual_mesh_names)
            self.assertIn("finger_frame_1.stl", visual_mesh_names)
            self.assertIn("scs0009.stl", visual_mesh_names)
            self.assertIn("proximal.stl", visual_mesh_names)
            self.assertIn("distal.stl", visual_mesh_names)
            shell_visual_origin = root.find(
                ".//link[@name='amazinghand_visual_shell']/visual/origin"
            )
            self.assertIsNotNone(shell_visual_origin)
            self.assertNotEqual(shell_visual_origin.attrib["xyz"], "0 0 0")  # type: ignore[union-attr]

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


if __name__ == "__main__":
    unittest.main(verbosity=2)
