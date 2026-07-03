from __future__ import annotations

import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from isaacsim_test.isaacsim.amazinghand_mjcf_to_urdf import convert_amazinghand_mjcf_to_urdf


ROOT = Path(__file__).resolve().parents[1]
RIGHT_MJCF = ROOT / "AmazingHand/Demo/AHSimulation/AHSimulation/AH_Right/mjcf/robot.xml"
LEFT_MJCF = ROOT / "AmazingHand/Demo/AHSimulation/AHSimulation/AH_Left/mjcf/robot.xml"


class AmazingHandMjcfToUrdfTest(unittest.TestCase):
    def test_converts_right_hand_mjcf_to_standalone_urdf(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "amazinghand_right.urdf"

            report = convert_amazinghand_mjcf_to_urdf(
                mjcf_path=RIGHT_MJCF,
                urdf_path=output,
                robot_name="amazinghand_right",
            )

            self.assertEqual(report["root_link"], "r_wrist_interface")
            self.assertEqual(report["body_count"], 33)
            self.assertEqual(report["hinge_joint_count"], 20)
            self.assertEqual(report["ball_joint_count"], 12)
            self.assertEqual(report["expanded_ball_revolute_joint_count"], 36)
            self.assertEqual(report["equality_constraint_count"], 20)
            self.assertEqual(report["unsupported_features"], ["mjcf_equality_connect"])
            self.assertTrue((output.parent / "assets" / "r_wrist_interface.stl").is_file())

            root = ET.parse(output).getroot()
            self.assertEqual(root.attrib["name"], "amazinghand_right")

            links = {link.attrib["name"] for link in root.findall("link")}
            joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
            materials = {material.attrib["name"]: material for material in root.findall("material")}

            self.assertIn("r_wrist_interface", links)
            self.assertIn("finger1_motor1", joints)
            self.assertIn("passive_ball1_x", joints)
            self.assertIn("passive_ball1_y", joints)
            self.assertIn("passive_ball1_z", joints)
            self.assertEqual(joints["finger1_motor1"].attrib["type"], "revolute")
            self.assertEqual(joints["passive_ball1_x"].attrib["type"], "revolute")
            self.assertEqual(joints["finger1_motor1"].find("parent").attrib["link"], "r_wrist_interface")  # type: ignore[union-attr]
            self.assertEqual(joints["finger1_motor1"].find("child").attrib["link"], "custom_servo_horn")  # type: ignore[union-attr]
            self.assertEqual(joints["finger1_motor1"].find("axis").attrib["xyz"], "0 0 1")  # type: ignore[union-attr]
            self.assertEqual(
                joints["finger1_motor1"].find("limit").attrib["lower"],  # type: ignore[union-attr]
                "-1.570796326768256",
            )
            self.assertIn("r_wrist_interface_visual_20", output.read_text(encoding="utf-8"))
            self.assertIn("assets/r_wrist_interface.stl", output.read_text(encoding="utf-8"))
            self.assertEqual(
                materials["rotule_ball_material"].find("color").attrib["rgba"],  # type: ignore[union-attr]
                "0.960784 0.835294 0.470588 1",
            )

    def test_converts_left_hand_with_left_root_and_assets(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "amazinghand_left.urdf"

            report = convert_amazinghand_mjcf_to_urdf(
                mjcf_path=LEFT_MJCF,
                urdf_path=output,
                robot_name="amazinghand_left",
            )

            self.assertEqual(report["root_link"], "l_wrist_interface")
            self.assertTrue((output.parent / "assets" / "l_wrist_interface.stl").is_file())
            root = ET.parse(output).getroot()
            links = {link.attrib["name"] for link in root.findall("link")}
            joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}

            self.assertIn("l_wrist_interface", links)
            self.assertIn("finger4_motor2", joints)
            self.assertEqual(joints["finger4_motor2"].attrib["type"], "revolute")


if __name__ == "__main__":
    unittest.main()
