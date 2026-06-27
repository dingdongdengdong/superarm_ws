"""Static contract checks for the RoboParty V2 Isaac Sim testbed.

Run with:
    python3 isaacsim_test/test_v2_roboparty_config.py
"""

from __future__ import annotations

import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
V2_URDF = ROOT / "roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf"
REQUIRED_ARM_JOINTS = [
    "right_arm_pitch_joint",
    "right_arm_roll_joint",
    "right_arm_yaw_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
]
FEATURE_JOINTS = [*REQUIRED_ARM_JOINTS, "amazinghand_grasp"]


def _read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


class RoboPartyV2ConfigTest(unittest.TestCase):
    def test_official_v2_urdf_contains_required_right_arm_joints(self) -> None:
        root = ET.parse(V2_URDF).getroot()
        joints = {joint.attrib["name"] for joint in root.findall("joint")}

        for joint_name in REQUIRED_ARM_JOINTS:
            self.assertIn(joint_name, joints)

    def test_lerobot_config_uses_v2_right_arm_plus_amazinghand(self) -> None:
        config_text = _read("isaacsim_test/lerobot/rpo_arm_isaacsim.yaml")
        robot_text = _read("isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py")

        for joint_name in FEATURE_JOINTS:
            pattern = rf"(^|[\s\"']){re.escape(joint_name)}([\s\"',]|$)"
            self.assertRegex(config_text, pattern)
            self.assertRegex(robot_text, pattern)

        self.assertNotIn("rpo_arm_j1", config_text)
        self.assertNotIn("rpo_arm_j1", robot_text)

    def test_compose_and_env_default_to_official_v2_urdf_and_six_features(self) -> None:
        compose_text = _read("isaacsim_test/docker-compose.yml")
        env_text = _read("isaacsim_test/.env.example")

        expected_path = "/workspace/superarm_ws/roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf"
        for text in (compose_text, env_text):
            self.assertIn(expected_path, text)
            self.assertTrue(
                "NUM_JOINTS=6" in text or 'NUM_JOINTS: "${NUM_JOINTS:-6}"' in text
            )
            for joint_name in FEATURE_JOINTS:
                self.assertIn(joint_name, text)

        self.assertNotIn("/workspace/isaacsim/rpo_arm.urdf", compose_text)
        self.assertNotIn("/workspace/isaacsim/rpo_arm.urdf", env_text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
