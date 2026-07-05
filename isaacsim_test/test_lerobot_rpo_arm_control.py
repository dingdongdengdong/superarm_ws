from __future__ import annotations

import sys
import unittest
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
LEROBOT_DIR = REPO_ROOT / "isaacsim_test" / "lerobot"
if str(LEROBOT_DIR) not in sys.path:
    sys.path.insert(0, str(LEROBOT_DIR))

from isaacsim_rpo_arm_robot import (  # noqa: E402
    ARM_JOINT_NAMES,
    IsaacSimRpoArmConfig,
    IsaacSimRpoArmRobot,
)
from verify_lerobot_sitl import (  # noqa: E402
    _default_target_for_config,
    _load_config,
    _validate_config_joint_names,
)


class LeRobotRpoArmControlTest(unittest.TestCase):
    def test_arm_only_config_declares_five_arm_joints_and_fixed_hand(self):
        config_path = LEROBOT_DIR / "rpo_arm_isaacsim_arm_only.yaml"
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))

        self.assertEqual(raw["_type"], "isaacsim_rpo_arm")
        self.assertEqual(raw["joint_names"], ARM_JOINT_NAMES)
        self.assertTrue(raw["fixed_hand"])
        self.assertEqual(raw["fixed_grasp"], 0.0)
        self.assertNotIn("amazinghand_grasp", raw["joint_names"])

    def test_arm_only_normalize_does_not_clamp_last_arm_joint(self):
        config = IsaacSimRpoArmConfig(
            joint_names=list(ARM_JOINT_NAMES),
            fixed_hand=True,
            fixed_grasp=0.0,
            mock=True,
        )
        robot = IsaacSimRpoArmRobot(config)

        normalized = robot._normalize_vector([0.1, -0.2, 0.3, -0.4, -0.5])

        self.assertEqual(normalized, [0.1, -0.2, 0.3, -0.4, -0.5])

    def test_full_config_fixed_hand_pins_grasp_when_present(self):
        config = IsaacSimRpoArmConfig(fixed_hand=True, fixed_grasp=0.25, mock=True)
        robot = IsaacSimRpoArmRobot(config)

        normalized = robot._normalize_vector([0.1, -0.2, 0.3, -0.4, -0.5, 1.0])

        self.assertEqual(normalized, [0.1, -0.2, 0.3, -0.4, -0.5, 0.25])

    def test_verify_default_target_follows_loaded_config_joint_count(self):
        full_config = _load_config(LEROBOT_DIR / "rpo_arm_isaacsim.yaml")
        arm_only_config = _load_config(LEROBOT_DIR / "rpo_arm_isaacsim_arm_only.yaml")

        self.assertEqual(len(_default_target_for_config(full_config)), 6)
        self.assertEqual(len(_default_target_for_config(arm_only_config)), 5)
        self.assertEqual(_default_target_for_config(arm_only_config), [0.2, 0.1, -0.2, 0.3, -0.15])

    def test_custom_source_arm_config_allows_non_roboparty_joint_names(self):
        config = IsaacSimRpoArmConfig(
            joint_names=["joint_rev_1", "joint_rev_2", "joint_rev_3", "joint_rev_4"],
            fixed_hand=True,
            fixed_grasp=0.0,
            allow_custom_joint_names=True,
            mock=True,
        )

        _validate_config_joint_names(config)

        robot = IsaacSimRpoArmRobot(config)
        self.assertEqual(robot._feature_keys, [
            "joint_rev_1.pos",
            "joint_rev_2.pos",
            "joint_rev_3.pos",
            "joint_rev_4.pos",
        ])


if __name__ == "__main__":
    unittest.main()
