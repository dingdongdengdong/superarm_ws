from __future__ import annotations

import math
import unittest

from isaacsim_test.lerobot.rpo_arm_contract import (
    AMAZINGHAND_MOTOR_JOINT_NAMES,
    ARM_JOINT_LIMITS,
    FEATURE_KEYS,
    JOINT_NAMES,
    grasp_scalar_to_servo_targets,
    normalize_action,
)


class RpoArmContractTest(unittest.TestCase):
    def test_normalize_action_clamps_arm_limits_and_grasp_scalar(self) -> None:
        raw = [9.0, -9.0, 9.0, -9.0, 9.0, 2.0, 123.0]

        normalized = normalize_action(raw)

        self.assertEqual(len(normalized), 6)
        for value, joint_name in zip(normalized[:5], JOINT_NAMES[:5], strict=True):
            lower, upper = ARM_JOINT_LIMITS[joint_name]
            self.assertGreaterEqual(value, lower)
            self.assertLessEqual(value, upper)
        self.assertEqual(
            normalized[:5],
            [
                ARM_JOINT_LIMITS["right_arm_pitch_joint"][1],
                ARM_JOINT_LIMITS["right_arm_roll_joint"][0],
                ARM_JOINT_LIMITS["right_arm_yaw_joint"][1],
                ARM_JOINT_LIMITS["right_elbow_pitch_joint"][0],
                ARM_JOINT_LIMITS["right_elbow_yaw_joint"][1],
            ],
        )
        self.assertEqual(normalized[5], 1.0)

    def test_normalize_action_pads_missing_values_with_safe_defaults(self) -> None:
        self.assertEqual(normalize_action([0.25]), [0.25, 0.0, 0.0, 0.0, 0.0, 0.0])

    def test_contract_feature_keys_match_sitl_order(self) -> None:
        self.assertEqual(
            FEATURE_KEYS,
            [
                "right_arm_pitch_joint.pos",
                "right_arm_roll_joint.pos",
                "right_arm_yaw_joint.pos",
                "right_elbow_pitch_joint.pos",
                "right_elbow_yaw_joint.pos",
                "amazinghand_grasp.pos",
            ],
        )

    def test_grasp_scalar_maps_to_eight_amazinghand_servo_targets(self) -> None:
        targets = grasp_scalar_to_servo_targets(
            0.5,
            middle_pos_deg=[3, 0, -5, -8, -2, 5, -12, 0],
        )

        self.assertEqual(set(targets), set(range(1, 9)))
        self.assertAlmostEqual(targets[1], math.radians(30.5))
        self.assertAlmostEqual(targets[2], math.radians(-27.5))
        self.assertAlmostEqual(targets[7], math.radians(15.5))

    def test_amazinghand_motor_joint_order_matches_servo_target_order(self) -> None:
        self.assertEqual(
            AMAZINGHAND_MOTOR_JOINT_NAMES,
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
        self.assertEqual(len(grasp_scalar_to_servo_targets(0.25)), len(AMAZINGHAND_MOTOR_JOINT_NAMES))

    def test_grasp_scalar_is_clamped_before_servo_mapping(self) -> None:
        closed = grasp_scalar_to_servo_targets(2.0)
        open_ = grasp_scalar_to_servo_targets(-1.0)

        self.assertAlmostEqual(closed[1], math.radians(90.0))
        self.assertAlmostEqual(open_[1], math.radians(-35.0))


if __name__ == "__main__":
    unittest.main()
