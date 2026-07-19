from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
LEROBOT_DIR = REPO_ROOT / "isaacsim_test" / "lerobot"
if str(LEROBOT_DIR) not in sys.path:
    sys.path.insert(0, str(LEROBOT_DIR))

from isaacsim_rpo_arm_robot import (  # noqa: E402
    ARM_JOINT_NAMES,
    HAND_ACTUATED_JOINT_NAMES,
    IsaacSimRpoArm,
    IsaacSimRpoArmConfig,
    IsaacSimRpoArmRobot,
    hand_grasp_scalar_action,
)
from isaacsim_test.isaacsim.graspable_hand_urdf import (  # noqa: E402
    fixed_hand_motion_library,
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

    def test_default_full_config_declares_fixed_visual_amazinghand(self):
        config_path = LEROBOT_DIR / "rpo_arm_isaacsim.yaml"
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))

        self.assertEqual(raw["joint_names"][-1], "amazinghand_grasp")
        self.assertTrue(raw["fixed_hand"])
        self.assertEqual(raw["fixed_grasp"], 0.0)

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
            joint_names=["joint_rev_1", "joint_rev_2", "joint_rev_3", "joint_rev_4", "joint_rev_5"],
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
            "joint_rev_5.pos",
        ])



    def test_hand_only_config_declares_eight_amazinghand_joints_and_hand_topics(self):
        config_path = LEROBOT_DIR / "amazinghand_isaacsim_hand_only.yaml"
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))

        self.assertEqual(raw["_type"], "isaacsim_rpo_arm")
        self.assertEqual(raw["joint_names"], HAND_ACTUATED_JOINT_NAMES)
        self.assertEqual(len(raw["joint_names"]), 8)
        self.assertEqual(raw["joint_state_topic"], "/hand/joint_states")
        self.assertEqual(raw["joint_command_topic"], "/hand/joint_commands")
        self.assertEqual(raw["phone_command_topic"], "/hand/leader_joint_commands")
        self.assertEqual(raw["screenshot_debug_topic"], "/hand/screenshot_debug")
        self.assertTrue(raw["allow_custom_joint_names"])
        self.assertFalse(raw.get("fixed_hand", False))
        self.assertEqual(raw["manual_leader"]["kind"], "amazinghand")
        self.assertEqual(raw["manual_leader"]["slider_min"], 0.0)
        self.assertEqual(raw["manual_leader"]["slider_max"], 1.2)
        self.assertGreaterEqual(float(raw["connect_timeout_s"]), 45.0)

    def test_combined_config_uses_six_logical_controls_and_fixed_hand_motions(self):
        config_path = LEROBOT_DIR / "source_arm_amazinghand.yaml"
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))

        self.assertEqual(raw["_type"], "isaacsim_rpo_arm")
        self.assertEqual(raw["joint_names"], [
            "joint_rev_1",
            "joint_rev_2",
            "joint_rev_3",
            "joint_rev_4",
            "joint_rev_5",
            "amazinghand_motion",
        ])
        self.assertEqual(raw["physical_joint_names"], [*raw["joint_names"][:5], *HAND_ACTUATED_JOINT_NAMES])
        self.assertEqual(len(raw["hand_motions"]), 3)
        expected = fixed_hand_motion_library()
        for actual, motion in zip(raw["hand_motions"], expected, strict=True):
            self.assertEqual(actual["name"], motion["name"])
            self.assertEqual(float(actual["code"]), motion["code"])
            self.assertEqual(
                [float(value) for value in actual["joint_targets"]],
                list(motion["joint_targets"].values()),
            )
        self.assertEqual(set(raw["arm_limits"]), set(raw["joint_names"][:5]))
        self.assertEqual(raw["manual_leader"]["hand_control"], "fixed_motions")

    def test_modern_lerobot_factory_constructs_six_control_robot(self):
        from lerobot.robots import make_robot_from_config
        from lerobot.robots.robot import Robot

        raw = yaml.safe_load((LEROBOT_DIR / "source_arm_amazinghand.yaml").read_text(encoding="utf-8"))
        raw.pop("_type")
        raw.pop("manual_leader")
        raw["mock"] = True
        config = IsaacSimRpoArmConfig(**raw)

        robot = make_robot_from_config(config)

        self.assertIsInstance(robot, Robot)
        self.assertIsInstance(robot, IsaacSimRpoArm)
        self.assertEqual(list(robot.action_features), [
            "joint_rev_1.pos",
            "joint_rev_2.pos",
            "joint_rev_3.pos",
            "joint_rev_4.pos",
            "joint_rev_5.pos",
            "amazinghand_motion.pos",
        ])
        self.assertTrue(all(feature is float for feature in robot.action_features.values()))
        self.assertEqual(robot.action_features, robot.observation_features)

    def test_modern_named_action_quantizes_motion_and_keeps_physical_state_separate(self):
        raw = yaml.safe_load((LEROBOT_DIR / "source_arm_amazinghand.yaml").read_text(encoding="utf-8"))
        raw.pop("_type")
        raw.pop("manual_leader")
        raw["mock"] = True
        robot = IsaacSimRpoArm(IsaacSimRpoArmConfig(**raw))
        robot.connect()
        action = {
            "joint_rev_1.pos": 0.1,
            "joint_rev_2.pos": -0.2,
            "joint_rev_3.pos": 0.3,
            "joint_rev_4.pos": -0.4,
            "joint_rev_5.pos": 0.5,
            "amazinghand_motion.pos": 0.77,
        }

        sent = robot.send_action(action)
        observation = robot.get_observation()
        physical = robot.get_visualization_joints()

        self.assertEqual(sent["amazinghand_motion.pos"], 1.0)
        self.assertEqual(observation, sent)
        self.assertEqual(len(observation), 6)
        self.assertEqual(len(physical), 13)
        self.assertEqual(physical["finger1_motor1"], 0.95)
        self.assertEqual(physical["finger1_motor2"], 1.1)
        robot.disconnect()

    def test_modern_robot_includes_configured_camera_in_recording_observation(self):
        import isaacsim_rpo_arm_robot as robot_module

        class _FakeCamera:
            height = 2
            width = 3
            use_rgb = True
            use_depth = False
            is_connected = False

            def connect(self):
                self.is_connected = True

            def read_latest(self):
                return np.ones((2, 3, 3), dtype=np.uint8)

            def disconnect(self):
                self.is_connected = False

        class _FakeCameraConfig:
            width = 3
            height = 2
            fps = 30

        original_factory = robot_module._make_cameras_from_configs
        robot_module._make_cameras_from_configs = lambda configs: {"wrist": _FakeCamera()}
        self.addCleanup(setattr, robot_module, "_make_cameras_from_configs", original_factory)
        config = IsaacSimRpoArmConfig(
            joint_names=[
                "joint_rev_1",
                "joint_rev_2",
                "joint_rev_3",
                "joint_rev_4",
                "joint_rev_5",
                "amazinghand_motion",
            ],
            mock=True,
            cameras={"wrist": _FakeCameraConfig()},
        )
        robot = IsaacSimRpoArm(config)

        robot.connect()
        observation = robot.get_observation()

        self.assertEqual(robot.observation_features["wrist"], (2, 3, 3))
        self.assertEqual(observation["wrist"].shape, (2, 3, 3))
        self.assertTrue(robot.is_connected)
        robot.disconnect()
        self.assertFalse(robot.is_connected)

    def test_hand_grasp_scalar_action_maps_to_eight_joint_action_vector(self):
        open_action = hand_grasp_scalar_action(0.0)
        half_action = hand_grasp_scalar_action(0.5)
        closed_action = hand_grasp_scalar_action(1.0)

        self.assertEqual(len(open_action), 8)
        self.assertEqual(len(half_action), 8)
        self.assertEqual(len(closed_action), 8)
        self.assertEqual(open_action, [0.05, 0.02] * 4)
        self.assertEqual(half_action, [0.5, 0.56] * 4)
        self.assertEqual(closed_action, [0.95, 1.1] * 4)

    def test_hand_only_normalize_preserves_all_eight_hand_joint_values(self):
        config = IsaacSimRpoArmConfig(
            joint_names=list(HAND_ACTUATED_JOINT_NAMES),
            allow_custom_joint_names=True,
            mock=True,
        )
        robot = IsaacSimRpoArmRobot(config)

        normalized = robot._normalize_vector([0.05, 0.02, 0.5, 0.56, 0.95, 1.1, -0.1, 1.2])

        self.assertEqual(normalized, [0.05, 0.02, 0.5, 0.56, 0.95, 1.1, -0.1, 1.2])

    def test_screenshot_debug_publishes_json_payload_when_debug_publisher_is_available(self):
        config = IsaacSimRpoArmConfig(mock=True)
        robot = IsaacSimRpoArmRobot(config)
        published = []

        class _FakePublisher:
            def publish(self, msg):
                published.append(msg.data)

        robot._debug_pub = _FakePublisher()

        payload = robot.publish_screenshot_debug({"capture_every_command": True, "output_dir": "/tmp/screens"})

        self.assertEqual(payload["capture_every_command"], True)
        self.assertEqual(payload["output_dir"], "/tmp/screens")
        self.assertEqual(len(published), 1)
        self.assertIn('"capture_every_command": true', published[0])
        self.assertIn('"output_dir": "/tmp/screens"', published[0])

    def test_send_action_becomes_latest_repeated_realtime_command(self):
        config = IsaacSimRpoArmConfig(
            joint_names=list(HAND_ACTUATED_JOINT_NAMES),
            allow_custom_joint_names=True,
            mock=False,
        )
        robot = IsaacSimRpoArmRobot(config)
        published = []

        class _FakePublisher:
            def publish(self, msg):
                published.append(list(msg.data))

        robot._pub = _FakePublisher()

        robot.send_action([0.05, 0.02] * 4)
        robot.teleop_step(record_data=False)

        self.assertEqual(len(published), 2)
        for actual, expected in zip(published[0], [0.05, 0.02] * 4, strict=True):
            self.assertAlmostEqual(actual, expected, places=5)
        for actual, expected in zip(published[1], [0.05, 0.02] * 4, strict=True):
            self.assertAlmostEqual(actual, expected, places=5)

if __name__ == "__main__":
    unittest.main()
