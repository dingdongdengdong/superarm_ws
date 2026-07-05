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


    def test_simready_usd_artifact_is_committed_and_profile_passed(self) -> None:
        simready_usd = ROOT / "isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd"
        simready_profile = ROOT / "isaacsim_test/outputs/simready/echo_full/pipeline/06_validation_final/simready-profile.json"

        self.assertTrue(simready_usd.is_file(), simready_usd)
        self.assertTrue(simready_profile.is_file(), simready_profile)
        self.assertIn('"passed": true', simready_profile.read_text(encoding="utf-8"))

    def test_compose_and_env_expose_simready_usd_path(self) -> None:
        compose_text = _read("isaacsim_test/docker-compose.yml")
        env_text = _read("isaacsim_test/.env.example")
        expected_path = "/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd"

        for text in (compose_text, env_text):
            self.assertIn("SIMREADY_USD_PATH", text)
            self.assertIn(expected_path, text)

    def test_compose_and_env_expose_simready_thumbnail_fallback_path(self) -> None:
        compose_text = _read("isaacsim_test/docker-compose.yml")
        env_text = _read("isaacsim_test/.env.example")
        expected_path = "/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/07_render/thumbnail.png"

        for text in (compose_text, env_text):
            self.assertIn("SIMREADY_THUMBNAIL_PATH", text)
            self.assertIn(expected_path, text)

    def test_scene_supports_simready_usd_import_and_mapping_evidence(self) -> None:
        scene_text = _read("isaacsim_test/isaacsim/setup_rpo_arm_scene.py")

        self.assertIn("SIMREADY_USD_PATH", scene_text)
        self.assertIn("Loading SimReady USD", scene_text)
        self.assertIn("echo_full_robot_arm_hand.usd", scene_text)
        self.assertIn("simready_prim_mapping.json", scene_text)
        self.assertIn("simready_root_prim", scene_text)
        self.assertIn("prim_hierarchy", scene_text)
        self.assertIn("bound_or_binding_pending_per_feature", scene_text)
        self.assertIn("binding_pending", scene_text)
        self.assertIn("AddReference", scene_text)

    def test_scene_supports_screenshot_after_lerobot_command(self) -> None:
        scene_text = _read("isaacsim_test/isaacsim/setup_rpo_arm_scene.py")
        compose_text = _read("isaacsim_test/docker-compose.yml")

        for env_name in ("SCREENSHOT_AFTER_COMMAND", "SCREENSHOT_ON_STARTUP", "SCREENSHOT_PATH", "EXIT_AFTER_SCREENSHOT"):
            self.assertIn(env_name, scene_text)
            self.assertIn(env_name, compose_text)
        self.assertIn("echo_full_simready_target.png", scene_text)
        self.assertIn("echo_full_simready_target.png", compose_text)
        self.assertNotIn("rpo_v2_lerobot_target.png", compose_text)
        isaac_service = compose_text.split("  # LeRobot", maxsplit=1)[0]
        self.assertIn('user: "0:0"', isaac_service)
        self.assertIn(
            "${SUPERARM_WS_PATH:?Set SUPERARM_WS_PATH in isaacsim_test/.env}:/workspace/superarm_ws:rw",
            isaac_service,
        )
        self.assertIn("SIMREADY_THUMBNAIL_PATH", scene_text)
        self.assertIn("thumbnail.png", scene_text)
        self.assertIn("_write_fallback_visual_evidence", scene_text)
        self.assertIn("Fallback visual evidence saved", scene_text)
        self.assertNotIn("rep.orchestrator.step", scene_text)
        self.assertIn('enable_extension("isaacsim.test.utils")', scene_text)
        self.assertIn('enable_extension("omni.kit.renderer.capture")', scene_text)
        self.assertIn("capture_next_frame_rp_resource", scene_text)
        self.assertIn("capture_viewport_to_file", scene_text)
        self.assertIn("last_applied_command", scene_text)
        self.assertIn("threading.Thread(target=rclpy.spin", scene_text)
        self.assertIn("last_processed_command_seq", scene_text)
        self.assertIn("_publish_current_state", scene_text)
        self.assertIn("Startup screenshot trigger accepted", scene_text)

    def test_scene_can_use_env_joint_names_for_local_source_urdf_control(self) -> None:
        scene_text = _read("isaacsim_test/isaacsim/setup_rpo_arm_scene.py")

        self.assertIn("JOINT_NAMES", scene_text)
        self.assertIn("_parse_controlled_arm_joint_names", scene_text)
        self.assertIn("joint_rev_1", scene_text)
        self.assertIn("COMMAND_EVIDENCE_PATH", scene_text)
        self.assertIn("articulation_readback", scene_text)

    def test_lerobot_sitl_verifier_uses_robot_config_and_checks_tolerance(self) -> None:
        verifier_text = _read("isaacsim_test/lerobot/verify_lerobot_sitl.py")

        self.assertIn("rpo_arm_isaacsim.yaml", verifier_text)
        self.assertIn("IsaacSimRpoArmRobot", verifier_text)
        self.assertIn("send_action", verifier_text)
        self.assertIn("capture_observation", verifier_text)
        self.assertIn("0.03", verifier_text)
        for joint_name in FEATURE_JOINTS:
            self.assertIn(joint_name, verifier_text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
