from __future__ import annotations

import json
import re
import tempfile
import unittest
from pathlib import Path

from isaacsim_test.lerobot.lelab_isaacsim51_control_contract import (
    CONTROL_FIELDS,
    FIVE_DOF_GRASP_TEST_CASES,
    create_timestamped_artifact_root,
    detect_isaacsim_compatibility,
    render_lelab_superarm_html_snapshot,
    write_contract_artifacts,
)


class LeLabIsaacSim51ControlContractTest(unittest.TestCase):
    def test_detects_local_isaacsim_51_as_compatible_target(self) -> None:
        compat = detect_isaacsim_compatibility(Path("/workspace/isaacsim/VERSION"))

        self.assertEqual(compat["required_major_minor"], "5.1")
        self.assertEqual(compat["detected_major_minor"], "5.1")
        self.assertTrue(compat["compatible"])
        self.assertIn("5.1.0", compat["detected_version"])


    def test_artifact_timestamp_must_be_utc_datetime(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(ValueError, "YYYYMMDDTHHMMSSZ"):
                create_timestamped_artifact_root(Path(tmp), now_utc="20260706TFINAL01Z")

    def test_control_fields_are_five_arm_dof_plus_grasp_in_order(self) -> None:
        self.assertEqual([field.name for field in CONTROL_FIELDS], [
            "right_arm_pitch_joint",
            "right_arm_roll_joint",
            "right_arm_yaw_joint",
            "right_elbow_pitch_joint",
            "right_elbow_yaw_joint",
            "amazinghand_grasp",
        ])
        self.assertEqual(len(CONTROL_FIELDS), 6)
        self.assertEqual(CONTROL_FIELDS[-1].minimum, 0.0)
        self.assertEqual(CONTROL_FIELDS[-1].maximum, 1.0)

    def test_test_cases_cover_each_control_dimension_once(self) -> None:
        self.assertEqual(len(FIVE_DOF_GRASP_TEST_CASES), 6)
        case_names = [case.name for case in FIVE_DOF_GRASP_TEST_CASES]
        self.assertEqual(case_names, [
            "pitch_positive",
            "roll_negative",
            "yaw_positive",
            "elbow_pitch_positive",
            "elbow_yaw_negative",
            "grasp_close",
        ])
        for index, case in enumerate(FIVE_DOF_GRASP_TEST_CASES):
            self.assertEqual(len(case.target), 6)
            active = [i for i, value in enumerate(case.target) if abs(value) > 1e-9]
            self.assertEqual(active, [index])


    def test_each_control_case_can_be_sent_through_mock_lerobot_backend(self) -> None:
        from isaacsim_test.lerobot.isaacsim_rpo_arm_robot import IsaacSimRpoArmConfig, IsaacSimRpoArmRobot

        robot = IsaacSimRpoArmRobot(IsaacSimRpoArmConfig(mock=True))
        robot.connect()

        for case in FIVE_DOF_GRASP_TEST_CASES:
            sent = robot.send_action(case.target).tolist()
            self.assertEqual(len(sent), len(case.target), case.name)
            for actual, expected in zip(sent, case.target, strict=True):
                self.assertAlmostEqual(actual, expected, places=6, msg=case.name)
            observed = robot.capture_observation()["observation.state"].tolist()
            for actual, expected in zip(observed, case.target, strict=True):
                self.assertAlmostEqual(actual, expected, places=6, msg=case.name)

        robot.disconnect()


    def test_real_lerobot_backend_send_action_requires_connection(self) -> None:
        from isaacsim_test.lerobot.isaacsim_rpo_arm_robot import IsaacSimRpoArmConfig, IsaacSimRpoArmRobot

        robot = IsaacSimRpoArmRobot(IsaacSimRpoArmConfig(mock=False))

        with self.assertRaisesRegex(RuntimeError, "not connected"):
            robot.send_action(FIVE_DOF_GRASP_TEST_CASES[0].target)

    def test_render_lelab_superarm_html_snapshot_exposes_six_sliders_and_presets(self) -> None:
        html = render_lelab_superarm_html_snapshot()

        self.assertIn("Isaac Sim 5.1", html)
        self.assertIn("rpo_arm_isaacsim.yaml", html)
        self.assertNotIn("source_arm_isaacsim_arm_only.yaml", html)
        self.assertEqual(len(re.findall(r'type="range"', html)), 6)
        for field in CONTROL_FIELDS:
            self.assertIn(field.name, html)
        for case in FIVE_DOF_GRASP_TEST_CASES:
            self.assertIn(case.name, html)

    def test_exported_lelab_patch_pins_roboparty_v2_right_arm_default(self) -> None:
        patch = Path(
            "isaacsim_test/lelab_patches/0005-Use-RoboParty-V2-right-arm-for-Isaac-Sim-backend.patch"
        ).read_text(encoding="utf-8")

        self.assertIn('+        or (lerobot_dir / "rpo_arm_isaacsim.yaml")', patch)
        self.assertIn('-        or (lerobot_dir / "source_arm_isaacsim_arm_only.yaml")', patch)
        self.assertIn('"right_arm_pitch_joint.pos"', patch)
        self.assertIn('"amazinghand_grasp.pos"', patch)
        self.assertIn("test_isaacsim_backend_default_config_is_roboparty_v2_right_arm", patch)

    def test_artifacts_use_datetime_folder_and_include_png_report_and_cases(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = create_timestamped_artifact_root(Path(tmp), now_utc="20260706T051500Z")
            report = write_contract_artifacts(root)

            self.assertEqual(root.name, "lelab_isaacsim51_control_20260706T051500Z")
            self.assertTrue((root / "logs" / "lelab_superarm_contract.log").is_file())
            self.assertTrue((root / "data" / "five_dof_grasp_cases.json").is_file())
            self.assertTrue((root / "lelab_superarm_control.html").is_file())
            self.assertTrue((root / "screenshots" / "lelab_superarm_control_verification.png").is_file())
            self.assertGreater((root / "screenshots" / "lelab_superarm_control_verification.png").stat().st_size, 500)
            self.assertEqual(report["test_case_count"], 6)
            self.assertTrue(report["lelab_superarm_png_evidence_path"].endswith("lelab_superarm_control_verification.png"))
            cases = json.loads((root / "data" / "five_dof_grasp_cases.json").read_text())
            self.assertEqual(len(cases["cases"]), 6)


if __name__ == "__main__":
    unittest.main()
