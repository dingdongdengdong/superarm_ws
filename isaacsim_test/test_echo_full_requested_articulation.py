"""Regression checks for the user-facing echo_full SITL articulation contract.

The requested output root now contains two coordinated artifacts:

* a small custom visual/provenance USDA loaded with ``CUSTOM_VISUAL_USD_PATH``;
* a direct physical arm+AmazingHand URDF loaded by Isaac Sim with
  ``PHYSICAL_ROBOT_URDF_PATH`` and ``URDFParseAndImportFile``.

These tests protect the finger-simulation contract: the manifest must point
Isaac Sim at the direct URDF importer path and the report must record the
AmazingHand motor joints that the LeRobot grasp scalar commands.
"""

from __future__ import annotations

import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REQUESTED_ROOT = ROOT / "isaacsim_test/outputs/simready/echo_full"
ARTICULATION_USD = REQUESTED_ROOT / "sitl/echo_full_lerobot_articulation.usda"
ARTICULATION_REPORT = REQUESTED_ROOT / "sitl/echo_full_lerobot_articulation_report.json"
FINAL_SIMREADY_USD = (
    REQUESTED_ROOT
    / "pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd"
)


AMAZINGHAND_MOTOR_JOINTS = [
    "finger1_motor1",
    "finger1_motor2",
    "finger2_motor1",
    "finger2_motor2",
    "finger3_motor1",
    "finger3_motor2",
    "finger4_motor1",
    "finger4_motor2",
]


class EchoFullRequestedArticulationArtifactTests(unittest.TestCase):
    def test_requested_path_articulation_usd_is_direct_urdf_visual_manifest(self) -> None:
        self.assertTrue(FINAL_SIMREADY_USD.is_file(), FINAL_SIMREADY_USD)
        self.assertTrue(ARTICULATION_USD.is_file(), ARTICULATION_USD)
        text = ARTICULATION_USD.read_text(encoding="utf-8", errors="ignore")

        self.assertIn('defaultPrim = "echo_full"', text)
        self.assertIn("custom_visual_usda_plus_direct_arm_hand_urdf", text)
        self.assertIn("URDFParseAndImportFile", text)
        self.assertIn("direct_urdf_import_artifact", text)
        self.assertIn("PHYSICAL_ROBOT_URDF_PATH", text)
        self.assertIn("roboto_v2_right_arm_amazinghand_full.urdf", text)
        for joint_name in AMAZINGHAND_MOTOR_JOINTS:
            self.assertIn(joint_name, text)

    def test_requested_path_articulation_report_marks_direct_urdf_finger_simulation(self) -> None:
        self.assertTrue(ARTICULATION_REPORT.is_file(), ARTICULATION_REPORT)
        report = json.loads(ARTICULATION_REPORT.read_text(encoding="utf-8"))

        self.assertEqual(report["status"], "PASS")
        self.assertEqual(report["root_prim"], "/echo_full")
        self.assertEqual(report["articulation_root"], "generated_by_isaac_urdf_importer_at_runtime")
        self.assertEqual(report["direct_urdf_import"]["mode"], "direct_urdf_import_artifact")
        self.assertEqual(report["direct_urdf_import"]["isaac_importer"], "URDFParseAndImportFile")
        self.assertFalse(report["direct_urdf_import"]["synthetic_usd_reconstruction"])
        self.assertIn("right_arm_pitch_joint", report["controlled_joints"])
        self.assertEqual(report["hand_motor_joints"], AMAZINGHAND_MOTOR_JOINTS)
        self.assertEqual(
            report["direct_urdf_import"]["hand_motor_joints"],
            AMAZINGHAND_MOTOR_JOINTS,
        )
        self.assertEqual(
            report["visual_binding_status"]["amazinghand"]["finger_dofs"],
            "present_in_physical_urdf_and_commanded_from_lerobot_grasp_scalar",
        )
        self.assertEqual(
            report["urdf_constraint_fidelity"]["status"],
            "LOSSY_MJCF_CONVERSION",
        )


if __name__ == "__main__":
    unittest.main()
