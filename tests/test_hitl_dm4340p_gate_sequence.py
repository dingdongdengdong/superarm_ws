import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROTOCOL = ROOT / "tools" / "hitl" / "dm4340p_protocol.py"
RUNNER = ROOT / "tools" / "hitl" / "dm4340p_gate_runner.py"
DEFAULT_CONFIG = ROOT / "configs" / "hitl" / "dm4340p_x2_read_only.json"


def load_protocol():
    spec = importlib.util.spec_from_file_location("dm4340p_protocol", PROTOCOL)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class Dm4340pGateSequenceTest(unittest.TestCase):
    def test_system_disable_frame_matches_protocol_bytes(self):
        protocol = load_protocol()
        frame = protocol.build_system_frame(send_id=1, command="disable")

        self.assertEqual(frame.arbitration_id, 1)
        self.assertEqual(frame.data_hex, "fffffffffffffffd")
        self.assertEqual(frame.kind, "system_disable")
        self.assertTrue(frame.transmits_if_executed)

    def test_enable_disable_plan_orders_enable_then_disable_but_blocks_execution(self):
        protocol = load_protocol()
        cfg = protocol.load_gate_config(DEFAULT_CONFIG)
        plan = protocol.build_enable_disable_plan(cfg, motor_label="first_motor_id1_confirmed_single_motor_hitl_2026_07_10")

        self.assertFalse(plan.execute_allowed)
        self.assertIn("Gate C disable proof is not passed on real hardware", " ".join(plan.block_reasons))
        self.assertEqual([frame.kind for frame in plan.frames], ["system_enable", "system_disable"])
        self.assertEqual([frame.data_hex for frame in plan.frames], ["fffffffffffffffc", "fffffffffffffffd"])

    def test_tiny_motion_plan_rejects_steps_over_one_hundredth_radian(self):
        protocol = load_protocol()
        cfg = protocol.load_gate_config(DEFAULT_CONFIG)

        with self.assertRaisesRegex(ValueError, "max tiny relative step is 0.01 rad"):
            protocol.build_tiny_motion_plan(cfg, motor_label="first_motor_id1_confirmed_single_motor_hitl_2026_07_10", relative_step_rad=0.02)

    def test_tiny_motion_plan_is_blocked_until_prior_gate_and_confirmed_sign_zero(self):
        protocol = load_protocol()
        cfg = protocol.load_gate_config(DEFAULT_CONFIG)
        plan = protocol.build_tiny_motion_plan(cfg, motor_label="first_motor_id1_confirmed_single_motor_hitl_2026_07_10", relative_step_rad=0.005)

        self.assertFalse(plan.execute_allowed)
        reasons = " ".join(plan.block_reasons)
        self.assertIn("Gate D enable-disable proof is not passed", reasons)
        self.assertIn("sign/zero/clamps are not confirmed", reasons)
        self.assertEqual(plan.relative_step_rad, 0.005)
        self.assertEqual(plan.frames, [])

    def test_two_motor_parity_plan_blocks_until_both_single_motor_checks_pass(self):
        protocol = load_protocol()
        cfg = protocol.load_gate_config(DEFAULT_CONFIG)
        plan = protocol.build_two_motor_parity_plan(cfg)

        self.assertFalse(plan.execute_allowed)
        self.assertIn("two verified single-motor tiny-motion results are required", " ".join(plan.block_reasons))
        self.assertEqual(plan.milestone, "F")

    def test_cli_dry_run_disable_proof_emits_no_transmit_and_blocked_gate(self):
        result = subprocess.run(
            [
                sys.executable,
                str(RUNNER),
                "disable-proof",
                "--config",
                str(DEFAULT_CONFIG),
                "--motor-label",
                "first_motor_id1_confirmed_single_motor_hitl_2026_07_10",
                "--json",
            ],
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        payload = json.loads(result.stdout)
        self.assertEqual(payload["milestone"], "C")
        self.assertFalse(payload["execute_allowed"])
        self.assertFalse(payload["transmitted"])
        self.assertEqual(payload["frames"][0]["data_hex"], "fffffffffffffffd")

    def test_lelab_integration_plan_blocks_until_milestone_f_passes(self):
        protocol = load_protocol()
        cfg = protocol.load_gate_config(DEFAULT_CONFIG)
        plan = protocol.build_lelab_integration_plan(cfg)

        self.assertEqual(plan.milestone, "G")
        self.assertFalse(plan.execute_allowed)
        self.assertFalse(plan.transmitted)
        self.assertEqual(plan.frames, [])
        reasons = " ".join(plan.block_reasons)
        self.assertIn("Milestone F two-motor parity is not passed", reasons)
        self.assertIn("LeLab UI must remain read-only/disabled", reasons)

    def test_policy_readiness_plan_blocks_until_safe_backend_and_calibration_exist(self):
        protocol = load_protocol()
        cfg = protocol.load_gate_config(DEFAULT_CONFIG)
        plan = protocol.build_policy_readiness_plan(cfg)

        self.assertEqual(plan.milestone, "H")
        self.assertFalse(plan.execute_allowed)
        self.assertFalse(plan.transmitted)
        self.assertEqual(plan.frames, [])
        reasons = " ".join(plan.block_reasons)
        self.assertIn("Milestone G LeLab safe-backend integration is not passed", reasons)
        self.assertIn("calibration startup and recovery procedure are missing", reasons)

    def test_cli_lelab_and_policy_plans_emit_blocked_no_transmit_outputs(self):
        commands = [
            ("lelab-integration-plan", "G"),
            ("policy-readiness-plan", "H"),
        ]
        for command, milestone in commands:
            with self.subTest(command=command):
                result = subprocess.run(
                    [
                        sys.executable,
                        str(RUNNER),
                        command,
                        "--config",
                        str(DEFAULT_CONFIG),
                        "--json",
                    ],
                    cwd=ROOT,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    check=False,
                )

                self.assertEqual(result.returncode, 0, result.stderr)
                payload = json.loads(result.stdout)
                self.assertEqual(payload["milestone"], milestone)
                self.assertFalse(payload["execute_allowed"])
                self.assertFalse(payload["transmitted"])
                self.assertEqual(payload["frames"], [])


if __name__ == "__main__":
    unittest.main()
