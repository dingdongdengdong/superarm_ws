import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
BACKEND = ROOT / "tools" / "hitl" / "read_only_backend.py"


def load_backend():
    spec = importlib.util.spec_from_file_location("read_only_backend", BACKEND)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class ReadOnlyBackendConfigTest(unittest.TestCase):
    def write_config(self, payload):
        handle = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
        json.dump(payload, handle)
        handle.close()
        return Path(handle.name)

    def base_payload(self):
        return {
            "channel": "/dev/ttyACM-test",
            "bitrate": 1000000,
            "motion_disabled": True,
            "motors": [
                {
                    "label": "dm4340p_a",
                    "joint_name": "right_arm_pitch_joint",
                    "send_id": 1,
                    "status_id": 17,
                    "mode": "MIT/CAN unknown until vendor confirmation",
                    "confirmed": False,
                },
                {
                    "label": "dm4340p_b",
                    "joint_name": "right_arm_roll_joint",
                    "send_id": 2,
                    "status_id": 18,
                    "mode": "MIT/CAN unknown until vendor confirmation",
                    "confirmed": False,
                },
            ],
        }

    def test_load_config_keeps_unconfirmed_placeholder_ids_blocked(self):
        backend = load_backend()
        cfg = backend.load_read_only_config(self.write_config(self.base_payload()))

        self.assertEqual(cfg.channel, "/dev/ttyACM-test")
        self.assertEqual(cfg.bitrate, 1_000_000)
        self.assertFalse(cfg.can_complete_gate2)
        self.assertEqual(cfg.expected_status_ids, [])
        self.assertIn("unconfirmed", cfg.block_reasons[0])

    def test_load_config_rejects_duplicate_confirmed_status_ids(self):
        backend = load_backend()
        payload = self.base_payload()
        for motor in payload["motors"]:
            motor["confirmed"] = True
            motor["status_id"] = 17

        with self.assertRaisesRegex(ValueError, "duplicate status_id"):
            backend.load_read_only_config(self.write_config(payload))

    def test_compare_observed_ids_blocks_unexpected_id(self):
        backend = load_backend()
        payload = self.base_payload()
        payload["motors"][0]["confirmed"] = True
        payload["motors"][1]["confirmed"] = True
        cfg = backend.load_read_only_config(self.write_config(payload))

        comparison = backend.compare_observed_ids(cfg, [17, 99])

        self.assertEqual(comparison.gate_state, "blocked")
        self.assertEqual(comparison.unexpected_ids, [99])
        self.assertEqual(comparison.missing_expected_ids, [18])

    def test_backend_blocks_all_transmit_enable_and_motion_methods(self):
        backend = load_backend()
        cfg = backend.load_read_only_config(self.write_config(self.base_payload()))
        safety_backend = backend.ReadOnlySafetyBackend(cfg)

        self.assertEqual(safety_backend.state.value, "disconnected")
        safety_backend.connect_read_only()
        self.assertEqual(safety_backend.state.value, "read_only")
        self.assertFalse(safety_backend.transmits_can_frames)

        for method_name in ["send_frame", "enable_motor", "disable_motor", "command_motion"]:
            with self.subTest(method_name=method_name):
                with self.assertRaisesRegex(PermissionError, "blocked by read-only HITL gate"):
                    getattr(safety_backend, method_name)(SimpleNamespace())

    def test_default_read_only_config_records_single_motor_hitl_state(self):
        backend = load_backend()
        cfg = backend.load_read_only_config(ROOT / "configs" / "hitl" / "dm4340p_x2_read_only.json")

        self.assertEqual([motor.joint_name for motor in cfg.motors], ["first_motor", "second_motor_not_connected"])
        self.assertFalse(cfg.can_complete_gate2)
        self.assertEqual(cfg.expected_status_ids, [0])
        self.assertIn("second_motor_not_connected", " ".join(cfg.block_reasons))


if __name__ == "__main__":
    unittest.main()
