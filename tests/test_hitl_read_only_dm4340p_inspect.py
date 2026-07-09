import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools" / "hitl" / "read_only_dm4340p_inspect.py"


def load_module():
    spec = importlib.util.spec_from_file_location("read_only_dm4340p_inspect", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class ReadOnlyDm4340pInspectTest(unittest.TestCase):
    def test_skip_open_json_schema_declares_no_transmit_and_no_enable(self):
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                "--skip-open",
                "--channel",
                "/dev/ttyACM-test",
                "--bitrate",
                "1000000",
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
        self.assertEqual(payload["channel"], "/dev/ttyACM-test")
        self.assertEqual(payload["bitrate"], 1_000_000)
        self.assertEqual(payload["mode"], "read_only_status_inspect")
        self.assertFalse(payload["transmits_can_frames"])
        self.assertFalse(payload["motor_enable_allowed"])
        self.assertFalse(payload["motion_command_allowed"])
        self.assertEqual(payload["status"], "skipped_open")
        self.assertEqual(payload["detected_ids"], [])
        self.assertEqual(payload["status_frames"], [])

    def test_collect_status_frames_filters_error_frames_and_records_ids(self):
        module = load_module()
        messages = [
            SimpleNamespace(is_error_frame=True, arbitration_id=0x321, data=b"\x01\x02"),
            SimpleNamespace(is_error_frame=False, arbitration_id=0x101, data=b"\x01\x02\x03"),
            SimpleNamespace(is_error_frame=False, arbitration_id=0x102, data=bytearray([0xAA, 0xBB])),
            None,
        ]

        class FakeBus:
            def recv(self, timeout=0.0):
                return messages.pop(0) if messages else None

        result = module.collect_status_frames(FakeBus(), max_frames=4, duration_s=0.01)

        self.assertEqual(result.error_frames_filtered, 1)
        self.assertEqual(result.non_error_frames, 2)
        self.assertEqual(result.detected_ids, [0x101, 0x102])
        self.assertEqual(
            result.status_frames,
            [
                {"arbitration_id": 0x101, "arbitration_id_hex": "0x101", "dlc": 3, "data_hex": "010203"},
                {"arbitration_id": 0x102, "arbitration_id_hex": "0x102", "dlc": 2, "data_hex": "aabb"},
            ],
        )

    def test_help_documents_read_only_safety_boundary(self):
        result = subprocess.run(
            [sys.executable, str(SCRIPT), "--help"],
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("read-only", result.stdout)
        self.assertIn("no torque enable", result.stdout)
        self.assertIn("no motion", result.stdout)

    def test_config_comparison_marks_unexpected_ids_without_enabling_motion(self):
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as handle:
            json.dump({
                "channel": "/dev/ttyACM-test",
                "bitrate": 1000000,
                "motion_disabled": True,
                "motors": [
                    {"label": "a", "joint_name": "j0", "send_id": 1, "status_id": 17, "mode": "MIT/CAN confirmed externally", "confirmed": True},
                    {"label": "b", "joint_name": "j1", "send_id": 2, "status_id": 18, "mode": "MIT/CAN confirmed externally", "confirmed": True},
                ],
            }, handle)
            config_path = handle.name

        module = load_module()
        result = module.ReadOnlyInspectResult(
            channel="/dev/ttyACM-test",
            bitrate=1_000_000,
            duration_s=0.0,
            max_frames=20,
            status="ok",
            non_error_frames=1,
            detected_ids=[17, 99],
            status_frames=[{"arbitration_id": 99, "arbitration_id_hex": "0x63", "dlc": 0, "data_hex": ""}],
        )

        module.apply_config_comparison(result, config_path)
        payload = result.to_dict()

        self.assertEqual(payload["status"], "unexpected_ids")
        self.assertEqual(payload["gate_state"], "blocked")
        self.assertEqual(payload["unexpected_ids"], [99])
        self.assertFalse(payload["transmits_can_frames"])
        self.assertFalse(payload["motor_enable_allowed"])
        self.assertFalse(payload["motion_command_allowed"])


if __name__ == "__main__":
    unittest.main()
