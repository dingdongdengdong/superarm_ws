import json
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools" / "hitl" / "passive_canable_check.py"


class PassiveCanableCheckTest(unittest.TestCase):
    def test_help_documents_no_transmit_safety_contract(self):
        result = subprocess.run(
            [sys.executable, str(SCRIPT), "--help"],
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("no CAN frames are transmitted", result.stdout)
        self.assertIn("--channel", result.stdout)
        self.assertIn("--bitrate", result.stdout)

    def test_json_result_marks_passive_listen_as_non_motion(self):
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
        self.assertEqual(payload["mode"], "passive_listen")
        self.assertFalse(payload["transmits_can_frames"])
        self.assertFalse(payload["motor_enable_allowed"])
        self.assertEqual(payload["status"], "skipped_open")


if __name__ == "__main__":
    unittest.main()
