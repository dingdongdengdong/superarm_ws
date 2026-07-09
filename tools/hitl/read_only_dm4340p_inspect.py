#!/usr/bin/env python3
"""Read-only DM4340P/CANable status-frame inspection for HITL Gate 2.

Safety contract: this tool opens a CANable slcan channel and observes received
CAN frames only. It never sends CAN frames, never enables torque, and never
commands motion. It records non-error frame IDs/status bytes if any are already
visible on the bus.
"""

from __future__ import annotations

import argparse
import json
import time
import importlib.util
from pathlib import Path
from typing import Any


def _load_backend_module():
    module_path = Path(__file__).with_name("read_only_backend.py")
    spec = importlib.util.spec_from_file_location("read_only_backend", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class ReadOnlyInspectResult:
    def __init__(
        self,
        *,
        channel: str,
        bitrate: int,
        duration_s: float,
        max_frames: int,
        status: str = "not_started",
        non_error_frames: int = 0,
        error_frames_filtered: int = 0,
        detected_ids: list[int] | None = None,
        status_frames: list[dict[str, Any]] | None = None,
        error: str | None = None,
    ) -> None:
        self.channel = channel
        self.bitrate = bitrate
        self.duration_s = duration_s
        self.max_frames = max_frames
        self.mode = "read_only_status_inspect"
        self.protocol_route = "python-can slcan passive frame inspection"
        self.transmits_can_frames = False
        self.motor_enable_allowed = False
        self.motion_command_allowed = False
        self.status = status
        self.non_error_frames = non_error_frames
        self.error_frames_filtered = error_frames_filtered
        self.detected_ids = detected_ids or []
        self.status_frames = status_frames or []
        self.error = error
        self.config_path: str | None = None
        self.gate_state = "not_evaluated"
        self.expected_status_ids: list[int] = []
        self.unexpected_ids: list[int] = []
        self.missing_expected_ids: list[int] = []
        self.block_reasons: list[str] = []

    def to_dict(self) -> dict[str, Any]:
        return {
            "bitrate": self.bitrate,
            "channel": self.channel,
            "detected_ids": self.detected_ids,
            "duration_s": self.duration_s,
            "error": self.error,
            "error_frames_filtered": self.error_frames_filtered,
            "max_frames": self.max_frames,
            "mode": self.mode,
            "config_path": self.config_path,
            "gate_state": self.gate_state,
            "expected_status_ids": self.expected_status_ids,
            "unexpected_ids": self.unexpected_ids,
            "missing_expected_ids": self.missing_expected_ids,
            "block_reasons": self.block_reasons,
            "motion_command_allowed": self.motion_command_allowed,
            "motor_enable_allowed": self.motor_enable_allowed,
            "non_error_frames": self.non_error_frames,
            "protocol_route": self.protocol_route,
            "status": self.status,
            "status_frames": self.status_frames,
            "transmits_can_frames": self.transmits_can_frames,
        }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawTextHelpFormatter,
        description=(
            "read-only DM4340P/CANable status-frame inspector for HITL Gate 2.\n"
            "Safety: passive receive only; no torque enable, no motor enable, no motion command, "
            "and no CAN frame transmission."
        ),
    )
    parser.add_argument("--channel", default="/dev/ttyACM0", help="CANable slcan device path")
    parser.add_argument("--bitrate", type=int, default=1_000_000, help="CAN bitrate for the DM bus")
    parser.add_argument("--duration", type=float, default=3.0, help="read-only listen duration in seconds")
    parser.add_argument("--max-frames", type=int, default=20, help="Maximum non-error status frames to record")
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON only")
    parser.add_argument("--config", help="Optional two-motor read-only HITL config JSON for expected ID comparison")
    parser.add_argument(
        "--skip-open",
        action="store_true",
        help="Do not open hardware; print the safety/result schema for tests and docs",
    )
    return parser


def _frame_to_record(msg: Any) -> dict[str, Any]:
    arbitration_id = int(getattr(msg, "arbitration_id"))
    data = bytes(getattr(msg, "data", b""))
    return {
        "arbitration_id": arbitration_id,
        "arbitration_id_hex": f"0x{arbitration_id:x}",
        "dlc": len(data),
        "data_hex": data.hex(),
    }


def collect_status_frames(bus: Any, *, max_frames: int, duration_s: float) -> ReadOnlyInspectResult:
    result = ReadOnlyInspectResult(
        channel="<injected-bus>",
        bitrate=0,
        duration_s=duration_s,
        max_frames=max_frames,
        status="listening",
    )
    seen_ids: set[int] = set()
    deadline = time.time() + max(0.0, duration_s)
    max_records = max(0, max_frames)

    while time.time() < deadline and result.non_error_frames < max_records:
        msg = bus.recv(timeout=min(0.2, max(0.0, deadline - time.time())))
        if msg is None:
            continue
        if getattr(msg, "is_error_frame", False):
            result.error_frames_filtered += 1
            continue
        record = _frame_to_record(msg)
        result.non_error_frames += 1
        result.status_frames.append(record)
        seen_ids.add(record["arbitration_id"])

    result.detected_ids = sorted(seen_ids)
    result.status = "ok"
    return result


def inspect_canable(channel: str, bitrate: int, duration_s: float, max_frames: int) -> ReadOnlyInspectResult:
    try:
        import can
    except Exception as exc:  # pragma: no cover - depends on host env
        return ReadOnlyInspectResult(
            channel=channel,
            bitrate=bitrate,
            duration_s=duration_s,
            max_frames=max_frames,
            status="missing_python_can",
            error=f"{type(exc).__name__}: {exc}",
        )

    try:
        bus = can.Bus(
            interface="slcan",
            channel=channel,
            bitrate=bitrate,
            receive_own_messages=False,
        )
    except Exception as exc:  # pragma: no cover - depends on hardware env
        return ReadOnlyInspectResult(
            channel=channel,
            bitrate=bitrate,
            duration_s=duration_s,
            max_frames=max_frames,
            status="open_failed",
            error=f"{type(exc).__name__}: {exc}",
        )

    try:
        result = collect_status_frames(bus, max_frames=max_frames, duration_s=duration_s)
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass

    result.channel = channel
    result.bitrate = bitrate
    return result


def apply_config_comparison(result: ReadOnlyInspectResult, config_path: str | Path) -> None:
    backend = _load_backend_module()
    config = backend.load_read_only_config(config_path)
    comparison = backend.compare_observed_ids(config, result.detected_ids)
    result.config_path = str(config_path)
    result.gate_state = comparison.gate_state
    result.expected_status_ids = comparison.expected_status_ids
    result.unexpected_ids = comparison.unexpected_ids
    result.missing_expected_ids = comparison.missing_expected_ids
    result.block_reasons = comparison.block_reasons
    if comparison.unexpected_ids:
        result.status = "unexpected_ids"


def emit(result: ReadOnlyInspectResult, json_only: bool) -> None:
    payload = result.to_dict()
    if json_only:
        print(json.dumps(payload, sort_keys=True))
        return
    print(json.dumps(payload, indent=2, sort_keys=True))
    print("Safety: read-only receive; no CAN transmit, no motor enable, no torque enable, no motion command.")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.skip_open:
        result = ReadOnlyInspectResult(
            channel=args.channel,
            bitrate=args.bitrate,
            duration_s=args.duration,
            max_frames=args.max_frames,
            status="skipped_open",
        )
        if args.config:
            apply_config_comparison(result, args.config)
        emit(result, args.json)
        return 0

    result = inspect_canable(args.channel, args.bitrate, args.duration, args.max_frames)
    if args.config:
        apply_config_comparison(result, args.config)
    emit(result, args.json)
    return 0 if result.status in {"ok", "skipped_open"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
