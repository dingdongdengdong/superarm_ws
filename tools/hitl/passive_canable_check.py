#!/usr/bin/env python3
"""Passive CANable/DM bus readiness check for HITL bring-up.

Safety contract: this tool opens a CANable slcan channel and listens only. It
never calls send(), never enables motors, and never writes DM4340P commands.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass


@dataclass
class PassiveListenResult:
    channel: str
    bitrate: int
    duration_s: float
    mode: str = "passive_listen"
    transmits_can_frames: bool = False
    motor_enable_allowed: bool = False
    status: str = "not_started"
    non_error_frames: int = 0
    error_frames_filtered: int = 0
    first_frames: list[str] | None = None
    error: str | None = None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawTextHelpFormatter,
        description=(
            "Passive CANable slcan listener for HITL readiness; "
            "no CAN frames are transmitted and no motor enable/torque command is allowed."
        )
    )
    parser.add_argument("--channel", default="/dev/ttyACM0", help="CANable slcan device path")
    parser.add_argument("--bitrate", type=int, default=1_000_000, help="CAN bitrate for DM bus")
    parser.add_argument("--duration", type=float, default=3.0, help="Passive listen duration in seconds")
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON only")
    parser.add_argument(
        "--skip-open",
        action="store_true",
        help="Do not open hardware; print the safety/result schema for tests and docs",
    )
    return parser


def passive_listen(channel: str, bitrate: int, duration_s: float) -> PassiveListenResult:
    result = PassiveListenResult(
        channel=channel,
        bitrate=bitrate,
        duration_s=duration_s,
        first_frames=[],
    )
    try:
        import can
    except Exception as exc:  # pragma: no cover - depends on host env
        result.status = "missing_python_can"
        result.error = f"{type(exc).__name__}: {exc}"
        return result

    try:
        bus = can.Bus(
            interface="slcan",
            channel=channel,
            bitrate=bitrate,
            receive_own_messages=False,
        )
    except Exception as exc:  # pragma: no cover - depends on hardware env
        result.status = "open_failed"
        result.error = f"{type(exc).__name__}: {exc}"
        return result

    result.status = "listening"
    deadline = time.time() + max(0.0, duration_s)
    try:
        while time.time() < deadline:
            msg = bus.recv(timeout=min(0.2, max(0.0, deadline - time.time())))
            if msg is None:
                continue
            if getattr(msg, "is_error_frame", False):
                result.error_frames_filtered += 1
                continue
            result.non_error_frames += 1
            if result.first_frames is not None and len(result.first_frames) < 5:
                result.first_frames.append(str(msg))
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass

    result.status = "ok"
    return result


def emit(result: PassiveListenResult, json_only: bool) -> None:
    payload = asdict(result)
    if json_only:
        print(json.dumps(payload, sort_keys=True))
        return
    print(json.dumps(payload, indent=2, sort_keys=True))
    print("Safety: passive listen only; no CAN transmit, no motor enable, no torque command.")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.skip_open:
        emit(
            PassiveListenResult(
                channel=args.channel,
                bitrate=args.bitrate,
                duration_s=args.duration,
                status="skipped_open",
                first_frames=[],
            ),
            args.json,
        )
        return 0

    result = passive_listen(args.channel, args.bitrate, args.duration)
    emit(result, args.json)
    return 0 if result.status == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
