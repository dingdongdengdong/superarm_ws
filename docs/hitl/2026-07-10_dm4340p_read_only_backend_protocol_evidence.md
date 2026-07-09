# DM4340P Read-Only Backend and Protocol Evidence - 2026-07-10

## Purpose

This note captures the next no-motion HITL slice after passive CANable readiness. It does **not** authorize torque enable, disable/enable proof, tiny motion, or LeLab-to-real-motor control.

## External protocol references checked

- DaMiao protocol reference: https://damiao-motor.jia-xie.com/concept/communication-protocol/
- Seeed Damiao 43-series getting-started reference: https://wiki.seeedstudio.com/damiao_series/

## Local protocol evidence

Repo-local planning docs describe a Damiao/OpenArm-style mapping with a default `master_id_offset=16`, so placeholder send/status IDs are represented as `1 -> 17` and `2 -> 18` in the read-only config. Those IDs are **not confirmed hardware facts** yet.

Known from this branch:

- CANable2 path: `/dev/ttyACM0`.
- Bitrate used for no-motion checks: `1000000`.
- Current read-only observation sees no unsolicited non-error frames, so it does not prove motor IDs.
- This checkout does not yet contain a verified real DM4340P LeLab hardware backend.

Still unknown:

- Actual DM4340P send IDs.
- Actual status/receive IDs.
- Whether both motors are in MIT/CAN mode.
- Whether a vendor-documented status query frame is safe to transmit before disable/enable proof.
- Zero offsets, signs, limits, and physical joint mapping.

## Config and safety backend

User-provided joint names: first motor = `j1`, second motor = `j2`. This names the two test joints only; it does not confirm CAN IDs, mode, sign, zero, or safe motion limits.


Config file:

```text
configs/hitl/dm4340p_x2_read_only.json
```

Safety backend module:

```text
tools/hitl/read_only_backend.py
```

Safety states are limited to:

- `disconnected`
- `read_only`
- `blocked`
- `fault`

The backend intentionally blocks all transmit-like methods: `send_frame`, `enable_motor`, `disable_motor`, and `command_motion`.

## Read-only config dry run

Command:

```bash
python3 tools/hitl/read_only_dm4340p_inspect.py --skip-open --config configs/hitl/dm4340p_x2_read_only.json --json
```

Result:

```json
{"bitrate": 1000000, "block_reasons": ["unconfirmed motor IDs: dm4340p_a_placeholder, dm4340p_b_placeholder"], "channel": "/dev/ttyACM0", "config_path": "configs/hitl/dm4340p_x2_read_only.json", "detected_ids": [], "duration_s": 3.0, "error": null, "error_frames_filtered": 0, "expected_status_ids": [], "gate_state": "blocked", "max_frames": 20, "missing_expected_ids": [], "mode": "read_only_status_inspect", "motion_command_allowed": false, "motor_enable_allowed": false, "non_error_frames": 0, "protocol_route": "python-can slcan passive frame inspection", "status": "skipped_open", "status_frames": [], "transmits_can_frames": false, "unexpected_ids": []}
```

## Read-only hardware config check

Command:

```bash
python3 tools/hitl/read_only_dm4340p_inspect.py --channel /dev/ttyACM0 --bitrate 1000000 --duration 3 --max-frames 20 --config configs/hitl/dm4340p_x2_read_only.json --json
```

Result:

```json
{"bitrate": 1000000, "block_reasons": ["unconfirmed motor IDs: dm4340p_a_placeholder, dm4340p_b_placeholder"], "channel": "/dev/ttyACM0", "config_path": "configs/hitl/dm4340p_x2_read_only.json", "detected_ids": [], "duration_s": 3.0, "error": null, "error_frames_filtered": 0, "expected_status_ids": [], "gate_state": "blocked", "max_frames": 20, "missing_expected_ids": [], "mode": "read_only_status_inspect", "motion_command_allowed": false, "motor_enable_allowed": false, "non_error_frames": 0, "protocol_route": "python-can slcan passive frame inspection", "status": "ok", "status_frames": [], "transmits_can_frames": false, "unexpected_ids": []}
```

## Gate decision

- CANable read-only access is still usable.
- Gate 2 is still **blocked** because expected IDs are placeholders and no status frames were observed.
- Any future status-query frame is **Gate 2B** and needs its own protocol citation and safety plan before transmitting anything.
- Continue to block torque enable, disable/enable proof, tiny motion, and LeLab hardware control.
