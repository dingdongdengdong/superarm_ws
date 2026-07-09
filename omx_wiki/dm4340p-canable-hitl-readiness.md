---
title: "DM4340P CANable HITL Readiness"
tags: ["HITL", "DM4340P", "CANable", "LeLab", "safety"]
created: 2026-07-09T15:01:24.835Z
updated: 2026-07-09T15:01:24.835Z
sources: []
links: []
category: decision
confidence: medium
schemaVersion: 1
---

# DM4340P CANable HITL Readiness

# DM4340P CANable HITL Readiness

Updated: 2026-07-09

Branch: `feature/hitl-dm4340p-canable-readiness`.

Verdict: do not drive the two DM4340P motors from LeLab yet. The USB CANable is visible and passive listen works, but only no-motion HITL readiness has been verified. A real DM4340P hardware backend with ID discovery, sign/zero calibration, clamps, disable/enable proof, and emergency-stop behavior is still required before LeLab can control real motors.

Evidence:

- CANable2 detected at `/dev/ttyACM0` via `/dev/serial/by-id/...CANable2...`.
- Onboard Jetson `can0` exists but is `mttcan`, state DOWN, and must not be confused with the USB CANable.
- `candump`, `slcand`, `python-can`, `pyserial`, and `numpy` are available.
- Passive check command: `python3 tools/hitl/passive_canable_check.py --channel /dev/ttyACM0 --bitrate 1000000 --duration 3 --json`.
- Passive result: `status=ok`, `non_error_frames=0`, `error_frames_filtered=0`, `transmits_can_frames=false`, `motor_enable_allowed=false`.

Next gates:

1. Physical safety setup: unloaded/constrained motors, reachable power cutoff, verified wiring/termination, IDs noted.
2. Read-only motor identity/position inspection; filter `is_error_frame == True`; no torque enable.
3. Disable/enable safety proof on one motor, then immediate disable.
4. Tiny single-motor relative movement only, starting around 0.005-0.01 rad with conservative clamp.
5. Only after gates pass, connect LeLab through a hardware-safe Robot backend; do not reuse the existing SITL IsaacSimRpoArmRobot as a real CAN backend.

Full branch document: `docs/hitl/2026-07-09_dm4340p_canable_hitl_readiness.md`.



## 2026-07-10 KST update

Added `tools/hitl/read_only_dm4340p_inspect.py` as the Gate 2 no-motion inspector. It opens CANable with `python-can`/`slcan`, filters `is_error_frame == True`, records non-error arbitration IDs/status bytes if visible, and declares `transmits_can_frames=false`, `motor_enable_allowed=false`, and `motion_command_allowed=false` in JSON output.

Sequential no-motion commands run:

```bash
python3 tools/hitl/passive_canable_check.py --channel /dev/ttyACM0 --bitrate 1000000 --duration 3 --json
python3 tools/hitl/read_only_dm4340p_inspect.py --channel /dev/ttyACM0 --bitrate 1000000 --duration 3 --max-frames 20 --json
```

Results:

- Passive check: `status=ok`, `non_error_frames=0`, `error_frames_filtered=0`, `transmits_can_frames=false`, `motor_enable_allowed=false`.
- Read-only inspector: `status=ok`, `detected_ids=[]`, `status_frames=[]`, `non_error_frames=0`, `error_frames_filtered=0`, `transmits_can_frames=false`, `motor_enable_allowed=false`, `motion_command_allowed=false`.

Gate decision: CANable path is still usable for no-motion inspection, but Gate 2 has **not** proved motor IDs/status. Continue no-motion/physical-safety work only. Do not proceed to torque enable, disable/enable proof, tiny motion, or LeLab real-motor control without a separate safety gate.


## 2026-07-10 read-only backend/config slice

Added a no-motion Gate 2 backend/config boundary:

- Config: `configs/hitl/dm4340p_x2_read_only.json`.
- Backend: `tools/hitl/read_only_backend.py`.
- Protocol note: `docs/hitl/2026-07-10_dm4340p_read_only_backend_protocol_evidence.md`.

The backend has only `disconnected`, `read_only`, `blocked`, and `fault` states and blocks `send_frame`, `enable_motor`, `disable_motor`, and `command_motion`. The config contains placeholder Damiao-style IDs (`1 -> 17`, `2 -> 18`) marked `confirmed=false`.

Latest safe config check:

```bash
python3 tools/hitl/read_only_dm4340p_inspect.py --channel /dev/ttyACM0 --bitrate 1000000 --duration 3 --max-frames 20 --config configs/hitl/dm4340p_x2_read_only.json --json
```

Result summary: `status=ok`, `gate_state=blocked`, `detected_ids=[]`, `expected_status_ids=[]`, `block_reasons=["unconfirmed motor IDs: dm4340p_a_placeholder, dm4340p_b_placeholder"]`, `transmits_can_frames=false`, `motor_enable_allowed=false`, `motion_command_allowed=false`.

Gate decision remains NO-GO for torque enable, disable/enable proof, tiny motion, and LeLab real-motor control. Any transmitted status-query frame is a separate Gate 2B plan.


## 2026-07-10 Milestones C-F dry-run preparation

Added dry-run-only tooling through Milestone F:

- `tools/hitl/dm4340p_protocol.py`
- `tools/hitl/dm4340p_gate_runner.py`
- `docs/hitl/2026-07-10_dm4340p_milestones_c_to_f_dry_run.md`

Current dry-run results:

- C disable proof: candidate disable frame `fffffffffffffffd`, but `execute_allowed=false`, `transmitted=false`.
- D enable-immediate-disable: candidate enable frame `fffffffffffffffc` followed by disable `fffffffffffffffd`, but blocked because real Gate C is not passed.
- E tiny single-motor motion: relative step guard accepts `0.005` and rejects values over `0.01`, but no motion frame is packed/transmitted until Gate D, sign/zero, and clamp evidence exist.
- F two-motor parity: blocked until both motors independently pass identity, disable, enable-disable, and tiny-motion sign checks.

Gate decision remains blocked at Gate 2 because motor IDs/mode are unconfirmed and no status frames were observed. No real torque, enable, disable transmit, tiny motion, or LeLab hardware control was run.


## 2026-07-10 Milestones G-H wrap-up

Added blocked dry-run plan outputs through the end of the roadmap:

- G LeLab integration: `python3 tools/hitl/dm4340p_gate_runner.py lelab-integration-plan --config configs/hitl/dm4340p_x2_read_only.json --json` returns `execute_allowed=false`, `transmitted=false`, `frames=[]`. It blocks because F is not passed and LeLab must stay read-only/disabled until expected IDs, disable path, clamps, and emergency stop are verified.
- H policy readiness: `python3 tools/hitl/dm4340p_gate_runner.py policy-readiness-plan --config configs/hitl/dm4340p_x2_read_only.json --json` returns `execute_allowed=false`, `transmitted=false`, `frames=[]`. It blocks because G is not passed and calibration/recovery/safety-envelope evidence is missing.

Wrap-up doc: `docs/hitl/2026-07-10_dm4340p_milestones_g_to_h_wrap_up.md`.

Final status: A-H are represented as tooling/docs/dry-run plans, but real hardware work remains blocked at Gate 2/F. No LeLab real-motor control, dataset collection, policy replay, torque enable, disable transmit, or motion was run.

## 2026-07-10 real j1 disable probe

Operator confirmed j1 send ID `1`, expected status ID `17`, MIT/CAN mode, 1 Mbps, unloaded arm, cutoff ready, and j1-only movement intent.

Action taken: transmitted exactly one j1 disable frame on CANable `/dev/ttyACM0` via slcan:

```json
{"gate":"C_real_disable_probe_j1","send_id":1,"expected_status_id":17,"data_hex":"fffffffffffffffd","transmitted":true,"status":"ok","non_error_frames":0,"error_frames_filtered":0,"status_frames":[]}
```

Follow-up passive/read-only inspection still saw no status ID `17` and no non-error frames. Therefore the CANable transmit path is proven for a disable frame, but j1 status proof is still missing. Do not enable torque or send motion/LeLab commands until the status-response gap is resolved or a separately documented risky gate is accepted.

## 2026-07-10 correction: j1/j2 were examples

Operator clarified that `j1`/`j2` were only example names. The config now marks both motors unconfirmed again. Candidate IDs remain `1 -> 17` and `2 -> 18`, but neither is treated as proven.

Fresh no-motion checks on `/dev/ttyACM0` at 1 Mbps still saw `non_error_frames=0` and `detected_ids=[]`. Therefore larger movement is not allowed by the current HITL gate.

## 2026-07-10 single first-motor motion proof

Operator clarified only one motor was connected and it is the first motor. Disable-only ID discovery found first motor on send ID `1`, receiving feedback at arbitration ID `0x0` with embedded motor ID/status in byte 0.

Progression completed on first motor only:

1. disable-only ID discovery over IDs `1..16`: reply only after send ID `1`.
2. repeat ID-1 disable proof: feedback received, no error frames.
3. enable-immediate-disable: feedback received after enable and after disable.
4. tiny MIT motion on ID `1`, then disable.

Motion evidence: initial position `2.6861600671 rad`, target `2.7661600671 rad`, final observed position `2.7285038529 rad`; `52` MIT command frames sent; final system command was disable. This proves only single-motor Milestone E. Milestone F and LeLab hardware control remain blocked until a second motor is connected/verified and a guarded backend uses this feedback model.

## 2026-07-10 LeLab API single-motor movement proof

`worktrees/leLab` was recloned from `https://github.com/huggingface/leLab.git` and existing project LeLab patches `0001`-`0005` were applied. A narrow `dm4340p_single` backend was added in the ignored LeLab clone and validated with `11 passed` LeLab tests.

Runtime used the patched lightweight LeLab server, not the full upstream server, because this host is Python 3.10 while current LeLab declares Python >=3.12 and imports a larger dependency stack.

LeLab endpoint flow completed on the single connected first motor:

- `POST /move-arm` with `robot_backend=dm4340p_single` connected at position `2.6777676051 rad`.
- `POST /send-joint-action` with action `[0.12]` moved/read back `2.7685587854 rad`.
- `GET /joint-positions` returned the same final position.
- `POST /stop-teleoperation` succeeded and backend disabled/disconnected.

This proves first-motor movement through a LeLab API path. It does not yet prove full web UI/manual leader, two-motor parity, recording, or policy execution.

## 2026-07-10 parent uv venv + real edited LeLab server

The uv environment was moved to the workspace root at `.venv/` and ignored in git. It uses Python 3.10 with system site packages so it can reuse the existing NVIDIA Torch install. The edited real LeLab server was run as `lelab.server:app` from `worktrees/leLab`, not the lightweight `lelab.superarm_server` helper.

Full LeLab imports for calibration/record/train are now guarded as optional in this local edited server so DM4340P teleoperation can run without importing unused heavy features. The route used was still the real LeLab server route:

- `GET /health` -> ok
- `POST /move-arm` with `robot_backend=dm4340p_single` -> connected at `2.7685587854 rad`
- `POST /send-joint-action` with `[0.12]` -> moved/read back `2.8604943923 rad`
- `GET /joint-positions` -> same final readback
- `POST /stop-teleoperation` -> success

This proves movement through the real edited LeLab server path for the single connected first motor.
