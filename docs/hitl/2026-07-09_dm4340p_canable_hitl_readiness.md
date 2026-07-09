# DM4340P x2 CANable HITL Readiness - 2026-07-09

## Verdict

**Do not drive the two DM4340P motors from LeLab yet.**

The USB CANable is visible and the Python CAN stack can open it passively, but this branch has only verified **no-motion HITL readiness**. The current LeLab work in this repo is a SITL/Isaac Sim control path; it does not yet contain a real DM4340P hardware adapter with ID discovery, sign/zero calibration, clamps, torque disable, and emergency-stop behavior.

Safe next target: **HITL Gate 1-2 only**: interface setup, passive listen, and read-only/inspection evidence. Tiny motion is a later gate and must be single-motor, unloaded, clamped, and supervised.

## Branch and scope

- Branch: `feature/hitl-dm4340p-canable-readiness`
- Date/time checked: 2026-07-09 KST
- Hardware stated by operator: two `DM4340P` motors connected through USB CANable.
- Local USB adapter found: `Openlight Labs CANable2` at `/dev/ttyACM0`.
- Local onboard CAN found: `can0`, but it is Jetson `mttcan`, state `DOWN`; **do not confuse this with the USB CANable**.
- Assumed CAN bitrate for this check: `1000000`.

## Evidence from this branch

### Repository/wiki evidence

- `omx_wiki` has LeLab/Isaac Sim/SITL evidence, but no dedicated HITL page existed before this branch.
- Existing SITL docs freeze the six-field action/state contract:
  1. `j1.pos`
  2. `j2.pos`
  3. `right_arm_yaw_joint.pos`
  4. `right_elbow_pitch_joint.pos`
  5. `right_elbow_yaw_joint.pos`
  6. `amazinghand_grasp.pos`
- Existing C07/C08 docs say real hardware mapping, signs, and clamps are pending and must not be treated as confirmed by SITL.

### Host evidence

```text
/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_2070388B3136-if00 -> ../../ttyACM0
/dev/ttyACM0 exists, group dialout
can0 exists but is Jetson mttcan and is DOWN
candump exists
slcand exists
python modules: can OK, serial OK, numpy OK
```

### Passive listen evidence

Command:

```bash
python3 tools/hitl/passive_canable_check.py --channel /dev/ttyACM0 --bitrate 1000000 --duration 3 --json
```

Result:

```json
{"bitrate": 1000000, "channel": "/dev/ttyACM0", "duration_s": 3.0, "error": null, "error_frames_filtered": 0, "first_frames": [], "mode": "passive_listen", "motor_enable_allowed": false, "non_error_frames": 0, "status": "ok", "transmits_can_frames": false}
```

Interpretation:

- The CANable slcan channel opened successfully.
- The check transmitted **zero** CAN frames.
- No non-error frames were seen in 3 seconds. That is not a motor-response proof; it only means no unsolicited traffic was observed.


## 2026-07-10 KST no-motion rerun and read-only inspection

### Local protocol/tooling route chosen

Repository inspection found Damiao/OpenArm support described in planning docs such as `integration_guide/03_motor_can_mapping.md` and `integration_guide/04_lerobot_custom_robot_skeleton.md`, but this checkout still has no verified real DM4340P hardware backend for LeLab. For this gate, the selected route is therefore:

```text
python-can -> slcan -> /dev/ttyACM0 CANable2 -> passive/read-only frame observation
```

This route does **not** send a DM command, torque-enable frame, disable/enable frame, or motion target. It can only record unsolicited non-error CAN frames if any are already visible on the bus.

### Passive CANable rerun

Command:

```bash
python3 tools/hitl/passive_canable_check.py --channel /dev/ttyACM0 --bitrate 1000000 --duration 3 --json
```

Result:

```json
{"bitrate": 1000000, "channel": "/dev/ttyACM0", "duration_s": 3.0, "error": null, "error_frames_filtered": 0, "first_frames": [], "mode": "passive_listen", "motor_enable_allowed": false, "non_error_frames": 0, "status": "ok", "transmits_can_frames": false}
```

### Read-only Gate 2 inspection

Command:

```bash
python3 tools/hitl/read_only_dm4340p_inspect.py --channel /dev/ttyACM0 --bitrate 1000000 --duration 3 --max-frames 20 --json
```

Result:

```json
{"bitrate": 1000000, "channel": "/dev/ttyACM0", "detected_ids": [], "duration_s": 3.0, "error": null, "error_frames_filtered": 0, "max_frames": 20, "mode": "read_only_status_inspect", "motion_command_allowed": false, "motor_enable_allowed": false, "non_error_frames": 0, "protocol_route": "python-can slcan passive frame inspection", "status": "ok", "status_frames": [], "transmits_can_frames": false}
```

Interpretation:

- `/dev/ttyACM0` CANable opened successfully at `1000000` bitrate.
- Both tools transmitted **zero** CAN frames.
- SocketCAN/CAN controller error frames would be filtered; none were observed in this run.
- No non-error CAN frames were observed, so no DM4340P IDs/status frames were detected.

Go/no-go:

- **GO** for continuing only no-motion documentation/tooling and physical safety checklist work.
- **NO-GO** for torque enable, disable/enable tests, tiny motion, or LeLab-to-real-motor control because Gate 2 has not proved responding motor IDs/status yet.


## 2026-07-10 KST read-only backend/config slice

Added a no-motion read-only safety backend and a two-motor placeholder config:

```text
configs/hitl/dm4340p_x2_read_only.json
tools/hitl/read_only_backend.py
```

The config uses placeholder Damiao-style IDs `1 -> 17` and `2 -> 18`, but both motors are marked `confirmed=false`. Therefore, config comparison can help detect unexpected observed IDs, but it cannot complete Gate 2 until the real motor IDs/mode are confirmed.

Safe hardware command:

```bash
python3 tools/hitl/read_only_dm4340p_inspect.py --channel /dev/ttyACM0 --bitrate 1000000 --duration 3 --max-frames 20 --config configs/hitl/dm4340p_x2_read_only.json --json
```

Result:

```json
{"bitrate": 1000000, "block_reasons": ["unconfirmed motor IDs: dm4340p_a_placeholder, dm4340p_b_placeholder"], "channel": "/dev/ttyACM0", "config_path": "configs/hitl/dm4340p_x2_read_only.json", "detected_ids": [], "duration_s": 3.0, "error": null, "error_frames_filtered": 0, "expected_status_ids": [], "gate_state": "blocked", "max_frames": 20, "missing_expected_ids": [], "mode": "read_only_status_inspect", "motion_command_allowed": false, "motor_enable_allowed": false, "non_error_frames": 0, "protocol_route": "python-can slcan passive frame inspection", "status": "ok", "status_frames": [], "transmits_can_frames": false, "unexpected_ids": []}
```

Interpretation: the backend/config path is in place, but Gate 2 remains **blocked** because the motor IDs are unconfirmed placeholders and no status frames were observed. Any future transmitted status query is a separate Gate 2B plan.


## 2026-07-10 KST Milestones C-F dry-run preparation

Added dry-run tooling for Milestones C-F:

```text
tools/hitl/dm4340p_protocol.py
tools/hitl/dm4340p_gate_runner.py
docs/hitl/2026-07-10_dm4340p_milestones_c_to_f_dry_run.md
```

This tooling prepares candidate gate plans only. It does **not** transmit CAN frames. Current dry-run status:

- Milestone C disable proof: candidate `system_disable` frame `fffffffffffffffd`, `execute_allowed=false`, `transmitted=false`.
- Milestone D enable-immediate-disable: candidate `system_enable` then `system_disable`, `execute_allowed=false`, `transmitted=false`.
- Milestone E tiny single-motor motion: max relative step guarded at `0.01 rad`, but no motion frame is packed or transmitted because signs/zeros/clamps and Gate D are missing.
- Milestone F two-motor parity: blocked until both motors pass independent ID, disable, enable-disable, and tiny-motion sign checks.

Conclusion: software prep exists through F, but hardware remains blocked at Gate 2.

## HITL gate sequence

### Gate 0 - physical safety setup

Do this before any command that could enable torque:

- [ ] Motor shafts/links are unloaded or mechanically constrained so unexpected motion cannot hit people, cables, or the table.
- [ ] External power supply voltage/current limit is known and conservative.
- [ ] A physical power cutoff is reachable by the operator.
- [ ] CAN H/L/GND wiring and termination are checked.
- [ ] Motor IDs are written down from the vendor tool or labels if available.
- [ ] Only the two intended DM4340P motors are on this test bus.

### Gate 1 - USB CANable and software readiness (completed for this branch)

- [x] Stable CANable serial path exists: `/dev/ttyACM0` via `/dev/serial/by-id/...CANable2...`.
- [x] `can0` was identified as onboard Jetson CAN, not the USB CANable.
- [x] `can-utils` tools exist.
- [x] `python-can` and `pyserial` import successfully.
- [x] Passive slcan open/listen works without transmitting frames.

### Gate 2 - read-only motor identity/position inspection

Goal: prove which motor IDs are present before motion.

- [x] Choose the exact low-level protocol path for this gate: `python-can`/`slcan` passive read-only frame observation, because no verified real DM4340P hardware backend exists in this checkout yet.
- [ ] Confirm whether the two motors are configured for MIT/CAN mode and the expected bitrate. Current config is placeholder-only and `confirmed=false`.
- [x] Run read-only status-frame observation only; no torque enable, disable/enable frame, or motion target was sent.
- [x] Filter `is_error_frame == True`; do not count SocketCAN/CAN controller errors as motor feedback.
- [ ] Record each responding CAN ID/status frame. Current result: no non-error frames, `detected_ids=[]`, and `gate_state=blocked`.
- [ ] Stop if only error frames appear, no IDs respond, or bus errors increase.

### Gate 3 - disable/enable safety proof

Goal: prove we can immediately leave a safe state.

- [x] Implement dry-run candidate command for DM4340P disable (`fffffffffffffffd`); real transmit remains blocked.
- [ ] Verify disable can be sent independently per motor on real hardware. Current state: dry-run only, `transmitted=false`.
- [ ] Only after real disable proof, test enable then immediate disable on **one motor**. Current state: dry-run candidate frames only.
- [ ] No position/velocity target yet.

### Gate 4 - first tiny single-motor motion

Goal: confirm sign and scaling without useful task motion.

- [ ] One motor only; second motor disabled.
- [x] Add dry-run tiny-motion planner enforcing relative command <= `0.01 rad`; real motion frame packing/transmit blocked.
- [ ] Confirm real p/v/t/kp/kd clamps before packing or transmitting any motion frame.
- [ ] Observe physical direction and encoder direction after Gate D passes; not run in this slice.
- [ ] Write sign/zero result to the hardware parity table after tiny single-motor checks; not available yet.
- [ ] Disable after every test.

### Gate 5 - two-motor HITL through LeLab

Only after Gates 2-4 pass:

- [x] Add gated dry-run DM4340P protocol/planning helper separate from SITL `IsaacSimRpoArmRobot`; real hardware backend still blocked until gates pass.
- [ ] Expose only the two verified motors, not the full six-field arm contract, unless all missing joints are mocked/no-op and clearly labeled.
- [x] Add blocked dry-run LeLab integration plan: UI must remain read-only/disabled until `robot.connect()` verifies bus, IDs, safe clamps, and disable path.
- [ ] Start with no-op and tiny relative actions only after G executes; current state is dry-run blocked.
- [ ] Log requested target, clamped target, raw motor response, and disable result after real Gate C-E execution; currently dry-run JSON only.


## 2026-07-10 KST Milestones G-H wrap-up

Added blocked dry-run plans for:

- Milestone G: LeLab safe-backend integration plan.
- Milestone H: controlled data/policy readiness plan.

Both emit `execute_allowed=false`, `transmitted=false`, and no frames. The docs are in:

```text
docs/hitl/2026-07-10_dm4340p_milestones_g_to_h_wrap_up.md
```

Conclusion: roadmap preparation exists through H, but real hardware progression remains blocked at Gate 2/F.

## Current LeLab/HITL architecture decision

Use LeLab as the operator UI **after** a hardware-safe `Robot` backend exists. Do not point the existing LeLab SITL route directly at the real CAN bus.

Reason:

```text
LeLab UI/API
  -> LeRobot Robot.send_action()
  -> must go through a hardware backend with safety clamps
  -> only then CANable / DM4340P
```

The current SITL backend is:

```text
LeLab
  -> IsaacSimRpoArmRobot
  -> ROS2 /follower/joint_commands
  -> Isaac Sim
```

That path validates the six-field contract and web control surface, but it is not a real DM4340P safety backend.

## Immediate commands for this branch

No-motion checks:

```bash
python3 tools/hitl/passive_canable_check.py --help
python3 tools/hitl/passive_canable_check.py --channel /dev/ttyACM0 --bitrate 1000000 --duration 3 --json
python3 -m unittest discover -s tests -p 'test_hitl_passive_canable_check.py' -v
```

Useful host inspection:

```bash
ip -details link show type can || true
ls -l /dev/serial/by-id /dev/ttyACM* 2>/dev/null || true
python3 - <<'PY'
import importlib.util
for name in ['can', 'serial', 'numpy']:
    print(name, 'OK' if importlib.util.find_spec(name) else 'MISSING')
PY
```

## Stop conditions

Stop immediately if any of these happen:

- Unexpected physical movement.
- Motor/link warms quickly or power supply current spikes.
- CAN bus shows repeated error frames or bus-off state.
- Motor ID is not the one being tested.
- Operator cannot reach the power cutoff.
- LeLab/UI command path bypasses the clamp/disable layer.
