# DM4340P HITL Milestones C-F Dry-Run Status - 2026-07-10

## Safety boundary

This slice implements planning and dry-run evidence through Milestone F. It does **not** transmit CAN frames and does **not** run real disable, enable, tiny motion, or two-motor motion.

Why: Gate 2 is still blocked. The two motor IDs/modes are placeholders and no unsolicited status frames have been observed.

## Protocol frame evidence used

Protocol source links:

- DaMiao protocol reference: https://damiao-motor.jia-xie.com/concept/communication-protocol/
- Seeed Damiao 43-series getting-started reference: https://wiki.seeedstudio.com/damiao_series/


The dry-run harness encodes documented DM motor system frames as standard 8-byte CAN payloads. The same protocol reference also documents MIT command payload layout, continuous feedback frames, status codes, and example motor/feedback ID pairs (`0x01 -> 0x11`, `0x02 -> 0x12`, `0x03 -> 0x13`).

- Enable: `ff ff ff ff ff ff ff fc`
- Disable: `ff ff ff ff ff ff ff fd`
- Set zero: `ff ff ff ff ff ff ff fe` (not used by the runner)
- Clear error: `ff ff ff ff ff ff ff fb` (not used by the runner)

The runner exposes these as candidate frames only. `transmitted=false` in every output.

## Tooling added

```text
tools/hitl/dm4340p_protocol.py
tools/hitl/dm4340p_gate_runner.py
```

The gate runner commands are dry-run only:

```bash
python3 tools/hitl/dm4340p_gate_runner.py disable-proof --config configs/hitl/dm4340p_x2_read_only.json --motor-label dm4340p_a_placeholder --json
python3 tools/hitl/dm4340p_gate_runner.py enable-disable-proof --config configs/hitl/dm4340p_x2_read_only.json --motor-label dm4340p_a_placeholder --json
python3 tools/hitl/dm4340p_gate_runner.py tiny-motion-plan --config configs/hitl/dm4340p_x2_read_only.json --motor-label dm4340p_a_placeholder --relative-step-rad 0.005 --json
python3 tools/hitl/dm4340p_gate_runner.py two-motor-parity-plan --config configs/hitl/dm4340p_x2_read_only.json --json
```

## Milestone C - disable-command proof

Dry-run output summary:

```json
{"milestone": "C", "execute_allowed": false, "transmitted": false, "frames": [{"arbitration_id": 1, "data_hex": "fffffffffffffffd", "kind": "system_disable"}], "block_reasons": ["unconfirmed motor IDs: dm4340p_a_placeholder, dm4340p_b_placeholder", "motor dm4340p_a_placeholder is not confirmed", "hardware transmission is disabled in this implementation slice"]}
```

Status: **blocked**. Candidate disable bytes are known, but real disable proof is not run.

## Milestone D - enable then immediate disable

Dry-run output summary:

```json
{"milestone": "D", "execute_allowed": false, "transmitted": false, "frames": [{"arbitration_id": 1, "data_hex": "fffffffffffffffc", "kind": "system_enable"}, {"arbitration_id": 1, "data_hex": "fffffffffffffffd", "kind": "system_disable"}], "block_reasons": ["unconfirmed motor IDs: dm4340p_a_placeholder, dm4340p_b_placeholder", "motor dm4340p_a_placeholder is not confirmed", "Gate C disable proof is not passed on real hardware"]}
```

Status: **blocked**. Enable is not allowed until real Gate C disable proof passes.

## Milestone E - tiny single-motor motion

Dry-run output summary:

```json
{"milestone": "E", "execute_allowed": false, "transmitted": false, "relative_step_rad": 0.005, "frames": [], "block_reasons": ["unconfirmed motor IDs: dm4340p_a_placeholder, dm4340p_b_placeholder", "motor dm4340p_a_placeholder is not confirmed", "Gate D enable-disable proof is not passed", "sign/zero/clamps are not confirmed", "motion frame packing is intentionally withheld until protocol and motor limits are confirmed"]}
```

Status: **blocked**. The harness enforces `abs(relative_step_rad) <= 0.01`, but does not pack or transmit a motion frame until signs, zeros, limits, and Gate D evidence exist.

## Milestone F - two-motor hardware parity

Dry-run output summary:

```json
{"milestone": "F", "execute_allowed": false, "transmitted": false, "frames": [], "block_reasons": ["unconfirmed motor IDs: dm4340p_a_placeholder, dm4340p_b_placeholder", "two verified single-motor tiny-motion results are required before two-motor parity", "sign, zero, limit, and emergency-disable evidence must exist for both motors"]}
```

Status: **blocked**. Two-motor parity requires both motors to pass independent identity, disable, enable-disable, and tiny single-motor checks first.

## Current conclusion

Software preparation is now in place through Milestone F, but real hardware progression remains blocked at Gate 2. The next real-world unblocker is confirmed motor identity/mode evidence from labels/vendor tool or a separately approved Gate 2B status-query plan.

## Real Gate C attempt - j1 disable probe after operator confirmation

Operator-confirmed inputs received after the dry-run milestone report:

```text
j1 send id=1
j1 status id=17
mode=MIT/CAN
bitrate=1000000
arm unloaded=yes
cutoff ready=yes
move j1 only=yes
```

Safety-limited action taken: one real **disable** system frame was sent to `j1` only. No enable frame, no MIT motion frame, no LeLab command, and no torque/motion command was sent.

Command shape:

```bash
# python-can slcan, /dev/ttyACM0, 1 Mbps
# arbitration_id=1, data=ff ff ff ff ff ff ff fd
```

Observed JSON:

```json
{"bitrate": 1000000, "channel": "/dev/ttyACM0", "command": "disable", "data_hex": "fffffffffffffffd", "error": null, "error_frames_filtered": 0, "expected_status_id": 17, "gate": "C_real_disable_probe_j1", "non_error_frames": 0, "send_id": 1, "status": "ok", "status_frames": [], "transmitted": true}
```

Follow-up read-only listen:

```json
{"bitrate": 1000000, "block_reasons": ["unconfirmed motor IDs: j2_dm4340p_pending_confirmation", "missing expected status IDs: [17]"], "channel": "/dev/ttyACM0", "config_path": "configs/hitl/dm4340p_x2_read_only.json", "detected_ids": [], "duration_s": 2.0, "error": null, "error_frames_filtered": 0, "expected_status_ids": [17], "gate_state": "blocked", "max_frames": 20, "missing_expected_ids": [17], "mode": "read_only_status_inspect", "motion_command_allowed": false, "motor_enable_allowed": false, "non_error_frames": 0, "protocol_route": "python-can slcan passive frame inspection", "status": "ok", "status_frames": [], "transmits_can_frames": false, "unexpected_ids": []}
```

Updated Gate C status: **transmit path proven for one disable frame, but status proof failed**. Because no non-error frame and no expected status ID `17` were observed, Gate C is not considered passed for enabling or motion.

Next allowed action remains diagnostic-only unless a separate risky-action gate is opened: verify power/mode/wiring/vendor-tool status response, or define an enable-immediate-disable proof with explicit torque risk acceptance and stop criteria.

## Correction - j1/j2 were example names, not confirmed joint names

Operator clarified that `j1`/`j2` were examples. The config has been reverted to unconfirmed labels:

- first motor candidate: send ID `1`, candidate status ID `17`, `confirmed=false`
- second motor candidate: send ID `2`, candidate status ID `18`, `confirmed=false`

Additional no-motion diagnostics after this correction:

```json
{"bitrate": 1000000, "channel": "/dev/ttyACM0", "duration_s": 3.0, "error": null, "error_frames_filtered": 0, "first_frames": [], "mode": "passive_listen", "motor_enable_allowed": false, "non_error_frames": 0, "status": "ok", "transmits_can_frames": false}
```

```json
{"bitrate": 1000000, "block_reasons": ["unconfirmed motor IDs: first_motor_operator_example_name_unconfirmed, second_motor_operator_example_name_unconfirmed"], "channel": "/dev/ttyACM0", "config_path": "configs/hitl/dm4340p_x2_read_only.json", "detected_ids": [], "duration_s": 3.0, "error": null, "error_frames_filtered": 0, "expected_status_ids": [], "gate_state": "blocked", "max_frames": 20, "missing_expected_ids": [], "mode": "read_only_status_inspect", "motion_command_allowed": false, "motor_enable_allowed": false, "non_error_frames": 0, "protocol_route": "python-can slcan passive frame inspection", "status": "ok", "status_frames": [], "transmits_can_frames": false, "unexpected_ids": []}
```

Gate decision: still **no motion**. Physical space may be safe, but the software gate has not identified a responding motor or a feedback/disable path.

## Single-motor HITL update - first motor moved and disabled

Operator clarified the physical setup: only one motor is connected, and it is the first motor. A disable-only ID discovery over candidate send IDs `1..16` found a reply only after probing send ID `1`:

```json
{"gate": "single_motor_disable_only_id_discovery", "candidate_send_ids": [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16], "detected_ids": [0], "frames_by_probe": {"1": [{"arbitration_id": 0, "arbitration_id_hex": "0x0", "data_hex": "019b817ff7f81c1a", "dlc": 8}]}, "status": "ok", "error_frames_filtered": 0}
```

Repeat ID-1 disable proof:

```json
{"gate": "repeat_single_motor_id1_disable_proof", "send_id": 1, "transmitted": true, "frames": [{"arbitration_id": 0, "arbitration_id_hex": "0x0", "data_hex": "019b817ff7ff1c1a", "dlc": 8, "embedded_motor_id_first_byte": 1}], "status": "ok", "error_frames_filtered": 0}
```

Enable-immediate-disable proof:

```json
{"gate": "single_motor_enable_immediate_disable_id1", "send_id": 1, "transmitted": ["enable", "disable"], "frames": [{"phase": "after_enable", "arbitration_id": 0, "data_hex": "119b817ff8001c1a", "embedded_motor_id_first_byte": 17}, {"phase": "after_disable", "arbitration_id": 0, "data_hex": "019b817ff8071c1a", "embedded_motor_id_first_byte": 1}], "status": "ok", "error_frames_filtered": 0}
```

Tiny single-motor MIT motion was then run on send ID `1` only, followed by disable. Parameters were deliberately conservative: requested relative target `+0.08 rad`, `kp=8.0`, `kd=0.4`, zero velocity target, zero feed-forward torque. The motor moved partially before disable:

```json
{"gate": "single_motor_tiny_mit_motion_id1", "send_id": 1, "requested_step_rad": 0.08, "initial_position_rad": 2.6861600671396957, "target_position_rad": 2.7661600671396958, "final_position_rad": 2.728503852903028, "motion_frames_sent": 52, "system_frames_sent": ["disable_pre", "enable", "disable_post"], "status": "ok", "error_frames_filtered": 0}
```

Status: Milestone E passed for **single connected first motor only**. Milestone F remains blocked until a second motor is connected and independently passes the same gates.
