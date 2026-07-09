# DM4340P HITL Milestones G-H Wrap-up - 2026-07-10

## Safety boundary

This wrap-up prepares Milestones G-H as blocked dry-run plans. It does **not** connect LeLab to real DM4340P motors, does **not** run hardware teleop, and does **not** run dataset/policy actuation.

The hardware chain remains blocked earlier at Gate 2/F because motor IDs/mode are unconfirmed, no status frames were observed, and no real disable/enable/tiny-motion proofs exist.

## Milestone G - LeLab integration through safe backend

Command:

```bash
python3 tools/hitl/dm4340p_gate_runner.py lelab-integration-plan --config configs/hitl/dm4340p_x2_read_only.json --json
```

Result summary:

```json
{"milestone": "G", "execute_allowed": false, "transmitted": false, "frames": [], "block_reasons": ["unconfirmed motor IDs: dm4340p_a_placeholder, dm4340p_b_placeholder", "Milestone F two-motor parity is not passed", "LeLab UI must remain read-only/disabled until expected IDs, disable path, clamps, and emergency stop are verified", "existing IsaacSimRpoArmRobot SITL path must not be reused as a real CAN backend"]}
```

Required before G can execute:

- Separate hardware-safe `Robot` backend exists after F passes.
- `robot.connect()` verifies expected IDs and disable path before UI enables controls.
- UI starts read-only/disabled.
- Requested target, clamped target, raw response, and disable result are logged.
- Start with no-op and tiny relative actions only after all prior gates pass.

## Milestone H - controlled data/policy readiness

Command:

```bash
python3 tools/hitl/dm4340p_gate_runner.py policy-readiness-plan --config configs/hitl/dm4340p_x2_read_only.json --json
```

Result summary:

```json
{"milestone": "H", "execute_allowed": false, "transmitted": false, "frames": [], "block_reasons": ["unconfirmed motor IDs: dm4340p_a_placeholder, dm4340p_b_placeholder", "Milestone G LeLab safe-backend integration is not passed", "calibration startup and recovery procedure are missing", "dataset/policy safety envelope is not validated on hardware", "dry-run policy replay must precede any real actuation"]}
```

Required before H can execute:

- Repeatable calibration startup exists.
- Teleop speed/torque/current limits are documented and enforced.
- Recovery/disable procedure is practiced.
- Dataset logging captures clamps and raw responses.
- Policy replay dry-run passes before any real actuation.

## A-H status ledger

| Milestone | Status | Reason |
| --- | --- | --- |
| A | Prepared/blocked | Read-only path exists but no IDs/status frames observed. |
| B | Prepared/blocked | No-motion safety backend exists; transmit/motion methods blocked. |
| C | Dry-run only | Candidate disable frame exists; no real transmit. |
| D | Dry-run only | Candidate enable-disable order exists; blocked until real C. |
| E | Dry-run only | Tiny step guard exists; no motion frame packing/transmit. |
| F | Dry-run only | Two-motor parity blocked until both single-motor checks pass. |
| G | Dry-run only | LeLab must remain read-only/disabled until F and safe backend pass. |
| H | Dry-run only | Dataset/policy readiness blocked until G, calibration, limits, and recovery exist. |

## Final wrap-up conclusion

The repository is now prepared with dry-run plans through Milestone H, but real hardware progression remains blocked. The next real-world unblocker is still confirmed DM4340P identity/mode/status evidence, then a separately approved Gate C real disable-proof run.

## Update after operator-confirmed j1 details

The config now records operator-confirmed `j1` as send ID `1`, expected status ID `17`, MIT/CAN mode, with the arm unloaded and cutoff ready. A single real j1 disable frame (`fffffffffffffffd`) was transmitted on `/dev/ttyACM0` at 1 Mbps.

Result: transmit succeeded, but zero non-error frames were observed and expected status ID `17` did not appear. Therefore Milestones D-H remain blocked; LeLab real-motor movement is still not allowed by the current gate evidence.

## Update - cloned LeLab and moved first motor through LeLab API

The ignored `worktrees/` directory was recreated locally and LeLab was cloned from the recorded upstream:

```text
repo: https://github.com/huggingface/leLab.git
path: worktrees/leLab
cloned main commit: def3e9e
```

Existing project LeLab patches `0001` through `0005` applied cleanly. A narrow `dm4340p_single` backend was then added inside the ignored LeLab clone. It uses the same LeLab API shape as the Isaac Sim backend:

- `POST /move-arm` with `robot_backend="dm4340p_single"`
- `POST /send-joint-action` with a one-element relative step vector
- `GET /joint-positions`
- `POST /stop-teleoperation`

The full `lelab.server` route was not used because current upstream LeLab declares Python `>=3.12` and imports a larger dependency stack; this host is Python `3.10.12`. The patched lightweight `lelab.superarm_server` route was used instead.

Validation in `worktrees/leLab`:

```text
python3 -m pytest tests/test_teleoperate.py tests/test_superarm_server.py -q
# 11 passed, 1 warning
```

Real LeLab API movement evidence, first motor only on `/dev/ttyACM0`:

```json
{"connect": {"success": true, "robot_backend": "dm4340p_single", "joint_positions": {"first_motor": 2.6777676050965127}}, "send_action": {"success": true, "sent_action": [2.7685587406158447], "joint_positions": {"first_motor": 2.768558785381858}}, "stop": {"success": true, "message": "Teleoperation stopped successfully"}}
```

Status: LeLab API movement is now proven for the single connected first motor only. Two-motor Milestone F, full LeLab manual web app operation, dataset recording, and policy replay remain future gates.

## Update - parent uv venv and real edited LeLab server

Per operator request, the uv environment was moved to the workspace root:

```text
/home/dong/echo/superarm_ws/.venv
```

The venv is local/ignored via `/.venv/`. It was created with Python 3.10 plus system site packages so it can reuse the existing NVIDIA Torch install instead of downloading duplicate large Torch/CUDA wheels. Runtime dependencies for the edited LeLab server were installed into this parent venv with `uv pip install`.

The real edited LeLab server was then run from the cloned worktree, not the lightweight helper:

```bash
cd worktrees/leLab
PYTHONPATH=/home/dong/echo/superarm_ws/worktrees/leLab:/home/dong/echo/superarm_ws/isaacsim_test/lerobot:/home/dong/ai/lerobot/src \
  /home/dong/echo/superarm_ws/.venv/bin/python -m uvicorn lelab.server:app --host 127.0.0.1 --port 8000
```

Because full LeLab imports calibration/record/train modules at startup, `lelab.server` was edited to treat those LeRobot-heavy features as optional/unavailable when their dependencies are absent. The teleoperation route remains the real `lelab.server:app` route and uses `lelab.teleoperate` plus the `dm4340p_single` backend.

Real LeLab endpoint proof:

```json
{"health": {"status": "ok", "message": "FastAPI server is running"}, "connect": {"success": true, "robot_backend": "dm4340p_single", "joint_positions": {"first_motor": 2.768558785381858}}, "send_action": {"success": true, "sent_action": [2.860494375228882], "joint_positions": {"first_motor": 2.8604943923094535}}, "stop": {"success": true, "message": "Teleoperation stopped successfully"}}
```

Status: real edited `lelab.server` moved the single first motor through `/move-arm` + `/send-joint-action`; final stop succeeded.
