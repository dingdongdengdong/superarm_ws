# LeLab → Isaac Sim SimReady Right-Arm Integration Notes

Branch: `feature/lelab-isaacsim-arm-control`

Local upstream clone:

```text
/workspaces/superarm_ws/worktrees/leLab
```

LeLab clone state captured during setup:

```text
repo: https://github.com/huggingface/leLab.git
upstream base commit: 8f8a50f Force-release camera/serial resources when a device disconnect fails
local integration commits:
  - f453845 Add Isaac Sim RPO arm teleoperation backend
  - b330278 Add SuperArm lightweight Isaac Sim control server
  - ae43d96 Switch SuperArm server to six-field control contract
local branch: feature/superarm-isaacsim-control
patch files:
  - isaacsim_test/lelab_patches/0001-Add-Isaac-Sim-RPO-arm-teleoperation-backend.patch
  - isaacsim_test/lelab_patches/0002-Add-SuperArm-lightweight-Isaac-Sim-control-server.patch
  - isaacsim_test/lelab_patches/0003-Switch-SuperArm-server-to-six-field-control-contract.patch
```

## Current test target answer

For this project, the default LeLab/Isaac Sim test target is **not** the old
`source_arm_isaacsim_arm_only.yaml` path. The current project-compatible target
matches `isaacsim_test/README.md`:

- **Primary LeLab target:** SimReady `echo_full` / RoboParty V2 right arm with
  AmazingHand grasp scalar.
- **Primary config:** `isaacsim_test/lerobot/rpo_arm_isaacsim.yaml`.
- **Control vector:** five right-arm joints plus `amazinghand_grasp` = six fields.
- **Current limitation:** LeLab/ROS command echo and Isaac Sim 5.1 rendering are
  validated; SimReady physical articulation is still `binding_pending`.

If we are intentionally testing **standalone arm + fixed hand**, use the
arm-only branch/config:

- **Arm-only fixed-hand config:**
  `isaacsim_test/lerobot/rpo_arm_isaacsim_arm_only.yaml`.
- **Runner:** `isaacsim_test/run_fixed_hand_arm_lerobot_capture.sh`.
- **Control vector:** five right-arm joints only; `fixed_hand: true`,
  `fixed_grasp: 0.0`, no `amazinghand_grasp` action exposed.

The `source_arm_isaacsim_arm_only.yaml` / `joint_rev_*` path is retained only as
a separate source-package experiment and should not be treated as the default
LeLab target.

## USD vs YAML control schema

The LeRobot YAML is only the **control interface schema**. It tells LeLab /
`IsaacSimRpoArmRobot` which feature names and ROS topics to use. It does not
load geometry into Isaac Sim.

The Isaac Sim scene asset is the SimReady USD:

```text
/workspaces/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd
```

Inside the Docker Isaac Sim container the same file is mounted at:

```text
/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd
```

`isaacsim_test/isaacsim/setup_rpo_arm_scene.py` loads that file from
`SIMREADY_USD_PATH` and references it into the stage at:

```text
/World/echo_full_simready
```

Current data flow:

```text
LeLab UI / API
  -> isaacsim_test/lerobot/rpo_arm_isaacsim.yaml  (feature/topic schema)
  -> IsaacSimRpoArmRobot
  -> ROS2 /follower/joint_commands
  -> setup_rpo_arm_scene.py
  -> SimReady USD referenced at /World/echo_full_simready
```

Important: the SimReady USD currently has `binding_status="binding_pending"`.
That means the USD is the right visual/scene file, but the five arm feature names
plus `amazinghand_grasp` are not yet bound to concrete controllable USD
articulation prims. Until that binding is authored, the bridge can validate
LeLab/ROS command transport and publish mirrored `/follower/joint_states`, but it
does not prove physical USD joint motion.

For real joint motion today, use the legacy URDF fallback path; for the desired
project asset, use the SimReady USD above and treat articulation binding as the
next implementation step.

## Current compatibility verdict

LeLab can be used as a browser UI layer for our custom Isaac Sim arm, but the
upstream app is not plug-and-play for this robot yet.

Observed upstream assumptions:

- `lelab/teleoperate.py` imports `SO101Follower`, `SO101FollowerConfig`,
  `SO101Leader`, and `SO101LeaderConfig` directly.
- `lelab/record.py` also constructs SO-101 follower/leader configs directly.
- `lelab/calibrate.py` has SO-101 config construction paths for leader/follower.
- `CLAUDE.md` documents that LeLab is currently hardcoded around
  `so101_leader` / `so101_follower`.

Our working backend already exists in this repo:

- `isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py`
- `isaacsim_test/lerobot/rpo_arm_isaacsim.yaml` for the six-field right-arm +
  AmazingHand contract
- `isaacsim_test/lerobot/rpo_arm_isaacsim_arm_only.yaml` for the standalone
  arm-only/fixed-hand branch
- `isaacsim_test/run_fixed_hand_arm_lerobot_capture.sh` for timestamped
  fixed-hand arm-only validation artifacts

The current project-compatible Isaac Sim / LeRobot joint contract is:

```text
right_arm_pitch_joint, right_arm_roll_joint, right_arm_yaw_joint,
right_elbow_pitch_joint, right_elbow_yaw_joint, amazinghand_grasp
```

The arm-only fixed-hand branch uses the same first five right-arm joints and
omits `amazinghand_grasp`.

## Integration plan

1. Keep our `IsaacSimRpoArmRobot` as the robot backend source of truth.
2. Keep the LeLab backend selector instead of hardcoding SO-101 everywhere.
3. Default LeLab work should target the project-compatible SimReady/RoboParty
   right-arm contract:
   - LeLab UI slider/vector action → `IsaacSimRpoArmRobot.send_action()`
   - Isaac/ROS readback → LeLab joint display/websocket broadcast
   - optional screenshot/debug endpoint → existing Isaac camera capture artifacts
4. Use the arm-only fixed-hand config only when the explicit goal is to test
   standalone arm motion while holding AmazingHand fixed.
5. After direct joint control works, adapt record/replay around the same backend.

## Known-good validation before LeLab work

Current project-compatible validation evidence:

```text
isaacsim_test/artifacts/lelab_isaacsim51_control_20260706T053212Z
isaacsim_test/artifacts/lelab_isaacsim51_control_panel_20260706T051927Z/screenshots/live_isaacsim51_clean_echo_full_view/echo_full_5_1_clean_view.png
```

Result:

```text
LeLab SuperArm six-field contract tests passed.
Isaac Sim 5.1 render evidence for SimReady echo_full passed vision review.
SimReady physical articulation remains binding_pending.
```

Legacy source-arm evidence is still available for the separate source-package
experiment:

```text
/workspaces/superarm_ws/isaacsim_test/artifacts/source_arm_lerobot_pose_cases_20260705T104845Z
```

## Useful commands

Start from this repo branch:

```bash
git checkout feature/lelab-isaacsim-arm-control
```

Inspect the cloned LeLab workspace:

```bash
cd /workspaces/superarm_ws/worktrees/leLab
git status -sb
```

Run the current Isaac Sim 5.1 LeLab control-contract verification:

```bash
cd /workspaces/superarm_ws
python3 isaacsim_test/run_lelab_isaacsim51_control_verification.py
```

Run the standalone arm + fixed-hand branch only when that is the explicit target:

```bash
cd /workspaces/superarm_ws
./isaacsim_test/run_fixed_hand_arm_lerobot_capture.sh
```

Run the older source-package experiment only when debugging `joint_rev_*` source
URDF behavior:

```bash
cd /workspaces/superarm_ws
./isaacsim_test/run_source_arm_lerobot_pose_capture.sh
```

## Implemented MVP patch

The local LeLab clone now has commits `f453845`, `b330278`, `ae43d96`,
`b940e3a`, and `361f171` with:

- `robot_backend="isaacsim_rpo_arm"` support in `lelab/teleoperate.py`.
- dynamic import of this repo's `IsaacSimRpoArmRobot` from `SUPERARM_WS_PATH` or `/workspaces/superarm_ws`.
- `/send-joint-action` API in `lelab/server.py` for sending one vector/list action to the active backend.
- generic right-arm feature/vector readback conversion for websocket/joint-position display.
- targeted LeLab test coverage in `tests/test_teleoperate.py`.
- full LeLab `lelab.server` Manual Web Leader route (`/manual-leader`) as the supported browser control surface. The old standalone `lelab.superarm_server` helper has been removed.
- six-field default UI/config for `right_arm_pitch_joint`, `right_arm_roll_joint`, `right_arm_yaw_joint`, `right_elbow_pitch_joint`, `right_elbow_yaw_joint`, and `amazinghand_grasp`.
- default Isaac Sim backend config pinned to this repo's RoboParty V2 right-arm
  LeRobot config, `isaacsim_test/lerobot/rpo_arm_isaacsim.yaml`; it no longer
  falls back to `source_arm_isaacsim_arm_only.yaml`.

Validation run inside `worktrees/leLab`:

```text
python3 -m py_compile worktrees/leLab/lelab/teleoperate.py worktrees/leLab/lelab/server.py worktrees/leLab/tests/test_teleoperate.py
cd worktrees/leLab && python3 -m pytest tests/test_teleoperate.py -q
# 10 passed, 2 warnings
```

API sketch for Isaac Sim mode:

```json
POST /move-arm
{
  "robot_backend": "isaacsim_rpo_arm",
  "leader_port": "unused",
  "follower_port": "unused",
  "leader_config": "unused",
  "follower_config": "isaacsim_test/lerobot/rpo_arm_isaacsim.yaml",
  "superarm_ws_path": "/workspaces/superarm_ws"
}

POST /send-joint-action
{
  "action": [0.25, -0.2, 0.3, -0.35, 0.1, 1.0]
}
```

### 2026-07-06 correction: study LeLab history before choosing the target

Commit-history check found why the control target was easy to mix up:

- Main repo commit `6e9537c` (`fix: use RoboParty V2 right arm URDF for Isaac
  Sim`) established the intended runtime target as the RoboParty V2 right arm at
  `/workspace/superarm_ws/roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf`.
- Main repo commit `a8a4bbf` later added the local source-arm path for a separate
  `joint_rev_1..4` source-package experiment.
- LeLab commit `f453845` accidentally kept that source-arm YAML as the Isaac Sim
  backend fallback; `ae43d96` fixed the SuperArm server UI to six fields but did
  not fix the backend fallback.
- LeLab commit `361f171` now fixes the backend default and tests so LeLab uses
  `rpo_arm_isaacsim.yaml` and right-arm feature names by default.

`/workspaces/superarm_ws/arm_with_hand_with_robot_file` currently contains
source CAD/STL (`echo_full.step`, `echo_full.stl`) used by the SimReady visual
asset pipeline. It is not the default LeLab control config. Use it as visual/source
asset context only unless the run is explicitly the older source-arm experiment.

If the RoboParty URDF is missing locally, initialize the submodule before running
the right-arm checks:

```bash
git submodule update --init roboparty
```

## Full LeLab Manual Web Leader runbook

The supported runtime path is the full LeLab server plus the Manual Web Leader page.
The former standalone helper module `lelab.superarm_server` has been deleted.

```bash
cd /workspaces/superarm_ws/worktrees/leLab
set +u
source /opt/ros/humble/setup.bash
set -u
export SUPERARM_WS_PATH=/workspaces/superarm_ws
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export PYTHONPATH=/workspaces/superarm_ws/worktrees/leLab:/workspaces/superarm_ws/isaacsim_test/lerobot:${PYTHONPATH:-}
python3 -m uvicorn lelab.server:app --host 0.0.0.0 --port 8000
```

Open the full app or jump directly to Manual Web Leader:

```text
http://127.0.0.1:8000/
http://127.0.0.1:8000/manual-leader?robot=SuperArm%20Source%20Arm
```

Validated on 2026-07-05 in:

```text
/workspaces/superarm_ws/isaacsim_test/artifacts/lelab_superarm_control_20260705T111521Z
```

Evidence:

- Isaac Sim bridge loaded `/workspaces/superarm_ws/isaacsim_test/outputs/robot_arm_hand_from_zip_local_drive/robot_arm_hand_sanitized.urdf`.
- `/move-arm` connected `robot_backend=isaacsim_rpo_arm`.
- `/send-joint-action` accepted and Isaac applied/read back these arm-only poses:
  - `[0.25, -0.2, 0.3, -0.35]`
  - `[-0.25, 0.2, -0.3, 0.35]`
  - `[0.4, 0.1, 0.15, -0.45]`
  - `[0.0, 0.0, 0.0, 0.0]`
- Per-command JSON evidence was written under `data/isaac_command_evidence/`.

## 2026-07-05 six-field control rerun

Runtime folder:

```text
/workspaces/superarm_ws/isaacsim_test/artifacts/lelab_superarm_6field_control_20260705T130140Z
```

What changed from the previous 4-joint fallback run:

- Isaac was restarted with the SimReady USD:
  `/workspaces/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd`
- `JOINT_NAMES` was left unset so `setup_rpo_arm_scene.py` used its default RoboParty V2 contract.
- LeLab server UI now defaults to `rpo_arm_isaacsim.yaml` and exposes 6 sliders:
  - `right_arm_pitch_joint`
  - `right_arm_roll_joint`
  - `right_arm_yaw_joint`
  - `right_elbow_pitch_joint`
  - `right_elbow_yaw_joint`
  - `amazinghand_grasp`

Validation evidence:

- `/move-arm` connected with `robot_backend=isaacsim_rpo_arm` and `rpo_arm_isaacsim.yaml`.
- `/send-joint-action` accepted/read back:
  - `[0.2, 0.1, -0.2, 0.3, -0.15, 0.0]`
  - `[-0.2, -0.1, 0.2, -0.3, 0.15, 0.5]`
  - `[0.35, -0.25, 0.15, 0.25, -0.25, 1.0]`
  - `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`
- Isaac command evidence confirms `using_simready=true`, `published_joint_names` has all 6 fields, and `binding_status="binding_pending"`.

Important limitation:

The current SimReady USD still does not expose a bound articulation for these
six fields. Isaac logs: `SimReady articulation binding is binding_pending;
publishing mirrored 6D LeRobot state until prim mapping is authored.` This means
LeLab/ROS 6D command and state echo works, but visual/physical SimReady joint
motion still needs USD articulation prim binding.

## 2026-07-06 Isaac Sim 5.1 LeLab SuperArm server control contract branch

Branch: `feature/isaacsim-5.1-lelab-superarm-control`

Local Isaac Sim version check:

```text
/workspace/isaacsim/VERSION = 5.1.0-rc.19+release.26219.9c81211b.gl
```

The LeLab/SuperArm server control contract remains the same six-field vector used
by the earlier 6.0-oriented notes:

1. `right_arm_pitch_joint`
2. `right_arm_roll_joint`
3. `right_arm_yaw_joint`
4. `right_elbow_pitch_joint`
5. `right_elbow_yaw_joint`
6. `amazinghand_grasp`

New branch-local verification helper:

```bash
python3 isaacsim_test/run_lelab_isaacsim51_control_verification.py
```

It creates a timestamped artifact folder under
`isaacsim_test/artifacts/lelab_isaacsim51_control_<UTC>/` with:

- `logs/lelab_superarm_contract.log`
- `data/five_dof_grasp_cases.json`
- `lelab_superarm_control.html`
- `screenshots/lelab_superarm_control_verification.png`
- `report.json`

Latest LeLab server contract artifact folder:

```text
isaacsim_test/artifacts/lelab_isaacsim51_control_20260706T053212Z
```

Subagent-verified accepted live Isaac render evidence remains in the earlier visual artifact folder:

```text
isaacsim_test/artifacts/lelab_isaacsim51_control_panel_20260706T051927Z
```

Validation result:

- Isaac Sim compatibility: `5.1` detected and accepted.
- Control fields: 5 arm DOF + 1 normalized AmazingHand grasp scalar.
- Test cases: 6 one-axis cases (`pitch_positive`, `roll_negative`,
  `yaw_positive`, `elbow_pitch_positive`, `elbow_yaw_negative`, `grasp_close`).
- Mock LeRobot backend: all six cases can be sent via `IsaacSimRpoArmRobot.send_action()`
  without requiring ROS message imports; mock observation readback updates to the sent vector.
- PNG evidence: `screenshots/lelab_superarm_control_verification.png`.
- Accepted live Isaac Sim 5.1 render evidence: `screenshots/live_isaacsim51_clean_echo_full_view/echo_full_5_1_clean_view.png` in the visual artifact folder. The earlier `live_isaacsim51_echo_full_view` and `live_isaacsim51_simready_view` captures failed vision review and are retained only as rejected diagnostics.

Important limitation:

The current SimReady USD remains `binding_pending` for the six-field articulation
contract. The new LeLab SuperArm server artifacts verify LeLab/control contract and Isaac
Sim 5.1 rendering compatibility. They do not yet prove physical six-axis motion
of the SimReady arm until USD articulation prim binding is authored.

### Subagent PNG verification follow-up

The initial live Isaac Sim 5.1 PNGs in this run were sent to a vision subagent and
rejected as poorly framed/occluded. They remain in the artifact folder only as
failed evidence.

A fixed no-ground recapture was generated at:

```text
isaacsim_test/artifacts/lelab_isaacsim51_control_panel_20260706T051927Z/screenshots/live_isaacsim51_clean_echo_full_view/echo_full_5_1_clean_view.png
```

Vision subagent verdict: `PASS` as visual render evidence because it is non-empty
and shows identifiable SimReady/`echo_full` arm/hand/end-effector geometry under
Isaac Sim 5.1. Caveat: it is still cropped and proves rendering only, not physics
or six-axis articulation/control.

The LeLab SuperArm matrix PNG was separately verified by a vision subagent as
`PASS` for the six-control/six-case contract and `not physics evidence`.

### Ralph cleanup/audit follow-up

Cleanup found one real masking-fallback issue in the testability patch: non-mock
`IsaacSimRpoArmRobot.send_action()` could succeed locally when `_pub is None`.
This is now fixed to raise `RuntimeError` unless `mock=True`; mock mode remains
available for local contract tests without ROS message packages.

The artifact generator also now validates custom timestamp suffixes against
`YYYYMMDDTHHMMSSZ` so log/artifact folder names stay date-time based.

Fresh verification after cleanup:

```text
python3 -m py_compile isaacsim_test/lerobot/lelab_isaacsim51_control_contract.py isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py isaacsim_test/run_lelab_isaacsim51_control_verification.py isaacsim_test/test_lelab_isaacsim51_control_contract.py
python3 -m unittest isaacsim_test.test_lelab_isaacsim51_control_contract -v  # 8 tests OK
python3 -m unittest isaacsim_test.test_lerobot_rpo_arm_control -v             # 5 tests OK
python3 isaacsim_test/run_lelab_isaacsim51_control_verification.py --timestamp 20260706T053212Z
```

### LeLab server and Manual Web Leader test evidence

The user clarified this should remain the full LeLab Manual Web Leader, not a standalone control panel. The obsolete `lelab.superarm_server` module and its dedicated tests were removed. Current LeLab-side tests verify:

- `/manual-leader-config/SuperArm Source Arm` exposes `joint_rev_1..5`;
- screenshot debug endpoints reject idle sessions;
- active `isaacsim_rpo_arm` sessions publish screenshot-debug JSON to Isaac Sim;
- status/image routes expose the latest Isaac-saved screenshot.

Fresh LeLab validation:

```text
cd worktrees/leLab && python3 -m pytest tests/test_server.py tests/test_teleoperate.py -q
```

## 2026-07-09 SuperArm AmazingHand Manual Leader

LeLab now has a built-in `SuperArm AmazingHand` robot record for controlling the
Isaac-friendly generated AmazingHand URDF directly through the normal LeLab Manual
Leader path.

- LeLab worktree: `worktrees/leLab`
- Exported patch: `isaacsim_test/lelab_patches/0007-Add-SuperArm-AmazingHand-manual-leader.patch`
- Hand-only LeRobot config: `isaacsim_test/lerobot/amazinghand_isaacsim_hand_only.yaml`
- Generated URDF: `isaacsim_test/outputs/robot_arm_hand_from_zip_local_drive/amazinghand_graspable.urdf`
- Topics: `/hand/joint_commands`, `/hand/joint_states`, `/hand/screenshot_debug`

Verified presets:

```text
Open hand  = [0.05, 0.02] * 4
Half close = [0.50, 0.56] * 4
Close hand = [0.95, 1.10] * 4
```

Latest accepted control/readback artifact:

```text
isaacsim_test/artifacts/lelab_amazinghand_control_only_20260709T065136Z/
```

Verification commands run from the workspace:

```bash
worktrees/leLab/.venv/bin/python -m pytest \
  worktrees/leLab/tests/test_server.py \
  worktrees/leLab/tests/test_utils_config.py \
  worktrees/leLab/tests/test_teleoperate.py \
  worktrees/leLab/tests/test_superarm_amazinghand_manual_config.py -q

worktrees/leLab/.venv/bin/python -m unittest isaacsim_test.test_lerobot_rpo_arm_control -v
worktrees/leLab/.venv/bin/python -m pytest isaacsim_test/test_setup_amazinghand_scene.py -q
```

Results: 58 LeLab tests passed, 11 LeRobot shim tests passed, and 4 Isaac hand
scene static tests passed.

Caveat: this validates realtime command/readback for the generated Isaac-friendly
URDF hand. It is not SimReady binding proof, and visual close-up finger evidence
is still pending because the current headless screenshot path can return no RGBA
data.

### 2026-07-09 AmazingHand remapped-visual rerun

A later rerun found that the generated hand URDF's STL visual paths were host
absolute. `setup_amazinghand_scene.py` now remaps those mesh filenames to
`/workspace/superarm_ws` inside the Isaac container before import.

Fresh control/readback artifact after the remap fix:

```text
isaacsim_test/artifacts/lelab_amazinghand_control_only_20260709T073737Z/
```

The run passed open/half/close again and wrote a USD package containing
`payloads/geometries.usd` and `payloads/instances.usda`, so the visual payloads
are now packaged with the hand USD. Close-up PNG proof is still pending because
headless Replicator capture currently hangs and must not run inside the realtime
control loop.
