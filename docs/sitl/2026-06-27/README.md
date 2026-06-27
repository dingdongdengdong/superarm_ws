# SITL Phase 2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the validated SimReady `echo_full` USD plus the existing LeRobot/ROS2 bridge into a repeatable SITL pipeline for asset loading, control binding, perception, data recording, policy replay, and later hardware parity.

**Architecture:** Keep Isaac Sim as the simulated follower, load the SimReady USD as the scene asset, use LeRobot as the control/record/replay interface, and use ROS2 as the transport boundary. Every phase must produce machine-checkable artifacts under `isaacsim_test/artifacts/` and a short human-readable run log so regressions can be found quickly.

**Tech Stack:** Isaac Sim 5.1 container, validated SimReady USD from `echo_full.step`, ROS2 Humble, local LeRobot `isaacsim_rpo_arm` robot type, Docker Compose, Python verifier scripts, NVIDIA Agent Skills catalog for NVIDIA/Omniverse-specific work.

**Created:** 2026-06-27, Asia/Seoul

**Directory:** `docs/sitl/2026-06-27/`

**Baseline commit:** `04ef79a test: add LeRobot SITL verifier with screenshots`

**Current asset commit:** `68787fa Add SimReady conversion artifacts for echo_full CAD`

---

## Historical baseline verified on 2026-06-27

- [x] Isaac Sim image: `nvcr.io/nvidia/isaac-sim:5.1.0`.
- [x] Source robot: official RoboParty / Roboto Origin V2.0 URDF at `roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf`.
- [x] LeRobot config path: `isaacsim_test/lerobot/rpo_arm_isaacsim.yaml`.
- [x] Controlled feature order:
  1. `right_arm_pitch_joint`
  2. `right_arm_roll_joint`
  3. `right_arm_yaw_joint`
  4. `right_elbow_pitch_joint`
  5. `right_elbow_yaw_joint`
  6. `amazinghand_grasp`
- [x] LeRobot command path verified with `IsaacSimRpoArmRobot.send_action()`.
- [x] Observed state verified with `IsaacSimRpoArmRobot.capture_observation()`.
- [x] Verified target: `[0.2, 0.1, -0.2, 0.3, 0.1, 0.5]`.
- [x] Observed state matched target with max absolute error `0.0` and tolerance `0.03`.
- [x] Screenshot artifact captured from Isaac Sim: `isaacsim_test/artifacts/rpo_v2_lerobot_target.png`.
- [x] Evidence artifact written by verifier: `isaacsim_test/artifacts/lerobot_sitl_verify.json`.

Known limits of that historical baseline:

- The current arm motion is a command/state bridge test; it is not yet a physics-quality actuator validation.
- The historical bridge represented hand intent as one `amazinghand_grasp` value. The new SimReady asset contains visible hand geometry, but Isaac articulation/control binding is still next work.
- The screenshot proves Isaac Sim rendering after command, but the camera framing is not yet an arm-pose measurement tool.
- The verifier checks one pose; it does not sweep joint limits, trajectories, contact, collisions, or policy rollouts.

---

## Current source of truth

- [x] Source CAD: `arm_with_hand_with_robot_file/echo_full.step`.
- [x] Final SimReady USD: `isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd`.
- [x] SimReady validation report: `isaacsim_test/outputs/simready/echo_full/omniverse-cad-to-simready-report.md`.
- [x] Profile validation: `Prop-Robotics-Neutral` v`1.0.0` passed with no remaining requirement failures.
- [ ] Treat the RoboParty V2.0 URDF as a legacy joint-name/control reference, not the primary scene asset.
- [ ] Do not recreate temporary arm/hand geometry while the SimReady USD exists.
- [ ] Before any NVIDIA/Isaac/Omniverse-specific task, check the NVIDIA skills catalog at <https://github.com/NVIDIA/skills>.
- [ ] Treat NVIDIA skills as agent guidance, not runtime project dependencies, unless a specific skill becomes part of a reproducible command.
- [ ] Save all generated runtime evidence under `isaacsim_test/artifacts/` so it stays out of git.
- [ ] Commit each completed phase separately with its verifier and docs update.

NVIDIA skills checked on 2026-06-27:

```bash
# Browse available skills without changing this repository.
npx skills add nvidia/skills --list

# Candidate skills for future Isaac/Omniverse work, if available in the catalog:
npx skills add nvidia/skills --skill omniverse-usd-performance-tuning --agent codex --yes
npx skills add nvidia/skills --skill omniverse-realtime-viewer --agent codex --yes
npx skills add nvidia/skills --skill omniverse-cad-to-simready --agent codex --yes
```

Acceptance check for this rule:

```bash
grep -R "NVIDIA skills" -n docs/sitl/2026-06-27/README.md
```

Expected: at least one line points future Isaac/Omniverse work to the NVIDIA skills catalog.

---

## Task 1: Make SITL bringup and evidence collection one command

**Purpose:** Replace manual multi-terminal bringup with a deterministic command that starts Isaac Sim, runs the LeRobot verifier, captures screenshot/evidence, stores logs, and exits cleanly.

**Files:**
- Create: `isaacsim_test/run_lerobot_sitl_check.sh`
- Modify: `isaacsim_test/README.md`
- Modify: `isaacsim_test/test_v2_roboparty_config.py`

- [ ] **Step 1: Add a static test for the bringup script**

Add a test in `isaacsim_test/test_v2_roboparty_config.py` that reads `isaacsim_test/run_lerobot_sitl_check.sh` and asserts these command fragments exist:

```python
self.assertIn("SCREENSHOT_AFTER_COMMAND=1", script_text)
self.assertIn("docker compose up --force-recreate isaac-sim-51", script_text)
self.assertIn("verify_lerobot_sitl.py", script_text)
self.assertIn("lerobot_sitl_verify.json", script_text)
self.assertIn("rpo_v2_lerobot_target.png", script_text)
```

Run:

```bash
python3 isaacsim_test/test_v2_roboparty_config.py
```

Expected before implementation: the new test fails because the script does not exist.

- [ ] **Step 2: Create the bringup script**

Create `isaacsim_test/run_lerobot_sitl_check.sh` with this behavior:

```bash
#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
mkdir -p artifacts

export HEADLESS="${HEADLESS:-1}"
export SCREENSHOT_AFTER_COMMAND=1
export SCREENSHOT_PATH="${SCREENSHOT_PATH:-/workspace/superarm_ws/isaacsim_test/artifacts/rpo_v2_lerobot_target.png}"
export EXIT_AFTER_SCREENSHOT=1

rm -f artifacts/isaac-sim-51.log artifacts/lerobot-sitl.log artifacts/lerobot_sitl_verify.json artifacts/rpo_v2_lerobot_target.png

docker compose rm -sf isaac-sim-51 lerobot foxglove || true

docker compose up --force-recreate isaac-sim-51 > artifacts/isaac-sim-51.log 2>&1 &
isaac_pid=$!

cleanup() {
  docker compose rm -sf isaac-sim-51 lerobot foxglove >/dev/null 2>&1 || true
}
trap cleanup EXIT

# Give Isaac time to import the V2.0 URDF and create ROS topics.
sleep "${ISAAC_BOOT_WAIT_S:-60}"

docker compose run --rm --entrypoint /bin/bash lerobot -lc '
  set -euo pipefail
  source /opt/ros/humble/setup.bash
  cd /workspace/superarm_ws/lerobot
  python3 -m pip install -e . --quiet --break-system-packages --ignore-installed blinker
  cd /workspace/superarm_ws
  PYTHONPATH=/workspace/superarm_ws/isaacsim_test/lerobot:/workspace/superarm_ws/lerobot:$PYTHONPATH \
  python3 isaacsim_test/lerobot/verify_lerobot_sitl.py \
    --timeout-s 30 \
    --evidence /workspace/superarm_ws/isaacsim_test/artifacts/lerobot_sitl_verify.json
' > artifacts/lerobot-sitl.log 2>&1

wait "$isaac_pid"

test -s artifacts/lerobot_sitl_verify.json
test -s artifacts/rpo_v2_lerobot_target.png
```

- [ ] **Step 3: Run the script and collect artifacts**

Run:

```bash
cd isaacsim_test
bash run_lerobot_sitl_check.sh
```

Expected:

```text
artifacts/isaac-sim-51.log exists and contains Loading RoboParty V2.0 URDF
artifacts/lerobot-sitl.log exists and contains "passed": true
artifacts/lerobot_sitl_verify.json exists and contains "passed": true
artifacts/rpo_v2_lerobot_target.png exists and is non-empty
```

- [ ] **Step 4: Document the one-command gate**

Add this command to `isaacsim_test/README.md` under Verification gates:

```bash
cd isaacsim_test
bash run_lerobot_sitl_check.sh
```

- [ ] **Step 5: Commit Task 1**

```bash
git add isaacsim_test/run_lerobot_sitl_check.sh isaacsim_test/README.md isaacsim_test/test_v2_roboparty_config.py
git commit -m "test: add one-command LeRobot SITL check"
```

---

## Task 2: Replace single-pose verification with a joint sweep verifier

**Purpose:** Prove every LeRobot-controlled joint moves through safe commanded values and reports the expected state.

**Files:**
- Modify: `isaacsim_test/lerobot/verify_lerobot_sitl.py`
- Modify: `isaacsim_test/test_v2_roboparty_config.py`

- [ ] **Step 1: Add verifier arguments for a named sweep**

Extend `verify_lerobot_sitl.py` so it accepts:

```bash
--sweep basic_right_arm
--per-target-timeout-s 10
```

The `basic_right_arm` sweep must use these targets:

```python
SWEEPS = {
    "basic_right_arm": [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.2, 0.1, -0.2, 0.3, 0.1, 0.5],
        [-0.2, -0.1, 0.2, -0.3, -0.1, 1.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]
}
```

- [ ] **Step 2: Write sweep evidence JSON**

Evidence must include one result per target:

```json
{
  "passed": true,
  "sweep": "basic_right_arm",
  "results": [
    {
      "target_index": 0,
      "target": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      "observed": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      "absolute_error": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    }
  ]
}
```

- [ ] **Step 3: Add a static test for sweep support**

Add assertions in `isaacsim_test/test_v2_roboparty_config.py`:

```python
self.assertIn("basic_right_arm", verifier_text)
self.assertIn("--sweep", verifier_text)
self.assertIn("per-target-timeout-s", verifier_text)
self.assertIn("results", verifier_text)
```

Run:

```bash
python3 isaacsim_test/test_v2_roboparty_config.py
```

Expected: all tests pass after implementation.

- [ ] **Step 4: Run the sweep in SITL**

Run:

```bash
cd isaacsim_test
bash run_lerobot_sitl_check.sh VERIFY_ARGS="--sweep basic_right_arm --per-target-timeout-s 10"
```

Expected: evidence JSON contains `"sweep": "basic_right_arm"` and `"passed": true`.

- [ ] **Step 5: Commit Task 2**

```bash
git add isaacsim_test/lerobot/verify_lerobot_sitl.py isaacsim_test/test_v2_roboparty_config.py
git commit -m "test: add right-arm SITL sweep verifier"
```

---

## Task 3: Make screenshots useful for intended-vs-observed pose review

**Purpose:** Capture repeatable views that make the right arm visible, not just the whole robot in the scene.

**Files:**
- Modify: `isaacsim_test/isaacsim/setup_rpo_arm_scene.py`
- Modify: `isaacsim_test/lerobot/verify_lerobot_sitl.py`
- Modify: `isaacsim_test/test_v2_roboparty_config.py`

- [ ] **Step 1: Add named camera views**

Add environment variable `SCREENSHOT_VIEW` with these supported values:

```text
iso
right_arm_closeup
front
side
```

Use these camera placements:

```python
SCREENSHOT_VIEWS = {
    "iso": {"position": (1.4, -1.6, 1.0), "look_at": (0.0, 0.0, 0.1)},
    "right_arm_closeup": {"position": (0.8, -0.8, 0.55), "look_at": (0.0, -0.25, 0.25)},
    "front": {"position": (1.2, 0.0, 0.7), "look_at": (0.0, 0.0, 0.25)},
    "side": {"position": (0.0, -1.2, 0.7), "look_at": (0.0, 0.0, 0.25)},
}
```

- [ ] **Step 2: Encode command metadata into screenshot filenames**

For sweep mode, write screenshots like:

```text
isaacsim_test/artifacts/screenshots/target_000_iso.png
isaacsim_test/artifacts/screenshots/target_000_right_arm_closeup.png
isaacsim_test/artifacts/screenshots/target_001_iso.png
isaacsim_test/artifacts/screenshots/target_001_right_arm_closeup.png
```

- [ ] **Step 3: Add evidence references**

Add screenshot paths to each target result in `lerobot_sitl_verify.json`:

```json
{
  "target_index": 1,
  "screenshots": [
    "isaacsim_test/artifacts/screenshots/target_001_iso.png",
    "isaacsim_test/artifacts/screenshots/target_001_right_arm_closeup.png"
  ]
}
```

- [ ] **Step 4: Run image checks**

Run:

```bash
python3 - <<'PY'
from pathlib import Path
from PIL import Image, ImageStat
for path in sorted(Path('isaacsim_test/artifacts/screenshots').glob('*.png')):
    with Image.open(path) as im:
        stat = ImageStat.Stat(im.convert('RGB'))
        assert im.size == (1280, 720), path
        assert max(stat.extrema[0]) > min(stat.extrema[0]), path
        print(path, im.size, [round(v, 2) for v in stat.mean])
PY
```

Expected: every screenshot is `1280x720` and non-blank.

- [ ] **Step 5: Commit Task 3**

```bash
git add isaacsim_test/isaacsim/setup_rpo_arm_scene.py isaacsim_test/lerobot/verify_lerobot_sitl.py isaacsim_test/test_v2_roboparty_config.py
git commit -m "test: add repeatable SITL pose screenshots"
```

---

## Task 4: Upgrade from command echo to physics-aware actuation checks

**Purpose:** Confirm Isaac Sim articulation behavior is plausible, including joint limits and no unexpected collisions or unstable simulation.

**Files:**
- Modify: `isaacsim_test/isaacsim/setup_rpo_arm_scene.py`
- Create: `isaacsim_test/isaacsim/export_joint_metadata.py`
- Create: `isaacsim_test/artifacts/README.md` only if artifact schema documentation is needed; keep generated artifact files ignored by git.

- [ ] **Step 1: Export joint metadata from the imported URDF**

Create a script that loads the V2.0 URDF and writes this schema to `isaacsim_test/artifacts/joint_metadata.json`:

```json
{
  "joint_names": ["right_arm_pitch_joint"],
  "lower_limits": [-1.57],
  "upper_limits": [1.57],
  "effort_limits": [10.0],
  "velocity_limits": [1.0]
}
```

- [ ] **Step 2: Clip commands using the exported limits**

In `setup_rpo_arm_scene.py`, clamp the first five command values to the joint limits exported by Isaac/URDF. Evidence JSON must include both original command and applied command.

- [ ] **Step 3: Add limit regression cases**

Use this target list for limit checks:

```python
LIMIT_CHECK_TARGETS = [
    [999.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [-999.0, 0.0, 0.0, 0.0, 0.0, 0.0],
]
```

Expected: observed state equals clipped applied command within tolerance, not the unclipped command.

- [ ] **Step 4: Run physics-aware verification**

Run:

```bash
cd isaacsim_test
bash run_lerobot_sitl_check.sh VERIFY_ARGS="--sweep basic_right_arm --include-limit-checks"
```

Expected: evidence JSON contains `applied_target` and all observed states match applied targets.

- [ ] **Step 5: Commit Task 4**

```bash
git add isaacsim_test/isaacsim/setup_rpo_arm_scene.py isaacsim_test/isaacsim/export_joint_metadata.py
git commit -m "test: add physics-aware SITL joint limit checks"
```

---

## Task 5: Load SimReady USD and establish control binding

**Purpose:** Move the Isaac scene from the historical URDF baseline to the validated SimReady `echo_full` USD, then document which USD prims are bound to the existing 6D LeRobot/ROS2 command interface.

**Files:**
- Modify: `isaacsim_test/isaacsim/setup_rpo_arm_scene.py`
- Modify: `isaacsim_test/.env.example`
- Modify: `isaacsim_test/docker-compose.yml`
- Modify: `isaacsim_test/README.md`
- Modify: `integration_guide/09_isaacsim_sim_loop_plan.md`
- Add tests in: `isaacsim_test/test_v2_roboparty_config.py`

- [x] **Step 1: Add SimReady asset configuration**

Add this default path to `.env.example` and Docker Compose:

```text
SIMREADY_USD_PATH=/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd
```

- [x] **Step 2: Load the SimReady USD in Isaac Sim**

Update `setup_rpo_arm_scene.py` so startup loads `SIMREADY_USD_PATH` and logs:

```text
[setup_rpo_arm_scene] Loading SimReady USD: .../echo_full_robot_arm_hand.usd
```

The historical URDF import may remain behind an explicit fallback flag, but it must not be the default for new SITL work.

- [x] **Step 3: Export prim/control mapping evidence**

Write `isaacsim_test/artifacts/simready_prim_mapping.json` with:

```json
{
  "asset": "echo_full_robot_arm_hand.usd",
  "control_contract": [
    "right_arm_pitch_joint.pos",
    "right_arm_roll_joint.pos",
    "right_arm_yaw_joint.pos",
    "right_elbow_pitch_joint.pos",
    "right_elbow_yaw_joint.pos",
    "amazinghand_grasp.pos"
  ],
  "binding_status": "bound_or_binding_pending_per_feature"
}
```

- [x] **Step 4: Keep the 6D LeRobot contract stable first**

For the first SimReady import pass, keep LeRobot action/state shape `(6,)`. If any feature cannot yet drive a real USD articulation, record `binding_pending` in evidence JSON rather than falling back to temporary geometry.

- [x] **Step 5: Verify SimReady asset visibility**

Run:

```bash
cd isaacsim_test
SIMREADY_USD_PATH=/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd \
  SCREENSHOT_ON_STARTUP=1 \
  SCREENSHOT_PATH=/workspace/superarm_ws/isaacsim_test/artifacts/echo_full_simready_startup.png \
  docker compose up isaac-sim-51
```

Expected: screenshot shows the SimReady `echo_full` asset, and evidence JSON still reports LeRobot shape `(6,)`.

Runtime result recorded in `docs/sitl/2026-06-27/simready_runtime_evidence.md`.
In this headless container, Isaac Sim loaded the SimReady USD and wrote the mapping evidence, then used the committed SimReady thumbnail as visual evidence because the viewport capture APIs were unavailable or timed out without a default window.

- [x] **Step 6: Commit Task 5**

```bash
git add isaacsim_test/isaacsim/setup_rpo_arm_scene.py isaacsim_test/.env.example isaacsim_test/docker-compose.yml integration_guide/09_isaacsim_sim_loop_plan.md isaacsim_test/test_v2_roboparty_config.py docs/sitl/2026-06-27/README.md docs/sitl/2026-06-27/simready_runtime_evidence.md
git commit -m "test: record SimReady SITL runtime evidence"
```

---

## Task 6: Add LeRobot episode recording and replay gates

**Purpose:** Make SITL useful for datasets, not just one-off commands.

**Files:**
- Create: `isaacsim_test/lerobot/record_sitl_episode.sh`
- Create: `isaacsim_test/lerobot/replay_sitl_episode.sh`
- Modify: `isaacsim_test/README.md`
- Modify: `isaacsim_test/test_v2_roboparty_config.py`

- [ ] **Step 1: Record a tiny deterministic episode**

The record script must use the local robot type:

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=isaacsim_rpo_arm \
  --control.type=record \
  --control.repo_id=local/rpo_v2_right_arm_sitl_smoke \
  --control.single_task="Move RoboParty V2 right arm through a short SITL smoke trajectory." \
  --control.fps=10 \
  --control.num_episodes=1
```

- [ ] **Step 2: Replay the episode into Isaac Sim**

The replay script must load the recorded episode and publish actions through `IsaacSimRpoArmRobot.send_action()`, not direct ROS publishing.

- [ ] **Step 3: Verify dataset schema**

Write a small Python check that asserts:

```python
assert dataset.features["observation.state"].shape == (6,)
assert dataset.features["action"].shape == (6,)
```

- [ ] **Step 4: Commit Task 6**

```bash
git add isaacsim_test/lerobot/record_sitl_episode.sh isaacsim_test/lerobot/replay_sitl_episode.sh isaacsim_test/README.md isaacsim_test/test_v2_roboparty_config.py
git commit -m "test: add LeRobot SITL record and replay gates"
```

---

## Task 7: Add policy rollout smoke test

**Purpose:** Verify a trained or deterministic LeRobot policy can drive the same Isaac Sim robot interface used for teleop and recording.

**Files:**
- Create: `isaacsim_test/lerobot/run_policy_sitl_smoke.py`
- Modify: `isaacsim_test/README.md`
- Modify: `isaacsim_test/test_v2_roboparty_config.py`

- [ ] **Step 1: Implement a deterministic policy smoke sequence first**

Create a Python script that loads `IsaacSimRpoArmRobot`, connects, and sends this sequence at 10 Hz:

```python
POLICY_SMOKE_ACTIONS = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.1, 0.0, -0.1, 0.2, 0.0, 0.3],
    [0.2, 0.1, -0.2, 0.3, 0.1, 0.6],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
]
```

- [ ] **Step 2: Save rollout evidence**

Write `isaacsim_test/artifacts/policy_sitl_smoke.json` with:

```json
{
  "passed": true,
  "num_actions": 4,
  "max_error": 0.0,
  "robot_type": "isaacsim_rpo_arm"
}
```

- [ ] **Step 3: Commit Task 7**

```bash
git add isaacsim_test/lerobot/run_policy_sitl_smoke.py isaacsim_test/README.md isaacsim_test/test_v2_roboparty_config.py
git commit -m "test: add LeRobot policy SITL smoke test"
```

---

## Task 8: Prepare hardware parity checklist

**Purpose:** Ensure the sim contract can map cleanly to the real RoboParty V2 arm and AmazingHand before real robot testing.

**Files:**
- Create: `docs/sitl/2026-06-27/hardware_parity_checklist.md`
- Modify: `docs/task_guides/07_validation_gates.md`

- [ ] **Step 1: Record the exact sim-to-real mapping**

Create `docs/sitl/2026-06-27/hardware_parity_checklist.md` with this table:

```markdown
| LeRobot feature | Isaac joint/source | Real hardware signal | Unit | Direction verified |
|---|---|---|---|---|
| right_arm_pitch_joint.pos | RoboParty V2 URDF | CAN/motor mapping pending hardware check | radians | no |
| right_arm_roll_joint.pos | RoboParty V2 URDF | CAN/motor mapping pending hardware check | radians | no |
| right_arm_yaw_joint.pos | RoboParty V2 URDF | CAN/motor mapping pending hardware check | radians | no |
| right_elbow_pitch_joint.pos | RoboParty V2 URDF | CAN/motor mapping pending hardware check | radians | no |
| right_elbow_yaw_joint.pos | RoboParty V2 URDF | CAN/motor mapping pending hardware check | radians | no |
| amazinghand_grasp.pos | SimReady USD hand/body prims, binding pending | AmazingHand command mapping pending hardware check | normalized 0..1 | no |
```

- [ ] **Step 2: Add validation gates before hardware power-on**

Add these gates to `docs/task_guides/07_validation_gates.md`:

```markdown
- SITL sweep evidence exists and passed.
- Each real joint direction is checked at low speed against the corresponding SITL feature name.
- Joint limits in software are stricter than mechanical limits.
- Emergency stop and power cut behavior are verified before policy replay.
```

- [ ] **Step 3: Commit Task 8**

```bash
git add docs/sitl/2026-06-27/hardware_parity_checklist.md docs/task_guides/07_validation_gates.md
git commit -m "docs: add SITL hardware parity checklist"
```

---

## Recommended next execution order

1. Task 1 — one-command SITL gate.
2. Task 2 — multi-target joint sweep.
3. Task 3 — close-up screenshots tied to each target.
4. Task 4 — physics-aware limits and applied-command evidence.
5. Continue from Task 5 evidence — inspect the full USD prim hierarchy and replace `binding_pending` entries with real control bindings where available.
6. Task 6 — record and replay LeRobot episodes.
7. Task 7 — policy rollout smoke test.
8. Task 8 — sim-to-real safety checklist before hardware.

Stop after each task and verify:

```bash
python3 isaacsim_test/test_v2_roboparty_config.py
python3 -m py_compile isaacsim_test/isaacsim/setup_rpo_arm_scene.py isaacsim_test/lerobot/verify_lerobot_sitl.py isaacsim_test/test_v2_roboparty_config.py
git diff --check
```

For tasks that touch Isaac Sim behavior, also run:

```bash
cd isaacsim_test
bash run_lerobot_sitl_check.sh
```

Expected before committing an Isaac Sim behavior task: JSON evidence says `"passed": true`, at least one screenshot exists, and the screenshot is visually inspected.
