# SITL Phase 2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the current verified Isaac Sim 5.1 + RoboParty V2.0 right-arm LeRobot command test into a repeatable SITL pipeline for actuation, perception, data recording, policy replay, and later hardware parity.

**Architecture:** Keep Isaac Sim as the simulated follower, LeRobot as the control/record/replay interface, and ROS2 as the transport boundary. Every phase must produce machine-checkable artifacts under `isaacsim_test/artifacts/` and a short human-readable run log so regressions can be found quickly.

**Tech Stack:** Isaac Sim 5.1 container, official RoboParty V2.0 URDF, ROS2 Humble, local LeRobot `isaacsim_rpo_arm` robot type, Docker Compose, Python verifier scripts, NVIDIA Agent Skills catalog for NVIDIA/Omniverse-specific work.

**Created:** 2026-06-27, Asia/Seoul

**Directory:** `docs/sitl/2026-06-27/`

**Baseline commit:** `04ef79a test: add LeRobot SITL verifier with screenshots`

---

## Baseline already verified on 2026-06-27

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

Known limits of this baseline:

- The current arm motion is a command/state bridge test; it is not yet a physics-quality actuator validation.
- `amazinghand_grasp` is still a synthetic scalar, not a mounted multi-DOF hand model.
- The screenshot proves Isaac Sim rendering after command, but the camera framing is not yet an arm-pose measurement tool.
- The verifier checks one pose; it does not sweep joint limits, trajectories, contact, collisions, or policy rollouts.

---

## Non-negotiable source rules

- [ ] Use RoboParty V2.0 assets by default. Do not recreate the V2 right-arm URDF while the official URDF exists.
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

## Task 5: Replace synthetic AmazingHand scalar with a mounted hand model

**Purpose:** Move from `amazinghand_grasp` as metadata to a visible simulated hand attached to the RoboParty V2 right-arm chain.

**Files:**
- Modify: `isaacsim_test/isaacsim/setup_rpo_arm_scene.py`
- Modify: `isaacsim_test/README.md`
- Modify: `integration_guide/09_isaacsim_sim_loop_plan.md`
- Add tests in: `isaacsim_test/test_v2_roboparty_config.py`

- [ ] **Step 1: Decide mount link from imported model evidence**

Run Isaac Sim metadata export and confirm the available right-arm terminal link. Expected candidate from the current contract is near `right_elbow_yaw_joint`; record the exact link name in `isaacsim_test/artifacts/joint_metadata.json`.

- [ ] **Step 2: Add a hand asset mount configuration**

Add environment variables:

```text
AMAZINGHAND_ASSET_PATH=/workspace/superarm_ws/AmazingHand/...
AMAZINGHAND_MOUNT_LINK=<exact terminal link from metadata>
AMAZINGHAND_MOUNT_ENABLED=1
```

- [ ] **Step 3: Keep the 6D LeRobot contract stable first**

For the first hand mount pass, keep LeRobot action/state shape `(6,)`. Map `amazinghand_grasp` to a simple visual open/close state or logged scalar while the visible hand is attached.

- [ ] **Step 4: Verify hand visibility**

Run:

```bash
cd isaacsim_test
AMAZINGHAND_MOUNT_ENABLED=1 SCREENSHOT_VIEW=right_arm_closeup bash run_lerobot_sitl_check.sh
```

Expected: screenshot shows the right arm with the mounted hand, and evidence JSON still reports shape `(6,)`.

- [ ] **Step 5: Commit Task 5**

```bash
git add isaacsim_test/isaacsim/setup_rpo_arm_scene.py isaacsim_test/README.md integration_guide/09_isaacsim_sim_loop_plan.md isaacsim_test/test_v2_roboparty_config.py
git commit -m "feat: mount AmazingHand asset in RoboParty SITL"
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

**Purpose:** Verify a trained or dummy LeRobot policy can drive the same Isaac Sim robot interface used for teleop and recording.

**Files:**
- Create: `isaacsim_test/lerobot/run_policy_sitl_smoke.py`
- Modify: `isaacsim_test/README.md`
- Modify: `isaacsim_test/test_v2_roboparty_config.py`

- [ ] **Step 1: Implement a deterministic dummy policy first**

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
| amazinghand_grasp.pos | synthetic scalar or mounted hand model | AmazingHand command mapping pending hardware check | normalized 0..1 | no |
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
5. Task 5 — visible AmazingHand mount while keeping 6D LeRobot compatibility.
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
