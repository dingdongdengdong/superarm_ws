# LeLab → Isaac Sim Source Arm Integration Notes

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
local branch: feature/superarm-isaacsim-control
patch files:
  - isaacsim_test/lelab_patches/0001-Add-Isaac-Sim-RPO-arm-teleoperation-backend.patch
  - isaacsim_test/lelab_patches/0002-Add-SuperArm-lightweight-Isaac-Sim-control-server.patch
```

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
- `isaacsim_test/lerobot/source_arm_isaacsim_arm_only.yaml`
- `isaacsim_test/run_source_arm_lerobot_pose_capture.sh`

The validated Isaac Sim joint contract is:

```text
joint_rev_1, joint_rev_2, joint_rev_3, joint_rev_4
```

## Integration plan

1. Keep our `IsaacSimRpoArmRobot` as the robot backend source of truth.
2. Add a LeLab backend selector instead of hardcoding SO-101 everywhere.
3. First MVP should be a source-arm joint-control mode, not leader/follower
   teleoperation:
   - LeLab UI slider/vector action → `IsaacSimRpoArmRobot.send_action()`
   - Isaac/ROS readback → LeLab joint display/websocket broadcast
   - optional screenshot/debug endpoint → existing Isaac camera capture artifacts
4. After direct joint control works, adapt record/replay around the same backend.

## Known-good validation before LeLab work

Latest run:

```text
/workspaces/superarm_ws/isaacsim_test/artifacts/source_arm_lerobot_pose_cases_20260705T104845Z
```

Result:

```text
4/4 LeRobot source-arm pose cases passed
4/4 live Isaac screenshots captured
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

Run our existing Isaac/LeRobot multi-pose validation:

```bash
cd /workspaces/superarm_ws
./isaacsim_test/run_source_arm_lerobot_pose_capture.sh
```

## Implemented MVP patch

The local LeLab clone now has commits `f453845` and `b330278` with:

- `robot_backend="isaacsim_rpo_arm"` support in `lelab/teleoperate.py`.
- dynamic import of this repo's `IsaacSimRpoArmRobot` from `SUPERARM_WS_PATH` or `/workspaces/superarm_ws`.
- `/send-joint-action` API in `lelab/server.py` for sending one vector/list action to the active backend.
- generic `joint_rev_*.pos` and vector readback conversion for websocket/joint-position display.
- targeted LeLab test coverage in `tests/test_teleoperate.py`.
- lightweight Python 3.10-compatible `lelab.superarm_server` FastAPI UI for ROS2 Humble + Isaac Sim control without importing the full upstream LeLab app stack.

Validation run inside `worktrees/leLab`:

```text
python3 -m py_compile worktrees/leLab/lelab/teleoperate.py worktrees/leLab/lelab/server.py worktrees/leLab/tests/test_teleoperate.py
cd worktrees/leLab && python3 -m pytest tests/test_teleoperate.py -q
# 7 passed, 2 warnings
```

API sketch for Isaac Sim mode:

```json
POST /move-arm
{
  "robot_backend": "isaacsim_rpo_arm",
  "leader_port": "unused",
  "follower_port": "unused",
  "leader_config": "unused",
  "follower_config": "isaacsim_test/lerobot/source_arm_isaacsim_arm_only.yaml",
  "superarm_ws_path": "/workspaces/superarm_ws"
}

POST /send-joint-action
{
  "action": [0.25, -0.2, 0.3, -0.35]
}
```

## Lightweight control server runbook

This is the runtime path used for Isaac Sim control because ROS2 Humble's
`rclpy` is Python 3.10-bound, while the full LeLab app stack currently pulls in
newer Python/runtime assumptions. The lightweight server imports only
`lelab.teleoperate` plus FastAPI and exposes the SuperArm endpoints/UI.

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
python3 -m uvicorn lelab.superarm_server:app --host 0.0.0.0 --port 8000
```

Open:

```text
http://127.0.0.1:8000/
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
