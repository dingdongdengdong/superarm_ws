# LeLab → Isaac Sim Source Arm Integration Notes

Branch: `feature/lelab-isaacsim-arm-control`

Local upstream clone:

```text
/workspaces/superarm_ws/worktrees/leLab
```

LeLab clone state captured during setup:

```text
repo: https://github.com/huggingface/leLab.git
commit: 8f8a50f Force-release camera/serial resources when a device disconnect fails
local branch: feature/superarm-isaacsim-control
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
