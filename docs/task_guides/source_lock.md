# Source Lock

This file records the upstream repositories currently present in the workspace.
The root project should commit planning docs and local integration code
intentionally; the upstream repositories are kept as external checkouts unless
the team later converts them to submodules.

| Component | Remote | Commit | Local state |
| --- | --- | --- | --- |
| `roboto_origin` | `https://github.com/Roboparty/roboto_origin` | `1488b8c4527eded3b663fb34132b8347cfc3b1e6` | clean |
| `AmazingHand` | `https://github.com/pollen-robotics/AmazingHand` | `3e8241074df3436a3044ced4881e3bb2133aa725` | clean |
| `lerobot` | `https://github.com/huggingface/lerobot` | `2d7a42011a4f8e05a8c85d5fb908da258d4cc7b1` | dirty |

## Dirty LeRobot files observed on 2026-06-22

```text
src/lerobot/robots/utils.py
src/lerobot/scripts/lerobot_calibrate.py
src/lerobot/scripts/lerobot_find_joint_limits.py
src/lerobot/scripts/lerobot_record.py
src/lerobot/scripts/lerobot_replay.py
src/lerobot/scripts/lerobot_rollout.py
src/lerobot/scripts/lerobot_teleoperate.py
src/lerobot/robots/roboparty_5dof_arm_amazinghand/
tests/robots/test_roboparty_5dof_arm_amazinghand.py
```

## Repo policy

```text
RoboParty, AmazingHand, and LeRobot remain external dependencies for now.
Commit docs and local project metadata in the root repository.
Convert upstream checkouts to submodules only if the team wants reproducible
source checkout management through the root repo.
```
