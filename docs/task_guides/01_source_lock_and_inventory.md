# 01 - Source Lock and Inventory

## Goal

Record which upstream sources are being used and freeze the exact versions before
hardware bring-up or custom LeRobot work starts.

## Sources

```text
RoboParty root:       roboto_origin/
RoboParty hardware:   roboto_origin/modules/rpo_hardware/
RoboParty deploy:     roboto_origin/modules/roboparty_deploy/
RoboParty description:roboto_origin/modules/rpo_description/
RoboParty XR teleop:  roboto_origin/modules/roboparty_xr_teleop/
AmazingHand:          AmazingHand/
LeRobot:              lerobot/
```

## Commands

Run these in each nested repo and copy the result into a source-lock note:

```bash
git remote -v
git rev-parse HEAD
git status --short
```

Output file:

```text
docs/task_guides/source_lock.md
```

Current source lock format:

```markdown
# Source Lock

| Component | Remote | Commit | Local changes |
| --- | --- | --- | --- |
| roboto_origin | upstream URL | exact commit hash | clean / dirty |
| AmazingHand | upstream URL | exact commit hash | clean / dirty |
| LeRobot | upstream URL | exact commit hash | clean / dirty |
```

## Inventory checks

Confirm these files or directories exist:

```text
roboto_origin/modules/rpo_hardware
roboto_origin/modules/roboparty_deploy
roboto_origin/modules/rpo_description
roboto_origin/modules/roboparty_xr_teleop
AmazingHand/PythonExample
lerobot/src/lerobot/robots
```

## Done when

```text
[ ] Each upstream repo has a recorded remote URL.
[ ] Each upstream repo has a recorded commit hash.
[ ] Dirty upstream repos are called out explicitly.
[ ] Hardware version V1.0 or V2.0 is recorded.
[ ] The team knows whether nested repos are external dependencies or submodules.
```
