# 00 - Branch and Repo Workflow

## Goal

Create a clean branch for task planning and push only intentional project files
to the remote repository.

## Current repo facts

```text
Workspace: /workspaces/superarm_ws
Remote:    https://github.com/dingdongdengdong/superarm_ws.git
Branch:    task-guides/roboparty-lerobot-plan
State:     initial repository with no committed baseline yet
```

The workspace contains nested cloned repositories:

```text
AmazingHand/.git
lerobot/.git
roboto_origin/.git
```

Do not blindly run `git add -A` from the workspace root. That can accidentally
stage nested repositories or large generated content.

## Safe branch workflow

Use a branch for each meaningful project change:

```bash
git switch -c task-guides/roboparty-lerobot-plan
```

For future implementation work, use names like:

```text
bringup/roboparty-arm-can
bringup/amazinghand-serial
feature/lerobot-rpo5-ah-wrapper
data/so100-cube-tray-debug
data/rpo5-ah-cube-tray-debug
```

## Safe staging rule

Stage explicit files:

```bash
git add roleandr.md
git add docs/task_guides
git add integration_guide
git add lerobot_custom_config_whole_arm_hand_control.md
```

Avoid this unless the repo has been cleaned and reviewed:

```bash
git add -A
```

## Nested repo decision

Pick one of these before pushing the full project:

```text
Option A: Keep nested repos as external dependencies.
  Commit only docs and integration code.
  Document exact upstream URLs and commits.

Option B: Add nested repos as git submodules.
  Good when you need reproducible source checkouts.
  Requires `git submodule add` or converting existing folders.

Option C: Vendor selected source files.
  Good only for small copied adapters.
  Avoid vendoring full upstream repos unless necessary.
```

Recommendation: use Option A first. The current workspace is large, and the
external repositories already have their own git history.

## Commit and push

After reviewing `git status -sb`, commit the intended files:

```bash
git commit -m "docs: add RoboParty LeRobot task guides"
```

Push the branch:

```bash
git push -u origin task-guides/roboparty-lerobot-plan
```

## Done when

```text
[ ] Branch exists locally.
[ ] Only intentional docs are staged.
[ ] Commit exists on the task branch.
[ ] Branch is pushed to origin.
[ ] GitHub shows the branch and committed markdown files.
```
