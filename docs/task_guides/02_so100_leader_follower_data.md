# 02 - SO-100 Leader/Follower Data Workflow

## Goal

Use the LeRobot SO-100 leader/follower setup as a low-risk proxy workflow for
operator training, camera placement, dataset QA, and first ACT training tests.

## Why this matters

SO-100 is already native to LeRobot. It lets the C.S side practice the full
collect-train-evaluate loop while M.E and E.E finish the RoboParty + AmazingHand
hardware integration.

Treat SO-100 data as proxy data. Do not mix it into the RoboParty + AmazingHand
training set unless an explicit action and observation mapping has been defined.

## Setup checks

```text
[ ] SO-100 follower powers on safely.
[ ] Leader arm connects to the control computer.
[ ] Motor voltage matches the SO-100 hardware variant.
[ ] Cameras are mounted in the same approximate positions planned for RoboParty.
[ ] Cube, tray, table height, and lighting are fixed.
```

## Suggested commands

Find ports:

```bash
lerobot-find-port
```

Calibrate follower and leader:

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=<FOLLOWER_PORT> --robot.id=so100_proxy_follower
lerobot-calibrate --teleop.type=so101_leader --teleop.port=<LEADER_PORT> --teleop.id=so100_proxy_leader
```

Teleoperate without recording:

```bash
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=<FOLLOWER_PORT> \
  --robot.id=so100_proxy_follower \
  --teleop.type=so101_leader \
  --teleop.port=<LEADER_PORT> \
  --teleop.id=so100_proxy_leader
```

Record 10 debug episodes:

```bash
lerobot-record \
  --robot.type=so101_follower \
  --robot.port=<FOLLOWER_PORT> \
  --robot.id=so100_proxy_follower \
  --teleop.type=so101_leader \
  --teleop.port=<LEADER_PORT> \
  --teleop.id=so100_proxy_leader \
  --repo-id=<HF_USER>/so100_cube_tray_debug_v1 \
  --fps=30 \
  --num-episodes=10
```

Adjust robot and teleop type names to match the exact LeRobot version in this
workspace.

## Dataset QA

After 10 episodes, check:

```text
[ ] Every episode has a visible cube, gripper, and tray.
[ ] The action stream changes smoothly.
[ ] Failed demonstrations are marked or removed.
[ ] Camera frames are not upside down or overexposed.
[ ] The operator can complete the task repeatedly without awkward motions.
[ ] Episode reset state is consistent.
```

## Transfer checklist before using SO-100 data for RoboParty

```text
[ ] Joint count and joint order are mapped.
[ ] Joint signs are mapped.
[ ] Joint limits are mapped.
[ ] Workspace scale difference is documented.
[ ] Gripper action is mapped to AmazingHand grasp scalar.
[ ] Camera viewpoints match closely enough.
[ ] Control frequency is compatible.
```

## Done when

```text
[ ] 10 SO-100 debug episodes are recorded.
[ ] Dataset visual inspection passes.
[ ] The team has a written list of SO-100 failure cases.
[ ] The task fixture is stable enough to reuse with RoboParty.
```
