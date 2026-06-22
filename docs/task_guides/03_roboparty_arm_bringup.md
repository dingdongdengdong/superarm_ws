# 03 - RoboParty Arm Bring-Up

## Goal

Bring up one RoboParty 5-DOF arm safely before connecting AmazingHand or running
LeRobot policy control.

## Required assumptions

Start with the right arm unless the hardware team decides otherwise:

```text
right arm candidate: can3, motor IDs 19-23
left arm candidate:  can2, motor IDs 14-18
```

These IDs are inferred from the RoboParty config and must be verified on the
physical robot.

## Safety setup

```text
[ ] Physical emergency stop is reachable.
[ ] Power supply current limit is configured.
[ ] Arm is mechanically supported during first tests.
[ ] No hand or payload is attached for first motor test.
[ ] CAN adapter is connected through a stable USB port.
[ ] One engineer watches the robot while another sends commands.
```

## Bring-up sequence

1. Confirm hardware version:

```text
V1.0 or V2.0
```

2. Confirm CAN interface:

```bash
ip link show
```

3. Bring up the candidate CAN interface:

```bash
sudo ip link set can3 up type can bitrate 1000000
```

Adjust bitrate to match the RoboParty deployment config.

4. Run RoboParty's existing zeroing or motor inspection script in read-only or
minimal-motion mode first.

5. Test one motor with a tiny command:

```text
target delta: 0.5 to 1.0 degree
speed: low
load: no payload
```

6. Record actual signs:

```text
rpo_arm_j1: +1 or -1
rpo_arm_j2: +1 or -1
rpo_arm_j3: +1 or -1
rpo_arm_j4: +1 or -1
rpo_arm_j5: +1 or -1
```

7. Test all five joints individually.

8. Test a slow neutral-pose move with all five joints.

## Bring-up log format

Create:

```text
docs/task_guides/roboparty_arm_bringup_log.md
```

Use this table:

```markdown
| Date | Arm side | CAN | Motor ID | Joint | Direction OK | Limit OK | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 2026-06-22 | right | can3 | 19 | rpo_arm_j1 | yes/no | yes/no | note |
```

## Done when

```text
[ ] Correct CAN interface is known.
[ ] Correct five motor IDs are known.
[ ] Joint signs are recorded.
[ ] Joint soft limits are recorded.
[ ] Zeroing procedure works for arm-only setup.
[ ] A tiny 5-joint motion runs without unexpected movement.
```
