# 06 — Dataset and Policy Workflow for 5-DOF RoboParty Arm + AmazingHand

Correction: this workflow assumes:

```text
RoboParty 5-DOF arm + AmazingHand
```

not OpenArm.

## 1. First action representation

Use this first:

```text
5 arm joint targets + 1 AmazingHand grasp scalar
```

LeRobot-facing flat feature keys:

```text
rpo_arm_j1.pos
rpo_arm_j2.pos
rpo_arm_j3.pos
rpo_arm_j4.pos
rpo_arm_j5.pos
amazinghand_grasp.pos
```

`amazinghand_grasp`:

```text
0.0 = open
1.0 = close / power grasp
```

This is easier and safer than exposing all 8 hand servos at the beginning.
Arm joint keys are in degrees. The hand key is a scalar in `[0.0, 1.0]`.

## 2. Minimum dataset features

```text
rpo_arm_j1.pos
rpo_arm_j2.pos
rpo_arm_j3.pos
rpo_arm_j4.pos
rpo_arm_j5.pos
amazinghand_grasp.pos
observation.images.front
observation.images.wrist   optional but recommended
timestamp
episode_index
frame_index
task
```

## 3. Why 5 DOF changes task design(but 5 dof is same with  so-100 arm)

A 5-DOF arm has less orientation freedom than a 7-DOF arm. The policy will have trouble if the task requires precise wrist orientation.

Choose first tasks like:

```text
front-facing pick
large foam cube
tray placement
fixed object zone
fixed camera
fixed table height
```

Avoid first:

```text
insertion
tool use
narrow peg tasks
side grasps
in-hand reorientation
cluttered scenes
```

## 4. First dataset task

```text
Task: pick foam cube and place into tray
Arm: fixed base/table-mounted
Hand: AmazingHand scalar open/close
Object: one soft/light cube
Start area: marked 10 cm x 10 cm square
Tray: fixed position
Episodes: 10 debug, then 50–100 baseline
```

## 5. Recording command template

```bash
lerobot-record \
  --robot.type=roboparty_5dof_arm_amazinghand_follower \
  --robot.id=rpo_right_arm_ah_v1 \
  --robot.arm_can_interface=can3 \
  --robot.hand_serial_port=/dev/ttyUSB_AH_RIGHT \
  --repo-id=YOUR_HF_USERNAME/rpo5_ah_pick_cube_v1_debug \
  --fps=30 \
  --num-episodes=10
```

If using XR teleop:

```bash
lerobot-record \
  --robot.type=roboparty_5dof_arm_amazinghand_follower \
  --teleop.type=roboparty_xr_5dof \
  --repo-id=YOUR_HF_USERNAME/rpo5_ah_xr_pick_cube_v1 \
  --fps=30 \
  --num-episodes=10
```

Adjust flags to your LeRobot version.

## 6. Dataset QA checklist

After 10 debug episodes:

- [ ] State shape is `(6,)`.
- [ ] Action shape is `(6,)`.
- [ ] Joint order is stable.
- [ ] `amazinghand_grasp` changes when the hand opens/closes.
- [ ] Camera sees the cube, hand, and tray.
- [ ] No sudden jump in arm targets.
- [ ] Hand scalar is clamped to `[0, 1]`.
- [ ] Failed episodes are marked or discarded.

## 7. First ACT training command

```bash
lerobot-train \
  --dataset.repo_id=YOUR_HF_USERNAME/rpo5_ah_pick_cube_v2_100eps \
  --policy.type=act \
  --output_dir=outputs/train/act_rpo5_ah_pick_cube_v2 \
  --job_name=act_rpo5_ah_pick_cube_v2 \
  --policy.device=cuda \
  --wandb.enable=true \
  --policy.repo_id=YOUR_HF_USERNAME/act_rpo5_ah_pick_cube_v2
```

## 8. Evaluation protocol

First evaluation should be in the same setup:

```text
same cube
same tray
same table
same lighting
same camera poses
same arm mounting
```

Run:

```text
20 trials
record success/failure
record failure reason
keep emergency stop ready
```

Failure labels:

```text
wrong approach angle
not enough reach
bad hand timing
missed cube
dropped object
hit table
hit tray
policy oscillation
camera failure
```

## 9. Scaling plan

```text
10 episodes  : debug only
50 episodes  : first ACT sanity check
100 episodes : first baseline
300 episodes : small object position variation
500+ episodes: hand pattern controls
```

Do not jump to full 8-servo dexterity until the 5+1 baseline works.

## 10. Later dexterous dataset

When ready, change the action to:

```text
5 arm joints + 8 AmazingHand servo targets
```

This is a later 13D policy surface, not v1:

```text
5 arm joint keys + 8 raw AmazingHand servo target keys
```

Do this only after:

```text
servo limits are safe
hand calibration is repeatable
operator can teleop fingers smoothly
you have a reason to need finger-level dexterity
```
