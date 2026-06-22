# 06 - Dataset, Training, and Evaluation

## Goal

Record a small RoboParty + AmazingHand dataset, train the first ACT baseline, and
evaluate on the same physical setup.

## First task

```text
Task: pick foam cube and place into tray
Arm: fixed base or table-mounted
Hand: AmazingHand scalar open/close
Object: one soft/light cube
Start area: marked 10 cm x 10 cm square
Tray: fixed position
Episodes: 10 debug, then 50-100 baseline
```

## Dataset features

```text
rpo_arm_j1.pos
rpo_arm_j2.pos
rpo_arm_j3.pos
rpo_arm_j4.pos
rpo_arm_j5.pos
amazinghand_grasp.pos
observation.images.front
observation.images.wrist
timestamp
episode_index
frame_index
task
```

## Record debug episodes

```bash
lerobot-record \
  --robot.type=roboparty_5dof_arm_amazinghand_follower \
  --robot.id=rpo_right_arm_ah_v1 \
  --robot.port=can3 \
  --robot.hand_serial_port=/dev/ttyUSB_AH_RIGHT \
  --repo-id=<HF_USER>/rpo5_ah_cube_tray_debug_v1 \
  --fps=30 \
  --num-episodes=10
```

If using XR teleop:

```bash
lerobot-record \
  --robot.type=roboparty_5dof_arm_amazinghand_follower \
  --teleop.type=roboparty_xr_5dof \
  --repo-id=<HF_USER>/rpo5_ah_xr_cube_tray_debug_v1 \
  --fps=30 \
  --num-episodes=10
```

Adjust CLI flags to the exact LeRobot version in this workspace.

## Debug dataset QA

```text
[ ] State shape is stable.
[ ] Action shape is stable.
[ ] Joint order is stable.
[ ] Joint signs are correct.
[ ] Hand scalar changes when the hand opens and closes.
[ ] Front camera sees cube, hand, and tray.
[ ] Wrist camera sees the grasp zone.
[ ] No sudden jumps in arm targets.
[ ] Failed episodes are marked or discarded.
```

## Train ACT baseline

```bash
lerobot-train \
  --dataset.repo_id=<HF_USER>/rpo5_ah_cube_tray_v1_100eps \
  --policy.type=act \
  --output_dir=outputs/train/act_rpo5_ah_cube_tray_v1 \
  --job_name=act_rpo5_ah_cube_tray_v1 \
  --policy.device=cuda \
  --wandb.enable=true \
  --policy.repo_id=<HF_USER>/act_rpo5_ah_cube_tray_v1
```

## Evaluation

Use the same setup as training:

```text
same cube
same tray
same table
same lighting
same camera poses
same arm mounting
```

Run 20 trials and record:

```text
success
failure reason
time to completion
operator intervention
collision or near-collision
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

## Done when

```text
[ ] 10 debug episodes pass QA.
[ ] 50-100 baseline episodes are recorded.
[ ] ACT training starts without feature-shape errors.
[ ] Policy can run on the real robot with emergency stop ready.
[ ] 20-trial evaluation result is documented.
```
