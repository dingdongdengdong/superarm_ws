# 06 - 데이터셋, 학습, 평가

## 목표

작은 RoboParty + AmazingHand dataset을 기록하고, 첫 ACT baseline을 학습한 뒤, 같은
물리 setup에서 평가합니다.

## 첫 task

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

## Debug episode 기록

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

XR teleop을 사용할 경우:

```bash
lerobot-record \
  --robot.type=roboparty_5dof_arm_amazinghand_follower \
  --teleop.type=roboparty_xr_5dof \
  --repo-id=<HF_USER>/rpo5_ah_xr_cube_tray_debug_v1 \
  --fps=30 \
  --num-episodes=10
```

CLI flag는 현재 LeRobot version에 맞게 조정합니다.

## Debug dataset QA

```text
[ ] State shape이 stable합니다.
[ ] Action shape이 stable합니다.
[ ] Joint order가 stable합니다.
[ ] Joint sign이 올바릅니다.
[ ] Hand scalar가 hand open/close에 따라 변합니다.
[ ] Front camera가 cube, hand, tray를 봅니다.
[ ] Wrist camera가 grasp zone을 봅니다.
[ ] Arm target에 갑작스러운 jump가 없습니다.
[ ] 실패 episode는 표시하거나 제거했습니다.
```

## ACT baseline 학습

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

Training과 같은 setup을 사용합니다.

```text
same cube
same tray
same table
same lighting
same camera poses
same arm mounting
```

20 trial을 실행하고 기록합니다.

```text
success
failure reason
time to completion
operator intervention
collision or near-collision
```

Failure label:

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

## 완료 조건

```text
[ ] 10 debug episodes가 QA를 통과했습니다.
[ ] 50-100 baseline episodes를 기록했습니다.
[ ] ACT training이 feature-shape error 없이 시작됩니다.
[ ] emergency stop 준비 상태에서 real robot policy를 실행할 수 있습니다.
[ ] 20-trial evaluation result를 문서화했습니다.
```
