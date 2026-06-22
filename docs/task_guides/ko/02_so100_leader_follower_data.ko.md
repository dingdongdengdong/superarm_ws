# 02 - SO-100 리더/팔로워 데이터 워크플로

## 목표

LeRobot SO-100 리더/팔로워 구성을 낮은 위험의 proxy workflow로 사용합니다. 목적은
operator training, camera placement, dataset QA, 첫 ACT training sanity check입니다.

## 왜 먼저 하는가

SO-100은 LeRobot에 이미 통합되어 있습니다. 따라서 RoboParty + AmazingHand 하드웨어
통합이 끝나기 전에 C.S 팀이 collect-train-evaluate 루프를 검증할 수 있습니다.

단, SO-100 데이터는 proxy data입니다. RoboParty + AmazingHand 정책 학습에 섞기
전에는 action/observation mapping을 명시해야 합니다.

## 준비 체크

```text
[ ] SO-100 follower가 안전하게 켜집니다.
[ ] leader arm이 control computer에 연결됩니다.
[ ] motor voltage가 SO-100 hardware variant와 맞습니다.
[ ] camera 위치가 RoboParty에서 쓸 계획과 최대한 비슷합니다.
[ ] cube, tray, table height, lighting이 고정되어 있습니다.
```

## 명령 예시

포트 찾기:

```bash
lerobot-find-port
```

Follower와 leader calibration:

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=<FOLLOWER_PORT> --robot.id=so100_proxy_follower
lerobot-calibrate --teleop.type=so101_leader --teleop.port=<LEADER_PORT> --teleop.id=so100_proxy_leader
```

녹화 없이 teleoperation 확인:

```bash
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=<FOLLOWER_PORT> \
  --robot.id=so100_proxy_follower \
  --teleop.type=so101_leader \
  --teleop.port=<LEADER_PORT> \
  --teleop.id=so100_proxy_leader
```

Debug episode 10개 기록:

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

정확한 robot/teleop type 이름은 현재 LeRobot checkout에 맞게 확인합니다.

## Dataset QA

10 episode 이후 확인합니다.

```text
[ ] 모든 episode에서 cube, gripper, tray가 보입니다.
[ ] action stream이 부드럽게 변합니다.
[ ] 실패 demo는 표시하거나 제거했습니다.
[ ] camera frame이 뒤집히거나 과노출되지 않았습니다.
[ ] operator가 무리한 동작 없이 task를 반복 완료할 수 있습니다.
[ ] episode reset state가 일정합니다.
```

## SO-100 데이터를 RoboParty에 쓰기 전 확인

```text
[ ] joint count와 joint order를 mapping했습니다.
[ ] joint sign을 mapping했습니다.
[ ] joint limit을 mapping했습니다.
[ ] workspace scale 차이를 기록했습니다.
[ ] gripper action을 AmazingHand grasp scalar로 mapping했습니다.
[ ] camera viewpoint가 충분히 비슷합니다.
[ ] control frequency가 호환됩니다.
```

## 완료 조건

```text
[ ] SO-100 debug episode 10개를 기록했습니다.
[ ] dataset visual inspection을 통과했습니다.
[ ] SO-100 failure case 목록을 작성했습니다.
[ ] task fixture를 RoboParty에서도 재사용할 수 있습니다.
```
