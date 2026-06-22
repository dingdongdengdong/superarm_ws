# 03 - RoboParty Arm Bring-Up

## 목표

AmazingHand를 연결하거나 LeRobot policy control을 실행하기 전에 RoboParty 5-DOF arm
하나를 안전하게 bring-up합니다.

## 기본 가정

하드웨어 팀이 다르게 정하지 않았다면 right arm부터 시작합니다.

```text
right arm candidate: can3, motor IDs 19-23
left arm candidate:  can2, motor IDs 14-18
```

이 ID는 RoboParty config에서 추론한 값이므로 실제 로봇에서 반드시 검증해야 합니다.

## 안전 준비

```text
[ ] physical emergency stop이 바로 닿는 위치에 있습니다.
[ ] power supply current limit이 설정되어 있습니다.
[ ] 첫 테스트 동안 arm이 기계적으로 지지되어 있습니다.
[ ] 첫 motor test에는 hand나 payload를 붙이지 않습니다.
[ ] CAN adapter가 안정적인 USB port에 연결되어 있습니다.
[ ] 한 명은 robot을 보고, 다른 한 명은 command를 보냅니다.
```

## Bring-up 순서

1. 하드웨어 버전 확인:

```text
V1.0 or V2.0
```

2. CAN interface 확인:

```bash
ip link show
```

3. 후보 CAN interface bring-up:

```bash
sudo ip link set can3 up type can bitrate 1000000
```

bitrate는 RoboParty deployment config에 맞춥니다.

4. RoboParty의 기존 zeroing 또는 motor inspection script를 먼저 read-only 또는
minimal-motion 모드로 실행합니다.

5. 한 motor에 아주 작은 command를 보냅니다.

```text
target delta: 0.5 to 1.0 degree
speed: low
load: no payload
```

6. 실제 sign을 기록합니다.

```text
rpo_arm_j1: +1 or -1
rpo_arm_j2: +1 or -1
rpo_arm_j3: +1 or -1
rpo_arm_j4: +1 or -1
rpo_arm_j5: +1 or -1
```

7. 다섯 joint를 각각 테스트합니다.

8. 다섯 joint를 모두 사용해 느린 neutral-pose motion을 테스트합니다.

## Bring-up 로그 형식

생성 파일:

```text
docs/task_guides/roboparty_arm_bringup_log.md
```

테이블 형식:

```markdown
| Date | Arm side | CAN | Motor ID | Joint | Direction OK | Limit OK | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 2026-06-22 | right | can3 | 19 | rpo_arm_j1 | yes/no | yes/no | note |
```

## 완료 조건

```text
[ ] 올바른 CAN interface를 확인했습니다.
[ ] 올바른 5개 motor ID를 확인했습니다.
[ ] joint sign을 기록했습니다.
[ ] joint soft limit을 기록했습니다.
[ ] arm-only zeroing 절차가 동작합니다.
[ ] 5-joint tiny motion이 예상 밖 움직임 없이 실행됩니다.
```
