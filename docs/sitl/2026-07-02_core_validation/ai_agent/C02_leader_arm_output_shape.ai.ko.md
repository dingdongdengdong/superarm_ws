# AI C02 - Leader Arm Output Shape 확인

## 목표

leader arm output의 length, order, range, gripper/hand scalar 여부를 evidence로 남긴다. C03 mapping의 전제 자료를 만든다.

## 읽을 파일

```text
docs/task_guides/ko/02_so100_leader_follower_data.ko.md
docs/sitl/2026-06-27/team_tiny_tasks_sitl.md
isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py
```

## 실행 후보

```bash
lerobot-find-port
```

현재 LeRobot checkout에서 leader type 이름을 확인한 뒤 calibration한다. SO-101 계열이면 아래 형식을 기준으로 한다.

```bash
lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=<LEADER_PORT> \
  --teleop.id=roboto_v2_sitl_leader
```

raw output을 읽는 repo-local script가 없으면 blocker로 기록한다. 이 blocker가 있으면 C03 mapping을 확정하지 않는다.

```text
blocker: leader raw output logging script is missing
needed: script or command that prints teleop vector without commanding follower hardware
```

## 산출물 경로

```text
docs/sitl/2026-07-02_core_validation/artifacts/C02_leader_raw_output_<name>.md
```

## 산출물 필수 표

| 항목 | 내용 |
|---|---|
| Date | 실행 날짜 |
| Leader port | `<LEADER_PORT>` |
| Teleop type | `<teleop.type>` |
| Calibration id | `roboto_v2_sitl_leader` |
| Output length | 숫자 |
| Frequency | Hz 또는 unknown |
| Has gripper/hand scalar | yes/no/unknown |
| Values are smooth | yes/no |

## Raw sample 필수 pose

```text
home
reach
elbow_fold
side
```

## 완료 기준

```text
[ ] leader port가 기록되어 있다.
[ ] leader type이 기록되어 있다.
[ ] raw output length가 기록되어 있다.
[ ] 최소 4개 pose sample이 있다.
[ ] C03에서 쓸 mapping 후보가 있다.
[ ] raw output logger가 없으면 blocker가 명확히 기록되어 있다.
```
