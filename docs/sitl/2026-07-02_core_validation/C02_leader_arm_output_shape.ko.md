# C02 - Leader Arm Output Shape 확인

## 목적

LeRobot leader arm이 실제로 몇 개의 값을 어떤 순서와 범위로 내보내는지 확인한다. 이 결과가 있어야 C03에서 6D SITL action mapping을 안전하게 만들 수 있다.

이 task는 leader arm만 읽는다. Robotov2.0 right arm이나 DM4340P 모터에는 command를 보내지 않는다.

## 1. 사람용

### 학습 목표

이 task의 학습 목표는 leader arm을 “움직이면 값이 나온다” 수준이 아니라, 그 값의 길이, 순서, 범위, 의미를 evidence로 남기는 것입니다. C03 mapping은 C02 결과를 근거로만 만들 수 있습니다. C02가 부정확하면 이후 SITL에서는 팔이 반대로 움직이거나, hand 값이 팔 joint로 들어가거나, limit 밖 command가 만들어질 수 있습니다.

완료 후 학생은 다음 질문에 답할 수 있어야 합니다.

```text
Q1. 현재 leader arm은 어떤 port와 teleop type으로 인식되는가?
Q2. raw output은 몇 차원인가?
Q3. 각 output index는 어떤 pose에서 어떻게 변하는가?
Q4. gripper 또는 hand scalar로 쓸 수 있는 값이 있는가?
Q5. C03 mapping을 확정하기에 충분한 증거가 있는가?
```

### 선수 지식

| 개념 | 학생이 알아야 할 수준 |
|---|---|
| Serial port | USB 장치가 `/dev/tty*` 같은 port로 잡힐 수 있다는 정도 |
| Calibration | leader arm의 zero/reference를 software가 기억해야 한다는 정도 |
| Vector shape | `[x0, x1, ...]`의 길이와 index 순서가 중요하다는 정도 |
| Signal smoothness | 팔을 천천히 움직일 때 값이 갑자기 튀지 않아야 한다는 정도 |

### 확인할 내용

- leader arm port가 무엇인지 확인한다.
- leader arm type이 현재 LeRobot checkout에서 어떤 이름인지 확인한다.
- leader arm calibration이 완료되었는지 확인한다.
- leader arm raw output length가 몇 개인지 확인한다.
- gripper 또는 hand scalar에 해당하는 값이 있는지 확인한다.
- leader arm을 home, reach, fold, side pose로 움직였을 때 값이 부드럽게 변하는지 확인한다.

### 진행 절차

1. leader arm 연결 전 안전 조건을 확인한다.

```text
[ ] Robotov2.0 right arm DM4340P motor power를 켜지 않았다.
[ ] CAN bus motor command를 실행하지 않는다.
[ ] 이 task는 leader arm read-only 확인이다.
[ ] follower hardware를 연결하지 않는다.
```

2. leader port를 탐색한다.

```bash
lerobot-find-port
```

기대 결과: leader arm으로 보이는 port가 출력된다. port가 여러 개면 USB를 뽑았다 꽂으며 어떤 port가 사라지고 생기는지 확인합니다.

3. teleop type 후보를 확인한다.

```bash
rg -n "so101|leader|teleop" lerobot docs isaacsim_test | head -80
```

기대 결과: calibration 또는 teleop 예시에서 사용할 `teleop.type` 후보를 찾는다.

4. calibration 상태를 확인한다.

SO-101 계열일 때의 후보 명령은 아래 형식입니다. 실제 type과 port는 Step 2-3 결과로 바꿉니다.

```bash
lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=<LEADER_PORT> \
  --teleop.id=roboto_v2_sitl_leader
```

기대 결과: calibration이 정상 종료되고 calibration id가 기록된다. 이 명령은 leader만 대상으로 해야 하며 follower hardware 인자를 넣지 않습니다.

5. raw output logger 유무를 확인한다.

```bash
rg -n "raw output|teleop.*print|leader.*sample|capture_observation|record" docs isaacsim_test lerobot | head -120
```

기대 결과: leader raw vector를 follower command 없이 출력하는 방법을 찾는다. 없으면 이 task는 `blocked: leader raw output logger missing`으로 기록하고 C03 mapping 확정을 중단합니다.

6. pose별 raw sample을 기록한다.

| Pose | 자세 설명 | 기록 목적 |
|---|---|---|
| `home` | 팔을 자연스러운 기준 위치에 둔다. | zero/reference 확인 |
| `reach` | 팔을 앞으로 뻗는다. | shoulder/elbow 전방 움직임 확인 |
| `elbow_fold` | 팔꿈치를 접는다. | elbow 관련 index 확인 |
| `side` | 팔을 옆으로 움직인다. | yaw/roll 관련 index 확인 |

각 pose에서 최소 3회 sample을 기록합니다. 값이 크게 흔들리면 평균을 쓰기 전에 원인을 먼저 확인합니다.

### 산출물에 적을 표

```markdown
| Pose | Sample 1 | Sample 2 | Sample 3 | 관찰 |
|---|---|---|---|---|
| home | `[ ... ]` | `[ ... ]` | `[ ... ]` | 기준 자세 |
| reach | `[ ... ]` | `[ ... ]` | `[ ... ]` | output[?] 증가 |
| elbow_fold | `[ ... ]` | `[ ... ]` | `[ ... ]` | output[?] 변화 큼 |
| side | `[ ... ]` | `[ ... ]` | `[ ... ]` | output[?] 부호 확인 필요 |
```

```markdown
| Output index | 추정 의미 | 관찰 min | 관찰 max | C03 사용 여부 | 비고 |
|---:|---|---:|---:|---|---|
| 0 | unknown |  |  | yes/no |  |
| 1 | unknown |  |  | yes/no |  |
```

### 사람이 승인할 기준

```text
[ ] leader arm output length를 알고 있다.
[ ] leader arm output order를 알고 있다.
[ ] 각 output 값의 대략적인 min/max를 알고 있다.
[ ] 5 arm joint + 1 hand scalar로 변환 가능한지 판단했다.
[ ] 부족하거나 남는 축에 대한 drop/scale/map 정책을 정했다.
[ ] leader raw output logger가 없으면 C03 전에 먼저 구현해야 한다는 blocker를 인정했다.
```

### 주의

- leader arm이 정상으로 보여도 실제 DM4340P에 바로 연결하지 않는다.
- C02 결과는 raw evidence다. control policy가 아니다.
- leader arm과 Robotov2.0 right arm의 joint 의미가 1:1로 맞는다고 가정하지 않는다.
- raw output logger가 없으면 대충 mapping하지 않는다. logger 구현 또는 repo-local command 확인이 먼저다.
- 값이 튀는 index는 손으로 평균내서 숨기지 않는다. pose별 sample 표에 그대로 남기고 원인을 따로 적는다.

### 보고서 템플릿

완료 후 아래 형식으로 `docs/sitl/2026-07-02_core_validation/artifacts/C02_leader_raw_output_<name>.md`를 작성한다.

````markdown
# C02 Leader Raw Output Shape - <name>

## Learning Summary

- leader output shape:
- 가장 헷갈린 index:
- C03 mapping에 바로 쓸 수 있는 값:
- blocker:

## Device Info

| 항목 | 값 |
|---|---|
| Leader port |  |
| Teleop type |  |
| Calibration id | `roboto_v2_sitl_leader` |
| Output length |  |
| Has hand scalar | yes/no/unknown |

## Raw Samples

| Pose | Vector | 관찰 |
|---|---|---|
| home |  |  |
| reach |  |  |
| elbow_fold |  |  |
| side |  |  |

## Decision

```text
[ ] C03 mapping으로 넘어가도 된다.
[ ] C03 mapping으로 넘어가면 안 된다. 이유:
```
````

## 2. AI Agent 내부 문서

AI agent에게 위임할 때는 아래 내부 문서를 사용한다.

```text
docs/sitl/2026-07-02_core_validation/ai_agent/C02_leader_arm_output_shape.ai.ko.md
```
