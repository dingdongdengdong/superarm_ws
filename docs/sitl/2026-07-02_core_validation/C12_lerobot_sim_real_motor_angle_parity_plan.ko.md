# C12 - LeRobot Sim-Real Motor Angle Parity

## 목적

LeRobot custom control 경로에서 Isaac Sim 팔과 실제 팔/손 motor가 같은 의미의 angle target으로 움직이는지 확인한다.

이 task의 핵심은 "움직인다"가 아니라 **SITL joint 이름, 실제 motor ID, 부호, offset, tolerance가 서로 맞는지** 확인하는 것이다. 실제 motor motion은 C07/C08/C11이 통과한 뒤 tiny target으로만 진행한다.

## 1. 사람용

### 사람이 이해해야 하는 것

완료 후 학생은 아래 질문에 답할 수 있어야 한다.

```text
Q1. LeRobot action 6D 순서는 무엇인가?
Q2. `right_arm_pitch_joint.pos`가 실제 어떤 motor ID 후보로 가는가?
Q3. sign 후보가 틀리면 어떤 증상이 생기는가?
Q4. simulation에서 맞는 값이 실제 motor에서도 바로 안전하다고 말할 수 없는 이유는 무엇인가?
Q5. parity test에서 tolerance를 왜 숫자로 기록해야 하는가?
```

### 기준 contract

| Index | LeRobot feature | 실제 target 후보 | 상태 |
|---:|---|---|---|
| 0 | `right_arm_pitch_joint.pos` | motor 19 / `can3` / sign `-1` | pending |
| 1 | `right_arm_roll_joint.pos` | motor 20 / `can3` / sign `1` | pending |
| 2 | `right_arm_yaw_joint.pos` | motor 21 / `can3` / sign `1` | pending |
| 3 | `right_elbow_pitch_joint.pos` | motor 22 / `can3` / sign `-1` | pending |
| 4 | `right_elbow_yaw_joint.pos` | motor 23 / `can3` / sign `1` | pending |
| 5 | `amazinghand_grasp.pos` | AmazingHand serial scalar | pending |

### 사람이 직접 해야 하는 부분

```text
[ ] C07 mapping table을 읽고 motor ID/sign 후보를 확인한다.
[ ] C08 clamp test 결과를 확인한다.
[ ] C11 AmazingHandControl upstream repo 실행 결과를 확인한다.
[ ] 실제 motor power를 켜기 전에 parity target과 기록 양식을 사람이 확인한다.
[ ] 첫 실제 parity는 joint 하나씩, 0.02 rad 이하 tiny target으로만 한다.
[ ] 움직임 방향이 틀리면 즉시 중단하고 mapping table을 수정 대상으로 표시한다.
```

### AI에게 도움받을 수 있는 부분

AI는 실험의 주체가 아니라 보조 도구다. 사람이 C07/C08/C11 결과를 읽고 parity 방법을 정한 뒤, 필요할 때 아래 작업을 부탁할 수 있다.

```text
[ ] C07 mapping table과 현재 code의 6D action 순서가 일치하는지 확인한다.
[ ] target, SITL observed, real observed를 비교하는 작은 표나 script를 만들어준다.
[ ] sign mismatch가 의심되는 행을 표시해준다.
[ ] 산출물 markdown 초안을 정리한다.
[ ] 테스트 로그를 읽고 다음에 사람이 확인할 항목을 요약한다.
```

AI에게 보조 작업을 맡길 때는 아래 문서를 참고할 수 있다.

```text
docs/sitl/2026-07-02_core_validation/ai_agent/C12_lerobot_sim_real_motor_angle_parity_plan.ai.ko.md
```

### 진행 순서

1. 사람이 C07 mapping table과 C08 clamp 결과를 읽는다.
2. 사람이 C11 결과를 보고 hand를 parity 범위에 넣을지 정한다.
3. 사람이 아주 작은 target sequence를 정한다.
4. SITL follower에서 같은 target을 실행해 simulation observed 값을 기록한다.
5. 실제 hardware는 joint 하나씩 tiny target으로만 확인한다.
6. target, sim observed, real observed의 차이를 표로 기록한다.

### 기록 예시

처음에는 자동화보다 아래 표를 정확히 채우는 것이 더 중요하다.

| Step | Feature | Target | SITL observed | Real observed | 사람 판단 |
|---:|---|---:|---:|---:|---|
| 1 | `right_arm_pitch_joint.pos` | `0.02` |  |  |  |
| 2 | `right_arm_roll_joint.pos` | `0.02` |  |  |  |

필요하면 AI에게 이 표를 JSON/CSV로 바꾸거나 error 계산 script를 만들게 할 수 있다.

### 중단 기준

```text
[ ] C07/C08/C11 중 하나라도 blocked다.
[ ] dry-run target이 0.02 rad를 초과한다.
[ ] real observed가 target과 반대 부호로 움직인다.
[ ] real error가 tolerance를 초과한다.
[ ] hand scalar가 `[0.0, 1.0]` 범위를 벗어난다.
```

### 산출물 경로

```text
docs/sitl/2026-07-02_core_validation/artifacts/C12_sim_real_parity_<name>.md
```

### 보고서 템플릿

````markdown
# C12 LeRobot Sim-Real Motor Angle Parity - <name>

## 실행 정보

- Date:
- Operator:
- C07 artifact:
- C08 artifact:
- C11 artifact:

## Parity Result

| Feature | Target | SITL observed | Real observed | Error | Sign OK | Decision |
|---|---:|---:|---:|---:|---|---|
| `right_arm_pitch_joint.pos` |  |  |  |  | pass/fail | pass/fail/blocked |

## 판단

```text
[ ] D435i grasp dry-run으로 넘어가도 된다.
[ ] D435i grasp로 넘어가면 안 된다. 이유:
```
````

### 승인 기준

```text
[ ] 6D feature 순서가 C03/C07과 일치한다.
[ ] dry-run evidence JSON이 있다.
[ ] SITL observed 값이 기록되어 있다.
[ ] 실제 motor를 움직였다면 joint별 tiny motion evidence가 있다.
[ ] sign/tolerance 문제가 있으면 pass로 처리하지 않았다.
```
