# C03 - Leader Output을 6D SITL Action으로 변환

## 목적

C02에서 확인한 leader arm output을 Robotov2.0 right arm + AmazingHand 6D SITL contract로 변환하는 정책을 고정한다.

이 task의 출력은 먼저 Isaac Sim SITL follower에만 연결한다. 실제 DM4340P 모터에는 연결하지 않는다.

## 1. 사람용

### 학습 목표

이 task의 학습 목표는 leader arm raw output을 Robotov2.0 right arm + AmazingHand SITL이 이해할 수 있는 6D action으로 바꾸는 규칙을 명확히 만드는 것입니다. mapping은 단순히 index를 복사하는 일이 아닙니다. joint 순서, 부호, scale, clamp, hand scalar 정책을 함께 정해야 합니다.

완료 후 학생은 다음 질문에 답할 수 있어야 합니다.

```text
Q1. C02 raw output 중 어떤 index를 6D action에 사용할 것인가?
Q2. 각 index는 어떤 SITL joint key로 들어가는가?
Q3. leader 값의 부호가 SITL joint 회전 방향과 맞는가?
Q4. URDF limit 밖 값은 어떻게 clamp되는가?
Q5. hand는 왜 AmazingHand raw 8-servo가 아니라 scalar 하나로 시작하는가?
```

### 선수 지식

| 개념 | 학생이 알아야 할 수준 |
|---|---|
| Joint order | 같은 숫자 6개라도 순서가 바뀌면 완전히 다른 동작이 된다는 점 |
| Sign | leader를 앞으로 움직였을 때 SITL joint가 반대로 움직일 수 있다는 점 |
| Scale | leader raw 값 범위와 URDF radian limit 범위가 다를 수 있다는 점 |
| Clamp | limit 밖 target을 안전 범위로 자르는 처리 |
| Contract | action key와 observation key의 길이, 이름, 단위에 대한 약속 |

### 확인할 내용

- leader arm output과 Robotov2.0 right arm joint 의미가 다를 수 있다.
- mapping은 항상 `joint order`, `sign`, `scale`, `clamp`를 함께 정의해야 한다.
- hand는 처음부터 AmazingHand 8-servo raw control이 아니라 `amazinghand_grasp.pos` scalar로만 보낸다.
- no-op action과 tiny action이 SITL에서 먼저 통과해야 한다.
- 검증/정책 replay 경로와 live leader teleop/recording 경로를 구분해야 한다.

### 경로 구분

| 목적 | 사용 경로 |
|---|---|
| verifier / policy replay / deterministic command 검증 | `IsaacSimRpoArmRobot.send_action()` |
| live leader teleop / recording | `/leader/joint_commands` 입력 + `IsaacSimRpoArmRobot.teleop_step()` |

### 기본 mapping 가정

| Leader output | SITL action key | 기본 처리 |
|---|---|---|
| output[0] | `right_arm_pitch_joint.pos` | scale/sign 후 URDF limit clamp |
| output[1] | `right_arm_roll_joint.pos` | scale/sign 후 URDF limit clamp |
| output[2] | `right_arm_yaw_joint.pos` | scale/sign 후 URDF limit clamp |
| output[3] | `right_elbow_pitch_joint.pos` | scale/sign 후 URDF limit clamp |
| output[4] | `right_elbow_yaw_joint.pos` | scale/sign 후 URDF limit clamp |
| output[5] or gripper | `amazinghand_grasp.pos` | `[0.0, 1.0]` clamp |

이 표는 시작 가정입니다. C02 raw sample과 C04 URDF limit 근거가 없으면 확정 mapping으로 승인하지 않습니다.

### 진행 절차

1. C02 산출물을 확인한다.

```bash
ls docs/sitl/2026-07-02_core_validation/artifacts/C02_*.md
```

기대 결과: C02 raw output 산출물이 있다. 없으면 C03은 blocked입니다.

2. 6D action key 순서를 확인한다.

```bash
rg -n "right_arm_pitch_joint|right_arm_roll_joint|right_arm_yaw_joint|right_elbow_pitch_joint|right_elbow_yaw_joint|amazinghand_grasp" isaacsim_test/lerobot isaacsim_test/*.json docs/sitl/2026-07-02_core_validation
```

기대 결과: C03/C04와 wrapper/config의 6D key가 같은 순서로 정리된다.

3. mapping table을 작성한다.

```markdown
| SITL index | SITL action key | Leader source | Sign | Scale | Clamp lower | Clamp upper | 근거 |
|---:|---|---|---:|---:|---:|---:|---|
| 0 | `right_arm_pitch_joint.pos` | `output[0]` | `+1` | `<value>` | -1.57 | 1.57 | C02 pose / C04 URDF |
| 1 | `right_arm_roll_joint.pos` | `output[1]` | `+1` | `<value>` | -1.0 | 0.25 | C02 pose / C04 URDF |
| 2 | `right_arm_yaw_joint.pos` | `output[2]` | `+1` | `<value>` | -1.57 | 1.57 | C02 pose / C04 URDF |
| 3 | `right_elbow_pitch_joint.pos` | `output[3]` | `+1` | `<value>` | -0.6 | 1.57 | C02 pose / C04 URDF |
| 4 | `right_elbow_yaw_joint.pos` | `output[4]` | `+1` | `<value>` | -1.57 | 1.57 | C02 pose / C04 URDF |
| 5 | `amazinghand_grasp.pos` | `output[5]` or constant | `+1` | `<value>` | 0.0 | 1.0 | C02 gripper evidence |
```

모르는 값은 추정으로 채우지 말고 `blocked` 또는 `needs C02 evidence`로 적습니다.

4. no-op example을 작성한다.

```text
raw leader input: <C02 home sample>
normalized 6D action: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
expected SITL behavior: no visible movement, no hand close command
```

5. tiny action example을 작성한다.

```text
raw leader input: <C02 tiny movement sample>
normalized 6D action: [small_pitch, small_roll, small_yaw, small_elbow_pitch, small_elbow_yaw, hand_scalar]
expected SITL behavior: only intended joint direction changes, values stay inside C04 limits
```

6. path separation 문구를 산출물에 명시한다.

```text
verifier/policy replay 검증은 `IsaacSimRpoArmRobot.send_action()` 경로를 사용한다.
live leader teleop/recording 검증은 `/leader/joint_commands` 입력과 `IsaacSimRpoArmRobot.teleop_step()` 경로를 사용한다.
```

### 흔한 실수와 병목

- leader output index와 Robotov2.0 joint 이름이 우연히 비슷해 보여도 1:1 대응이라고 가정하지 않습니다.
- scale 없이 raw 값을 radian target으로 넣으면 limit을 초과할 수 있습니다.
- hand를 처음부터 AmazingHand raw 8-servo로 열면 action space가 커져서 SITL 검증과 dataset 수집이 복잡해집니다.
- C03은 실제 동작 검증이 아니라 mapping 정책 확정입니다. 실제 no-op/tiny 동작은 C05/C06에서 봅니다.

### 사람이 승인할 기준

```text
[ ] no-op leader pose가 no-op SITL action으로 변환된다.
[ ] tiny leader movement가 예상 joint의 tiny SITL movement로 변환된다.
[ ] hand 값은 항상 `[0.0, 1.0]` 안에 있다.
[ ] limit 밖 값은 clamp된다.
[ ] `send_action()` 검증 경로와 `/leader/joint_commands` live teleop 경로가 문서에서 분리되어 있다.
[ ] 변환 결과를 실제 DM4340P가 아니라 SITL follower에만 보낸다.
```

### 보고서 템플릿

완료 후 아래 형식으로 `docs/sitl/2026-07-02_core_validation/artifacts/C03_leader_to_sitl_mapping_<name>.md`를 작성한다.

````markdown
# C03 Leader to SITL 6D Mapping - <name>

## Learning Summary

- mapping에서 가장 중요한 결정:
- sign/scale/clamp를 분리해야 하는 이유:
- hand를 scalar로 유지하는 이유:

## Inputs

| 항목 | 경로 |
|---|---|
| C02 raw output artifact |  |
| C04 URDF audit artifact |  |

## Mapping Table

| SITL index | SITL action key | Leader source | Sign | Scale | Clamp | 근거 |
|---:|---|---|---:|---:|---|---|
| 0 | `right_arm_pitch_joint.pos` |  |  |  |  |  |
| 1 | `right_arm_roll_joint.pos` |  |  |  |  |  |
| 2 | `right_arm_yaw_joint.pos` |  |  |  |  |  |
| 3 | `right_elbow_pitch_joint.pos` |  |  |  |  |  |
| 4 | `right_elbow_yaw_joint.pos` |  |  |  |  |  |
| 5 | `amazinghand_grasp.pos` |  |  |  | `[0.0, 1.0]` |  |

## Decision

```text
[ ] C05 no-op 검증으로 넘어가도 된다.
[ ] C05로 넘어가면 안 된다. 이유:
```
````

## 2. AI Agent 내부 문서

AI agent에게 위임할 때는 아래 내부 문서를 사용한다.

```text
docs/sitl/2026-07-02_core_validation/ai_agent/C03_leader_to_sitl_6d_action.ai.ko.md
```
