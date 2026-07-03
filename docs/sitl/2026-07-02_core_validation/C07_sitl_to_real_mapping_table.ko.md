# C07 - SITL-to-real Mapping Table

## 목적

C01-C06에서 확인한 SITL 6D contract를 실제 Robotov2.0 right arm DM4340P motor 후보와 연결하는 mapping table을 작성한다.

이 task는 실제 motor를 움직이지 않는다. 학생이 배워야 하는 핵심은 SITL joint 이름과 실제 motor ID/CAN/sign 후보를 한 표에 묶되, "확정값"과 "검증 전 후보값"을 구분하는 것이다.

## 1. 사람용

### 학습 목표

```text
[ ] SITL joint 이름과 실제 motor ID 후보를 같은 표에 정리할 수 있다.
[ ] URDF/SITL limit과 실제 hardware safe limit이 다르다는 점을 설명할 수 있다.
[ ] sign 후보가 왜 tiny motion 전에는 확정값이 아닌지 설명할 수 있다.
[ ] C07 산출물이 C08 clamp와 E04-E06 hardware bringup의 입력이 된다는 점을 설명할 수 있다.
```

### 왜 필요한가

SITL에서는 `right_arm_pitch_joint.pos` 같은 이름으로 joint를 다룬다. 실제 hardware에서는 CAN bus, motor ID, sign, zero offset, safe relative target이 필요하다. 두 세계를 바로 연결하면 위험하므로 C07에서 사람이 검토할 mapping table을 먼저 만든다.

| 구분 | SITL | 실제 hardware |
|---|---|---|
| joint 식별 | joint name | CAN bus + motor ID |
| 방향 | URDF axis/sign | 실제 조립 방향 + motor encoder sign |
| limit | URDF lower/upper | 보수적 safe limit, tiny motion으로 측정 |
| hand | `amazinghand_grasp.pos` scalar | serial adapter + 8 servo target 후보 |

### 시작 기준

```text
[ ] C03 mapping 문서가 있다.
[ ] C04 URDF joint/limit audit이 있다.
[ ] C05 no-op evidence가 있다.
[ ] C06 sweep evidence가 있다.
[ ] 실제 motor power를 켜지 않는다.
```

### 기본 mapping 후보

아래 값은 시작 후보이다. 실제 robot wiring에서 read-only scan과 tiny motion으로 다시 확인해야 한다.

| SITL feature | Real target 후보 | CAN/port 후보 | Sign 후보 | 검증 상태 |
|---|---:|---|---:|---|
| `right_arm_pitch_joint.pos` | motor ID 19 | `can3` | -1 | pending |
| `right_arm_roll_joint.pos` | motor ID 20 | `can3` | 1 | pending |
| `right_arm_yaw_joint.pos` | motor ID 21 | `can3` | 1 | pending |
| `right_elbow_pitch_joint.pos` | motor ID 22 | `can3` | -1 | pending |
| `right_elbow_yaw_joint.pos` | motor ID 23 | `can3` | 1 | pending |
| `amazinghand_grasp.pos` | hand serial adapter | serial | scalar | pending |

### 작성 절차

1. C03의 6D output order를 복사한다.
2. C04의 URDF lower/upper limit을 붙인다.
3. 상위 계획서의 motor ID/CAN/sign 후보를 붙인다.
4. 각 행에 `source`, `confidence`, `verification_method`, `status`를 추가한다.
5. 실제 hardware로 확인되지 않은 값은 `pending`으로 둔다.
6. 다음 단계에서 쓸 C08 clamp 입력값을 별도 열로 표시한다.

### 산출물 경로

```text
docs/sitl/2026-07-02_core_validation/artifacts/C07_hardware_parity_checklist_<name>.md
```

### 산출물 템플릿

````markdown
# C07 Hardware Parity Checklist - <name>

## Source Evidence

| Source | Path | Used for |
|---|---|---|
| C03 mapping | `<path>` | 6D order |
| C04 URDF audit | `<path>` | SITL lower/upper |
| C05 no-op | `<path>` | no-op safety |
| C06 sweep | `<path>` | SITL motion sanity |

## Mapping Table

| SITL feature | URDF lower | URDF upper | Real target candidate | CAN/port candidate | Sign candidate | Hardware status | Next verification |
|---|---:|---:|---|---|---:|---|---|
| `right_arm_pitch_joint.pos` | -1.57 | 1.57 | `19` | `can3` | -1 | pending | read-only ID scan |
| `right_arm_roll_joint.pos` | -1.0 | 0.25 | `20` | `can3` | 1 | pending | read-only ID scan |
| `right_arm_yaw_joint.pos` | -1.57 | 1.57 | `21` | `can3` | 1 | pending | read-only ID scan |
| `right_elbow_pitch_joint.pos` | -0.6 | 1.57 | `22` | `can3` | -1 | pending | read-only ID scan |
| `right_elbow_yaw_joint.pos` | -1.57 | 1.57 | `23` | `can3` | 1 | pending | read-only ID scan |
| `amazinghand_grasp.pos` | 0.0 | 1.0 | serial adapter | serial | scalar | pending | servo map check |

## Safety Decision

```text
[ ] C08 clamp 설계로 넘어가도 된다.
[ ] C08로 넘어가면 안 된다. 이유:
```
````

### 승인 기준

```text
[ ] 6D feature가 모두 mapping table에 있다.
[ ] 각 행에 SITL limit과 real target 후보가 있다.
[ ] 후보값과 확정값이 구분되어 있다.
[ ] 실제 motor command 없이 작성되었다.
[ ] 다음 검증 방법이 read-only scan 또는 tiny motion으로 분리되어 있다.
```

## 2. AI Agent 내부 문서

AI agent에게 위임할 때는 아래 내부 실행 문서를 직접 전달한다.

```text
docs/sitl/2026-07-02_core_validation/ai_agent/C07_sitl_to_real_mapping_table.ai.ko.md
```
