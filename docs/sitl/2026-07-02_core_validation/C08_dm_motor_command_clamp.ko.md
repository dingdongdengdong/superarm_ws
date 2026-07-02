# C08 - DM Motor Command Clamp 구현/검증

## 목적

실제 DM4340P motor command로 넘어가기 전에 6D action이 안전 범위를 벗어나지 않도록 clamp 정책과 test 기준을 문서화한다.

이 task는 실제 motor를 움직이지 않는다. 학생이 배워야 하는 핵심은 clamp가 "값을 예쁘게 만드는 기능"이 아니라 hardware damage를 막는 마지막 software gate라는 점이다.

## 1. 사람용

### 학습 목표

```text
[ ] clamp, limit, saturation의 차이를 설명할 수 있다.
[ ] URDF limit과 hardware safe limit을 분리해 기록할 수 있다.
[ ] 잘못된 action 길이, 큰 값, NaN/inf, hand scalar 범위 오류를 test case로 만들 수 있다.
[ ] clamp test가 통과해도 실제 motor tiny motion 승인은 별도라는 점을 설명할 수 있다.
```

### 왜 필요한가

SITL에서는 잘못된 target을 보내도 simulation만 이상해진다. 실제 DM4340P에서는 과도한 target, sign 오류, zero offset 오류가 mechanical collision이나 over-current로 이어질 수 있다. 따라서 C08은 hardware command path에 들어가기 전에 action을 보수적으로 제한하는 정책을 확정한다.

### Clamp 정책

| 입력 종류 | 정책 |
|---|---|
| action 길이가 6보다 짧음 | 부족한 값은 safe default `0.0`으로 padding |
| action 길이가 6보다 김 | 6D contract 밖 값은 사용하지 않음 |
| arm joint 값 | C07에서 정한 safe lower/upper 안으로 clamp |
| `amazinghand_grasp.pos` | `[0.0, 1.0]` 안으로 clamp |
| NaN/inf | command 금지, fail |
| hardware status pending | 실제 motor command 금지 |

초기 safe limit은 URDF limit보다 더 보수적으로 잡는다. 실제 tiny motion 전에는 `0.005-0.02 rad` 수준의 relative target만 허용하는 정책을 별도 hardware bringup에서 검토한다.

### 최소 test case

| Case | Input | 기대 결과 |
|---|---|---|
| 정상 6D | `[0, 0, 0, 0, 0, 0]` | 그대로 통과 |
| 짧은 action | `[0.1]` | `[0.1, 0, 0, 0, 0, 0]` |
| 긴 action | `[0, 1, 2, 3, 4, 0.5, 99]` | 앞 6개만 사용 |
| 큰 arm 값 | `[9, -9, 9, -9, 9, 0.5]` | 각 joint limit으로 clamp |
| hand 값 초과 | `[0, 0, 0, 0, 0, 2]` | 마지막 값 `1.0` |
| hand 값 미만 | `[0, 0, 0, 0, 0, -1]` | 마지막 값 `0.0` |
| NaN/inf | `[nan, 0, 0, 0, 0, 0]` | fail, publish 금지 |

### 학생 실습 절차

1. C07 mapping table을 읽고 safe limit 후보를 확인한다.
2. 기존 wrapper가 action padding/truncation/hand clamp를 어디서 하는지 찾는다.
3. hardware command가 없는 pure function 또는 mock test로 clamp를 검증한다.
4. test 결과를 C08 artifact에 기록한다.
5. 실제 motor command path에 연결하기 전까지는 `hardware_commanded=false`를 유지한다.

### 산출물 경로

```text
docs/sitl/2026-07-02_core_validation/artifacts/C08_dm_motor_command_clamp_<name>.md
```

### 보고서 템플릿

````markdown
# C08 DM Motor Command Clamp - <name>

## Learning Summary

- clamp가 필요한 이유:
- URDF limit과 hardware safe limit을 분리해야 하는 이유:
- 실제 motor command를 보내지 않은 근거:

## Clamp Policy

| Feature | Lower | Upper | Source | Hardware status |
|---|---:|---:|---|---|
| `right_arm_pitch_joint.pos` | `<value>` | `<value>` | C07 | pending |

## Test Results

| Case | Input | Expected | Result |
|---|---|---|---|
| short action padding | `[0.1]` | 6D padded vector | pass/fail |

## Decision

```text
[ ] C09 dataset schema check로 넘어가도 된다.
[ ] C09로 넘어가면 안 된다. 이유:
```
````

### 승인 기준

```text
[ ] C07 mapping table을 근거로 clamp 기준을 작성했다.
[ ] short/long/out-of-range/hand scalar/NaN test case가 있다.
[ ] test는 hardware 없이 실행되었다.
[ ] 실제 DM4340P command를 보내지 않았다.
[ ] 실패 시 publish 금지 조건이 명시되어 있다.
```

## 2. AI Agent 내부 문서

AI agent에게 위임할 때는 아래 내부 실행 문서를 직접 전달한다.

```text
docs/sitl/2026-07-02_core_validation/ai_agent/C08_dm_motor_command_clamp.ai.ko.md
```
