# C04 - Robotov2.0 + AmazingHand URDF Joint/Limit Audit

## 목적

`roboto_v2_right_arm_amazinghand_full.urdf`에서 Robotov2.0 right arm 5개 joint의 parent/child/axis/limit을 추출해 C03 clamp와 SITL sweep 기준으로 사용한다.

URDF limit은 SITL 검증 기준이다. 실제 DM4340P safe limit은 C05 이후 실제 hardware parity 단계에서 따로 확인해야 한다.

## 1. 사람용

### 학습 목표

이 task의 학습 목표는 URDF를 시뮬레이션 파일로만 보지 않고, C03 clamp와 C06 sweep target의 근거 자료로 읽는 것입니다. URDF에는 joint 이름, 회전축, parent/child link, lower/upper limit, effort, velocity가 들어 있습니다. 이 값이 틀리거나 C03과 다르면 SITL에서는 통과해도 실제 hardware parity 단계에서 위험해질 수 있습니다.

완료 후 학생은 다음 질문에 답할 수 있어야 합니다.

```text
Q1. Robotov2.0 right arm에서 C03이 쓰는 5개 joint가 URDF에 모두 있는가?
Q2. 각 joint의 axis는 어떤 방향인가?
Q3. lower/upper limit은 C03 clamp와 일치하는가?
Q4. AmazingHand는 왜 초기 contract에서 scalar 하나로 추상화하는가?
Q5. URDF limit과 실제 DM4340P safe limit은 왜 같은 것이 아닌가?
```

### 선수 지식

| 개념 | 학생이 알아야 할 수준 |
|---|---|
| URDF joint | link와 link를 연결하고 회전축/limit을 가진다는 정도 |
| Parent/child link | joint가 어떤 link 사이에 있는지 나타낸다는 정도 |
| Axis | joint 회전 방향을 나타내는 3D vector |
| Limit | 시뮬레이션상 허용되는 joint target 범위 |

### 확인할 URDF

```text
isaacsim_test/outputs/simready/echo_full/sitl/roboto_v2_right_arm_amazinghand_full.urdf
```

### 사람이 확인할 내용

- right arm 5개 joint가 URDF에 존재한다.
- 각 joint의 axis와 limit이 표로 추출되어 있다.
- AmazingHand에는 많은 motor/passive joint가 있지만, 초기 LeRobot contract는 `amazinghand_grasp.pos` scalar 하나로 유지한다.
- URDF limit을 실제 motor safe limit으로 오해하지 않는다.

### 진행 절차

1. URDF 파일 존재를 확인한다.

```bash
ls isaacsim_test/outputs/simready/echo_full/sitl/roboto_v2_right_arm_amazinghand_full.urdf
```

기대 결과: URDF 파일 경로가 출력된다.

2. right arm joint 이름을 검색한다.

```bash
rg -n "right_arm_pitch_joint|right_arm_roll_joint|right_arm_yaw_joint|right_elbow_pitch_joint|right_elbow_yaw_joint" isaacsim_test/outputs/simready/echo_full/sitl/roboto_v2_right_arm_amazinghand_full.urdf
```

기대 결과: 5개 joint 이름이 모두 나온다. 하나라도 없으면 C04는 fail입니다.

3. XML parser로 axis/limit을 추출한다.

```bash
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET

urdf = Path("isaacsim_test/outputs/simready/echo_full/sitl/roboto_v2_right_arm_amazinghand_full.urdf")
root = ET.parse(urdf).getroot()
targets = [
    "right_arm_pitch_joint",
    "right_arm_roll_joint",
    "right_arm_yaw_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
]

for name in targets:
    joint = root.find(f"./joint[@name='{name}']")
    if joint is None:
        print(f"{name}: MISSING")
        continue
    parent = joint.find("parent").attrib.get("link")
    child = joint.find("child").attrib.get("link")
    axis_node = joint.find("axis")
    limit_node = joint.find("limit")
    axis = axis_node.attrib.get("xyz") if axis_node is not None else "MISSING"
    limit = limit_node.attrib if limit_node is not None else {}
    print(name, parent, child, axis, limit)
PY
```

기대 결과: 각 joint의 parent, child, axis, limit dict가 출력된다.

4. C03 clamp 기준과 비교한다.

| Joint | Axis | Lower | Upper | Effort | Velocity |
|---|---|---:|---:|---:|---:|
| `right_arm_pitch_joint` | `0 1 0` | -1.57 | 1.57 | 18 | 3.7692 |
| `right_arm_roll_joint` | `1 0 0` | -1.0 | 0.25 | 18 | 3.7692 |
| `right_arm_yaw_joint` | `0 0 -1` | -1.57 | 1.57 | 18 | 3.7692 |
| `right_elbow_pitch_joint` | `0 1 0` | -0.6 | 1.57 | 18 | 3.7692 |
| `right_elbow_yaw_joint` | `1 0 0` | -1.57 | 1.57 | 18 | 3.7692 |

5. AmazingHand joint와 초기 scalar 정책을 구분한다.

```bash
rg -n "amazinghand|hand|finger|thumb|index|middle" isaacsim_test/outputs/simready/echo_full/sitl/roboto_v2_right_arm_amazinghand_full.urdf | head -120
```

기대 결과: hand 관련 joint가 많이 보일 수 있다. 그러나 C01-C06 초기 contract에서는 `amazinghand_grasp.pos` scalar 하나만 사용한다고 산출물에 명시합니다.

### 산출물에 적을 표

```markdown
| Joint | Parent | Child | Axis | Lower | Upper | Effort | Velocity | C03 clamp 일치 |
|---|---|---|---|---:|---:|---:|---:|---|
| `right_arm_pitch_joint` |  |  |  |  |  |  |  | yes/no |
| `right_arm_roll_joint` |  |  |  |  |  |  |  | yes/no |
| `right_arm_yaw_joint` |  |  |  |  |  |  |  | yes/no |
| `right_elbow_pitch_joint` |  |  |  |  |  |  |  | yes/no |
| `right_elbow_yaw_joint` |  |  |  |  |  |  |  | yes/no |
```

### 흔한 실수와 병목

- URDF의 `lower/upper`를 실제 모터 safe limit으로 그대로 쓰면 안 됩니다. 실제 DM4340P limit은 hardware parity 단계에서 더 보수적으로 잡아야 합니다.
- `axis` 방향은 C03 sign 결정에 영향을 줍니다. 단순히 joint 이름만 보고 부호를 정하지 않습니다.
- AmazingHand joint가 많이 보인다고 C03 action space에 모두 넣지 않습니다. 초기 dataset contract는 작게 유지합니다.

### 사람이 승인할 기준

```text
[ ] 5개 right arm joint가 모두 audit table에 있다.
[ ] lower/upper/effort/velocity 값이 있다.
[ ] C03 clamp 기준과 일치한다.
[ ] AmazingHand raw joint를 초기 dataset action에 넣지 않는다는 점이 명시되어 있다.
```

### 보고서 템플릿

완료 후 아래 형식으로 `docs/sitl/2026-07-02_core_validation/artifacts/C04_urdf_joint_limit_audit_<name>.md`를 작성한다.

````markdown
# C04 URDF Joint/Limit Audit - <name>

## Learning Summary

- URDF에서 확인한 right arm joint:
- C03 clamp와 일치하지 않는 항목:
- URDF limit과 hardware safe limit을 구분해야 하는 이유:

## Commands

```bash
rg -n "right_arm_pitch_joint|right_arm_roll_joint|right_arm_yaw_joint|right_elbow_pitch_joint|right_elbow_yaw_joint" isaacsim_test/outputs/simready/echo_full/sitl/roboto_v2_right_arm_amazinghand_full.urdf
```

## Audit Table

| Joint | Parent | Child | Axis | Lower | Upper | Effort | Velocity | Pass/Fail |
|---|---|---|---|---:|---:|---:|---:|---|
| `right_arm_pitch_joint` |  |  |  |  |  |  |  |  |
| `right_arm_roll_joint` |  |  |  |  |  |  |  |  |
| `right_arm_yaw_joint` |  |  |  |  |  |  |  |  |
| `right_elbow_pitch_joint` |  |  |  |  |  |  |  |  |
| `right_elbow_yaw_joint` |  |  |  |  |  |  |  |  |

## Decision

```text
[ ] C05 no-op 검증으로 넘어가도 된다.
[ ] C05로 넘어가면 안 된다. 이유:
```
````

## 2. AI Agent 내부 문서

AI agent에게 위임할 때는 아래 내부 문서를 사용한다.

```text
docs/sitl/2026-07-02_core_validation/ai_agent/C04_robotov2_urdf_joint_limit_audit.ai.ko.md
```
