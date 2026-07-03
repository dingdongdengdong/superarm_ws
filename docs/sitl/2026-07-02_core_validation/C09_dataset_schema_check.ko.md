# C09 - Dataset Schema Check

## 목적

LeRobot dataset을 기록하기 전에 `observation.state`와 `action`의 shape, dtype, feature names가 C01-C08에서 고정한 6D contract와 일치하는지 확인한다.

이 task는 실제 hardware를 움직이지 않는다. 학생이 배워야 하는 핵심은 dataset 품질이 policy 학습 전에 schema에서 먼저 결정된다는 점이다.

## 1. 사람용

### 학습 목표

```text
[ ] LeRobot dataset에서 `observation.state`와 `action`의 역할을 설명할 수 있다.
[ ] shape `(6,)`와 feature name order가 왜 중요한지 설명할 수 있다.
[ ] dataset schema mismatch가 학습 실패로 이어지는 이유를 설명할 수 있다.
[ ] raw AmazingHand 8-servo 값을 초기 dataset action에 넣지 않는 이유를 설명할 수 있다.
```

### 확인할 schema

| Key | dtype | shape | names |
|---|---|---|---|
| `observation.state` | `float32` | `(6,)` | 5 arm joints + `amazinghand_grasp.pos` |
| `action` | `float32` | `(6,)` | 5 arm joints + `amazinghand_grasp.pos` |

Feature order:

```text
right_arm_pitch_joint.pos
right_arm_roll_joint.pos
right_arm_yaw_joint.pos
right_elbow_pitch_joint.pos
right_elbow_yaw_joint.pos
amazinghand_grasp.pos
```

### 왜 중요한가

Dataset schema가 틀리면 policy는 잘못된 의미의 숫자를 학습한다. 예를 들어 action index 3이 elbow pitch라고 생각했는데 실제로 yaw 값이면, 학습된 policy는 simulation replay에서 이상하게 보이고 hardware로 옮기면 위험해진다.

### 실행 절차

1. wrapper feature metadata를 출력한다.

```bash
python3 - <<'PY'
from pathlib import Path
import sys

wrapper_dir = Path("isaacsim_test/lerobot").resolve()
if str(wrapper_dir) not in sys.path:
    sys.path.insert(0, str(wrapper_dir))

from isaacsim_rpo_arm_robot import IsaacSimRpoArmConfig, IsaacSimRpoArmRobot

robot = IsaacSimRpoArmRobot(IsaacSimRpoArmConfig(mock=True))
for key, value in robot.features.items():
    print(key, value)
PY
```

2. schema가 6D인지 확인한다.

```text
[ ] `features["observation.state"]["shape"] == (6,)`
[ ] `features["action"]["shape"] == (6,)`
[ ] 두 names list가 동일하다.
[ ] 마지막 name이 `amazinghand_grasp.pos`이다.
```

3. dataset recording 명령을 실행하기 전에 dry-run report를 작성한다.

### 산출물 경로

```text
docs/sitl/2026-07-02_core_validation/artifacts/C09_dataset_schema_check_<name>.md
```

### 보고서 템플릿

````markdown
# C09 Dataset Schema Check - <name>

## Learning Summary

- `observation.state`와 `action`의 차이:
- 6D feature order가 중요한 이유:
- raw 8-servo hand를 아직 dataset action에 넣지 않는 이유:

## Feature Metadata

| Key | dtype | shape | names |
|---|---|---|---|
| `observation.state` | `float32` | `(6,)` | `<names>` |
| `action` | `float32` | `(6,)` | `<names>` |

## Decision

```text
[ ] C10 debug dataset record/replay로 넘어가도 된다.
[ ] C10으로 넘어가면 안 된다. 이유:
```
````

### 승인 기준

```text
[ ] `observation.state` shape가 `(6,)`이다.
[ ] `action` shape가 `(6,)`이다.
[ ] 두 feature order가 일치한다.
[ ] `amazinghand_grasp.pos`가 마지막 feature다.
[ ] 실제 hardware command 없이 mock 또는 SITL metadata로 확인했다.
```

## 2. AI Agent 내부 문서

AI agent에게 위임할 때는 아래 내부 실행 문서를 직접 전달한다.

```text
docs/sitl/2026-07-02_core_validation/ai_agent/C09_dataset_schema_check.ai.ko.md
```
