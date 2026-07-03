# AI C09 - Dataset Schema Check

## 목표

LeRobot dataset recording 전에 `observation.state`와 `action` feature metadata가 6D contract와 일치하는지 확인하고 학생용 schema report를 작성한다.

## 읽을 파일

```text
docs/sitl/2026-07-02_core_validation/C09_dataset_schema_check.ko.md
isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py
isaacsim_test/lerobot/rpo_arm_isaacsim.yaml
isaacsim_test/test_v2_roboparty_config.py
```

## 작성할 산출물

```text
docs/sitl/2026-07-02_core_validation/artifacts/C09_dataset_schema_check_<name>.md
```

## 실행 명령

```bash
python3 - <<'PY'
from pathlib import Path
import sys

wrapper_dir = Path("isaacsim_test/lerobot").resolve()
if str(wrapper_dir) not in sys.path:
    sys.path.insert(0, str(wrapper_dir))

from isaacsim_rpo_arm_robot import IsaacSimRpoArmConfig, IsaacSimRpoArmRobot

robot = IsaacSimRpoArmRobot(IsaacSimRpoArmConfig(mock=True))
features = robot.features
expected_names = [
    "right_arm_pitch_joint.pos",
    "right_arm_roll_joint.pos",
    "right_arm_yaw_joint.pos",
    "right_elbow_pitch_joint.pos",
    "right_elbow_yaw_joint.pos",
    "amazinghand_grasp.pos",
]
for key in ("observation.state", "action"):
    item = features[key]
    if tuple(item["shape"]) != (6,):
        raise SystemExit(f"{key} shape mismatch: {item['shape']}")
    if list(item["names"]) != expected_names:
        raise SystemExit(f"{key} names mismatch: {item['names']}")
    if item["dtype"] != "float32":
        raise SystemExit(f"{key} dtype mismatch: {item['dtype']}")
print("C09 dataset schema check passed")
PY
```

## Report 필수 항목

```text
[ ] observation.state dtype/shape/names
[ ] action dtype/shape/names
[ ] 두 names list가 동일한지
[ ] raw AmazingHand 8-servo를 아직 action에 넣지 않는 이유
[ ] hardware_commanded=false
```

## Stop condition

```text
[ ] shape가 `(6,)`가 아님
[ ] feature order 불일치
[ ] action과 observation.state names 불일치
[ ] dtype이 float32가 아님
[ ] 실제 hardware 연결 필요 상태
```
