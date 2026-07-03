# AI C04 - Robotov2.0 + AmazingHand URDF Joint/Limit Audit

## 목표

`roboto_v2_right_arm_amazinghand_full.urdf`에서 Robotov2.0 right arm 5개 joint의 parent/child/axis/limit을 추출한다.

## 읽을 파일

```text
isaacsim_test/outputs/simready/echo_full/sitl/roboto_v2_right_arm_amazinghand_full.urdf
isaacsim_test/lerobot/rpo_arm_isaacsim.yaml
docs/sitl/2026-07-02_core_validation/C03_leader_to_sitl_6d_action.ko.md
```

## 실행할 audit 명령

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
    axis = joint.find("axis").attrib.get("xyz")
    limit = joint.find("limit").attrib
    print(name, parent, child, axis, limit)
PY
```

## 기준값

| Joint | Axis | Lower | Upper | Effort | Velocity |
|---|---|---:|---:|---:|---:|
| `right_arm_pitch_joint` | `0 1 0` | -1.57 | 1.57 | 18 | 3.7692 |
| `right_arm_roll_joint` | `1 0 0` | -1.0 | 0.25 | 18 | 3.7692 |
| `right_arm_yaw_joint` | `0 0 -1` | -1.57 | 1.57 | 18 | 3.7692 |
| `right_elbow_pitch_joint` | `0 1 0` | -0.6 | 1.57 | 18 | 3.7692 |
| `right_elbow_yaw_joint` | `1 0 0` | -1.57 | 1.57 | 18 | 3.7692 |

## 산출물 경로

```text
docs/sitl/2026-07-02_core_validation/artifacts/C04_urdf_joint_limit_audit_<name>.md
```

## 완료 기준

```text
[ ] 5개 right arm joint가 모두 추출되어 있다.
[ ] parent/child/axis/lower/upper/effort/velocity가 있다.
[ ] AmazingHand scalar-only policy가 명시되어 있다.
[ ] URDF limit과 real hardware safe limit을 구분했다.
```
