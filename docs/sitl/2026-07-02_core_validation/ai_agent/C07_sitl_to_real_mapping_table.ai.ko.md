# AI C07 - SITL-to-real Mapping Table

## 목표

C01-C06 evidence를 근거로 SITL 6D feature와 실제 Robotov2.0 right arm DM4340P 후보 mapping을 표로 정리한다. 이 task는 문서 작성만 수행하며 실제 hardware command는 금지한다.

## 읽을 파일

```text
docs/sitl/2026-07-02_core_validation/C03_leader_to_sitl_6d_action.ko.md
docs/sitl/2026-07-02_core_validation/C04_robotov2_urdf_joint_limit_audit.ko.md
docs/sitl/2026-07-02_core_validation/C05_sitl_no_op_test.ko.md
docs/sitl/2026-07-02_core_validation/C06_sitl_joint_sweep_test.ko.md
docs/sitl/2026-07-02_roboto_v2_right_arm_amazinghand_sitl_plan.ko.md
```

## 작성할 산출물

```text
docs/sitl/2026-07-02_core_validation/artifacts/C07_hardware_parity_checklist_<name>.md
```

## 필수 table

```markdown
| SITL feature | URDF lower | URDF upper | Real target candidate | CAN/port candidate | Sign candidate | Hardware status | Next verification |
|---|---:|---:|---|---|---:|---|---|
| `right_arm_pitch_joint.pos` | -1.57 | 1.57 | `19` | `can3` | -1 | pending | read-only ID scan |
| `right_arm_roll_joint.pos` | -1.0 | 0.25 | `20` | `can3` | 1 | pending | read-only ID scan |
| `right_arm_yaw_joint.pos` | -1.57 | 1.57 | `21` | `can3` | 1 | pending | read-only ID scan |
| `right_elbow_pitch_joint.pos` | -0.6 | 1.57 | `22` | `can3` | -1 | pending | read-only ID scan |
| `right_elbow_yaw_joint.pos` | -1.57 | 1.57 | `23` | `can3` | 1 | pending | read-only ID scan |
| `amazinghand_grasp.pos` | 0.0 | 1.0 | serial adapter | serial | scalar | pending | servo map check |
```

## 검증 명령

문서 내 feature 누락을 확인한다.

```bash
python3 - <<'PY'
from pathlib import Path

path = Path("docs/sitl/2026-07-02_core_validation/artifacts/C07_hardware_parity_checklist_<name>.md")
text = path.read_text(encoding="utf-8")
required = [
    "right_arm_pitch_joint.pos",
    "right_arm_roll_joint.pos",
    "right_arm_yaw_joint.pos",
    "right_elbow_pitch_joint.pos",
    "right_elbow_yaw_joint.pos",
    "amazinghand_grasp.pos",
    "hardware_commanded",
    "pending",
]
missing = [item for item in required if item not in text]
if missing:
    raise SystemExit(f"missing: {missing}")
print("C07 mapping artifact check passed")
PY
```

## Stop condition

```text
[ ] C03/C04 근거 없이 mapping table 작성 금지
[ ] 후보값을 확정값처럼 쓰기 금지
[ ] 실제 CAN scan 또는 motor command 실행 금지
[ ] hardware status가 없는 행은 승인 금지
```
