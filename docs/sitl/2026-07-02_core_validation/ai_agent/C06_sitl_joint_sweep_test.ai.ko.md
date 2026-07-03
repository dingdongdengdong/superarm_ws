# AI C06 - SITL Joint Sweep Test

## 목표

C05 no-op 검증 뒤 4개 sweep case를 실행해 screenshot/contact sheet와 numerical evidence를 남긴다. 학생용 산출물에는 "무엇을 배웠는지"와 "왜 pass/fail인지"가 함께 들어가야 한다.

## 범위

허용 작업:

```text
[ ] `simready_motion_cases.json`의 4개 case를 확인한다.
[ ] screenshot runner를 실행하거나 기존 산출물을 검토한다.
[ ] case별 numerical evidence JSON을 생성하거나 생성 불가 사유를 명시한다.
[ ] `docs/sitl/2026-07-02_core_validation/artifacts/C06_sitl_joint_sweep_test_<name>.md`를 작성한다.
```

금지 작업:

```text
[ ] 실제 DM4340P motor enable 금지
[ ] 실제 CAN bus command 금지
[ ] hand servo serial write 금지
[ ] C05 실패 상태에서 C06 pass 보고 금지
[ ] screenshot만 보고 numerical pass라고 주장 금지
[ ] 2026-06-27 문서의 예전 C06 의미를 적용 금지
```

## 읽을 파일

```text
docs/sitl/2026-07-02_core_validation/C06_sitl_joint_sweep_test.ko.md
docs/sitl/2026-07-02_core_validation/C05_sitl_no_op_test.ko.md
docs/sitl/2026-07-02_core_validation/C03_leader_to_sitl_6d_action.ko.md
docs/sitl/2026-07-02_core_validation/C04_robotov2_urdf_joint_limit_audit.ko.md
isaacsim_test/simready_motion_cases.json
isaacsim_test/run_simready_motion_screenshot_cases.sh
isaacsim_test/lerobot/verify_lerobot_sitl.py
isaacsim_test/isaacsim/setup_rpo_arm_scene.py
isaacsim_test/test_v2_roboparty_config.py
```

## 핵심 구현 지식

Sweep case는 아래 순서의 6D vector로 해석한다.

```text
0 right_arm_pitch_joint.pos
1 right_arm_roll_joint.pos
2 right_arm_yaw_joint.pos
3 right_elbow_pitch_joint.pos
4 right_elbow_yaw_joint.pos
5 amazinghand_grasp.pos
```

`simready_motion_cases.json`의 case:

```text
home          -> 0,0,0,0,0,0
reach_forward -> 0.35,-0.25,0.40,-0.60,0.30,0.50
elbow_fold    -> 0.15,0.10,0.0,-0.85,0.45,0.75
side_sweep    -> -0.30,0.25,-0.35,-0.45,-0.25,0.25
```

## 단계별 실행

### Step 0 - C05 선행 조건 확인

Check:

```bash
python3 - <<'PY'
from pathlib import Path
import json

path = Path("isaacsim_test/artifacts/c05_sitl_no_op_evidence.json")
if not path.is_file():
    raise SystemExit("missing C05 evidence; do not run C06 as pass gate")
data = json.loads(path.read_text(encoding="utf-8"))
if not data.get("passed"):
    raise SystemExit("C05 evidence is not passed; do not run C06 as pass gate")
print("C05 prerequisite passed")
PY
```

If this fails because C05 has not been run in the current environment, C06 may still document static/screenshot preparation, but final status must be `BLOCKED`, not `PASS`.

### Step 1 - Static contract test

Run:

```bash
python3 isaacsim_test/test_v2_roboparty_config.py
```

Expected:

```text
Ran 25 tests
OK
```

### Step 2 - Validate sweep case file

Run:

```bash
python3 - <<'PY'
from pathlib import Path
import json

path = Path("isaacsim_test/simready_motion_cases.json")
data = json.loads(path.read_text(encoding="utf-8"))
expected_names = [
    "right_arm_pitch_joint",
    "right_arm_roll_joint",
    "right_arm_yaw_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
    "amazinghand_grasp",
]
if data["joint_names"] != expected_names:
    raise SystemExit(f"unexpected joint_names: {data['joint_names']}")
case_names = [case["name"] for case in data["cases"]]
if case_names != ["home", "reach_forward", "elbow_fold", "side_sweep"]:
    raise SystemExit(f"unexpected case order: {case_names}")
for case in data["cases"]:
    missing = [name for name in expected_names if name not in case["positions"]]
    if missing:
        raise SystemExit(f"{case['name']} missing positions: {missing}")
print("C06 sweep cases schema passed")
PY
```

Expected:

```text
C06 sweep cases schema passed
```

### Step 3 - Runtime screenshot evidence

Run:

```bash
bash isaacsim_test/run_simready_motion_screenshot_cases.sh
```

Expected outputs:

```text
isaacsim_test/artifacts/simready_motion_cases/
isaacsim_test/artifacts/simready_motion_cases_contact_sheet.png
isaacsim_test/artifacts/runtime_logs/direct_urdf_motion_<timestamp>.log
```

Validate screenshot count:

```bash
python3 - <<'PY'
from pathlib import Path

image_dir = Path("isaacsim_test/artifacts/simready_motion_cases")
images = sorted(image_dir.glob("*.png"))
if len(images) < 4:
    raise SystemExit(f"expected at least 4 screenshots, found {len(images)}")
contact_sheet = Path("isaacsim_test/artifacts/simready_motion_cases_contact_sheet.png")
if not contact_sheet.is_file():
    raise SystemExit(f"missing contact sheet: {contact_sheet}")
print("screenshots:", [image.name for image in images[:4]])
print("contact_sheet:", contact_sheet)
PY
```

### Step 4 - Numerical evidence per case

Run each case through `verify_lerobot_sitl.py`.

```bash
python3 isaacsim_test/lerobot/verify_lerobot_sitl.py \
  --target 0,0,0,0,0,0 \
  --tolerance 0.03 \
  --evidence isaacsim_test/artifacts/c06_sitl_joint_sweep_home.json

python3 isaacsim_test/lerobot/verify_lerobot_sitl.py \
  --target 0.35,-0.25,0.40,-0.60,0.30,0.50 \
  --tolerance 0.03 \
  --evidence isaacsim_test/artifacts/c06_sitl_joint_sweep_reach_forward.json

python3 isaacsim_test/lerobot/verify_lerobot_sitl.py \
  --target 0.15,0.10,0.0,-0.85,0.45,0.75 \
  --tolerance 0.03 \
  --evidence isaacsim_test/artifacts/c06_sitl_joint_sweep_elbow_fold.json

python3 isaacsim_test/lerobot/verify_lerobot_sitl.py \
  --target -0.30,0.25,-0.35,-0.45,-0.25,0.25 \
  --tolerance 0.03 \
  --evidence isaacsim_test/artifacts/c06_sitl_joint_sweep_side_sweep.json
```

### Step 5 - Validate per-case evidence

Run:

```bash
python3 - <<'PY'
from pathlib import Path
import json

cases = ["home", "reach_forward", "elbow_fold", "side_sweep"]
for name in cases:
    path = Path(f"isaacsim_test/artifacts/c06_sitl_joint_sweep_{name}.json")
    data = json.loads(path.read_text(encoding="utf-8"))
    if not data.get("passed"):
        raise SystemExit(f"{name} passed=false")
    for key in ["target", "sent_action", "observed", "absolute_error"]:
        if len(data[key]) != 6:
            raise SystemExit(f"{name} {key} is not 6D: {data[key]}")
    max_error = max(float(value) for value in data["absolute_error"])
    print(name, "max_error=", max_error)
print("C06 numerical evidence passed")
PY
```

Expected:

```text
C06 numerical evidence passed
```

## 통합 evidence schema

Optional aggregator output:

```text
isaacsim_test/artifacts/c06_sitl_joint_sweep_evidence.json
```

Recommended schema:

```json
{
  "mode": "sitl_joint_sweep",
  "passed": true,
  "timestamp": "2026-07-02T00:00:00Z",
  "joint_names": [
    "right_arm_pitch_joint",
    "right_arm_roll_joint",
    "right_arm_yaw_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
    "amazinghand_grasp"
  ],
  "tolerance": 0.03,
  "cases": ["home", "reach_forward", "elbow_fold", "side_sweep"],
  "per_case_results": [
    {
      "name": "home",
      "target": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      "sent_action": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      "observed": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      "absolute_error": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      "passed": true
    }
  ],
  "screenshots": [
    "isaacsim_test/artifacts/simready_motion_cases/00_home.png",
    "isaacsim_test/artifacts/simready_motion_cases/01_reach_forward.png",
    "isaacsim_test/artifacts/simready_motion_cases/02_elbow_fold.png",
    "isaacsim_test/artifacts/simready_motion_cases/03_side_sweep.png"
  ],
  "contact_sheet": "isaacsim_test/artifacts/simready_motion_cases_contact_sheet.png",
  "hardware_commanded": false
}
```

## Report 작성

Create:

```text
docs/sitl/2026-07-02_core_validation/artifacts/C06_sitl_joint_sweep_test_<name>.md
```

Minimum content:

````markdown
# C06 SITL Joint Sweep Test - <name>

## Summary

- Result: `<PASS/FAIL/BLOCKED/PARTIAL>`
- Screenshot dir: `isaacsim_test/artifacts/simready_motion_cases/`
- Contact sheet: `isaacsim_test/artifacts/simready_motion_cases_contact_sheet.png`
- Hardware commanded: `false`

## Case Results

| Case | Screenshot | Numerical JSON | Passed | Max absolute error |
|---|---|---|---|---:|
| `home` | `<path>` | `<path>` | `<true/false>` | `<value>` |
| `reach_forward` | `<path>` | `<path>` | `<true/false>` | `<value>` |
| `elbow_fold` | `<path>` | `<path>` | `<true/false>` | `<value>` |
| `side_sweep` | `<path>` | `<path>` | `<true/false>` | `<value>` |

## Student Explanation

- 여러 case를 검증해야 하는 이유:
- screenshot과 numerical JSON이 서로 보완하는 방식:
- C07 hardware parity로 넘어가도 되는지:
````

## Stop condition

다음 중 하나라도 발생하면 완료로 보고하지 말고 `BLOCKED`, `FAIL`, 또는 `PARTIAL`로 보고한다.

```text
[ ] C05 evidence 없음 또는 passed=false
[ ] static contract test 실패
[ ] screenshot runner 실패
[ ] screenshot만 있고 numerical evidence 없음
[ ] case별 JSON 중 하나라도 passed=false
[ ] 6D shape 불일치
[ ] 실제 hardware command가 발생했거나 발생 여부를 확인할 수 없음
```
