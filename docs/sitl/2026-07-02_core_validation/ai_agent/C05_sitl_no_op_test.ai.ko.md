# AI C05 - SITL No-op Test

## 목표

No-op 6D action이 Isaac Sim SITL follower에서 target jump 없이 유지되는지 evidence로 남긴다. 산출물은 학생이 읽을 수 있는 report와 machine-readable JSON 둘 다 필요하다.

## 범위

허용 작업:

```text
[ ] 문서와 evidence만 생성한다.
[ ] Isaac Sim SITL runtime 또는 mock/static 검증만 사용한다.
[ ] `isaacsim_test/artifacts/c05_sitl_no_op_evidence.json`을 생성 또는 검토한다.
[ ] `docs/sitl/2026-07-02_core_validation/artifacts/C05_sitl_no_op_test_<name>.md`를 작성한다.
```

금지 작업:

```text
[ ] 실제 DM4340P motor enable 금지
[ ] 실제 CAN bus command 금지
[ ] hand servo serial write 금지
[ ] C06 또는 hardware parity로 임의 진행 금지
[ ] 2026-06-27 문서의 예전 C05 의미를 적용 금지
```

## 읽을 파일

```text
docs/sitl/2026-07-02_core_validation/C05_sitl_no_op_test.ko.md
docs/sitl/2026-07-02_core_validation/C03_leader_to_sitl_6d_action.ko.md
docs/sitl/2026-07-02_core_validation/C04_robotov2_urdf_joint_limit_audit.ko.md
isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py
isaacsim_test/lerobot/verify_lerobot_sitl.py
isaacsim_test/lerobot/rpo_arm_isaacsim.yaml
isaacsim_test/test_v2_roboparty_config.py
```

## 핵심 구현 지식

`IsaacSimRpoArmRobot`의 contract는 6D이다.

```text
0 right_arm_pitch_joint.pos
1 right_arm_roll_joint.pos
2 right_arm_yaw_joint.pos
3 right_elbow_pitch_joint.pos
4 right_elbow_yaw_joint.pos
5 amazinghand_grasp.pos
```

검증해야 하는 no-op target:

```text
0,0,0,0,0,0
```

핵심 코드 경로:

```text
verify_lerobot_sitl.py
  -> IsaacSimRpoArmConfig
  -> IsaacSimRpoArmRobot.connect()
  -> IsaacSimRpoArmRobot.send_action(target)
  -> IsaacSimRpoArmRobot.capture_observation()
  -> evidence JSON write
```

## 단계별 실행

### Step 1 - 정적 contract test

Run:

```bash
python3 isaacsim_test/test_v2_roboparty_config.py
```

Expected:

```text
Ran 25 tests
OK
```

Fail이면 C05 runtime 검증으로 넘어가지 말고 실패 test name과 traceback을 report에 기록한다.

### Step 2 - Python compile check

Run:

```bash
python3 -m py_compile \
  isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py \
  isaacsim_test/lerobot/verify_lerobot_sitl.py \
  isaacsim_test/test_v2_roboparty_config.py
```

Expected: no output, exit code 0.

### Step 3 - SITL no-op runtime check

Prerequisite: Isaac Sim SITL and ROS2 bridge are running with matching `ROS_DOMAIN_ID`.

Run:

```bash
python3 isaacsim_test/lerobot/verify_lerobot_sitl.py \
  --target 0,0,0,0,0,0 \
  --tolerance 0.03 \
  --evidence isaacsim_test/artifacts/c05_sitl_no_op_evidence.json
```

Expected:

```text
"passed": true
"target": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
"sent_action": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

### Step 4 - Evidence schema validation

Run:

```bash
python3 - <<'PY'
from pathlib import Path
import json

path = Path("isaacsim_test/artifacts/c05_sitl_no_op_evidence.json")
data = json.loads(path.read_text(encoding="utf-8"))
required = [
    "passed",
    "joint_names",
    "target",
    "sent_action",
    "observed",
    "absolute_error",
    "tolerance",
    "config",
]
missing = [key for key in required if key not in data]
if missing:
    raise SystemExit(f"missing keys: {missing}")
if len(data["joint_names"]) != 6:
    raise SystemExit(f"joint_names length is not 6: {data['joint_names']}")
if len(data["target"]) != 6 or len(data["sent_action"]) != 6 or len(data["observed"]) != 6:
    raise SystemExit("target/sent_action/observed must all be 6D")
if data["sent_action"] != [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]:
    raise SystemExit(f"unexpected sent_action: {data['sent_action']}")
if not data["passed"]:
    raise SystemExit("C05 evidence passed=false")
print("C05 evidence schema passed")
PY
```

Expected:

```text
C05 evidence schema passed
```

## 권장 확장 schema

현재 verifier는 단일 target-vs-observed schema를 쓴다. no-op drift까지 엄밀히 보려면 follow-up task에서 아래 schema를 추가한다.

```json
{
  "mode": "sitl_no_op",
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
  "target": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "sent_action": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "observed_before": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "observed_after": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "absolute_delta": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "max_absolute_delta": 0.0,
  "tolerance": 0.01,
  "hardware_commanded": false
}
```

## Report 작성

Create:

```text
docs/sitl/2026-07-02_core_validation/artifacts/C05_sitl_no_op_test_<name>.md
```

Minimum content:

````markdown
# C05 SITL No-op Test - <name>

## Summary

- Result: `<PASS/FAIL/BLOCKED>`
- Evidence JSON: `isaacsim_test/artifacts/c05_sitl_no_op_evidence.json`
- Hardware commanded: `false`

## Verification

| Check | Result | Evidence |
|---|---|---|
| Static contract | `<PASS/FAIL>` | `python3 isaacsim_test/test_v2_roboparty_config.py` |
| Compile | `<PASS/FAIL>` | `python3 -m py_compile ...` |
| Runtime no-op | `<PASS/FAIL/BLOCKED>` | `c05_sitl_no_op_evidence.json` |

## JSON Highlights

```text
passed:
target:
sent_action:
observed:
absolute_error:
```

## Student Explanation

- no-op이 중요한 이유:
- 6D contract가 유지되었다고 판단한 이유:
- C06으로 넘어가도 되는지:
````

## Stop condition

다음 중 하나라도 발생하면 완료로 보고하지 말고 `BLOCKED` 또는 `FAIL`로 보고한다.

```text
[ ] static contract test 실패
[ ] py_compile 실패
[ ] SITL runtime 미기동으로 no-op evidence 생성 불가
[ ] evidence JSON schema 불일치
[ ] `passed=false`
[ ] 실제 hardware command가 발생했거나 발생 여부를 확인할 수 없음
```
