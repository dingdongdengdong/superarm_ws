# C05 - SITL No-op Test

## 목적

LeRobot leader arm 입력을 실제 DM4340P 모터가 아니라 Isaac Sim SITL follower에 연결했을 때, no-op action이 Robotov2.0 right arm + AmazingHand 6D contract 안에서 안전하게 유지되는지 확인한다.

이 task는 실제 hardware command를 보내지 않는다. 학생이 배워야 하는 핵심은 "로봇이 움직이지 않아야 하는 명령"도 중요한 검증 대상이라는 점이다. no-op이 흔들리면 이후 tiny motion, joint sweep, hardware parity는 모두 위험해진다.

> 번호 주의: 이 문서의 C05는 2026-07-02 핵심 검증부 기준의 `SITL no-op test`이다. `docs/sitl/2026-06-27/team_tiny_tasks_sitl.md`의 예전 C05 의미와 섞지 않는다.

## 1. 사람용

### 학습 목표

이 task를 끝내면 학생은 다음을 설명할 수 있어야 한다.

```text
[ ] SITL이 실제 hardware 검증 전에 필요한 이유를 설명할 수 있다.
[ ] LeRobot action/state가 왜 6D contract인지 설명할 수 있다.
[ ] no-op vector `[0, 0, 0, 0, 0, 0]`의 각 원소가 어떤 joint 또는 hand scalar인지 말할 수 있다.
[ ] `target`, `sent_action`, `observed`, `absolute_error`, `tolerance`의 의미를 구분할 수 있다.
[ ] "통과"와 "실제 하드웨어로 넘어가도 됨"이 같은 말이 아니라는 점을 설명할 수 있다.
```

### 이 검증을 하는 이유

로봇 제어에서 가장 먼저 확인할 것은 "움직이라는 명령"이 아니라 "움직이지 말라는 명령"이다. no-op command가 안전하지 않으면 다음 문제가 생길 수 있다.

| 문제 | 왜 위험한가 |
|---|---|
| action dimension mismatch | 6개 값만 보내야 하는데 5개 또는 8개 값이 들어가면 joint가 밀려 매핑될 수 있다. |
| default target jump | 초기 상태가 의도치 않게 0으로 당겨지면 실제 모터에서는 갑작스러운 움직임이 된다. |
| hand scalar 오류 | `amazinghand_grasp.pos`가 `[0.0, 1.0]` 밖으로 나가면 hand servo 제어로 확장할 때 위험하다. |
| ROS topic mismatch | command topic과 state topic이 다른 robot을 보고 있으면 검증 결과를 믿을 수 없다. |

### 선수 지식

학생은 아래 개념을 완벽히 구현할 필요는 없지만, 뜻은 알고 있어야 한다.

| 용어 | 여기서의 의미 |
|---|---|
| SITL | Software-in-the-loop. 실제 로봇 없이 simulator와 software path만 먼저 검증하는 단계. |
| LeRobot wrapper | LeRobot이 robot을 다루는 공통 interface. 이 repo에서는 `IsaacSimRpoArmRobot`이다. |
| action | robot에 보내는 목표값. 이 task에서는 6개 float vector. |
| observation.state | simulator에서 다시 읽은 현재 상태값. 이 task에서도 6개 float vector. |
| tolerance | target과 observed가 얼마나 달라도 통과로 볼지 정한 허용 오차. |
| no-op | 움직임을 의도하지 않는 명령. 여기서는 6개 값이 모두 0인 action. |

### 6D contract

C05에서는 아래 순서가 절대 바뀌면 안 된다.

| Index | Feature | No-op target | 학생 확인 질문 |
|---:|---|---:|---|
| 0 | `right_arm_pitch_joint.pos` | 0.0 | shoulder pitch에 해당하는가? |
| 1 | `right_arm_roll_joint.pos` | 0.0 | shoulder roll에 해당하는가? |
| 2 | `right_arm_yaw_joint.pos` | 0.0 | shoulder yaw에 해당하는가? |
| 3 | `right_elbow_pitch_joint.pos` | 0.0 | elbow pitch에 해당하는가? |
| 4 | `right_elbow_yaw_joint.pos` | 0.0 | elbow yaw에 해당하는가? |
| 5 | `amazinghand_grasp.pos` | 0.0 | hand open/close scalar인가? |

`amazinghand_grasp.pos = 0.0`은 초기 정책에서 open 또는 neutral grasp intent로 취급한다. AmazingHand의 실제 8개 servo ID를 여기서 직접 제어하지 않는다.

### 실습 전 안전 체크

실습자는 아래를 먼저 확인하고 체크한다.

```text
[ ] 실제 DM4340P motor power를 켜지 않았다.
[ ] 실제 CAN bus에 command를 보낼 script를 실행하지 않는다.
[ ] hand servo serial adapter에 write command를 보내지 않는다.
[ ] 이 task의 명령은 `isaacsim_test/lerobot/verify_lerobot_sitl.py`만 사용한다.
[ ] evidence는 `isaacsim_test/artifacts/` 아래 JSON으로 남긴다.
```

### 실행 절차

1. repo root로 이동한다.

```bash
cd /home/sim/Documents/superarm_ws
```

2. 정적 contract test를 먼저 실행한다.

```bash
python3 isaacsim_test/test_v2_roboparty_config.py
```

기대 결과:

```text
Ran 25 tests
OK
```

3. verifier script가 Python 문법 오류 없이 import 가능한지 확인한다.

```bash
python3 -m py_compile \
  isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py \
  isaacsim_test/lerobot/verify_lerobot_sitl.py \
  isaacsim_test/test_v2_roboparty_config.py
```

기대 결과: 출력 없이 exit code 0.

4. Isaac Sim SITL과 ROS2 bridge가 떠 있는 상태에서 no-op 검증을 실행한다.

```bash
python3 isaacsim_test/lerobot/verify_lerobot_sitl.py \
  --target 0,0,0,0,0,0 \
  --tolerance 0.03 \
  --evidence isaacsim_test/artifacts/c05_sitl_no_op_evidence.json
```

5. JSON을 열어 핵심 field를 확인한다.

```bash
python3 - <<'PY'
from pathlib import Path
import json

path = Path("isaacsim_test/artifacts/c05_sitl_no_op_evidence.json")
data = json.loads(path.read_text(encoding="utf-8"))
print("passed:", data.get("passed"))
print("target:", data.get("target"))
print("sent_action:", data.get("sent_action"))
print("observed:", data.get("observed"))
print("absolute_error:", data.get("absolute_error"))
PY
```

기대 결과:

```text
passed: True
target: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
sent_action: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

`observed`와 `absolute_error`는 simulator 상태에 따라 작은 float 오차가 있을 수 있다. 모든 `absolute_error`가 tolerance 이하이면 no-op 검증을 통과로 본다.

### 결과 해석법

| Field | 뜻 | 통과 기준 |
|---|---|---|
| `passed` | verifier가 target과 observed를 비교한 최종 결과 | `true` |
| `joint_names` | 6D vector가 어떤 joint 순서인지 설명하는 이름 목록 | 6개이며 C03/C04와 같은 순서 |
| `target` | 사용자가 요청한 no-op command | 모두 0.0 |
| `sent_action` | wrapper가 실제 publish한 command | 모두 0.0 |
| `observed` | simulator에서 읽은 state | 6D vector |
| `absolute_error` | `abs(observed - target)` | 각 값 <= `tolerance` |
| `tolerance` | 허용 오차 | 기본 0.03 |

### 실패했을 때 보는 순서

| 증상 | 먼저 확인할 것 |
|---|---|
| `TimeoutError` | Isaac Sim이 떠 있는지, `ROS_DOMAIN_ID=42`인지, `/follower/joint_states`가 publish되는지 확인한다. |
| `Unexpected config joint_names` | `isaacsim_test/lerobot/rpo_arm_isaacsim.yaml`의 joint order가 C03/C04와 같은지 확인한다. |
| `sent_action`이 6개가 아님 | `IsaacSimRpoArmRobot._normalize_vector()`와 config `joint_names` 길이를 확인한다. |
| 마지막 값이 0.0-1.0 밖 | hand scalar clamp가 깨진 것이므로 C06으로 넘어가지 않는다. |
| `absolute_error`가 큼 | simulator articulation binding, joint name, topic 연결을 확인한다. |

### 사람이 승인할 기준

```text
[ ] 정적 contract test가 통과했다.
[ ] py_compile이 통과했다.
[ ] no-op command가 6D action으로 전송된다.
[ ] `sent_action`이 `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`이다.
[ ] `observed` shape가 `(6,)`이다.
[ ] 각 joint absolute error 또는 pre/post delta가 tolerance 이하다.
[ ] `amazinghand_grasp.pos`는 `[0.0, 1.0]` 범위 안에 있다.
[ ] 실제 DM4340P motor, CAN bus, hand servo에는 command를 보내지 않았다.
```

### 보고서 템플릿

완료 후 아래 형식으로 `docs/sitl/2026-07-02_core_validation/artifacts/C05_sitl_no_op_test_<name>.md`를 작성한다.

````markdown
# C05 SITL No-op Test - <name>

## Learning Summary

- 오늘 확인한 contract:
- no-op이 중요한 이유:
- 실제 hardware로 넘어가지 않은 이유:

## Commands

```bash
python3 isaacsim_test/test_v2_roboparty_config.py
python3 -m py_compile isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py isaacsim_test/lerobot/verify_lerobot_sitl.py isaacsim_test/test_v2_roboparty_config.py
python3 isaacsim_test/lerobot/verify_lerobot_sitl.py --target 0,0,0,0,0,0 --tolerance 0.03 --evidence isaacsim_test/artifacts/c05_sitl_no_op_evidence.json
```

## Evidence

| Item | Value |
|---|---|
| Evidence JSON | `isaacsim_test/artifacts/c05_sitl_no_op_evidence.json` |
| Passed | `<true/false>` |
| Max absolute error | `<value>` |
| Hardware commanded | `false` |

## Decision

```text
[ ] C06 joint sweep로 넘어가도 된다.
[ ] C06으로 넘어가면 안 된다. 이유:
```
````

## 2. AI Agent 내부 문서

AI agent에게 위임할 때는 아래 내부 실행 문서를 직접 전달한다.

```text
docs/sitl/2026-07-02_core_validation/ai_agent/C05_sitl_no_op_test.ai.ko.md
```
