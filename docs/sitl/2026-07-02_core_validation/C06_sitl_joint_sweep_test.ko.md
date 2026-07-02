# C06 - SITL Joint Sweep Test

## 목적

C05 no-op 검증이 통과한 뒤, Robotov2.0 right arm 5개 joint와 `amazinghand_grasp.pos` 1개 scalar가 Isaac Sim SITL에서 6D action/state contract로 반복 동작하는지 확인한다.

이 task는 실제 DM4340P 모터를 움직이지 않는다. 학생이 배워야 하는 핵심은 "한 pose가 맞는지"보다 "여러 pose를 순서대로 보내도 joint order, limit, visual evidence, numerical evidence가 모두 일관되는지"를 확인하는 방법이다.

> 번호 주의: 이 문서의 C06은 2026-07-02 핵심 검증부 기준의 `SITL joint sweep test`이다. `docs/sitl/2026-06-27/team_tiny_tasks_sitl.md`의 예전 C06 의미와 섞지 않는다.

## 1. 사람용

### 학습 목표

이 task를 끝내면 학생은 다음을 설명할 수 있어야 한다.

```text
[ ] no-op 검증과 joint sweep 검증의 차이를 설명할 수 있다.
[ ] sweep case가 왜 여러 개 필요한지 설명할 수 있다.
[ ] screenshot evidence와 numerical evidence의 역할 차이를 설명할 수 있다.
[ ] `simready_motion_cases.json`의 pose 값이 6D contract와 어떻게 연결되는지 설명할 수 있다.
[ ] C06 통과가 실제 hardware safe limit 검증을 대체하지 않는다는 점을 설명할 수 있다.
```

### 이 검증을 하는 이유

C05는 "움직이지 않는 명령이 안전한가"를 본다. C06은 "움직이는 명령을 여러 방향으로 보내도 contract가 유지되는가"를 본다.

| C05 | C06 |
|---|---|
| no-op 한 개 target | 여러 pose target |
| target jump 방지 | joint order, sign 후보, limit, visual pose 확인 |
| JSON 중심 | JSON + screenshot/contact sheet |
| 다음 단계 진입 전 안전 gate | hardware parity 전 시뮬레이션 동작 gate |

### 핵심 개념

| 용어 | 여기서의 의미 |
|---|---|
| joint sweep | 여러 joint target을 case 단위로 바꿔가며 simulator 반응을 확인하는 검증. |
| pose case | 특정 robot 자세를 만들기 위한 6D target 묶음. |
| visual evidence | screenshot, contact sheet처럼 사람이 눈으로 확인하는 증거. |
| numerical evidence | target, sent_action, observed, absolute_error처럼 수치로 pass/fail을 판단하는 증거. |
| contact sheet | 여러 screenshot을 한 장으로 모아 비교하기 쉽게 만든 이미지. |

### 선행 조건

C06은 아래 조건을 만족해야 시작한다.

```text
[ ] C01 wrapper 구조 확인 완료
[ ] C02 leader output shape 확인 완료
[ ] C03 6D mapping 기준 확인 완료
[ ] C04 URDF joint/limit audit 확인 완료
[ ] C05 no-op evidence JSON 통과
[ ] 실제 DM4340P motor power 또는 CAN command를 사용하지 않음
```

### Sweep case

기준 입력은 `isaacsim_test/simready_motion_cases.json`이다.

| Case | 6D target | 학습 목적 |
|---|---|---|
| `home` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | C05와 같은 neutral 기준선. |
| `reach_forward` | `[0.35, -0.25, 0.40, -0.60, 0.30, 0.50]` | shoulder/elbow가 함께 움직일 때 order가 유지되는지 확인. |
| `elbow_fold` | `[0.15, 0.10, 0.0, -0.85, 0.45, 0.75]` | elbow fold와 grasp scalar가 동시에 들어가도 contract가 유지되는지 확인. |
| `side_sweep` | `[-0.30, 0.25, -0.35, -0.45, -0.25, 0.25]` | yaw/roll 방향 전환에서 sign 후보와 visual pose를 확인. |

각 target의 마지막 값은 `amazinghand_grasp.pos`이다. 이것은 아직 AmazingHand raw 8-servo command가 아니다.

### 실습 전 안전 체크

```text
[ ] C05 no-op evidence가 `passed=true`이다.
[ ] `simready_motion_cases.json`의 joint_names가 6개이다.
[ ] 각 case의 positions가 6개 feature를 모두 포함한다.
[ ] screenshot runner는 Isaac Sim container만 사용한다.
[ ] 실제 motor/CAN/hand serial command는 사용하지 않는다.
```

### 실행 절차 A - 정적 확인

1. repo root로 이동한다.

```bash
cd /home/sim/Documents/superarm_ws
```

2. static contract test를 실행한다.

```bash
python3 isaacsim_test/test_v2_roboparty_config.py
```

기대 결과:

```text
Ran 25 tests
OK
```

3. sweep case JSON을 사람이 읽을 수 있게 출력한다.

```bash
python3 - <<'PY'
from pathlib import Path
import json

path = Path("isaacsim_test/simready_motion_cases.json")
data = json.loads(path.read_text(encoding="utf-8"))
print("joint_names:", data["joint_names"])
for case in data["cases"]:
    ordered = [case["positions"][name] for name in data["joint_names"]]
    print(case["name"], ordered)
PY
```

기대 결과: `home`, `reach_forward`, `elbow_fold`, `side_sweep` 네 줄이 출력된다.

### 실행 절차 B - Screenshot evidence

Isaac Sim container 환경이 준비되어 있으면 screenshot runner를 실행한다.

```bash
bash isaacsim_test/run_simready_motion_screenshot_cases.sh
```

기본 산출물:

```text
isaacsim_test/artifacts/simready_motion_cases/
isaacsim_test/artifacts/simready_motion_cases_contact_sheet.png
isaacsim_test/artifacts/runtime_logs/direct_urdf_motion_<timestamp>.log
isaacsim_test/outputs/simready/echo_full/sitl/echo_full_lerobot_articulation_report.json
```

확인할 것:

```text
[ ] 4개 PNG가 생성되었다.
[ ] contact sheet가 생성되었다.
[ ] 각 screenshot 이름이 case 이름과 대응된다.
[ ] robot이 화면 밖으로 나가거나 blank image가 아니다.
```

### 실행 절차 C - Numerical evidence

현재 `verify_lerobot_sitl.py`는 단일 target 검증만 지원한다. 따라서 case별로 4번 실행하거나, 별도 follow-up에서 sweep mode를 추가한다.

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

각 JSON에서 확인할 것:

```text
[ ] `passed`가 true이다.
[ ] `target`, `sent_action`, `observed`, `absolute_error`가 모두 6D이다.
[ ] 모든 `absolute_error`가 tolerance 이하이다.
[ ] 마지막 값 `amazinghand_grasp.pos`가 `[0.0, 1.0]` 범위 안에 있다.
```

### 결과 해석법

| Evidence | 통과로 볼 수 있는 것 | 통과로 볼 수 없는 것 |
|---|---|---|
| screenshot PNG | robot pose가 시각적으로 변했는지, 화면이 blank가 아닌지 | target과 observed의 수치 오차 |
| contact sheet | 4개 pose가 서로 다른지 한 번에 비교 | joint별 pass/fail |
| per-case JSON | target/sent_action/observed/error 수치 검증 | 실제 hardware sign/safe limit |
| runtime log | Isaac Sim load, runner error 여부 | 정책 성능 |

### Pass/Fail 기준

```text
PASS:
[ ] C05가 통과했다.
[ ] 4개 screenshot이 존재한다.
[ ] contact sheet가 존재한다.
[ ] 4개 numerical evidence JSON이 모두 `passed=true`이다.
[ ] 실제 hardware command가 없다.

PARTIAL:
[ ] screenshot은 있지만 numerical evidence가 없다.
[ ] 이 경우 "visual evidence present"일 뿐 "joint sweep pass"가 아니다.

FAIL:
[ ] case 하나라도 tolerance를 넘었다.
[ ] screenshot이 blank이거나 case 수가 부족하다.
[ ] 6D shape가 깨졌다.
[ ] 실제 hardware command 여부가 불명확하다.
```

### 실패했을 때 보는 순서

| 증상 | 먼저 확인할 것 |
|---|---|
| screenshot이 없다 | Docker/Isaac Sim container, `run_simready_motion_screenshot_cases.sh` log 확인. |
| screenshot이 blank | camera framing, custom visual USD load, viewport/replicator capture path 확인. |
| 특정 case만 fail | 해당 target이 URDF limit 안인지, joint order가 C03/C04와 같은지 확인. |
| 모든 case가 fail | ROS domain, articulation binding, `/follower/joint_states` topic 확인. |
| hand 값만 이상 | `amazinghand_grasp.pos` clamp와 target 마지막 값 확인. |

### 보고서 템플릿

완료 후 아래 형식으로 `docs/sitl/2026-07-02_core_validation/artifacts/C06_sitl_joint_sweep_test_<name>.md`를 작성한다.

````markdown
# C06 SITL Joint Sweep Test - <name>

## Learning Summary

- C05와 C06의 차이:
- screenshot evidence와 numerical evidence의 차이:
- 6D contract가 유지되었다고 판단한 이유:

## Commands

```bash
python3 isaacsim_test/test_v2_roboparty_config.py
bash isaacsim_test/run_simready_motion_screenshot_cases.sh
python3 isaacsim_test/lerobot/verify_lerobot_sitl.py --target 0,0,0,0,0,0 --tolerance 0.03 --evidence isaacsim_test/artifacts/c06_sitl_joint_sweep_home.json
```

## Evidence

| Item | Value |
|---|---|
| Screenshot dir | `isaacsim_test/artifacts/simready_motion_cases/` |
| Contact sheet | `isaacsim_test/artifacts/simready_motion_cases_contact_sheet.png` |
| Home JSON | `isaacsim_test/artifacts/c06_sitl_joint_sweep_home.json` |
| Reach forward JSON | `isaacsim_test/artifacts/c06_sitl_joint_sweep_reach_forward.json` |
| Elbow fold JSON | `isaacsim_test/artifacts/c06_sitl_joint_sweep_elbow_fold.json` |
| Side sweep JSON | `isaacsim_test/artifacts/c06_sitl_joint_sweep_side_sweep.json` |
| Hardware commanded | `false` |

## Case Results

| Case | Screenshot | Numerical JSON | Passed | Max absolute error |
|---|---|---|---|---:|
| `home` | `<path>` | `<path>` | `<true/false>` | `<value>` |
| `reach_forward` | `<path>` | `<path>` | `<true/false>` | `<value>` |
| `elbow_fold` | `<path>` | `<path>` | `<true/false>` | `<value>` |
| `side_sweep` | `<path>` | `<path>` | `<true/false>` | `<value>` |

## Decision

```text
[ ] C07 hardware parity 준비로 넘어가도 된다.
[ ] C07로 넘어가면 안 된다. 이유:
```
````

## 2. AI Agent 내부 문서

AI agent에게 위임할 때는 아래 내부 실행 문서를 직접 전달한다.

```text
docs/sitl/2026-07-02_core_validation/ai_agent/C06_sitl_joint_sweep_test.ai.ko.md
```
