# C01 - LeRobot Checkout과 SITL Wrapper 구조 확인

## 목적

현재 repo의 local LeRobot checkout과 `IsaacSimRpoArmRobot` 구조를 확인해서, 팀원이 어디로 action을 보내고 어디서 observation을 읽는지 설명할 수 있게 만든다.

이 task는 실제 하드웨어를 움직이지 않는다. 코드 구조와 mock/SITL wrapper contract를 문서화하는 검증이다.

## 1. 사람용

### 학습 목표

이 task의 학습 목표는 코드를 실행하기 전에 제어 경로를 말로 설명할 수 있는 상태를 만드는 것입니다. 로봇 시스템에서는 같은 `action`이라는 단어도 정책 replay, teleoperation, ROS2 topic publish, 실제 motor command에서 의미가 달라질 수 있습니다. C01은 그 혼동을 줄이기 위한 구조 확인 task입니다.

완료 후 학생은 다음 질문에 답할 수 있어야 합니다.

```text
Q1. LeRobot에서 robot wrapper는 어떤 lifecycle로 움직이는가?
Q2. 이 repo에서 Isaac Sim SITL wrapper 파일과 config 파일은 어디에 있는가?
Q3. `send_action()`과 `teleop_step()`은 언제 다르게 쓰는가?
Q4. `/leader/joint_commands`, `/follower/joint_commands`, `/follower/joint_states`는 각각 어떤 방향의 데이터인가?
Q5. 이 task가 실제 DM4340P 모터를 움직이지 않는다고 판단하는 근거는 무엇인가?
```

### 선수 지식

| 개념 | 학생이 알아야 할 수준 |
|---|---|
| Python class | class가 상태와 method를 가진다는 정도 |
| ROS2 topic | publisher와 subscriber가 topic 이름으로 데이터를 주고받는다는 정도 |
| LeRobot robot wrapper | `connect()`, `send_action()`, `capture_observation()` 같은 표준 interface가 있다는 정도 |
| SITL | 실제 하드웨어 대신 시뮬레이터를 follower로 사용하는 검증 방식이라는 정도 |

### 확인할 내용

- 이 repo의 SITL robot type은 `isaacsim_rpo_arm`이다.
- SITL wrapper 파일은 `isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py`이다.
- SITL config 파일은 `isaacsim_test/lerobot/rpo_arm_isaacsim.yaml`이다.
- 1차 action/state contract는 6D이다.
- ROS2는 이 repo의 Isaac Sim SITL transport일 뿐, LeRobot 자체 요구사항이 아니다.

### 이해해야 할 lifecycle

```text
IsaacSimRpoArmConfig 생성
  -> IsaacSimRpoArmRobot(config) 생성
  -> connect()
  -> send_action(action) 또는 teleop_step()
  -> capture_observation()
  -> disconnect()
```

### 경로 구분

| 목적 | 사용 경로 |
|---|---|
| verifier / policy replay / deterministic command 검증 | `IsaacSimRpoArmRobot.send_action()` |
| live leader teleop / recording | `/leader/joint_commands` 입력 + `teleop_step()` |

### 진행 절차

1. repo 위치를 확인한다.

```bash
pwd
```

기대 결과:

```text
/home/sim/Documents/superarm_ws
```

2. wrapper와 config 파일 존재를 확인한다.

```bash
ls isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py
ls isaacsim_test/lerobot/rpo_arm_isaacsim.yaml
```

기대 결과: 두 파일 경로가 그대로 출력된다. 파일이 없으면 C01은 blocked입니다.

3. wrapper에서 lifecycle method를 찾는다.

```bash
rg -n "def connect|def disconnect|def send_action|def teleop_step|def capture_observation" isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py
```

기대 결과: 각 method 정의 line이 출력된다. 산출물에는 line number와 method 역할을 표로 정리합니다.

4. config에서 topic 이름과 feature 이름을 확인한다.

```bash
rg -n "leader|follower|joint_commands|joint_states|features|joint_names" isaacsim_test/lerobot/rpo_arm_isaacsim.yaml
```

기대 결과: leader/follower topic과 6D feature 관련 설정을 확인할 수 있다.

5. Python 문법 검증을 실행한다.

```bash
python3 -m py_compile isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py
```

기대 결과: 출력 없이 exit code 0. 문법 오류가 있으면 C02로 넘어가지 않습니다.

### 산출물에 적을 표

```markdown
| Method 또는 Topic | 파일 위치 | 역할 | Hardware command 여부 |
|---|---|---|---|
| `connect()` | `isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py:<line>` | <설명> | no |
| `send_action()` | `isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py:<line>` | <설명> | no, SITL follower command |
| `teleop_step()` | `isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py:<line>` | <설명> | no, ROS2 topic bridge |
| `capture_observation()` | `isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py:<line>` | <설명> | no |
| `/leader/joint_commands` | `rpo_arm_isaacsim.yaml:<line>` | leader 입력 topic | no |
| `/follower/joint_commands` | `rpo_arm_isaacsim.yaml:<line>` | SITL follower 명령 topic | no |
| `/follower/joint_states` | `rpo_arm_isaacsim.yaml:<line>` | SITL follower 관측 topic | no |
```

### 흔한 실수와 병목

- `send_action()`만 보고 leader arm live teleop 경로라고 착각하면 안 됩니다. live teleop은 `/leader/joint_commands` 입력과 `teleop_step()` 경로를 따로 봐야 합니다.
- ROS2 topic이 나온다고 해서 실제 하드웨어를 제어한다고 판단하면 안 됩니다. 이 문서에서는 Isaac Sim SITL transport인지 먼저 확인합니다.
- `capture_observation()`의 key와 action key가 다르면 나중에 dataset collection에서 feature mismatch가 납니다.

### 사람이 승인할 기준

```text
[ ] `connect()`가 무엇을 여는지 설명할 수 있다.
[ ] `send_action()`과 `teleop_step()`의 차이를 설명할 수 있다.
[ ] `capture_observation()`이 어떤 key를 반환하는지 설명할 수 있다.
[ ] `/leader/joint_commands`, `/follower/joint_commands`, `/follower/joint_states`의 차이를 설명할 수 있다.
[ ] 실제 DM4340P hardware wrapper와 SITL wrapper를 혼동하지 않는다.
```

### 보고서 템플릿

완료 후 아래 형식으로 `docs/sitl/2026-07-02_core_validation/artifacts/C01_lerobot_wrapper_layout_<name>.md`를 작성한다.

````markdown
# C01 LeRobot Wrapper Layout - <name>

## Learning Summary

- LeRobot wrapper lifecycle:
- `send_action()`과 `teleop_step()`의 차이:
- 실제 hardware command가 아니라고 판단한 이유:

## Commands

```bash
rg -n "def connect|def disconnect|def send_action|def teleop_step|def capture_observation" isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py
rg -n "leader|follower|joint_commands|joint_states|features|joint_names" isaacsim_test/lerobot/rpo_arm_isaacsim.yaml
python3 -m py_compile isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py
```

## Wrapper Contract

| Item | Path/Line | Meaning |
|---|---|---|
| Robot type |  |  |
| Action path |  |  |
| Teleop path |  |  |
| Observation path |  |  |

## Decision

```text
[ ] C02로 넘어가도 된다.
[ ] C02로 넘어가면 안 된다. 이유:
```
````

## 2. AI Agent 내부 문서

AI agent에게 위임할 때는 아래 내부 문서를 사용한다.

```text
docs/sitl/2026-07-02_core_validation/ai_agent/C01_lerobot_wrapper_layout.ai.ko.md
```
