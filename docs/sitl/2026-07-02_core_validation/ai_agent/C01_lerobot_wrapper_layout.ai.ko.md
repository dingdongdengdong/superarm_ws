# AI C01 - LeRobot Checkout과 SITL Wrapper 구조 확인

## 목표

local LeRobot checkout과 `IsaacSimRpoArmRobot` 구조를 조사해 wrapper lifecycle, 6D feature, ROS2 topic contract를 문서화한다.

## 읽을 파일

```text
isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py
isaacsim_test/lerobot/rpo_arm_isaacsim.yaml
lerobot/pyproject.toml
docs/sitl/2026-06-27/task_separation_lerobot_isaac_sim_arm_sitl.md
```

## 실행할 명령

```bash
git -C lerobot rev-parse --short HEAD
grep -n "version" lerobot/pyproject.toml | head
python3 -m py_compile isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py
```

## 산출물 경로

```text
docs/sitl/2026-07-02_core_validation/artifacts/C01_lerobot_wrapper_layout_<name>.md
```

## 산출물 필수 내용

- LeRobot commit/version line.
- robot type: `isaacsim_rpo_arm`.
- wrapper file/config file.
- 6D feature key 순서.
- lifecycle method 표.
- 경로 구분:
  - verifier / policy replay / deterministic command: `send_action()`.
  - live leader teleop / recording: `/leader/joint_commands` + `teleop_step()`.

## 완료 기준

```text
[ ] 산출물에 LeRobot commit/version 정보가 있다.
[ ] 산출물에 wrapper lifecycle 설명이 있다.
[ ] 산출물에 6D feature key 순서가 있다.
[ ] `send_action()`과 `teleop_step()` 경로가 분리되어 있다.
[ ] `python3 -m py_compile`이 통과했다.
```
