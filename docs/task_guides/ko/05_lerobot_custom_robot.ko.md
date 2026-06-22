# 05 - LeRobot 커스텀 로봇 래퍼

## 목표

RoboParty CAN control과 AmazingHand serial control을 표준 LeRobot `Robot`
인터페이스 뒤에 숨기는 하나의 robot type을 만듭니다.

## Robot type

```text
roboparty_5dof_arm_amazinghand_follower
```

## Package layout

local LeRobot checkout 안에 생성합니다.

```text
lerobot/src/lerobot/robots/roboparty_5dof_arm_amazinghand/
├── __init__.py
├── config_roboparty_5dof_arm_amazinghand.py
└── roboparty_5dof_arm_amazinghand.py
```

Robot type registration 위치:

```text
lerobot/src/lerobot/robots/utils.py
```

## Feature contract

Flat LeRobot key를 노출합니다.

```text
rpo_arm_j1.pos
rpo_arm_j2.pos
rpo_arm_j3.pos
rpo_arm_j4.pos
rpo_arm_j5.pos
amazinghand_grasp.pos
```

단위:

```text
arm joints: degrees
hand grasp: scalar in [0.0, 1.0]
```

## Configuration fields

최소 config field:

```text
side
port
motor_ids
motor_signs
joint_limits
max_relative_target
hand_enabled
hand_serial_port
hand_servo_ids
hand_safe_limits
```

## Required behavior

`get_observation()`은 다음을 해야 합니다.

```text
read raw arm positions
apply calibration offsets
apply motor signs
return rpo_arm_j*.pos in degrees
return amazinghand_grasp.pos
include camera observations through normal LeRobot camera config
```

`send_action(action)`은 다음을 해야 합니다.

```text
read flat action keys
clamp joint limits
clamp max relative target per step
apply motor signs back to raw motor targets
send RoboParty arm command over CAN
clamp AmazingHand scalar to [0.0, 1.0]
send AmazingHand command over serial
return the clipped action actually sent
```

## Smoke test

먼저 no-op action을 실행합니다.

```python
obs = robot.get_observation()
action = {key: obs[key] for key in robot.action_features}
sent = robot.send_action(action)
print(sent)
```

그 다음 작은 joint delta를 실행합니다.

```python
action["rpo_arm_j1.pos"] = obs["rpo_arm_j1.pos"] + 0.5
action["amazinghand_grasp.pos"] = 0.0
sent = robot.send_action(action)
print(sent)
```

## 완료 조건

```text
[ ] LeRobot이 robot type을 instantiate할 수 있습니다.
[ ] connect()가 arm과 hand connection을 엽니다.
[ ] get_observation()이 6개 flat key를 모두 반환합니다.
[ ] send_action()이 clipped action value를 반환합니다.
[ ] joint sign이 physical motion과 일치합니다.
[ ] relative target clamp가 큰 jump를 막습니다.
[ ] hand scalar가 AmazingHand를 안전하게 open/close합니다.
```
