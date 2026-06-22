# 05 - LeRobot Custom Robot Wrapper

## Goal

Create one LeRobot robot type that hides RoboParty CAN control and AmazingHand
serial control behind a standard LeRobot `Robot` interface.

## Robot type

```text
roboparty_5dof_arm_amazinghand_follower
```

## Package layout

Create this in the local LeRobot checkout:

```text
lerobot/src/lerobot/robots/roboparty_5dof_arm_amazinghand/
├── __init__.py
├── config_roboparty_5dof_arm_amazinghand.py
└── roboparty_5dof_arm_amazinghand.py
```

Register the robot type in:

```text
lerobot/src/lerobot/robots/utils.py
```

## Feature contract

Expose flat LeRobot keys:

```text
rpo_arm_j1.pos
rpo_arm_j2.pos
rpo_arm_j3.pos
rpo_arm_j4.pos
rpo_arm_j5.pos
amazinghand_grasp.pos
```

Use these units:

```text
arm joints: degrees
hand grasp: scalar in [0.0, 1.0]
```

## Configuration fields

Minimum config fields:

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

`get_observation()` must:

```text
read raw arm positions
apply calibration offsets
apply motor signs
return rpo_arm_j*.pos in degrees
return amazinghand_grasp.pos
include camera observations through normal LeRobot camera config
```

`send_action(action)` must:

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

Run a no-op action first:

```python
obs = robot.get_observation()
action = {key: obs[key] for key in robot.action_features}
sent = robot.send_action(action)
print(sent)
```

Then run a tiny joint delta:

```python
action["rpo_arm_j1.pos"] = obs["rpo_arm_j1.pos"] + 0.5
action["amazinghand_grasp.pos"] = 0.0
sent = robot.send_action(action)
print(sent)
```

## Done when

```text
[ ] LeRobot can instantiate the robot type.
[ ] connect() opens arm and hand connections.
[ ] get_observation() returns all 6 flat keys.
[ ] send_action() returns clipped action values.
[ ] Joint signs match physical motion.
[ ] Relative target clamp prevents large jumps.
[ ] Hand scalar opens and closes AmazingHand safely.
```
