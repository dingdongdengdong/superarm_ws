# 04 - LeRobot Custom Robot Skeleton for RoboParty 5-DOF Arm + AmazingHand

```text
RoboParty 5-DOF arm + AmazingHand
```

LeRobot robot type:

```text
roboparty_5dof_arm_amazinghand_follower
```

## 1. In-tree package layout

This integration now lives in the local LeRobot checkout, not as an external
`lerobot.common.*` style package:

```text
lerobot/src/lerobot/robots/roboparty_5dof_arm_amazinghand/
├── __init__.py
├── config_roboparty_5dof_arm_amazinghand.py
└── roboparty_5dof_arm_amazinghand.py
```

The factory branch is registered in:

```text
lerobot/src/lerobot/robots/utils.py
```

CLI modules that decode `RobotConfig` must import the package for draccus choice
registration, matching the existing in-tree robot pattern.

## 2. Current LeRobot imports

Use the current APIs:

```python
from lerobot.robots.config import RobotConfig
from lerobot.robots.robot import Robot
```

not the older `lerobot.common.robots.*` import paths.

## 3. Config class

The registered config is:

```python
@RobotConfig.register_subclass("roboparty_5dof_arm_amazinghand_follower")
@dataclass
class Roboparty5DofArmAmazingHandFollowerConfig(RobotConfig):
    side: str = "right"
    port: str = "can3"
    can_interface: str = "socketcan"
    use_can_fd: bool = True
    motor_config: dict[str, tuple[int, int, str]]
    motor_signs: dict[str, int]
    joint_limits: dict[str, tuple[float, float]]
    position_kp: list[float] | float
    position_kd: list[float] | float
    max_relative_target: float | dict[str, float] | None

    hand_enabled: bool = True
    hand_serial_port: str = "/dev/ttyUSB_AH_RIGHT"
    hand_baudrate: int = 1_000_000
    hand_timeout: float = 0.5
    hand_servo_ids: list[int]
    hand_middle_pos_deg: list[float]
    hand_safe_limits_deg: dict[int, tuple[float, float]]
    hand_default_speed: int = 3
```

Default side mappings:

```text
right: port can3, motor IDs 19-23, signs [-1, 1, 1, -1, 1]
left:  port can2, motor IDs 14-18, signs [ 1, 1, 1,  1, 1]
```

Damiao receive IDs are derived as:

```text
recv_id = send_id + master_id_offset
```

with `master_id_offset=16` by default.

## 4. Feature contract

The first learning version uses flat LeRobot feature keys:

```text
rpo_arm_j1.pos
rpo_arm_j2.pos
rpo_arm_j3.pos
rpo_arm_j4.pos
rpo_arm_j5.pos
amazinghand_grasp.pos
```

Do not publish this v1 robot as a direct `observation.state` array. LeRobot's
recording and policy processor layers assemble state/action tensors from flat
robot features.

Units exposed to LeRobot:

```text
RoboParty arm joints: degrees
AmazingHand grasp:   scalar in [0.0, 1.0]
```

The AmazingHand adapter converts servo targets to radians internally before
calling `rustypot`.

## 5. Combined robot behavior

The robot class:

```python
class Roboparty5DofArmAmazingHandFollower(Robot):
    name = "roboparty_5dof_arm_amazinghand_follower"
    config_class = Roboparty5DofArmAmazingHandFollowerConfig
```

Expected data flow:

```text
get_observation()
  Damiao raw joint degrees
  -> apply motor_signs
  -> emit rpo_arm_j*.pos in LeRobot degrees
  -> emit amazinghand_grasp.pos from last/read scalar
  -> add camera frames using their camera keys

send_action(action)
  flat .pos action keys
  -> clamp arm joint limits
  -> clamp max_relative_target against present position
  -> apply motor_signs back to raw Damiao targets
  -> send arm first
  -> clamp hand scalar to [0.0, 1.0]
  -> send hand second
  -> return the clipped flat action actually sent
```

## 6. First smoke test

```python
from lerobot.robots.roboparty_5dof_arm_amazinghand import (
    Roboparty5DofArmAmazingHandFollower,
    Roboparty5DofArmAmazingHandFollowerConfig,
)

cfg = Roboparty5DofArmAmazingHandFollowerConfig(
    side="right",
    hand_enabled=True,
)
robot = Roboparty5DofArmAmazingHandFollower(cfg)
robot.connect(calibrate=False)

obs = robot.get_observation()
print({k: obs[k] for k in robot.action_features})

action = {k: obs[k] for k in robot.action_features}
action["rpo_arm_j1.pos"] += 0.5
action["amazinghand_grasp.pos"] = 0.0
sent = robot.send_action(action)
print(sent)

robot.disconnect()
```

Use tiny arm deltas first, around `0.5` to `1.0` degree. Validate mechanical
mounting, reachable workspace, wrist load, joint signs, and soft limits before
recording real episodes.

## 7. Later work

Keep the first policy at 6D:

```text
5 RoboParty arm joints + 1 AmazingHand grasp scalar
```

Raw AmazingHand servo control is later work:

```text
5 arm joints + 8 AmazingHand servo targets = 13D action
```
