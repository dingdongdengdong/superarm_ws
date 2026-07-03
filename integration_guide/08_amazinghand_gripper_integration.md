# 08 — AmazingHand on RoboParty 5-DOF Arm

Correction: AmazingHand will be mounted on the **RoboParty humanoid 5-DOF arm**, not on OpenArm.

Target system:

```text
RoboParty 5-DOF arm
        +
AmazingHand 8-DOF hand
        ↓
LeRobot robot type:
roboparty_5dof_arm_amazinghand_follower
```

## 1. Why this matters

AmazingHand is not a simple one-motor gripper. It is an 8-DOF, 4-finger dexterous hand.

But the arm has only 5 DOF, so the first learning setup should stay simple:

```text
5 arm joints + 1 hand scalar
```

not:

```text
5 arm joints + 8 raw hand servos
```

Full finger-level dexterity can come later.

## 2. Recommended action stages

### Stage 1 — Scalar gripper abstraction

Best first version.

LeRobot-facing flat action/observation keys:

```text
rpo_arm_j1.pos
rpo_arm_j2.pos
rpo_arm_j3.pos
rpo_arm_j4.pos
rpo_arm_j5.pos
amazinghand_grasp.pos
```

`amazinghand_grasp`:

```text
0.0 = open
1.0 = close / grasp
```

Arm joint positions exposed to LeRobot are in degrees. The AmazingHand adapter
converts servo targets to radians internally for `rustypot`.

### Stage 2 — Pattern-level hand control

```text
action shape = (8,) or similar
```

Example:

```text
5 arm joints + open_close + pinch + spread
```

### Stage 3 — Raw servo control

Later work, not v1:

```text
5 arm joints + 8 AmazingHand servo targets
```

Use only after the simple policy works.

## 3. Hardware/control route

Use AmazingHand’s Python serial-bus route first:

```text
host computer
  → serial bus driver
  → Feetech SCS0009 servos
  → AmazingHand
```

This integrates more naturally with LeRobot than an Arduino-only control loop.

## 4. Servo ID mapping

For one hand, AmazingHand examples use 8 servo IDs:

```yaml
amazinghand:
  serial_port: /dev/ttyUSB_AH_RIGHT
  baudrate: 1000000
  servo_ids:
    index:  [1, 2]
    middle: [3, 4]
    ring:   [5, 6]
    thumb:  [7, 8]
```

If left and right hands share one serial bus, use unique IDs:

```yaml
right_hand: [1, 2, 3, 4, 5, 6, 7, 8]
left_hand:  [11, 12, 13, 14, 15, 16, 17, 18]
```

## 5. Calibration storage

AmazingHand examples use `MiddlePos` offsets. Store them in config:

```yaml
amazinghand:
  middle_pos_deg: [3, 0, -5, -8, -2, 5, -12, 0]
  default_speed: 3
  safe_limits_deg:
    servo_1: [-45, 95]
    servo_2: [-95, 45]
    servo_3: [-45, 95]
    servo_4: [-95, 45]
    servo_5: [-45, 95]
    servo_6: [-95, 45]
    servo_7: [-90, 95]
    servo_8: [-95, 45]
```

These values are provisional calibration values. Measure your assembled hand before commanding real servos.

## 6. Scalar grasp mapping

```python
import numpy as np


def grasp_scalar_to_servo_targets(grasp: float, middle_pos_deg: list[float]) -> dict[int, float]:
    g = float(np.clip(grasp, 0.0, 1.0))

    open_deg = [-35, 35, -35, 35, -35, 35, -35, 35]
    close_deg = [90, -90, 90, -90, 90, -90, 90, -90]

    targets = {}
    for i, servo_id in enumerate(range(1, 9)):
        deg = (1.0 - g) * open_deg[i] + g * close_deg[i]
        deg += middle_pos_deg[i]
        targets[servo_id] = np.deg2rad(deg)

    return targets
```

## 7. Combined config example

```yaml
robot:
  type: roboparty_5dof_arm_amazinghand_follower
  id: rpo_right_5dof_ah_v1

arm:
  can_interface: can3
  joint_names: [rpo_arm_j1, rpo_arm_j2, rpo_arm_j3, rpo_arm_j4, rpo_arm_j5]
  motor_ids: [19, 20, 21, 22, 23]
  motor_signs: [-1, 1, 1, -1, 1]
  max_relative_target_rad: 0.02

amazinghand:
  serial_port: /dev/ttyUSB_AH_RIGHT
  baudrate: 1000000
  action_mode: scalar_grasp
  servo_ids: [1, 2, 3, 4, 5, 6, 7, 8]
  middle_pos_deg: [3, 0, -5, -8, -2, 5, -12, 0]
  default_speed: 3

features:
  flat_keys:
    - rpo_arm_j1.pos
    - rpo_arm_j2.pos
    - rpo_arm_j3.pos
    - rpo_arm_j4.pos
    - rpo_arm_j5.pos
    - amazinghand_grasp.pos
  arm_units: degrees
  grasp_units: scalar_0_to_1
```

## 8. Mechanical integration note

AmazingHand’s original wrist interface targets Reachy2, but its CAD is intended to be adaptable. For RoboParty’s arm, design an adapter:

```text
RoboParty 5-DOF wrist/end link
        ↓
custom adapter plate
        ↓
AmazingHand wrist mount
```

Update URDF/MJCF:

```text
rpo_arm_j5_child_link
└── amazinghand_base_link
```

At minimum, add:

```text
hand mass: about 0.4 kg
adapter mass
collision box/spheres
new tool frame
wrist camera transform if used
```

## 9. First task recommendation

Use a simple task that fits 5 DOF:

```text
front-facing cube pick
soft cube
tray placement
scalar hand open/close
fixed table height
fixed camera
```

Avoid first:

```text
precise wrist orientation tasks
insertion
tool use
small object dexterity
cluttered bins
```

## 10. Strong recommendation

Your first policy should be:

```text
ACT with 6D action:
5 RoboParty arm joints + 1 AmazingHand grasp scalar
```

That is the most realistic path to getting a successful first manipulation demo.
