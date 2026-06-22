# 00 — Scope Correction: RoboParty 5-DOF Humanoid Arm, Not OpenArm

Correction: the arm target in this project is **not OpenArm**. It is the **5-DOF arm/upper-limb section from the RoboParty ROBOTO_ORIGIN humanoid**, combined with **AmazingHand** as the end-effector.



Correct assumption:

```text
RoboParty humanoid 5-DOF arm + AmazingHand dexterous hand
```

## 1. Updated target robot

Use this LeRobot robot type name:

```text
roboparty_5dof_arm_amazinghand_follower
```

Recommended action layout for the first learning system:

```text
5 arm joint targets + 1 hand grasp scalar
```

Example:

```text
action = [
  arm_j1,
  arm_j2,
  arm_j3,
  arm_j4,
  arm_j5,
  hand_grasp,
]
```

Later:

```text
5 arm joint targets + AmazingHand pattern controls
```

Much later:

```text
5 arm joint targets + 8 AmazingHand servo targets
```

## 2. RoboParty motor groups relevant to the arm

From the RoboParty deploy configs and hardware connection docs, the full humanoid has 23 motors grouped as:

```text
can0: 6 motors
can1: 7 motors
can2: 5 motors
can3: 5 motors
```

The public deploy documentation maps:

```text
can2 → left hand / left upper-limb group
can3 → right hand / right upper-limb group
```

For your project, treat those 5-motor groups as the relevant RoboParty arm section.

Likely starting point:

```text
Left arm group:  motor IDs 14–18 on can2
Right arm group: motor IDs 19–23 on can3
```

Verify this against the actual CAD/URDF and your wiring before moving anything.

## 3. Updated package boundary

Use RoboParty for:

```text
5-DOF arm mechanical design
DM motor/CAN mapping
motor signs
zero offsets
URDF/MJCF model
AmazingHand wrist adapter design
```

Use LeRobot for:

```text
robot interface
teleoperation
recording
dataset format
ACT/SmolVLA training
policy inference
```

Do not import OpenArm classes unless you only use them as coding examples.

## 4. Updated repository layout

```text
roboparty_lerobot/
├── roboparty_5dof_arm_config.py
├── roboparty_5dof_arm_amazinghand.py
├── motor_bus/
│   └── dm_can_adapter.py
├── hand/
│   └── amazinghand_adapter.py
├── processors/
│   ├── action_clamp.py
│   ├── hand_scalar_to_servo_targets.py
│   └── observation_formatter.py
└── configs/
    ├── left_arm_can2.yaml
    ├── right_arm_can3.yaml
    ├── amazinghand_right.yaml
    └── cameras.yaml
```

## 5. Updated milestones

### Milestone 1 — 5-DOF arm only

```text
connect CAN
read 5 motor positions
set zero offsets
send very small single-joint targets
validate signs and limits
```

### Milestone 2 — AmazingHand only

```text
connect serial bus
read 8 servo positions
calibrate middle positions
open/close at low speed
validate safe servo limits
```

### Milestone 3 — Combined arm + hand

```text
LeRobot flat keys = 5 arm joint .pos keys + amazinghand_grasp.pos
record 10 debug episodes
```

### Milestone 4 — ACT baseline

```text
train ACT on one simple task
use 5+1 action dimension first
```

## 6. Important consequence of 5 DOF

A 5-DOF arm has less end-effector orientation freedom than a 7-DOF arm. That affects manipulation.

For first tasks, choose objects and setups that do not require complex wrist orientation:

```text
front-facing pick
top-down or shallow-angle approach
large/light objects
tray placement
fixed table height
```

Avoid early tasks that require:

```text
precise wrist roll
complex in-hand reorientation
side insertion
tool use
cluttered grasp planning
```

AmazingHand helps with contact and grasping, but it does not fully replace missing arm DOF.
