# 03 — Motor and CAN Mapping for RoboParty 5-DOF Arm

Correction: this project uses the **5-DOF arm section from RoboParty ROBOTO_ORIGIN**,
Therefore, the LeRobot mapping should be based on RoboParty’s 5-motor CAN groups.

## 1. RoboParty full-body motor grouping

The RoboParty deployment config exposes 23 motors:

```yaml
motor_id: [1, 2, 3, ..., 23]
motor_interface: ["can0", "can1", "can2", "can3"]
motor_num: [6, 7, 5, 5]
motor_type: "DM"
master_id_offset: 16
```

The public hardware connection docs describe:

```text
can0 → left leg
can1 → right leg + waist
can2 → left hand / left upper-limb group
can3 → right hand / right upper-limb group
```

For an arm-only LeRobot system:

```text
Left RoboParty 5-DOF arm:  motor IDs 14–18 on can2
Right RoboParty 5-DOF arm: motor IDs 19–23 on can3
```

Verify this with your actual robot wiring.

## 2. Motor sign values from RoboParty motion config

The full-body `motor_sign` list is:

```text
[1, 1, 1, 1, 1, 1,
 1, 1, -1, -1, -1, -1, 1,
 1, 1, 1, 1, 1,
 -1, 1, 1, -1, 1]
```

So the likely arm group signs are:

```text
Left arm / can2 / IDs 14–18:  [1, 1, 1, 1, 1]
Right arm / can3 / IDs 19–23: [-1, 1, 1, -1, 1]
```

Use these as initial values only. Confirm one joint at a time.

## 3. KP/KD values from RoboParty motion config

The same config gives the arm-side gains approximately as:

```text
IDs 14–18: kp [40, 40, 40, 30, 20], kd [2, 2, 2, 1.5, 1]
IDs 19–23: kp [40, 40, 40, 30, 20], kd [2, 2, 2, 1.5, 1]
```

For LeRobot learning, start with lower speed and conservative command deltas even if these gains are valid.

## 4. Recommended LeRobot joint names

Until you confirm URDF joint names, use neutral names:

```text
rpo_arm_j1
rpo_arm_j2
rpo_arm_j3
rpo_arm_j4
rpo_arm_j5
```

Later replace with physical names such as:

```text
shoulder_yaw
shoulder_roll
shoulder_pitch
elbow_pitch
wrist_pitch_or_roll
```

Do not guess physical names in the dataset if you are not sure. Stable ordering matters more than pretty names.

## 5. Right arm config example

```yaml
robot:
  type: roboparty_5dof_arm_amazinghand_follower
  id: rpo_right_arm_ah_v1
  side: right

arm:
  type: rpo_5dof_arm
  can_interface: can3
  motor_type: DM
  master_id_offset: 16
  joint_names:
    - rpo_arm_j1
    - rpo_arm_j2
    - rpo_arm_j3
    - rpo_arm_j4
    - rpo_arm_j5
  motor_ids:
    rpo_arm_j1: 19
    rpo_arm_j2: 20
    rpo_arm_j3: 21
    rpo_arm_j4: 22
    rpo_arm_j5: 23
  motor_signs:
    rpo_arm_j1: -1
    rpo_arm_j2: 1
    rpo_arm_j3: 1
    rpo_arm_j4: -1
    rpo_arm_j5: 1
  kp:
    rpo_arm_j1: 40
    rpo_arm_j2: 40
    rpo_arm_j3: 40
    rpo_arm_j4: 30
    rpo_arm_j5: 20
  kd:
    rpo_arm_j1: 2.0
    rpo_arm_j2: 2.0
    rpo_arm_j3: 2.0
    rpo_arm_j4: 1.5
    rpo_arm_j5: 1.0
  zero_offsets_rad:
    rpo_arm_j1: 0.0
    rpo_arm_j2: 0.0
    rpo_arm_j3: 0.0
    rpo_arm_j4: 0.0
    rpo_arm_j5: 0.0
  joint_limits_rad:
    rpo_arm_j1: [-1.2, 1.2]
    rpo_arm_j2: [-1.2, 1.2]
    rpo_arm_j3: [-1.5, 1.5]
    rpo_arm_j4: [-1.8, 0.3]
    rpo_arm_j5: [-1.5, 1.5]
  max_relative_target_rad: 0.02
```

The joint limits above are placeholders. Replace them with measured or URDF-derived values.

## 6. Left arm config example

```yaml
robot:
  type: roboparty_5dof_arm_amazinghand_follower
  id: rpo_left_arm_ah_v1
  side: left

arm:
  type: rpo_5dof_arm
  can_interface: can2
  motor_type: DM
  master_id_offset: 16
  joint_names: [rpo_arm_j1, rpo_arm_j2, rpo_arm_j3, rpo_arm_j4, rpo_arm_j5]
  motor_ids:
    rpo_arm_j1: 14
    rpo_arm_j2: 15
    rpo_arm_j3: 16
    rpo_arm_j4: 17
    rpo_arm_j5: 18
  motor_signs:
    rpo_arm_j1: 1
    rpo_arm_j2: 1
    rpo_arm_j3: 1
    rpo_arm_j4: 1
    rpo_arm_j5: 1
  kp:
    rpo_arm_j1: 40
    rpo_arm_j2: 40
    rpo_arm_j3: 40
    rpo_arm_j4: 30
    rpo_arm_j5: 20
  kd:
    rpo_arm_j1: 2.0
    rpo_arm_j2: 2.0
    rpo_arm_j3: 2.0
    rpo_arm_j4: 1.5
    rpo_arm_j5: 1.0
```

## 7. AmazingHand action addition

With AmazingHand as the gripper, the first policy action should be:

```text
5 arm targets + 1 hand grasp scalar
```

LeRobot-facing flat action/observation keys:

```text
rpo_arm_j1.pos
rpo_arm_j2.pos
rpo_arm_j3.pos
rpo_arm_j4.pos
rpo_arm_j5.pos
amazinghand_grasp.pos
```

Arm joint keys are in degrees. `amazinghand_grasp.pos` is a scalar in `[0.0, 1.0]`.

Full dexterous action later, not v1:

```text
5 arm targets + 8 AmazingHand servo targets = 13D action
```

## 8. Conversion formulas

Raw motor position to LeRobot joint position:

```python
joint_rad = sign * (raw_motor_rad - zero_offset_rad)
```

LeRobot joint target to raw motor target:

```python
raw_target_rad = sign * joint_target_rad + zero_offset_rad
```

Always clamp:

```python
safe_target = clamp(joint_target_rad, min_rad, max_rad)
safe_target = clamp_delta(safe_target, current_joint_rad, max_relative_target_rad)
```

## 9. Validation table

Fill this before recording data.

| LeRobot joint | Motor ID | CAN | Sign | Physical joint | Zero OK? | Limit OK? |
|---|---:|---|---:|---|---|---|
| rpo_arm_j1 | 19 or 14 | can3/can2 | TBD | TBD | no | no |
| rpo_arm_j2 | 20 or 15 | can3/can2 | TBD | TBD | no | no |
| rpo_arm_j3 | 21 or 16 | can3/can2 | TBD | TBD | no | no |
| rpo_arm_j4 | 22 or 17 | can3/can2 | TBD | TBD | no | no |
| rpo_arm_j5 | 23 or 18 | can3/can2 | TBD | TBD | no | no |
| amazinghand_grasp | serial servos 1–8 | ttyUSB | n/a | hand | no | no |
```
