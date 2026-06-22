# 04 - AmazingHand Integration

## Goal

Mount and control AmazingHand as a safe scalar gripper first, then reserve raw
8-servo dexterity for later work.

## First control surface

Expose one LeRobot feature:

```text
amazinghand_grasp.pos
```

Meaning:

```text
0.0 = open
1.0 = close / power grasp
```

The AmazingHand adapter converts that scalar to eight servo targets.

## Mechanical tasks

```text
[ ] Measure RoboParty wrist mounting pattern.
[ ] Measure AmazingHand mounting pattern.
[ ] Design wrist-to-hand adapter.
[ ] Confirm cable exit direction.
[ ] Check hand mass and wrist torque margin.
[ ] Print or machine first adapter.
[ ] Mount hand and verify no collision in neutral pose.
```

## Electrical tasks

```text
[ ] Confirm AmazingHand servo voltage.
[ ] Confirm serial bus adapter.
[ ] Assign or verify all 8 servo IDs.
[ ] Add strain relief for serial and power wiring.
[ ] Add fuse or current protection.
[ ] Label hand power and serial connectors.
```

## Software smoke test

Use the official AmazingHand Python example first. Then implement only these
three adapter operations:

```python
connect()
set_grasp_scalar(value: float)
disconnect()
```

Clamp every command:

```python
value = max(0.0, min(1.0, float(value)))
```

## Servo mapping document

Create:

```text
docs/task_guides/amazinghand_servo_map.md
```

Use this table:

```markdown
| Servo ID | Finger | Open target | Closed target | Safe min | Safe max | Verified |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | thumb/index/middle/ring | measured open target | measured closed target | measured safe min | measured safe max | yes/no |
```

## Done when

```text
[ ] Hand opens and closes from Python.
[ ] Servo IDs are documented.
[ ] Open and closed targets are documented.
[ ] Scalar command is clamped to [0.0, 1.0].
[ ] Hand can hold the foam cube without overloading servos.
[ ] Hand cables do not collide with the wrist or table.
```
