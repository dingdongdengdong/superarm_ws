# 07 — Validation Checklist

Use this checklist before moving from hardware bring-up to data collection and policy inference.

## 1. Hardware safety

- [ ] Arm is bolted or clamped to a stable base.
- [ ] Emergency stop cuts actuator power.
- [ ] Software disable command works.
- [ ] Power supply current limit is reasonable.
- [ ] No exposed wires can be pulled by arm motion.
- [ ] CAN wires are strain relieved.
- [ ] Operator can stand outside the reachable workspace.
- [ ] Gripper force is limited for first tests.

## 2. CAN and motor validation

For each motor:

- [ ] Motor ID known.
- [ ] CAN interface known.
- [ ] Motor model known.
- [ ] Position can be read.
- [ ] Torque can be disabled.
- [ ] Small position command works.
- [ ] Temperature/current status readable if supported.

Table:

| Joint | Motor ID | CAN | Read OK | Disable OK | Small move OK | Notes |
|---|---:|---|---|---|---|---|
| joint_1 | TBD | TBD | no | no | no | |
| joint_2 | TBD | TBD | no | no | no | |
| joint_3 | TBD | TBD | no | no | no | |
| joint_4 | TBD | TBD | no | no | no | |
| joint_5 | TBD | TBD | no | no | no | |
| gripper | TBD | TBD | no | no | no | |

## 3. Joint sign validation

For each joint:

- [ ] Positive command direction is known.
- [ ] Positive command matches URDF convention, or sign conversion is documented.
- [ ] LeRobot observation increases when the physical joint moves in the positive direction.
- [ ] LeRobot action increase moves the physical joint in the positive direction.

Procedure:

```text
1. Put robot in safe pose.
2. Move one joint by hand if backdrivable, or command +0.5 to +1.0 deg slowly.
3. Observe sign in the matching flat key, for example rpo_arm_j1.pos.
4. Fix config before testing next joint.
```

## 4. Zero/calibration validation

- [ ] Known zero pose defined in a photo or diagram.
- [ ] Raw motor positions saved at zero pose.
- [ ] `zero_offsets_rad` saved.
- [ ] Reboot robot and verify zero is repeatable.
- [ ] Calibration file is versioned.
- [ ] Calibration file is tied to robot ID.

## 5. Joint limits

For each joint:

- [ ] Mechanical limit known.
- [ ] Software min/max set inside mechanical limit.
- [ ] First test uses smaller temporary limit.
- [ ] Limit clamp tested.
- [ ] Max relative target tested.

Recommended first test:

```text
software limit = 50–70% of true mechanical range
max delta      = 0.5–1.0 deg per command
speed          = 10–20% normal
```

## 6. LeRobot smoke tests

- [ ] `robot.connect()` works.
- [ ] `robot.get_observation()` returns stable values.
- [ ] flat feature keys match the 6D contract.
- [ ] camera image keys are correct.
- [ ] dry-run `send_action()` prints expected targets.
- [ ] real `send_action()` moves one joint slowly.
- [ ] robot disconnect disables torque or enters safe mode.

## 7. Camera validation

- [ ] Front camera sees object, gripper, and placement area.
- [ ] Wrist camera, if used, sees object during grasp.
- [ ] Camera FPS is stable.
- [ ] Exposure is stable.
- [ ] No camera is mirrored unexpectedly.
- [ ] Camera names match dataset keys.

## 8. Teleoperation validation

Leader arm or XR teleop:

- [ ] Teleop enable/disable works.
- [ ] Reset reference works.
- [ ] Gripper open/close works.
- [ ] No large jump occurs when enabling.
- [ ] Teleop action shape equals policy action shape.
- [ ] Teleop commands are clamped by robot safety layer.

## 9. Dataset QA

After recording 10 debug episodes:

- [ ] Dataset loads.
- [ ] Videos play.
- [ ] State/action arrays have expected shape.
- [ ] State/action values are physically plausible.
- [ ] No NaNs.
- [ ] No frame drops causing large action jumps.
- [ ] Task string is correct.
- [ ] Episode success/failure is tracked.

## 10. Policy evaluation safety

Before running a trained policy:

- [ ] Run policy in dry-run mode and inspect action range.
- [ ] Run with motors disabled if possible.
- [ ] Run with reduced max delta.
- [ ] Keep object lightweight.
- [ ] Keep workspace clear.
- [ ] One person watches emergency stop.
- [ ] Stop after any unexpected oscillation.

## 11. Common blockers

| Blocker | Symptom | Fix |
|---|---|---|
| Wrong joint order | policy moves wrong joint | freeze order and regenerate dataset |
| Wrong zero | arm jumps at enable | recalibrate zero pose |
| Wrong sign | teleop mirrored or unstable | flip sign in config |
| Bad CAN | dropped frames or timeout | verify bitrate, termination, txqueuelen, power |
| Bad camera | policy misses object | fix viewpoint/exposure before more data |
| Data mismatch | ACT loss trains but robot fails | inspect action/state/video alignment |
| Unstable XR IK | jerky demos | record joint-space leader demos first |

## 12. Go/no-go gates

### Gate A — Hardware ready

Proceed only when:

```text
all motors read correctly
all motors disable correctly
single-joint small commands pass
emergency stop tested
```

### Gate B — LeRobot ready

Proceed only when:

```text
lerobot-style get_observation works
lerobot-style send_action works
state/action order is final
camera keys are final
```

### Gate C — Dataset ready

Proceed only when:

```text
10 debug episodes pass QA
no sign/order/zero bugs remain
front camera is useful
teleop is repeatable
```

### Gate D — Policy ready

Proceed only when:

```text
ACT output range is inspected
dry-run policy looks reasonable
safety limits are reduced
manual stop is ready
```
