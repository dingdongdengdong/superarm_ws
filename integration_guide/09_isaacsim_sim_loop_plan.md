# 09 — Isaac Sim 5.1 Sim-in-the-Loop Plan (RoboParty V2.0 Right Arm + AmazingHand)

## Goal

Use the official RoboParty / Roboto Origin **V2.0** URDF from the RoboParty GitHub checkout for the Isaac Sim testbed instead of a handmade arm URDF.

The LeRobot sim contract is 6D:

```text
right_arm_pitch_joint.pos
right_arm_roll_joint.pos
right_arm_yaw_joint.pos
right_elbow_pitch_joint.pos
right_elbow_yaw_joint.pos
amazinghand_grasp.pos
```

The first five names come directly from the official V2.0 URDF. `amazinghand_grasp` is a synthetic scalar in `[0.0, 1.0]` until a physical AmazingHand URDF/USD mount is added.

---

## Source geometry

Use:

```text
roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf
roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/meshes/
```

Do **not** use the old primitive `isaacsim_test/isaacsim/rpo_arm.urdf` approach. The official V2.0 URDF already includes the right-arm chain and meshes:

```text
right_arm_pitch_joint
right_arm_roll_joint
right_arm_yaw_joint
right_elbow_pitch_joint
right_elbow_yaw_joint
```

---

## Implementation contract

- `setup_rpo_arm_scene.py` imports the full official V2.0 URDF.
- Isaac Sim controls only the five official right-arm DOFs.
- ROS2 `/follower/joint_states` publishes six names: the five right-arm joints plus synthetic `amazinghand_grasp`.
- ROS2 `/follower/joint_commands` accepts six floats; the first five are applied to the right arm, the sixth is clipped to `[0.0, 1.0]` and mirrored into joint state.
- LeRobot `robot.type=isaacsim_rpo_arm` records `observation.state` and `action` with shape `(6,)` and the feature names above.

---

## Verification checklist

```bash
# Static contract
python3 isaacsim_test/test_v2_roboparty_config.py

# Compose syntax
cd isaacsim_test && docker compose config >/tmp/isaacsim-compose.yml

# Isaac Sim startup
docker compose up isaac-sim-51
# expect: Loading RoboParty V2.0 URDF ... roboto_origin.urdf
# expect: Controlled LeRobot joints: ['right_arm_pitch_joint', ..., 'amazinghand_grasp']

# Command round-trip
ros2 topic pub /leader/joint_commands std_msgs/msg/Float64MultiArray \
  "data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.5]" --once
ros2 topic echo /follower/joint_states --once
```

---

## Later work

- Add a proper AmazingHand URDF/USD mount to `right_elbow_yaw_link` or a dedicated wrist adapter.
- Replace the synthetic `amazinghand_grasp` scalar with real hand state/control if multi-DOF hand simulation becomes necessary.
- Move the LeRobot custom robot shim into a separate installable `lerobot_rpo_arm` package if upstream cleanliness becomes more important than local test speed.
