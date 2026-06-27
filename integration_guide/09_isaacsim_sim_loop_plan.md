# 09 — Isaac Sim 5.1 Sim-in-the-Loop Plan (RoboParty Arm + AmazingHand)

## Goal

Replace the Franka Panda placeholder in the `tasks/lerobot-isaacsim-arm-test` testbed with
the actual RoboParty 5-DOF arm + AmazingHand, and wire the LeRobot side to use the same
joint names and flat feature keys as the real hardware.

When complete:
- Isaac Sim 5.1 container simulates the actual robot geometry and physics
- NVIDIA-ROS container runs LeRobot with `robot_type=isaacsim_rpo_arm`
- Flat feature keys in recorded datasets match real hardware exactly:
  `rpo_arm_j1.pos` … `rpo_arm_j5.pos` + `amazinghand_grasp.pos` (6D)
- Switching from sim to real hardware later requires only changing the robot type string
  and CAN/serial port config — datasets and policies are reusable

---

## Geometry: two-track approach

The source files `arm_with_hand_with_robot_file/echo_full.step` and `echo_full.stl` are a
single assembled mesh. Isaac Sim needs a per-link articulated model.

### Track A — Scripted URDF (done first, no GUI)

Create `isaacsim_test/isaacsim/rpo_arm.urdf`:
- 6 links: `base_link`, `link_1` … `link_5`, `gripper_link`
- 5 revolute joints: `rpo_arm_j1` … `rpo_arm_j5`
- 1 prismatic joint: `amazinghand_grasp` (range `[0.0, 1.0]`)
- Visuals: cylinder/box primitives per link — no STL dependency, loads headlessly immediately
- Inertia seed values from `03_motor_can_mapping.md`:
  - Joints 1–3: ~0.1 kg·m², joints 4–5: ~0.05 kg·m², gripper: ~0.01 kg·m²
- Joint limits from `03_motor_can_mapping.md` section 5 as initial placeholders

### Track B — Proper USD from STEP (visual upgrade, done later interactively)

1. Open Isaac Sim 5.1 GUI → File → Import → STEP (`echo_full.step`)
2. Isaac Sim STEP importer splits multi-body STEP into separate prims
3. In USD Composer: add `ArticulationRoot` + `RevoluteJoint` prims between link prims
4. Set joint names to `rpo_arm_j1` … `rpo_arm_j5` + `amazinghand_grasp`
5. Save as `isaacsim_test/isaacsim/rpo_arm.usd`
6. Update scene script to load USD directly (skip URDF importer)

Track A is the prerequisite. Track B is a drop-in visual upgrade — scene script and
LeRobot config stay identical.

---

## Implementation steps

### 1. `isaacsim_test/isaacsim/rpo_arm.urdf` (new)

URDF with 6 links, 5 revolute + 1 prismatic joint.
Joint axis directions and origins: rough estimates initially — verify against real hardware
before recording real data.

### 2. `isaacsim_test/isaacsim/setup_rpo_arm_scene.py` (new)

Copy of `setup_openarm_scene.py` with:
- `URDF_CANDIDATES` first priority: `/workspace/isaacsim/rpo_arm.urdf`
- `dest_path` → `/World/RpoArm`
- Node name → `"isaac_sim_rpo_arm_bridge"`
- Print tag → `[setup_rpo_arm_scene]`

ROS2 topics unchanged:
- Publishes: `/follower/joint_states` (JointState, ~60 Hz)
- Subscribes: `/follower/joint_commands` (Float64MultiArray, 6 floats)

### 3. `isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py` (new)

`IsaacSimRpoArmConfig`:
```python
joint_names = ["rpo_arm_j1", "rpo_arm_j2", "rpo_arm_j3",
               "rpo_arm_j4", "rpo_arm_j5", "amazinghand_grasp"]
joint_state_topic = "/follower/joint_states"
joint_command_topic = "/follower/joint_commands"
phone_command_topic = "/leader/joint_commands"
```

`IsaacSimRpoArmRobot` (robot_type = `"isaacsim_rpo_arm"`):
- Same ROS2 bridge pattern as `IsaacSimOpenArmRobot`
- `features` emits flat keys: `rpo_arm_j1.pos` … `amazinghand_grasp.pos`, each `shape=(1,)`
- `capture_observation()` maps joint index → flat key
- `teleop_step()` / `send_action()` collapse flat keys → `Float64MultiArray`

### 4. `isaacsim_test/lerobot/rpo_arm_isaacsim.yaml` (new)

```yaml
_type: isaacsim_rpo_arm
joint_names: [rpo_arm_j1, rpo_arm_j2, rpo_arm_j3, rpo_arm_j4, rpo_arm_j5, amazinghand_grasp]
joint_state_topic: /follower/joint_states
joint_command_topic: /follower/joint_commands
phone_command_topic: /leader/joint_commands
connect_timeout_s: 10.0
mock: false
```

### 5. `lerobot/lerobot/common/robot_devices/robots/utils.py` (modify)

Add in `make_robot_config()` after the `isaacsim_openarm` block:
```python
elif robot_type == "isaacsim_rpo_arm":
    import sys as _sys, os as _os
    _sys.path.insert(0, _os.path.join(_os.path.dirname(__file__),
        "../../../../../../isaacsim_test/lerobot"))
    from isaacsim_rpo_arm_robot import IsaacSimRpoArmConfig
    return IsaacSimRpoArmConfig(**kwargs)
```

Add in `make_robot_from_config()` before the fallback `else`:
```python
elif hasattr(config, "joint_state_topic") and \
        config.__class__.__name__ == "IsaacSimRpoArmConfig":
    import sys as _sys, os as _os
    _sys.path.insert(0, _os.path.join(_os.path.dirname(__file__),
        "../../../../../../isaacsim_test/lerobot"))
    from isaacsim_rpo_arm_robot import IsaacSimRpoArmRobot
    return IsaacSimRpoArmRobot(config)
```

### 6. `isaacsim_test/docker-compose.yml` (modify)

`isaac-sim-51` service command:
```yaml
command: >
  source /opt/ros/humble/setup.bash &&
  exec /isaac-sim/python.sh /workspace/isaacsim/setup_rpo_arm_scene.py
```

`lerobot` service environment — add:
```yaml
NUM_JOINTS: "6"
```

### 7. `isaacsim_test/.env` (modify)

```
OPENARM_URDF_PATH=/workspace/isaacsim/rpo_arm.urdf
```

### 8. `isaacsim_test/lerobot/run_smartphone_teleop.sh` (modify)

- `NUM_JOINTS` default → `6`
- `--robot.type=isaacsim_openarm` → `--robot.type=isaacsim_rpo_arm`
- `REPO_ID` default → `local/rpo_arm_isaacsim_v1`
- Print header → `RoboParty Arm + AmazingHand Smartphone Teleop`

### 9. `isaacsim_test/README.md` (modify)

- Title → `Isaac Sim 5.1 — RoboParty Arm + AmazingHand Sim-in-the-Loop`
- Update robot type references: `isaacsim_openarm` → `isaacsim_rpo_arm`
- Update expected startup log: 6 joints with `rpo_arm_*` names
- Add geometry upgrade path section (Track B)

---

## Files summary

| Action | File |
|--------|------|
| Create | `isaacsim_test/isaacsim/rpo_arm.urdf` |
| Create | `isaacsim_test/isaacsim/setup_rpo_arm_scene.py` |
| Create | `isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py` |
| Create | `isaacsim_test/lerobot/rpo_arm_isaacsim.yaml` |
| Modify | `isaacsim_test/lerobot/run_smartphone_teleop.sh` |
| Modify | `isaacsim_test/docker-compose.yml` |
| Modify | `isaacsim_test/.env` |
| Modify | `lerobot/lerobot/common/robot_devices/robots/utils.py` |
| Modify | `isaacsim_test/README.md` |

Old `setup_openarm_scene.py`, `isaacsim_robot.py`, and `openarm_isaacsim.yaml` are kept
as reference — the `isaacsim_openarm` robot type still works for Franka/OpenArm.

---

## Verification checklist

```bash
# 1. Isaac Sim starts with correct joints
docker compose up isaac-sim-51
# → [setup_rpo_arm_scene] Loaded 6 joints: ['rpo_arm_j1', 'rpo_arm_j2', 'rpo_arm_j3',
#                                             'rpo_arm_j4', 'rpo_arm_j5', 'amazinghand_grasp']

# 2. ROS2 topics visible at 60 Hz
export ROS_DOMAIN_ID=42 && source /opt/ros/humble/setup.bash
ros2 topic hz /follower/joint_states   # → ~60 Hz

# 3. LeRobot connects
docker compose up lerobot
# → [IsaacSimRpoArmRobot] Connected. Joints: [rpo_arm_j1, ...]

# 4. Command round-trip
ros2 topic pub /leader/joint_commands std_msgs/msg/Float64MultiArray \
  "data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]" --once
ros2 topic echo /follower/joint_states --once
# → position[0] close to 0.1

# 5. Recording produces correct dataset keys
docker exec isaacsim-test-lerobot bash -c '
  cd /workspace/superarm_ws/lerobot &&
  python lerobot/scripts/control_robot.py \
    --robot.type=isaacsim_rpo_arm \
    --control.type=record \
    --control.repo_id=local/rpo_arm_isaacsim_v1 \
    --control.num_episodes=3 --control.fps=30'
# → dataset keys: rpo_arm_j1.pos … amazinghand_grasp.pos, state/action shape (6,)
```

---

## Relation to other guides

- `00_scope_correction_5dof_rpo_arm.md` — defines target robot and milestones
- `03_motor_can_mapping.md` — joint names, signs, limits, gains used here
- `04_lerobot_custom_robot_skeleton.md` — flat feature key contract (`rpo_arm_j*.pos`)
- `06_dataset_policy_workflow.md` — recording and training workflow after this is working
- `07_validation_checklist.md` — Gate B (LeRobot ready) applies once sim testbed is live
