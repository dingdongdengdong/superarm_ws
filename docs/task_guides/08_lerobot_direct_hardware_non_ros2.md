# 08 - LeRobot Direct Hardware Custom Robot, Non-ROS2

## Goal

Build the next LeRobot custom robot preset for the **real RoboParty 5-DOF arm + AmazingHand hardware** without depending on ROS2.

This guide is for the direct hardware path:

```text
LeRobot CLI / policy / dataset tools
        ↓
Roboparty5DofArmAmazingHandConfig
        ↓
Roboparty5DofArmAmazingHandFollower Robot class
        ↓
RoboParty arm adapter over CAN / motor SDK
AmazingHand adapter over serial / SCS servo protocol
LeRobot camera adapters
```

This is different from the current SITL path:

```text
LeRobot local SITL wrapper
        ↓
ROS2 topics
        ↓
Isaac Sim scene
```

## Key decision

**LeRobot does not require ROS2.**

Official LeRobot custom-hardware architecture asks you to implement a robot abstraction:

- a robot config
- observation features
- action features
- connection/disconnection
- observation reading
- action sending
- calibration/configuration if needed

The transport inside that robot class is your choice. It can be CAN, serial, USB, vendor SDK, direct simulator API, ROS2, or a mock backend.

For this direct hardware preset, use direct hardware transports first:

```text
RoboParty arm: CAN / motor SDK / board-specific API
AmazingHand: serial / SCS servo protocol / hand SDK
Cameras: normal LeRobot camera config
```

Use ROS2 only if the real hardware bring-up stack already requires ROS2. Do not add ROS2 just because the current Isaac Sim SITL bridge uses it.

Official references:

- LeRobot Bring Your Own Hardware: <https://huggingface.co/docs/lerobot/en/integrate_hardware>
- LeRobotDataset v3.0: <https://huggingface.co/docs/lerobot/lerobot-dataset-v3>
- Local existing guide: `docs/task_guides/05_lerobot_custom_robot.md`
- Existing long architecture note: `lerobot_custom_config_whole_arm_hand_control.md`

---

## 1. Mental model for team members

### 1.1 Config does not move motors

A LeRobot config file stores hardware facts. It does not move hardware by itself.

Examples of hardware facts:

```text
which CAN interface to open
which motor IDs exist
which motor signs are reversed
which joint limits are safe
which serial port controls the hand
which hand servo IDs are safe to command
which cameras should be used
```

The `Robot` class uses those facts to connect to devices, read observations, clamp actions, and send commands.

### 1.2 One LeRobot robot should own the integrated system

Use one robot type for the whole arm-hand system:

```text
roboparty_5dof_arm_amazinghand_follower
```

Recommended Python class name:

```text
Roboparty5DofArmAmazingHandFollower
```

Recommended config class name:

```text
Roboparty5DofArmAmazingHandConfig
```

Why one robot type?

- One policy should output one action vector for the arm and hand.
- One dataset should record one synchronized state/action stream.
- Safety clamping should happen in one place before hardware moves.
- The hand grasp timing should be coordinated with the arm pose.

### 1.3 Keep the first direct-hardware contract simple

Start with six command/state features:

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
arm joints: degrees after calibration and sign correction
hand grasp: scalar in [0.0, 1.0]
```

Dataset and policy shape:

```text
observation.state shape = (6,)
action shape            = (6,)
```

Do not start with raw multi-servo hand control unless the hardware lead has already validated safe hand servo limits.

### 1.4 SITL feature names vs direct hardware feature names

The current SITL bridge uses simulation-oriented names:

```text
right_arm_pitch_joint.pos
right_arm_roll_joint.pos
right_arm_yaw_joint.pos
right_elbow_pitch_joint.pos
right_elbow_yaw_joint.pos
amazinghand_grasp.pos
```

The direct hardware guide uses hardware-oriented names:

```text
rpo_arm_j1.pos
rpo_arm_j2.pos
rpo_arm_j3.pos
rpo_arm_j4.pos
rpo_arm_j5.pos
amazinghand_grasp.pos
```

Do not mix them accidentally. If the team later wants one sim-to-real contract, create an explicit mapping table and migration plan.

---

## 2. Target file layout

The exact local LeRobot layout must be checked before implementation because this repo has an older local LeRobot checkout. The conceptual package should be one focused robot package:

```text
lerobot/...
└── robots/
    └── roboparty_5dof_arm_amazinghand/
        ├── __init__.py
        ├── config_roboparty_5dof_arm_amazinghand.py
        ├── roboparty_5dof_arm_amazinghand.py
        ├── arm_can_adapter.py
        ├── amazinghand_serial_adapter.py
        └── safety.py
```

Responsibilities:

| File | Responsibility |
|---|---|
| `config_roboparty_5dof_arm_amazinghand.py` | Dataclass/config schema for arm, hand, limits, signs, cameras. |
| `roboparty_5dof_arm_amazinghand.py` | LeRobot `Robot` implementation that owns adapters and exposes features. |
| `arm_can_adapter.py` | Low-level RoboParty arm read/write through CAN or motor SDK. |
| `amazinghand_serial_adapter.py` | Low-level AmazingHand open/close or servo command transport. |
| `safety.py` | Clamping, relative target limit, sign application, unit conversion helpers. |
| tests | Mock-only tests first; hardware tests only after safety gates. |

Implementation must adapt imports to the actual local LeRobot version. The official docs explain the architecture, but local import paths may differ.

---

## 3. Config fields

Minimum direct hardware config fields:

| Field | Example | Meaning |
|---|---|---|
| `id` | `rpo_right_arm_ah_v1` | Stable robot identity used for calibration/data. |
| `side` | `right` | Which arm side this config controls. |
| `port` | `can3` | CAN interface or arm bus identifier. |
| `motor_ids` | `[1, 2, 3, 4, 5]` | RoboParty arm motor IDs in feature order. |
| `motor_signs` | `[1, -1, 1, 1, -1]` | Direction correction from raw motor readings to LeRobot features. |
| `calibration_offsets` | `[0, 0, 0, 0, 0]` | Zero offsets in degrees. |
| `joint_limits` | `[[-90, 90], ...]` | Safe software limits, stricter than mechanical limits. |
| `max_relative_target` | `2.0` | Maximum degrees per command step. |
| `hand_enabled` | `true` | Allows disabling hand while testing arm. |
| `hand_serial_port` | `/dev/ttyUSB_AH_RIGHT` | Serial device for AmazingHand controller. |
| `hand_servo_ids` | `[1, 2, 3, 4, 5, 6, 7, 8]` | Raw hand servo IDs, even if first policy uses one scalar. |
| `hand_safe_limits` | `{open: ..., closed: ...}` | Safe hand command range or per-servo bounds. |
| `cameras` | LeRobot camera config | Front/wrist cameras if installed. |
| `mock` | `false` | Mock backend for tests without hardware. |

Example YAML-style preset:

```yaml
robot:
  type: roboparty_5dof_arm_amazinghand_follower
  id: rpo_right_arm_ah_v1
  side: right
  port: can3
  motor_ids: [1, 2, 3, 4, 5]
  motor_signs: [1, 1, 1, 1, 1]
  calibration_offsets: [0.0, 0.0, 0.0, 0.0, 0.0]
  joint_limits:
    rpo_arm_j1.pos: [-60.0, 60.0]
    rpo_arm_j2.pos: [-45.0, 45.0]
    rpo_arm_j3.pos: [-90.0, 90.0]
    rpo_arm_j4.pos: [-75.0, 75.0]
    rpo_arm_j5.pos: [-90.0, 90.0]
  max_relative_target: 2.0
  hand_enabled: true
  hand_serial_port: /dev/ttyUSB_AH_RIGHT
  hand_servo_ids: [1, 2, 3, 4, 5, 6, 7, 8]
  hand_safe_limits:
    grasp_min: 0.0
    grasp_max: 1.0
  mock: false
```

All numbers above are placeholders for software structure only. The hardware lead must replace limits and signs with measured safe values before real motion.

---

## 4. Required LeRobot robot behavior

### 4.1 `observation_features`

Expose hardware-level feature names:

```python
{
    "rpo_arm_j1.pos": float,
    "rpo_arm_j2.pos": float,
    "rpo_arm_j3.pos": float,
    "rpo_arm_j4.pos": float,
    "rpo_arm_j5.pos": float,
    "amazinghand_grasp.pos": float,
}
```

If cameras are configured, add camera features through the normal LeRobot camera path.

### 4.2 `action_features`

Start with the same six controllable features:

```python
{
    "rpo_arm_j1.pos": float,
    "rpo_arm_j2.pos": float,
    "rpo_arm_j3.pos": float,
    "rpo_arm_j4.pos": float,
    "rpo_arm_j5.pos": float,
    "amazinghand_grasp.pos": float,
}
```

### 4.3 `connect()`

`connect()` must:

```text
open CAN / motor SDK connection
open hand serial connection if hand_enabled is true
connect configured cameras
load calibration data if available
set is_connected true only after required devices are available
```

### 4.4 `get_observation()`

`get_observation()` must:

```text
read raw arm motor positions
convert raw units to degrees
apply calibration offsets
apply motor signs
read or estimate hand grasp scalar
capture camera frames if configured
return flat LeRobot observation dict
```

### 4.5 `send_action(action)`

`send_action(action)` must:

```text
read six flat action keys
clamp each arm target to joint_limits
clamp each arm target to max_relative_target from current position
convert degrees back to raw motor units
apply inverse motor signs
send arm command through CAN / motor SDK
clip amazinghand_grasp.pos to [0.0, 1.0]
send hand command through serial / hand SDK if enabled
return the clipped action actually sent
```

The return value matters. Dataset/debug code should be able to see what was actually sent after safety clamps.

---

## 5. Tiny implementation tasks for team distribution

### Task DH01 - Local LeRobot interface inspection

**Purpose:** Confirm the exact local import paths and base-class method names before coding.

**Files:**

- Inspect local `lerobot` package.
- Update this guide if import paths differ.

**Steps:**

```bash
grep -R "class Robot" -n lerobot | head -20
grep -R "class RobotConfig" -n lerobot | head -20
grep -R "observation_features" -n lerobot | head -20
```

**Done when:** Team knows the exact base class/import paths for this checkout.

---

### Task DH02 - Config class skeleton

**Purpose:** Create the direct-hardware config dataclass.

**Deliverable:** `Roboparty5DofArmAmazingHandConfig` with all fields from section 3.

**Tests:** Mock config can instantiate without hardware.

---

### Task DH03 - Safety helper tests

**Purpose:** Lock clamping behavior before hardware adapters exist.

**Test cases:**

```text
hand grasp -1.0 -> 0.0
hand grasp 2.0 -> 1.0
joint target above limit -> upper limit
joint target below limit -> lower limit
relative target jump above max_relative_target -> clipped step
```

**Done when:** Safety tests pass with no hardware connected.

---

### Task DH04 - Mock arm adapter

**Purpose:** Let developers test the robot class without moving hardware.

**Behavior:**

```text
connect() marks mock arm connected
read_positions() returns last commanded safe positions
write_positions() stores safe positions
```

**Done when:** Mock arm adapter passes unit tests.

---

### Task DH05 - Mock hand adapter

**Purpose:** Let developers test hand grasp clamping without serial hardware.

**Behavior:**

```text
connect() marks mock hand connected
read_grasp() returns last commanded scalar
write_grasp() stores clipped scalar
```

**Done when:** Mock hand adapter passes open/close tests.

---

### Task DH06 - Robot class feature properties

**Purpose:** Expose the six direct-hardware features to LeRobot.

**Done when:** `observation_features` and `action_features` contain exactly the six baseline keys.

---

### Task DH07 - Robot `connect()` and `disconnect()` in mock mode

**Purpose:** Test lifecycle without hardware.

**Done when:** Repeated connect/disconnect does not leak state and `is_connected` is correct.

---

### Task DH08 - Robot `get_observation()` in mock mode

**Purpose:** Prove the robot returns all six flat observation keys.

**Done when:** Mock observation contains all six keys and values are floats.

---

### Task DH09 - Robot `send_action()` in mock mode

**Purpose:** Prove LeRobot action dicts are clamped and routed correctly.

**Done when:** Mock send returns clipped action and updates mock observation.

---

### Task DH10 - No-op smoke test

**Purpose:** First safe behavior before motion.

**Test:**

```python
obs = robot.get_observation()
action = {key: obs[key] for key in robot.action_features}
sent = robot.send_action(action)
assert sent == action
```

**Done when:** No-op action causes no target jump.

---

### Task DH11 - Tiny arm delta test

**Purpose:** Validate one small commanded movement after safety approval.

**Mock test first:**

```text
rpo_arm_j1.pos += 0.5 degrees
hand stays open
```

**Hardware version:** Only after emergency stop and direction checks.

---

### Task DH12 - Tiny hand open/close test

**Purpose:** Validate hand scalar path.

**Mock test first:**

```text
amazinghand_grasp.pos = 0.0
amazinghand_grasp.pos = 0.5
amazinghand_grasp.pos = 1.0
```

**Hardware version:** Only after hand safe limits are reviewed.

---

### Task DH13 - Dataset schema check

**Purpose:** Ensure direct hardware dataset remains six-dimensional.

**Done when:** First debug dataset has:

```text
observation.state shape = (6,)
action shape = (6,)
```

---

### Task DH14 - Direct hardware recording command doc

**Purpose:** Give operators a command template for later recording.

**Template:**

```bash
lerobot-record \
  --robot.type=roboparty_5dof_arm_amazinghand_follower \
  --robot.id=rpo_right_arm_ah_v1 \
  --robot.port=can3 \
  --robot.hand_serial_port=/dev/ttyUSB_AH_RIGHT \
  --repo-id=<HF_USER>/rpo5_ah_cube_tray_debug_v1 \
  --fps=30 \
  --num-episodes=10
```

Adjust CLI flags to the exact local LeRobot version.

---

## 6. Safety gates before real hardware movement

Do not move real hardware until these are true:

```text
[ ] Emergency stop works.
[ ] Power cut works.
[ ] Correct CAN interface is known.
[ ] Correct motor IDs are known.
[ ] Arm can be disabled quickly.
[ ] Hand can be disabled quickly.
[ ] Software joint limits are stricter than mechanical limits.
[ ] Relative target clamp is tested in mock mode.
[ ] Hand scalar clamp is tested in mock mode.
[ ] Direction check plan exists for one joint at a time.
[ ] Operator and safety observer agree on stop procedure.
```

Do not replay policy on real hardware until these are true:

```text
[ ] Low-speed direction checks passed for all five arm joints.
[ ] Hand open/close scalar is verified at safe speed/current.
[ ] No-op smoke test passed on hardware.
[ ] Tiny joint delta passed on hardware.
[ ] Tiny hand open/close test passed on hardware.
[ ] Dataset debug episode can be recorded without unexpected jumps.
```

---

## 7. What this guide intentionally does not do

This guide does not implement:

- ROS2 nodes.
- Isaac Sim bridge code.
- Learned policy training.
- Raw 8-servo AmazingHand policy output.
- Real motor IDs, signs, or limits.
- Any command that moves hardware immediately.

Those require separate implementation and hardware-lead review.

---

## 8. Done when

```text
[ ] Team understands direct LeRobot hardware path is non-ROS2 by default.
[ ] Direct hardware feature contract is documented.
[ ] Config fields are documented.
[ ] Required robot behavior is documented.
[ ] Tiny implementation tasks are documented.
[ ] Safety gates are documented.
[ ] This guide is linked from the task guide index.
```
