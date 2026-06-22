# Can a LeRobot Custom Config Control the Whole RoboParty Arm + AmazingHand System?

Short answer: **yes**, but not by the config file alone.

A LeRobot custom config can describe the whole integrated robot, and a matching custom `Robot` class can control the whole integrated hardware. For your target hardware, the correct mental model is:

```text
LeRobot CLI / policy / dataset tools
        ↓
Roboparty5DofArmAmazingHandConfig
        ↓
Roboparty5DofArmAmazingHandFollower Robot class
        ↓
 ┌──────────────────────┬──────────────────────────┐
 │ RoboParty 5-DOF arm  │ AmazingHand dexterous hand │
 │ CAN / DM motors      │ Serial / SCS0009 servos    │
 └──────────────────────┴──────────────────────────┘
```

The config does not magically move motors. The config stores the hardware facts. The `Robot` class and hardware adapters use those facts to read sensors and send commands.

---

## 1. Final verdict

| Question | Answer |
|---|---|
| Can one LeRobot robot type represent the whole arm-hand integrated hardware? | **Yes.** Use one robot type: `roboparty_5dof_arm_amazinghand_follower`. |
| Can one config include both the arm and the hand? | **Yes.** Put CAN, motor IDs, signs, hand serial port, servo IDs, limits, and cameras in one config. |
| Can one policy output commands for both arm and hand? | **Yes.** First use 6D action: 5 arm joints + 1 hand grasp scalar. Later use 13D: 5 arm joints + 8 hand servos. |
| Does LeRobot care that the arm is CAN and the hand is serial? | **No.** If your `Robot` class hides it behind `get_observation()` and `send_action()`, LeRobot only sees features/actions. |
| Does the integration guide already include all low-level motor code? | **No.** The guide gives the architecture and skeleton. You still need real CAN read/write and real AmazingHand serial commands. |
| Is OpenArm the target robot? | **No.** OpenArm should only be used as a coding reference. The target is RoboParty 5-DOF arm + AmazingHand. |

So the correct answer is:

```text
Yes: LeRobot can control the whole integrated system if you implement one custom Robot class that owns both the RoboParty arm adapter and AmazingHand adapter.

No: a custom config alone is not enough. The CAN motor adapter, serial hand adapter, calibration, limits, and safety layer must actually work.
```

---

## 2. Your recommended first control contract

From the integration guide, your first working system should not expose all 8 AmazingHand servos directly. Start simple:

```text
5 RoboParty arm joint targets + 1 AmazingHand grasp scalar
```

Action order:

```text
[
  rpo_arm_j1,
  rpo_arm_j2,
  rpo_arm_j3,
  rpo_arm_j4,
  rpo_arm_j5,
  amazinghand_grasp,
]
```

Meaning:

```text
rpo_arm_j1..j5       = calibrated arm joint targets
amazinghand_grasp    = 0.0 open, 1.0 closed/grasp
```

Dataset/policy shape:

```text
observation.state shape = (6,)
action shape            = (6,)
```

Later, after this baseline works:

```text
Stage 1: 5 arm joints + 1 grasp scalar       = 6D action
Stage 2: 5 arm joints + hand pattern controls = maybe 8D action
Stage 3: 5 arm joints + 8 raw hand servos     = 13D action
```

Do **not** start with 13D raw servo control unless you already have safe hand calibration and a reason to need finger-level dexterity.

---

## 3. Important LeRobot version detail in this repo

Your integration guide uses skeleton imports like this:

```python
from lerobot.common.robots.configs import RobotConfig
from lerobot.common.robots.robot import Robot
```

But in the LeRobot repo currently present at:

```text
/Users/dong/Downloads/integrated _arm_proj/lerobot
```

the actual paths are:

```python
from lerobot.robots.config import RobotConfig
from lerobot.robots.robot import Robot
```

The current base class requires these members on a robot subclass:

```python
class Roboparty5DofArmAmazingHandFollower(Robot):
    config_class = Roboparty5DofArmAmazingHandConfig
    name = "roboparty_5dof_arm_amazinghand_follower"

    @property
    def observation_features(self) -> dict: ...

    @property
    def action_features(self) -> dict: ...

    @property
    def is_connected(self) -> bool: ...

    def connect(self, calibrate: bool = True) -> None: ...
    def disconnect(self) -> None: ...

    @property
    def is_calibrated(self) -> bool: ...

    def calibrate(self) -> None: ...
    def configure(self) -> None: ...

    def get_observation(self) -> dict: ...
    def send_action(self, action: dict) -> dict: ...
```

This matters because the guide's conceptual design is right, but you should adapt the import paths and interface shape to this LeRobot checkout.

---

## 4. How LeRobot sees the integrated robot

In the current LeRobot implementation, the hardware `Robot` usually returns a flat dictionary of hardware features, not directly an `observation.state` vector.

For example, your robot can expose raw hardware features like this:

```python
observation_features = {
    "rpo_arm_j1.pos": float,
    "rpo_arm_j2.pos": float,
    "rpo_arm_j3.pos": float,
    "rpo_arm_j4.pos": float,
    "rpo_arm_j5.pos": float,
    "amazinghand_grasp.pos": float,
    "front": (480, 640, 3),
    "wrist": (480, 640, 3),
}

action_features = {
    "rpo_arm_j1.pos": float,
    "rpo_arm_j2.pos": float,
    "rpo_arm_j3.pos": float,
    "rpo_arm_j4.pos": float,
    "rpo_arm_j5.pos": float,
    "amazinghand_grasp.pos": float,
}
```

Then LeRobot's dataset utility converts those flat motor features into standard dataset vectors:

```text
raw robot observation dict
        ↓
LeRobot build_dataset_frame / feature utilities
        ↓
observation.state = [rpo_arm_j1.pos, ..., amazinghand_grasp.pos]
action            = [rpo_arm_j1.pos, ..., amazinghand_grasp.pos]
```

So both statements are true:

```text
Inside your Robot class: use flat dict keys like "rpo_arm_j1.pos".
Inside the dataset/policy: LeRobot packs them into observation.state/action vectors.
```

If you choose to return `"observation.state"` directly from `get_observation()`, you need a custom processor path and matching features. The simpler path in this repo is to follow existing robots like `openarm_follower`: expose flat joint keys and let LeRobot build the dataset vectors.

---

## 5. What the custom config should contain

The config should describe **both** subsystems.

### 5.1 Arm section

Right-arm starting point from your guide:

```yaml
robot:
  type: roboparty_5dof_arm_amazinghand_follower
  id: rpo_right_arm_ah_v1
  side: right

arm:
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
  max_relative_target_rad: 0.02
```

Left-arm starting point:

```yaml
arm:
  can_interface: can2
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
```

You must still measure/fill:

```yaml
zero_offsets_rad:
  rpo_arm_j1: measured_value
  rpo_arm_j2: measured_value
  rpo_arm_j3: measured_value
  rpo_arm_j4: measured_value
  rpo_arm_j5: measured_value

joint_limits_rad:
  rpo_arm_j1: [measured_min, measured_max]
  rpo_arm_j2: [measured_min, measured_max]
  rpo_arm_j3: [measured_min, measured_max]
  rpo_arm_j4: [measured_min, measured_max]
  rpo_arm_j5: [measured_min, measured_max]
```

Do not trust placeholder limits for real movement. Use conservative limits first, around 50-70% of the real mechanical range.

### 5.2 AmazingHand section

```yaml
amazinghand:
  enabled: true
  serial_port: /dev/ttyUSB_AH_RIGHT
  baudrate: 1000000
  timeout: 0.5
  action_mode: scalar_grasp
  servo_ids: [1, 2, 3, 4, 5, 6, 7, 8]
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

The `middle_pos_deg` and `safe_limits_deg` values are placeholders until measured on your assembled hand.

---

## 6. What the Robot class must do

The custom `Robot` class is where whole-system control happens.

### 6.1 Constructor

It should create both adapters:

```python
class Roboparty5DofArmAmazingHandFollower(Robot):
    config_class = Roboparty5DofArmAmazingHandConfig
    name = "roboparty_5dof_arm_amazinghand_follower"

    def __init__(self, config: Roboparty5DofArmAmazingHandConfig):
        super().__init__(config)
        self.config = config
        self.arm = Rpo5DofArmAdapter(config)
        self.hand = AmazingHandAdapter(config)
        self.cameras = make_cameras_from_configs(config.cameras)
```

### 6.2 `connect()`

It should connect all hardware:

```text
connect()
  ├── arm.connect()   → open CAN, verify IDs, enable/configure DM motors
  ├── hand.connect()  → open serial, verify servo IDs, enable torque
  └── cameras.connect()
```

### 6.3 `get_observation()`

It should read both subsystems and return one observation dict:

```python
def get_observation(self) -> dict:
    arm_joint_rad = self.arm.read_joint_positions_rad()
    hand_grasp = self.hand.read_or_estimate_grasp_scalar()

    obs = {
        "rpo_arm_j1.pos": arm_joint_rad["rpo_arm_j1"],
        "rpo_arm_j2.pos": arm_joint_rad["rpo_arm_j2"],
        "rpo_arm_j3.pos": arm_joint_rad["rpo_arm_j3"],
        "rpo_arm_j4.pos": arm_joint_rad["rpo_arm_j4"],
        "rpo_arm_j5.pos": arm_joint_rad["rpo_arm_j5"],
        "amazinghand_grasp.pos": hand_grasp,
    }

    obs["front"] = self.cameras["front"].read_latest()
    # obs["wrist"] = self.cameras["wrist"].read_latest()
    return obs
```

### 6.4 `send_action()`

It should split one LeRobot action dict into arm and hand commands:

```python
def send_action(self, action: dict) -> dict:
    arm_targets = {
        "rpo_arm_j1": action["rpo_arm_j1.pos"],
        "rpo_arm_j2": action["rpo_arm_j2.pos"],
        "rpo_arm_j3": action["rpo_arm_j3.pos"],
        "rpo_arm_j4": action["rpo_arm_j4.pos"],
        "rpo_arm_j5": action["rpo_arm_j5.pos"],
    }
    grasp = float(action["amazinghand_grasp.pos"])

    safe_arm_targets = self.arm.clamp_joint_targets(arm_targets)
    self.arm.write_joint_targets_rad(safe_arm_targets)

    safe_grasp = min(1.0, max(0.0, grasp))
    self.hand.command_grasp_scalar(safe_grasp)

    return {
        "rpo_arm_j1.pos": safe_arm_targets["rpo_arm_j1"],
        "rpo_arm_j2.pos": safe_arm_targets["rpo_arm_j2"],
        "rpo_arm_j3.pos": safe_arm_targets["rpo_arm_j3"],
        "rpo_arm_j4.pos": safe_arm_targets["rpo_arm_j4"],
        "rpo_arm_j5.pos": safe_arm_targets["rpo_arm_j5"],
        "amazinghand_grasp.pos": safe_grasp,
    }
```

This is the key point: LeRobot sends one action; your robot class decides which part goes to CAN and which part goes to serial.

---

## 7. Arm conversion formulas

Your guide's conversion formulas are correct and should be implemented in the arm adapter.

Raw motor position to LeRobot joint position:

```python
joint_rad = sign * (raw_motor_rad - zero_offset_rad)
```

LeRobot joint target to raw motor target:

```python
raw_target_rad = sign * joint_target_rad + zero_offset_rad
```

Always clamp before sending:

```python
safe_target = clamp(joint_target_rad, min_rad, max_rad)
safe_target = clamp_delta(safe_target, current_joint_rad, max_relative_target_rad)
```

For first powered movement, use:

```text
max_relative_target_rad = 0.005 to 0.02 rad
speed/gain              = low
one joint at a time
hand empty
operator outside workspace
emergency stop ready
```

---

## 8. AmazingHand scalar grasp mapping

Stage 1 maps one scalar to 8 servo targets:

```python
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

Before using this on real hardware:

```text
1. Verify every servo ID responds.
2. Verify direction per servo.
3. Reduce open/close range first.
4. Clamp to safe limits.
5. Test speed 1 or very low speed first.
6. Check no finger self-collision or cable pull happens.
```

---

## 9. The real blockers

The integration architecture is sound. The risk is hardware bring-up, not LeRobot.

### Blocker A: CAN motor communication

The guide's arm adapter is still a skeleton. You need real implementation for:

```python
read_raw_positions()
write_raw_positions()
disable_torque()
configure_motors()
```

You have two possible paths:

```text
Path B: Reuse LeRobot Damiao/OpenArm support
  Use if RoboParty DM protocol is compatible with LeRobot's DamiaoMotorsBus.

Path C: Wrap RoboParty deploy motor code
  Use if RoboParty's own motor stack is more reliable or protocol details differ.
```

This is the #1 risk. If you cannot read motor ID 19 on `can3`, the custom config cannot control anything.

### Blocker B: motor ID ↔ physical joint mapping

The guide uses neutral names:

```text
rpo_arm_j1..rpo_arm_j5
```

That is good for early code, but before data collection you must know:

```text
motor 19/14 moves which physical joint?
motor 20/15 moves which physical joint?
...
```

If the joint order is wrong, your policy will learn the wrong action semantics.

### Blocker C: signs and zero offsets

Wrong signs cause mirrored/unstable control. Wrong zero offsets cause jumps at enable.

Validate one joint at a time:

```text
1. Safe pose.
2. Command +0.005 or +0.01 rad.
3. Confirm physical positive direction.
4. Confirm LeRobot observation increases in the same convention.
5. Fix sign before next joint.
```

### Blocker D: 5-DOF arm task limits

A 5-DOF arm can learn useful tasks, but it cannot behave like a full 6/7-DOF manipulator.

Good first tasks:

```text
front-facing pick
large/light foam cube
tray placement
fixed table height
fixed camera
simple open/close grasp
```

Bad first tasks:

```text
side insertion
tool use
precise wrist roll
in-hand reorientation
cluttered bin picking
small object dexterity
```

---

## 10. How to register the custom robot type

You need the config class registered:

```python
from dataclasses import dataclass, field
from lerobot.robots.config import RobotConfig

@RobotConfig.register_subclass("roboparty_5dof_arm_amazinghand_follower")
@dataclass
class Roboparty5DofArmAmazingHandConfig(RobotConfig):
    id: str | None = "rpo_right_arm_ah_v1"
    side: str = "right"
    arm_can_interface: str = "can3"
    hand_serial_port: str = "/dev/ttyUSB_AH_RIGHT"
    hand_action_mode: str = "scalar_grasp"
    # add the full fields here
```

Then LeRobot must be able to instantiate the robot.

You have two choices:

### Option 1: In-tree LeRobot robot

Add a package under:

```text
lerobot/src/lerobot/robots/roboparty_5dof_arm_amazinghand/
```

and update:

```text
lerobot/src/lerobot/robots/utils.py
```

with a branch similar to:

```python
elif config.type == "roboparty_5dof_arm_amazinghand_follower":
    from .roboparty_5dof_arm_amazinghand import Roboparty5DofArmAmazingHandFollower
    return Roboparty5DofArmAmazingHandFollower(config)
```

You may also need to import the module in scripts the same way built-in robots are imported, so draccus sees the registered config.

### Option 2: Third-party plugin

Create an installable package whose distribution name starts with:

```text
lerobot_robot_
```

LeRobot's `register_third_party_plugins()` scans packages with this prefix and imports them, which lets your config register itself. This is cleaner long-term if you do not want to keep patching LeRobot source.

For your current integration project, Option 1 is probably faster. Option 2 is cleaner after the robot works.

---

## 11. Minimal validation sequence

Do not jump straight to dataset collection. Use gates.

### Gate A: arm only

```text
[ ] can3/can2 exists on Linux
[ ] motor IDs respond
[ ] all 5 raw positions can be read
[ ] torque disable works
[ ] one tiny single-joint command works
[ ] sign per joint confirmed
[ ] zero offsets measured
[ ] conservative joint limits configured
```

### Gate B: hand only

```text
[ ] serial port exists
[ ] all 8 servo IDs respond
[ ] torque enable/disable works
[ ] middle positions measured
[ ] scalar 0.0 opens safely
[ ] scalar 1.0 closes safely, with reduced range first
```

### Gate C: combined LeRobot robot

```text
[ ] robot.connect() connects arm + hand + cameras
[ ] robot.observation_features lists 6 numeric features + cameras
[ ] robot.action_features lists 6 numeric action features
[ ] robot.get_observation() returns stable values
[ ] robot.send_action() moves the correct joint/hand with tiny command
[ ] robot.disconnect() disables torque / safe mode
```

### Gate D: dataset

```text
[ ] 10 debug episodes record successfully
[ ] observation.state shape is (6,)
[ ] action shape is (6,)
[ ] state/action order is stable
[ ] no NaNs
[ ] no sudden action jumps
[ ] front camera sees object, hand, and tray
[ ] hand scalar changes when hand opens/closes
```

### Gate E: policy

```text
[ ] train ACT on 50-100 simple episodes
[ ] dry-run policy with motors disabled or commands logged
[ ] inspect output range
[ ] evaluate with reduced max delta
[ ] emergency stop ready
```

---

## 12. Recommended first implementation milestone

Your first useful milestone should be **not** "full dexterous hand policy".

It should be:

```text
A LeRobot robot type named roboparty_5dof_arm_amazinghand_follower
that exposes a 6D policy/action contract:

  5 calibrated RoboParty arm joint positions
  +
  1 AmazingHand scalar grasp value
```

Success criteria:

```text
1. `lerobot-record` can instantiate the robot by type.
2. `robot.get_observation()` returns 6 stable numeric features plus images.
3. `robot.send_action()` moves the arm and hand safely.
4. A 10-episode debug dataset has observation.state/action shape (6,).
5. The same 6D action representation is used for teleop, recording, training, and eval.
```

That is enough for the first ACT baseline.

---

## 13. Bottom line

Yes, LeRobot custom config can control your whole integrated arm-hand hardware **if implemented as one custom robot**, not as separate unrelated devices.

The correct boundary is:

```text
LeRobot standard world
  - robot.type
  - observation_features
  - action_features
  - get_observation()
  - send_action()
  - dataset observation.state/action

RoboParty/AmazingHand hardware world
  - CAN interface
  - DM motor IDs/signs/zero offsets
  - AmazingHand serial port/servo IDs
  - scalar-to-servo mapping
  - hardware safety limits
```

Keep all hardware-specific details behind the custom `Robot` class and adapters. Then LeRobot can record, train, and evaluate policies as if the whole arm + hand is one normal robot.

Your highest-risk work is not LeRobot config. It is:

```text
1. CAN communication to RoboParty DM motors.
2. Correct motor ID / joint order mapping.
3. Zero offsets and signs.
4. Safe AmazingHand servo calibration.
```

Pass those gates first, then the LeRobot custom config is the right architecture for controlling the whole system.
