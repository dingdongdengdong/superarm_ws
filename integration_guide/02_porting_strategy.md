# 02 — Porting Strategy: Making RoboParty Transparent to LeRobot

## What “transparent to LeRobot” means

A RoboParty/OpenArm-derived arm is transparent to LeRobot when LeRobot tools do not need to know the internal RoboParty structure.

From the LeRobot side, the robot should look like this:

```bash
lerobot-calibrate --robot.type=roboparty_5dof_arm_amazinghand_follower ...
lerobot-teleoperate --robot.type=roboparty_5dof_arm_amazinghand_follower ...
lerobot-record --robot.type=roboparty_5dof_arm_amazinghand_follower ...
lerobot-eval --robot.type=roboparty_5dof_arm_amazinghand_follower ...
```

Internally, your adapter may use:

```text
RoboParty motor IDs
RoboParty CAN mappings
RoboParty zero offsets
RoboParty URDF joint names
RoboParty XR teleop
```

Externally, LeRobot sees only:

```text
rpo_arm_j1.pos
rpo_arm_j2.pos
rpo_arm_j3.pos
rpo_arm_j4.pos
rpo_arm_j5.pos
amazinghand_grasp.pos
observation.images.front
observation.images.wrist
observation.environment_state, optional
```

## Four transparency levels

### 1. CLI transparency

Goal:

```bash
--robot.type=roboparty_5dof_arm_amazinghand_follower
```

LeRobot can construct your robot through its normal robot registry.

### 2. Data transparency

Goal:

```text
LeRobotDataset contains standard keys.
```

Example:

```text
rpo_arm_j1.pos
rpo_arm_j2.pos
rpo_arm_j3.pos
rpo_arm_j4.pos
rpo_arm_j5.pos
amazinghand_grasp.pos
observation.images.front
observation.images.wrist
```

ACT/SmolVLA should not care whether the physical arm came from OpenArm, RoboParty, or a custom design.

### 3. Policy transparency

Goal:

```text
policy output → LeRobot action → arm motion
```

The trained policy should output the same flat action keys used during data collection.

### 4. Teleoperation transparency

Goal:

```text
OpenArm leader arm
XR controller
keyboard/gamepad
scripted test trajectory
```

All teleoperators eventually produce the same action dictionary.

## Recommended architecture

```text
roboparty_lerobot package
├── Robot class
│   ├── connect()
│   ├── disconnect()
│   ├── get_observation()
│   ├── send_action()
│   └── calibrate(), if needed
├── Motor bus adapter
│   ├── read_positions()
│   ├── read_currents(), optional
│   ├── write_positions()
│   └── disable_torque()
├── Camera adapters
├── Processors
└── YAML configs
```

#

## Porting option B — Custom LeRobot config, reuse LeRobot Damiao support

This is usually the best first custom path.

Use when:

```text
motors are still Damiao/OpenArm-style
CAN protocol is compatible
only IDs, limits, signs, or joint names changed
```

Implementation:

```text
copy OpenArm config structure
change robot.type string
change motor table
change joint order
change limits
change calibration metadata
```

Result:

```bash
lerobot-calibrate --robot.type=roboparty_5dof_arm_amazinghand_follower --robot.port=can3 ...
```

## Porting option C — Wrap RoboParty motor implementation

Use when RoboParty motor stack is more reliable than LeRobot’s existing motor support for your hardware.

Implementation:

```text
RoboParty motors_py_example / C++ motor logic
        ↓
RobopartyMotorBusAdapter
        ↓
LeRobot Robot class
```

Adapter API:

```python
class RobopartyMotorBusAdapter:
    def connect(self): ...
    def disconnect(self): ...
    def read_positions(self) -> dict[str, float]: ...
    def read_velocities(self) -> dict[str, float]: ...
    def write_positions(self, targets: dict[str, float]) -> None: ...
    def disable_torque(self) -> None: ...
```

This keeps the RoboParty low-level implementation, but LeRobot owns data and policy flow.

## What not to port initially

Do not port these into the first LeRobot arm package:

```text
full humanoid inference
walking/balance policy
leg/waist control
Nav2/mobile base logic
full robolab RL stack
XR direct motor sender
```

They can be integrated later.

## Minimal first version

For the first working version, implement only:

```text
connect()
disconnect()
get_observation()
send_action()
joint limit clamp
motor sign conversion
zero offset conversion
front camera
optional wrist camera
```

Then validate with:

```bash
lerobot-record --num-episodes=10 --fps=30 ...
```

## Design rule

All RoboParty-specific details should be hidden behind one boundary:

```text
RoboParty-specific world | LeRobot-standard world
─────────────────────────|────────────────────────
CAN IDs                  | joint names
motor signs              | normalized action order
zero offsets             | calibrated joint positions
XR pose conventions      | action dictionary
URDF naming              | observation/action features
```
