# 01 — Source Inventory: What to Reuse from RoboParty

This document separates the RoboParty resources into the parts that are useful for a LeRobot arm-learning stack.

## 1. `roboto_origin`

Role: top-level aggregation repository.

Use it to understand the project layout, not as the package to port directly.

Extract:

- module list
- repo relationships
- high-level robot architecture
- links to hardware, deployment, description, firmware, navigation, XR teleop, and training repos

Do not extract directly:

- full humanoid control assumptions
- leg/waist balance policy assumptions
- whole-body deployment launch flow

Reason:

Your immediate target is a tabletop or mobile-mounted arm. A humanoid deployment stack contains many assumptions that are unrelated to LeRobot arm imitation learning.

## 2. `rpo_hardware`

Role: mechanical and electrical truth source.

Extract:

- CAD/mechanical files for arm geometry
- PCB/CAN wiring information
- BOM and motor model references
- hardware version differences
- joint physical limits
- gripper mechanics
- mounting constraints

Port to LeRobot:

```text
rpo_hardware/CAD + drawings
        ↓
URDF/kinematics verification
        ↓
LeRobot config: joint limits, motor IDs, motor types, safety limits
```

Important:

If your team changed the arm geometry from OpenArm, the official OpenArm LeRobot config should be treated as a reference only.

## 3. `rpo_description`

Role: robot model source.

Extract:

- URDF
- MJCF
- meshes
- joint names
- link names
- end-effector frame
- camera or mounting frames if available

Port to LeRobot:

```text
URDF/MJCF joint names
        ↓
ordered joint list
        ↓
LeRobot flat feature keys, e.g. rpo_arm_j1.pos
```

Use this repo to answer:

```text
What is joint_1 physically?
What direction is positive?
Where is the gripper frame?
What is the wrist camera frame?
What is the arm base frame?
```

## 4. `roboparty_deploy`

Role: real-robot deployment and hardware communication reference.

Extract:

- CAN bring-up commands
- motor ID conventions
- motor model list
- motor sign conventions
- zeroing scripts
- IMU/motor configuration style
- Python motor examples
- C++/ROS2 motor drivers if needed

Port to LeRobot:

```text
roboparty_deploy motor config
        ↓
RobopartyMotorBusAdapter
        ↓
LeRobot Robot.get_observation()
LeRobot Robot.send_action()
```

Use this repo only for low-level hardware access. Avoid pulling its full inference or humanoid balance flow into LeRobot.

## 5. `roboparty_firmware`

Role: embedded/board support reference.

Extract:

- USB2CAN firmware assumptions
- Orange Pi/RDK image build details
- device naming/udev ideas
- CAN hardware support

Port to LeRobot:

```text
USB2CAN / board setup
        ↓
Linux can0/can1/... devices
        ↓
LeRobot motor bus can connect
```

Usually you do not need to modify LeRobot for this. You need a stable Linux CAN device.

## 6. `roboparty_xr_teleop`

Role: optional demonstration collection interface.

Extract:

- XR controller/hand tracking input
- end-effector target calculation
- IK code path
- control frequency assumptions
- operator button semantics

Port to LeRobot:

```text
XR controller pose
        ↓
XR teleop adapter
        ↓
LeRobot action dictionary
        ↓
LeRobot record / send_action
```

Do not send XR commands directly to motors during early development. Let LeRobot receive the teleop action so data recording remains standardized.

## 7. `robolab`

Role: simulation/RL sandbox.

Extract later:

- simulated tasks
- terrain/assets
- MuJoCo tooling
- RL experiments

Do not start here for the real arm.

First real-world path:

```text
real arm → teleop → LeRobotDataset → ACT → real evaluation
```

Simulation can be useful later, but sim-to-real will slow down your first success.

## 8. LeRobot

Role: main learning framework.

Use:

- Bring Your Own Hardware robot interface
- Robot config registration
- processor pipelines
- LeRobotDataset v3
- `lerobot-calibrate`
- `lerobot-teleoperate`
- `lerobot-record`
- `lerobot-train`
- `lerobot-eval`

Your custom arm should look like a normal LeRobot robot to the rest of the framework.

## Summary table

| Source | Reuse now? | LeRobot destination |
|---|---:|---|
| `rpo_hardware` | Yes | joint limits, geometry, motor list, wiring |
| `rpo_description` | Yes | joint names, URDF, kinematics, EE frame |
| `roboparty_deploy/src/motors` | Yes | motor bus adapter or reference implementation |
| `roboparty_deploy/scripts/set_zero.py` | Yes | calibration/zeroing reference |
| `roboparty_firmware` | Maybe | CAN hardware setup |
| `roboparty_xr_teleop` | Later | teleop adapter / processor |
| `robolab` | Later | sim/RL experiments |
| LeRobot OpenArm | Yes | config style and Damiao motor reference |
| LeRobot BYOH | Yes | integration contract |
