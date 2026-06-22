# RoboParty 5-DOF Arm + AmazingHand + LeRobot Plan

## Short answer

The open-source base is already strong. Your team should not rebuild the robot
stack from zero.

```text
RoboParty repos = hardware, ROS2 deployment, CAN, zeroing, XR teleop, robot models
AmazingHand repo = hand CAD, servo control, calibration examples
LeRobot = robot interface, teleoperation, dataset recording, training, inference
SO-100 / leader-arm setup = additional practical data-collection path
```

The remaining work is mostly integration:

```text
extract one 5-DOF RoboParty arm
mount and control AmazingHand
wrap both devices as one LeRobot-compatible robot
collect demonstrations through XR and/or leader-arm teleoperation
train and evaluate a small imitation-learning baseline
```

The safest first target is still:

```text
RoboParty 5-DOF arm + AmazingHand
Action = 5 arm joint targets + 1 AmazingHand grasp scalar
Task   = pick a soft cube and place it into a fixed tray
```

---

# 1. What is already implemented?

## RoboParty / `roboto_origin`

`roboto_origin` is not just an arm repository. It aggregates RoboParty hardware,
deployment, robot description, firmware, navigation, training, and XR teleop
sub-repositories. ([GitHub][1]) The useful pieces for this project are:

| Area | Already available? | Use it for |
| --- | ---: | --- |
| Full humanoid system overview | Yes | Reference architecture |
| Arm-related hardware source | Yes, through `rpo_hardware` | CAD and mechanical reference |
| ROS2 motor deployment | Yes, through `roboparty_deploy` | CAN, zeroing, motor configuration |
| URDF / MJCF robot model | Yes, through `rpo_description` | Arm model extraction |
| XR teleop | Yes, through `roboparty_xr_teleop` | One teleoperation input path |
| LeRobot integration | No | You still need to build this wrapper |

Use RoboParty as the low-level hardware reference. Do not copy the full humanoid
control stack into the learning project unless a part is directly needed for the
single-arm setup.

---

# 2. Mechanical engineering work

## Already reusable

`rpo_hardware` provides the open-source hardware library for RoboParty / Roboto
Origin, including mechanical files, PCB assets, manufacturing assets, and
versioned documentation. The repository says V2.0 is the recommended version,
with structure, cabling, stability, and arm improvements. ([GitHub][2])

You can reuse:

```text
RoboParty arm CAD
mechanical structure reference
mounting geometry reference
PCB / hardware layout reference
versioned hardware documentation
URDF / MJCF model assets from rpo_description
```

The `rpo_description` repository is also useful because it already provides robot
model assets instead of requiring a fresh model from zero. ([GitHub][3])

## Still owned by M.E

```text
[ ] Confirm whether the physical hardware is V1.0 or V2.0.
[ ] Extract only the 5-DOF arm geometry from the full humanoid model.
[ ] Design the RoboParty wrist-to-AmazingHand adapter.
[ ] Define the AmazingHand tool frame / grasp center.
[ ] Check wrist payload, stiffness, and backlash with AmazingHand attached.
[ ] Check serial and power cable routing through the wrist area.
[ ] Define a safe table-mounted workspace.
[ ] Build a repeatable cube/tray fixture for data collection.
[ ] Update collision assumptions after the hand and cameras are mounted.
```

Important: `rpo_hardware` says V2.0 is not compatible with V1.0, so the hardware
version must be confirmed before reusing CAD or PCB files. ([GitHub][2])

---

# 3. Electrical engineering work

## Already reusable

`roboparty_deploy` documents the CAN hardware mapping:

```text
can0 = left leg
can1 = right leg + waist
can2 = left hand / upper-limb group
can3 = right hand / upper-limb group
```

It also recommends USB 3.0 for CAN devices and USB 2.0 for IMU/gamepad devices.
([GitHub][4])

The RoboParty zeroing config shows:

```yaml
motor_num: [6, 7, 5, 5]
motor_interface: ["can0", "can1", "can2", "can3"]
motor_id: [1, 2, ..., 23]
```

That strongly suggests the 5-motor groups are the left and right upper-limb
groups: likely IDs 14-18 on `can2` and IDs 19-23 on `can3`. This must still be
verified on the physical robot. ([GitHub][5])

`roboparty_deploy` also already provides zero-calibration tools, including the
ROS service `/set_zeros` and `python3 scripts/set_zero.py`. Your team should
adapt these tools for arm-only bring-up instead of writing motor zeroing from
scratch. ([GitHub][4])

## Still owned by E.E

```text
[ ] Verify the left/right arm CAN interface on the real hardware.
[ ] Verify the exact 5 motor IDs.
[ ] Verify motor direction/sign for each joint.
[ ] Test one motor at a time before testing all 5 joints.
[ ] Adapt zeroing config to arm-only mode.
[ ] Build stable power distribution for RoboParty arm + AmazingHand.
[ ] Add fuse protection.
[ ] Add a physical emergency stop.
[ ] Label all CAN, serial, USB, and power cables.
[ ] Make camera device names stable.
```

---

# 4. AmazingHand work

AmazingHand is a good fit for an early dexterous-hand prototype because the repo
already includes CAD, docs, Python examples, Arduino examples, calibration
examples, and demos. It describes the hand as an 8-DOF, 4-finger design using two
Feetech SCS0009 servos per finger, with all actuators inside the hand and a mass
around 400 g. ([GitHub][6])

## Already reusable

```text
AmazingHand CAD
servo ID convention
Python serial-bus examples
Arduino examples
calibration examples
predefined hand poses
servo read/write examples
```

## Still owned by the team

```text
[ ] Mechanical adapter from RoboParty wrist to AmazingHand.
[ ] AmazingHand power wiring on the robot.
[ ] Serial-bus connection to the control computer.
[ ] Servo ID verification.
[ ] Safe open/close scalar command.
[ ] Hand joint limit table.
[ ] LeRobot AmazingHand driver wrapper.
[ ] Mapping from 1 grasp scalar to 8 servo positions.
```

For the first policy, do not expose all 8 hand servos. Start with:

```text
0.0 = open
1.0 = close / power grasp
```

Move to 8-servo hand control only after the 5+1 baseline is stable.

---

# 5. LeRobot integration work

LeRobot already provides the right integration pattern: define a custom robot
type with observation features, action features, `connect()`, `disconnect()`,
`get_observation()`, and `send_action()`. ([Hugging Face][7])

## First robot type

Use one LeRobot robot type for the integrated target hardware:

```text
roboparty_5dof_arm_amazinghand_follower
```

The first feature/action contract should be:

```text
rpo_arm_j1.pos
rpo_arm_j2.pos
rpo_arm_j3.pos
rpo_arm_j4.pos
rpo_arm_j5.pos
amazinghand_grasp.pos
```

Units:

```text
RoboParty arm joints = degrees
AmazingHand grasp    = scalar in [0.0, 1.0]
```

## Required data flow

```text
get_observation()
  read RoboParty arm joint positions
  apply motor signs and calibration offsets
  read or remember AmazingHand grasp scalar
  add camera observations
  return flat LeRobot feature dictionary

send_action(action)
  clamp 5 arm joint targets
  clamp max relative joint movement
  apply motor signs back to raw motor targets
  send RoboParty arm command over CAN
  clamp AmazingHand grasp scalar
  send AmazingHand serial command
  return the clipped action actually sent
```

## Still owned by C.S

```text
[ ] Create the custom LeRobot robot package.
[ ] Reuse RoboParty CAN control inside the LeRobot Robot class.
[ ] Reuse AmazingHand Python control inside the same Robot class.
[ ] Implement the 5 arm joints + 1 grasp scalar action mapper.
[ ] Implement the observation builder.
[ ] Add safety clamps before every motor command.
[ ] Add calibration loading.
[ ] Add cameras.
[ ] Record a small LeRobotDataset.
[ ] Train an ACT baseline.
[ ] Evaluate on the real robot.
```

---

# 6. Teleoperation and data collection paths

You now have two useful data-collection paths.

## Path A: RoboParty XR teleop

`roboparty_xr_teleop` already targets RoboParty / Roboto with PICO VR
teleoperation. It uses ROS2 Python, Python 3.10, PICO VR, Pinocchio, and CasADi
for IK-related work. ([GitHub][8])

Use XR teleop as an input device, not as a separate motor-control stack:

```text
XR controller pose
-> IK / pose-to-joint conversion
-> LeRobot action dictionary
-> robot.send_action()
-> LeRobotDataset recording
```

Avoid this for the learning path:

```text
XR controller pose
-> direct motor command
```

Direct motor teleop bypasses the same action format that the policy will later
use, which makes the dataset less useful.

## Path B: Leader arm + LeRobot SO-100 follower

Your additional plan to collect data with a leader arm and a LeRobot SO-100
follower is useful. It gives you a faster, lower-risk way to practice the
workflow before the RoboParty + AmazingHand system is fully integrated.

Use it for:

```text
operator training
camera placement testing
task design
episode naming and dataset QA
LeRobot recording workflow validation
ACT training sanity checks
baseline task difficulty measurement
```

The SO-100 follower path is especially valuable because it is already native to
LeRobot, so you can test the full collect-train-evaluate loop while the custom
RoboParty wrapper is still being built.

However, treat SO-100 data as proxy data, not automatically as final RoboParty
training data. Before mixing or transferring it, check:

```text
joint count and joint order
joint limits
joint signs
workspace scale
camera viewpoints
gripper/hand action meaning
task fixture geometry
action units
control frequency
```

The practical recommendation:

```text
Use SO-100 leader/follower first to prove the task and dataset workflow.
Use RoboParty + AmazingHand data for the final policy that runs on RoboParty.
Only transfer SO-100 data after you define an explicit action/observation mapping.
```

---

# 7. First dataset design

Start with a simple task:

```text
Task: pick foam cube and place into tray
Arm: fixed base or table-mounted
Hand: AmazingHand scalar open/close
Object: one soft/light cube
Start area: marked 10 cm x 10 cm square
Tray: fixed position
Episodes: 10 debug, then 50-100 baseline
```

Minimum RoboParty + AmazingHand dataset features:

```text
rpo_arm_j1.pos
rpo_arm_j2.pos
rpo_arm_j3.pos
rpo_arm_j4.pos
rpo_arm_j5.pos
amazinghand_grasp.pos
observation.images.front
observation.images.wrist   optional but recommended
timestamp
episode_index
frame_index
task
```

After 10 debug episodes, check:

```text
[ ] State shape is stable.
[ ] Action shape is stable.
[ ] Joint order is stable.
[ ] Joint signs are correct.
[ ] amazinghand_grasp changes when the hand opens/closes.
[ ] Camera sees the cube, hand, and tray.
[ ] No sudden jumps in arm targets.
[ ] Hand scalar is clamped to [0.0, 1.0].
[ ] Failed episodes are marked or discarded.
```

---

# 8. What not to rebuild

Reduce or remove these from the original to-do list:

```text
Do not design full arm CAD from zero.
Use rpo_hardware.

Do not write motor zeroing from zero.
Use roboparty_deploy set_zero.py.

Do not build the full URDF/MJCF from zero.
Use rpo_description and extract the arm.

Do not build XR teleop from zero.
Use roboparty_xr_teleop as a reference/input layer.

Do not invent the LeRobot integration pattern.
Use LeRobot's custom Robot interface.

Do not write AmazingHand servo examples from zero.
Use the AmazingHand Python/Arduino examples.

Do not wait for RoboParty integration before learning LeRobot workflows.
Use the SO-100 leader/follower setup as a proxy data-collection path.
```

---

# 9. Recommended technical sequence

```text
1. Clone and lock exact RoboParty, AmazingHand, and LeRobot commits.
2. Confirm RoboParty hardware version: V1.0 or V2.0.
3. Bring up the LeRobot SO-100 leader/follower setup.
4. Record 10 debug SO-100 episodes for the cube-to-tray task.
5. Train a tiny ACT sanity-check policy on the SO-100 dataset.
6. Extract the RoboParty arm-only motor config from roboparty_deploy.
7. Test one RoboParty arm motor through existing RoboParty scripts.
8. Test all 5 RoboParty arm motors with conservative limits.
9. Test AmazingHand using its official Python example.
10. Design and mount the AmazingHand wrist adapter.
11. Build the LeRobot wrapper around RoboParty CAN + AmazingHand serial control.
12. Record 10 debug RoboParty + AmazingHand episodes.
13. Train the first ACT baseline on RoboParty + AmazingHand data.
14. Compare SO-100 and RoboParty failures to improve the fixture and cameras.
```

---

# 10. Final project framing

The project is not mainly a hardware invention project. It is an integration and
robot-learning project.

The clean architecture is:

```text
teleop source
  XR controller OR leader arm

LeRobot action interface
  5 RoboParty arm joints + 1 AmazingHand grasp scalar

custom robot wrapper
  RoboParty CAN adapter + AmazingHand serial adapter

LeRobotDataset
  standardized demos

policy
  ACT baseline first
```

Your strongest next move is to run the SO-100 leader/follower data workflow in
parallel with RoboParty hardware bring-up. That lets the C.S side validate
LeRobot recording and training immediately, while M.E/E.E finish the RoboParty
arm and AmazingHand integration.

[1]: https://github.com/Roboparty/roboto_origin "GitHub - Roboparty/roboto_origin"
[2]: https://github.com/Roboparty/rpo_hardware "GitHub - Roboparty/rpo_hardware"
[3]: https://github.com/Roboparty/rpo_description "GitHub - Roboparty/rpo_description"
[4]: https://github.com/Roboparty/roboparty_deploy "GitHub - Roboparty/roboparty_deploy"
[5]: https://raw.githubusercontent.com/Roboparty/roboparty_deploy/main/scripts/config/set_zero.yaml "RoboParty set_zero.yaml"
[6]: https://github.com/pollen-robotics/AmazingHand "GitHub - pollen-robotics/AmazingHand"
[7]: https://huggingface.co/docs/lerobot/integrate_hardware "LeRobot Bring Your Own Hardware"
[8]: https://github.com/Roboparty/roboparty_xr_teleop "GitHub - Roboparty/roboparty_xr_teleop"
