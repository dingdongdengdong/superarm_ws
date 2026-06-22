# 05 — RoboParty XR Teleop Bridge to LeRobot

RoboParty XR teleop should be treated as an **input device** for LeRobot, not as the owner of the motor stack.

## Recommended bridge

```text
PICO / XR controller pose
        ↓
RoboParty XR tracking code
        ↓
coordinate transform
        ↓
IK solver
        ↓
LeRobot action dictionary
        ↓
LeRobot send_action()
        ↓
LeRobot record()
```

Avoid this early:

```text
XR controller pose → direct motor command
```

Why?

Because direct motor teleop bypasses the same action/observation format that the policy will later use.

## LeRobot-compatible teleop output

The XR bridge should output one of these forms.

### Option A — Joint-space action

Best for first dataset.

```python
action = {
    "action": np.array([
        joint_1_target,
        joint_2_target,
        joint_3_target,
        joint_4_target,
        joint_5_target,
        gripper_target,
    ], dtype=np.float32)
}
```

Pros:

- simple
- same as default LeRobot joint-space behavior
- ACT training is straightforward
- no policy-time IK needed

Cons:

- operator control may feel less intuitive unless XR IK is good

### Option B — End-effector action + processor

Good later.

```python
action = {
    "action.ee_delta": np.array([
        dx, dy, dz,
        droll, dpitch, dyaw,
        gripper_delta,
    ], dtype=np.float32)
}
```

Then a LeRobot processor converts:

```text
EE delta → IK → joint target → robot.send_action()
```

Pros:

- more intuitive for XR
- easier to retarget

Cons:

- requires correct URDF/kinematics
- IK failures can corrupt data
- policy must use matching representation

## Processor layout

Use three processor paths.

```text
1. teleop action → dataset action
2. dataset action → robot command
3. robot observation → dataset observation
```

This is the cleanest way to keep teleoperation, recording, and policy inference aligned.

## Coordinate frames

Define these frames explicitly:

```text
xr_world
xr_controller_right
robot_base
arm_base_link
wrist_link
tool0 / gripper_frame
camera_front
camera_wrist
```

Minimum transform config:

```yaml
xr_to_robot:
  translation_m: [0.0, 0.0, 0.0]
  rotation_rpy_rad: [0.0, 0.0, 0.0]
  scale: 1.0

control:
  position_gain: 0.5
  rotation_gain: 0.5
  max_ee_delta_m: 0.02
  max_joint_delta_rad: 0.03
```

## XR bridge skeleton

```python
class XRToLeRobotTeleop:
    def __init__(self, robot, ik_solver, cfg):
        self.robot = robot
        self.ik_solver = ik_solver
        self.cfg = cfg
        self.enabled = False

    def read_xr(self):
        # Read controller or hand pose from RoboParty XR stack.
        raise NotImplementedError

    def pose_to_joint_target(self, xr_pose, current_joints):
        ee_target = self.transform_xr_pose_to_robot_ee(xr_pose)
        joint_target = self.ik_solver.solve(
            ee_target=ee_target,
            current_joints=current_joints,
        )
        return joint_target

    def get_action(self):
        obs = self.robot.get_observation()
        current = [
            obs["rpo_arm_j1.pos"],
            obs["rpo_arm_j2.pos"],
            obs["rpo_arm_j3.pos"],
            obs["rpo_arm_j4.pos"],
            obs["rpo_arm_j5.pos"],
        ]

        xr_pose = self.read_xr()
        joint_target = self.pose_to_joint_target(xr_pose, current)

        return {
            "rpo_arm_j1.pos": float(joint_target[0]),
            "rpo_arm_j2.pos": float(joint_target[1]),
            "rpo_arm_j3.pos": float(joint_target[2]),
            "rpo_arm_j4.pos": float(joint_target[3]),
            "rpo_arm_j5.pos": float(joint_target[4]),
            "amazinghand_grasp.pos": obs["amazinghand_grasp.pos"],
        }
```

## Button mapping recommendation

Use buttons for safety and dataset quality.

```text
enable/disable teleop
reset XR reference pose
open gripper
close gripper
slow mode
discard current episode
mark success/failure
```

Do not overload a single button with safety-critical behavior.

## Bring-up sequence

### Stage 1 — XR only, no robot

```text
XR device connects
controller pose updates at target frequency
buttons detected
coordinate axes visualized
```

### Stage 2 — XR + IK dry-run

```text
XR pose produces joint target
joint target stays inside limits
IK failures are logged
no motor power
```

### Stage 3 — XR + robot slow mode

```text
max joint delta: 0.01–0.03 rad per command
low velocity
empty gripper
operator far from arm
emergency stop active
```

### Stage 4 — XR + LeRobot recording

```bash
lerobot-record \
  --robot.type=roboparty_5dof_arm_amazinghand_follower \
  --teleop.type=roboparty_xr \
  --repo-id=YOUR_HF_USERNAME/roboparty_xr_pick_cube_v1 \
  --fps=30 \
  --num-episodes=10
```

## Dataset rule

Whatever action representation you use during XR recording must be the same action representation used during policy inference.

Bad:

```text
record EE delta actions
train ACT as if actions are joint targets
```

Good:

```text
XR EE pose → IK → joint target
record joint target
train ACT on joint target
policy outputs joint target
```

## First XR task

Use a constrained task:

```text
pick one cube from fixed marked area
place into fixed tray
same lighting
same cameras
same table
short episodes
```

Do not collect general-purpose demos first.
