# Isaac Sim Workflow

## Version baseline

This project currently targets:

```text
Isaac Sim 5.1
+ custom SimReady USD
+ Lula motion generation
+ ROS 2 / LeRobot six-dimensional bridge
```

The source of truth is documented in [`isaacsim_test/README.md`](../isaacsim_test/README.md).

## Current SimReady asset

The validated asset is located at:

```text
isaacsim_test/outputs/simready/echo_full/
  pipeline/04_conform/repair-loop-02-fet005/
  fet005-grasp/echo_full_robot_arm_hand.usd
```

Before training, confirm that the asset:

- loads without unstable falling or exploding links;
- preserves expected joint axes and limits;
- supports deterministic reset;
- accepts the six-dimensional command contract;
- publishes the matching state order;
- can open and close the hand abstraction.

## Isaac Sim is the data-generation environment

Use Isaac Sim for:

- physics and collision checking;
- exact object and target poses;
- external and wrist cameras;
- automatic scene reset;
- environment randomization;
- planner-generated demonstrations;
- safe failure and recovery tests.

LeRobot remains responsible for dataset formatting and learned-policy training.

## Teacher state machine

Implement a deterministic teacher before training ACT:

```text
RESET
  ↓
SELECT TARGET
  ↓
MOVE TO PRE-GRASP
  ↓
APPROACH
  ↓
CLOSE HAND
  ↓
VERIFY GRASP
  ↓
LIFT
  ↓
MOVE TO DESTINATION
  ↓
LOWER
  ↓
OPEN HAND
  ↓
VERIFY SUCCESS
```

Recommended division:

- Lula IK or RMPflow: arm movement;
- scripted state machine: phase transitions;
- hand controller: map `amazinghand_grasp` to finger targets;
- simulator ground truth: success checks.

## Episode randomization

Randomize each reset rather than replaying one trajectory:

```text
object position and yaw
bin position
initial arm pose
object mass and friction
camera pose
lighting and texture
observation and action delay
```

Begin with narrow ranges and expand only after the nominal task works.

## Observation groups

### Privileged teacher observation

```text
exact object pose
exact destination pose
EEF pose
contact state
joint position and velocity
hand state
```

### Deployable student observation

```text
external image
wrist image
joint position and velocity
hand state
target condition
```

Never allow the ACT student to depend on simulator-only object poses if those poses will not be available on the physical robot.

## Recording rule

Record the command that was actually applied to the articulation, not an internal planner waypoint.

```text
planner output
   ↓
controller/action adapter
   ↓
final applied command  ← dataset action
```

This keeps simulation training and physical deployment semantically consistent.

## Verification gates

Before generating a large dataset:

1. no-op action keeps the robot stable;
2. each controlled joint moves in the correct direction;
3. action order matches state order;
4. grasp scalar opens and closes consistently;
5. reset returns all objects and joints to valid states;
6. one complete scripted pick-and-place succeeds;
7. recorded episodes replay correctly.

Existing validation material is under [`docs/sitl/2026-07-02_core_validation/`](../docs/sitl/2026-07-02_core_validation/).

## 한국어 요약

Isaac Sim은 학습 모델 자체보다 자동 demonstration 생성기로 먼저 사용합니다. Lula/RMPflow와 상태 머신으로 성공 동작을 만들고, 매 episode마다 위치·물리·카메라를 랜덤화하여 실제 적용 가능한 observation과 최종 command를 기록합니다.
