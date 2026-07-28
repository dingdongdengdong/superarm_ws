# Sim-to-Real

## Principle

Sim-to-real is not a single algorithm. It is the process of keeping observation, action, timing, controller behavior, and task conditions consistent enough that a policy trained in simulation can operate on the physical robot.

```text
calibrate nominal simulation
        ↓
randomize uncertain parameters
        ↓
train and validate in simulation
        ↓
add a small amount of real data
        ↓
deploy with the same action interface
        ↓
collect failures and iterate
```

## Three required consistencies

### Input consistency

The real robot must provide the inputs used during training:

- matching camera views and resolution;
- matching state order and units;
- matching grasp-state definition;
- matching preprocessing and normalization.

### Output consistency

The action must have the same meaning in both domains:

- same joint order;
- same absolute or relative definition;
- same units;
- same scale and clipping;
- same gripper interpretation.

### Response consistency

The simulated robot should react approximately like the physical robot:

- controller frequency;
- velocity and acceleration limits;
- delay;
- damping and stiffness;
- friction and dead zone;
- hand closing time.

## Shared control contract

Use the same configuration and action-adapter implementation for:

```text
simulation rollout
simulation recording
LeRobot replay
real shadow mode
real command mode
```

The current first contract is the repository’s five arm joints plus `amazinghand_grasp`.

## Minimal real measurements

A large real dataset is not required before simulation training, but perform these measurements:

### Arm

- small and medium joint step responses;
- commanded and measured position;
- response delay;
- maximum safe velocity;
- steady-state error;
- backlash or dead zone.

### Hand

- opening and closing time;
- contact current or closure threshold;
- object slip behavior;
- repeatability of grasp scalar commands.

### Camera

- intrinsic calibration;
- camera-to-base or hand-eye calibration;
- depth scale;
- frame latency and synchronization.

Use these measurements to set nominal simulator values and realistic randomization ranges.

## Domain randomization

Randomize uncertainty, not every possible value.

### Visual

- lighting intensity and direction;
- texture and background;
- camera pose and exposure;
- image noise and blur;
- object color and appearance.

### Physical

- object mass and center of mass;
- friction;
- joint gain and damping;
- action delay;
- initial robot pose;
- object and bin pose.

Too-narrow randomization overfits simulation. Too-wide randomization may prevent learning.

## Simulation and real-data training

Recommended sequence:

```text
simulation pretraining
        ↓
real demonstration collection
        ↓
real-heavy fine-tuning with simulation replay
```

Real data should focus on gaps:

- actual lighting and reflective materials;
- calibration error;
- grasp slip;
- motor lag;
- failed and corrected approaches;
- deformable objects.

## Deployment sequence

### 1. Offline replay

Run policy inference on recorded real observations without commanding the robot.

### 2. Shadow mode

Use live real observations and log predicted actions only.

### 3. Limited rollout

Use low velocity, a narrow workspace, one light object, and a fixed destination.

### 4. Progressive expansion

Expand object pose, object type, clutter, and destination variation only after the previous stage is stable.

## Failure taxonomy

| Failure | Primary area to inspect |
|---|---|
| Object not detected | Perception data and model |
| Gripper misses object | Calibration, depth, latency |
| Arm motion differs from simulation | Controller and actuator model |
| Object slips | Hand controller and friction |
| Correct pick, wrong bin | Routing logic |
| Policy cannot recover | Recovery demonstrations or RL refinement |

## 한국어 요약

Sim-to-real의 핵심은 실물을 완벽히 복제하는 것이 아니라 observation·action·controller 의미를 동일하게 유지하고, 측정하기 어려운 차이는 domain randomization과 소량의 실제 데이터로 보정하는 것입니다. 배포는 offline replay, shadow mode, 저속 제한 실행 순서로 진행합니다.
