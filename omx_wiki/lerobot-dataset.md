# LeRobot Dataset Pipeline

## Role of LeRobot

LeRobot should be the common interface for:

```text
recording
→ dataset validation
→ visualization
→ policy training
→ policy evaluation
→ physical-robot rollout
```

Isaac Sim generates episodes. LeRobot stores and trains from them.

## First dataset contract

Keep the first dataset intentionally stable.

### State

```text
[right_arm_pitch_joint.pos,
 right_arm_roll_joint.pos,
 right_arm_yaw_joint.pos,
 right_elbow_pitch_joint.pos,
 right_elbow_yaw_joint.pos,
 amazinghand_grasp.pos]
```

### Action

Use the same shape `(6,)`:

```text
[arm command 0,
 arm command 1,
 arm command 2,
 arm command 3,
 arm command 4,
 grasp command]
```

Choose one action meaning and never mix it silently:

- absolute joint-position target; or
- relative joint-position delta.

The repository should record the selected meaning in dataset metadata.

## Recommended observation keys

```text
observation.images.external
observation.images.wrist
observation.state
action
timestamp
episode_index
task_index
```

Useful metadata:

```text
target_class
destination_bin
success
source_domain  # sim or real
randomization_seed
```

## Recording in simulation

The existing workflow in [`isaacsim_test/README.md`](../isaacsim_test/README.md) records episodes through the custom `isaacsim_rpo_arm` interface.

Before scaling data generation, validate:

- images and state belong to the same timestep;
- action is the final executed command;
- episode boundaries are correct;
- failed resets do not leak into the next episode;
- all joint arrays use the documented order;
- dataset replay reproduces the motion.

## Dataset stages

### Stage A: smoke dataset

```text
20–50 successful episodes
```

Purpose:

- verify schema;
- verify replay;
- test normalization;
- deliberately overfit ACT;
- find synchronization errors.

### Stage B: first baseline

```text
300–1,000 simulated episodes
```

Purpose:

- one or two rigid object classes;
- randomized object pose;
- fixed camera layout;
- moderate visual and physical variation.

### Stage C: generalization dataset

```text
thousands of episodes
```

Add:

- multiple waste shapes;
- clutter and occlusion;
- camera and lighting variation;
- physics variation;
- perturbed and recovery trajectories.

## Simulation and real data

Do not combine a very large simulation dataset with a tiny real dataset using uniform sampling.

Recommended training sequence:

```text
1. simulation-only pretraining
2. real-heavy fine-tuning
3. retain some simulation replay
```

A reasonable starting sampling ratio for fine-tuning is:

```text
60–80% real
20–40% simulation replay
```

Tune this using validation results rather than treating it as a fixed rule.

## Real-data priorities

Real episodes should cover gaps simulation models poorly:

- actual lighting and reflections;
- camera calibration error;
- motor delay and backlash;
- object slip;
- partial grasp failure;
- recovery from small misalignment;
- deformable or dirty waste appearance.

## Validation checklist

For every dataset release, record:

- number of episodes;
- successful and failed episodes;
- object classes;
- camera configuration;
- control frequency;
- state and action definitions;
- normalization statistics;
- simulation randomization ranges;
- real/sim sample ratio;
- train/validation/test split.

## 한국어 요약

LeRobot 데이터셋은 simulation과 실물에서 동일한 6차원 state/action 의미를 유지해야 합니다. 먼저 20~50개 episode로 replay와 overfit을 검증하고, 이후 simulation pretraining과 real-heavy fine-tuning으로 확장합니다.
