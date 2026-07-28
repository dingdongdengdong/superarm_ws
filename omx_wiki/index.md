# SuperArm Project Wiki

Updated: 2026-07-28

This wiki is the repository-level design and implementation guide for the custom 5-DoF arm, AmazingHand, Isaac Sim 5.1, LeRobot, ACT, waste sorting, and sim-to-real workflow.

## Start here

Read these pages in order:

1. [Project overview](project-overview.md)
2. [System architecture](system-architecture.md)
3. [Isaac Sim workflow](isaac-sim-workflow.md)
4. [LeRobot dataset pipeline](lerobot-dataset.md)
5. [ACT training](act-training.md)
6. [Waste sorting pipeline](waste-sorting.md)
7. [Sim-to-real](sim-to-real.md)
8. [Implementation roadmap](roadmap.md)
9. [Validation and troubleshooting](troubleshooting.md)

## Category map

### Project and architecture

- [Project overview](project-overview.md): goals, scope, model strategy, and success criteria.
- [System architecture](system-architecture.md): perception, routing, manipulation, control contract, and safety layers.

### Simulation and robot assets

- [Isaac Sim workflow](isaac-sim-workflow.md): SimReady USD, Lula/RMPflow teacher, randomization, and recording.
- [AmazingHand in Isaac Sim](amazinghand-isaacsim.md): hand conversion decisions, evidence, and known limitations.
- [`isaacsim_test/README.md`](../isaacsim_test/README.md): executable Isaac Sim 5.1 and LeRobot SITL entrypoint.

### Robot learning

- [LeRobot dataset pipeline](lerobot-dataset.md): state/action schema, recording, validation, and sim/real mixing.
- [ACT training](act-training.md): overfit test, simulation pretraining, real fine-tuning, evaluation, and optional RL.

### Task pipeline

- [Waste sorting](waste-sorting.md): segmentation, class-to-bin rules, target selection, grasp abstraction, and language decision.
- [Sim-to-real](sim-to-real.md): consistency requirements, calibration, randomization, real data, and deployment stages.

### Execution and operations

- [Implementation roadmap](roadmap.md): phased milestones and exit criteria.
- [Validation and troubleshooting](troubleshooting.md): layered debugging and failure taxonomy.
- [Project log](log.md): chronological decisions and evidence.
- [`docs/sitl/2026-07-02_core_validation/`](../docs/sitl/2026-07-02_core_validation/): detailed SITL validation tasks.
- [`docs/task_guides/`](../docs/task_guides/): focused implementation procedures.
- [`integration_guide/`](../integration_guide/): motor, hand, and Isaac Sim integration notes.

## Current first policy contract

```text
[right_arm_pitch_joint,
 right_arm_roll_joint,
 right_arm_yaw_joint,
 right_elbow_pitch_joint,
 right_elbow_yaw_joint,
 amazinghand_grasp]
```

Keep this order and action meaning identical across Isaac Sim, LeRobotDataset, ACT training, replay, and physical deployment until an intentional versioned migration is introduced.

## Documentation policy

- Record important debugging conclusions and validation artifact paths before committing related code.
- Keep transient simulator outputs out of the wiki unless they are required as evidence.
- Do not treat a report `PASS` alone as sufficient for visual or contact tasks; record image and runtime inspection results.
- Update the wiki when the control contract, dataset schema, model input, or deployment interface changes.

## 한국어 안내

이 위키는 커스텀 팔의 시뮬레이션, LeRobot 데이터셋, ACT 학습, 쓰레기 분류, sim-to-real 구현 순서를 하나의 구조로 정리합니다. 처음에는 위의 `Start here` 순서대로 읽는 것을 권장합니다.
