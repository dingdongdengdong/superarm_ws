# Implementation Roadmap

## Phase 0 — Freeze the interface

- Confirm the six state/action names and their order.
- Decide absolute versus relative joint-position actions.
- Fix policy and low-level controller frequencies.
- Define joint soft limits and grasp-scalar semantics.
- Store these values in one shared configuration.

**Exit criterion:** simulation commands, recorded actions, and replayed actions have identical meanings.

## Phase 1 — Validate the SimReady robot

- Load the validated USD from `isaacsim_test/outputs/simready/echo_full/`.
- Verify no-op stability.
- Run individual joint sweeps.
- Verify AmazingHand open/close abstraction.
- Verify deterministic reset.

**Exit criterion:** all six command dimensions pass the SITL validation gates.

## Phase 2 — Build one scripted pick-and-place teacher

Start with one rigid object and one fixed bin.

- calculate pre-grasp pose;
- move with Lula IK or RMPflow;
- approach slowly;
- close the hand;
- verify lift;
- move to the bin;
- release and verify success.

**Exit criterion:** at least 20 consecutive nominal simulated successes.

## Phase 3 — Record and validate a smoke dataset

- Record 20–50 episodes.
- Convert to LeRobotDataset.
- Validate images, state, action, and timestamps.
- Replay episodes.
- Train ACT to overfit this dataset.

**Exit criterion:** ACT reproduces the small training set and rollout motion is semantically correct.

## Phase 4 — Randomized simulation pretraining

Add moderate variation:

- object position and yaw;
- initial arm pose;
- object mass and friction;
- camera pose and lighting;
- small command delay.

Generate 300–1,000 successful episodes.

**Exit criterion:** ACT succeeds on held-out simulation seeds and positions.

## Phase 5 — Add perception and sorting

- Train or integrate waste instance segmentation.
- Convert target masks and depth into 3D targets.
- Add class-to-bin routing.
- Condition the manipulation policy on target and destination.

Start with a can and one recycling bin before adding general waste.

**Exit criterion:** the system picks the selected object and uses the correct destination in simulation.

## Phase 6 — Real calibration and data

- Measure joint response and delay.
- Calibrate cameras and depth.
- Measure hand closing and slip behavior.
- Collect 30–50 carefully selected real episodes.
- Add corrected and recovery demonstrations.

**Exit criterion:** real episodes use the same LeRobot schema and action adapter as simulation.

## Phase 7 — Real-data fine-tuning

- Fine-tune with real-heavy sampling.
- Retain a smaller amount of simulation replay.
- Validate on held-out real scenes.

**Exit criterion:** offline predictions and shadow-mode actions are stable and correctly scaled.

## Phase 8 — Controlled deployment

Progress through:

```text
offline replay
→ shadow mode
→ low-speed single-object rollout
→ randomized object position
→ multiple waste classes
→ cluttered scene
```

**Exit criterion:** success and safety metrics satisfy the project threshold.

## Phase 9 — Optional improvements

Only after the baseline works:

- Diffusion Policy comparison;
- grasp-mode and closure outputs;
- recovery-focused DAgger;
- residual RL correction;
- high-level language or VLA instruction selector.

## Recommended first milestone

```text
one can
+ randomized tabletop position
+ fixed recycling bin
+ planner-generated simulation data
+ ACT simulation pretraining
+ 30–50 real episodes
+ low-speed real rollout
```

## 한국어 요약

전체 개발은 제어 계약 고정, USD 검증, scripted teacher, 소규모 dataset overfit, simulation randomization, perception 결합, 실제 데이터 fine-tuning, 제한 배포 순서로 진행합니다. 각 단계의 exit criterion을 통과하기 전에는 다음 복잡도를 추가하지 않는 것이 좋습니다.
