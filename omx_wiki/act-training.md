# ACT Training

## Why ACT is the first policy

ACT is the recommended first learned manipulation policy for this project because it provides a direct path from synchronized camera/state/action demonstrations to action-chunk prediction.

```text
images + robot state + task condition
                ↓
               ACT
                ↓
future arm and grasp actions
```

A full VLA is not required while the task is fixed and the destination rule is deterministic.

## First policy inputs

Recommended inputs:

```text
external RGB
wrist RGB
6D robot state
target object indication
destination bin ID
```

Recommended outputs:

```text
5 arm actions
1 grasp scalar
```

## Target conditioning options

Use one of these approaches:

### Option A: separate structured condition

```text
target class embedding
+ destination-bin embedding
```

This is clean but requires modifying the policy input path.

### Option B: target mask as an image channel

```text
RGB image + selected target mask
```

This is often easier for an initial custom implementation.

### Option C: crop around selected target

Feed both the full scene and a target crop. This can improve small-object handling while preserving global context.

## Training sequence

### Step 1: overfit the smoke dataset

Train on 20–50 clean episodes until the model can reproduce them.

If ACT cannot overfit this small dataset, do not collect more data. Check:

- state/action order;
- image timestamps;
- normalization;
- action semantics;
- grasp timing;
- episode boundaries.

### Step 2: simulation baseline

Train on several hundred randomized simulated episodes.

Evaluate on held-out object positions and random seeds.

### Step 3: real-data fine-tuning

Collect a small real dataset using a planner, teleoperation, or corrected rollout. Fine-tune with real-heavy sampling while retaining some simulation replay.

### Step 4: failure-focused iteration

Add episodes from states where the current policy fails:

- slightly misaligned pre-grasp;
- object moved during approach;
- partial closure;
- dropped object;
- destination placement error.

This is more useful than collecting only additional perfect trajectories.

## Chunk size

Chunk size controls the tradeoff between smooth temporal behavior and closed-loop reactivity.

- larger chunks: smoother and more coherent, but less reactive;
- smaller chunks: more reactive, but may be noisier.

Start with a moderate chunk and tune using task duration, control frequency, and grasp-contact behavior. Contact-sensitive phases may require shorter effective horizons than free-space transport.

## Evaluation metrics

Report separate rates for:

```text
reach success
grasp success
lift success
correct-bin success
complete episode success
```

Also record:

- average completion time;
- wrong-bin count;
- collision count;
- action saturation rate;
- recovery success;
- performance by object class.

## When to compare Diffusion Policy

Compare Diffusion Policy after ACT works if demonstrations contain several distinct valid grasp strategies and ACT averages them poorly.

Examples:

- pinch versus wrap grasp;
- left-side versus right-side approach;
- several valid orientations for irregular waste.

## When to add RL

Add RL only after the imitation policy already performs the task.

Useful RL targets:

- final grasp alignment;
- recovery after a missed grasp;
- shorter task duration;
- robustness to physics variation.

A residual structure is safer than replacing the full policy initially:

```text
final action = ACT action + small RL correction
```

## 한국어 요약

ACT는 먼저 20~50개 episode에 overfit되는지 확인한 뒤, simulation 데이터로 baseline을 만들고 실제 데이터를 높은 비율로 fine-tuning합니다. 성능 개선은 완벽한 성공 데이터만 늘리기보다 현재 정책이 실패하는 상태와 recovery 데이터를 추가하는 방식이 효과적입니다.
