# 07 - Validation Gates

## Goal

Prevent unsafe or low-quality data from entering training by using explicit gates
between hardware bring-up, teleoperation, recording, training, and evaluation.

## Gate A - Hardware safe to move

```text
[ ] Emergency stop works.
[ ] Power limits are set.
[ ] Correct CAN interface is known.
[ ] Correct motor IDs are known.
[ ] Joint signs are verified.
[ ] Joint soft limits are configured.
[ ] AmazingHand opens and closes safely.
[ ] Cables do not collide during slow movement.
```

Do not record data before Gate A passes.

## Gate B - LeRobot wrapper safe

```text
[ ] Robot type instantiates.
[ ] connect() and disconnect() work repeatedly.
[ ] get_observation() returns all expected keys.
[ ] send_action() clamps joint targets.
[ ] send_action() clamps relative movement.
[ ] send_action() clamps hand scalar.
[ ] No-op action does not move the arm unexpectedly.
[ ] Tiny 0.5 degree joint command moves the expected joint.
```

Do not train a RoboParty policy before Gate B passes.

## Gate C - Teleoperation usable

```text
[ ] Operator can complete 5 consecutive cube-to-tray trials.
[ ] The arm does not hit the table or tray.
[ ] Hand timing is controllable.
[ ] Camera views show the full task.
[ ] Reset position is repeatable.
[ ] Failed demos can be identified.
```

Do not record baseline data before Gate C passes.

## Gate D - Dataset quality

```text
[ ] Episode count matches target.
[ ] No missing camera frames.
[ ] No missing action keys.
[ ] No unexpected action spikes.
[ ] Task labels are consistent.
[ ] Failed episodes are removed or labeled.
[ ] Dataset visualizer confirms usable videos.
```

Do not train the baseline before Gate D passes.

## Gate E - Policy evaluation

```text
[ ] Evaluation uses the same fixture as training.
[ ] Emergency stop operator is present.
[ ] 20 trials are recorded.
[ ] Success rate is calculated.
[ ] Failure labels are assigned.
[ ] Next dataset change is based on observed failures.
```

## Minimum reporting format

Create:

```text
docs/task_guides/evaluation_log.md
```

Use:

```markdown
# Evaluation Log

| Date | Dataset | Policy | Trials | Successes | Success rate | Top failure |
| --- | --- | --- | ---: | ---: | ---: | --- |
| 2026-06-22 | rpo5_ah_cube_tray_v1 | act_rpo5_ah_cube_tray_v1 | 20 | 0 | 0% | not run |
```

## Done when

```text
[ ] Every gate has a named owner.
[ ] Gate pass/fail is recorded in markdown.
[ ] No policy is trained on unreviewed debug data.
[ ] Real-robot evaluation results are documented.
```
