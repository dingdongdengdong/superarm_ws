# Validation and Troubleshooting

## Debug in layers

Do not debug the entire AI stack at once.

```text
asset and articulation
→ command interface
→ teacher motion
→ dataset
→ policy
→ perception
→ sim-to-real deployment
```

## Asset and articulation

| Symptom | Check |
|---|---|
| Robot falls apart | Joint hierarchy, articulation root, mass, inertia |
| Links explode on contact | Collision overlap, tiny inertia, solver settings |
| Joint moves backward | Joint axis and sign mapping |
| Hand jitters | Finger collision, stiffness, damping, timestep |
| Reset is unstable | Initial overlap, invalid joint pose, residual velocity |

Use the existing AmazingHand notes in [`amazinghand-isaacsim.md`](amazinghand-isaacsim.md).

## Control contract

| Symptom | Likely cause |
|---|---|
| Wrong joint moves | Joint-order mismatch |
| Motion is ten times too large | Unit or action-scale mismatch |
| Simulation works but real arm jumps | Different absolute/relative action meaning |
| Motion is slow or delayed | Policy/control frequency mismatch |
| Dataset replay differs | Recorder saved planner waypoint instead of applied command |

Validation procedure:

1. send a one-hot action to one joint;
2. confirm only the named joint moves;
3. record and replay the action;
4. compare simulation and real action-adapter output.

## Teacher and motion planning

| Symptom | Check |
|---|---|
| IK fails near target | Reachability and 5-DoF orientation constraint |
| RMPflow stalls | Collision spheres, obstacle model, gains |
| Grasp closes too early | State-machine threshold and timing |
| Object is pushed away | Pre-grasp pose, approach speed, collision offset |
| Lift is detected incorrectly | Use object height and grasp/contact conditions together |

## Dataset

| Symptom | Check |
|---|---|
| ACT cannot overfit 20 episodes | Synchronization, normalization, action meaning |
| Grasp timing is always late | Image/action timestamp offset |
| Policy predicts mean pose | Mixed incompatible strategies or bad labels |
| Validation appears unrealistically good | Train/validation scene leakage |
| Real fine-tuning has no effect | Real samples are underweighted |

Every dataset should pass:

```text
schema validation
→ timestamp inspection
→ episode replay
→ action-range report
→ state/action ordering report
→ visual sample review
```

## Policy rollout

| Symptom | Check |
|---|---|
| Good offline loss, poor rollout | Compounding error and missing recovery data |
| Motion is overly smooth and unresponsive | Chunk size or temporal ensemble too large |
| Motion oscillates | Action delta too large, controller gain, short horizon |
| Reaches object but never grasps | Grasp label timing or insufficient hand-state input |
| Correct grasp, wrong destination | Conditioning or routing bug |

## Perception

| Symptom | Check |
|---|---|
| Transparent objects missed | Real-image data, lighting, depth limitations |
| Mask correct but grasp misses | Camera calibration and mask-to-3D projection |
| Wrong waste class | Class definition and real-data balance |
| Correct object not selected | Target-selector scoring and reachability filter |

## Sim-to-real

| Symptom | Primary experiment |
|---|---|
| Real arm lags simulation | Compare joint step response |
| Constant spatial offset | Hand-eye and base calibration |
| Intermittent overshoot | Delay jitter and controller interpolation |
| Object slips only in reality | Hand closure and friction measurements |
| Model fails under real lighting | Add real images and visual randomization |

## Required logs

Record these for every rollout:

```text
timestamp
camera frame ID
joint state
raw policy action
processed action
commanded target
measured target response
selected object and bin
success or failure reason
safety intervention
```

Without both raw and processed actions, action-adapter errors are difficult to diagnose.

## Existing repository checks

- [`isaacsim_test/README.md`](../isaacsim_test/README.md): environment and command verification;
- [`docs/sitl/2026-07-02_core_validation/`](../docs/sitl/2026-07-02_core_validation/): core SITL checks;
- [`docs/task_guides/`](../docs/task_guides/): focused implementation guides;
- [`integration_guide/`](../integration_guide/): hardware integration notes.

## 한국어 요약

문제는 asset, command, teacher, dataset, policy, perception, deployment 순서로 분리해서 확인합니다. 특히 `raw policy action`, `processed action`, `실제 command`, `measured state`를 모두 기록해야 simulation과 실물의 차이를 정확히 찾을 수 있습니다.
