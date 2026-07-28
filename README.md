# SuperArm Workspace

Custom 5-DoF manipulator and AmazingHand workspace for Isaac Sim 5.1, LeRobot, imitation learning, and sim-to-real waste pick-and-place research.

## Project goals

- Control a custom 5-DoF arm with a low-dimensional hand command.
- Detect and sort waste into recycling and general-waste bins.
- Generate demonstrations in Isaac Sim and train LeRobot policies such as ACT.
- Mix simulated and real demonstrations for deployment on the physical robot.

## Wiki

Start with the [project wiki](omx_wiki/index.md).

Recommended reading order:

1. [Project overview](omx_wiki/project-overview.md)
2. [System architecture](omx_wiki/system-architecture.md)
3. [Isaac Sim workflow](omx_wiki/isaac-sim-workflow.md)
4. [LeRobot dataset pipeline](omx_wiki/lerobot-dataset.md)
5. [ACT training](omx_wiki/act-training.md)
6. [Waste sorting pipeline](omx_wiki/waste-sorting.md)
7. [Sim-to-real](omx_wiki/sim-to-real.md)
8. [Implementation roadmap](omx_wiki/roadmap.md)
9. [Troubleshooting](omx_wiki/troubleshooting.md)

## Current control contract

The first stable policy and SITL interface uses six values:

```text
[right_arm_pitch_joint,
 right_arm_roll_joint,
 right_arm_yaw_joint,
 right_elbow_pitch_joint,
 right_elbow_yaw_joint,
 amazinghand_grasp]
```

The first five values control the arm. `amazinghand_grasp` is a scalar in `[0, 1]`.

## Existing implementation entrypoints

- Isaac Sim setup: [`isaacsim_test/README.md`](isaacsim_test/README.md)
- SimReady USD and validation artifacts: [`isaacsim_test/outputs/simready/echo_full/`](isaacsim_test/outputs/simready/echo_full/)
- LeRobot Isaac adapter: [`isaacsim_test/lerobot/`](isaacsim_test/lerobot/)
- SITL validation tasks: [`docs/sitl/2026-07-02_core_validation/`](docs/sitl/2026-07-02_core_validation/)
- Hardware integration notes: [`integration_guide/`](integration_guide/)

## 한국어 요약

이 저장소는 커스텀 5-DoF 팔과 AmazingHand를 Isaac Sim 5.1 및 LeRobot에 연결하고, 시뮬레이션 demonstration으로 ACT를 학습한 뒤 소량의 실물 데이터를 섞어 sim-to-real을 수행하기 위한 프로젝트입니다. 전체 구현 순서는 `omx_wiki/index.md`에서 확인할 수 있습니다.
