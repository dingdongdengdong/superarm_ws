# Project Overview

## Objective

Build a custom manipulator system that can identify waste, pick it up, and place it into either a recycling bin or a general-waste bin.

The project combines:

- a custom 5-DoF arm;
- AmazingHand, initially exposed as a single grasp scalar;
- Isaac Sim 5.1 for simulation and demonstration generation;
- LeRobot for dataset management and policy training;
- ACT as the first learned vision-action policy;
- a small amount of real data for sim-to-real adaptation.

## First operational scope

The first complete milestone should be deliberately narrow:

```text
one rigid object type
+ one fixed camera layout
+ one fixed recycling bin
+ randomized object position
+ 5 arm actions and one grasp action
```

Do not begin with every waste type, full finger-level control, reinforcement learning, and language conditioning at the same time.

## Model strategy

The recommended first system is modular:

```text
YOLO instance segmentation
        ↓
class-to-bin rule
        ↓
target object and target bin
        ↓
ACT or motion planner
        ↓
5-DoF arm + grasp scalar
```

The modules have different responsibilities:

| Module | Responsibility |
|---|---|
| Perception | Detect and segment each waste object |
| Routing rule | Map waste class to recycling or general waste |
| Target selector | Choose which visible object to pick next |
| Motion teacher | Produce successful simulated demonstrations |
| ACT policy | Learn image/state-to-action behavior |
| Safety layer | Enforce joint, speed, workspace, and stop limits |

## Why ACT first

ACT is a practical first learned policy because it:

- predicts a chunk of future actions;
- is lighter than a full VLA;
- works with multi-camera image and robot-state inputs;
- has a detailed LeRobot training workflow;
- can be trained from simulated and real demonstrations.

Language is not required while the sorting rule is fixed. Use explicit inputs such as `target_class` and `destination_bin` before adding a language model.

## Current repository state

The repository already contains:

- a validated SimReady USD generated from the complete CAD model;
- a six-dimensional LeRobot command contract;
- Isaac Sim 5.1 Docker and ROS 2 integration files;
- AmazingHand visual and contact tests;
- SITL validation guides;
- direct-hardware LeRobot notes.

See [`isaacsim_test/README.md`](../isaacsim_test/README.md) for the current simulation entrypoint.

## Success criteria

A useful first policy should be evaluated by:

- pick success rate;
- lift success rate;
- correct-bin placement rate;
- wrong-bin rate;
- average task duration;
- collision and safety-stop count;
- recovery success after small pose disturbances.

## 한국어 요약

첫 목표는 한 종류의 물체를 랜덤 위치에서 집어 고정된 통에 넣는 전체 파이프라인을 완성하는 것입니다. 인식은 YOLO, 분리 규칙은 rule table, manipulation은 ACT 또는 planner로 분리하고, 손은 처음에는 `grasp scalar` 하나로 단순화하는 것이 좋습니다.
