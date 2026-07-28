# System Architecture

## Recommended layered design

```text
External RGB-D camera ─┐
Wrist camera ──────────┼→ Perception → Target selector → Manipulation policy
Joint and hand state ──┘                                      ↓
                                                      Shared action adapter
                                                               ↓
                                                Isaac Sim or physical robot
```

Keep semantic decisions and continuous motor control separate.

## Perception layer

Recommended initial output:

```text
class_id
confidence
instance_mask
depth-derived 3D position
```

Suggested first classes:

- aluminum can;
- PET bottle;
- paper;
- plastic bag;
- cup;
- tissue;
- unknown.

The target selector should expose only the selected target to the manipulation policy. This prevents the policy from having to infer both task selection and motion from scratch.

## Task logic layer

Use a deterministic rule table for fixed waste sorting:

```python
WASTE_TO_BIN = {
    "aluminum_can": "recycle",
    "pet_bottle": "recycle",
    "paper": "recycle",
    "tissue": "general",
    "unknown": "reject",
}
```

Language conditioning is unnecessary while these rules are fixed.

## Manipulation layer

Two manipulation paths should coexist:

### Teacher path

```text
exact simulator object pose
        ↓
Lula IK / RMPflow / state machine
        ↓
successful action trajectory
```

This path creates demonstrations and provides a deterministic baseline.

### Student path

```text
camera images + robot state + target condition
        ↓
ACT
        ↓
action chunk
```

The student must only use observations that can also be produced on the real robot.

## Stable six-dimensional control contract

The current repository contract is:

```text
0 right_arm_pitch_joint
1 right_arm_roll_joint
2 right_arm_yaw_joint
3 right_elbow_pitch_joint
4 right_elbow_yaw_joint
5 amazinghand_grasp
```

The first model should preserve this shape `(6,)` across:

- simulation control;
- dataset recording;
- policy training;
- replay;
- real-robot inference.

## Shared action adapter

Use one post-processing implementation for simulation and reality:

```text
normalized policy output
        ↓
reorder by joint name
        ↓
apply action scale
        ↓
clip to soft limits
        ↓
apply velocity limit
        ↓
send joint targets and grasp command
```

Never duplicate this logic in separate simulation and real scripts.

## Safety layer

The learned model must not be the only safety mechanism. Add:

- hard motor limits;
- policy soft limits;
- maximum action delta;
- maximum joint velocity;
- workspace boundary;
- collision or proximity stop;
- command timeout;
- emergency stop.

## Deployment modes

| Mode | Motor command | Purpose |
|---|---:|---|
| Offline replay | Disabled | Validate preprocessing and action shape |
| Shadow mode | Disabled | Observe live predictions |
| Low-speed rollout | Enabled with strict limits | First real tests |
| Normal rollout | Enabled | Validated operation |

## 한국어 요약

시스템을 인식, 분류 규칙, manipulation, 안전 계층으로 분리합니다. 현재 6차원 제어 계약을 simulation·dataset·ACT·실물에서 동일하게 유지하고, action scaling과 clipping은 하나의 공통 adapter에서 처리해야 합니다.
