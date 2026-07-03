# AmazingHand motor-faithful finger/frame 개선을 위한 NVIDIA reference hand 조사 (2026-07-03)

## 결론

이번 단계의 목표는 Isaac Sim 5.1에서 AmazingHand를 Shadow/Allegro/Inspire 계열 손으로 바꾸는 것이 아니다.
목표는 **AmazingHand 구조를 유지한 채**, 실제 SCS0009 servo 특성을 반영해서 finger movement와 hand frame을 단계적으로 더 믿을 수 있게 만드는 것이다.

현재 기준 baseline은 아래 5.1 runtime이다.

- Output: `isaacsim_test/outputs/robot_arm_hand_graspable_20260703_115016_KST_mjcf_motor_anchor_back_lip_runtime/`
- Screenshot root: `isaacsim_test/artifacts/visual_verification_20260703_115016_KST_mjcf_motor_anchor_back_lip_runtime/`
- Status: `PASS_WITH_FALLBACK`
- Runtime/grasp/preshape/finger/lift-retain: 모두 `PASS`
- Capture method: `focused_viewport`

이 문서는 다음 구현 단계의 기준이다.

1. AmazingHand는 4 finger / 8 servo / 8 generated joint 체계를 유지한다.
2. 각 finger는 2개 SCS0009 servo가 반대 방향으로 움직이는 구조로 본다.
3. Isaac hand frame은 4개 fixed `finger*_base`와 2-link generated finger를 유지한다.
4. MJCF `custom_servo_horn*` anchor를 finger base frame 기준으로 삼는다.
5. Grasp 성공률 튜닝은 finger movement와 motor-frame report가 먼저 안정된 뒤 진행한다.

## 참고한 NVIDIA / Isaac reference

| Reference | 관찰 | AmazingHand에 적용할 점 |
|---|---|---|
| Isaac Sim 5.1 Robot Assets | Isaac Sim robot assets 문서는 robot asset들이 Content Browser의 `Isaac Sim/Robots` 아래 제공된다고 설명한다. | reference hand는 asset/library 기준표로만 사용한다. AmazingHand identity를 유지한다. |
| Shadow Hand | IsaacGymEnvs 문서는 Shadow Hand task를 complex contact dynamics와 tendon을 포함한 dexterity task로 설명하고, position target으로 hand motion을 제어한다고 기록한다. | closed-loop/tendon을 그대로 복제하지 말고, tendon/constraint가 있는 손은 drive/report/observation이 중요하다는 점만 가져온다. |
| Allegro Hand | Isaac Lab은 Allegro cube reorientation environment를 제공하고, IsaacGymEnvs는 Shadow Hand와 같은 cube manipulation task를 Allegro로 수행한다고 설명한다. | 16-DOF hand처럼 raw DOF를 바로 열지 말고, 먼저 preshape와 contact validation을 분리한다. |
| Inspire / humanoid pick-place tasks | Isaac Lab에는 Unitree G1 + InspireFTP pick-place task가 있다. | 손 단독이 아니라 arm pose, wrist frame, grasp object reset이 함께 안정되어야 한다. |
| Kuka + Allegro DexSuite | Isaac Lab에는 Kuka+Allegro lift/reorient task가 있다. | AmazingHand도 hand-only screenshot보다 arm+hand+object smoke를 유지해야 한다. |

Sources:

- NVIDIA Isaac Sim 5.1 Robot Assets: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_robots.html
- Isaac Lab Available Environments: https://isaac-sim.github.io/IsaacLab/main/source/overview/environments.html
- IsaacGymEnvs RL examples: https://github.com/isaac-sim/IsaacGymEnvs/blob/main/docs/rl_examples.md

## AmazingHand motor facts to preserve

Upstream local config:

- Source: `AmazingHand/Demo/AHControl/config/r_hand.toml`
- Servo model: `SCS0009`
- Servo IDs: `1..8`
- Right hand layout:

| Generated joint | Finger | Servo ID | Offset rad | Offset deg | Invert |
|---|---:|---:|---:|---:|---|
| `finger1_motor1` | 1 index | 1 | `0.12217304763960307` | 7 | false |
| `finger1_motor2` | 1 index | 2 | `0.08726646259971647` | 5 | false |
| `finger2_motor1` | 2 middle | 3 | `0.0` | 0 | false |
| `finger2_motor2` | 2 middle | 4 | `0.12217304763960307` | 7 | false |
| `finger3_motor1` | 3 ring | 5 | `0.08726646259971647` | 5 | false |
| `finger3_motor2` | 3 ring | 6 | `0.12217304763960307` | 7 | false |
| `finger4_motor1` | 4 thumb | 7 | `0.0` | 0 | false |
| `finger4_motor2` | 4 thumb | 8 | `0.12217304763960307` | 7 | false |

Upstream examples use opposite directions per finger pair:

- open-like command: motor1 negative, motor2 positive
- close-like command: motor1 positive, motor2 negative
- example speed: `write_goal_speed(..., 6)`

Therefore the Isaac report must distinguish two coordinate systems:

1. **generated joint target**: positive two-link revolute target used by the stable Isaac tree hand.
2. **servo command target**: SCS0009 target with offset and motor-pair direction matching the real AmazingHand examples.

## Current implemented hand structure

Current sim structure remains intentionally simpler than the real closed-loop MJCF hand.

```text
r_wrist_interface
└── palm
    ├── finger1_base  fixed at MJCF custom_servo_horn anchor
    │   └── finger1_proximal --finger1_motor1--> finger1_distal --finger1_motor2-->
    ├── finger2_base  fixed at MJCF custom_servo_horn_2 anchor
    ├── finger3_base  fixed at MJCF custom_servo_horn_3 anchor
    └── finger4_base  fixed at MJCF custom_servo_horn_4 anchor
```

This is not exact closed-loop AmazingHand kinematics. It is a stable Isaac 5.1 fallback tree that can be measured and improved step by step.

## Implementation criteria from this research

### 1. Motor-faithful command metadata

Every hand command should report:

- controlled generated joint names,
- controlled SCS0009 servo IDs,
- per-joint servo offset,
- per-joint invert flag,
- generated joint target rad,
- real servo target rad,
- default servo speed used by upstream example.

### 2. Finger frame evidence

Every focused finger motion result should report:

- `finger_index`,
- role: index/middle/ring/thumb,
- `finger*_base` palm-local xyz,
- source MJCF anchor body,
- servo pair IDs,
- target/achieved generated joint rad,
- proximal/distal link translation delta.

This prevents a screenshot-only claim. A close-up image is still required, but PASS must also have motor/frame metrics.

### 3. Reference-hand lessons applied narrowly

Use Shadow/Allegro/Inspire lessons only for these decisions:

- keep explicit joint/drive/contact reporting,
- separate finger motion validation from grasp validation,
- keep arm+hand+object validation after finger motion is stable,
- use preshapes before raw 8-servo policy control,
- do not call CAD shell alignment PASS until shell/linkage/frame/contact all agree.

## Next debugging order

1. Motor-faithful metadata and tests.
2. Focused 5.1 finger movement report using servo pair + base-frame evidence.
3. Hand frame cleanup if any finger base anchor is visibly wrong.
4. Preshape contact smoke.
5. Grasp/lift-retain tuning after finger movement is trusted.

## Non-goals

- Do not switch this hand path to Isaac Sim 6.0.
- Do not replace AmazingHand with Shadow Hand, Allegro Hand, InspireFTP, or any other hand asset.
- Do not expose raw 8-servo LeRobot policy control before safe hardware limits are measured.
- Do not judge finger movement from whole-scene screenshots only.
