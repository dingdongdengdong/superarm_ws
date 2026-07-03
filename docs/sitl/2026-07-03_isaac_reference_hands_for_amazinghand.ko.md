# Isaac Sim 레퍼런스 손 모델 기준과 AmazingHand 개선 기록 (2026-07-03)

## 목적

이번 브랜치의 목적은 Isaac Sim에 이미 포함된 Shadow Hand / Allegro Hand를 그대로 가져다 쓰는 것이 아니다.
두 모델을 **작동하는 덱스터러스 핸드의 구현 체크리스트**로 삼아, 현재 AmazingHand fallback을 더 안전한 순서로 개선하는 것이다.

현재 작업 원칙은 다음과 같다.

- AmazingHand 정체성은 유지한다.
- Isaac Sim physics는 지금처럼 단순화된 open-chain tree articulation을 우선 사용한다.
- CAD/MJCF closed-loop fidelity보다 접촉 안정성, 검증 가능성, LeRobot action 안정성을 먼저 본다.
- 이미 실행 중인 Isaac Sim 인스턴스는 건드리지 않는다. Runtime 검증은 별도 Isaac Sim 버전/인스턴스에서만 수행한다.

## 참고한 공식/업스트림 레퍼런스

| 레퍼런스 | 확인한 점 | 링크 |
| --- | --- | --- |
| Isaac Sim Robot Assets | Shadow Hand는 ShadowRobot 경로의 24 revolute joint 손, Allegro Hand는 WonikRobotics 경로의 16 torque-controlled revolute joint 손으로 제공된다. | <https://docs.isaacsim.omniverse.nvidia.com/5.0.0/assets/usd_assets_robots.html> |
| Isaac Lab Environments | Allegro/Shadow cube reorientation 환경이 있으며 Shadow vision/direct/OpenAI 계열과 DexSuite Kuka+Allegro 환경이 제공된다. | <https://isaac-sim.github.io/IsaacLab/main/source/overview/environments.html> |
| IsaacGymEnvs RL examples | Shadow Hand task는 복잡한 contact dynamics, tendon, position target control, fingertip/object observation, reset 흐름을 명시한다. Allegro task는 같은 cube manipulation을 Allegro로 수행한다. | <https://github.com/isaac-sim/IsaacGymEnvs/blob/main/docs/rl_examples.md> |
| Isaac Lab release notes | 2.3.0 계열에서 dexterous manipulation, Kuka+Allegro DexSuite, teleoperation/Mimic, gripper joint/contact tuning 개선이 언급된다. | <https://isaac-sim.github.io/IsaacLab/main/source/refs/release_notes.html> |

## 왜 Shadow/Allegro를 그대로 쓰지 않는가

Shadow Hand와 Allegro Hand는 이미 Isaac 생태계에서 잘 알려진 레퍼런스다.
하지만 AmazingHand 프로젝트의 목표는 RoboParty/RPO arm end-effector로 AmazingHand를 쓰고, LeRobot에는 먼저 `amazinghand_grasp.pos`를 안정적으로 노출하는 것이다.

따라서 Shadow/Allegro를 그대로 붙이면 다음 문제가 생긴다.

1. 실제 하드웨어와 다른 손이 된다.
2. wrist mount, cable, servo, action mapping이 AmazingHand와 맞지 않는다.
3. LeRobot dataset/action contract가 `5 arm joints + 1 hand scalar`에서 갑자기 고자유도 손으로 커진다.
4. 성공/실패가 AmazingHand 개선인지 레퍼런스 손 교체 효과인지 분리되지 않는다.

결론: Shadow/Allegro는 **대체 asset**이 아니라 **검증 기준표**로만 사용한다.

## Shadow Hand에서 배울 것

Shadow Hand는 24 revolute joint 수준의 고자유도 손이다.
여기서 AmazingHand가 배울 것은 joint 개수를 따라가는 것이 아니라 다음 항목이다.

- 손도 팔처럼 명시적인 articulation root, link, joint, limit이 필요하다.
- 접촉 task에서는 fingertip/object pose, joint position/velocity, reset sequence가 중요하다.
- tendon/equality 같은 복잡한 구조는 Isaac importer와 runtime 안정성을 따로 검증해야 한다.
- position target 기반 제어라도 contact dynamics가 어렵기 때문에 metric 기반 smoke test가 필요하다.

AmazingHand 적용 기준:

- 현재 closed-loop MJCF를 그대로 믿지 않고, Isaac-friendly tree articulation을 유지한다.
- 손가락 4개 x 2 joint = 8 generated joint를 먼저 안정화한다.
- finger motion, contact proxy, lift-retain을 각각 별도 metric으로 검증한다.

## Allegro Hand에서 배울 것

Allegro Hand는 Shadow보다 낮은 자유도인 16 torque-controlled revolute joint 손으로 제공된다.
Allegro는 “복잡하지만 학습 환경으로 다룰 수 있는 손”의 현실적인 기준에 가깝다.

배울 점은 다음과 같다.

- 고자유도 손도 처음부터 raw actuator 전체를 정책에 노출하지 않는 편이 안전하다.
- cube reorientation처럼 작은 물체 접촉 task는 collision shape, friction, drive tuning이 핵심이다.
- Kuka+Allegro DexSuite처럼 arm+hand 조합은 hand만이 아니라 wrist/arm pose와 함께 평가해야 한다.

AmazingHand 적용 기준:

- 지금은 `wrap`, `pinch`, `wide`, `single_finger` 같은 preshape를 먼저 둔다.
- raw 8-servo/8-joint action은 접촉과 안전 limit이 충분히 검증된 뒤에만 연다.
- LeRobot 기록용 action은 계속 `amazinghand_grasp.pos` scalar 호환을 유지한다.

## IsaacGymEnvs / Isaac Lab dexterous task에서 배울 것

레퍼런스 task들은 단순히 손 asset만 제공하지 않는다.
손이 작동하려면 다음 흐름이 같이 있어야 한다.

1. object reset
2. hand open pose
3. staged close
4. contact/fingertip metric
5. lift 또는 reorientation metric
6. 실패 시 어떤 stage에서 실패했는지 report

이번 브랜치에서 이 흐름은 다음 코드로 반영됐다.

- `build_preshape_grasp_validation_stage_specs()`
  - `single_finger -> pinch -> wrap` 순서로 runtime 검증 stage를 정의한다.
- `build_hand_preshape_position_command()`
  - single finger, pinch, wrap을 명시적으로 command한다.
- runtime report key
  - `preshape_grasp_validation`을 `lift_retain_validation` 전에 기록하도록 했다.

## AmazingHand 현재 구현 상태

현재 핵심 파일은 다음과 같다.

| 파일 | 역할 |
| --- | --- |
| `isaacsim_test/isaacsim/graspable_hand_urdf.py` | AmazingHand MJCF/CAD visual을 Isaac-friendly 8-joint tree URDF로 생성한다. |
| `isaacsim_test/isaacsim/robot_arm_hand_from_zip.py` | arm+hand zip을 준비/변환/runtime 검증하고 grasp/contact/lift report를 작성한다. |
| `isaacsim_test/test_graspable_hand_urdf.py` | generated hand topology, visual mapping, scalar/preshape target mapping을 검증한다. |
| `isaacsim_test/test_robot_arm_hand_from_zip.py` | command mapping, contact proxy, preshape validation stage, runtime report ordering을 검증한다. |

현재 physics 원칙:

- visual mesh는 visual로만 사용한다.
- collision은 primitive proxy를 사용한다.
- closed-loop equality constraint는 Isaac fallback physics에서는 사용하지 않는다.
- 작은 screw/washer/detail visual은 skeleton-first 검증 단계에서 제외한다.

## 이번 브랜치 개선: scalar에서 preshape로

기존 인터페이스:

```text
amazinghand_grasp.pos = 0.0 -> open
amazinghand_grasp.pos = 1.0 -> close/wrap
```

이번 브랜치의 추가 개념:

```text
grasp_type = wrap / pinch / wide / single_finger
grasp_amount = 0.0 ~ 1.0
```

코드 기준:

- `grasp_scalar_to_hand_joint_targets(grasp)`는 기존 wrap 동작을 유지한다.
- `grasp_preshape_to_hand_joint_targets(grasp_amount, grasp_type)`가 추가됐다.
- `build_hand_grasp_position_command(..., grasp_type="wrap")`는 기본값으로 기존 호출을 깨지 않는다.
- `build_hand_preshape_position_command()`는 runtime/debug stage에서 single finger, pinch, wrap을 명시적으로 테스트한다.

## Runtime 검증 운영 제약

사용자 지시: 현재 다른 작업이 기존 Isaac Sim에서 실행 중이다.
따라서 이 브랜치에서 다음은 금지한다.

- 현재 실행 중인 Isaac Sim 세션에 stage를 열거나 runtime test를 실행하는 것.
- 기존 Isaac Sim process를 kill/restart하는 것.
- 같은 Nucleus/cache/output을 공유해 현재 작업을 오염시키는 것.

허용되는 검증:

1. host-side unit/static test
2. `py_compile`
3. `git diff --check`
4. 별도 설치/버전의 Isaac Sim에서만 runtime smoke 실행

### Isaac Sim 6.0 multi-instance 원칙

후속 지시에 따라 새 runtime은 Isaac Sim 5.1 병렬 UI가 아니라 6.0 multi-instance 방식으로 분리한다.
Isaac Sim 6.0 공식 문서는 Docker Compose가 Isaac Sim + WebRTC web-viewer 배포를 지원하고,
multi-instance에서는 project name, unique port, per-instance data directory, GPU pinning을 사용한다고 설명한다.

이번 브랜치에는 레포-local runner를 추가했다.

```bash
# AmazingHand convert/runtime smoke: 별도 6.0 disposable container 2개(convert, runtime)
isaacsim_test/run_isaacsim60_multi_instance.sh run-hand

# UI/WebRTC용 6.0 streaming container: 기본 포트와 데이터 루트가 기존 세션과 분리됨
ISAACSIM60_INSTANCE=amazinghand \
ISAACSIM_SIGNAL_PORT=49200 \
ISAACSIM_STREAM_PORT=48100 \
WEB_VIEWER_PORT=8211 \
isaacsim_test/run_isaacsim60_multi_instance.sh start-ui

isaacsim_test/run_isaacsim60_multi_instance.sh status
isaacsim_test/run_isaacsim60_multi_instance.sh stop-ui
```

기본값:

| 항목 | 값 | 이유 |
| --- | --- | --- |
| image | `nvcr.io/nvidia/isaac-sim:6.0.0` | 6.0 multi-instance 지원 기준 |
| signal TCP | `49200` | 기본 `49100`과 충돌 회피 |
| stream UDP | `48100` | 기본 `47998`과 충돌 회피 |
| web viewer TCP | `8211` | 기본 `8210`과 충돌 회피; 공식 Compose web-viewer용 예약 |
| data root | `~/docker/isaac-sim-6-amazinghand` | cache/config/log 분리 |
| output root | `isaacsim_test/outputs/robot_arm_hand_graspable_<RUN_ID>` | 현재 작업 evidence 오염 방지 |

주의:

- WebRTC livestream은 host network와 실제 host IP/port가 중요하다.
- browser web-viewer까지 필요하면 NVIDIA 공식 Docker Compose web-viewer를 같은 port set으로 사용한다.
- 이 runner의 `run-hand`는 UI가 아니라 headless script 검증이며, Isaac 6.0 종료 assertion을 피하기 위해
  disposable process에서만 `ROBOT_ARM_HAND_ISAAC_FAST_CLOSE=1`을 켠다.
- 기본 5.1/local 경로는 fast close를 켜지 않으므로 기존 compose workflow는 그대로 유지된다.

별도 Isaac Sim runtime을 사용할 때는 output root도 분리한다.
예:

```bash
ROBOT_ARM_HAND_OUTPUT_ROOT=/workspace/superarm_ws/isaacsim_test/outputs/robot_arm_hand_graspable_YYYYMMDD_reference_runtime \
ROBOT_ARM_HAND_SCREENSHOT_OUTPUT_DIR=/workspace/superarm_ws/isaacsim_test/artifacts/robot_arm_hand_graspable_YYYYMMDD_reference_runtime \
<OTHER_ISAACSIM>/python.sh /workspace/superarm_ws/isaacsim_test/isaacsim/robot_arm_hand_from_zip.py --mode all
```

## 다음 단계

1. host-side 검증은 이번 브랜치에서 완료한다.
2. 별도 Isaac Sim 버전/인스턴스를 찾아 runtime smoke를 실행한다.
3. runtime report에서 다음 키를 확인한다.
   - `runtime_validation.grasp_validation.status`
   - `runtime_validation.preshape_grasp_validation.status`
   - `runtime_validation.finger_motion_validation.status`
   - `runtime_validation.lift_retain_validation.status`
4. runtime이 PASS/WARN이면 screenshot과 JSON evidence를 이 문서 또는 후속 runtime evidence 문서에 링크한다.

## 판정 기준

- `scalar-compatible PASS`: 기존 `amazinghand_grasp.pos` 흐름이 깨지지 않는다.
- `preshape command PASS`: single_finger, pinch, wrap command가 의도한 joint만 움직인다.
- `reference checklist PASS`: Shadow/Allegro를 대체 asset이 아닌 비교 기준으로 기록한다.
- `runtime isolated PASS`: 기존 Isaac Sim이 아닌 별도 Isaac Sim에서만 runtime evidence를 만든다.
