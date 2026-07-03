# Robotov2.0 Right Arm + AmazingHand SITL 우선 진행 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` 또는 `superpowers:executing-plans`로 task 단위 실행. 각 task는 checkbox (`- [ ]`)로 추적하고, 완료 시 검증 증거를 남긴다.

**Goal:** LeRobot leader arm 입력으로 Robotov2.0 RoboParty right arm + AmazingHand 조합을 먼저 시뮬레이션에서 검증하고, 이후 DM4340P 실제 모터 검증, AmazingHand open-source test, 데이터셋 수집, 모방학습으로 넘어간다.

**Target hardware / asset:**
- Arm: RoboParty / Robotov2.0 right arm, 5-DOF.
- Motor path: right arm candidate `can3`, motor IDs `19-23`, DM motor group. 실제 배선에서 재검증 필요.
- Hand: AmazingHand right hand. 초기 LeRobot interface는 `amazinghand_grasp.pos` scalar 하나로 시작.
- SITL URDF: `isaacsim_test/outputs/simready/echo_full/sitl/roboto_v2_right_arm_amazinghand_full.urdf`.
- Current SITL LeRobot wrapper: `isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py`.
- Current SITL config: `isaacsim_test/lerobot/rpo_arm_isaacsim.yaml`.

---

## 1. 결론: 제안한 순서는 괜찮은가?

네, 방향은 좋다. 특히 **leader arm -> simulation follower -> evidence -> real DM motor parity -> AmazingHand test -> dataset** 순서는 안전하고 재현 가능하다.

단, 아래 4가지는 gate로 강제해야 한다.

1. **Leader arm 입력을 실제 DM4340P 모터에 바로 연결하지 않는다.**
   먼저 Isaac Sim SITL follower에만 연결해 joint order, sign, limit, hand scalar를 검증한다.
2. **SITL feature 이름과 실제 하드웨어 feature 이름을 섞지 않는다.**
   SITL은 현재 `right_arm_pitch_joint.pos` 계열이고, 실제 하드웨어 문서는 `rpo_arm_j1.pos` 계열을 쓴다. 둘 사이 mapping table을 먼저 만든다.
3. **AmazingHand는 처음부터 8-servo raw control로 열지 않는다.**
   open-source 예제로 servo ID, safe open/close target을 확인한 뒤, LeRobot에는 우선 `amazinghand_grasp.pos` 하나만 노출한다.
4. **모방학습 데이터는 Gate A/B/C/D를 통과한 뒤 수집한다.**
   디버그 데이터와 학습용 baseline data를 섞지 않는다.

---

## 2. 난이도와 병목 요약

| 단계 | 난이도 | 병목도 | 핵심 병목 |
|---|---:|---:|---|
| 0. 공통 contract 고정 | 중 | 높음 | SITL feature와 real hardware feature mapping 불일치 |
| 1. Leader arm -> SITL 연동 | 중상 | 높음 | leader output을 5-arm + 1-hand scalar로 안정 변환 |
| 2. Robotov2.0 right arm URDF/Isaac Sim 동작 검증 | 상 | 높음 | URDF articulation, joint limit, screenshot/evidence 자동화 |
| 3. DM4340P motor parity 검증 | 상 | 매우 높음 | CAN interface, motor ID, sign, zero, current limit, e-stop |
| 4. AmazingHand open-source test | 중상 | 높음 | servo ID, safe target, wiring strain relief, wrist collision |
| 5. 통합 teleop dry run | 상 | 매우 높음 | leader motion이 arm/hand를 동시에 과격하게 움직이지 않게 clamp |
| 6. 디버그 데이터셋 10 episode | 중 | 중 | camera, reset pose, failed demo labeling |
| 7. baseline 데이터셋 50-100 episode + imitation learning | 중상 | 중 | dataset quality, action spike, task consistency |

**가장 큰 병목:** DM4340P 실제 모터 parity다. 시뮬레이션이 통과해도 실제 모터 sign, CAN ID, zero offset, soft limit이 틀리면 데이터 수집 단계로 가면 안 된다.

---

## 3. 권장 전체 순서

```text
문서/contract 고정
  -> leader arm 입력 확인
  -> leader arm -> SITL follower 연동
  -> Robotov2.0 right arm URDF sweep/screenshot evidence
  -> SITL-to-real mapping table 작성
  -> DM4340P no-motion/read-only 확인
  -> DM4340P single-joint tiny motion 확인
  -> AmazingHand open-source open/close test
  -> arm + hand integrated no-op/tiny teleop
  -> debug dataset 10 episodes
  -> dataset QA
  -> baseline dataset 50-100 episodes
  -> ACT/SmolVLA 등 imitation learning baseline
```

---

## 4. 안정적인 1차 action contract

### 4.1 SITL side

현재 SITL wrapper와 config는 이 순서를 사용한다.

| Index | SITL feature | 의미 | 초기 범위 |
|---:|---|---|---|
| 0 | `right_arm_pitch_joint.pos` | right arm pitch | URDF limit 기준 |
| 1 | `right_arm_roll_joint.pos` | right arm roll | URDF limit 기준 |
| 2 | `right_arm_yaw_joint.pos` | right arm yaw | URDF limit 기준 |
| 3 | `right_elbow_pitch_joint.pos` | elbow pitch | URDF limit 기준 |
| 4 | `right_elbow_yaw_joint.pos` | elbow yaw | URDF limit 기준 |
| 5 | `amazinghand_grasp.pos` | hand open/close intent | `0.0` open, `1.0` close |

### 4.2 Real hardware side

실제 Robotov2.0 right arm에서는 우선 아래 alias를 쓴다.

| Real feature | 예상 motor ID | 예상 CAN | 예상 sign | 검증 상태 |
|---|---:|---|---:|---|
| `rpo_arm_j1.pos` | 19 | can3 | -1 | pending |
| `rpo_arm_j2.pos` | 20 | can3 | 1 | pending |
| `rpo_arm_j3.pos` | 21 | can3 | 1 | pending |
| `rpo_arm_j4.pos` | 22 | can3 | -1 | pending |
| `rpo_arm_j5.pos` | 23 | can3 | 1 | pending |
| `amazinghand_grasp.pos` | hand serial adapter | serial | scalar | pending |

이 값은 시작점일 뿐이다. 실제 robot wiring에서 read-only inspection과 tiny motion으로 다시 확인해야 한다.

---

## 5. 파트별 업무분담

### [핵심 검증부](2026-07-02_core_validation/README.md)

| Task | 난이도 | 병목도 | 산출물 |
|---|---:|---:|---|
| [C01. 현재 LeRobot checkout과 local wrapper 구조 확인](2026-07-02_core_validation/C01_lerobot_wrapper_layout.ko.md) | 중 | 중 | `team_notes/local_lerobot_layout_<name>.md` |
| [C02. leader arm output shape 확인](2026-07-02_core_validation/C02_leader_arm_output_shape.ko.md) | 중 | 높음 | leader raw output log, port, calibration note |
| [C03. leader output을 6D SITL action으로 변환](2026-07-02_core_validation/C03_leader_to_sitl_6d_action.ko.md) | 중상 | 높음 | teleop adapter 또는 bridge note |
| [C04. `roboto_v2_right_arm_amazinghand_full.urdf` joint/limit audit](2026-07-02_core_validation/C04_robotov2_urdf_joint_limit_audit.ko.md) | 중 | 중 | joint limit table |
| C05. SITL no-op test | 중 | 높음 | no-op evidence JSON |
| C06. SITL joint sweep test | 중상 | 높음 | sweep evidence JSON + screenshots |
| C07. SITL-to-real mapping table 작성 | 중 | 매우 높음 | `hardware_parity_checklist.md` |
| C08. DM motor command clamp 구현/검증 | 상 | 매우 높음 | clamp unit test, no hardware movement |
| C09. dataset schema check | 중 | 중 | `observation.state`/`action` shape `(6,)` report |
| C10. debug dataset 10 episode record/replay | 중상 | 중 | dataset QA report |

### 전자 배선 및 통신부

| Task | 난이도 | 병목도 | 산출물 |
|---|---:|---:|---|
| E01. e-stop / power cut 동작 확인 | 상 | 매우 높음 | Gate A check |
| E02. power supply current limit 설정 | 중 | 높음 | current limit value log |
| E03. CAN adapter와 `can3` 확인 | 중 | 높음 | `ip link`, bitrate, adapter serial |
| E04. motor IDs `19-23` read-only 확인 | 상 | 매우 높음 | motor ID scan log |
| E05. DM4340P sign 검증 계획 작성 | 상 | 매우 높음 | one-joint-at-a-time protocol |
| E06. tiny motion으로 sign/limit 확인 | 상 | 매우 높음 | bringup log |
| E07. AmazingHand voltage/serial/servo ID 확인 | 중상 | 높음 | servo map |
| E08. hand wiring strain relief, fuse/current protection | 중 | 중 | wiring checklist |


### 시뮬레이션 체크

| Task | 난이도 | 병목도 | 산출물 |
|---|---:|---:|---|
| M01. Robotov2.0 right arm 고정 fixture 확인 | 중 | 높음 | fixture photo/note |
| M02. wrist-to-AmazingHand adapter 확인 | 중상 | 높음 | mount check |
| M03. cable routing / collision check | 중 | 높음 | cable path checklist |
| M04. neutral pose, reset pose 정의 | 중 | 중 | pose note |
| M05. cube/tray/table 위치 고정 | 낮음 | 중 | fixture dimensions |
| M06. camera view 후보 정리 | 중 | 중 | front/wrist view screenshots |
| M07. hand mass / wrist torque margin 확인 | 중상 | 높음 | risk note |

---

## 6. 상세 task checklist

### Phase 0 - 공통 준비

- [ ] `docs/sitl/2026-06-27/README.md`를 읽고 현재 SITL 흐름을 이해했다.
- [ ] `isaacsim_test/lerobot/rpo_arm_isaacsim.yaml`의 joint order를 팀 전체가 확인했다.
- [ ] `roboto_v2_right_arm_amazinghand_full.urdf`에서 5개 right arm joint와 AmazingHand joint가 존재함을 확인했다.
- [ ] `right_arm_*` SITL feature와 `rpo_arm_j*` hardware feature mapping table 초안을 만들었다.
- [ ] 데이터셋에 사용할 feature 이름을 임시/최종으로 구분했다.

**완료 기준:** 팀원이 같은 6D action vector를 보고 같은 joint/hand 의미로 설명할 수 있다.

### Phase 1 - Leader arm -> SITL 연동

- [ ] leader arm 포트와 type을 확인했다.
- [ ] leader arm calibration을 완료했다.
- [ ] leader arm raw output을 로그로 저장했다.
- [ ] leader output joint count가 5축 arm + hand scalar에 맞는지 확인했다.
- [ ] 맞지 않는 축은 mapping/drop/scale 정책을 문서화했다.
- [ ] leader arm 움직임이 `/leader/joint_commands` 또는 동등한 teleop 입력으로 들어오는지 확인했다.
- [ ] verifier/policy replay 검증은 `IsaacSimRpoArmRobot.send_action()` 경로로만 보냈다.
- [ ] live leader teleop/recording 검증은 `/leader/joint_commands` 입력과 `teleop_step()` 경로로만 보냈다.
- [ ] no-op action에서 SITL arm이 움직이지 않음을 확인했다.
- [ ] tiny action에서 예상 joint만 움직임을 확인했다.

**완료 기준:** leader arm을 움직이면 Isaac Sim의 Robotov2.0 right arm SITL follower가 동일한 6D contract로 반응하고, evidence JSON이 남는다.

### Phase 2 - URDF/Isaac Sim right arm motion 검증

- [ ] `roboto_v2_right_arm_amazinghand_full.urdf`를 Isaac Sim에서 load했다.
- [ ] right arm 5개 joint limit을 표로 추출했다.
- [ ] `amazinghand_grasp.pos`는 아직 8-servo raw control이 아니라 scalar intent로만 취급했다.
- [ ] home pose screenshot을 저장했다.
- [ ] reach-forward pose screenshot을 저장했다.
- [ ] elbow-fold pose screenshot을 저장했다.
- [ ] side-sweep pose screenshot을 저장했다.
- [ ] 각 pose별 target, observed, absolute error를 JSON으로 저장했다.
- [ ] joint limit 밖 command가 clamp되는지 확인했다.

**완료 기준:** 5개 arm joint sweep이 통과하고, screenshot/evidence가 `isaacsim_test/artifacts/` 아래에 남는다.

### Phase 3 - DM4340P motor parity 준비

- [ ] e-stop이 실제로 motor power를 끊는지 확인했다.
- [ ] power supply current limit을 설정했다.
- [ ] `can3`가 right arm bus인지 read-only로 확인했다.
- [ ] motor IDs `19-23`이 실제 right arm 5축인지 확인했다.
- [ ] zero offset 측정 절차를 문서화했다.
- [ ] max relative target을 보수적으로 설정했다. 시작값 예: `0.005-0.02 rad`.
- [ ] command clamp unit test를 hardware 없이 통과시켰다.
- [ ] 실제 motion 전, command path가 no-op이면 target jump가 없음을 확인했다.

**완료 기준:** 실제 motor를 움직이기 전 필요한 CAN, ID, sign 후보, limit 후보, e-stop, current limit이 문서화되어 있다.

### Phase 4 - DM4340P single-joint tiny motion

- [ ] hand/payload 없이 arm-only 상태로 시작했다.
- [ ] 한 번에 하나의 motor만 enable했다.
- [ ] `0.5-1.0 degree` 또는 그 이하의 tiny target만 보냈다.
- [ ] 실제 움직임 방향이 mapping과 일치하는지 기록했다.
- [ ] 반대 방향이면 sign을 바꾸고 다시 tiny motion으로 확인했다.
- [ ] 각 joint별 safe min/max 후보를 기록했다.
- [ ] 5개 joint tiny motion이 모두 예상 밖 움직임 없이 끝났다.
- [ ] no-op action이 실제 arm을 움직이지 않음을 다시 확인했다.

**완료 기준:** `docs/task_guides/roboparty_arm_bringup_log.md`에 right arm 5개 motor의 CAN, ID, sign, limit, notes가 기록되어 있다.

### Phase 5 - AmazingHand open-source test

- [ ] AmazingHand open-source Python example을 실행할 환경을 준비했다.
- [ ] hand serial port를 확인했다.
- [ ] servo voltage를 확인했다.
- [ ] 8개 servo ID를 확인했다.
- [ ] open target을 측정했다.
- [ ] closed target을 측정했다.
- [ ] safe min/max를 측정했다.
- [ ] command 값은 항상 safe range로 clamp했다.
- [ ] open/close 반복 중 overload, heat, cable strain이 없는지 확인했다.
- [ ] foam cube grasp test를 낮은 힘으로 수행했다.

**완료 기준:** `docs/task_guides/amazinghand_servo_map.md`에 servo ID, open target, closed target, safe min/max가 기록되어 있다.

### Phase 6 - Arm + hand integrated teleop dry run

- [ ] arm 5D + hand scalar 1D action을 하나의 LeRobot action으로 묶었다.
- [ ] leader arm 입력 중 hand scalar가 `[0.0, 1.0]` 안에 clamp된다.
- [ ] arm joint target은 soft limit과 max relative target으로 clamp된다.
- [ ] teleop 중 emergency stop operator가 있다.
- [ ] no-op integrated action에서 arm/hand가 움직이지 않는다.
- [ ] tiny integrated action에서 arm과 hand가 각각 예상대로 반응한다.
- [ ] camera가 cube, hand, tray를 모두 본다.
- [ ] reset pose가 반복 가능하다.

**완료 기준:** 실제 dataset recording 전 Gate A, Gate B, Gate C가 pass 또는 pass 직전 상태로 문서화되어 있다.

### Phase 7 - Debug dataset 10 episodes

- [ ] task는 `foam cube pick and place into tray`처럼 단순하게 고정했다.
- [ ] object start zone을 테이프로 표시했다.
- [ ] tray 위치를 고정했다.
- [ ] front camera가 cube, hand, tray를 본다.
- [ ] wrist camera가 있으면 grasp zone을 본다.
- [ ] 10 debug episodes를 기록했다.
- [ ] failed episode를 표시하거나 제거했다.
- [ ] action spike가 없는지 확인했다.
- [ ] missing frame이 없는지 확인했다.
- [ ] `observation.state` shape가 `(6,)`인지 확인했다.
- [ ] `action` shape가 `(6,)`인지 확인했다.

**완료 기준:** debug dataset은 학습에 바로 쓰지 않고 QA report를 남긴다.

### Phase 8 - Baseline imitation learning

- [ ] debug dataset QA에서 발견된 문제를 수정했다.
- [ ] baseline dataset 50-100 episodes를 기록했다.
- [ ] episode reset 상태가 일정하다.
- [ ] task label이 일관적이다.
- [ ] failed episode를 제거하거나 label했다.
- [ ] dataset visualizer로 video/state/action을 확인했다.
- [ ] 첫 ACT 또는 SmolVLA baseline training을 시작했다.
- [ ] feature-shape error 없이 training이 시작되는지 확인했다.
- [ ] real robot rollout 전 deterministic policy smoke를 먼저 실행했다.

**완료 기준:** baseline training이 feature shape error 없이 시작되고, real robot evaluation은 별도 Gate E로 관리한다.

---

## 7. Gate checklist

### Gate S - SITL ready

- [ ] leader arm calibration 완료.
- [ ] leader arm input log 확보.
- [ ] SITL follower가 no-op에서 움직이지 않음.
- [ ] SITL follower가 tiny command에 예상대로 움직임.
- [ ] 5 joint sweep 통과.
- [ ] screenshot/evidence JSON 확보.
- [ ] `amazinghand_grasp.pos` scalar clamp 확인.

### Gate A - 실제 arm motion 가능

- [ ] e-stop works.
- [ ] power limit 설정.
- [ ] correct CAN interface 확인.
- [ ] correct motor IDs 확인.
- [ ] joint signs 확인.
- [ ] joint soft limits 설정.
- [ ] hand/payload 없이 tiny motion 통과.
- [ ] cable collision 없음.

### Gate B - LeRobot wrapper safe

- [ ] robot type instantiate 가능.
- [ ] connect/disconnect 반복 가능.
- [ ] get/capture observation이 모든 expected key 반환.
- [ ] send action이 joint target clamp.
- [ ] send action이 relative movement clamp.
- [ ] hand scalar clamp.
- [ ] no-op action에서 target jump 없음.
- [ ] tiny joint command가 expected joint만 움직임.

### Gate C - Teleoperation usable

- [ ] operator가 5회 연속 cube-to-tray trial을 수행.
- [ ] arm이 table/tray를 치지 않음.
- [ ] hand timing 제어 가능.
- [ ] camera view가 task 전체를 봄.
- [ ] reset pose 반복 가능.
- [ ] failed demo 식별 가능.

### Gate D - Dataset quality

- [ ] episode count가 목표와 일치.
- [ ] missing camera frame 없음.
- [ ] missing action key 없음.
- [ ] unexpected action spike 없음.
- [ ] task label 일관.
- [ ] failed episode 제거 또는 labeling.
- [ ] visualizer로 usable video 확인.

---

## 8. 팀 운영 방식

- 한 task는 한 명이 소유한다.
- 한 PR 또는 한 commit은 한 task만 담는다.
- `setup_rpo_arm_scene.py`, `isaacsim_rpo_arm_robot.py`, hardware config는 동시에 여러 명이 수정하지 않는다.
- 매 standup에서 아래 3가지만 말한다.
  - 무엇을 바꿨는가.
  - 무엇으로 검증했는가.
  - 어떤 gate가 막혀 있는가.

---

## 9. 참고 문서

- Local SITL plan: `docs/sitl/2026-06-27/README.md`
- Local team tasks: `docs/sitl/2026-06-27/team_tiny_tasks_sitl.md`
- Local responsibility split: `docs/sitl/2026-06-27/task_separation_lerobot_isaac_sim_arm_sitl.md`
- Robotov2.0 + AmazingHand URDF: `isaacsim_test/outputs/simready/echo_full/sitl/roboto_v2_right_arm_amazinghand_full.urdf`
- SITL wrapper: `isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py`
- SITL config: `isaacsim_test/lerobot/rpo_arm_isaacsim.yaml`
- RoboParty bring-up guide: `docs/task_guides/ko/03_roboparty_arm_bringup.ko.md`
- AmazingHand guide: `docs/task_guides/ko/04_amazinghand_integration.ko.md`
- Dataset guide: `integration_guide/06_dataset_policy_workflow.md`
- Official LeRobot imitation learning: <https://huggingface.co/docs/lerobot/il_robots>
- Official LeRobot custom hardware: <https://huggingface.co/docs/lerobot/integrate_hardware>
- Official LeRobotDataset v3.0: <https://huggingface.co/docs/lerobot/lerobot-dataset-v3>
