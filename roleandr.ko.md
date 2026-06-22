# RoboParty 5-DOF Arm + AmazingHand + LeRobot 계획

## 짧은 결론

오픈소스 기반은 이미 충분히 강합니다. 팀이 로봇 stack을 처음부터 다시 만들 필요는
없습니다.

```text
RoboParty repos = hardware, ROS2 deployment, CAN, zeroing, XR teleop, robot models
AmazingHand repo = hand CAD, servo control, calibration examples
LeRobot = robot interface, teleoperation, dataset recording, training, inference
SO-100 / leader-arm setup = 추가적인 실전 데이터 수집 경로
```

남은 핵심 작업은 대부분 integration입니다.

```text
RoboParty 5-DOF arm 하나만 추출
AmazingHand 장착 및 제어
두 장치를 하나의 LeRobot-compatible robot으로 wrapping
XR 또는 leader-arm teleoperation으로 demonstration 수집
작은 imitation-learning baseline 학습 및 평가
```

가장 안전한 첫 목표는 그대로 아래입니다.

```text
RoboParty 5-DOF arm + AmazingHand
Action = 5 arm joint targets + 1 AmazingHand grasp scalar
Task   = soft cube를 집어서 fixed tray에 놓기
```

---

# 1. 이미 구현되어 있는 것

## RoboParty / `roboto_origin`

`roboto_origin`은 단순한 arm repo가 아닙니다. RoboParty hardware, deployment,
robot description, firmware, navigation, training, XR teleop sub-repo를 모아둔
상위 repo입니다. ([GitHub][1])

| 영역 | 이미 있음? | 사용 목적 |
| --- | ---: | --- |
| 전체 humanoid system overview | Yes | Reference architecture |
| Arm-related hardware source | Yes, through `rpo_hardware` | CAD와 mechanical reference |
| ROS2 motor deployment | Yes, through `roboparty_deploy` | CAN, zeroing, motor config |
| URDF / MJCF robot model | Yes, through `rpo_description` | Arm model extraction |
| XR teleop | Yes, through `roboparty_xr_teleop` | Teleoperation input path |
| LeRobot integration | No | 직접 wrapper 구현 필요 |

RoboParty는 low-level hardware reference로 사용합니다. Single-arm learning에 직접
필요한 부분이 아니라면 full humanoid control stack을 그대로 가져오지 않습니다.

---

# 2. Mechanical engineering 작업

## 재사용 가능한 것

`rpo_hardware`는 RoboParty / Roboto Origin hardware library를 제공합니다. Mechanical
files, PCB assets, manufacturing assets, versioned documentation이 포함되어 있고,
V2.0은 구조, cabling, stability, arm 측면의 개선이 들어간 recommended version입니다.
([GitHub][2])

재사용할 수 있는 것:

```text
RoboParty arm CAD
mechanical structure reference
mounting geometry reference
PCB / hardware layout reference
versioned hardware documentation
URDF / MJCF model assets from rpo_description
```

`rpo_description`도 robot model assets를 제공하므로 모델을 완전히 새로 만들 필요가
없습니다. ([GitHub][3])

## M.E가 해야 할 일

```text
[ ] 실제 hardware가 V1.0인지 V2.0인지 확인합니다.
[ ] full humanoid model에서 5-DOF arm geometry만 추출합니다.
[ ] RoboParty wrist-to-AmazingHand adapter를 설계합니다.
[ ] AmazingHand tool frame / grasp center를 정의합니다.
[ ] AmazingHand 장착 후 wrist payload, stiffness, backlash를 확인합니다.
[ ] wrist 주변 serial/power cable routing을 확인합니다.
[ ] safe table-mounted workspace를 정의합니다.
[ ] 반복 가능한 cube/tray fixture를 만듭니다.
[ ] hand와 camera 장착 후 collision assumption을 업데이트합니다.
```

중요: `rpo_hardware`는 V2.0이 V1.0과 호환되지 않는다고 설명합니다. CAD나 PCB 파일을
재사용하기 전에 hardware version을 반드시 확인해야 합니다. ([GitHub][2])

---

# 3. Electrical engineering 작업

## 재사용 가능한 것

`roboparty_deploy`는 CAN hardware mapping을 문서화합니다.

```text
can0 = left leg
can1 = right leg + waist
can2 = left hand / upper-limb group
can3 = right hand / upper-limb group
```

또한 CAN device에는 USB 3.0, IMU/gamepad에는 USB 2.0을 권장합니다. ([GitHub][4])

RoboParty zeroing config는 다음을 보여줍니다.

```yaml
motor_num: [6, 7, 5, 5]
motor_interface: ["can0", "can1", "can2", "can3"]
motor_id: [1, 2, ..., 23]
```

따라서 5-motor group은 left/right upper-limb group일 가능성이 큽니다. 초기 추정은
`can2`의 IDs 14-18, `can3`의 IDs 19-23입니다. 실제 로봇에서 반드시 검증해야 합니다.
([GitHub][5])

`roboparty_deploy`는 `/set_zeros` ROS service와 `python3 scripts/set_zero.py`를
포함한 zero-calibration tool을 이미 제공합니다. Arm-only bring-up에서는 이를
재사용하고, motor zeroing을 새로 만들지 않습니다. ([GitHub][4])

## E.E가 해야 할 일

```text
[ ] 실제 hardware에서 left/right arm CAN interface를 확인합니다.
[ ] 정확한 5개 motor ID를 확인합니다.
[ ] 각 joint의 motor direction/sign을 확인합니다.
[ ] 전체 5 joint 전에 motor 하나씩 테스트합니다.
[ ] zeroing config를 arm-only mode로 조정합니다.
[ ] RoboParty arm + AmazingHand에 안정적인 power distribution을 구성합니다.
[ ] fuse protection을 추가합니다.
[ ] physical emergency stop을 추가합니다.
[ ] CAN, serial, USB, power cable을 모두 labeling합니다.
[ ] camera device name을 stable하게 만듭니다.
```

---

# 4. AmazingHand 작업

AmazingHand는 초기 dexterous-hand prototype에 적합합니다. Repo에는 CAD, docs, Python
examples, Arduino examples, calibration examples, demos가 이미 있습니다. Hand는
8-DOF, 4-finger 구조이며 finger마다 Feetech SCS0009 servo 2개를 사용하고, actuator가
hand 내부에 있으며 무게는 약 400 g입니다. ([GitHub][6])

## 재사용 가능한 것

```text
AmazingHand CAD
servo ID convention
Python serial-bus examples
Arduino examples
calibration examples
predefined hand poses
servo read/write examples
```

## 팀이 해야 할 일

```text
[ ] RoboParty wrist에서 AmazingHand로 가는 mechanical adapter.
[ ] Robot 위 AmazingHand power wiring.
[ ] Control computer로 가는 serial-bus connection.
[ ] Servo ID verification.
[ ] Safe open/close scalar command.
[ ] Hand joint limit table.
[ ] LeRobot AmazingHand driver wrapper.
[ ] 1 grasp scalar에서 8 servo position으로 가는 mapping.
```

첫 policy에서는 8개 servo를 모두 노출하지 않습니다. 먼저 아래처럼 시작합니다.

```text
0.0 = open
1.0 = close / power grasp
```

5+1 baseline이 안정화된 뒤에 8-servo hand control로 넘어갑니다.

---

# 5. LeRobot integration 작업

LeRobot은 custom robot integration pattern을 이미 제공합니다. Custom robot type은
observation features, action features, `connect()`, `disconnect()`,
`get_observation()`, `send_action()`을 정의하면 됩니다. ([Hugging Face][7])

## 첫 robot type

통합된 target hardware는 하나의 LeRobot robot type으로 표현합니다.

```text
roboparty_5dof_arm_amazinghand_follower
```

첫 feature/action contract:

```text
rpo_arm_j1.pos
rpo_arm_j2.pos
rpo_arm_j3.pos
rpo_arm_j4.pos
rpo_arm_j5.pos
amazinghand_grasp.pos
```

단위:

```text
RoboParty arm joints = degrees
AmazingHand grasp    = scalar in [0.0, 1.0]
```

## 필요한 data flow

```text
get_observation()
  RoboParty arm joint positions 읽기
  motor signs와 calibration offsets 적용
  AmazingHand grasp scalar 읽기 또는 마지막 command 기억
  camera observations 추가
  flat LeRobot feature dictionary 반환

send_action(action)
  5 arm joint targets clamp
  max relative joint movement clamp
  motor signs를 raw motor target으로 역적용
  RoboParty arm command를 CAN으로 전송
  AmazingHand grasp scalar clamp
  AmazingHand serial command 전송
  실제 보낸 clipped action 반환
```

## C.S가 해야 할 일

```text
[ ] Custom LeRobot robot package를 만듭니다.
[ ] RoboParty CAN control을 LeRobot Robot class 안에서 재사용합니다.
[ ] AmazingHand Python control을 같은 Robot class 안에서 재사용합니다.
[ ] 5 arm joints + 1 grasp scalar action mapper를 구현합니다.
[ ] observation builder를 구현합니다.
[ ] 모든 motor command 전에 safety clamp를 추가합니다.
[ ] calibration loading을 추가합니다.
[ ] cameras를 추가합니다.
[ ] 작은 LeRobotDataset을 기록합니다.
[ ] ACT baseline을 학습합니다.
[ ] real robot에서 평가합니다.
```

---

# 6. Teleoperation과 data collection 경로

사용 가능한 데이터 수집 경로는 두 가지입니다.

## Path A: RoboParty XR teleop

`roboparty_xr_teleop`은 PICO VR 기반 RoboParty / Roboto teleoperation을 이미
대상으로 합니다. ROS2 Python, Python 3.10, PICO VR, Pinocchio, CasADi를 사용합니다.
([GitHub][8])

XR teleop은 별도의 motor-control stack이 아니라 input device로 사용합니다.

```text
XR controller pose
-> IK / pose-to-joint conversion
-> LeRobot action dictionary
-> robot.send_action()
-> LeRobotDataset recording
```

Learning path에서는 아래 구조를 피합니다.

```text
XR controller pose
-> direct motor command
```

Direct motor teleop은 나중에 policy가 사용할 action format을 우회하므로 dataset
품질과 재사용성을 떨어뜨립니다.

## Path B: Leader arm + LeRobot SO-100 follower

Leader arm과 LeRobot SO-100 follower로 데이터를 수집하는 추가 계획은 유용합니다.
RoboParty + AmazingHand integration이 끝나기 전에 더 빠르고 낮은 위험으로 workflow를
연습할 수 있습니다.

사용 목적:

```text
operator training
camera placement testing
task design
episode naming and dataset QA
LeRobot recording workflow validation
ACT training sanity checks
baseline task difficulty measurement
```

SO-100 follower는 LeRobot native이므로 custom RoboParty wrapper가 완성되기 전에
collect-train-evaluate loop를 검증할 수 있습니다.

하지만 SO-100 data는 proxy data입니다. RoboParty final policy용 data로 자동 취급하면
안 됩니다. 섞거나 transfer하기 전에는 아래를 확인합니다.

```text
joint count and joint order
joint limits
joint signs
workspace scale
camera viewpoints
gripper/hand action meaning
task fixture geometry
action units
control frequency
```

실용적인 권장안:

```text
먼저 SO-100 leader/follower로 task와 dataset workflow를 검증합니다.
RoboParty에서 실행할 final policy는 RoboParty + AmazingHand data로 학습합니다.
SO-100 data는 명시적인 action/observation mapping을 정의한 뒤에만 transfer합니다.
```

---

# 7. 첫 dataset design

간단한 task부터 시작합니다.

```text
Task: pick foam cube and place into tray
Arm: fixed base or table-mounted
Hand: AmazingHand scalar open/close
Object: one soft/light cube
Start area: marked 10 cm x 10 cm square
Tray: fixed position
Episodes: 10 debug, then 50-100 baseline
```

Minimum RoboParty + AmazingHand dataset features:

```text
rpo_arm_j1.pos
rpo_arm_j2.pos
rpo_arm_j3.pos
rpo_arm_j4.pos
rpo_arm_j5.pos
amazinghand_grasp.pos
observation.images.front
observation.images.wrist   optional but recommended
timestamp
episode_index
frame_index
task
```

10 debug episodes 후 확인:

```text
[ ] State shape이 stable합니다.
[ ] Action shape이 stable합니다.
[ ] Joint order가 stable합니다.
[ ] Joint sign이 올바릅니다.
[ ] amazinghand_grasp가 hand open/close에 따라 변합니다.
[ ] Camera가 cube, hand, tray를 봅니다.
[ ] Arm target에 갑작스러운 jump가 없습니다.
[ ] Hand scalar가 [0.0, 1.0]으로 clamp됩니다.
[ ] 실패 episode는 표시하거나 제거했습니다.
```

---

# 8. 새로 만들지 말아야 할 것

원래 todo list에서 줄이거나 제거해야 할 항목:

```text
Full arm CAD를 처음부터 설계하지 않습니다.
rpo_hardware를 사용합니다.

Motor zeroing을 처음부터 작성하지 않습니다.
roboparty_deploy set_zero.py를 사용합니다.

Full URDF/MJCF를 처음부터 만들지 않습니다.
rpo_description을 사용하고 arm만 추출합니다.

XR teleop을 처음부터 만들지 않습니다.
roboparty_xr_teleop을 reference/input layer로 사용합니다.

LeRobot integration pattern을 새로 만들지 않습니다.
LeRobot custom Robot interface를 사용합니다.

AmazingHand servo example을 처음부터 작성하지 않습니다.
AmazingHand Python/Arduino examples를 사용합니다.

RoboParty integration이 끝날 때까지 LeRobot workflow 학습을 기다리지 않습니다.
SO-100 leader/follower setup을 proxy data-collection path로 사용합니다.
```

---

# 9. 권장 기술 순서

```text
1. RoboParty, AmazingHand, LeRobot의 exact commit을 clone/lock합니다.
2. RoboParty hardware version이 V1.0인지 V2.0인지 확인합니다.
3. LeRobot SO-100 leader/follower setup을 bring-up합니다.
4. Cube-to-tray task로 SO-100 debug episodes 10개를 기록합니다.
5. SO-100 dataset으로 tiny ACT sanity-check policy를 학습합니다.
6. roboparty_deploy에서 RoboParty arm-only motor config를 추출합니다.
7. 기존 RoboParty script로 RoboParty arm motor 하나를 테스트합니다.
8. Conservative limits로 5개 arm motor를 모두 테스트합니다.
9. Official Python example로 AmazingHand를 테스트합니다.
10. AmazingHand wrist adapter를 설계하고 장착합니다.
11. RoboParty CAN + AmazingHand serial control을 감싸는 LeRobot wrapper를 만듭니다.
12. RoboParty + AmazingHand debug episodes 10개를 기록합니다.
13. RoboParty + AmazingHand data로 첫 ACT baseline을 학습합니다.
14. SO-100과 RoboParty failure를 비교해 fixture와 camera를 개선합니다.
```

---

# 10. 최종 프로젝트 framing

이 프로젝트는 주로 hardware invention project가 아닙니다. Integration과 robot-learning
project입니다.

Clean architecture:

```text
teleop source
  XR controller OR leader arm

LeRobot action interface
  5 RoboParty arm joints + 1 AmazingHand grasp scalar

custom robot wrapper
  RoboParty CAN adapter + AmazingHand serial adapter

LeRobotDataset
  standardized demos

policy
  ACT baseline first
```

가장 강한 다음 단계는 SO-100 leader/follower data workflow와 RoboParty hardware
bring-up을 병렬로 진행하는 것입니다. 이렇게 하면 C.S는 LeRobot recording/training을
즉시 검증하고, M.E/E.E는 RoboParty arm과 AmazingHand integration을 마무리할 수
있습니다.

[1]: https://github.com/Roboparty/roboto_origin "GitHub - Roboparty/roboto_origin"
[2]: https://github.com/Roboparty/rpo_hardware "GitHub - Roboparty/rpo_hardware"
[3]: https://github.com/Roboparty/rpo_description "GitHub - Roboparty/rpo_description"
[4]: https://github.com/Roboparty/roboparty_deploy "GitHub - Roboparty/roboparty_deploy"
[5]: https://raw.githubusercontent.com/Roboparty/roboparty_deploy/main/scripts/config/set_zero.yaml "RoboParty set_zero.yaml"
[6]: https://github.com/pollen-robotics/AmazingHand "GitHub - pollen-robotics/AmazingHand"
[7]: https://huggingface.co/docs/lerobot/integrate_hardware "LeRobot Bring Your Own Hardware"
[8]: https://github.com/Roboparty/roboparty_xr_teleop "GitHub - Roboparty/roboparty_xr_teleop"
