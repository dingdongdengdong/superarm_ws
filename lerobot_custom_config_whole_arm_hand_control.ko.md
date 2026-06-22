# LeRobot Custom Config로 RoboParty Arm + AmazingHand 전체 시스템을 제어할 수 있는가?

짧은 답: **가능합니다.** 하지만 config file 하나만으로 되는 것은 아닙니다.

LeRobot custom config는 통합 robot의 hardware facts를 설명할 수 있고, 그 config와
짝을 이루는 custom `Robot` class가 실제 hardware를 제어합니다. 이 프로젝트의 올바른
mental model은 아래입니다.

```text
LeRobot CLI / policy / dataset tools
        ↓
Roboparty5DofArmAmazingHandConfig
        ↓
Roboparty5DofArmAmazingHandFollower Robot class
        ↓
 ┌──────────────────────┬──────────────────────────┐
 │ RoboParty 5-DOF arm  │ AmazingHand dexterous hand │
 │ CAN / DM motors      │ Serial / SCS0009 servos    │
 └──────────────────────┴──────────────────────────┘
```

Config는 motor를 마법처럼 움직이지 않습니다. Config는 hardware 정보를 저장합니다.
`Robot` class와 hardware adapter가 그 정보를 사용해 sensor를 읽고 command를 보냅니다.

---

## 1. 최종 판단

| 질문 | 답 |
|---|---|
| 하나의 LeRobot robot type이 arm-hand 통합 hardware를 표현할 수 있나? | **Yes.** `roboparty_5dof_arm_amazinghand_follower` 하나를 사용합니다. |
| 하나의 config에 arm과 hand를 모두 넣을 수 있나? | **Yes.** CAN, motor IDs, signs, hand serial port, servo IDs, limits, cameras를 한 config에 넣습니다. |
| 하나의 policy가 arm과 hand command를 모두 출력할 수 있나? | **Yes.** 먼저 6D action: 5 arm joints + 1 hand grasp scalar. 나중에 13D: 5 arm joints + 8 hand servos. |
| Arm은 CAN이고 hand는 serial이어도 LeRobot이 신경 쓰나? | **No.** `Robot` class가 `get_observation()`과 `send_action()` 뒤에 숨기면 LeRobot은 features/actions만 봅니다. |
| Guide에 low-level motor code가 모두 포함되어 있나? | **No.** Architecture와 skeleton입니다. 실제 CAN read/write와 AmazingHand serial command는 구현해야 합니다. |
| OpenArm이 target robot인가? | **No.** OpenArm은 coding reference일 뿐입니다. Target은 RoboParty 5-DOF arm + AmazingHand입니다. |

정확한 결론:

```text
Yes: arm adapter와 AmazingHand adapter를 모두 소유하는 custom Robot class를 구현하면 LeRobot이 전체 통합 시스템을 제어할 수 있습니다.

No: custom config만으로는 충분하지 않습니다. CAN motor adapter, serial hand adapter, calibration, limits, safety layer가 실제로 동작해야 합니다.
```

---

## 2. 첫 control contract 권장안

처음부터 AmazingHand 8개 servo를 모두 노출하지 않습니다. 먼저 단순하게 시작합니다.

```text
5 RoboParty arm joint targets + 1 AmazingHand grasp scalar
```

Action order:

```text
[
  rpo_arm_j1,
  rpo_arm_j2,
  rpo_arm_j3,
  rpo_arm_j4,
  rpo_arm_j5,
  amazinghand_grasp,
]
```

의미:

```text
rpo_arm_j1..j5       = calibrated arm joint targets
amazinghand_grasp    = 0.0 open, 1.0 closed/grasp
```

Dataset/policy shape:

```text
observation.state shape = (6,)
action shape            = (6,)
```

이 baseline이 동작한 뒤에만 확장합니다.

```text
Stage 1: 5 arm joints + 1 grasp scalar        = 6D action
Stage 2: 5 arm joints + hand pattern controls = maybe 8D action
Stage 3: 5 arm joints + 8 raw hand servos     = 13D action
```

Safe hand calibration과 finger-level dexterity가 필요한 명확한 이유가 없다면 13D raw
servo control로 시작하지 않습니다.

---

## 3. 이 repo의 중요한 LeRobot version detail

과거 skeleton은 이런 import를 사용할 수 있습니다.

```python
from lerobot.common.robots.configs import RobotConfig
from lerobot.common.robots.robot import Robot
```

하지만 현재 checkout의 실제 path는 아래입니다.

```python
from lerobot.robots.config import RobotConfig
from lerobot.robots.robot import Robot
```

현재 base class에서 robot subclass가 가져야 하는 형태:

```python
class Roboparty5DofArmAmazingHandFollower(Robot):
    config_class = Roboparty5DofArmAmazingHandConfig
    name = "roboparty_5dof_arm_amazinghand_follower"

    @property
    def observation_features(self) -> dict: ...

    @property
    def action_features(self) -> dict: ...

    @property
    def is_connected(self) -> bool: ...

    def connect(self, calibrate: bool = True) -> None: ...
    def disconnect(self) -> None: ...

    @property
    def is_calibrated(self) -> bool: ...

    def calibrate(self) -> None: ...
    def configure(self) -> None: ...

    def get_observation(self) -> dict: ...
    def send_action(self, action: dict) -> dict: ...
```

Conceptual design은 맞지만 import path와 interface shape은 현재 LeRobot checkout에
맞춰야 합니다.

---

## 4. LeRobot이 통합 robot을 보는 방식

현재 LeRobot 구현에서 hardware `Robot`은 보통 `observation.state` vector를 직접
반환하지 않고, flat hardware feature dictionary를 반환합니다.

예:

```python
observation_features = {
    "rpo_arm_j1.pos": float,
    "rpo_arm_j2.pos": float,
    "rpo_arm_j3.pos": float,
    "rpo_arm_j4.pos": float,
    "rpo_arm_j5.pos": float,
    "amazinghand_grasp.pos": float,
    "front": (480, 640, 3),
    "wrist": (480, 640, 3),
}

action_features = {
    "rpo_arm_j1.pos": float,
    "rpo_arm_j2.pos": float,
    "rpo_arm_j3.pos": float,
    "rpo_arm_j4.pos": float,
    "rpo_arm_j5.pos": float,
    "amazinghand_grasp.pos": float,
}
```

그 다음 LeRobot dataset utility가 flat motor features를 standard dataset vector로
변환합니다.

```text
raw robot observation dict
        ↓
LeRobot build_dataset_frame / feature utilities
        ↓
observation.state = [rpo_arm_j1.pos, ..., amazinghand_grasp.pos]
action            = [rpo_arm_j1.pos, ..., amazinghand_grasp.pos]
```

따라서 두 말이 모두 맞습니다.

```text
Robot class 내부: "rpo_arm_j1.pos" 같은 flat dict key 사용.
Dataset/policy 내부: LeRobot이 observation.state/action vector로 packing.
```

더 단순한 경로는 existing robot pattern처럼 flat joint key를 노출하고, LeRobot이
dataset vector를 만들게 하는 것입니다.

---

## 5. Custom config에 들어갈 내용

Config는 **arm과 hand 두 subsystem**을 모두 설명해야 합니다.

### 5.1 Arm section

Right-arm starting point:

```yaml
robot:
  type: roboparty_5dof_arm_amazinghand_follower
  id: rpo_right_arm_ah_v1
  side: right

arm:
  can_interface: can3
  motor_type: DM
  master_id_offset: 16
  joint_names:
    - rpo_arm_j1
    - rpo_arm_j2
    - rpo_arm_j3
    - rpo_arm_j4
    - rpo_arm_j5
  motor_ids:
    rpo_arm_j1: 19
    rpo_arm_j2: 20
    rpo_arm_j3: 21
    rpo_arm_j4: 22
    rpo_arm_j5: 23
  motor_signs:
    rpo_arm_j1: -1
    rpo_arm_j2: 1
    rpo_arm_j3: 1
    rpo_arm_j4: -1
    rpo_arm_j5: 1
  max_relative_target_rad: 0.02
```

Left-arm starting point:

```yaml
arm:
  can_interface: can2
  motor_ids:
    rpo_arm_j1: 14
    rpo_arm_j2: 15
    rpo_arm_j3: 16
    rpo_arm_j4: 17
    rpo_arm_j5: 18
  motor_signs:
    rpo_arm_j1: 1
    rpo_arm_j2: 1
    rpo_arm_j3: 1
    rpo_arm_j4: 1
    rpo_arm_j5: 1
```

아래 값은 반드시 측정해서 채웁니다.

```yaml
zero_offsets_rad:
  rpo_arm_j1: measured_value
  rpo_arm_j2: measured_value
  rpo_arm_j3: measured_value
  rpo_arm_j4: measured_value
  rpo_arm_j5: measured_value

joint_limits_rad:
  rpo_arm_j1: [measured_min, measured_max]
  rpo_arm_j2: [measured_min, measured_max]
  rpo_arm_j3: [measured_min, measured_max]
  rpo_arm_j4: [measured_min, measured_max]
  rpo_arm_j5: [measured_min, measured_max]
```

실제 movement에는 placeholder limit을 믿지 않습니다. 처음에는 실제 mechanical range의
약 50-70% 수준으로 보수적인 limit을 사용합니다.

### 5.2 AmazingHand section

```yaml
amazinghand:
  enabled: true
  serial_port: /dev/ttyUSB_AH_RIGHT
  baudrate: 1000000
  timeout: 0.5
  action_mode: scalar_grasp
  servo_ids: [1, 2, 3, 4, 5, 6, 7, 8]
  middle_pos_deg: [3, 0, -5, -8, -2, 5, -12, 0]
  default_speed: 3
  safe_limits_deg:
    servo_1: [-45, 95]
    servo_2: [-95, 45]
    servo_3: [-45, 95]
    servo_4: [-95, 45]
    servo_5: [-45, 95]
    servo_6: [-95, 45]
    servo_7: [-90, 95]
    servo_8: [-95, 45]
```

`middle_pos_deg`와 `safe_limits_deg`는 실제 assembled hand에서 측정하기 전까지
placeholder입니다.

---

## 6. Robot class가 해야 할 일

Custom `Robot` class에서 whole-system control이 일어납니다.

### 6.1 Constructor

두 adapter를 모두 만듭니다.

```python
class Roboparty5DofArmAmazingHandFollower(Robot):
    config_class = Roboparty5DofArmAmazingHandConfig
    name = "roboparty_5dof_arm_amazinghand_follower"

    def __init__(self, config: Roboparty5DofArmAmazingHandConfig):
        super().__init__(config)
        self.config = config
        self.arm = Rpo5DofArmAdapter(config)
        self.hand = AmazingHandAdapter(config)
        self.cameras = make_cameras_from_configs(config.cameras)
```

### 6.2 `connect()`

모든 hardware를 연결합니다.

```text
connect()
  ├── arm.connect()   -> open CAN, verify IDs, enable/configure DM motors
  ├── hand.connect()  -> open serial, verify servo IDs, enable torque
  └── cameras.connect()
```

### 6.3 `get_observation()`

두 subsystem을 읽고 하나의 observation dict를 반환합니다.

```python
def get_observation(self) -> dict:
    arm_joint_rad = self.arm.read_joint_positions_rad()
    hand_grasp = self.hand.read_or_estimate_grasp_scalar()

    obs = {
        "rpo_arm_j1.pos": arm_joint_rad["rpo_arm_j1"],
        "rpo_arm_j2.pos": arm_joint_rad["rpo_arm_j2"],
        "rpo_arm_j3.pos": arm_joint_rad["rpo_arm_j3"],
        "rpo_arm_j4.pos": arm_joint_rad["rpo_arm_j4"],
        "rpo_arm_j5.pos": arm_joint_rad["rpo_arm_j5"],
        "amazinghand_grasp.pos": hand_grasp,
    }

    obs["front"] = self.cameras["front"].read_latest()
    return obs
```

### 6.4 `send_action()`

하나의 LeRobot action dict를 arm command와 hand command로 나눕니다.

```python
def send_action(self, action: dict) -> dict:
    arm_targets = {
        "rpo_arm_j1": action["rpo_arm_j1.pos"],
        "rpo_arm_j2": action["rpo_arm_j2.pos"],
        "rpo_arm_j3": action["rpo_arm_j3.pos"],
        "rpo_arm_j4": action["rpo_arm_j4.pos"],
        "rpo_arm_j5": action["rpo_arm_j5.pos"],
    }
    grasp = float(action["amazinghand_grasp.pos"])

    safe_arm_targets = self.arm.clamp_joint_targets(arm_targets)
    self.arm.write_joint_targets_rad(safe_arm_targets)

    safe_grasp = min(1.0, max(0.0, grasp))
    self.hand.command_grasp_scalar(safe_grasp)

    return {
        "rpo_arm_j1.pos": safe_arm_targets["rpo_arm_j1"],
        "rpo_arm_j2.pos": safe_arm_targets["rpo_arm_j2"],
        "rpo_arm_j3.pos": safe_arm_targets["rpo_arm_j3"],
        "rpo_arm_j4.pos": safe_arm_targets["rpo_arm_j4"],
        "rpo_arm_j5.pos": safe_arm_targets["rpo_arm_j5"],
        "amazinghand_grasp.pos": safe_grasp,
    }
```

핵심은 이것입니다. LeRobot은 action 하나를 보내고, robot class가 어떤 부분을 CAN으로
보낼지, 어떤 부분을 serial로 보낼지 결정합니다.

---

## 7. Arm conversion formulas

Arm adapter에는 아래 변환식을 구현합니다.

Raw motor position에서 LeRobot joint position으로:

```python
joint_rad = sign * (raw_motor_rad - zero_offset_rad)
```

LeRobot joint target에서 raw motor target으로:

```python
raw_target_rad = sign * joint_target_rad + zero_offset_rad
```

전송 전에는 항상 clamp합니다.

```python
safe_target = clamp(joint_target_rad, min_rad, max_rad)
safe_target = clamp_delta(safe_target, current_joint_rad, max_relative_target_rad)
```

첫 powered movement에서는 아래를 사용합니다.

```text
max_relative_target_rad = 0.005 to 0.02 rad
speed/gain              = low
one joint at a time
hand empty
operator outside workspace
emergency stop ready
```

---

## 8. AmazingHand scalar grasp mapping

Stage 1은 scalar 하나를 8개 servo target으로 mapping합니다.

```python
def grasp_scalar_to_servo_targets(grasp: float, middle_pos_deg: list[float]) -> dict[int, float]:
    g = float(np.clip(grasp, 0.0, 1.0))

    open_deg = [-35, 35, -35, 35, -35, 35, -35, 35]
    close_deg = [90, -90, 90, -90, 90, -90, 90, -90]

    targets = {}
    for i, servo_id in enumerate(range(1, 9)):
        deg = (1.0 - g) * open_deg[i] + g * close_deg[i]
        deg += middle_pos_deg[i]
        targets[servo_id] = np.deg2rad(deg)

    return targets
```

Real hardware에서 사용하기 전:

```text
1. 모든 servo ID가 응답하는지 확인합니다.
2. Servo별 direction을 확인합니다.
3. Open/close range를 먼저 줄여서 테스트합니다.
4. Safe limits로 clamp합니다.
5. Speed 1 또는 매우 낮은 speed부터 테스트합니다.
6. Finger self-collision이나 cable pull이 없는지 확인합니다.
```

---

## 9. 실제 blocker

Integration architecture는 타당합니다. Risk는 LeRobot이 아니라 hardware bring-up입니다.

### Blocker A: CAN motor communication

Arm adapter skeleton에는 실제 구현이 필요합니다.

```python
read_raw_positions()
write_raw_positions()
disable_torque()
configure_motors()
```

가능한 경로:

```text
Path B: LeRobot Damiao/OpenArm support 재사용
  RoboParty DM protocol이 LeRobot DamiaoMotorsBus와 호환되면 사용합니다.

Path C: RoboParty deploy motor code wrapping
  RoboParty motor stack이 더 안정적이거나 protocol detail이 다르면 사용합니다.
```

`can3`에서 motor ID 19를 읽을 수 없다면 custom config는 아무것도 제어할 수 없습니다.

### Blocker B: motor ID와 physical joint mapping

Guide는 초기 code를 위해 `rpo_arm_j1..rpo_arm_j5`라는 중립 이름을 사용합니다. 하지만
데이터 수집 전에는 반드시 알아야 합니다.

```text
motor 19/14 moves which physical joint?
motor 20/15 moves which physical joint?
...
```

Joint order가 틀리면 policy는 잘못된 action semantics를 학습합니다.

### Blocker C: signs and zero offsets

Sign이 틀리면 mirrored/unstable control이 됩니다. Zero offset이 틀리면 enable 시 jump가
납니다.

Joint 하나씩 검증합니다.

```text
1. Safe pose.
2. Command +0.005 or +0.01 rad.
3. Physical positive direction 확인.
4. LeRobot observation도 같은 convention으로 증가하는지 확인.
5. 다음 joint 전에 sign 수정.
```

### Blocker D: 5-DOF arm task limits

5-DOF arm은 유용한 task를 학습할 수 있지만 full 6/7-DOF manipulator처럼 움직일 수는
없습니다.

좋은 첫 task:

```text
front-facing pick
large/light foam cube
tray placement
fixed table height
fixed camera
simple open/close grasp
```

나쁜 첫 task:

```text
side insertion
tool use
precise wrist roll
in-hand reorientation
cluttered bin picking
small object dexterity
```

---

## 10. Custom robot type 등록 방법

Config class를 등록해야 합니다.

```python
from dataclasses import dataclass, field
from lerobot.robots.config import RobotConfig

@RobotConfig.register_subclass("roboparty_5dof_arm_amazinghand_follower")
@dataclass
class Roboparty5DofArmAmazingHandConfig(RobotConfig):
    id: str | None = "rpo_right_arm_ah_v1"
    side: str = "right"
    arm_can_interface: str = "can3"
    hand_serial_port: str = "/dev/ttyUSB_AH_RIGHT"
    hand_action_mode: str = "scalar_grasp"
    # add the full fields here
```

그 다음 LeRobot이 robot을 instantiate할 수 있어야 합니다.

### Option 1: In-tree LeRobot robot

아래 package를 추가합니다.

```text
lerobot/src/lerobot/robots/roboparty_5dof_arm_amazinghand/
```

그리고 아래 파일을 업데이트합니다.

```text
lerobot/src/lerobot/robots/utils.py
```

예:

```python
elif config.type == "roboparty_5dof_arm_amazinghand_follower":
    from .roboparty_5dof_arm_amazinghand import Roboparty5DofArmAmazingHandFollower
    return Roboparty5DofArmAmazingHandFollower(config)
```

Draccus가 registered config를 볼 수 있도록 built-in robot과 같은 방식으로 script에서
module import가 필요할 수 있습니다.

### Option 2: Third-party plugin

Distribution name이 아래 prefix로 시작하는 installable package를 만듭니다.

```text
lerobot_robot_
```

LeRobot의 `register_third_party_plugins()`가 이 prefix의 package를 scan/import하면
config가 register됩니다. 장기적으로는 더 깨끗하지만, 현재 integration project에서는
Option 1이 더 빠를 가능성이 큽니다.

---

## 11. 최소 검증 sequence

Dataset collection으로 바로 가지 않습니다. Gate를 사용합니다.

### Gate A: arm only

```text
[ ] can3/can2 exists on Linux
[ ] motor IDs respond
[ ] all 5 raw positions can be read
[ ] torque disable works
[ ] one tiny single-joint command works
[ ] sign per joint confirmed
[ ] zero offsets measured
[ ] conservative joint limits configured
```

### Gate B: hand only

```text
[ ] serial port exists
[ ] all 8 servo IDs respond
[ ] torque enable/disable works
[ ] middle positions measured
[ ] scalar 0.0 opens safely
[ ] scalar 1.0 closes safely, with reduced range first
```

### Gate C: combined LeRobot robot

```text
[ ] robot.connect() connects arm + hand + cameras
[ ] robot.observation_features lists 6 numeric features + cameras
[ ] robot.action_features lists 6 numeric action features
[ ] robot.get_observation() returns stable values
[ ] robot.send_action() moves the correct joint/hand with tiny command
[ ] robot.disconnect() disables torque / safe mode
```

### Gate D: dataset

```text
[ ] 10 debug episodes record successfully
[ ] observation.state shape is (6,)
[ ] action shape is (6,)
[ ] state/action order is stable
[ ] no NaNs
[ ] no sudden action jumps
[ ] front camera sees object, hand, and tray
[ ] hand scalar changes when hand opens/closes
```

### Gate E: policy

```text
[ ] train ACT on 50-100 simple episodes
[ ] dry-run policy with motors disabled or commands logged
[ ] inspect output range
[ ] evaluate with reduced max delta
[ ] emergency stop ready
```

---

## 12. 권장 첫 implementation milestone

첫 milestone은 "full dexterous hand policy"가 아닙니다.

목표는 아래입니다.

```text
LeRobot robot type: roboparty_5dof_arm_amazinghand_follower
6D policy/action contract:

  5 calibrated RoboParty arm joint positions
  +
  1 AmazingHand scalar grasp value
```

Success criteria:

```text
1. `lerobot-record`가 robot type으로 robot을 instantiate할 수 있습니다.
2. `robot.get_observation()`이 6 stable numeric features plus images를 반환합니다.
3. `robot.send_action()`이 arm과 hand를 안전하게 움직입니다.
4. 10-episode debug dataset의 observation.state/action shape이 (6,)입니다.
5. teleop, recording, training, eval에서 같은 6D action representation을 사용합니다.
```

이 정도면 첫 ACT baseline에 충분합니다.

---

## 13. Bottom line

LeRobot custom config는 **하나의 custom robot**으로 구현할 때 전체 arm-hand hardware를
제어할 수 있습니다. 서로 무관한 장치 두 개로 다루는 것이 아닙니다.

올바른 boundary:

```text
LeRobot standard world
  - robot.type
  - observation_features
  - action_features
  - get_observation()
  - send_action()
  - dataset observation.state/action

RoboParty/AmazingHand hardware world
  - CAN interface
  - DM motor IDs/signs/zero offsets
  - AmazingHand serial port/servo IDs
  - scalar-to-servo mapping
  - hardware safety limits
```

Hardware-specific detail은 custom `Robot` class와 adapter 뒤에 숨깁니다. 그러면
LeRobot은 전체 arm + hand를 하나의 일반 robot처럼 record, train, evaluate할 수
있습니다.

가장 위험한 작업은 LeRobot config가 아닙니다.

```text
1. RoboParty DM motor와 CAN communication.
2. 올바른 motor ID / joint order mapping.
3. Zero offsets and signs.
4. Safe AmazingHand servo calibration.
```

이 gate를 먼저 통과하면 LeRobot custom config는 전체 시스템 제어에 맞는 architecture입니다.
