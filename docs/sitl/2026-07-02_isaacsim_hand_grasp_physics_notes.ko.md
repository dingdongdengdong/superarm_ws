# Isaac Sim 손 모델링과 Grasp 물리 노트

작성일: 2026-07-02  
대상 패키지: `robot_arm_hand_package.zip`  
목표: 기존 AmazingHand 비주얼 자산을 최대한 살리면서, Isaac Sim에서 실제 작은 물체를 집을 수 있는 손 모델을 만들기 위한 학습/설계 노트

## 결론

이번 손 모델은 원본 MJCF를 Isaac Sim MJCF importer로 그대로 가져오는 방식보다, Isaac 친화적인 새 손 articulation을 만들고 기존 STL을 visual로 재사용하는 방식이 맞다.

이유는 다음과 같다.

- 원본 hand MJCF에는 mesh와 actuator 정보가 있다.
- 하지만 MJCF 안에 `equality/connect` 기반 closed-loop 구조가 많다.
- Isaac Sim 5.1의 MJCF importer는 해당 손 모델을 `MJCFCreateAsset` 단계에서 실패시켰다.
- 실제 grasp 물리가 목적이면 visual mesh와 collision geometry를 분리해야 한다.
- 작은 쓰레기 grasp는 손끝 접촉, 마찰, drive force, solver 안정성이 핵심이다.

## 현재 손 모델에서 확인된 사실

최신 실행 결과:

- 리포트: `isaacsim_test/outputs/robot_arm_hand_from_zip_fixed_20260702T065932Z/robot_arm_hand_connected_report.json`
- 런타임 로그: `isaacsim_test/artifacts/runtime_logs/robot_arm_hand_from_zip_20260702T065932Z.log`
- 원본 MJCF: `isaacsim_test/inputs/robot_arm_hand_package/robot_arm_hand_package/hand_mjcf/robot.xml`

원본 MJCF 분석 결과:

- root body: `r_wrist_interface`
- mesh count: 23
- missing meshes: 없음
- position actuator count: 8
- actuator names:
  - `finger1_motor1`
  - `finger1_motor2`
  - `finger2_motor1`
  - `finger2_motor2`
  - `finger3_motor1`
  - `finger3_motor2`
  - `finger4_motor1`
  - `finger4_motor2`
- equality/connect count: 20

즉 손 데이터가 없는 것이 아니다. 손은 mesh, joint, actuator, site/equality constraint가 포함된 복잡한 MJCF 모델이다.

## 실패한 지점

파이프라인은 먼저 MJCF를 sanitize한다.

- top-level default 3개를 1개로 병합
- mesh name 23개 추가
- equality/connect name 20개 추가

그 후 Isaac Sim에서 다음 명령을 실행한다.

- `MJCFCreateImportConfig`
- `MJCFCreateAsset`

실패 지점은 `MJCFCreateAsset`이다. 최신 로그의 핵심 오류:

```text
Failed to execute a command: MJCFCreateAsset
RuntimeError: basic_string::_M_construct null not valid
```

직전 warning:

```text
Could not determine geometry type, defaulting to Sphere
```

현재 코드는 이 실패를 감지하면 visible proxy hand USD를 만든다. 이 proxy는 팔 끝에 붙고 rigid body/collision은 갖지만, 실제 finger joint articulation은 아니다. 따라서 팔은 움직이고 손은 물체를 집을 수 있는 손가락 구조가 아니다.

## Isaac Sim에서 중요한 기본 개념

### 1. Articulation은 tree 구조가 가장 안정적이다

Isaac/PhysX articulation은 rigid body link들이 joint로 연결된 tree 형태일 때 가장 안정적이다.

공식 Isaac Sim 5.1 physics 문서는 articulation joint를 만들 때 Body 0이 Body 1의 parent가 되도록 구성해야 한다고 설명한다. 그래야 PhysX SDK와 USD 사이에서 joint drive target, force 등 joint 관련 값이 일관되게 대응된다.

프로젝트 적용:

- 손바닥 또는 wrist interface를 root link로 둔다.
- 각 손가락은 palm에서 뻗는 독립 tree branch로 둔다.
- closed-loop link를 직접 만들지 않는다.
- `body0 = parent`, `body1 = child` 규칙을 지킨다.

### 2. Closed-loop는 조심해야 한다

원본 MJCF에는 `equality/connect`가 20개 있다. MuJoCo에서는 이런 closed-loop constraint가 자연스럽게 쓰일 수 있지만, Isaac Sim으로 가져올 때는 importer와 PhysX articulation 안정성 면에서 부담이 된다.

Isaac Sim에는 closed-loop 구조를 rigging하는 방법이 있지만, 이것은 일반적인 importer path보다 더 세심한 joint/limit/material 설정이 필요하다. 실제 grasp가 목표라면 첫 구현은 closed-loop를 버리고 tree 구조로 단순화하는 것이 낫다.

프로젝트 적용:

- 원본 손의 closed-loop linkage는 직접 재현하지 않는다.
- 구동 관절 8개는 유지할 수 있다.
- passive linkage는 mimic/coupling/controller로 근사한다.
- 필요하면 2차 단계에서 closed-loop 근사 품질을 높인다.

### 3. Visual mesh와 collision geometry는 분리한다

비주얼 STL은 보기 좋게 만드는 용도이고, contact physics는 안정적으로 부딪히는 용도다. 두 목적은 다르다.

작은 손가락 부품에 원본 STL triangle mesh collision을 그대로 쓰면 다음 문제가 생긴다.

- contact가 불안정해질 수 있다.
- 손가락이 물체를 뚫거나 튕길 수 있다.
- solver 비용이 커진다.
- 작은 쓰레기처럼 가볍고 작은 물체는 접촉 오차에 민감하다.

공식 Isaac Sim/PhysX 문서와 튜토리얼은 collision approximation, convex hull, primitive collider, physics material을 중요한 설정으로 다룬다.

프로젝트 적용:

- 기존 STL: visual 전용
- 손바닥 collision: box 또는 convex hull
- 손가락 마디 collision: capsule 또는 rounded box
- fingertip collision: 넓은 pad 형태의 capsule/box
- 작은 나사, 핀, washer 등: visual만 유지하고 collision은 생략

### 4. Grasp는 마찰과 contact pad가 중요하다

작은 쓰레기를 집을 때는 손가락이 닫히는 것보다 물체가 미끄러지지 않는 것이 더 중요하다.

Isaac Sim closed-loop gripper 튜토리얼에서는 fingertip physics material을 추가하고 마찰 계수를 높이는 방식을 사용한다. 프로젝트에서도 같은 원칙을 쓴다.

프로젝트 적용:

- fingertip material:
  - static friction: 높게
  - dynamic friction: 높게
  - friction combine mode: 가능하면 `max`
- 물체 material:
  - 너무 낮은 friction은 피한다.
  - 플라스틱/종이/고무류를 나눠 테스트한다.
- restitution:
  - 작은 쓰레기는 튀면 grasp 검증이 어려우므로 낮게 둔다.

### 5. Drive는 강하게만 주면 안 된다

물체를 집는 손은 rigid하게 닫히는 clamp가 아니라, 접촉 후 적당히 힘을 유지하는 compliant gripper처럼 동작해야 한다.

너무 강한 drive:

- 물체를 튕긴다.
- 손가락이 contact를 뚫고 들어간다.
- solver가 불안정해진다.

너무 약한 drive:

- 물체가 미끄러진다.
- 들어 올릴 때 grasp가 풀린다.

프로젝트 적용:

- position drive를 사용하되 force limit을 둔다.
- stiffness/damping을 튜닝한다.
- grasp command를 바로 1.0으로 점프하지 말고 ramp로 닫는다.
- contact 후 holding phase를 둔다.

### 6. 검증은 스크린샷이 아니라 grasp stability다

팔 검증은 여러 joint pose screenshot으로 충분할 수 있다. 하지만 손 검증은 다르다.

손 검증 기준:

- 물체와 손가락 contact 발생
- 물체가 손바닥/손가락 사이에 유지됨
- 들어 올리기 전후 물체 pose가 안정적임
- 2~3초 동안 slip 없이 유지
- 여러 형태에서 반복 성공

## 작은 쓰레기 grasp 목표

사용자 목표:

- 작은 쓰레기
- 형태가 많이 무너지지 않은 물체
- 실제로 집어야 함

1차 테스트 물체 범위:

- 크기: 약 2~8 cm
- 질량: 가벼운 rigid/semirigid 물체
- 형태:
  - 작은 cube
  - 작은 cylinder
  - 낮은 box 조각
  - 불규칙 convex debris
  - 병뚜껑에 가까운 납작한 원통

1차에서 제외할 물체:

- 비닐처럼 크게 휘는 물체
- 종이처럼 접히는 물체
- cloth/deformable body가 필요한 물체
- 매우 얇아서 collider가 거의 없는 물체

이들은 2차 deformable/soft-body 단계로 분리한다.

## 권장 손 모델 구조

첫 구현은 안정성을 우선한다.

```text
HandRoot / wrist_interface
└── palm
    ├── finger_1_base
    │   └── finger_1_proximal
    │       └── finger_1_distal
    ├── finger_2_base
    │   └── finger_2_proximal
    │       └── finger_2_distal
    ├── finger_3_base
    │   └── finger_3_proximal
    │       └── finger_3_distal
    └── finger_4_base
        └── finger_4_proximal
            └── finger_4_distal
```

권장 DOF:

- finger당 2 actuated revolute joints
- 총 8 actuated joints
- 원본 actuator 8개와 매핑하기 좋음

대체 DOF:

- finger당 3 joints
- distal joint는 mimic 또는 coupled drive
- contact shape는 더 자연스럽지만 튜닝 난이도가 올라간다.

## 기존 visual mesh 재사용 전략

원본 MJCF의 asset mesh:

- `r_wrist_interface.stl`
- `r_hand_plate.stl`
- `proximal.stl`
- `proximal_shell.stl`
- `distal.stl`
- `distal_shell.stl`
- `custom_servo_horn.stl`
- `rotule_*`, `link.stl`, `gimbal.stl`, `pin`, `washer`, screw류

1차 visual 재사용:

- wrist/palm:
  - `r_wrist_interface`
  - `r_hand_plate`
- finger main:
  - `proximal`
  - `proximal_shell`
  - `distal`
  - `distal_shell`
- optional:
  - `custom_servo_horn`

1차에서 visual-only로 둘 것:

- screw
- washer
- pin
- tiny bushing
- decorative/support linkage

이 작은 부품들을 모두 collision body로 만들면 실제 grasp stability가 나빠질 가능성이 크다.

## Collision 설계

손바닥:

- box 또는 convex hull
- 물체를 받치는 shallow concave 느낌은 여러 box로 근사

손가락 proximal:

- capsule 또는 elongated box
- 길이는 visual proximal mesh와 맞춤
- 두께는 실제보다 약간 두껍게 시작

손가락 distal/fingertip:

- capsule + pad box
- fingertip pad는 실제 visual보다 약간 넓게 시작
- contact normal이 안정적으로 나오게 한다.

물체:

- cube, cylinder, convex debris
- collision approximation은 primitive 또는 convex hull
- 너무 얇은 mesh collider는 피한다.

## 제어 설계

명령 인터페이스:

```text
grasp = 0.0  -> 손 열림
grasp = 1.0  -> 손 닫힘
```

내부 매핑:

```text
finger1_joint1 = f1(grasp)
finger1_joint2 = f2(grasp)
...
finger4_joint1 = f7(grasp)
finger4_joint2 = f8(grasp)
```

추가로 raw 8D motor command도 남겨둘 수 있다.

```text
finger1_motor1
finger1_motor2
finger2_motor1
finger2_motor2
finger3_motor1
finger3_motor2
finger4_motor1
finger4_motor2
```

grasp 실행은 ramp 방식이 좋다.

```text
open -> approach -> close slowly -> hold -> lift -> hold -> release
```

## Grasp 검증 시나리오

최소 검증:

1. 손을 물체 위/옆으로 이동
2. 손가락을 천천히 닫음
3. contact 발생 확인
4. 팔을 위로 10~20 cm 이동
5. 2~3초 유지
6. 물체가 손 안에 남아 있으면 pass

테스트 물체:

- `small_cube_3cm`
- `small_cylinder_4cm`
- `flat_box_6cm`
- `convex_debris_a`
- `bottle_cap_like_cylinder`

측정 항목:

- grasp success/fail
- object height after lift
- object slip distance
- contact count
- max joint effort
- whether object penetrates finger collider
- whether simulation remains stable

## 성공 기준 초안

1차 성공:

- 손이 Isaac articulation으로 로드된다.
- 8개 손가락 joint가 제어 가능하다.
- 기존 visual mesh 중 wrist, palm, proximal, distal 계열이 표시된다.
- 작은 cube/cylinder를 들어 올릴 수 있다.
- 5회 반복 중 3회 이상 grasp 유지에 성공한다.

2차 성공:

- 불규칙 convex debris를 들어 올릴 수 있다.
- visual mesh 배치가 원본 손과 크게 어긋나지 않는다.
- contact tuning 후 5회 중 4회 이상 성공한다.

3차 성공:

- 실제 perception/teleop control contract와 연결한다.
- grasp scalar 또는 8D raw hand command를 LeRobot action schema에 포함한다.
- 다양한 쓰레기 형태에 대해 자동 회귀 테스트를 돌린다.

## 구현 전 주의점

원본 MJCF를 그대로 고치려는 접근은 비용 대비 불확실성이 크다.

이유:

- importer crash가 C++ 내부에서 난다.
- closed-loop/equality를 모두 유지하려면 importer 호환성을 계속 추적해야 한다.
- 설령 import가 되더라도 contact grasp 안정성은 별도 문제다.

따라서 구현 순서는 다음이 좋다.

1. MJCF에서 visual mesh와 대략적인 link placement를 읽는다.
2. Isaac용 hand USD/URDF를 새로 author한다.
3. visual mesh는 link 아래 reference로 붙인다.
4. collision은 단순 primitive로 별도 author한다.
5. joint drive/material/solver를 grasp 기준으로 튜닝한다.
6. grasp 테스트로 검증한다.

## 참고 자료

공식 문서:

- Isaac Sim 5.1 Physics Simulation Fundamentals  
  https://docs.isaacsim.omniverse.nvidia.com/5.1.0/physics/simulation_fundamentals.html
- Isaac Sim 5.1 Tutorial: Articulate a Basic Robot  
  https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup_tutorials/tutorial_gui_simple_robot.html
- Isaac Sim 5.1 Tutorial: Rig Closed-Loop Structures  
  https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup_tutorials/rig_closed_loop_structures.html
- Isaac Sim 5.1 Articulation Joint Sensors  
  https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_physics_articulation_force.html
- PhysX Geometry documentation  
  https://nvidia-omniverse.github.io/PhysX/physx/5.1.2/docs/Geometry.html

프로젝트 증거:

- `isaacsim_test/isaacsim/robot_arm_hand_from_zip.py`
- `isaacsim_test/test_robot_arm_hand_from_zip.py`
- `isaacsim_test/artifacts/runtime_logs/robot_arm_hand_from_zip_20260702T065932Z.log`
- `isaacsim_test/outputs/robot_arm_hand_from_zip_fixed_20260702T065932Z/robot_arm_hand_connected_report.json`

