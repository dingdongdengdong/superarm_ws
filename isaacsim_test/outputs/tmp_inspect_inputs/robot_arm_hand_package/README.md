# Robot Arm + Hand 시뮬레이션 패키지

로봇 팔(5-DOF, rpo_description 기반)과 AmazingHand(4손가락, 8-DOF 병렬)를
결합하기 위한 전달 패키지입니다. Isaac Sim에서 강화학습(RL)에 사용하는 것을
목표로 합니다.

## 패키지 구성

```
robot_arm_hand_package/
├── README.md                 ← 이 파일 (전체 개요)
├── ASSEMBLY_GUIDE.md         ← ★ 팔+손 연결 방법 (반드시 읽을 것)
├── arm_description/          ← 로봇 팔 (URDF 형식)
│   ├── urdf/
│   │   ├── robot_arm_hand_urdf.xacro   ← 팔 본체 정의
│   │   ├── robot_arm_hand_urdf.trans   ← 액추에이터(transmission)
│   │   ├── robot_arm_hand_urdf.gazebo  ← gazebo 설정
│   │   └── materials.xacro
│   ├── meshes/               ← 팔 STL 18개 (전부 영문 이름)
│   ├── launch/ , config/     ← ROS2 launch/config
│   └── package.xml, CMakeLists.txt
└── hand_mjcf/                ← AmazingHand 오른손 (MuJoCo MJCF 형식)
    ├── scene.xml             ← 손 단독 실행 진입점
    ├── robot.xml             ← 손 본체 (body, joint, 병렬 equality)
    ├── joints_properties.xml , additional.xml , keyframes.xml
    └── assets/               ← 손 STL 29개

```

## 두 부분이 형식이 다른 이유

- **팔** = URDF: rpo_description(오른팔)을 Fusion 360에서 재구성해 fusion2urdf로 추출.
- **손** = MJCF: AmazingHand의 손가락은 **병렬 메커니즘**(closed-loop)이라 URDF로
  표현이 불가능합니다. 공식 저장소가 제공하는 MuJoCo MJCF는 이 병렬 구조를
  `<equality><connect>` 제약으로 정확히 구현하고 있어, **손의 실제 움직임이
  100% 보존**됩니다.

## 빠른 시작

1. **`ASSEMBLY_GUIDE.md`를 먼저 읽으세요.** 팔과 손을 어디에/어떻게 붙이는지,
   Isaac Sim에서 각각 USD로 변환하고 결합하는 절차가 모두 정리되어 있습니다.
2. 팔은 URDF Importer로, 손은 MJCF Importer로 Isaac Sim에 각각 임포트합니다.
3. 팔의 `hand_mount` 프레임에 손의 `r_wrist_interface`를 정렬해 결합합니다.

## 좌표/단위

- 모든 길이 단위: **meter**
- 팔 URDF: base_link가 루트(고정 베이스). Isaac Sim 임포트 시 "Fix Base Link" 권장.
- 손 MJCF: `r_wrist_interface`(pos 0 0 0)가 손의 루트.

## 출처 및 라이선스

- 팔: [Roboparty/rpo_description](https://github.com/Roboparty/rpo_description)
- 손: [pollen-robotics/AmazingHand](https://github.com/pollen-robotics/AmazingHand)
  (Demo/AHSimulation/AH_Right). 손 모델은 onshape-to-robot으로 생성됨.
- 각 원본 저장소의 라이선스를 따르세요. (arm_description/LICENSE 포함)
