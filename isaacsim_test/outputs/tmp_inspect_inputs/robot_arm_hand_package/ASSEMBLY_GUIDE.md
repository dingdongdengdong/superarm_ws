# ASSEMBLY GUIDE — 팔 + 손 연결 방법

로봇 팔(URDF)과 AmazingHand(MJCF)를 Isaac Sim에서 하나의 로봇으로 결합하는
전체 절차입니다. 순서대로 따라 하세요.

---

## 0. 전체 그림

```
   [팔 URDF]                         [손 MJCF]
   base_link (고정)                   r_wrist_interface (손 루트)
      │                                    │
      ... 5-DOF ...                        ├─ finger1 (motor1, motor2)
      │                                    ├─ finger2 (motor1, motor2)
   wrist_adapter_arm                       ├─ finger3 (motor1, motor2)
      │                                    └─ finger4 (motor1, motor2)
   wrist_adapter_hand ── hand_mount ◄──── 여기에 손 루트를 정렬/부착
```

연결의 핵심은 딱 하나:
**팔의 `hand_mount` 지점에 손의 `r_wrist_interface`(pos 0 0 0)를 맞붙인다.**

---

## 1. 부착 지점 (attach point) — 좌표

Fusion 360에서 실측한 값입니다. 단위는 meter, **월드 원점(설계 원점) 기준**입니다.

| 지점            | X (m)     | Y (m)     | Z (m)      | 의미                       |
|-----------------|-----------|-----------|------------|----------------------------|
| `hand_mount`    | -0.035112 |  0.282649 |  0.609522  | 손이 붙는 면(손목 어댑터 끝) |
| `base_fixed`    | -0.040112 |  0.282789 |  0.009519  | 팔이 세상에 고정되는 지점   |

> **base_link(=base_fixed) 기준 상대 좌표로 변환:**
> hand_mount - base_fixed = **(0.0050, -0.00014, 0.600003)** [m]
> 즉 팔 베이스에서 손 부착면까지 **Z 방향으로 정확히 0.600 m**, X는 +5mm,
> Y는 거의 0. 팔이 거의 수직으로 서 있고 그 끝에 손이 붙는다는 뜻입니다.
>
> ※ 이 상대좌표(특히 ΔZ=0.600 m)는 재측정으로 검증된 값입니다. 손을 팔
>   base_link 프림 하위에 놓고 이 로컬 위치를 주면 정확히 손목 끝에 옵니다.

방향(회전)은 팔 URDF 안의 `hand_mount`(또는 `wrist_adapter_hand`) 링크의
프레임 방향을 따릅니다. 손의 `r_wrist_interface`는 자기 로컬 원점이 곧
손목 접합면이므로, 두 프레임을 **원점 정렬(coincident)** 시키면 됩니다.

> ⚠️ 방향 미세조정: 손을 붙였을 때 손바닥/손가락이 의도한 방향을 향하지 않으면,
> 손 프리미티브에 quaternion 회전을 주어 맞춥니다. (§4 참고)

---

## 2. 팔 URDF → USD 변환 (Isaac Sim)

Isaac Sim(또는 Isaac Lab)에서 URDF Importer를 사용합니다.

### GUI 방식
1. Isaac Sim 실행 → `File > Import` (또는 Isaac Utils > Workflows > URDF Importer)
2. `arm_description/urdf/robot_arm_hand_urdf.xacro`를 선택
   - xacro라면 먼저 평탄화(flatten)가 필요할 수 있습니다. ROS 없이 하려면:
     `pip install xacrodoc` 후
     `python -c "from xacrodoc import XacroDoc; XacroDoc.from_file('robot_arm_hand_urdf.xacro').to_urdf_file('arm.urdf')"`
3. 임포트 옵션:
   - **Fix Base Link: 체크** (팔은 고정 베이스)
   - **Stage Units Per Meter: 1.0**
   - **Self Collision: 끄기** (권장, 필요 시만 켬)
   - Joint Drive: 각 revolute 관절을 Position 드라이브로

### 스크립트 방식 (Isaac Lab)
```bash
./isaaclab.sh -p scripts/tools/convert_urdf.py \
    arm_description/urdf/arm.urdf \
    arm.usd \
    --fix-base --joint-stiffness 0.0 --joint-damping 0.0
```

> ⚠️ **gazebo/transmission 태그 주의:** 이 URDF에는 `.gazebo`, `.trans` 파일이
> 딸려 있습니다. Isaac Sim URDF Importer는 `<gazebo>`, `<transmission>` 태그를
> 지원하지 않으므로, xacro 평탄화 시 이들을 include하지 말거나 변환 후 제거하세요.
> fixed joint가 병합되는 게 싫으면 fixed joint에 `<dont_collapse>`를 추가합니다.

---

## 3. 손 MJCF → USD 변환 (Isaac Sim)

손은 MJCF Importer를 사용합니다.

### GUI 방식
1. `File > Import` → MJCF 선택 (안 보이면 `Window > Extensions`에서
   `isaacsim.asset.importer.mjcf` 활성화)
2. `hand_mjcf/scene.xml`이 아니라 **`hand_mjcf/robot.xml`을 임포트**하세요.
   (scene.xml은 바닥/조명/IK타겟까지 포함하므로, 로봇만 원하면 robot.xml)
3. 임포트 옵션:
   - **Fix Base: 끄기** (손은 팔에 붙을 것이므로 베이스 고정 불필요.
     단, 손만 단독 테스트할 땐 켜도 됨)
   - meshdir은 `assets/`이며 robot.xml에 상대경로로 지정되어 있음

### ★ 병렬 메커니즘(closed-loop) 보존 — 가장 중요
손가락은 `<equality><connect>` 22개로 구현된 **폐루프**입니다.
- Isaac Sim MJCF Importer는 equality/connect 제약을 USD의 물리 조인트로
  변환합니다. 임포트 후 **손가락을 움직여 보고**, 두 마디가 연동되어 접히는지
  확인하세요.
- 만약 손가락이 따로 놀거나 루프가 끊겼다면, 임포트 시 equality가 누락된 것이니
  USD에서 해당 지점에 `PhysicsJoint`(D6/Spherical)를 수동 추가해 closing site를
  다시 이어야 합니다. (robot.xml의 `<equality>` 블록에 어떤 site끼리 연결하는지
  명시되어 있음 — closing_1..3, closing_ball1..4)
- **대안:** 병렬 재현이 너무 까다로우면, 각 손가락을 proximal→distal 2관절
  직렬로 근사하고 두 관절을 mimic/coupled로 묶는 방법도 있습니다(움직임은
  비슷하되 실제 병렬 동역학과는 다름).

### 손 액추에이터 (제어 인터페이스)
- 8개 position 액추에이터: `finger1_motor1/2` ~ `finger4_motor1/2`
- 손가락 하나당 서보 2개가 flexion/extension + abduction/adduction을 함께 생성.
- `keyframes.xml`의 `zero` 키프레임 = 손을 편 초기 자세.

---

## 4. 팔 + 손 결합 (USD 조립)

두 USD를 하나의 Stage에서 합칩니다.

1. 새 Stage에 **팔 USD를 로드**(reference)합니다. base_link가 원점 고정.
2. 같은 Stage에 **손 USD를 reference**로 추가합니다.
3. 손 프림(root=`r_wrist_interface`)의 Transform을 팔의 `hand_mount`에 맞춥니다:
   - Position: §1의 base_link 기준 상대좌표 **(0.0050, -0.00014, 0.600003) m**
     (팔 base_link 프림 하위에 손을 놓고 이 값을 로컬 위치로 지정)
   - Orientation: 손바닥/손가락 방향이 맞도록 quaternion 조정
     (기본은 회전 없음부터 시작해, 화면 보며 90°/180° 단위로 맞춤)
4. 손 루트를 팔의 `hand_mount`(또는 `wrist_adapter_hand`) 프림에
   **Fixed Joint**로 고정합니다. (팔 끝과 손이 강체로 붙음)
5. Play를 눌러 팔을 움직였을 때 손이 따라오고, 손가락이 접히는지 확인.

> **팁 — 정렬 검증:** 결합 후 팔 손목 관절을 회전시켜 손이 자연스럽게
> 따라 도는지 보세요. 손이 엉뚱한 축으로 돌면 orientation quaternion을
> 다시 잡아야 합니다.

---

## 5. 좌표계 참고 (손 루트 세부)

- 손 루트 `r_wrist_interface`: `pos="0 0 0" quat="1 0 0 0"` (로컬 원점 = 접합면)
- 손의 여러 부품(hand_plate, wrist_interface geom 등)은
  `pos="-0.0115 0.0245 0.1022"` 부근에 배치 → 손 몸통이 +Z로 뻗는 형태.
- 따라서 팔의 hand_mount에서 손이 **팔 축(+Z) 방향으로 이어지도록** 놓으면
  자연스럽습니다.

---

## 6. 남은 개선 사항 (선택)

1. **팔 관절 limit:** 현재 팔의 회전 관절 4개는 `continuous`(무한회전)입니다.
   실제 모터 범위로 제한하려면 아래 rpo 원본 값을 참고해 각 관절을
   `revolute` + `<limit>`으로 바꾸세요. (RL 품질/sim2real에 중요)

   | 관절            | axis    | lower  | upper | effort | velocity |
   |-----------------|---------|--------|-------|--------|----------|
   | arm_pitch       | 0 1 0   | -3.14  | 1.57  | 27     | 8.0      |
   | arm_roll        | 1 0 0   | -3.14  | 0.25  | 27     | 8.0      |
   | arm_yaw         | 0 0 -1  | -1.57  | 1.57  | 27     | 8.0      |
   | elbow_pitch     | 0 1 0   | -0.6   | 1.57  | 27     | 8.0      |
   | elbow_yaw       | 1 0 0   | -1.57  | 1.57  | 27     | 8.0      |

   (주의: 위는 rpo 원본 오른팔 관절 순서 기준. 현재 fusion2urdf 팔의
   joint_rev_1~4와 1:1로 매핑되는지는 축 방향을 보고 확인하세요.)

2. **카메라(camera_d435):** 현재 팔 URDF 말단에 fixed로 포함되어 있습니다.
   RealSense 센서로 쓰려면 Isaac Sim에서 카메라 프림을 이 링크에 부착하세요.

3. **instanceable USD:** 대규모 병렬 RL(수천 개 환경)을 돌린다면 팔·손 USD를
   instanceable 포맷으로 저장해 메모리를 절약하세요.

---

## 7. 체크리스트

- [ ] 팔 URDF → USD 변환 성공 (base 고정, 5관절 확인)
- [ ] 손 MJCF → USD 변환 성공
- [ ] 손가락 병렬 메커니즘 동작 확인 (접힘 연동)
- [ ] 손을 hand_mount에 부착 (위치·방향 정렬)
- [ ] 팔+손 fixed joint 결합
- [ ] Play 후 팔 움직임에 손이 따라오는지 확인
- [ ] (선택) 팔 관절 limit 적용
- [ ] (선택) 카메라/instanceable 설정
