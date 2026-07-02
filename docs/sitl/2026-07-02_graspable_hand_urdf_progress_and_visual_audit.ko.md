# Isaac Sim 손 URDF 진행 기록 및 이미지 판독

작성일: 2026-07-02

## 목표

원본 AmazingHand MJCF를 그대로 Isaac Sim에 넣는 대신, Isaac이 안정적으로 처리하는 tree articulation 손을 별도로 만들고 기존 STL visual을 재사용하는 것이 목표였다. 최종 용도는 작은 쓰레기처럼 형태가 크게 무너지지 않은 물체를 실제 접촉 물리로 집는 것이다.

## 진행 순서

1. 원본 `robot_arm_hand_package.zip`을 풀고 arm URDF와 hand MJCF를 분리해서 검사했다.
2. arm 쪽은 URDF import와 articulation 제어가 가능했다.
3. hand 쪽은 원본 MJCF의 closed-loop 구조와 `equality/connect` 제약 때문에 Isaac MJCF importer가 실패했다.
4. 원본 MJCF를 sanitize해서 다시 시도했지만 `MJCFCreateAsset` 단계에서 계속 실패했다.
5. 그래서 원본 MJCF는 구조 참고 자료로만 두고, Isaac용 손을 별도 URDF로 생성하는 방식을 구현했다.
6. 새 URDF는 다음 조건으로 만들었다.
   - root link: `r_wrist_interface`
   - finger actuator 이름: `finger1_motor1`부터 `finger4_motor2`까지 8개 유지
   - visual: 기존 STL 파일 재사용
   - collision: STL convex가 아니라 primitive box collision 사용
   - closed-loop/equality constraint 없음
7. Isaac 변환 파이프라인은 다음 순서로 바뀌었다.
   - 원본 hand MJCF import 시도
   - 실패 시 `amazinghand_graspable.urdf` import
   - 이마저 실패하면 기존 proxy hand 사용

## 구현된 파일

- 계획 문서: `docs/superpowers/plans/2026-07-02-graspable-hand-articulation.md`
- 손 URDF 생성기: `isaacsim_test/isaacsim/graspable_hand_urdf.py`
- zip 파이프라인 통합: `isaacsim_test/isaacsim/robot_arm_hand_from_zip.py`
- 정적 테스트:
  - `isaacsim_test/test_graspable_hand_urdf.py`
  - `isaacsim_test/test_robot_arm_hand_from_zip.py`

## 검증 증거

최종 Isaac 실행 산출물:

- report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_fix3/robot_arm_hand_connected_report.json`
- screenshots: `isaacsim_test/artifacts/robot_arm_hand_graspable_20260702_fix3/`
- contact sheet: `isaacsim_test/artifacts/robot_arm_hand_graspable_20260702_fix3/contact_sheet.png`

report 결과:

- 전체 status: `PASS_WITH_FALLBACK`
- Isaac conversion: `PASS_WITH_FALLBACK`
- 원본 MJCF hand: `BLOCKED`
- 새 graspable URDF fallback: `PASS`
- runtime validation: `PASS`
- fixed joint body0: `/World/RobotArmHandFromZip/Arm/wrist_adapter_hand`
- fixed joint body1: `/World/RobotArmHandFromZip/Hand/r_wrist_interface`
- loaded DOF:
  - arm: `joint_rev_1`, `joint_rev_2`, `joint_rev_3`, `joint_rev_4`
  - hand: `finger1_motor1`, `finger2_motor1`, `finger3_motor1`, `finger4_motor1`, `finger1_motor2`, `finger2_motor2`, `finger3_motor2`, `finger4_motor2`

생성된 screenshot 크기:

- `01_startup.png`: 574782 bytes
- `02_home.png`: 574143 bytes
- `03_reach.png`: 571762 bytes
- `04_fold.png`: 597118 bytes
- `05_side_sweep.png`: 588397 bytes
- `contact_sheet.png`: 457037 bytes

## 이미지 판독

최종 contact sheet를 보면 arm articulation은 정상적으로 움직이고, 손 쪽 rigid bodies와 finger DOF도 scene에 올라와 있다. 다만 시각적으로는 아직 문제가 있다.

관찰한 문제:

- 손목 부품이 팔 끝에 자연스럽게 붙어 보이지 않는다.
- 이미지상으로는 손바닥 또는 손가락 묶음이 팔목 위쪽에 떠 있는 것처럼 보인다.
- finger STL visual은 처음보다 손가락 형태로 보이지만, palm/wrist 기준 좌표계 정렬이 아직 CAD 원본과 맞지 않는다.
- 현재 단순 모델은 네 손가락을 모두 비슷한 parallel finger로 배치했다. 사용자가 원하는 구성은 총 4개 손가락이며, 새끼손가락을 제외한 구성으로 이해해야 한다.

중요한 구분:

- 물리 연결 자체는 report상 `/Arm/wrist_adapter_hand`에서 `/Hand/r_wrist_interface`로 걸려 있다.
- 그러나 visual origin/orientation이 틀어져 있어서 실제 이미지에서는 손목이 팔에 붙은 것처럼 보이지 않는다.
- 즉 현재 문제는 fixed joint target이 손바닥으로 잘못 걸린 문제라기보다, 손 URDF의 visual/link 배치가 부정확한 문제다.

## 원본 MJCF에서 확인한 구조

원본 MJCF root body는 `r_wrist_interface`다. 이 root body 아래에 palm/wrist plate visual과 finger actuator들이 함께 들어 있다.

확인된 actuator:

- `finger1_motor1`
- `finger1_motor2`
- `finger2_motor1`
- `finger2_motor2`
- `finger3_motor1`
- `finger3_motor2`
- `finger4_motor1`
- `finger4_motor2`

원본은 각 finger가 단순 2-link serial chain이 아니라 ball joint, passive hinge, equality/connect로 닫힌 링크를 만든다. 이 구조는 MuJoCo에서는 의미가 있지만 Isaac URDF/tree articulation에는 그대로 옮기면 안 된다.

## 현재 한계

현재 구현은 “Isaac에서 import되고 움직이는 안정 tree articulation”을 먼저 달성한 버전이다. 하지만 실제 손 형태 복원은 아직 충분하지 않다.

남은 문제:

- palm과 wrist interface의 link-local transform을 원본 MJCF 기준으로 다시 잡아야 한다.
- thumb/index/middle/ring처럼 새끼손가락을 제외한 4-finger 레이아웃을 명확히 해야 한다.
- finger visual STL의 mesh-local 축과 link-local 축을 더 정확히 맞춰야 한다.
- collision primitive도 visual에 맞게 재배치해야 한다.
- 작은 쓰레기 집기 테스트를 위해 fingertip pad, friction, solver iteration, drive gain을 튜닝해야 한다.

## 다음 수정 기준

다음 단계에서는 다음 순서로 수정한다.

1. 원본 MJCF의 `r_wrist_interface`를 손의 실제 mount root로 유지한다.
2. arm fixed joint는 계속 `/World/RobotArmHandFromZip/Arm/wrist_adapter_hand`에서 `/World/RobotArmHandFromZip/Hand/r_wrist_interface`로 유지한다.
3. palm은 `r_wrist_interface` 아래 fixed child로 두되, visual에서 palm이 팔목에 직접 붙어 보이지 않도록 wrist plate와 hand plate offset을 조정한다.
4. 네 손가락은 사용자가 지적한 요구대로 새끼손가락을 제외한 4-finger 구성을 기준으로 재배치한다.
5. 각 finger visual과 collision이 같은 위치에 보이도록 link-local transform을 보정한다.
6. 수정 후 Isaac에서 다시 `convert`와 `runtime`을 실행하고 contact sheet를 확인한다.

## 2026-07-02 레이아웃 수정 후 재검증

사용자 지적 사항:

- 손목 부품이 팔에 달린 것이 아니라 손바닥이 팔목에 직접 달린 것처럼 보였다.
- 총 손가락은 4개이며, 새끼손가락은 제외된 구성이어야 한다.
- 이미지 판독 없이 report만 보고 통과로 판단하면 안 된다.

이에 따라 `isaacsim_test/isaacsim/graspable_hand_urdf.py`를 수정했다.

수정 내용:

- 손 root는 계속 `r_wrist_interface`로 유지했다. arm과 hand의 fixed joint target은 그대로 `/Arm/wrist_adapter_hand` -> `/Hand/r_wrist_interface`다.
- `r_wrist_interface` visual origin을 원본 STL 중심에 맞춰 보정했다.
- `r_hand_plate` palm visual origin을 손목 root 아래 child로 재배치했다.
- 손가락 레이아웃을 parallel 4-finger에서 `index`, `middle`, `ring`, `thumb` 구성으로 바꿨다.
- `excluded_human_finger`를 `pinky`로 명시했다.
- proximal/distal finger collision box는 visual 방향과 맞도록 손가락 진행 방향을 link-local Y축으로 바꿨다.
- thumb은 옆쪽에서 들어오는 별도 base pose와 Z축 회전 joint를 사용하도록 분리했다.

수정 후 정적 검증:

- `python3 isaacsim_test/test_graspable_hand_urdf.py`: `OK`, 3 tests
- `python3 isaacsim_test/test_robot_arm_hand_from_zip.py`: `OK`, 11 tests

수정 후 Isaac 런타임 검증:

- output root: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_layoutfix`
- artifact root: `isaacsim_test/artifacts/robot_arm_hand_graspable_20260702_layoutfix`
- report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_layoutfix/robot_arm_hand_connected_report.json`
- overall status: `PASS_WITH_FALLBACK`
- runtime validation: `PASS`
- loaded DOF:
  - arm: `joint_rev_1`, `joint_rev_2`, `joint_rev_3`, `joint_rev_4`
  - hand: `finger1_motor1`, `finger2_motor1`, `finger3_motor1`, `finger4_motor1`, `finger1_motor2`, `finger2_motor2`, `finger3_motor2`, `finger4_motor2`

수정 후 screenshot artifact:

- `01_startup.png`: 579585 bytes
- `02_home.png`: 581126 bytes
- `03_reach.png`: 564477 bytes
- `04_fold.png`: 587334 bytes
- `05_side_sweep.png`: 581769 bytes
- `contact_sheet.png`: 439983 bytes

수정 후 이미지 판독:

- 이전처럼 손바닥 덩어리가 바로 팔목에 붙어 보이는 상태는 완화됐다.
- 팔 끝에는 `wrist_adapter_hand`가 있고, 그 위에 `r_wrist_interface` 원통형 손목 부품이 올라가며, 그 위쪽에 palm/finger assembly가 배치된다.
- 화면상 손가락은 세 개의 전방 손가락과 한 개의 엄지로 읽힌다. 이는 "새끼손가락 제외 4손가락" 요구와 일치하는 방향이다.
- 다만 thumb/fingertip의 실제 접촉 위치와 grip envelope은 아직 물체 집기 테스트로 검증하지 않았다.

현재 결론:

- 이번 수정으로 "Isaac에서 import되는 안정 tree articulation"과 "이미지상 손목-root 기반 hand assembly"는 통과했다.
- 아직 최종 손 물리는 아니다. 작은 쓰레기를 실제로 집으려면 다음 단계에서 collision pad, fingertip friction, drive stiffness/damping, articulation solver 설정, 테스트 물체 세트를 추가해야 한다.

## 2026-07-02 원본 AmazingHand visual shell 적용

사용자 재지적 사항:

- screenshot 파일이 생긴 것만으로는 충분하지 않다.
- 이미지를 직접 보고 손 비주얼이 제대로 되었는지 판독해야 한다.
- generated hand가 원본 AmazingHand 구현/변환 자료와 실제로 매칭되는지 확인해야 한다.

Root cause:

- 이전 generated URDF는 원본 MJCF의 visual 구조를 보존하지 않았다.
- 원본 MJCF에는 총 162개의 mesh geom visual이 있고, 각 geom은 `pos`와 `quat`로 배치되어 있다.
- 이전 URDF는 `r_wrist_interface`, `r_hand_plate`, `proximal`, `distal` 같은 일부 STL만 임의 origin으로 넣었다.
- 따라서 손가락 수는 보였지만 AmazingHand 원본의 servo horn, finger frame, screw, ball, link, pin 계열 부품이 빠져 있었다.

수정 내용:

- `isaacsim_test/isaacsim/graspable_hand_urdf.py`에서 MJCF `robot.xml`을 직접 읽도록 했다.
- MJCF body/geom transform을 재귀적으로 누적해 각 visual mesh의 root 기준 위치와 회전을 계산했다.
- MJCF quaternion은 URDF `rpy`로 변환했다.
- 원본 MJCF의 162개 mesh geom을 모두 `amazinghand_visual_shell` fixed link에 넣었다.
- 물리용 tree articulation과 primitive collision은 그대로 유지했다.

정적 검증:

- `python3 isaacsim_test/test_graspable_hand_urdf.py`: `OK`, 3 tests
- `python3 isaacsim_test/test_robot_arm_hand_from_zip.py`: `OK`, 14 tests
- generated URDF visual count: `162`
- `mjcf_visual_geom_count`: `162`
- `missing_mjcf_visual_meshes`: `[]`
- 대표 포함 mesh:
  - `r_wrist_interface.stl`
  - `r_hand_plate.stl`
  - `finger_frame_1.stl`
  - `finger_frame_2.stl`
  - `scs0009.stl`
  - `custom_servo_horn.stl`
  - `rotule_ball.stl`
  - `proximal.stl`
  - `distal.stl`

Isaac 검증:

- output root: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_visualshell`
- artifact root: `isaacsim_test/artifacts/robot_arm_hand_graspable_20260702_visualshell`
- report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_visualshell/robot_arm_hand_connected_report.json`
- overall status: `PASS_WITH_FALLBACK`
- runtime validation: `PASS`
- contact sheet: `isaacsim_test/artifacts/robot_arm_hand_graspable_20260702_visualshell/contact_sheet.png`

이미지 판독:

- 새 contact sheet에서는 손목 위에 원본 AmazingHand의 조립식 손 구조가 보인다.
- 손바닥/프레임/손가락 링크/힌지 계열 visual이 이전보다 훨씬 원본에 가깝게 복원됐다.
- 팔 끝에 손목 interface가 있고, 그 위로 AmazingHand visual assembly가 올라간다. 이전처럼 단순 box/palm 또는 막대 손가락만 보이는 상태는 아니다.
- 총 손가락 구성은 새끼손가락 제외 4-finger 요구에 맞는 형태로 읽힌다.

현재 한계:

- 이번 단계는 "원본 AmazingHand static visual shell"을 맞춘 것이다.
- 손가락별 visual을 tree articulation의 각 moving link에 완전히 분해해서 붙인 것은 아직 아니다.
- 따라서 실제 집기 물리에서 finger collision은 움직이지만, 원본 전체 visual shell은 손목 기준 fixed visual이다.
- 다음 단계에서는 visual shell을 finger1/finger2/finger3/thumb link별로 나눠서, 실제 hand DOF와 visual이 같이 움직이도록 분해해야 한다.

## 왜 이번 접근에서 해결됐는가

이전 접근이 잘 안 된 이유:

- 처음에는 Isaac에서 안정적으로 import되는 tree articulation을 만드는 데 집중했다.
- 원본 MJCF가 closed-loop equality/connect 구조라서 그대로 Isaac articulation으로 가져오면 실패했고, 그래서 물리 구조를 단순 open-chain으로 다시 만들었다.
- 이 과정에서 visual도 같이 단순화했다. 즉 "물리적으로 Isaac이 먹는 손"은 만들었지만, "원본 AmazingHand처럼 보이는 손"을 보존하지 못했다.
- screenshot을 생성하고 크기/report만 확인한 것도 부족했다. 이미지 파일이 존재한다는 사실은 렌더 성공만 의미하지, 손목/손바닥/손가락 visual이 맞다는 뜻은 아니다.

사용자 질문이 문제 해결에 준 영향:

- "이미지 검증해봤나요?"라는 지적이 판단 기준을 바꿨다.
- 이전 기준은 `runtime PASS`, DOF load, screenshot 생성 여부였다.
- 이후 기준은 contact sheet를 직접 보고 원본 AmazingHand visual과 비교하는 것으로 바뀌었다.
- "AmazingHand 구현한 부분이랑 변환한 거랑 매칭"이라는 요구 때문에 generated URDF의 visual origin을 감으로 조정하는 방식을 버리고, 원본 MJCF의 visual transform을 데이터 소스로 삼게 됐다.

새로 찾은 접근:

- 물리 구조와 visual 구조를 분리했다.
- 물리는 Isaac이 안정적으로 처리하는 tree articulation으로 유지했다.
- visual은 원본 MJCF의 `body/geom mesh/pos/quat`를 읽어서 `amazinghand_visual_shell`에 그대로 재구성했다.
- 즉 MuJoCo의 closed-loop constraint는 버리지만, CAD에서 온 visual mesh 배치 정보는 버리지 않는 방식이다.

이번 접근이 맞았다는 증거:

- 원본 MJCF mesh geom count: `162`
- generated URDF visual count: `162`
- missing visual mesh: `0`
- Isaac runtime status: `PASS`
- contact sheet에서 손바닥, finger frame, servo horn, ball/link/pin 계열 부품이 보인다.

교훈:

- Isaac용 변환에서는 "physics import 가능성"과 "visual fidelity"를 별도 요구사항으로 다뤄야 한다.
- closed-loop hand는 물리 articulation으로 그대로 가져오지 말고, tree physics와 original visual shell을 분리해서 결합하는 편이 안정적이다.
- 이미지 검증은 artifact 존재 여부가 아니라 사람이 contact sheet를 판독하고, 원본 구조와 비교하는 단계까지 포함해야 한다.

## 2026-07-02 per-link visual partition

이 단계의 목적:

- 이전 단계의 `amazinghand_visual_shell`은 원본 손 모양을 잘 보여줬지만, 손목 기준 fixed visual이었다.
- 실제 손가락 DOF를 움직일 때 visual도 같이 움직이려면 MJCF visual들을 URDF tree link에 나눠 붙여야 한다.

수정 내용:

- `amazinghand_visual_shell` fixed link를 제거했다.
- MJCF visual 162개를 다음 link로 분배했다.
  - `r_wrist_interface`: 손목/손바닥/고정 frame visual
  - `finger*_proximal`: servo horn, proximal shell, linkage 대부분
  - `finger*_distal`: distal/distal shell 계열 visual
- 각 visual은 MJCF root 기준 world transform을 먼저 계산한 뒤, 해당 URDF link의 초기 frame으로 다시 변환해서 붙였다.
- 초기 자세에서는 원본 AmazingHand 모양을 유지하고, 이후 articulation motion에서는 해당 link와 함께 움직이도록 했다.

정적 검증:

- `python3 isaacsim_test/test_graspable_hand_urdf.py`: `OK`, 3 tests
- `python3 isaacsim_test/test_robot_arm_hand_from_zip.py`: `OK`, 14 tests
- visual attachment mode: `mjcf_visuals_partitioned_to_tree_links`
- `mjcf_visual_geom_count`: `162`
- `missing_mjcf_visual_meshes`: `[]`
- link visual count:
  - `r_wrist_interface`: 26
  - `finger1_proximal`: 30
  - `finger1_distal`: 4
  - `finger2_proximal`: 30
  - `finger2_distal`: 4
  - `finger3_proximal`: 30
  - `finger3_distal`: 4
  - `finger4_proximal`: 30
  - `finger4_distal`: 4

Isaac 검증:

- output root: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_visualpartition`
- artifact root: `isaacsim_test/artifacts/robot_arm_hand_graspable_20260702_visualpartition`
- report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_visualpartition/robot_arm_hand_connected_report.json`
- overall status: `PASS_WITH_FALLBACK`
- runtime validation: `PASS`
- contact sheet: `isaacsim_test/artifacts/robot_arm_hand_graspable_20260702_visualpartition/contact_sheet.png`

이미지 판독:

- contact sheet에서 AmazingHand의 조립식 손 형상이 유지된다.
- `reach`, `fold`, `side_sweep` 자세에서도 손 visual이 팔 끝에 붙어 있고, 손가락 부분이 손목 기준 고정 덩어리처럼만 남지 않는다.
- 이전 static shell보다 다음 물리 단계에 더 적합하다. collision tree와 visual tree가 같은 link 구조를 공유하기 때문이다.

현재 남은 한계:

- MJCF closed-loop의 모든 passive linkage 운동학을 그대로 복원한 것은 아니다.
- visual 분배는 Isaac tree articulation에 맞춘 근사다.
- 다음 단계는 distal/proximal visual 분류를 더 정밀하게 하고, fingertip collision pad와 실제 물체 grasp/lift-retain 검증을 붙이는 것이다.

## 2026-07-02 손가락 구동 visual 깨짐 디버깅

사용자 재지적 사항:

- 손가락이 움직일 때 다시 visual이 깨지는 것처럼 보인다.
- 손 부분에 집중해서 원인을 찾아야 한다.
- 프로젝트 메모리도 계속 남기고 git commit에 포함해야 한다.

확인한 사실:

- 기존 canonical project memory 위치인 `omx_wiki/`에는 아직 기록 파일이 없었다.
- 앞 단계의 per-link visual partition은 Isaac report와 정적 테스트는 통과했다.
- `robot_arm_hand_graspable_20260702_visualpartition/contact_sheet.png`를 다시 판독하면, 손가락 부품들이 움직이는 link를 따라가지만 원본 AmazingHand closed-loop linkage처럼 움직이지는 않는다.

Root cause:

- 원본 AmazingHand MJCF는 단순 2-link serial finger가 아니다.
- 각 finger는 servo horn, ball, passive link, pin, shell이 equality/connect constraint로 닫힌 기구를 만든다.
- 현재 Isaac용 손은 안정성을 위해 closed-loop를 버리고 `palm -> proximal -> distal` tree articulation으로 단순화했다.
- per-link visual partition은 MJCF visual의 초기 world transform을 보존한 뒤 임의의 tree link에 나눠 붙인 근사다.
- 따라서 초기 자세에서는 AmazingHand처럼 보이지만, 손가락을 굽힐 때 visual 부품들이 원본 MJCF의 pivot/axis가 아니라 단순 URDF pivot/axis를 따라 회전한다.
- 이 차이가 사용자가 본 "움직일 때 깨짐"의 직접 원인이다.

수정 결정:

- 기본 visual mode를 `mjcf_visuals_partitioned_to_tree_links`에서 `mjcf_static_visual_shell`로 되돌렸다.
- `amazinghand_visual_shell`은 원본 MJCF의 162개 visual geom을 wrist 기준 fixed link에 그대로 보존한다.
- 물체 접촉과 집기 동작은 계속 primitive collision finger tree가 담당한다.
- per-link moving visual은 `visual_mode="partitioned_links"` 옵션으로만 남기고, 실험 모드로 취급한다.

이 결정의 의미:

- 장점: 손가락을 구동해도 원본 AmazingHand visual 조립체가 잘못된 pivot으로 찢어져 보이지 않는다.
- 장점: Isaac import 안정성과 primitive collision 기반 접촉 물리는 유지된다.
- 단점: 기본 모드에서는 원본 STL visual 손가락 자체가 굽혀 보이지 않는다. 움직이는 것은 collision finger tree다.
- 다음에 진짜 animated visual까지 맞추려면 MJCF closed-loop의 passive linkage 운동학을 tree용 구동 joint와 별도로 재구성하거나, USD에서 visual-only follower linkage를 계산해야 한다.

검증 기준:

- default generated URDF:
  - visual mode: `static_shell`
  - attachment mode: `mjcf_static_visual_shell`
  - visual geom count: `162`
  - `amazinghand_visual_shell` fixed link 존재
- optional partitioned URDF:
  - visual mode: `partitioned_links`
  - attachment mode: `mjcf_visuals_partitioned_to_tree_links`
  - 실험 모드로 유지

수정 후 정적 검증:

- `python3 isaacsim_test/test_graspable_hand_urdf.py`: `OK`, 4 tests
- `python3 isaacsim_test/test_robot_arm_hand_from_zip.py`: `OK`, 14 tests

수정 후 Isaac 검증:

- output root: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_visualfix_static`
- artifact root: `isaacsim_test/artifacts/robot_arm_hand_graspable_20260702_visualfix_static`
- report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_visualfix_static/robot_arm_hand_connected_report.json`
- overall status: `PASS_WITH_FALLBACK`
- runtime validation: `PASS`
- generated hand visual mode: `static_shell`
- generated hand attachment mode: `mjcf_static_visual_shell`
- generated hand MJCF visual geom count: `162`
- missing MJCF visual meshes: `[]`

수정 후 이미지 판독:

- `contact_sheet.png`에서 손목 위 AmazingHand 조립체가 팔 끝에 안정적으로 붙어 있다.
- 손가락 구동 pose에서도 per-link partition처럼 visual 부품이 잘못된 pivot으로 벌어지는 현상은 보이지 않는다.
- 이는 기본 모드가 원본 visual shell을 wrist 기준 fixed link로 유지하기 때문이다.
- 단, 이 상태는 visual 안정성 우선 모드다. 작은 물체를 실제로 집는 기능은 계속 primitive collision finger tree와 drive target으로 검증해야 한다.

## 2026-07-02 손가락별 2-link 물리 구동 검증

사용자 확인 사항:

- 손가락 하나당 link가 두 개다.
- 각 손가락은 `motor1`과 `motor2` 두 motor가 조종한다.
- 이를 기준으로 물리환경에서 손가락 움직임이 잘 동작하는지 먼저 테스트해야 한다.

현재 Isaac용 hand topology:

- `finger*_motor1`: `palm`에서 `finger*_proximal`을 구동한다.
- `finger*_motor2`: `finger*_proximal`에서 `finger*_distal`을 구동한다.
- 즉 손가락 하나는 `proximal`과 `distal` 두 link, 두 revolute motor로 구성된다.
- 이 구조는 `isaacsim_test/test_graspable_hand_urdf.py`의 topology assertion으로 고정했다.

추가한 runtime 검증:

- `robot_arm_hand_from_zip.py`에 `finger_motion_validation`을 추가했다.
- Isaac runtime에서 finger1부터 finger4까지 한 손가락씩 독립적으로 구동한다.
- 각 손가락에 대해 `motor1=0.78 rad`, `motor2=0.96 rad` target을 넣는다.
- physics step 후 Articulation joint position readback을 기록한다.
- 각 finger별 screenshot도 별도로 저장한다.

정적 검증:

- `python3 isaacsim_test/test_graspable_hand_urdf.py`: `OK`, 4 tests
- `python3 isaacsim_test/test_robot_arm_hand_from_zip.py`: `OK`, 15 tests

Isaac 검증:

- output root: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_finger2link`
- artifact root: `isaacsim_test/artifacts/robot_arm_hand_graspable_20260702_finger2link`
- report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_finger2link/robot_arm_hand_connected_report.json`
- overall status: `PASS_WITH_FALLBACK`
- runtime validation: `PASS`
- finger motion validation: `PASS`
- loaded DOF count: `12`
- hand DOF:
  - `finger1_motor1`, `finger2_motor1`, `finger3_motor1`, `finger4_motor1`
  - `finger1_motor2`, `finger2_motor2`, `finger3_motor2`, `finger4_motor2`

Finger별 readback:

- finger1:
  - before: `[0.0493, 0.0151]`
  - target: `[0.78, 0.96]`
  - achieved: `[0.7791, 0.9622]`
  - delta: `[0.7297, 0.9471]`
  - target error: `[0.0009, 0.0022]`
- finger2:
  - before: `[-0.0003, -0.0]`
  - target: `[0.78, 0.96]`
  - achieved: `[0.7785, 0.9614]`
  - delta: `[0.7787, 0.9614]`
  - target error: `[0.0015, 0.0014]`
- finger3:
  - before: `[-0.0003, 0.0]`
  - target: `[0.78, 0.96]`
  - achieved: `[0.7785, 0.9614]`
  - delta: `[0.7787, 0.9614]`
  - target error: `[0.0015, 0.0014]`
- finger4:
  - before: `[-0.0, 0.0]`
  - target: `[0.78, 0.96]`
  - achieved: `[0.7795, 0.9366]`
  - delta: `[0.7795, 0.9366]`
  - target error: `[0.0005, 0.0234]`

판정:

- 네 손가락 모두 두 motor가 physics articulation에서 목표 위치 근처까지 움직였다.
- 손가락별 2-link motor command는 Isaac 물리환경에서 동작한다.
- 기본 visual mode가 `static_shell`이므로 STL visual 손가락 자체는 구부러져 보이지 않는다. 이번 테스트의 판정 기준은 물리 DOF readback이다.
- 다음 단계로 작은 쓰레기 집기를 위한 collision pad, friction, drive gain, lift-retain 테스트를 진행할 수 있다.
