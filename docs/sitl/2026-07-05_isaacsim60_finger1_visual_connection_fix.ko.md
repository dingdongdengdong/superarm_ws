# Isaac Sim 6.0 Finger1 Visual Connection Fix 기록

작성일: 2026-07-05  
브랜치: `arm-hand-isaacsim60-test`  
대상: AmazingHand generated URDF fallback hand, finger1(index finger) visual/physics linkage 검증

## 문제 요약

이전 finger1 검증은 숫자상으로는 `PASS`였지만, close-up 이미지에서 finger1 주변의 흰색 curved linkage 부품이 손가락 본체에서 떨어져 보였다.

원인은 원래 AmazingHand의 closed-loop passive linkage mesh를 단순 2-link Isaac physics tree에 잘못 나누어 붙인 것이었다. 이 부품들은 실제로 follower kinematics 없이는 손가락 proximal/distal link와 같이 움직일 수 없는데, 코드가 passive rods/pins/linkage visual까지 moving finger link로 partition해서 close-up에서 detached visual이 생겼다.

## 해결 원칙

현재 목표는 “완전한 closed-loop AmazingHand”가 아니라 Isaac Sim 6.0에서 안정적으로 움직이는 skeleton-first hand이다.

따라서 visual 정책을 다음처럼 바꿨다.

- `finger*_proximal`, `finger*_distal`에는 실제 손가락 segment shell만 attach한다.
- passive closed-loop linkage, rods, pins, servo horn 등은 wrist/root 쪽 static visual로 둔다.
- passive linkage를 같이 움직이려면 다음 단계에서 follower kinematics 또는 tree-compatible decomposition을 별도로 구현한다.

## 주요 코드 변경

### 1. Moving visual partition 수정

파일: `isaacsim_test/isaacsim/graspable_hand_urdf.py`

변경 내용:

- `_classify_mjcf_visual_link()`가 proximal/distal segment shell만 moving finger link로 분류하게 수정했다.
- passive linkage류 visual은 `r_wrist_interface`에 남긴다.
- `generate_graspable_hand_urdf(... include_finger_shells=True)`를 기본값으로 바꿔 finger shell visual이 기본 포함되게 했다.
- report policy를 `only_proximal_distal_segment_shells_follow_generated_finger_links`로 기록한다.

결과 report 기준:

- `finger1_proximal`: 2 visuals
- `finger1_distal`: 2 visuals
- `r_wrist_interface`: passive/root visual 다수

### 2. Isaac Sim 6.0 nested prim path resolver 추가

파일: `isaacsim_test/isaacsim/robot_arm_hand_from_zip.py`

Isaac Sim 6.0 URDF class API는 import 결과를 아래처럼 깊게 감싼다.

```text
/World/RobotArmHandFromZip/Hand/Geometry/r_wrist_interface/palm/finger1_base/finger1_proximal
/World/RobotArmHandFromZip/Hand/Geometry/r_wrist_interface/palm/finger1_base/finger1_proximal/finger1_distal
```

그래서 기존 `/Hand/finger1_proximal` 식의 고정 path lookup은 실패했다. 이를 해결하기 위해:

- `resolve_connected_hand_link_path(stage, link_name)` 추가
- `resolve_connected_arm_link_path(stage, link_name)` 추가
- finger motion, contact proxy, object reset anchor, screenshot focus에서 resolved path를 사용하게 수정

### 3. Arm-hand fixed joint body0 수정

이전 fixed joint body0가 존재하지 않는 prim을 가리켰다.

```text
/World/RobotArmHandFromZip/Arm/wrist_adapter_hand  # invalid
```

실제 Isaac Sim 6.0 import path는 깊은 nested path였다.

```text
/World/RobotArmHandFromZip/Arm/Geometry/.../wrist_adapter_arm/wrist_adapter_hand
```

수정 후 `_author_connected_usd()`가 `resolve_connected_arm_link_path(stage, "wrist_adapter_hand")`로 실제 rigid body를 찾아 fixed joint body0에 넣는다.

### 4. Runtime articulation 선택 수정

Isaac Sim 6.0에서는 arm과 hand articulation 후보가 별도로 잡히거나, traversal 순서에 따라 arm-only articulation이 먼저 잡힐 수 있었다.

수정:

- `_find_articulation_paths(stage)` 추가
- `select_preferred_runtime_articulation_candidate(candidates)` 추가
- hand joint coverage가 완전한 후보를 우선 선택

최종 검증 run에서는 `/World/RobotArmHandFromZip/Arm/Geometry` articulation이 12 DOF를 로드했고 finger1 motor도 정상 제어됐다.

### 5. Close-up screenshot fallback 개선

Headless viewport close-up은 때때로 카메라 target은 맞지만 PNG가 grid/floor만 보여주는 문제가 있었다. 그래서 close-up evidence용 fallback을 추가했다.

- full-scene capture 보존
- hand/finger 영역 crop 후 upsample
- red square annotation image 생성

최종 evidence는 runtime6의 crop image를 사용했다.

## 추가/수정한 테스트

파일:

- `isaacsim_test/test_graspable_hand_urdf.py`
- `isaacsim_test/test_robot_arm_hand_from_zip.py`

주요 테스트:

- proximal/distal segment shell만 moving link에 붙는지 확인
- passive linkage가 moving finger visual로 들어가지 않는지 확인
- nested Isaac Sim 6.0 hand path resolver 확인
- nested Isaac Sim 6.0 arm wrist path resolver 확인
- articulation candidate selection이 hand/finger DOF를 선호하는지 확인
- finger1 movement report가 distal link motion을 PASS로 기록하는지 확인

검증 명령:

```bash
python3 -m py_compile \
  isaacsim_test/isaacsim/graspable_hand_urdf.py \
  isaacsim_test/isaacsim/robot_arm_hand_from_zip.py \
  isaacsim_test/test_graspable_hand_urdf.py \
  isaacsim_test/test_robot_arm_hand_from_zip.py

python3 -m unittest \
  isaacsim_test.test_graspable_hand_urdf \
  isaacsim_test.test_robot_arm_hand_from_zip.RobotArmHandFromZipTests.test_connected_arm_link_resolver_handles_nested_isaacsim60_tree \
  isaacsim_test.test_robot_arm_hand_from_zip.RobotArmHandFromZipTests.test_connected_hand_link_resolver_handles_nested_isaacsim60_tree \
  isaacsim_test.test_robot_arm_hand_from_zip.RobotArmHandFromZipTests.test_connected_hand_link_resolver_handles_geometry_wrapped_urdf_import \
  isaacsim_test.test_robot_arm_hand_from_zip.RobotArmHandFromZipTests.test_articulation_selection_prefers_hand_finger_chain_over_arm_only_root \
  isaacsim_test.test_robot_arm_hand_from_zip.RobotArmHandFromZipTests.test_prepare_source_artifacts_generates_graspable_hand_urdf \
  isaacsim_test.test_robot_arm_hand_from_zip.RobotArmHandFromZipTests.test_single_finger_preshape_uses_distal_tip_object_reset \
  isaacsim_test.test_robot_arm_hand_from_zip.RobotArmHandFromZipTests.test_finger1_movement_report_marks_distal_motion_pass
```

최종 host test 결과:

```text
Ran 22 tests in 2.477s
OK
```

## Isaac Sim 6.0 최종 검증 run

Run ID:

```text
20260705T104500Z_isaacsim60_finger1_fixed_joint_visual
```

Report:

```text
isaacsim_test/outputs/robot_arm_hand_graspable_20260705T104500Z_isaacsim60_finger1_fixed_joint_visual/robot_arm_hand_connected_report.json
```

핵심 결과:

```text
top_status: PASS_WITH_FALLBACK
runtime_status: PASS
finger_motion_status: PASS
finger1_status: PASS
finger1 motor errors: [0.001999883651733425, 0.0036518788337707164]
finger1 distal_link_translation_delta_m: 0.041296574164599434
```

Finger1 evidence images:

```text
isaacsim_test/artifacts/robot_arm_hand_graspable_20260705T104500Z_isaacsim60_finger1_fixed_joint_visual_runtime6/finger1_two_link_motion.png
isaacsim_test/artifacts/robot_arm_hand_graspable_20260705T104500Z_isaacsim60_finger1_fixed_joint_visual_runtime6/finger1_two_link_motion_red_square.png
isaacsim_test/artifacts/robot_arm_hand_graspable_20260705T104500Z_isaacsim60_finger1_fixed_joint_visual_runtime6/finger1_two_link_motion_zoom_red_square.png
```

## Link / joint mapping 확인

Finger1 motor chain:

```text
finger1_motor1 -> finger1_proximal
finger1_motor2 -> finger1_distal
```

Resolved Isaac Sim 6.0 prims:

```text
/World/RobotArmHandFromZip/Hand/Geometry/r_wrist_interface/palm/finger1_base/finger1_proximal
/World/RobotArmHandFromZip/Hand/Geometry/r_wrist_interface/palm/finger1_base/finger1_proximal/finger1_distal
```

최종 fixed joint 연결:

```text
body0: /World/RobotArmHandFromZip/Arm/Geometry/.../wrist_adapter_arm/wrist_adapter_hand
body1: /World/RobotArmHandFromZip/Hand/Geometry/r_wrist_interface
```

## 아직 남은 의심점

이번 수정은 finger1 visual connection smoke test를 통과시키기 위한 것이다. 아래는 아직 “완료”가 아니다.

1. 전체 top status는 `PASS_WITH_FALLBACK`이다. 원본 MJCF closed-loop hand import는 아직 막혀 있고 generated URDF fallback을 사용한다.
2. passive linkage는 움직이지 않는다. 실제 closed-loop linkage 움직임은 follower kinematics가 필요하다.
3. single-finger preshape는 object/contact load 상황에서 아직 `WARN`이다.
4. lift-retain 검증도 아직 `WARN`이다.
5. screenshot close-up은 headless viewport 특성 때문에 fallback crop을 쓴다. smoke evidence로는 충분하지만 CAD 정밀 alignment evidence는 아니다.
6. 일부 legacy transform diagnostic path는 아직 unresolved path를 같이 기록한다. 핵심 finger1 motion path는 resolved path를 쓰지만 diagnostic cleanup은 후속 작업이다.

## 결론

Finger1은 현재 Isaac Sim 6.0 fallback hand에서 다음 기준을 만족한다.

- 두 motor command가 목표각에 도달한다.
- distal link가 실제 world space에서 움직인다.
- 다른 hand joints는 열린 상태 근처에 유지된다.
- detached curved passive linkage visual 문제는 제거됐다.
- visible finger1 segment shell은 손가락 본체와 붙어 보인다.

즉, finger1 isolated two-link visual/motion smoke test는 통과했다. 다음 단계는 preshape/contact/lift-retain과 true closed-loop visual follower 구현이다.
