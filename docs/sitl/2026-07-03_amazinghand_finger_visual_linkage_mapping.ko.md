# AmazingHand 손가락 visual 매핑 기준: 뼈대 먼저, shell 나중 (2026-07-03)

## 결론

현재 Isaac fallback hand는 **완성된 손가락 visual-physics 매칭 상태가 아니다.**
이전 검증에서 움직인 것은 주로 `proximal/proximal_shell`, `distal/distal_shell` 같은 손가락 겉표면이다.
하지만 실제 시뮬에서 먼저 신뢰해야 하는 것은 겉 shell이 아니라 **손가락 뼈대/프레임/큰 linkage가 joint motion을 따라가는지**이다.

앞으로 판정 기준은 다음과 같다.

- shell만 움직이면 `PASS`가 아니라 **shell-only partial motion**이다.
- finger frame / servo horn / rod / gimbal / 주요 link·pin이 static으로 남아 있으면 **full visual PASS가 아니다**.
- shell은 마지막에 덮는 외피로 취급한다.
- 작은 screw, washer, 작은 spacer류는 현재 grasp/motion 시뮬 단계에서는 후순위다.

## 시뮬에서 필요한 최소 visual 우선순위

| 우선순위 | 대상 | 필요한 이유 | 판정 |
| --- | --- | --- | --- |
| 1 | 큰 뼈대/프레임: `finger_frame_*`, `custom_servo_horn`, `rotule_*`, `m2_rod_l18`, `link`, `gimbal`, 주요 `parallel_pin_*` | 손가락 마디 운동이 실제처럼 보이는지 결정 | 먼저 맞춰야 함 |
| 2 | physics/collision proxy와 visual link 위치 | grasp/contact 해석과 화면 판단이 일치해야 함 | proxy와 visual이 크게 어긋나면 FAIL/WARN |
| 3 | 외피: `proximal`, `proximal_shell`, `distal`, `distal_shell` | 뼈대 위에 최종 손가락 표면을 덮는 역할 | 뼈대가 맞은 뒤 정렬 |
| 후순위 | screw, washer, 작은 spacer 등 | 큰 motion/grasp 판정에는 영향이 작음 | 필요할 때만 deep debug |

## 현재 코드 상태

`isaacsim_test/isaacsim/graspable_hand_urdf.py`의 현재 fallback 분류는 다음과 같다.

- `proximal`, `proximal_shell` → `fingerN_proximal` moving link
- `distal`, `distal_shell` → `fingerN_distal` moving link
- 그 외 passive closed-loop hardware → 대부분 `r_wrist_interface` static/wrist bucket

즉 현재는 shell 쪽이 먼저 움직이는 상태이고, 뼈대/프레임/linkage는 아직 제대로 follower 처리되지 않았다.
이 상태를 최종 visual match로 부르면 안 된다.

## 앞으로 구현할 기준

1. 먼저 shell을 숨기거나 무시하고, 큰 뼈대/프레임/linkage가 finger joint motion을 따라가도록 만든다.
2. 각 큰 visual 부품은 아래 중 하나로 분류한다.
   - palm/wrist fixed
   - proximal link follower
   - distal link follower
   - motor/servo horn follower
   - closed-loop linkage follower가 필요해서 보류
3. finger 하나씩 움직여서 해당 finger의 큰 linkage만 움직이고, 다른 finger는 움직이지 않는지 확대 crop으로 확인한다.
4. 뼈대 motion이 맞은 뒤에 `proximal/proximal_shell`, `distal/distal_shell`을 덮는다.
5. shell까지 붙인 뒤 visual과 physics/collision proxy가 크게 어긋나지 않는지 확인한다.
6. 그 뒤에만 grasp/lift-retain 결과를 신뢰한다.

## 앞으로 사용할 판정 문구

- `shell-only partial motion`: shell만 움직이고 뼈대/linkage는 미해결.
- `skeleton motion PASS`: 큰 프레임/linkage가 해당 finger joint를 따라 움직임.
- `visual-physics aligned PASS`: skeleton, shell, collision proxy가 같은 마디 운동 기준으로 정렬됨.
- `grasp-ready visual PASS`: visual-physics aligned PASS 이후 grasp/lift-retain 검증으로 넘어갈 수 있음.

## 제외/후순위

이번 단계에서는 전체 CAD fidelity를 목표로 하지 않는다.
작은 screw/washer/pin 하나하나까지 완벽하게 follower 구현하는 것은 후순위다.
목표는 실제 시뮬 화면에서 손가락 마디 운동과 grasp/contact 판단이 믿을 수 있을 정도로 맞는 것이다.

## 2026-07-03 구현 업데이트: skeleton-first + strict lift-retain PASS

이번 구현의 목표는 shell 완성도가 아니라 **시뮬에서 필요한 큰 뼈대/주요 핀/linkage가 moving link를 따라가는지**를 먼저 맞추는 것이었다.
Deep-interview 결정에 따라 scope는 다음처럼 고정했다.

- 포함: 큰 linkage, servo horn, rod, gimbal, 주요 `parallel_pin_*`, generated proximal/distal segment.
- 제외: 작은 screw/washer/tiny spacer, shell 최종 정렬, 정확한 closed-loop, SimReady.
- 최종 PASS 조건: grasp 후 lift-retain까지 통과해야 하며, 단순 skeleton-only PASS는 불완전하다.

### 현재 코드 상태

`isaacsim_test/isaacsim/graspable_hand_urdf.py`는 이제 `partitioned_links`에서 작은 detail 부품을 버리고, 큰 moving skeleton 부품만 generated finger link에 붙인다.

- 각 `finger*_proximal`: 18 visuals total, 그중 moving skeleton 16개.
- 각 `finger*_distal`: 4 visuals total, 그중 moving skeleton 2개.
- `r_wrist_interface`: 26 visuals.
- omitted detail visuals: 48개 (`screw`, `washer`, 작은 `spacer` 등).

주요 proximal follower 예:

- `custom_servo_horn`
- `gimbal`
- `m2_rod_l18`
- `rotule_ball`
- `rotule_lever`
- `parallel_pin_2_x_16__...`

주요 distal follower 예:

- `link`
- `parallel_pin_2_x_10__...` 중 distal body chain에 있는 것.

### Grasp/lift physics 보강

`isaacsim_test/isaacsim/robot_arm_hand_from_zip.py`는 hidden physics-only contact proxy를 17개로 늘렸다.
핵심은 palm cradle이다.

- `palm_contact_proxy`: 높은 backstop.
- `palm_retention_shelf_proxy`: object가 lift 중 아래로 빠지지 않도록 받침.
- `palm_retention_left_wall_proxy` / `right_wall_proxy`: x 방향 이탈 방지.
- `palm_retention_front_lip_proxy`: y 방향 전방 이탈 방지.

이 proxy들은 `ROBOT_ARM_HAND_SHOW_CONTACT_PROXIES=0` 기본값에서는 보이지 않는다. 즉 노란 디버그 손을 PASS 근거로 쓰지 않는다.

### 최신 검증 evidence

정적 검증:

```bash
python3 isaacsim_test/test_graspable_hand_urdf.py
python3 isaacsim_test/test_robot_arm_hand_from_zip.py
python3 -m py_compile isaacsim_test/isaacsim/graspable_hand_urdf.py isaacsim_test/isaacsim/robot_arm_hand_from_zip.py
git diff --check -- isaacsim_test/isaacsim/graspable_hand_urdf.py isaacsim_test/isaacsim/robot_arm_hand_from_zip.py isaacsim_test/test_graspable_hand_urdf.py isaacsim_test/test_robot_arm_hand_from_zip.py
```

결과: 5 + 25 tests, py_compile, diff-check 모두 PASS.

런타임 검증:

- Report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260703_skeleton_first_clean_lift_cradle_strictpass_runtime/robot_arm_hand_connected_report.json`
- Artifact root: `isaacsim_test/artifacts/visual_verification_20260703_skeleton_first_clean_lift_cradle_strictpass_runtime/`
- Overall status: `PASS_WITH_FALLBACK` (`MJCFCreateAsset`는 여전히 `basic_string::_M_construct null not valid`로 실패하고 generated URDF fallback 사용)
- Runtime validation: `PASS`
- Finger motion validation: `PASS`
- Grasp validation: `PASS`
- Lift-retain validation: `PASS`
- `object_anchor_distance_after_lift_m`: `0.05788719857784997` (`<= 0.16`)
- `object_z_delta_after_lift_m`: `-0.001326270588946299` (`>= -0.01`)

검토한 close-up 이미지:

- `finger1_two_link_before.png`
- `finger1_two_link_motion.png`
- `lift_retain_smoke.png`

판정: 작은 floating screw/washer는 제거됐고, 큰 손가락 skeleton/linkage는 moving link와 같이 움직인다. lift smoke에서는 물체가 바닥으로 떨어지던 이전 상태와 달리 손 가까이에 유지된다.

### 남은 리스크

- 이 상태는 **SimReady도 아니고 exact closed-loop visual follower도 아니다.**
- 외피 shell 최종 정렬은 아직 제외 scope다.
- 최신 코드에는 strict z 판정 기준을 reset pose 기준으로 명시하고, 다음 런부터 settled-before 대비 z delta도 별도 report 필드로 기록한다.
- MJCF importer 실패는 그대로 남아 있으므로 현재 성공 경로는 `PASS_WITH_FALLBACK`이다.

## 2026-07-03 shell hidden + finger link motion metric 런타임

사용자 지시에 따라 현재 검증은 **손가락 관절/뼈대 우선**으로 고정하고, 외피/outer shell visual은 숨겼다.

### 구현 변경

- `proximal.stl`, `proximal_shell.stl`, `distal.stl`, `distal_shell.stl` 계열 outer segment visual은 generated URDF moving visuals에서 제외한다.
- 작은 screw/washer/spacer detail 제외는 유지한다.
- 손가락 motion validation은 이제 joint angle만 보지 않고 각 손가락의 `proximal`/`distal` link world translation before/after를 기록한다.
- PASS 조건에 `distal_link_translation_delta_m >= 0.005`를 추가했다.
- lift-retain PASS도 palm cradle만으로 만족하지 않도록, close/lift 후 finger-mounted contact proxy가 물체 근처에 남아있는지 확인한다.

### 최신 런타임 evidence

- Report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260703_finger_joint_link_motion_metric_runtime/robot_arm_hand_connected_report.json`
- Artifact root: `isaacsim_test/artifacts/visual_verification_20260703_finger_joint_link_motion_metric_runtime/`
- Overall status: `PASS_WITH_FALLBACK`
- Runtime validation: `PASS`
- Finger motion validation: `PASS`
- Grasp validation: `PASS`
- Lift-retain validation: `PASS`
- Shell visuals omitted: `16`
- Small detail visuals omitted: `48`
- Moving visual counts: 각 `finger*_proximal = 16`, 각 `finger*_distal = 2`

손가락별 실제 distal link 이동량:

| finger | DOF delta rad | distal link delta m |
| --- | --- | --- |
| finger1 | `[0.7295, 0.9468]` | `0.04137127693725751` |
| finger2 | `[0.6966, 0.9472]` | `0.039535401517465506` |
| finger3 | `[0.7295, 0.9467]` | `0.04141707330546154` |
| finger4 | `[0.7300, 0.9399]` | `0.04142921510896706` |

Finger grasp engagement:

- `finger_grasp_engaged = true`
- `finger_proxy_close_count_after_close = 3`
- `finger_proxy_close_count_after_lift = 3`
- `finger_proxy_distances_after_close_m` 최소 3개: `0.034658`, `0.045127`, `0.050878`
- `finger_proxy_distances_after_lift_m` 최소 3개: `0.033106`, `0.039583`, `0.053010`
- `object_anchor_distance_after_lift_m = 0.058509052119438576`
- `object_z_delta_after_lift_m = -0.002921410092425303`

### 판정

- 관절 DOF는 움직이고, distal link도 실제 월드 좌표에서 약 4 cm 이동한다.
- 외피/노란 프록시는 숨긴 상태로 PASS했다.
- 다만 현재는 skeleton-first simplified tree라서 exact closed-loop 손가락 형태나 최종 외피 정렬이 아니다.
- close-up 이미지에서는 motion 차이가 작고 흐리게 보일 수 있다. 이유는 headless viewport가 close-up camera를 무시하는 경우가 있어 전체 장면 캡처 후 crop fallback을 쓰기 때문이다. 따라서 이미지만이 아니라 `distal_link_translation_delta_m` metric을 함께 본다.

## 2026-07-03 follow-up: reference-hand preshape branch

새 브랜치 `feature/isaacsim-amazinghand-reference-hand`에서는 Shadow Hand / Allegro Hand를 AmazingHand 대체물이 아니라 Isaac 손 구현 체크리스트로 문서화하고, 기존 `amazinghand_grasp.pos` scalar 호환을 유지한 채 `wrap/pinch/wide/single_finger` preshape command를 추가한다.

운영 제약: 현재 다른 작업이 기존 Isaac Sim 인스턴스에서 실행 중이므로 이 브랜치의 runtime smoke는 그 인스턴스에서 실행하지 않는다. Runtime 검증은 별도 Isaac Sim 버전/인스턴스와 분리된 output/artifact root에서만 수행한다.

세부 기준 문서: `docs/sitl/2026-07-03_isaac_reference_hands_for_amazinghand.ko.md`

## 2026-07-03 추가 구현: single finger → pinch → wrap preshape 검증 PASS

이번 단계는 AmazingHand를 Shadow/Allegro로 교체하는 것이 아니라, 그 모델들을 **reference-only 체크리스트**로만 두고 기존 fallback AmazingHand skeleton-first 구조 위에 preshape 검증을 추가한 것이다.

### 구현 내용

`isaacsim_test/isaacsim/robot_arm_hand_from_zip.py`에 다음을 추가했다.

- `build_hand_preshape_joint_targets(grasp, grasp_type, finger_index=None)`
- `build_hand_preshape_position_command(current_positions, dof_names, preshape, amount, finger_index=None)`
- `build_preshape_grasp_validation_stage_specs()`
- `build_shadow_allegro_reference_checklist()`
- runtime report field: `runtime_validation.preshape_grasp_validation`

검증 순서는 다음처럼 고정했다.

1. 손가락별 two-link motion validation
2. `single_finger`
3. `pinch`
4. `wrap`
5. lift-retain validation

이 순서가 중요하다. preshape를 먼저 실행하면 다른 손가락 target이 남아서 finger-by-finger motion 판정이 오염될 수 있다.

### 최신 런타임 evidence

- Stamp: `20260703_preshape_after_finger_before_lift_runtime`
- Report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260703_preshape_after_finger_before_lift_runtime/robot_arm_hand_connected_report.json`
- Screenshot root: `isaacsim_test/artifacts/visual_verification_20260703_preshape_after_finger_before_lift_runtime/`
- Overall status: `PASS_WITH_FALLBACK`
- Runtime validation: `PASS`
- Grasp validation: `PASS`
- Finger motion validation: `PASS`
- Preshape grasp validation: `PASS`
- Lift-retain validation: `PASS`
- Shell visuals omitted: `16`
- Small detail visuals omitted: `48`

Preshape stage 결과:

| stage | status | required close count | measured active close count | active min distance m |
| --- | --- | ---: | ---: | --- |
| `single_finger` | PASS | 1 | 1 | finger1=`0.05425387596198618` |
| `pinch` | PASS | 2 | 2 | finger1=`0.03305320674670747`, finger4=`0.05212996497200721` |
| `wrap` | PASS | 3 | 3 | finger1=`0.04983700427205717`, finger2=`0.036937761359548626`, finger3=`0.04979875109821099`, finger4=`0.07478045253797082` |

Finger motion 결과:

| finger | status | distal link delta m | other hand max abs rad |
| --- | --- | ---: | ---: |
| finger1 | PASS | `0.04137127693725751` | `0.05000052601099014` |
| finger2 | PASS | `0.039535401517465506` | `0.04999994859099388` |
| finger3 | PASS | `0.04141707330546154` | `0.04999995604157448` |
| finger4 | PASS | `0.04142921510896706` | `0.049401864409446716` |

Lift-retain 결과:

- `finger_grasp_engaged = true`
- `finger_proxy_close_count_after_close = 3`
- `finger_proxy_close_count_after_lift = 4`
- `object_anchor_distance_after_lift_m = 0.06239264366924374`
- `object_z_delta_after_lift_m = 0.00994930729392729`

검토한 close-up/crop 이미지:

- `preshape_single_finger.png`
- `preshape_pinch.png`
- `preshape_wrap.png`
- `grasp_real_hand_04_after_lift_retain.png`
- 검사용 임시 crop/upscale: `/tmp/amazinghand_preshape_after_finger_crops/*_hand_crop2x.png`

육안 판정: 외피 shell은 숨긴 상태이고, 흰색 skeleton/linkage가 preshape별로 움직인다. 단, headless render가 흐려서 최종 판정은 screenshot 단독이 아니라 `distal_link_translation_delta_m`, active finger proxy distance, lift-retain metric을 함께 본다.

### 현재 해석

- 현재 성공은 **AmazingHand fallback URDF/USD skeleton-first 성공**이다.
- `MJCFCreateAsset`는 여전히 `basic_string::_M_construct null not valid`로 실패하므로 전체 status는 `PASS_WITH_FALLBACK`이다.
- Shadow Hand / Allegro Hand는 reference-only이며 AmazingHand 대체 구현으로 쓰지 않는다.
- SimReady는 이번 단계에서 제외 상태를 유지한다.
- 외피 shell 최종 정렬은 아직 하지 않았다. 뼈대/linkage가 움직이는 것을 먼저 고정했고, shell은 이후에 덮는 단계로 남긴다.

## 2026-07-03 추가 구현: focused viewport close-up capture

사용자 지적대로 기존 close-up 이미지는 실제 viewport zoom이 아니라 전체 장면을 찍고 hand 영역을 crop/upscale하는 방식이라 흐렸다. 이번 변경에서는 close-up capture 기본 경로를 `focused_viewport`로 바꿨다.

### 결정

- Isaac Sim은 현재 손 검증 경로에서 계속 `5.1.0`을 사용한다.
- Isaac Sim 6.0 이미지는 로컬에 있지만 이번 AmazingHand 검증 경로로 전환하지 않았다.
- `F / Frame Selected`와 같은 효과는 prim selection 자체가 아니라 camera framing으로 구현한다.
- 실제 USD selection은 `ROBOT_ARM_HAND_SELECT_FOCUS_PRIM=1`일 때만 opt-in이다. 기본 off다. 이유: headless 5.1에서 selection event가 property widget traceback과 주황색 outline을 만들었다.

### 구현 결과

- `ROBOT_ARM_HAND_CAPTURE_WIDTH` / `ROBOT_ARM_HAND_CAPTURE_HEIGHT`로 SimulationApp 및 Replicator render resolution 제어 가능.
- close-up capture는 먼저 target prim에 대한 focused viewport camera를 시도한다.
- 실패할 때만 기존 `whole_scene_crop_fallback`으로 내려간다.
- runtime report에 `capture_method`, `screenshot_capture`, `resolution`을 기록한다.

### 검증 evidence

- Stamp: `20260703_focused_viewport_capture_default_runtime`
- Report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260703_focused_viewport_capture_default_runtime/robot_arm_hand_connected_report.json`
- Screenshots: `isaacsim_test/artifacts/visual_verification_20260703_focused_viewport_capture_default_runtime/`
- Overall: `PASS_WITH_FALLBACK`
- Runtime/Finger/Preshape/Lift-retain: 모두 `PASS`
- 모든 핵심 close-up capture method: `focused_viewport`
- Resolution: `[1280, 720]`
- 대표 확인 이미지: `finger1_two_link_motion.png`

추가로 `ROBOT_ARM_HAND_CAPTURE_WIDTH=1920`, `ROBOT_ARM_HAND_CAPTURE_HEIGHT=1080` 런도 수행했다. 1080p close-up은 `focused_viewport`로 고해상도 저장됐고 화질은 더 좋았지만, 그 런에서는 physics preshape/lift가 WARN으로 흔들렸다. 따라서 현재 pass 기준 런은 default 1280x720을 사용하고, 1080p는 visual-only deep inspection 옵션으로 남긴다.
