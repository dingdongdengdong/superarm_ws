# AmazingHand Isaac Sim Memory

Updated: 2026-07-02

## Goal

Build an Isaac Sim friendly robot arm + AmazingHand package that can physically grasp small trash-like objects whose shape is mostly intact.

## Source Package

- Source zip: `robot_arm_hand_package.zip`
- Original hand source: `hand_mjcf/robot.xml`
- Original visual assets: `hand_mjcf/assets/*.stl`

## Core Decision

Do not use the original AmazingHand MJCF as the runtime physics articulation in Isaac Sim. The original MJCF is a closed-loop MuJoCo mechanism with equality/connect constraints. Isaac import has been brittle for this model, including MJCF import failure before fallback.

Use a separate Isaac-oriented URDF tree articulation for physics:

- root link: `r_wrist_interface`
- four fingers: index, middle, ring, thumb
- excluded human finger: pinky
- actuated hand joints: `finger1_motor1` through `finger4_motor2`
- collision: primitive boxes for stable contact
- equality constraints: none

## Visual Policy

Current direction from the user: **do not use SimReady for this hand now**. The priority is matching the real hand visual parts to the moving physics links, then running grasp validation only after visual/physics alignment is acceptable.

## Hard Rule: Hand Debug Camera / Crop Memory

When debugging AmazingHand visual/physics alignment, **never judge from the whole-arm screenshot or a generic viewport crop**.

Required hand screenshot protocol:

- Use close-up crops focused on the actual hand/finger, not the arm and not the floor grid.
- For finger validation, move **one finger at a time** and reset every other hand joint open before that finger's test.
- Capture and inspect both:
  - `fingerN_two_link_before.png`
  - `fingerN_two_link_motion.png`
- The camera target must be the generated finger chain:
  - focus root: `/World/RobotArmHandFromZip/Hand/fingerN_proximal`
  - include sibling: `/World/RobotArmHandFromZip/Hand/fingerN_distal`
  - if USD render bounds are missing/stale, fall back to the proximal/distal link world transforms and their contact-proxy world transforms.
- A screenshot that shows only the floor/grid is **invalid evidence**, even if the numeric joint validation says `PASS`.
- Yellow contact proxies are collision/debug helpers only. They are not the real hand visual and must not be used as proof that the real hand visual is correct.
- The real visual parts must visibly remain attached to the moving proximal/distal physics links. Floating/separated phalanges are a failure, not an acceptable normal state.

The visual source of truth is the original MJCF visual geometry list. The parser reads body and geom `pos`/`quat` transforms and reuses the existing STL files.

Default visual mode as of 2026-07-02:

- `partitioned_links`
- attachment mode: `mjcf_visuals_partitioned_to_tree_links`
- only true finger segment meshes follow moving links:
  - `proximal.stl` and `proximal_shell.stl` -> `finger*_proximal`
  - `distal.stl` and `distal_shell.stl` -> `finger*_distal`
- passive closed-loop linkage/body/servo/rod/screw/ball hardware stays attached to `r_wrist_interface` for now
- expected MJCF visual geom count: `162`
- expected moving segment count: 2 visuals per proximal link and 2 visuals per distal link
- expected wrist/static count: 146 visuals
- expected missing MJCF visual meshes: `[]`

Reason:

- The original hand visual assembly is a closed-loop MJCF mechanism with passive linkage behavior.
- The simplified Isaac hand physics is a serial tree (`palm -> proximal -> distal`) for stable articulation and contact.
- Earlier naive partitioning attached too many passive linkage pieces to proximal links, making the hand look torn/wrong during motion.
- The current safer partition moves the real external proximal/distal finger segment visuals with the physics links, and leaves passive linkage details static until a real follower-kinematics pass is implemented.

Legacy fallback visual mode:

- `static_shell`
- attachment mode: `mjcf_static_visual_shell`
- one fixed `amazinghand_visual_shell` under `r_wrist_interface`
- use only as a fallback when the moving visual partition is worse than static; it is no longer the default.

## Current Limitation

The real external finger segment visuals now curl with the generated two-link physics fingers. This is acceptable for near-term visual/physics matching.

Remaining mismatch: passive closed-loop linkage hardware from the original MJCF is still wrist/static, not animated by follower joints. To fully match the real hand, implement visual follower joints/transforms derived from the MJCF passive linkage pivots, or build a tree-compatible USD/URDF visual decomposition for those passive parts.

## Finger Physics Contract

Each generated finger is a two-link tree chain:

- `finger*_motor1`: `palm -> finger*_proximal`
- `finger*_motor2`: `finger*_proximal -> finger*_distal`

The runtime validation now includes `finger_motion_validation`. It commands each finger independently with two motor targets and records Isaac Articulation joint-position readback.

## Validation Evidence

Latest validation after the static-shell default fix:

- `python3 isaacsim_test/test_graspable_hand_urdf.py`: passed
- `python3 isaacsim_test/test_robot_arm_hand_from_zip.py`: passed
- Isaac report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_visualfix_static/robot_arm_hand_connected_report.json`
- Isaac status: `PASS_WITH_FALLBACK`
- Runtime validation: `PASS`
- Generated hand visual mode: `static_shell`
- Generated hand attachment mode: `mjcf_static_visual_shell`
- MJCF visual geom count: `162`
- Missing MJCF visual meshes: `[]`
- Isaac artifact reviewed: `isaacsim_test/artifacts/robot_arm_hand_graspable_20260702_visualfix_static/contact_sheet.png`

The reviewed contact sheet shows the AmazingHand assembly stable at the arm tip during pose changes. This fixes the visible tearing caused by approximate partitioned visual links. The physical grasping surface is still the primitive collision finger tree.

Latest two-link finger physics validation:

- Isaac output root: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_finger2link`
- Isaac artifact root: `isaacsim_test/artifacts/robot_arm_hand_graspable_20260702_finger2link`
- Overall status: `PASS_WITH_FALLBACK`
- Runtime validation: `PASS`
- Finger motion validation: `PASS`
- finger1 achieved `[0.7791, 0.9622]` for target `[0.78, 0.96]`
- finger2 achieved `[0.7785, 0.9614]` for target `[0.78, 0.96]`
- finger3 achieved `[0.7785, 0.9614]` for target `[0.78, 0.96]`
- finger4 achieved `[0.7795, 0.9366]` for target `[0.78, 0.96]`

Latest contact/lift-retain state:

- Isaac output root: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_objectreset`
- Runtime validation: `PASS`
- Contact tuning: `PASS`
- Authored hidden hand collision proxies: `13`
- Bound high-friction collision proxies: `13`
- Missing hand link paths for proxy authoring: `[]`
- Finger motion validation: `PASS`
- Grasp smoke: `PASS`
- Lift-retain validation: `WARN`
- Object reset before close works; close starts with object at `[0.005, 0.02986, 0.642278]`.
- Close brings the object near the hand (`0.1381 m` from hand root), but it still drops during the retain/lift phase.
- Next physics work: tune palm/finger proxy geometry, object spawn pose, close targets, drive force/damping, and solver/contact offsets for sustained small-trash grasping.

## Latest moving-visual verification (2026-07-02)

User direction: no SimReady now. Focus on real hand visuals matching moving physics.

Important correction: the earlier same-body segment hardware mapping (`6/5` moving visuals per finger) looked wrong in close-up: phalanges/hardware floated and separated. That evidence must not be treated as a success. The current intended mapping keeps only the true proximal/distal shell meshes on the moving links (`2/2`) and leaves passive linkage hardware static until follower kinematics exist.

Code/runtime state verified:

- Generated hand default visual mode: `partitioned_links`
- Visual attachment mode: `mjcf_visuals_partitioned_to_tree_links`
- MJCF visual geom count: `162`
- Moving visual mapping:
  - each `finger*_proximal`: 2 visuals
  - each `finger*_distal`: 2 visuals
  - `r_wrist_interface`: 146 visuals
- Yellow contact proxies are debug-only and hidden unless explicitly enabled.
- Real finger segment visuals must be proven with valid close-up crops before claiming they move correctly.
- Passive MJCF linkage details are still wrist-attached/static; next visual work is follower kinematics, not SimReady.

Latest tree-aligned run caveat:

- Output root: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_tree_aligned_segments`
- Artifact root: `isaacsim_test/artifacts/robot_arm_hand_graspable_20260702_tree_aligned_segments`
- Numeric finger motion validation: `PASS`
- But `finger*_two_link_*.png` and `grasp_real_hand_*.png` close-up captures showed only floor/grid, so those screenshots are **invalid** for visual verification.
- Next implementation step: fix close-up camera targeting/capture first, rerun finger-by-finger validation, then inspect the new zoomed images before making any visual correctness claim.

Evidence paths:

- Output root: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_moving_visual_segment_hw`
- Artifact root: `isaacsim_test/artifacts/robot_arm_hand_graspable_20260702_moving_visual_segment_hw`
- Hand report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_moving_visual_segment_hw/amazinghand_graspable_report.json`
- Runtime report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_moving_visual_segment_hw/robot_arm_hand_connected_report.json`
- Reviewed screenshots after same-body segment hardware mapping:
  - `finger1_two_link_motion.png`
  - `finger2_two_link_motion.png`
  - `finger3_two_link_motion.png`
  - `finger4_two_link_motion.png`
  - `grasp_real_hand_02_half_close.png`
  - `grasp_real_hand_03_full_close_before_lift.png`
  - `grasp_real_hand_04_after_lift_retain.png`

Runtime result:

- Overall Isaac status: `PASS_WITH_FALLBACK` because MJCF import failed and the generated URDF fallback was used.
- Runtime validation: `PASS`
- Finger motion validation: `PASS`
- Grasp smoke: `PASS`
- Lift-retain validation: `WARN`

Next technical step:

1. Keep `partitioned_links` as default.
2. Do not re-enable static shell as default.
3. Do not use SimReady yet.
4. Build/author visual follower transforms for passive MJCF linkage pieces if exact visual match is required.
5. After visual/physics match is good enough, tune contact/object placement/drive/solver for sustained grasp retention.

## SimReady Physics Scene Root Cause (2026-07-02)

User symptom: the SimReady `echo_full` file rendered correctly in Isaac Sim but the parts were not jointed/connected in physics.

Root cause: `pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd` passed the `Prop-Robotics-Neutral` profile as a CAD/prop-style asset. FET004 promoted CAD mesh components to rigid bodies, but the repair report had no authored joint prims, so importing that final SimReady USD directly creates loose rigid bodies rather than a robot articulation.

Fix applied: copied the already-authored Isaac physics articulation from the top-level `echo_full/sitl` mirror into the user-requested output root:

- `isaacsim_test/outputs/simready/echo_full/sitl/echo_full_lerobot_articulation.usda`
- `isaacsim_test/outputs/simready/echo_full/sitl/echo_full_lerobot_articulation_report.json`
- old manifest-only file backed up as `echo_full_lerobot_articulation.usda.manifest_only.bak`

Validation evidence:

- `python3 isaacsim_test/test_echo_full_requested_articulation.py`: passed
- `python3 -m unittest isaacsim_test.test_echo_full_requested_articulation isaacsim_test.test_echo_full_contact_tuning isaacsim_test.test_echo_full_joint_pose_fix`: passed, 6 tests
- OpenUSD inspection of the requested-path articulation USD: `/echo_full` has `PhysicsArticulationRootAPI`, 29 `UsdPhysics.Joint` prims, and 50 `PhysicsCollisionAPI` prims.

Usage decision: load `sitl/echo_full_lerobot_articulation.usda` for physics scenes; treat the final SimReady CAD USD as visual/provenance context unless a future workflow authors a robot-specific articulation profile.

## Arm-Only Physics Package (2026-07-02)

User approved discarding the hand side for this path. Created a dated arm-only package at:

- `isaacsim_test/outputs/simready/echo_full/20260702_arm_only/`

Artifacts:

- `arm_only.urdf`: Roboto V2 right-arm URDF only.
- `arm_only_physics.usda`: OpenUSD physics summary/articulation stage.
- `load_arm_only.env`: Isaac loader environment variables.
- `arm_only_report.json` and `arm_only_report.md`: provenance and validation.

Scope:

- Excluded AmazingHand, all finger links/joints, and wrist-interface hand attachment.
- Kept six arm/base links: `torso_link`, `right_arm_pitch_link`, `right_arm_roll_link`, `right_arm_yaw_link`, `right_elbow_pitch_link`, `right_elbow_yaw_link`.
- Kept five controlled arm joints: `right_arm_pitch_joint`, `right_arm_roll_joint`, `right_arm_yaw_joint`, `right_elbow_pitch_joint`, `right_elbow_yaw_joint`.

Validation:

- `python3 isaacsim_test/test_echo_full_arm_only_package.py`: passed.
- `python3 -m unittest isaacsim_test.test_echo_full_arm_only_package isaacsim_test.test_rpo_arm_contract isaacsim_test.test_echo_full_requested_articulation`: passed, 9 tests.
- Dated artifact inspection found no hand/finger/AmazingHand name leaks; USD default prim and articulation root are `/echo_full_arm_only`; USD has 5 revolute joints and 5 collision API prims.

Usage decision: for arm-only connected physics, source `load_arm_only.env` and load/import the `arm_only.urdf` (or inspect `arm_only_physics.usda`). Do not use the final CAD SimReady USD directly as the robot articulation.

## 2026-07-03 Resolved-visual finger motion validation

Latest verified run:

- Runtime report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260703_resolved_visuals_crop_v5_runtime/robot_arm_hand_connected_report.json`
- Finger close-up sheet: `isaacsim_test/artifacts/visual_verification_20260703/resolved_visuals_crop_v5_runtime/finger_before_motion_sheet.png`
- Raw close-up crops: `isaacsim_test/artifacts/robot_arm_hand_graspable_20260703_resolved_visuals_crop_v5_runtime/finger{1..4}_two_link_{before,motion}.png`

Result:

- `runtime_validation.status = PASS`
- `finger_motion_validation.status = PASS`
- Each finger was tested independently with the other hand joints reset open.
- Finger joint deltas were approximately `0.73 rad` and `0.94 rad` for each tested two-link finger command.
- Other hand joint max abs position stayed around `0.05 rad`, so the one-finger command did not close the other fingers.
- Visual sheet shows real white hand/finger geometry, not yellow contact proxies. Finger2/finger3/finger4 visibly fold in before→motion crops; finger1 also passes numerically but is harder to see from the current crop angle due overlap.

Connected USD visual composition check:

- `isaacsim_test/outputs/robot_arm_hand_from_zip/robot_arm_hand_connected.usd` composes `162` hand `Mesh` prims.
- Each `fingerN_proximal/resolved_visuals` has `2` mesh prims.
- Each `fingerN_distal/resolved_visuals` has `2` mesh prims.
- The root cause of previous invisible/static real hand visual evidence was that importer-local `/visuals` libraries were not composed under the final connected hand reference. The fix is to add explicit external references under per-link `resolved_visuals` children pointing back to `configuration/hand_base.usd@</visuals/<link>>`.

Known remaining issue:

- `lift_retain_validation.status = WARN`: object moved/lifted but was not retained near the hand. Do not claim grasp success yet. Next step after visual/physics alignment is contact/object placement/lift-retention tuning.
- A palm visual reference warning remains for `</visuals/palm>`, but this is not the finger segment visual blocker.

## Official Onshape Live Export Validation (2026-07-03)

- Live Onshape API export succeeded for the official AmazingHand right hand; credentials were supplied at runtime only and are not stored in artifacts.
- Output root: `isaacsim_test/outputs/amazinghand_onshape_live_20260702T155051Z`
- Exported source audit: `PASS_WITH_WARNINGS` with 77 links, 76 joints, 46 assets, and all 8 required motor joints present (`finger1_motor1` … `finger4_motor2`).
- Stage A original live URDF Isaac import verdict: `FAIL_UNUSABLE_FOR_ORIGINAL_HAND_ARTICULATION` because Isaac logged unsupported `ball` joint parsing and the imported USD did not retain the 8 motor joints.
- Stage B reduced/tree articulation verdict: `PASS_WITH_WARNINGS_KINEMATIC_FALLBACK`; the reduced model preserves the 8 motor DOF names and one-finger-at-a-time targets/readbacks reach `[0.75, 0.95]` for fingers 1–4.
- Valid visual evidence: `isaacsim_test/outputs/amazinghand_onshape_live_20260702T155051Z/stage_b_rep_debug_big_contact_sheet.png` and `isaacsim_test/outputs/amazinghand_onshape_live_20260702T155051Z/stage_b_rep_debug_big_image_diff_stats.json` show nonblank close-up debug/reduced motion with non-null diffs for all four fingers.
- Limitation: Stage B images are Replicator debug geometry driven by the reduced interface, not exact Onshape mesh rendering; dynamic Isaac Articulation handle remained unavailable in the headless validation.
- Exclusions honored: standalone right hand only; no left hand; no Roboto/arm integration.
- Final report: `isaacsim_test/outputs/amazinghand_onshape_live_20260702T155051Z/final_report.md`


## 2026-07-03 grasp/lift-retain debug update

- Real AmazingHand STL visuals are now referenced under moving generated hand links (`resolved_visuals`) and finger-by-finger close-up validation passes. Keep using close-up/cropped hand screenshots for visual debugging; do not judge from whole-scene shots only.
- The yellow/orange contact proxies are physics-only debug collision geometry, not the real hand visual. They should stay hidden unless `ROBOT_ARM_HAND_SHOW_CONTACT_PROXIES=1`.
- Lift-retain metric now uses `/World/RobotArmHandFromZip/Hand/palm` as the grasp anchor, not the top-level `/World/RobotArmHandFromZip/Hand` Xform. The top-level hand Xform can remain static while the palm rigid body moves.
- `20260703_lift_palm_backstop_runtime`: arm lift command `[0.0, -0.32, 0.42, 0.18]` made the palm fall to about z=0.03 and ejected the object about 9.38 m from the palm. Root cause: unsafe lift pose/abrupt target, not a valid grasp failure signal by itself.
- Implemented ramped lift-retain command toward stable fold pose `[0.15, 0.2, -0.35, 0.45]` while keeping hand close targets active. `20260703_lift_ramped_fold_runtime2` kept arm stable (after-lift arm joints about `[0.126, 0.173, -0.359, 0.374]`) but the object still fell/slipped: after-lift palm/object distance about 0.740 m, object z delta about -0.633 m. Lift-retain remains WARN, not PASS.
- Failed hypothesis: increasing fallback hand URDF drive strength/damping to 120/12 produced identical lift-retain result, so drive strength is not the observed blocker. This change was reverted.
- Failed hypothesis: raising/tallening the palm backstop z-overlap worsened the result (`20260703_lift_ramped_palm_z_runtime`, after-lift distance about 1.496 m). This change was reverted.
- Current standing result: motion/visual alignment PASS; grasp smoke PASS; lift-retain still WARN. Do not claim grasp success. Next useful debug should inspect/tune collision proxy geometry and object placement as a grasp pair, or replace the simplified generated fallback hand with a better kinematic/collision mapping from the official CAD/MJCF; avoid SimReady for now per user direction.


## 2026-07-03 Finger visual/linkage mapping rule

User correction accepted and standing rule updated: use **skeleton/linkage first, shell last**. Current fallback motion of `proximal/proximal_shell` and `distal/distal_shell` is only `shell-only partial motion`, not visual PASS. Full visual trust requires the larger finger frame/linkage silhouette (`finger_frame_*`, `custom_servo_horn`, `rotule_*`, `m2_rod_l18`, `link`, `gimbal`, major `parallel_pin_*`) to follow the correct finger joint/follower first; small screws/washers/spacers are lower priority.

Detailed Korean rule: `docs/sitl/2026-07-03_amazinghand_finger_visual_linkage_mapping.ko.md`

## Latest Skeleton-First + Strict Lift-Retain State (2026-07-03)

User-approved scope after deep interview:

- Use skeleton/linkage first; shell final alignment later.
- Include major linkage and major pins.
- Exclude small screws/washers/tiny spacers, exact closed-loop, shell final alignment, SimReady.
- Final pass requires grasp + strict lift-retain, not skeleton-only motion.

Implemented:

- `graspable_hand_urdf.py` now attaches major proximal/distal follower visuals to generated moving links and omits 48 small detail visuals.
- Moving skeleton counts: each proximal has 16 major skeleton visuals, each distal has 2.
- Hidden palm cradle contact proxies added for lift-retain: shelf, backstop, left/right walls, front lip. Total hidden contact proxies: 17.
- Lift-retain arm target is `[-0.25, 0.15, 0.3, -0.2]`.
- Reset pose is used as the strict z reference; settled-before z delta is also exposed in newer reports for debugging.

Latest verified runtime evidence:

- Report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260703_skeleton_first_clean_lift_cradle_strictpass_runtime/robot_arm_hand_connected_report.json`
- Screenshots: `isaacsim_test/artifacts/visual_verification_20260703_skeleton_first_clean_lift_cradle_strictpass_runtime/`
- Overall: `PASS_WITH_FALLBACK` because MJCF importer still fails and generated URDF fallback is used.
- Runtime: `PASS`
- Finger motion: `PASS`
- Grasp: `PASS`
- Lift-retain: `PASS`
- `object_anchor_distance_after_lift_m = 0.05788719857784997`
- `object_z_delta_after_lift_m = -0.001326270588946299`

Close-up review rule remains mandatory:

- Use `fingerN_two_link_before.png` and `fingerN_two_link_motion.png` crops.
- Do not judge from whole-arm screenshots.
- Yellow contact proxies are not real visual evidence; they are hidden by default.

## Latest Shell-Hidden Finger Joint + Grasp State (2026-07-03)

Current focus is finger joints/skeleton only; outer shell visual is hidden.

Latest verified runtime:

- Report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260703_finger_joint_link_motion_metric_runtime/robot_arm_hand_connected_report.json`
- Screenshots: `isaacsim_test/artifacts/visual_verification_20260703_finger_joint_link_motion_metric_runtime/`
- Overall: `PASS_WITH_FALLBACK` because Isaac MJCF importer still fails with `basic_string::_M_construct null not valid`; generated URDF fallback is used.
- Runtime/Finger motion/Grasp/Lift-retain: all `PASS`.
- Outer shell visuals omitted: `16`; small detail visuals omitted: `48`.
- Moving visuals are now skeleton-first: each proximal link has `16` major visuals; each distal has `2`.
- New finger validation records actual link motion, not only DOF readback:
  - finger1 distal delta: `0.04137127693725751 m`
  - finger2 distal delta: `0.039535401517465506 m`
  - finger3 distal delta: `0.04141707330546154 m`
  - finger4 distal delta: `0.04142921510896706 m`
- Finger grasp engagement is required for lift-retain PASS:
  - close count: `3`, lift count: `3` within `0.055 m`
  - closest close distances: `0.034658`, `0.045127`, `0.050878 m`
  - closest lift distances: `0.033106`, `0.039583`, `0.053010 m`
  - object anchor distance after lift: `0.058509052119438576 m`

Standing caveat: visual close-up crops still look subtle/blurred because the headless viewport often ignores exact close-up camera and the code crops from whole-scene captures. Trust the link-motion metric plus screenshots together. This is not exact closed-loop kinematics, not final shell alignment, and not SimReady.

## Latest skeleton-first preshape validation (2026-07-03)

Current branch implements AmazingHand fallback URDF/USD only. Shadow Hand and Allegro Hand are reference-only checklists, not replacements. SimReady remains out of scope for the hand.

Latest runtime:

- Report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260703_preshape_after_finger_before_lift_runtime/robot_arm_hand_connected_report.json`
- Screenshots: `isaacsim_test/artifacts/visual_verification_20260703_preshape_after_finger_before_lift_runtime/`
- Overall: `PASS_WITH_FALLBACK`
- Runtime: `PASS`
- Finger motion: `PASS`
- Preshape grasp validation: `PASS`
- Lift-retain: `PASS`
- Shell visuals omitted: `16`
- Small detail visuals omitted: `48`

Preshape stages now run after finger-by-finger motion validation and before lift-retain:

- `single_finger`: PASS, active finger1 close count 1/1, min distance `0.05425387596198618 m`
- `pinch`: PASS, active finger1+finger4 close count 2/2, min distances `0.03305320674670747 m`, `0.05212996497200721 m`
- `wrap`: PASS, close count 3/3, active min distances around `0.0369–0.0748 m`

Lift-retain latest: `finger_grasp_engaged=true`, close/lift proxy counts `3/4`, anchor distance `0.06239264366924374 m`, z delta `0.00994930729392729 m`.

Important caveat: MJCF importer still fails with `basic_string::_M_construct null not valid`; Isaac runtime uses generated fallback URDF/USD. The real outer shell remains hidden for this stage. Do not claim final visual-physics shell alignment yet.

## Latest focused viewport screenshot rule (2026-07-03)

Current AmazingHand validation remains on Isaac Sim 5.1.0. The 6.0 image exists locally but is not used for this hand path.

Close-up screenshots no longer rely on whole-scene crop/upscale by default. The runtime now frames the focus prim directly with the viewport camera and records capture metadata:

- `capture_method`: expected `focused_viewport` for finger/preshape/grasp close-ups
- `resolution`: from `ROBOT_ARM_HAND_CAPTURE_WIDTH/HEIGHT`, default `[1280, 720]`
- `whole_scene_crop_fallback`: fallback only if focused capture fails

Do not enable USD selection by default. `ROBOT_ARM_HAND_SELECT_FOCUS_PRIM=1` is opt-in only because headless Isaac Sim 5.1 selection events can trigger property-window tracebacks and add orange selection outlines to screenshots.

Latest verified focused capture runtime:

- Report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260703_focused_viewport_capture_default_runtime/robot_arm_hand_connected_report.json`
- Screenshots: `isaacsim_test/artifacts/visual_verification_20260703_focused_viewport_capture_default_runtime/`
- Overall: `PASS_WITH_FALLBACK`
- Runtime/Finger/Preshape/Lift-retain: all `PASS`
- All key close-ups: `focused_viewport`, resolution `[1280, 720]`

A 1920x1080 visual run produced sharper focused screenshots but physics validation became WARN, so use 1080p only for visual inspection unless retuned.

## Latest implemented-only debug view and finger-base frames (2026-07-03)

For hand debugging, fake/confusing CAD visuals can now be hidden with:

```bash
ROBOT_ARM_HAND_VISUAL_MODE=implemented_only
ROBOT_ARM_HAND_INCLUDE_FINGER_SHELLS=0
ROBOT_ARM_HAND_SHOW_CONTACT_PROXIES=0
```

This mode shows only generated implemented geometry:

- 13 collision primitive debug boxes: palm, proximal, distal, tip pads.
- 4 fixed finger-base/motor-frame debug boxes.
- No MJCF CAD mesh visuals.
- No outer shell visuals.
- No yellow contact proxy visuals unless explicitly enabled.

Structural update:

```text
r_wrist_interface
  └── palm
        ├── finger1_base  fixed
        │     └── finger1_motor1 -> finger1_proximal
        │           └── finger1_motor2 -> finger1_distal
        ├── finger2_base  fixed
        ├── finger3_base  fixed
        └── finger4_base  fixed
```

So finger joints are no longer authored as direct `palm -> proximal` joints; each finger now has an explicit fixed base/motor-frame link before the actuated proximal joint.

Latest verified runtime:

- Report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260703_112507_KST_implemented_base_frames_debug_runtime/robot_arm_hand_connected_report.json`
- Screenshots: `isaacsim_test/artifacts/visual_verification_20260703_112507_KST_implemented_base_frames_debug_runtime/`
- Overall: `PASS_WITH_FALLBACK`
- Runtime/Finger/Preshape/Lift-retain: all `PASS`
- `visual_mode`: `implemented_only`
- `link_count`: `14`
- `joint_count`: `13`
- `collision_primitive_count`: `13`
- `implemented_debug_visual_count`: `17`
- `finger_shell_visuals_enabled`: `false`
- `moving_shell_visual_count`: `0`

Use this run as the default evidence when judging whether the implemented hand skeleton/base frames are attached correctly. Use shell-overlay runs only after the base/joint structure is accepted.

## 2026-07-03 11:50 KST update: MJCF motor-anchor bases + hidden back lip strict PASS

User-visible hand debug default remains **implemented-only**:

```bash
ROBOT_ARM_HAND_VISUAL_MODE=implemented_only
ROBOT_ARM_HAND_INCLUDE_FINGER_SHELLS=0
ROBOT_ARM_HAND_SHOW_CONTACT_PROXIES=0
```

Do not show CAD shell/outer skin/yellow proxy visuals while judging finger joint motion. Use focused viewport screenshots and link-motion metrics together.

### Finger base anchor policy

The generated fallback hand now uses explicit fixed `fingerN_base` motor-frame links, and their origins are aligned to the MJCF `custom_servo_horn*` body positions instead of the earlier hand-authored approximate palm layout.

| finger | role | base / motor anchor source | base_xyz |
| --- | --- | --- | --- |
| finger1 | index | `custom_servo_horn` | `[-0.00505, 0.03055, 0.06980]` |
| finger2 | middle | `custom_servo_horn_2` | `[-0.00505, 0.00110, 0.06456]` |
| finger3 | ring | `custom_servo_horn_3` | `[-0.00505, -0.02705, 0.05505]` |
| finger4 | thumb | `custom_servo_horn_4` | `[-0.00030, 0.00773, 0.03615]` |

Report key: `finger_base_anchor_policy = fixed finger_base frames use MJCF custom_servo_horn world positions as palm-local motor-frame anchors`.

### Grasp object retention fix

After moving finger bases to the MJCF motor anchors, wrap preshape initially WARNed because the test cube slid backward before the fingers closed. The hidden palm cradle now has one additional physics-only rear lip:

- `palm_retention_back_lip_proxy`
- `local_xyz = (0.0, -0.028, 0.078)`
- `scale = (0.095, 0.010, 0.086)`

This is **not a visible fake hand**. It is hidden unless `ROBOT_ARM_HAND_SHOW_CONTACT_PROXIES=1`; default implemented-only screenshots still show only implemented gray skeleton/debug boxes.

### Latest verified runtime

- Report: `isaacsim_test/outputs/robot_arm_hand_graspable_20260703_115016_KST_mjcf_motor_anchor_back_lip_runtime/robot_arm_hand_connected_report.json`
- Artifact root: `isaacsim_test/artifacts/visual_verification_20260703_115016_KST_mjcf_motor_anchor_back_lip_runtime/`
- Overall status: `PASS_WITH_FALLBACK`
- Runtime validation: `PASS`
- Finger motion validation: `PASS`
- Preshape validation: `PASS`
- Lift-retain validation: `PASS`
- Visual mode: `implemented_only`
- `implemented_debug_visual_count`: `17`
- `finger_shell_visuals_enabled`: `false`

Preshape details:

| stage | result | close count | required | active min distances m |
| --- | --- | --- | --- | --- |
| single_finger | PASS | 1 | 1 | finger1 `0.03215` |
| pinch | PASS | 2 | 2 | finger1 `0.04190`, finger4 `0.05870` |
| wrap | PASS | 4 | 3 | finger1 `0.04998`, finger2 `0.02999`, finger3 `0.03204`, finger4 `0.06472` |

Lift-retain details:

- `finger_proxy_close_count_after_close = 7`
- `finger_proxy_close_count_after_lift = 7`
- Key images:
  - `finger1_two_link_motion.png`
  - `finger2_two_link_motion.png`
  - `finger3_two_link_motion.png`
  - `finger4_two_link_motion.png`
  - `preshape_wrap.png`
  - `grasp_real_hand_04_after_lift_retain.png`
  - `contact_sheet.png`

### Static verification

```bash
python3 isaacsim_test/test_robot_arm_hand_from_zip.py   # 43 tests OK
python3 isaacsim_test/test_graspable_hand_urdf.py       # 12 tests OK
python3 -m py_compile isaacsim_test/isaacsim/robot_arm_hand_from_zip.py isaacsim_test/isaacsim/graspable_hand_urdf.py
git diff --check -- isaacsim_test/isaacsim/robot_arm_hand_from_zip.py isaacsim_test/test_robot_arm_hand_from_zip.py isaacsim_test/isaacsim/graspable_hand_urdf.py isaacsim_test/test_graspable_hand_urdf.py isaacsim_test/run_robot_arm_hand_from_zip.sh
```

All passed.

### Current caveat

MJCF importer still fails with `basic_string::_M_construct null not valid`, so this is still `PASS_WITH_FALLBACK`. The current trusted hand path is generated URDF/USD fallback with implemented-only skeleton debug view, not final CAD shell alignment.

## 2026-07-03 Isaac Sim finger motion screenshot/debug evidence

- Non-grasp debug run completed with Isaac Sim 5.1.0 using `isaacsim_test/run_simready_motion_screenshot_cases.sh`.
- Contact sheet: `isaacsim_test/artifacts/simready_motion_cases_contact_sheet.png`.
- Raw screenshots:
  - `isaacsim_test/artifacts/simready_motion_cases/01_home.png`
  - `isaacsim_test/artifacts/simready_motion_cases/02_reach_forward.png`
  - `isaacsim_test/artifacts/simready_motion_cases/03_elbow_fold.png`
  - `isaacsim_test/artifacts/simready_motion_cases/04_side_sweep.png`
- Runtime log: `isaacsim_test/artifacts/runtime_logs/direct_urdf_motion_20260703T_debug_finger_motion.log`.
- Runtime report: `isaacsim_test/outputs/simready/echo_full/sitl/echo_full_lerobot_articulation_report.json`.
- Result: runtime status `PASS`; 61 URDF DOFs loaded; eight AmazingHand motors commanded from `amazinghand_grasp`; missing hand motors `[]`; wrist gap about `8.48e-08 m`.
- Important limitation from project memory: this contact sheet is a broad scene/motion screenshot, not the required close-up finger visual proof. It does not replace the hand debug camera protocol (`fingerN_two_link_before.png` and `fingerN_two_link_motion.png`, one finger at a time, focused on proximal/distal links).
- Next screenshot work: run/repair the close-up finger capture path and log `finger{1..4}_two_link_{before,motion}.png` plus a close-up contact sheet under a new timestamped artifact root before claiming visual correctness.

