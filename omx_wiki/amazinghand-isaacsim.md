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

The visual source of truth is the original MJCF visual geometry list. The parser reads body and geom `pos`/`quat` transforms and reuses the existing STL files.

Default visual mode:

- `static_shell`
- attachment mode: `mjcf_static_visual_shell`
- one fixed `amazinghand_visual_shell` under `r_wrist_interface`
- expected MJCF visual geom count: `162`
- expected missing MJCF visual meshes: `[]`

Reason:

- The original hand visual assembly is closed-loop and has passive linkage behavior.
- The simplified Isaac tree hand uses approximate serial pivots.
- If original visual parts are partitioned onto the simplified moving tree links, initial pose can look good but finger motion can tear or separate parts around the wrong pivots.
- Static shell keeps the original AmazingHand appearance stable while primitive collision fingers perform contact physics.

Experimental visual mode:

- `partitioned_links`
- attachment mode: `mjcf_visuals_partitioned_to_tree_links`
- keep for diagnostics only until MJCF linkage kinematics are reconstructed.

## Current Limitation

In the default stable mode, STL finger visuals do not visibly curl with the collision fingers. This is intentional for now because the animated partition is not kinematically faithful to the original closed-loop AmazingHand.

To get both accurate animated visual and physical grasping later, implement a visual follower system derived from the MJCF linkage pivots or a proper tree-compatible USD/URDF visual decomposition.

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

## 2026-07-03 Fixed-Hand Arm-Only LeRobot Branch

This branch keeps AmazingHand fixed while moving the arm through LeRobot. Use `isaacsim_test/lerobot/rpo_arm_isaacsim_arm_only.yaml` for the 5D action/state surface. It exposes only:

- `right_arm_pitch_joint.pos`
- `right_arm_roll_joint.pos`
- `right_arm_yaw_joint.pos`
- `right_elbow_pitch_joint.pos`
- `right_elbow_yaw_joint.pos`

`amazinghand_grasp` is intentionally omitted from the arm-only config. The config records `fixed_hand: true` and `fixed_grasp: 0.0`. Runtime helper for real screenshots/logs: `isaacsim_test/run_fixed_hand_arm_lerobot_capture.sh`, which creates timestamped artifact folders under `isaacsim_test/artifacts/arm_fixed_hand_lerobot_<UTC>/`. Local verification evidence: `isaacsim_test/artifacts/arm_fixed_hand_lerobot_local_20260703T111832Z/`.

## 2026-07-05 Fixed-Hand LeRobot Recheck

Rechecked the current checkout against the fixed-hand arm-only LeRobot contract. Evidence root: `isaacsim_test/artifacts/arm_fixed_hand_lerobot_recheck_20260705T095748Z/`.

- `python3 -m py_compile isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py isaacsim_test/lerobot/verify_lerobot_sitl.py isaacsim_test/test_lerobot_rpo_arm_control.py`: passed.
- `python3 -m unittest isaacsim_test.test_lerobot_rpo_arm_control -v`: passed 4 tests.
- `python3 -m unittest isaacsim_test.test_v2_roboparty_config.RoboPartyV2ConfigTest.test_lerobot_sitl_verifier_uses_robot_config_and_checks_tolerance -v`: passed.
- `bash -n isaacsim_test/run_fixed_hand_arm_lerobot_capture.sh`: passed.
- Full unittest discovery still has the known external checkout gap: missing `roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf`.
- Live Isaac screenshot/control verification could not run in this environment because the Docker daemon socket was unavailable.

## 2026-07-05 Local Isaac Source-Arm LeRobot Drive

Docker was not usable for nested containers in this workspace: a manually started `dockerd` could initialize, but even `hello-world` failed during layer registration with `unshare: operation not permitted`. Local Isaac Sim exists at `/workspace/isaacsim`, while the repo root is `/workspaces/superarm_ws`.

Actual local Isaac/LeRobot arm drive was verified using the source-package sanitized URDF instead of Docker:

- Generated source URDF: `isaacsim_test/outputs/robot_arm_hand_from_zip_local_drive/robot_arm_hand_sanitized.urdf`.
- LeRobot config: `isaacsim_test/lerobot/source_arm_isaacsim_arm_only.yaml`.
- Controlled joints: `joint_rev_1`, `joint_rev_2`, `joint_rev_3`, `joint_rev_4`.
- Artifact root: `isaacsim_test/artifacts/source_arm_lerobot_actual_20260705T102314Z/`.
- LeRobot verifier: `PASS`, target/readback `[0.2, -0.2, 0.3, -0.4]`.
- Isaac command evidence: `data/isaac_command_evidence.json`, `binding_status: articulation_bound`.
- Isaac articulation readback after command: `[0.20000000298023224, -0.20000000298023224, 0.30000001192092896, -0.4000000059604645]`.
- Limitation: headless screenshot capture failed (`omni.kit.viewport_legacy` missing; viewport capture timeout), so this run has command/readback evidence but no visual screenshot.

Do not describe SimReady runs as real arm motion yet: the SimReady scene currently reports `binding_pending` and mirrors ROS state until articulation prim mapping is authored.

## 2026-07-05 - Isaac Sim viewport screenshot skill
- Created Codex skill: /root/.codex/skills/isaac-sim-viewport-debugger
- Purpose: frame/capture Isaac Sim or Omniverse Kit viewport/camera visual evidence for USD/URDF/articulation debugging.
- MVP script: scripts/frame_prim_and_capture.py uses Isaac Sim Python and camera-render fallback for headless workspaces.
- Validated with: quick_validate.py, py_compile, and live Isaac Sim URDF capture.
- Latest successful capture artifact: /workspaces/superarm_ws/isaacsim_test/artifacts/viewport_skill_capture_20260705T104129Z/source_arm_view.png
- Capture report: /workspaces/superarm_ws/isaacsim_test/artifacts/viewport_skill_capture_20260705T104129Z/viewport_debug_report.json

## 2026-07-05 - LeRobot multi-pose screenshot run
- Added multi-pose LeRobot source-arm runner and captured 4 pose cases under /workspaces/superarm_ws/isaacsim_test/artifacts/source_arm_lerobot_pose_cases_20260705T104845Z.
- Pose readback passed for all 4 cases; screenshots command_001.png..command_004.png plus contact_sheet.png were generated.
