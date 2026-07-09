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

## 2026-07-09 Isaac Sim 6.0 Visual-Proof Guardrail

### Correct target and separation

For hand-fidelity work, use the standalone generated graspable-hand tree:

- source package: `robot_arm_hand_package.zip` -> `hand_mjcf/robot.xml` and `hand_mjcf/assets/*.stl`
- runtime target: `isaacsim_test/outputs/robot_arm_hand_from_zip_local_drive/amazinghand_graspable.urdf`
- articulation root: `/amazinghand_graspable/Geometry`
- required DOFs: `finger1_motor1`, `finger1_motor2`, ..., `finger4_motor1`, `finger4_motor2`

Do **not** use SimReady `echo_full` screenshots, the six-field LeLab contract, or yellow contact proxies as proof that the standalone hand fingers visibly move.  Those are separate paths; SimReady remains `binding_pending` for this purpose.

### Capture acceptance criteria (mandatory)

An articulation target/readback proves control plumbing only.  It is not visual proof.

1. Import the standalone hand in Isaac Sim 6.0 and command named open and close poses for all eight DOFs.
2. Capture a close-up with a headless render product/Replicator or a camera sensor; on Ubuntu Server this is supported and does not require a desktop viewport.
3. Save the before/after PNGs and command/readback JSON in the same timestamped artifact directory.
4. Inspect the PNG pixels.  Reject uniform white, uniform black, empty, badly framed, or proxy-only images even when a writer emits non-empty files or a report says `PASS`.
5. Only then report `visual motion verified`; physics/contact-grasp verification remains a separate claim.

### Failure mode found on 2026-07-09

Host-side generation wrote STL filenames rooted at the host checkout, for example
`/home/dong/july/superarm_ws.omx-worktrees/launch-feat-hitl/.../hand_mjcf/assets/*.stl`.
Inside the Isaac Sim container the checkout is `/workspace/superarm_ws`; the host path is absent.  Isaac then imports the 8-DOF articulation and its primitive collision boxes but silently omits the visual meshes, producing blank Replicator output.

Before any container import, generate the URDF inside the container or rewrite the mesh filename prefix to `/workspace/superarm_ws`.  Confirm the imported USD contains real `Mesh` prims (the corrected diagnostic import contained 114), then still apply the image-inspection rule above.  The first Replicator frames can be cleared textures, so warm several render frames and retain the inspected final open/closed frames.

Evidence roots from this diagnosis:

- blank/stale-capture rejection: `isaacsim_test/artifacts/hand_replicator_detail_20260709T194300Z/`
- container-path import and mesh check: `isaacsim_test/artifacts/hand_replicator_visual_paths_20260709T195000Z/`
- bounds-framed, warmed capture attempt: `isaacsim_test/artifacts/hand_replicator_framed_20260709T200000Z/`

The current direct 8-DOF readback is successful, but the images above are still rejected as blank.  Do not present this run as visual verification until a non-blank, close-up before/after pair is reviewed.

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

## 2026-07-06 Isaac Sim 5.1 LeLab 5DOF + Grasp LeLab SuperArm Control

Created branch `feature/isaacsim-5.1-lelab-superarm-control` for Isaac Sim 5.1 work.
Local version evidence: `/workspace/isaacsim/VERSION` reports
`5.1.0-rc.19+release.26219.9c81211b.gl`.

Added a branch-local 5.1 LeLab SuperArm server control contract helper and tests:

- `isaacsim_test/lerobot/lelab_isaacsim51_control_contract.py`
- `isaacsim_test/run_lelab_isaacsim51_control_verification.py`
- `isaacsim_test/test_lelab_isaacsim51_control_contract.py`

Latest LeLab server contract artifact root:
`isaacsim_test/artifacts/lelab_isaacsim51_control_20260706T053212Z/`.
Subagent-verified live Isaac render evidence remains in:
`isaacsim_test/artifacts/lelab_isaacsim51_control_panel_20260706T051927Z/`.
It contains timestamped `logs/`, six one-axis 5DOF+grasp cases, static
`lelab_superarm_control.html`, `screenshots/lelab_superarm_control_verification.png`, and
`report.json`. Live Isaac Sim 5.1 render captures were also written under
`screenshots/live_isaacsim51_*_view/`.

Visual inspection notes:

- `lelab_superarm_control_verification.png` is a non-empty control/case matrix showing
  six controls and six one-axis test cases.
- Accepted live render evidence is `live_isaacsim51_clean_echo_full_view/echo_full_5_1_clean_view.png`, which passed subagent review as an identifiable Isaac Sim 5.1 render of the SimReady/`echo_full` arm/hand/end-effector area.
- Rejected diagnostic captures `live_isaacsim51_echo_full_view/echo_full_5_1_view.png` and `live_isaacsim51_simready_view/simready_5_1_view.png` are non-proof artifacts because subagent review found poor framing/occlusion.

Control limitation remains: SimReady six-field articulation binding is still
`binding_pending`, so this branch verifies 5.1 compatibility and LeLab control
contract, not physical six-axis SimReady motion yet.

### 2026-07-06 subagent PNG verification update

The first visual pass found a real problem and was fixed:

- Vision subagent PASS: `screenshots/lelab_superarm_control_verification.png` shows the
  six-control/six-case contract matrix. It is sufficient for LeLab SuperArm server
  contract evidence only, not physics.
- Vision subagent FAIL: initial live captures
  `screenshots/live_isaacsim51_echo_full_view/echo_full_5_1_view.png` and
  `screenshots/live_isaacsim51_simready_view/simready_5_1_view.png` were
  non-empty but poorly framed/occluded, so they are not acceptable visual asset
  evidence.
- Fix: recaptured without adding a ground plane using
  `capture_usd_preview_no_ground.py` in the artifact folder.
- Vision subagent PASS after fix:
  `screenshots/live_isaacsim51_clean_echo_full_view/echo_full_5_1_clean_view.png`
  is non-empty and shows identifiable SimReady/`echo_full` arm/hand/end-effector
  geometry under Isaac Sim 5.1. It is still cropped and is render evidence only.

Preferred live PNG evidence for this run:
`isaacsim_test/artifacts/lelab_isaacsim51_control_panel_20260706T051927Z/screenshots/live_isaacsim51_clean_echo_full_view/echo_full_5_1_clean_view.png`.

Do not use the rejected initial live PNGs as proof in user-facing summaries.

### 2026-07-06 Ralph cleanup/audit update

During Ralph cleanup, a masking fallback was found in the LeRobot shim change:
`send_action()` could silently update local state when `_pub is None` outside
mock mode. Fixed behavior: only `mock=True` updates local readback without ROS;
non-mock `send_action()` now raises `RuntimeError` if called before `connect()`.
Regression test: `test_real_lerobot_backend_send_action_requires_connection`.

Also added a timestamp guard for `create_timestamped_artifact_root()` so custom
artifact suffixes must match `YYYYMMDDTHHMMSSZ`. This protects the user
requirement that logs/artifact folders include a real date-time. Regression test:
`test_artifact_timestamp_must_be_utc_datetime`.

Fresh post-cleanup verification:

- `python3 -m py_compile isaacsim_test/lerobot/lelab_isaacsim51_control_contract.py isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py isaacsim_test/run_lelab_isaacsim51_control_verification.py isaacsim_test/test_lelab_isaacsim51_control_contract.py`: passed.
- `python3 -m unittest isaacsim_test.test_lelab_isaacsim51_control_contract -v`: passed 8 tests.
- `python3 -m unittest isaacsim_test.test_lerobot_rpo_arm_control -v`: passed 5 tests.
- `python3 isaacsim_test/run_lelab_isaacsim51_control_verification.py --timestamp 20260706T053212Z`: generated a valid date-time artifact with `status=done`, `control_count=6`, and `test_case_count=6`.

### 2026-07-06 LeLab server correction

User clarified the target is not a standalone control panel; it is LeLab controlling the simulated arm. The persistent evidence is now LeLab-server focused:

- branch renamed to `feature/isaacsim-5.1-lelab-superarm-control`;
- helper files renamed to `lelab_isaacsim51_control_*`;
- artifact prefix changed to `lelab_isaacsim51_control_<YYYYMMDDTHHMMSSZ>`;
- added LeLab patch `isaacsim_test/lelab_patches/0004-Add-SuperArm-server-six-field-contract-tests.patch`;
- LeLab pytest evidence: `cd worktrees/leLab && python3 -m pytest tests/test_superarm_server.py tests/test_teleoperate.py -q` passed 9 tests.

The static PNG is now treated only as a LeLab SuperArm six-field contract matrix. Actual LeLab route/API behavior is covered by the LeLab pytest patch.

### 2026-07-06 RoboParty V2 right-arm correction for LeLab

Do not use `source_arm_isaacsim_arm_only.yaml` as the default LeLab Isaac Sim backend target. That file belongs to the separate local source-arm experiment; current sanitizer now exposes `joint_rev_1..5`. The LeLab SuperArm Isaac Sim backend should default to `isaacsim_test/lerobot/rpo_arm_isaacsim.yaml`, which controls:

1. `right_arm_pitch_joint`
2. `right_arm_roll_joint`
3. `right_arm_yaw_joint`
4. `right_elbow_pitch_joint`
5. `right_elbow_yaw_joint`
6. `amazinghand_grasp`

Commit-history basis: main repo commit `6e9537c` established the RoboParty V2 right-arm URDF target; main repo commit `a8a4bbf` added source-arm support only as a separate experiment; LeLab commit `361f171` corrects the stale LeLab fallback to `rpo_arm_isaacsim.yaml`.

`arm_with_hand_with_robot_file/echo_full.step` and `echo_full.stl` are source/visual asset inputs for the SimReady pipeline, not the default LeLab backend config. Initialize the `roboparty` submodule (`git submodule update --init roboparty`) before running checks that inspect `roboto_origin.urdf`.

### 2026-07-06 live LeLab -> Isaac Sim RoboParty right-arm control

Live control through LeLab was verified on Isaac Sim 5.1 using the RoboParty V2
right-arm URDF path, not the source-arm-only experiment config.

- LeLab URL used: `http://127.0.0.1:8000/` (`/docs` for API docs).
- LeLab server/control attribution: `lelab.superarm_server`, endpoints
  `POST /move-arm`, `POST /send-joint-action`, and `GET /joint-positions`.
- Backend/config: `isaacsim_rpo_arm` via
  `isaacsim_test/lerobot/rpo_arm_isaacsim.yaml`.
- Isaac target: `roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf`.
- Command order:
  `[right_arm_pitch_joint,right_arm_roll_joint,right_arm_yaw_joint,right_elbow_pitch_joint,right_elbow_yaw_joint,amazinghand_grasp]`.
- Command sent: `[0.35,-0.25,0.15,0.25,-0.25,1.0]`.
- Final LeLab readback: pitch `0.3499999940395355`, roll `-0.25`, yaw
  `0.15000000596046448`, elbow_pitch `0.25`, elbow_yaw `-0.25`, grasp `1.0`.
- Isaac evidence: `binding_status=articulation_bound`, `using_simready=false`
  because the run forced RoboParty URDF fallback for actual articulation proof.
- Artifact root: `isaacsim_test/artifacts/lelab_live_control_20260706T060150Z/`.
- PNG: `screenshots/isaac_after_command/command_001.png`; vision subagent
  verdict PASS as a non-empty Isaac/Omniverse render with visible white robot-arm
  segment.

No LeLab source edits were made during this live run; control used the existing
LeLab server endpoints.

### 2026-07-06 five one-axis LeLab screenshot set

A final five-action screenshot set was captured through LeLab using the RoboParty
right-arm Isaac backend. The final accepted artifacts are under
`isaacsim_test/artifacts/lelab_five_axis_fullscreens_20260706T062906Z/`.

The five commands were sent through `POST /send-joint-action` in joint order
`[right_arm_pitch_joint,right_arm_roll_joint,right_arm_yaw_joint,right_elbow_pitch_joint,right_elbow_yaw_joint,amazinghand_grasp]`:

1. `[1.57,0,0,0,0,0]`
2. `[0,1.57,0,0,0,0]`
3. `[0,0,1.57,0,0,0]`
4. `[0,0,0,1.57,0,0]`
5. `[0,0,0,0,1.57,0]`

Final labeled PNGs: `screenshots/final_full_labeled/`.
Contact sheet: `screenshots/final_full_labeled/five_axis_final_full_contact_sheet_labeled.png`.

Vision subagent verdict: GREAT PASS — all five labeled screenshots are readable,
show the robot/right arm clearly, and distinguish the intended one-axis 1.57 rad
commands for pitch, roll, yaw, elbow_pitch, and elbow_yaw.


### 2026-07-06T07:00:00Z — source standalone arm corrected to 5 DOF

Comparison/fix for standalone source arm vs RoboParty V2 right arm:

- RoboParty V2 right-arm URDF already has 5 arm joints: `right_arm_pitch_joint`, `right_arm_roll_joint`, `right_arm_yaw_joint`, `right_elbow_pitch_joint`, `right_elbow_yaw_joint`.
- The standalone source package contained five motor meshes (`motor_1..motor_5`), but its exported xacro only marked `joint_rev_1..joint_rev_4` as continuous and exported the `motor_5 -> arm_link3b` output as fixed joint `joint_fix_28`.
- Fixed in `isaacsim_test/isaacsim/robot_arm_hand_from_zip.py`: sanitizer promotes that specific exported fixed joint to continuous `joint_rev_5` with parent `motor_5`, child `arm_link3b`, axis `0.0 0.0 1.0`.
- Updated source-arm LeRobot config `isaacsim_test/lerobot/source_arm_isaacsim_arm_only.yaml` to use `joint_rev_1..joint_rev_5`.
- Updated `isaacsim_test/run_source_arm_lerobot_pose_capture.sh` and `isaacsim_test/lerobot/run_lerobot_pose_cases.py` pose targets to 5 values.
- Regenerated current standalone URDF at `isaacsim_test/outputs/robot_arm_hand_from_zip_local_drive/robot_arm_hand_sanitized.urdf`; moving joints now equal `joint_rev_1..joint_rev_5`.
- Verification artifact: `isaacsim_test/artifacts/source_arm_5dof_fix_20260706T065945Z/`.
- Tests: `python3 -m unittest isaacsim_test.test_robot_arm_hand_from_zip isaacsim_test.test_lerobot_rpo_arm_control isaacsim_test.test_v2_roboparty_config -v` passed 31 tests.
