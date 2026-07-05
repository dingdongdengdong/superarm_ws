# Project Memory Log

## 2026-07-02

- Verified that `omx_wiki/` did not previously contain committed project memory for this task.
- Created repo-backed project memory under `omx_wiki/`.
- Recorded the Isaac Sim hand decision: physics uses a tree URDF; original MJCF remains the visual/source reference.
- Debugged the finger visual breakage:
  - per-link visual partition preserved the initial MJCF shape but used simplified URDF tree pivots during motion.
  - original AmazingHand is a closed-loop linkage, so partitioning visual parts onto approximate serial links can tear the visual during finger closing.
- Chosen fix:
  - default back to `static_shell` visual mode for stable original AmazingHand appearance.
  - keep `partitioned_links` as an explicit experimental mode.
- Static-shell fix validation:
  - `python3 isaacsim_test/test_graspable_hand_urdf.py`: passed, 4 tests.
  - `python3 isaacsim_test/test_robot_arm_hand_from_zip.py`: passed, 14 tests.
  - Isaac `robot_arm_hand_graspable_20260702_visualfix_static`: `PASS_WITH_FALLBACK`, runtime `PASS`.
  - Report fields: visual mode `static_shell`, attachment mode `mjcf_static_visual_shell`, MJCF visual geom count `162`, missing visual meshes `[]`.
  - Reviewed contact sheet: `isaacsim_test/artifacts/robot_arm_hand_graspable_20260702_visualfix_static/contact_sheet.png`.
- Next technical focus:
  - tune contact pads, friction, drive gains, and lift-retain tests for small trash grasping.
  - reconstruct a true animated visual follower only after the MJCF linkage pivots are mapped.

## 2026-07-02 Finger Two-Link Physics Validation

- Added runtime `finger_motion_validation`.
- Confirmed generated topology:
  - `finger*_motor1` drives `palm -> finger*_proximal`.
  - `finger*_motor2` drives `finger*_proximal -> finger*_distal`.
- Static tests:
  - `python3 isaacsim_test/test_graspable_hand_urdf.py`: passed, 4 tests.
  - `python3 isaacsim_test/test_robot_arm_hand_from_zip.py`: passed, 15 tests.
- Isaac run:
  - output root: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_finger2link`
  - artifact root: `isaacsim_test/artifacts/robot_arm_hand_graspable_20260702_finger2link`
  - runtime validation: `PASS`
  - finger motion validation: `PASS`
- Evidence:
  - all four fingers reached target `[0.78, 0.96]` rad within small readback error.
  - screenshots: `finger1_two_link_motion.png` through `finger4_two_link_motion.png`.
- Next step:
  - proceed to small-trash grasp physics: collision pad placement, friction, drive gains, solver settings, and lift-retain validation.

## 2026-07-02 Contact Proxy And Lift-Retain Debug

- Added explicit runtime hand collision proxies because the imported hand stage exposed empty `collisions` Xforms without usable `CollisionAPI` geometry.
- Contact proxy contract:
  - 13 hidden cube collision proxies total.
  - palm: 1 proxy.
  - each of 4 fingers: proximal, distal, distal tip pad.
  - high-friction material: static `1.6`, dynamic `1.35`, restitution `0.02`.
- Fixed a validation bug where the grasp object fell during open/settle before close:
  - reset object to the hand-local target immediately before close.
  - zero linear/angular velocity on reset.
- Static tests:
  - `python3 isaacsim_test/test_graspable_hand_urdf.py`: passed, 4 tests.
  - `python3 isaacsim_test/test_robot_arm_hand_from_zip.py`: passed, 16 tests.
- Isaac run:
  - output root: `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_objectreset`
  - artifact root: `isaacsim_test/artifacts/robot_arm_hand_graspable_20260702_objectreset`
  - runtime validation: `PASS`
  - contact tuning: `PASS`
  - authored/bound collision proxies: `13/13`
  - finger motion validation: `PASS`
  - grasp smoke: `PASS`
  - lift-retain validation: `WARN`
- Current physical limitation:
  - close starts with the object at the hand target and brings it near the hand.
  - the object still drops during retain/lift.
  - next work must tune proxy geometry, object placement, drive strength, damping, and contact solver settings for sustained grasp.

## 2026-07-03 Fixed-Hand Arm-Only LeRobot Config

- Added an arm-only LeRobot config for this branch: `isaacsim_test/lerobot/rpo_arm_isaacsim_arm_only.yaml`.
- Contract: move only the five right-arm joints; keep AmazingHand fixed. The config omits `amazinghand_grasp`, sets `fixed_hand: true`, and `fixed_grasp: 0.0`.
- Updated `IsaacSimRpoArmRobot` so arm-only configs do not clamp the fifth arm joint as if it were a grasp scalar. Clamping now applies only to the named `amazinghand_grasp` feature; fixed-hand configs pin that value.
- Updated `verify_lerobot_sitl.py` so target length follows the selected config: 5D for arm-only, 6D for the existing full arm+grasp config.
- Added timestamped run helper: `isaacsim_test/run_fixed_hand_arm_lerobot_capture.sh`. Real Isaac runs write `logs/`, `screenshots/`, `data/`, `report.json`, and `report.md` under `isaacsim_test/artifacts/arm_fixed_hand_lerobot_<UTC_DATE_TIME>/`.
- Local verification artifact: `isaacsim_test/artifacts/arm_fixed_hand_lerobot_local_20260703T111832Z/`.
  - Targeted tests: `logs/final_targeted_unittest.log` passed 5 tests.
  - Static checks: `logs/py_compile.log` and `logs/bash_n.log` passed.
  - Full unittest discovery still has the known external checkout gap: missing `roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf`.
  - No live Isaac screenshot was captured in the local artifact folder because Docker daemon access was unavailable in this environment; use `run_fixed_hand_arm_lerobot_capture.sh` on the Isaac/Docker host for live screenshots.

## 2026-07-05 Fixed-Hand Arm-Only LeRobot Recheck

- Checked recent commits and current project memory: the uncommitted LeRobot branch is the arm-only fixed-hand path.
- Recheck artifact root: `isaacsim_test/artifacts/arm_fixed_hand_lerobot_recheck_20260705T095748Z/`.
- Passed local contract validation:
  - `python3 -m py_compile isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py isaacsim_test/lerobot/verify_lerobot_sitl.py isaacsim_test/test_lerobot_rpo_arm_control.py`
  - `python3 -m unittest isaacsim_test.test_lerobot_rpo_arm_control -v`
  - `python3 -m unittest isaacsim_test.test_v2_roboparty_config.RoboPartyV2ConfigTest.test_lerobot_sitl_verifier_uses_robot_config_and_checks_tolerance -v`
  - `bash -n isaacsim_test/run_fixed_hand_arm_lerobot_capture.sh`
- Full unittest discovery ran 33 tests with 32 passing and the known missing external RoboParty V2 URDF checkout error.
- Live Isaac/ROS movement and screenshot verification remains blocked in this workspace by unavailable Docker daemon access (`Cannot connect to the Docker daemon at unix:///var/run/docker.sock`).

## 2026-07-05 Local Isaac Source-Arm LeRobot Drive

- User clarified the workspace uses `/workspaces/superarm_ws`; local Isaac Sim is installed at `/workspace/isaacsim`.
- Docker diagnosis:
  - default daemon unavailable.
  - manually starting `dockerd` on `/tmp/superarm-docker.sock` worked briefly in a persistent session.
  - running `hello-world` failed with `failed to register layer: unshare: operation not permitted`, so nested Docker is not viable in this workspace.
- Local Isaac/ROS path:
  - Isaac Python needs `LD_LIBRARY_PATH=/workspace/isaacsim/exts/isaacsim.ros2.bridge/humble/lib` and `PYTHONPATH=/workspace/isaacsim/exts/isaacsim.ros2.bridge/humble/rclpy`.
  - system verifier needs a clean `/opt/ros/humble/setup.bash` environment; do not inherit Isaac bridge libraries into verifier Python.
- Patched local control support:
  - `setup_rpo_arm_scene.py` can parse `JOINT_NAMES` from env, enabling source URDF joints.
  - `verify_lerobot_sitl.py` allows configs with `allow_custom_joint_names: true`.
  - added `isaacsim_test/lerobot/source_arm_isaacsim_arm_only.yaml`.
  - added Isaac command evidence JSON with `articulation_readback`.
- Actual drive evidence:
  - artifact root: `isaacsim_test/artifacts/source_arm_lerobot_actual_20260705T102314Z/`
  - URDF: `isaacsim_test/outputs/robot_arm_hand_from_zip_local_drive/robot_arm_hand_sanitized.urdf`
  - LeRobot verifier passed for joints `joint_rev_1..joint_rev_4` and target `[0.2, -0.2, 0.3, -0.4]`.
  - Isaac internal readback matched target in `data/isaac_command_evidence.json` with `binding_status: articulation_bound`.
  - Screenshot capture failed in headless local mode; do not claim visual proof for this run.

## 2026-07-05T10:42Z
- Created /root/.codex/skills/isaac-sim-viewport-debugger via skill-creator and used it to capture source arm URDF visual evidence.
- Successful screenshot: /workspaces/superarm_ws/isaacsim_test/artifacts/viewport_skill_capture_20260705T104129Z/source_arm_view.png; report status=done, image_size_bytes=576513.

## 2026-07-05T10:50Z
- Ran source arm LeRobot multi-pose screenshot validation. Run dir: /workspaces/superarm_ws/isaacsim_test/artifacts/source_arm_lerobot_pose_cases_20260705T104845Z. 4/4 cases passed; 4/4 screenshots captured.
