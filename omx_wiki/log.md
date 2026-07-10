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

## 2026-07-06 Isaac Sim 5.1 LeLab SuperArm Control Branch

- Created branch `feature/isaacsim-5.1-lelab-superarm-control`.
- Verified local Isaac Sim is 5.1: `/workspace/isaacsim/VERSION` = `5.1.0-rc.19+release.26219.9c81211b.gl`.
- Added 5DOF+grasp LeLab SuperArm server control contract module, CLI artifact generator, and unit tests.
- LeLab contract artifact root: `isaacsim_test/artifacts/lelab_isaacsim51_control_20260706T053212Z/`. Live PNG verification artifact root: `isaacsim_test/artifacts/lelab_isaacsim51_control_panel_20260706T051927Z/`.
- Generated timestamped `logs/`, six one-axis control cases, `lelab_superarm_control.html`, and PNG evidence.
- Initial live Isaac Sim 5.1 PNGs were captured but later rejected by subagent review for poor framing/occlusion; do not use them as proof. The accepted live render proof is the clean no-ground recapture listed below.
- Kept limitation explicit: current SimReady USD still has `binding_pending`, so this is contract/render compatibility evidence, not final physical six-axis articulation proof.

## 2026-07-06 Subagent PNG Verification

- Sent the LeLab SuperArm matrix PNG and live Isaac Sim render PNGs to vision subagents.
- Contract PNG verdict: PASS for six controls + six one-axis cases; not physics evidence.
- Initial live PNG verdict: FAIL due poor framing/occlusion.
- Fixed by recapturing the SimReady USD without a ground plane.
- Clean recapture verdict: PASS as Isaac Sim 5.1 visual render evidence, with caveat that it is cropped and not control/physics evidence.
- Preferred live PNG: `isaacsim_test/artifacts/lelab_isaacsim51_control_panel_20260706T051927Z/screenshots/live_isaacsim51_clean_echo_full_view/echo_full_5_1_clean_view.png`.

## 2026-07-06 Ralph Cleanup Audit

- Found and fixed a masking fallback introduced during mock control testing: non-mock `send_action()` now errors when called before ROS connection instead of silently updating local state.
- Added regression test for disconnected non-mock `send_action()`.
- Added timestamp validation for LeLab SuperArm artifact roots; custom suffixes must match `YYYYMMDDTHHMMSSZ`.
- Fresh checks passed: py_compile, 8 LeLab SuperArm tests, 5 LeRobot RPO arm tests, and artifact generator smoke with `20260706T053212Z`.

## 2026-07-06 LeLab Server Correction

- User clarified: not standalone control panel; LeLab should control the simulated arm.
- Renamed branch to `feature/isaacsim-5.1-lelab-superarm-control`.
- Renamed helper/test/runner to LeLab-focused names and changed artifact prefix to `lelab_isaacsim51_control_<YYYYMMDDTHHMMSSZ>`.
- Added LeLab patch `isaacsim_test/lelab_patches/0004-Add-SuperArm-server-six-field-contract-tests.patch`.
- Verified actual LeLab server/API contract with pytest: `tests/test_superarm_server.py tests/test_teleoperate.py` => 9 passed, 2 warnings.

## 2026-07-06 RoboParty V2 Right-Arm LeLab Correction

- User corrected the target: study LeLab first and use the RoboParty/Roboto right-arm control path, not the old source-arm reference.
- Commit-history finding:
  - `6e9537c fix: use RoboParty V2 right arm URDF for Isaac Sim` made `/workspaces/superarm_ws/roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf` the intended right-arm runtime target.
  - `a8a4bbf Validate LeRobot source arm pose capture` added `source_arm_isaacsim_arm_only.yaml` only for the separate source-arm experiment; current sanitizer now exposes `joint_rev_1..5`.
  - LeLab commit `f453845` had a stale fallback to `source_arm_isaacsim_arm_only.yaml`; the later six-field server patch did not fix that backend default.
- Fixed LeLab worktree commit: `361f171 Use RoboParty V2 right arm for Isaac Sim backend`.
- Exported main-repo patch: `isaacsim_test/lelab_patches/0005-Use-RoboParty-V2-right-arm-for-Isaac-Sim-backend.patch`.
- The patch changes LeLab `_create_isaacsim_rpo_arm_robot()` default config to `rpo_arm_isaacsim.yaml` and updates tests to require right-arm feature names plus `amazinghand_grasp`.
- Initialized `roboparty` submodule to verify the actual URDF path; before init, `python3 -m unittest isaacsim_test.test_v2_roboparty_config -v` failed with missing `roboto_origin.urdf`, after init it passed 10 tests.
- Verification artifact: `isaacsim_test/artifacts/lelab_roboparty_right_arm_fix_20260706T055444Z/logs/`.

## 2026-07-06 LeLab -> Isaac Sim RoboParty right-arm live control verification
- Live run folder: `isaacsim_test/artifacts/lelab_live_control_20260706T060150Z/` (date-time stamped logs/data/screenshots).
- LeLab website/control URL used: `http://127.0.0.1:8000/`; server module `lelab.superarm_server`, endpoints `POST /move-arm`, `POST /send-joint-action`, `GET /joint-positions`.
- Correct robot/control reference: `isaacsim_test/lerobot/rpo_arm_isaacsim.yaml` targeting RoboParty V2 right-arm URDF `roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf`; do not use the old source-arm-only reference for this LeLab control path.
- Command sent in joint order `[right_arm_pitch_joint,right_arm_roll_joint,right_arm_yaw_joint,right_elbow_pitch_joint,right_elbow_yaw_joint,amazinghand_grasp]`: `[0.35,-0.25,0.15,0.25,-0.25,1.0]`.
- Verified LeLab final state: pitch `0.3499999940395355`, roll `-0.25`, yaw `0.15000000596046448`, elbow_pitch `0.25`, elbow_yaw `-0.25`, grasp `1.0`.
- Isaac evidence: `binding_status=articulation_bound`, `using_simready=false` (URDF fallback forced for actual articulation), screenshot `screenshots/isaac_after_command/command_001.png`.
- Vision subagent PNG verdict: PASS — non-empty Isaac/Omniverse render with a visible white robot-arm segment against the blue scene background.
- No LeLab source edits were made for this live run; control was through existing LeLab endpoints.

## 2026-07-06 LeLab five-axis full screenshot verification
- User requested five one-axis LeLab actions with visible arm-position screenshots and labels.
- Artifact root: `isaacsim_test/artifacts/lelab_five_axis_fullscreens_20260706T062906Z/`.
- Used `isaac-sim-viewport-debugger` workflow and added Isaac scene camera override knobs to capture clearer right-arm views.
- Final commands through LeLab `/send-joint-action` in order `[pitch, roll, yaw, elbow_pitch, elbow_yaw, grasp]`:
  1. `[1.57,0,0,0,0,0]`
  2. `[0,1.57,0,0,0,0]`
  3. `[0,0,1.57,0,0,0]`
  4. `[0,0,0,1.57,0,0]`
  5. `[0,0,0,0,1.57,0]`
- LeLab readback matched each target; Isaac raw manifest records articulation readback for all five.
- Final labeled PNG folder: `screenshots/final_full_labeled/`; contact sheet: `five_axis_final_full_contact_sheet_labeled.png`.
- Vision subagent verdict: GREAT PASS — all five labeled screenshots are readable, show the robot/right arm clearly, and distinguish the intended one-axis 1.57 rad commands.

### 2026-07-06T06:44:49Z — standalone arm asset search

User clarified the previous LeLab captures showed the full RoboParty robot; next work should use only a standalone arm where possible. Search result:

- Existing standalone/source arm package: `robot_arm_hand_package.zip`, generated through `isaacsim_test/isaacsim/robot_arm_hand_from_zip.py`.
- Generated standalone URDF currently present at `isaacsim_test/outputs/robot_arm_hand_from_zip_local_drive/robot_arm_hand_sanitized.urdf`.
- Matching LeRobot config: `isaacsim_test/lerobot/source_arm_isaacsim_arm_only.yaml` with custom joints `joint_rev_1..joint_rev_5` and `fixed_hand: true`.
- Matching runner: `isaacsim_test/run_source_arm_lerobot_pose_capture.sh`; prior verified artifact: `isaacsim_test/artifacts/source_arm_lerobot_pose_cases_20260705T104845Z/report.json` (`status=done`, 4 pose cases, 4 screenshots).
- Existing 5-DOF RoboParty right-arm-only control config is `isaacsim_test/lerobot/rpo_arm_isaacsim_arm_only.yaml`, but it still targets the full RoboParty V2 URDF unless we create/hide/extract a standalone right-arm visual/asset.

Decision note: for a visually standalone arm, use the source-arm URDF path above after regeneration; it now has 5 active arm joints. For RoboParty-named 5-DOF control, keep using `rpo_arm_isaacsim_arm_only.yaml` or create a new extracted RoboParty right-arm-only asset.


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
## [2026-07-06T07:11:54.032Z] session-end
- **Pages:** session-log-2026-07-06-8-hcpbfw.md
- **Summary:** Auto-captured session log for omx-1783321913498-hcpbfw

## [2026-07-06T07:15:19.840Z] session-end
- **Pages:** session-log-2026-07-06-8-0s6ja2.md
- **Summary:** Auto-captured session log for omx-1783322042108-0s6ja2

## [2026-07-06T07:16:41.815Z] session-end
- **Pages:** session-log-2026-07-06-9-qlmx4g.md
- **Summary:** Auto-captured session log for omx-1783322194529-qlmx4g

## [2026-07-07T01:05:59.935Z] session-end
- **Pages:** session-log-2026-07-07-0-7lmfi0.md
- **Summary:** Auto-captured session log for omx-1783383804550-7lmfi0

## [2026-07-07T01:05:59.947Z] session-end
- **Pages:** session-log-2026-07-07-1-lmzjnh.md
- **Summary:** Auto-captured session log for omx-1783334617641-lmzjnh


## 2026-07-09 LeLab AmazingHand hand-only realtime control
- Current branch/workspace: `/home/dong/july/superarm_ws`, branch `feature/lelab-handpart-manual`.
- LeLab repo/worktree is at `/home/dong/july/superarm_ws/worktrees/leLab`.
- Added LeLab built-in robot record `SuperArm AmazingHand` and Manual Leader config for `isaacsim_test/lerobot/amazinghand_isaacsim_hand_only.yaml`.
- Generated/used hand-only Isaac URDF: `isaacsim_test/outputs/robot_arm_hand_from_zip_local_drive/amazinghand_graspable.urdf` with 8 joints `finger1_motor1..finger4_motor2`.
- Isaac Sim 6.0 scene bridge now uses the generated URDF hand tree, queues ROS callbacks onto the sim loop, holds the latest command, and writes command evidence JSONL.
- LeLab live control-only smoke passed for open, half, close through `/hand/joint_commands` -> `/hand/joint_states`:
  - Artifact: `isaacsim_test/artifacts/lelab_amazinghand_control_only_20260709T065136Z/`.
  - `hand_command_evidence.jsonl` contains exact applied readbacks for open `[0.05,0.02]*4`, half `[0.50,0.56]*4`, and close `[0.95,1.10]*4`.
  - Direct ROS isolation check also passed open/half/close/open2 at `isaacsim_test/artifacts/direct_amazinghand_control_20260709T065049Z/`.
- Test evidence:
  - `worktrees/leLab/.venv/bin/python -m pytest worktrees/leLab/tests/test_server.py worktrees/leLab/tests/test_utils_config.py worktrees/leLab/tests/test_teleoperate.py worktrees/leLab/tests/test_superarm_amazinghand_manual_config.py -q` -> 58 passed.
  - `worktrees/leLab/.venv/bin/python -m unittest isaacsim_test.test_lerobot_rpo_arm_control -v` -> 11 tests OK.
  - `worktrees/leLab/.venv/bin/python -m pytest isaacsim_test/test_setup_amazinghand_scene.py -q` -> 4 passed.
- Exported LeLab patch: `isaacsim_test/lelab_patches/0007-Add-SuperArm-AmazingHand-manual-leader.patch`; fresh apply-check over patches 0001..0007 passed.
- Still not done: headless close-up visual screenshots are not valid yet. Current camera screenshot path can fail with `Camera returned no RGBA data`; do not claim visual/finger proof until close-up per-finger PNG evidence exists.
- SimReady status remains `binding_pending`; this work proves LeLab realtime hand control on the Isaac-friendly generated URDF, not physical SimReady articulation binding.

## 2026-07-09 Realtime Viewer screenshot attempt
- Tried the `$omniverse-realtime-viewer` route for close-up hand screenshots after Isaac Camera/Replicator capture blocked the sim loop.
- Host and LeLab venv did not have `ovrtx`, `ovui`, or `ovstream` installed.
- Started a separate `.viewer-venv` and attempted `pip install --upgrade ovrtx --index-url https://pypi.nvidia.com --extra-index-url https://pypi.org/simple`; the wheel was ~2.5GB and the transfer stalled with no cache growth, so the install was stopped.
- Isaac container did not expose standalone `usdrecord`/`usdview` capture commands. Omniverse Python modules require launching `SimulationApp`, which is the same path that currently blocks on visual capture.
- Current screenshot state: Realtime control/readback is proven, but close-up per-finger visual evidence remains pending until `ovrtx` is installed successfully or a working Isaac viewport capture path is isolated.

## 2026-07-09 AmazingHand visual-import root cause + refreshed LeLab control
- Found the visual-import root cause while following the Realtime Viewer/screenshot path: the generated `amazinghand_graspable.urdf` contains host-absolute STL paths under `/home/dong/july/superarm_ws/...`, but the Isaac container sees the repo at `/workspace/superarm_ws`. Without remapping, Isaac imported the joints/collision boxes but dropped the real STL visual payloads.
- Fixed `isaacsim_test/isaacsim/setup_amazinghand_scene.py` to remap URDF mesh filenames to the container repo path before Isaac's URDF importer runs.
- Also fixed the Isaac 6 USD output default to use the active `HAND_SCREENSHOT_OUTPUT_DIR` artifact root, avoiding stale `manual_hand_screenshot_debug/usd` permission collisions.
- Fresh LeLab control/readback after the fix passed again:
  - Artifact: `isaacsim_test/artifacts/lelab_amazinghand_control_only_20260709T073737Z/`.
  - Scene log shows `remapped host URDF mesh paths for container`, `Loaded AmazingHand joints`, and applied open/half/close commands.
  - USD output now includes `payloads/geometries.usd` and `payloads/instances.usda`, confirming the STL visual payloads are present in the generated USD package.
- Realtime/Replicator PNG capture is still not accepted: Isaac Replicator with `wait_for_render=True` hung in headless mode after `step async`; the container was killed. Keep screenshot capture outside the realtime ROS loop until a nonblocking Realtime Viewer/ovrtx path is available.

## 2026-07-09 ovrtx install retry succeeded
- Retried the Omniverse Realtime Viewer dependency install in `.viewer-venv`.
- Previous blocker was network/download stall on the large `ovrtx` wheel, not package auth. The wheel is ~2.55GB.
- Successful command path:
  - `python3 -m venv .viewer-venv`
  - `.viewer-venv/bin/python -m pip install --upgrade pip setuptools wheel`
  - `PIP_CACHE_DIR=$PWD/.pip-cache-ovrtx .viewer-venv/bin/python -m pip install --upgrade ovrtx --index-url https://pypi.nvidia.com --extra-index-url https://pypi.org/simple`
- Installed version verified: `ovrtx-0.3.0.312915`.
- Import verified with `OVRTX_SKIP_USD_CHECK=1`.
- Added small PNG helper deps to `.viewer-venv`: `numpy`, `pillow`.
- Host runtime was missing `libOpenGL.so.0`; installed `libopengl0` via apt.
- Cleanup: removed temporary `/tmp/pip-unpack-*` wheel files. `.viewer-venv` is about 4.8GB after install.
- Render smoke is not accepted yet: first `ovrtx` smoke attempts with the bundled scene created stale high-CPU renderer processes and were killed. Next step is a clean minimal render with correct RenderProduct path and unbuffered logging, then AmazingHand USD capture.
## [2026-07-09T18:10:07.493Z] session-end
- **Pages:** session-log-2026-07-09-4-nrvehl.md
- **Summary:** Auto-captured session log for omx-1783597629794-nrvehl

## [2026-07-09T18:10:10.500Z] session-end
- **Pages:** session-log-2026-07-09-2-4gokfp.md
- **Summary:** Auto-captured session log for omx-1783577226092-4gokfp

