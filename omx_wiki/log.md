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

## 2026-07-02 AmazingHand moving visual verification

- User rejected SimReady for now; focus is matching real hand visuals to moving physics.
- Changed AmazingHand generated hand default visual mode to `partitioned_links`.
- Kept yellow contact proxies debug-only/hidden by default.
- Narrowed MJCF visual partitioning so segment shells plus same-MJCF-body pins/screws move with generated finger links; passive linkage hardware remains wrist-attached until follower kinematics.
- Verified with Isaac run at `isaacsim_test/outputs/robot_arm_hand_graspable_20260702_moving_visual_segment_hw` and screenshots under `isaacsim_test/artifacts/robot_arm_hand_graspable_20260702_moving_visual_segment_hw`.
- Current result: finger motion/grasp smoke pass; lift-retain still warns/fails, so next work is visual followers then grasp retention tuning.

## 2026-07-02 SimReady Physics Scene Fix

- Root cause: final `echo_full_robot_arm_hand.usd` was SimReady prop/CAD output, not an authored robot articulation; it rendered but had no joint prims for physics connectivity.
- Fixed requested output root by replacing `isaacsim_test/outputs/simready/echo_full/sitl/echo_full_lerobot_articulation.usda` with the authored 29-joint Isaac articulation from the top-level mirror.
- Preserved the old manifest-only USDA as `echo_full_lerobot_articulation.usda.manifest_only.bak`.
- Added regression `isaacsim_test/test_echo_full_requested_articulation.py`.
- Validation: requested-path artifact has `/echo_full` articulation root, 29 joints, and 50 collision APIs; related unittest suite passed 6 tests.

## 2026-07-02 Arm-Only Physics Package

- User asked to discard the hand side and implement the remaining robot in detail under a date folder.
- Added generator `isaacsim_test/isaacsim/create_echo_full_arm_only_package.py`.
- Added regression `isaacsim_test/test_echo_full_arm_only_package.py` with TDD RED/GREEN cycle.
- Created `isaacsim_test/outputs/simready/echo_full/20260702_arm_only/` containing `arm_only.urdf`, `arm_only_physics.usda`, `load_arm_only.env`, and reports.
- Verified 6 URDF links, 5 arm revolute joints, no hand/finger/AmazingHand name leaks, USD `/echo_full_arm_only` articulation root, and 5 collision API prims.
- Validation command: `python3 -m unittest isaacsim_test.test_echo_full_arm_only_package isaacsim_test.test_rpo_arm_contract isaacsim_test.test_echo_full_requested_articulation` passed 9 tests.
## [2026-07-02T14:51:03.840Z] query
- **Pages:** none
- **Summary:** Query "hand debug camera crop AmazingHand finger close-up floor grid" → 0 results (of 0 total)

## 2026-07-03 Official Onshape Live Export Validation

- Confirmed newly registered Onshape API access by performing a live right-hand AmazingHand export.
- Artifact root: `isaacsim_test/outputs/amazinghand_onshape_live_20260702T155051Z`
- Original exported URDF failed direct Isaac articulation import due to unsupported ball joints / parse failure.
- Reduced eight-motor tree model preserved `finger1_motor1`…`finger4_motor2` and passed kinematic one-finger-at-a-time validation.
- Reviewed valid close-up debug contact sheet: `isaacsim_test/outputs/amazinghand_onshape_live_20260702T155051Z/stage_b_rep_debug_big_contact_sheet.png`
- Final report: `isaacsim_test/outputs/amazinghand_onshape_live_20260702T155051Z/final_report.md`

## [2026-07-03T00:32:46.845Z] session-end
- **Pages:** session-log-2026-07-03-7-jlyuwh.md
- **Summary:** Auto-captured session log for omx-1783038702877-jlyuwh

## [2026-07-03T02:08:14.831Z] session-end
- **Pages:** session-log-2026-07-03-8-q1yl8b.md
- **Summary:** Auto-captured session log for omx-1782696414808-q1yl8b

## [2026-07-03T02:08:17.107Z] session-end
- **Pages:** session-log-2026-07-03-4-58r4dz.md
- **Summary:** Auto-captured session log for omx-1782711676924-58r4dz

## [2026-07-03T02:08:18.173Z] session-end
- **Pages:** session-log-2026-07-03-1-55j59d.md
- **Summary:** Auto-captured session log for omx-1782869759881-55j59d

## [2026-07-03T02:08:19.204Z] session-end
- **Pages:** session-log-2026-07-03-2-is92ao.md
- **Summary:** Auto-captured session log for omx-1782892588132-is92ao

## [2026-07-03T02:08:20.301Z] session-end
- **Pages:** session-log-2026-07-03-4-9bh6jw.md
- **Summary:** Auto-captured session log for omx-1783002071324-9bh6jw

## [2026-07-03T02:08:21.692Z] session-end
- **Pages:** session-log-2026-07-03-2-l30b7y.md
- **Summary:** Auto-captured session log for omx-1783002089692-l30b7y

## [2026-07-03T02:08:22.703Z] session-end
- **Pages:** session-log-2026-07-03-5-79a1v7.md
- **Summary:** Auto-captured session log for omx-1783005996755-79a1v7

## [2026-07-03T02:08:23.770Z] session-end
- **Pages:** session-log-2026-07-03-1-mvweh4.md
- **Summary:** Auto-captured session log for omx-1782572167851-mvweh4

## [2026-07-03T02:08:24.436Z] session-end
- **Pages:** session-log-2026-07-03-5-7a4f3p.md
- **Summary:** Auto-captured session log for omx-1782575917175-7a4f3p

## [2026-07-03T02:08:25.458Z] session-end
- **Pages:** session-log-2026-07-03-7-jm1lvj.md
- **Summary:** Auto-captured session log for omx-1782695921407-jm1lvj

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

