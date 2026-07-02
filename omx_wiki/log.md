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
