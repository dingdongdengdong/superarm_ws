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
