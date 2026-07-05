# echo_full Physics Scene Fix Report

Status: PASS

## Root cause

The final SimReady USD at `pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd` passed the `Prop-Robotics-Neutral` SimReady profile, but it is a CAD/prop-style visual and rigid-body asset. The FET004 repair promoted CAD mesh components to rigid bodies, but did not author robot articulation constraints; the earlier FET004 report had `joint_prims: []`.

That means the file can render correctly while still behaving as disconnected physics parts in an Isaac physics scene.

## Fix

The requested output root now contains the authored Isaac physics artifact:

- Physics articulation USD: `isaacsim_test/outputs/simready/echo_full/sitl/echo_full_lerobot_articulation.usda`
- Report: `isaacsim_test/outputs/simready/echo_full/sitl/echo_full_lerobot_articulation_report.json`
- Backup of old manifest-only file: `isaacsim_test/outputs/simready/echo_full/sitl/echo_full_lerobot_articulation.usda.manifest_only.bak`

Use the `sitl/echo_full_lerobot_articulation.usda` file for physics scenes. Keep the final SimReady CAD USD as visual/provenance input, not as the movable robot articulation.

## Validation

- `python3 isaacsim_test/test_echo_full_requested_articulation.py` — PASS
- `python3 -m unittest isaacsim_test.test_echo_full_requested_articulation isaacsim_test.test_echo_full_contact_tuning isaacsim_test.test_echo_full_joint_pose_fix` — PASS, 6 tests
- OpenUSD schema inspection — PASS:
  - articulation root: `/echo_full`
  - joint count: `29`
  - collision API count: `50`
  - default prim: `/echo_full`
