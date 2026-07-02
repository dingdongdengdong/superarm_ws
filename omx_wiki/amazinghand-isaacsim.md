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
