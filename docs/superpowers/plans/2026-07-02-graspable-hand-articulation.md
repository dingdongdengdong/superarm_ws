# Graspable Hand Articulation Implementation Plan

## Goal

Create an Isaac-friendly AmazingHand replacement hand model for the delivered arm package:

- Tree articulation, no MJCF equality/connect closed loops.
- Eight controllable finger joints matching the delivered actuator names.
- Existing STL visual assets reused for visual inspection.
- Primitive collision geometry used for stable contact and small-trash grasping.
- Existing arm import, arm motion screenshots, and connected USD workflow preserved.

## Success Criteria

- Static tests prove the hand model has one tree root, no closed-loop constraints, four fingers, eight actuated joints, valid limits, visual STL references, inertials, and collision primitives.
- The generated URDF references real STL files from the extracted package.
- The zip pipeline generates this URDF during preparation/conversion.
- When Isaac's MJCF importer fails, conversion uses the generated graspable hand URDF instead of the old non-articulated cube proxy.
- The report clearly distinguishes original MJCF failure from the new URDF hand fallback.

## Implementation Steps

1. Add failing tests for a new `graspable_hand_urdf` module.
   - Verify the hand spec keeps actuator names `finger1_motor1` through `finger4_motor2`.
   - Verify grasp scalar targets clamp and close monotonically.
   - Verify generated URDF contains real mesh references, collision primitives, inertials, and no MJCF equality comments.

2. Implement `isaacsim_test/isaacsim/graspable_hand_urdf.py`.
   - Build a conservative tree hand model with links: wrist, palm, four proximal phalanges, four distal phalanges.
   - Use STL visual meshes from `hand_mjcf/assets`.
   - Use primitive box collisions for palm, proximal links, distal links, and fingertip pads.
   - Add inertial blocks and joint limits suitable for Isaac URDF import.

3. Integrate the generated hand URDF into `robot_arm_hand_from_zip.py`.
   - Generate `amazinghand_graspable.urdf` beside existing sanitized artifacts.
   - Add a hand URDF importer with lower drive strengths than the arm.
   - On MJCF import failure, import the generated URDF and report it as `graspable_hand_fallback`.
   - Keep the old visual proxy only as a last-resort fallback if URDF import also fails.

4. Run verification.
   - First run the new tests and existing zip tests locally.
   - If local static tests pass, run the existing Isaac container script if available.
   - Record any remaining runtime limitation explicitly.

## Risks

- Joint origin placement is a first stable approximation, not a CAD-exact kinematic reconstruction.
- STL visuals may need per-link visual origin tuning after screenshots.
- Small-trash grasp quality depends on later contact tuning: friction, fingertip material, object mass, solver iterations, and controller gains.
