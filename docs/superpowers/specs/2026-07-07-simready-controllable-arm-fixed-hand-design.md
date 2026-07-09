# SimReady Controllable Arm with Fixed AmazingHand Design

## Goal
Create a non-destructive Isaac Sim control asset for the existing `echo_full_robot_arm_hand.usd` so LeLab/LeRobot can drive the five arm joints while AmazingHand remains a fixed visual reference.

## Constraints
- Do not overwrite the SimReady source USD.
- Preserve the existing LeRobot field order for compatibility: five arm joints plus `amazinghand_grasp`.
- Bind only the five arm joints to physics drives.
- Treat `amazinghand_grasp` as `fixed_visual` with no physics joint or drive.
- Keep the original SimReady visual referenced in the new asset so the full arm+hand/cart remains visible.

## Architecture
Add a pure-USD authoring utility that creates a derived overlay USD beside the SimReady output. The overlay references the existing SimReady USD under `VisualFixed` and authors a small five-DOF PhysX/UsdPhysics control rig under `ControlRig`. The rig has named revolute joints matching the LeRobot arm contract and a mapping JSON that marks those joints as `articulation_bound`; `amazinghand_grasp` is marked `fixed_visual`.

## Data Flow
LeLab/LeRobot sends the current six-field vector. Isaac scene loading prefers the controllable overlay USD when available. Runtime applies the first five command values to the authored articulation joints and ignores the sixth value for physics while still publishing it for state compatibility.

## Validation
Local tests inspect the authored USD with `pxr.Usd`/`UsdPhysics`, verify articulation/joint/drive schemas, verify the original asset is referenced, verify the source asset remains unchanged, and verify the mapping JSON documents fixed-hand status. Live Isaac validation remains a separate step because Docker/ROS/Isaac are not running in this shell.
