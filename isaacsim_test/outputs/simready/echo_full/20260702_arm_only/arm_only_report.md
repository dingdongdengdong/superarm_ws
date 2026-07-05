# echo_full Arm-Only Physics Package

Status: `PASS`
Date folder: `20260702_arm_only`

## Scope

- AmazingHand, fingers, and wrist-interface hand links are intentionally excluded.
- The remaining Roboto V2 right-arm chain keeps five controlled revolute joints.
- URDF remains the Isaac runtime source of truth; the USDA mirrors the same physics contract for inspection and USD-based loading experiments.

## Artifacts

- URDF: `isaacsim_test/outputs/simready/echo_full/20260702_arm_only/arm_only.urdf`
- USD: `isaacsim_test/outputs/simready/echo_full/20260702_arm_only/arm_only_physics.usda`
- Env: `isaacsim_test/outputs/simready/echo_full/20260702_arm_only/load_arm_only.env`

## Validation Summary

- Links: `6`
- Revolute joints: `5`
- Collision shapes: `5`
- Controlled joints: `right_arm_pitch_joint, right_arm_roll_joint, right_arm_yaw_joint, right_elbow_pitch_joint, right_elbow_yaw_joint`

## Usage

Load this arm-only package instead of the final CAD SimReady USD when you need connected physics.
The final SimReady CAD asset can still be used separately as visual reference, but this package deliberately removes the hand side.
