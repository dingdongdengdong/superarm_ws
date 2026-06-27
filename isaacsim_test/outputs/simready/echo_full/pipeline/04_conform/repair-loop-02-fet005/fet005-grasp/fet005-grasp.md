# Grasp Line Authoring Report

- Status: `PASS`
- Output USD: `/home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd`
- Grasp vector: `/echo_full/grasp_identifier_01`
- Parent prim: `/echo_full`
- Points: `[[-80.0, 293.0, 665.0], [20.0, 293.0, 665.0]]`
- Source visual asset: `/home/sim/Documents/superarm_ws/arm_with_hand_with_robot_file/echo_full.step`
- Rationale: Vision-reviewed four-panel point-cloud evidence shows the AmazingHand palm/wrist body is the broad stable region; the selected line crosses the palm shell body and avoids thin fingers, wheels, sensors, and loose layout geometry.
- Coordinate note: Points are authored in /echo_full local coordinates because /echo_full has xformOp:scale:meter_normalization=(0.001,0.001,0.001); local points (-80,293,665) and (20,293,665) map to world points (-0.080,0.293,0.665) and (0.020,0.293,0.665), crossing RPalm_Shell world bounds.

## Visual Evidence

- `/home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-01/fet005-visual/grasp-preview.png`
- `/home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-01/fet005-visual/grasp-preview-overlay-x-localmm.png`

## Warnings

- None

## Errors

- None
