# 2026-07-03 AmazingHand Team Debug Findings

- Team: amazinghand-isaac-sim-439811d9
- Runtime target: Isaac Sim 5.1 only
- Baseline evidence: /home/sim/Documents/superarm_ws/isaacsim_test/outputs/robot_arm_hand_graspable_20260703_155212_KST_motor_faithful_finger_frame_runtime/robot_arm_hand_connected_report.json
- Screenshot evidence: /home/sim/Documents/superarm_ws/isaacsim_test/artifacts/visual_verification_20260703_155212_KST_motor_faithful_finger_frame_runtime

## Findings

1. Contact/debug proxies are hidden in screenshots. Report has `show_contact_proxies=false`; 23 PNGs are present.
2. `single_finger` WARN is object reset/settle drift before command, not finger command failure. Current run drifted about 1.328 m; previous 20260703_115016_KST pass drifted about 0.014 m.
3. `wrap` WARN is finger3 high-target under-tracking. Current finger3 wrap errors are about 0.154 / 0.349 rad; previous 20260703_115016_KST pass hit the same targets within about 0.001 / 0.004 rad.
4. `finger_motion_validation` previously allowed visible movement to pass without explicitly surfacing target-reached status.

## Implemented in team snapshot, then applied to main

- Added preshape object-reset stability reporting and bounded retry.
- Added explicit target-error threshold helper/reporting.
- Added `target_reached` / `target_error_threshold_rad` to finger-motion results.
- Added regression tests for reset drift and wrap/finger3 threshold behavior.

## Required next runtime validation

Run Isaac Sim 5.1 from the main checkout with no contact proxies and a fresh timestamp. Required improvement: `preshape_grasp_validation.status=PASS`, `single_finger` object reset stable and close count >= 1, and `wrap` target_reached true or finger3 errors explicitly documented for further drive/convergence tuning.

## Fresh runtime after patch

- Run stamp: `20260703_162039_KST_motor_faithful_finger_frame_runtime_retry`
- Report: `/home/sim/Documents/superarm_ws/isaacsim_test/outputs/robot_arm_hand_graspable_20260703_162039_KST_motor_faithful_finger_frame_runtime_retry/robot_arm_hand_connected_report.json`
- Screenshots: `/home/sim/Documents/superarm_ws/isaacsim_test/artifacts/visual_verification_20260703_162039_KST_motor_faithful_finger_frame_runtime_retry`
- Result: `runtime_validation.status=PASS`; `grasp_validation=PASS`; `preshape_grasp_validation=PASS`; `finger_motion_validation=PASS`; `lift_retain_validation=PASS`.
- Preshape reset evidence: single_finger reset drift `0.0168 m`, pinch `0.0217 m`, wrap `0.0264 m`, all below reset-stability guard and no reset retry required.
- Wrap finger3 evidence: target errors improved to approximately `0.00118` and `0.00354` rad in the retry run.
