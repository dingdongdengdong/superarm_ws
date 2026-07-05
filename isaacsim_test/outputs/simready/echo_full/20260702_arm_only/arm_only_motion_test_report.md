# Echo Full Arm-only Isaac Sim Motion Test

- Status: PASS (automated runtime + pixel validation; visual inspection pending below)
- Date folder: `isaacsim_test/outputs/simready/echo_full/20260702_arm_only`
- Runtime log: `isaacsim_test/artifacts/runtime_logs/arm_only_motion_20260702T150302Z.log`
- Contact sheet: `isaacsim_test/outputs/simready/echo_full/20260702_arm_only/arm_only_motion_contact_sheet.png`
- Screenshot count: 5
- Loaded DOFs: 5 / right_arm_pitch_joint, right_arm_roll_joint, right_arm_yaw_joint, right_elbow_pitch_joint, right_elbow_yaw_joint
- Hand scope: intentionally omitted; hand motor control `SKIPPED`; wrist check `SKIPPED`.

## Motion cases

| # | name | command | screenshot | size | avg stddev | RMS vs home |
|---|------|---------|------------|------|------------|-------------|
| 1 | `home_zero` | `+0.00, +0.00, +0.00, +0.00, +0.00` | `isaacsim_test/outputs/simready/echo_full/20260702_arm_only/motion_screenshots/01_home_zero.png` | 579240 | 63.066 | None |
| 2 | `reach_forward_left` | `+0.35, -0.25, +0.40, -0.55, +0.30` | `isaacsim_test/outputs/simready/echo_full/20260702_arm_only/motion_screenshots/02_reach_forward_left.png` | 768052 | 35.008 | 72.882 |
| 3 | `elbow_fold_high` | `+0.15, +0.10, +0.00, -0.60, +0.45` | `isaacsim_test/outputs/simready/echo_full/20260702_arm_only/motion_screenshots/03_elbow_fold_high.png` | 569654 | 65.262 | 40.495 |
| 4 | `side_sweep_right` | `-0.30, +0.25, -0.35, -0.45, -0.25` | `isaacsim_test/outputs/simready/echo_full/20260702_arm_only/motion_screenshots/04_side_sweep_right.png` | 592966 | 61.644 | 61.91 |
| 5 | `mixed_limit_safe` | `+0.55, -0.45, -0.55, -0.20, +0.60` | `isaacsim_test/outputs/simready/echo_full/20260702_arm_only/motion_screenshots/05_mixed_limit_safe.png` | 734187 | 33.447 | 73.172 |

## Automated image checks

- Nonblank pixel-stat check: `True`.
- Pose-difference RMS check against home pose: `True`.
- Individual screenshots are preserved at `motion_screenshots/*.png`.

## Visual inspection

- Pending manual/agent image inspection of the contact sheet.
