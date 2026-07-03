# SimReady `echo_full` SITL Runtime Evidence

Date: 2026-06-28

## Command

```bash
cd isaacsim_test
SIMREADY_USD_PATH=/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd \
  SCREENSHOT_ON_STARTUP=1 \
  SCREENSHOT_PATH=/workspace/superarm_ws/isaacsim_test/artifacts/echo_full_simready_startup.png \
  docker compose up --force-recreate --abort-on-container-exit --exit-code-from isaac-sim-51 isaac-sim-51
```

## Result

- Exit code: `0`.
- Isaac Sim loaded `/World/echo_full_simready` from `echo_full_robot_arm_hand.usd`.
- Published 6D LeRobot contract remained:
  - `right_arm_pitch_joint.pos`
  - `right_arm_roll_joint.pos`
  - `right_arm_yaw_joint.pos`
  - `right_elbow_pitch_joint.pos`
  - `right_elbow_yaw_joint.pos`
  - `amazinghand_grasp.pos`
- Generated local evidence:
  - `isaacsim_test/artifacts/simready_prim_mapping.json`
  - `isaacsim_test/artifacts/echo_full_simready_startup.png`

## Evidence Notes

- `simready_prim_mapping.json` reported `binding_status: binding_pending`, 6 control features, and 200 captured prim hierarchy entries with `prim_hierarchy_truncated: true`.
- No USD articulation/control prims were bound in this pass; the next implementation step is real prim/articulation binding.
- The container had no default window. Renderer-resource capture failed because `omni.kit.viewport_legacy` was unavailable, and viewport capture timed out. The script then saved fallback visual evidence from the committed SimReady thumbnail at `pipeline/07_render/thumbnail.png` instead of crashing in Replicator.
