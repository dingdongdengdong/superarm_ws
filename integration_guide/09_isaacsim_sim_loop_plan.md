# 09 — Isaac Sim 5.1 Sim-in-the-Loop Plan (SimReady `echo_full`)

## Goal

Use the validated SimReady USD generated from the real `echo_full.step` CAD as the primary Isaac Sim test asset.

```text
Source CAD:       arm_with_hand_with_robot_file/echo_full.step
Final SimReady:  isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd
Report:          isaacsim_test/outputs/simready/echo_full/omniverse-cad-to-simready-report.md
Thumbnail:       isaacsim_test/outputs/simready/echo_full/pipeline/07_render/thumbnail.png
```

The older RoboParty V2.0 URDF remains useful as a historical joint-name and control-reference baseline, but new scene work should not design around a stand-in arm or a separate hand mount. The SimReady USD already contains the visible robot/mobile-base/arm/hand asset and passed `Prop-Robotics-Neutral` v`1.0.0` validation.

---

## LeRobot control contract

Keep the first integration pass compatible with the existing 6D LeRobot interface:

```text
right_arm_pitch_joint.pos
right_arm_roll_joint.pos
right_arm_yaw_joint.pos
right_elbow_pitch_joint.pos
right_elbow_yaw_joint.pos
amazinghand_grasp.pos
```

The task is now to map this stable command/state interface onto the imported SimReady USD prims. If the USD does not expose true articulated joints yet, the first pass should still load the SimReady asset, publish observation state, and record which prims need articulation authoring.

---

## Source geometry

Primary source:

```text
isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd
```

Supporting evidence:

```text
isaacsim_test/outputs/simready/echo_full/pipeline/06_validation_final/simready-profile.json
isaacsim_test/outputs/simready/echo_full/pipeline/07_render/thumbnail.png
```

Legacy reference only:

```text
roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf
```

---

## Implementation contract

- Add `SIMREADY_USD_PATH` to the Isaac Sim runtime environment and default it to the final SimReady USD path.
- Update `setup_rpo_arm_scene.py` to load the SimReady USD as the scene asset before any command binding work.
- Inspect the loaded USD prim hierarchy and write the chosen control/articulation mapping to `isaacsim_test/artifacts/simready_prim_mapping.json`.
- Keep ROS2 `/follower/joint_commands` accepting six floats and `/follower/joint_states` publishing the same six feature names during the first binding pass.
- If an arm/hand command cannot yet move a real USD articulation, log that as `binding_pending` in evidence JSON instead of silently falling back to a stand-in model.
- Keep generated runtime evidence under `isaacsim_test/artifacts/` so it stays out of git.

---

## Verification checklist

```bash
# Static contract
python3 isaacsim_test/test_v2_roboparty_config.py

# SimReady asset exists and profile passed
test -f isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd
python3 - <<'PY'
import json
from pathlib import Path
p = Path('isaacsim_test/outputs/simready/echo_full/pipeline/06_validation_final/simready-profile.json')
assert json.loads(p.read_text())['passed'] is True
PY

# Compose syntax
cd isaacsim_test && docker compose config >/tmp/isaacsim-compose.yml

# Isaac Sim startup target after scene update
SIMREADY_USD_PATH=/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd \
  docker compose up isaac-sim-51
# expect: Loading SimReady USD ... echo_full_robot_arm_hand.usd
# expect: Controlled LeRobot joints: ['right_arm_pitch_joint', ..., 'amazinghand_grasp']

# Command round-trip
ros2 topic pub /leader/joint_commands std_msgs/msg/Float64MultiArray \
  "data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.5]" --once
ros2 topic echo /follower/joint_states --once
```

---

## Next work

- Add `SIMREADY_USD_PATH` to `.env.example` and Docker Compose.
- Load the SimReady USD in Isaac Sim and save a startup screenshot.
- Export the SimReady prim hierarchy and choose the initial control/articulation mapping.
- Bind the existing 6D ROS2/LeRobot command interface to that mapping or record explicit `binding_pending` evidence for prims that need articulation authoring.
- Rerun the LeRobot SITL verifier and update the evidence artifacts.
