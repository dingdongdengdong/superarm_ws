# Isaac Sim 5.1 — Custom Visual USD + Direct Arm/Hand URDF SITL

The current Isaac Sim scene is hybrid: the user's custom USDA remains the visual
arm + hand + frame assembly, while the physical articulation is imported
directly from the generated arm+hand URDF.  The URDF starts with the exact
Roboto V2 right-arm chain and attaches J5 (`right_elbow_yaw_link`) to the
AmazingHand wrist/root. The fixed custom frame stays visual/fixed and is not
part of the movable arm chain.

| Artifact | Path |
|---|---|
| Source CAD | `arm_with_hand_with_robot_file/echo_full.step` |
| Source SimReady visual USD | `isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd` |
| Custom visual USD loaded with the physical URDF | `isaacsim_test/outputs/simready/echo_full/sitl/echo_full_lerobot_articulation.usda` |
| Roboto V2 physical reference URDF | `roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf` |
| Generated physical arm+hand URDF imported by Isaac Sim | `isaacsim_test/outputs/simready/echo_full/sitl/roboto_v2_right_arm_amazinghand_full.urdf` |
| Direct-URDF articulation report | `isaacsim_test/outputs/simready/echo_full/sitl/echo_full_lerobot_articulation_report.json` |
| Runtime motion screenshots | `isaacsim_test/artifacts/simready_motion_cases/` |
| Conversion / validation report | `isaacsim_test/outputs/simready/echo_full/omniverse-cad-to-simready-report.md` |
| Render thumbnail | `isaacsim_test/outputs/simready/echo_full/pipeline/07_render/thumbnail.png` |

The generated physical URDF copies the Roboto V2 right-arm joints unchanged,
removes torso/base motion from the arm model, then fixes the AmazingHand root to
`right_elbow_yaw_link`. The loader imports that URDF with
`URDFParseAndImportFile` and separately references the custom visual USDA under
`/World/echo_full_visual`. The current LeRobot contract still controls only the
five arm DOFs plus the synthetic `amazinghand_grasp` scalar; detailed hand finger
DOF control is deferred until that control contract is expanded.

---

## Current LeRobot control contract

Keep the first working control surface stable while the USD articulation binding is added:

1. `right_arm_pitch_joint.pos`
2. `right_arm_roll_joint.pos`
3. `right_arm_yaw_joint.pos`
4. `right_elbow_pitch_joint.pos`
5. `right_elbow_yaw_joint.pos`
6. `amazinghand_grasp.pos`

```
Phone browser → phone_teleop_server → /leader/joint_commands (ROS2, 6 floats)
                                              ↓
                          IsaacSimRpoArmRobot (lerobot container)
                                              ↓
                            /follower/joint_commands (ROS2, 6 floats)
                                              ↓
      Isaac Sim loads custom visual USDA + imports physical arm/hand URDF
                                              ↓
                            /follower/joint_states → observation recording
```

> Next SITL roadmap/checklist: `docs/sitl/2026-06-27/README.md`.

---

## Prerequisites

| Requirement | Check |
|---|---|
| SimReady USD generated | `test -f isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd` |
| NGC account + API key | https://ngc.nvidia.com → Setup → API Key |
| ~55 GB free disk | `df -h .` |
| NVIDIA Container Toolkit | `sudo apt install nvidia-container-toolkit && sudo systemctl restart docker` |
| Docker 29+ | `docker --version` |
| RTX GPU, driver ≥ 535 | `nvidia-smi` |

> **Important:** Stop any running native Isaac Sim instance before starting the container — both share GPU memory.

---

## Step 1 — Configure environment

```bash
cp isaacsim_test/.env.example isaacsim_test/.env
# Edit .env and set SUPERARM_WS_PATH if your checkout path differs
```

Recommended current defaults:

```bash
export SUPERARM_WS_PATH=/home/sim/Documents/superarm_ws
export ROS_DOMAIN_ID=42
export USE_SIMREADY_USD=0
export USE_SIMREADY_ARTICULATION_USD=0
export LOAD_CUSTOM_VISUAL_USD=1
export CUSTOM_VISUAL_USD_PATH=/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/sitl/echo_full_lerobot_articulation.usda
export CUSTOM_VISUAL_PRIM_PATH=/World/echo_full_visual
export PHYSICAL_ROBOT_URDF_PATH=/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/sitl/roboto_v2_right_arm_amazinghand_full.urdf
export SIMREADY_USD_PATH=/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd
export SIMREADY_ARTICULATION_USD_PATH=/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/sitl/echo_full_lerobot_articulation.usda
export NUM_JOINTS=6
export JOINT_NAMES=right_arm_pitch_joint,right_arm_roll_joint,right_arm_yaw_joint,right_elbow_pitch_joint,right_elbow_yaw_joint,amazinghand_grasp
```

If the arm or hand source changes, regenerate
`roboto_v2_right_arm_amazinghand_full.urdf` from the Roboto V2 right-arm URDF
and AmazingHand hand URDF/MJCF before running Isaac Sim.

---

## Step 2 — Pull images

```bash
cd isaacsim_test
bash pull_images.sh
```

To pull both images **in parallel**:

```bash
# Terminal 1:
docker pull nvcr.io/nvidia/isaac-sim:5.1.0
# Terminal 2:
docker pull nvcr.io/nvidia/isaac-sim:6.0.0
```

---

## Step 3 — LeRobot robot registry

The local checkout is patched to register `robot.type=isaacsim_rpo_arm` and load `isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py`.

The current LeRobot interface remains a 6D command/state bridge. The first five
channels drive the Roboto V2 right-arm URDF joints; `amazinghand_grasp` is
published as a synthetic scalar until a real AmazingHand finger-DOF contract is
introduced.

All runtime command entry points now use the same 6D contract helper:
arm joint targets are clamped to the generated URDF limits, and
`amazinghand_grasp` is clamped to `[0.0, 1.0]`. For screenshot/evidence runs,
the scene also records the eight AmazingHand servo targets implied by the grasp
scalar, but those servo targets are not yet published as raw LeRobot action
features.

---

## Step 4 — Allow X11 (headed mode only)

```bash
xhost +local:docker   # only needed if HEADLESS=0 in .env
```

---

## Step 5 — Start Isaac Sim 5.1

```bash
cd isaacsim_test
docker compose up isaac-sim-51
```

The default physical-scene log should include:

```text
[setup_rpo_arm_scene] Loading custom visual USD: /workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/sitl/echo_full_lerobot_articulation.usda -> /World/echo_full_visual
[setup_rpo_arm_scene] Loading physical robot URDF: /workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/sitl/roboto_v2_right_arm_amazinghand_full.urdf
[setup_rpo_arm_scene] Imported articulation prim: /roboto_v2_right_arm_amazinghand_full/root_joint
[setup_rpo_arm_scene] Loaded ... total URDF joints: [...]
[setup_rpo_arm_scene] Controlled LeRobot joints: ['right_arm_pitch_joint', 'right_arm_roll_joint', 'right_arm_yaw_joint', 'right_elbow_pitch_joint', 'right_elbow_yaw_joint', 'amazinghand_grasp']
[setup_rpo_arm_scene] Simulation running.
```

---

## Step 6 — Start LeRobot + phone server + Foxglove

```bash
# In a new terminal — starts both the slider teleop server and Foxglove bridge:
docker compose up lerobot foxglove
```

Wait for:

```text
Open on your phone: http://192.168.x.x:8766
```

---

## Step 7 — Connect your phone

### Option A — Custom slider UI (teleoperation)

Open `http://<host-ip>:8766` in your phone's browser on the same WiFi. The first five sliders are arm joint command values; the last slider is `amazinghand_grasp` in `[0.0, 1.0]`.

### Option B — Foxglove (visualization + publishing)

The Foxglove WebSocket bridge runs on port **8765**.

1. Open [studio.foxglove.dev](https://studio.foxglove.dev)
2. Click **Open connection** → **Foxglove WebSocket**
3. Enter: `ws://<host-ip>:8765`

Once connected you can:
- Visualize `/follower/joint_states` in a Plot or Raw Messages panel
- Add a 3D panel and load the SimReady USD scene after the binding phase
- Use the **Publish** panel to send a one-shot `Float64MultiArray` to `/leader/joint_commands`

| Port | Service | Use |
|------|---------|-----|
| 8765 | Foxglove WebSocket bridge | Visualization, Foxglove Studio / mobile |
| 8766 | Phone slider server | Real-time joint teleoperation |

---

## Verification gates

```bash
# Gate 1 — SimReady asset exists
test -f isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd

# Gate 2 — SimReady validation passed
python3 - <<'PY'
import json
from pathlib import Path
p = Path('isaacsim_test/outputs/simready/echo_full/pipeline/06_validation_final/simready-profile.json')
data = json.loads(p.read_text())
assert data['passed'] is True, data
print('SimReady profile passed:', data['profile_target'])
PY

# Gate 3 — direct physical arm+hand URDF/report generated
python3 isaacsim_test/test_v2_roboparty_config.py

# Gate 4 — images present
docker images | grep "isaac-sim"

# Gate 5 — Isaac Sim started (check container logs)
docker logs isaacsim-test-sim | grep "Loading custom visual USD\\|Loading physical robot URDF\\|Loaded .* total URDF joints\\|Controlled LeRobot joints\\|ERROR"

# Gate 6 — four direct-URDF motion screenshots
bash isaacsim_test/run_simready_motion_screenshot_cases.sh

# Gate 7 — ROS2 topics visible
export ROS_DOMAIN_ID=42 && source /opt/ros/humble/setup.bash
ros2 topic list

# Gate 8 — joint states publishing at ~60 Hz
ros2 topic hz /follower/joint_states

# Gate 9a — phone slider server reachable
curl http://localhost:8766 | head -5
# Gate 9b — Foxglove bridge reachable
curl -s --include --no-buffer -H "Upgrade: websocket" http://localhost:8765 | head -3

# Gate 10 — command round-trip
ros2 topic pub /leader/joint_commands std_msgs/msg/Float64MultiArray \
  "data: [0.1, 0.2, 0.3, 0.0, 0.0, 0.5]" --once
ros2 topic echo /follower/joint_states --once
# Expected names: right_arm_pitch_joint ... right_elbow_yaw_joint, amazinghand_grasp
```

---

## Recording episodes

```bash
docker exec isaacsim-test-lerobot bash -c '
  source /opt/ros/humble/setup.bash &&
  cd /workspace/superarm_ws/lerobot &&
  python lerobot/scripts/control_robot.py \
    --robot.type=isaacsim_rpo_arm \
    --control.type=record \
    --control.repo_id=YOUR_HF_USER/echo_full_simready_isaacsim_v1 \
    --control.single_task="Teleoperate the SimReady echo_full robot arm and hand in Isaac Sim." \
    --control.fps=30 \
    --control.num_episodes=5'
```

Dataset feature names stay the five arm command keys plus `amazinghand_grasp.pos` in LeRobot `observation.state` and `action` metadata, shape `(6,)`, until a richer multi-DOF hand policy contract is intentionally introduced.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `pull_images.sh` → 401 Unauthorized | NGC not logged in | `docker login nvcr.io` (`$oauthtoken` / API key) |
| Generated physical URDF not found | Artifact not regenerated | Rebuild `isaacsim_test/outputs/simready/echo_full/sitl/roboto_v2_right_arm_amazinghand_full.urdf` from the Roboto V2 right-arm URDF and AmazingHand hand source |
| SimReady USD not found | Visual/provenance artifact missing or path typo | Confirm `SIMREADY_USD_PATH` points to `isaacsim_test/outputs/simready/echo_full/.../echo_full_robot_arm_hand.usd` |
| Isaac Sim logs SimReady binding instead of URDF import | `USE_SIMREADY_USD=1` enabled | Leave `USE_SIMREADY_USD=0` for the physical direct-URDF scene; only set `USE_SIMREADY_ARTICULATION_USD=1` for a real physics articulation USD, not the provenance manifest |
| No topics on `ros2 topic list` | Wrong `ROS_DOMAIN_ID` | Ensure both containers use `ROS_DOMAIN_ID=42` |
| Phone can't reach server | Different subnet or VPN | Ensure phone and host are on the same WiFi |
| Arm doesn't move | Phone server not publishing, or physical URDF did not expose the five Roboto V2 arm DOFs | `ros2 topic echo /leader/joint_commands`; then check log for `Loaded ... total URDF joints` and the five controlled arm joint names |
| Out of VRAM | Native Isaac Sim still running | Close the native app before starting the container |
| DDS discovery fails | Firewall blocking multicast | `sudo ufw allow in on lo` or set `FASTRTPS_DEFAULT_PROFILES_FILE` |
