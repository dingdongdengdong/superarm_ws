# Isaac Sim 5.1 — SimReady `echo_full` Arm + Hand Sim-in-the-Loop

> **Current LeLab RL path:** use the pinned Isaac Sim 6.0 V3 integration in
> [`lelab_isaac_rl_v3_integration.md`](lelab_isaac_rl_v3_integration.md).
> The 5.1 `echo_full` material below is retained as historical SITL evidence,
> not as the RL distribution.

The current source of truth for Isaac Sim work is the validated SimReady USD produced from the real CAD file:

| Artifact | Path |
|---|---|
| Source CAD | `arm_with_hand_with_robot_file/echo_full.step` |
| Final SimReady USD | `isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd` |
| Conversion / validation report | `isaacsim_test/outputs/simready/echo_full/omniverse-cad-to-simready-report.md` |
| Render thumbnail | `isaacsim_test/outputs/simready/echo_full/pipeline/07_render/thumbnail.png` |

The earlier RoboParty V2.0 URDF path remains a **legacy control-reference baseline** for joint names and the existing ROS2/LeRobot bridge. New simulation scene work should load the SimReady USD first, then bind its useful arm/hand prims to the ROS2 and LeRobot control contract.

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
      Isaac Sim scene loads the SimReady USD and maps commands onto selected arm/hand prims
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
export SIMREADY_USD_PATH=/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd
export NUM_JOINTS=6
export JOINT_NAMES=right_arm_pitch_joint,right_arm_roll_joint,right_arm_yaw_joint,right_elbow_pitch_joint,right_elbow_yaw_joint,amazinghand_grasp
```

Legacy URDF baseline, kept only for existing bridge tests until the SimReady USD binding replaces it:

```bash
export RPO_ARM_URDF_PATH=/workspace/superarm_ws/roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf
```

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

The current LeRobot interface remains a 6D command/state bridge while the Isaac scene moves from the legacy URDF import to the SimReady USD import.

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

Current bridge logs may still mention the legacy URDF until the next implementation phase lands. The target log after the SimReady scene update should include:

```text
[setup_rpo_arm_scene] Loading SimReady USD: /workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd
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

# Gate 3 — images present
docker images | grep "isaac-sim"

# Gate 4 — Isaac Sim started (check container logs)
docker logs isaacsim-test-sim | grep "SimReady\|Controlled LeRobot joints\|ERROR"

# Gate 5 — ROS2 topics visible
export ROS_DOMAIN_ID=42 && source /opt/ros/humble/setup.bash
ros2 topic list

# Gate 6 — joint states publishing at ~60 Hz
ros2 topic hz /follower/joint_states

# Gate 7a — phone slider server reachable
curl http://localhost:8766 | head -5
# Gate 7b — Foxglove bridge reachable
curl -s --include --no-buffer -H "Upgrade: websocket" http://localhost:8765 | head -3

# Gate 8 — command round-trip
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

## Arm-only fixed-hand branch

For this branch, move only the five RoboParty right-arm joints and keep the
AmazingHand fixed. Use:

```text
isaacsim_test/lerobot/rpo_arm_isaacsim_arm_only.yaml
```

That config exposes exactly five LeRobot action/state keys:

1. `right_arm_pitch_joint.pos`
2. `right_arm_roll_joint.pos`
3. `right_arm_yaw_joint.pos`
4. `right_elbow_pitch_joint.pos`
5. `right_elbow_yaw_joint.pos`

It deliberately omits `amazinghand_grasp`; `fixed_hand: true` and
`fixed_grasp: 0.0` document that the hand should not be commanded on this
branch.

To run the fixed-hand arm capture with timestamped artifacts:

```bash
SUPERARM_WS_PATH=$PWD \
ROS_DOMAIN_ID=42 \
bash isaacsim_test/run_fixed_hand_arm_lerobot_capture.sh
```

Each run creates:

```text
isaacsim_test/artifacts/arm_fixed_hand_lerobot_<UTC_DATE_TIME>/
  logs/
  screenshots/
  data/
  report.json
  report.md
```

Use `TARGET=0.1,0.0,-0.1,0.2,-0.2` to override the default five-joint arm
target. Do not include a sixth hand/grasp value for this branch.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `pull_images.sh` → 401 Unauthorized | NGC not logged in | `docker login nvcr.io` (`$oauthtoken` / API key) |
| SimReady USD not found | Conversion artifact missing or path typo | Confirm `SIMREADY_USD_PATH` points to `isaacsim_test/outputs/simready/echo_full/.../echo_full_robot_arm_hand.usd` |
| Isaac Sim still logs URDF import | Scene setup has not yet been updated to load the SimReady USD | Implement the next SITL task: import SimReady USD and bind controls |
| No topics on `ros2 topic list` | Wrong `ROS_DOMAIN_ID` | Ensure both containers use `ROS_DOMAIN_ID=42` |
| Phone can't reach server | Different subnet or VPN | Ensure phone and host are on the same WiFi |
| Arm doesn't move | Phone server not publishing, or USD control binding missing | `ros2 topic echo /leader/joint_commands`; then verify SimReady prim binding |
| Out of VRAM | Native Isaac Sim still running | Close the native app before starting the container |
| DDS discovery fails | Firewall blocking multicast | `sudo ufw allow in on lo` or set `FASTRTPS_DEFAULT_PROFILES_FILE` |
