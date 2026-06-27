# Isaac Sim 5.1 — RoboParty V2.0 Right Arm + AmazingHand Sim-in-the-Loop

Runs the official RoboParty / Roboto Origin **V2.0 right arm** from the RoboParty GitHub URDF as a simulated follower in Isaac Sim 5.1, controlled by your phone browser over WiFi.

The LeRobot side uses `robot_type=isaacsim_rpo_arm` with a 6D state/action contract:

1. `right_arm_pitch_joint.pos`
2. `right_arm_roll_joint.pos`
3. `right_arm_yaw_joint.pos`
4. `right_elbow_pitch_joint.pos`
5. `right_elbow_yaw_joint.pos`
6. `amazinghand_grasp.pos` synthetic scalar in `[0.0, 1.0]`

```
Phone browser → phone_teleop_server → /leader/joint_commands (ROS2, 6 floats)
                                              ↓
                          IsaacSimRpoArmRobot (lerobot container)
                                              ↓
                            /follower/joint_commands (ROS2, 6 floats)
                                              ↓
      setup_rpo_arm_scene.py imports official RoboParty V2.0 URDF and controls 5 right-arm joints
                                              ↓
                            /follower/joint_states → observation recording
```

---

## Prerequisites

| Requirement | Check |
|---|---|
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

```bash
# Quick defaults (bash):
export SUPERARM_WS_PATH=/home/sim/Documents/superarm_ws
export ROS_DOMAIN_ID=42
export RPO_ARM_URDF_PATH=/workspace/superarm_ws/roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf
export NUM_JOINTS=6
export JOINT_NAMES=right_arm_pitch_joint,right_arm_roll_joint,right_arm_yaw_joint,right_elbow_pitch_joint,right_elbow_yaw_joint,amazinghand_grasp
```

The default URDF is the official RoboParty V2.0 model at:

```text
roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf
```

The scene imports the full V2.0 robot and bridges only the right-arm chain. `amazinghand_grasp` is a synthetic LeRobot scalar for now; a physical AmazingHand URDF mount is a later step.

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

Wait for logs like:

```text
[setup_rpo_arm_scene] Loading RoboParty V2.0 URDF: /workspace/superarm_ws/roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf
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

Open `http://<host-ip>:8766` in your phone's browser on the same WiFi. The first five sliders are right-arm joint radians; the last slider is `amazinghand_grasp` in `[0.0, 1.0]`.

### Option B — Foxglove (visualization + publishing)

The Foxglove WebSocket bridge runs on port **8765**.

1. Open [studio.foxglove.dev](https://studio.foxglove.dev)
2. Click **Open connection** → **Foxglove WebSocket**
3. Enter: `ws://<host-ip>:8765`

Once connected you can:
- Visualize `/follower/joint_states` in a Plot or Raw Messages panel
- Add a 3D panel and load the RoboParty V2.0 URDF to see the right arm move
- Use the **Publish** panel to send a one-shot `Float64MultiArray` to `/leader/joint_commands`

| Port | Service | Use |
|------|---------|-----|
| 8765 | Foxglove WebSocket bridge | Visualization, Foxglove Studio / mobile |
| 8766 | Phone slider server | Real-time joint teleoperation |

---

## Verification gates

```bash
# Gate 1 — images present
docker images | grep "isaac-sim"

# Gate 2 — Isaac Sim started (check container logs)
docker logs isaacsim-test-sim | grep "RoboParty V2.0\|Controlled LeRobot joints\|ERROR"

# Gate 3 — ROS2 topics visible
export ROS_DOMAIN_ID=42 && source /opt/ros/humble/setup.bash
ros2 topic list

# Gate 4 — joint states publishing at ~60 Hz
ros2 topic hz /follower/joint_states

# Gate 5a — phone slider server reachable
curl http://localhost:8766 | head -5
# Gate 5b — Foxglove bridge reachable
curl -s --include --no-buffer -H "Upgrade: websocket" http://localhost:8765 | head -3

# Gate 6 — command round-trip
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
    --control.repo_id=YOUR_HF_USER/rpo_v2_right_arm_isaacsim_v1 \
    --control.single_task="Teleoperate the RoboParty V2.0 right arm and AmazingHand grasp in Isaac Sim." \
    --control.fps=30 \
    --control.num_episodes=5'
```

Dataset feature names are the five official V2 right-arm joints plus `amazinghand_grasp.pos` in LeRobot `observation.state` and `action` metadata, shape `(6,)`.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `pull_images.sh` → 401 Unauthorized | NGC not logged in | `docker login nvcr.io` (`$oauthtoken` / API key) |
| Isaac Sim exits immediately | URDF not found | Confirm `RPO_ARM_URDF_PATH` points to the V2.0 URDF under `/workspace/superarm_ws/roboparty/...` |
| No topics on `ros2 topic list` | Wrong `ROS_DOMAIN_ID` | Ensure both containers use `ROS_DOMAIN_ID=42` |
| Phone can't reach server | Different subnet or VPN | Ensure phone and host are on the same WiFi |
| Arm doesn't move | Phone server not publishing | `ros2 topic echo /leader/joint_commands` — should update when you move sliders |
| Right-arm joint missing at startup | Wrong/old URDF | Use `roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf` |
| Out of VRAM | Native Isaac Sim still running | Close the native app before starting the container |
| DDS discovery fails | Firewall blocking multicast | `sudo ufw allow in on lo` or set `FASTRTPS_DEFAULT_PROFILES_FILE` |
