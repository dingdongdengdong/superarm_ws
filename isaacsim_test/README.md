# Isaac Sim 5.1 — RoboParty Arm + AmazingHand Sim-in-the-Loop

Runs the RoboParty 5-DOF arm + AmazingHand as a simulated follower in Isaac Sim 5.1, controlled by your phone browser over WiFi.
The LeRobot side uses `robot_type=isaacsim_rpo_arm` with the same joint names and flat feature keys as the real hardware,
so datasets and policies are directly transferable to the physical Damiao arm.

```
Phone browser → phone_teleop_server → /leader/joint_commands (ROS2)
                                              ↓
                          IsaacSimRpoArmRobot (lerobot container)
                                              ↓
                            /follower/joint_commands (ROS2)
                                              ↓
                          setup_rpo_arm_scene.py (isaac-sim-51 container)
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

> **Important:** Stop any running native Isaac Sim instance before starting the container — both share the 12 GB VRAM on the RTX 4070 Ti.

---

## Step 1 — Configure environment

```bash
cp isaacsim_test/.env.example isaacsim_test/.env
# Edit .env and set SUPERARM_WS_PATH and OPENARM_URDF_PATH
```

```bash
# Quick defaults (bash):
export SUPERARM_WS_PATH=/home/sim/Documents/superarm_ws
export ROS_DOMAIN_ID=42
export OPENARM_URDF_PATH="${SUPERARM_WS_PATH}/lerobot/lerobot/robots/openarm/assets/openarm.urdf"
```

If the OpenArm URDF doesn't exist at that path, use Isaac Sim's built-in Franka as a stand-in:
```bash
# Inside the isaac-sim-51 container the Franka URDF is at:
export OPENARM_URDF_PATH=/isaac-sim/exts/omni.isaac.franka/data/urdf/robots/panda_arm_hand.urdf
# (7-DOF Panda instead of 6-DOF OpenArm, but the ROS2 bridge is identical)
```

---

## Step 2 — Pull images

```bash
cd isaacsim_test
bash pull_images.sh
```

To pull both images **in parallel** (recommended — saves time):
```bash
# Terminal 1:
docker pull nvcr.io/nvidia/isaac-sim:5.1.0
# Terminal 2:
docker pull nvcr.io/nvidia/isaac-sim:6.0.0
```

---

## Step 3 — Patch LeRobot robot registry

Add two `elif` blocks to `lerobot/lerobot/common/robot_devices/robots/utils.py`:

```python
# in make_robot_config(), before the final else:
elif robot_type == "isaacsim_rpo_arm":
    import sys as _sys, os as _os
    _sys.path.insert(0, _os.path.join(_os.path.dirname(__file__),
        "../../../../../../isaacsim_test/lerobot"))
    from isaacsim_rpo_arm_robot import IsaacSimRpoArmConfig
    return IsaacSimRpoArmConfig(**kwargs)

# in make_robot_from_config(), before the fallback else:
elif hasattr(config, "joint_state_topic") and \
        config.__class__.__name__ == "IsaacSimRpoArmConfig":
    import sys as _sys, os as _os
    _sys.path.insert(0, _os.path.join(_os.path.dirname(__file__),
        "../../../../../../isaacsim_test/lerobot"))
    from isaacsim_rpo_arm_robot import IsaacSimRpoArmRobot
    return IsaacSimRpoArmRobot(config)
```

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

Wait for:
```
[setup_rpo_arm_scene] Loaded 6 joints: ['rpo_arm_j1', 'rpo_arm_j2', 'rpo_arm_j3', 'rpo_arm_j4', 'rpo_arm_j5', 'amazinghand_grasp']
[setup_rpo_arm_scene] Simulation running.
```

---

## Step 6 — Start LeRobot + phone server + Foxglove

```bash
# In a new terminal — starts both the slider teleop server and Foxglove bridge:
docker compose up lerobot foxglove
```

Wait for:
```
Open on your phone: http://192.168.x.x:8766
```

---

## Step 7 — Connect your phone

### Option A — Custom slider UI (teleoperation)
Open `http://<host-ip>:8766` in your phone's browser (same WiFi).  
You'll see one slider per joint. Moving a slider sends joint position commands to Isaac Sim at 10 Hz.

### Option B — Foxglove (visualization + publishing)
The Foxglove WebSocket bridge runs on port **8765** (standard default).

**Web browser (desktop or phone):**
1. Open [studio.foxglove.dev](https://studio.foxglove.dev)
2. Click **Open connection** → **Foxglove WebSocket**
3. Enter: `ws://<host-ip>:8765`

**Foxglove mobile app** (iOS / Android):
1. Install [Foxglove](https://foxglove.dev/download)
2. Tap **+** → **Open connection** → **Foxglove WebSocket**
3. Enter: `ws://<host-ip>:8765`

Once connected you can:
- Visualize `/follower/joint_states` in a Plot or Raw Messages panel
- Add a 3D panel and load the URDF to see the arm move
- Use the **Publish** panel to send a one-shot `Float64MultiArray` to `/leader/joint_commands`

> **Port summary:**
> | Port | Service | Use |
> |------|---------|-----|
> | 8765 | Foxglove WebSocket bridge | Visualization, Foxglove Studio / mobile |
> | 8766 | Phone slider server | Real-time joint teleoperation |

---

## Verification gates

```bash
# Gate 1 — images present
docker images | grep "isaac-sim"

# Gate 2 — Isaac Sim started (check container logs)
docker logs isaacsim-test-sim | grep "Loaded\|joint\|ERROR"

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
  "data: [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]" --once
ros2 topic echo /follower/joint_states --once
# Expected: positions close to [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]
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
    --control.repo_id=YOUR_HF_USER/rpo_arm_isaacsim_v1 \
    --control.fps=30 \
    --control.num_episodes=5'
```

Dataset keys: `rpo_arm_j1.pos` … `rpo_arm_j5.pos` + `amazinghand_grasp.pos` (state/action shape `(6,)`).
These match the real hardware feature contract — see `integration_guide/09_isaacsim_sim_loop_plan.md`.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `pull_images.sh` → 401 Unauthorized | NGC not logged in | `docker login nvcr.io` (`$oauthtoken` / API key) |
| Isaac Sim exits immediately | URDF not found | Set `OPENARM_URDF_PATH`; use Franka fallback path |
| No topics on `ros2 topic list` | Wrong `ROS_DOMAIN_ID` | Ensure both containers use `ROS_DOMAIN_ID=42` |
| Phone can't reach server | Different subnet or VPN | Ensure phone and host are on the same WiFi |
| Arm doesn't move | Phone server not publishing | `ros2 topic echo /leader/joint_commands` — should update when you move sliders |
| URDF import fails with extension error | Extension renamed in this sim version | Script auto-tries both `omni.isaac.urdf` and `omni.importer.urdf` |
| Out of VRAM | Native Isaac Sim still running | `sudo systemctl stop isaacsim` or close the native app |
| DDS discovery fails | Firewall blocking multicast | `sudo ufw allow in on lo` or set `FASTRTPS_DEFAULT_PROFILES_FILE` |
