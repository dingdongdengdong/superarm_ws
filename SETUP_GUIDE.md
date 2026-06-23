# SuperArm WS — Dev Environment & CI/CD Guide

## What's Running on This Machine

| Component | Details |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| Kernel | 6.8.0-124-generic |
| GPU | NVIDIA GeForce RTX 4070 Ti (12 GB VRAM) |
| Driver | 580.159.03 |
| CUDA | 13.0 |
| Isaac Sim | 5.1.0 |
| Isaac Lab | 2.3.2 |
| Docker | 29.6.0 |

---

## 1. Isaac ROS Dev Container

The dev container (`isaac_ros_dev-x86_64`) is a full robotics AI environment built from NVIDIA Isaac ROS 3.2.

### What's inside

| Tool | Version |
|---|---|
| ROS 2 | Humble (406 packages) |
| PyTorch | 2.12.1 + CUDA 13.0 |
| TensorRT | 10.3.0 |
| cuDNN | 9.3.0 |
| Triton Inference Server | 6.9 GB — full ML serving runtime |
| Python | 3.10 + 400+ ML packages |

### Start / stop the container

```bash
# Start (already set up — just run this after reboot)
docker start isaac-ros-dev

# Open a shell inside
docker exec -it isaac-ros-dev bash

# Stop
docker stop isaac-ros-dev
```

### Your workspace inside the container

Your `superarm_ws` folder is mounted at `/workspaces/isaac_ros-dev` inside the container.
Any file you edit on the host is instantly visible inside the container and vice versa.

```bash
# Inside the container
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
```

### Build your ROS packages

```bash
docker exec -it isaac-ros-dev bash
# Then inside:
source /opt/ros/humble/setup.bash
cd /workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash
```

### Run ROS 2 commands

```bash
docker exec isaac-ros-dev bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"
docker exec isaac-ros-dev bash -c "source /opt/ros/humble/setup.bash && ros2 node list"
```

### Use GPU / PyTorch inside container

```bash
docker exec -it isaac-ros-dev python3 -c "import torch; print(torch.cuda.get_device_name(0))"
```

---

## 2. GitHub Actions CI/CD Pipeline

Every time you push to `main`, the pipeline runs automatically on **this machine** (self-hosted runner).

### Pipeline stages

```
git push → Build ROS packages → Run tests → Build Docker image → Push to GHCR → Run simulation
```

### Workflow file

`.github/workflows/ci.yml`

| Job | Trigger | What it does |
|---|---|---|
| `build-and-test` | Every push | `colcon build` + `colcon test` inside Isaac ROS container |
| `docker-build-push` | Push to `main` only | Builds a Docker image of your workspace and pushes to GitHub Container Registry |
| `deploy-simulation` | Push to `main` only | Pulls the new image and runs your simulation headlessly |

### Check pipeline status

```bash
gh run list --repo dingdongdengdong/superarm_ws
gh run watch --repo dingdongdengdong/superarm_ws   # live stream
```

### Self-hosted runner

The runner (`isaac-ros-runner`) runs as a systemd service and starts automatically on boot.

```bash
# Check status
sudo systemctl status actions.runner.dingdongdengdong-superarm_ws.isaac-ros-runner

# Restart if needed
sudo systemctl restart actions.runner.dingdongdengdong-superarm_ws.isaac-ros-runner

# View logs
journalctl -u actions.runner.dingdongdengdong-superarm_ws.isaac-ros-runner -f
```

---

## 3. Isaac Sim & Isaac Lab

### Launch Isaac Sim (GUI)

```bash
cd ~/IsaacLab
./isaaclab.sh -s
```

### Run Isaac Lab headless

```bash
cd ~/IsaacLab
./isaaclab.sh -p scripts/demos/quadrupeds.py --headless
```

### Open Isaac Sim from Docker

```bash
docker exec -it isaac-sim bash
```

---

## 4. Daily Workflow

```bash
# 1. Make changes to your code in superarm_ws/
# 2. Push to GitHub
git add .
git commit -m "your message"
git push

# 3. CI/CD runs automatically — check it with:
gh run list

# 4. If you need to test manually inside the container:
docker exec -it isaac-ros-dev bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch your_package your_launch_file.launch.py
```

---

## 5. After Reboot

Everything is set to auto-start except the container. Run this after reboot:

```bash
docker start isaac-ros-dev
```

The GitHub Actions runner starts automatically via systemd.

---

## 6. Docker Images on This Machine

| Image | Size | Purpose |
|---|---|---|
| `isaac_ros_dev-x86_64:latest` | 57.4 GB | Isaac ROS 3.2 dev environment (your CI/CD env) |
| `nvcr.io/nvidia/isaac-sim:5.1.0` | 22.9 GB | Isaac Sim standalone |
| `nvidia/cuda:12.4.1-base-ubuntu22.04` | 348 MB | CUDA base layer |
| `nvidia/cuda:12.2.0-base-ubuntu22.04` | 341 MB | CUDA base layer |
| `ubuntu:latest` | 160 MB | Base Ubuntu |
