#!/usr/bin/env bash
# Self-hosted GitHub Actions runner setup for Isaac ROS
# Run this ONCE on the host machine (not inside container)
# Usage: ./setup-runner.sh <GITHUB_TOKEN> [runner-name]

set -e

REPO="dingdongdengdong/superarm_ws"
TOKEN="${1:?Usage: $0 <GITHUB_REGISTRATION_TOKEN> [runner-name]}"
RUNNER_NAME="${2:-isaac-ros-runner}"
RUNNER_DIR="$HOME/actions-runner"
RUNNER_VERSION="2.319.1"

echo "==> Creating runner directory: $RUNNER_DIR"
mkdir -p "$RUNNER_DIR" && cd "$RUNNER_DIR"

echo "==> Downloading GitHub Actions runner v${RUNNER_VERSION}"
curl -sL "https://github.com/actions/runner/releases/download/v${RUNNER_VERSION}/actions-runner-linux-x64-${RUNNER_VERSION}.tar.gz" \
  | tar xz

echo "==> Configuring runner for $REPO"
./config.sh \
  --url "https://github.com/$REPO" \
  --token "$TOKEN" \
  --name "$RUNNER_NAME" \
  --labels "self-hosted,linux,gpu,isaac-ros" \
  --work "_work" \
  --unattended

echo "==> Installing runner as systemd service"
sudo ./svc.sh install "$USER"
sudo ./svc.sh start

echo ""
echo "Runner '$RUNNER_NAME' is now active."
echo "Check status: sudo $RUNNER_DIR/svc.sh status"
