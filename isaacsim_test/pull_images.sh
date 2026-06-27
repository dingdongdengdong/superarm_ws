#!/usr/bin/env bash
set -euo pipefail

IMAGES=(
    "nvcr.io/nvidia/isaac-sim:5.1.0"
    "nvcr.io/nvidia/isaac-sim:6.0.0"
)

echo "================================================================"
echo "  Isaac Sim Image Pull Script"
echo "  Images: 4.5.0 (~22 GB) + 6.0.0 (~25 GB)"
echo "  Ensure you have at least 55 GB of free disk space."
echo "================================================================"
echo ""

# Check NGC login
if ! grep -q "nvcr.io" "${HOME}/.docker/config.json" 2>/dev/null; then
    echo "ERROR: Not logged in to nvcr.io."
    echo ""
    echo "Run the following to log in:"
    echo "  docker login nvcr.io"
    echo "  Username: \$oauthtoken"
    echo "  Password: <your NGC API key from https://ngc.nvidia.com > Setup > API Key>"
    echo ""
    exit 1
fi

# Check available disk space (need ~55 GB)
AVAIL_GB=$(df -BG . | awk 'NR==2 {gsub("G",""); print $4}')
if [ "${AVAIL_GB}" -lt 55 ]; then
    echo "WARNING: Only ${AVAIL_GB} GB available. Recommended: 55+ GB."
    read -r -p "Continue anyway? [y/N] " confirm
    [[ "${confirm}" =~ ^[Yy]$ ]] || exit 1
fi

for IMAGE in "${IMAGES[@]}"; do
    echo ""
    echo "--- ${IMAGE} ---"
    if docker image inspect "${IMAGE}" &>/dev/null; then
        echo "Already present, skipping pull."
    else
        echo "Pulling (this may take a while)..."
        docker pull "${IMAGE}"
    fi
done

echo ""
echo "================================================================"
echo "  Pulled images:"
echo "================================================================"
docker images --format "table {{.Repository}}:{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}" \
    | grep -E "REPO|isaac-sim:(4\.5\.0|6\.0\.0)"

echo ""
echo "  Version strings:"
for IMAGE in "${IMAGES[@]}"; do
    VERSION=$(docker run --rm --entrypoint /bin/bash "${IMAGE}" \
        -c "cat /isaac-sim/VERSION 2>/dev/null || cat /VERSION 2>/dev/null || echo unknown" 2>/dev/null || echo "unknown")
    echo "  ${IMAGE} -> ${VERSION}"
done

echo ""
echo "Done. Test with Isaac Sim 4.5.0:"
echo "  cd $(dirname "$0") && docker compose up isaac-sim-45"
