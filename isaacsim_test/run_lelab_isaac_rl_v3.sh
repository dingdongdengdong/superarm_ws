#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "$script_dir/.." && pwd)
lelab_repo=${LELAB_REPO:-"$repo_root/leLab"}
SUPERARM_ASSET_ROOT=${SUPERARM_ASSET_ROOT:-"$repo_root"}
SUPERARM_LEROBOT_CONFIG=${SUPERARM_LEROBOT_CONFIG:-"$repo_root/isaacsim_test/lerobot/source_arm_amazinghand.yaml"}
: "${SUPERARM_ISAAC_DISTRIBUTION_ZIP:?Set SUPERARM_ISAAC_DISTRIBUTION_ZIP to the V3 archive}"
export SUPERARM_ASSET_ROOT SUPERARM_LEROBOT_CONFIG

python3 "$script_dir/lelab_rl_v3_integration.py" \
  --superarm-repo "$repo_root" \
  --lelab-repo "$lelab_repo" \
  --distribution-zip "$SUPERARM_ISAAC_DISTRIBUTION_ZIP"

if [[ "${1:-}" == "--check-only" ]]; then
  exit 0
fi

cd "$lelab_repo"
if [[ -n "${LELAB_PYTHON:-}" ]]; then
  export PYTHONPATH="$lelab_repo${PYTHONPATH:+:$PYTHONPATH}"
  exec "$LELAB_PYTHON" -m lelab.scripts.lelab --no-open "$@"
fi

command -v uv >/dev/null 2>&1 || {
  echo "uv is required to launch the pinned LeLab runtime" >&2
  exit 2
}

exec uv run lelab --no-open "$@"
