#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "$script_dir/.." && pwd)
lelab_repo=${LELAB_REPO:-"$repo_root/leLab"}
: "${SUPERARM_ISAAC_DISTRIBUTION_ZIP:?Set SUPERARM_ISAAC_DISTRIBUTION_ZIP to the V3 archive}"

python3 "$script_dir/lelab_rl_v3_integration.py" \
  --lelab-repo "$lelab_repo" \
  --distribution-zip "$SUPERARM_ISAAC_DISTRIBUTION_ZIP"

if [[ "${1:-}" == "--check-only" ]]; then
  exit 0
fi

command -v uv >/dev/null 2>&1 || {
  echo "uv is required to launch the pinned LeLab runtime" >&2
  exit 2
}

cd "$lelab_repo"
exec uv run lelab --no-open "$@"
