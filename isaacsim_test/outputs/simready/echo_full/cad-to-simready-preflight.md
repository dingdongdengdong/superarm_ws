# CAD to SimReady Preflight

- Status: `blocked`
- Platform: `linux`
- Manifest: `/home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/cad-to-simready-preflight.json`

## Runtimes

- `content_agents`: `blocked` - NVIDIA_API_KEY is required for managed local Content Agents deployment
- `git_lfs`: `blocked` - `git-lfs` was not found on PATH
- `repo_python`: `skipped` - no pyproject.toml found near cwd or preflight script
- `request`: `ready` - request inputs are ready

## Services

- `material`: `blocked` - http://localhost:8100
- `ovrtx`: `blocked` - http://localhost:8001
- `physics`: `blocked` - http://localhost:8200

## Blockers

- content_agents: NVIDIA_API_KEY is required for managed local Content Agents deployment
- git_lfs: `git-lfs` was not found on PATH
- material: material health endpoint did not respond
- ovrtx: ovrtx health endpoint did not respond
- physics: physics health endpoint did not respond
