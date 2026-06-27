# CAD to SimReady Preflight

- Status: `ready`
- Platform: `linux`
- Manifest: `/home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/cad-to-simready-content-agents-preflight.json`

## Runtimes

- `content_agents`: `ready` - Content Agents services are healthy
- `git_lfs`: `ready` - Git LFS is available
- `repo_python`: `skipped` - no pyproject.toml found near cwd or preflight script
- `request`: `ready` - request inputs are ready

## Services

- `material`: `ready` - http://localhost:8100
- `ovrtx`: `ready` - http://localhost:8001
- `physics`: `ready` - http://localhost:8200

## Blockers

- None
