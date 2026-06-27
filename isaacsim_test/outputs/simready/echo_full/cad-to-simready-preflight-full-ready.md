# CAD to SimReady Preflight

- Status: `ready`
- Platform: `linux`
- Manifest: `/home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/cad-to-simready-preflight-full-ready.json`

## Runtimes

- `asset_validator`: `ready` - omni_asset_validate CLI is on PATH
- `content_agents`: `ready` - Content Agents services are already healthy
- `git_lfs`: `ready` - Git LFS is available
- `openusd_python`: `ready` - OpenUSD Python APIs are importable
- `repo_python`: `skipped` - no pyproject.toml found near cwd or preflight script
- `request`: `ready` - request inputs are ready
- `simready_validate`: `ready` - simready-validate executable is on PATH
- `usd_convert_cad`: `ready` - usd-convert-cad is installed and validated

## Services

- `material`: `ready` - http://localhost:8100
- `ovrtx`: `ready` - http://localhost:8001
- `physics`: `ready` - http://localhost:8200

## Blockers

- None
