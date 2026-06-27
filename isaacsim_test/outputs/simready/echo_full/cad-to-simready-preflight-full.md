# CAD to SimReady Preflight

- Status: `blocked`
- Platform: `linux`
- Manifest: `/home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/cad-to-simready-preflight-full.json`

## Runtimes

- `asset_validator`: `blocked` - omni_asset_validate CLI and omni.asset_validator module are unavailable
- `content_agents`: `ready` - Content Agents services are already healthy
- `git_lfs`: `ready` - Git LFS is available
- `openusd_python`: `blocked` - OpenUSD Python APIs are not importable
- `repo_python`: `skipped` - no pyproject.toml found near cwd or preflight script
- `request`: `ready` - request inputs are ready
- `simready_validate`: `ready` - simready-validate was installed into the preflight venv
- `usd_convert_cad`: `blocked` - usd-convert-cad install or validation failed

## Services

- `material`: `ready` - http://localhost:8100
- `ovrtx`: `ready` - http://localhost:8001
- `physics`: `ready` - http://localhost:8200

## Blockers

- asset_validator: omni_asset_validate CLI and omni.asset_validator module are unavailable
- openusd_python: OpenUSD Python APIs are not importable
- usd_convert_cad: usd-convert-cad install or validation failed
