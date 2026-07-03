# material-agent-client Report

- Asset: `/home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/01_conversion/echo_full.usd`
- Agent: `material-agent`
- Passed: `True`
- Status: `PASS`
- Session: `322020f8-39fc-411a-96b0-84060d3096e2`
- Output USD: `/home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/03_content_agents/material/echo_full_material.usd`
- Next step: `physics-agent-client`

## Checks

- `PASS` `preflight_material-agent-client_ready`: Preflight manifest: /home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/cad-to-simready-preflight-full-ready.json; material status: ready
- `PASS` `asset_exists`: Asset path exists
- `PASS` `supported_usd_extension`: Asset uses a supported USD extension
- `PASS` `material_optimizer_default_enabled`: Using Material Agent optimize_usd path after USD topology inspection
- `PASS` `base_url_available`: Using Content Agents service http://localhost:8100
- `PASS` `base_url_from_env_base_url`: Resolved base URL from env_base_url
- `PASS` `upload_asset_single_file`: No external USD dependencies detected for upload
- `PASS` `session_started`: Started Content Agents session 322020f8-39fc-411a-96b0-84060d3096e2
- `PASS` `session_completed`: Content Agents session status: completed
- `PASS` `material_output_cleanup_noop`: No unbound material subtrees with broken sourceAsset shader references were found
- `PASS` `required_artifacts_downloaded`: Downloaded required output artifact

## Artifacts

- `materialized_usd`: downloaded `/home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/03_content_agents/material/echo_full_material.usd`
- `predictions`: downloaded `/home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/03_content_agents/material/echo_full_material_predictions.jsonl`
- `report`: downloaded `/home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/03_content_agents/material/echo_full_material_report.html`

## Material Output Cleanup

- Removed stale material count: `0`
- Repaired bound shader count: `0`
