# ovrtx-render-service Report

- Asset: `/home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd`
- Output image: `/home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/07_render/thumbnail.png`
- Renderer endpoint kind: `local-service`
- Renderer auth mode: `none`
- Passed: `True`
- Next step: `inspect-render-output`

## Checks

- `PASS` `asset_exists`: Asset path exists
- `PASS` `supported_usd_extension`: Asset uses a supported USD extension
- `PASS` `render_endpoint_available`: Using renderer endpoint http://localhost:8001/render
- `PASS` `render_endpoint_from_cli`: Resolved renderer endpoint from cli
- `PASS` `render_token_not_required`: Renderer endpoint does not require a bearer token before request
- `PASS` `openusd_stage_opened`: USD stage opened
- `PASS` `renderable_meshes_found`: Renderable mesh prims found
- `PASS` `render_stage_prepared`: Prepared composition-preserving, camera-fit render stage
- `PASS` `renderer_returned_png`: Renderer returned PNG data
- `PASS` `output_png_written`: Wrote /home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/07_render/thumbnail.png
- `FAIL` `output_png_pixel_inspected`: Pillow is unavailable: No module named 'PIL'

## Warnings

- Pillow is unavailable: No module named 'PIL'
