# Conversion Report

- Source asset: `/home/sim/Documents/superarm_ws/arm_with_hand_with_robot_file/echo_full.step`
- Source format: `cad`
- Converter skill: `usd-convert-cad`
- Converter tool: `usd-convert-cad`
- Converter command: `/home/sim/.physical-ai-skill-hub/venvs/simready-validate/bin/python3 /home/sim/.physical-ai-skill-hub/upstreams/usd-convert-cad/convert.py /home/sim/Documents/superarm_ws/arm_with_hand_with_robot_file/echo_full.step /home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/01_conversion/echo_full.usd --report /home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/01_conversion/echo_full_usd_convert_cad_status.json --quiet --log /home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/01_conversion/echo_full_usd_convert_cad_status.log`
- Output directory: `/home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/01_conversion`
- Output USD: `/home/sim/Documents/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/01_conversion/echo_full.usd`
- Next step: `validate-usd-minimum`

## Generated Files

- `echo_full.usd`
- `echo_full_usd_convert_cad_status.json`
- `echo_full_usd_convert_cad_status.log`
- `echo_full_usd_convert_cad_validate.log`

## Warnings

- Delegating CAD conversion to upstream usd-convert-cad: https://github.com/NVIDIA-Omniverse/usd-convert-cad.
- Upstream agent skill reference: https://github.com/NVIDIA-Omniverse/usd-convert-cad/blob/main/.agents/skills/usd-convert-cad/SKILL.md.
- Router selected `usd-convert-cad` from upstream converter capability probes.

## Errors

- None
