# HITL Bring-up Notes

HITL work starts after SITL/LeLab/Isaac Sim validation and before any real policy or dataset collection on hardware.

Current focused branch:

- `feature/hitl-dm4340p-canable-readiness`
- Main gate document: [`2026-07-09_dm4340p_canable_hitl_readiness.md`](2026-07-09_dm4340p_canable_hitl_readiness.md)

Safety default: no motor enable, torque, or motion until the checklist explicitly reaches the tiny-motion gate.


No-motion tools:

- `tools/hitl/passive_canable_check.py`: opens CANable through `slcan` and passively listens without transmitting CAN frames.
- `tools/hitl/read_only_dm4340p_inspect.py`: read-only Gate 2 inspector; records non-error arbitration IDs/status bytes if unsolicited frames are visible, while still sending no CAN frames, no torque enable, and no motion command.


Current no-motion backend/config slice:

- Protocol/backend note: [`2026-07-10_dm4340p_read_only_backend_protocol_evidence.md`](2026-07-10_dm4340p_read_only_backend_protocol_evidence.md)
- Read-only config: `configs/hitl/dm4340p_x2_read_only.json`
- Safety backend: `tools/hitl/read_only_backend.py`


Milestone C-F dry-run preparation:

- Dry-run status note: [`2026-07-10_dm4340p_milestones_c_to_f_dry_run.md`](2026-07-10_dm4340p_milestones_c_to_f_dry_run.md)
- Protocol planning helper: `tools/hitl/dm4340p_protocol.py`
- Gate dry-run runner: `tools/hitl/dm4340p_gate_runner.py`


Milestone G-H wrap-up:

- Wrap-up note: [`2026-07-10_dm4340p_milestones_g_to_h_wrap_up.md`](2026-07-10_dm4340p_milestones_g_to_h_wrap_up.md)
- Dry-run commands: `lelab-integration-plan`, `policy-readiness-plan`
