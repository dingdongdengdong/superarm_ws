# SuperArm LeLab Isaac RL V3 integration

## Decision

SuperArm consumes the verified LeLab RL runtime from the visible top-level
`leLab` Git submodule instead of copying another USD into this repository. The
integration lock is `isaacsim_test/lelab_rl_v3.lock.json`.

Pinned inputs:

- LeLab commit: `a336c943dd821fe2e554ff6e864fde9f72470a0a`
- V3 archive SHA-256:
  `c356d1157318b72532b82d73270ef06b5b11ed5b8a90641ea4e431941e4554f7`
- USD entrypoint: `usd/superarm_amazinghand/superarm_amazinghand.usda`
- visual profile: `superarm_isaac60_passive_linkage_no_shell/v1`
- 13 physical DOFs, six logical actions, 88 visual-only passive followers,
  and zero outer shells
- real-hardware grasp maximum `0.5`; full close `1.0` remains simulation-only

## Operator boundary

`isaacsim_test/run_lelab_isaac_rl_v3.sh` validates both pinned inputs before
launch. It defaults to the top-level `leLab` submodule and requires an explicit
`SUPERARM_ISAAC_DISTRIBUTION_ZIP`, so it cannot silently fall back to an older
local asset. `LELAB_REPO` remains an optional development override and is still
subject to the exact clean-commit check.

Use `--check-only` to validate the boundary without launching LeLab.

## Proof status

The imported LeLab result has reviewed open/half-close RGB and repeated seeded
reset/300-frame hold evidence. Contact, scripted grasp-and-lift, learner update,
and policy improvement remain unproven gates.
