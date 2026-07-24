# SuperArm → LeLab Isaac RL V3 Integration

This is the current SuperArm integration boundary for LeLab reinforcement
learning. It replaces the old Isaac Sim 5.1 `binding_pending` route for RL
without deleting that historical test path.

The integration deliberately does not copy LeLab source or another robot USD
into this repository. `lelab_rl_v3.lock.json` pins:

- LeLab commit `a336c943dd821fe2e554ff6e864fde9f72470a0a`;
- the single passive-linkage/no-shell V3 distribution checksum;
- the `usd/superarm_amazinghand/superarm_amazinghand.usda` entrypoint;
- 13 physical DOFs and six logical actions;
- 88 visual-only passive followers and zero outer shells;
- real-hardware grasp maximum `0.5` (half-close);
- simulation-only full close `1.0`.

## Validate without launching

```bash
export LELAB_REPO=/path/to/verified/leLab
export SUPERARM_ISAAC_DISTRIBUTION_ZIP=/path/to/superarm_amazinghand_isaac60_passive_linkage_no_shell_distribution_20260724_v3.zip
./isaacsim_test/run_lelab_isaac_rl_v3.sh --check-only
```

The command fails before launch if the LeLab checkout is dirty, is not exactly
at the pinned commit, or if the archive filename, checksum, entrypoint, visual
profile, joint contract, shell exclusion, or hardware grasp boundary differs.

## Launch

```bash
./isaacsim_test/run_lelab_isaac_rl_v3.sh
```

Open `http://127.0.0.1:8000/reinforcement-learning`. LeLab performs its own
runtime readiness check again before starting Isaac, learner, and actor.

## Proof boundary

The current V3 integration has reviewed open/half-close RGB and repeated seeded
reset/300-frame hold evidence. Collider contact, scripted grasp-and-lift,
learner update, and policy improvement remain separate gates and are not
claimed by this SuperArm-side launcher.
