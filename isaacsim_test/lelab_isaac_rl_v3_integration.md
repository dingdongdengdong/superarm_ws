# SuperArm → LeLab Isaac RL V3 Integration

This is the current SuperArm integration boundary for LeLab reinforcement
learning. It replaces the old Isaac Sim 5.1 `binding_pending` route for RL
without deleting that historical test path.

LeLab is visible as the top-level `leLab` Git submodule. The integration does
not copy another robot USD into this repository. `lelab_rl_v3.lock.json` pins:

- LeLab commit `5bcae6b6fa85497ea34346829780a0c879397c53`;
- SuperArm's edited LeRobot config
  `isaacsim_test/lerobot/source_arm_amazinghand.yaml`;
- its `isaacsim_rpo_arm_robot.py` robot and
  `superarm_action_adapter.py` SO101 mapping modules;
- the single passive-linkage/no-shell V3 distribution checksum;
- the `usd/superarm_amazinghand/superarm_amazinghand.usda` entrypoint;
- 13 physical DOFs and six logical actions;
- 88 visual-only passive followers and zero outer shells;
- real-hardware grasp maximum `0.5` (half-close);
- simulation-only full close `1.0`.

## Validate without launching

```bash
git submodule update --init leLab
export SUPERARM_ISAAC_DISTRIBUTION_ZIP=/path/to/superarm_amazinghand_isaac60_passive_linkage_no_shell_distribution_20260724_v3.zip
./isaacsim_test/run_lelab_isaac_rl_v3.sh --check-only
```

The command fails before launch if the LeLab checkout is dirty, is not exactly
at the pinned commit, if the selected SuperArm LeRobot config is not the exact
`joint_rev_1..5 + amazinghand_motion` six-control/13-physical-joint contract,
or if the archive filename, checksum, entrypoint, visual profile, joint
contract, shell exclusion, or hardware grasp boundary differs.
`LELAB_REPO` may override the top-level submodule for development, but the same
exact commit and clean-worktree checks still apply.

## Launch

```bash
./isaacsim_test/run_lelab_isaac_rl_v3.sh
```

Open `http://127.0.0.1:8000/reinforcement-learning`. LeLab performs its own
runtime readiness check again before starting Isaac, learner, and actor.
The launcher exports `SUPERARM_ASSET_ROOT` and `SUPERARM_LEROBOT_CONFIG`, so
the normal LeLab website also exposes the SuperArm-edited configuration instead
of its bundled fallback or the legacy `right_arm_*` file.

## Proof boundary

The current V3 integration has reviewed open/half-close RGB and repeated seeded
reset/300-frame hold evidence. Collider contact, scripted grasp-and-lift,
learner update, and policy improvement remain separate gates and are not
claimed by this SuperArm-side launcher.
