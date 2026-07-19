---
title: "LeLab MuJoCo Web LeRobot 6D Focus"
tags: ["lelab", "mujoco", "lerobot", "superarm", "amazinghand", "6d"]
created: 2026-07-19T13:34:06.965Z
updated: 2026-07-19T14:16:00.000Z
sources: []
links: []
category: architecture
confidence: medium
schemaVersion: 1
---

# LeLab MuJoCo Web LeRobot 6D Focus

# LeLab MuJoCo Web LeRobot 6D focus (2026-07-19)

## Scope and repository boundary

- Official upstream: `https://github.com/huggingface/leLab.git`.
- Focused worktree: `/home/dong/july/superarm_ws.omx-worktrees/lelab-mujoco-web-lerobot`.
- Branch: `feature/mujoco-web-lerobot`.
- This branch owns the LeLab website, direct MuJoCo runtime, LeRobot integration, and the custom SuperArm + AmazingHand asset. It contains no alternate simulator backend source.
- Normal LeLab is primary. `/superarm` is an additional MuJoCo diagnostic page, not a replacement website.

## Canonical action contract

The policy, dataset, manual leader, and SO101 adapter expose exactly six ordered features:

1. `joint_rev_1.pos`
2. `joint_rev_2.pos`
3. `joint_rev_3.pos`
4. `joint_rev_4.pos`
5. `joint_rev_5.pos`
6. `amazinghand_motion.pos`

The grasp feature is quantized to `0.0` open, `0.5` half-close, or `1.0` close and expands into eight physical hand actuator targets. SO101 `gripper.pos` maps to the same fixed motion.

## Website behavior

- The built-in `SuperArm + AmazingHand` record is selected as the primary robot when its three runtime asset variables resolve.
- The normal recording modal selects either Manual Web Leader or SO101 Leader. SO101 requires a serial port and calibration ID; both inputs enter the same 6D LeRobot action contract.
- Manual Web Leader exposes five arm sliders plus Open, Half, and Close buttons.
- Leaving Teleoperation stops the runtime; a tested back/re-enter cycle stopped cleanly, reconnected successfully, and stopped again.
- The original LeLab Three.js showroom now retains mesh filename extensions, resolves mesh URLs relative to the record URDF, waits for mesh loading before camera framing, and reports 13/13 physical joints while the runtime is connected.
- The inherited URDF hand mount used a detached `0.600003 m` display offset. LeLab now serves a non-destructive browser-only alignment at `0 0 0.011753`, matching the attached MuJoCo adapter chain. The input URDF is not modified.
- Joint 5 was generated at the wrong moving boundary: `motor_5 -> arm_link3b` rotated around a 25 mm shell offset. The focused branch now keeps that shell mount fixed and rotates `arm_link2b -> motor_5`, matching joint 3. The correction is applied non-destructively to served URDF and runtime MJCF.

## Runtime inputs

Use neutral focused paths:

```bash
export SUPERARM_ASSET_ROOT=$HOME/.cache/huggingface/lerobot/superarm/showroom
export SUPERARM_URDF_PATH=$SUPERARM_ASSET_ROOT/superarm_amazinghand.urdf
export SUPERARM_MUJOCO_MODEL_PATH=$HOME/.cache/huggingface/lerobot/amazinghand/model/superarm_amazinghand.xml
MUJOCO_GL=egl lelab
```

Live URL during the verification session: `http://127.0.0.1:8000`.

## Evidence

- Evidence root: `/home/dong/july/superarm_ws.omx-worktrees/lelab-mujoco-web-lerobot/artifacts/mujoco_web_live_20260719T1307Z`.
- GIF: `superarm_mujoco_lelab.gif` (six reviewed MuJoCo poses: open, half, close, arm motion).
- Primary showroom screenshot: `showroom.png` (attached arm and hand, real STL geometry, 13/13 joint coverage).
- Browser screenshots: `landing.png`, `manual.png`, `mujoco.png`, and `recording_so101_selector.png`.
- Contract reports: `live_contract_report.json`, `recording_evidence.json`, `dataset_contract_report.json`.
- Joint 5 audit: `artifacts/joint5_audit_20260719T1352Z` contains bad/fixed close-ups, live MuJoCo frames, and showroom screenshots. Before repair, motor-cover separation reached `35.341 mm` at `1.57 rad`; after repair the relative placement stays constant across the tested range.
- Real state-only LeRobot episode: `/home/dong/.cache/huggingface/lerobot/local/superarm_mujoco_manual_motion_evidence_20260719_133152`.
- Episode proof: 29 frames; `action` width 6; `observation.state` width 6; motion codes `0.0`, `0.5`, and `1.0`; five distinct arm action rows; observed state changed.

## Important limits

- Physical source-arm hardware: NOT TESTED.
- Physical eight-servo AmazingHand transport: NOT TESTED in this focused run.
- Camera capture: NOT TESTED; the recorded episode is state-only.
- Trained ACT/VLA rollout: NOT TESTED. Only ACT model construction for a 6D action head plus camera feature and the live state-only dataset boundary are verified.
