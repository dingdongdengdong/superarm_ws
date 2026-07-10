# AmazingHandControl provenance

The LeLab SuperArm dashboard adapts controller behavior from the upstream
AmazingHandControl project. The checkout under `worktrees/AmazingHandControl`
is an ignored reference checkout and is **not** a runtime dependency.

- Repository: <https://github.com/Betatester777/AmazingHandControl>
- Inspected revision: `2a59fd8fbf521bcdf547cc48cc0f55f4b74ee697`
- Revision shorthand: `2a59fd8`
- Upstream release metadata: `0.8.0`
- License: Apache License 2.0
- Inspected files: `hand_logic.py`, `data/config.yaml`,
  `data/hand_config.yaml`, and upstream tests

Adapted behavior includes the Ring/Middle/Pointer/Thumb program ordering,
servo-ID mapping, even-servo inversion, speed range, serial defaults, keyboard
controls, validation, and pose/sequence import rules. Tkinter, matplotlib, and
the upstream standalone CLI are intentionally not embedded.

The hand geometry and closed-loop constraints come from the official
`hand_mjcf/robot.xml` contained in this repository's
`robot_arm_hand_package.zip`, not from the reference checkout.

