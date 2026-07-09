# SimReady Controllable Arm Fixed Hand Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a derived controllable USD for the five arm DOFs while keeping AmazingHand fixed visual-only.

**Architecture:** Add tests first, then a pure-USD generator, then integrate `setup_rpo_arm_scene.py` to bind the overlay articulation when present. The SimReady source USD remains untouched.

**Tech Stack:** Python 3.10, Pixar USD Python bindings (`pxr.Usd`, `UsdGeom`, `UsdPhysics`), unittest/pytest-compatible tests, Isaac Sim scene script.

---

### Task 1: Contract Tests
**Files:**
- Create: `isaacsim_test/test_simready_controllable_arm_overlay.py`

- [ ] Write tests that import `isaacsim_test.simready_controllable_arm`, generate a temporary overlay USD from the existing SimReady USD, and assert five revolute joints, one articulation root, drive APIs, referenced fixed visual, and fixed_visual AmazingHand mapping.
- [ ] Run `python3 -m unittest isaacsim_test.test_simready_controllable_arm_overlay -v`; expected first failure is `ModuleNotFoundError` before implementation.

### Task 2: USD Generator
**Files:**
- Create: `isaacsim_test/simready_controllable_arm.py`

- [ ] Implement constants for the five arm joint names and fixed hand feature.
- [ ] Implement `create_controllable_arm_usd(source_usd, output_usd, mapping_json)` using USD APIs only.
- [ ] Author `/echo_full_controllable/VisualFixed` as a reference to the source USD.
- [ ] Author `/echo_full_controllable/ControlRig` with an articulation root, six rigid-body link prims, and five revolute joints with position drives.
- [ ] Write mapping JSON with five `articulation_bound` features and `amazinghand_grasp.pos` as `fixed_visual`.
- [ ] Run the contract test until green.

### Task 3: Scene Binding
**Files:**
- Modify: `isaacsim_test/isaacsim/setup_rpo_arm_scene.py`

- [ ] Add environment/default path support for the controllable overlay USD.
- [ ] Prefer the overlay when present and initialize `Articulation` at its control rig path.
- [ ] Keep fallback behavior for visual-only SimReady USD and legacy URDF.
- [ ] Ensure command handling applies only the five arm values to articulation joints and leaves AmazingHand fixed visual.

### Task 4: Verification
**Files:**
- Test: `isaacsim_test/test_simready_controllable_arm_overlay.py`
- Test: existing LeLab contract tests

- [ ] Run `python3 -m unittest isaacsim_test.test_simready_controllable_arm_overlay -v`.
- [ ] Run `python3 -m unittest isaacsim_test.test_lelab_isaacsim51_control_contract -v`.
- [ ] Run `cd worktrees/leLab && python3 -m pytest tests/test_teleoperate.py -q`.
- [ ] Inspect generated USD schema counts and report final artifact paths.
