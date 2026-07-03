# Task Separation: LeRobot, Isaac Sim, and Arm SITL

Date: 2026-06-29

## Purpose

This document separates responsibilities for the RoboParty / AmazingHand SITL path so future work does not mix LeRobot dataset/control work, Isaac Sim scene/physics work, this repo's ROS2 bridge work, and real-arm safety work.

Use this alongside:

- `docs/sitl/2026-06-27/README.md` for the phased SITL implementation plan.
- `docs/sitl/2026-06-27/simready_runtime_evidence.md` for the latest SimReady runtime evidence.
- Official LeRobot docs:
  - Bring Your Own Hardware: <https://huggingface.co/docs/lerobot/en/integrate_hardware>
  - LeRobotDataset v3.0: <https://huggingface.co/docs/lerobot/lerobot-dataset-v3>
  - LeIsaac / EnvHub: <https://huggingface.co/docs/lerobot/envhub_leisaac>
  - Robot and teleoperator processors: <https://huggingface.co/docs/lerobot/processors_robots_teleop>


## Official LeRobot vs this repo

Official LeRobot does **not** say custom robots must use ROS2. The official custom hardware flow asks you to define a robot configuration/class, expose observation and action features, implement connection/disconnection, read observations, and send actions. The transport inside that robot class can be a motor bus, serial, CAN, vendor SDK, simulator API, ROS2, or something else.

For this project, ROS2 is an implementation detail of the local SITL wrapper:

- `IsaacSimRpoArmRobot.send_action()` publishes to `/follower/joint_commands`.
- `IsaacSimRpoArmRobot.capture_observation()` reads state received from `/follower/joint_states`.
- `setup_rpo_arm_scene.py` is the Isaac Sim side of that ROS2 bridge.

Therefore, when making a LeRobot custom config preset, use ROS2 only if the target backend is this Isaac Sim SITL bridge or another ROS2-speaking robot backend. Do not present ROS2 as a general LeRobot requirement.


## Official Isaac Sim vs this repo

Isaac Sim also does **not** require ROS2 for general simulation. NVIDIA documents ROS2 as a bridge/extension for ROS system integration. Isaac Sim can also be controlled through its native Python APIs, extensions, OmniGraph, IsaacLab environments, or other simulator interfaces.

For this project, ROS2 is useful because it creates a simple boundary between the local LeRobot wrapper and the Isaac Sim scene:

- LeRobot-side Python publishes a six-value command vector.
- Isaac Sim-side Python receives that command vector and publishes joint state feedback.
- The same conceptual boundary can later be replaced by a direct simulator API if the team chooses a non-ROS SITL architecture.

Therefore, do not teach the team that "Isaac Sim requires ROS2." Teach: "our current Isaac Sim SITL bridge uses ROS2."

## Compatibility note

The official LeRobot documentation describes the current target architecture, including robot interfaces with `observation_features`, `action_features`, `get_observation()`, and `send_action()`, plus newer command-line tools such as `lerobot-record`.

This repository currently uses an older local LeRobot checkout (`lerobot/pyproject.toml` reports version `0.1.0`) and a local SITL wrapper:

- `isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py`
- `isaacsim_test/lerobot/verify_lerobot_sitl.py`
- `isaacsim_test/lerobot/rpo_arm_isaacsim.yaml`

For near-term SITL tasks, preserve the local wrapper contract unless a separate LeRobot upgrade task is opened. Do not copy newer API names blindly into this checkout without checking local compatibility.

## Stable SITL control contract

Keep the first SITL interface as a six-value command/state contract:

| Index | LeRobot feature | Meaning | Current unit / range | Notes |
|---:|---|---|---|---|
| 0 | `right_arm_pitch_joint.pos` | Right arm pitch command/state | radians in SITL | Real hardware sign and limit still need parity check. |
| 1 | `right_arm_roll_joint.pos` | Right arm roll command/state | radians in SITL | Real hardware sign and limit still need parity check. |
| 2 | `right_arm_yaw_joint.pos` | Right arm yaw command/state | radians in SITL | Real hardware sign and limit still need parity check. |
| 3 | `right_elbow_pitch_joint.pos` | Right elbow pitch command/state | radians in SITL | Real hardware sign and limit still need parity check. |
| 4 | `right_elbow_yaw_joint.pos` | Right elbow yaw command/state | radians in SITL | Real hardware sign and limit still need parity check. |
| 5 | `amazinghand_grasp.pos` | Hand grasp intent | normalized `0.0` open to `1.0` grasp | Multi-DOF hand binding is future work. |

Do not change this shape from `(6,)` until record/replay, policy smoke, and hardware parity tasks explicitly define a migration path.

## Responsibility separation

| Area | Owner/system | Responsibilities | Inputs | Outputs/evidence | Done criteria |
|---|---|---|---|---|---|
| LeRobot robot wrapper | Local LeRobot + `IsaacSimRpoArmRobot` | Expose the six action/observation features, send actions through `send_action()`, capture state through the wrapper, and provide record/replay/policy entrypoints. In this repo, the wrapper internally uses ROS2 to reach Isaac Sim; official LeRobot itself is transport-agnostic. | `rpo_arm_isaacsim.yaml`, ROS2 joint state topic, ROS2 command topic, LeRobot control scripts. | `observation.state`, `action`, verifier JSON, dataset feature metadata. | Wrapper instantiates, connects, sends actions, captures observations, and preserves `(6,)` action/state shape. |
| LeRobot dataset and policy flow | LeRobot control/record/train/eval tooling | Record deterministic SITL episodes, replay episodes through the wrapper, run deterministic policy smoke before real policy rollout, keep dataset feature names aligned with the six-feature contract. | SITL wrapper, teleop source, task description, episode configuration. | Local dataset, replay log, policy smoke JSON, optional Hub-compatible dataset artifacts. | Dataset has `observation.state` and `action` shape `(6,)`; replay and policy smoke use `IsaacSimRpoArmRobot.send_action()`. |
| Isaac Sim scene and asset | Isaac Sim container + SimReady USD | Load the validated SimReady USD, inspect prim hierarchy, bind controls to real USD/articulation targets where possible, keep unavailable controls marked as `binding_pending`. | `SIMREADY_USD_PATH`, SimReady USD, scene setup script. | `simready_prim_mapping.json`, screenshots, Isaac logs. | SimReady asset loads as primary scene asset and every LeRobot feature is either bound or explicitly reported as pending. |
| Isaac Sim physics and visual evidence | Isaac Sim scene script and verifiers | Validate commanded motion, joint clipping, stable simulation, and repeatable screenshots for intended-vs-observed review. | Target sweeps, exported joint metadata, camera view settings. | Sweep evidence JSON, applied-vs-requested target data, screenshot files. | Sweep passes within tolerance; screenshots are non-empty and tied to target indices. |
| ROS2 / SITL bridge | Docker Compose, ROS2 topics, bridge code | Keep topic names, `ROS_DOMAIN_ID`, container startup order, and one-command verifier stable. Avoid direct ROS publishing from tests when LeRobot wrapper coverage is required. | `docker-compose.yml`, `.env`, ROS2 messages, verifier scripts. | Bringup logs, LeRobot verifier logs, one-command SITL artifacts. | One command starts Isaac Sim, runs the LeRobot verifier, writes evidence, and exits cleanly. |
| Real arm / hardware parity | RoboParty arm + AmazingHand integration | Map each simulated feature to real motor/CAN/serial signals, verify signs, limits, relative movement clamps, emergency stop, and power cut before any real policy replay. | SITL feature table, hardware motor IDs, hand servo IDs, safety limits. | Hardware parity checklist, low-speed direction-check log, validation gate updates. | Every real joint direction and limit is checked at low speed; emergency stop is verified before policy or dataset collection on hardware. |
| Documentation and regression gates | SITL docs and tests | Keep task order, assumptions, and evidence locations discoverable; update docs when a binding changes from pending to bound. | SITL README, runtime evidence, verifier results. | Markdown task logs, checklists, static tests. | New behavior has a documented gate and generated artifacts stay under `isaacsim_test/artifacts/`. |

## Execution order

1. **Freeze the LeRobot contract**: keep the six feature names and `(6,)` action/state shape stable in the local wrapper and tests.
2. **Keep SimReady as the primary Isaac asset**: load `SIMREADY_USD_PATH` by default and treat the RoboParty URDF as legacy/reference input unless explicitly needed.
3. **Bind or mark each feature**: replace `binding_pending` only when a real USD/articulation/control binding is verified.
4. **Run command/state SITL verification**: use the LeRobot wrapper path, not direct ROS publishing, to prove the control path used by datasets and policies.
5. **Add sweep and visual evidence**: record per-target requested/applied/observed values and screenshots for pose review.
6. **Add LeRobot record/replay gates**: only after the sweep passes, record a tiny deterministic SITL episode and replay it through the same wrapper.
7. **Add policy smoke**: run a deterministic policy-like sequence before any learned policy is evaluated.
8. **Prepare hardware parity**: do not move the real arm until SITL sweep evidence, low-speed direction checks, stricter software limits, and emergency stop checks are documented.

## LeRobot-specific investigation findings

- Official custom-hardware guidance makes the robot class boundary the key abstraction: feature definitions must match the dictionaries returned by observation and accepted by action sending.
- Official dataset guidance treats `observation.state`, `action`, timestamps, metadata, and videos as standardized data surfaces. For this repo, the first safe dataset target is a simple `(6,)` proprioceptive/action dataset with optional images added later.
- Official LeIsaac guidance is useful as an architectural reference for simulation-first collection and training. Its examples use IsaacLab/Gym-style environment loading and `env.step(...)`, not this repo's ROS2 topic bridge. Treat LeIsaac as reference material, not an immediate replacement, unless a migration task is created.
- Official processor guidance supports a future cleanup path: teleop commands, dataset actions, and robot commands can be represented as separate processing pipelines. For now, keep the raw six-feature joint/grasp representation to avoid mixing representation migration with SITL binding work.

## Non-goals for this document

- No LeRobot package upgrade.
- No migration from local `control_robot.py` usage to newer `lerobot-record` commands.
- No change from six-feature grasp scalar to multi-DOF hand control.
- No direct real-arm motion.
