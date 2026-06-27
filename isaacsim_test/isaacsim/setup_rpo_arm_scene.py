"""
Isaac Sim 5.1.0 scene script for the RoboParty/Roboto Origin V2.0
right arm plus a synthetic AmazingHand grasp channel.

Run via: /isaac-sim/python.sh /workspace/isaacsim/setup_rpo_arm_scene.py
"""
from __future__ import annotations

import argparse
import asyncio
import os
import sys
import threading
import time

import numpy as np

CONTROLLED_ARM_JOINT_NAMES = [
    "right_arm_pitch_joint",
    "right_arm_roll_joint",
    "right_arm_yaw_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
]
SYNTHETIC_GRASP_NAME = "amazinghand_grasp"
PUBLISHED_JOINT_NAMES = [*CONTROLLED_ARM_JOINT_NAMES, SYNTHETIC_GRASP_NAME]
DEFAULT_ROBOPARTY_V2_URDF = (
    "/workspace/superarm_ws/roboparty/modules/rpo_hardware/V2.0/"
    "roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf"
)
DEFAULT_SCREENSHOT_PATH = (
    "/workspace/superarm_ws/isaacsim_test/artifacts/"
    "rpo_v2_lerobot_target.png"
)


def _env_flag(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


SCREENSHOT_AFTER_COMMAND = _env_flag("SCREENSHOT_AFTER_COMMAND")
SCREENSHOT_PATH = os.environ.get("SCREENSHOT_PATH", DEFAULT_SCREENSHOT_PATH)
EXIT_AFTER_SCREENSHOT = _env_flag("EXIT_AFTER_SCREENSHOT", default=SCREENSHOT_AFTER_COMMAND)

parser = argparse.ArgumentParser()
parser.add_argument(
    "--headless",
    action="store_true",
    default=bool(int(os.environ.get("HEADLESS", "1"))),
)
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import omni.kit.commands  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension, get_extension_path_from_name  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.importer.urdf")
enable_extension("isaacsim.test.utils")
enable_extension("omni.kit.renderer.capture")
simulation_app.update()

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

ext_path = get_extension_path_from_name("isaacsim.asset.importer.urdf")
URDF_CANDIDATES = [
    os.environ.get("RPO_ARM_URDF_PATH", ""),
    DEFAULT_ROBOPARTY_V2_URDF,
]
urdf_path = next((p for p in URDF_CANDIDATES if p and os.path.isfile(p)), None)

if urdf_path is None:
    print(
        "[setup_rpo_arm_scene] ERROR: No RoboParty V2.0 URDF found. Tried:\n"
        + "\n".join(f"  {p}" for p in URDF_CANDIDATES if p)
        + "\n\nSet RPO_ARM_URDF_PATH to the official RoboParty V2.0 URDF, "
        "or mount the repo at /workspace/superarm_ws."
        + f"\nURDF importer extension path: {ext_path}",
        flush=True,
    )
    simulation_app.close()
    sys.exit(1)

print(f"[setup_rpo_arm_scene] Loading RoboParty V2.0 URDF: {urdf_path}", flush=True)
print(
    "[setup_rpo_arm_scene] Screenshot config: "
    f"after_command={SCREENSHOT_AFTER_COMMAND}, "
    f"path={SCREENSHOT_PATH}, "
    f"exit_after={EXIT_AFTER_SCREENSHOT}",
    flush=True,
)

status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.fix_base = True
import_config.import_inertia_tensor = True
import_config.distance_scale = 1.0
import_config.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_POSITION
import_config.default_drive_strength = 1047.2
import_config.default_position_drive_damping = 52.4

status, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=urdf_path,
    import_config=import_config,
    get_articulation_root=True,
)
assert status, f"[setup_rpo_arm_scene] URDF import failed: {urdf_path}"
prim_path = prim_path or "/roboto_origin"
print(f"[setup_rpo_arm_scene] Imported articulation prim: {prim_path}", flush=True)

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
print("[setup_rpo_arm_scene] Resetting world.", flush=True)
world.reset()

art = Articulation(prim_path)
print("[setup_rpo_arm_scene] Initializing articulation.", flush=True)
art.initialize()

num_dof = art.num_dof
all_dof_names = list(art.dof_names)
missing_joints = [name for name in CONTROLLED_ARM_JOINT_NAMES if name not in all_dof_names]
if missing_joints:
    print(
        "[setup_rpo_arm_scene] ERROR: Official V2 right-arm joints missing from imported URDF: "
        f"{missing_joints}\nAvailable joints: {all_dof_names}",
        flush=True,
    )
    simulation_app.close()
    sys.exit(1)

controlled_indices = [all_dof_names.index(name) for name in CONTROLLED_ARM_JOINT_NAMES]
print(f"[setup_rpo_arm_scene] Loaded {num_dof} total URDF joints: {all_dof_names}", flush=True)
print(
    "[setup_rpo_arm_scene] Controlled LeRobot joints: "
    f"{PUBLISHED_JOINT_NAMES}",
    flush=True,
)


def _position_list(raw_positions) -> list[float]:
    arr = np.asarray(raw_positions, dtype=np.float32)
    if arr.ndim == 2:
        arr = arr[0]
    return arr.reshape(-1).astype(float).tolist()


def _capture_rgb_screenshot(path: str, last_applied_command: list[float]) -> None:
    """Capture a headless RGB screenshot from a temporary camera."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    print(
        f"[setup_rpo_arm_scene] Capturing screenshot after LeRobot command "
        f"{last_applied_command} -> {path}",
        flush=True,
    )
    errors = []
    capture_methods = [
        ("renderer resource", _capture_renderer_resource_screenshot),
        ("Replicator RGB", _capture_replicator_rgb_screenshot),
        ("viewport", _capture_viewport_screenshot),
    ]
    for label, capture_method in capture_methods:
        print(
            f"[setup_rpo_arm_scene] Trying {label} screenshot capture.",
            flush=True,
        )
        try:
            capture_method(path)
            if os.path.isfile(path) and os.path.getsize(path) > 0:
                print(
                    f"[setup_rpo_arm_scene] Screenshot saved via {label}: {path}",
                    flush=True,
                )
                return
            raise RuntimeError("capture returned but did not create a non-empty file")
        except BaseException as exc:
            if isinstance(exc, KeyboardInterrupt):
                raise
            errors.append(f"{label}: {type(exc).__name__}: {exc}")
            print(
                f"[setup_rpo_arm_scene] {label} screenshot failed: "
                f"{type(exc).__name__}: {exc}",
                flush=True,
            )
    raise RuntimeError("All screenshot capture methods failed: " + "; ".join(errors))


def _wait_for_screenshot_file(path: str, timeout_s: float = 10.0) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        simulation_app.update()
        if os.path.isfile(path) and os.path.getsize(path) > 0:
            return
        time.sleep(0.05)
    raise TimeoutError(f"Timed out waiting for screenshot file: {path}")


def _capture_renderer_resource_screenshot(path: str) -> None:
    """Capture the viewport's LDR render resource without blocking on async helpers."""
    import omni.kit.viewport_legacy
    import omni.renderer_capture

    renderer = omni.renderer_capture.acquire_renderer_capture_interface()
    viewport_interface = omni.kit.viewport_legacy.acquire_viewport_interface()
    viewport_window = viewport_interface.get_viewport_window(None)
    if viewport_window is None:
        raise RuntimeError("No legacy viewport window available")

    drawable_resource = None
    deadline = time.time() + 10.0
    while time.time() < deadline:
        drawable_resource = viewport_window.get_drawable_ldr_resource()
        if drawable_resource is not None:
            break
        simulation_app.update()
        time.sleep(0.05)
    if drawable_resource is None:
        raise TimeoutError("Timed out waiting for a drawable LDR viewport resource")

    renderer.capture_next_frame_rp_resource(path, drawable_resource)
    _wait_for_screenshot_file(path)


def _capture_replicator_rgb_screenshot(path: str) -> None:
    """Capture an RGB image via synchronous Replicator annotators."""
    import omni.replicator.core as rep
    import omni.usd
    from PIL import Image

    temp_cam = None
    temp_render_product = None
    annot = None
    camera_path = None
    try:
        temp_cam = rep.functional.create.camera(
            position=(1.4, -1.6, 1.0),
            look_at=(0.0, 0.0, 0.1),
        )
        camera_path = str(temp_cam.GetPath())
        temp_render_product = rep.create.render_product(camera_path, (1280, 720))
        annot = rep.AnnotatorRegistry.get_annotator("rgb")
        annot.attach(temp_render_product)

        rep.orchestrator.set_capture_on_play(False)
        for _ in range(3):
            simulation_app.update()
        rep.orchestrator.step(rt_subframes=4, pause_timeline=False, wait_for_render=True)
        rgb = np.asarray(annot.get_data())
    finally:
        if annot is not None and temp_render_product is not None:
            annot.detach()
        if temp_render_product is not None:
            temp_render_product.destroy()
        if camera_path is not None:
            stage = omni.usd.get_context().get_stage()
            stage.RemovePrim(camera_path)

    if rgb.ndim != 3 or rgb.shape[-1] < 3:
        raise RuntimeError(f"Unexpected RGB capture shape: {rgb.shape}")
    Image.fromarray(rgb[..., :3].astype(np.uint8)).save(path)


def _capture_viewport_screenshot(path: str) -> None:
    """Capture the active viewport if Replicator capture is unavailable."""
    from omni.kit.viewport.utility import (
        capture_viewport_to_file,
        frame_viewport_prims,
        get_active_viewport,
    )

    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("No active viewport available for screenshot capture")

    frame_viewport_prims(viewport, prims=[prim_path])
    simulation_app.update()

    async def _capture():
        capture = capture_viewport_to_file(viewport, file_path=path)
        await asyncio.wait_for(capture.wait_for_result(), timeout=10.0)

    loop = asyncio.get_event_loop()
    loop.run_until_complete(_capture())


all_positions = _position_list(art.get_joint_positions())
current_arm_positions = [all_positions[i] for i in controlled_indices]
current_grasp = 0.0
current_positions = [*current_arm_positions, current_grasp]
current_velocities = [0.0] * len(PUBLISHED_JOINT_NAMES)

rclpy.init()


class SimBridgeNode(Node):
    def __init__(self):
        super().__init__("isaac_sim_rpo_arm_bridge")
        self._cmd = None
        self._cmd_seq = 0
        self._cmd_lock = threading.Lock()
        self._pub = self.create_publisher(JointState, "/follower/joint_states", 10)
        self._sub = self.create_subscription(
            Float64MultiArray, "/follower/joint_commands", self._cmd_cb, 10
        )

    def _cmd_cb(self, msg: Float64MultiArray):
        with self._cmd_lock:
            self._cmd = list(msg.data)
            self._cmd_seq += 1

    def get_command(self):
        with self._cmd_lock:
            cmd = list(self._cmd) if self._cmd is not None else None
            return self._cmd_seq, cmd

    def publish_state(self, names, positions, velocities, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = names
        msg.position = [float(p) for p in positions]
        msg.velocity = [float(v) for v in velocities]
        self._pub.publish(msg)


bridge = SimBridgeNode()
bridge_spin_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
bridge_spin_thread.start()


def _publish_current_state() -> None:
    stamp = bridge.get_clock().now().to_msg()
    bridge.publish_state(PUBLISHED_JOINT_NAMES, current_positions, current_velocities, stamp)

import omni.timeline  # noqa: E402

timeline = omni.timeline.get_timeline_interface()
timeline.play()

print("[setup_rpo_arm_scene] Simulation running. Ctrl+C to stop.", flush=True)
screenshot_taken = False
last_processed_command_seq = -1
try:
    app_updated = False
    while simulation_app.is_running():
        # In this container build, repeated Kit/world stepping can block after the
        # first frame. Keep a mirrored ROS joint bridge responsive and push the
        # commanded official V2 right-arm joints into Isaac best-effort for visualization.
        if not app_updated:
            simulation_app.update()
            app_updated = True

        cmd_seq, cmd = bridge.get_command()
        should_exit_after_screenshot = False
        if (
            cmd is not None
            and cmd_seq != last_processed_command_seq
            and len(cmd) >= len(CONTROLLED_ARM_JOINT_NAMES)
        ):
            last_processed_command_seq = cmd_seq
            command_values = [float(v) for v in cmd]
            arm_command = command_values[: len(CONTROLLED_ARM_JOINT_NAMES)]
            current_grasp = float(
                np.clip(
                    command_values[len(CONTROLLED_ARM_JOINT_NAMES)]
                    if len(command_values) > len(CONTROLLED_ARM_JOINT_NAMES)
                    else current_grasp,
                    0.0,
                    1.0,
                )
            )
            current_positions = [*arm_command, current_grasp]

            for idx, value in zip(controlled_indices, arm_command, strict=True):
                all_positions[idx] = value
            art.set_joint_positions(np.array([all_positions], dtype=np.float32))
            last_applied_command = list(current_positions)
            _publish_current_state()
            print(
                f"[setup_rpo_arm_scene] Applied LeRobot command #{cmd_seq}: "
                f"{last_applied_command}",
                flush=True,
            )

            if SCREENSHOT_AFTER_COMMAND and not screenshot_taken:
                print("[setup_rpo_arm_scene] Screenshot trigger accepted.", flush=True)
                _capture_rgb_screenshot(SCREENSHOT_PATH, last_applied_command)
                screenshot_taken = True
                if EXIT_AFTER_SCREENSHOT:
                    print(
                        "[setup_rpo_arm_scene] Exiting after screenshot as requested.",
                        flush=True,
                    )
                    should_exit_after_screenshot = True

        _publish_current_state()
        if should_exit_after_screenshot:
            for _ in range(10):
                time.sleep(0.05)
                _publish_current_state()
            break
        time.sleep(1.0 / 60.0)

except KeyboardInterrupt:
    pass
finally:
    print("[setup_rpo_arm_scene] Shutting down.", flush=True)
    timeline.stop()
    rclpy.shutdown()
    bridge_spin_thread.join(timeout=2.0)
    bridge.destroy_node()
    simulation_app.close()
