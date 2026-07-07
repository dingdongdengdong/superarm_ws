"""Isaac Sim hand-only scene bridge for the generated AmazingHand URDF.

Run via Isaac Sim Python, for example:
    /isaac-sim/python.sh /workspace/isaacsim/setup_amazinghand_scene.py --headless

ROS contract:
  publishes sensor_msgs/JointState on /hand/joint_states
  subscribes std_msgs/Float64MultiArray on /hand/joint_commands
  subscribes std_msgs/String JSON on /hand/screenshot_debug
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np

CONTAINER_ROOT = "/workspace/superarm_ws"
HOST_ROOT = Path(CONTAINER_ROOT) if Path(CONTAINER_ROOT).is_dir() else Path(__file__).resolve().parents[2]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

try:
    from isaacsim_test.isaacsim.graspable_hand_urdf import HAND_ACTUATED_JOINT_NAMES
    from isaacsim_test.isaacsim.robot_arm_hand_from_zip import build_named_joint_position_command
except ModuleNotFoundError:
    from graspable_hand_urdf import HAND_ACTUATED_JOINT_NAMES
    from robot_arm_hand_from_zip import build_named_joint_position_command

DEFAULT_HAND_URDF = f"{CONTAINER_ROOT}/isaacsim_test/outputs/robot_arm_hand_from_zip_local_drive/amazinghand_graspable.urdf"
HAND_URDF_PATH = os.environ.get("AMAZINGHAND_URDF_PATH", DEFAULT_HAND_URDF)
JOINT_STATE_TOPIC = os.environ.get("HAND_JOINT_STATE_TOPIC", "/hand/joint_states")
JOINT_COMMAND_TOPIC = os.environ.get("HAND_JOINT_COMMAND_TOPIC", "/hand/joint_commands")
SCREENSHOT_DEBUG_TOPIC = os.environ.get("HAND_SCREENSHOT_DEBUG_TOPIC", "/hand/screenshot_debug")
DEFAULT_SCREENSHOT_DIR = f"{CONTAINER_ROOT}/isaacsim_test/artifacts/manual_hand_screenshot_debug/screenshots"

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", default=bool(int(os.environ.get("HEADLESS", "1"))))
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from pxr import UsdLux  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64MultiArray, String  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.importer.urdf")
simulation_app.update()


def _position_list(raw_positions) -> list[float]:
    arr = np.asarray(raw_positions, dtype=np.float32)
    if arr.ndim == 2:
        arr = arr[0]
    return arr.reshape(-1).astype(float).tolist()


def _capture_screenshot(world: World, path: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    stage = omni.usd.get_context().get_stage()
    dome = UsdLux.DomeLight.Define(stage, "/World/AmazingHandDebugDomeLight")
    dome.CreateIntensityAttr(600.0)
    camera = Camera(
        prim_path="/World/AmazingHandDebugCamera",
        position=np.array([0.45, -0.75, 0.45]),
        orientation=np.array([0.82, 0.36, 0.18, 0.40]),
        resolution=(1280, 720),
    )
    camera.initialize()
    camera.set_focal_length(35.0)
    for _ in range(10):
        world.step(render=True)
    rgba = camera.get_rgba()
    if rgba is None or np.asarray(rgba).size == 0:
        raise RuntimeError("Camera returned no RGBA data")
    try:
        from PIL import Image

        arr = np.asarray(rgba)
        if arr.dtype != np.uint8:
            arr = np.clip(arr * 255 if arr.max(initial=0) <= 1.0 else arr, 0, 255).astype(np.uint8)
        Image.fromarray(arr[:, :, :4]).save(path)
    except Exception:
        import matplotlib.pyplot as plt

        plt.imsave(path, np.asarray(rgba)[:, :, :3])


class AmazingHandBridge(Node):
    def __init__(self, articulation: Articulation, world: World):
        super().__init__("amazinghand_isaacsim_bridge")
        self.articulation = articulation
        self.world = world
        self.dof_names = list(articulation.dof_names)
        self.command_seq = 0
        self.screenshot_count = 0
        self.capture_every_command = False
        self.output_dir = Path(os.environ.get("HAND_SCREENSHOT_OUTPUT_DIR", DEFAULT_SCREENSHOT_DIR))
        missing = [name for name in HAND_ACTUATED_JOINT_NAMES if name not in self.dof_names]
        if missing:
            raise RuntimeError(f"Generated hand URDF missing joints {missing}; available={self.dof_names}")
        self.pub = self.create_publisher(JointState, JOINT_STATE_TOPIC, 10)
        self.create_subscription(Float64MultiArray, JOINT_COMMAND_TOPIC, self._command_cb, 10)
        self.create_subscription(String, SCREENSHOT_DEBUG_TOPIC, self._screenshot_debug_cb, 10)

    def publish_state(self) -> None:
        positions = _position_list(self.articulation.get_joint_positions())
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(HAND_ACTUATED_JOINT_NAMES)
        msg.position = [positions[self.dof_names.index(name)] for name in HAND_ACTUATED_JOINT_NAMES]
        self.pub.publish(msg)

    def _command_cb(self, msg: Float64MultiArray) -> None:
        values = [float(v) for v in list(msg.data)[: len(HAND_ACTUATED_JOINT_NAMES)]]
        if len(values) < len(HAND_ACTUATED_JOINT_NAMES):
            values.extend([0.0] * (len(HAND_ACTUATED_JOINT_NAMES) - len(values)))
        targets = dict(zip(HAND_ACTUATED_JOINT_NAMES, values, strict=True))
        command = build_named_joint_position_command(
            current_positions=self.articulation.get_joint_positions(),
            dof_names=self.dof_names,
            joint_targets=targets,
        )
        self.articulation.set_joint_positions(command["positions"])
        self.command_seq += 1
        if self.capture_every_command:
            self._capture_now()

    def _screenshot_debug_cb(self, msg: String) -> None:
        payload = json.loads(msg.data or "{}")
        if "output_dir" in payload and payload["output_dir"]:
            self.output_dir = Path(str(payload["output_dir"]))
        if "capture_every_command" in payload:
            self.capture_every_command = bool(payload["capture_every_command"])
        if payload.get("request_capture"):
            self._capture_now()

    def _capture_now(self) -> None:
        self.screenshot_count += 1
        path = self.output_dir / f"hand_command_{self.command_seq:03d}_{self.screenshot_count:03d}.png"
        _capture_screenshot(self.world, str(path))


def main() -> int:
    urdf_path = Path(HAND_URDF_PATH)
    if not urdf_path.exists():
        print(f"[setup_amazinghand_scene] ERROR: hand URDF not found: {urdf_path}", flush=True)
        simulation_app.close()
        return 1

    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.fix_base = True
    import_config.import_inertia_tensor = True
    import_config.distance_scale = 1.0
    import_config.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.default_drive_strength = 200.0
    import_config.default_position_drive_damping = 20.0
    status, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(urdf_path),
        import_config=import_config,
        get_articulation_root=True,
    )
    if not status:
        raise RuntimeError(f"URDF import failed: {urdf_path}")

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    world.reset()
    articulation = Articulation(prim_path or "/amazinghand_graspable")
    articulation.initialize()
    print(f"[setup_amazinghand_scene] Loaded AmazingHand joints: {list(articulation.dof_names)}", flush=True)

    rclpy.init(args=None)
    bridge = AmazingHandBridge(articulation, world)
    spin_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    spin_thread.start()
    try:
        while simulation_app.is_running():
            world.step(render=True)
            bridge.publish_state()
            time.sleep(0.02)
    finally:
        bridge.destroy_node()
        rclpy.shutdown()
        simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
