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
import re
import shutil
import sys
import tempfile
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
RENDER_EVERY_STEP = os.environ.get("HAND_RENDER_EVERY_STEP", "0").strip() == "1"
_REPO_INPUTS_MARKER = "/isaacsim_test/inputs/robot_arm_hand_package/"

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", default=bool(int(os.environ.get("HEADLESS", "1"))))
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux  # noqa: E402

try:  # Isaac Sim 5.x command API.
    from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType  # noqa: E402
except ModuleNotFoundError:  # Isaac Sim 6.0 public importer API.
    UrdfJointTargetType = None  # type: ignore[assignment]

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


def _container_accessible_urdf_path(urdf_path: Path) -> Path:
    """Return a URDF whose mesh paths are readable from the Isaac container.

    The generated hand URDF is often created on the host, so MJCF visual meshes can
    be authored as absolute host paths such as ``/home/dong/.../isaacsim_test``.
    Inside the Isaac container this repo is mounted at ``/workspace/superarm_ws``.
    If those host paths are left untouched, Isaac imports the articulation joints
    and collision boxes, but silently drops the real AmazingHand STL visuals.
    """
    if not Path(CONTAINER_ROOT).is_dir():
        return urdf_path

    text = urdf_path.read_text(encoding="utf-8")
    changed = False

    def _replace(match: re.Match[str]) -> str:
        nonlocal changed
        prefix, filename, suffix = match.groups()
        normalized = filename.replace("\\", "/")
        if Path(normalized).exists():
            return match.group(0)
        marker_index = normalized.find(_REPO_INPUTS_MARKER)
        if marker_index < 0:
            return match.group(0)
        candidate = CONTAINER_ROOT + normalized[marker_index:]
        if not Path(candidate).exists():
            return match.group(0)
        changed = True
        return f'{prefix}{candidate}{suffix}'

    remapped = re.sub(r'(filename=")([^"]+)(")', _replace, text)
    if not changed:
        return urdf_path

    remapped_path = Path(tempfile.mkdtemp(prefix="amazinghand_urdf_mesh_paths_")) / urdf_path.name
    remapped_path.write_text(remapped, encoding="utf-8")
    print(
        f"[setup_amazinghand_scene] remapped host URDF mesh paths for container: "
        f"{urdf_path} -> {remapped_path}",
        flush=True,
    )
    return remapped_path


def _save_rgba_png(path: str, rgba) -> None:
    try:
        from PIL import Image

        arr = np.asarray(rgba)
        if arr.dtype != np.uint8:
            arr = np.clip(arr * 255 if arr.max(initial=0) <= 1.0 else arr, 0, 255).astype(np.uint8)
        Image.fromarray(arr[:, :, :4]).save(path)
    except Exception:
        import matplotlib.pyplot as plt

        plt.imsave(path, np.asarray(rgba)[:, :, :3])


def _frame_camera(stage, root_prim_path: str) -> tuple[tuple[float, float, float], tuple[float, float, float], float]:
    prim = stage.GetPrimAtPath(root_prim_path)
    if not prim.IsValid():
        prim = stage.GetDefaultPrim()
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Cannot frame invalid hand prim: {root_prim_path}")
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=True,
    )
    bbox = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
    bbox_min = bbox.GetMin()
    bbox_max = bbox.GetMax()
    center = (bbox_min + bbox_max) * 0.5
    size = bbox_max - bbox_min
    radius = max(float(size[0]), float(size[1]), float(size[2]), 0.08)
    eye = Gf.Vec3d(
        float(center[0]) + radius * 1.4,
        float(center[1]) - radius * 2.0,
        float(center[2]) + radius * 1.1,
    )
    target = Gf.Vec3d(float(center[0]), float(center[1]), float(center[2]))
    return tuple(float(v) for v in eye), tuple(float(v) for v in target), radius


def _capture_replicator(path: str, root_prim_path: str) -> None:
    import omni.replicator.core as rep

    stage = omni.usd.get_context().get_stage()
    eye, target, _ = _frame_camera(stage, root_prim_path)
    if not stage.GetPrimAtPath("/World/AmazingHandDebugDomeLight").IsValid():
        UsdLux.DomeLight.Define(stage, "/World/AmazingHandDebugDomeLight").CreateIntensityAttr(600.0)
    output_dir = tempfile.mkdtemp(prefix=Path(path).name + ".")
    try:
        with rep.new_layer():
            camera = rep.create.camera(position=eye, look_at=target, focal_length=35)
            render_product = rep.create.render_product(camera, (1280, 720))
            writer = rep.WriterRegistry.get("BasicWriter")
            writer.initialize(output_dir=output_dir, rgb=True)
            writer.attach([render_product])
            for _ in range(8):
                rep.orchestrator.step(rt_subframes=8)
            writer.detach()
        frames = sorted(Path(output_dir).glob("rgb*.png"))
        if not frames:
            raise RuntimeError(f"Replicator did not write RGB frames in {output_dir}")
        shutil.copyfile(frames[-1], path)
    finally:
        shutil.rmtree(output_dir, ignore_errors=True)


def _capture_screenshot(world: World, path: str, root_prim_path: str) -> None:
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
    last_shape = None
    for _ in range(60):
        world.step(render=True)
        rgba = camera.get_rgba()
        last_shape = None if rgba is None else list(np.asarray(rgba).shape)
        if rgba is not None and np.asarray(rgba).size > 0:
            _save_rgba_png(path, rgba)
            return
    print(
        f"[setup_amazinghand_scene] Camera.get_rgba had no data after 60 frames; "
        f"last_shape={last_shape}; trying Replicator fallback",
        flush=True,
    )
    _capture_replicator(path, root_prim_path)


class AmazingHandBridge(Node):
    def __init__(self, articulation: Articulation, world: World, root_prim_path: str):
        super().__init__("amazinghand_isaacsim_bridge")
        self.articulation = articulation
        self.world = world
        self.root_prim_path = root_prim_path
        self.dof_names = list(articulation.dof_names)
        self.command_seq = 0
        self.screenshot_count = 0
        self.capture_every_command = False
        self.output_dir = Path(os.environ.get("HAND_SCREENSHOT_OUTPUT_DIR", DEFAULT_SCREENSHOT_DIR))
        self.command_evidence_path = Path(
            os.environ.get(
                "HAND_COMMAND_EVIDENCE_PATH",
                str(self.output_dir.parent / "hand_command_evidence.jsonl"),
            )
        )
        self._command_lock = threading.Lock()
        self._pending_targets: dict[str, float] | None = None
        self._active_targets: dict[str, float] | None = None
        self._pending_capture_count = 0
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
        with self._command_lock:
            self._pending_targets = targets
        print(f"[setup_amazinghand_scene] queued hand command {self.command_seq + 1}: {targets}", flush=True)

    def apply_pending_command(self) -> None:
        with self._command_lock:
            targets = self._pending_targets
            self._pending_targets = None
        if targets is None:
            return
        self._active_targets = dict(targets)
        readback = self._apply_targets(targets)
        self.command_seq += 1
        self._write_command_evidence(
            {
                "seq": self.command_seq,
                "targets": targets,
                "readback": readback,
                "dof_names": self.dof_names,
            }
        )
        print(f"[setup_amazinghand_scene] applied hand command {self.command_seq}: {readback}", flush=True)
        if self.capture_every_command:
            self.queue_capture()

    def hold_active_command(self) -> None:
        if self._active_targets is not None:
            self._apply_targets(self._active_targets)

    def _apply_targets(self, targets: dict[str, float]) -> dict[str, float]:
        command = build_named_joint_position_command(
            current_positions=self.articulation.get_joint_positions(),
            dof_names=self.dof_names,
            joint_targets=targets,
        )
        self.articulation.set_joint_positions(command["positions"])
        if hasattr(self.articulation, "set_joint_position_targets"):
            self.articulation.set_joint_position_targets(command["positions"])
        return self._named_readback()

    def _screenshot_debug_cb(self, msg: String) -> None:
        payload = json.loads(msg.data or "{}")
        if "output_dir" in payload and payload["output_dir"]:
            self.output_dir = Path(str(payload["output_dir"]))
        if "capture_every_command" in payload:
            self.capture_every_command = bool(payload["capture_every_command"])
        print(f"[setup_amazinghand_scene] screenshot debug payload: {payload}", flush=True)
        if payload.get("request_capture"):
            self.queue_capture()

    def queue_capture(self) -> None:
        with self._command_lock:
            self._pending_capture_count += 1

    def apply_pending_capture(self) -> None:
        with self._command_lock:
            pending = self._pending_capture_count
            self._pending_capture_count = 0
        for _ in range(pending):
            self._capture_now_safe()

    def _capture_now(self) -> None:
        self.screenshot_count += 1
        path = self.output_dir / f"hand_command_{self.command_seq:03d}_{self.screenshot_count:03d}.png"
        _capture_screenshot(self.world, str(path), self.root_prim_path)

    def _capture_now_safe(self) -> None:
        try:
            self._capture_now()
        except Exception as exc:
            print(f"[setup_amazinghand_scene] screenshot capture failed: {exc}", flush=True)

    def _named_readback(self) -> dict[str, float]:
        positions = _position_list(self.articulation.get_joint_positions())
        return {name: positions[self.dof_names.index(name)] for name in HAND_ACTUATED_JOINT_NAMES}

    def _write_command_evidence(self, payload: dict) -> None:
        self.command_evidence_path.parent.mkdir(parents=True, exist_ok=True)
        with self.command_evidence_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(payload, sort_keys=True) + "\n")


def main() -> int:
    urdf_path = Path(HAND_URDF_PATH)
    if not urdf_path.exists():
        print(f"[setup_amazinghand_scene] ERROR: hand URDF not found: {urdf_path}", flush=True)
        simulation_app.close()
        return 1
    urdf_path = _container_accessible_urdf_path(urdf_path)

    if UrdfJointTargetType is not None:
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
    else:
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

        usd_base_dir = os.environ.get(
            "AMAZINGHAND_USD_OUTPUT_DIR",
            os.environ.get("HAND_SCREENSHOT_OUTPUT_DIR", DEFAULT_SCREENSHOT_DIR),
        )
        usd_output_dir = Path(usd_base_dir).parent / "usd"
        usd_output_dir.mkdir(parents=True, exist_ok=True)
        import_config = URDFImporterConfig(
            urdf_path=str(urdf_path),
            usd_path=str(usd_output_dir),
            collision_from_visuals=False,
            merge_mesh=False,
        )
        output_path = URDFImporter(import_config).import_urdf()
        if not output_path:
            raise RuntimeError(f"URDF import failed: {urdf_path}")
        omni.usd.get_context().open_stage(output_path)
        for _ in range(10):
            simulation_app.update()
        stage = omni.usd.get_context().get_stage()
        default_prim = stage.GetDefaultPrim()
        prim_path = str(default_prim.GetPath()) if default_prim else "/amazinghand_graspable"

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    world.reset()
    articulation = Articulation(prim_path or "/amazinghand_graspable")
    articulation.initialize()
    print(f"[setup_amazinghand_scene] Loaded AmazingHand joints: {list(articulation.dof_names)}", flush=True)

    rclpy.init(args=None)
    bridge = AmazingHandBridge(articulation, world, prim_path or "/amazinghand_graspable")
    spin_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    spin_thread.start()
    try:
        while simulation_app.is_running():
            world.step(render=RENDER_EVERY_STEP)
            bridge.apply_pending_command()
            bridge.hold_active_command()
            bridge.apply_pending_capture()
            bridge.publish_state()
            time.sleep(0.02)
    finally:
        bridge.destroy_node()
        rclpy.shutdown()
        simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
