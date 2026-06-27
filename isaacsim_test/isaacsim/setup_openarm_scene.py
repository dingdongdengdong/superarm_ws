"""
Isaac Sim 4.5 scene script — loads OpenArm URDF and bridges joint state/commands to ROS2.

Run via:  /isaac-sim/python.sh /workspace/isaacsim/setup_openarm_scene.py
"""
import argparse
import os
import sys
import threading
import time

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true",
                    default=bool(int(os.environ.get("HEADLESS", "1"))))
args, _ = parser.parse_known_args()

# SimulationApp must be created before any omni imports
from isaacsim import SimulationApp  # noqa: E402
simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

# --- Extensions (must load after SimulationApp) ---
from omni.isaac.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("omni.isaac.ros2_bridge")

# Isaac Sim 5.x uses omni.importer.urdf (renamed from omni.isaac.urdf in 4.x)
try:
    enable_extension("omni.importer.urdf")
except Exception:
    enable_extension("omni.isaac.urdf")

simulation_app.update()  # flush extension loads before importing their APIs

# --- Domain-specific imports (after extensions are loaded) ---
import carb  # noqa: E402
import omni.kit.commands  # noqa: E402
from omni.isaac.core import World  # noqa: E402
from omni.isaac.core.articulations import ArticulationView  # noqa: E402

try:
    from omni.isaac.urdf import _urdf as urdf_mod
    ImportConfig = urdf_mod.ImportConfig
    PARSE_CMD = "URDFParseAndImportFile"
except ImportError:
    from omni.importer.urdf import _urdf as urdf_mod
    ImportConfig = urdf_mod.ImportConfig
    PARSE_CMD = "URDFParseAndImportFile"

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

# --- URDF discovery ---
URDF_CANDIDATES = [
    os.environ.get("OPENARM_URDF_PATH", ""),
    "/workspace/superarm_ws/lerobot/lerobot/robots/openarm/assets/openarm.urdf",
    "/workspace/superarm_ws/lerobot/lerobot/configs/robot/openarm.urdf",
    "/workspace/openarm/urdf/openarm.urdf",
]
urdf_path = next((p for p in URDF_CANDIDATES if p and os.path.isfile(p)), None)
if urdf_path is None:
    print(
        "[setup_openarm_scene] ERROR: OpenArm URDF not found.\n"
        "  Set OPENARM_URDF_PATH env var, or place the URDF at one of:\n"
        + "\n".join(f"    {p}" for p in URDF_CANDIDATES if p) +
        "\n\n  Fallback option: use Isaac Sim's built-in Franka Panda by setting:\n"
        "    OPENARM_URDF_PATH=/isaac-sim/exts/omni.isaac.franka/data/urdf/robots/panda_arm_hand.urdf\n"
        "  (6-DOF bridge still works; only the visual model differs.)"
    )
    simulation_app.close()
    sys.exit(1)

print(f"[setup_openarm_scene] Loading URDF: {urdf_path}")

# --- World + URDF import ---
world = World(stage_units_in_meters=1.0)

import_cfg = ImportConfig()
import_cfg.fix_base = True
import_cfg.import_inertia_tensor = True
import_cfg.distance_scale = 1.0
import_cfg.default_drive_type = urdf_mod.UrdfJointTargetType.JOINT_DRIVE_POSITION
import_cfg.default_drive_strength = 1047.2
import_cfg.default_position_drive_damping = 52.4

ok, _ = omni.kit.commands.execute(
    PARSE_CMD,
    urdf_path=urdf_path,
    import_config=import_cfg,
    dest_path="/World/OpenArm",
)
if not ok:
    print(f"[setup_openarm_scene] ERROR: URDF import failed for {urdf_path}")
    simulation_app.close()
    sys.exit(1)

world.scene.add_default_ground_plane()

art_view = ArticulationView(prim_paths_expr="/World/OpenArm", name="openarm_view")
world.scene.add(art_view)
world.reset()

num_joints = art_view.num_dof
JOINT_NAMES = list(art_view.dof_names)
print(f"[setup_openarm_scene] Loaded {num_joints} joints: {JOINT_NAMES}")
print("[setup_openarm_scene] Update openarm_isaacsim.yaml with these joint names if different.")

# --- ROS2 bridge node (rclpy.init MUST come after extensions + simulation_app.update) ---
rclpy.init()


class SimBridgeNode(Node):
    def __init__(self):
        super().__init__("isaac_sim_openarm_bridge")
        self._cmd = None
        self._lock = threading.Lock()
        self._pub = self.create_publisher(JointState, "/follower/joint_states", 10)
        self._sub = self.create_subscription(
            Float64MultiArray, "/follower/joint_commands", self._cmd_cb, 10
        )

    def _cmd_cb(self, msg: Float64MultiArray):
        with self._lock:
            self._cmd = list(msg.data)

    def get_command(self):
        with self._lock:
            return self._cmd

    def publish_state(self, names, positions, velocities, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = names
        msg.position = [float(p) for p in positions]
        msg.velocity = [float(v) for v in velocities]
        self._pub.publish(msg)


bridge = SimBridgeNode()
spin_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
spin_thread.start()

# --- Simulation loop ---
CONTROL_HZ = 60
SLEEP_S = 1.0 / CONTROL_HZ

print("[setup_openarm_scene] Simulation running. Ctrl+C to stop.")
try:
    while simulation_app.is_running():
        world.step(render=not args.headless)

        # Read state — shape is (num_robots, num_joints); index [0] for single robot
        positions = art_view.get_joint_positions()[0]
        velocities = art_view.get_joint_velocities()[0]

        stamp = bridge.get_clock().now().to_msg()
        bridge.publish_state(JOINT_NAMES, positions.tolist(), velocities.tolist(), stamp)

        cmd = bridge.get_command()
        if cmd is not None and len(cmd) == num_joints:
            targets = np.array([cmd], dtype=np.float32)  # shape (1, num_joints)
            art_view.set_joint_position_targets(targets)

        time.sleep(SLEEP_S)
except KeyboardInterrupt:
    pass
finally:
    print("[setup_openarm_scene] Shutting down.")
    rclpy.shutdown()
    simulation_app.close()
