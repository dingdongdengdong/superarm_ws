"""
Isaac Sim 5.1.0 scene script — loads OpenArm/Franka URDF and bridges
joint state/commands over ROS2 via isaacsim.ros2.bridge.

Run via:  /isaac-sim/python.sh /workspace/isaacsim/setup_openarm_scene.py

API reference (Isaac Sim 5.1.0):
  SimulationApp      : isaacsim
  World              : isaacsim.core.api
  Articulation       : isaacsim.core.prims
  enable_extension   : isaacsim.core.utils.extensions
  ROS2 bridge ext    : isaacsim.ros2.bridge
  URDF importer ext  : isaacsim.asset.importer.urdf
"""
import argparse
import os
import sys

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true",
                    default=bool(int(os.environ.get("HEADLESS", "1"))))
args, _ = parser.parse_known_args()

# SimulationApp must be the very first Isaac Sim call
from isaacsim import SimulationApp  # noqa: E402
simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

# --- Extensions (after SimulationApp, before everything else) ---
import omni.kit.commands  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension, get_extension_path_from_name  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.importer.urdf")
simulation_app.update()

# --- Core API imports (after extensions are loaded) ---
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

# --- URDF discovery ---
ext_path = get_extension_path_from_name("isaacsim.asset.importer.urdf")
URDF_CANDIDATES = [
    os.environ.get("OPENARM_URDF_PATH", ""),
    "/workspace/superarm_ws/lerobot/lerobot/robots/openarm/assets/openarm.urdf",
    f"{ext_path}/data/urdf/robots/franka_description/robots/panda_arm.urdf",
    f"{ext_path}/data/urdf/robots/franka_description/robots/panda_arm_hand.urdf",
]
urdf_path = next((p for p in URDF_CANDIDATES if p and os.path.isfile(p)), None)

if urdf_path is None:
    print(
        "[setup_openarm_scene] ERROR: No URDF found. Tried:\n"
        + "\n".join(f"  {p}" for p in URDF_CANDIDATES if p)
        + f"\n\nSet OPENARM_URDF_PATH, or the Franka fallback is at:\n"
        f"  {ext_path}/data/urdf/robots/franka_description/robots/panda_arm.urdf"
    )
    simulation_app.close()
    sys.exit(1)

print(f"[setup_openarm_scene] Loading URDF: {urdf_path}")

# --- URDF import (Isaac Sim 5.1 API) ---
status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.fix_base = True
import_config.import_inertia_tensor = True
import_config.distance_scale = 1.0
import_config.default_drive_type = 1       # position drive
import_config.default_drive_strength = 1047.2
import_config.default_position_drive_damping = 52.4

status, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=urdf_path,
    import_config=import_config,
    dest_path="/World/OpenArm",
    get_articulation_root=True,
)
assert status, f"[setup_openarm_scene] URDF import failed: {urdf_path}"
prim_path = prim_path or "/World/OpenArm"

# --- World setup ---
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
world.reset()

# --- Articulation (initialize after world.reset()) ---
art = Articulation(prim_path)
art.initialize()

num_dof = art.num_dof
JOINT_NAMES = list(art.dof_names)
print(f"[setup_openarm_scene] Loaded {num_dof} joints: {JOINT_NAMES}")
print("[setup_openarm_scene] Update openarm_isaacsim.yaml with these joint names if different.")

# --- ROS2 bridge node ---
# rclpy.init() must be called AFTER enable_extension("isaacsim.ros2.bridge") + simulation_app.update()
rclpy.init()


class SimBridgeNode(Node):
    def __init__(self):
        super().__init__("isaac_sim_openarm_bridge")
        self._cmd = None
        self._pub = self.create_publisher(JointState, "/follower/joint_states", 10)
        self._sub = self.create_subscription(
            Float64MultiArray, "/follower/joint_commands", self._cmd_cb, 10
        )

    def _cmd_cb(self, msg: Float64MultiArray):
        self._cmd = list(msg.data)

    def get_command(self):
        return self._cmd

    def publish_state(self, names, positions, velocities, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = names
        msg.position = [float(p) for p in positions]
        msg.velocity = [float(v) for v in velocities]
        self._pub.publish(msg)


bridge = SimBridgeNode()

# Start the timeline
import omni.timeline  # noqa: E402
timeline = omni.timeline.get_timeline_interface()
timeline.play()

# --- Simulation loop ---
# Isaac Sim 5.1 pattern: world.step() + rclpy.spin_once() in main loop (no threads needed)
print("[setup_openarm_scene] Simulation running. Ctrl+C to stop.")
try:
    while simulation_app.is_running():
        world.step(render=not args.headless)

        # spin_once processes any incoming /follower/joint_commands messages
        rclpy.spin_once(bridge, timeout_sec=0.0)

        # Publish current joint state — shape (1, num_dof), index [0] for single robot
        positions = art.get_joint_positions()[0]
        velocities = art.get_joint_velocities()[0]
        stamp = bridge.get_clock().now().to_msg()
        bridge.publish_state(JOINT_NAMES, positions.tolist(), velocities.tolist(), stamp)

        # Apply command if received
        cmd = bridge.get_command()
        if cmd is not None and len(cmd) == num_dof:
            art.set_joint_positions(np.array([cmd], dtype=np.float32))

except KeyboardInterrupt:
    pass
finally:
    print("[setup_openarm_scene] Shutting down.")
    timeline.stop()
    bridge.destroy_node()
    rclpy.shutdown()
    simulation_app.close()
