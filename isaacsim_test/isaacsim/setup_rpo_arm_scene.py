"""
Isaac Sim 5.1.0 scene script for the RoboParty/Roboto Origin V2.0
right arm plus a synthetic AmazingHand grasp channel.

Run via: /isaac-sim/python.sh /workspace/isaacsim/setup_rpo_arm_scene.py
"""
from __future__ import annotations

import argparse
import os
import sys
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

import omni.timeline  # noqa: E402

timeline = omni.timeline.get_timeline_interface()
timeline.play()

print("[setup_rpo_arm_scene] Simulation running. Ctrl+C to stop.", flush=True)
try:
    app_updated = False
    while simulation_app.is_running():
        # In this container build, repeated Kit/world stepping can block after the
        # first frame. Keep a mirrored ROS joint bridge responsive and push the
        # commanded official V2 right-arm joints into Isaac best-effort for visualization.
        if not app_updated:
            simulation_app.update()
            app_updated = True
        rclpy.spin_once(bridge, timeout_sec=0.0)

        stamp = bridge.get_clock().now().to_msg()
        bridge.publish_state(PUBLISHED_JOINT_NAMES, current_positions, current_velocities, stamp)

        cmd = bridge.get_command()
        if cmd is not None and len(cmd) >= len(CONTROLLED_ARM_JOINT_NAMES):
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
        time.sleep(1.0 / 60.0)

except KeyboardInterrupt:
    pass
finally:
    print("[setup_rpo_arm_scene] Shutting down.", flush=True)
    timeline.stop()
    bridge.destroy_node()
    rclpy.shutdown()
    simulation_app.close()
