"""
LeRobot robot class for Isaac Sim OpenArm follower.

Implements the lerobot.common.robot_devices.robots.utils.Robot protocol
(old API, compatible with lerobot commit 2d7a420).

Register by adding to lerobot/common/robot_devices/robots/utils.py:

    # in make_robot_config():
    elif robot_type == "isaacsim_openarm":
        from isaacsim_robot import IsaacSimOpenArmConfig
        return IsaacSimOpenArmConfig(**kwargs)

    # in make_robot_from_config():
    elif isinstance(config, IsaacSimOpenArmConfig):
        from isaacsim_robot import IsaacSimOpenArmRobot
        return IsaacSimOpenArmRobot(config)
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class IsaacSimOpenArmConfig:
    # Joint names must match dof_names printed by setup_openarm_scene.py at startup.
    joint_names: list[str] = field(default_factory=lambda: [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ])
    joint_state_topic: str = "/follower/joint_states"
    joint_command_topic: str = "/follower/joint_commands"
    phone_command_topic: str = "/leader/joint_commands"
    connect_timeout_s: float = 10.0
    mock: bool = False


class IsaacSimOpenArmRobot:
    robot_type = "isaacsim_openarm"

    def __init__(self, config: IsaacSimOpenArmConfig):
        self.config = config
        self._node = None
        self._spin_thread = None
        self._latest_positions: Optional[list[float]] = None
        self._latest_phone_cmd: Optional[list[float]] = None
        self._state_lock = threading.Lock()
        self._cmd_lock = threading.Lock()
        self._pub = None

    @property
    def features(self) -> dict:
        n = len(self.config.joint_names)
        return {
            "observation.state": {
                "dtype": "float32",
                "shape": (n,),
                "names": {"joints": self.config.joint_names},
            },
            "action": {
                "dtype": "float32",
                "shape": (n,),
                "names": {"joints": self.config.joint_names},
            },
        }

    def connect(self):
        if self.config.mock:
            print("[IsaacSimOpenArmRobot] Mock mode — skipping ROS2 connection.")
            self._latest_positions = [0.0] * len(self.config.joint_names)
            return

        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray

        try:
            rclpy.init()
        except RuntimeError:
            pass  # already initialized

        self._node = Node("isaacsim_lerobot_bridge")

        self._pub = self._node.create_publisher(
            Float64MultiArray, self.config.joint_command_topic, 10
        )

        self._node.create_subscription(
            JointState, self.config.joint_state_topic, self._joint_state_cb, 10
        )
        self._node.create_subscription(
            Float64MultiArray, self.config.phone_command_topic, self._phone_cmd_cb, 10
        )

        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True
        )
        self._spin_thread.start()

        # Block until first joint state arrives
        deadline = time.time() + self.config.connect_timeout_s
        while time.time() < deadline:
            with self._state_lock:
                if self._latest_positions is not None:
                    print(
                        f"[IsaacSimOpenArmRobot] Connected. "
                        f"Joints: {self.config.joint_names}"
                    )
                    return
            time.sleep(0.1)

        raise TimeoutError(
            f"No joint states received on '{self.config.joint_state_topic}' "
            f"within {self.config.connect_timeout_s}s. "
            "Is Isaac Sim running? Check: ROS_DOMAIN_ID matches in both containers."
        )

    def _joint_state_cb(self, msg):
        positions = list(msg.position)
        with self._state_lock:
            self._latest_positions = positions

    def _phone_cmd_cb(self, msg):
        with self._cmd_lock:
            self._latest_phone_cmd = list(msg.data)

    def run_calibration(self):
        print("[IsaacSimOpenArmRobot] Simulated robot — no calibration needed.")

    def capture_observation(self) -> dict:
        with self._state_lock:
            positions = list(self._latest_positions) if self._latest_positions else \
                [0.0] * len(self.config.joint_names)
        return {"observation.state": np.array(positions, dtype=np.float32)}

    def teleop_step(self, record_data: bool = False):
        from std_msgs.msg import Float64MultiArray

        with self._cmd_lock:
            phone_cmd = list(self._latest_phone_cmd) if self._latest_phone_cmd else None

        if phone_cmd is None:
            # Hold current positions if phone hasn't sent anything yet
            with self._state_lock:
                phone_cmd = list(self._latest_positions) if self._latest_positions else \
                    [0.0] * len(self.config.joint_names)

        msg = Float64MultiArray()
        msg.data = [float(v) for v in phone_cmd]
        if self._pub is not None:
            self._pub.publish(msg)

        if not record_data:
            return

        obs = self.capture_observation()
        action = {"action": np.array(phone_cmd, dtype=np.float32)}
        return obs, action

    def send_action(self, action: dict) -> dict:
        from std_msgs.msg import Float64MultiArray

        positions = action["action"].tolist()
        msg = Float64MultiArray()
        msg.data = [float(v) for v in positions]
        if self._pub is not None:
            self._pub.publish(msg)
        return action

    def disconnect(self):
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        try:
            import rclpy
            rclpy.shutdown()
        except Exception:
            pass
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
            self._spin_thread = None
        print("[IsaacSimOpenArmRobot] Disconnected.")
