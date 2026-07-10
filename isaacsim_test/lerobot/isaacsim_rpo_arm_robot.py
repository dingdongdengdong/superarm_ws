"""LeRobot robot shim for Isaac Sim RoboParty V2.0 right arm + AmazingHand."""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


try:
    from isaacsim_test.isaacsim.graspable_hand_urdf import (
        HAND_ACTUATED_JOINT_NAMES,
        grasp_scalar_to_hand_joint_targets,
    )
except ModuleNotFoundError:
    try:
        from graspable_hand_urdf import HAND_ACTUATED_JOINT_NAMES, grasp_scalar_to_hand_joint_targets
    except ModuleNotFoundError:
        HAND_ACTUATED_JOINT_NAMES = [
            "finger1_motor1",
            "finger1_motor2",
            "finger2_motor1",
            "finger2_motor2",
            "finger3_motor1",
            "finger3_motor2",
            "finger4_motor1",
            "finger4_motor2",
        ]

        def grasp_scalar_to_hand_joint_targets(grasp: float) -> dict[str, float]:
            closedness = max(0.0, min(1.0, float(grasp)))
            targets: dict[str, float] = {}
            for finger_index in range(1, 5):
                targets[f"finger{finger_index}_motor1"] = 0.05 + closedness * 0.90
                targets[f"finger{finger_index}_motor2"] = 0.02 + closedness * 1.08
            return targets

try:  # LeRobot versions used by older containers.
    from lerobot.common.robot_devices.robots.configs import RobotConfig
except ModuleNotFoundError:
    try:  # Newer upstream LeRobot checkout layout.
        from lerobot.robots.config import RobotConfig
    except ModuleNotFoundError:
        class RobotConfig:  # type: ignore[no-redef]
            """Small local fallback so contract tests can run without LeRobot installed."""

            @classmethod
            def register_subclass(cls, _name: str):
                def _decorator(subclass):
                    return subclass

                return _decorator


ARM_JOINT_NAMES = [
    "right_arm_pitch_joint",
    "right_arm_roll_joint",
    "right_arm_yaw_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
]
SYNTHETIC_GRASP_NAME = "amazinghand_grasp"
JOINT_NAMES = [
    *ARM_JOINT_NAMES,
    "amazinghand_grasp",
]
FEATURE_KEYS = [f"{name}.pos" for name in JOINT_NAMES]


def _float64_multi_array_message():
    try:
        from std_msgs.msg import Float64MultiArray
    except ModuleNotFoundError:
        class Float64MultiArray:  # type: ignore[no-redef]
            def __init__(self) -> None:
                self.data = []

    return Float64MultiArray()


def hand_grasp_scalar_action(grasp: float) -> list[float]:
    """Return the canonical 8D AmazingHand action vector for a normalized grasp."""
    targets = grasp_scalar_to_hand_joint_targets(grasp)
    return [round(float(targets[name]), 6) for name in HAND_ACTUATED_JOINT_NAMES]


@RobotConfig.register_subclass("isaacsim_rpo_arm")
@dataclass
class IsaacSimRpoArmConfig(RobotConfig):
    joint_names: list[str] = field(default_factory=lambda: list(JOINT_NAMES))
    joint_state_topic: str = "/follower/joint_states"
    joint_command_topic: str = "/follower/joint_commands"
    phone_command_topic: str = "/leader/joint_commands"
    screenshot_debug_topic: str = "/follower/screenshot_debug"
    connect_timeout_s: float = 10.0
    fixed_hand: bool = False
    fixed_grasp: float = 0.0
    allow_custom_joint_names: bool = False
    mock: bool = False


class IsaacSimRpoArmRobot:
    robot_type = "isaacsim_rpo_arm"

    def __init__(self, config: IsaacSimRpoArmConfig):
        self.config = config
        self._node = None
        self._spin_thread = None
        self._latest_positions: Optional[list[float]] = None
        self._latest_phone_cmd: Optional[list[float]] = None
        self._state_lock = threading.Lock()
        self._cmd_lock = threading.Lock()
        self._pub = None
        self._debug_pub = None
        self.is_connected = False
        self.cameras = {}

    @property
    def camera_features(self) -> dict:
        return {}

    @property
    def motor_features(self) -> dict:
        feature_keys = self._feature_keys
        return {
            "observation.state": {
                "dtype": "float32",
                "shape": (len(feature_keys),),
                "names": feature_keys,
            },
            "action": {
                "dtype": "float32",
                "shape": (len(feature_keys),),
                "names": feature_keys,
            },
        }

    @property
    def features(self) -> dict:
        return self.motor_features

    @property
    def _feature_keys(self) -> list[str]:
        return [f"{name}.pos" for name in self.config.joint_names]

    def connect(self):
        if self.config.mock:
            print("[IsaacSimRpoArmRobot] Mock mode - skipping ROS2 connection.")
            self._latest_positions = [0.0] * len(self.config.joint_names)
            self.is_connected = True
            return

        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray, String

        try:
            rclpy.init()
        except RuntimeError:
            pass

        self._node = Node("isaacsim_rpo_arm_lerobot_bridge")
        self._pub = self._node.create_publisher(
            Float64MultiArray, self.config.joint_command_topic, 10
        )
        self._debug_pub = self._node.create_publisher(
            String, self.config.screenshot_debug_topic, 10
        )
        self._node.create_subscription(
            JointState, self.config.joint_state_topic, self._joint_state_cb, 10
        )
        self._node.create_subscription(
            Float64MultiArray, self.config.phone_command_topic, self._phone_cmd_cb, 10
        )

        self._spin_thread = threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()

        deadline = time.time() + self.config.connect_timeout_s
        while time.time() < deadline:
            with self._state_lock:
                if self._latest_positions is not None:
                    print(f"[IsaacSimRpoArmRobot] Connected. Joints: {self.config.joint_names}")
                    self.is_connected = True
                    return
            time.sleep(0.1)

        raise TimeoutError(
            f"No joint states received on '{self.config.joint_state_topic}' "
            f"within {self.config.connect_timeout_s}s. Is Isaac Sim running, and does "
            "ROS_DOMAIN_ID match in both containers?"
        )

    def _joint_state_cb(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position, strict=False))
        positions = [
            float(name_to_pos.get(name, 0.0))
            for name in self.config.joint_names
        ]
        with self._state_lock:
            self._latest_positions = positions

    def _phone_cmd_cb(self, msg):
        with self._cmd_lock:
            self._latest_phone_cmd = self._normalize_vector(list(msg.data))

    def _normalize_vector(self, values: list[float]) -> list[float]:
        target_len = len(self.config.joint_names)
        normalized = [float(v) for v in values[:target_len]]
        if len(normalized) < target_len:
            for joint_name in self.config.joint_names[len(normalized):]:
                if joint_name == SYNTHETIC_GRASP_NAME:
                    normalized.append(float(self.config.fixed_grasp))
                else:
                    normalized.append(0.0)

        for idx, joint_name in enumerate(self.config.joint_names):
            if joint_name != SYNTHETIC_GRASP_NAME:
                continue
            if self.config.fixed_hand:
                normalized[idx] = float(np.clip(self.config.fixed_grasp, 0.0, 1.0))
            else:
                normalized[idx] = float(np.clip(normalized[idx], 0.0, 1.0))
        return normalized

    def run_calibration(self):
        print("[IsaacSimRpoArmRobot] Simulated robot - no calibration needed.")

    def capture_observation(self) -> dict:
        with self._state_lock:
            positions = (
                list(self._latest_positions)
                if self._latest_positions is not None
                else [0.0] * len(self.config.joint_names)
            )
        return {"observation.state": np.array(positions, dtype=np.float32)}

    def teleop_step(self, record_data: bool = False):
        with self._cmd_lock:
            phone_cmd = list(self._latest_phone_cmd) if self._latest_phone_cmd else None

        if phone_cmd is None:
            with self._state_lock:
                phone_cmd = (
                    list(self._latest_positions)
                    if self._latest_positions is not None
                    else [0.0] * len(self.config.joint_names)
                )

        phone_cmd = self._normalize_vector(phone_cmd)
        msg = _float64_multi_array_message()
        msg.data = phone_cmd
        if self._pub is not None:
            self._pub.publish(msg)

        if not record_data:
            return

        obs = self.capture_observation()
        action = {"action": np.array(phone_cmd, dtype=np.float32)}
        return obs, action

    def send_action(self, action) -> np.ndarray:
        if isinstance(action, dict):
            if "action" in action:
                positions = action["action"]
            else:
                positions = [action[key] for key in self._feature_keys]
        else:
            positions = action

        positions = self._normalize_vector(np.asarray(positions, dtype=np.float32).reshape(-1).tolist())
        if self.config.mock:
            with self._state_lock:
                self._latest_positions = list(positions)
            return np.array(positions, dtype=np.float32)
        if self._pub is None:
            raise RuntimeError("IsaacSimRpoArmRobot is not connected; call connect() before send_action().")

        msg = _float64_multi_array_message()
        msg.data = positions
        self._pub.publish(msg)
        with self._cmd_lock:
            self._latest_phone_cmd = list(positions)
        return np.array(positions, dtype=np.float32)

    def publish_screenshot_debug(self, payload: dict) -> dict:
        """Publish a screenshot debug-control JSON message for the Isaac bridge."""
        if self._debug_pub is None:
            raise RuntimeError("Isaac Sim screenshot debug publisher is not connected.")
        data = json.dumps(payload, sort_keys=True)
        try:
            from std_msgs.msg import String
        except ModuleNotFoundError:
            class String:  # type: ignore[no-redef]
                def __init__(self) -> None:
                    self.data = ""

        msg = String()
        msg.data = data
        self._debug_pub.publish(msg)
        return payload

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
        self.is_connected = False
        print("[IsaacSimRpoArmRobot] Disconnected.")
