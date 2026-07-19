"""LeRobot robot shim for Isaac Sim RoboParty V2.0 right arm + AmazingHand."""

from __future__ import annotations

import json
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


# LeLab imports this shim as a top-level module while tests and other tooling may
# import it through `isaacsim_test.lerobot`. Keep both names on one module object
# so Draccus sees one registered RobotConfig subclass instead of two aliases.
sys.modules.setdefault("isaacsim_rpo_arm_robot", sys.modules[__name__])
sys.modules.setdefault("isaacsim_test.lerobot.isaacsim_rpo_arm_robot", sys.modules[__name__])


try:
    from isaacsim_test.isaacsim.graspable_hand_urdf import (
        HAND_ACTUATED_JOINT_NAMES,
        fixed_hand_motion_library,
        grasp_scalar_to_hand_joint_targets,
        resolve_fixed_hand_motion,
    )
except ModuleNotFoundError:
    try:
        from graspable_hand_urdf import (
            HAND_ACTUATED_JOINT_NAMES,
            fixed_hand_motion_library,
            grasp_scalar_to_hand_joint_targets,
            resolve_fixed_hand_motion,
        )
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

        def fixed_hand_motion_library() -> list[dict]:
            return [
                {"name": "open", "code": 0.0, "joint_targets": grasp_scalar_to_hand_joint_targets(0.0)},
                {
                    "name": "half_close",
                    "code": 0.5,
                    "joint_targets": grasp_scalar_to_hand_joint_targets(0.5),
                },
                {"name": "close", "code": 1.0, "joint_targets": grasp_scalar_to_hand_joint_targets(1.0)},
            ]

        def resolve_fixed_hand_motion(value: float, *, previous_code=None, hysteresis=0.05) -> dict:
            del previous_code, hysteresis
            command = float(np.clip(value, 0.0, 1.0))
            return min(fixed_hand_motion_library(), key=lambda motion: abs(command - motion["code"]))

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

try:
    from lerobot.robots.robot import Robot
except ModuleNotFoundError:
    class Robot:  # type: ignore[no-redef]
        """Fallback base for source-level contract tests without LeRobot installed."""

        def __init__(self, config) -> None:
            self.config = config


ARM_JOINT_NAMES = [
    "right_arm_pitch_joint",
    "right_arm_roll_joint",
    "right_arm_yaw_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
]
SYNTHETIC_GRASP_NAME = "amazinghand_grasp"
HAND_MOTION_NAME = "amazinghand_motion"
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
    physical_joint_names: list[str] = field(default_factory=list)
    combined_urdf_path: str | None = None
    motion_hysteresis: float = 0.05
    arm_limits: dict[str, dict[str, float]] = field(default_factory=dict)
    hand_motions: list[dict] = field(default_factory=list)
    so101_leader_mapping: list[dict] = field(default_factory=list)
    so101_gripper_feature: str = "gripper.pos"
    mock: bool = False


class IsaacSimRpoArm(Robot):
    config_class = IsaacSimRpoArmConfig
    name = "isaacsim_rpo_arm"

    def __init__(self, config: IsaacSimRpoArmConfig):
        super().__init__(config)
        self.config = config
        self._node = None
        self._spin_thread = None
        self._latest_positions: Optional[list[float]] = None
        self._latest_phone_cmd: Optional[list[float]] = None
        self._state_lock = threading.Lock()
        self._cmd_lock = threading.Lock()
        self._pub = None
        self._debug_pub = None
        self._is_connected = False
        self._active_motion_code = 0.0
        self._latest_physical_positions: dict[str, float] = {}
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
    def observation_features(self) -> dict[str, type]:
        return {key: float for key in self._feature_keys}

    @property
    def action_features(self) -> dict[str, type]:
        return {key: float for key in self._feature_keys}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    @property
    def _feature_keys(self) -> list[str]:
        return [f"{name}.pos" for name in self.config.joint_names]

    def connect(self, calibrate: bool = True):
        del calibrate
        if self.is_connected:
            return
        if self.config.mock:
            print("[IsaacSimRpoArmRobot] Mock mode - skipping ROS2 connection.")
            self._latest_positions = [0.0] * len(self.config.joint_names)
            self._latest_physical_positions = self._expanded_physical_positions(
                self._latest_positions
            )
            self._is_connected = True
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
                    self._is_connected = True
                    return
            time.sleep(0.1)

        raise TimeoutError(
            f"No joint states received on '{self.config.joint_state_topic}' "
            f"within {self.config.connect_timeout_s}s. Is Isaac Sim running, and does "
            "ROS_DOMAIN_ID match in both containers?"
        )

    def _joint_state_cb(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position, strict=False))
        physical_names = self.config.physical_joint_names or self.config.joint_names
        physical_positions = {
            name: float(name_to_pos.get(name, 0.0)) for name in physical_names
        }
        if HAND_MOTION_NAME in self.config.joint_names:
            motion = min(
                fixed_hand_motion_library(),
                key=lambda candidate: sum(
                    (
                        physical_positions.get(name, 0.0)
                        - candidate["joint_targets"][name]
                    )
                    ** 2
                    for name in HAND_ACTUATED_JOINT_NAMES
                ),
            )
            self._active_motion_code = float(motion["code"])
        positions = [
            self._active_motion_code
            if name == HAND_MOTION_NAME
            else float(name_to_pos.get(name, 0.0))
            for name in self.config.joint_names
        ]
        with self._state_lock:
            self._latest_positions = positions
            self._latest_physical_positions = physical_positions

    def _phone_cmd_cb(self, msg):
        with self._cmd_lock:
            self._latest_phone_cmd = self._normalize_vector(list(msg.data))

    def _normalize_vector(self, values: list[float]) -> list[float]:
        target_len = len(self.config.joint_names)
        normalized = [float(v) for v in values[:target_len]]
        if len(normalized) < target_len:
            for joint_name in self.config.joint_names[len(normalized):]:
                if joint_name in {SYNTHETIC_GRASP_NAME, HAND_MOTION_NAME}:
                    normalized.append(float(self.config.fixed_grasp))
                else:
                    normalized.append(0.0)

        for idx, joint_name in enumerate(self.config.joint_names):
            if joint_name == HAND_MOTION_NAME:
                resolved = resolve_fixed_hand_motion(
                    normalized[idx],
                    previous_code=self._active_motion_code,
                    hysteresis=self.config.motion_hysteresis,
                )
                normalized[idx] = float(resolved["code"])
                self._active_motion_code = float(resolved["code"])
                continue
            if joint_name != SYNTHETIC_GRASP_NAME:
                continue
            if self.config.fixed_hand:
                normalized[idx] = float(np.clip(self.config.fixed_grasp, 0.0, 1.0))
            else:
                normalized[idx] = float(np.clip(normalized[idx], 0.0, 1.0))
        return normalized

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def run_calibration(self):
        print("[IsaacSimRpoArmRobot] Simulated robot - no calibration needed.")

    def get_observation(self) -> dict[str, float]:
        if not self.is_connected:
            raise RuntimeError("IsaacSimRpoArm is not connected; call connect() before get_observation().")
        with self._state_lock:
            positions = (
                list(self._latest_positions)
                if self._latest_positions is not None
                else [0.0] * len(self.config.joint_names)
            )
        return dict(zip(self._feature_keys, positions, strict=True))

    def capture_observation(self) -> dict:
        observation = self.get_observation()
        positions = [observation[key] for key in self._feature_keys]
        return {"observation.state": np.array(positions, dtype=np.float32)}

    def _expanded_physical_positions(self, logical_positions: list[float]) -> dict[str, float]:
        logical = dict(zip(self.config.joint_names, logical_positions, strict=False))
        physical = {
            name: float(logical.get(name, 0.0))
            for name in (self.config.physical_joint_names or self.config.joint_names)
            if name not in HAND_ACTUATED_JOINT_NAMES
        }
        if HAND_MOTION_NAME in logical:
            motion = resolve_fixed_hand_motion(
                logical[HAND_MOTION_NAME],
                previous_code=self._active_motion_code,
                hysteresis=self.config.motion_hysteresis,
            )
            physical.update(motion["joint_targets"])
        return physical

    def get_visualization_joints(self) -> dict[str, float]:
        """Return physical joint values for the URDF viewer, separate from 6D policy state."""
        with self._state_lock:
            if self._latest_physical_positions:
                return dict(self._latest_physical_positions)
            logical = list(self._latest_positions or [0.0] * len(self.config.joint_names))
        return self._expanded_physical_positions(logical)

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

    def send_action(self, action):
        named_action = isinstance(action, dict) and "action" not in action
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
                self._latest_physical_positions = self._expanded_physical_positions(positions)
            if named_action:
                return dict(zip(self._feature_keys, positions, strict=True))
            return np.array(positions, dtype=np.float32)
        if self._pub is None:
            raise RuntimeError("IsaacSimRpoArmRobot is not connected; call connect() before send_action().")

        msg = _float64_multi_array_message()
        msg.data = positions
        self._pub.publish(msg)
        with self._cmd_lock:
            self._latest_phone_cmd = list(positions)
        if named_action:
            return dict(zip(self._feature_keys, positions, strict=True))
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
        self._is_connected = False
        print("[IsaacSimRpoArmRobot] Disconnected.")


# Backward-compatible import used by the existing LeLab teleoperation hook.
IsaacSimRpoArmRobot = IsaacSimRpoArm
