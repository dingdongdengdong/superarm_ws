#!/usr/bin/env python3
"""Mirror LeLab joint-position HTTP data into ROS 2 topics for Foxglove.

This script is intentionally read-only with respect to the robot: it only polls
LeLab's /joint-positions endpoint and publishes the returned state. It never
sends /send-joint-action, torque-enable, or motion commands.
"""

from __future__ import annotations

import argparse
import json
import time
import urllib.error
import urllib.request
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class LeLabJointMirror(Node):
    def __init__(self, *, base_url: str, joint_name: str, hz: float) -> None:
        super().__init__("lelab_dm4340p_joint_mirror")
        self.base_url = base_url.rstrip("/")
        self.joint_name = joint_name
        self.pub_joint_state = self.create_publisher(
            JointState, "/dm4340p/first_motor/joint_states", 10
        )
        self.pub_status = self.create_publisher(
            String, "/dm4340p/first_motor/status", 10
        )
        self.timer = self.create_timer(1.0 / hz, self.tick)
        self.get_logger().info(
            f"Mirroring {self.base_url}/joint-positions as joint '{self.joint_name}'"
        )

    def _fetch_joint_positions(self) -> dict[str, Any]:
        with urllib.request.urlopen(
            f"{self.base_url}/joint-positions", timeout=0.25
        ) as response:
            return json.loads(response.read().decode("utf-8"))

    def tick(self) -> None:
        status = String()
        try:
            payload = self._fetch_joint_positions()
            payload["mirror_time"] = time.time()
            status.data = json.dumps(payload, sort_keys=True)
            self.pub_status.publish(status)

            positions = payload.get("joint_positions") or {}
            if payload.get("success") is True and self.joint_name in positions:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = [self.joint_name]
                msg.position = [float(positions[self.joint_name])]
                self.pub_joint_state.publish(msg)
        except (OSError, urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            status.data = json.dumps(
                {"success": False, "error": str(exc), "mirror_time": time.time()},
                sort_keys=True,
            )
            self.pub_status.publish(status)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://127.0.0.1:8000")
    parser.add_argument("--joint-name", default="first_motor")
    parser.add_argument("--hz", type=float, default=10.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.hz <= 0:
        raise SystemExit("--hz must be positive")
    rclpy.init()
    node = LeLabJointMirror(
        base_url=args.base_url, joint_name=args.joint_name, hz=args.hz
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
