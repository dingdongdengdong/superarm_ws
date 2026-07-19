from __future__ import annotations

from pathlib import Path


SCENE_PATH = Path(__file__).resolve().parent / "isaacsim" / "setup_rpo_arm_scene.py"


def test_scene_bridge_subscribes_to_screenshot_debug_control_topic() -> None:
    source = SCENE_PATH.read_text(encoding="utf-8")

    assert "from std_msgs.msg import Float64MultiArray, String" in source
    assert '"/follower/screenshot_debug"' in source
    assert "_debug_control_cb" in source


def test_scene_debug_control_supports_enable_one_shot_and_output_dir() -> None:
    source = SCENE_PATH.read_text(encoding="utf-8")

    assert "capture_every_command" in source
    assert "request_capture" in source
    assert "output_dir" in source
    assert "debug_one_shot_requested" in source


def test_scene_expands_six_logical_controls_to_physical_hand_joints() -> None:
    source = SCENE_PATH.read_text(encoding="utf-8")

    assert "LOGICAL_HAND_MOTION_NAME" in source
    assert "resolve_logical_command" in source
    assert "physical_joint_names" in source
    assert "resolved_logical_command" in source
    assert "invalid logical command" in source
