"""Static checks for the AmazingHand hand-only Isaac scene bridge."""

from __future__ import annotations

from pathlib import Path


def test_hand_only_scene_bridge_declares_isolated_hand_topics_and_generated_urdf() -> None:
    scene_path = Path(__file__).resolve().parent / "isaacsim" / "setup_amazinghand_scene.py"
    text = scene_path.read_text(encoding="utf-8")

    assert "amazinghand_graspable.urdf" in text
    assert '"/hand/joint_states"' in text
    assert '"/hand/joint_commands"' in text
    assert '"/hand/screenshot_debug"' in text
    assert "HAND_ACTUATED_JOINT_NAMES" in text


def test_hand_only_scene_bridge_uses_ros_messages_and_screenshot_debug_json() -> None:
    scene_path = Path(__file__).resolve().parent / "isaacsim" / "setup_amazinghand_scene.py"
    text = scene_path.read_text(encoding="utf-8")

    assert "JointState" in text
    assert "Float64MultiArray" in text
    assert "String" in text
    assert "capture_every_command" in text
    assert "request_capture" in text
    assert "output_dir" in text
