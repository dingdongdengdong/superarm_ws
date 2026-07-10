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
    assert "_capture_replicator" in text
    assert "BasicWriter" in text
    assert "Camera.get_rgba had no data after 60 frames" in text


def test_hand_only_scene_bridge_supports_isaacsim60_urdf_importer_api() -> None:
    scene_path = Path(__file__).resolve().parent / "isaacsim" / "setup_amazinghand_scene.py"
    text = scene_path.read_text(encoding="utf-8")

    assert "URDFImporter" in text
    assert "URDFImporterConfig" in text
    assert "UrdfJointTargetType = None" in text


def test_hand_only_scene_bridge_remaps_host_mesh_paths_inside_container() -> None:
    scene_path = Path(__file__).resolve().parent / "isaacsim" / "setup_amazinghand_scene.py"
    text = scene_path.read_text(encoding="utf-8")

    assert "_container_accessible_urdf_path" in text
    assert "/isaacsim_test/inputs/robot_arm_hand_package/" in text
    assert "remapped host URDF mesh paths for container" in text
    assert "urdf_path = _container_accessible_urdf_path(urdf_path)" in text
    assert "HAND_SCREENSHOT_OUTPUT_DIR" in text
    assert "AMAZINGHAND_USD_OUTPUT_DIR" in text


def test_hand_only_scene_bridge_applies_ros_commands_on_simulation_thread() -> None:
    scene_path = Path(__file__).resolve().parent / "isaacsim" / "setup_amazinghand_scene.py"
    text = scene_path.read_text(encoding="utf-8")

    assert "_pending_targets" in text
    assert "_active_targets" in text
    assert "apply_pending_command" in text
    assert "hold_active_command" in text
    assert "bridge.apply_pending_command()" in text
    assert "bridge.hold_active_command()" in text
    assert "apply_pending_capture" in text
    assert "bridge.apply_pending_capture()" in text
    assert "HAND_COMMAND_EVIDENCE_PATH" in text
    assert "root_prim_path" in text
