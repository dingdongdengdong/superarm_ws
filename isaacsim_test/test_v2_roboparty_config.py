"""Static contract checks for the Isaac Sim physical asset testbed.

Run with:
    python3 isaacsim_test/test_v2_roboparty_config.py
"""

from __future__ import annotations

import json
import math
import re
import struct
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
V2_URDF = ROOT / "roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/urdf/roboto_origin.urdf"
REQUIRED_ARM_JOINTS = [
    "right_arm_pitch_joint",
    "right_arm_roll_joint",
    "right_arm_yaw_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
]
FEATURE_JOINTS = [*REQUIRED_ARM_JOINTS, "amazinghand_grasp"]
ARTICULATED_SIMREADY_USD = (
    ROOT
    / "isaacsim_test/outputs/simready/echo_full/sitl/"
    "echo_full_lerobot_articulation.usda"
)
ARTICULATION_REPORT = (
    ROOT
    / "isaacsim_test/outputs/simready/echo_full/sitl/"
    "echo_full_lerobot_articulation_report.json"
)
DIRECT_PHYSICAL_URDF = (
    ROOT
    / "isaacsim_test/outputs/simready/echo_full/sitl/"
    "roboto_v2_right_arm_amazinghand_full.urdf"
)
FULL_ARM_HAND_URDF = (
    ROOT
    / "isaacsim_test/outputs/simready/echo_full/sitl/"
    "roboto_v2_right_arm_amazinghand_full.urdf"
)
RIGHT_ELBOW_YAW_LINK_MESH = (
    ROOT
    / "roboparty/modules/rpo_hardware/V2.0/roboto_origin_mechanic/03_URDF/meshes/"
    "right_elbow_yaw_link.STL"
)


def _read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def _float_list(raw: str) -> list[float]:
    return [float(part) for part in raw.split()]


def _urdf_joint_topology(urdf_path: Path) -> list[dict[str, object]]:
    root = ET.parse(urdf_path).getroot()
    joints_by_name = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    topology = []
    for joint_name in REQUIRED_ARM_JOINTS:
        joint = joints_by_name[joint_name]
        origin = joint.find("origin")
        axis = joint.find("axis")
        limit = joint.find("limit")
        assert origin is not None
        assert axis is not None
        assert limit is not None
        topology.append(
            {
                "joint": joint_name,
                "parent": joint.find("parent").attrib["link"],  # type: ignore[union-attr]
                "child": joint.find("child").attrib["link"],  # type: ignore[union-attr]
                "origin_xyz": _float_list(origin.attrib.get("xyz", "0 0 0")),
                "origin_rpy": _float_list(origin.attrib.get("rpy", "0 0 0")),
                "axis_xyz": _float_list(axis.attrib["xyz"]),
                "limit_lower": float(limit.attrib["lower"]),
                "limit_upper": float(limit.attrib["upper"]),
                "effort": float(limit.attrib["effort"]),
                "velocity": float(limit.attrib["velocity"]),
            }
        )
    return topology


def _reference_urdf_joint_topology() -> list[dict[str, object]]:
    return _urdf_joint_topology(V2_URDF)


def _stl_bbox(stl_path: Path) -> tuple[list[float], list[float]]:
    data = stl_path.read_bytes()
    vertices: list[tuple[float, float, float]] = []
    if len(data) >= 84:
        triangle_count = struct.unpack("<I", data[80:84])[0]
        if 84 + (50 * triangle_count) == len(data):
            offset = 84
            for _ in range(triangle_count):
                offset += 12  # normal
                for _ in range(3):
                    vertices.append(struct.unpack("<fff", data[offset : offset + 12]))
                    offset += 12
                offset += 2
    if not vertices:
        text = data.decode(errors="ignore")
        vertices = [
            tuple(float(part) for part in match.groups())  # type: ignore[misc]
            for match in re.finditer(
                r"vertex\s+([-+eE0-9.]+)\s+([-+eE0-9.]+)\s+([-+eE0-9.]+)",
                text,
            )
        ]
    if not vertices:
        raise AssertionError(f"No STL vertices found in {stl_path}")
    mins = [min(vertex[index] for vertex in vertices) for index in range(3)]
    maxs = [max(vertex[index] for vertex in vertices) for index in range(3)]
    return mins, maxs


def _assert_vector_almost_equal(
    test_case: unittest.TestCase,
    actual: list[float],
    expected: list[float],
    *,
    places: int = 6,
) -> None:
    test_case.assertEqual(len(actual), len(expected))
    for actual_item, expected_item in zip(actual, expected, strict=True):
        test_case.assertAlmostEqual(actual_item, expected_item, places=places)


class IsaacSimAssetConfigTest(unittest.TestCase):
    def test_official_v2_urdf_contains_required_right_arm_joints(self) -> None:
        root = ET.parse(V2_URDF).getroot()
        joints = {joint.attrib["name"] for joint in root.findall("joint")}

        for joint_name in REQUIRED_ARM_JOINTS:
            self.assertIn(joint_name, joints)

    def test_lerobot_config_uses_v2_right_arm_plus_amazinghand(self) -> None:
        config_text = _read("isaacsim_test/lerobot/rpo_arm_isaacsim.yaml")
        robot_text = _read("isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py")
        contract_text = _read("isaacsim_test/lerobot/rpo_arm_contract.py")

        for joint_name in FEATURE_JOINTS:
            pattern = rf"(^|[\s\"']){re.escape(joint_name)}([\s\"',]|$)"
            self.assertRegex(config_text, pattern)
            self.assertRegex(contract_text, pattern)

        self.assertNotIn("rpo_arm_j1", config_text)
        self.assertNotIn("rpo_arm_j1", robot_text)
        self.assertIn("rpo_arm_contract", robot_text)

    def test_runtime_paths_use_shared_6d_contract_clamping(self) -> None:
        scene_text = _read("isaacsim_test/isaacsim/setup_rpo_arm_scene.py")
        robot_text = _read("isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py")
        phone_text = _read("isaacsim_test/lerobot/phone_teleop_server.py")

        for text in (scene_text, robot_text, phone_text):
            self.assertIn("rpo_arm_contract", text)
            self.assertIn("normalize_action", text)

        self.assertIn("grasp_scalar_to_servo_targets", scene_text)
        self.assertIn("last_grasp_servo_targets", scene_text)
        self.assertIn("AMAZINGHAND_MOTOR_JOINT_NAMES", scene_text)
        self.assertIn("hand_motor_indices", scene_text)
        self.assertIn("hand_motor_control_status", scene_text)
        self.assertIn("ARM_JOINT_LIMITS", phone_text)
        self.assertIn("data-min", phone_text)
        self.assertIn("data-max", phone_text)

    def test_compose_and_env_default_to_direct_physical_urdf_artifact(self) -> None:
        compose_text = _read("isaacsim_test/docker-compose.yml")
        env_text = _read("isaacsim_test/.env.example")
        scene_text = _read("isaacsim_test/isaacsim/setup_rpo_arm_scene.py")
        expected_physical_urdf = (
            "/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/sitl/"
            "roboto_v2_right_arm_amazinghand_full.urdf"
        )

        for text in (compose_text, env_text, scene_text):
            self.assertIn("PHYSICAL_ROBOT_URDF_PATH", text)
            self.assertIn(expected_physical_urdf, text)
            self.assertNotIn("RPO_ARM_URDF_PATH", text)
            self.assertNotIn("DEFAULT_ROBOPARTY_V2_URDF", text)
            self.assertNotIn("roboparty/modules/rpo_hardware", text)
            self.assertNotIn("roboto_origin.urdf", text)
            self.assertNotIn("RoboParty", text)

        for text in (compose_text, env_text):
            self.assertTrue(
                "NUM_JOINTS=6" in text or 'NUM_JOINTS: "${NUM_JOINTS:-6}"' in text
            )
            for joint_name in FEATURE_JOINTS:
                self.assertIn(joint_name, text)

        self.assertNotIn("/workspace/isaacsim/rpo_arm.urdf", compose_text)
        self.assertNotIn("/workspace/isaacsim/rpo_arm.urdf", env_text)


    def test_simready_usd_artifact_is_committed_and_profile_passed(self) -> None:
        simready_usd = ROOT / "isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd"
        simready_profile = ROOT / "isaacsim_test/outputs/simready/echo_full/pipeline/06_validation_final/simready-profile.json"

        self.assertTrue(simready_usd.is_file(), simready_usd)
        self.assertTrue(simready_profile.is_file(), simready_profile)
        self.assertIn('"passed": true', simready_profile.read_text(encoding="utf-8"))

    def test_compose_and_env_expose_simready_usd_path(self) -> None:
        compose_text = _read("isaacsim_test/docker-compose.yml")
        env_text = _read("isaacsim_test/.env.example")
        expected_path = "/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/04_conform/repair-loop-02-fet005/fet005-grasp/echo_full_robot_arm_hand.usd"

        for text in (compose_text, env_text):
            self.assertIn("SIMREADY_USD_PATH", text)
            self.assertIn(expected_path, text)

    def test_scene_defaults_to_custom_visual_usda_plus_physical_arm_hand_urdf(self) -> None:
        scene_text = _read("isaacsim_test/isaacsim/setup_rpo_arm_scene.py")
        compose_text = _read("isaacsim_test/docker-compose.yml")
        env_text = _read("isaacsim_test/.env.example")
        custom_visual_path = "/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/sitl/echo_full_lerobot_articulation.usda"
        physical_urdf_path = "/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/sitl/roboto_v2_right_arm_amazinghand_full.urdf"

        for text in (scene_text, compose_text, env_text):
            self.assertIn("CUSTOM_VISUAL_USD_PATH", text)
            self.assertIn(custom_visual_path, text)
            self.assertIn("LOAD_CUSTOM_VISUAL_USD", text)
            self.assertIn(physical_urdf_path, text)

        self.assertIn("LOAD_CUSTOM_VISUAL_USD=1", env_text)
        self.assertIn('LOAD_CUSTOM_VISUAL_USD: "${LOAD_CUSTOM_VISUAL_USD:-1}"', compose_text)
        self.assertIn(f"CUSTOM_VISUAL_USD_PATH={custom_visual_path}", env_text)
        self.assertIn(f'CUSTOM_VISUAL_USD_PATH: "${{CUSTOM_VISUAL_USD_PATH:-{custom_visual_path}}}"', compose_text)
        self.assertIn("DEFAULT_CUSTOM_VISUAL_USD", scene_text)
        self.assertIn("DEFAULT_CUSTOM_VISUAL_PRIM_PATH", scene_text)
        self.assertIn("Loading custom visual USD", scene_text)
        self.assertIn("_load_custom_visual_usd", scene_text)

    def test_compose_and_env_expose_simready_thumbnail_fallback_path(self) -> None:
        compose_text = _read("isaacsim_test/docker-compose.yml")
        env_text = _read("isaacsim_test/.env.example")
        expected_path = "/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/pipeline/07_render/thumbnail.png"

        for text in (compose_text, env_text):
            self.assertIn("SIMREADY_THUMBNAIL_PATH", text)
            self.assertIn(expected_path, text)

    def test_scene_supports_simready_usd_import_and_mapping_evidence(self) -> None:
        scene_text = _read("isaacsim_test/isaacsim/setup_rpo_arm_scene.py")

        self.assertIn("USE_SIMREADY_USD", scene_text)
        self.assertIn("SIMREADY_USD_PATH", scene_text)
        self.assertIn("Loading SimReady USD", scene_text)
        self.assertIn("echo_full_robot_arm_hand.usd", scene_text)
        self.assertIn("simready_prim_mapping.json", scene_text)
        self.assertIn("simready_root_prim", scene_text)
        self.assertIn("prim_hierarchy", scene_text)
        self.assertIn("bound_or_binding_pending_per_feature", scene_text)
        self.assertIn("binding_pending", scene_text)
        self.assertIn("AddReference", scene_text)
        self.assertIn("set_camera_view", scene_text)
        self.assertIn("_write_simready_mapping_evidence", scene_text)

    def test_direct_import_manifest_contains_lerobot_contract_without_synthetic_joints(self) -> None:
        self.assertTrue(ARTICULATED_SIMREADY_USD.is_file(), ARTICULATED_SIMREADY_USD)

        text = ARTICULATED_SIMREADY_USD.read_text(encoding="utf-8")
        self.assertIn("echo_full_robot_arm_hand.usd", text)
        self.assertIn("roboto_v2_right_arm_amazinghand_full.urdf", text)
        self.assertIn("custom_visual_usda_plus_direct_arm_hand_urdf", text)
        self.assertIn("direct_urdf_import_artifact", text)
        self.assertIn("URDFParseAndImportFile", text)
        self.assertIn("echo_full_lerobot_articulation_metadata", text)
        self.assertIn("r_wrist_interface", text)
        self.assertNotIn("synthetic Roboto V2 URDF physical link bodies", text)
        self.assertNotIn("binding_pending", text)
        for joint_name in REQUIRED_ARM_JOINTS:
            self.assertIn(joint_name, text)

    def test_generated_physical_urdf_and_report_match_v2_urdf_topology(self) -> None:
        self.assertTrue(ARTICULATED_SIMREADY_USD.is_file(), ARTICULATED_SIMREADY_USD)
        self.assertTrue(ARTICULATION_REPORT.is_file(), ARTICULATION_REPORT)
        self.assertTrue(DIRECT_PHYSICAL_URDF.is_file(), DIRECT_PHYSICAL_URDF)

        reference_topology = _reference_urdf_joint_topology()
        physical_topology = _urdf_joint_topology(DIRECT_PHYSICAL_URDF)
        report = json.loads(ARTICULATION_REPORT.read_text(encoding="utf-8"))

        self.assertEqual(report["reference_urdf_path"], str(V2_URDF.relative_to(ROOT)))
        self.assertEqual(report["physical_robot_urdf_path"], str(DIRECT_PHYSICAL_URDF.relative_to(ROOT)))
        self.assertEqual(report["custom_visual_usd_path"], str(ARTICULATED_SIMREADY_USD.relative_to(ROOT)))
        self.assertEqual(report["direct_urdf_import"]["mode"], "direct_urdf_import_artifact")
        self.assertEqual(report["direct_urdf_import"]["isaac_importer"], "URDFParseAndImportFile")
        self.assertEqual(
            report["direct_urdf_import"]["artifact_path"],
            str(DIRECT_PHYSICAL_URDF.relative_to(ROOT)),
        )
        self.assertEqual(
            report["direct_urdf_import"]["wrist_attachment_transform"]["joint_name"],
            "right_elbow_yaw_to_r_wrist_interface",
        )
        self.assertEqual(
            report["direct_urdf_import"]["wrist_attachment_transform"]["parent_link"],
            "right_elbow_yaw_link",
        )
        self.assertEqual(
            report["direct_urdf_import"]["wrist_attachment_transform"]["child_link"],
            "r_wrist_interface",
        )
        _assert_vector_almost_equal(
            self,
            report["direct_urdf_import"]["wrist_attachment_transform"]["origin_xyz"],
            [0.13825, 0.0, 0.0],
        )
        _assert_vector_almost_equal(
            self,
            report["direct_urdf_import"]["wrist_attachment_transform"]["origin_rpy"],
            [0.0, math.pi / 2.0, 0.0],
        )
        self.assertFalse(report["direct_urdf_import"]["synthetic_usd_reconstruction"])
        self.assertEqual(report["direct_urdf_import"]["controlled_joint_count"], 5)
        self.assertEqual(report["direct_urdf_import"]["hand_motor_joint_count"], 8)
        self.assertEqual(
            report["direct_urdf_import"]["hand_motor_joints"],
            [
                "finger1_motor1",
                "finger1_motor2",
                "finger2_motor1",
                "finger2_motor2",
                "finger3_motor1",
                "finger3_motor2",
                "finger4_motor1",
                "finger4_motor2",
            ],
        )
        self.assertEqual(
            report["direct_urdf_import"]["urdf_constraint_fidelity"]["status"],
            "LOSSY_MJCF_CONVERSION",
        )
        self.assertFalse(
            report["direct_urdf_import"]["urdf_constraint_fidelity"]["mjcf_constraints_preserved"]
        )
        self.assertEqual(report["reference_joint_topology"], reference_topology)
        self.assertEqual(report["joint_topology"], physical_topology)
        self.assertEqual(report["physical_urdf_joint_topology"], physical_topology)
        self.assertEqual([item["joint"] for item in reference_topology], REQUIRED_ARM_JOINTS)
        for physical, reference in zip(physical_topology, reference_topology):
            if physical["joint"] == "right_arm_pitch_joint":
                self.assertEqual(physical["parent"], "right_arm_base_link")
            else:
                self.assertEqual(physical["parent"], reference["parent"])
            for key in (
                "child",
                "origin_xyz",
                "origin_rpy",
                "axis_xyz",
                "limit_lower",
                "limit_upper",
                "effort",
                "velocity",
            ):
                self.assertEqual(physical[key], reference[key])

        physical_root = ET.parse(DIRECT_PHYSICAL_URDF).getroot()
        links = {link.attrib["name"] for link in physical_root.findall("link")}
        joints = {joint.attrib["name"]: joint for joint in physical_root.findall("joint")}
        self.assertNotIn("torso_link", links)
        self.assertNotIn("custom_frame_link", links)
        self.assertNotIn("amazinghand_fixed_link", links)
        self.assertIn("right_arm_base_link", links)
        self.assertIn("r_wrist_interface", links)
        self.assertEqual(joints["right_elbow_yaw_to_r_wrist_interface"].attrib["type"], "fixed")
        self.assertEqual(joints["right_elbow_yaw_to_r_wrist_interface"].find("parent").attrib["link"], "right_elbow_yaw_link")  # type: ignore[union-attr]
        self.assertEqual(joints["right_elbow_yaw_to_r_wrist_interface"].find("child").attrib["link"], "r_wrist_interface")  # type: ignore[union-attr]

        visual_status = report["visual_binding_status"]
        self.assertEqual(visual_status["simready_visual_source"], report["source_usd"])
        self.assertEqual(visual_status["custom_visual_usd_path"], str(ARTICULATED_SIMREADY_USD.relative_to(ROOT)))
        self.assertEqual(visual_status["strategy"], "custom_visual_usda_plus_direct_arm_hand_urdf")
        self.assertEqual(visual_status["custom_frame"]["status"], "fixed_base_frame_from_custom_usda")
        self.assertEqual(visual_status["custom_frame"]["physical_urdf_membership"], "excluded")
        self.assertEqual(visual_status["physical_arm"]["root_link"], "right_arm_base_link")
        self.assertEqual(visual_status["physical_arm"]["torso_membership"], "excluded")
        self.assertEqual(visual_status["amazinghand"]["status"], "attached_to_terminal_urdf_link")
        self.assertEqual(visual_status["amazinghand"]["attached_to"], "right_elbow_yaw_link")
        self.assertEqual(visual_status["amazinghand"]["root_link"], "r_wrist_interface")
        _assert_vector_almost_equal(
            self,
            visual_status["amazinghand"]["attachment_origin_xyz"],
            [0.13825, 0.0, 0.0],
        )
        _assert_vector_almost_equal(
            self,
            visual_status["amazinghand"]["attachment_origin_rpy"],
            [0.0, math.pi / 2.0, 0.0],
        )
        self.assertEqual(
            visual_status["amazinghand"]["finger_dofs"],
            "present_in_physical_urdf_and_commanded_from_lerobot_grasp_scalar",
        )
        self.assertGreater(visual_status["amazinghand"]["finger_dof_count"], 0)
        self.assertEqual(visual_status["amazinghand"]["authoritative_reference"], "mjcf")
        self.assertFalse(visual_status["amazinghand"]["mjcf_constraints_preserved_in_urdf"])
        self.assertEqual(visual_status["amazinghand"]["joint_name"], "right_elbow_yaw_to_r_wrist_interface")

    def test_runtime_report_records_live_wrist_attachment_gap(self) -> None:
        scene_text = _read("isaacsim_test/isaacsim/setup_rpo_arm_scene.py")
        report = json.loads(ARTICULATION_REPORT.read_text(encoding="utf-8"))

        self.assertIn("WRIST_ATTACHMENT_PARENT_LINK", scene_text)
        self.assertIn("WRIST_ATTACHMENT_CHILD_LINK", scene_text)
        self.assertIn("WRIST_ATTACHMENT_ORIGIN_XYZ = (0.13825, 0.0, 0.0)", scene_text)
        self.assertIn("def _compute_wrist_attachment_runtime_validation", scene_text)
        self.assertIn("Runtime wrist attachment validation failed", scene_text)

        runtime_validation = report["runtime_validation"]
        wrist_validation = runtime_validation["wrist_attachment_runtime_validation"]
        self.assertEqual(runtime_validation["status"], "PASS")
        self.assertEqual(runtime_validation["hand_motor_control_status"], "PASS")
        self.assertEqual(
            runtime_validation["hand_motor_dofs_commanded"],
            [
                "finger1_motor1",
                "finger1_motor2",
                "finger2_motor1",
                "finger2_motor2",
                "finger3_motor1",
                "finger3_motor2",
                "finger4_motor1",
                "finger4_motor2",
            ],
        )
        self.assertEqual(runtime_validation["missing_hand_motor_dofs"], [])
        for motor_joint in runtime_validation["hand_motor_dofs_commanded"]:
            self.assertIn(motor_joint, runtime_validation["controlled_dofs_moved"])
        self.assertEqual(
            runtime_validation["urdf_constraint_fidelity"]["status"],
            "LOSSY_MJCF_CONVERSION",
        )
        self.assertEqual(wrist_validation["status"], "PASS")
        self.assertEqual(wrist_validation["parent_link"], "right_elbow_yaw_link")
        self.assertEqual(wrist_validation["child_link"], "r_wrist_interface")
        _assert_vector_almost_equal(
            self,
            wrist_validation["expected_parent_local_origin_xyz"],
            [0.13825, 0.0, 0.0],
        )
        self.assertLessEqual(
            wrist_validation["gap_m"],
            wrist_validation["max_allowed_gap_m"],
        )

    def test_physical_urdf_attaches_amazinghand_and_defers_finger_control_contract(self) -> None:
        self.assertTrue(DIRECT_PHYSICAL_URDF.is_file(), DIRECT_PHYSICAL_URDF)

        root = ET.parse(DIRECT_PHYSICAL_URDF).getroot()
        joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
        hand_joint = joints["right_elbow_yaw_to_r_wrist_interface"]
        self.assertEqual(hand_joint.attrib["type"], "fixed")
        self.assertEqual(hand_joint.find("parent").attrib["link"], "right_elbow_yaw_link")  # type: ignore[union-attr]
        self.assertEqual(hand_joint.find("child").attrib["link"], "r_wrist_interface")  # type: ignore[union-attr]
        for motor_joint in (
            "finger1_motor1",
            "finger1_motor2",
            "finger2_motor1",
            "finger2_motor2",
            "finger3_motor1",
            "finger3_motor2",
            "finger4_motor1",
            "finger4_motor2",
        ):
            self.assertIn(motor_joint, joints)
            self.assertEqual(joints[motor_joint].attrib["type"], "revolute")

    def test_amazinghand_mjcf_is_recorded_as_authoritative_reference(self) -> None:
        report = json.loads(ARTICULATION_REPORT.read_text(encoding="utf-8"))
        mjcf = report["amazinghand_mjcf_source"]

        self.assertEqual(
            mjcf["path"],
            "AmazingHand/Demo/AHSimulation/AHSimulation/AH_Right/mjcf/robot.xml",
        )
        self.assertEqual(mjcf["root_body"], "r_wrist_interface")
        self.assertEqual(mjcf["position_actuator_count"], 8)
        self.assertEqual(mjcf["equality_connect_count"], 20)
        self.assertEqual(mjcf["missing_meshes"], [])
        self.assertEqual(report["urdf_constraint_fidelity"]["status"], "LOSSY_MJCF_CONVERSION")
        self.assertFalse(report["urdf_constraint_fidelity"]["mjcf_constraints_preserved"])

    def test_amazinghand_wrist_fixed_joint_uses_arm_endpoint_transform(self) -> None:
        self.assertTrue(DIRECT_PHYSICAL_URDF.is_file(), DIRECT_PHYSICAL_URDF)
        self.assertTrue(RIGHT_ELBOW_YAW_LINK_MESH.is_file(), RIGHT_ELBOW_YAW_LINK_MESH)

        root = ET.parse(DIRECT_PHYSICAL_URDF).getroot()
        joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
        hand_joint = joints["right_elbow_yaw_to_r_wrist_interface"]
        origin = hand_joint.find("origin")
        self.assertIsNotNone(origin)

        _, elbow_yaw_bbox_max = _stl_bbox(RIGHT_ELBOW_YAW_LINK_MESH)
        expected_xyz = [elbow_yaw_bbox_max[0], 0.0, 0.0]
        expected_rpy = [0.0, math.pi / 2.0, 0.0]
        actual_xyz = _float_list(origin.attrib.get("xyz", "0 0 0"))  # type: ignore[union-attr]
        actual_rpy = _float_list(origin.attrib.get("rpy", "0 0 0"))  # type: ignore[union-attr]

        self.assertEqual(hand_joint.attrib["type"], "fixed")
        self.assertEqual(hand_joint.find("parent").attrib["link"], "right_elbow_yaw_link")  # type: ignore[union-attr]
        self.assertEqual(hand_joint.find("child").attrib["link"], "r_wrist_interface")  # type: ignore[union-attr]
        _assert_vector_almost_equal(self, actual_xyz, expected_xyz)
        _assert_vector_almost_equal(self, actual_rpy, expected_rpy)

    def test_full_arm_hand_urdf_excludes_torso_and_attaches_j5_to_hand_root(self) -> None:
        self.assertTrue(FULL_ARM_HAND_URDF.is_file(), FULL_ARM_HAND_URDF)

        root = ET.parse(FULL_ARM_HAND_URDF).getroot()
        links = {link.attrib["name"] for link in root.findall("link")}
        joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}

        self.assertNotIn("torso_link", links)
        self.assertIn("right_arm_base_link", links)
        self.assertIn("r_wrist_interface", links)
        self.assertEqual(
            joints["right_arm_pitch_joint"].find("parent").attrib["link"],  # type: ignore[union-attr]
            "right_arm_base_link",
        )

        reference_topology = _reference_urdf_joint_topology()
        physical_topology = _urdf_joint_topology(FULL_ARM_HAND_URDF)
        for physical, reference in zip(physical_topology, reference_topology):
            if physical["joint"] == "right_arm_pitch_joint":
                self.assertEqual(physical["parent"], "right_arm_base_link")
            else:
                self.assertEqual(physical["parent"], reference["parent"])
            for key in (
                "child",
                "origin_xyz",
                "origin_rpy",
                "axis_xyz",
                "limit_lower",
                "limit_upper",
                "effort",
                "velocity",
            ):
                self.assertEqual(physical[key], reference[key])

        attach_joint = joints["right_elbow_yaw_to_r_wrist_interface"]
        self.assertEqual(attach_joint.attrib["type"], "fixed")
        self.assertEqual(attach_joint.find("parent").attrib["link"], "right_elbow_yaw_link")  # type: ignore[union-attr]
        self.assertEqual(attach_joint.find("child").attrib["link"], "r_wrist_interface")  # type: ignore[union-attr]

        for motor_joint in (
            "finger1_motor1",
            "finger1_motor2",
            "finger2_motor1",
            "finger2_motor2",
            "finger3_motor1",
            "finger3_motor2",
            "finger4_motor1",
            "finger4_motor2",
        ):
            self.assertIn(motor_joint, joints)
            self.assertEqual(joints[motor_joint].attrib["type"], "revolute")

        for ball_index in range(1, 13):
            for axis in ("x", "y", "z"):
                self.assertIn(f"passive_ball{ball_index}_{axis}", joints)

    def test_articulation_manifest_authors_direct_physical_urdf_and_provenance(self) -> None:
        manifest_text = ARTICULATED_SIMREADY_USD.read_text(encoding="utf-8")
        report_text = ARTICULATION_REPORT.read_text(encoding="utf-8")

        for text in (manifest_text, report_text):
            self.assertIn("custom_visual_usda_plus_direct_arm_hand_urdf", text)
            self.assertIn("direct_urdf_import_artifact", text)
            self.assertIn("URDFParseAndImportFile", text)
            self.assertIn("right_elbow_yaw_to_r_wrist_interface", text)
            self.assertNotIn("synthetic Roboto V2 URDF physical link bodies", text)

        self.assertIn('uniform token purpose = "default"', manifest_text)
        self.assertIn("custom string visual_context_role", manifest_text)
        self.assertNotIn('uniform token purpose = "Visual context only', manifest_text)

    def test_scene_defaults_to_direct_physical_urdf_and_keeps_simready_opt_in(self) -> None:
        scene_text = _read("isaacsim_test/isaacsim/setup_rpo_arm_scene.py")
        compose_text = _read("isaacsim_test/docker-compose.yml")
        env_text = _read("isaacsim_test/.env.example")

        expected_simready_path = "/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/sitl/echo_full_lerobot_articulation.usda"
        expected_physical_urdf = "/workspace/superarm_ws/isaacsim_test/outputs/simready/echo_full/sitl/roboto_v2_right_arm_amazinghand_full.urdf"
        for text in (scene_text, compose_text, env_text):
            self.assertIn("SIMREADY_ARTICULATION_USD_PATH", text)
            self.assertIn(expected_simready_path, text)
            self.assertIn("PHYSICAL_ROBOT_URDF_PATH", text)
            self.assertIn(expected_physical_urdf, text)

        self.assertIn("USE_SIMREADY_USD=0", env_text)
        self.assertIn("USE_SIMREADY_ARTICULATION_USD=0", env_text)
        self.assertIn("LOAD_CUSTOM_VISUAL_USD=1", env_text)
        self.assertIn('USE_SIMREADY_USD: "${USE_SIMREADY_USD:-0}"', compose_text)
        self.assertIn('LOAD_CUSTOM_VISUAL_USD: "${LOAD_CUSTOM_VISUAL_USD:-1}"', compose_text)
        self.assertIn(
            'USE_SIMREADY_ARTICULATION_USD: "${USE_SIMREADY_ARTICULATION_USD:-0}"',
            compose_text,
        )
        self.assertIn(f"PHYSICAL_ROBOT_URDF_PATH={expected_physical_urdf}", env_text)
        self.assertIn(f'PHYSICAL_ROBOT_URDF_PATH: "${{PHYSICAL_ROBOT_URDF_PATH:-{expected_physical_urdf}}}"', compose_text)
        self.assertIn("DEFAULT_PHYSICAL_ROBOT_URDF", scene_text)
        self.assertIn('USE_SIMREADY_USD = _env_flag("USE_SIMREADY_USD", default=False)', scene_text)
        self.assertIn(
            'USE_SIMREADY_ARTICULATION_USD = _env_flag("USE_SIMREADY_ARTICULATION_USD", default=False)',
            scene_text,
        )
        self.assertIn("if USE_SIMREADY_USD and USE_SIMREADY_ARTICULATION_USD", scene_text)
        self.assertIn("Loading articulated SimReady USD", scene_text)
        self.assertIn("SimReady articulation binding is bound", scene_text)
        self.assertIn("No physical robot URDF found", scene_text)
        self.assertIn("Loading physical robot URDF", scene_text)
        self.assertIn("Loading custom visual USD", scene_text)
        self.assertIn("Imported articulation prim", scene_text)
        self.assertIn("Loaded {num_dof} total URDF joints", scene_text)
        self.assertIn("controlled_indices = [all_dof_names.index(name)", scene_text)

    def test_scene_supports_screenshot_after_lerobot_command(self) -> None:
        scene_text = _read("isaacsim_test/isaacsim/setup_rpo_arm_scene.py")
        compose_text = _read("isaacsim_test/docker-compose.yml")

        for env_name in ("SCREENSHOT_AFTER_COMMAND", "SCREENSHOT_ON_STARTUP", "SCREENSHOT_PATH", "EXIT_AFTER_SCREENSHOT"):
            self.assertIn(env_name, scene_text)
            self.assertIn(env_name, compose_text)
        self.assertIn("echo_full_simready_target.png", scene_text)
        self.assertIn("echo_full_simready_target.png", compose_text)
        self.assertNotIn("rpo_v2_lerobot_target.png", compose_text)
        isaac_service = compose_text.split("  # LeRobot", maxsplit=1)[0]
        self.assertIn('user: "0:0"', isaac_service)
        self.assertIn(
            "${SUPERARM_WS_PATH:?Set SUPERARM_WS_PATH in isaacsim_test/.env}:/workspace/superarm_ws:rw",
            isaac_service,
        )
        self.assertIn("SIMREADY_THUMBNAIL_PATH", scene_text)
        self.assertIn("thumbnail.png", scene_text)
        self.assertIn("_write_fallback_visual_evidence", scene_text)
        self.assertIn("Fallback visual evidence saved", scene_text)
        self.assertIn("_capture_replicator_screenshot", scene_text)
        self.assertIn('rep.WriterRegistry.get("BasicWriter")', scene_text)
        self.assertIn("rep.orchestrator.step", scene_text)
        self.assertIn('enable_extension("isaacsim.test.utils")', scene_text)
        self.assertIn('enable_extension("omni.kit.renderer.capture")', scene_text)
        self.assertIn("capture_next_frame_rp_resource", scene_text)
        self.assertIn("capture_viewport_to_file", scene_text)
        self.assertIn("last_applied_command", scene_text)
        self.assertIn("threading.Thread(target=rclpy.spin", scene_text)
        self.assertIn("last_processed_command_seq", scene_text)
        self.assertIn("_publish_current_state", scene_text)
        self.assertIn("Startup screenshot trigger accepted", scene_text)

    def test_scene_supports_batch_motion_screenshot_cases_while_physics_runs(self) -> None:
        scene_text = _read("isaacsim_test/isaacsim/setup_rpo_arm_scene.py")
        compose_text = _read("isaacsim_test/docker-compose.yml")
        env_text = _read("isaacsim_test/.env.example")
        runner_path = ROOT / "isaacsim_test/run_simready_motion_screenshot_cases.sh"
        cases_path = ROOT / "isaacsim_test/simready_motion_cases.json"

        for env_name in (
            "MOTION_SCREENSHOT_CASES_PATH",
            "MOTION_SCREENSHOT_CASES_JSON",
            "MOTION_SCREENSHOT_OUTPUT_DIR",
            "MOTION_SCREENSHOT_SETTLE_STEPS",
            "EXIT_AFTER_MOTION_SCREENSHOTS",
        ):
            self.assertIn(env_name, scene_text)
            self.assertIn(env_name, compose_text)
            self.assertIn(env_name, env_text)

        self.assertIn("def _parse_motion_screenshot_cases", scene_text)
        self.assertIn("def _run_motion_screenshot_cases", scene_text)
        self.assertIn("Motion screenshot case", scene_text)
        self.assertIn("world.step(render=True)", scene_text)
        self.assertLess(scene_text.index("timeline.play()"), scene_text.index("motion_screenshot_cases_ran = _run_motion_screenshot_cases()"))
        self.assertIn("simready_motion_cases", scene_text)

        self.assertTrue(runner_path.is_file(), runner_path)
        runner_text = runner_path.read_text(encoding="utf-8")
        self.assertIn("MOTION_SCREENSHOT_CASES_PATH", runner_text)
        self.assertIn("simready_motion_cases.json", runner_text)
        self.assertIn("USE_SIMREADY_USD=0", runner_text)
        self.assertIn("PHYSICAL_ROBOT_URDF_PATH", runner_text)
        self.assertIn("roboto_v2_right_arm_amazinghand_full.urdf", runner_text)
        self.assertIn("CUSTOM_VISUAL_USD_PATH", runner_text)
        self.assertIn("LOAD_CUSTOM_VISUAL_USD=${LOAD_CUSTOM_VISUAL_USD:-1}", runner_text)
        self.assertIn("SCREENSHOT_ON_STARTUP=0", runner_text)
        self.assertIn("except PermissionError as exc", runner_text)
        self.assertIn("could not update runtime report contact sheet", runner_text)

        self.assertTrue(cases_path.is_file(), cases_path)
        cases_text = cases_path.read_text(encoding="utf-8")
        for case_name in ("home", "reach_forward", "elbow_fold", "side_sweep"):
            self.assertIn(case_name, cases_text)
        for joint_name in FEATURE_JOINTS:
            self.assertIn(joint_name, cases_text)

    def test_motion_screenshot_runner_recovers_articulation_after_capture(self) -> None:
        scene_text = _read("isaacsim_test/isaacsim/setup_rpo_arm_scene.py")

        self.assertIn("def _ensure_articulation_initialized", scene_text)
        self.assertIn("is_physics_handle_valid", scene_text)
        self.assertIn("_physics_view", scene_text)
        self.assertIn("art.initialize()", scene_text)
        self.assertIn("Motion screenshot cases failed", scene_text)
        self.assertIn("sys.exit(1)", scene_text)

    def test_motion_screenshot_cases_can_run_in_kinematic_capture_mode(self) -> None:
        scene_text = _read("isaacsim_test/isaacsim/setup_rpo_arm_scene.py")

        self.assertIn("MOTION_SCREENSHOT_KINEMATIC_CAPTURE", scene_text)
        self.assertIn("set_joint_position_targets", scene_text)
        self.assertIn("Kinematic capture mode", scene_text)
        self.assertIn("settle_steps=0", scene_text)

    def test_custom_visual_can_follow_arm_link_for_motion_screenshots(self) -> None:
        scene_text = _read("isaacsim_test/isaacsim/setup_rpo_arm_scene.py")
        compose_text = _read("isaacsim_test/docker-compose.yml")
        runner_text = _read("isaacsim_test/run_simready_motion_screenshot_cases.sh")

        for text in (scene_text, compose_text, runner_text):
            self.assertIn("CUSTOM_VISUAL_FOLLOW_LINK", text)
            self.assertIn("CUSTOM_VISUAL_FOLLOW_XYZ", text)

        self.assertIn("_sync_custom_visual_to_follow_link", scene_text)
        self.assertIn("_capture_prim_paths", scene_text)
        self.assertIn("ComputeLocalToWorldTransform", scene_text)
        self.assertIn('prim_path.rsplit("/", maxsplit=1)', scene_text)

    def test_replicator_screenshot_frames_simready_asset_not_world_root(self) -> None:
        scene_text = _read("isaacsim_test/isaacsim/setup_rpo_arm_scene.py")

        self.assertIn("capture_prim_paths = _capture_prim_paths()", scene_text)
        self.assertIn('if capture_prim_path.startswith("/World/"):', scene_text)
        self.assertIn("root_path = capture_prim_path", scene_text)
        self.assertIn('root_path = "/" + capture_prim_path.strip("/").split("/", maxsplit=1)[0]', scene_text)

    def test_custom_visual_camera_fallback_is_tight_enough_for_wrist_check(self) -> None:
        scene_text = _read("isaacsim_test/isaacsim/setup_rpo_arm_scene.py")

        self.assertIn("DEFAULT_CUSTOM_VISUAL_CAPTURE_RADIUS = 0.28", scene_text)
        self.assertNotIn("center = Gf.Vec3d(*DEFAULT_CUSTOM_VISUAL_CAPTURE_CENTER)", scene_text)
        self.assertNotIn("if use_custom_visual_camera_fallback:", scene_text)
        self.assertIn("focal_length = 35", scene_text)

    def test_headed_sunshine_runtime_keeps_kit_ui_event_loop_updating(self) -> None:
        scene_text = _read("isaacsim_test/isaacsim/setup_rpo_arm_scene.py")

        self.assertIn("CONTINUOUS_APP_UPDATE", scene_text)
        self.assertIn("not args.headless", scene_text)
        self.assertIn("if CONTINUOUS_APP_UPDATE or not app_updated", scene_text)
        self.assertIn("simulation_app.update()", scene_text)

    def test_lerobot_sitl_verifier_uses_robot_config_and_checks_tolerance(self) -> None:
        verifier_text = _read("isaacsim_test/lerobot/verify_lerobot_sitl.py")
        contract_text = _read("isaacsim_test/lerobot/rpo_arm_contract.py")

        self.assertIn("rpo_arm_isaacsim.yaml", verifier_text)
        self.assertIn("IsaacSimRpoArmRobot", verifier_text)
        self.assertIn("send_action", verifier_text)
        self.assertIn("capture_observation", verifier_text)
        self.assertIn("normalize_action", verifier_text)
        self.assertIn("0.03", verifier_text)
        for joint_name in FEATURE_JOINTS:
            self.assertIn(joint_name, contract_text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
