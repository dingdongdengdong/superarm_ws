"""Model-contract tests for the combined source arm + AmazingHand MJCF."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from isaacsim_test.mujoco_models.generate_superarm_amazinghand import (
    ACTUATOR_ORDER,
    ARM_JOINT_NAMES,
    HAND_ACTUATOR_NAMES,
    generate_combined_model,
)

ROOT = Path(__file__).resolve().parents[1]


class CombinedMuJoCoModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        try:
            import mujoco
        except ImportError as exc:
            raise unittest.SkipTest("mujoco is not installed") from exc
        cls.mujoco = mujoco
        cls.temp = tempfile.TemporaryDirectory()
        cls.report = generate_combined_model(
            workspace_root=ROOT,
            output_dir=Path(cls.temp.name) / "model",
        )
        cls.model = mujoco.MjModel.from_xml_path(cls.report["model_path"])

    @classmethod
    def tearDownClass(cls) -> None:
        if hasattr(cls, "temp"):
            cls.temp.cleanup()

    def test_model_contract(self) -> None:
        mujoco = self.mujoco
        names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
            for index in range(self.model.nu)
        ]
        self.assertEqual(self.model.nu, 13)
        self.assertEqual(names, ACTUATOR_ORDER)
        self.assertEqual(names[:5], ARM_JOINT_NAMES)
        self.assertEqual(names[5:], HAND_ACTUATOR_NAMES)
        self.assertEqual(self.model.neq, 20)
        self.assertEqual(self.model.nmesh, 41)

    def test_portable_assets_and_manifest(self) -> None:
        model_path = Path(self.report["model_path"])
        text = model_path.read_text(encoding="utf-8")
        self.assertNotIn(str(ROOT), text)
        self.assertNotIn("/home/", text)
        self.assertTrue(Path(self.report["manifest_path"]).is_file())
        self.assertEqual(self.report["actuator_count"], 13)
        self.assertEqual(self.report["equality_count"], 20)
        self.assertEqual(self.report["mesh_count"], 41)
        for asset in model_path.parent.glob("assets/**/*.stl"):
            self.assertGreater(asset.stat().st_size, 0)
        self.assertEqual(len(list(model_path.parent.glob("assets/**/*.stl"))), 41)

    def test_finite_step_and_stable_reset(self) -> None:
        mujoco = self.mujoco
        data = mujoco.MjData(self.model)
        data.ctrl[:5] = [0.2, -0.2, 0.3, -0.3, 0.1]
        data.ctrl[5:] = [0.50, -0.56] * 4
        for _ in range(1000):
            mujoco.mj_step(self.model, data)
        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertTrue(np.isfinite(data.qvel).all())
        mujoco.mj_resetData(self.model, data)
        mujoco.mj_forward(self.model, data)
        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertLess(float(np.linalg.norm(data.qvel)), 1e-12)

    def test_arm_joint_anchors_stay_inside_motor_mesh_bounds(self) -> None:
        """Keep every revolute axis physically located inside its motor body."""
        mujoco = self.mujoco
        data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, data)
        mesh_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_MESH, index)
            for index in range(self.model.nmesh)
        ]
        for index, joint_name in enumerate(ARM_JOINT_NAMES, start=1):
            joint_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_name,
            )
            motor_geoms = [
                geom_id
                for geom_id in range(self.model.ngeom)
                if self.model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH
                and mesh_names[int(self.model.geom_dataid[geom_id])] == f"motor_{index}"
            ]
            self.assertEqual(len(motor_geoms), 1, joint_name)
            geom_id = motor_geoms[0]
            center_distance = float(
                np.linalg.norm(data.geom_xpos[geom_id] - data.xanchor[joint_id])
            )
            self.assertLessEqual(
                center_distance,
                float(self.model.geom_rbound[geom_id]),
                f"{joint_name} axis is outside motor_{index}",
            )

    def test_each_finger_visibly_flexes_toward_the_wrist(self) -> None:
        """Guard against commanding both hinges positive (mostly side motion)."""
        mujoco = self.mujoco
        wrist = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "r_wrist_interface",
        )
        for finger in range(1, 5):
            data = mujoco.MjData(self.model)
            for index in range(1, 5):
                motor1 = mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_ACTUATOR,
                    f"finger{index}_motor1",
                )
                motor2 = mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_ACTUATOR,
                    f"finger{index}_motor2",
                )
                data.ctrl[motor1] = 0.05
                data.ctrl[motor2] = -0.02
            for _ in range(1200):
                mujoco.mj_step(self.model, data)
            tip = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_SITE,
                f"tip{finger}",
            )
            open_position = data.site_xpos[tip].copy()
            open_radius = float(np.linalg.norm(open_position - data.xpos[wrist]))
            data.ctrl[
                mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_ACTUATOR,
                    f"finger{finger}_motor1",
                )
            ] = 0.95
            data.ctrl[
                mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_ACTUATOR,
                    f"finger{finger}_motor2",
                )
            ] = -1.10
            for _ in range(1500):
                mujoco.mj_step(self.model, data)
            closed_position = data.site_xpos[tip].copy()
            closed_radius = float(np.linalg.norm(closed_position - data.xpos[wrist]))
            self.assertGreater(float(np.linalg.norm(closed_position - open_position)), 0.05)
            self.assertGreater(open_radius - closed_radius, 0.02)


if __name__ == "__main__":
    unittest.main()
