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
        data.ctrl[5:] = [0.50, 0.56] * 4
        for _ in range(1000):
            mujoco.mj_step(self.model, data)
        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertTrue(np.isfinite(data.qvel).all())
        mujoco.mj_resetData(self.model, data)
        mujoco.mj_forward(self.model, data)
        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertLess(float(np.linalg.norm(data.qvel)), 1e-12)


if __name__ == "__main__":
    unittest.main()
