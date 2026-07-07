#!/usr/bin/env python3
"""Headless Isaac Sim validation for the controllable SimReady arm overlay."""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

from isaacsim import SimulationApp


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--usd", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--headless", action="store_true", default=True)
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    report_path = args.out_dir / "live_validation_report.json"
    log_payload: dict = {
        "status": "started",
        "usd": str(args.usd),
        "out_dir": str(args.out_dir),
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }

    simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})
    try:
        import numpy as np
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import Articulation
        from pxr import Usd, UsdGeom

        stage = omni.usd.get_context().get_stage()
        root_path = "/World/echo_full_simready"
        prim = stage.DefinePrim(root_path, "Xform")
        prim.GetReferences().AddReference(str(args.usd.resolve()))
        articulation_path = root_path + "/ControlRig"
        for _ in range(30):
            simulation_app.update()

        world = World(stage_units_in_meters=1.0)
        world.scene.add_default_ground_plane()
        art = Articulation(articulation_path)
        world.reset()
        for _ in range(10):
            simulation_app.update()
        art.initialize()

        dof_names = list(art.dof_names)
        expected = [
            "right_arm_pitch_joint",
            "right_arm_roll_joint",
            "right_arm_yaw_joint",
            "right_elbow_pitch_joint",
            "right_elbow_yaw_joint",
        ]
        missing = [name for name in expected if name not in dof_names]
        log_payload.update(
            {
                "articulation_path": articulation_path,
                "dof_names": dof_names,
                "num_dof": int(art.num_dof),
                "missing_expected_joints": missing,
            }
        )
        if missing:
            log_payload["status"] = "failed_missing_joints"
            report_path.write_text(json.dumps(log_payload, indent=2, sort_keys=True) + "\n")
            return 2

        all_positions = np.asarray(art.get_joint_positions(), dtype=np.float32).reshape(-1)
        initial_positions = all_positions.astype(float).tolist()
        target_by_name = {
            "right_arm_pitch_joint": 0.25,
            "right_arm_roll_joint": -0.20,
            "right_arm_yaw_joint": 0.18,
            "right_elbow_pitch_joint": 0.30,
            "right_elbow_yaw_joint": -0.15,
        }
        for name, value in target_by_name.items():
            all_positions[dof_names.index(name)] = value
        art.set_joint_positions(all_positions)
        for _ in range(30):
            world.step(render=True)
        readback = np.asarray(art.get_joint_positions(), dtype=np.float32).reshape(-1).astype(float).tolist()
        readback_by_name = {name: float(readback[dof_names.index(name)]) for name in expected}
        max_abs_error = max(abs(readback_by_name[name] - target_by_name[name]) for name in expected)
        # After stepping live physics, small settling/drift is expected. 5e-3 rad is
        # ~0.29 degrees and is tight enough to prove the commanded DOFs are bound.
        readback_tolerance_rad = 5e-3

        # USD stage sanity: AmazingHand fixed visual exists, but no grasp DOF was created.
        stage_for_inspect = Usd.Stage.Open(str(args.usd))
        has_grasp_joint = bool(
            stage_for_inspect.GetPrimAtPath("/echo_full_controllable/ControlRig/Joints/amazinghand_grasp")
        )
        hand_prim = stage_for_inspect.GetPrimAtPath(
            "/echo_full_controllable/VisualFixed/tn____xaZ2Ve2pr5yw2tw0WSflDbf1/"
            "tn__v341_a4a4XfWAou7zcLveO/tn__AmazingHand_righthandv201_lQjM"
        )
        hand_visibility = None
        if hand_prim and hand_prim.IsValid():
            attr = hand_prim.GetAttribute("visibility")
            hand_visibility = attr.Get() if attr else None

        log_payload.update(
            {
                "initial_positions": initial_positions,
                "target_by_name": target_by_name,
                "readback_by_name": readback_by_name,
                "max_abs_error": float(max_abs_error),
                "readback_tolerance_rad": readback_tolerance_rad,
                "has_amazinghand_grasp_joint": has_grasp_joint,
                "amazinghand_visual_prim_valid": bool(hand_prim and hand_prim.IsValid()),
                "amazinghand_visual_visibility": hand_visibility,
                "status": "passed"
                if (
                    max_abs_error <= readback_tolerance_rad
                    and not has_grasp_joint
                    and bool(hand_prim and hand_prim.IsValid())
                )
                else "failed_readback_or_hand_contract",
                "finished_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            }
        )
        report_path.write_text(json.dumps(log_payload, indent=2, sort_keys=True) + "\n")
        print(json.dumps(log_payload, indent=2, sort_keys=True), flush=True)
        return 0 if log_payload["status"] == "passed" else 3
    except Exception as exc:
        log_payload.update({"status": "exception", "exception": repr(exc)})
        report_path.write_text(json.dumps(log_payload, indent=2, sort_keys=True) + "\n")
        print(json.dumps(log_payload, indent=2, sort_keys=True), flush=True)
        return 1
    finally:
        simulation_app.close()


if __name__ == "__main__":
    raise SystemExit(main())
