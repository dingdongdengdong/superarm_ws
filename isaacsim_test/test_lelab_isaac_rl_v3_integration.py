from __future__ import annotations

import hashlib
import json
import subprocess
import zipfile
from pathlib import Path

import pytest

from isaacsim_test.lelab_rl_v3_integration import (
    LOCK_PATH,
    load_lock,
    validate_distribution,
    validate_lelab_checkout,
    validate_superarm_lerobot,
)


def _write_distribution(path: Path, lock: dict) -> str:
    root = path.stem
    manifest = {
        "schema": lock["distribution"]["manifest_schema"],
        "entrypoint": lock["distribution"]["entrypoint"],
        "visual_contract": {
            "profile": lock["distribution"]["visual_profile"],
            "passive_follower_count": 88,
            "outer_shells_included": False,
        },
        "robot_contract": {"logical_action_width": 6, "physical_dof_count": 13},
        "grasp_contract": {
            "real_hardware_max_code": 0.5,
            "full_close_simulation_only": True,
        },
    }
    with zipfile.ZipFile(path, "w") as archive:
        archive.writestr(f"{root}/manifest.json", json.dumps(manifest))
        archive.writestr(f"{root}/{lock['distribution']['entrypoint']}", "#usda 1.0")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_committed_lock_pins_verified_v3_and_half_close_boundary() -> None:
    lock = load_lock(LOCK_PATH)

    assert lock["lelab"]["commit"] == "5bcae6b6fa85497ea34346829780a0c879397c53"
    assert lock["distribution"]["sha256"] == (
        "c356d1157318b72532b82d73270ef06b5b11ed5b8a90641ea4e431941e4554f7"
    )
    assert lock["distribution"]["passive_follower_count"] == 88
    assert lock["distribution"]["outer_shells_included"] is False
    assert lock["contract"]["real_hardware_grasp_max"] == 0.5
    assert lock["contract"]["simulation_grasp_max"] == 1.0
    assert lock["superarm_lerobot"] == {
        "config_path": "isaacsim_test/lerobot/source_arm_amazinghand.yaml",
        "robot_module_path": "isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py",
        "action_adapter_path": "isaacsim_test/lerobot/superarm_action_adapter.py",
        "backend_type": "isaacsim_rpo_arm",
    }


def test_launcher_defaults_to_visible_lelab_submodule() -> None:
    launcher = LOCK_PATH.with_name("run_lelab_isaac_rl_v3.sh").read_text(
        encoding="utf-8"
    )

    assert 'lelab_repo=${LELAB_REPO:-"$repo_root/leLab"}' in launcher
    assert '--lelab-repo "$lelab_repo"' in launcher
    assert 'SUPERARM_ASSET_ROOT=${SUPERARM_ASSET_ROOT:-"$repo_root"}' in launcher
    assert (
        'SUPERARM_LEROBOT_CONFIG=${SUPERARM_LEROBOT_CONFIG:-"$repo_root/'
        'isaacsim_test/lerobot/source_arm_amazinghand.yaml"}'
    ) in launcher
    assert "export SUPERARM_ASSET_ROOT SUPERARM_LEROBOT_CONFIG" in launcher
    assert 'if [[ -n "${LELAB_PYTHON:-}" ]]' in launcher
    assert 'exec "$LELAB_PYTHON" -m lelab.scripts.lelab --no-open "$@"' in launcher


def test_superarm_lerobot_validation_pins_edited_six_control_config() -> None:
    repo_root = LOCK_PATH.parents[1]

    report = validate_superarm_lerobot(repo_root, load_lock())

    assert report["config"].endswith(
        "isaacsim_test/lerobot/source_arm_amazinghand.yaml"
    )
    assert report["backend_type"] == "isaacsim_rpo_arm"
    assert report["logical_joint_names"] == [
        "joint_rev_1",
        "joint_rev_2",
        "joint_rev_3",
        "joint_rev_4",
        "joint_rev_5",
        "amazinghand_motion",
    ]
    assert report["physical_joint_count"] == 13
    assert report["motion_codes"] == [0.0, 0.5, 1.0]


def test_distribution_validation_accepts_only_matching_archive_and_manifest(
    tmp_path: Path,
) -> None:
    lock = load_lock()
    archive = tmp_path / lock["distribution"]["filename"]
    lock["distribution"]["sha256"] = _write_distribution(archive, lock)

    report = validate_distribution(archive, lock)

    assert report["checks"] == {
        "schema": True,
        "entrypoint": True,
        "visual_profile": True,
        "passive_follower_count": True,
        "outer_shells_excluded": True,
        "logical_action_width": True,
        "physical_dof_count": True,
        "real_hardware_half_close": True,
        "full_close_simulation_only": True,
    }

    archive.write_bytes(b"wrong")
    with pytest.raises(ValueError, match="checksum mismatch"):
        validate_distribution(archive, lock)


def test_lelab_checkout_must_match_exact_pinned_clean_commit(tmp_path: Path) -> None:
    repo = tmp_path / "lelab"
    repo.mkdir()
    subprocess.run(["git", "init", "-q", str(repo)], check=True)
    subprocess.run(["git", "-C", str(repo), "config", "user.name", "Test"], check=True)
    subprocess.run(
        ["git", "-C", str(repo), "config", "user.email", "test@example.com"], check=True
    )
    required = [
        "frontend/dist/index.html",
        "isaacsim_validation/control_bridge.py",
        "lelab/rl/config.py",
    ]
    for relative in required:
        target = repo / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(relative, encoding="utf-8")
    subprocess.run(["git", "-C", str(repo), "add", "."], check=True)
    subprocess.run(["git", "-C", str(repo), "commit", "-qm", "fixture"], check=True)
    head = subprocess.run(
        ["git", "-C", str(repo), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    lock = {
        "lelab": {
            "commit": head,
            "required_paths": required,
        }
    }

    assert validate_lelab_checkout(repo, lock)["commit"] == head

    (repo / required[0]).write_text("dirty", encoding="utf-8")
    with pytest.raises(ValueError, match="not clean"):
        validate_lelab_checkout(repo, lock)
