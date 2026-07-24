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

    assert lock["lelab"]["commit"] == "a336c943dd821fe2e554ff6e864fde9f72470a0a"
    assert lock["distribution"]["sha256"] == (
        "c356d1157318b72532b82d73270ef06b5b11ed5b8a90641ea4e431941e4554f7"
    )
    assert lock["distribution"]["passive_follower_count"] == 88
    assert lock["distribution"]["outer_shells_included"] is False
    assert lock["contract"]["real_hardware_grasp_max"] == 0.5
    assert lock["contract"]["simulation_grasp_max"] == 1.0


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
