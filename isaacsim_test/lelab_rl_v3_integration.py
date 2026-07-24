"""Strict SuperArm-side validation for the pinned LeLab Isaac RL runtime."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import zipfile
from pathlib import Path
from typing import Any

LOCK_PATH = Path(__file__).with_name("lelab_rl_v3.lock.json")
LOCK_SCHEMA = "superarm.lelab_isaac_rl_integration/v1"


def load_lock(path: Path = LOCK_PATH) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema") != LOCK_SCHEMA:
        raise ValueError(f"unexpected integration lock schema: {data.get('schema')!r}")

    lelab = data.get("lelab")
    distribution = data.get("distribution")
    contract = data.get("contract")
    if (
        not isinstance(lelab, dict)
        or not isinstance(distribution, dict)
        or not isinstance(contract, dict)
    ):
        raise TypeError(
            "integration lock must define lelab, distribution, and contract objects"
        )

    commit = lelab.get("commit")
    if (
        not isinstance(commit, str)
        or len(commit) != 40
        or any(c not in "0123456789abcdef" for c in commit)
    ):
        raise ValueError("integration lock must pin one full lowercase LeLab commit")
    required_paths = lelab.get("required_paths")
    if (
        not isinstance(required_paths, list)
        or not required_paths
        or not all(isinstance(item, str) and item for item in required_paths)
    ):
        raise ValueError("integration lock must list required LeLab paths")

    if contract != {
        "logical_action_width": 6,
        "physical_dof_count": 13,
        "real_hardware_grasp_max": 0.5,
        "simulation_grasp_max": 1.0,
    }:
        raise ValueError(
            "integration lock changed the approved SuperArm action or grasp boundary"
        )
    if distribution.get("passive_follower_count") != 88:
        raise ValueError("integration lock must require exactly 88 passive followers")
    if distribution.get("outer_shells_included") is not False:
        raise ValueError("integration lock must reject AmazingHand outer shells")
    return data


def _git(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode:
        detail = result.stderr.strip() or result.stdout.strip()
        raise ValueError(f"git {' '.join(args)} failed for {repo}: {detail}")
    return result.stdout.strip()


def validate_lelab_checkout(repo: Path, lock: dict[str, Any]) -> dict[str, Any]:
    repo = repo.expanduser().resolve()
    if not repo.is_dir():
        raise ValueError(f"LeLab checkout does not exist: {repo}")

    pinned = lock["lelab"]["commit"]
    head = _git(repo, "rev-parse", "HEAD")
    if head != pinned:
        raise ValueError(
            f"LeLab checkout must be exactly at pinned commit {pinned}; found {head}"
        )
    dirty = _git(repo, "status", "--porcelain")
    if dirty:
        raise ValueError("LeLab checkout is not clean; refusing an unpinned runtime")

    missing = [
        path for path in lock["lelab"]["required_paths"] if not (repo / path).is_file()
    ]
    if missing:
        raise ValueError(f"LeLab checkout is incomplete; missing {missing}")
    return {"path": str(repo), "commit": head, "worktree_clean": True}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_distribution(path: Path, lock: dict[str, Any]) -> dict[str, Any]:
    path = path.expanduser().resolve()
    expected = lock["distribution"]
    if not path.is_file():
        raise ValueError(f"Isaac distribution does not exist: {path}")
    if path.name != expected["filename"]:
        raise ValueError(f"expected only {expected['filename']}; found {path.name}")

    digest = _sha256(path)
    if digest != expected["sha256"]:
        raise ValueError(f"Isaac distribution checksum mismatch: {digest}")

    with zipfile.ZipFile(path) as archive:
        if archive.testzip() is not None:
            raise ValueError("Isaac distribution contains a corrupt member")
        names = archive.namelist()
        manifest_names = [name for name in names if name.endswith("/manifest.json")]
        if len(manifest_names) != 1:
            raise ValueError(
                "Isaac distribution must contain exactly one rooted manifest"
            )
        manifest_name = manifest_names[0]
        root = manifest_name.removesuffix("/manifest.json")
        if root != path.stem:
            raise ValueError(
                "Isaac distribution root directory must match the archive filename"
            )
        manifest = json.loads(archive.read(manifest_name))
        entrypoint_member = f"{root}/{expected['entrypoint']}"
        if entrypoint_member not in names:
            raise ValueError(f"Isaac distribution is missing {expected['entrypoint']}")

    checks = {
        "schema": manifest.get("schema") == expected["manifest_schema"],
        "entrypoint": manifest.get("entrypoint") == expected["entrypoint"],
        "visual_profile": (
            manifest.get("visual_contract", {}).get("profile")
            == expected["visual_profile"]
        ),
        "passive_follower_count": (
            manifest.get("visual_contract", {}).get("passive_follower_count")
            == expected["passive_follower_count"]
        ),
        "outer_shells_excluded": (
            manifest.get("visual_contract", {}).get("outer_shells_included") is False
        ),
        "logical_action_width": (
            manifest.get("robot_contract", {}).get("logical_action_width")
            == lock["contract"]["logical_action_width"]
        ),
        "physical_dof_count": (
            manifest.get("robot_contract", {}).get("physical_dof_count")
            == lock["contract"]["physical_dof_count"]
        ),
        "real_hardware_half_close": (
            manifest.get("grasp_contract", {}).get("real_hardware_max_code")
            == lock["contract"]["real_hardware_grasp_max"]
        ),
        "full_close_simulation_only": (
            manifest.get("grasp_contract", {}).get("full_close_simulation_only") is True
        ),
    }
    failed = [name for name, passed in checks.items() if not passed]
    if failed:
        raise ValueError(f"Isaac distribution violates the pinned contract: {failed}")
    return {
        "path": str(path),
        "sha256": digest,
        "entrypoint": expected["entrypoint"],
        "visual_profile": expected["visual_profile"],
        "checks": checks,
    }


def validate_integration(lelab_repo: Path, distribution_zip: Path) -> dict[str, Any]:
    lock = load_lock()
    return {
        "status": "PASS",
        "lock": str(LOCK_PATH),
        "lelab": validate_lelab_checkout(lelab_repo, lock),
        "distribution": validate_distribution(distribution_zip, lock),
        "contract": lock["contract"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lelab-repo", type=Path, required=True)
    parser.add_argument("--distribution-zip", type=Path, required=True)
    args = parser.parse_args()
    print(
        json.dumps(
            validate_integration(args.lelab_repo, args.distribution_zip), indent=2
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
