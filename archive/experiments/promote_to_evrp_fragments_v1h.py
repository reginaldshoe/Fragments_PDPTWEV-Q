"""Promote the passing consolidated EVRP solver into the evrp_fragments package.

Environment:
- Python 3.12

Role: draft output v1h patch 2 / promotion overlay.

Patch 2 handles Windows permission issues when deleting stale __pycache__ folders.
If the old evrp_fragments package cannot be fully removed after archiving, the
script falls back to overlay mode: it writes the promoted runtime files into the
existing evrp_fragments directory and records that fallback in the manifest.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import stat
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

SOURCE_PACKAGE = Path("evrp_fragments_consolidated_v1")
TARGET_PACKAGE = Path("evrp_fragments")
ARCHIVE_ROOT = Path("archive") / "refactor_history"

RUNTIME_FILES = [
    "fragment_core.py",
    "master_core.py",
    "callback_core.py",
    "dependency_bridge.py",
    "pipeline_v1e.py",
    "pipeline.py",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Promote consolidated EVRP runtime into evrp_fragments.")
    parser.add_argument("--source-package", default=str(SOURCE_PACKAGE))
    parser.add_argument("--target-package", default=str(TARGET_PACKAGE))
    parser.add_argument("--archive-root", default=str(ARCHIVE_ROOT))
    parser.add_argument("--force", action="store_true", help="Create a suffixed archive folder if the archive target exists.")
    parser.add_argument("--strict-delete", action="store_true", help="Fail if the old evrp_fragments folder cannot be fully removed.")
    return parser.parse_args()


def unique_archive_path(base: Path, force: bool) -> Path:
    if not base.exists():
        return base
    if not force:
        raise FileExistsError(f"Archive path already exists: {base}. Re-run with --force to create a suffixed archive folder.")
    i = 2
    while True:
        candidate = Path(f"{base}_{i}")
        if not candidate.exists():
            return candidate
        i += 1


def require_file(path: Path) -> None:
    if not path.exists() or not path.is_file():
        raise FileNotFoundError(f"Required file missing: {path}")


def remove_readonly(func: Any, path: str, exc_info: Any) -> None:
    """Retry rmtree removal after making a path writeable."""
    try:
        os.chmod(path, stat.S_IWRITE)
        func(path)
    except Exception:
        raise


def safe_rmtree(path: Path) -> tuple[bool, str | None]:
    """Try to remove a directory robustly on Windows.

    Returns (removed, error_message). It first uses shutil.rmtree with a chmod
    retry hook. If that fails, it tries to remove __pycache__ directories and
    then retries once.
    """
    if not path.exists():
        return True, None
    try:
        shutil.rmtree(path, onexc=remove_readonly)
        return True, None
    except Exception as first_exc:
        first_error = f"{type(first_exc).__name__}: {first_exc}"

    try:
        for pycache in sorted(path.rglob("__pycache__"), reverse=True):
            try:
                shutil.rmtree(pycache, onexc=remove_readonly)
            except Exception:
                pass
        shutil.rmtree(path, onexc=remove_readonly)
        return True, None
    except Exception as second_exc:
        return False, f"initial={first_error}; retry={type(second_exc).__name__}: {second_exc}"


def write_promoted_runtime(source_package: Path, target_package: Path) -> None:
    target_package.mkdir(parents=True, exist_ok=True)
    init_text = (
        '"""EVRP fragments package.\n\n'
        'Role: promoted canonical runtime package.\n\n'
        'This package was promoted from evrp_fragments_consolidated_v1 after the canonical\n'
        'regression gate passed. The public API is run_solver via pipeline.py.\n'
        '"""\n\n'
        'from .pipeline import run_solver, summary_to_json\n\n'
        '__all__ = ["run_solver", "summary_to_json"]\n'
    )
    (target_package / "__init__.py").write_text(init_text, encoding="utf-8")
    for filename in RUNTIME_FILES:
        shutil.copy2(source_package / filename, target_package / filename)


def main() -> None:
    args = parse_args()
    source_package = Path(args.source_package)
    target_package = Path(args.target_package)
    archive_root = Path(args.archive_root)

    if not source_package.exists() or not source_package.is_dir():
        raise FileNotFoundError(f"Source package not found: {source_package}")
    for filename in RUNTIME_FILES:
        require_file(source_package / filename)

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    archive_root.mkdir(parents=True, exist_ok=True)
    archive_target = unique_archive_path(archive_root / f"evrp_fragments_superseded_pre_v1h_{timestamp}", args.force)

    archived_previous_target = None
    deletion_removed = True
    deletion_error = None
    promotion_mode = "clean_replace"

    if target_package.exists():
        shutil.copytree(target_package, archive_target)
        archived_previous_target = str(archive_target)
        deletion_removed, deletion_error = safe_rmtree(target_package)
        if not deletion_removed:
            if args.strict_delete:
                raise PermissionError(
                    "Archived existing evrp_fragments but could not delete it. "
                    f"Error: {deletion_error}"
                )
            promotion_mode = "overlay_after_archive_delete_failed"

    write_promoted_runtime(source_package, target_package)

    manifest = {
        "name": "evrp_fragments_promoted_v1h_patch2",
        "role": "promoted canonical runtime package",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source_package": str(source_package),
        "target_package": str(target_package),
        "archived_previous_target": archived_previous_target,
        "promotion_mode": promotion_mode,
        "old_target_fully_removed_before_promotion": deletion_removed,
        "old_target_delete_error": deletion_error,
        "runtime_files": [str(target_package / f) for f in ["__init__.py"] + RUNTIME_FILES],
        "notes": [
            "Existing evrp_fragments package was archived before promotion if it existed.",
            "If Windows denied deletion of stale cache folders, promoted runtime files were overlaid into evrp_fragments.",
            "No solver logic was changed by the promotion script.",
            "Canonical experiments now import evrp_fragments.pipeline."
        ],
    }
    Path("MANIFEST_v1h_promote_to_evrp_fragments_result.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
