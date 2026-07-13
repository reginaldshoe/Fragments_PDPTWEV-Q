#!/usr/bin/env python3
"""
APPLY_v6d3_rollback_final_status_reporting.py

Purpose
-------
Rollback the v6d final-status reporting overlays because the post-patch run
behaviour is no longer matching the pre-patch benchmark evidence.

Environment
-----------
- Python 3.12
- Standard library only
- Target shape: standalone rollback script

Scope
-----
Restores, when backups exist:
- evrp_fragments/callback_core.py from callback_core.py.bak_v6d1_final_solver_status_reporting
- evrp_fragments/pipeline.py from pipeline.py.bak_v6d1_final_solver_status_reporting
- evrp_fragments/master_core.py from master_core.py.bak_v6d1_final_solver_status_reporting

This removes v6d1/v6d1b/v6d2 reporting changes and returns the affected source
files to their pre-v6d1 state. It does not touch result JSON files.

Usage
-----
Audit only:
    python APPLY_v6d3_rollback_final_status_reporting.py --audit-only

Rollback and compile:
    python APPLY_v6d3_rollback_final_status_reporting.py --apply --compile

Safety backups
--------------
Before restoring, the script saves the current patched files as:
    *.bak_v6d3_before_rollback
"""
from __future__ import annotations

import argparse
import py_compile
import shutil
from pathlib import Path

OVERLAY_NAME = "v6d3_rollback_final_status_reporting"
PRE_ROLLBACK_SUFFIX = ".bak_v6d3_before_rollback"

RESTORE_PAIRS = [
    (
        Path("evrp_fragments") / "callback_core.py",
        Path("evrp_fragments") / "callback_core.py.bak_v6d1_final_solver_status_reporting",
    ),
    (
        Path("evrp_fragments") / "pipeline.py",
        Path("evrp_fragments") / "pipeline.py.bak_v6d1_final_solver_status_reporting",
    ),
    (
        Path("evrp_fragments") / "master_core.py",
        Path("evrp_fragments") / "master_core.py.bak_v6d1_final_solver_status_reporting",
    ),
]


def safety_backup(path: Path) -> Path | None:
    if not path.exists():
        return None
    backup_path = path.with_name(path.name + PRE_ROLLBACK_SUFFIX)
    if not backup_path.exists():
        shutil.copy2(path, backup_path)
    return backup_path


def restore_file(target: Path, source_backup: Path) -> str:
    if not source_backup.exists():
        return f"WARNING: missing rollback source {source_backup}; did not restore {target}"
    safety = safety_backup(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_backup, target)
    if safety is None:
        return f"restored {target} from {source_backup}; no prior target file existed"
    return f"restored {target} from {source_backup}; saved current patched file as {safety}"


def compile_targets() -> None:
    for target, _ in RESTORE_PAIRS:
        if target.exists():
            py_compile.compile(str(target), doraise=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=f"Apply {OVERLAY_NAME}")
    parser.add_argument("--audit-only", action="store_true")
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--compile", action="store_true")
    args = parser.parse_args()

    if not args.audit_only and not args.apply:
        parser.error("choose --audit-only or --apply")

    print(f"[{OVERLAY_NAME}] audit")
    for target, source_backup in RESTORE_PAIRS:
        status = "available" if source_backup.exists() else "MISSING"
        print(f"- {target} <= {source_backup} [{status}]")

    if args.audit_only:
        return

    for target, source_backup in RESTORE_PAIRS:
        print(f"[{OVERLAY_NAME}] {restore_file(target, source_backup)}")

    if args.compile:
        compile_targets()
        print(f"[{OVERLAY_NAME}] compile OK")


if __name__ == "__main__":
    main()
