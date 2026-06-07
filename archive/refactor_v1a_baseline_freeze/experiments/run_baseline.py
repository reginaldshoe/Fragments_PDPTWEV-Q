"""Baseline-freeze runner for the EV fragment solver.

Role: derived output / baseline freeze.

This module copies the current root-level ``ev_fragmentsv3.py`` into
``legacy/ev_fragmentsv3_legacy.py`` and optionally executes that copy.
It deliberately does not refactor or alter the solver algorithm.

Target environment:
- Python 3.12
- gurobipy 13.0.1, if that is the installed solver version
- Target shape: script
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path


@dataclass(frozen=True)
class BaselineRunMetadata:
    """Metadata captured for an auditable baseline run."""

    source_path: str
    legacy_copy_path: str
    artefact_dir: str
    source_sha256: str
    legacy_sha256: str
    copy_only: bool
    return_code: int | None
    created_utc: str


def sha256_file(path: Path) -> str:
    """Return the SHA-256 hash of ``path``.

    Parameters
    ----------
    path:
        File to hash.

    Returns
    -------
    str
        Hexadecimal SHA-256 digest.

    Raises
    ------
    FileNotFoundError
        If ``path`` does not exist.
    """

    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def copy_legacy_source(source_path: Path, legacy_copy_path: Path) -> tuple[str, str]:
    """Copy the working source file into the frozen legacy location.

    Parameters
    ----------
    source_path:
        Current root-level ``ev_fragmentsv3.py``.
    legacy_copy_path:
        Destination for the frozen legacy copy.

    Returns
    -------
    tuple[str, str]
        ``(source_sha256, legacy_sha256)``.

    Raises
    ------
    FileNotFoundError
        If the source file does not exist.
    RuntimeError
        If the copied file hash does not match the source hash.
    """

    if not source_path.exists():
        raise FileNotFoundError(
            f"Source file not found: {source_path}. Run this from the project root "
            "or pass --source explicitly."
        )

    legacy_copy_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_path, legacy_copy_path)

    source_hash = sha256_file(source_path)
    legacy_hash = sha256_file(legacy_copy_path)
    if source_hash != legacy_hash:
        raise RuntimeError(
            "Legacy copy hash mismatch. The baseline was not frozen safely."
        )
    return source_hash, legacy_hash


def run_legacy_script(legacy_copy_path: Path, project_root: Path) -> subprocess.CompletedProcess[str]:
    """Execute the frozen legacy script in a subprocess.

    The subprocess working directory is the project root so that legacy code using
    ``Path.cwd() / 'instances'`` behaves as it did before the refactor.
    """

    return subprocess.run(
        [sys.executable, str(legacy_copy_path)],
        cwd=project_root,
        text=True,
        capture_output=True,
        check=False,
    )


def write_text(path: Path, text: str) -> None:
    """Write UTF-8 text, creating parent directories if required."""

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def write_json(path: Path, payload: dict) -> None:
    """Write formatted JSON, creating parent directories if required."""

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line parser."""

    parser = argparse.ArgumentParser(description="Freeze and run the EV fragment legacy baseline.")
    parser.add_argument(
        "--source",
        default="ev_fragmentsv3.py",
        help="Path to the current working source file, relative to the project root unless absolute.",
    )
    parser.add_argument(
        "--legacy-copy",
        default="refactor_v1a_baseline_freeze/legacy/ev_fragmentsv3_legacy.py",
        help="Destination path for the frozen legacy copy.",
    )
    parser.add_argument(
        "--artefact-dir",
        default="refactor_v1a_baseline_freeze/artefacts",
        help="Directory for stdout, stderr, and metadata artefacts.",
    )
    parser.add_argument(
        "--copy-only",
        action="store_true",
        help="Only create the frozen copy and metadata; do not execute the legacy script.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Freeze the current source and optionally run the legacy baseline.

    Parameters
    ----------
    argv:
        Optional command-line arguments for testing.

    Returns
    -------
    int
        Process-style return code. Zero indicates copy success and, if executed,
        a zero return code from the legacy script.
    """

    args = build_parser().parse_args(argv)
    project_root = Path.cwd()

    source_path = Path(args.source)
    if not source_path.is_absolute():
        source_path = project_root / source_path

    legacy_copy_path = Path(args.legacy_copy)
    if not legacy_copy_path.is_absolute():
        legacy_copy_path = project_root / legacy_copy_path

    artefact_dir = Path(args.artefact_dir)
    if not artefact_dir.is_absolute():
        artefact_dir = project_root / artefact_dir

    source_hash, legacy_hash = copy_legacy_source(source_path, legacy_copy_path)

    completed: subprocess.CompletedProcess[str] | None = None
    if not args.copy_only:
        completed = run_legacy_script(legacy_copy_path, project_root)
        write_text(artefact_dir / "baseline_stdout.txt", completed.stdout)
        write_text(artefact_dir / "baseline_stderr.txt", completed.stderr)

    metadata = BaselineRunMetadata(
        source_path=str(source_path),
        legacy_copy_path=str(legacy_copy_path),
        artefact_dir=str(artefact_dir),
        source_sha256=source_hash,
        legacy_sha256=legacy_hash,
        copy_only=args.copy_only,
        return_code=None if completed is None else completed.returncode,
        created_utc=datetime.now(timezone.utc).isoformat(),
    )
    write_json(artefact_dir / "baseline_metadata.json", asdict(metadata))

    if completed is not None:
        return completed.returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
