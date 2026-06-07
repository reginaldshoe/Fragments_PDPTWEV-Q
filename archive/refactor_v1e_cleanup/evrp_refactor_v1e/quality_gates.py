"""Quality-gate helpers for the cleanup stage."""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from typing import Any


def compile_package(path: str | Path) -> dict[str, Any]:
    package_path = Path(path)
    if not package_path.exists():
        return {'path': str(package_path), 'exists': False, 'returncode': None, 'stdout': '', 'stderr': 'path does not exist'}
    result = subprocess.run(
        [sys.executable, '-m', 'compileall', str(package_path)],
        text=True,
        capture_output=True,
        check=False,
    )
    return {
        'path': str(package_path),
        'exists': True,
        'returncode': result.returncode,
        'stdout': result.stdout,
        'stderr': result.stderr,
    }


def write_json(path: str | Path, payload: dict[str, Any]) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding='utf-8')
