"""Base-path adapter for v1c."""
from __future__ import annotations
from pathlib import Path
from typing import Any
from ..legacy_adapter import get_legacy_function

def read_instance(path: str | Path, source_path: str | Path = 'ev_fragmentsv3.py') -> dict[str, Any]:
    return get_legacy_function('read_instance', source_path)(Path(path))

def enumerate_base_paths(data: dict[str, Any], maxlen: int, source_path: str | Path = 'ev_fragmentsv3.py') -> tuple[list[Any], Any]:
    return get_legacy_function('enumerate_base_paths', source_path)(data, maxlen)
