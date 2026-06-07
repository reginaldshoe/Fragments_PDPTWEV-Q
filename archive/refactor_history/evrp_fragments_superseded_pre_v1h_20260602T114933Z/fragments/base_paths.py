"""Base path generation.

Transition note: delegates to legacy `enumerate_base_paths` and `step` until
this layer has its own complete parity test.
"""
from __future__ import annotations
from pathlib import Path
from typing import Any
from ..legacy_adapter import get_legacy_function

def step(data: dict[str, Any], state: Any, j: int, source_path: str | Path = 'ev_fragmentsv3.py') -> Any:
    return get_legacy_function('step', source_path)(data, state, j)

def enumerate_base_paths(data: dict[str, Any], maxlen: int, source_path: str | Path = 'ev_fragmentsv3.py') -> tuple[list[Any], Any]:
    return get_legacy_function('enumerate_base_paths', source_path)(data, maxlen)
