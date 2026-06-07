"""Base-path adapter for v1b.

This module intentionally delegates base-path generation to the legacy source.
The base-path logic is moved in a later step after fragment-pipeline parity is
established.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from ..legacy_adapter import get_legacy_function


def read_instance(path: str | Path, source_path: str | Path = "ev_fragmentsv3.py") -> dict[str, Any]:
    func = get_legacy_function("read_instance", source_path)
    return func(Path(path))


def step(data: dict[str, Any], state: Any, j: int, source_path: str | Path = "ev_fragmentsv3.py") -> Any:
    func = get_legacy_function("step", source_path)
    return func(data, state, j)


def enumerate_base_paths(
    data: dict[str, Any],
    maxlen: int,
    source_path: str | Path = "ev_fragmentsv3.py",
) -> tuple[list[Any], Any]:
    func = get_legacy_function("enumerate_base_paths", source_path)
    return func(data, maxlen)
