"""Legacy-prefix adapter for v1b.

This module loads function definitions from the current root-level
``ev_fragmentsv3.py`` without executing the script-level solve block.
It is a transition device only. Later refactor stages should remove this
adapter once `read_instance`, `step`, and `enumerate_base_paths` have been
fully moved into tested modules.
"""

from __future__ import annotations

from pathlib import Path
from types import MappingProxyType
from typing import Any


DEFAULT_STOP_MARKERS = (
    "instance =",
    "instance=",
)


def extract_legacy_prefix(source_text: str) -> str:
    """Return the legacy source prefix before script-level execution begins."""

    lines = source_text.splitlines()
    stop_at = len(lines)
    for idx, line in enumerate(lines):
        stripped = line.strip()
        if any(stripped.startswith(marker) for marker in DEFAULT_STOP_MARKERS):
            stop_at = idx
            break
    return "\n".join(lines[:stop_at]) + "\n"


def load_legacy_namespace(source_path: str | Path = "ev_fragmentsv3.py") -> MappingProxyType[str, Any]:
    """Execute the function-definition prefix of the legacy source.

    Parameters
    ----------
    source_path:
        Path to the current root-level `ev_fragmentsv3.py`.

    Returns
    -------
    MappingProxyType[str, Any]
        Read-only namespace containing functions defined before the script block.

    Raises
    ------
    FileNotFoundError
        If the source file cannot be found.
    RuntimeError
        If required functions are missing after prefix execution.
    """

    path = Path(source_path)
    if not path.exists():
        raise FileNotFoundError(f"Legacy source not found: {path}")

    source_text = path.read_text(encoding="utf-8")
    prefix = extract_legacy_prefix(source_text)
    namespace: dict[str, Any] = {"__name__": "ev_fragmentsv3_legacy_prefix"}
    exec(compile(prefix, str(path), "exec"), namespace)

    required = ["read_instance", "step", "enumerate_base_paths"]
    missing = [name for name in required if name not in namespace]
    if missing:
        raise RuntimeError(f"Legacy prefix did not define required functions: {missing}")

    return MappingProxyType(namespace)


def get_legacy_function(name: str, source_path: str | Path = "ev_fragmentsv3.py") -> Any:
    """Return a named function from the legacy-prefix namespace."""

    namespace = load_legacy_namespace(source_path)
    if name not in namespace:
        raise RuntimeError(f"Legacy function not found: {name}")
    return namespace[name]
