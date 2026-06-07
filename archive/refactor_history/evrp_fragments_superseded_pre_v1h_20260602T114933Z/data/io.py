"""Instance loading.

Transition note: this currently delegates to root-level `ev_fragmentsv3.py` to
preserve baseline behaviour until the parser is fully migrated.
"""
from __future__ import annotations
from pathlib import Path
from typing import Any
from ..legacy_adapter import get_legacy_function

def read_instance(path: str | Path, source_path: str | Path = 'ev_fragmentsv3.py') -> dict[str, Any]:
    return get_legacy_function('read_instance', source_path)(Path(path))
