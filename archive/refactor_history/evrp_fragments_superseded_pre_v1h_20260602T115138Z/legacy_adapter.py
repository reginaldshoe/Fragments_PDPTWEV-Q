"""Legacy-prefix adapter for preserving baseline behaviour during integration."""
from __future__ import annotations
from pathlib import Path
from types import MappingProxyType
from typing import Any

DEFAULT_STOP_MARKERS = ('instance =', 'instance=')

def extract_legacy_prefix(source_text: str) -> str:
    lines = source_text.splitlines()
    stop_at = len(lines)
    for idx, line in enumerate(lines):
        if any(line.strip().startswith(marker) for marker in DEFAULT_STOP_MARKERS):
            stop_at = idx
            break
    return '\n'.join(lines[:stop_at]) + '\n'

def load_legacy_namespace(source_path: str | Path = 'ev_fragmentsv3.py') -> MappingProxyType[str, Any]:
    path = Path(source_path)
    if not path.exists():
        raise FileNotFoundError(f'Legacy source not found: {path}')
    prefix = extract_legacy_prefix(path.read_text(encoding='utf-8'))
    ns: dict[str, Any] = {'__name__': 'ev_fragmentsv3_legacy_prefix'}
    exec(compile(prefix, str(path), 'exec'), ns)
    required = ['read_instance', 'step', 'enumerate_base_paths']
    missing = [name for name in required if name not in ns]
    if missing:
        raise RuntimeError(f'Legacy prefix did not define required functions: {missing}')
    return MappingProxyType(ns)

def get_legacy_function(name: str, source_path: str | Path = 'ev_fragmentsv3.py') -> Any:
    ns = load_legacy_namespace(source_path)
    if name not in ns:
        raise RuntimeError(f'Legacy function not found: {name}')
    return ns[name]
