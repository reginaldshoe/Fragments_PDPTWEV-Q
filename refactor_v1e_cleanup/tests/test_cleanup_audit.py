"""Smoke tests for v1e cleanup audit helpers."""
from __future__ import annotations

from pathlib import Path

from ..evrp_refactor_v1e.audit import audit_package, extract_function_defs


def test_extract_function_defs(tmp_path: Path) -> None:
    file_path = tmp_path / 'sample.py'
    file_path.write_text('def alpha():\n    return 1\n\ndef beta():\n    return 2\n', encoding='utf-8')
    defs = extract_function_defs(file_path)
    assert [item.name for item in defs] == ['alpha', 'beta']


def test_audit_package_detects_duplicates(tmp_path: Path) -> None:
    (tmp_path / 'a.py').write_text('def duplicated():\n    return 1\n', encoding='utf-8')
    (tmp_path / 'b.py').write_text('def duplicated():\n    return 2\n', encoding='utf-8')
    audit = audit_package(tmp_path)
    assert audit.exists is True
    assert 'duplicated' in audit.duplicate_functions
