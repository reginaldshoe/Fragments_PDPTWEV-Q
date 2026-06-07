"""Smoke tests for refactor_v1a_baseline_freeze.

These tests do not run Gurobi or the legacy solver. They only check the
baseline-freeze support functions.
"""

from __future__ import annotations

from pathlib import Path

from refactor_v1a_baseline_freeze.experiments.run_baseline import copy_legacy_source, sha256_file


def test_copy_legacy_source_hash_match(tmp_path: Path) -> None:
    source = tmp_path / "ev_fragmentsv3.py"
    target = tmp_path / "legacy" / "ev_fragmentsv3_legacy.py"
    source.write_text("print('baseline')\n", encoding="utf-8")

    source_hash, target_hash = copy_legacy_source(source, target)

    assert source_hash == target_hash
    assert sha256_file(source) == sha256_file(target)
