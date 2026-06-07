"""Fragment-pipeline orchestration."""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from .base_paths import enumerate_base_paths
from .restricted import enumerate_fragments
from .metadata import attach_metadata, dedup_by_signature, dedup_exact
from .dominance import dominance_filter
from .extended import extend_all_fragments

@dataclass(frozen=True)
class FragmentBuildResult:
    base_paths: list[Any]
    pruned: Any
    restricted_raw: list[dict[str, Any]]
    restricted_dedup: list[dict[str, Any]]
    restricted_meta: list[dict[str, Any]]
    restricted_undominated: list[dict[str, Any]]
    extended_raw: list[dict[str, Any]]
    extended_dedup: list[dict[str, Any]]
    extended_meta: list[dict[str, Any]]
    extended_undominated: list[dict[str, Any]]
    def summary(self) -> dict[str, int]:
        return {k: len(getattr(self, k)) for k in ['base_paths','restricted_raw','restricted_dedup','restricted_meta','restricted_undominated','extended_raw','extended_dedup','extended_meta','extended_undominated']}

def build_fragment_sets(data: dict[str, Any], max_base_path_len: int = 18, source_path: str | Path = 'ev_fragmentsv3.py') -> FragmentBuildResult:
    if max_base_path_len <= 0: raise ValueError('max_base_path_len must be positive.')
    base_paths, pruned = enumerate_base_paths(data, max_base_path_len, source_path=source_path)
    rr = enumerate_fragments(data, base_paths); rd = dedup_exact(rr); rm = attach_metadata(data, rd, False); ru = dedup_by_signature(dominance_filter(rm))
    er = extend_all_fragments(data, ru); ed = dedup_exact(er); em = attach_metadata(data, ed, True); eu = dedup_by_signature(dominance_filter(em))
    return FragmentBuildResult(list(base_paths), pruned, rr, rd, rm, ru, er, ed, em, eu)
