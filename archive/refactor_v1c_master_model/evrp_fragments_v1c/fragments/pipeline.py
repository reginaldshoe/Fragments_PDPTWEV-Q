"""Fragment-pipeline orchestration for v1c."""
from __future__ import annotations
from pathlib import Path
from typing import Any, Callable
from ..types import FragmentBuildResult, FragmentSummary
from .base_paths import enumerate_base_paths
from .dominance import dominance_filter
from .extended import extend_all_fragments
from .metadata import attach_metadata, dedup_by_signature, dedup_exact
from .restricted import enumerate_fragments

BasePathBuilder = Callable[[dict[str, Any], int], tuple[list[Any], Any]]

def summarise_fragments(frags: list[dict[str, Any]]) -> FragmentSummary:
    if not frags:
        return FragmentSummary(0, None, None, None, 0, 0)
    lengths = [len(f['seq']) for f in frags]
    with_charging = sum(1 for f in frags if f.get('contains_charge', False))
    return FragmentSummary(len(frags), min(lengths), max(lengths), sum(lengths) / len(lengths), with_charging, len(frags) - with_charging)

def build_fragment_sets(data: dict[str, Any], max_base_path_len: int = 18, source_path: str | Path = 'ev_fragmentsv3.py', base_path_builder: BasePathBuilder | None = None) -> FragmentBuildResult:
    if max_base_path_len <= 0:
        raise ValueError('max_base_path_len must be positive.')
    base_paths, pruned = enumerate_base_paths(data, max_base_path_len, source_path=source_path) if base_path_builder is None else base_path_builder(data, max_base_path_len)
    restricted_raw = enumerate_fragments(data, base_paths)
    restricted_dedup = dedup_exact(restricted_raw)
    restricted_meta = attach_metadata(data, restricted_dedup, exclude_last_ef=False)
    restricted_undominated = dedup_by_signature(dominance_filter(restricted_meta))
    extended_raw = extend_all_fragments(data, restricted_undominated)
    extended_dedup = dedup_exact(extended_raw)
    extended_meta = attach_metadata(data, extended_dedup, exclude_last_ef=True)
    extended_undominated = dedup_by_signature(dominance_filter(extended_meta))
    return FragmentBuildResult(list(base_paths), pruned, restricted_raw, restricted_dedup, restricted_meta, restricted_undominated, extended_raw, extended_dedup, extended_meta, extended_undominated)
