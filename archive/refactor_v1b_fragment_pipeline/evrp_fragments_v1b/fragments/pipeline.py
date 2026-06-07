"""Fragment-pipeline orchestration for v1b."""

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
    """Return a compact summary for a list of fragment dictionaries."""

    if not frags:
        return FragmentSummary(
            count=0,
            min_len=None,
            max_len=None,
            avg_len=None,
            with_charging=0,
            without_charging=0,
        )
    lengths = [len(frag["seq"]) for frag in frags]
    with_charging = sum(1 for frag in frags if frag.get("contains_charge", False))
    return FragmentSummary(
        count=len(frags),
        min_len=min(lengths),
        max_len=max(lengths),
        avg_len=sum(lengths) / len(lengths),
        with_charging=with_charging,
        without_charging=len(frags) - with_charging,
    )


def build_fragment_sets(
    data: dict[str, Any],
    max_base_path_len: int = 18,
    source_path: str | Path = "ev_fragmentsv3.py",
    base_path_builder: BasePathBuilder | None = None,
) -> FragmentBuildResult:
    """Build restricted and extended fragment sets.

    Parameters
    ----------
    data:
        Parsed instance data using the current legacy schema.
    max_base_path_len:
        Maximum base-path length passed to base-path enumeration.
    source_path:
        Legacy source used only when the default base-path adapter is used.
    base_path_builder:
        Optional injected base-path generator for testing. It must return
        ``(base_paths, pruned)``.

    Returns
    -------
    FragmentBuildResult
        Full staged outputs for parity checking.
    """

    if max_base_path_len <= 0:
        raise ValueError("max_base_path_len must be positive.")

    if base_path_builder is None:
        base_paths, pruned = enumerate_base_paths(data, max_base_path_len, source_path=source_path)
    else:
        base_paths, pruned = base_path_builder(data, max_base_path_len)

    restricted_raw = enumerate_fragments(data, base_paths)
    restricted_dedup = dedup_exact(restricted_raw)
    restricted_meta = attach_metadata(data, restricted_dedup, exclude_last_ef=False)
    restricted_undominated = dominance_filter(restricted_meta)
    restricted_undominated = dedup_by_signature(restricted_undominated)

    extended_raw = extend_all_fragments(data, restricted_undominated)
    extended_dedup = dedup_exact(extended_raw)
    extended_meta = attach_metadata(data, extended_dedup, exclude_last_ef=True)
    extended_undominated = dominance_filter(extended_meta)
    extended_undominated = dedup_by_signature(extended_undominated)

    return FragmentBuildResult(
        base_paths=list(base_paths),
        pruned=pruned,
        restricted_raw=restricted_raw,
        restricted_dedup=restricted_dedup,
        restricted_meta=restricted_meta,
        restricted_undominated=restricted_undominated,
        extended_raw=extended_raw,
        extended_dedup=extended_dedup,
        extended_meta=extended_meta,
        extended_undominated=extended_undominated,
    )
