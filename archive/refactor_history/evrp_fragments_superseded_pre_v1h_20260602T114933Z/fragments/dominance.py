"""Dominance filtering."""
from __future__ import annotations
from typing import Any

def dominates(a: dict[str, Any], b: dict[str, Any]) -> bool:
    if a['dom_key'] != b['dom_key']: return False
    better_or_equal = a['Ef'] <= b['Ef'] + 1e-9 and a['Lf'] >= b['Lf'] - 1e-9 and a['Tf'] <= b['Tf'] + 1e-9 and a['Emin'] <= b['Emin'] + 1e-9
    strictly_better = a['Ef'] < b['Ef'] - 1e-9 or a['Lf'] > b['Lf'] + 1e-9 or a['Tf'] < b['Tf'] - 1e-9 or a['Emin'] < b['Emin'] - 1e-9
    return better_or_equal and strictly_better

def filter_by_key(items: list[dict[str, Any]]) -> list[dict[str, Any]]:
    keep = []
    for item in items:
        if any(dominates(existing, item) for existing in keep): continue
        keep = [existing for existing in keep if not dominates(item, existing)]
        keep.append(item)
    return keep

def dominance_filter(items: list[dict[str, Any]]) -> list[dict[str, Any]]:
    buckets = {}
    for item in items: buckets.setdefault(item['dom_key'], []).append(item)
    out = []
    for group in buckets.values(): out.extend(filter_by_key(group))
    return out
