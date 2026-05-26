"""Dominance filtering for restricted and extended fragments."""
from __future__ import annotations


def dominates(a, b):

    if a['dom_key'] != b['dom_key']:
        return False
    better_or_equal = (a['Ef'] <= b['Ef'] + 1e-9 and
                       a['Lf'] >= b['Lf'] - 1e-9 and
                       a['Tf'] <= b['Tf'] + 1e-9 and
                       a['Emin'] <= b['Emin'] + 1e-9)
    strictly_better = (a['Ef'] < b['Ef'] - 1e-9 or
                       a['Lf'] > b['Lf'] + 1e-9 or
                       a['Tf'] < b['Tf'] - 1e-9 or
                       a['Emin'] < b['Emin'] - 1e-9)
    return better_or_equal and strictly_better

def filter_by_key(items):
    keep = []
    for x in items:
        # if any in kept dominates x -> drop x
        if any(dominates(k, x) for k in keep):
            continue
        # else remove those dominated by x
        keep = [k for k in keep if not dominates(x, k)]
        keep.append(x)
    return keep

def dominance_filter(items):
    buckets = {}
    for f in items:
        buckets.setdefault(f['dom_key'], []).append(f)
    out = []
    for key, group in buckets.items():
        out.extend(filter_by_key(group))
    return out
