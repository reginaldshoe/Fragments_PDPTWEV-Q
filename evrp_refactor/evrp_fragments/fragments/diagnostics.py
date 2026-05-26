"""Fragment and DP diagnostic printing helpers."""
from __future__ import annotations
from ..node_utils import is_station


def print_dp_path_structure(tag, data, dp_path, dp_dist):
    stations = [sid for sid in dp_path if is_station(data, sid)]
    print(f"[{tag}] DP distance = {dp_dist:.6f}")
    print(f"[{tag}] DP path = {list(dp_path)}")
    print(f"[{tag}] Stations used ({len(stations)}): {stations}")

def stats_frags(frags):
    lens = [len(f['seq']) for f in frags]
    with_ch = sum(1 for f in frags if f['contains_charge'])
    out = {
        'count': len(frags),
        'min_len': min(lens),
        'max_len': max(lens),
        'avg_len': sum(lens) / len(lens),
        'with_charging': with_ch,
        'without_charging': len(frags) - with_ch,
    }
    print(out)

def stats_ext(efrags):
    if not efrags:
        out = {'count': 0}
        print(out)
        return out
    lens = [len(f['seq']) for f in efrags]
    depot_end = sum(1 for f in efrags if f.get('ext_to') == 'D0')
    with_station = sum(1 for f in efrags if f.get('ext_station') is not None)
    out = {
        'count': len(efrags),
        'min_len': min(lens),
        'max_len': max(lens),
        'avg_len': sum(lens)/len(lens),
        'depot_extensions': depot_end,
        'extensions_with_station': with_station,
    }
    print(out)
    return out
