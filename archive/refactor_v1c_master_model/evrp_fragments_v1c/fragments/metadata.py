"""Fragment metadata attachment and duplicate removal."""
from __future__ import annotations
from typing import Any
from ..node_utils import is_customer
from ..resources import compute_T_E_L, compute_distance

def cust_locs(data: dict[str, Any], seq: tuple[str, ...] | list[str], exclude_last: bool) -> frozenset[str]:
    out = [sid for sid in seq if is_customer(data, sid)]
    if seq and exclude_last and is_customer(data, seq[-1]):
        out = [sid for sid in out if sid != seq[-1]]
    return frozenset(out)

def dedup_exact(frags: list[dict[str, Any]]) -> list[dict[str, Any]]:
    seen = set(); out = []
    for frag in frags:
        sig = (tuple(frag['seq']), frozenset(frag['start_onboard']), frozenset(frag['end_onboard']))
        if sig in seen: continue
        seen.add(sig); out.append(frag)
    return out

def dedup_by_signature(frags: list[dict[str, Any]]) -> list[dict[str, Any]]:
    seen = set(); out = []
    for frag in frags:
        sig = (tuple(frag['seq']), frozenset(frag['start_onboard']), frozenset(frag['end_onboard']), frag['Ef'], frag['Lf'], frag['Tf'], frag['Emin'])
        if sig in seen: continue
        seen.add(sig); out.append(frag)
    return out

def attach_metadata(data: dict[str, Any], frags: list[dict[str, Any]], exclude_last_ef: bool = False) -> list[dict[str, Any]]:
    out = []
    for frag in frags:
        seq = tuple(frag['seq'])
        tf, ef, lf = compute_T_E_L(data, seq)
        item = dict(frag)
        item.update({'seq': seq, 'Tf': tf, 'Ef': ef, 'Lf': lf, 'Df': compute_distance(data, seq), 'Emin': item['min_start_energy'], 'Start': seq[0], 'End': seq[-1], 'LocsC': cust_locs(data, seq, exclude_last_ef)})
        item['dom_key'] = (item['Start'], item['End'], item['start_onboard'], item['end_onboard'], item['LocsC'])
        out.append(item)
    return out
