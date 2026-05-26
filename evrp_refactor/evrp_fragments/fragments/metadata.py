"""Fragment metadata attachment and duplicate removal."""
from __future__ import annotations
from ..node_utils import is_customer
from ..resources import compute_T_E_L, compute_distance


def cust_locs(data, seq, exclude_last):
    if not seq:
        return frozenset()
    out = []
    end_sid = seq[-1]
    for sid in seq:
        if is_customer(data, sid):
            out.append(sid)
    if exclude_last and is_customer(data, end_sid):
        out = [x for x in out if x != end_sid]
    return frozenset(out)

def dedup_exact(frags):
    seen = set()
    out = []
    for f in frags:
        sig = (f['seq'], f['start_onboard'], f['end_onboard'])
        if sig in seen:
            continue
        seen.add(sig)
        out.append(f)
    return out

def dedup_by_signature(frags):
    seen = set()
    out = []
    for f in frags:
        sig = (f['seq'], f['start_onboard'], f['end_onboard'], f['Ef'], f['Lf'], f['Tf'], f['Emin'])
        if sig in seen:
            continue
        seen.add(sig)
        out.append(f)
    return out

def attach_metadata(data, frags, exclude_last_ef = False):
    out = []
    for f in frags:
        seq = f['seq']
        Tf, Ef, Lf = compute_T_E_L(data, seq)
        g = dict(f)
        g['Tf'] = Tf
        g['Ef'] = Ef
        g['Lf'] = Lf
        g['Df'] = compute_distance(data, seq)
        g['Emin'] = g['min_start_energy']  # already computed in trimming
        g['Start'] = seq[0]
        g['End'] = seq[-1]
        g['LocsC'] = cust_locs(data, seq, exclude_last=exclude_last_ef)
        # Dominance key without stations
        g['dom_key'] = (g['Start'], g['End'], g['start_onboard'], g['end_onboard'], g['LocsC'])
        out.append(g)
    return out
