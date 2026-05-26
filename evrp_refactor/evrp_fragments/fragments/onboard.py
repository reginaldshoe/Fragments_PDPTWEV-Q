"""Onboard-state evolution for fragment boundary states."""
from __future__ import annotations


def compute_onboard_states(seq, start_onboard, data):
    nodes = data['nodes']
    p2d = data['p2d']
    d2p = data['d2p']

    # identify pickups
    pickup_nodes = {nodes[i][0] for i in data['P']}

    load = set(start_onboard)
    start_on = load.copy()

    for sid in seq:
        if sid in pickup_nodes:
            load.add(sid)
        elif sid in d2p:
            # SAFE REMOVE (important)
            if d2p[sid] in load:
                load.remove(d2p[sid])
            else:
                # this can happen if fragment starts mid-route
                # → skip instead of crashing
                pass

    end_on = load.copy()

    return frozenset(start_on), frozenset(end_on)
