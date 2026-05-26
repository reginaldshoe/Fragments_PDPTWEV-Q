"""Restricted-fragment generation from base paths."""
from __future__ import annotations
from ..config import DEBUG
from ..resources import compute_Emin


def trim_base_path(data, base_path):

    nodes = data['nodes']
    d2p = data['d2p']

    P = list(data['P'])
    D = list(data['D'])

    # find first delivery index within a base path (phase switch)
    d_switch = None
    for d, idx in enumerate(base_path):
        if nodes[idx][2] == 'cd':
            d_switch = d
            break
    # no delivery = no fragment
    if d_switch is None:
        return []

    # split base path into two parts, pickup and delivery
    pickup_part = base_path[:d_switch]
    delivery_part = base_path[d_switch:]

    # convert back to sid
    Pseq = [nodes[i][0] for i in pickup_part if i in P]
    Dseq = [nodes[i][0] for i in delivery_part if i in D]

    # validate that both pickup/delivery occurs
    if not Pseq or not Dseq or len(Pseq) != len(Dseq):
        return []

    # position maps for slicing
    pos = {}
    for t, idx in enumerate(base_path):
        sid = nodes[idx][0]
        pos[sid] = t

    frags = []
    p_len = len(Pseq)
    d_len = len(Dseq)

    # loop all possible starting point in base_path
    for a in range(p_len):
        start_onboard = frozenset(Pseq[:a+1])
        start_sid = Pseq[a]
        s0 = pos[start_sid]

        # loop all possible end points, track what is still on board
        for b in range(d_len):
            kept_delivery = Dseq[:d_len - b]
            removed_delivery = Dseq[d_len - b:]
            # at least one delivery
            if not kept_delivery:
                continue
            end_sid = kept_delivery[-1]
            s1 = pos[end_sid]
            if s1 < s0:
                continue

            # fragment sid
            subseq = base_path[s0:s1 + 1]
            seq_sids = tuple(nodes[i][0] for i in subseq)

            # onboard at end of fragment
            end_onboard = set()
            for d_sid in removed_delivery:
                p_sid = d2p.get(d_sid)
                if p_sid:
                    end_onboard.add(p_sid)

            # calculate energy required to reach first charging station
            # prefix fragment in network would need to know this for feasibility
            energy_req = compute_Emin(data, seq_sids)

            frags.append({
                'seq': seq_sids,
                'start_onboard': start_onboard,
                'end_onboard': frozenset(end_onboard),
                'contains_charge': any(nodes[i][1] == 'S' or nodes[i][2] == 'f' for i in subseq),
                'min_start_energy': energy_req,
            })
    if DEBUG:
        print("trim_base_path returning", len(frags), "frags")
    return frags

def enumerate_fragments(data, base_paths):
    frags = []
    for bp in base_paths:
        frags.extend(trim_base_path(data, bp))
    return frags
