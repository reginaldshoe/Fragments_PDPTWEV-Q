"""Extended-fragment generation from restricted fragments."""
from __future__ import annotations
from ..resources import energy_ok_fullbatt, compute_Emin, earliest_delivery_possible, best_station_between
from .onboard import compute_onboard_states


def extend_all_fragments(data, frags):

    nodes = data['nodes']
    sid_to_i = data['sid_to_i']
    p2d = data['p2d']
    CapL = data['CapL']
    P = data['P']

    pickups = [nodes[i][0] for i in P]  # list of pickup sids
    out = []

    for f in frags:
        seq = f['seq']
        start_on, end_on = compute_onboard_states(seq, f['start_onboard'],data)

        # visited customers + stations from sequence
        visited = set(seq)

        # end sid of this fragment
        end_sid = seq[-1]

        # depot extension if end_onboard empty
        if len(end_on) == 0:
            if energy_ok_fullbatt(data, end_sid,'D0'):
                seq2 = seq + ('D0',)
            # allow additional station if depot unreachable
            else:
                s = best_station_between(data, end_sid, 'D0')
                if s is None:
                    continue
                seq2 = seq + (s,'D0')
            out.append({
                **f,
                'seq': seq2,
                'Start': seq2[0],
                'End': 'D0',
                'start_onboard': f['start_onboard'],
                'min_start_energy': compute_Emin(data, seq2),
                'end_onboard': frozenset(),
                'ext_to': 'D0',
                'ext_station': None,
                'ext_delivery': None,
            })

        # extend to every pickup i
        for i_sid in pickups:
            # EXCLUSIONS
            # exclude if next pickup already visited in fragment
            if i_sid in visited:
                continue
            # exclude if next pickup had been onboard at some stage during fragment
            if i_sid in start_on or i_sid in end_on:
                continue
            # exclude next pickup if its delivery already occurred inside the fragment
            d_sid = p2d.get(i_sid)
            if d_sid in visited:
                continue

            # capacity after picking i
            new_end_on = end_on | {i_sid}
            if sum(nodes[sid_to_i[n]][5] for n in new_end_on) > CapL + 1e-9:
                continue

            # the request from pickup to delivery must be time-feasible on its own
            if not earliest_delivery_possible(data, i_sid):
                continue

            # energy reachability end -> i (allow one station)
            ext_station = None
            if energy_ok_fullbatt(data, end_sid, i_sid):
                ext_station = None
            else:
                ext_station = best_station_between(data, end_sid, i_sid)
                if ext_station is None:
                    continue

            # build extended sequence
            if ext_station is None:
                seq2 = seq + (i_sid,)
            else:
                seq2 = seq + (ext_station, i_sid)

            out.append({
                'seq': seq2,
                'start_onboard': f['start_onboard'],
                'end_onboard': frozenset(new_end_on),
                'contains_charge': f['contains_charge'] or (ext_station is not None),
                'min_start_energy': compute_Emin(data, seq2),
                'ext_to': i_sid,
                'ext_station': ext_station,
                'ext_delivery': d_sid,
            })

    return out
