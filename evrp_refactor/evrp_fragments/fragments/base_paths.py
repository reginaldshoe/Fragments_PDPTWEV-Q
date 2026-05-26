"""Base-path enumeration using the local state-extension routine."""
from __future__ import annotations
from ..config import DEBUG
from ..node_utils import is_station, is_pickup, is_delivery


def step(data, state, j):

    nodes = data['nodes']
    traveltime = data['traveltime']
    energy = data['energy']
    CapE = data['CapE']
    CapL = data['CapL']
    rech = data['rech']
    sid_to_i = data['sid_to_i']
    d2p = data['d2p']
    dist = data['dist']


    # current state
    # path, pickup/delivery 0/1, load onboard, energy level, time depart last node, prev visited P/D/S, delivery count, total distance
    path, phase, onboard, E, t_depart, seenP, seenD, seenS, deliv_count,total_dist = state

    # last step of path
    i = path[-1]
    sid_j, kind_j, typ_j, xj, yj, dem_j, ready_j, due_j, serv_j, partner_j = nodes[j]

    dist2 = total_dist

    # avoid revisiting same station
    # if is_station(data, sid_j) and sid_j in seenS:
    #     return None, 'revisit_station'

    # time feasibility
    t_arr = t_depart + traveltime(i, j)
    t_start = max(ready_j, t_arr)
    if t_start > due_j + 1e-9:
        return None, 'timewindow'

    # energy feasibility
    E_arr = E - energy(i, j)
    if E_arr < 0:
        return None, 'energy'

    # If node is a charging station, charge to full capacity and return
    if is_station(data, sid_j):
        charge_time = (CapE - E_arr) * rech
        t_depart2 = t_start + charge_time
        E2 = CapE
        seenS2 = set(seenS)
        seenS2.add(sid_j)
        dist2 += dist(i,j)
        return (path + (j,), phase, onboard, E2, t_depart2, seenP, seenD, frozenset(seenS2), deliv_count, dist2), None

    # if node is a customer, update params
    onboard2 = set(onboard)
    seenP2 = set(seenP)
    seenD2 = set(seenD)
    phase2 = phase
    deliv2 = deliv_count

    # add load if pickup
    if is_pickup(data, sid_j):
        onboard2.add(sid_j)
        seenP2.add(sid_j)
    elif is_delivery(data, sid_j):
        # must correspond to an onboard pickup
        p_sid = d2p.get(sid_j)
        if p_sid not in onboard2:
            return None, 'delivery_not_onboard'
        onboard2.remove(p_sid)
        seenD2.add(sid_j)
        deliv2 += 1
        if phase2 == 0:
            phase2 = 1  # switch to DELIVERY

    # if pickup, check current demand and make sure += new demand <CapL
    load = 0.0
    for p_sid in onboard2:
        pi = sid_to_i[p_sid]
        load += nodes[pi][5] #demand
    if load > CapL + 1e-9:
        return None, 'capacity'

    # update time and distance
    t_depart2 = t_start + serv_j
    dist2 += dist(i,j)

    return (path + (j,), phase2, frozenset(onboard2), E_arr, t_depart2,
            frozenset(seenP2), frozenset(seenD2), seenS, deliv2,dist2), None

def enumerate_base_paths(data, maxlen):

    nodes = data['nodes']
    CapE = data['CapE']
    energy = data['energy']
    traveltime = data['traveltime']
    sid_to_i = data['sid_to_i']
    p2d = data['p2d']
    dist = data['dist']

    P = list(data['P'])
    S = list(data['S'])


    pruned = {}
    def prune(r):
        pruned[r] = pruned.get(r, 0) + 1

    # seed states from each pickup, starting at depot with full charge
    depot = 0

    # start a set for 'working' paths with all seeds
    Work = set()
    for p in P:
        sid_p, kind_p, typ_p, xp, yp, dem_p, ready_p, due_p, serv_p, partner_p = nodes[p]
        # travel from depot
        E_arr = CapE - energy(depot, p)
        if E_arr < -1e-9:
            prune('seed_energy')
            continue
        t_arr = traveltime(depot, p)
        t_start = max(ready_p, t_arr)
        if t_start > due_p + 1e-9:
            prune('seed_timewindow')
            continue
        t_depart = t_start + serv_p
        distance = dist(depot, p)

        onboard = frozenset([sid_p])
        state = ((p,), 0, onboard, E_arr, t_depart, frozenset([sid_p]), frozenset(), frozenset(), 0,distance)
        Work.add(state)

    # set of base paths
    Base = set()

    for depth in range(maxlen):
        NewWork = set()

        # check each working path to see if it can be completed, otherwise attempt extension
        for st in Work:
            path, phase, onboard, E, t_depart, seenP, seenD, seenS, deliv_count,distance = st

            # base path complete (i.e. empty + currently in delivery phase + at least one delivery)
            if phase == 1 and len(onboard) == 0 and deliv_count > 0:
                Base.add(path)
                continue

            # otherwise extend path
            # try a delivery of currently onboard pickups
            for p_sid in onboard:
                d_sid = p2d.get(p_sid)
                j = sid_to_i.get(d_sid)
                newst, reason = step(data, st, j)
                if newst is None:
                    prune(reason)
                else:
                    NewWork.add(newst)

            # if still pickup phase, allow more pickups
            if phase == 0:
                for q in P:
                    sid_q = nodes[q][0]
                    if sid_q in seenP:
                        continue
                    newst, reason = step(data, st, q)
                    if newst is None:
                        prune(reason)
                    else:
                        NewWork.add(newst)

            # move to charging station (avoid station to station)
            prev_station = (path[-1] in S)
            if prev_station:
                prune('prev_station')
            if not prev_station:
                for s in S:
                    newst, reason = step(data, st, s)
                    if newst is None:
                        prune(reason)
                    else:
                        NewWork.add(newst)

        Work = NewWork
        if DEBUG:
            print('depth', depth, 'work', len(Work), 'base', len(Base))
        if not Work:
            break

    return list(Base), pruned
