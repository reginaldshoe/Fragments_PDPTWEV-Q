# Focusing on trying to develop fragment algo first.

# Implementation structure, tried to follow principles from Rist and Forbes
# 1. enumerate full 'base' paths (defined where onboard load is empty at the start and the end of path)
# 1a. using a helper algorithm to update the next step
# 2. IN PROGRESS: trim paths to truncated restricted fragments (defined where path has exactly one switch from pickup to delivery)
# 3. TODO: extend truncated restricted fragments to the next pickup

import math


# Used Copilot to develop read_instance block, needed some editing
def read_instance(path):
    #ID, type, x, y, demand, ready, due, service time, partner ID
    with open(path, 'r') as f:
        f.readline()
        rows = []
        first_param = None
        for line in f:
            s = line.strip()
            if not s:
                continue
            if ':' in s:
                first_param = s
                break
            parts = s.split()
            if len(parts) < 9:
                continue
            sid, typ, x, y, dem, rt, dt, st, partner = parts[:9]
            rows.append((sid, typ, float(x), float(y), float(dem), float(rt), float(dt), float(st), partner))

        if first_param is None:
            raise ValueError('Missing parameter section.')

        def val_after_colon(s):
            return float(s.split(':', 1)[1].strip())

        CapE = val_after_colon(first_param)
        CapL = val_after_colon(next(f).strip())
        cons = val_after_colon(next(f).strip())
        rech = val_after_colon(next(f).strip())
        speed = val_after_colon(next(f).strip())

        rest = [ln.strip() for ln in f if ln.strip()]

    belongs = {}
    if rest and rest[0].lower() == 'belongs':
        for ln in rest[1:]:
            p = ln.split()
            if not p:
                continue
            if p[0].lower().startswith('numowner') or p[0].lower() == 'end':
                break
            if p[0].startswith('C') and len(p) >= 2:
                try:
                    belongs[p[0]] = int(p[1])
                except:
                    pass

    # Build flat arrays
    # Indexing: 0 = depot, then customers (C*), then stations (S*)
    depot = None
    customers = []
    stations = []
    for sid, typ, x, y, dem, rt, dt, st, partner in rows:
        kind = sid[0]
        if kind == 'D':
            depot = (sid, kind, typ, x, y, dem, rt, dt, st, partner)
        elif kind == 'C':
            customers.append((sid, kind, typ, x, y, dem, rt, dt, st, partner))
        elif kind == 'S':
            stations.append((sid, kind, typ, x, y, dem, rt, dt, st, partner))

    if depot is None:
        raise ValueError('Depot D0 not found')

    nodes = [depot] + customers + stations

    sid_to_i = {nodes[i][0]: i for i in range(len(nodes))}

    P = set(i for i in range(len(nodes)) if nodes[i][1] == 'C' and nodes[i][2] == 'cp')
    D = set(i for i in range(len(nodes)) if nodes[i][1] == 'C' and nodes[i][2] == 'cd')
    S = set(i for i in range(len(nodes)) if nodes[i][1] == 'S' or nodes[i][2] == 'f')

    # partner maps on StringIDs
    p2d = {}
    d2p = {}
    for sid, kind, typ, *_ , partner in nodes:
        if kind != 'C':
            continue
        if partner != '0':
            if typ == 'cp':
                p2d[sid] = partner
            elif typ == 'cd':
                d2p[sid] = partner

    # allow d2p even if only given via cp
    for p_sid, d_sid in p2d.items():
        d2p.setdefault(d_sid, p_sid)

    xs = [nodes[i][3] for i in range(len(nodes))]
    ys = [nodes[i][4] for i in range(len(nodes))]

    def dist(i, j):
        return math.hypot(xs[i] - xs[j], ys[i] - ys[j])

    def traveltime(i, j):
        return speed * dist(i, j)

    def energy(i, j):
        return cons * dist(i, j)

    horizon = nodes[0][7]  # depot due time

    data = {
        'nodes': nodes,
        'sid_to_i': sid_to_i,
        'P': P,
        'D': D,
        'S': S,
        'p2d': p2d,
        'd2p': d2p,
        'CapE': CapE,
        'CapL': CapL,
        'cons': cons,
        'rech': rech,
        'speed': speed,
        'horizon': horizon,
        'dist': dist,
        'traveltime': traveltime,
        'energy': energy,
    }

    return data


# step update (analogous to Algo 1: Appendix B of Rist/Forbes), extend existing path to node j
# Original paper only tracked time, load and distance
# had to include a bunch of extra state information including energy and load capacity
# returns (new_state, None) if feasible, else (None, reason).

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
    if kind_j == 'S' and sid_j in seenS:
        return None, 'revisit_station'

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
    if kind_j == 'S' or typ_j == 'f':
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

    # add load
    if typ_j == 'cp':
        onboard2.add(sid_j)
        seenP2.add(sid_j)
    elif typ_j == 'cd':
        # must correspond to an onboard pickup
        p_sid = d2p.get(sid_j)
        if p_sid not in onboard2:
            return None, 'delivery_not_onboard'
        onboard2.remove(p_sid)
        seenD2.add(sid_j)
        deliv2 += 1
        if phase2 == 0:
            phase2 = 1  # switch to DELIVERY

    # if pickup, check current load and make sure += new load <CapL
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


# base paths as per step 1 (copilot helped me turn this into a nice function that reports the pruned paths)

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

    # set for array of base paths
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

            # try a charging station move
            for s in S:
                newst, reason = step(data, st, s)
                if newst is None:
                    prune(reason)
                else:
                    NewWork.add(newst)

        Work = NewWork
        print('depth', depth, 'work', len(Work), 'base', len(Base))
        if not Work:
            break

    return list(Base), pruned


# trim from a base path to a restricted fragment (i.e. 1 pick-up -> delivery switch)

def trim_fragments(data, base_path):

    nodes = data['nodes']

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
    Pseq = [nodes[i][0] for i in pickup_part if nodes[i][2] == 'cp']
    Dseq = [nodes[i][0] for i in delivery_part if nodes[i][2] == 'cd']

    # validate that both pickup/delivery occurs
    if not Pseq or not Dseq or len(Pseq) != len(Dseq):
        return []

    return 0





