"""Instance parsing and core data dictionary construction."""
from __future__ import annotations
import math


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
