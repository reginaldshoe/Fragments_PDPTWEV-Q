"""Node classification helpers for string node identifiers."""
from __future__ import annotations


def is_station(data, sid):
    nodes = data['nodes']
    i = data['sid_to_i'].get(sid)
    if i is None:
        return False
    return (nodes[i][1] == 'S') or (nodes[i][2] == 'f')

def is_customer(data, sid):
    nodes = data['nodes']
    i = data['sid_to_i'].get(sid)
    if i is None:
        return False
    return nodes[i][1] == 'C'

def is_pickup(data, sid):
    i = data['sid_to_i'].get(sid)
    if i is None:
        return False
    return data['nodes'][i][2] == 'cp'

def is_delivery(data, sid):
    i = data['sid_to_i'].get(sid)
    if i is None:
        return False
    return data['nodes'][i][2] == 'cd'

def strip_stations(seq, data):
    return [sid for sid in seq if not is_station(data, sid)]
