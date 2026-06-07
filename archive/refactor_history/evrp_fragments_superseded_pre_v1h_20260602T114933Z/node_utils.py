"""Node classification helpers."""
from __future__ import annotations
from typing import Any

def _node(data: dict[str, Any], sid: str) -> tuple | None:
    idx = data['sid_to_i'].get(sid)
    return None if idx is None else data['nodes'][idx]

def is_station(data: dict[str, Any], sid: str) -> bool:
    node = _node(data, sid)
    return False if node is None else node[1] == 'S' or node[2] == 'f'

def is_customer(data: dict[str, Any], sid: str) -> bool:
    node = _node(data, sid)
    return False if node is None else node[1] == 'C'

def is_pickup(data: dict[str, Any], sid: str) -> bool:
    node = _node(data, sid)
    return False if node is None else node[2] == 'cp'

def is_delivery(data: dict[str, Any], sid: str) -> bool:
    node = _node(data, sid)
    return False if node is None else node[2] == 'cd'

def strip_stations(data: dict[str, Any], seq: tuple[str, ...] | list[str]) -> list[str]:
    return [sid for sid in seq if not is_station(data, sid)]
