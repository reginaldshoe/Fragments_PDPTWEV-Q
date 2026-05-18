"""
instance.py
-----------
Defines the Node and Instance dataclasses and the read_instance() parser.

Node layout mirrors the raw instance file:
    Depot   : node_id starts with 'D', node_type == 'depot'
    Pickup  : node_id starts with 'C', node_type == 'pickup'
    Delivery: node_id starts with 'C', node_type == 'delivery'
    Station : node_id starts with 'S', node_type == 'station'

All distance / travel-time / energy calculations are methods on Instance so
the rest of the codebase never touches raw coordinates directly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, FrozenSet, Set, Tuple


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Node:
    """A single network node (depot, pickup, delivery, or charging station)."""

    node_id:      str    # e.g. 'D0', 'C1', 'S2'
    node_type:    str    # 'depot' | 'pickup' | 'delivery' | 'station'
    x:            float
    y:            float
    demand:       float  # positive for pickups, negative for deliveries, 0 otherwise
    ready_time:   float  # earliest service-start time
    due_time:     float  # latest service-start time
    service_time: float  # time spent at node once service begins
    partner_id:   str    # paired pickup/delivery node_id; '0' if none

    # ------------------------------------------------------------------
    # Convenience predicates
    # ------------------------------------------------------------------

    @property
    def is_depot(self) -> bool:
        return self.node_type == 'depot'

    @property
    def is_pickup(self) -> bool:
        return self.node_type == 'pickup'

    @property
    def is_delivery(self) -> bool:
        return self.node_type == 'delivery'

    @property
    def is_station(self) -> bool:
        return self.node_type == 'station'

    @property
    def is_customer(self) -> bool:
        return self.node_type in ('pickup', 'delivery')


# ---------------------------------------------------------------------------
# Instance
# ---------------------------------------------------------------------------

@dataclass
class Instance:
    """
    Complete problem instance.

    Nodes are stored in a flat list with a guaranteed ordering:
        index 0         : depot
        indices 1..n_c  : customers (pickups then deliveries, as read)
        indices n_c+1.. : charging stations

    Integer indices into this list are used throughout for fast
    distance / time / energy lookups.
    """

    # --- node data ---
    nodes:      list[Node]           # ordered node list; index 0 is always the depot
    node_index: Dict[str, int]       # node_id -> position in nodes[]

    # --- node-type index sets (integer indices into nodes[]) ---
    pickup_indices:   FrozenSet[int]
    delivery_indices: FrozenSet[int]
    station_indices:  FrozenSet[int]

    # --- request pairing (node_id strings) ---
    pickup_to_delivery: Dict[str, str]   # pickup node_id  -> delivery node_id
    delivery_to_pickup: Dict[str, str]   # delivery node_id -> pickup node_id

    # --- vehicle parameters ---
    battery_capacity: float   # CapE : maximum battery energy (kWh or equivalent)
    load_capacity:    float   # CapL : maximum cargo load
    energy_rate:      float   # cons : energy consumed per unit distance
    charge_rate:      float   # rech : time to recharge one unit of energy  (1/power)
    speed:            float   # vehicle speed (distance per time unit)

    # --- planning horizon ---
    horizon: float            # depot due_time; no vehicle may return after this

    # ------------------------------------------------------------------
    # Geometry helpers  (all take integer node indices)
    # ------------------------------------------------------------------

    def distance(self, i: int, j: int) -> float:
        """Euclidean distance between nodes i and j."""
        ni, nj = self.nodes[i], self.nodes[j]
        return math.hypot(ni.x - nj.x, ni.y - nj.y)

    def travel_time(self, i: int, j: int) -> float:
        """Travel time from node i to node j (distance / speed)."""
        return self.distance(i, j) / self.speed

    def energy_consumed(self, i: int, j: int) -> float:
        """Energy consumed travelling from node i to node j."""
        return self.energy_rate * self.distance(i, j)

    def full_charge_time(self) -> float:
        """Time to charge from empty to full battery."""
        return self.battery_capacity * self.charge_rate

    def charge_time_from(self, current_energy: float) -> float:
        """Time to charge from current_energy to full battery."""
        return (self.battery_capacity - current_energy) * self.charge_rate

    # ------------------------------------------------------------------
    # Reachability helpers (by node_id strings, for use in fragment code)
    # ------------------------------------------------------------------

    def reachable_on_full_battery(self, from_id: str, to_id: str) -> bool:
        """True if the arc from_id -> to_id is feasible starting with a full battery."""
        i = self.node_index[from_id]
        j = self.node_index[to_id]
        return self.energy_consumed(i, j) <= self.battery_capacity + 1e-9

    def best_station_between(self, from_id: str, to_id: str) -> str | None:
        """
        Find the charging station that minimises travel-time detour on the leg
        from_id -> station -> to_id, subject to both sub-legs being reachable on
        a full battery.  Returns the station node_id, or None if no such station
        exists.
        """
        a = self.node_index[from_id]
        b = self.node_index[to_id]

        best_id    = None
        best_score = None

        for s in self.station_indices:
            e1 = self.energy_consumed(a, s)
            e2 = self.energy_consumed(s, b)
            if e1 <= self.battery_capacity + 1e-9 and e2 <= self.battery_capacity + 1e-9:
                score = self.travel_time(a, s) + self.travel_time(s, b)
                if best_score is None or score < best_score:
                    best_score = score
                    best_id    = self.nodes[s].node_id

        return best_id

    def minimum_start_energy(self, sequence: Tuple[str, ...]) -> float:
        """
        Minimum energy required at the start of sequence to reach the first
        charging station (or the end of the sequence if there is none).

        This is used as a lower-bound on the energy a vehicle must carry when
        beginning a fragment.
        """
        total = 0.0
        for u_id, v_id in zip(sequence, sequence[1:]):
            u = self.node_index[u_id]
            v = self.node_index[v_id]
            total += self.energy_consumed(u, v)
            if self.nodes[v].is_station:
                break
        return total

    def earliest_delivery_feasible(self, pickup_id: str) -> bool:
        """
        Quick check: can the paired delivery be reached in time if we depart
        the pickup at its earliest ready time, travelling direct (no station)?
        If this fails, no routing via this pickup can ever be time-feasible.
        """
        delivery_id = self.pickup_to_delivery.get(pickup_id)
        if delivery_id is None:
            return False

        p = self.node_index[pickup_id]
        d = self.node_index[delivery_id]
        np_ = self.nodes[p]
        nd  = self.nodes[d]

        earliest_depart_pickup = np_.ready_time
        arrival_at_delivery    = earliest_depart_pickup + np_.service_time + self.travel_time(p, d)

        return (earliest_depart_pickup <= np_.due_time + 1e-9 and
                arrival_at_delivery    <= nd.due_time  + 1e-9)

    def max_distance(self) -> float:
        """Maximum pairwise distance across all nodes. Used as big-M in MILP."""
        n = len(self.nodes)
        return max(
            self.distance(i, j)
            for i in range(n)
            for j in range(n)
            if i != j
        )

    # ------------------------------------------------------------------
    # Convenience lookups
    # ------------------------------------------------------------------

    def node(self, node_id: str) -> Node:
        return self.nodes[self.node_index[node_id]]

    def idx(self, node_id: str) -> int:
        return self.node_index[node_id]

    @property
    def depot(self) -> Node:
        return self.nodes[0]

    @property
    def depot_id(self) -> str:
        return self.nodes[0].node_id

    @property
    def pickup_ids(self) -> list[str]:
        return [self.nodes[i].node_id for i in self.pickup_indices]

    @property
    def delivery_ids(self) -> list[str]:
        return [self.nodes[i].node_id for i in self.delivery_indices]

    @property
    def station_ids(self) -> list[str]:
        return [self.nodes[i].node_id for i in self.station_indices]


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------

def _parse_node_type(node_id: str, raw_type: str) -> str:
    """
    Map the raw file type codes to human-readable node_type strings.

    The instance format uses:
        First character of node_id: D = depot, C = customer, S = station
        raw_type field for customers: 'cp' = pickup, 'cd' = delivery
        raw_type field for stations:  'f'  = (fast?) charging station
    """
    prefix = node_id[0]
    if prefix == 'D':
        return 'depot'
    if prefix == 'C':
        if raw_type == 'cp':
            return 'pickup'
        if raw_type == 'cd':
            return 'delivery'
        raise ValueError(f"Unknown customer type '{raw_type}' for node '{node_id}'")
    if prefix == 'S' or raw_type == 'f':
        return 'station'
    raise ValueError(f"Cannot determine node type for id='{node_id}', raw_type='{raw_type}'")


def read_instance(path: Path) -> Instance:
    """
    Parse a PDPTWEV instance file and return a fully constructed Instance.

    File format (whitespace-delimited):
        Header line (skipped)
        Node rows: node_id  raw_type  x  y  demand  ready  due  service  partner
        (blank lines ignored)
        Parameter block (colon-separated key: value pairs):
            battery_capacity: <float>
            load_capacity:    <float>
            energy_rate:      <float>
            charge_rate:      <float>
            speed:            <float>
        Optional 'BELONGS' section (ownership data, currently stored but unused)
    """
    with open(path, 'r') as f:
        f.readline()  # skip header

        raw_rows = []
        first_param_line = None

        for line in f:
            stripped = line.strip()
            if not stripped:
                continue
            if ':' in stripped:
                first_param_line = stripped
                break
            parts = stripped.split()
            if len(parts) < 9:
                continue
            node_id, raw_type, x, y, demand, ready, due, service, partner = parts[:9]
            raw_rows.append((
                node_id, raw_type,
                float(x), float(y), float(demand),
                float(ready), float(due), float(service),
                partner,
            ))

        if first_param_line is None:
            raise ValueError(f"Parameter section not found in '{path}'")

        def _float_after_colon(s: str) -> float:
            return float(s.split(':', 1)[1].strip())

        battery_capacity = _float_after_colon(first_param_line)
        load_capacity    = _float_after_colon(next(f).strip())
        energy_rate      = _float_after_colon(next(f).strip())
        charge_rate      = _float_after_colon(next(f).strip())
        speed            = _float_after_colon(next(f).strip())

        remaining_lines = [ln.strip() for ln in f if ln.strip()]

    # --- optional BELONGS section (stored, not used in routing) ---
    belongs: Dict[str, int] = {}
    if remaining_lines and remaining_lines[0].lower() == 'belongs':
        for ln in remaining_lines[1:]:
            parts = ln.split()
            if not parts:
                continue
            if parts[0].lower().startswith('numowner') or parts[0].lower() == 'end':
                break
            if parts[0].startswith('C') and len(parts) >= 2:
                try:
                    belongs[parts[0]] = int(parts[1])
                except ValueError:
                    pass

    # --- build Node objects, enforce ordering: depot first, then customers, then stations ---
    depot_node = None
    customer_nodes: list[Node] = []
    station_nodes:  list[Node] = []

    for node_id, raw_type, x, y, demand, ready, due, service, partner in raw_rows:
        node_type = _parse_node_type(node_id, raw_type)
        node = Node(
            node_id      = node_id,
            node_type    = node_type,
            x            = x,
            y            = y,
            demand       = demand,
            ready_time   = ready,
            due_time     = due,
            service_time = service,
            partner_id   = partner,
        )
        if node_type == 'depot':
            depot_node = node
        elif node_type in ('pickup', 'delivery'):
            customer_nodes.append(node)
        else:
            station_nodes.append(node)

    if depot_node is None:
        raise ValueError(f"No depot node (starting with 'D') found in '{path}'")

    nodes = [depot_node] + customer_nodes + station_nodes
    node_index = {n.node_id: i for i, n in enumerate(nodes)}

    # --- index sets ---
    pickup_indices   = frozenset(i for i, n in enumerate(nodes) if n.is_pickup)
    delivery_indices = frozenset(i for i, n in enumerate(nodes) if n.is_delivery)
    station_indices  = frozenset(i for i, n in enumerate(nodes) if n.is_station)

    # --- request pairing ---
    # Build pickup->delivery from explicit partner fields on pickup nodes.
    # Then back-fill delivery->pickup from the same data (some files only
    # specify the partner on one side of the pair).
    pickup_to_delivery: Dict[str, str] = {}
    delivery_to_pickup: Dict[str, str] = {}

    for node in nodes:
        if node.is_pickup and node.partner_id != '0':
            pickup_to_delivery[node.node_id] = node.partner_id
        elif node.is_delivery and node.partner_id != '0':
            delivery_to_pickup[node.node_id] = node.partner_id

    for p_id, d_id in pickup_to_delivery.items():
        delivery_to_pickup.setdefault(d_id, p_id)

    return Instance(
        nodes              = nodes,
        node_index         = node_index,
        pickup_indices     = pickup_indices,
        delivery_indices   = delivery_indices,
        station_indices    = station_indices,
        pickup_to_delivery = pickup_to_delivery,
        delivery_to_pickup = delivery_to_pickup,
        battery_capacity   = battery_capacity,
        load_capacity      = load_capacity,
        energy_rate        = energy_rate,
        charge_rate        = charge_rate,
        speed              = speed,
        horizon            = depot_node.due_time,
    )
