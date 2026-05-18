"""
network.py
----------
Builds the state-expanded network used as the MILP backbone.

Network structure
-----------------
Nodes in the MILP network are (location, onboard_cargo) pairs — a vehicle at
a given physical location carrying a specific set of pickups is a distinct
network state.  Two routes that are at the same location but carrying different
cargo cannot be merged, so they occupy different network nodes.

Arcs in the network correspond to extended fragments (EFs).  Each EF connects
a tail network-node (its start location/onboard state) to a head network-node
(its end location/onboard state).

Depot arcs are separate: they represent a vehicle leaving the depot (empty)
and travelling directly to its first pickup.  One depot arc exists per pickup
node.

The output of build_milp_network() is a NetworkArc list and a node-state
index dict; these are consumed directly by master.py.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, FrozenSet, List, Tuple

from instance import Instance
from fragment import Fragment, attach_fragment_metrics, _compute_timing_metrics, _sequence_distance


# ---------------------------------------------------------------------------
# NetworkArc
# ---------------------------------------------------------------------------

@dataclass
class NetworkArc:
    """
    A single arc in the state-expanded MILP network.

    Fields
    ------
    arc_id          : unique integer index (position in the arc list)
    sequence        : ordered tuple of node_ids traversed by this arc
    start_node_id   : node_id at the tail of the arc (first element of sequence)
    end_node_id     : node_id at the head of the arc (last element of sequence)
    start_onboard   : cargo set when the arc begins
    end_onboard     : cargo set when the arc ends
    tail_index      : integer index of the tail network-node
    head_index      : integer index of the head network-node
    duration        : Tf — elapsed time from service-start at tail to service-start at head
    earliest_end    : Ef — earliest feasible service-start time at head
    latest_start    : Lf — latest permissible service-start time at tail
    distance        : Df — total arc distance
    min_start_energy: minimum battery level required at the tail
    """

    arc_id:           int
    sequence:         Tuple[str, ...]
    start_node_id:    str
    end_node_id:      str
    start_onboard:    FrozenSet[str]
    end_onboard:      FrozenSet[str]
    tail_index:       int
    head_index:       int
    duration:         float
    earliest_end:     float
    latest_start:     float
    distance:         float
    min_start_energy: float


# ---------------------------------------------------------------------------
# State-node registry
# ---------------------------------------------------------------------------

# A network node is identified by (location node_id, onboard cargo frozenset)
NetworkNodeState = Tuple[str, FrozenSet[str]]


class NodeRegistry:
    """
    Assigns a unique integer index to each (location, onboard) state
    encountered while building the network.  Indices are assigned in
    insertion order.
    """

    def __init__(self) -> None:
        self._state_to_index: Dict[NetworkNodeState, int] = {}

    def get_or_create(self, location_id: str, onboard: FrozenSet[str]) -> int:
        state = (location_id, onboard)
        if state not in self._state_to_index:
            self._state_to_index[state] = len(self._state_to_index)
        return self._state_to_index[state]

    def ensure(self, location_id: str, onboard: FrozenSet[str]) -> int:
        """Like get_or_create but makes intent explicit when we must guarantee existence."""
        return self.get_or_create(location_id, onboard)

    @property
    def state_to_index(self) -> Dict[NetworkNodeState, int]:
        return dict(self._state_to_index)

    @property
    def size(self) -> int:
        return len(self._state_to_index)


# ---------------------------------------------------------------------------
# Depot arcs
# ---------------------------------------------------------------------------

def build_depot_arcs(inst: Instance, registry: NodeRegistry) -> List[NetworkArc]:
    """
    Build one depot arc per pickup node.

    Each depot arc represents a vehicle departing the depot empty and arriving
    at the pickup ready to begin service.  The vehicle picks up cargo on
    arrival, so end_onboard = {pickup_id}.

    Depot arcs are not restricted or extended fragments — they exist outside
    the RF/EF machinery and are added separately before MILP construction.
    """
    depot_id    = inst.depot_id
    depot_empty = frozenset()
    arcs: List[NetworkArc] = []

    for arc_id, p_idx in enumerate(sorted(inst.pickup_indices)):
        p_id   = inst.nodes[p_idx].node_id
        seq    = (depot_id, p_id)

        Tf, Ef, Lf = _compute_timing_metrics(inst, seq)
        Df         = _sequence_distance(inst, seq)

        tail = registry.get_or_create(depot_id, depot_empty)
        head = registry.get_or_create(p_id,     frozenset({p_id}))

        arcs.append(NetworkArc(
            arc_id           = arc_id,
            sequence         = seq,
            start_node_id    = depot_id,
            end_node_id      = p_id,
            start_onboard    = depot_empty,
            end_onboard      = frozenset({p_id}),
            tail_index       = tail,
            head_index       = head,
            duration         = Tf,
            earliest_end     = Ef,
            latest_start     = Lf,
            distance         = Df,
            min_start_energy = 0.0,
        ))

    return arcs


# ---------------------------------------------------------------------------
# Main network builder
# ---------------------------------------------------------------------------

def build_milp_network(
    inst:               Instance,
    extended_fragments: List[Fragment],
) -> Tuple[List[NetworkArc], Dict[NetworkNodeState, int], int]:
    """
    Construct the full state-expanded MILP network.

    Parameters
    ----------
    inst               : problem instance
    extended_fragments : undominated EFs from fragment.build_extended_fragments()

    Returns
    -------
    arcs        : list of NetworkArc objects (depot arcs first, then EF arcs)
    node_states : dict mapping (location_id, onboard) -> integer node index
    depot_node_index : integer index of the depot network node (location=depot, onboard=empty)

    Notes
    -----
    - Arc IDs are contiguous integers starting from 0.
    - The depot node (depot_id, frozenset()) is always guaranteed to exist in
      node_states, even if no arcs happen to create it (which should not occur
      in practice since depot arcs always exist).
    """
    registry = NodeRegistry()

    # --- depot arcs ---
    depot_arcs = build_depot_arcs(inst, registry)

    # Re-index depot arcs starting from 0 (build_depot_arcs uses local indices)
    for i, arc in enumerate(depot_arcs):
        arc.arc_id = i
    offset = len(depot_arcs)

    # --- EF arcs ---
    ef_arcs: List[NetworkArc] = []
    for i, frag in enumerate(extended_fragments):
        tail = registry.get_or_create(frag.start_node,  frag.start_onboard)
        head = registry.get_or_create(frag.end_node,    frag.end_onboard)

        ef_arcs.append(NetworkArc(
            arc_id           = offset + i,
            sequence         = frag.sequence,
            start_node_id    = frag.start_node,
            end_node_id      = frag.end_node,
            start_onboard    = frag.start_onboard,
            end_onboard      = frag.end_onboard,
            tail_index       = tail,
            head_index       = head,
            duration         = frag.duration,
            earliest_end     = frag.earliest_end,
            latest_start     = frag.latest_start,
            distance         = frag.distance,
            min_start_energy = frag.min_start_energy,
        ))

    all_arcs = depot_arcs + ef_arcs

    # Guarantee depot node exists (it always should, but be explicit)
    depot_node_index = registry.ensure(inst.depot_id, frozenset())

    return all_arcs, registry.state_to_index, depot_node_index


# ---------------------------------------------------------------------------
# Arc adjacency helpers  (used by master.py and the callback)
# ---------------------------------------------------------------------------

def build_adjacency(
    arcs:       List[NetworkArc],
    node_count: int,
) -> Tuple[Dict[int, List[int]], Dict[int, List[int]]]:
    """
    Build forward and backward adjacency lists over network node indices.

    Returns
    -------
    out_arcs : node_index -> list of arc_ids leaving that node
    in_arcs  : node_index -> list of arc_ids entering that node
    """
    out_arcs: Dict[int, List[int]] = {n: [] for n in range(node_count)}
    in_arcs:  Dict[int, List[int]] = {n: [] for n in range(node_count)}

    for arc in arcs:
        out_arcs[arc.tail_index].append(arc.arc_id)
        in_arcs[arc.head_index].append(arc.arc_id)

    return out_arcs, in_arcs


def arcs_ending_at_pickup(
    inst: Instance,
    arcs: List[NetworkArc],
) -> Dict[str, List[int]]:
    """
    Map each pickup node_id to the list of arc_ids whose end_node_id is that
    pickup.  Used to build the MILP coverage constraints.
    """
    coverage: Dict[str, List[int]] = {p_id: [] for p_id in inst.pickup_ids}
    for arc in arcs:
        if arc.end_node_id in coverage:
            coverage[arc.end_node_id].append(arc.arc_id)
    return coverage
