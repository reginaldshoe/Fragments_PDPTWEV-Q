"""
state.py
--------
Defines RouteState, the immutable snapshot of a vehicle's situation at a point
in its route, and step(), which advances that state to the next node.

This is the core feasibility engine for the enumeration phase.  It corresponds
to Algorithm 1 (Appendix B) of Rist & Forbes, extended here to track energy
and load capacity in addition to the original time / distance / load.

Design notes
------------
- RouteState is a frozen dataclass so it can be placed in sets and used as a
  dict key during BFS/DFS enumeration without copying.
- All node references inside the state use integer indices (into Instance.nodes)
  for speed; only the path stores indices. Node-id strings are used at the
  fragment / MILP layer above this one.
- step() returns (new_state, None) on success or (None, reason_string) on
  infeasibility, preserving the original API so callers can record pruning
  statistics without extra branching.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet, Optional, Tuple

from instance import Instance


# ---------------------------------------------------------------------------
# Phase constants  (pickup phase -> delivery phase)
# ---------------------------------------------------------------------------

PHASE_PICKUP   = 0   # vehicle has not yet made its first delivery on this route
PHASE_DELIVERY = 1   # vehicle has made at least one delivery


# ---------------------------------------------------------------------------
# RouteState
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class RouteState:
    """
    Immutable snapshot of a vehicle's state at the moment it *departs* the
    last node in its current path.

    Attributes
    ----------
    path : tuple[int, ...]
        Sequence of node indices visited so far (integer indices into
        Instance.nodes).  The last element is the node the vehicle just left.

    phase : int
        PHASE_PICKUP (0) or PHASE_DELIVERY (1).  Transitions to PHASE_DELIVERY
        on the first delivery visit and never reverts.

    onboard_pickups : frozenset[str]
        node_ids of pickups whose cargo is currently on the vehicle (i.e.
        picked up but not yet delivered).

    energy : float
        Remaining battery charge at the moment of departure from the last node.

    departure_time : float
        Time at which the vehicle departs the last node (after service).

    visited_pickups : frozenset[str]
        node_ids of all pickup nodes visited so far (superset of onboard_pickups
        once deliveries begin).

    visited_deliveries : frozenset[str]
        node_ids of all delivery nodes visited so far.

    visited_stations : frozenset[str]
        node_ids of all charging stations visited so far.  Used to prevent
        revisiting the same station (which cannot improve the route).

    delivery_count : int
        Number of deliveries completed.  A base path is complete when this > 0
        and onboard_pickups is empty.

    total_distance : float
        Cumulative route distance so far.
    """

    path:               Tuple[int, ...]
    phase:              int
    onboard_pickups:    FrozenSet[str]
    energy:             float
    departure_time:     float
    visited_pickups:    FrozenSet[str]
    visited_deliveries: FrozenSet[str]
    visited_stations:   FrozenSet[str]
    delivery_count:     int
    total_distance:     float

    # ------------------------------------------------------------------
    # Convenience properties
    # ------------------------------------------------------------------

    @property
    def current_node_index(self) -> int:
        """Index of the last (current) node in the path."""
        return self.path[-1]

    @property
    def is_base_path_complete(self) -> bool:
        """
        True when the vehicle has made at least one delivery and is now empty.
        This is the termination condition for base-path enumeration.
        """
        return (self.phase == PHASE_DELIVERY and
                len(self.onboard_pickups) == 0 and
                self.delivery_count > 0)


# ---------------------------------------------------------------------------
# Factory: initial state for a vehicle leaving the depot to its first pickup
# ---------------------------------------------------------------------------

def initial_state(inst: Instance, pickup_index: int) -> Optional[RouteState]:
    """
    Build the RouteState for a vehicle that departs the depot (index 0) and
    travels directly to *pickup_index*.

    Returns None if the depot -> pickup leg is infeasible (energy or time).
    """
    depot_index = 0
    pickup_node = inst.nodes[pickup_index]

    # --- energy feasibility ---
    energy_needed = inst.energy_consumed(depot_index, pickup_index)
    if energy_needed > inst.battery_capacity + 1e-9:
        return None

    energy_on_arrival = inst.battery_capacity - energy_needed

    # --- time feasibility ---
    arrival_time = inst.travel_time(depot_index, pickup_index)
    service_start = max(pickup_node.ready_time, arrival_time)
    if service_start > pickup_node.due_time + 1e-9:
        return None

    departure_time = service_start + pickup_node.service_time
    distance       = inst.distance(depot_index, pickup_index)

    return RouteState(
        path               = (pickup_index,),
        phase              = PHASE_PICKUP,
        onboard_pickups    = frozenset([pickup_node.node_id]),
        energy             = energy_on_arrival,
        departure_time     = departure_time,
        visited_pickups    = frozenset([pickup_node.node_id]),
        visited_deliveries = frozenset(),
        visited_stations   = frozenset(),
        delivery_count     = 0,
        total_distance     = distance,
    )


# ---------------------------------------------------------------------------
# step()
# ---------------------------------------------------------------------------

InfeasibilityReason = str   # type alias for clarity

def step(
    inst:  Instance,
    state: RouteState,
    next_node_index: int,
) -> Tuple[Optional[RouteState], Optional[InfeasibilityReason]]:
    """
    Attempt to extend *state* by travelling to *next_node_index*.

    Returns
    -------
    (new_state, None)
        if the move is feasible; new_state is the updated RouteState after
        the vehicle departs next_node_index.

    (None, reason)
        if the move is infeasible; reason is a short string describing the
        violated constraint.  Callers can aggregate these for pruning stats.

    Feasibility checks (in order)
    ------------------------------
    1. Station revisit guard: never visit the same station twice.
    2. Time window: arrival (after waiting) must not exceed due_time.
    3. Energy: must have enough battery for the leg.
    4. Delivery pairing: a delivery can only be made if its pickup is onboard.
    5. Load capacity: total onboard demand must not exceed load_capacity.
    """
    current_index = state.current_node_index
    next_node     = inst.nodes[next_node_index]
    next_id       = next_node.node_id

    # ------------------------------------------------------------------
    # 1. Station revisit guard
    # ------------------------------------------------------------------
    if next_node.is_station and next_id in state.visited_stations:
        return None, 'revisit_station'

    # ------------------------------------------------------------------
    # 2. Time feasibility
    # ------------------------------------------------------------------
    arrival_time  = state.departure_time + inst.travel_time(current_index, next_node_index)
    service_start = max(next_node.ready_time, arrival_time)
    if service_start > next_node.due_time + 1e-9:
        return None, 'time_window'

    # ------------------------------------------------------------------
    # 3. Energy feasibility
    # ------------------------------------------------------------------
    energy_needed  = inst.energy_consumed(current_index, next_node_index)
    energy_on_arrival = state.energy - energy_needed
    if energy_on_arrival < -1e-9:
        return None, 'energy'

    distance = inst.distance(current_index, next_node_index)

    # ------------------------------------------------------------------
    # 4a. Charging station: recharge to full, no cargo change
    # ------------------------------------------------------------------
    if next_node.is_station:
        charge_time    = inst.charge_time_from(energy_on_arrival)
        departure_time = service_start + charge_time

        return RouteState(
            path               = state.path + (next_node_index,),
            phase              = state.phase,
            onboard_pickups    = state.onboard_pickups,
            energy             = inst.battery_capacity,
            departure_time     = departure_time,
            visited_pickups    = state.visited_pickups,
            visited_deliveries = state.visited_deliveries,
            visited_stations   = state.visited_stations | {next_id},
            delivery_count     = state.delivery_count,
            total_distance     = state.total_distance + distance,
        ), None

    # ------------------------------------------------------------------
    # 4b. Customer node: update cargo and phase
    # ------------------------------------------------------------------
    onboard    = set(state.onboard_pickups)
    visited_p  = set(state.visited_pickups)
    visited_d  = set(state.visited_deliveries)
    phase      = state.phase
    deliveries = state.delivery_count

    if next_node.is_pickup:
        onboard.add(next_id)
        visited_p.add(next_id)

    elif next_node.is_delivery:
        paired_pickup_id = inst.delivery_to_pickup.get(next_id)
        if paired_pickup_id not in onboard:
            return None, 'delivery_not_onboard'
        onboard.discard(paired_pickup_id)
        visited_d.add(next_id)
        deliveries += 1
        if phase == PHASE_PICKUP:
            phase = PHASE_DELIVERY

    # ------------------------------------------------------------------
    # 5. Load capacity check (after updating onboard set)
    # ------------------------------------------------------------------
    current_load = sum(inst.nodes[inst.node_index[p_id]].demand for p_id in onboard)
    if current_load > inst.load_capacity + 1e-9:
        return None, 'capacity'

    departure_time = service_start + next_node.service_time

    return RouteState(
        path               = state.path + (next_node_index,),
        phase              = phase,
        onboard_pickups    = frozenset(onboard),
        energy             = energy_on_arrival,
        departure_time     = departure_time,
        visited_pickups    = frozenset(visited_p),
        visited_deliveries = frozenset(visited_d),
        visited_stations   = state.visited_stations,
        delivery_count     = deliveries,
        total_distance     = state.total_distance + distance,
    ), None
