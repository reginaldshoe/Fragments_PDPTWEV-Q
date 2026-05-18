"""
fragment.py
-----------
Fragment enumeration, extension, metadata computation, and dominance filtering.

Terminology (following Rist & Forbes)
--------------------------------------
Base path
    A complete vehicle route from the first pickup to the last delivery,
    ending with an empty vehicle (all cargo delivered).  Stations may appear
    anywhere.  Produced by enumerate_base_paths().

Restricted fragment  (RF)
    A contiguous sub-sequence of a base path that contains exactly one
    pickup-to-delivery phase transition.  Every RF starts at some pickup node
    and ends at some delivery node, with cargo still potentially onboard at
    both endpoints.  Produced by trim_base_path() / enumerate_restricted_fragments().

Extended fragment  (EF)
    An RF extended by one more pickup (and optionally an intermediate charging
    station), or extended to the depot if the vehicle ends empty.  EFs are
    the arcs of the MILP network.  Produced by extend_restricted_fragments().

Fragment dataclass
    Carries the node-id sequence plus all metadata needed for dominance
    comparison and MILP construction (Tf, Ef, Lf, Emin, Df, dom_key, …).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, FrozenSet, List, Optional, Tuple

from instance import Instance
from state import RouteState, initial_state, step, PHASE_DELIVERY

EPS = 1e-9   # numerical tolerance used throughout


# ---------------------------------------------------------------------------
# Fragment dataclass
# ---------------------------------------------------------------------------

@dataclass
class Fragment:
    """
    A restricted or extended fragment ready for use in the MILP.

    Sequence and cargo
    ------------------
    sequence        : ordered tuple of node_ids visited (including any stations)
    start_onboard   : pickups on the vehicle when it *begins* this fragment
    end_onboard     : pickups still on the vehicle when the fragment *ends*
    contains_charge : True if the sequence passes through a charging station

    Dominance metrics  (computed by attach_fragment_metrics)
    -----------------
    min_start_energy : minimum battery level required at the fragment start to
                       reach the first charging station (or the end if none).
                       Lower-bounds the energy a vehicle must carry on entry.
    duration         : Tf — elapsed time from start-of-service at seq[0] to
                       start-of-service at seq[-1], assuming full charges at
                       stations and pessimistic (worst-case) dwell times.
    earliest_end     : Ef — earliest possible start-of-service at seq[-1].
    latest_start     : Lf — latest permissible start-of-service at seq[0] that
                       still allows on-time arrival at seq[-1].
    distance         : Df — total arc distance of the sequence.

    MILP network keys
    -----------------
    start_node  : node_id of seq[0]
    end_node    : node_id of seq[-1]
    customer_locations : frozenset of customer node_ids visited, used in the
                         dominance key.  For EFs the final pickup is excluded
                         (it will appear in the next fragment).
    dominance_key : (start_node, end_node, start_onboard, end_onboard,
                     customer_locations) — fragments with the same key are
                     comparable for dominance.

    Extension metadata  (EFs only, None for RFs)
    ------------------
    extended_to       : node_id of the pickup (or depot) added by extension
    extension_station : node_id of intermediate charging station inserted, or None
    extension_delivery: node_id of the delivery paired with extended_to, or None
    """

    # core sequence
    sequence:         Tuple[str, ...]
    start_onboard:    FrozenSet[str]
    end_onboard:      FrozenSet[str]
    contains_charge:  bool

    # dominance metrics (None until attach_fragment_metrics() is called)
    min_start_energy: float = 0.0
    duration:         Optional[float] = None   # Tf
    earliest_end:     Optional[float] = None   # Ef
    latest_start:     Optional[float] = None   # Lf
    distance:         Optional[float] = None   # Df

    # MILP keys (None until attach_fragment_metrics() is called)
    start_node:          Optional[str]              = None
    end_node:            Optional[str]              = None
    customer_locations:  Optional[FrozenSet[str]]   = None
    dominance_key:       Optional[tuple]            = None

    # extension metadata
    extended_to:        Optional[str] = None
    extension_station:  Optional[str] = None
    extension_delivery: Optional[str] = None

    # ------------------------------------------------------------------
    # Convenience
    # ------------------------------------------------------------------

    @property
    def has_metrics(self) -> bool:
        return self.duration is not None

    def exact_signature(self) -> tuple:
        """Signature for exact-duplicate detection (before metrics)."""
        return (self.sequence, self.start_onboard, self.end_onboard)

    def metric_signature(self) -> tuple:
        """Signature for post-metrics duplicate detection."""
        return (self.sequence, self.earliest_end, self.latest_start,
                self.duration, self.min_start_energy)


# ---------------------------------------------------------------------------
# Metrics helpers
# ---------------------------------------------------------------------------

def _sequence_distance(inst: Instance, sequence: Tuple[str, ...]) -> float:
    """Total arc distance along a node-id sequence."""
    return sum(
        inst.distance(inst.idx(u), inst.idx(v))
        for u, v in zip(sequence, sequence[1:])
    )


def _sequence_customer_locations(
    inst:         Instance,
    sequence:     Tuple[str, ...],
    exclude_last: bool,
) -> FrozenSet[str]:
    """
    Return the frozenset of customer node_ids in *sequence*.

    If *exclude_last* is True and the final node is a customer, only that
    final occurrence is excluded — interior visits to the same node_id are
    retained.  This is used for extended fragments, where the final pickup
    belongs to the *next* fragment in a chain and should not contribute to
    this fragment's dominance key.

    Critically: we must not strip *all* occurrences of end_sid from the
    frozenset.  A node that appears both as an interior node and as the final
    node of a sequence is a legitimate interior visit.  Stripping all
    occurrences would conflate two fragments with different interior visits
    under the same dominance key, causing incorrect dominance pruning.

    Note: because the result is a frozenset (no multiplicity), a customer
    that appears interior-only vs interior+final produces the same frozenset
    either way.  The only case that matters is when the customer appears
    *only* as the final node — in that case exclude_last correctly removes it.
    """
    last = sequence[-1]
    last_is_customer = inst.node(last).is_customer

    # Collect customers from all positions except the final node
    interior_customers = frozenset(
        sid for sid in sequence[:-1]
        if inst.node(sid).is_customer
    )

    if not exclude_last or not last_is_customer:
        # Include the final node too
        return interior_customers | ({last} if last_is_customer else frozenset())

    # exclude_last=True and last is a customer:
    # Return only interior customers; the final pickup is excluded.
    return interior_customers


def _compute_timing_metrics(
    inst:     Instance,
    sequence: Tuple[str, ...],
) -> Tuple[float, float, float]:
    """
    Compute (Tf, Ef, Lf) for a node-id sequence.

    Tf  (duration)
        Total elapsed time from start-of-service at seq[0] to start-of-service
        at seq[-1], using pessimistic (full-charge) dwell times at stations.

    Ef  (earliest_end)
        Earliest possible start-of-service time at seq[-1], propagated forward
        from each node's ready_time.

    Lf  (latest_start)
        Latest permissible start-of-service time at seq[0] such that seq[-1]
        can still be reached within its due_time, propagated backward.

    Stations are treated pessimistically: dwell time = time to charge from
    empty to full.  This ensures dominance comparisons are conservative (a
    fragment that dominates another will do so regardless of actual charge
    level on entry).
    """
    full_charge_time = inst.full_charge_time()

    def process_time(node_id: str) -> float:
        """Time spent at node_id before departing (service or full charge)."""
        node = inst.node(node_id)
        return full_charge_time if node.is_station else node.service_time

    # --- Tf: forward sweep, ignoring time windows ---
    Tf = sum(
        process_time(u) + inst.travel_time(inst.idx(u), inst.idx(v))
        for u, v in zip(sequence, sequence[1:])
    )

    # --- Ef: forward sweep, respecting ready times ---
    t = inst.node(sequence[0]).ready_time
    for u, v in zip(sequence, sequence[1:]):
        t += process_time(u) + inst.travel_time(inst.idx(u), inst.idx(v))
        t = max(t, inst.node(v).ready_time)
    Ef = t

    # --- Lf: backward sweep from due_time at the last node ---
    t = inst.node(sequence[-1]).due_time
    for u, v in zip(reversed(sequence[:-1]), reversed(sequence[1:])):
        t -= inst.travel_time(inst.idx(u), inst.idx(v)) + process_time(u)
        t = min(t, inst.node(u).due_time)
    Lf = t

    return Tf, Ef, Lf


def attach_fragment_metrics(
    inst:         Instance,
    fragments:    List[Fragment],
    exclude_last: bool = False,
) -> List[Fragment]:
    """
    Return a new list of Fragment objects with all metrics populated.

    *exclude_last* should be True for extended fragments (the final pickup is
    excluded from customer_locations so the dominance key correctly reflects
    that this pickup belongs to the next fragment in the chain).

    Dominance key
    -------------
    The dominance key is:
        (start_node, end_node, start_onboard, end_onboard, customer_locations)

    customer_locations is the frozenset of all customer nodes visited
    *interior* to the fragment (i.e. not the final node when exclude_last=True,
    and not the start node since that is already captured by start_node).

    Critically, interior customer visits must be part of the dom_key so that
    only fragments with identical interior visits are compared for dominance.
    Two EFs that visit different interior customers can never substitute for
    one another in a route — allowing one to dominate the other would permit
    the solver to chain arcs that revisit the same customer.
    """
    result = []
    for f in fragments:
        Tf, Ef, Lf = _compute_timing_metrics(inst, f.sequence)
        Df          = _sequence_distance(inst, f.sequence)
        cust_locs   = _sequence_customer_locations(inst, f.sequence, exclude_last)
        dom_key     = (
            f.sequence[0],
            f.sequence[-1],
            f.start_onboard,
            f.end_onboard,
            cust_locs,          # includes ALL interior customer visits
        )
        result.append(Fragment(
            sequence          = f.sequence,
            start_onboard     = f.start_onboard,
            end_onboard       = f.end_onboard,
            contains_charge   = f.contains_charge,
            min_start_energy  = f.min_start_energy,
            duration          = Tf,
            earliest_end      = Ef,
            latest_start      = Lf,
            distance          = Df,
            start_node        = f.sequence[0],
            end_node          = f.sequence[-1],
            customer_locations = cust_locs,
            dominance_key     = dom_key,
            extended_to       = f.extended_to,
            extension_station = f.extension_station,
            extension_delivery= f.extension_delivery,
        ))
    return result


# ---------------------------------------------------------------------------
# Dominance
# ---------------------------------------------------------------------------

def _dominates(a: Fragment, b: Fragment) -> bool:
    """
    Return True if fragment *a* dominates fragment *b*.

    *a* dominates *b* when they share the same dominance_key and *a* is at
    least as good as *b* on all four metrics, and strictly better on at least
    one.  "Better" means:
        - smaller Ef  (can finish earlier)
        - larger  Lf  (can start later)
        - smaller Tf  (takes less time)
        - smaller Emin (needs less energy to start)
    """
    if a.dominance_key != b.dominance_key:
        return False

    better_or_equal = (
        a.earliest_end  <= b.earliest_end  + EPS and
        a.latest_start  >= b.latest_start  - EPS and
        a.duration      <= b.duration      + EPS and
        a.min_start_energy <= b.min_start_energy + EPS
    )
    strictly_better = (
        a.earliest_end   < b.earliest_end  - EPS or
        a.latest_start   > b.latest_start  + EPS or
        a.duration       < b.duration      - EPS or
        a.min_start_energy < b.min_start_energy - EPS
    )
    return better_or_equal and strictly_better


def _filter_dominated(group: List[Fragment]) -> List[Fragment]:
    """Remove dominated fragments from a group sharing the same dominance_key."""
    keep: List[Fragment] = []
    for candidate in group:
        if any(_dominates(kept, candidate) for kept in keep):
            continue
        keep = [k for k in keep if not _dominates(candidate, k)]
        keep.append(candidate)
    return keep


def dominance_filter(fragments: List[Fragment]) -> List[Fragment]:
    """
    Remove dominated fragments from *fragments*.

    Fragments are grouped by dominance_key first; dominance is only tested
    within groups (fragments with different keys are never comparable).
    """
    buckets: Dict[tuple, List[Fragment]] = {}
    for f in fragments:
        buckets.setdefault(f.dominance_key, []).append(f)
    result = []
    for group in buckets.values():
        result.extend(_filter_dominated(group))
    return result


# ---------------------------------------------------------------------------
# Deduplication
# ---------------------------------------------------------------------------

def dedup_exact(fragments: List[Fragment]) -> List[Fragment]:
    """
    Remove fragments with identical (sequence, start_onboard, end_onboard).

    Applied before metric computation to avoid wasted work on duplicates that
    arise from multiple base paths sharing the same sub-sequence.
    """
    seen = set()
    result = []
    for f in fragments:
        sig = f.exact_signature()
        if sig not in seen:
            seen.add(sig)
            result.append(f)
    return result


def dedup_by_metrics(fragments: List[Fragment]) -> List[Fragment]:
    """
    Remove fragments with identical (sequence, Ef, Lf, Tf, Emin).

    Applied after dominance filtering to clean up any remaining duplicates
    that share the same sequence and metric values but differ only in fields
    that are irrelevant to the MILP (e.g. extension metadata).
    """
    seen = set()
    result = []
    for f in fragments:
        sig = f.metric_signature()
        if sig not in seen:
            seen.add(sig)
            result.append(f)
    return result


# ---------------------------------------------------------------------------
# Base-path enumeration
# ---------------------------------------------------------------------------

def enumerate_base_paths(
    inst:   Instance,
    maxlen: int,
) -> Tuple[List[Tuple[int, ...]], Dict[str, int]]:
    """
    Enumerate all feasible base paths using breadth-first expansion.

    A base path is a sequence of node indices (not node_ids) starting at some
    pickup and ending when the vehicle becomes empty after at least one
    delivery.  The depot is implicit: every base path is assumed to start from
    depot index 0.

    Parameters
    ----------
    inst   : problem instance
    maxlen : maximum number of BFS expansion rounds (bounds path length)

    Returns
    -------
    base_paths : list of tuples of integer node indices
    pruning_stats : dict mapping infeasibility reason -> count of pruned states
    """
    pickup_indices  = list(inst.pickup_indices)
    station_indices = list(inst.station_indices)
    station_index_set = set(station_indices)

    pruning_stats: Dict[str, int] = {}

    def record_prune(reason: str) -> None:
        pruning_stats[reason] = pruning_stats.get(reason, 0) + 1

    # Seed one working state per reachable pickup from the depot
    active_states: set[RouteState] = set()
    for p_idx in pickup_indices:
        st = initial_state(inst, p_idx)
        if st is None:
            record_prune('seed_infeasible')
        else:
            active_states.add(st)

    completed_paths: set[Tuple[int, ...]] = set()

    for _depth in range(maxlen):
        if not active_states:
            break

        next_states: set[RouteState] = set()

        for state in active_states:

            # Base path complete: vehicle empty after at least one delivery
            if state.is_base_path_complete:
                completed_paths.add(state.path)
                continue

            # --- Try delivering any onboard cargo ---
            for pickup_id in state.onboard_pickups:
                delivery_id  = inst.pickup_to_delivery.get(pickup_id)
                delivery_idx = inst.node_index.get(delivery_id)
                if delivery_idx is None:
                    continue
                new_state, reason = step(inst, state, delivery_idx)
                if new_state is None:
                    record_prune(reason)
                else:
                    next_states.add(new_state)

            # --- Try picking up more cargo (pickup phase only) ---
            if state.phase == 0:   # PHASE_PICKUP
                for p_idx in pickup_indices:
                    p_id = inst.nodes[p_idx].node_id
                    if p_id in state.visited_pickups:
                        continue
                    new_state, reason = step(inst, state, p_idx)
                    if new_state is None:
                        record_prune(reason)
                    else:
                        next_states.add(new_state)

            # --- Try visiting a charging station ---
            # Never visit a station immediately after another station
            # (the vehicle would already be at full charge).
            last_is_station = state.current_node_index in station_index_set
            if last_is_station:
                record_prune('consecutive_station')
            else:
                for s_idx in station_indices:
                    new_state, reason = step(inst, state, s_idx)
                    if new_state is None:
                        record_prune(reason)
                    else:
                        next_states.add(new_state)

        active_states = next_states

    return list(completed_paths), pruning_stats


# ---------------------------------------------------------------------------
# Restricted fragment enumeration  (trim base paths)
# ---------------------------------------------------------------------------

def _trim_base_path(
    inst:      Instance,
    base_path: Tuple[int, ...],
) -> List[Fragment]:
    """
    Generate all restricted fragments from a single base path.

    A restricted fragment is a contiguous sub-sequence of the base path that:
    - starts at some pickup node (with zero or more earlier pickups already
      onboard, captured in start_onboard)
    - ends at some delivery node (with zero or more later deliveries still
      pending, captured in end_onboard)
    - contains exactly one pickup→delivery phase transition

    We enumerate all valid (start_pickup_index, end_delivery_index) pairs and
    slice the base path accordingly.
    """
    nodes = inst.nodes

    # --- Locate the first delivery in the path (marks the phase transition) ---
    phase_switch = next(
        (pos for pos, idx in enumerate(base_path) if nodes[idx].is_delivery),
        None,
    )
    if phase_switch is None:
        return []   # no delivery in path — shouldn't happen for valid base paths

    # Split the base path into its pickup and delivery halves
    pickup_part   = base_path[:phase_switch]
    delivery_part = base_path[phase_switch:]

    # Collect the ordered pickup and delivery node_ids from each half
    pickup_ids   = [nodes[i].node_id for i in pickup_part   if i in inst.pickup_indices]
    delivery_ids = [nodes[i].node_id for i in delivery_part if i in inst.delivery_indices]

    if not pickup_ids or not delivery_ids or len(pickup_ids) != len(delivery_ids):
        return []

    # Position map: node_id -> index in base_path (for slicing)
    position = {nodes[idx].node_id: pos for pos, idx in enumerate(base_path)}

    fragments: List[Fragment] = []

    for start_pickup_pos in range(len(pickup_ids)):
        # Pickups already onboard when this fragment starts
        start_onboard = frozenset(pickup_ids[: start_pickup_pos + 1])
        start_node_id = pickup_ids[start_pickup_pos]
        seq_start     = position[start_node_id]

        for end_delivery_pos in range(len(delivery_ids)):
            # Deliveries that occur before the fragment ends
            kept_deliveries    = delivery_ids[: len(delivery_ids) - end_delivery_pos]
            pending_deliveries = delivery_ids[len(delivery_ids) - end_delivery_pos :]

            if not kept_deliveries:
                continue

            end_node_id = kept_deliveries[-1]
            seq_end     = position[end_node_id]

            if seq_end < seq_start:
                continue

            # Slice the base path to get the fragment sequence
            subpath   = base_path[seq_start : seq_end + 1]
            sequence  = tuple(nodes[i].node_id for i in subpath)

            # Pickups still onboard at the end: those whose deliveries were not
            # included in this fragment
            end_onboard = frozenset(
                inst.delivery_to_pickup[d_id]
                for d_id in pending_deliveries
                if d_id in inst.delivery_to_pickup
            )

            contains_charge = any(nodes[i].is_station for i in subpath)
            min_start_energy = inst.minimum_start_energy(sequence)

            fragments.append(Fragment(
                sequence         = sequence,
                start_onboard    = start_onboard,
                end_onboard      = end_onboard,
                contains_charge  = contains_charge,
                min_start_energy = min_start_energy,
            ))

    return fragments


def enumerate_restricted_fragments(
    inst:       Instance,
    base_paths: List[Tuple[int, ...]],
) -> List[Fragment]:
    """Trim all base paths into restricted fragments."""
    fragments: List[Fragment] = []
    for bp in base_paths:
        fragments.extend(_trim_base_path(inst, bp))
    return fragments


# ---------------------------------------------------------------------------
# Extended fragment generation
# ---------------------------------------------------------------------------

def extend_restricted_fragments(
    inst:      Instance,
    fragments: List[Fragment],
) -> List[Fragment]:
    """
    Extend each restricted fragment by one more pickup (or to the depot).

    For each RF we generate:
    1. A depot extension  (if end_onboard is empty, the vehicle can return).
    2. One extension per unvisited pickup reachable from the RF's end node,
       subject to capacity, time feasibility, and energy reachability checks.

    A single intermediate charging station may be inserted on the extension
    leg when the direct arc exceeds the battery capacity.

    Returns a list of extended Fragment objects (without metrics — call
    attach_fragment_metrics separately).
    """
    extensions: List[Fragment] = []

    for rf in fragments:
        sequence   = rf.sequence
        end_id     = sequence[-1]
        visited    = set(sequence)
        start_on   = rf.start_onboard
        end_on     = rf.end_onboard

        # ------------------------------------------------------------------
        # 1. Depot extension (only when vehicle is empty at fragment end)
        # ------------------------------------------------------------------
        if not end_on:
            depot_id = inst.depot_id
            if inst.reachable_on_full_battery(end_id, depot_id):
                ext_seq = sequence + (depot_id,)
                bridging_station = None
            else:
                bridging_station = inst.best_station_between(end_id, depot_id)
                if bridging_station is not None:
                    ext_seq = sequence + (bridging_station, depot_id)
                else:
                    ext_seq = None   # depot unreachable even via a station

            if ext_seq is not None:
                extensions.append(Fragment(
                    sequence          = ext_seq,
                    start_onboard     = start_on,
                    end_onboard       = frozenset(),
                    contains_charge   = rf.contains_charge or (bridging_station is not None),
                    min_start_energy  = inst.minimum_start_energy(ext_seq),
                    extended_to       = depot_id,
                    extension_station = bridging_station,
                    extension_delivery= None,
                ))

        # ------------------------------------------------------------------
        # 2. Pickup extensions
        # ------------------------------------------------------------------
        for p_idx in inst.pickup_indices:
            p_id = inst.nodes[p_idx].node_id

            # Already visited or already in cargo
            if p_id in visited or p_id in start_on or p_id in end_on:
                continue

            # Delivery already occurred inside this fragment
            d_id = inst.pickup_to_delivery.get(p_id)
            if d_id in visited:
                continue

            # Capacity after adding this pickup
            new_end_on = end_on | {p_id}
            new_load   = sum(
                inst.nodes[inst.idx(pid)].demand for pid in new_end_on
            )
            if new_load > inst.load_capacity + EPS:
                continue

            # Quick time-feasibility check on the new pickup-delivery pair
            if not inst.earliest_delivery_feasible(p_id):
                continue

            # Energy reachability: end_id -> p_id (direct or via one station)
            if inst.reachable_on_full_battery(end_id, p_id):
                bridging_station = None
                ext_seq = sequence + (p_id,)
            else:
                bridging_station = inst.best_station_between(end_id, p_id)
                if bridging_station is None:
                    continue   # pickup unreachable even via a station
                ext_seq = sequence + (bridging_station, p_id)

            extensions.append(Fragment(
                sequence          = ext_seq,
                start_onboard     = start_on,
                end_onboard       = frozenset(new_end_on),
                contains_charge   = rf.contains_charge or (bridging_station is not None),
                min_start_energy  = inst.minimum_start_energy(ext_seq),
                extended_to       = p_id,
                extension_station = bridging_station,
                extension_delivery= d_id,
            ))

    return extensions


# ---------------------------------------------------------------------------
# Full preprocessing pipeline
# ---------------------------------------------------------------------------

def build_restricted_fragments(
    inst:   Instance,
    maxlen: int = 18,
) -> Tuple[List[Fragment], Dict[str, int]]:
    """
    Run the full restricted-fragment pipeline:
        enumerate base paths
        → trim to restricted fragments
        → exact dedup
        → attach metrics
        → dominance filter
        → metric dedup

    Returns (undominated_restricted_fragments, pruning_stats).
    """
    base_paths, pruning_stats = enumerate_base_paths(inst, maxlen)
    raw          = enumerate_restricted_fragments(inst, base_paths)
    deduped      = dedup_exact(raw)
    with_metrics = attach_fragment_metrics(inst, deduped, exclude_last=False)
    filtered     = dominance_filter(with_metrics)
    final        = dedup_by_metrics(filtered)
    return final, pruning_stats


def build_extended_fragments(
    inst:                Instance,
    restricted_fragments: List[Fragment],
) -> List[Fragment]:
    """
    Run the full extended-fragment pipeline:
        extend restricted fragments
        → exact dedup
        → attach metrics
        → dominance filter
        → metric dedup

    Returns undominated extended fragments ready for MILP construction.
    """
    raw          = extend_restricted_fragments(inst, restricted_fragments)
    deduped      = dedup_exact(raw)
    with_metrics = attach_fragment_metrics(inst, deduped, exclude_last=True)
    filtered     = dominance_filter(with_metrics)
    final        = dedup_by_metrics(filtered)
    return final


# ---------------------------------------------------------------------------
# Diagnostic stats
# ---------------------------------------------------------------------------

def fragment_stats(fragments: List[Fragment], label: str = '') -> Dict:
    """Return a dict of summary statistics for a fragment list."""
    if not fragments:
        stats = {'count': 0}
        print(f'{label} {stats}')
        return stats

    lengths      = [len(f.sequence) for f in fragments]
    with_charge  = sum(1 for f in fragments if f.contains_charge)
    depot_ends   = sum(1 for f in fragments if f.extended_to == inst_depot_sentinel)

    stats = {
        'count':                len(fragments),
        'min_seq_length':       min(lengths),
        'max_seq_length':       max(lengths),
        'avg_seq_length':       sum(lengths) / len(lengths),
        'with_charging_stop':   with_charge,
        'without_charging_stop': len(fragments) - with_charge,
    }
    if label:
        print(f'{label}:', stats)
    return stats


# Sentinel used in stats only — avoids importing depot_id everywhere
inst_depot_sentinel = 'D0'
