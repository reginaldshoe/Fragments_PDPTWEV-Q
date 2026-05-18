"""
diagnose.py
-----------
Run this locally against c104C10.txt to identify the source of the repeated
customer nodes in the skeleton.

Usage:
    python diagnose.py instances/c104C10.txt
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from instance import read_instance
from fragment import (
    build_restricted_fragments, build_extended_fragments,
    enumerate_base_paths, enumerate_restricted_fragments,
    _trim_base_path, dedup_exact, attach_fragment_metrics,
)
from network import build_milp_network
from master import stitch_arc_sequences, skeleton_from_sequence


def has_repeated_customers(inst, sequence):
    custs = [s for s in sequence if inst.node(s).is_customer]
    return len(custs) != len(set(custs))


def repeated_customers(inst, sequence):
    custs = [s for s in sequence if inst.node(s).is_customer]
    seen, dups = set(), []
    for s in custs:
        if s in seen and s not in dups:
            dups.append(s)
        seen.add(s)
    return dups


def main():
    if len(sys.argv) < 2:
        print("Usage: python diagnose.py <instance_file>")
        sys.exit(1)

    path = Path(sys.argv[1])
    inst = read_instance(path)

    print(f"Instance: {path.name}")
    print(f"  Pickups:   {sorted(inst.pickup_ids)}")
    print(f"  Deliveries:{sorted(inst.delivery_ids)}")
    print(f"  Stations:  {sorted(inst.station_ids)}")
    print(f"  p2d: {inst.pickup_to_delivery}")
    print()

    # ---------------------------------------------------------------
    # Stage 1: base paths
    # ---------------------------------------------------------------
    base_paths, pruning = enumerate_base_paths(inst, maxlen=18)
    print(f"Base paths: {len(base_paths)}")
    for bp in base_paths:
        ids = [inst.nodes[i].node_id for i in bp]
        if has_repeated_customers(inst, ids):
            print(f"  REPEATED CUSTOMERS in base path: {ids}")
    print()

    # ---------------------------------------------------------------
    # Stage 2: restricted fragments (before dedup/dominance)
    # ---------------------------------------------------------------
    raw_rf = enumerate_restricted_fragments(inst, base_paths)
    print(f"Raw RFs: {len(raw_rf)}")
    rf_repeats = [(f, repeated_customers(inst, f.sequence))
                  for f in raw_rf if has_repeated_customers(inst, f.sequence)]
    if rf_repeats:
        print(f"  RFs with repeated customers: {len(rf_repeats)}")
        for f, dups in rf_repeats[:5]:
            print(f"    dups={dups}  seq={f.sequence}")
            print(f"    start_on={f.start_onboard} end_on={f.end_onboard}")
    else:
        print("  No repeated customers in raw RFs.")
    print()

    # ---------------------------------------------------------------
    # Stage 3: restricted fragments after full pipeline
    # ---------------------------------------------------------------
    rf, _ = build_restricted_fragments(inst, maxlen=18)
    print(f"Final RFs (after dominance filter): {len(rf)}")
    rf_repeats2 = [(f, repeated_customers(inst, f.sequence))
                   for f in rf if has_repeated_customers(inst, f.sequence)]
    if rf_repeats2:
        print(f"  RFs with repeated customers: {len(rf_repeats2)}")
        for f, dups in rf_repeats2[:5]:
            print(f"    dups={dups}  seq={f.sequence}")
    else:
        print("  No repeated customers in final RFs.")
    print()

    # ---------------------------------------------------------------
    # Stage 4: extended fragments (before dedup/dominance)
    # ---------------------------------------------------------------
    from fragment import extend_restricted_fragments
    raw_ef = extend_restricted_fragments(inst, rf)
    print(f"Raw EFs: {len(raw_ef)}")
    ef_repeats = [(f, repeated_customers(inst, f.sequence))
                  for f in raw_ef if has_repeated_customers(inst, f.sequence)]
    if ef_repeats:
        print(f"  EFs with repeated customers: {len(ef_repeats)}")
        for f, dups in ef_repeats[:10]:
            print(f"    dups={dups}  seq={f.sequence}")
            print(f"    start_on={f.start_onboard} end_on={f.end_onboard}")
            print(f"    extended_to={f.extended_to}")
    else:
        print("  No repeated customers in raw EFs.")
    print()

    # ---------------------------------------------------------------
    # Stage 5: extended fragments after full pipeline
    # ---------------------------------------------------------------
    ef = build_extended_fragments(inst, rf)
    print(f"Final EFs (after dominance filter): {len(ef)}")
    ef_repeats2 = [(f, repeated_customers(inst, f.sequence))
                   for f in ef if has_repeated_customers(inst, f.sequence)]
    if ef_repeats2:
        print(f"  EFs with repeated customers: {len(ef_repeats2)}")
        for f, dups in ef_repeats2[:10]:
            print(f"    dups={dups}  seq={f.sequence}")
            print(f"    start_on={f.start_onboard} end_on={f.end_onboard}")
    else:
        print("  No repeated customers in final EFs.")
    print()

    # ---------------------------------------------------------------
    # Stage 6: arc sequences (in network)
    # ---------------------------------------------------------------
    arcs, node_states, depot_idx = build_milp_network(inst, ef)
    arc_by_id = {a.arc_id: a for a in arcs}
    print(f"Network: {len(arcs)} arcs, {len(node_states)} nodes")

    arc_repeats = [(a, repeated_customers(inst, a.sequence))
                   for a in arcs if has_repeated_customers(inst, a.sequence)]
    if arc_repeats:
        print(f"  Arcs with repeated customers in sequence: {len(arc_repeats)}")
        for a, dups in arc_repeats[:10]:
            print(f"    arc_id={a.arc_id} dups={dups}")
            print(f"    seq={a.sequence}")
            print(f"    start_on={a.start_onboard} end_on={a.end_onboard}")
    else:
        print("  No repeated customers in arc sequences.")
    print()

    # ---------------------------------------------------------------
    # Stage 7: pairwise arc chains within single vehicle
    # (only arcs whose sequence does NOT pass through depot as interior node)
    # ---------------------------------------------------------------
    depot_id = inst.depot_id
    tail_map = {}
    for arc in arcs:
        tail_map.setdefault(arc.tail_index, []).append(arc)

    print("Arc pairs (same-vehicle chains, no depot interior) with skeleton repeats:")
    found = 0
    for arc1 in arcs:
        # skip arcs whose interior sequence contains the depot (cross-vehicle boundary)
        if depot_id in arc1.sequence[1:]:
            continue
        for arc2 in tail_map.get(arc1.head_index, []):
            if depot_id in arc2.sequence[1:]:
                continue
            s1, s2 = arc1.sequence, arc2.sequence
            combined = s1 + (s2[1:] if s1 and s2 and s1[-1] == s2[0] else s2)
            skel = tuple(s for s in combined if not inst.node(s).is_station)
            dups = repeated_customers(inst, skel)
            if dups:
                print(f"  arc{arc1.arc_id}->arc{arc2.arc_id}: dups={dups}")
                print(f"    arc1 seq={arc1.sequence}")
                print(f"      start_on={arc1.start_onboard} end_on={arc1.end_onboard}")
                print(f"    arc2 seq={arc2.sequence}")
                print(f"      start_on={arc2.start_onboard} end_on={arc2.end_onboard}")
                found += 1
    if found == 0:
        print("  None found — duplicate must arise from 3+ arc chains or stitch logic.")
    print()

    # ---------------------------------------------------------------
    # Stage 8: check the specific skeleton from the bug report
    # ---------------------------------------------------------------
    target = ('D0','C9','C5','C6','C3','C1','C7','C4','C10','C8','C7','C4','C2','D0')
    print(f"Checking bug-report skeleton: {list(target)}")
    dups = repeated_customers(inst, target)
    print(f"  Repeated customers: {dups}")

    # Find which arcs could produce C7 or C4 as non-terminal nodes
    problem_nodes = set(dups)
    print(f"\nArcs containing {problem_nodes} as interior (non-terminal) nodes:")
    for arc in arcs:
        interior = arc.sequence[1:-1]
        hits = [s for s in interior if s in problem_nodes]
        if hits:
            print(f"  arc_id={arc.arc_id} interior hits={hits}")
            print(f"    seq={arc.sequence}")
            print(f"    start_on={arc.start_onboard} end_on={arc.end_onboard}")
            print(f"    tail_idx={arc.tail_index} head_idx={arc.head_index}")
    print()

    # ---------------------------------------------------------------
    # Stage 9: validate skeleton sanity check in DP
    # ---------------------------------------------------------------
    from dp import solve_route, DPStats
    print("Testing DP skeleton sanity check on repeated skeleton:")
    try:
        stats = DPStats()
        result = solve_route(inst, target, stats=stats)
        print(f"  WARNING: DP accepted repeated skeleton without error!")
        print(f"  feasible={result.feasible}, dist={result.total_distance}")
    except ValueError as e:
        print(f"  DP raised ValueError as expected: {e}")
    print()


if __name__ == "__main__":
    main()
