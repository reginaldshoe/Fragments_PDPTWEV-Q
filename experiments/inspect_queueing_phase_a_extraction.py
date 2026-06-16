"""Inspect Phase A queueing activity extraction from a solved master incumbent.

Environment:
- Python 3.12
- gurobipy available in the target environment

Role:
- Diagnostic runner only.
- Builds the current fragment/master model.
- Solves the master without callback to obtain a selected incumbent.
- Runs queueing_core Phase A extraction.
- Writes a JSON activity certificate and checks basic invariants.

This runner intentionally does not call callback_core.py or the energy-DP.
"""
from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

from evrp_fragments import fragment_core as frag
from evrp_fragments import master_core as master
from evrp_fragments import queueing_core as qcore


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Inspect Phase A queueing activity extraction from a solved master incumbent."
    )
    parser.add_argument("--instance", required=True)
    parser.add_argument("--max-base-path-len", type=int, required=True)
    parser.add_argument("--k-max", type=int, required=True)
    parser.add_argument("--force-exact-k", action="store_true")
    parser.add_argument("--time-limit-sec", type=float, default=None)
    parser.add_argument("--out", default="queueing_phase_a_extraction.json")
    return parser.parse_args()


def _get_var_value(var: Any) -> float:
    for attr in ("X", "x"):
        if hasattr(var, attr):
            return float(getattr(var, attr))
    return float(var)


def _build_fragment_sets(instance: str, max_base_path_len: int) -> dict[str, Any]:
    data = frag.read_instance(instance)
    base_paths, base_path_metadata = frag.enumerate_base_paths(data, max_base_path_len)
    restricted_raw = frag.enumerate_fragments(data, base_paths)
    restricted_dedup = frag.dedup_exact(restricted_raw)
    restricted_meta = frag.attach_metadata(data, restricted_dedup)
    restricted_undominated = frag.dominance_filter(restricted_meta)

    extended_raw = frag.extend_all_fragments(data, restricted_undominated)
    extended_dedup = frag.dedup_exact(extended_raw)
    extended_meta = frag.attach_metadata(data, extended_dedup, exclude_last_ef=True)
    extended_undominated = frag.dominance_filter(extended_meta)

    return {
        "data": data,
        "base_paths": base_paths,
        "base_path_metadata": base_path_metadata,
        "restricted_raw": restricted_raw,
        "restricted_dedup": restricted_dedup,
        "restricted_meta": restricted_meta,
        "restricted_undominated": restricted_undominated,
        "extended_raw": extended_raw,
        "extended_dedup": extended_dedup,
        "extended_meta": extended_meta,
        "extended_undominated": extended_undominated,
    }


def _assert_phase_a_invariants(payload: dict[str, Any]) -> None:
    activities = payload["activities"]
    routes = payload["routes"]

    activity_ids = [a["activity_id"] for a in activities]
    if len(activity_ids) != len(set(activity_ids)):
        raise AssertionError("activity_id values are not unique")

    route_ids = {r["route_id"] for r in routes}
    for activity in activities:
        aid = activity["activity_id"]
        if activity["route_id"] not in route_ids:
            raise AssertionError(f"activity {aid} references unknown route_id={activity['route_id']}")

        if abs(activity["base_end"] - (activity["base_start"] + activity["duration"])) > 1e-7:
            raise AssertionError(f"base_end mismatch for activity {aid}")

        if activity["kind"] == "movement":
            if activity["sid_from"] is None or activity["sid_to"] is None:
                raise AssertionError(f"movement activity missing endpoints: {activity}")
            if activity["resource_type"] != "route":
                raise AssertionError(f"movement activity should use route resource: {activity}")

        elif activity["kind"] == "service":
            if activity["node_sid"] is None:
                raise AssertionError(f"service activity missing node_sid: {activity}")
            if activity["resource_type"] != "customer":
                raise AssertionError(f"service activity missing customer resource: {activity}")
            if not str(activity["resource_id"]).startswith("customer_location:"):
                raise AssertionError(f"service activity does not use physical-location key: {activity}")
            if activity["latest_start"] is None:
                raise AssertionError(f"service activity missing latest_start: {activity}")
            if activity["slide"] is None:
                raise AssertionError(f"service activity missing slide: {activity}")
        else:
            raise AssertionError(f"unexpected activity kind for activity {aid}: {activity['kind']}")

        if len(activity["supporting_arc_ids"]) != 1:
            raise AssertionError(f"activity does not have specific arc provenance: {activity}")
        if activity["supporting_arc_ids"][0] != activity["arc_id"]:
            raise AssertionError(f"supporting_arc_ids does not match arc_id: {activity}")


def _resource_summary(payload: dict[str, Any]) -> dict[str, Any]:
    service_by_resource: dict[str, list[str]] = defaultdict(list)
    for activity in payload["activities"]:
        if activity["kind"] == "service":
            service_by_resource[str(activity["resource_id"])].append(str(activity["node_sid"]))

    shared_resources = {
        resource: sorted(nodes)
        for resource, nodes in service_by_resource.items()
        if len(set(nodes)) > 1
    }

    return {
        "customer_resource_count": len(service_by_resource),
        "shared_customer_resource_count": len(shared_resources),
        "shared_customer_resources": shared_resources,
    }


def main() -> None:
    args = parse_args()

    sets = _build_fragment_sets(args.instance, args.max_base_path_len)
    data = sets["data"]
    ef = sets["extended_undominated"]

    model, x_vars, arcs, node_id, depot_u, arc_by_id, theta, big_m = master.build_master_model(
        data,
        ef,
        args.k_max,
        force_exact_K=args.force_exact_k,
    )

    if args.time_limit_sec is not None:
        if args.time_limit_sec <= 0:
            raise ValueError("--time-limit-sec must be positive when supplied")
        model.Params.TimeLimit = float(args.time_limit_sec)

    # Deliberately solve the master without callback: this tests extraction only.
    model.optimize()

    selected_arc_ids = sorted(
        aid for aid, var in x_vars.items()
        if _get_var_value(var) > 0.5
    )

    extraction = qcore.extract_queue_activities_from_selected_routes(
        data=data,
        arcs=arcs,
        depot_u=depot_u,
        selected_arc_ids=selected_arc_ids,
    )
    payload = qcore.activities_to_jsonable(extraction)
    _assert_phase_a_invariants(payload)

    activity_kind_counts = Counter(a["kind"] for a in payload["activities"])
    resource_info = _resource_summary(payload)

    output = {
        "summary": {
            "instance": args.instance,
            "status": int(getattr(model, "Status", getattr(model, "status", 0))),
            "selected_arc_count": len(selected_arc_ids),
            "route_count": len(payload["routes"]),
            "activity_count": len(payload["activities"]),
            "movement_count": int(activity_kind_counts.get("movement", 0)),
            "service_count": int(activity_kind_counts.get("service", 0)),
            "ignored_station_count": len(payload["ignored_station_sids"]),
            "base_paths": len(sets["base_paths"]),
            "restricted_raw": len(sets["restricted_raw"]),
            "restricted_undominated": len(sets["restricted_undominated"]),
            "extended_raw": len(sets["extended_raw"]),
            "extended_undominated": len(sets["extended_undominated"]),
            **resource_info,
        },
        "selected_arc_ids": selected_arc_ids,
        "phase_a_extraction": payload,
    }

    Path(args.out).write_text(json.dumps(output, indent=2), encoding="utf-8")
    print(json.dumps(output["summary"], indent=2))


if __name__ == "__main__":
    main()
