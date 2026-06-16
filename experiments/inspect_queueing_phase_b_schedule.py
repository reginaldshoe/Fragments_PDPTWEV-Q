"""Inspect Phase B queueing no-overlap scheduling from a solved master incumbent."""
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
    parser = argparse.ArgumentParser(description="Inspect Phase B queueing no-overlap scheduling.")
    parser.add_argument("--instance", required=True)
    parser.add_argument("--max-base-path-len", type=int, required=True)
    parser.add_argument("--k-max", type=int, required=True)
    parser.add_argument("--force-exact-k", action="store_true")
    parser.add_argument("--time-limit-sec", type=float, default=None)
    parser.add_argument("--out", default="queueing_phase_b_schedule.json")
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
        "restricted_raw": restricted_raw,
        "restricted_undominated": restricted_undominated,
        "extended_raw": extended_raw,
        "extended_undominated": extended_undominated,
    }


def _resource_summary(extraction_payload: dict[str, Any]) -> dict[str, Any]:
    service_by_resource: dict[str, list[str]] = defaultdict(list)
    for activity in extraction_payload["activities"]:
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
    model, x_vars, arcs, node_id, depot_u, arc_by_id, theta, big_m = master.build_master_model(
        data,
        sets["extended_undominated"],
        args.k_max,
        force_exact_K=args.force_exact_k,
    )
    if args.time_limit_sec is not None:
        if args.time_limit_sec <= 0:
            raise ValueError("--time-limit-sec must be positive when supplied")
        model.Params.TimeLimit = float(args.time_limit_sec)
    model.optimize()

    selected_arc_ids = sorted(aid for aid, var in x_vars.items() if _get_var_value(var) > 0.5)
    extraction = qcore.extract_queue_activities_from_selected_routes(
        data=data,
        arcs=arcs,
        depot_u=depot_u,
        selected_arc_ids=selected_arc_ids,
    )
    extraction_payload = qcore.activities_to_jsonable(extraction)
    result = qcore.check_queueing_no_overlap(extraction)
    result_payload = qcore.schedule_result_to_jsonable(result)
    activity_kind_counts = Counter(a["kind"] for a in extraction_payload["activities"])
    resource_info = _resource_summary(extraction_payload)

    summary = {
        "instance": args.instance,
        "status": int(getattr(model, "Status", getattr(model, "status", 0))),
        "selected_arc_count": len(selected_arc_ids),
        "route_count": len(extraction_payload["routes"]),
        "activity_count": len(extraction_payload["activities"]),
        "movement_count": int(activity_kind_counts.get("movement", 0)),
        "service_count": int(activity_kind_counts.get("service", 0)),
        "ignored_station_count": len(extraction_payload["ignored_station_sids"]),
        **resource_info,
        "phase_b_ok": result.ok,
        "phase_b_total_delay": result.total_delay,
        "phase_b_total_service_delay": result.total_service_delay,
        "phase_b_conflict_count": len(result.conflicts),
        "phase_b_infeasible_arc_ids": list(result.infeasible_arc_ids),
        "base_paths": len(sets["base_paths"]),
        "restricted_raw": len(sets["restricted_raw"]),
        "restricted_undominated": len(sets["restricted_undominated"]),
        "extended_raw": len(sets["extended_raw"]),
        "extended_undominated": len(sets["extended_undominated"]),
    }

    output = {
        "summary": summary,
        "selected_arc_ids": selected_arc_ids,
        "phase_a_extraction": extraction_payload,
        "phase_b_schedule": result_payload,
    }
    Path(args.out).write_text(json.dumps(output, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
