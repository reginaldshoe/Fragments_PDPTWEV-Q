#!/usr/bin/env python3
"""Simple master-callback route checker.

Purpose:
  Run the solver, reconstruct the selected master route, validate it under the
  callback DP-repaired interpretation, and print/export a compact route certificate.

Output contract:
  - validation_ok: true/false
  - dp_repaired_skeleton: mandatory customer/depot skeleton used by route-DP
  - dp_repaired_route: executable route with inserted stations
  - distance/objective/theta consistency checks

This runner is intentionally quiet and output-oriented. It does not modify solver,
callback, master, or fragment code.
"""
from __future__ import annotations

import argparse
import dataclasses
import inspect
import json
from pathlib import Path
from typing import Any


def _jsonable(value: Any):
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return _jsonable(dataclasses.asdict(value))
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, (set, frozenset)):
        return sorted(_jsonable(v) for v in value)
    if hasattr(value, "__dict__"):
        return {str(k): _jsonable(v) for k, v in vars(value).items() if not k.startswith("_")}
    return repr(value)


def _summary_to_dict(summary: Any) -> dict[str, Any]:
    if isinstance(summary, dict):
        return dict(summary)
    if dataclasses.is_dataclass(summary) and not isinstance(summary, type):
        return dataclasses.asdict(summary)
    if hasattr(summary, "_asdict"):
        return dict(summary._asdict())
    if hasattr(summary, "__dict__"):
        return {k: v for k, v in vars(summary).items() if not k.startswith("_")}
    out = {}
    for name in dir(summary):
        if name.startswith("_"):
            continue
        try:
            value = getattr(summary, name)
        except Exception:
            continue
        if not callable(value):
            out[name] = value
    return out


def _call_with_supported_kwargs(fn, **kwargs):
    sig = inspect.signature(fn)
    params = sig.parameters
    if any(p.kind == inspect.Parameter.VAR_KEYWORD for p in params.values()):
        return fn(**kwargs)
    return fn(**{k: v for k, v in kwargs.items() if k in params})


def _time_limit_kwarg_name(fn):
    params = inspect.signature(fn).parameters
    for name in ("time_limit_sec", "solver_time_limit_sec", "time_limit", "TimeLimit"):
        if name in params:
            return name
    return None


def _extract_routes_from_solution(chosen_arc_ids, arc_by_id, depot_sid="D0"):
    start_map = {}
    for aid in chosen_arc_ids:
        a = arc_by_id[aid]
        start_map.setdefault(a["Start"], []).append(aid)
    routes = []
    for aid0 in start_map.get(depot_sid, []):
        route = [aid0]
        cur = arc_by_id[aid0]["End"]
        safety = 0
        while cur != depot_sid:
            nxt = start_map.get(cur, [])
            if len(nxt) != 1:
                raise RuntimeError(f"Ambiguous or missing continuation at {cur}: {nxt}")
            aid = nxt[0]
            route.append(aid)
            cur = arc_by_id[aid]["End"]
            safety += 1
            if safety > len(chosen_arc_ids) + 5:
                raise RuntimeError("route reconstruction safety limit exceeded")
        routes.append(route)
    return routes


def _stitch_sid_sequence(route, arc_by_id):
    out = []
    for k, aid in enumerate(route):
        seq = list(arc_by_id[aid]["seq"])
        if k == 0:
            out = seq
        elif out and out[-1] == seq[0]:
            out.extend(seq[1:])
        else:
            out.extend(seq)
    return out


def _is_station(data, sid):
    return data["nodes"][data["sid_to_i"][sid]][1] == "S"


def _route_distance(data, sid_seq):
    return sum(data["dist"](data["sid_to_i"][u], data["sid_to_i"][v]) for u, v in zip(sid_seq, sid_seq[1:]))


def check_master_callback_solution(instance, max_base_path_len, k_max=1, use_callback=True, time_limit_sec=300.0, depot_sid="D0"):
    from evrp_fragments.fragment_core import read_instance
    from evrp_fragments.pipeline_v1e import run_solver
    import evrp_fragments.pipeline_v1e as pipeline_v1e
    import evrp_fragments.callback_core as cbcore

    data = read_instance(instance)
    kwargs = {
        "instance": instance,
        "max_base_path_len": max_base_path_len,
        "k_max": k_max,
        "force_exact_k": False,
        "use_callback": use_callback,
    }
    tl_name = _time_limit_kwarg_name(run_solver)
    if tl_name:
        kwargs[tl_name] = time_limit_sec

    summary_obj = _call_with_supported_kwargs(run_solver, **kwargs)
    summary = _summary_to_dict(summary_obj)
    selected_arc_ids = summary.get("selected_arc_ids") or getattr(summary_obj, "selected_arc_ids", None) or []
    arc_by_id = getattr(getattr(pipeline_v1e, "cbcore", None), "arc_by_id", None)
    if not isinstance(arc_by_id, dict):
        arc_by_id = getattr(cbcore, "arc_by_id", None)
    if not isinstance(arc_by_id, dict):
        return {"validation_ok": False, "error": "arc_by_id unavailable after solve", "summary": summary}

    missing = [aid for aid in selected_arc_ids if aid not in arc_by_id]
    if missing:
        return {"validation_ok": False, "error": f"selected arc ids missing from arc_by_id: {missing}", "summary": summary}

    routes = _extract_routes_from_solution(selected_arc_ids, arc_by_id, depot_sid=depot_sid)
    route_results = []
    all_ok = True
    total_base = 0.0
    total_dp = 0.0

    for route in routes:
        selected_sid_seq = _stitch_sid_sequence(route, arc_by_id)
        skeleton = [sid for sid in selected_sid_seq if not _is_station(data, sid)]
        dp_ok, dp_dist, dp_path, fail_i = cbcore.dp_route_min_dist(
            data,
            skeleton,
            t0=0.0,
            E0=data["CapE"],
            max_station_visits_per_leg=getattr(cbcore, "DP_MAX_STATION_VISITS_PER_LEG", 6),
            max_labels_per_node=getattr(cbcore, "DP_MAX_LABELS_PER_NODE", 500),
        )
        base_cost = sum(float(arc_by_id[aid]["Df"]) for aid in route)
        delta = None if dp_dist is None else dp_dist - base_cost
        if not dp_ok:
            all_ok = False
        total_base += base_cost
        total_dp += dp_dist if dp_dist is not None else 0.0
        route_results.append({
            "route_arc_ids": route,
            "dp_repaired_skeleton": skeleton,
            "dp_repaired_route": list(dp_path) if dp_path is not None else None,
            "validation_ok": bool(dp_ok),
            "fail_i": fail_i,
            "base_route_cost": base_cost,
            "dp_repaired_distance": dp_dist,
            "dp_delta": delta,
            "selected_fragment_route": selected_sid_seq,
            "selected_fragment_stations": [sid for sid in selected_sid_seq if _is_station(data, sid)],
            "dp_repaired_stations": [sid for sid in dp_path if _is_station(data, sid)] if dp_path is not None else None,
        })

    theta = summary.get("theta")
    objective = summary.get("objective")
    total_delta = total_dp - total_base
    return {
        "validation_ok": all_ok and bool(routes),
        "instance": instance,
        "selected_arc_ids": selected_arc_ids,
        "route_count": len(routes),
        "routes": route_results,
        "total_base_route_cost": total_base,
        "total_dp_repaired_distance": total_dp,
        "total_dp_delta": total_delta,
        "theta": theta,
        "objective": objective,
        "delta_minus_theta": (total_delta - theta) if isinstance(theta, (int, float)) else None,
        "dp_distance_minus_objective": (total_dp - objective) if isinstance(objective, (int, float)) else None,
        "summary": summary,
        "error": None,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Check and print the master-callback DP-repaired incumbent route")
    parser.add_argument("--instance", required=True)
    parser.add_argument("--max-base-path-len", type=int, required=True)
    parser.add_argument("--k-max", type=int, default=1)
    parser.add_argument("--use-callback", action="store_true")
    parser.add_argument("--time-limit-sec", type=float, default=300.0)
    parser.add_argument("--output", default=None, help="Optional JSON output path")
    parser.add_argument("--quiet-json", action="store_true", help="Print compact JSON only")
    args = parser.parse_args()

    result = check_master_callback_solution(
        instance=args.instance,
        max_base_path_len=args.max_base_path_len,
        k_max=args.k_max,
        use_callback=args.use_callback,
        time_limit_sec=args.time_limit_sec,
    )

    if args.output:
        Path(args.output).write_text(json.dumps(_jsonable(result), indent=2), encoding="utf-8")

    if args.quiet_json:
        print(json.dumps(_jsonable(result), indent=2))
    else:
        print("[ROUTE-CHECK] validation_ok=" + str(result["validation_ok"]))
        print("[ROUTE-CHECK] selected_arc_ids=" + str(result.get("selected_arc_ids")))
        print("[ROUTE-CHECK] total_base_route_cost=" + str(result.get("total_base_route_cost")))
        print("[ROUTE-CHECK] total_dp_repaired_distance=" + str(result.get("total_dp_repaired_distance")))
        print("[ROUTE-CHECK] total_dp_delta=" + str(result.get("total_dp_delta")))
        print("[ROUTE-CHECK] theta=" + str(result.get("theta")))
        print("[ROUTE-CHECK] objective=" + str(result.get("objective")))
        print("[ROUTE-CHECK] delta_minus_theta=" + str(result.get("delta_minus_theta")))
        print("[ROUTE-CHECK] dp_distance_minus_objective=" + str(result.get("dp_distance_minus_objective")))
        for idx, route in enumerate(result.get("routes", []), start=1):
            print(f"[ROUTE-CHECK] route_{idx}_validation_ok={route['validation_ok']}")
            print(f"[ROUTE-CHECK] route_{idx}_dp_repaired_skeleton={route['dp_repaired_skeleton']}")
            print(f"[ROUTE-CHECK] route_{idx}_dp_repaired_route={route['dp_repaired_route']}")
    return 0 if result["validation_ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
