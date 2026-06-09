#!/usr/bin/env python3
"""Export the incumbent route under the callback DP-repaired interpretation.

This runner is diagnostic/output-only. It does not modify solver, callback, master,
fragment, or helper code.

It reconstructs the selected incumbent arcs from the actual callback/master arc_by_id,
builds the selected stitched fragment SID sequence, forms the station-free skeleton,
runs callback_core.dp_route_min_dist(...) on that skeleton, and exports both:

- selected_fragment_sid_sequence: the literal stitched selected fragments;
- executable_dp_repaired_sid_sequence: the DP-repaired route with station insertions.

The exported distance comparison should explain the final objective/theta when the
callback interpretation is active:

    base_route_cost + dp_delta ~= objective
    dp_delta ~= theta
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


def _stitch_sid_sequence(route, arc_by_id):
    sid_seq = []
    for k, aid in enumerate(route):
        s = list(arc_by_id[aid]["seq"])
        if k == 0:
            sid_seq = list(s)
        else:
            if sid_seq and sid_seq[-1] == s[0]:
                sid_seq.extend(s[1:])
            else:
                sid_seq.extend(s)
    return sid_seq


def _route_distance_from_sids(data, sid_seq):
    total = 0.0
    for u_sid, v_sid in zip(sid_seq, sid_seq[1:]):
        ui = data["sid_to_i"][u_sid]
        vi = data["sid_to_i"][v_sid]
        total += data["dist"](ui, vi)
    return total


def _extract_routes_from_solution(chosen_arc_ids, arc_by_id, depot_sid="D0"):
    start_map = {}
    for aid in chosen_arc_ids:
        a = arc_by_id[aid]
        start_map.setdefault(a["Start"], []).append(aid)
    routes = []
    for aid0 in start_map.get(depot_sid, []):
        route = [aid0]
        cur_aid = aid0
        cur_end = arc_by_id[cur_aid]["End"]
        safety = 0
        while cur_end != depot_sid:
            nexts = start_map.get(cur_end, [])
            if len(nexts) != 1:
                raise RuntimeError(f"Ambiguous or missing continuation at {cur_end}: {nexts}")
            cur_aid = nexts[0]
            route.append(cur_aid)
            cur_end = arc_by_id[cur_aid]["End"]
            safety += 1
            if safety > len(chosen_arc_ids) + 5:
                raise RuntimeError("route reconstruction safety limit exceeded")
        routes.append(route)
    return routes


def _simulate_sid_sequence_with_step(data, sid_seq):
    from evrp_fragments.fragment_core import step
    st = ((0,), 0, frozenset(), data["CapE"], 0.0, frozenset(), frozenset(), frozenset(), 0, 0.0)
    trace = []
    for sid in sid_seq[1:]:
        prev_idx = st[0][-1]
        j = data["sid_to_i"][sid]
        before_E = st[3]
        travel_energy = data["energy"](prev_idx, j)
        st2, reason = step(data, st, j)
        trace.append({
            "from_sid": data["nodes"][prev_idx][0],
            "to_sid": sid,
            "energy_before": before_E,
            "travel_energy": travel_energy,
            "energy_after_direct_travel_without_recharge": before_E - travel_energy,
            "ok": st2 is not None,
            "reason": reason,
        })
        if st2 is None:
            return False, sid, reason, trace
        st = st2
    return True, None, None, trace


def _is_station(data, sid):
    idx = data["sid_to_i"][sid]
    return data["nodes"][idx][1] == "S"


def main() -> int:
    parser = argparse.ArgumentParser(description="Export incumbent DP-repaired route")
    parser.add_argument("--instance", required=True)
    parser.add_argument("--max-base-path-len", type=int, required=True)
    parser.add_argument("--k-max", type=int, default=1)
    parser.add_argument("--use-callback", action="store_true")
    parser.add_argument("--time-limit-sec", type=float, default=300.0)
    parser.add_argument("--depot-sid", default="D0")
    parser.add_argument("--selected-arc-ids", default=None, help="Optional comma-separated selected arc ids; otherwise use solver summary")
    parser.add_argument("--output", default="incumbent_dp_repaired_route.json")
    args = parser.parse_args()

    from evrp_fragments.fragment_core import read_instance
    from evrp_fragments.pipeline_v1e import run_solver
    import evrp_fragments.pipeline_v1e as pipeline_v1e
    import evrp_fragments.callback_core as cbcore

    data = read_instance(args.instance)
    solver_kwargs = {
        "instance": args.instance,
        "max_base_path_len": args.max_base_path_len,
        "k_max": args.k_max,
        "force_exact_k": False,
        "use_callback": args.use_callback,
    }
    tl_name = _time_limit_kwarg_name(run_solver)
    if tl_name is not None:
        solver_kwargs[tl_name] = args.time_limit_sec

    summary_obj = _call_with_supported_kwargs(run_solver, **solver_kwargs)
    summary = _summary_to_dict(summary_obj)

    if args.selected_arc_ids:
        selected_arc_ids = [int(x.strip()) for x in args.selected_arc_ids.split(",") if x.strip()]
    else:
        selected_arc_ids = summary.get("selected_arc_ids") or getattr(summary_obj, "selected_arc_ids", None) or []

    arc_by_id = getattr(getattr(pipeline_v1e, "cbcore", None), "arc_by_id", None)
    if not isinstance(arc_by_id, dict):
        arc_by_id = getattr(cbcore, "arc_by_id", None)

    payload = {
        "instance": args.instance,
        "summary": summary,
        "selected_arc_ids": selected_arc_ids,
        "arc_by_id_found": isinstance(arc_by_id, dict),
        "arc_by_id_count": len(arc_by_id) if isinstance(arc_by_id, dict) else None,
        "routes": [],
        "error": None,
    }

    if not isinstance(arc_by_id, dict):
        payload["error"] = "arc_by_id not available after run_solver"
        Path(args.output).write_text(json.dumps(_jsonable(payload), indent=2), encoding="utf-8")
        print(json.dumps(_jsonable(payload), indent=2))
        return 1

    missing = [aid for aid in selected_arc_ids if aid not in arc_by_id]
    if missing:
        payload["error"] = f"selected arc ids missing from arc_by_id: {missing}"
        Path(args.output).write_text(json.dumps(_jsonable(payload), indent=2), encoding="utf-8")
        print(json.dumps(_jsonable(payload), indent=2))
        return 1

    try:
        routes = _extract_routes_from_solution(selected_arc_ids, arc_by_id, depot_sid=args.depot_sid)
    except Exception as exc:
        payload["error"] = f"route reconstruction failed: {type(exc).__name__}: {exc}"
        Path(args.output).write_text(json.dumps(_jsonable(payload), indent=2), encoding="utf-8")
        print(json.dumps(_jsonable(payload), indent=2))
        return 1

    for route in routes:
        selected_sid_seq = _stitch_sid_sequence(route, arc_by_id)
        selected_base_distance = _route_distance_from_sids(data, selected_sid_seq)
        selected_replay_ok, selected_failed_sid, selected_failed_reason, selected_replay_trace = _simulate_sid_sequence_with_step(data, selected_sid_seq)

        skeleton = [sid for sid in selected_sid_seq if not _is_station(data, sid)]
        dp_ok, dp_dist, dp_full_path, fail_i = cbcore.dp_route_min_dist(
            data,
            skeleton,
            t0=0.0,
            E0=data["CapE"],
            max_station_visits_per_leg=getattr(cbcore, "DP_MAX_STATION_VISITS_PER_LEG", 6),
            max_labels_per_node=getattr(cbcore, "DP_MAX_LABELS_PER_NODE", 500),
        )
        base_route_cost = sum(float(arc_by_id[aid]["Df"]) for aid in route)
        delta = None if dp_dist is None else dp_dist - base_route_cost
        objective = summary.get("objective")
        theta = summary.get("theta")
        route_payload = {
            "route_arc_ids": route,
            "selected_arc_records": {str(aid): arc_by_id[aid] for aid in route},
            "selected_fragment_sid_sequence": selected_sid_seq,
            "selected_fragment_station_sids": [sid for sid in selected_sid_seq if _is_station(data, sid)],
            "selected_fragment_route_distance": selected_base_distance,
            "selected_fragment_route_is_directly_replay_feasible": selected_replay_ok,
            "selected_fragment_failed_sid": selected_failed_sid,
            "selected_fragment_failed_reason": selected_failed_reason,
            "selected_fragment_replay_trace_tail": selected_replay_trace[-5:],
            "skeleton": skeleton,
            "callback_dp_repaired_route_is_feasible": bool(dp_ok),
            "callback_dp_fail_i": fail_i,
            "executable_route_sid_sequence": list(dp_full_path) if dp_full_path is not None else None,
            "executable_route_station_sids": [sid for sid in dp_full_path if _is_station(data, sid)] if dp_full_path is not None else None,
            "callback_dp_distance": dp_dist,
            "base_route_cost": base_route_cost,
            "callback_dp_delta": delta,
            "theta": theta,
            "objective": objective,
            "base_plus_delta": (base_route_cost + delta) if delta is not None else None,
            "delta_minus_theta": (delta - theta) if delta is not None and isinstance(theta, (int, float)) else None,
            "base_plus_delta_minus_objective": ((base_route_cost + delta) - objective) if delta is not None and isinstance(objective, (int, float)) else None,
        }
        payload["routes"].append(route_payload)

    Path(args.output).write_text(json.dumps(_jsonable(payload), indent=2), encoding="utf-8")
    print(json.dumps(_jsonable({
        "selected_arc_ids": selected_arc_ids,
        "routes": [
            {
                "route_arc_ids": r["route_arc_ids"],
                "selected_fragment_route_is_directly_replay_feasible": r["selected_fragment_route_is_directly_replay_feasible"],
                "selected_fragment_failed_sid": r["selected_fragment_failed_sid"],
                "selected_fragment_failed_reason": r["selected_fragment_failed_reason"],
                "callback_dp_repaired_route_is_feasible": r["callback_dp_repaired_route_is_feasible"],
                "base_route_cost": r["base_route_cost"],
                "callback_dp_distance": r["callback_dp_distance"],
                "callback_dp_delta": r["callback_dp_delta"],
                "theta": r["theta"],
                "objective": r["objective"],
                "executable_route_sid_sequence": r["executable_route_sid_sequence"],
            }
            for r in payload["routes"]
        ],
    }), indent=2))
    print(f"[INCUMBENT-DP-REPAIRED-ROUTE-WRITTEN] path={args.output}", flush=True)
    return 0 if all(r["callback_dp_repaired_route_is_feasible"] for r in payload["routes"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
