#!/usr/bin/env python3
"""Trace selected-route energy replay at SID-step level.

This diagnostic runner is for the post-solver route reconstructed through the actual
master/callback arc_by_id mapping. It deliberately does not modify solver, callback,
master, or fragment code.

It:
1. runs the existing solver;
2. reads the actual cbcore.arc_by_id mapping populated during solve;
3. extracts selected route arc chains using solution_helpers;
4. stitches the selected fragment sequences into SID sequences;
5. replays each SID transition through fragment_core.step(...);
6. writes a detailed before/after energy/time/load trace, including the failing step.
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


def _node_record(data, idx):
    sid, kind, typ, x, y, dem, ready, due, serv, partner = data["nodes"][idx]
    return {
        "idx": idx,
        "sid": sid,
        "kind": kind,
        "typ": typ,
        "demand": dem,
        "ready": ready,
        "due": due,
        "service": serv,
        "partner": partner,
    }


def _state_snapshot(data, state):
    path, phase, onboard, E, t_depart, seenP, seenD, seenS, deliv_count, distance = state
    cur_idx = path[-1]
    return {
        "path_indices": list(path),
        "path_sids": [data["nodes"][i][0] for i in path],
        "current_idx": cur_idx,
        "current_sid": data["nodes"][cur_idx][0],
        "phase": phase,
        "onboard": sorted(onboard),
        "E": E,
        "t_depart": t_depart,
        "seenP": sorted(seenP),
        "seenD": sorted(seenD),
        "seenS": sorted(seenS),
        "deliv_count": deliv_count,
        "distance": distance,
    }


def _initial_state(data):
    return (
        (0,),             # path of node indices, starting at depot index 0
        0,                # pickup phase
        frozenset(),      # onboard pickup SIDs
        data["CapE"],     # remaining battery energy
        0.0,              # departure time
        frozenset(),      # seen pickups
        frozenset(),      # seen deliveries
        frozenset(),      # seen stations
        0,                # delivered count
        0.0,              # accumulated distance
    )


def trace_sid_sequence(data, sid_seq):
    """Replay a stitched SID sequence through fragment_core.step with detailed trace."""
    from evrp_fragments.fragment_core import step

    st = _initial_state(data)
    trace = []
    ok = True
    failed_sid = None
    failed_reason = None

    for pos, sid in enumerate(sid_seq[1:], start=1):
        before = _state_snapshot(data, st)
        prev_idx = st[0][-1]
        j = data["sid_to_i"][sid]
        travel_energy = data["energy"](prev_idx, j)
        travel_time = data["traveltime"](prev_idx, j)
        travel_distance = data["dist"](prev_idx, j)
        target_node = _node_record(data, j)

        newst, reason = step(data, st, j)
        event = {
            "position": pos,
            "from_sid": before["current_sid"],
            "to_sid": sid,
            "from_idx": prev_idx,
            "to_idx": j,
            "target_node": target_node,
            "travel_energy": travel_energy,
            "travel_time": travel_time,
            "travel_distance": travel_distance,
            "energy_before": before["E"],
            "energy_after_direct_travel_without_recharge": before["E"] - travel_energy,
            "state_before": before,
            "step_ok": newst is not None,
            "reason": reason,
            "state_after": _state_snapshot(data, newst) if newst is not None else None,
        }
        trace.append(event)
        if newst is None:
            ok = False
            failed_sid = sid
            failed_reason = reason
            break
        st = newst

    return {
        "ok": ok,
        "failed_sid": failed_sid,
        "reason": failed_reason,
        "trace": trace,
        "final_state": _state_snapshot(data, st) if ok else None,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Trace selected route energy failure through fragment_core.step")
    parser.add_argument("--instance", required=True)
    parser.add_argument("--max-base-path-len", type=int, required=True)
    parser.add_argument("--k-max", type=int, default=1)
    parser.add_argument("--use-callback", action="store_true")
    parser.add_argument("--time-limit-sec", type=float, default=300.0)
    parser.add_argument("--depot-sid", default="D0")
    parser.add_argument("--selected-arc-ids", default=None, help="Optional comma-separated selected arc ids; otherwise read from solver summary")
    parser.add_argument("--output", default="solution_route_energy_trace.json")
    args = parser.parse_args()

    from evrp_fragments import solution_helpers as helpers
    from evrp_fragments.fragment_core import read_instance
    from evrp_fragments.pipeline_v1e import run_solver
    import evrp_fragments.pipeline_v1e as pipeline_v1e

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
    payload = {
        "instance": args.instance,
        "summary": summary,
        "selected_arc_ids": selected_arc_ids,
        "arc_by_id_found": isinstance(arc_by_id, dict),
        "arc_by_id_count": len(arc_by_id) if isinstance(arc_by_id, dict) else None,
        "routes": None,
        "route_traces": [],
        "error": None,
    }

    if not isinstance(arc_by_id, dict):
        payload["error"] = "pipeline_v1e.cbcore.arc_by_id was not available after run_solver"
        Path(args.output).write_text(json.dumps(_jsonable(payload), indent=2), encoding="utf-8")
        print(json.dumps(_jsonable(payload), indent=2))
        return 1

    missing = [aid for aid in selected_arc_ids if aid not in arc_by_id]
    if missing:
        payload["error"] = f"selected arc ids missing from arc_by_id: {missing}"
        Path(args.output).write_text(json.dumps(_jsonable(payload), indent=2), encoding="utf-8")
        print(json.dumps(_jsonable(payload), indent=2))
        return 1

    routes = helpers.extract_routes_from_solution(selected_arc_ids, arc_by_id, depot_sid=args.depot_sid)
    payload["routes"] = routes
    if not routes:
        payload["error"] = "selected arc ids were present but no depot-starting route was extracted"
        Path(args.output).write_text(json.dumps(_jsonable(payload), indent=2), encoding="utf-8")
        print(json.dumps(_jsonable(payload), indent=2))
        return 1

    for route in routes:
        sid_seq = helpers.stitch_sid_sequence(route, arc_by_id)
        distance = helpers.route_distance_from_sids(data, sid_seq)
        trace_result = trace_sid_sequence(data, sid_seq)
        payload["route_traces"].append({
            "route_arc_ids": route,
            "selected_arc_records": {str(aid): arc_by_id[aid] for aid in route},
            "sid_sequence": sid_seq,
            "route_distance": distance,
            "trace_result": trace_result,
        })

    Path(args.output).write_text(json.dumps(_jsonable(payload), indent=2), encoding="utf-8")
    print(json.dumps(_jsonable({
        "selected_arc_ids": selected_arc_ids,
        "routes": routes,
        "route_trace_summaries": [
            {
                "route_arc_ids": r["route_arc_ids"],
                "route_distance": r["route_distance"],
                "ok": r["trace_result"]["ok"],
                "failed_sid": r["trace_result"]["failed_sid"],
                "reason": r["trace_result"]["reason"],
            }
            for r in payload["route_traces"]
        ],
    }), indent=2))
    print(f"[SOLUTION-ROUTE-ENERGY-TRACE-WRITTEN] path={args.output}", flush=True)
    return 0 if all(r["trace_result"]["ok"] for r in payload["route_traces"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
