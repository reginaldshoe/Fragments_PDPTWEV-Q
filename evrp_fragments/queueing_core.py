"""Queueing / synchronisation activity extraction and Phase B scheduling layer.

Environment:
- Python 3.12
- No optimisation solver required for Phase A or the initial Phase B checker

Role:
- Phase A: convert selected master-callback fragment routes into movable activity records.
- Phase B: compute route-propagated slack and run a deterministic no-overlap checker
  for customer physical-location resources.
- Does not call or modify the energy DP.
- Does not modify callback_core.py.

Phase A design choices frozen in v5b:
- Queueing extraction remains separate from callback_core.py.
- Activities are extracted arc-by-arc, not from a fully stitched route only.
- Movement activities are retained.
- Stations are recorded in ignored_station_sids and ignored as congested resources.
- Customer/resource keys use physical location, not operation SID.
- Each activity is mapped to the specific selected fragment arc that generated it.

Phase B initial behaviour:
- Computes route-level propagated latest-start windows with a backward pass.
- Applies a simple deterministic customer-resource no-overlap schedule by delaying
  later customer-service activities and propagating that delay to the remaining
  suffix of the same route.
- Reports infeasibility if a required delay would push any activity beyond its
  propagated latest-start window.
- Still ignores charger/station congestion and energy-DP integration.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from math import isfinite
from typing import Any, Iterable

from .fragment_core import is_customer, is_station

EPS = 1e-7
INF = float("inf")


@dataclass(frozen=True)
class QueueActivity:
    """A fixed-duration activity extracted from a selected fragment arc."""

    activity_id: int
    route_id: int
    route_pos: int
    arc_id: int
    arc_pos: int
    kind: str
    sid_from: str | None
    sid_to: str | None
    node_sid: str | None
    resource_type: str | None
    resource_id: str | None
    duration: float
    earliest_start: float
    latest_start: float | None
    base_start: float
    base_end: float
    slide: float | None
    supporting_arc_ids: tuple[int, ...]


@dataclass(frozen=True)
class QueueRoute:
    """Selected master route represented as arc ids and stitched SID sequence."""

    route_id: int
    arc_ids: tuple[int, ...]
    sid_sequence: tuple[str, ...]


@dataclass(frozen=True)
class QueueActivityExtraction:
    """Phase A extraction result."""

    routes: tuple[QueueRoute, ...]
    activities: tuple[QueueActivity, ...]
    ignored_station_sids: tuple[str, ...]


@dataclass(frozen=True)
class QueueActivityWindow:
    """Route-propagated scheduling window for one extracted activity."""

    activity_id: int
    route_id: int
    route_pos: int
    arc_id: int
    kind: str
    resource_type: str | None
    resource_id: str | None
    base_start: float
    base_end: float
    duration: float
    earliest_start: float
    local_latest_start: float | None
    propagated_latest_start: float | None
    propagated_slack: float | None


@dataclass(frozen=True)
class ScheduledQueueActivity:
    """A scheduled activity after the Phase B no-overlap pass."""

    activity_id: int
    route_id: int
    route_pos: int
    arc_id: int
    kind: str
    resource_type: str | None
    resource_id: str | None
    base_start: float
    base_end: float
    scheduled_start: float
    scheduled_end: float
    delay: float
    propagated_latest_start: float | None
    supporting_arc_ids: tuple[int, ...]


@dataclass(frozen=True)
class QueueingConflict:
    """Conflict or infeasibility detected by the Phase B checker."""

    reason: str
    resource_id: str | None
    activity_ids: tuple[int, ...]
    arc_ids: tuple[int, ...]
    detail: str


@dataclass(frozen=True)
class QueueingScheduleResult:
    """Phase B no-overlap scheduling result."""

    ok: bool
    total_delay: float
    total_service_delay: float
    windows: tuple[QueueActivityWindow, ...]
    scheduled_activities: tuple[ScheduledQueueActivity, ...]
    conflicts: tuple[QueueingConflict, ...]
    infeasible_activity_ids: tuple[int, ...]
    infeasible_arc_ids: tuple[int, ...]


def _node_index(data: dict[str, Any], sid: str) -> int:
    return data["sid_to_i"][sid]


def _node_row(data: dict[str, Any], sid: str) -> tuple[Any, ...]:
    return data["nodes"][_node_index(data, sid)]


def _node_ready_due_service(data: dict[str, Any], sid: str) -> tuple[float, float, float]:
    row = _node_row(data, sid)
    return float(row[6]), float(row[7]), float(row[8])


def _node_xy(data: dict[str, Any], sid: str) -> tuple[float, float]:
    row = _node_row(data, sid)
    return float(row[3]), float(row[4])


def _travel_time(data: dict[str, Any], sid_from: str, sid_to: str) -> float:
    i = _node_index(data, sid_from)
    j = _node_index(data, sid_to)
    return float(data["traveltime"](i, j))


def _format_coord(value: float) -> str:
    return f"{float(value):.9g}"


def customer_physical_resource_id(data: dict[str, Any], sid: str) -> str:
    """Return the physical-location resource key for a customer operation."""

    x, y = _node_xy(data, sid)
    return f"customer_location:{_format_coord(x)},{_format_coord(y)}"


def fragment_arc_sid_sequence(arcs: list[dict[str, Any]], arc_id: int) -> tuple[str, ...]:
    """Return the SID sequence stored on a selected fragment arc."""

    arc = arcs[arc_id]
    if "seq" in arc:
        return tuple(arc["seq"])
    if "Seq" in arc:
        return tuple(arc["Seq"])
    raise KeyError(f"Arc {arc_id!r} has no 'seq' or 'Seq' field; keys={list(arc.keys())}")


def stitch_arc_sid_sequence(arcs: list[dict[str, Any]], route_arc_ids: Iterable[int]) -> tuple[str, ...]:
    """Stitch selected fragment-arc SID sequences into one route SID sequence."""

    sid_sequence: list[str] = []
    for pos, arc_id in enumerate(route_arc_ids):
        seq = fragment_arc_sid_sequence(arcs, arc_id)
        if pos == 0:
            sid_sequence = list(seq)
        elif sid_sequence and seq and sid_sequence[-1] == seq[0]:
            sid_sequence.extend(seq[1:])
        else:
            sid_sequence.extend(seq)
    return tuple(sid_sequence)


def reconstruct_selected_routes(
    arcs: list[dict[str, Any]],
    depot_u: int,
    selected_arc_ids: Iterable[int],
) -> tuple[QueueRoute, ...]:
    """Reconstruct depot-rooted selected routes from incumbent arc ids."""

    selected = list(selected_arc_ids)
    out_map: dict[int, list[int]] = {}
    for arc_id in selected:
        arc = arcs[arc_id]
        out_map.setdefault(int(arc["u"]), []).append(arc_id)

    routes: list[QueueRoute] = []
    for route_id, first_arc_id in enumerate(out_map.get(depot_u, [])):
        route_arc_ids = [first_arc_id]
        cur = int(arcs[first_arc_id]["v"])
        while cur != depot_u:
            next_ids = out_map.get(cur, [])
            if not next_ids:
                break
            next_arc_id = next_ids[0]
            route_arc_ids.append(next_arc_id)
            cur = int(arcs[next_arc_id]["v"])
            if len(route_arc_ids) > len(selected) + 5:
                raise RuntimeError("route reconstruction exceeded selected arc safety limit")
        routes.append(
            QueueRoute(
                route_id=route_id,
                arc_ids=tuple(route_arc_ids),
                sid_sequence=stitch_arc_sid_sequence(arcs, route_arc_ids),
            )
        )
    return tuple(routes)


def _emit_movement_activity(
    *,
    activity_id: int,
    route_id: int,
    route_pos: int,
    arc_id: int,
    arc_pos: int,
    sid_from: str,
    sid_to: str,
    duration: float,
    start_time: float,
) -> QueueActivity:
    return QueueActivity(
        activity_id=activity_id,
        route_id=route_id,
        route_pos=route_pos,
        arc_id=arc_id,
        arc_pos=arc_pos,
        kind="movement",
        sid_from=str(sid_from),
        sid_to=str(sid_to),
        node_sid=None,
        resource_type="route",
        resource_id=f"route:{route_id}",
        duration=float(duration),
        earliest_start=float(start_time),
        latest_start=None,
        base_start=float(start_time),
        base_end=float(start_time) + float(duration),
        slide=None,
        supporting_arc_ids=(int(arc_id),),
    )


def _emit_customer_service_activity(
    *,
    data: dict[str, Any],
    activity_id: int,
    route_id: int,
    route_pos: int,
    arc_id: int,
    arc_pos: int,
    node_sid: str,
    arrival_time: float,
) -> QueueActivity:
    ready, due, service_duration = _node_ready_due_service(data, node_sid)
    service_start = max(float(arrival_time), ready)
    slide = due - service_start
    return QueueActivity(
        activity_id=activity_id,
        route_id=route_id,
        route_pos=route_pos,
        arc_id=arc_id,
        arc_pos=arc_pos,
        kind="service",
        sid_from=None,
        sid_to=None,
        node_sid=str(node_sid),
        resource_type="customer",
        resource_id=customer_physical_resource_id(data, node_sid),
        duration=float(service_duration),
        earliest_start=ready,
        latest_start=due,
        base_start=service_start,
        base_end=service_start + float(service_duration),
        slide=slide,
        supporting_arc_ids=(int(arc_id),),
    )


def extract_route_activities(
    data: dict[str, Any],
    arcs: list[dict[str, Any]],
    route: QueueRoute,
    *,
    start_time: float = 0.0,
    first_activity_id: int = 0,
) -> tuple[tuple[QueueActivity, ...], tuple[str, ...]]:
    """Extract movement and customer-service activities for one route."""

    activities: list[QueueActivity] = []
    ignored_stations: list[str] = []
    t = float(start_time)
    activity_id = int(first_activity_id)
    route_pos = 0

    for arc_pos, arc_id in enumerate(route.arc_ids):
        seq = fragment_arc_sid_sequence(arcs, arc_id)
        if len(seq) < 2:
            continue
        for sid_from, sid_to in zip(seq, seq[1:]):
            move_duration = _travel_time(data, sid_from, sid_to)
            move = _emit_movement_activity(
                activity_id=activity_id,
                route_id=route.route_id,
                route_pos=route_pos,
                arc_id=arc_id,
                arc_pos=arc_pos,
                sid_from=str(sid_from),
                sid_to=str(sid_to),
                duration=move_duration,
                start_time=t,
            )
            activities.append(move)
            activity_id += 1
            route_pos += 1
            t = move.base_end

            if is_station(data, sid_to):
                ignored_stations.append(str(sid_to))
                continue

            if not is_customer(data, sid_to):
                continue

            service = _emit_customer_service_activity(
                data=data,
                activity_id=activity_id,
                route_id=route.route_id,
                route_pos=route_pos,
                arc_id=arc_id,
                arc_pos=arc_pos,
                node_sid=str(sid_to),
                arrival_time=t,
            )
            activities.append(service)
            activity_id += 1
            route_pos += 1
            t = service.base_end

    return tuple(activities), tuple(ignored_stations)


def extract_queue_activities_from_selected_routes(
    data: dict[str, Any],
    arcs: list[dict[str, Any]],
    depot_u: int,
    selected_arc_ids: Iterable[int],
) -> QueueActivityExtraction:
    """Phase A public entry point."""

    routes = reconstruct_selected_routes(arcs, depot_u, selected_arc_ids)
    activities: list[QueueActivity] = []
    ignored_stations: list[str] = []
    next_activity_id = 0
    for route in routes:
        route_activities, route_ignored_stations = extract_route_activities(
            data,
            arcs,
            route,
            first_activity_id=next_activity_id,
        )
        activities.extend(route_activities)
        ignored_stations.extend(route_ignored_stations)
        next_activity_id += len(route_activities)
    return QueueActivityExtraction(
        routes=routes,
        activities=tuple(activities),
        ignored_station_sids=tuple(ignored_stations),
    )


def _activities_by_route(extraction: QueueActivityExtraction) -> dict[int, list[QueueActivity]]:
    out: dict[int, list[QueueActivity]] = {}
    for activity in extraction.activities:
        out.setdefault(activity.route_id, []).append(activity)
    for route_activities in out.values():
        route_activities.sort(key=lambda a: a.route_pos)
    return out


def compute_route_propagated_windows(extraction: QueueActivityExtraction) -> tuple[QueueActivityWindow, ...]:
    """Compute route-propagated latest-start windows for extracted activities.

    This is the first Phase B step. Local customer due times are propagated
    backwards through movement and service activities so each activity receives a
    latest start that respects all downstream customer latest-start constraints
    on its route.
    """

    windows_by_id: dict[int, QueueActivityWindow] = {}
    for route_id, route_activities in _activities_by_route(extraction).items():
        latest_next = INF
        for activity in reversed(route_activities):
            local_latest = activity.latest_start if activity.latest_start is not None else INF
            if isfinite(latest_next):
                propagated_latest = min(float(local_latest), latest_next - float(activity.duration))
            else:
                propagated_latest = float(local_latest)

            if isfinite(propagated_latest):
                propagated_slack = propagated_latest - float(activity.base_start)
                propagated_latest_out: float | None = propagated_latest
            else:
                propagated_slack = None
                propagated_latest_out = None

            windows_by_id[activity.activity_id] = QueueActivityWindow(
                activity_id=activity.activity_id,
                route_id=activity.route_id,
                route_pos=activity.route_pos,
                arc_id=activity.arc_id,
                kind=activity.kind,
                resource_type=activity.resource_type,
                resource_id=activity.resource_id,
                base_start=float(activity.base_start),
                base_end=float(activity.base_end),
                duration=float(activity.duration),
                earliest_start=float(activity.earliest_start),
                local_latest_start=activity.latest_start,
                propagated_latest_start=propagated_latest_out,
                propagated_slack=propagated_slack,
            )
            latest_next = propagated_latest

    return tuple(windows_by_id[k] for k in sorted(windows_by_id))


def _window_lookup(windows: Iterable[QueueActivityWindow]) -> dict[int, QueueActivityWindow]:
    return {w.activity_id: w for w in windows}


def _activity_lookup(extraction: QueueActivityExtraction) -> dict[int, QueueActivity]:
    return {a.activity_id: a for a in extraction.activities}


def _delay_route_suffix(
    *,
    activities_by_id: dict[int, QueueActivity],
    windows_by_id: dict[int, QueueActivityWindow],
    starts: dict[int, float],
    route_id: int,
    from_route_pos: int,
    delay: float,
) -> QueueingConflict | None:
    """Delay an activity and all later activities on the same route.

    Returns a conflict if any finite propagated latest-start window is breached.
    """

    if delay <= EPS:
        return None

    affected = [
        a for a in activities_by_id.values()
        if a.route_id == route_id and a.route_pos >= from_route_pos
    ]
    affected.sort(key=lambda a: a.route_pos)

    for activity in affected:
        candidate_start = starts[activity.activity_id] + float(delay)
        window = windows_by_id[activity.activity_id]
        latest = window.propagated_latest_start
        if latest is not None and candidate_start > latest + EPS:
            return QueueingConflict(
                reason="route_suffix_delay_exceeds_propagated_latest_start",
                resource_id=activity.resource_id,
                activity_ids=(activity.activity_id,),
                arc_ids=tuple(activity.supporting_arc_ids),
                detail=(
                    f"activity_id={activity.activity_id} candidate_start={candidate_start} "
                    f"latest_start={latest} delay={delay}"
                ),
            )

    for activity in affected:
        starts[activity.activity_id] += float(delay)
    return None


def check_queueing_no_overlap(extraction: QueueActivityExtraction) -> QueueingScheduleResult:
    """Run the initial Phase B customer-resource no-overlap checker.

    The checker is deterministic and conservative. It schedules activities at
    their Phase A base starts, then processes customer resources in chronological
    order. When two service intervals overlap at the same physical customer
    resource, it delays the later service activity and propagates that delay to
    the remaining suffix of the same route. If this breaches a propagated route
    latest-start window, the result is infeasible.
    """

    windows = compute_route_propagated_windows(extraction)
    windows_by_id = _window_lookup(windows)
    activities_by_id = _activity_lookup(extraction)
    starts = {a.activity_id: float(a.base_start) for a in extraction.activities}
    conflicts: list[QueueingConflict] = []

    # First, detect route-time infeasibility from propagated windows alone.
    for window in windows:
        if window.propagated_slack is not None and window.propagated_slack < -EPS:
            activity = activities_by_id[window.activity_id]
            conflicts.append(
                QueueingConflict(
                    reason="negative_route_propagated_slack",
                    resource_id=activity.resource_id,
                    activity_ids=(activity.activity_id,),
                    arc_ids=tuple(activity.supporting_arc_ids),
                    detail=(
                        f"activity_id={activity.activity_id} base_start={activity.base_start} "
                        f"propagated_latest_start={window.propagated_latest_start}"
                    ),
                )
            )

    service_by_resource: dict[str, list[QueueActivity]] = {}
    for activity in extraction.activities:
        if activity.kind == "service" and activity.resource_type == "customer" and activity.resource_id is not None:
            service_by_resource.setdefault(activity.resource_id, []).append(activity)

    if not conflicts:
        # Multiple passes are allowed because delaying one resource can create a
        # later conflict at another resource on the same route.
        max_passes = max(1, len(extraction.activities) * 2)
        for _pass in range(max_passes):
            changed = False
            for resource_id in sorted(service_by_resource):
                resource_activities = sorted(
                    service_by_resource[resource_id],
                    key=lambda a: (starts[a.activity_id], a.route_id, a.route_pos, a.activity_id),
                )
                previous: QueueActivity | None = None
                for activity in resource_activities:
                    if previous is None:
                        previous = activity
                        continue
                    previous_end = starts[previous.activity_id] + float(previous.duration)
                    current_start = starts[activity.activity_id]
                    if current_start < previous_end - EPS:
                        required_delay = previous_end - current_start
                        conflict = _delay_route_suffix(
                            activities_by_id=activities_by_id,
                            windows_by_id=windows_by_id,
                            starts=starts,
                            route_id=activity.route_id,
                            from_route_pos=activity.route_pos,
                            delay=required_delay,
                        )
                        if conflict is not None:
                            conflicts.append(
                                QueueingConflict(
                                    reason="customer_no_overlap_delay_infeasible",
                                    resource_id=resource_id,
                                    activity_ids=(previous.activity_id, activity.activity_id),
                                    arc_ids=tuple(sorted(set(previous.supporting_arc_ids + activity.supporting_arc_ids))),
                                    detail=conflict.detail,
                                )
                            )
                            break
                        changed = True
                    previous = activity
                if conflicts:
                    break
            if conflicts or not changed:
                break
        else:
            conflicts.append(
                QueueingConflict(
                    reason="no_overlap_pass_limit_reached",
                    resource_id=None,
                    activity_ids=tuple(),
                    arc_ids=tuple(),
                    detail=f"max_passes={max_passes}",
                )
            )

    scheduled: list[ScheduledQueueActivity] = []
    for activity in sorted(extraction.activities, key=lambda a: a.activity_id):
        scheduled_start = starts[activity.activity_id]
        scheduled.append(
            ScheduledQueueActivity(
                activity_id=activity.activity_id,
                route_id=activity.route_id,
                route_pos=activity.route_pos,
                arc_id=activity.arc_id,
                kind=activity.kind,
                resource_type=activity.resource_type,
                resource_id=activity.resource_id,
                base_start=float(activity.base_start),
                base_end=float(activity.base_end),
                scheduled_start=scheduled_start,
                scheduled_end=scheduled_start + float(activity.duration),
                delay=scheduled_start - float(activity.base_start),
                propagated_latest_start=windows_by_id[activity.activity_id].propagated_latest_start,
                supporting_arc_ids=tuple(activity.supporting_arc_ids),
            )
        )

    infeasible_activity_ids = tuple(sorted({aid for c in conflicts for aid in c.activity_ids}))
    infeasible_arc_ids = tuple(sorted({arc_id for c in conflicts for arc_id in c.arc_ids}))
    total_delay = sum(max(0.0, s.delay) for s in scheduled)
    total_service_delay = sum(max(0.0, s.delay) for s in scheduled if s.kind == "service")

    return QueueingScheduleResult(
        ok=not conflicts,
        total_delay=total_delay,
        total_service_delay=total_service_delay,
        windows=tuple(sorted(windows, key=lambda w: w.activity_id)),
        scheduled_activities=tuple(scheduled),
        conflicts=tuple(conflicts),
        infeasible_activity_ids=infeasible_activity_ids,
        infeasible_arc_ids=infeasible_arc_ids,
    )


def activities_to_jsonable(extraction: QueueActivityExtraction) -> dict[str, Any]:
    """Return a JSON-serialisable view of a Phase A extraction result."""

    return {
        "routes": [asdict(route) for route in extraction.routes],
        "activities": [asdict(activity) for activity in extraction.activities],
        "ignored_station_sids": list(extraction.ignored_station_sids),
    }


def schedule_result_to_jsonable(result: QueueingScheduleResult) -> dict[str, Any]:
    """Return a JSON-serialisable view of a Phase B schedule result."""

    return {
        "ok": result.ok,
        "total_delay": result.total_delay,
        "total_service_delay": result.total_service_delay,
        "windows": [asdict(window) for window in result.windows],
        "scheduled_activities": [asdict(activity) for activity in result.scheduled_activities],
        "conflicts": [asdict(conflict) for conflict in result.conflicts],
        "infeasible_activity_ids": list(result.infeasible_activity_ids),
        "infeasible_arc_ids": list(result.infeasible_arc_ids),
    }

# --- v5f callback-facing queueing evaluator ---

@dataclass(frozen=True)
class QueueingCallbackEvaluation:
    """Compact queueing result intended for callback integration.

    This object deliberately hides the full Phase A/Phase B diagnostic payload and
    exposes only the fields the callback needs for lazy cuts and future queueing
    recourse/optimality work.
    """

    ok: bool
    reason: str
    selected_arc_count: int
    route_count: int
    activity_count: int
    service_count: int
    total_delay: float
    total_service_delay: float
    delayed_activity_count: int
    delayed_service_count: int
    resolved_delay_activity_ids: tuple[int, ...]
    resolved_delay_arc_ids: tuple[int, ...]
    conflict_count: int
    infeasible_activity_ids: tuple[int, ...]
    infeasible_arc_ids: tuple[int, ...]


def evaluate_queueing_for_selected_arcs(
    data: dict[str, Any],
    arcs: list[dict[str, Any]],
    depot_u: int,
    selected_arc_ids: Iterable[int],
) -> QueueingCallbackEvaluation:
    """Evaluate queueing for a selected incumbent arc set.

    This is the callback-facing entry point. It wraps Phase A extraction and the
    initial Phase B no-overlap checker, returning a compact result that can be
    consumed by callback_core.py without exposing queueing internals.

    It does not add cuts, does not call the energy-DP, and does not require a
    model object.
    """

    selected = tuple(int(aid) for aid in selected_arc_ids)
    if not selected:
        return QueueingCallbackEvaluation(
            ok=True,
            reason="no_selected_arcs",
            selected_arc_count=0,
            route_count=0,
            activity_count=0,
            service_count=0,
            total_delay=0.0,
            total_service_delay=0.0,
            delayed_activity_count=0,
            delayed_service_count=0,
            resolved_delay_activity_ids=tuple(),
            resolved_delay_arc_ids=tuple(),
            conflict_count=0,
            infeasible_activity_ids=tuple(),
            infeasible_arc_ids=tuple(),
        )

    extraction = extract_queue_activities_from_selected_routes(
        data=data,
        arcs=arcs,
        depot_u=depot_u,
        selected_arc_ids=selected,
    )

    route_count = len(extraction.routes)
    activity_count = len(extraction.activities)
    service_count = sum(1 for activity in extraction.activities if activity.kind == "service")

    if route_count == 0:
        return QueueingCallbackEvaluation(
            ok=False,
            reason="selected_arcs_but_no_depot_rooted_routes",
            selected_arc_count=len(selected),
            route_count=0,
            activity_count=0,
            service_count=0,
            total_delay=0.0,
            total_service_delay=0.0,
            delayed_activity_count=0,
            delayed_service_count=0,
            resolved_delay_activity_ids=tuple(),
            resolved_delay_arc_ids=tuple(),
            conflict_count=1,
            infeasible_activity_ids=tuple(),
            infeasible_arc_ids=tuple(sorted(set(selected))),
        )

    schedule = check_queueing_no_overlap(extraction)
    delayed = tuple(
        activity for activity in schedule.scheduled_activities
        if activity.delay > EPS
    )
    delayed_services = tuple(activity for activity in delayed if activity.kind == "service")
    resolved_delay_activity_ids = tuple(sorted(activity.activity_id for activity in delayed))
    resolved_delay_arc_ids = tuple(
        sorted({arc_id for activity in delayed for arc_id in activity.supporting_arc_ids})
    )

    if schedule.ok:
        if schedule.total_service_delay > EPS:
            reason = "queueing_feasible_with_delay"
        else:
            reason = "queueing_feasible_no_delay"
    else:
        reason = "queueing_infeasible"

    return QueueingCallbackEvaluation(
        ok=bool(schedule.ok),
        reason=reason,
        selected_arc_count=len(selected),
        route_count=route_count,
        activity_count=activity_count,
        service_count=service_count,
        total_delay=float(schedule.total_delay),
        total_service_delay=float(schedule.total_service_delay),
        delayed_activity_count=len(delayed),
        delayed_service_count=len(delayed_services),
        resolved_delay_activity_ids=resolved_delay_activity_ids,
        resolved_delay_arc_ids=resolved_delay_arc_ids,
        conflict_count=len(schedule.conflicts),
        infeasible_activity_ids=tuple(schedule.infeasible_activity_ids),
        infeasible_arc_ids=tuple(schedule.infeasible_arc_ids),
    )


def queueing_callback_evaluation_to_jsonable(evaluation: QueueingCallbackEvaluation) -> dict[str, Any]:
    """Return a JSON-serialisable callback-facing queueing evaluation."""

    return asdict(evaluation)

# --- end v5f callback-facing queueing evaluator ---

__all__ = [
    "queueing_callback_evaluation_to_jsonable",
    "evaluate_queueing_for_selected_arcs",
    "QueueingCallbackEvaluation",
    "QueueActivity",
    "QueueActivityExtraction",
    "QueueActivityWindow",
    "QueueRoute",
    "QueueingConflict",
    "QueueingScheduleResult",
    "ScheduledQueueActivity",
    "activities_to_jsonable",
    "check_queueing_no_overlap",
    "compute_route_propagated_windows",
    "customer_physical_resource_id",
    "extract_queue_activities_from_selected_routes",
    "extract_route_activities",
    "fragment_arc_sid_sequence",
    "reconstruct_selected_routes",
    "schedule_result_to_jsonable",
    "stitch_arc_sid_sequence",
]

# --- v5h queueing wait metric refinement overrides ---

@dataclass(frozen=True)
class QueueWaitEvent:
    """Queueing wait inserted specifically to clear a customer-resource overlap."""
    resource_id: str
    waiting_activity_id: int
    blocking_activity_id: int
    waiting_arc_id: int
    blocking_arc_id: int
    wait_duration: float
    previous_end: float
    original_start: float
    scheduled_start: float

@dataclass(frozen=True)
class QueueingScheduleResult:
    """Phase B no-overlap scheduling result with explicit queue-wait metrics."""
    ok: bool
    total_delay: float
    total_service_delay: float
    resource_wait_time: float
    resource_wait_event_count: int
    windows: tuple[QueueActivityWindow, ...]
    scheduled_activities: tuple[ScheduledQueueActivity, ...]
    wait_events: tuple[QueueWaitEvent, ...]
    conflicts: tuple[QueueingConflict, ...]
    infeasible_activity_ids: tuple[int, ...]
    infeasible_arc_ids: tuple[int, ...]

@dataclass(frozen=True)
class QueueingCallbackEvaluation:
    """Compact queueing result intended for callback integration."""
    ok: bool
    reason: str
    selected_arc_count: int
    route_count: int
    activity_count: int
    service_count: int
    total_delay: float
    total_service_delay: float
    resource_wait_time: float
    resource_wait_event_count: int
    resolved_overlap_count: int
    delayed_activity_count: int
    delayed_service_count: int
    resolved_delay_activity_ids: tuple[int, ...]
    resolved_delay_arc_ids: tuple[int, ...]
    conflict_count: int
    infeasible_activity_ids: tuple[int, ...]
    infeasible_arc_ids: tuple[int, ...]

def check_queueing_no_overlap(extraction: QueueActivityExtraction) -> QueueingScheduleResult:
    windows = compute_route_propagated_windows(extraction)
    windows_by_id = _window_lookup(windows)
    activities_by_id = _activity_lookup(extraction)
    starts = {a.activity_id: float(a.base_start) for a in extraction.activities}
    conflicts: list[QueueingConflict] = []
    wait_events: list[QueueWaitEvent] = []

    for window in windows:
        if window.propagated_slack is not None and window.propagated_slack < -EPS:
            activity = activities_by_id[window.activity_id]
            conflicts.append(QueueingConflict(
                reason="negative_route_propagated_slack",
                resource_id=activity.resource_id,
                activity_ids=(activity.activity_id,),
                arc_ids=tuple(activity.supporting_arc_ids),
                detail=f"activity_id={activity.activity_id} base_start={activity.base_start} propagated_latest_start={window.propagated_latest_start}",
            ))

    service_by_resource: dict[str, list[QueueActivity]] = {}
    for activity in extraction.activities:
        if activity.kind == "service" and activity.resource_type == "customer" and activity.resource_id is not None:
            service_by_resource.setdefault(activity.resource_id, []).append(activity)

    if not conflicts:
        max_passes = max(1, len(extraction.activities) * 2)
        for _pass in range(max_passes):
            changed = False
            for resource_id in sorted(service_by_resource):
                resource_activities = sorted(
                    service_by_resource[resource_id],
                    key=lambda a: (starts[a.activity_id], a.route_id, a.route_pos, a.activity_id),
                )
                previous: QueueActivity | None = None
                for activity in resource_activities:
                    if previous is None:
                        previous = activity
                        continue
                    previous_end = starts[previous.activity_id] + float(previous.duration)
                    current_start = starts[activity.activity_id]
                    if current_start < previous_end - EPS:
                        required_delay = previous_end - current_start
                        conflict = _delay_route_suffix(
                            activities_by_id=activities_by_id,
                            windows_by_id=windows_by_id,
                            starts=starts,
                            route_id=activity.route_id,
                            from_route_pos=activity.route_pos,
                            delay=required_delay,
                        )
                        if conflict is not None:
                            conflicts.append(QueueingConflict(
                                reason="customer_no_overlap_delay_infeasible",
                                resource_id=resource_id,
                                activity_ids=(previous.activity_id, activity.activity_id),
                                arc_ids=tuple(sorted(set(previous.supporting_arc_ids + activity.supporting_arc_ids))),
                                detail=conflict.detail,
                            ))
                            break
                        wait_events.append(QueueWaitEvent(
                            resource_id=resource_id,
                            waiting_activity_id=activity.activity_id,
                            blocking_activity_id=previous.activity_id,
                            waiting_arc_id=activity.arc_id,
                            blocking_arc_id=previous.arc_id,
                            wait_duration=float(required_delay),
                            previous_end=float(previous_end),
                            original_start=float(current_start),
                            scheduled_start=float(current_start + required_delay),
                        ))
                        changed = True
                    previous = activity
                if conflicts:
                    break
            if conflicts or not changed:
                break
        else:
            conflicts.append(QueueingConflict(
                reason="no_overlap_pass_limit_reached",
                resource_id=None,
                activity_ids=tuple(),
                arc_ids=tuple(),
                detail=f"max_passes={max_passes}",
            ))

    scheduled: list[ScheduledQueueActivity] = []
    for activity in sorted(extraction.activities, key=lambda a: a.activity_id):
        scheduled_start = starts[activity.activity_id]
        scheduled.append(ScheduledQueueActivity(
            activity_id=activity.activity_id,
            route_id=activity.route_id,
            route_pos=activity.route_pos,
            arc_id=activity.arc_id,
            kind=activity.kind,
            resource_type=activity.resource_type,
            resource_id=activity.resource_id,
            base_start=float(activity.base_start),
            base_end=float(activity.base_end),
            scheduled_start=scheduled_start,
            scheduled_end=scheduled_start + float(activity.duration),
            delay=scheduled_start - float(activity.base_start),
            propagated_latest_start=windows_by_id[activity.activity_id].propagated_latest_start,
            supporting_arc_ids=tuple(activity.supporting_arc_ids),
        ))

    infeasible_activity_ids = tuple(sorted({aid for c in conflicts for aid in c.activity_ids}))
    infeasible_arc_ids = tuple(sorted({arc_id for c in conflicts for arc_id in c.arc_ids}))
    total_delay = sum(max(0.0, s.delay) for s in scheduled)
    total_service_delay = sum(max(0.0, s.delay) for s in scheduled if s.kind == "service")
    resource_wait_time = sum(max(0.0, event.wait_duration) for event in wait_events)

    return QueueingScheduleResult(
        ok=not conflicts,
        total_delay=total_delay,
        total_service_delay=total_service_delay,
        resource_wait_time=resource_wait_time,
        resource_wait_event_count=len(wait_events),
        windows=tuple(sorted(windows, key=lambda w: w.activity_id)),
        scheduled_activities=tuple(scheduled),
        wait_events=tuple(wait_events),
        conflicts=tuple(conflicts),
        infeasible_activity_ids=infeasible_activity_ids,
        infeasible_arc_ids=infeasible_arc_ids,
    )

def schedule_result_to_jsonable(result: QueueingScheduleResult) -> dict[str, Any]:
    return {
        "ok": result.ok,
        "total_delay": result.total_delay,
        "total_service_delay": result.total_service_delay,
        "resource_wait_time": result.resource_wait_time,
        "resource_wait_event_count": result.resource_wait_event_count,
        "windows": [asdict(window) for window in result.windows],
        "scheduled_activities": [asdict(activity) for activity in result.scheduled_activities],
        "wait_events": [asdict(event) for event in result.wait_events],
        "conflicts": [asdict(conflict) for conflict in result.conflicts],
        "infeasible_activity_ids": list(result.infeasible_activity_ids),
        "infeasible_arc_ids": list(result.infeasible_arc_ids),
    }

def evaluate_queueing_for_selected_arcs(data: dict[str, Any], arcs: list[dict[str, Any]], depot_u: int, selected_arc_ids: Iterable[int]) -> QueueingCallbackEvaluation:
    selected = tuple(int(aid) for aid in selected_arc_ids)
    if not selected:
        return QueueingCallbackEvaluation(True, "no_selected_arcs", 0, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0, 0, tuple(), tuple(), 0, tuple(), tuple())

    extraction = extract_queue_activities_from_selected_routes(data=data, arcs=arcs, depot_u=depot_u, selected_arc_ids=selected)
    route_count = len(extraction.routes)
    activity_count = len(extraction.activities)
    service_count = sum(1 for activity in extraction.activities if activity.kind == "service")

    if route_count == 0:
        return QueueingCallbackEvaluation(False, "selected_arcs_but_no_depot_rooted_routes", len(selected), 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0, 0, tuple(), tuple(), 1, tuple(), tuple(sorted(set(selected))))

    schedule = check_queueing_no_overlap(extraction)
    delayed = tuple(activity for activity in schedule.scheduled_activities if activity.delay > EPS)
    delayed_services = tuple(activity for activity in delayed if activity.kind == "service")
    resolved_delay_activity_ids = tuple(sorted(activity.activity_id for activity in delayed))
    resolved_delay_arc_ids = tuple(sorted({arc_id for activity in delayed for arc_id in activity.supporting_arc_ids}))

    if schedule.ok:
        reason = "queueing_feasible_with_delay" if schedule.resource_wait_time > EPS else "queueing_feasible_no_delay"
    else:
        reason = "queueing_infeasible"

    return QueueingCallbackEvaluation(
        ok=bool(schedule.ok),
        reason=reason,
        selected_arc_count=len(selected),
        route_count=route_count,
        activity_count=activity_count,
        service_count=service_count,
        total_delay=float(schedule.total_delay),
        total_service_delay=float(schedule.total_service_delay),
        resource_wait_time=float(schedule.resource_wait_time),
        resource_wait_event_count=int(schedule.resource_wait_event_count),
        resolved_overlap_count=int(schedule.resource_wait_event_count),
        delayed_activity_count=len(delayed),
        delayed_service_count=len(delayed_services),
        resolved_delay_activity_ids=resolved_delay_activity_ids,
        resolved_delay_arc_ids=resolved_delay_arc_ids,
        conflict_count=len(schedule.conflicts),
        infeasible_activity_ids=tuple(schedule.infeasible_activity_ids),
        infeasible_arc_ids=tuple(schedule.infeasible_arc_ids),
    )

def queueing_callback_evaluation_to_jsonable(evaluation: QueueingCallbackEvaluation) -> dict[str, Any]:
    return asdict(evaluation)

try:
    if "QueueWaitEvent" not in __all__:
        __all__.append("QueueWaitEvent")
except NameError:
    pass

# --- end v5h queueing wait metric refinement overrides ---
