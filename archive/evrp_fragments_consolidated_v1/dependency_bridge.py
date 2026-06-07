"""Dependency bridge for consolidated-v1d.

Environment:
- Python 3.12

Role: draft output v1d / cleanup globals bridge.

This module centralises the remaining cross-module legacy global bindings. It
is deliberately conservative: no algorithm bodies are changed, no callback
factory is introduced, and no model formulation is altered.
"""

from __future__ import annotations

from typing import Any


FRAGMENT_HELPERS_FOR_MASTER: tuple[str, ...] = (
    "is_station",
    "is_customer",
    "is_pickup",
    "is_delivery",
    "compute_T_E_L",
    "compute_distance",
    "compute_Emin",
    "cust_locs",
)

FRAGMENT_HELPERS_FOR_CALLBACK: tuple[str, ...] = (
    "is_station",
    "is_customer",
    "is_pickup",
    "is_delivery",
    "compute_T_E_L",
    "compute_distance",
    "compute_Emin",
    "strip_stations",
)

CALLBACK_RUNTIME_NAMES: tuple[str, ...] = (
    "data",
    "e_frags_aug",
    "arc_by_id",
    "arcs_full",
    "node_id_tmp",
    "depot_u_tmp",
    "theta_tmp",
    "M_tmp",
    "x_tmp",
)

MASTER_RUNTIME_NAMES: tuple[str, ...] = (
    "data",
    "e_frags_aug",
)


def sync_function_globals(module: Any) -> None:
    """Synchronise a mechanically extracted module's functions with its namespace.

    The extracted functions still carry legacy assumptions about module-global
    visibility. This helper keeps that compatibility in one place rather than
    scattering ``__globals__`` updates through runner code.
    """
    namespace = vars(module)
    for value in list(namespace.values()):
        globals_dict = getattr(value, "__globals__", None)
        if isinstance(globals_dict, dict):
            globals_dict.update(namespace)


def bind_fragment_helpers(fragment_module: Any, target_module: Any, names: tuple[str, ...]) -> None:
    """Expose selected fragment helpers to a split target module."""
    for name in names:
        if hasattr(fragment_module, name):
            setattr(target_module, name, getattr(fragment_module, name))


def bind_runtime_names(target_module: Any, values: dict[str, Any], names: tuple[str, ...]) -> None:
    """Bind selected runtime values into a split target module."""
    for name in names:
        if name in values:
            setattr(target_module, name, values[name])


def bind_all_static_dependencies(fragment_module: Any, master_module: Any, callback_module: Any, legacy_module: Any | None = None) -> None:
    """Bind static helper dependencies for all split modules."""
    sync_function_globals(fragment_module)
    bind_fragment_helpers(fragment_module, master_module, FRAGMENT_HELPERS_FOR_MASTER)
    bind_fragment_helpers(fragment_module, callback_module, FRAGMENT_HELPERS_FOR_CALLBACK)
    sync_function_globals(master_module)
    sync_function_globals(callback_module)
    if legacy_module is not None:
        sync_function_globals(legacy_module)


def bind_data_context(data: Any, fragment_module: Any, master_module: Any, callback_module: Any, legacy_module: Any | None = None) -> None:
    """Bind the current instance data to all split modules."""
    for module in (fragment_module, master_module, callback_module):
        setattr(module, "data", data)
        sync_function_globals(module)
    if legacy_module is not None:
        setattr(legacy_module, "data", data)
        sync_function_globals(legacy_module)


def bind_fragment_context(extended_fragments: Any, master_module: Any, callback_module: Any, legacy_module: Any | None = None) -> None:
    """Bind the current extended fragment set to relevant modules."""
    values = {"e_frags_aug": extended_fragments}
    bind_runtime_names(master_module, values, MASTER_RUNTIME_NAMES)
    bind_runtime_names(callback_module, values, CALLBACK_RUNTIME_NAMES)
    sync_function_globals(master_module)
    sync_function_globals(callback_module)
    if legacy_module is not None:
        bind_runtime_names(legacy_module, values, CALLBACK_RUNTIME_NAMES)
        sync_function_globals(legacy_module)


def bind_callback_context(
    *,
    data: Any,
    extended_fragments: Any,
    arcs: Any,
    arc_by_id: Any,
    node_id: Any,
    depot_u: Any,
    theta: Any,
    big_m: Any,
    x_vars: Any,
    callback_module: Any,
    legacy_module: Any | None = None,
) -> None:
    """Bind callback runtime state into the callback module."""
    values = {
        "data": data,
        "e_frags_aug": extended_fragments,
        "arc_by_id": arc_by_id,
        "arcs_full": arcs,
        "node_id_tmp": node_id,
        "depot_u_tmp": depot_u,
        "theta_tmp": theta,
        "M_tmp": big_m,
        "x_tmp": x_vars,
    }
    bind_runtime_names(callback_module, values, CALLBACK_RUNTIME_NAMES)
    sync_function_globals(callback_module)
    if legacy_module is not None:
        bind_runtime_names(legacy_module, values, CALLBACK_RUNTIME_NAMES)
        sync_function_globals(legacy_module)
