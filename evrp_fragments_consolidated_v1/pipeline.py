"""Canonical consolidated solver pipeline.

Environment:
- Python 3.12
- gurobipy available in the target environment

Role: draft output v1f / canonical runner.

This module is a stable alias over the v1e runtime path. It does not alter
algorithm bodies, callback signatures, DP logic, route extraction or model
formulation. It exists to provide a non-versioned canonical import target while
retaining the versioned v1e implementation for auditability.
"""

from __future__ import annotations

from .pipeline_v1e import ConsolidatedV1eRunSummary, build_fragment_sets, run_solver, summary_to_json

__all__ = [
    "ConsolidatedV1eRunSummary",
    "build_fragment_sets",
    "run_solver",
    "summary_to_json",
]
