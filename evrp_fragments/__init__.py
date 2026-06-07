"""EVRP fragments package.

Role: promoted canonical runtime package.

This package was promoted from evrp_fragments_consolidated_v1 after the canonical
regression gate passed. The public API is run_solver via pipeline.py.
"""

from .pipeline import run_solver, summary_to_json

__all__ = ["run_solver", "summary_to_json"]
