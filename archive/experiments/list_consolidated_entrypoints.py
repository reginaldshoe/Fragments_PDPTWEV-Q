"""List consolidated EVRP entry points.

Environment:
- Python 3.12

Role: v1g documentation utility.

This script is informational only. It does not run the solver and does not
modify files.
"""

from __future__ import annotations


def main() -> None:
    print("Canonical consolidated solver:")
    print("  python -m experiments.run_solver_consolidated --instance instances/c101C6_2.txt --max-base-path-len 18 --k-max 1 --force-exact-k --use-callback")
    print()
    print("Canonical consolidated regression gate:")
    print("  python -m experiments.regression_consolidated_c101C6_2")
    print()
    print("Active runtime modules:")
    for path in (
        "evrp_fragments_consolidated_v1/fragment_core.py",
        "evrp_fragments_consolidated_v1/master_core.py",
        "evrp_fragments_consolidated_v1/callback_core.py",
        "evrp_fragments_consolidated_v1/dependency_bridge.py",
        "evrp_fragments_consolidated_v1/pipeline_v1e.py",
        "evrp_fragments_consolidated_v1/pipeline.py",
    ):
        print(f"  {path}")
    print()
    print("Provenance/reference:")
    for path in (
        "ev_fragmentsv3.py",
        "evrp_fragments_consolidated_v1/legacy_core.py",
        "evrp_fragments_parity_v2/",
    ):
        print(f"  {path}")


if __name__ == "__main__":
    main()
