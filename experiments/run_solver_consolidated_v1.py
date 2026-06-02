"""Run consolidated-v1 solver from the repository root."""

from __future__ import annotations

import argparse

from evrp_fragments_consolidated_v1.pipeline import run_solver, summary_to_json


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run consolidated-v1 EVRP solver.")
    parser.add_argument("--instance", required=True)
    parser.add_argument("--max-base-path-len", type=int, required=True)
    parser.add_argument("--k-max", type=int, required=True)
    parser.add_argument("--force-exact-k", action="store_true")
    parser.add_argument("--use-callback", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    summary = run_solver(
        instance=args.instance,
        max_base_path_len=args.max_base_path_len,
        k_max=args.k_max,
        force_exact_k=args.force_exact_k,
        use_callback=args.use_callback,
    )
    print(summary_to_json(summary))


if __name__ == "__main__":
    main()
