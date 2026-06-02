"""Run the parity-v2 harness from the repository root."""
from __future__ import annotations
import argparse
from evrp_fragments_parity_v2.pipeline import run_parity_solver, summary_to_json

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run parity-v2 EVRP harness.")
    p.add_argument("--source", default="ev_fragmentsv3.py")
    p.add_argument("--instance", required=True)
    p.add_argument("--max-base-path-len", type=int, required=True)
    p.add_argument("--k-max", type=int, required=True)
    p.add_argument("--force-exact-k", action="store_true")
    p.add_argument("--use-callback", action="store_true")
    return p.parse_args()

def main() -> None:
    a = parse_args()
    s = run_parity_solver(a.source, a.instance, a.max_base_path_len, a.k_max, a.force_exact_k, a.use_callback)
    print(summary_to_json(s))

if __name__ == "__main__":
    main()
