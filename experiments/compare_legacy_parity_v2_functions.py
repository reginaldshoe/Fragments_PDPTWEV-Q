"""Audit which legacy functions parity-v2 can mechanically load."""
from __future__ import annotations
import argparse, json
from evrp_fragments_parity_v2.legacy_loader import DEFAULT_FUNCTIONS, load_legacy

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Audit parity-v2 legacy function loading.")
    p.add_argument("--legacy", default="ev_fragmentsv3.py")
    return p.parse_args()

def main() -> None:
    a = parse_args()
    loaded = load_legacy(a.legacy, DEFAULT_FUNCTIONS)
    payload = {"source": str(loaded.source_path), "source_sha256": loaded.source_sha256, "requested_count": len(DEFAULT_FUNCTIONS), "loaded_count": len(loaded.loaded_functions), "loaded_functions": list(loaded.loaded_functions), "missing_requested_functions": list(loaded.missing_requested_functions), "function_hashes": dict(loaded.function_hashes)}
    print(json.dumps(payload, indent=2, sort_keys=True))
    if loaded.missing_requested_functions:
        raise SystemExit(2)

if __name__ == "__main__":
    main()
