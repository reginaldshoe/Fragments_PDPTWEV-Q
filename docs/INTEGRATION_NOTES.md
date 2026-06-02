# Integration notes

The integrated package combines v1b fragment logic, v1c master-model logic and v1d callback/DP logic into one `evrp_fragments` namespace.

Remaining transition dependency: `legacy_adapter.py` is still used for `read_instance`, `step` and `enumerate_base_paths` to preserve baseline behaviour until those functions are moved with parity tests.
