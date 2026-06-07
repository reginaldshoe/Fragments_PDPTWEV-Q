"""Cleanup manifest for the staged refactor.

This module records the intended role of each refactor stage and the expected
canonical landing zones for major function families.
"""
from __future__ import annotations

EXPECTED_REFACTOR_PACKAGES = [
    'refactor_v1a_baseline_freeze',
    'refactor_v1b_fragment_pipeline',
    'refactor_v1c_master_model',
    'refactor_v1d_callback_dp',
]

CANONICAL_FUNCTION_GROUPS = {
    'baseline_freeze': ['copy_legacy_source', 'sha256_file', 'run_legacy_script'],
    'fragment_pipeline': [
        'enumerate_fragments',
        'trim_base_path',
        'extend_all_fragments',
        'attach_metadata',
        'dominance_filter',
        'build_fragment_sets',
    ],
    'master_model': ['raw_depot_arcs', 'build_network', 'make_gurobi_env', 'build_master_model'],
    'callback_dp': [
        'dp_leg_frontier_charge_to_full',
        'dp_route_min_dist',
        'extract_routes_from_solution',
        'stitch_sid_sequence',
        'make_energy_callback',
    ],
}

TRANSITION_ADAPTER_FUNCTIONS = {
    'read_instance',
    'step',
    'enumerate_base_paths',
    'load_legacy_namespace',
    'get_legacy_function',
}
