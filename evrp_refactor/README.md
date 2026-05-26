# EV Fragment Solver Refactor

This directory is a behaviour-preserving modular refactor of `ev_fragmentsv3.py`.

The refactor separates the prototype into explicit layers:

1. instance parsing and node/resource helpers;
2. base-path, restricted-fragment and extended-fragment generation;
3. fragment metadata and dominance filtering;
4. master network/model construction;
5. callback route extraction and energy-DP validation; and
6. queueing placeholders for the next research stage.

The first implementation goal is not to change the algorithm. The baseline runner in
`experiments/run_baseline.py` reproduces the original preprocessing and solve flow,
with clearer variable names and standardised comments.
