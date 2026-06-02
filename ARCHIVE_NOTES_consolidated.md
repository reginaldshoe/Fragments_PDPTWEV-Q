# Archive notes for consolidated refactor

Role: **archive guidance / prior-output classification**.

## Do not delete yet

Keep the following until a deliberate archive-cleanup step is performed:

```text
ev_fragmentsv3.py
evrp_fragments_parity_v2/
evrp_fragments_consolidated_v1/legacy_core.py
evrp_fragments_consolidated_v1/pipeline_v1a.py
evrp_fragments_consolidated_v1/pipeline_v1b.py
evrp_fragments_consolidated_v1/pipeline_v1c.py
evrp_fragments_consolidated_v1/pipeline_v1d.py
evrp_fragments_consolidated_v1/pipeline_v1e.py
experiments/run_solver_consolidated_v1*.py
experiments/regression_consolidated_v1*.py
MANIFEST_consolidated_v1*.json
README_consolidated_v1*.md
```

These are prior outputs or provenance artefacts. They are useful for auditability while the thesis and benchmark evidence are still active.

## Canonical files to use going forward

Use:

```text
evrp_fragments_consolidated_v1/pipeline.py
experiments/run_solver_consolidated.py
experiments/regression_consolidated_c101C6_2.py
```

## Suggested future archive step

When the repository is safely committed, a later archive step can move prior outputs into a folder such as:

```text
archive/refactor_history/
```

That future archive step should be its own versioned overlay and should include a regression gate afterwards.
