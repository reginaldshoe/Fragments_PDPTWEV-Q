# Duplication policy

Role: **derived output / cleanup policy**.

Duplicated functions are acceptable temporarily when they are part of a transition adapter. They should not remain in the final implementation unless there is a clear reason.

## Intentional temporary duplicates

- `read_instance`, `step`, and `enumerate_base_paths` may remain delegated to the legacy source until the base-path layer has its own parity test.
- Route utility functions may appear in both legacy and v1d until callback integration is complete.

## Removal order

1. Remove unused diagnostic-only copies.
2. Remove duplicated resource helpers after one canonical resource module is used everywhere.
3. Remove legacy-prefix adapters only after the base-path layer is fully moved and tested.
4. Remove compatibility entry points only after the new package has a stable runner.
