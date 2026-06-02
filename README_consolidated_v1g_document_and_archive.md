# evrp_fragments_consolidated_v1g_document_and_archive

Role: **draft output v1g / documentation and archive markers**.

## Purpose

This overlay documents the current canonical consolidated solver state after the canonical regression gate passed.

It does not change solver logic, move files, delete files, add batch handling, or alter algorithm internals.

## Files added

```text
README_consolidated.md
ARCHIVE_NOTES_consolidated.md
MANIFEST_consolidated_current.json
experiments/list_consolidated_entrypoints.py
README_consolidated_v1g_document_and_archive.md
MANIFEST_consolidated_v1g_document_and_archive.json
```

## Recommended check

After unzipping, you can run:

```powershell
python -m experiments.list_consolidated_entrypoints
```

and confirm the canonical regression still passes:

```powershell
python -m experiments.regression_consolidated_c101C6_2
```
