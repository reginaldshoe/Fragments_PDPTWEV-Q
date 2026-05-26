"""Queueing operation extraction placeholders.

This module is intentionally not wired into the baseline runner yet. It is the
planned landing zone for converting selected fragment routes into service
operations for the request-site queueing subproblem.
"""
from __future__ import annotations


def build_queue_operations(*args, **kwargs):
    """Future hook: build request-site service operations from selected routes."""
    raise NotImplementedError("Queueing operation extraction is planned for the next implementation stage.")
