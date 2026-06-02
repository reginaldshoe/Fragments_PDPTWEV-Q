"""Canonical integrated EVRP fragment package."""
from .data import read_instance
from .fragments import build_fragment_sets

__all__ = ["read_instance", "build_fragment_sets"]
