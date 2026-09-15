"""Consensus-based adjacency pruning — headless (non-GUI) baseline.

Public API
----------
run_adjacency_consensus_trial  – single simulation trial → dict of metrics
run_batch                      – repeated trials          → list[dict]
"""

from consensus_baseline.engine import run_adjacency_consensus_trial, run_batch

__all__ = ["run_adjacency_consensus_trial", "run_batch"]
