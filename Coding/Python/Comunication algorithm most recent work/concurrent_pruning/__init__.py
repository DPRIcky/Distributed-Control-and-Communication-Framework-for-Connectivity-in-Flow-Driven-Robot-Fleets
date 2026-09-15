"""Concurrent Consensus + Pruning with Lyapunov Constraints.

This module implements distributed concurrent consensus and topology pruning
where edge removal happens DURING consensus convergence (not after).

Key novelty: Lyapunov-constrained pruning ensures that removing edges does not
disrupt consensus convergence, enabling simultaneous operation with provable
stability guarantees.

Modules:
    local_lyapunov: Local Lyapunov function computation (distributed)
    max_disagreement: Max disagreement metric computation (distributed)
    adaptive_thresholds: Conservative-to-aggressive threshold scheduling
    concurrent_pruning_manager: Main orchestrator for concurrent operation
"""

from .local_lyapunov import LocalLyapunovMonitor
from .max_disagreement import MaxDisagreementMonitor
from .adaptive_thresholds import AdaptiveThresholdScheduler
from .concurrent_pruning_manager import ConcurrentPruningManager

__all__ = [
    'LocalLyapunovMonitor',
    'MaxDisagreementMonitor',
    'AdaptiveThresholdScheduler',
    'ConcurrentPruningManager',
]

__version__ = '1.0.0'
