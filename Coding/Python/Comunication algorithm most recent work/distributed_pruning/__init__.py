"""
Distributed Graph Pruning with Formal Connectivity Guarantees

This module implements provably safe distributed edge pruning for multi-robot systems.
Uses only local 2-hop information to maintain global connectivity while creating
sparse communication topologies.

Key Features:
- Formal proof of connectivity preservation
- Progressive pruning with increasing k-hop knowledge
- O(d²) computation per robot (d = max degree)
- Fully distributed (no central coordinator)
"""

from .pruning_controller import ProgressivePruningController
from .pruned_simulation import PrunedSimulation

__all__ = [
    'ProgressivePruningController',
    'PrunedSimulation',
]
