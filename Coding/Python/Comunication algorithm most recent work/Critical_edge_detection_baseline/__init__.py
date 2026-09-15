"""Critical Edge Detection Baseline Package.

This package provides a distributed algorithm for MST construction with
critical edge (bridge) preservation for underwater robot communication.

Modules:
- distributed_critical_mst: Core distributed MST algorithm
- critical_edge_baseline: Baseline class for simulation integration
- critical_edge_simulation: Simulation runner with robot dynamics
- test_critical_baseline: Comprehensive test suite
- simple_test_mst: Simple standalone tests with GUI

Main Classes:
- DistributedCriticalMST: Core algorithm implementation
- CriticalEdgeBaseline: Baseline for pruning integration
- CriticalEdgeSimulation: Complete simulation runner

Quick Start:
    from Critical_edge_detection_baseline import CriticalEdgeBaseline
    
    baseline = CriticalEdgeBaseline(num_robots=10)
    edge_to_prune = baseline.find_edge_to_prune(edge_lengths)
"""

from .distributed_critical_mst import (
    DistributedCriticalMST,
    build_mst,
    detect_and_prune,
    get_mst
)

from .critical_edge_baseline import (
    CriticalEdgeBaseline,
    get_critical_edge_mst
)

__version__ = "1.0.0"
__author__ = "Distributed Systems Research"
__all__ = [
    # Core algorithm
    'DistributedCriticalMST',
    'build_mst',
    'detect_and_prune',
    'get_mst',
    # Baseline
    'CriticalEdgeBaseline',
    'get_critical_edge_mst',
]

# Optional imports (may not be available in standalone mode)
try:
    from .critical_edge_simulation import CriticalEdgeSimulation
    __all__.append('CriticalEdgeSimulation')
except ImportError:
    pass
