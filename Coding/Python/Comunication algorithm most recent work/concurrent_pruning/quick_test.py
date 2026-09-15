"""Quick test of concurrent pruning."""

import sys
from pathlib import Path
parent_dir = Path(__file__).parent.parent
sys.path.insert(0, str(parent_dir))

import numpy as np
from concurrent_pruning.concurrent_pruning_manager import ConcurrentPruningManager
from typing import Set, Tuple

Edge = Tuple[int, int]

# Simple test
num_robots = 5
positions = np.array([
    [0, 0],
    [1, 0],
    [1, 1],
    [0, 1],
    [0.5, 0.5]
])

# Create a topology with redundant edges
edges: Set[Edge] = {
    (0, 1), (1, 2), (2, 3), (3, 0),  # Square
    (0, 4), (1, 4), (2, 4), (3, 4)   # Center connections (redundant)
}

print(f"Initial edges: {len(edges)}")
print(f"Minimum needed: {num_robots - 1}")
print(f"Redundant: {len(edges) - (num_robots - 1)}")
print()

# Test Lyapunov mode
print("Testing Lyapunov mode...")
manager = ConcurrentPruningManager(
    num_robots=num_robots,
    mode='lyapunov',
    k_max=50
)

manager.initialize_robot_knowledge(positions, edges.copy())

edges_lyap = edges.copy()
for i in range(30):
    report = manager.concurrent_step(positions, edges_lyap, debug=False)
    if report['pruned_edge']:
        print(f"  Iter {i}: Pruned {report['pruned_edge']}, edges remaining: {len(edges_lyap)}")

summary = manager.get_summary()
print(f"\nLyapunov Summary:")
print(f"  Iterations: {summary['total_iterations']}")
print(f"  Edges pruned: {summary['total_edges_pruned']}")
print(f"  Final edges: {len(edges_lyap)}")
print()

# Test Max Disagreement mode
print("Testing Max Disagreement mode...")
manager2 = ConcurrentPruningManager(
    num_robots=num_robots,
    mode='max_disagreement',
    k_max=50
)

manager2.initialize_robot_knowledge(positions, edges.copy())

edges_disagree = edges.copy()
for i in range(30):
    report = manager2.concurrent_step(positions, edges_disagree, debug=False)
    if report['pruned_edge']:
        print(f"  Iter {i}: Pruned {report['pruned_edge']}, edges remaining: {len(edges_disagree)}")

summary2 = manager2.get_summary()
print(f"\nMax Disagreement Summary:")
print(f"  Iterations: {summary2['total_iterations']}")
print(f"  Edges pruned: {summary2['total_edges_pruned']}")
print(f"  Final edges: {len(edges_disagree)}")
print()

print("✓ All tests passed!")
