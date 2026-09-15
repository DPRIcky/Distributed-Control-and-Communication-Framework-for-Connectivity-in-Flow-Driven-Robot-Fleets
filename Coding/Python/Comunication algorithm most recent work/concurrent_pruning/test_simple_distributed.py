"""Simple test to show distributed coordination phases clearly."""

import sys
from pathlib import Path
parent_dir = Path(__file__).parent.parent
sys.path.insert(0, str(parent_dir))

import numpy as np
from concurrent_pruning.concurrent_pruning_manager import ConcurrentPruningManager

print("=" * 70)
print("DISTRIBUTED COORDINATION PROTOCOL - Simple Demo")
print("=" * 70)

# Setup: 5 robots with redundant edges
num_robots = 5
positions = np.array([[0, 0], [1, 0], [1, 1], [0, 1], [0.5, 0.5]])
edges = {(0, 1), (1, 2), (2, 3), (3, 0), (0, 4), (1, 4), (2, 4), (3, 4)}

print(f"\nSetup: {num_robots} robots, {len(edges)} edges")
print(f"Edges: {sorted(edges)}")

manager = ConcurrentPruningManager(
    num_robots=num_robots,
    mode='lyapunov',
    k_max=30
)
manager.initialize_robot_knowledge(positions, edges.copy())

print("\nRunning distributed pruning...")
print("-" * 70)

edges_working = edges.copy()
pruned_count = 0

for iteration in range(30):
    report = manager.concurrent_step(positions, edges_working, debug=False)
    
    if report['pruned_edge']:
        pruned_count += 1
        print(f"Iter {iteration:2d}: Pruned {report['pruned_edge']} "
              f"(Phase: {report['phase']}, Remaining: {report['edges_after']})")
    
    if len(edges_working) == num_robots - 1:
        break

print("-" * 70)
print(f"\nSummary:")
print(f"  Total iterations: {iteration + 1}")
print(f"  Edges pruned: {pruned_count}")
print(f"  Final edges: {len(edges_working)}")
print(f"  Final topology: {sorted(edges_working)}")

print("\nDistributed Protocol Features:")
print("  [1] Each robot evaluates only INCIDENT edges (local)")
print("  [2] Both endpoints must AGREE to prune (bilateral)")
print("  [3] Conflicts resolved DETERMINISTICALLY (no coordinator)")
print("  [4] Uses consensus estimates A^l(k) (not true adjacency)")
print("\n" + "=" * 70)
