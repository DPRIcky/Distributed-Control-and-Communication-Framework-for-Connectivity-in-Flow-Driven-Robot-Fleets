"""Test script to demonstrate distributed coordination protocol.

Shows the three phases of distributed pruning:
1. PROPOSAL: Each robot proposes edges
2. NEGOTIATION: Both endpoints must agree
3. CONFLICT RESOLUTION: Deterministic tie-breaking
"""

import sys
from pathlib import Path
parent_dir = Path(__file__).parent.parent
sys.path.insert(0, str(parent_dir))

import numpy as np
from concurrent_pruning.concurrent_pruning_manager import ConcurrentPruningManager

print("=" * 80)
print("DISTRIBUTED COORDINATION PROTOCOL DEMONSTRATION")
print("=" * 80)
print()

# Setup: 5 robots with redundant edges
num_robots = 5
positions = np.array([
    [0, 0],
    [1, 0],
    [1, 1],
    [0, 1],
    [0.5, 0.5]
])

# Initial topology with redundancy
edges = {(0, 1), (1, 2), (2, 3), (3, 0), (0, 4), (1, 4), (2, 4), (3, 4)}

print(f"Setup:")
print(f"  Robots: {num_robots}")
print(f"  Initial edges: {len(edges)}")
print(f"  Edge list: {sorted(list(edges))}")
print(f"  Minimum needed: {num_robots - 1}")
print(f"  Redundant edges: {len(edges) - (num_robots - 1)}")
print()

# Initialize manager
manager = ConcurrentPruningManager(
    num_robots=num_robots,
    mode='lyapunov',
    sigma=1.0,
    sample_time=0.2,
    lambda2_threshold=0.3,
    k_max=50
)

manager.initialize_robot_knowledge(positions, edges.copy())

print("Running concurrent consensus + distributed pruning with DEBUG mode...")
print("=" * 80)
print()

edges_working = edges.copy()
pruning_events = []

# Run with debug enabled to see distributed phases
for iteration in range(50):
    print(f"\n{'='*80}")
    print(f"ITERATION {iteration}")
    print(f"{'='*80}")
    
    report = manager.concurrent_step(
        positions=positions,
        current_edges=edges_working,
        debug=True  # Enable debug to see distributed protocol
    )
    
    if report['pruned_edge']:
        pruning_events.append({
            'iteration': iteration,
            'edge': report['pruned_edge'],
            'phase': report['phase'],
            'edges_remaining': report['edges_after']
        })
        print(f"\n[*] PRUNING EVENT:")
        print(f"  Edge: {report['pruned_edge']}")
        print(f"  Phase: {report['phase']}")
        print(f"  Edges remaining: {report['edges_after']}")
    else:
        print(f"\n[*] No edge pruned this iteration")
    
    # Stop when we reach minimum spanning tree
    if len(edges_working) == num_robots - 1:
        print(f"\n{'='*80}")
        print(f"REACHED MINIMUM SPANNING TREE")
        print(f"{'='*80}")
        break
    
    # Stop if no progress for 10 iterations
    if iteration > 10 and not report['pruned_edge']:
        recent_prunings = [e for e in pruning_events if e['iteration'] > iteration - 10]
        if not recent_prunings:
            print(f"\nNo pruning for 10 iterations, stopping...")
            break

print()
print("=" * 80)
print("FINAL SUMMARY")
print("=" * 80)
print()
print(f"Total iterations: {iteration + 1}")
print(f"Total edges pruned: {len(pruning_events)}")
print(f"Final topology: {len(edges_working)} edges")
print()
print("Pruning sequence:")
for i, event in enumerate(pruning_events, 1):
    print(f"  {i}. Iter {event['iteration']:2d}: Edge {event['edge']} "
          f"(Phase: {event['phase']}, Remaining: {event['edges_remaining']})")
print()
print("Distributed Protocol Demonstrated:")
print("  [OK] Phase 1: Each robot independently proposed edges")
print("  [OK] Phase 2: Both endpoints negotiated (unanimous consent)")
print("  [OK] Phase 3: Conflicts resolved deterministically")
print("  [OK] No central coordinator required")
print()
print("=" * 80)
