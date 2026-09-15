"""
Demo: Critical Edge Detection Baseline Integration

This demo shows how to use the critical edge detection baseline with
the existing simulation infrastructure.
"""

import sys
import os
import io
import contextlib

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import SimulationConfig, ControlConfig

# Suppress import warnings during initialization (clean environment like GUI simulation)
with contextlib.redirect_stdout(io.StringIO()):
    from Critical_edge_detection_baseline.simple_simulation import CriticalEdgeSimulation


def main():
    """Run a quick demo of the critical edge detection baseline."""
    
    print("=" * 80)
    print("CRITICAL EDGE DETECTION BASELINE - QUICK DEMO")
    print("=" * 80)
    print("\nThis demo shows the distributed MST algorithm in action:")
    print("  - Robots start in a cluster and move toward a goal")
    print("  - Communication graph is pruned to MST using distributed algorithm")
    print("  - Critical edges (bridges) are automatically preserved")
    print("  - O(n) rounds per MST update")
    
    # Configuration
    sim_config = SimulationConfig(
        num_robots=10,
        communication_radius=3.0,
        dt=0.05,
        workspace_size=(10.0, 10.0),
        verbose=False,
        seed=123
    )
    
    control_config = ControlConfig()
    
    # Create simulation
    print("\nInitializing simulation...")
    sim = CriticalEdgeSimulation(
        sim_config=sim_config,
        control_config=control_config,
        update_frequency=10,  # Update MST every 10 steps
        prefer_critical=True,  # Prioritize critical edges
        verbose=False
    )
    
    print(f"  Robots: {sim.num_robots}")
    print(f"  Communication radius: {sim.communication_radius}m")
    print(f"  Expected MST size: {sim.num_robots - 1} edges")
    
    # Run simulation
    print("\nRunning simulation...")
    max_steps = 200
    
    for step in range(max_steps):
        sim.step()
        
        # Print progress every 40 steps
        if step % 40 == 0 and step > 0:
            stats = sim.baseline.get_statistics()
            print(f"  Step {step:3d} | t={sim.time:5.2f}s | "
                  f"Edges: {sim.total_edges:3d} | "
                  f"Pruned: {sim.edges_pruned_count:3d} | "
                  f"MST updates: {stats['mst_updates']:2d}")
    
    # Final statistics
    print("\n" + "=" * 80)
    print("SIMULATION COMPLETE")
    print("=" * 80)
    
    stats = sim.baseline.get_statistics()
    
    print(f"\nFinal State:")
    print(f"  Time: {sim.time:.2f}s")
    print(f"  Current edges: {sim.total_edges}")
    print(f"  Expected (MST): {sim.num_robots - 1}")
    print(f"  Match: {'YES' if sim.total_edges == sim.num_robots - 1 else 'NO'}")
    
    print(f"\nPruning Statistics:")
    print(f"  Initial edges: ~{sim.max_edges}")
    print(f"  Final edges: {sim.total_edges}")
    print(f"  Total pruned: {sim.edges_pruned_count}")
    print(f"  Reduction: {100 * (1 - sim.total_edges / sim.max_edges):.1f}%")
    
    print(f"\nDistributed Algorithm Statistics:")
    print(f"  MST updates: {stats['mst_updates']}")
    print(f"  Total DFS rounds: {stats['total_rounds']}")
    print(f"  Avg rounds/update: {stats['avg_rounds_per_update']:.2f}")
    print(f"  Critical edges detected: {stats['critical_edges_detected']}")
    
    print("\n" + "=" * 80)
    print("KEY INSIGHTS:")
    print("=" * 80)
    print(f"1. Distributed MST converges in O(n) = O({sim.num_robots}) rounds")
    print(f"   Actual avg: {stats['avg_rounds_per_update']:.1f} rounds")
    print(f"2. All critical edges (bridges) are preserved in the MST")
    print(f"3. No centralized oracle needed - fully distributed algorithm")
    print(f"4. Communication reduced by ~{100 * (1 - sim.total_edges / sim.max_edges):.0f}%")
    print("=" * 80)
    
    return sim


if __name__ == "__main__":
    main()
