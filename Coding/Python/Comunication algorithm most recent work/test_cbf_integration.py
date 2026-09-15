#!/usr/bin/env python3
"""Test CBF integration with critical edges for 1m communication radius."""

import sys
import os
import io
import contextlib
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent))

# Suppress warnings
with contextlib.redirect_stdout(io.StringIO()):
    from config import SimulationConfig, ControlConfig
    from Critical_edge_detection_baseline.simple_simulation import CriticalEdgeSimulation

def run_test():
    """Run test with 1m communication radius and verify CBF constraints."""
    
    print("=" * 70)
    print("CBF Integration Test - Critical Edge Constraints")
    print("=" * 70)
    
    # Create configs with small communication radius
    sim_config = SimulationConfig(
        num_robots=8,
        communication_radius=1.0,  # 1 meter - small radius
        workspace_size=[10.0, 10.0],
        dt=0.01,
        verbose=False,
        seed=42
    )
    
    control_config = ControlConfig(
        cbf_connectivity_gain=10.0,
        max_control_force=5.0
    )
    
    print(f"\nConfiguration:")
    print(f"  Robots: {sim_config.num_robots}")
    print(f"  Communication radius: {sim_config.communication_radius}m")
    print(f"  Workspace: {sim_config.workspace_size}")
    print(f"  CBF gain: {control_config.cbf_connectivity_gain}")
    
    # Create simulation
    print("\nInitializing simulation...")
    sim = CriticalEdgeSimulation(
        sim_config=sim_config,
        control_config=control_config,
        update_frequency=10,
        prefer_critical=True,
        verbose=False
    )
    
    print(f"Initial edge count: {len(sim._get_edge_lengths())}")
    
    # Run simulation for 100 steps
    print("\nRunning 100 simulation steps...")
    max_critical_distance = 0.0
    min_critical_distance = float('inf')
    violations = 0
    constraint_margin = sim_config.communication_radius * 0.95  # 95% of comm radius
    
    for step in range(100):
        # Before step, check critical edge distances
        edge_lengths = sim._get_edge_lengths()
        
        # Check distances for edges in critical edges
        if sim.baseline.critical_edges:
            for i, j in sim.baseline.critical_edges:
                if (i, j) in edge_lengths or (j, i) in edge_lengths:
                    dist = edge_lengths.get((i, j), edge_lengths.get((j, i)))
                    max_critical_distance = max(max_critical_distance, dist)
                    min_critical_distance = min(min_critical_distance, dist)
                    
                    # Check if violating constraint
                    if dist > constraint_margin:
                        violations += 1
        
        # Step simulation
        sim.step()
        
        if (step + 1) % 20 == 0:
            print(f"  Step {step + 1}: {len(edge_lengths)} edges, "
                  f"{len(sim.baseline.current_mst)} MST edges, "
                  f"{len(sim.baseline.critical_edges)} critical edges")
    
    # Print results
    print("\n" + "=" * 70)
    print("TEST RESULTS")
    print("=" * 70)
    print(f"Simulation completed: 100 steps")
    print(f"\nCritical Edge Distances:")
    if min_critical_distance < float('inf'):
        print(f"  Min distance: {min_critical_distance:.4f}m")
        print(f"  Max distance: {max_critical_distance:.4f}m")
        print(f"  Constraint margin: {constraint_margin:.4f}m (95% of {sim_config.communication_radius}m)")
        print(f"  Violations: {violations}")
    else:
        print("  No critical edges detected")
    
    print(f"\nFinal Edge Status:")
    print(f"  Total edges: {sim.total_edges}")
    print(f"  MST size: {len(sim.baseline.current_mst)}")
    print(f"  Critical edges: {len(sim.baseline.critical_edges)}")
    print(f"  Edges pruned: {sim.edges_pruned_count}")
    
    sim.print_statistics()
    
    # Determine test status
    if violations == 0 and len(sim.baseline.critical_edges) > 0:
        print("\n✓ TEST PASSED: CBF successfully constrained all critical edges")
        return True
    elif len(sim.baseline.critical_edges) == 0:
        print("\n⚠ WARNING: No critical edges detected (network may have disconnected)")
        return False
    else:
        print(f"\n✗ TEST FAILED: {violations} constraint violations detected")
        return False

if __name__ == "__main__":
    try:
        success = run_test()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"\n✗ TEST ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
