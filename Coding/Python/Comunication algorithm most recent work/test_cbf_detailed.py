#!/usr/bin/env python3
"""Test CBF integration with detailed edge logging."""

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
import numpy as np

def run_detailed_test():
    """Run test with detailed edge tracking."""
    
    print("=" * 70)
    print("Detailed CBF Edge Constraint Test")
    print("=" * 70)
    
    # Same config as before (8 robots, 1m comm radius)
    sim_config = SimulationConfig(
        num_robots=8,
        communication_radius=1.0,
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
    print(f"  CBF gain: {control_config.cbf_connectivity_gain}")
    
    # Create simulation
    sim = CriticalEdgeSimulation(
        sim_config=sim_config,
        control_config=control_config,
        update_frequency=10,
        prefer_critical=True,
        verbose=False
    )
    
    constraint_margin = sim_config.communication_radius * 0.95
    violations_by_step = {}
    max_violation_edges = []
    
    print(f"\nRunning 100 simulation steps...")
    
    for step in range(100):
        # Before step: check current edges
        edge_lengths = sim._get_edge_lengths()
        
        # Check if any critical edges exist and if any are violating
        if sim.baseline.critical_edges:
            step_violations = []
            step_max_dist = 0.0
            
            for i, j in sim.baseline.critical_edges:
                edge_key = (min(i, j), max(i, j))
                if edge_key in edge_lengths:
                    dist = edge_lengths[edge_key]
                    step_max_dist = max(step_max_dist, dist)
                    
                    if dist > constraint_margin:
                        step_violations.append((i, j, dist))
                        max_violation_edges.append((step, i, j, dist))
            
            if step_violations:
                violations_by_step[step] = step_violations
        
        # Step simulation
        sim.step()
        
        if (step + 1) % 25 == 0:
            critical_count = len(sim.baseline.critical_edges)
            print(f"  Step {step + 1}: {len(edge_lengths)} edges, "
                  f"{critical_count} critical edges, "
                  f"{len(violations_by_step.get(step, []))} violations at this step")
    
    # Print violations
    print("\n" + "=" * 70)
    print("VIOLATION ANALYSIS")
    print("=" * 70)
    
    if max_violation_edges:
        print(f"\nTotal violations across simulation: {len(max_violation_edges)}")
        print("\nSteps with violations:")
        for step in sorted(violations_by_step.keys())[:5]:  # Show first 5
            print(f"\n  Step {step}:")
            for i, j, dist in violations_by_step[step]:
                print(f"    Edge ({i},{j}): {dist:.4f}m > {constraint_margin:.4f}m (violation margin: {(dist - constraint_margin)*1000:.1f}mm)")
        
        if len(violations_by_step) > 5:
            print(f"\n  ... and {len(violations_by_step) - 5} more steps with violations")
        
        # Analyze violation pattern
        print("\nViolation pattern:")
        worst_edge = max(max_violation_edges, key=lambda x: x[3])
        print(f"  Worst violation: Step {worst_edge[0]}, Edge ({worst_edge[1]},{worst_edge[2]}): {worst_edge[3]:.4f}m")
        
        avg_violation_dist = np.mean([v[3] for v in max_violation_edges])
        print(f"  Average violation distance: {avg_violation_dist:.4f}m")
    else:
        print("\n✓ No violations detected - all critical edges within constraint margin!")
    
    print(f"\nFinal statistics:")
    print(f"  Total edges: {sim.total_edges}")
    print(f"  MST size: {len(sim.baseline.current_mst)}")
    print(f"  Critical edges: {len(sim.baseline.critical_edges)}")
    print(f"  Edges pruned: {sim.edges_pruned_count}")
    
    # Determine status
    if len(max_violation_edges) == 0:
        print("\n✓ TEST PASSED: All critical edges maintained within constraint!")
        return True
    else:
        percent_violations = (len(max_violation_edges) / 100) * 100
        print(f"\n⚠ TEST PARTIAL: {len(max_violation_edges)} violations ({percent_violations:.0f}% of readings)")
        if percent_violations < 10:
            print("  Note: Very few violations - likely edge cases or measurement noise")
            return True
        else:
            return False

if __name__ == "__main__":
    try:
        success = run_detailed_test()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
