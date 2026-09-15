#!/usr/bin/env python3
"""Test CBF prevents actual disconnection over longer simulation."""

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

def test_no_disconnection():
    """Test that CBF prevents actual disconnection."""
    
    print("=" * 70)
    print("CBF Disconnection Prevention Test (500 steps)")
    print("=" * 70)
    
    # Config with small communication radius
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
    
    print(f"Configuration:")
    print(f"  Robots: {sim_config.num_robots}")
    print(f"  Communication radius: {sim_config.communication_radius}m")
    print(f"  Simulation time: 5 seconds (500 steps @ 0.01s/step)")
    
    # Create simulation
    sim = CriticalEdgeSimulation(
        sim_config=sim_config,
        control_config=control_config,
        update_frequency=10,
        prefer_critical=True,
        verbose=False
    )
    
    constraint_margin = sim_config.communication_radius * 0.95
    disconnections = 0
    network_fragmented = False
    min_mst_edges = float('inf')
    edges_history = []
    
    print(f"\nRunning simulation...")
    
    for step in range(500):
        # Check connectivity before this step
        edge_lengths = sim._get_edge_lengths()
        edges_history.append(len(edge_lengths))
        
        # Check for disconnection (MST requires num_robots-1 edges minimum for connectivity)
        mst_edges = len(sim.baseline.current_mst)
        if mst_edges > 0:
            min_mst_edges = min(min_mst_edges, mst_edges)
        
        # Count MST < n-1 as potential disconnection
        if mst_edges > 0 and mst_edges < sim_config.num_robots - 1:
            disconnections += 1
            if not network_fragmented:
                print(f"\n⚠ Network fragmentation detected at step {step}")
                print(f"  MST size: {mst_edges} (needs {sim_config.num_robots - 1} for connectivity)")
                network_fragmented = True
        
        # Execute step
        sim.step()
        
        if (step + 1) % 100 == 0:
            print(f"  Step {step + 1}/500: {len(edge_lengths)} edges, "
                  f"MST={len(sim.baseline.current_mst)}, "
                  f"pruned={sim.edges_pruned_count}")
    
    # Analysis
    print("\n" + "=" * 70)
    print("RESULTS")
    print("=" * 70)
    
    print(f"\nEdge statistics over simulation:")
    print(f"  Min edges seen: {min(edges_history)}")
    print(f"  Max edges seen: {max(edges_history)}")
    print(f"  Final edges: {len(sim._get_edge_lengths())}")
    print(f"  Final MST size: {len(sim.baseline.current_mst)}")
    print(f"  Min MST edges: {min_mst_edges if min_mst_edges < float('inf') else 'N/A'}")
    
    print(f"\nConnectivity status:")
    print(f"  Total steps: 500")
    print(f"  Steps with MST < {sim_config.num_robots - 1}: {disconnections}")
    print(f"  Network fragmented: {'YES' if network_fragmented else 'NO'}")
    
    final_mst = len(sim.baseline.current_mst)
    required_edges = sim_config.num_robots - 1
    
    if final_mst >= required_edges and not network_fragmented:
        print(f"\n✓ TEST PASSED: Network maintained connectivity over {5}s simulation")
        print(f"  Final MST: {final_mst}/{required_edges} edges needed")
        return True
    else:
        print(f"\n✗ TEST FAILED: Network lost connectivity")
        return False

if __name__ == "__main__":
    try:
        success = test_no_disconnection()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
