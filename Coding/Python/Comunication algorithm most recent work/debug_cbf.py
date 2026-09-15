"""Debug CBF integration to see what's happening with critical edges."""

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

def debug_test():
    """Debug test to see critical edges and control forces."""
    
    print("=" * 70)
    print("CBF Integration Debug Test")
    print("=" * 70)
    
    # Create configs with small communication radius
    sim_config = SimulationConfig(
        num_robots=6,
        communication_radius=1.0,
        workspace_size=[10.0, 10.0],
        dt=0.01,
        verbose=False,
        seed=42
    )
    
    control_config = ControlConfig(
        cbf_connectivity_gain=20.0,  # Increased gain for testing
        max_control_force=5.0
    )
    
    # Create simulation
    print(f"\nInitializing simulation with {sim_config.num_robots} robots...")
    sim = CriticalEdgeSimulation(
        sim_config=sim_config,
        control_config=control_config,
        update_frequency=5,  # Update more frequently
        prefer_critical=True,
        verbose=False
    )
    
    print(f"Initial edges: {len(sim._get_edge_lengths())}")
    print(f"Initial MST size: {len(sim.baseline.current_mst)}")
    print(f"Initial critical edges: {len(sim.baseline.critical_edges)}")
    
    # Run first 10 steps with detailed logging
    print("\nRunning 10 simulation steps with detailed logging:")
    print("-" * 70)
    
    for step in range(10):
        # Get edge lengths before step
        edge_lengths_before = sim._get_edge_lengths()
        
        # Log critical edges before control
        print(f"\nStep {step + 1}:")
        print(f"  Edges before: {len(edge_lengths_before)}")
        print(f"  MST size: {len(sim.baseline.current_mst)}")
        print(f"  Critical edges: {sim.baseline.critical_edges}")
        
        # Check if critical edges are being used in control
        if sim.baseline.critical_edges:
            print(f"  Critical edge count: {len(sim.baseline.critical_edges)}")
            for i, j in sim.baseline.critical_edges:
                dist = sim.robots[i].position - sim.robots[j].position
                dist_norm = np.linalg.norm(dist)
                constraint_margin = sim_config.communication_radius * 0.95
                violation = dist_norm > constraint_margin
                print(f"    Edge ({i},{j}): {dist_norm:.4f}m (constraint: {constraint_margin:.4f}m) {'VIOLATION' if violation else 'OK'}")
        else:
            print("  No critical edges yet")
        
        # Execute step
        sim.step()
        
        # Get edge lengths after step
        edge_lengths_after = sim._get_edge_lengths()
        print(f"  Edges after: {len(edge_lengths_after)}")
    
    print("\n" + "=" * 70)
    print("Debug test complete")
    print("=" * 70)

# Import numpy for distance calculations
import numpy as np

if __name__ == "__main__":
    try:
        debug_test()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
