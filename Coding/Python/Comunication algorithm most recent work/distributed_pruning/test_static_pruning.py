"""
Test: Static Pruning (No Movement)

Tests if distributed pruning works correctly when robots are STATIONARY.
This isolates the pruning algorithm from geometric/CBF complications.
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from config import SimulationConfig, ControlConfig
from distributed_pruning import PrunedSimulation

def test_static_pruning():
    """Test pruning with stationary robots."""
    
    print("="*70)
    print("STATIC PRUNING TEST (No Robot Movement)")
    print("="*70)
    
    # Configuration
    sim_config = SimulationConfig(
        num_robots=6,
        communication_radius=1.0,
        workspace_size=(10.0, 10.0),
        dt=0.05,
        seed=42
    )
    
    control_config = ControlConfig()
    
    # Create simulation
    sim = PrunedSimulation(
        sim_config,
        control_config,
        target_degree=2,
        k_connectivity=1
    )
    
    # Fix robot positions in a LINE (ensure all connected)
    # Place 6 robots 0.8m apart (well within 1m radius)
    print("\n[Fixed Robot Positions]")
    for i in range(6):
        sim.robots[i].position = np.array([2.0 + i * 0.8, 5.0])
        sim.robots[i].velocity = np.zeros(2)
        print(f"  Robot {i}: ({sim.robots[i].position[0]:.1f}, {sim.robots[i].position[1]:.1f})")
    
    # Check initial connectivity
    print("\n[Initial Connectivity]")
    initial_edges = []
    for i in range(6):
        for j in range(i+1, 6):
            dist = np.linalg.norm(sim.robots[i].position - sim.robots[j].position)
            if dist <= 1.0:
                initial_edges.append((i, j, dist))
                print(f"  Edge ({i},{j}): {dist:.2f}m")
    
    print(f"\n  Total initial edges: {len(initial_edges)}")
    print(f"  Minimum required: {sim_config.num_robots - 1}")
    
    # Run pruning for many iterations WITHOUT robot movement
    print("\n[Running Pruning - Robots FIXED]")
    print("  (Robots do NOT move - testing pruning algorithm only)")
    
    for step in range(300):  # 15 seconds
        # Update pruning only
        sim.update_pruning()
        
        # DO NOT call sim.step() - that would move robots
        # Just update time
        sim.time += sim.dt
        
        # Print progress
        if step % 100 == 0:
            stats = sim.get_pruning_statistics()
            if stats:
                s = stats[-1]
                print(f"\n  [t={s['time']:.1f}s] Step {step}/300")
                print(f"    Potential edges: {s['total_potential_edges']}")
                print(f"    Active edges: {s['active_edges']}")
                print(f"    Info completeness: {s['avg_completeness']:.0%}")
                print(f"    Avg degree: {s['avg_degree']:.1f}")
    
    # Final results
    print("\n" + "="*70)
    print("FINAL RESULTS (After 15s of pruning)")
    print("="*70)
    
    stats = sim.get_pruning_statistics()
    final = stats[-1]
    
    print(f"\n  Potential edges: {final['total_potential_edges']}")
    print(f"  Active edges: {final['active_edges']}")
    print(f"  Pruned edges: {final['total_potential_edges'] - final['active_edges']}")
    print(f"  Minimum required: {sim_config.num_robots - 1}")
    
    # Check connectivity
    print(f"\n[Connectivity Check]")
    if final['active_edges'] >= sim_config.num_robots - 1:
        print(f"  ✓ PASSED: {final['active_edges']} >= {sim_config.num_robots - 1}")
        print(f"  Algorithm correctly maintained connectivity!")
    else:
        print(f"  ✗ FAILED: {final['active_edges']} < {sim_config.num_robots - 1}")
        print(f"  Algorithm BUG - disconnected graph!")
    
    # Show which edges are active
    print(f"\n[Active Edges]")
    for i in range(6):
        controller = sim.pruning_controllers[i]
        neighbors = controller.get_active_edges()
        print(f"  Robot {i}: neighbors = {sorted(neighbors)}")
    
    print("\n" + "="*70)
    
    # Verify positions didn't change
    print("\n[Verification: Positions Unchanged?]")
    for i in range(6):
        expected = np.array([2.0 + i * 0.8, 5.0])
        actual = sim.robots[i].position
        diff = np.linalg.norm(actual - expected)
        print(f"  Robot {i}: diff = {diff:.6f}m (should be ~0)")
        if diff > 0.01:
            print(f"    WARNING: Robot moved! Test invalid.")


if __name__ == "__main__":
    test_static_pruning()
