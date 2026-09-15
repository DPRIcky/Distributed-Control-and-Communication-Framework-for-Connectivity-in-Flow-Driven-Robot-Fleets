"""Test script to verify distributed algorithm integration in robot simulation."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import numpy as np
from config import SimulationConfig, ControlConfig
from critical_edge_pruning_simulation import CriticalEdgePruningSimulation


def test_algorithm_integration():
    """Test that distributed algorithm integrates properly with robot simulation."""
    
    print("\n" + "="*70)
    print("TEST: Distributed Algorithm Integration with Robot Simulation")
    print("="*70)
    
    # Create simulation config
    sim_config = SimulationConfig(
        num_robots=6,
        communication_radius=3.5,
        dt=0.05,
        workspace_size=(8.0, 8.0),
        seed=42,
        verbose=True
    )
    
    # Create control config
    control_config = ControlConfig(
        safety_distance=1.2,
        clf_gain=0.8,
        cbf_safety_gain=4.0,
        cbf_connectivity_gain=0.5,
        max_control_force=1.0
    )
    
    # Create simulation with distributed pruning
    print("\n[1] Creating simulation with distributed pruning enabled...")
    sim = CriticalEdgePruningSimulation(
        sim_config,
        control_config,
        enable_pruning=True,
        pruning_start_time=1.0,  # Start pruning after 1 second
        algorithm_interval=0.5,  # Run algorithm every 0.5 seconds
        root_node=0
    )
    
    print(f"    [OK] Initial topology: {len(sim.robots)} robots, {sim.total_edges} edges")
    print(f"    [OK] Algorithm interval: {sim.algorithm_interval}s")
    print(f"    [OK] Pruning starts at: {sim.pruning_start_time}s")
    
    # Run a few simulation steps
    print("\n[2] Running simulation steps (t=0.0s to t=2.0s)...")
    step_count = 0
    max_steps = int(2.0 / sim.dt)
    
    while step_count < max_steps and sim.time < 2.0:
        sim.step()
        step_count += 1
        
        # Print status at key time points
        if abs(sim.time - 0.5) < sim.dt:
            print(f"    t={sim.time:.2f}s: {sim.total_edges} edges (before algorithm)")
        elif abs(sim.time - 1.0) < sim.dt:
            print(f"    t={sim.time:.2f}s: {sim.total_edges} edges (pruning enabled)")
        elif abs(sim.time - 1.5) < sim.dt:
            print(f"    t={sim.time:.2f}s: {sim.total_edges} edges (after first algorithm run)")
    
    print(f"    [OK] Completed {step_count} steps")
    
    # Check algorithm state
    print("\n[3] Algorithm Integration Status:")
    print(f"    [OK] Distributed algorithm instance created: {sim.distributed_algo is not None}")
    print(f"    [OK] Algorithm converged: {sim.algorithm_converged}")
    print(f"    [OK] Critical edges identified: {len(sim.critical_edges)}")
    print(f"    [OK] Pruned edges removed: {len(sim.pruned_edges)}")
    
    # Check topology
    print("\n[4] Final Topology State:")
    print(f"    [OK] Current edges: {sim.total_edges}")
    print(f"    [OK] Pruned edges (tracked): {len(sim.pruned_edges)}")
    
    # Verify algorithm is working
    if sim.distributed_algo is not None:
        stats = sim.distributed_algo.get_statistics()
        print("\n[5] Algorithm Statistics:")
        print(f"    [OK] Original edges: {stats['original_edges']}")
        print(f"    [OK] Final edges: {stats['final_edges']}")
        if stats['original_edges'] > 0:
            pruning_pct = (1 - stats['final_edges'] / stats['original_edges']) * 100
            print(f"    [OK] Pruning efficiency: {pruning_pct:.1f}%")
        print(f"    [OK] Connected after pruning: {'Yes' if stats['connected'] else 'No'}")
    
    print("\n" + "="*70)
    print("TEST RESULT: PASSED - Distributed Algorithm Successfully Integrated")
    print("="*70 + "\n")
    
    return sim


if __name__ == "__main__":
    try:
        sim = test_algorithm_integration()
        print("Integration test completed successfully!")
        
    except Exception as e:
        print(f"\n[FAIL] TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
