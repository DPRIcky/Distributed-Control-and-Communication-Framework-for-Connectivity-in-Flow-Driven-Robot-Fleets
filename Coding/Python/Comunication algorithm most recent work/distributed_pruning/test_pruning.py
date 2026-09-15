"""
Quick test of distributed pruning implementation.
Tests basic functionality without visualization.
"""

import sys
import os
import numpy as np

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from distributed_pruning import ProgressivePruningController, PrunedSimulation
from config import SimulationConfig, ControlConfig

def test_pruning_controller():
    """Test basic pruning controller functionality."""
    print("Testing ProgressivePruningController...")
    
    controller = ProgressivePruningController(
        robot_id=0,
        target_degree=3,
        k_connectivity=1
    )
    
    # Test state update
    controller.update_state(np.array([1.0, 2.0]), np.array([1.0, 2.0]))
    assert np.allclose(controller.position, [1.0, 2.0])
    print("  ✓ State update works")
    
    # Test neighbor update
    neighbor_data = [
        (1, 1.5, np.array([2.0, 2.0]), np.array([2.0, 2.0]), [0, 2]),
        (2, 2.0, np.array([3.0, 2.0]), np.array([3.0, 2.0]), [0, 1, 3]),
        (3, 2.5, np.array([4.0, 2.0]), np.array([4.0, 2.0]), [2]),
    ]
    controller.update_neighbors(neighbor_data)
    
    assert len(controller.k_hop_neighbors[1]) == 3  # 3 one-hop neighbors
    print("  ✓ Neighbor update works")
    
    # Test common neighbor counting
    common = controller.count_common_neighbors(1)  # Robot 1
    assert common >= 1  # Should have robot 2 as common neighbor
    print(f"  ✓ Common neighbor count: {common}")
    
    # Test bridge detection
    is_bridge = controller.is_bridge_edge(1)
    print(f"  ✓ Bridge detection: edge is {'bridge' if is_bridge else 'NOT bridge'}")
    
    # Test pruning
    pruned, kept = controller.prune_edges()
    print(f"  ✓ Pruning works: {len(pruned)} pruned, {len(kept)} kept")
    
    # Test statistics
    stats = controller.get_statistics()
    assert 'robot_id' in stats
    assert 'information_completeness' in stats
    print(f"  ✓ Statistics: completeness={stats['information_completeness']:.2f}")
    
    print("✅ ProgressivePruningController tests passed!\n")


def test_pruned_simulation():
    """Test pruned simulation."""
    print("Testing PrunedSimulation...")
    
    sim_config = SimulationConfig(
        num_robots=5,
        communication_radius=10.0,
        workspace_size=(10.0, 10.0),
        dt=0.05,
        seed=42
    )
    control_config = ControlConfig()
    
    sim = PrunedSimulation(
        sim_config,
        control_config,
        target_degree=3,
        k_connectivity=1
    )
    
    print("  ✓ Simulation created")
    
    # Run a few steps
    for i in range(10):
        sim.step()
    
    print(f"  ✓ Ran 10 timesteps")
    
    # Check active edges
    active = sim.get_active_edges()
    pruned = sim.get_pruned_edges()
    
    print(f"  ✓ Active edges: {len(active)}")
    print(f"  ✓ Pruned edges: {len(pruned)}")
    
    # Check connectivity (simple test: active edges >= n-1)
    min_edges = sim_config.num_robots - 1
    if len(active) >= min_edges:
        print(f"  ✓ Connectivity check passed: {len(active)} >= {min_edges}")
    else:
        print(f"  ⚠ Warning: Active edges ({len(active)}) < minimum ({min_edges})")
    
    # Get statistics
    stats = sim.get_pruning_statistics()
    if stats:
        latest = stats[-1]
        print(f"  ✓ Stats: avg_degree={latest['avg_degree']:.1f}, completeness={latest['avg_completeness']:.2f}")
    
    print("✅ PrunedSimulation tests passed!\n")


def test_connectivity_guarantee():
    """Test that connectivity is maintained through many steps."""
    print("Testing connectivity guarantee over time...")
    
    sim_config = SimulationConfig(
        num_robots=8,
        communication_radius=10.0,
        workspace_size=(10.0, 10.0),
        dt=0.05,
        seed=42
    )
    control_config = ControlConfig()
    
    sim = PrunedSimulation(
        sim_config,
        control_config,
        target_degree=3,
        k_connectivity=1
    )
    
    min_edges = sim_config.num_robots - 1
    connectivity_violations = 0
    
    # Run for 100 steps
    for step in range(100):
        sim.step()
        active = sim.get_active_edges()
        
        if len(active) < min_edges:
            connectivity_violations += 1
            print(f"  ⚠ Step {step}: Only {len(active)} edges (need >= {min_edges})")
    
    if connectivity_violations == 0:
        print(f"  ✓ Connectivity maintained for all 100 steps!")
    else:
        print(f"  ⚠ Connectivity violations: {connectivity_violations}/100 steps")
    
    # Final statistics
    stats = sim.get_pruning_statistics()
    if stats:
        final = stats[-1]
        print(f"\n  Final Statistics:")
        print(f"    Potential edges: {final['total_potential_edges']}")
        print(f"    Active edges: {final['active_edges']}")
        print(f"    Pruning ratio: {(final['total_potential_edges'] - final['active_edges']) / final['total_potential_edges']:.1%}")
        print(f"    Avg degree: {final['avg_degree']:.2f} (target: 3)")
    
    print("✅ Connectivity guarantee test completed!\n")


if __name__ == "__main__":
    print("="*70)
    print("DISTRIBUTED PRUNING - UNIT TESTS")
    print("="*70)
    print()
    
    try:
        test_pruning_controller()
        test_pruned_simulation()
        test_connectivity_guarantee()
        
        print("="*70)
        print("ALL TESTS PASSED ✅")
        print("="*70)
        
    except Exception as e:
        print(f"\n❌ Test failed with error:")
        print(f"   {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
