"""Quick debug script to test a single simulation."""

import sys
from pathlib import Path

# Add parent directories to path
root_dir = Path(__file__).parent.parent.parent
sys.path.insert(0, str(root_dir))

import numpy as np
from config.scenarios import get_scenario
from simulation import HybridUnderwaterSimulation
from concurrent_pruning.gui_simulation_concurrent import ConcurrentPruningSimulation
from baselines.baseline_simulation import BaselineSimulation

def test_simulation(method='ConcurrentPruning', scenario='A', max_steps=20):
    """Test a single simulation with debugging output."""
    
    print(f"\n{'='*80}")
    print(f"TESTING: {method} - Scenario {scenario}")
    print(f"{'='*80}\n")
    
    # Get configuration
    sim_config, control_config, consensus_config, flow_params, _ = get_scenario(scenario)
    sim_config.seed = 1000
    
    # Initialize simulation
    if method == 'ConcurrentPruning':
        sim = ConcurrentPruningSimulation(sim_config, control_config, consensus_config, pruning_mode='lyapunov')
    elif method == 'AdjacencyConsensus':
        sim = HybridUnderwaterSimulation(sim_config, control_config, consensus_config)
    elif method == 'FullGraph':
        sim = BaselineSimulation(sim_config, control_config, consensus_config, baseline_method='full_graph')
    elif method == 'CentralizedMST':
        sim = BaselineSimulation(sim_config, control_config, consensus_config, baseline_method='centralized_mst')
    
    # Setup flow
    sim.flow.base_vector = np.array([flow_params['base_flow'], flow_params['base_flow'] * 0.2])
    sim.flow.swirl_amplitude = flow_params['flow_scale']
    
    print(f"Configuration:")
    print(f"  Robots: {sim.num_robots}")
    print(f"  Communication radius: {sim.communication_radius}m")
    print(f"  Timestep: {sim.dt}s")
    print(f"  Goal: {sim.goal_position}")
    
    # Get initial state
    initial_edges = sum(len(neighbors) for neighbors in sim.topology_manager.neighbor_graph) // 2
    print(f"\nInitial state:")
    print(f"  Edges: {initial_edges}")
    
    # Run simulation
    print(f"\nRunning {max_steps} steps...")
    for step in range(max_steps):
        # Execute step
        try:
            step_result = sim.step()
        except Exception as e:
            print(f"\n✗ ERROR at step {step}: {e}")
            import traceback
            traceback.print_exc()
            return False
        
        # Get metrics
        current_edges = sum(len(neighbors) for neighbors in sim.topology_manager.neighbor_graph) // 2
        
        # Get lambda2
        if len(sim.metrics.lambda2_values) > 0:
            lambda2 = sim.metrics.lambda2_values[-1]
        else:
            lambda2 = 0.0
        
        # Get control effort
        if len(sim.metrics.control_magnitudes) > 0:
            total_effort = np.sum(sim.metrics.control_magnitudes[-1])
        else:
            total_effort = 0.0
        
        # Print every 5 steps
        if step % 5 == 0 or step < 3:
            print(f"  Step {step:3d}: edges={current_edges:2d} | λ₂={lambda2:.4f} | control={total_effort:.3f}")
        
        # Check for issues
        if lambda2 < 0.01:
            print(f"\n⚠ WARNING: Low connectivity at step {step} (λ₂={lambda2:.4f})")
            if step > 10:
                print(f"✗ Graph disconnected!")
                return False
    
    # Final state
    final_edges = sum(len(neighbors) for neighbors in sim.topology_manager.neighbor_graph) // 2
    final_lambda2 = sim.metrics.lambda2_values[-1] if sim.metrics.lambda2_values else 0.0
    
    print(f"\nFinal state:")
    print(f"  Edges: {initial_edges} → {final_edges} ({initial_edges - final_edges} pruned)")
    print(f"  Edge reduction: {100*(initial_edges-final_edges)/initial_edges:.1f}%")
    print(f"  Final λ₂: {final_lambda2:.4f}")
    print(f"  Min λ₂: {min(sim.metrics.lambda2_values) if sim.metrics.lambda2_values else 0:.4f}")
    
    success = final_lambda2 >= 0.01
    print(f"\n{'✓ SUCCESS' if success else '✗ FAILED'}")
    
    return success

if __name__ == '__main__':
    methods = ['ConcurrentPruning', 'AdjacencyConsensus', 'FullGraph', 'CentralizedMST']
    
    print("\n" + "="*80)
    print("QUICK DEBUG TEST")
    print("="*80)
    
    results = {}
    for method in methods:
        try:
            results[method] = test_simulation(method, scenario='A', max_steps=50)
        except Exception as e:
            print(f"\n✗ {method} CRASHED: {e}")
            import traceback
            traceback.print_exc()
            results[method] = False
    
    print("\n" + "="*80)
    print("SUMMARY")
    print("="*80)
    for method, success in results.items():
        status = "✓ PASS" if success else "✗ FAIL"
        print(f"  {method:20s}: {status}")
    
    all_passed = all(results.values())
    print(f"\n{'✓ ALL TESTS PASSED' if all_passed else '✗ SOME TESTS FAILED'}")
    print("="*80)
