"""Test baseline methods to verify they work correctly."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from config.scenarios import get_scenario
from baselines.baseline_simulation import BaselineSimulation


def test_baseline(method_name: str, num_steps: int = 100):
    """Test a single baseline method.
    
    Args:
        method_name: Name of the method to test
        num_steps: Number of simulation steps (default 100 for comprehensive results)
    """
    print(f'\n{"="*70}')
    print(f'TESTING: {method_name.upper()}')
    print(f'{"="*70}')
    
    # Use Scenario A (baseline)
    sim_cfg, ctrl_cfg, cons_cfg, flow_params, _ = get_scenario('A')
    
    # Create simulation with baseline method
    sim = BaselineSimulation(sim_cfg, ctrl_cfg, cons_cfg, baseline_method=method_name)
    
    print(f'Method: {sim.get_method_name()}')
    print(f'Robots: {sim.num_robots}, R_max: {sim.communication_radius}m')
    
    # Run simulation
    print(f'\nRunning {num_steps} steps...')
    for i in range(num_steps):
        sim.step()
        if i % 10 == 0:
            lambda2 = sim.metrics.lambda2_values[-1] if sim.metrics.lambda2_values else 0
            edges = len(sim.topology_manager.get_current_edges())
            print(f'  Step {i:2d}: edges={edges:3d}, lambda2={lambda2:.3f}')
    
    # Summary
    summary = sim.metrics.get_summary_statistics()
    print(f'\n{"-"*70}')
    print(f'RESULTS:')
    print(f'  Initial edges: {summary["initial_edges"]}')
    print(f'  Final edges: {summary["final_edges"]}')
    print(f'  Edge reduction: {summary["edge_reduction_pct"]:.1f}%')
    print(f'  Pruning events: {summary["pruning_events"]}')
    print(f'  lambda2 range: [{summary["min_lambda2"]:.3f}, {summary["mean_lambda2"]:.3f}]')
    print(f'  Graph disconnections: {summary["graph_disconnections"]}')
    
    passed = summary["graph_disconnections"] == 0
    status = "[PASS]" if passed else "[FAIL]"
    print(f'\n{status}')
    
    return passed, summary


if __name__ == "__main__":
    print('\n' + '='*70)
    print('TESTING ALL BASELINE METHODS')
    print('='*70)
    
    methods = [
        'hybrid',           # Your method
        'full_graph',       # No pruning
        'centralized_mst',  # Oracle
        'random',           # Naive
        'greedy_distance'   # Simple heuristic
    ]
    
    results = {}
    
    for method in methods:
        try:
            passed, summary = test_baseline(method, num_steps=100)
            results[method] = {
                'passed': passed,
                'final_edges': summary['final_edges'],
                'pruning_events': summary['pruning_events'],
                'min_lambda2': summary['min_lambda2'],
                'disconnections': summary['graph_disconnections']
            }
        except Exception as e:
            import traceback
            traceback.print_exc()
            results[method] = {'passed': False, 'error': str(e)}
    
    # Comparison table
    print('\n' + '='*70)
    print('COMPARISON SUMMARY')
    print('='*70)
    print(f'{"Method":<20} {"Edges":<8} {"Pruned":<8} {"Lambda2":<8} {"Status":<8}')
    print('-'*70)
    
    for method, result in results.items():
        if 'error' in result:
            print(f'{method:<20} ERROR: {result["error"]}')
        else:
            edges = result['final_edges']
            pruned = result['pruning_events']
            lambda2 = result['min_lambda2']
            status = "[OK]" if result['passed'] else "[FAIL]"
            print(f'{method:<20} {edges:<8} {pruned:<8} {lambda2:<8.3f} {status:<8}')
    
    print('\n' + '='*70)
