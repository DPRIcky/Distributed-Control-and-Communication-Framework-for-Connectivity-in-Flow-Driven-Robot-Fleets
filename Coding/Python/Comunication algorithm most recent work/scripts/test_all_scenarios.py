"""Test all tuned scenarios to verify parameter choices."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from config.scenarios import get_scenario
from simulation import HybridUnderwaterSimulation


def test_scenario(scenario_id, num_steps=50):
    """Test a single scenario."""
    sim_cfg, ctrl_cfg, cons_cfg, flow_params, name = get_scenario(scenario_id)
    
    print(f'\n{"="*70}')
    print(f'TESTING {name.upper()}')
    print(f'{"="*70}')
    print(f'Robots: {sim_cfg.num_robots}, R_max: {sim_cfg.communication_radius}m, dt: {sim_cfg.dt}s')
    print(f'Control: cbf_conn={ctrl_cfg.cbf_connectivity_gain}, cbf_safety={ctrl_cfg.cbf_safety_gain}')
    print(f'         clf={ctrl_cfg.clf_gain}, max_force={ctrl_cfg.max_control_force}')
    
    sim = HybridUnderwaterSimulation(sim_cfg, ctrl_cfg, cons_cfg)
    
    print(f'\nRunning {num_steps} steps...')
    for i in range(num_steps):
        sim.step()
        if i % 10 == 0:
            lambda2 = sim.metrics.lambda2_values[-1] if sim.metrics.lambda2_values else 0
            edges = len(sim.topology_manager.get_current_edges())
            print(f'  Step {i:2d}: edges={edges:3d}, λ₂={lambda2:.3f}')
    
    # Final summary
    summary = sim.metrics.get_summary_statistics()
    print(f'\n{"─"*70}')
    print(f'RESULTS:')
    print(f'  Initial edges: {summary["initial_edges"]}')
    print(f'  Final edges: {summary["final_edges"]} (reduced by {summary["edge_reduction_pct"]:.1f}%)')
    print(f'  Pruning events: {summary["pruning_events"]}')
    print(f'  λ₂ range: [{summary["min_lambda2"]:.3f}, {summary["mean_lambda2"]:.3f}]')
    print(f'  Graph disconnections: {summary["graph_disconnections"]}')
    print(f'  Safety violations: {summary["safety_violations_total"]}')
    print(f'  Connectivity violations: {summary["connectivity_violations_total"]}')
    
    # Check if passed
    passed = summary["graph_disconnections"] == 0
    status = "✓ PASSED" if passed else "✗ FAILED"
    print(f'\n{status}')
    
    return passed


if __name__ == "__main__":
    print('\n' + '='*70)
    print('TESTING ALL TUNED SCENARIOS')
    print('='*70)
    
    scenarios = ['A', 'B', 'C', 'D', 'E']
    results = {}
    
    for scenario_id in scenarios:
        try:
            passed = test_scenario(scenario_id, num_steps=50)
            results[scenario_id] = passed
        except Exception as e:
            print(f'\n✗ ERROR in Scenario {scenario_id}: {e}')
            results[scenario_id] = False
    
    # Overall summary
    print('\n' + '='*70)
    print('OVERALL SUMMARY')
    print('='*70)
    for scenario_id, passed in results.items():
        status = "✓" if passed else "✗"
        print(f'  Scenario {scenario_id}: {status}')
    
    total_passed = sum(results.values())
    print(f'\nTotal: {total_passed}/{len(scenarios)} scenarios passed')
    
    if total_passed == len(scenarios):
        print('\n🎉 All scenarios working correctly!')
    else:
        print('\n⚠️  Some scenarios need further tuning')
