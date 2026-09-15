"""Batch simulation runner for comprehensive experimental evaluation.

Runs all scenarios × methods × seeds to generate data for ACC 2026 figures.
Results are saved to JSON files for later analysis and visualization.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import json
import time
from datetime import datetime
from config.scenarios import get_scenario
from baselines.baseline_simulation import BaselineSimulation


def run_single_experiment(scenario_name: str, method_name: str, seed: int, 
                         num_steps: int = 100, verbose: bool = False):
    """Run a single experiment configuration.
    
    Args:
        scenario_name: Scenario identifier (A, B, C, D, E)
        method_name: Method name (hybrid, full_graph, etc.)
        seed: Random seed for reproducibility
        num_steps: Number of simulation steps
        verbose: Print detailed progress
        
    Returns:
        Dictionary with results and metadata
    """
    try:
        # Load scenario configuration
        sim_cfg, ctrl_cfg, cons_cfg, flow_params, _ = get_scenario(scenario_name)
        sim_cfg.seed = seed
        
        if verbose:
            print(f"  Running: Scenario {scenario_name}, Method {method_name}, Seed {seed}")
        
        # Create and run simulation
        sim = BaselineSimulation(sim_cfg, ctrl_cfg, cons_cfg, method_name)
        
        start_time = time.time()
        for step in range(num_steps):
            sim.step()
        elapsed_time = time.time() - start_time
        
        # Collect results
        summary = sim.metrics.get_summary_statistics()
        
        result = {
            'scenario': scenario_name,
            'method': method_name,
            'seed': seed,
            'num_steps': num_steps,
            'elapsed_time': elapsed_time,
            'num_robots': sim_cfg.num_robots,
            'communication_radius': sim_cfg.communication_radius,
            'dt': sim_cfg.dt,
            'initial_edges': summary['initial_edges'],
            'final_edges': summary['final_edges'],
            'edge_reduction_pct': summary['edge_reduction_pct'],
            'pruning_events': summary['pruning_events'],
            'min_lambda2': summary['min_lambda2'],
            'mean_lambda2': summary['mean_lambda2'],
            'graph_disconnections': summary['graph_disconnections'],
            'safety_violations': summary['safety_violations_total'],
            'connectivity_violations': summary['connectivity_violations_total'],
            'goal_distance': summary['final_goal_distance_mean'],
            'min_distance_ever': summary['min_distance_ever'],
            'success': summary['graph_disconnections'] == 0
        }
        
        if verbose:
            print(f"    -> Edges: {result['final_edges']}, "
                  f"Pruned: {result['pruning_events']}, "
                  f"Lambda2: {result['min_lambda2']:.3f}, "
                  f"Time: {elapsed_time:.1f}s")
        
        return result
        
    except Exception as e:
        print(f"  ERROR in Scenario {scenario_name}, Method {method_name}, Seed {seed}: {e}")
        import traceback
        traceback.print_exc()
        return {
            'scenario': scenario_name,
            'method': method_name,
            'seed': seed,
            'error': str(e),
            'success': False
        }


def run_batch_experiments(scenarios: list = None, 
                         methods: list = None,
                         seeds: list = None,
                         num_steps: int = 100,
                         output_dir: str = 'results',
                         verbose: bool = True):
    """Run batch of experiments across scenarios, methods, and seeds.
    
    Args:
        scenarios: List of scenario names (default: all A-E)
        methods: List of method names (default: all 5 methods)
        seeds: List of random seeds (default: [42, 123, 456, 789, 1011])
        num_steps: Number of simulation steps per run
        output_dir: Directory to save results
        verbose: Print progress updates
        
    Returns:
        Dictionary with all results
    """
    # Default configurations
    if scenarios is None:
        scenarios = ['A', 'B', 'C', 'D', 'E']
    
    if methods is None:
        methods = ['hybrid', 'full_graph', 'centralized_mst', 'random', 'greedy_distance']
    
    if seeds is None:
        seeds = [42, 123, 456, 789, 1011]
    
    # Create output directory
    output_path = Path(output_dir)
    output_path.mkdir(exist_ok=True)
    
    # Calculate total runs
    total_runs = len(scenarios) * len(methods) * len(seeds)
    
    print("="*70)
    print("BATCH SIMULATION RUNNER")
    print("="*70)
    print(f"Scenarios: {scenarios}")
    print(f"Methods: {methods}")
    print(f"Seeds: {seeds}")
    print(f"Steps per run: {num_steps}")
    print(f"Total experiments: {total_runs}")
    print(f"Output directory: {output_path.absolute()}")
    print("="*70)
    
    # Run experiments
    results = []
    completed = 0
    failed = 0
    start_time = time.time()
    
    for scenario in scenarios:
        print(f"\n--- Scenario {scenario} ---")
        
        for method in methods:
            print(f"  Method: {method}")
            
            for seed in seeds:
                result = run_single_experiment(
                    scenario, method, seed, num_steps, verbose=verbose
                )
                results.append(result)
                
                completed += 1
                if not result.get('success', False):
                    failed += 1
                
                # Progress update
                elapsed = time.time() - start_time
                avg_time = elapsed / completed
                remaining = (total_runs - completed) * avg_time
                
                print(f"    Progress: {completed}/{total_runs} "
                      f"({100*completed/total_runs:.1f}%) | "
                      f"Failed: {failed} | "
                      f"ETA: {remaining/60:.1f} min")
    
    # Save results
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Save full results as JSON
    results_file = output_path / f"batch_results_{timestamp}.json"
    with open(results_file, 'w') as f:
        json.dump({
            'metadata': {
                'timestamp': timestamp,
                'scenarios': scenarios,
                'methods': methods,
                'seeds': seeds,
                'num_steps': num_steps,
                'total_runs': total_runs,
                'completed': completed,
                'failed': failed,
                'elapsed_time': time.time() - start_time
            },
            'results': results
        }, f, indent=2)
    
    print("\n" + "="*70)
    print("BATCH COMPLETE")
    print("="*70)
    print(f"Total runs: {completed}/{total_runs}")
    print(f"Failed: {failed}")
    print(f"Success rate: {100*(completed-failed)/completed:.1f}%")
    print(f"Total time: {(time.time()-start_time)/60:.1f} minutes")
    print(f"Results saved to: {results_file}")
    print("="*70)
    
    return results


def print_summary_statistics(results: list):
    """Print summary statistics grouped by method and scenario.
    
    Args:
        results: List of result dictionaries
    """
    import numpy as np
    
    print("\n" + "="*70)
    print("SUMMARY STATISTICS")
    print("="*70)
    
    # Group by method
    methods = sorted(set(r['method'] for r in results if 'error' not in r))
    scenarios = sorted(set(r['scenario'] for r in results if 'error' not in r))
    
    print("\n--- BY METHOD (averaged across all scenarios and seeds) ---")
    print(f"{'Method':<20} {'Edges':<10} {'Pruned':<10} {'Lambda2':<10} {'Success':<10}")
    print("-"*70)
    
    for method in methods:
        method_results = [r for r in results if r.get('method') == method and 'error' not in r]
        if method_results:
            avg_edges = np.mean([r['final_edges'] for r in method_results])
            avg_pruned = np.mean([r['pruning_events'] for r in method_results])
            avg_lambda2 = np.mean([r['min_lambda2'] for r in method_results])
            success_rate = 100 * sum(r['success'] for r in method_results) / len(method_results)
            
            print(f"{method:<20} {avg_edges:<10.1f} {avg_pruned:<10.1f} "
                  f"{avg_lambda2:<10.3f} {success_rate:<10.1f}%")
    
    print("\n--- BY SCENARIO (averaged across all methods and seeds) ---")
    print(f"{'Scenario':<20} {'Edges':<10} {'Pruned':<10} {'Lambda2':<10} {'Success':<10}")
    print("-"*70)
    
    for scenario in scenarios:
        scenario_results = [r for r in results if r.get('scenario') == scenario and 'error' not in r]
        if scenario_results:
            avg_edges = np.mean([r['final_edges'] for r in scenario_results])
            avg_pruned = np.mean([r['pruning_events'] for r in scenario_results])
            avg_lambda2 = np.mean([r['min_lambda2'] for r in scenario_results])
            success_rate = 100 * sum(r['success'] for r in scenario_results) / len(scenario_results)
            
            print(f"{scenario:<20} {avg_edges:<10.1f} {avg_pruned:<10.1f} "
                  f"{avg_lambda2:<10.3f} {success_rate:<10.1f}%")
    
    print("="*70)


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Run batch simulations')
    parser.add_argument('--scenarios', nargs='+', default=['A', 'B', 'C', 'D', 'E'],
                       help='Scenarios to run (default: all)')
    parser.add_argument('--methods', nargs='+', 
                       default=['hybrid', 'full_graph', 'centralized_mst', 'random', 'greedy_distance'],
                       help='Methods to test (default: all)')
    parser.add_argument('--seeds', nargs='+', type=int, default=[42, 123, 456, 789, 1011],
                       help='Random seeds (default: 5 seeds)')
    parser.add_argument('--steps', type=int, default=100,
                       help='Steps per simulation (default: 100)')
    parser.add_argument('--output', type=str, default='results',
                       help='Output directory (default: results/)')
    parser.add_argument('--quiet', action='store_true',
                       help='Suppress verbose output')
    
    args = parser.parse_args()
    
    # Run batch experiments
    results = run_batch_experiments(
        scenarios=args.scenarios,
        methods=args.methods,
        seeds=args.seeds,
        num_steps=args.steps,
        output_dir=args.output,
        verbose=not args.quiet
    )
    
    # Print summary
    print_summary_statistics(results)
