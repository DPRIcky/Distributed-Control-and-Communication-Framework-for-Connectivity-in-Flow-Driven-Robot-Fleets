"""Batch simulation runner including concurrent pruning and baselines.

Runs all scenarios x methods x seeds to generate data for ACC 2026-style figures
with ConcurrentPruning as the method under test.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import json
import time
from datetime import datetime
from config.scenarios import get_scenario
from baselines.baseline_simulation import BaselineSimulation
from concurrent_pruning.gui_simulation_concurrent import ConcurrentPruningSimulation


def run_single_experiment(scenario_name: str, method_name: str, seed: int,
                         num_steps: int = 100, verbose: bool = False):
    """Run a single experiment configuration."""
    try:
        sim_cfg, ctrl_cfg, cons_cfg, _flow_params, _ = get_scenario(scenario_name)
        sim_cfg.seed = seed
        sim_cfg.verbose = False

        if verbose:
            print(f"  Running: Scenario {scenario_name}, Method {method_name}, Seed {seed}")

        if method_name == 'concurrent':
            sim = ConcurrentPruningSimulation(sim_cfg, ctrl_cfg, cons_cfg)
            sim.verbose = False
        else:
            sim = BaselineSimulation(sim_cfg, ctrl_cfg, cons_cfg, method_name)
            sim.verbose = False

        start_time = time.time()
        for _ in range(num_steps):
            sim.step()
        elapsed_time = time.time() - start_time

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
            print(
                f"    -> Edges: {result['final_edges']}, "
                f"Pruned: {result['pruning_events']}, "
                f"Lambda2: {result['min_lambda2']:.3f}, "
                f"Time: {elapsed_time:.1f}s"
            )

        return result

    except Exception as exc:
        print(f"  ERROR in Scenario {scenario_name}, Method {method_name}, Seed {seed}: {exc}")
        import traceback
        traceback.print_exc()
        return {
            'scenario': scenario_name,
            'method': method_name,
            'seed': seed,
            'error': str(exc),
            'success': False
        }


def run_batch_experiments(scenarios=None, methods=None, seeds=None,
                         num_steps: int = 100,
                         output_dir: str = 'concurrent_pruning/results',
                         verbose: bool = True):
    """Run batch of experiments across scenarios, methods, and seeds."""
    if scenarios is None:
        scenarios = ['A', 'B', 'C', 'D', 'E']

    if methods is None:
        methods = ['concurrent', 'full_graph', 'centralized_mst', 'random', 'greedy_distance']

    if seeds is None:
        seeds = [42, 123, 456, 789, 1011]

    output_path = Path(output_dir)
    output_path.mkdir(exist_ok=True, parents=True)

    total_runs = len(scenarios) * len(methods) * len(seeds)

    print("=" * 70)
    print("BATCH SIMULATION RUNNER (CONCURRENT)")
    print("=" * 70)
    print(f"Scenarios: {scenarios}")
    print(f"Methods: {methods}")
    print(f"Seeds: {seeds}")
    print(f"Steps per run: {num_steps}")
    print(f"Total experiments: {total_runs}")
    print(f"Output directory: {output_path.absolute()}")
    print("=" * 70)

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

                elapsed = time.time() - start_time
                avg_time = elapsed / completed
                remaining = (total_runs - completed) * avg_time

                print(
                    f"    Progress: {completed}/{total_runs} "
                    f"({100*completed/total_runs:.1f}%) | "
                    f"Failed: {failed} | "
                    f"ETA: {remaining/60:.1f} min"
                )

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    results_file = output_path / f"batch_results_concurrent_{timestamp}.json"

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

    print("\n" + "=" * 70)
    print("BATCH COMPLETE")
    print("=" * 70)
    print(f"Total runs: {completed}/{total_runs}")
    print(f"Failed: {failed}")
    print(f"Success rate: {100*(completed-failed)/completed:.1f}%")
    print(f"Total time: {(time.time()-start_time)/60:.1f} minutes")
    print(f"Results saved to: {results_file}")
    print("=" * 70)

    return results_file


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Run batch simulations with concurrent pruning')
    parser.add_argument('--scenarios', nargs='+', default=['A', 'B', 'C', 'D', 'E'],
                        help='Scenarios to run (default: all)')
    parser.add_argument('--methods', nargs='+',
                        default=['concurrent', 'full_graph', 'centralized_mst', 'random', 'greedy_distance'],
                        help='Methods to test (default: concurrent + 4 baselines)')
    parser.add_argument('--seeds', nargs='+', type=int, default=[42, 123, 456, 789, 1011],
                        help='Random seeds (default: 5 seeds)')
    parser.add_argument('--steps', type=int, default=100,
                        help='Steps per simulation (default: 100)')
    parser.add_argument('--output', type=str, default='concurrent_pruning/results',
                        help='Output directory (default: concurrent_pruning/results)')
    parser.add_argument('--quiet', action='store_true',
                        help='Suppress verbose output')

    args = parser.parse_args()

    run_batch_experiments(
        scenarios=args.scenarios,
        methods=args.methods,
        seeds=args.seeds,
        num_steps=args.steps,
        output_dir=args.output,
        verbose=not args.quiet
    )
