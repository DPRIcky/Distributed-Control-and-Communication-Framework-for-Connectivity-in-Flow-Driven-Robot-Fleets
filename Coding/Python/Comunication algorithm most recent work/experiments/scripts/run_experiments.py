"""Comprehensive experiment runner for all methods and scenarios.

Runs 4 methods x 5 scenarios with proper simulation architecture:
- ConcurrentPruning: Uses ConcurrentPruningSimulation (Lyapunov-based)
- AdjacencyConsensus: Uses HybridUnderwaterSimulation (default consensus pruning)
- FullGraph: Uses BaselineSimulation with 'full_graph' (no pruning)
- CentralizedMST: Uses BaselineSimulation with 'centralized_mst' (oracle)

Tracks comprehensive metrics including:
- Edge evolution over time
- Lambda2 (connectivity) over time
- Robot positions over time (for trajectories)
- Control effort over time
- Pruning events
"""

import sys
from pathlib import Path

# Add parent directories to path
root_dir = Path(__file__).parent.parent.parent
sys.path.insert(0, str(root_dir))

import json
import time
import numpy as np
from datetime import datetime
from typing import Dict, List

from config import SimulationConfig, ControlConfig, ConsensusConfig
from config.scenarios import get_scenario
from simulation import HybridUnderwaterSimulation
from concurrent_pruning.gui_simulation_concurrent import ConcurrentPruningSimulation
from baselines.baseline_simulation import BaselineSimulation


def run_single_simulation(method: str, scenario_name: str, seed: int, 
                         max_steps: int = 500, verbose: bool = False) -> Dict:
    """Run a single simulation and collect comprehensive metrics.
    
    Args:
        method: One of 'ConcurrentPruning', 'AdjacencyConsensus', 'FullGraph', 'CentralizedMST'
        scenario_name: Scenario name ('A', 'B', 'C', 'D', 'E')
        seed: Random seed
        max_steps: Maximum simulation steps
        verbose: Print progress
        
    Returns:
        Dictionary with all metrics
    """
    if verbose:
        print(f"  Running {method} on Scenario {scenario_name} (seed={seed})...")
    
    # Get scenario configuration
    sim_config, control_config, consensus_config, flow_params, scenario_full_name = get_scenario(scenario_name)
    sim_config.seed = seed  # Override seed for this trial
    
    start_time = time.time()
    
    # Initialize appropriate simulation type
    if method == 'ConcurrentPruning':
        sim = ConcurrentPruningSimulation(
            sim_config, control_config, consensus_config,
            pruning_mode='lyapunov'
        )
    elif method == 'AdjacencyConsensus':
        # Default HybridUnderwaterSimulation uses consensus pruning
        sim = HybridUnderwaterSimulation(sim_config, control_config, consensus_config)
    elif method == 'FullGraph':
        sim = BaselineSimulation(
            sim_config, control_config, consensus_config,
            baseline_method='full_graph'
        )
    elif method == 'CentralizedMST':
        sim = BaselineSimulation(
            sim_config, control_config, consensus_config,
            baseline_method='centralized_mst'
        )
    else:
        raise ValueError(f"Unknown method: {method}")
    
    # Setup flow field parameters
    sim.flow.base_vector = np.array([flow_params['base_flow'], flow_params['base_flow'] * 0.2])
    sim.flow.swirl_amplitude = flow_params['flow_scale']
    
    # Initialize metrics tracking
    positions_history = []  # List of [num_robots, 2] arrays
    edge_counts_history = []
    lambda2_history = []
    control_effort_history = []  # Total control effort per step
    pruning_events = []
    
    initial_edges = sum(len(neighbors) for neighbors in sim.topology_manager.neighbor_graph) // 2
    
    # Run simulation
    success = True
    try:
        for step in range(max_steps):
            # Record positions before step
            positions = np.array([robot.position.copy() for robot in sim.robots])
            positions_history.append(positions)
            
            # Execute step
            step_result = sim.step()
            
            # Get metrics from simulation
            current_edges = sum(len(neighbors) for neighbors in sim.topology_manager.neighbor_graph) // 2
            edge_counts_history.append(current_edges)
            
            # Get lambda2 (computed by metrics.record_step() during sim.step())
            if len(sim.metrics.lambda2_values) > 0:
                lambda2 = sim.metrics.lambda2_values[-1]
            else:
                lambda2 = 0.0
            lambda2_history.append(lambda2)
            
            # Get control effort from metrics
            # Note: control_magnitudes may have an extra entry added by record_step
            # We want the filled entry, not the newly appended zero array
            if len(sim.metrics.control_magnitudes) > 1:
                # After first step, use -2 (second to last) as -1 is the new zero array
                total_effort = np.sum(sim.metrics.control_magnitudes[-2])
            elif len(sim.metrics.control_magnitudes) == 1:
                # First step only
                total_effort = np.sum(sim.metrics.control_magnitudes[-1])
            else:
                total_effort = 0.0
            control_effort_history.append(total_effort)
            
            # Track pruning events (if available)
            if isinstance(step_result, dict) and step_result.get('pruned_edge'):
                pruning_events.append({
                    'step': step,
                    'edge': step_result['pruned_edge'],
                    'edges_remaining': current_edges
                })
            
            # Early stopping if disconnected
            if lambda2 < 0.01 and step > 50:
                if verbose:
                    print(f"    WARNING: Graph disconnected at step {step} (λ₂={lambda2:.4f})")
                success = False
                break
            
            # Early stopping if converged
            if current_edges <= sim.num_robots - 1:
                if verbose:
                    print(f"    Converged to minimal tree at step {step}")
                break
                
    except Exception as e:
        if verbose:
            print(f"    ERROR: {str(e)}")
        success = False
    
    elapsed_time = time.time() - start_time
    
    # Get final metrics
    final_edges = edge_counts_history[-1] if edge_counts_history else initial_edges
    final_lambda2 = lambda2_history[-1] if lambda2_history else 0.0
    min_lambda2 = min(lambda2_history) if lambda2_history else 0.0
    mean_lambda2 = np.mean(lambda2_history) if lambda2_history else 0.0
    
    # Compute final robot positions
    final_positions = positions_history[-1] if positions_history else np.zeros((sim.num_robots, 2))
    goal_distances = [np.linalg.norm(pos - sim.goal_position) for pos in final_positions]
    mean_goal_distance = np.mean(goal_distances)
    
    # Package results
    results = {
        'method': method,
        'scenario': scenario_name,
        'seed': seed,
        'success': success,
        
        # Edge statistics
        'initial_edges': initial_edges,
        'final_edges': final_edges,
        'edges_pruned': initial_edges - final_edges,
        'edge_reduction_pct': 100.0 * (initial_edges - final_edges) / initial_edges if initial_edges > 0 else 0.0,
        'pruning_events_count': len(pruning_events),
        
        # Connectivity statistics
        'min_lambda2': float(min_lambda2),
        'mean_lambda2': float(mean_lambda2),
        'final_lambda2': float(final_lambda2),
        'lambda2_violations': sum(1 for l2 in lambda2_history if l2 < consensus_config.lambda2_threshold),
        
        # Control statistics
        'total_control_effort': float(sum(control_effort_history)),
        'mean_control_effort': float(np.mean(control_effort_history)) if control_effort_history else 0.0,
        'max_control_effort': float(max(control_effort_history)) if control_effort_history else 0.0,
        
        # Goal reaching
        'mean_goal_distance': float(mean_goal_distance),
        'min_goal_distance': float(min(goal_distances)),
        
        # Runtime
        'elapsed_time': elapsed_time,
        'steps_executed': len(edge_counts_history),
        
        # Time series data (for plotting)
        'edge_counts_history': [int(x) for x in edge_counts_history],
        'lambda2_history': [float(x) for x in lambda2_history],
        'control_effort_history': [float(x) for x in control_effort_history],
        'pruning_events': pruning_events,
        
        # Trajectory data (positions over time)
        # Store as list of lists for JSON serialization
        'positions_history': [pos.tolist() for pos in positions_history],
        'goal_position': sim.goal_position.tolist(),
        
        # Configuration
        'num_robots': sim.num_robots,
        'communication_radius': sim.communication_radius,
        'dt': sim.dt,
    }
    
    return results


def run_batch_experiments(
    methods: List[str],
    scenarios: List[str],
    trials_per_config: int = 10,
    max_steps: int = 500,
    output_file: str = None,
    verbose: bool = True
) -> Dict:
    """Run full batch of experiments.
    
    Args:
        methods: List of method names
        scenarios: List of scenario names
        trials_per_config: Number of trials per method/scenario combination
        max_steps: Maximum steps per simulation
        output_file: Output JSON file path
        verbose: Print progress
        
    Returns:
        Dictionary with all results
    """
    total_runs = len(methods) * len(scenarios) * trials_per_config
    
    if verbose:
        print(f"=" * 80)
        print(f"RUNNING COMPREHENSIVE EXPERIMENTS")
        print(f"=" * 80)
        print(f"Methods: {methods}")
        print(f"Scenarios: {scenarios}")
        print(f"Trials per config: {trials_per_config}")
        print(f"Total simulations: {total_runs}")
        print(f"=" * 80)
    
    all_results = []
    run_count = 0
    start_time = time.time()
    
    for method in methods:
        for scenario in scenarios:
            if verbose:
                print(f"\n{method} - Scenario {scenario}:")
            
            for trial in range(trials_per_config):
                seed = 1000 + trial  # Consistent seeds across methods
                
                try:
                    result = run_single_simulation(
                        method=method,
                        scenario_name=scenario,
                        seed=seed,
                        max_steps=max_steps,
                        verbose=False
                    )
                    all_results.append(result)
                    
                    run_count += 1
                    
                    if verbose:
                        status = "✓" if result['success'] else "✗"
                        elapsed = time.time() - start_time
                        avg_time = elapsed / run_count
                        remaining = (total_runs - run_count) * avg_time
                        
                        print(f"  {status} Trial {trial+1:2d}/{trials_per_config} | "
                              f"λ₂_min={result['min_lambda2']:.3f} | "
                              f"edges: {result['initial_edges']}→{result['final_edges']} | "
                              f"Progress: {run_count}/{total_runs} ({100*run_count/total_runs:.1f}%) | "
                              f"ETA: {remaining/60:.1f}min")
                
                except Exception as e:
                    print(f"  ✗ ERROR in trial {trial}: {str(e)}")
                    continue
    
    total_time = time.time() - start_time
    
    # Package results
    results_package = {
        'metadata': {
            'timestamp': datetime.now().isoformat(),
            'methods': methods,
            'scenarios': scenarios,
            'trials_per_config': trials_per_config,
            'max_steps': max_steps,
            'total_runs': run_count,
            'total_time_seconds': total_time,
        },
        'results': all_results
    }
    
    # Save to file
    if output_file is None:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_file = str(root_dir / 'experiments' / f'comprehensive_results_{timestamp}.json')
    
    Path(output_file).parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_file, 'w') as f:
        json.dump(results_package, f, indent=2)
    
    if verbose:
        print(f"\n" + "=" * 80)
        print(f"EXPERIMENTS COMPLETE")
        print(f"=" * 80)
        print(f"Total time: {total_time/60:.1f} minutes")
        print(f"Average time per simulation: {total_time/run_count:.2f}s")
        print(f"Success rate: {sum(r['success'] for r in all_results)}/{len(all_results)} "
              f"({100*sum(r['success'] for r in all_results)/len(all_results):.1f}%)")
        print(f"Results saved to: {output_file}")
        print(f"=" * 80)
    
    return results_package


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Run comprehensive experiments')
    parser.add_argument('--methods', nargs='+', 
                       default=['ConcurrentPruning', 'AdjacencyConsensus', 'FullGraph', 'CentralizedMST'],
                       help='Methods to test')
    parser.add_argument('--scenarios', nargs='+',
                       default=['A', 'B', 'C', 'D', 'E'],
                       help='Scenarios to test')
    parser.add_argument('--trials', type=int, default=10,
                       help='Trials per configuration')
    parser.add_argument('--steps', type=int, default=500,
                       help='Maximum steps per simulation')
    parser.add_argument('--output', type=str, default=None,
                       help='Output file path')
    parser.add_argument('--quick', action='store_true',
                       help='Quick test (2 trials, 100 steps, scenario A only)')
    
    args = parser.parse_args()
    
    if args.quick:
        print("=" * 80)
        print("QUICK TEST MODE")
        print("=" * 80)
        args.trials = 2
        args.steps = 100
        args.scenarios = ['A']
        print(f"Settings: {args.trials} trials, {args.steps} steps, scenarios {args.scenarios}")
        print("=" * 80 + "\n")
    
    run_batch_experiments(
        methods=args.methods,
        scenarios=args.scenarios,
        trials_per_config=args.trials,
        max_steps=args.steps,
        output_file=args.output,
        verbose=True
    )
