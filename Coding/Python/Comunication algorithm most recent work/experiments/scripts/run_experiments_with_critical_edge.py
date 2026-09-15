"""Comprehensive experiment runner including CriticalEdgeDetection method.

Runs 5 methods x 5 scenarios with proper simulation architecture:
- ConcurrentPruning: Uses ConcurrentPruningSimulation (Lyapunov-based)
- AdjacencyConsensus: Uses HybridUnderwaterSimulation (default consensus pruning)
- CriticalEdgeDetection: Uses CriticalEdgeSimulation (distributed MST with bridge preservation)
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

# Handle import of modules with spaces in folder names
import importlib.util
baseline_methods_path = root_dir / "baseline methods"
spec = importlib.util.spec_from_file_location(
    "baseline_methods", 
    baseline_methods_path / "__init__.py"
)
baseline_methods = importlib.util.module_from_spec(spec)
sys.modules["baseline_methods"] = baseline_methods
spec.loader.exec_module(baseline_methods)

import json
import time
import numpy as np
from datetime import datetime
from typing import Dict, List

from config import SimulationConfig, ControlConfig, ConsensusConfig
from config.scenarios import get_scenario
from simulation import HybridUnderwaterSimulation
from concurrent_pruning.gui_simulation_concurrent import ConcurrentPruningSimulation
from Critical_edge_detection_baseline.critical_edge_simulation import CriticalEdgeSimulation
from baseline_methods.baseline_simulation import BaselineSimulation


def run_single_simulation(method: str, scenario_name: str, seed: int, 
                         max_steps: int = 500, verbose: bool = False) -> Dict:
    """Run a single simulation and collect comprehensive metrics.
    
    Args:
        method: One of 'ConcurrentPruning', 'AdjacencyConsensus', 'CriticalEdgeDetection', 
                'FullGraph', 'CentralizedMST'
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
    
    # Initialize appropriate simulation type with enhanced error handling
    sim = None
    try:
        if method == 'ConcurrentPruning':
            sim = ConcurrentPruningSimulation(
                sim_config, control_config, consensus_config,
                pruning_mode='lyapunov'
            )
        elif method == 'AdjacencyConsensus':
            # Default HybridUnderwaterSimulation uses consensus pruning
            sim = HybridUnderwaterSimulation(sim_config, control_config, consensus_config)
        elif method == 'CriticalEdgeDetection':
            # New method: distributed MST with bridge preservation
            # Adaptive update frequency: more frequent updates for larger networks
            update_freq = max(5, 10 - sim_config.num_robots // 5)  # 10 for 10 robots, 6 for 20 robots
            sim = CriticalEdgeSimulation(
                sim_config, control_config, consensus_config,
                update_frequency=update_freq,  # Adapt to network size
                prefer_critical=True,  # Prioritize critical edges
                verbose=False
            )
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
    except Exception as e:
        if verbose:
            print(f"    ERROR during simulation creation: {str(e)}")
        elapsed_time = time.time() - start_time
        return {
            'method': method,
            'scenario': scenario_name,
            'seed': seed,
            'success': False,
            'error': f"Simulation creation failed: {str(e)}",
            'initial_edges': 0,
            'final_edges': 0,
            'edges_pruned': 0,
            'edge_reduction_pct': 0.0,
            'pruning_events_count': 0,
            'min_lambda2': 0.0,
            'mean_lambda2': 0.0,
            'final_lambda2': 0.0,
            'lambda2_violations': 0,
            'total_control_effort': 0.0,
            'mean_control_effort': 0.0,
            'max_control_effort': 0.0,
            'mean_goal_distance': 0.0,
            'min_goal_distance': 0.0,
            'elapsed_time': elapsed_time,
            'steps_executed': 0,
            'positions_history': [],
            'edge_counts_history': [],
            'lambda2_history': [],
            'control_effort_history': [],
            'pruning_events': []
        }
    
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
    error_message = None
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
            if len(sim.metrics.control_magnitudes) > 1:
                total_effort = np.sum(sim.metrics.control_magnitudes[-2])
            elif len(sim.metrics.control_magnitudes) == 1:
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
            
            # Early stopping if truly disconnected (very strict threshold for numerical stability)
            # Only stop if graph is completely broken (lambda2 is NaN or very negative)
            if np.isnan(lambda2) or lambda2 < -1.0:
                if verbose:
                    print(f"    WARNING: Graph critically disconnected at step {step}")
                success = False
                break
            
            # Early stopping if converged (reached minimal spanning tree)
            if current_edges <= sim.num_robots - 1:
                if verbose:
                    print(f"    Converged to minimal tree at step {step}")
                break
                
    except Exception as e:
        error_message = str(e)
        if verbose:
            print(f"    ERROR: {error_message}")
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
        
        # History data for plotting (sample for file size)
        'positions_history': positions_history[::max(1, len(positions_history)//100)],  # Sample every 1%
        'edge_counts_history': edge_counts_history,
        'lambda2_history': lambda2_history,
        'control_effort_history': control_effort_history,
        'pruning_events': pruning_events
    }
    
    # Add error message if present
    if error_message:
        results['error'] = error_message
    
    return results


def run_all_experiments(num_trials: int = 5, max_steps: int = 500, verbose: bool = True) -> List[Dict]:
    """Run complete experiment suite.
    
    Args:
        num_trials: Number of trials per configuration
        max_steps: Maximum steps per trial
        verbose: Print progress
        
    Returns:
        List of all results
    """
    # Methods to test (now including CriticalEdgeDetection)
    methods = [
        'ConcurrentPruning',
        'AdjacencyConsensus',
        'CriticalEdgeDetection',  # NEW METHOD
        'FullGraph',
        'CentralizedMST'
    ]
    
    # Scenarios to test
    scenarios = ['A', 'B', 'C', 'D', 'E']
    
    total_configs = len(methods) * len(scenarios) * num_trials
    current = 0
    
    all_results = []
    
    print(f"\n{'='*70}")
    print(f"COMPREHENSIVE EXPERIMENT SUITE")
    print(f"{'='*70}")
    print(f"Methods: {len(methods)} ({', '.join(methods)})")
    print(f"Scenarios: {len(scenarios)} (A-E)")
    print(f"Trials: {num_trials} per configuration")
    print(f"Total simulations: {total_configs}")
    print(f"{'='*70}\n")
    
    for method in methods:
        for scenario in scenarios:
            for trial in range(num_trials):
                current += 1
                print(f"[{current:3d}/{total_configs}] {method:25s} Scenario {scenario} Trial {trial+1}/{num_trials}")
                
                result = run_single_simulation(
                    method=method,
                    scenario_name=scenario,
                    seed=trial + scenario.encode()[0],  # Unique seed per trial
                    max_steps=max_steps,
                    verbose=False
                )
                
                all_results.append(result)
    
    return all_results


def save_results(results: List[Dict], output_file: str = None) -> str:
    """Save results to JSON file.
    
    Args:
        results: List of result dictionaries
        output_file: Output filename (auto-generated if None)
        
    Returns:
        Path to saved file
    """
    if output_file is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = f"comprehensive_results_with_critical_edge_{timestamp}.json"
    
    output_path = Path(__file__).parent.parent / output_file
    
    # Convert numpy arrays to lists for JSON serialization
    serializable_results = []
    for result in results:
        r = result.copy()
        if isinstance(r.get('positions_history'), list):
            r['positions_history'] = [[list(pos) for pos in positions] for positions in r['positions_history']]
        serializable_results.append(r)
    
    with open(output_path, 'w') as f:
        json.dump(serializable_results, f, indent=2)
    
    print(f"\n✓ Results saved to: {output_path}")
    return str(output_path)


def print_summary(results: List[Dict]):
    """Print summary statistics."""
    print(f"\n{'='*70}")
    print(f"EXPERIMENT SUMMARY")
    print(f"{'='*70}\n")
    
    methods = sorted(set(r['method'] for r in results))
    scenarios = sorted(set(r['scenario'] for r in results))
    
    # Success rates
    print("✓ SUCCESS RATES:")
    for method in methods:
        method_results = [r for r in results if r['method'] == method]
        success_count = sum(1 for r in method_results if r['success'])
        success_rate = 100.0 * success_count / len(method_results)
        print(f"  {method:25s}: {success_rate:5.1f}% ({success_count}/{len(method_results)})")
    
    # Edge reduction
    print(f"\n✂ EDGE REDUCTION:")
    for method in methods:
        method_results = [r for r in results if r['method'] == method and r['success']]
        if method_results:
            mean_reduction = np.mean([r['edge_reduction_pct'] for r in method_results])
            std_reduction = np.std([r['edge_reduction_pct'] for r in method_results])
            print(f"  {method:25s}: {mean_reduction:5.1f}% ± {std_reduction:5.1f}%")
    
    # Connectivity (min lambda2)
    print(f"\n📊 MIN CONNECTIVITY (λ₂):")
    for method in methods:
        method_results = [r for r in results if r['method'] == method and r['success']]
        if method_results:
            mean_lambda2 = np.mean([r['min_lambda2'] for r in method_results])
            std_lambda2 = np.std([r['min_lambda2'] for r in method_results])
            print(f"  {method:25s}: {mean_lambda2:6.3f} ± {std_lambda2:6.3f}")
    
    # Control effort
    print(f"\n⚡ CONTROL EFFORT (mean):")
    for method in methods:
        method_results = [r for r in results if r['method'] == method and r['success']]
        if method_results:
            mean_effort = np.mean([r['total_control_effort'] for r in method_results])
            std_effort = np.std([r['total_control_effort'] for r in method_results])
            print(f"  {method:25s}: {mean_effort:8.1f} ± {std_effort:8.1f}")
    
    print(f"\n{'='*70}\n")


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Run comprehensive experiments with CriticalEdgeDetection')
    parser.add_argument('--trials', type=int, default=5, help='Number of trials per configuration')
    parser.add_argument('--steps', type=int, default=500, help='Maximum steps per trial')
    parser.add_argument('--quick', action='store_true', help='Quick mode: 2 trials, 300 steps')
    parser.add_argument('--output', type=str, default=None, help='Output filename')
    
    args = parser.parse_args()
    
    if args.quick:
        args.trials = 2
        args.steps = 300
        print("[QUICK MODE] 2 trials, 300 steps per configuration")
    
    # Run experiments
    results = run_all_experiments(num_trials=args.trials, max_steps=args.steps, verbose=True)
    
    # Save results
    output_path = save_results(results, args.output)
    
    # Print summary
    print_summary(results)
    
    print(f"✓ Experiments complete! Results saved to: {output_path}")
    print(f"✓ Next: Run 'python experiments/scripts/generate_plots_with_critical_edge.py' to generate plots")
