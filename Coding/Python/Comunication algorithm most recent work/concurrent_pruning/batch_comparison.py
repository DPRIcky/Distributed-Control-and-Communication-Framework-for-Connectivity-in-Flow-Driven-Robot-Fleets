"""Batch comparison runner for Concurrent Pruning vs Baselines.

Runs multiple simulations with different seeds to statistically compare:
- ConcurrentPruning (distributed Lyapunov pruning)
- AdjacencyConsensus (distributed consensus pruning)
- FullGraph (no pruning baseline)
- CentralizedMST (oracle baseline with global knowledge)

Results saved to JSON and visualized with box plots and time series.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import json
import time
import argparse
import numpy as np
from datetime import datetime
from typing import Dict, List, Tuple, Set

from concurrent_pruning.concurrent_pruning_manager import ConcurrentPruningManager
from consensus.hybrid_pruning import HybridPruningManager
from config.consensus_config import ConsensusConfig
from baselines import FullGraphBaseline, CentralizedMSTBaseline


def generate_test_topology(num_robots: int, density: float = 0.6, 
                          seed: int = 42) -> Tuple[np.ndarray, Set[Tuple[int, int]]]:
    """Generate random robot positions and initial topology.
    
    Args:
        num_robots: Number of robots
        density: Edge density (0-1) - higher creates more redundant edges
        seed: Random seed
        
    Returns:
        (positions, edges) tuple
    """
    np.random.seed(seed)
    
    # Random positions in 10m x 10m workspace
    positions = np.random.rand(num_robots, 2) * 10.0
    
    # Create edges based on density - use more aggressive threshold
    edges = set()
    for i in range(num_robots):
        for j in range(i + 1, num_robots):
            distance = np.linalg.norm(positions[i] - positions[j])
            
            # Add edge with probability based on distance and density
            # Higher density = more edges at all distances
            threshold = 3.0 + 4.0 * density  # Range: 3.0 to 7.0 meters
            if distance < threshold:
                edges.add((i, j))
    
    # Ensure connectivity - add minimum spanning tree edges if needed
    if not is_connected(edges, num_robots):
        # Add nearest neighbor edges until connected
        connected_components = find_connected_components(edges, num_robots)
        while len(connected_components) > 1:
            # Find closest pair between different components
            min_dist = float('inf')
            best_edge = None
            
            for comp1_idx, comp1 in enumerate(connected_components):
                for comp2_idx, comp2 in enumerate(connected_components[comp1_idx + 1:], comp1_idx + 1):
                    for i in comp1:
                        for j in comp2:
                            dist = np.linalg.norm(positions[i] - positions[j])
                            if dist < min_dist:
                                min_dist = dist
                                best_edge = (min(i, j), max(i, j))
            
            if best_edge:
                edges.add(best_edge)
                connected_components = find_connected_components(edges, num_robots)
    
    return positions, edges


def compute_edge_lengths(positions: np.ndarray, edges: Set[Tuple[int, int]]) -> Dict[Tuple[int, int], float]:
    """Compute edge lengths for a set of edges."""
    lengths: Dict[Tuple[int, int], float] = {}
    for i, j in edges:
        lengths[(i, j)] = float(np.linalg.norm(positions[i] - positions[j]))
    return lengths


def is_connected(edges: Set[Tuple[int, int]], num_robots: int) -> bool:
    """Check if graph is connected."""
    if not edges:
        return num_robots <= 1
    
    # Build adjacency list
    adj = {i: set() for i in range(num_robots)}
    for i, j in edges:
        adj[i].add(j)
        adj[j].add(i)
    
    # BFS from node 0
    visited = {0}
    queue = [0]
    
    while queue:
        node = queue.pop(0)
        for neighbor in adj[node]:
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)
    
    return len(visited) == num_robots


def find_connected_components(edges: Set[Tuple[int, int]], num_robots: int) -> List[Set[int]]:
    """Find connected components in graph."""
    # Build adjacency list
    adj = {i: set() for i in range(num_robots)}
    for i, j in edges:
        adj[i].add(j)
        adj[j].add(i)
    
    visited = set()
    components = []
    
    for start in range(num_robots):
        if start in visited:
            continue
        
        # BFS from start
        component = {start}
        queue = [start]
        visited.add(start)
        
        while queue:
            node = queue.pop(0)
            for neighbor in adj[node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    component.add(neighbor)
                    queue.append(neighbor)
        
        components.append(component)
    
    return components


def compute_trust_matrix(positions: np.ndarray, edges: Set[Tuple[int, int]], 
                        sigma: float = 1.0) -> np.ndarray:
    """Compute trust-weighted adjacency matrix."""
    n = len(positions)
    A = np.zeros((n, n))
    
    for i, j in edges:
        distance = np.linalg.norm(positions[i] - positions[j])
        trust = np.exp(-distance**2 / sigma**2)
        A[i, j] = A[j, i] = trust
    
    return A


def compute_unweighted_adjacency(num_robots: int, edges: Set[Tuple[int, int]]) -> np.ndarray:
    """Compute unweighted (binary) adjacency matrix for connectivity analysis."""
    A = np.zeros((num_robots, num_robots))
    for i, j in edges:
        A[i, j] = A[j, i] = 1.0
    return A


def compute_lambda2(A: np.ndarray) -> float:
    """Compute algebraic connectivity (second eigenvalue of Laplacian)."""
    if A.size == 0:
        return 0.0
    
    D = np.diag(A.sum(axis=1))
    L = D - A
    
    # Compute eigenvalues and handle numerical issues
    eigenvalues = np.linalg.eigvalsh(L)
    eigenvalues = np.sort(eigenvalues)
    
    # Second smallest eigenvalue (skip first which should be ~0)
    # For connected graphs, lambda2 > 0
    if len(eigenvalues) > 1:
        # Handle numerical noise around zero
        lambda2 = eigenvalues[1]
        return max(0.0, lambda2)  # Ensure non-negative
    return 0.0


def compute_consensus_metric(A: np.ndarray) -> float:
    """Compute consensus disagreement metric."""
    n = A.shape[0]
    if n == 0:
        return 0.0
    
    # Compute row sums (degrees)
    row_sums = A.sum(axis=1)
    
    # Consensus metric: variance in adjacency matrix rows
    metric = 0.0
    for i in range(n):
        for j in range(n):
            if row_sums[i] > 0:
                metric += (A[i, j] - row_sums[j] / n) ** 2
    
    return np.sqrt(metric / (n * n)) if n > 0 else 0.0


def run_concurrent_pruning(num_robots: int, positions: np.ndarray, 
                          edges: Set[Tuple[int, int]], max_iter: int = 100,
                          verbose: bool = False) -> Dict:
    """Run concurrent pruning simulation.
    
    Args:
        num_robots: Number of robots
        positions: Robot positions (n x 2)
        edges: Initial edge set
        max_iter: Maximum iterations
        verbose: Print progress
        
    Returns:
        Results dictionary
    """
    start_time = time.time()
    
    # Initialize manager with relaxed thresholds for better pruning
    manager = ConcurrentPruningManager(
        num_robots=num_robots,
        mode='lyapunov',
        sigma=5.0,
        k_max=max_iter,
        lambda2_threshold=0.2,  # Relaxed from default 0.3
        lambda2_safety_margin=0.05,  # Smaller safety margin
        lyapunov_sim_steps=5  # Simulate multiple steps for pruning check
    )
    
    # Initialize robot knowledge with current topology
    manager.initialize_robot_knowledge(positions, edges)
    
    current_edges = edges.copy()
    initial_edge_count = len(edges)
    
    # History tracking
    edge_counts = [len(current_edges)]
    lambda2_values = []
    consensus_metrics = []
    pruning_events = []
    
    # Compute initial metrics
    A = compute_trust_matrix(positions, current_edges)
    A_unweighted = compute_unweighted_adjacency(num_robots, current_edges)
    lambda2_values.append(compute_lambda2(A_unweighted))
    consensus_metrics.append(compute_consensus_metric(A))
    
    # Run iterations
    for iteration in range(max_iter):
        # Compute current adjacency matrix
        A_current = compute_trust_matrix(positions, current_edges)
        
        # Run concurrent consensus + pruning step
        # Note: concurrent_step modifies current_edges in-place
        report = manager.concurrent_step(
            positions=positions,
            current_edges=current_edges,
            debug=verbose
        )
        
        # Track pruning events (edge already removed by concurrent_step)
        if report.get('pruned_edge'):
            pruned_edge = report['pruned_edge']
            pruning_events.append({
                'iteration': iteration,
                'edge': pruned_edge,
                'edges_remaining': len(current_edges)
            })
            
            if verbose:
                print(f"  Iter {iteration}: Pruned {pruned_edge}, {len(current_edges)} edges remain")
        
        # Update metrics
        A_current = compute_trust_matrix(positions, current_edges)
        A_unweighted = compute_unweighted_adjacency(num_robots, current_edges)
        edge_counts.append(len(current_edges))
        lambda2_values.append(compute_lambda2(A_unweighted))
        consensus_metrics.append(compute_consensus_metric(A_current))
        
        # Early stopping if minimal tree reached
        if len(current_edges) == num_robots - 1:
            if verbose:
                print(f"  Reached minimal spanning tree at iteration {iteration}")
            break
    
    elapsed_time = time.time() - start_time
    
    return {
        'method': 'ConcurrentPruning',
        'initial_edges': initial_edge_count,
        'final_edges': len(current_edges),
        'edges_pruned': initial_edge_count - len(current_edges),
        'edge_reduction_pct': 100 * (initial_edge_count - len(current_edges)) / initial_edge_count if initial_edge_count > 0 else 0,
        'pruning_events_count': len(pruning_events),
        'min_lambda2': float(min(lambda2_values)) if lambda2_values else 0.0,
        'mean_lambda2': float(np.mean(lambda2_values)) if lambda2_values else 0.0,
        'final_lambda2': float(lambda2_values[-1]) if lambda2_values else 0.0,
        'min_consensus_metric': float(min(consensus_metrics)) if consensus_metrics else 0.0,
        'final_consensus_metric': float(consensus_metrics[-1]) if consensus_metrics else 0.0,
        'iterations': len(edge_counts) - 1,
        'elapsed_time': elapsed_time,
        'edge_counts_history': [int(x) for x in edge_counts],
        'lambda2_history': [float(x) for x in lambda2_values],
        'consensus_history': [float(x) for x in consensus_metrics],
        'pruning_events': pruning_events,
        'success': bool(len(current_edges) >= num_robots - 1 and min(lambda2_values) >= 0.0)
    }


def run_full_graph(num_robots: int, positions: np.ndarray, 
                  edges: Set[Tuple[int, int]], max_iter: int = 100) -> Dict:
    """Run full graph baseline (no pruning).
    
    Args:
        num_robots: Number of robots
        positions: Robot positions
        edges: Initial edges
        max_iter: Maximum iterations
        
    Returns:
        Results dictionary
    """
    start_time = time.time()
    
    initial_edge_count = len(edges)
    
    # No pruning - just track metrics
    edge_counts = [len(edges)] * (max_iter + 1)
    lambda2_values = []
    consensus_metrics = []
    
    A = compute_trust_matrix(positions, edges)
    A_unweighted = compute_unweighted_adjacency(num_robots, edges)
    
    for _ in range(max_iter + 1):
        lambda2_values.append(compute_lambda2(A_unweighted))
        consensus_metrics.append(compute_consensus_metric(A))
    
    elapsed_time = time.time() - start_time
    
    return {
        'method': 'FullGraph',
        'initial_edges': initial_edge_count,
        'final_edges': len(edges),
        'edges_pruned': 0,
        'edge_reduction_pct': 0.0,
        'pruning_events_count': 0,
        'min_lambda2': float(min(lambda2_values)),
        'mean_lambda2': float(np.mean(lambda2_values)),
        'final_lambda2': float(lambda2_values[-1]),
        'min_consensus_metric': float(min(consensus_metrics)),
        'final_consensus_metric': float(consensus_metrics[-1]),
        'iterations': max_iter,
        'elapsed_time': elapsed_time,
        'edge_counts_history': [int(x) for x in edge_counts],
        'lambda2_history': [float(x) for x in lambda2_values],
        'consensus_history': [float(x) for x in consensus_metrics],
        'pruning_events': [],
        'success': True
    }


def run_centralized_mst(num_robots: int, positions: np.ndarray, 
                       edges: Set[Tuple[int, int]], max_iter: int = 100) -> Dict:
    """Run centralized MST baseline (oracle with global knowledge).
    
    Args:
        num_robots: Number of robots
        positions: Robot positions
        edges: Initial edges
        max_iter: Maximum iterations
        
    Returns:
        Results dictionary
    """
    start_time = time.time()
    
    initial_edge_count = len(edges)
    
    # Build MST using Kruskal's algorithm
    # Sort edges by length
    edge_list = [(i, j, np.linalg.norm(positions[i] - positions[j])) 
                 for i, j in edges]
    edge_list.sort(key=lambda x: x[2])
    
    # Union-find for MST
    parent = list(range(num_robots))
    
    def find(x):
        if parent[x] != x:
            parent[x] = find(parent[x])
        return parent[x]
    
    def union(x, y):
        px, py = find(x), find(y)
        if px != py:
            parent[px] = py
            return True
        return False
    
    mst_edges = set()
    for i, j, dist in edge_list:
        if union(i, j):
            mst_edges.add((i, j))
            if len(mst_edges) == num_robots - 1:
                break
    
    # Track metrics over iterations (MST is immediate)
    edge_counts = [initial_edge_count] + [len(mst_edges)] * max_iter
    lambda2_values = []
    consensus_metrics = []
    
    A_mst = compute_trust_matrix(positions, mst_edges)
    A_unweighted = compute_unweighted_adjacency(num_robots, mst_edges)
    
    for _ in range(max_iter + 1):
        lambda2_values.append(compute_lambda2(A_unweighted))
        consensus_metrics.append(compute_consensus_metric(A_mst))
    
    elapsed_time = time.time() - start_time
    
    return {
        'method': 'CentralizedMST',
        'initial_edges': initial_edge_count,
        'final_edges': len(mst_edges),
        'edges_pruned': initial_edge_count - len(mst_edges),
        'edge_reduction_pct': 100 * (initial_edge_count - len(mst_edges)) / initial_edge_count,
        'pruning_events_count': initial_edge_count - len(mst_edges),
        'min_lambda2': float(min(lambda2_values)),
        'mean_lambda2': float(np.mean(lambda2_values)),
        'final_lambda2': float(lambda2_values[-1]),
        'min_consensus_metric': float(min(consensus_metrics)),
        'final_consensus_metric': float(consensus_metrics[-1]),
        'iterations': max_iter,
        'elapsed_time': elapsed_time,
        'edge_counts_history': [int(x) for x in edge_counts],
        'lambda2_history': [float(x) for x in lambda2_values],
        'consensus_history': [float(x) for x in consensus_metrics],
        'pruning_events': [{'iteration': 0, 'edge': list(e), 'edges_remaining': len(mst_edges)} 
                          for e in (edges - mst_edges)],
        'success': bool(len(mst_edges) == num_robots - 1)
    }


def run_adjacency_consensus_pruning(num_robots: int, positions: np.ndarray,
                                   edges: Set[Tuple[int, int]], max_iter: int = 100,
                                   verbose: bool = False) -> Dict:
    """Run adjacency consensus-based pruning (pre-concurrent method)."""
    start_time = time.time()

    config = ConsensusConfig(
        consensus_sigma=5.0,
        consensus_sample_time=0.2,
        consensus_convergence_epsilon=1e-4,
        lambda2_threshold=0.2,
        lambda2_safety_margin=0.05,
        edge_quality_threshold=0.01
    )

    manager = HybridPruningManager(config, num_robots)

    current_edges = edges.copy()
    initial_edge_count = len(edges)

    edge_counts = [len(current_edges)]
    lambda2_values = []
    consensus_metrics = []
    pruning_events = []

    A = compute_trust_matrix(positions, current_edges, sigma=config.consensus_sigma)
    A_unweighted = compute_unweighted_adjacency(num_robots, current_edges)
    lambda2_values.append(compute_lambda2(A_unweighted))
    consensus_metrics.append(compute_consensus_metric(A))

    for iteration in range(max_iter):
        edge_lengths = compute_edge_lengths(positions, current_edges)

        if current_edges and not manager.consensus_converged:
            for _ in range(manager.max_consensus_iterations):
                converged = manager.run_consensus_phase(
                    positions=positions,
                    edge_lengths=edge_lengths,
                    current_edges=current_edges
                )
                if converged:
                    break
            else:
                manager.consensus_converged = True

        robot_decisions = []
        for robot_id in range(num_robots):
            decision = manager.find_redundant_edge_distributed(robot_id, debug=False)
            robot_decisions.append(decision)

        unique_decisions = set(robot_decisions)
        pruned_edge = None

        if len(unique_decisions) == 1 and robot_decisions[0] is not None:
            candidate = robot_decisions[0]
            i, j = candidate
            normalized = (min(i, j), max(i, j))

            if normalized in current_edges:
                pruned_edge = normalized
                current_edges.remove(normalized)

                for robot_id in range(num_robots):
                    A_est = manager.adjacency_consensus.A_estimates[robot_id]
                    A_est[i, j] = 0.0
                    A_est[j, i] = 0.0

                manager.consensus_converged = False
                manager.consensus_iterations = 0
                manager.failed_candidates.clear()

                pruning_events.append({
                    'iteration': iteration,
                    'edge': pruned_edge,
                    'edges_remaining': len(current_edges)
                })

                if verbose:
                    print(f"  Iter {iteration}: Pruned {pruned_edge}, {len(current_edges)} edges remain")

        A_current = compute_trust_matrix(positions, current_edges, sigma=config.consensus_sigma)
        A_unweighted = compute_unweighted_adjacency(num_robots, current_edges)
        edge_counts.append(len(current_edges))
        lambda2_values.append(compute_lambda2(A_unweighted))
        consensus_metrics.append(compute_consensus_metric(A_current))

        if len(current_edges) == num_robots - 1:
            if verbose:
                print(f"  Reached minimal spanning tree at iteration {iteration}")
            break

    elapsed_time = time.time() - start_time

    return {
        'method': 'AdjacencyConsensus',
        'initial_edges': initial_edge_count,
        'final_edges': len(current_edges),
        'edges_pruned': initial_edge_count - len(current_edges),
        'edge_reduction_pct': 100 * (initial_edge_count - len(current_edges)) / initial_edge_count if initial_edge_count > 0 else 0,
        'pruning_events_count': len(pruning_events),
        'min_lambda2': float(min(lambda2_values)) if lambda2_values else 0.0,
        'mean_lambda2': float(np.mean(lambda2_values)) if lambda2_values else 0.0,
        'final_lambda2': float(lambda2_values[-1]) if lambda2_values else 0.0,
        'min_consensus_metric': float(min(consensus_metrics)) if consensus_metrics else 0.0,
        'final_consensus_metric': float(consensus_metrics[-1]) if consensus_metrics else 0.0,
        'iterations': max_iter,
        'elapsed_time': elapsed_time,
        'edge_counts_history': [int(x) for x in edge_counts],
        'lambda2_history': [float(x) for x in lambda2_values],
        'consensus_history': [float(x) for x in consensus_metrics],
        'pruning_events': [
            {'iteration': e['iteration'], 'edge': list(e['edge']), 'edges_remaining': e['edges_remaining']}
            for e in pruning_events
        ],
        'success': bool(is_connected(current_edges, num_robots))
    }


def run_single_trial(num_robots: int, density: float, seed: int, 
                     methods: List[str], max_iter: int = 100,
                     verbose: bool = False) -> List[Dict]:
    """Run single trial with all methods.
    
    Args:
        num_robots: Number of robots
        density: Edge density
        seed: Random seed
        methods: List of method names to test
        max_iter: Maximum iterations
        verbose: Print progress
        
    Returns:
        List of result dictionaries (one per method)
    """
    # Generate topology
    positions, edges = generate_test_topology(num_robots, density, seed)
    
    if verbose:
        print(f"  Seed {seed}: {num_robots} robots, {len(edges)} initial edges")
    
    results = []
    
    for method in methods:
        try:
            if method == 'ConcurrentPruning':
                result = run_concurrent_pruning(num_robots, positions, edges, max_iter, verbose)
            elif method == 'AdjacencyConsensus':
                result = run_adjacency_consensus_pruning(num_robots, positions, edges, max_iter, verbose)
            elif method == 'FullGraph':
                result = run_full_graph(num_robots, positions, edges, max_iter)
            elif method == 'CentralizedMST':
                result = run_centralized_mst(num_robots, positions, edges, max_iter)
            else:
                raise ValueError(f"Unknown method: {method}")
            
            result['seed'] = seed
            result['num_robots'] = num_robots
            result['density'] = density
            results.append(result)
            
        except Exception as e:
            print(f"    ERROR in {method}: {e}")
            results.append({
                'method': method,
                'seed': seed,
                'num_robots': num_robots,
                'density': density,
                'error': str(e),
                'success': False
            })
    
    return results


def run_batch_comparison(num_robots: int = 15, density: float = 0.7,
                        num_seeds: int = 125, max_iter: int = 100,
                        methods: List[str] = None,
                        output_dir: str = 'concurrent_pruning/results',
                        verbose: bool = True) -> Dict:
    """Run batch comparison across multiple seeds.
    
    Args:
        num_robots: Number of robots
        density: Edge density
        num_seeds: Number of random seeds to test
        max_iter: Maximum iterations per trial
        methods: List of methods to compare
        output_dir: Output directory for results
        verbose: Print progress
        
    Returns:
        Dictionary with all results and metadata
    """
    if methods is None:
        methods = ['ConcurrentPruning', 'AdjacencyConsensus', 'FullGraph', 'CentralizedMST']
    
    # Create output directory
    output_path = Path(output_dir)
    output_path.mkdir(exist_ok=True, parents=True)
    
    # Generate seeds
    base_seed = 42
    seeds = list(range(base_seed, base_seed + num_seeds))
    
    total_trials = num_seeds * len(methods)
    
    print("=" * 80)
    print("CONCURRENT PRUNING BATCH COMPARISON")
    print("=" * 80)
    print(f"Configuration:")
    print(f"  Robots: {num_robots}")
    print(f"  Edge density: {density}")
    print(f"  Seeds: {num_seeds} (from {base_seed} to {base_seed + num_seeds - 1})")
    print(f"  Max iterations: {max_iter}")
    print(f"  Methods: {methods}")
    print(f"  Total trials: {total_trials}")
    print(f"  Output: {output_path.absolute()}")
    print("=" * 80)
    
    all_results = []
    completed = 0
    failed = 0
    start_time = time.time()
    
    for seed_idx, seed in enumerate(seeds):
        if verbose and seed_idx % 10 == 0:
            print(f"\nProcessing seeds {seed_idx+1}-{min(seed_idx+10, num_seeds)} of {num_seeds}...")
        
        trial_results = run_single_trial(
            num_robots, density, seed, methods, max_iter, verbose=False
        )
        
        all_results.extend(trial_results)
        
        for result in trial_results:
            completed += 1
            if not result.get('success', False):
                failed += 1
        
        # Progress update every 10 seeds
        if verbose and (seed_idx + 1) % 10 == 0:
            elapsed = time.time() - start_time
            avg_time = elapsed / (seed_idx + 1)
            remaining = (num_seeds - seed_idx - 1) * avg_time
            success_rate = 100 * (completed - failed) / completed if completed > 0 else 0
            
            print(f"  Progress: {seed_idx + 1}/{num_seeds} seeds "
                  f"({100*(seed_idx+1)/num_seeds:.1f}%) | "
                  f"Success: {success_rate:.1f}% | "
                  f"ETA: {remaining/60:.1f} min")
    
    # Save results
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    results_file = output_path / f"concurrent_comparison_{timestamp}.json"
    
    metadata = {
        'timestamp': timestamp,
        'num_robots': num_robots,
        'density': density,
        'num_seeds': num_seeds,
        'max_iter': max_iter,
        'methods': methods,
        'total_trials': total_trials,
        'completed': completed,
        'failed': failed,
        'elapsed_time': time.time() - start_time,
        'success_rate': 100 * (completed - failed) / completed if completed > 0 else 0
    }
    
    output_data = {
        'metadata': metadata,
        'results': all_results
    }
    
    with open(results_file, 'w') as f:
        json.dump(output_data, f, indent=2)
    
    print("\n" + "=" * 80)
    print("BATCH COMPARISON COMPLETE")
    print("=" * 80)
    print(f"Total trials: {completed}/{total_trials}")
    print(f"Failed: {failed}")
    print(f"Success rate: {metadata['success_rate']:.1f}%")
    print(f"Total time: {(time.time() - start_time)/60:.1f} minutes")
    print(f"Results saved to: {results_file}")
    print("=" * 80)
    
    return output_data


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Run batch comparison: ConcurrentPruning vs Baselines'
    )
    parser.add_argument('--robots', type=int, default=15,
                       help='Number of robots (default: 15)')
    parser.add_argument('--density', type=float, default=0.7,
                       help='Edge density 0-1 (default: 0.7)')
    parser.add_argument('--seeds', type=int, default=125,
                       help='Number of random seeds (default: 125)')
    parser.add_argument('--iterations', type=int, default=100,
                       help='Max iterations per trial (default: 100)')
    parser.add_argument('--methods', nargs='+',
                       default=['ConcurrentPruning', 'AdjacencyConsensus', 'FullGraph', 'CentralizedMST'],
                       help='Methods to compare')
    parser.add_argument('--output', type=str, default='concurrent_pruning/results',
                       help='Output directory')
    parser.add_argument('--quiet', action='store_true',
                       help='Suppress verbose output')
    
    args = parser.parse_args()
    
    # Run batch comparison
    run_batch_comparison(
        num_robots=args.robots,
        density=args.density,
        num_seeds=args.seeds,
        max_iter=args.iterations,
        methods=args.methods,
        output_dir=args.output,
        verbose=not args.quiet
    )
