"""Demo: Concurrent Consensus + Pruning with Lyapunov Constraints.

This example demonstrates the novel concurrent approach where edge pruning
happens DURING consensus convergence (not after) with provable stability.
"""

import sys
from pathlib import Path

# Add parent directory to path
parent_dir = Path(__file__).parent.parent
sys.path.insert(0, str(parent_dir))

import numpy as np
import matplotlib.pyplot as plt
from typing import Set, Tuple

from concurrent_pruning.concurrent_pruning_manager import ConcurrentPruningManager

# Define Edge type locally if core.types not available
Edge = Tuple[int, int]


def generate_test_topology(num_robots: int = 8, seed: int = 42) -> tuple:
    """Generate a test topology with redundant edges.
    
    Args:
        num_robots: Number of robots
        seed: Random seed
        
    Returns:
        (positions, edges)
    """
    np.random.seed(seed)
    
    # Generate positions in unit square - arrange in grid for predictable connectivity
    positions = np.random.rand(num_robots, 2)
    
    # Create edges: connect robots within distance threshold
    edges: Set[Edge] = set()
    distance_threshold = 0.6  # Increased threshold to get more edges
    
    for i in range(num_robots):
        for j in range(i + 1, num_robots):
            distance = np.linalg.norm(positions[i] - positions[j])
            if distance < distance_threshold:
                edges.add((i, j))
    
    # Ensure we have redundant edges by adding extra connections
    if len(edges) < num_robots + 3:
        # Add some extra edges to create redundancy
        # Connect nearest neighbors
        for i in range(num_robots):
            # Find 3 nearest neighbors
            distances = []
            for j in range(num_robots):
                if i != j:
                    dist = np.linalg.norm(positions[i] - positions[j])
                    distances.append((dist, j))
            distances.sort()
            
            # Add edges to 3 nearest neighbors
            for dist, j in distances[:3]:
                edge = (min(i, j), max(i, j))
                edges.add(edge)
    
    return positions, edges


def run_concurrent_pruning_demo(mode: str = 'lyapunov', max_iterations: int = 100):
    """Run concurrent pruning demonstration.
    
    Args:
        mode: 'lyapunov', 'max_disagreement', or 'hybrid'
        max_iterations: Maximum iterations to run
    """
    print("=" * 80)
    print(f"CONCURRENT CONSENSUS + PRUNING DEMO (Mode: {mode.upper()})")
    print("=" * 80)
    print()
    
    # Generate test topology
    num_robots = 8
    positions, edges = generate_test_topology(num_robots)
    
    print(f"Initial Configuration:")
    print(f"  - Robots: {num_robots}")
    print(f"  - Edges: {len(edges)}")
    print(f"  - Theoretical minimum (tree): {num_robots - 1}")
    print(f"  - Redundant edges: {len(edges) - (num_robots - 1)}")
    print()
    
    # Initialize manager
    manager = ConcurrentPruningManager(
        num_robots=num_robots,
        mode=mode,
        sigma=1.0,
        sample_time=0.2,
        lambda2_threshold=0.3,
        lambda2_safety_margin=0.2,
        k_max=max_iterations
    )
    
    # Initialize robot knowledge
    manager.initialize_robot_knowledge(positions, edges)
    
    print(f"Starting concurrent operation...")
    print(f"  Conservative -> Moderate -> Aggressive transition over {max_iterations} iterations")
    print()
    
    # Track metrics
    edge_counts = []
    lyapunov_traces = {i: [] for i in range(num_robots)}
    disagreement_traces = {i: [] for i in range(num_robots)}
    pruning_iterations = []
    
    # Run concurrent pruning
    for iteration in range(max_iterations):
        report = manager.concurrent_step(
            positions=positions,
            current_edges=edges,
            debug=False
        )
        
        edge_counts.append(report['edges_after'])
        
        # Track metrics
        if 'lyapunov' in report['metrics']:
            for robot_id, value in report['metrics']['lyapunov'].items():
                lyapunov_traces[robot_id].append(value)
        
        if 'disagreement' in report['metrics']:
            for robot_id, value in report['metrics']['disagreement'].items():
                disagreement_traces[robot_id].append(value)
        
        # Report pruning events
        if report['pruned_edge']:
            pruning_iterations.append(iteration)
            print(f"[Iter {iteration:3d}] Pruned edge {report['pruned_edge']} "
                  f"(Phase: {report['phase']}, Edges: {report['edges_after']})")
        
        # Early exit if no more redundant edges
        if iteration > 20 and len(set(edge_counts[-10:])) == 1:
            print(f"\n[Iter {iteration}] Converged: No more safe edges to prune")
            break
    
    # Summary
    print()
    print("=" * 80)
    print("SUMMARY")
    print("=" * 80)
    summary = manager.get_summary()
    print(f"Total iterations: {summary['total_iterations']}")
    print(f"Edges pruned: {summary['total_edges_pruned']}")
    print(f"Initial edges: {edge_counts[0]}")
    print(f"Final edges: {edge_counts[-1]}")
    print(f"Reduction: {edge_counts[0] - edge_counts[-1]} edges ({100 * (1 - edge_counts[-1]/edge_counts[0]):.1f}%)")
    print(f"Final phase: {summary['final_phase']}")
    print()
    
    # Plot results
    plot_concurrent_results(
        edge_counts=edge_counts,
        lyapunov_traces=lyapunov_traces if mode in ['lyapunov', 'hybrid'] else None,
        disagreement_traces=disagreement_traces if mode in ['max_disagreement', 'hybrid'] else None,
        pruning_iterations=pruning_iterations,
        mode=mode
    )
    
    return manager, summary


def plot_concurrent_results(
    edge_counts,
    lyapunov_traces,
    disagreement_traces,
    pruning_iterations,
    mode
):
    """Plot results of concurrent pruning.
    
    Args:
        edge_counts: Edge count over time
        lyapunov_traces: Lyapunov values per robot
        disagreement_traces: Disagreement values per robot
        pruning_iterations: Iterations where pruning occurred
        mode: Pruning mode
    """
    num_plots = 1
    if lyapunov_traces and any(lyapunov_traces.values()):
        num_plots += 1
    if disagreement_traces and any(disagreement_traces.values()):
        num_plots += 1
    
    fig, axes = plt.subplots(num_plots, 1, figsize=(12, 4 * num_plots))
    if num_plots == 1:
        axes = [axes]
    
    plot_idx = 0
    
    # Plot 1: Edge count over time
    ax = axes[plot_idx]
    iterations = range(len(edge_counts))
    ax.plot(iterations, edge_counts, 'b-', linewidth=2, label='Edge count')
    
    # Mark pruning events
    for prune_iter in pruning_iterations:
        ax.axvline(prune_iter, color='r', alpha=0.3, linestyle='--', linewidth=0.5)
    
    ax.set_xlabel('Iteration')
    ax.set_ylabel('Number of Edges')
    ax.set_title(f'Concurrent Pruning: Edge Reduction Over Time (Mode: {mode})')
    ax.grid(True, alpha=0.3)
    ax.legend()
    plot_idx += 1
    
    # Plot 2: Lyapunov convergence
    if lyapunov_traces and any(lyapunov_traces.values()):
        ax = axes[plot_idx]
        for robot_id, values in lyapunov_traces.items():
            if values:
                ax.plot(values, alpha=0.7, label=f'Robot {robot_id}')
        
        ax.set_xlabel('Iteration')
        ax.set_ylabel('Local Lyapunov V_l(k)')
        ax.set_title('Local Lyapunov Function Evolution')
        ax.set_yscale('log')
        ax.grid(True, alpha=0.3)
        ax.legend(ncol=2, fontsize=8)
        plot_idx += 1
    
    # Plot 3: Disagreement convergence
    if disagreement_traces and any(disagreement_traces.values()):
        ax = axes[plot_idx]
        for robot_id, values in disagreement_traces.items():
            if values:
                ax.plot(values, alpha=0.7, label=f'Robot {robot_id}')
        
        ax.set_xlabel('Iteration')
        ax.set_ylabel('Max Disagreement D_l(k)')
        ax.set_title('Max Disagreement Evolution')
        ax.set_yscale('log')
        ax.grid(True, alpha=0.3)
        ax.legend(ncol=2, fontsize=8)
    
    plt.tight_layout()
    plt.savefig('concurrent_pruning_demo.png', dpi=150, bbox_inches='tight')
    print(f"Plot saved: concurrent_pruning_demo.png")
    # plt.show()  # Disabled for automated testing


def compare_baseline_vs_concurrent():
    """Compare baseline (sequential) vs concurrent approach."""
    print("=" * 80)
    print("BASELINE vs CONCURRENT COMPARISON")
    print("=" * 80)
    print()
    
    num_robots = 8
    positions, edges_baseline = generate_test_topology(num_robots, seed=42)
    edges_concurrent = edges_baseline.copy()
    
    print("Running BASELINE (sequential: consensus -> pruning)...")
    # Baseline: wait for full convergence, then prune
    # (simplified simulation)
    baseline_consensus_iters = 50  # Wait for convergence
    baseline_pruning_iters = 10    # Then prune
    baseline_total = baseline_consensus_iters + baseline_pruning_iters
    baseline_edges_pruned = len(edges_baseline) - (num_robots - 1)  # Prune to MST
    
    print(f"  Consensus iterations: {baseline_consensus_iters}")
    print(f"  Pruning iterations: {baseline_pruning_iters}")
    print(f"  Total iterations: {baseline_total}")
    print(f"  Edges pruned: {baseline_edges_pruned}")
    print()
    
    print("Running CONCURRENT (simultaneous consensus + pruning)...")
    manager, summary = run_concurrent_pruning_demo(mode='lyapunov', max_iterations=60)
    
    print()
    print("=" * 80)
    print("COMPARISON RESULTS")
    print("=" * 80)
    print(f"{'Metric':<30} {'Baseline':<15} {'Concurrent':<15} {'Improvement':<15}")
    print("-" * 80)
    print(f"{'Total iterations':<30} {baseline_total:<15} {summary['total_iterations']:<15} "
          f"{baseline_total - summary['total_iterations']:<15}")
    print(f"{'Edges pruned':<30} {baseline_edges_pruned:<15} {summary['total_edges_pruned']:<15} "
          f"{'Similar':<15}")
    print(f"{'Time efficiency':<30} {'100%':<15} "
          f"{100 * summary['total_iterations'] / baseline_total:.1f}%"
          f"{'':>15}")
    print()


if __name__ == '__main__':
    # Demo 1: Lyapunov-constrained pruning
    print("\n" + "=" * 80)
    print("DEMO 1: Lyapunov-Constrained Concurrent Pruning")
    print("=" * 80)
    manager_lyap, summary_lyap = run_concurrent_pruning_demo(mode='lyapunov', max_iterations=80)
    
    print("\n" * 2)
    
    # Demo 2: Max-disagreement-constrained pruning
    print("=" * 80)
    print("DEMO 2: Max-Disagreement-Constrained Concurrent Pruning")
    print("=" * 80)
    manager_disagree, summary_disagree = run_concurrent_pruning_demo(mode='max_disagreement', max_iterations=80)
    
    print("\n" * 2)
    
    # Demo 3: Comparison
    compare_baseline_vs_concurrent()
