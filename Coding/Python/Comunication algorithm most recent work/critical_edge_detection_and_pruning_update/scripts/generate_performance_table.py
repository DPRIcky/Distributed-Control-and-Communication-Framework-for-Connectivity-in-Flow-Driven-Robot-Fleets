"""Generate performance statistics for distributed pruning algorithm (Monte Carlo trials).

Evaluates:
- Success rate (graph remains connected)
- Edge reduction percentage
- Minimum algebraic connectivity (eigenvalue)
- Runtime
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import numpy as np
import networkx as nx
import time
from scipy import sparse
from scipy.sparse import linalg
from distributed_pruning_algorithm import DistributedPruningAlgorithm


def compute_algebraic_connectivity(graph):
    """Compute the algebraic connectivity (second smallest eigenvalue of Laplacian)."""
    if len(graph) == 0 or not nx.is_connected(graph):
        return 0.0
    
    try:
        L = nx.laplacian_matrix(graph).asformat('csr')
        eigenvalues = linalg.eigsh(L, k=2, which='SM', return_eigenvectors=False)
        # Return the largest (second smallest is typically the algebraic connectivity)
        return float(eigenvalues[-1])
    except:
        return 0.0


def run_monte_carlo_trial(num_robots=10, graph_type='random', seed=None):
    """Run a single Monte Carlo trial of the distributed pruning algorithm.
    
    Returns:
        dict: Trial results including success, edge reduction, connectivity, runtime
    """
    result = {
        'success': False,
        'edge_reduction': 0.0,
        'min_lambda2': 0.0,
        'runtime': 0.0,
        'initial_edges': 0,
        'final_edges': 0,
        'connected': False,
        'initial_lambda2': 0.0
    }
    
    try:
        # Generate graph based on type
        if graph_type == 'random':
            G = nx.erdos_renyi_graph(num_robots, 0.3, seed=seed)
        elif graph_type == 'grid':
            side = int(np.sqrt(num_robots))
            G = nx.grid_2d_graph(side, side)
            G = nx.convert_node_labels_to_integers(G)
        elif graph_type == 'complete':
            G = nx.complete_graph(num_robots)
        else:  # geometric
            G = nx.random_geometric_graph(num_robots, 0.4, seed=seed)
        
        # Ensure connectivity
        if not nx.is_connected(G):
            # Connect components
            components = list(nx.connected_components(G))
            for i in range(len(components) - 1):
                u = list(components[i])[0]
                v = list(components[i+1])[0]
                G.add_edge(u, v)
        
        result['initial_edges'] = len(G.edges())
        result['initial_lambda2'] = compute_algebraic_connectivity(G)
        
        # Run distributed pruning algorithm
        start_time = time.time()
        algo = DistributedPruningAlgorithm(G.copy(), root=0)  # Use node 0 as root
        algo.run_full_pipeline()
        runtime = time.time() - start_time
        
        # Get results
        pruned_graph = algo.get_pruned_graph()
        result['final_edges'] = len(pruned_graph.edges())
        result['connected'] = nx.is_connected(pruned_graph)
        result['success'] = result['connected']
        
        # Calculate metrics
        if result['initial_edges'] > 0:
            result['edge_reduction'] = (1 - result['final_edges'] / result['initial_edges']) * 100
        
        if result['connected']:
            result['min_lambda2'] = compute_algebraic_connectivity(pruned_graph)
        else:
            # For disconnected graphs, set to 0 (not negative)
            result['min_lambda2'] = 0.0
        
        result['runtime'] = runtime
        
    except Exception as e:
        print(f"Trial error: {e}")
        result['success'] = False
    
    return result


def run_monte_carlo_experiment(num_trials=10, num_robots=10, graph_type='random'):
    """Run multiple Monte Carlo trials and collect statistics.
    
    Returns:
        dict: Aggregated statistics
    """
    results = []
    
    print(f"\nRunning {num_trials} trials for {graph_type} graphs ({num_robots} robots)...")
    for trial in range(num_trials):
        result = run_monte_carlo_trial(num_robots=num_robots, graph_type=graph_type, seed=42+trial)
        results.append(result)
        
        if (trial + 1) % 2 == 0:
            print(f"  Completed trial {trial + 1}/{num_trials}")
    
    # Compute statistics
    successes = sum(1 for r in results if r['success'])
    success_rate = (successes / num_trials) * 100
    
    edge_reductions = [r['edge_reduction'] for r in results]
    min_lambda2s = [r['min_lambda2'] for r in results if r['connected']]
    runtimes = [r['runtime'] for r in results]
    
    # Handle case where no graphs remain connected
    if len(min_lambda2s) == 0:
        mean_lambda2 = 0.0
        std_lambda2 = 0.0
    else:
        mean_lambda2 = np.mean(min_lambda2s)
        std_lambda2 = np.std(min_lambda2s)
    
    stats = {
        'success_rate': success_rate,
        'edge_reduction_mean': np.mean(edge_reductions) if edge_reductions else 0,
        'edge_reduction_std': np.std(edge_reductions) if edge_reductions else 0,
        'min_lambda2_mean': mean_lambda2,
        'min_lambda2_std': std_lambda2,
        'runtime_mean': np.mean(runtimes),
        'runtime_std': np.std(runtimes),
        'trials': num_trials,
        'graph_type': graph_type,
        'num_robots': num_robots,
        'connected_trials': len(min_lambda2s)
    }
    
    return stats


def generate_latex_table(results_list):
    """Generate LaTeX table from multiple experiment results."""
    
    print("\n" + "="*80)
    print("PERFORMANCE COMPARISON TABLE (LaTeX)")
    print("="*80)
    
    latex_code = r"""\begin{table}[H]
\centering
\caption{Aggregate performance of Distributed Pruning Algorithm across scenarios (10 Monte-Carlo trials per scenario).}
\label{tab:distributed_pruning_stats}
\footnotesize
\setlength{\tabcolsep}{2.4pt}
\resizebox{\columnwidth}{!}{%
\begin{tabular}{lcccc}
\toprule
Method & Success (\%) & Edge red.\ (\%) & $\min_t \lambda_2$ & Runtime (s) \\
\midrule
"""
    
    for result in results_list:
        method_name = f"Distributed Pruning ({result['graph_type'].capitalize()}, n={result['num_robots']})"
        success = f"{result['success_rate']:.2f}"
        edge_red = f"${result['edge_reduction_mean']:.2f}\\pm {result['edge_reduction_std']:.2f}$"
        min_lambda2 = f"${result['min_lambda2_mean']:.4f}\\pm {result['min_lambda2_std']:.4f}$"
        runtime = f"${result['runtime_mean']:.2f}\\pm {result['runtime_std']:.2f}$"
        
        latex_code += f"{method_name} & {success} & {edge_red} & {min_lambda2} & {runtime} \\\\\n"
    
    latex_code += r"""\bottomrule
\end{tabular}}
\end{table}
\FloatBarrier
"""
    
    print(latex_code)
    
    # Save to file
    output_file = Path(__file__).parent.parent / "Docs" / "PERFORMANCE_TABLE_DISTRIBUTED_PRUNING.tex"
    output_file.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_file, 'w') as f:
        f.write(latex_code)
    
    print(f"\nSaved LaTeX table to: {output_file}")
    
    return latex_code


if __name__ == "__main__":
    print("\n" + "="*80)
    print("DISTRIBUTED PRUNING ALGORITHM - MONTE CARLO PERFORMANCE EVALUATION")
    print("="*80)
    
    # Run experiments for different scenarios
    num_trials = 10
    scenarios = [
        {'graph_type': 'random', 'num_robots': 10},
        {'graph_type': 'random', 'num_robots': 15},
        {'graph_type': 'grid', 'num_robots': 9},
        {'graph_type': 'geometric', 'num_robots': 12},
    ]
    
    all_results = []
    
    for scenario in scenarios:
        stats = run_monte_carlo_experiment(
            num_trials=num_trials,
            num_robots=scenario['num_robots'],
            graph_type=scenario['graph_type']
        )
        all_results.append(stats)
        
        # Print summary
        print(f"\n{scenario['graph_type'].upper()} Graph ({scenario['num_robots']} nodes):")
        print(f"  Success Rate: {stats['success_rate']:.2f}%")
        print(f"  Connected graphs: {stats['connected_trials']}/{num_trials}")
        print(f"  Edge Reduction: {stats['edge_reduction_mean']:.2f} ± {stats['edge_reduction_std']:.2f}%")
        print(f"  Min Lambda2: {stats['min_lambda2_mean']:.4f} ± {stats['min_lambda2_std']:.4f}")
        print(f"  Runtime: {stats['runtime_mean']:.2f} ± {stats['runtime_std']:.2f}s")
    
    # Generate LaTeX table
    generate_latex_table(all_results)
    
    print("\n" + "="*80)
    print("EVALUATION COMPLETE")
    print("="*80 + "\n")
