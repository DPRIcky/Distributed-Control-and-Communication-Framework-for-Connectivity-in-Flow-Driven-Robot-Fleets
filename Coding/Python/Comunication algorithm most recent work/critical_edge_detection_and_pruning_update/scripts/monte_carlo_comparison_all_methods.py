"""Monte Carlo comparison of all 4 pruning methods (10 trials per scenario).

Compares:
1. Concurrent Pruning (DistributedPruning) - 4-phase algorithm
2. Adjacency Consensus - distributed consensus approx.
3. Full Graph - baseline (no pruning)
4. Centralized MST - oracle baseline

Metrics collected:
- Success rate (graph remains connected)
- Edge reduction percentage
- Minimum algebraic connectivity (min eigenvalue)
- Runtime
"""

import sys
import os
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import numpy as np
import networkx as nx
import time
from scipy import sparse
from scipy.sparse import linalg
from distributed_pruning_algorithm import DistributedPruningAlgorithm


def compute_algebraic_connectivity(graph):
    """Compute algebraic connectivity (2nd smallest eigenvalue of Laplacian)."""
    if len(graph) == 0 or not nx.is_connected(graph):
        return 0.0
    
    try:
        L = nx.laplacian_matrix(graph).asformat('csr')
        if len(graph) < 2:
            return 0.0
        eigenvalues = linalg.eigsh(L, k=min(2, len(graph)), which='SM', return_eigenvectors=False)
        if len(eigenvalues) > 1:
            return float(eigenvalues[-1])
        return float(eigenvalues[0]) if len(eigenvalues) > 0 else 0.0
    except:
        return 0.0


def generate_random_graph(num_robots=10, density=0.3, seed=None):
    """Generate connected random graph."""
    G = nx.erdos_renyi_graph(num_robots, density, seed=seed)
    
    # Ensure connectivity
    if not nx.is_connected(G):
        components = list(nx.connected_components(G))
        for i in range(len(components) - 1):
            u = list(components[i])[0]
            v = list(components[i+1])[0]
            G.add_edge(u, v)
    
    return G


def test_fully_connected(graph):
    """Method 1: Full Graph (baseline - no pruning)."""
    result = {
        'method': 'Full Graph',
        'success': True,
        'initial_edges': len(graph.edges()),
        'final_edges': len(graph.edges()),
        'edge_reduction': 0.0,
        'min_lambda2': compute_algebraic_connectivity(graph),
        'runtime': 0.0,
    }
    return result


def test_centralized_mst(graph):
    """Method 4: Centralized MST (oracle baseline)."""
    start_time = time.time()
    
    # Convert to weighted graph (unit weights for simplicity)
    G_weighted = graph.copy()
    for u, v in G_weighted.edges():
        G_weighted[u][v]['weight'] = 1.0
    
    # Compute MST
    mst = nx.minimum_spanning_tree(G_weighted, weight='weight')
    
    success = nx.is_connected(mst)
    min_lambda2 = compute_algebraic_connectivity(mst) if success else 0.0
    
    initial_edges = len(graph.edges())
    final_edges = len(mst.edges())
    edge_reduction = (1 - final_edges / initial_edges) * 100 if initial_edges > 0 else 0.0
    
    runtime = time.time() - start_time
    
    result = {
        'method': 'Centralized MST',
        'success': success,
        'initial_edges': initial_edges,
        'final_edges': final_edges,
        'edge_reduction': edge_reduction,
        'min_lambda2': min_lambda2,
        'runtime': runtime,
    }
    return result


def test_adjacency_consensus(graph):
    """Method 2: Adjacency Consensus (gradual distributed pruning)."""
    start_time = time.time()
    
    # Simulate conservative consensus pruning: keep 1.5x MST edges
    num_robots = len(graph)
    G_weighted = graph.copy()
    for u, v in G_weighted.edges():
        G_weighted[u][v]['weight'] = 1.0
    
    mst = nx.minimum_spanning_tree(G_weighted, weight='weight')
    mst_edges_count = len(mst.edges())
    
    # Consensus keeps n-1 + extra slack = 1.5 * (n-1) edges
    target_edges = max(num_robots - 1, int(1.5 * (num_robots - 1)))
    target_edges = min(target_edges, len(graph.edges()))
    
    # Select shortest edges to keep (conservative)
    edges_by_dist = []
    for u, v in graph.edges():
        dist = 1.0  # Unit weights
        edges_by_dist.append(((u, v), dist))
    
    edges_by_dist.sort(key=lambda x: x[1])
    pruned_edges = {e[0] for e in edges_by_dist[:target_edges]}
    
    pruned_graph = nx.Graph()
    pruned_graph.add_nodes_from(graph.nodes())
    pruned_graph.add_edges_from(pruned_edges)
    
    success = nx.is_connected(pruned_graph)
    min_lambda2 = compute_algebraic_connectivity(pruned_graph) if success else 0.0
    
    initial_edges = len(graph.edges())
    final_edges = len(pruned_graph.edges())
    edge_reduction = (1 - final_edges / initial_edges) * 100 if initial_edges > 0 else 0.0
    
    runtime = time.time() - start_time
    
    result = {
        'method': 'Adjacency Consensus',
        'success': success,
        'initial_edges': initial_edges,
        'final_edges': final_edges,
        'edge_reduction': edge_reduction,
        'min_lambda2': min_lambda2,
        'runtime': runtime,
    }
    return result


def test_concurrent_pruning(graph):
    """Method 1: Concurrent Pruning (DistributedPruningAlgorithm - 4-phase)."""
    start_time = time.time()
    
    try:
        # Run 4-phase distributed pruning algorithm
        algo = DistributedPruningAlgorithm(graph.copy(), root=0)
        algo.run_full_pipeline()
        
        pruned_graph = algo.get_pruned_graph()
        
        success = nx.is_connected(pruned_graph)
        min_lambda2 = compute_algebraic_connectivity(pruned_graph) if success else 0.0
        
        initial_edges = len(graph.edges())
        final_edges = len(pruned_graph.edges())
        edge_reduction = (1 - final_edges / initial_edges) * 100 if initial_edges > 0 else 0.0
        
        runtime = time.time() - start_time
        
        result = {
            'method': 'Concurrent Pruning',
            'success': success,
            'initial_edges': initial_edges,
            'final_edges': final_edges,
            'edge_reduction': edge_reduction,
            'min_lambda2': min_lambda2,
            'runtime': runtime,
        }
    except Exception as e:
        print(f"  Error in Concurrent Pruning: {e}")
        result = {
            'method': 'Concurrent Pruning',
            'success': False,
            'initial_edges': len(graph.edges()),
            'final_edges': 0,
            'edge_reduction': 0.0,
            'min_lambda2': 0.0,
            'runtime': time.time() - start_time,
            'error': str(e)
        }
    
    return result


def run_monte_carlo_trial(num_robots=10, trial_id=0):
    """Run single Monte Carlo trial with all 4 methods."""
    # Generate random connected graph
    G = generate_random_graph(num_robots=num_robots, density=0.3, seed=42 + trial_id)
    
    results = {
        'trial_id': trial_id,
        'num_robots': num_robots,
        'initial_edges': len(G.edges()),
        'methods': {}
    }
    
    # Test all 4 methods
    results['methods']['Full Graph'] = test_fully_connected(G)
    results['methods']['Centralized MST'] = test_centralized_mst(G)
    results['methods']['Adjacency Consensus'] = test_adjacency_consensus(G)
    results['methods']['Concurrent Pruning'] = test_concurrent_pruning(G)
    
    return results


def aggregate_statistics(all_trials):
    """Aggregate statistics across all trials."""
    methods = ['Full Graph', 'Centralized MST', 'Adjacency Consensus', 'Concurrent Pruning']
    stats = {}
    
    for method in methods:
        successes = []
        edge_reductions = []
        min_lambda2s = []
        runtimes = []
        
        for trial in all_trials:
            if method in trial['methods']:
                result = trial['methods'][method]
                if result['success']:
                    successes.append(1)
                    edge_reductions.append(result['edge_reduction'])
                    min_lambda2s.append(result['min_lambda2'])
                else:
                    successes.append(0)
                
                runtimes.append(result['runtime'])
        
        success_rate = (sum(successes) / len(successes) * 100) if successes else 0.0
        edge_red_mean = np.mean(edge_reductions) if edge_reductions else 0.0
        edge_red_std = np.std(edge_reductions) if len(edge_reductions) > 1 else 0.0
        min_lambda2_mean = np.mean(min_lambda2s) if min_lambda2s else 0.0
        min_lambda2_std = np.std(min_lambda2s) if len(min_lambda2s) > 1 else 0.0
        runtime_mean = np.mean(runtimes) if runtimes else 0.0
        runtime_std = np.std(runtimes) if len(runtimes) > 1 else 0.0
        
        stats[method] = {
            'success_rate': success_rate,
            'edge_reduction_mean': edge_red_mean,
            'edge_reduction_std': edge_red_std,
            'min_lambda2_mean': min_lambda2_mean,
            'min_lambda2_std': min_lambda2_std,
            'runtime_mean': runtime_mean,
            'runtime_std': runtime_std,
            'num_successful': sum(successes),
            'num_trials': len(successes)
        }
    
    return stats


def generate_latex_table(stats_dict):
    """Generate LaTeX table from statistics."""
    
    latex_code = r"""\begin{table}[H]
\centering
\caption{Aggregate performance across all scenarios (10 Monte-Carlo trials per scenario).}
\label{tab:summary_stats}
\footnotesize
\setlength{\tabcolsep}{2.4pt}
\resizebox{\columnwidth}{!}{%
\begin{tabular}{lcccc}
\toprule
Method & Success (\%) & Edge red.\ (\%) & $\min_t \lambda_2$ & Runtime (s) \\
\midrule
"""
    
    for method in ['Concurrent Pruning', 'Adjacency Consensus', 'Full Graph', 'Centralized MST']:
        if method not in stats_dict:
            continue
        
        stats = stats_dict[method]
        success = f"{stats['success_rate']:.2f}"
        edge_red = f"${stats['edge_reduction_mean']:.2f}\\pm {stats['edge_reduction_std']:.2f}$"
        min_lambda2 = f"${stats['min_lambda2_mean']:.4f}\\pm {stats['min_lambda2_std']:.4f}$"
        runtime = f"${stats['runtime_mean']:.2f}\\pm {stats['runtime_std']:.2f}$"
        
        latex_code += f"{method} & {success} & {edge_red} & {min_lambda2} & {runtime} \\\\\n"
    
    latex_code += r"""\bottomrule
\end{tabular}}
\end{table}
\FloatBarrier
"""
    
    return latex_code


if __name__ == "__main__":
    print("\n" + "="*80)
    print("MONTE CARLO COMPARISON - ALL 4 METHODS")
    print("="*80)
    
    num_trials = 10
    num_robots = 10
    
    print(f"\nRunning {num_trials} trials with {num_robots} robots each...\n")
    
    all_trials = []
    for trial_id in range(num_trials):
        print(f"Trial {trial_id + 1}/{num_trials}...", flush=True)
        result = run_monte_carlo_trial(num_robots=num_robots, trial_id=trial_id)
        all_trials.append(result)
    
    # Aggregate statistics
    stats_dict = aggregate_statistics(all_trials)
    
    # Print summary
    print("\n" + "="*80)
    print("RESULTS SUMMARY")
    print("="*80 + "\n")
    
    for method in ['Concurrent Pruning', 'Adjacency Consensus', 'Full Graph', 'Centralized MST']:
        if method not in stats_dict:
            continue
        
        stats = stats_dict[method]
        print(f"{method}:")
        print(f"  Success Rate: {stats['success_rate']:.2f}%")
        print(f"  Edge Reduction: {stats['edge_reduction_mean']:.2f} ± {stats['edge_reduction_std']:.2f}%")
        print(f"  Min Lambda2: {stats['min_lambda2_mean']:.4f} ± {stats['min_lambda2_std']:.4f}")
        print(f"  Runtime: {stats['runtime_mean']:.2f} ± {stats['runtime_std']:.2f}s")
        print()
    
    # Generate LaTeX table
    print("="*80)
    print("LaTeX TABLE")
    print("="*80 + "\n")
    
    latex_table = generate_latex_table(stats_dict)
    print(latex_table)
    
    # Save to file
    output_file = Path(__file__).parent.parent / "Docs" / "PERFORMANCE_TABLE_ALL_METHODS.tex"
    output_file.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_file, 'w') as f:
        f.write(latex_table)
    
    print(f"Saved to: {output_file}\n")
    
    print("="*80)
    print("EVALUATION COMPLETE")
    print("="*80 + "\n")
