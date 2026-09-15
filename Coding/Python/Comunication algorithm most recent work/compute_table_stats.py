"""Compute summary statistics for Table II from batch results."""
import json
import numpy as np
from pathlib import Path

# Load the most recent batch results
results_file = Path('concurrent_pruning/results/batch_results_concurrent_20260218_145519.json')

with open(results_file, 'r') as f:
    data = json.load(f)

results = data['results']

# Group by method
methods = {
    'concurrent': 'Concurrent Pruning',
    'full_graph': 'Full Graph',
    'centralized_mst': 'Centralized MST'
}

# Also check for adjacency_consensus method
if any(r.get('method') == 'adjacency_consensus' for r in results):
    methods['adjacency_consensus'] = 'Adjacency Consensus'

print("="*100)
print("TABLE II: Aggregate Performance Across All Scenarios")
print("="*100)
print(f"{'Method':<25} {'Success (%)':<15} {'Edge red. (%)':<25} {'min_t λ₂':<25} {'Runtime (s)':<20}")
print("-"*100)

for method_key, method_name in methods.items():
    method_results = [r for r in results if r.get('method') == method_key and r.get('success', False)]
    
    if method_results:
        # Calculate success rate across all runs (including failures)
        all_runs = [r for r in results if r.get('method') == method_key]
        success_count = sum(1 for r in all_runs if r.get('success', False))
        success_rate = 100 * success_count / len(all_runs)
        
        # Edge reduction
        edge_red = [r['edge_reduction_pct'] for r in method_results if 'edge_reduction_pct' in r]
        edge_red_mean = np.mean(edge_red)
        edge_red_std = np.std(edge_red)
        
        # Min lambda2
        min_lambda2 = [r['min_lambda2'] for r in method_results if 'min_lambda2' in r]
        min_lambda2_mean = np.mean(min_lambda2)
        min_lambda2_std = np.std(min_lambda2)
        
        # Runtime
        runtime = [r['elapsed_time'] for r in method_results if 'elapsed_time' in r]
        runtime_mean = np.mean(runtime)
        runtime_std = np.std(runtime)
        
        print(f"{method_name:<25} {success_rate:<15.1f} "
              f"{edge_red_mean:.2f}±{edge_red_std:.2f}{'':<13} "
              f"{min_lambda2_mean:.4f}±{min_lambda2_std:.4f}{'':<11} "
              f"{runtime_mean:.2f}±{runtime_std:.2f}")

print("="*100)

# Generate LaTeX table format
print("\n\nLaTeX Format:")
print("-"*100)
print("\\begin{table}[H]")
print("\\centering")
print("\\caption{Aggregate performance across all scenarios (10 Monte-Carlo trials per scenario).}")
print("\\label{tab:summary_stats}")
print("\\footnotesize")
print("\\setlength{\\tabcolsep}{2.4pt}")
print("\\resizebox{\\columnwidth}{!}{%")
print("\\begin{tabular}{lcccc}")
print("\\toprule")
print("Method & Success (\\%) & Edge red.\\ (\\%) & $\\min_t \\lambda_2$ & Runtime (s) \\\\")
print("\\midrule")

for method_key, method_name in methods.items():
    method_results = [r for r in results if r.get('method') == method_key and r.get('success', False)]
    
    if method_results:
        # Calculate success rate
        all_runs = [r for r in results if r.get('method') == method_key]
        success_count = sum(1 for r in all_runs if r.get('success', False))
        success_rate = 100 * success_count / len(all_runs)
        
        # Edge reduction
        edge_red = [r['edge_reduction_pct'] for r in method_results if 'edge_reduction_pct' in r]
        edge_red_mean = np.mean(edge_red)
        edge_red_std = np.std(edge_red)
        
        # Min lambda2
        min_lambda2 = [r['min_lambda2'] for r in method_results if 'min_lambda2' in r]
        min_lambda2_mean = np.mean(min_lambda2)
        min_lambda2_std = np.std(min_lambda2)
        
        # Runtime
        runtime = [r['elapsed_time'] for r in method_results if 'elapsed_time' in r]
        runtime_mean = np.mean(runtime)
        runtime_std = np.std(runtime)
        
        print(f"{method_name} & {success_rate:.1f} & "
              f"${edge_red_mean:.2f}\\pm {edge_red_std:.2f}$ & "
              f"${min_lambda2_mean:.4f}\\pm {min_lambda2_std:.4f}$ & "
              f"${runtime_mean:.2f}\\pm {runtime_std:.2f}$ \\\\")

print("\\bottomrule")
print("\\end{tabular}}")
print("\\end{table}")
print("\\FloatBarrier")
