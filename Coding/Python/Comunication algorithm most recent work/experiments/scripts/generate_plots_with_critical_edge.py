"""
Generate all plots from simulation experiment results including CriticalEdgeDetection.
This script creates comprehensive visualizations comparing 5 methods across 5 scenarios.
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from pathlib import Path
from typing import List, Dict
from datetime import datetime
import glob

# Try to import seaborn
try:
    import seaborn as sns
    HAS_SEABORN = True
except ImportError:
    HAS_SEABORN = False

# Set professional style
plt.style.use('default')
plt.rcParams.update({
    'figure.figsize': (7, 5),
    'font.size': 9,
    'axes.labelsize': 10,
    'axes.titlesize': 11,
    'xtick.labelsize': 8,
    'ytick.labelsize': 8,
    'legend.fontsize': 8,
    'font.family': 'sans-serif',
    'axes.grid': True,
    'grid.alpha': 0.3,
})

# Color scheme for methods (now with CriticalEdgeDetection)
METHOD_COLORS = {
    'ConcurrentPruning': '#2E86AB',       # Blue
    'AdjacencyConsensus': '#A23B72',      # Purple
    'CriticalEdgeDetection': '#C7254E',   # Red/Crimson (NEW)
    'FullGraph': '#F18F01',               # Orange
    'CentralizedMST': '#28A745'           # Green
}

METHOD_LABELS = {
    'ConcurrentPruning': 'Concurrent Pruning',
    'AdjacencyConsensus': 'Adjacency Consensus',
    'CriticalEdgeDetection': 'Critical Edge Detection (NEW)',
    'FullGraph': 'Full Graph',
    'CentralizedMST': 'Centralized MST'
}

SCENARIO_LABELS = {
    'A': 'Baseline',
    'B': 'Large Timestep',
    'C': 'Tight Radius',
    'D': 'High Robot Count',
    'E': 'Extreme Parameters'
}


def find_latest_results(directory: str = None) -> str:
    """Find the most recent results file."""
    if directory is None:
        directory = str(Path(__file__).parent.parent)
    
    # Look for both old and new format results files
    result_files = glob.glob(f"{directory}/comprehensive_results*.json")
    
    if not result_files:
        raise FileNotFoundError(f"No results files found in {directory}")
    
    # Return most recent (by modification time)
    latest = max(result_files, key=lambda f: Path(f).stat().st_mtime)
    return latest


def load_results(results_file: str) -> List[Dict]:
    """Load experiment results from JSON file."""
    with open(results_file, 'r') as f:
        return json.load(f)


def plot_summary_comparison(results: List[Dict], output_dir: Path):
    """Generate summary comparison plot (all metrics for all methods)."""
    print("Generating Summary Comparison Plots...")
    
    methods = sorted(set(r['method'] for r in results))
    scenarios = sorted(set(r['scenario'] for r in results))
    
    # Create figure with subplots for each metric
    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    axes = axes.flatten()
    
    # Metric 1: Success Rate
    ax = axes[0]
    success_rates = {}
    for method in methods:
        method_results = [r for r in results if r['method'] == method]
        success_count = sum(1 for r in method_results if r['success'])
        success_rates[method] = 100.0 * success_count / len(method_results)
    
    bars = ax.bar(range(len(methods)), [success_rates[m] for m in methods],
                  color=[METHOD_COLORS[m] for m in methods], alpha=0.7, edgecolor='black')
    ax.set_xticks(range(len(methods)))
    ax.set_xticklabels([METHOD_LABELS[m].replace(' (NEW)', '') for m in methods], rotation=45, ha='right')
    ax.set_ylabel('Success Rate (%)')
    ax.set_title('Success Rate by Method', fontweight='bold')
    ax.set_ylim([0, 105])
    ax.grid(True, alpha=0.3, axis='y')
    
    # Metric 2: Edge Reduction
    ax = axes[1]
    edge_reductions = {}
    for method in methods:
        method_results = [r for r in results if r['method'] == method and r['success']]
        if method_results:
            edge_reductions[method] = np.mean([r['edge_reduction_pct'] for r in method_results])
        else:
            edge_reductions[method] = 0
    
    bars = ax.bar(range(len(methods)), [edge_reductions[m] for m in methods],
                  color=[METHOD_COLORS[m] for m in methods], alpha=0.7, edgecolor='black')
    ax.set_xticks(range(len(methods)))
    ax.set_xticklabels([METHOD_LABELS[m].replace(' (NEW)', '') for m in methods], rotation=45, ha='right')
    ax.set_ylabel('Edge Reduction (%)')
    ax.set_title('Average Edge Reduction', fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # Metric 3: Min Lambda2 (Connectivity Safety)
    ax = axes[2]
    min_lambda2s = {}
    for method in methods:
        method_results = [r for r in results if r['method'] == method and r['success']]
        if method_results:
            min_lambda2s[method] = np.mean([r['min_lambda2'] for r in method_results])
        else:
            min_lambda2s[method] = 0
    
    bars = ax.bar(range(len(methods)), [min_lambda2s[m] for m in methods],
                  color=[METHOD_COLORS[m] for m in methods], alpha=0.7, edgecolor='black')
    ax.set_xticks(range(len(methods)))
    ax.set_xticklabels([METHOD_LABELS[m].replace(' (NEW)', '') for m in methods], rotation=45, ha='right')
    ax.set_ylabel('Min λ₂')
    ax.set_title('Minimum Connectivity (λ₂)', fontweight='bold')
    ax.axhline(y=0.2, color='red', linestyle='--', linewidth=1, alpha=0.5, label='Safety Threshold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # Metric 4: Mean Lambda2
    ax = axes[3]
    mean_lambda2s = {}
    for method in methods:
        method_results = [r for r in results if r['method'] == method and r['success']]
        if method_results:
            mean_lambda2s[method] = np.mean([r['mean_lambda2'] for r in method_results])
        else:
            mean_lambda2s[method] = 0
    
    bars = ax.bar(range(len(methods)), [mean_lambda2s[m] for m in methods],
                  color=[METHOD_COLORS[m] for m in methods], alpha=0.7, edgecolor='black')
    ax.set_xticks(range(len(methods)))
    ax.set_xticklabels([METHOD_LABELS[m].replace(' (NEW)', '') for m in methods], rotation=45, ha='right')
    ax.set_ylabel('Mean λ₂')
    ax.set_title('Average Connectivity', fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # Metric 5: Control Effort
    ax = axes[4]
    control_efforts = {}
    for method in methods:
        method_results = [r for r in results if r['method'] == method and r['success']]
        if method_results:
            control_efforts[method] = np.mean([r['mean_control_effort'] for r in method_results])
        else:
            control_efforts[method] = 0
    
    bars = ax.bar(range(len(methods)), [control_efforts[m] for m in methods],
                  color=[METHOD_COLORS[m] for m in methods], alpha=0.7, edgecolor='black')
    ax.set_xticks(range(len(methods)))
    ax.set_xticklabels([METHOD_LABELS[m].replace(' (NEW)', '') for m in methods], rotation=45, ha='right')
    ax.set_ylabel('Total Control Effort')
    ax.set_title('Cumulative Control Cost', fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # Metric 6: Lambda2 Violations
    ax = axes[5]
    violations = {}
    for method in methods:
        method_results = [r for r in results if r['method'] == method and r['success']]
        if method_results:
            violations[method] = np.mean([r['lambda2_violations'] for r in method_results])
        else:
            violations[method] = 0
    
    bars = ax.bar(range(len(methods)), [violations[m] for m in methods],
                  color=[METHOD_COLORS[m] for m in methods], alpha=0.7, edgecolor='black')
    ax.set_xticks(range(len(methods)))
    ax.set_xticklabels([METHOD_LABELS[m].replace(' (NEW)', '') for m in methods], rotation=45, ha='right')
    ax.set_ylabel('Number of Violations')
    ax.set_title('Connectivity Violations (λ₂ < 0.2)', fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    plt.suptitle('Comprehensive Method Comparison (All Scenarios)', fontsize=14, fontweight='bold', y=1.00)
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_0_summary_comparison.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_0_summary_comparison.png")


def plot_scenario_heatmap(results: List[Dict], output_dir: Path):
    """Generate heatmap showing method × scenario performance."""
    print("Generating Scenario Heatmap...")
    
    methods = sorted(set(r['method'] for r in results))
    scenarios = sorted(set(r['scenario'] for r in results))
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Metric 1: Min Lambda2 by method × scenario
    ax = axes[0, 0]
    min_lambda2_matrix = np.zeros((len(methods), len(scenarios)))
    for i, method in enumerate(methods):
        for j, scenario in enumerate(scenarios):
            method_results = [r for r in results if r['method'] == method 
                            and r['scenario'] == scenario and r['success']]
            if method_results:
                min_lambda2_matrix[i, j] = np.mean([r['min_lambda2'] for r in method_results])
    
    im = ax.imshow(min_lambda2_matrix, aspect='auto', cmap='RdYlGn', vmin=0, vmax=0.5)
    ax.set_xticks(range(len(scenarios)))
    ax.set_yticks(range(len(methods)))
    ax.set_xticklabels(scenarios)
    ax.set_yticklabels([METHOD_LABELS[m].replace(' (NEW)', '') for m in methods], fontsize=8)
    ax.set_xlabel('Scenario')
    ax.set_title('Min λ₂ (Higher is Better)', fontweight='bold')
    plt.colorbar(im, ax=ax)
    
    # Add text annotations
    for i in range(len(methods)):
        for j in range(len(scenarios)):
            text = ax.text(j, i, f'{min_lambda2_matrix[i, j]:.2f}',
                          ha="center", va="center", color="black", fontsize=8)
    
    # Metric 2: Edge Reduction
    ax = axes[0, 1]
    edge_reduction_matrix = np.zeros((len(methods), len(scenarios)))
    for i, method in enumerate(methods):
        for j, scenario in enumerate(scenarios):
            method_results = [r for r in results if r['method'] == method 
                            and r['scenario'] == scenario and r['success']]
            if method_results:
                edge_reduction_matrix[i, j] = np.mean([r['edge_reduction_pct'] for r in method_results])
    
    im = ax.imshow(edge_reduction_matrix, aspect='auto', cmap='YlOrRd', vmin=0, vmax=100)
    ax.set_xticks(range(len(scenarios)))
    ax.set_yticks(range(len(methods)))
    ax.set_xticklabels(scenarios)
    ax.set_yticklabels([METHOD_LABELS[m].replace(' (NEW)', '') for m in methods], fontsize=8)
    ax.set_xlabel('Scenario')
    ax.set_title('Edge Reduction (%) (Higher is Better)', fontweight='bold')
    plt.colorbar(im, ax=ax)
    
    for i in range(len(methods)):
        for j in range(len(scenarios)):
            text = ax.text(j, i, f'{edge_reduction_matrix[i, j]:.0f}',
                          ha="center", va="center", color="black", fontsize=8)
    
    # Metric 3: Success Rate
    ax = axes[1, 0]
    success_matrix = np.zeros((len(methods), len(scenarios)))
    for i, method in enumerate(methods):
        for j, scenario in enumerate(scenarios):
            method_results = [r for r in results if r['method'] == method 
                            and r['scenario'] == scenario]
            if method_results:
                success_count = sum(1 for r in method_results if r['success'])
                success_matrix[i, j] = 100.0 * success_count / len(method_results)
    
    im = ax.imshow(success_matrix, aspect='auto', cmap='RdYlGn', vmin=0, vmax=100)
    ax.set_xticks(range(len(scenarios)))
    ax.set_yticks(range(len(methods)))
    ax.set_xticklabels(scenarios)
    ax.set_yticklabels([METHOD_LABELS[m].replace(' (NEW)', '') for m in methods], fontsize=8)
    ax.set_xlabel('Scenario')
    ax.set_title('Success Rate (%) (Higher is Better)', fontweight='bold')
    plt.colorbar(im, ax=ax)
    
    for i in range(len(methods)):
        for j in range(len(scenarios)):
            text = ax.text(j, i, f'{success_matrix[i, j]:.0f}',
                          ha="center", va="center", color="black", fontsize=8)
    
    # Metric 4: Control Effort
    ax = axes[1, 1]
    effort_matrix = np.zeros((len(methods), len(scenarios)))
    for i, method in enumerate(methods):
        for j, scenario in enumerate(scenarios):
            method_results = [r for r in results if r['method'] == method 
                            and r['scenario'] == scenario and r['success']]
            if method_results:
                # Normalize for better visualization
                effort_matrix[i, j] = np.mean([r['mean_control_effort'] for r in method_results])
    
    # Normalize for visualization
    if effort_matrix.max() > 0:
        effort_matrix_norm = 100.0 * effort_matrix / effort_matrix.max()
    else:
        effort_matrix_norm = effort_matrix
    
    im = ax.imshow(effort_matrix_norm, aspect='auto', cmap='YlOrRd', vmin=0, vmax=100)
    ax.set_xticks(range(len(scenarios)))
    ax.set_yticks(range(len(methods)))
    ax.set_xticklabels(scenarios)
    ax.set_yticklabels([METHOD_LABELS[m].replace(' (NEW)', '') for m in methods], fontsize=8)
    ax.set_xlabel('Scenario')
    ax.set_title('Control Effort (Lower is Better)', fontweight='bold')
    plt.colorbar(im, ax=ax, label='Normalized Effort')
    
    for i in range(len(methods)):
        for j in range(len(scenarios)):
            text = ax.text(j, i, f'{effort_matrix[i, j]:.0f}',
                          ha="center", va="center", color="black", fontsize=7)
    
    plt.suptitle('Performance Heatmaps: Methods × Scenarios', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_0a_scenario_heatmap.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_0a_scenario_heatmap.png")


def plot_edge_evolution_detailed(results: List[Dict], output_dir: Path):
    """Plot edge count evolution."""
    print("Generating Edge Evolution Plot...")
    
    scenarios = ['A', 'B', 'C', 'D', 'E']
    methods = sorted(set(r['method'] for r in results))
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Subplot 1: All methods for Scenario A
    ax = axes[0, 0]
    for method in methods:
        trials = [r for r in results if r['method'] == method and r['scenario'] == 'A' and r['success']]
        if not trials:
            continue
        
        max_len = max(len(t['edge_counts_history']) for t in trials)
        edge_counts_padded = np.array([
            np.pad(t['edge_counts_history'], (0, max_len - len(t['edge_counts_history'])), mode='edge') 
            for t in trials
        ])
        mean_edges = np.mean(edge_counts_padded, axis=0)
        
        time = np.arange(len(mean_edges))
        ax.plot(time, mean_edges, label=METHOD_LABELS[method].replace(' (NEW)', ''),
               color=METHOD_COLORS[method], linewidth=2)
    
    ax.set_xlabel('Time Step')
    ax.set_ylabel('Number of Edges')
    ax.set_title('All Methods (Scenario A: Baseline)')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, framealpha=0.9)
    
    # Subplot 2: CriticalEdgeDetection for all scenarios
    ax = axes[0, 1]
    for scenario in scenarios:
        trials = [r for r in results if r['method'] == 'CriticalEdgeDetection' 
                 and r['scenario'] == scenario and r['success']]
        if not trials:
            continue
        
        max_len = max(len(t['edge_counts_history']) for t in trials)
        edge_counts_padded = np.array([
            np.pad(t['edge_counts_history'], (0, max_len - len(t['edge_counts_history'])), mode='edge') 
            for t in trials
        ])
        mean_edges = np.mean(edge_counts_padded, axis=0)
        
        time = np.arange(len(mean_edges))
        ax.plot(time, mean_edges, label=f'Scenario {scenario}', linewidth=2)
    
    ax.set_xlabel('Time Step')
    ax.set_ylabel('Number of Edges')
    ax.set_title('Critical Edge Detection (All Scenarios)')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, framealpha=0.9)
    
    # Subplot 3: Method comparison on difficult scenario (C)
    ax = axes[1, 0]
    for method in methods:
        trials = [r for r in results if r['method'] == method and r['scenario'] == 'C' and r['success']]
        if not trials:
            continue
        
        max_len = max(len(t['edge_counts_history']) for t in trials)
        edge_counts_padded = np.array([
            np.pad(t['edge_counts_history'], (0, max_len - len(t['edge_counts_history'])), mode='edge') 
            for t in trials
        ])
        mean_edges = np.mean(edge_counts_padded, axis=0)
        
        time = np.arange(len(mean_edges))
        ax.plot(time, mean_edges, label=METHOD_LABELS[method].replace(' (NEW)', ''),
               color=METHOD_COLORS[method], linewidth=2)
    
    ax.set_xlabel('Time Step')
    ax.set_ylabel('Number of Edges')
    ax.set_title('All Methods (Scenario C: Tight Radius)')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, framealpha=0.9)
    
    # Subplot 4: High robot count scenario (D)
    ax = axes[1, 1]
    for method in methods:
        trials = [r for r in results if r['method'] == method and r['scenario'] == 'D' and r['success']]
        if not trials:
            continue
        
        max_len = max(len(t['edge_counts_history']) for t in trials)
        edge_counts_padded = np.array([
            np.pad(t['edge_counts_history'], (0, max_len - len(t['edge_counts_history'])), mode='edge') 
            for t in trials
        ])
        mean_edges = np.mean(edge_counts_padded, axis=0)
        
        time = np.arange(len(mean_edges))
        ax.plot(time, mean_edges, label=METHOD_LABELS[method].replace(' (NEW)', ''),
               color=METHOD_COLORS[method], linewidth=2)
    
    ax.set_xlabel('Time Step')
    ax.set_ylabel('Number of Edges')
    ax.set_title('All Methods (Scenario D: High Robot Count)')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, framealpha=0.9)
    
    plt.suptitle('Edge Count Evolution Over Time', fontsize=13, fontweight='bold')
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_1_edge_evolution.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_1_edge_evolution.png")


def plot_edge_reduction_comparison(results: List[Dict], output_dir: Path):
    """Plot edge reduction (pruning efficiency) by method and scenario."""
    print("Generating Edge Reduction Comparison Plot...")
    
    scenarios = ['A', 'B', 'C', 'D', 'E']
    methods = sorted(set(r['method'] for r in results))
    
    fig, ax = plt.subplots(figsize=(12, 6))
    
    x = np.arange(len(scenarios))
    width = 0.15
    
    for idx, method in enumerate(methods):
        values = []
        for scenario in scenarios:
            trials = [r for r in results if r['method'] == method and r['scenario'] == scenario and r['success']]
            if trials:
                values.append(np.mean([r['edge_reduction_pct'] for r in trials]))
            else:
                values.append(0)  # Failed trials show 0 reduction
        
        ax.bar(x + idx * width, values, width, label=METHOD_LABELS[method], 
              color=METHOD_COLORS[method], alpha=0.85, edgecolor='black', linewidth=0.5)
    
    ax.set_xlabel('Scenario', fontsize=11, fontweight='bold')
    ax.set_ylabel('Edge Reduction (%)', fontsize=11, fontweight='bold')
    ax.set_title('Network Sparsification: Edge Reduction Comparison', fontsize=12, fontweight='bold')
    ax.set_xticks(x + width * 2)
    ax.set_xticklabels([SCENARIO_LABELS[s] for s in scenarios])
    ax.legend(fontsize=9, framealpha=0.95, loc='upper left')
    ax.grid(True, alpha=0.3, axis='y')
    ax.set_ylim([0, 105])
    
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_2_edge_reduction.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_2_edge_reduction.png")


def plot_success_rate_by_scenario(results: List[Dict], output_dir: Path):
    """Plot success rate heatmap: method vs scenario."""
    print("Generating Success Rate Heatmap...")
    
    scenarios = ['A', 'B', 'C', 'D', 'E']
    methods = sorted(set(r['method'] for r in results))
    
    # Build success rate matrix
    success_matrix = np.zeros((len(methods), len(scenarios)))
    for i, method in enumerate(methods):
        for j, scenario in enumerate(scenarios):
            trials = [r for r in results if r['method'] == method and r['scenario'] == scenario]
            if trials:
                success_rate = 100.0 * sum(1 for r in trials if r['success']) / len(trials)
                success_matrix[i, j] = success_rate
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    im = ax.imshow(success_matrix, cmap='RdYlGn', aspect='auto', vmin=0, vmax=100)
    
    # Set ticks
    ax.set_xticks(np.arange(len(scenarios)))
    ax.set_yticks(np.arange(len(methods)))
    ax.set_xticklabels([SCENARIO_LABELS[s] for s in scenarios])
    ax.set_yticklabels([METHOD_LABELS[m] for m in methods])
    
    # Rotate the tick labels
    plt.setp(ax.get_xticklabels(), rotation=45, ha="right", rotation_mode="anchor")
    
    # Add text annotations
    for i in range(len(methods)):
        for j in range(len(scenarios)):
            text = ax.text(j, i, f'{success_matrix[i, j]:.0f}%',
                          ha="center", va="center", color="black", fontsize=10, fontweight='bold')
    
    ax.set_title('Success Rate by Method and Scenario', fontsize=12, fontweight='bold', pad=15)
    ax.set_xlabel('Scenario', fontsize=11, fontweight='bold')
    ax.set_ylabel('Method', fontsize=11, fontweight='bold')
    
    # Add colorbar
    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label('Success Rate (%)', fontsize=10)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_3_success_rate_heatmap.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_3_success_rate_heatmap.png")


def plot_control_effort_analysis(results: List[Dict], output_dir: Path):
    """Plot control effort (computational cost) comparison."""
    print("Generating Control Effort Analysis Plot...")
    
    scenarios = ['A', 'B', 'C', 'D', 'E']
    methods = sorted(set(r['method'] for r in results))
    
    fig, ax = plt.subplots(figsize=(12, 6))
    
    x = np.arange(len(scenarios))
    width = 0.15
    
    for idx, method in enumerate(methods):
        values = []
        for scenario in scenarios:
            trials = [r for r in results if r['method'] == method and r['scenario'] == scenario and r['success']]
            if trials:
                # Use mean control effort across successful trials
                values.append(np.mean([r['mean_control_effort'] for r in trials]))
            else:
                values.append(0)
        
        ax.bar(x + idx * width, values, width, label=METHOD_LABELS[method], 
              color=METHOD_COLORS[method], alpha=0.85, edgecolor='black', linewidth=0.5)
    
    ax.set_xlabel('Scenario', fontsize=11, fontweight='bold')
    ax.set_ylabel('Mean Control Effort (per step)', fontsize=11, fontweight='bold')
    ax.set_title('Computational Cost: Control Effort Comparison', fontsize=12, fontweight='bold')
    ax.set_xticks(x + width * 2)
    ax.set_xticklabels([SCENARIO_LABELS[s] for s in scenarios])
    ax.legend(fontsize=9, framealpha=0.95, loc='upper left')
    ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_4_control_effort.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_4_control_effort.png")


def plot_convergence_speed(results: List[Dict], output_dir: Path):
    """Plot convergence speed: steps to reach spanning tree."""
    print("Generating Convergence Speed Plot...")
    
    scenarios = ['A', 'B', 'C', 'D', 'E']
    methods = sorted(set(r['method'] for r in results))
    
    fig, ax = plt.subplots(figsize=(12, 6))
    
    x = np.arange(len(scenarios))
    width = 0.15
    
    for idx, method in enumerate(methods):
        values = []
        for scenario in scenarios:
            trials = [r for r in results if r['method'] == method and r['scenario'] == scenario and r['success']]
            if trials:
                # Use steps executed (when converged to spanning tree)
                values.append(np.mean([r['steps_executed'] for r in trials]))
            else:
                values.append(300)  # Max steps if failed
        
        ax.bar(x + idx * width, values, width, label=METHOD_LABELS[method], 
              color=METHOD_COLORS[method], alpha=0.85, edgecolor='black', linewidth=0.5)
    
    ax.set_xlabel('Scenario', fontsize=11, fontweight='bold')
    ax.set_ylabel('Steps to Convergence', fontsize=11, fontweight='bold')
    ax.set_title('Algorithm Efficiency: Convergence Speed (Lower is Better)', fontsize=12, fontweight='bold')
    ax.set_xticks(x + width * 2)
    ax.set_xticklabels([SCENARIO_LABELS[s] for s in scenarios])
    ax.legend(fontsize=9, framealpha=0.95, loc='upper left')
    ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_5_convergence_speed.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_5_convergence_speed.png")



def plot_efficiency_metrics(results: List[Dict], output_dir: Path):
    """Plot efficiency metrics: edge reduction vs connectivity."""
    print("Generating Efficiency Metrics Plot...")
    
    scenarios = ['A', 'B', 'C', 'D', 'E']
    methods = sorted(set(r['method'] for r in results))
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Plot 1: Edge Reduction by Scenario
    ax = axes[0, 0]
    x = np.arange(len(scenarios))
    width = 0.15
    for idx, method in enumerate(methods):
        values = []
        for scenario in scenarios:
            trials = [r for r in results if r['method'] == method and r['scenario'] == scenario and r['success']]
            if trials:
                values.append(np.mean([r['edge_reduction_pct'] for r in trials]))
            else:
                values.append(0)
        ax.bar(x + idx * width, values, width, label=METHOD_LABELS[method].replace(' (NEW)', ''),
              color=METHOD_COLORS[method], alpha=0.8, edgecolor='black', linewidth=0.5)
    
    ax.set_xlabel('Scenario')
    ax.set_ylabel('Edge Reduction (%)')
    ax.set_title('Network Sparsity Achieved')
    ax.set_xticks(x + width * 2)
    ax.set_xticklabels(scenarios)
    ax.legend(fontsize=7, framealpha=0.9, ncol=2)
    ax.grid(True, alpha=0.3, axis='y')
    
    # Plot 2: Control Effort by Scenario
    ax = axes[0, 1]
    for idx, method in enumerate(methods):
        values = []
        for scenario in scenarios:
            trials = [r for r in results if r['method'] == method and r['scenario'] == scenario and r['success']]
            if trials:
                values.append(np.mean([r['mean_control_effort'] for r in trials]))
            else:
                values.append(0)
        ax.bar(x + idx * width, values, width, label=METHOD_LABELS[method].replace(' (NEW)', ''),
              color=METHOD_COLORS[method], alpha=0.8, edgecolor='black', linewidth=0.5)
    
    ax.set_xlabel('Scenario')
    ax.set_ylabel('Total Control Effort')
    ax.set_title('Control Complexity')
    ax.set_xticks(x + width * 2)
    ax.set_xticklabels(scenarios)
    ax.legend(fontsize=7, framealpha=0.9, ncol=2)
    ax.grid(True, alpha=0.3, axis='y')
    
    # Plot 3: Trade-off: Edge Reduction vs Connectivity
    ax = axes[1, 0]
    for method in methods:
        edge_reductions = []
        mean_lambda2s = []
        for scenario in scenarios:
            trials = [r for r in results if r['method'] == method and r['scenario'] == scenario and r['success']]
            if trials:
                edge_reductions.append(np.mean([r['edge_reduction_pct'] for r in trials]))
                mean_lambda2s.append(np.mean([r['mean_lambda2'] for r in trials]))
        
        ax.scatter(edge_reductions, mean_lambda2s, s=150, label=METHOD_LABELS[method].replace(' (NEW)', ''),
                  color=METHOD_COLORS[method], alpha=0.7, edgecolors='black', linewidth=1.5)
    
    ax.set_xlabel('Edge Reduction (%)')
    ax.set_ylabel('Mean λ₂')
    ax.set_title('Pareto Trade-off: Sparsity vs Connectivity')
    ax.legend(fontsize=8, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0.2, color='red', linestyle='--', linewidth=1, alpha=0.5)
    
    # Plot 4: Pruning Events
    ax = axes[1, 1]
    pruning_by_scenario = {scenario: {} for scenario in scenarios}
    for method in methods:
        for scenario in scenarios:
            trials = [r for r in results if r['method'] == method and r['scenario'] == scenario and r['success']]
            if trials:
                pruning_by_scenario[scenario][method] = np.mean([r['pruning_events_count'] for r in trials])
    
    x = np.arange(len(scenarios))
    for idx, method in enumerate(methods):
        values = [pruning_by_scenario[s].get(method, 0) for s in scenarios]
        ax.bar(x + idx * width, values, width, label=METHOD_LABELS[method].replace(' (NEW)', ''),
              color=METHOD_COLORS[method], alpha=0.8, edgecolor='black', linewidth=0.5)
    
    ax.set_xlabel('Scenario')
    ax.set_ylabel('Average Pruning Events')
    ax.set_title('Pruning Activity')
    ax.set_xticks(x + width * 2)
    ax.set_xticklabels(scenarios)
    ax.legend(fontsize=7, framealpha=0.9, ncol=2)
    ax.grid(True, alpha=0.3, axis='y')
    
    plt.suptitle('Efficiency Metrics Analysis', fontsize=13, fontweight='bold')
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_3_efficiency.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_3_efficiency.png")


def generate_summary_statistics(results: List[Dict], output_dir: Path):
    """Generate text summary of results."""
    print("Generating Summary Statistics...")
    
    methods = sorted(set(r['method'] for r in results))
    scenarios = sorted(set(r['scenario'] for r in results))
    
    output_file = output_dir / 'summary_statistics_with_critical_edge.txt'
    
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write("="*80 + "\n")
        f.write("COMPREHENSIVE EXPERIMENT RESULTS (WITH CRITICAL EDGE DETECTION)\n")
        f.write("="*80 + "\n\n")
        
        # Overall success rates
        f.write("SUCCESS RATES:\n")
        f.write("-"*80 + "\n")
        for method in methods:
            method_results = [r for r in results if r['method'] == method]
            success_count = sum(1 for r in method_results if r['success'])
            success_rate = 100.0 * success_count / len(method_results)
            f.write(f"{METHOD_LABELS[method]:40s}: {success_rate:6.1f}% ({success_count}/{len(method_results)})\n")
        
        f.write("\n" + "="*80 + "\n")
        f.write("DETAILED METRICS BY METHOD\n")
        f.write("="*80 + "\n\n")
        
        for method in methods:
            f.write(f"\n{METHOD_LABELS[method]}\n")
            f.write("-"*80 + "\n")
            
            method_results = [r for r in results if r['method'] == method and r['success']]
            
            if not method_results:
                f.write("  No successful runs\n")
                continue
            
            # Compute statistics
            edge_reductions = [r['edge_reduction_pct'] for r in method_results]
            min_lambda2s = [r['min_lambda2'] for r in method_results]
            mean_lambda2s = [r['mean_lambda2'] for r in method_results]
            control_efforts = [r['mean_control_effort'] for r in method_results]
            
            f.write(f"\nEdge Reduction (%):\n")
            f.write(f"  Mean: {np.mean(edge_reductions):8.2f}%  Std: {np.std(edge_reductions):8.2f}%\n")
            f.write(f"  Min:  {np.min(edge_reductions):8.2f}%  Max: {np.max(edge_reductions):8.2f}%\n")
            
            f.write(f"\nMinimum λ₂ (Connectivity Safety):\n")
            f.write(f"  Mean: {np.mean(min_lambda2s):8.4f}  Std: {np.std(min_lambda2s):8.4f}\n")
            f.write(f"  Min:  {np.min(min_lambda2s):8.4f}  Max: {np.max(min_lambda2s):8.4f}\n")
            
            f.write(f"\nMean λ₂:\n")
            f.write(f"  Mean: {np.mean(mean_lambda2s):8.4f}  Std: {np.std(mean_lambda2s):8.4f}\n")
            
            f.write(f"\nTotal Control Effort:\n")
            f.write(f"  Mean: {np.mean(control_efforts):10.2f}  Std: {np.std(control_efforts):10.2f}\n")
            f.write(f"  Min:  {np.min(control_efforts):10.2f}  Max: {np.max(control_efforts):10.2f}\n")
        
        f.write("\n" + "="*80 + "\n")
        f.write("CRITICAL EDGE DETECTION - SCENARIO BREAKDOWN\n")
        f.write("="*80 + "\n\n")
        
        for scenario in scenarios:
            scenario_results = [r for r in results if r['method'] == 'CriticalEdgeDetection' 
                              and r['scenario'] == scenario and r['success']]
            
            f.write(f"\nScenario {scenario}: {SCENARIO_LABELS[scenario]}\n")
            f.write("-"*80 + "\n")
            
            if scenario_results:
                f.write(f"  Successful Trials: {len(scenario_results)}\n")
                f.write(f"  Edge Reduction:    {np.mean([r['edge_reduction_pct'] for r in scenario_results]):6.2f}%\n")
                f.write(f"  Min λ₂:             {np.mean([r['min_lambda2'] for r in scenario_results]):8.4f}\n")
                f.write(f"  Mean λ₂:            {np.mean([r['mean_lambda2'] for r in scenario_results]):8.4f}\n")
                f.write(f"  Control Effort:     {np.mean([r['mean_control_effort'] for r in scenario_results]):10.2f}\n")
            else:
                f.write("  No successful trials\n")
    
    print(f"  ✓ Saved: {output_file}")
    return str(output_file)


def main():
    """Main plotting routine."""
    print("\n" + "="*80)
    print("GENERATING PLOTS FROM EXPERIMENT RESULTS")
    print("="*80 + "\n")
    
    # Find results file
    try:
        results_file = find_latest_results()
        print(f"✓ Found results file: {results_file}\n")
    except FileNotFoundError as e:
        print(f"✗ Error: {e}")
        return
    
    # Load results
    print("Loading results...")
    results = load_results(results_file)
    print(f"✓ Loaded {len(results)} simulation results\n")
    
    # Create output directory
    output_dir = Path(results_file).parent
    print(f"Output directory: {output_dir}\n")
    
    # Generate all plots
    print("Generating plots...\n")
    
    try:
        plot_summary_comparison(results, output_dir)
        plot_scenario_heatmap(results, output_dir)
        plot_edge_evolution_detailed(results, output_dir)
        plot_edge_reduction_comparison(results, output_dir)
        plot_success_rate_by_scenario(results, output_dir)
        plot_control_effort_analysis(results, output_dir)
        plot_convergence_speed(results, output_dir)
        generate_summary_statistics(results, output_dir)
        
        print("\n" + "="*80)
        print("✓ ALL PLOTS GENERATED SUCCESSFULLY")
        print("="*80)
        print(f"\nOutput files saved to: {output_dir}")
        print("\nGenerated plots:")
        print("  - plot_0_summary_comparison.png")
        print("  - plot_0a_scenario_heatmap.png")
        print("  - plot_1_edge_evolution.png")
        print("  - plot_2_edge_reduction.png")
        print("  - plot_3_success_rate_heatmap.png")
        print("  - plot_4_control_effort.png")
        print("  - plot_5_convergence_speed.png")
        print("  - summary_statistics_with_critical_edge.txt")
        print("\n")
        
    except Exception as e:
        print(f"\n✗ Error generating plots: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
