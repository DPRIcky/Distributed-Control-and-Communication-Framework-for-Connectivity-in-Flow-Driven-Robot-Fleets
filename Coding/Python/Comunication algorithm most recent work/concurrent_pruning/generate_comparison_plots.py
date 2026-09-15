"""Generate comparison plots from batch comparison results.

Creates publication-quality figures comparing ConcurrentPruning vs baselines:
- Box plots of edge reduction across methods
- Box plots of lambda2 (connectivity) across methods
- Time series showing pruning dynamics
- Summary statistics tables

ENHANCED VISUALIZATIONS:
For more insightful plots, use:
    from visualization.enhanced_comparison_plots import generate_all_enhanced_plots
    generate_all_enhanced_plots(results_file, output_dir)

The enhanced module provides:
- Time-series dynamics with confidence bands
- Trade-off analysis (scatter with ellipses)
- Violin distributions (better than box plots)
- Performance profiles (reliability curves)
- CDFs (statistical comparison)
- Correlation heatmaps

See docs/ENHANCED_VISUALIZATION_GUIDE.md for details.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import json
import argparse
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from collections import defaultdict

# Set publication-quality parameters
plt.rcParams.update({
    'font.size': 11,
    'font.family': 'serif',
    'axes.labelsize': 12,
    'axes.titlesize': 13,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 10,
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight'
})

# Color scheme
METHOD_COLORS = {
    'ConcurrentPruning': '#e74c3c',      # Red (our method)
    'AdjacencyConsensus': '#f39c12',     # Orange (prior method)
    'FullGraph': '#95a5a6',              # Gray
    'CentralizedMST': '#3498db'          # Blue
}

METHOD_LABELS = {
    'ConcurrentPruning': 'Concurrent\nPruning',
    'AdjacencyConsensus': 'Adjacency\nConsensus',
    'FullGraph': 'Full\nGraph',
    'CentralizedMST': 'Centralized\nMST'
}


def load_results(filepath: str):
    """Load batch comparison results from JSON."""
    with open(filepath, 'r') as f:
        data = json.load(f)
    return data


def generate_box_plots(results_file: str, output_dir: str = 'concurrent_pruning/figures'):
    """Generate box plot comparisons across all metrics.
    
    Args:
        results_file: Path to batch results JSON
        output_dir: Output directory for figures
    """
    data = load_results(results_file)
    results = data['results']
    metadata = data['metadata']
    
    # Create output directory
    output_path = Path(output_dir)
    output_path.mkdir(exist_ok=True, parents=True)
    
    # Filter successful runs
    successful = [r for r in results if r.get('success', False) and 'error' not in r]
    
    # Group by method
    methods = sorted(set(r['method'] for r in successful))
    
    print(f"Loaded {len(successful)}/{len(results)} successful trials")
    print(f"Methods: {methods}")
    print(f"Seeds per method: {len([r for r in successful if r['method'] == methods[0]])}")
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    
    # 1. Final Edge Count
    ax = axes[0, 0]
    plot_data = []
    labels = []
    colors = []
    
    for method in methods:
        method_results = [r for r in successful if r['method'] == method]
        values = [r['final_edges'] for r in method_results]
        plot_data.append(values)
        labels.append(METHOD_LABELS.get(method, method))
        colors.append(METHOD_COLORS.get(method, '#95a5a6'))
    
    bp = ax.boxplot(plot_data, labels=labels, patch_artist=True, showfliers=True)
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    ax.set_ylabel('Final Edge Count', fontweight='bold')
    ax.set_title('Edge Count Distribution', fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # 2. Edge Reduction Percentage
    ax = axes[0, 1]
    plot_data = []
    
    for method in methods:
        method_results = [r for r in successful if r['method'] == method]
        values = [r['edge_reduction_pct'] for r in method_results]
        plot_data.append(values)
    
    bp = ax.boxplot(plot_data, labels=labels, patch_artist=True, showfliers=True)
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    ax.set_ylabel('Edge Reduction (%)', fontweight='bold')
    ax.set_title('Pruning Efficiency', fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # 3. Minimum Lambda2
    ax = axes[0, 2]
    plot_data = []
    
    for method in methods:
        method_results = [r for r in successful if r['method'] == method]
        values = [r['min_lambda2'] for r in method_results]
        plot_data.append(values)
    
    bp = ax.boxplot(plot_data, labels=labels, patch_artist=True, showfliers=True)
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    ax.axhline(y=0.3, color='orange', linestyle='--', linewidth=2, 
              label='Safety Threshold', alpha=0.7)
    ax.set_ylabel('Minimum λ₂', fontweight='bold')
    ax.set_title('Connectivity Preservation', fontweight='bold')
    ax.legend(loc='best', fontsize=9)
    ax.grid(True, alpha=0.3, axis='y')
    
    # 4. Mean Lambda2
    ax = axes[1, 0]
    plot_data = []
    
    for method in methods:
        method_results = [r for r in successful if r['method'] == method]
        values = [r['mean_lambda2'] for r in method_results]
        plot_data.append(values)
    
    bp = ax.boxplot(plot_data, labels=labels, patch_artist=True, showfliers=True)
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    ax.set_ylabel('Average λ₂', fontweight='bold')
    ax.set_title('Average Connectivity', fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # 5. Computation Time
    ax = axes[1, 1]
    plot_data = []
    
    for method in methods:
        method_results = [r for r in successful if r['method'] == method]
        values = [r['elapsed_time'] for r in method_results]
        plot_data.append(values)
    
    bp = ax.boxplot(plot_data, labels=labels, patch_artist=True, showfliers=True)
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    ax.set_ylabel('Computation Time (s)', fontweight='bold')
    ax.set_title('Computational Efficiency', fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # 6. Pruning Events
    ax = axes[1, 2]
    plot_data = []
    
    for method in methods:
        method_results = [r for r in successful if r['method'] == method]
        values = [r['pruning_events_count'] for r in method_results]
        plot_data.append(values)
    
    bp = ax.boxplot(plot_data, labels=labels, patch_artist=True, showfliers=True)
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    ax.set_ylabel('Number of Pruning Events', fontweight='bold')
    ax.set_title('Pruning Activity', fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # Overall title
    fig.suptitle(f'Concurrent Pruning Comparison ({metadata["num_seeds"]} trials, '
                f'{metadata["num_robots"]} robots, density={metadata["density"]})',
                fontsize=14, fontweight='bold', y=0.995)
    
    plt.tight_layout()
    
    # Save figure
    output_file = output_path / 'comparison_boxplots.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Saved: {output_file}")
    
    plt.close()


def generate_time_series_plot(results_file: str, output_dir: str = 'concurrent_pruning/figures'):
    """Generate time series plots showing dynamics over iterations.
    
    Args:
        results_file: Path to batch results JSON
        output_dir: Output directory for figures
    """
    data = load_results(results_file)
    results = data['results']
    metadata = data['metadata']
    
    output_path = Path(output_dir)
    output_path.mkdir(exist_ok=True, parents=True)
    
    # Filter successful runs
    successful = [r for r in results if r.get('success', False) and 'error' not in r]
    methods = sorted(set(r['method'] for r in successful))
    
    # Create figure
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    
    # For each method, plot median and confidence intervals
    for method in methods:
        method_results = [r for r in successful if r['method'] == method]
        
        # Get edge count histories
        if method_results and 'edge_counts_history' in method_results[0]:
            # 1. Edge counts over iterations
            ax = axes[0]
            edge_histories = [r['edge_counts_history'] for r in method_results]
            
            # Pad to same length
            max_len = max(len(h) for h in edge_histories)
            padded = []
            for h in edge_histories:
                if len(h) < max_len:
                    padded.append(h + [h[-1]] * (max_len - len(h)))
                else:
                    padded.append(h)
            
            edge_array = np.array(padded)
            iterations = np.arange(max_len)
            
            median = np.median(edge_array, axis=0)
            q25 = np.percentile(edge_array, 25, axis=0)
            q75 = np.percentile(edge_array, 75, axis=0)
            
            color = METHOD_COLORS.get(method, '#95a5a6')
            ax.plot(iterations, median, linewidth=2.5, color=color, 
                   label=METHOD_LABELS.get(method, method))
            ax.fill_between(iterations, q25, q75, alpha=0.2, color=color)
            
            # 2. Lambda2 over iterations
            ax = axes[1]
            lambda2_histories = [r['lambda2_history'] for r in method_results]
            
            # Pad to same length
            padded = []
            for h in lambda2_histories:
                if len(h) < max_len:
                    padded.append(h + [h[-1]] * (max_len - len(h)))
                else:
                    padded.append(h)
            
            lambda2_array = np.array(padded)
            
            median = np.median(lambda2_array, axis=0)
            q25 = np.percentile(lambda2_array, 25, axis=0)
            q75 = np.percentile(lambda2_array, 75, axis=0)
            
            ax.plot(iterations, median, linewidth=2.5, color=color,
                   label=METHOD_LABELS.get(method, method))
            ax.fill_between(iterations, q25, q75, alpha=0.2, color=color)
            
            # 3. Consensus metric over iterations
            ax = axes[2]
            consensus_histories = [r['consensus_history'] for r in method_results]
            
            # Pad to same length
            padded = []
            for h in consensus_histories:
                if len(h) < max_len:
                    padded.append(h + [h[-1]] * (max_len - len(h)))
                else:
                    padded.append(h)
            
            consensus_array = np.array(padded)
            
            median = np.median(consensus_array, axis=0)
            q25 = np.percentile(consensus_array, 25, axis=0)
            q75 = np.percentile(consensus_array, 75, axis=0)
            
            ax.semilogy(iterations, median, linewidth=2.5, color=color,
                       label=METHOD_LABELS.get(method, method))
            ax.fill_between(iterations, q25, q75, alpha=0.2, color=color)
    
    # Format axes
    axes[0].set_xlabel('Iteration', fontweight='bold')
    axes[0].set_ylabel('Edge Count', fontweight='bold')
    axes[0].set_title('Edge Pruning Dynamics', fontweight='bold')
    axes[0].legend(loc='best')
    axes[0].grid(True, alpha=0.3)
    
    axes[1].axhline(y=0.3, color='orange', linestyle='--', linewidth=2, alpha=0.7)
    axes[1].set_xlabel('Iteration', fontweight='bold')
    axes[1].set_ylabel('λ₂ (Algebraic Connectivity)', fontweight='bold')
    axes[1].set_title('Connectivity Evolution', fontweight='bold')
    axes[1].legend(loc='best')
    axes[1].grid(True, alpha=0.3)
    
    axes[2].set_xlabel('Iteration', fontweight='bold')
    axes[2].set_ylabel('Consensus Metric (log scale)', fontweight='bold')
    axes[2].set_title('Consensus Convergence', fontweight='bold')
    axes[2].legend(loc='best')
    axes[2].grid(True, alpha=0.3)
    
    fig.suptitle(f'Dynamics Over Time (median ± IQR, {metadata["num_seeds"]} trials)',
                fontsize=14, fontweight='bold', y=0.995)
    
    plt.tight_layout()
    
    # Save figure
    output_file = output_path / 'comparison_timeseries.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Saved: {output_file}")
    
    plt.close()


def generate_summary_table(results_file: str, output_dir: str = 'concurrent_pruning/figures'):
    """Generate summary statistics table.
    
    Args:
        results_file: Path to batch results JSON
        output_dir: Output directory for table
    """
    data = load_results(results_file)
    results = data['results']
    metadata = data['metadata']
    
    output_path = Path(output_dir)
    output_path.mkdir(exist_ok=True, parents=True)
    
    # Filter successful runs
    successful = [r for r in results if r.get('success', False) and 'error' not in r]
    methods = sorted(set(r['method'] for r in successful))
    
    # Create summary table
    summary_file = output_path / 'comparison_summary.txt'
    
    with open(summary_file, 'w', encoding='utf-8') as f:
        f.write("=" * 100 + "\n")
        f.write("CONCURRENT PRUNING BATCH COMPARISON SUMMARY\n")
        f.write("=" * 100 + "\n\n")
        
        f.write(f"Configuration:\n")
        f.write(f"  Robots: {metadata['num_robots']}\n")
        f.write(f"  Edge Density: {metadata['density']}\n")
        f.write(f"  Random Seeds: {metadata['num_seeds']}\n")
        f.write(f"  Max Iterations: {metadata['max_iter']}\n")
        f.write(f"  Methods: {', '.join(metadata['methods'])}\n")
        f.write(f"  Total Trials: {metadata['total_trials']}\n")
        f.write(f"  Success Rate: {metadata['success_rate']:.1f}%\n")
        f.write(f"  Computation Time: {metadata['elapsed_time']/60:.1f} minutes\n")
        f.write("\n" + "=" * 100 + "\n\n")
        
        f.write("RESULTS BY METHOD:\n")
        f.write("-" * 100 + "\n")
        f.write(f"{'Method':<20} {'Trials':<10} {'Final Edges':<15} {'Pruned (%)':<15} "
               f"{'Min λ₂':<15} {'Time (s)':<15}\n")
        f.write("-" * 100 + "\n")
        
        for method in methods:
            method_results = [r for r in successful if r['method'] == method]
            
            if method_results:
                n = len(method_results)
                
                final_edges = [r['final_edges'] for r in method_results]
                reduction_pct = [r['edge_reduction_pct'] for r in method_results]
                min_lambda2 = [r['min_lambda2'] for r in method_results]
                elapsed = [r['elapsed_time'] for r in method_results]
                
                f.write(f"{method:<20} {n:<10} "
                       f"{np.mean(final_edges):>6.1f} ± {np.std(final_edges):>5.1f}  "
                       f"{np.mean(reduction_pct):>6.1f} ± {np.std(reduction_pct):>5.1f}  "
                       f"{np.mean(min_lambda2):>6.3f} ± {np.std(min_lambda2):>5.3f}  "
                       f"{np.mean(elapsed):>6.3f} ± {np.std(elapsed):>5.3f}\n")
        
        f.write("\n" + "=" * 100 + "\n\n")
        
        # Detailed statistics
        f.write("DETAILED STATISTICS:\n")
        f.write("-" * 100 + "\n")
        
        for method in methods:
            method_results = [r for r in successful if r['method'] == method]
            
            if method_results:
                f.write(f"\n{method}:\n")
                
                metrics = {
                    'Final Edges': [r['final_edges'] for r in method_results],
                    'Edges Pruned': [r['edges_pruned'] for r in method_results],
                    'Edge Reduction (%)': [r['edge_reduction_pct'] for r in method_results],
                    'Pruning Events': [r['pruning_events_count'] for r in method_results],
                    'Min λ₂': [r['min_lambda2'] for r in method_results],
                    'Mean λ₂': [r['mean_lambda2'] for r in method_results],
                    'Final λ₂': [r['final_lambda2'] for r in method_results],
                    'Iterations': [r['iterations'] for r in method_results],
                    'Time (s)': [r['elapsed_time'] for r in method_results]
                }
                
                for metric_name, values in metrics.items():
                    f.write(f"  {metric_name:<25} mean={np.mean(values):>8.3f}  "
                           f"std={np.std(values):>8.3f}  "
                           f"min={np.min(values):>8.3f}  "
                           f"max={np.max(values):>8.3f}\n")
        
        f.write("\n" + "=" * 100 + "\n")
    
    print(f"Saved: {summary_file}")
    
    # Also print to console
    with open(summary_file, 'r') as f:
        print(f.read())


def generate_all_plots(results_file: str, output_dir: str = 'concurrent_pruning/figures'):
    """Generate all comparison plots and tables.
    
    Args:
        results_file: Path to batch results JSON
        output_dir: Output directory
    """
    print(f"\nGenerating comparison plots from: {results_file}")
    print(f"Output directory: {output_dir}\n")
    
    # Generate box plots
    print("Generating box plots...")
    generate_box_plots(results_file, output_dir)
    
    # Generate time series
    print("\nGenerating time series plots...")
    generate_time_series_plot(results_file, output_dir)
    
    # Generate summary table
    print("\nGenerating summary table...")
    generate_summary_table(results_file, output_dir)
    
    print(f"\nAll plots generated successfully!")
    print(f"Output location: {Path(output_dir).absolute()}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Generate comparison plots from batch results'
    )
    parser.add_argument('results_file', type=str,
                       help='Path to batch results JSON file')
    parser.add_argument('--output', type=str, default='concurrent_pruning/figures',
                       help='Output directory for figures')
    parser.add_argument('--enhanced', action='store_true',
                       help='Also generate enhanced visualization plots')
    
    args = parser.parse_args()
    
    generate_all_plots(args.results_file, args.output)
    
    if args.enhanced:
        print("\n" + "="*80)
        print("Generating ENHANCED visualizations...")
        print("="*80 + "\n")
        try:
            from visualization.enhanced_comparison_plots import generate_all_enhanced_plots
            enhanced_dir = Path(args.output) / 'enhanced'
            generate_all_enhanced_plots(args.results_file, str(enhanced_dir))
        except ImportError:
            print("Enhanced plots require: pip install pandas seaborn scipy")
        except Exception as e:
            print(f"Error generating enhanced plots: {e}")

