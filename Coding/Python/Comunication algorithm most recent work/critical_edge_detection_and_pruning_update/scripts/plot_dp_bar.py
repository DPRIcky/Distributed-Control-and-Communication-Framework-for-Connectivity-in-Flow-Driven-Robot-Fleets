"""Line plot: number of edges for DistributedPruning across all scenarios."""

import json
import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from typing import Dict


def load_data(filepath: str) -> Dict:
    """Load time-series data from JSON."""
    with open(filepath, 'r') as f:
        return json.load(f)


def plot_distributed_pruning_line(data: Dict, output_dir: str = "experiments_timeseries"):
    """Create line plot: number of edges for DistributedPruning across all scenarios."""
    
    fig, ax = plt.subplots(figsize=(12, 6))
    
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
    
    for idx, scenario_name in enumerate(sorted(data.keys())):
        scenario_data = data[scenario_name]
        methods_data = scenario_data.get('methods', {})
        
        # Only get DistributedPruning data
        if 'DistributedPruning' not in methods_data:
            continue
        
        method_data = methods_data['DistributedPruning']
        times = np.array(method_data.get('times', []))
        edge_counts_mean = np.array(method_data.get('edge_counts_mean', []))
        
        if len(times) == 0 or len(edge_counts_mean) == 0:
            continue
        
        # Extract scenario short name
        scenario_short = scenario_name.split(':')[0].replace('Scenario ', '')
        
        # Plot line
        ax.plot(times, edge_counts_mean, 
               color=colors[idx % len(colors)],
               linewidth=2.5,
               label=f'Scenario {scenario_short}',
               marker='o',
               markersize=5,
               markevery=max(1, len(times) // 8))
    
    ax.set_xlabel('Time (seconds)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Number of Edges', fontsize=12, fontweight='bold')
    ax.set_title('DistributedPruning - Number of Edges Over Time (All Scenarios)', 
                fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.legend(loc='upper right', fontsize=11, framealpha=0.95)
    
    plt.tight_layout()
    
    # Save figure
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    filepath = output_path / "distributed_pruning_line_plot.png"
    
    fig.savefig(filepath, dpi=150, bbox_inches='tight')
    plt.close(fig)
    
    print(f"[OK] Saved: {filepath}")
    
    return str(filepath)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='DistributedPruning line plot')
    parser.add_argument('--data', type=str, required=True,
                       help='Path to timeseries data JSON file')
    parser.add_argument('--output-dir', type=str, default=None,
                       help='Output directory for plot')
    
    args = parser.parse_args()
    
    # Determine output directory
    output_dir = args.output_dir or str(Path(args.data).parent)
    
    # Load data
    data = load_data(args.data)
    
    # Generate plot
    plot_distributed_pruning_line(data, output_dir=output_dir)


if __name__ == "__main__":
    main()
