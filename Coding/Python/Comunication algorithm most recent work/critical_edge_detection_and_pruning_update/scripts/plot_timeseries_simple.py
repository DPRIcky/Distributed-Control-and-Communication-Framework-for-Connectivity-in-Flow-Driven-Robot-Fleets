"""Simple clean plot: edges vs time for 4 methods."""

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


def plot_simple(data: Dict, scenario: str = None, output_dir: str = "experiments_timeseries"):
    """Create simple clean plot for one scenario."""
    
    # If no scenario specified, use first one
    if scenario is None:
        scenario = list(data.keys())[0]
    
    if scenario not in data:
        print(f"[ERROR] Scenario '{scenario}' not found")
        return None
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    scenario_data = data[scenario]
    methods = scenario_data.get('methods', {})
    
    # Simple color scheme
    colors = {
        'FullyConnected': '#1f77b4',      # blue
        'CentralizedMST': '#ff7f0e',      # orange
        'AdjacencyConsensus': '#2ca02c',  # green
        'DistributedPruning': '#d62728'   # red
    }
    
    # Plot each method
    for method_name in ['FullyConnected', 'CentralizedMST', 'AdjacencyConsensus', 'DistributedPruning']:
        if method_name not in methods:
            continue
        
        method_data = methods[method_name]
        times = np.array(method_data.get('times', []))
        edge_counts_mean = np.array(method_data.get('edge_counts_mean', []))
        
        if len(times) == 0 or len(edge_counts_mean) == 0:
            continue
        
        # Simple line plot
        ax.plot(times, edge_counts_mean, 
               color=colors[method_name],
               linewidth=2.5,
               label=method_name,
               marker='o',
               markersize=5,
               markevery=max(1, len(times) // 8))
    
    ax.set_xlabel('Time (seconds)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Number of Edges', fontsize=12, fontweight='bold')
    ax.set_title(f'{scenario} - Edge Count Over Time', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.legend(loc='upper right', fontsize=11, framealpha=0.95)
    
    plt.tight_layout()
    
    # Save figure
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    filepath = output_path / "simple_timeseries.png"
    
    fig.savefig(filepath, dpi=150, bbox_inches='tight')
    plt.close(fig)
    
    print(f"[OK] Saved: {filepath}")
    return str(filepath)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Simple time-series plot')
    parser.add_argument('--data', type=str, required=True,
                       help='Path to timeseries data JSON file')
    parser.add_argument('--scenario', type=str, default=None,
                       help='Scenario name (default: first scenario)')
    parser.add_argument('--output-dir', type=str, default=None,
                       help='Output directory for plot')
    
    args = parser.parse_args()
    
    # Determine output directory
    output_dir = args.output_dir or str(Path(args.data).parent)
    
    # Load data
    data = load_data(args.data)
    
    # Generate plot
    plot_simple(data, scenario=args.scenario, output_dir=output_dir)


if __name__ == "__main__":
    main()
