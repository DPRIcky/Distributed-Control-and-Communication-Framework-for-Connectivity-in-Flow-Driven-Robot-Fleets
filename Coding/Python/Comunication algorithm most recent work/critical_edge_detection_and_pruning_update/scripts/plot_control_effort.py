"""Control effort comparison for 4 methods."""

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


def calculate_control_effort(times: np.ndarray, edge_counts: np.ndarray) -> float:
    """Calculate control effort as area under curve (integral of edges over time)."""
    if len(times) == 0 or len(edge_counts) == 0:
        return 0
    return np.trapz(edge_counts, times)


def plot_control_effort_comparison(data: Dict, scenario: str = None, output_dir: str = "experiments_timeseries"):
    """Create comparison plot: control effort for 4 methods in one scenario."""
    
    # If no scenario specified, use first one
    if scenario is None:
        scenario = list(data.keys())[0]
    
    if scenario not in data:
        print(f"[ERROR] Scenario '{scenario}' not found")
        return None
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    methods = ['FullyConnected', 'CentralizedMST', 'AdjacencyConsensus', 'DistributedPruning']
    colors = {
        'FullyConnected': '#1f77b4',      # blue
        'CentralizedMST': '#ff7f0e',      # orange
        'AdjacencyConsensus': '#2ca02c',  # green
        'DistributedPruning': '#d62728'   # red
    }
    
    scenario_data = data[scenario]
    methods_data = scenario_data.get('methods', {})
    
    efforts = []
    
    for method_name in methods:
        if method_name not in methods_data:
            efforts.append(0)
            continue
        
        method_data = methods_data[method_name]
        times = np.array(method_data.get('times', []))
        edge_counts_mean = np.array(method_data.get('edge_counts_mean', []))
        
        if len(times) == 0 or len(edge_counts_mean) == 0:
            efforts.append(0)
            continue
        
        # Calculate control effort (integral)
        effort = calculate_control_effort(times, edge_counts_mean)
        efforts.append(effort)
    
    # Bar plot
    x_pos = np.arange(len(methods))
    bars = ax.bar(x_pos, efforts, 
                  color=[colors[m] for m in methods],
                  edgecolor='black',
                  linewidth=1.5,
                  alpha=0.8)
    
    # Add value labels on bars
    for bar, effort in zip(bars, efforts):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height,
               f'{effort:.1f}',
               ha='center', va='bottom', fontsize=11, fontweight='bold')
    
    scenario_short = scenario.split(':')[0].replace('Scenario ', '')
    ax.set_xlabel('Method', fontsize=12, fontweight='bold')
    ax.set_ylabel('Control Effort', fontsize=12, fontweight='bold')
    ax.set_title(f'Control Effort Comparison - Scenario {scenario_short}', 
                fontsize=13, fontweight='bold')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(methods, rotation=45, ha='right')
    ax.grid(True, alpha=0.3, linestyle='--', axis='y')
    
    plt.tight_layout()
    
    # Save figure
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    filepath = output_path / "control_effort_comparison.png"
    
    fig.savefig(filepath, dpi=150, bbox_inches='tight')
    plt.close(fig)
    
    print(f"[OK] Saved: {filepath}")
    print(f"\nControl Effort for Scenario {scenario_short}:")
    for method_name, effort in zip(methods, efforts):
        print(f"  {method_name}: {effort:.1f}")
    
    return str(filepath)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Control effort comparison plot')
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
    plot_control_effort_comparison(data, scenario=args.scenario, output_dir=output_dir)


if __name__ == "__main__":
    main()
