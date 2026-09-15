"""Plotting script for time-series comparison data.

Generates "edges vs time" plots for each method across scenarios.
"""

import json
import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from typing import Dict, List, Optional


class TimeSeriesPlotter:
    """Generate time-series plots from collected data."""
    
    def __init__(self, output_dir: str = "experiments_timeseries"):
        """Initialize plotter."""
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.data = {}
    
    def load_data(self, filepath: str) -> bool:
        """Load time-series data from JSON."""
        try:
            with open(filepath, 'r') as f:
                self.data = json.load(f)
            print(f"[OK] Loaded data from: {filepath}")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to load {filepath}: {e}")
            return False
    
    def plot_all_methods_per_scenario(self) -> List[str]:
        """Create 'edges vs time' plot for each scenario comparing all methods."""
        if not self.data:
            print("[ERROR] No data loaded")
            return []
        
        saved_figures = []
        
        for scenario_name, scenario_data in self.data.items():
            fig, ax = plt.subplots(figsize=(12, 7))
            
            methods = scenario_data.get('methods', {})
            colors = {
                'FullyConnected': '#FF6B6B',
                'CentralizedMST': '#4ECDC4',
                'AdjacencyConsensus': '#45B7D1',
                'DistributedPruning': '#FFA07A'
            }
            
            linestyles = {
                'FullyConnected': '--',
                'CentralizedMST': ':',
                'AdjacencyConsensus': '-.',
                'DistributedPruning': '-'
            }
            
            for method_name in ['FullyConnected', 'CentralizedMST', 'AdjacencyConsensus', 'DistributedPruning']:
                if method_name not in methods:
                    continue
                
                method_data = methods[method_name]
                times = method_data.get('times', [])
                edge_counts_mean = method_data.get('edge_counts_mean', [])
                edge_counts_std = method_data.get('edge_counts_std', [])
                
                if not times or not edge_counts_mean:
                    continue
                
                times = np.array(times)
                edge_counts_mean = np.array(edge_counts_mean)
                edge_counts_std = np.array(edge_counts_std)
                
                # Plot mean line
                ax.plot(times, edge_counts_mean, 
                       color=colors.get(method_name, 'gray'),
                       linestyle=linestyles.get(method_name, '-'),
                       linewidth=2.5,
                       label=method_name,
                       marker='o',
                       markersize=4,
                       markevery=max(1, len(times) // 5))
                
                # Add shaded error band
                ax.fill_between(times, 
                               edge_counts_mean - edge_counts_std,
                               edge_counts_mean + edge_counts_std,
                               color=colors.get(method_name, 'gray'),
                               alpha=0.2)
            
            ax.set_xlabel('Time (seconds)', fontsize=12, fontweight='bold')
            ax.set_ylabel('Number of Edges', fontsize=12, fontweight='bold')
            ax.set_title(f'{scenario_name} - Edge Count Over Time', fontsize=13, fontweight='bold')
            ax.grid(True, alpha=0.3)
            ax.legend(loc='best', fontsize=11, framealpha=0.9)
            
            # Add statistics box
            stats_text = "Statistics:\n"
            for method_name in ['FullyConnected', 'CentralizedMST', 'AdjacencyConsensus', 'DistributedPruning']:
                if method_name in methods:
                    method_data = methods[method_name]
                    initial = method_data.get('edge_counts_mean', [method_data.get('initial_edges', 0)])[0]
                    final = method_data.get('edge_counts_mean', [0])[-1]
                    reduction = ((initial - final) / initial * 100) if initial > 0 else 0
                    stats_text += f"{method_name}: {initial:.0f}→{final:.0f} ({reduction:.0f}%)\n"
            
            ax.text(0.98, 0.97, stats_text, transform=ax.transAxes,
                   fontsize=9, verticalalignment='top', horizontalalignment='right',
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                   family='monospace')
            
            plt.tight_layout()
            
            # Save figure
            filename = scenario_name.replace(':', '').replace(' ', '_').lower()
            filepath = self.output_dir / f"timeseries_{filename}.png"
            fig.savefig(filepath, dpi=150, bbox_inches='tight')
            saved_figures.append(str(filepath))
            plt.close(fig)
            
            print(f"[OK] Saved: {filepath}")
        
        return saved_figures
    
    def plot_all_scenarios_per_method(self) -> List[str]:
        """Create subplots: one method with all scenarios."""
        if not self.data:
            print("[ERROR] No data loaded")
            return []
        
        saved_figures = []
        methods = ['FullyConnected', 'CentralizedMST', 'AdjacencyConsensus', 'DistributedPruning']
        
        for method_name in methods:
            fig, axes = plt.subplots(2, 3, figsize=(16, 10))
            fig.suptitle(f'{method_name} - Edge Count Over Time (All Scenarios)', 
                        fontsize=14, fontweight='bold')
            
            axes = axes.flatten()
            scenario_names = list(self.data.keys())
            
            colors = {
                'FullyConnected': '#FF6B6B',
                'CentralizedMST': '#4ECDC4',
                'AdjacencyConsensus': '#45B7D1',
                'DistributedPruning': '#FFA07A'
            }
            
            for idx, scenario_name in enumerate(scenario_names):
                if idx >= len(axes):
                    break
                
                ax = axes[idx]
                scenario_data = self.data[scenario_name]
                methods_data = scenario_data.get('methods', {})
                
                if method_name not in methods_data:
                    ax.text(0.5, 0.5, 'No Data', ha='center', va='center',
                           transform=ax.transAxes, fontsize=12)
                    ax.set_title(scenario_name.split(':')[0])
                    continue
                
                method_data = methods_data[method_name]
                times = method_data.get('times', [])
                edge_counts_mean = method_data.get('edge_counts_mean', [])
                edge_counts_std = method_data.get('edge_counts_std', [])
                
                if not times or not edge_counts_mean:
                    ax.text(0.5, 0.5, 'No Data', ha='center', va='center',
                           transform=ax.transAxes, fontsize=12)
                    ax.set_title(scenario_name.split(':')[0])
                    continue
                
                times = np.array(times)
                edge_counts_mean = np.array(edge_counts_mean)
                edge_counts_std = np.array(edge_counts_std)
                
                # Plot
                color = colors.get(method_name, 'gray')
                ax.plot(times, edge_counts_mean, color=color, linewidth=2.5,
                       marker='o', markersize=4, markevery=max(1, len(times) // 5))
                ax.fill_between(times, 
                               edge_counts_mean - edge_counts_std,
                               edge_counts_mean + edge_counts_std,
                               color=color, alpha=0.2)
                
                ax.set_xlabel('Time (s)', fontsize=10)
                ax.set_ylabel('Edges', fontsize=10)
                ax.set_title(scenario_name.split(':')[0], fontweight='bold')
                ax.grid(True, alpha=0.3)
                
                # Add reduction percentage
                initial = edge_counts_mean[0] if len(edge_counts_mean) > 0 else 0
                final = edge_counts_mean[-1] if len(edge_counts_mean) > 0 else 0
                reduction = ((initial - final) / initial * 100) if initial > 0 else 0
                
                ax.text(0.95, 0.95, f'{reduction:.0f}%\nreduction',
                       transform=ax.transAxes, fontsize=10, fontweight='bold',
                       verticalalignment='top', horizontalalignment='right',
                       bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
            
            # Hide unused subplots
            for idx in range(len(scenario_names), len(axes)):
                axes[idx].set_visible(False)
            
            plt.tight_layout()
            
            # Save figure
            filename = method_name.lower().replace(' ', '_')
            filepath = self.output_dir / f"method_{filename}_all_scenarios.png"
            fig.savefig(filepath, dpi=150, bbox_inches='tight')
            saved_figures.append(str(filepath))
            plt.close(fig)
            
            print(f"[OK] Saved: {filepath}")
        
        return saved_figures
    
    def plot_comparison_heatmap(self) -> Optional[str]:
        """Create heatmap: edge reduction final vs initial."""
        if not self.data:
            print("[ERROR] No data loaded")
            return None
        
        fig, ax = plt.subplots(figsize=(10, 8))
        
        scenarios = list(self.data.keys())
        methods = ['FullyConnected', 'CentralizedMST', 'AdjacencyConsensus', 'DistributedPruning']
        
        # Build reduction matrix
        matrix = np.zeros((len(scenarios), len(methods)))
        
        for i, scenario_name in enumerate(scenarios):
            scenario_data = self.data[scenario_name]
            methods_data = scenario_data.get('methods', {})
            
            for j, method_name in enumerate(methods):
                if method_name in methods_data:
                    method_data = methods_data[method_name]
                    times = method_data.get('times', [])
                    edge_counts_mean = method_data.get('edge_counts_mean', [])
                    
                    if times and edge_counts_mean:
                        initial = edge_counts_mean[0]
                        final = edge_counts_mean[-1]
                        reduction = ((initial - final) / initial * 100) if initial > 0 else 0
                        matrix[i, j] = reduction
        
        # Plot heatmap
        im = ax.imshow(matrix, cmap='RdYlGn', aspect='auto', vmin=0, vmax=70)
        
        ax.set_xticks(range(len(methods)))
        ax.set_yticks(range(len(scenarios)))
        ax.set_xticklabels(methods, rotation=45, ha='right')
        ax.set_yticklabels([s.split(':')[0] for s in scenarios])
        
        ax.set_xlabel('Method', fontsize=12, fontweight='bold')
        ax.set_ylabel('Scenario', fontsize=12, fontweight='bold')
        ax.set_title('Edge Reduction (%) - Over Time Final vs Initial', 
                    fontsize=13, fontweight='bold')
        
        # Add values
        for i in range(len(scenarios)):
            for j in range(len(methods)):
                text = ax.text(j, i, f'{matrix[i, j]:.0f}%',
                             ha="center", va="center", color="black", 
                             fontsize=10, fontweight='bold')
        
        plt.colorbar(im, ax=ax, label='Edge Reduction (%)')
        plt.tight_layout()
        
        filepath = self.output_dir / "heatmap_edge_reduction.png"
        fig.savefig(filepath, dpi=150, bbox_inches='tight')
        plt.close(fig)
        
        print(f"[OK] Saved: {filepath}")
        return str(filepath)
    
    def generate_all_plots(self) -> List[str]:
        """Generate all plots."""
        saved_figures = []
        
        print("\n" + "="*80)
        print("GENERATING TIME-SERIES PLOTS")
        print("="*80)
        
        print("\n[1/3] Plotting edges vs time for each scenario...")
        saved_figures.extend(self.plot_all_methods_per_scenario())
        
        print("\n[2/3] Plotting edges vs time for each method...")
        saved_figures.extend(self.plot_all_scenarios_per_method())
        
        print("\n[3/3] Creating reduction heatmap...")
        heatmap = self.plot_comparison_heatmap()
        if heatmap:
            saved_figures.append(heatmap)
        
        return saved_figures


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Plot time-series comparison data')
    parser.add_argument('--data', type=str, required=True,
                       help='Path to timeseries data JSON file')
    parser.add_argument('--output-dir', type=str, default=None,
                       help='Output directory for plots')
    
    args = parser.parse_args()
    
    # Determine output directory
    output_dir = args.output_dir or str(Path(args.data).parent)
    
    # Create plotter
    plotter = TimeSeriesPlotter(output_dir=output_dir)
    
    # Load data
    if not plotter.load_data(args.data):
        return
    
    # Generate plots
    figures = plotter.generate_all_plots()
    
    print("\n" + "="*80)
    print("PLOT GENERATION COMPLETE")
    print("="*80)
    print(f"\nGenerated {len(figures)} plots:")
    for fig_path in figures:
        print(f"  - {fig_path}")


if __name__ == "__main__":
    main()
