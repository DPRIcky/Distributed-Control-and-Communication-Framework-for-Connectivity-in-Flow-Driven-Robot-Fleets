"""
Generate all plots from simulation experiment results.
This script creates comprehensive visualizations comparing 4 methods across 5 scenarios.
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from pathlib import Path
from typing import List, Dict
from datetime import datetime

# Try to import seaborn, but proceed without it
try:
    import seaborn as sns
    HAS_SEABORN = True
except ImportError:
    HAS_SEABORN = False
    print("Warning: seaborn not available - using default matplotlib styles")

# Set professional style for research paper
plt.style.use('default')
plt.rcParams.update({
    'figure.figsize': (7, 5),  # Standard research paper figure size
    'font.size': 9,
    'axes.labelsize': 10,
    'axes.titlesize': 11,
    'xtick.labelsize': 8,
    'ytick.labelsize': 8,
    'legend.fontsize': 8,
    'figure.titlesize': 12,
    'font.family': 'sans-serif',
    'font.sans-serif': ['DejaVu Sans', 'Arial'],
    'axes.linewidth': 0.8,
    'grid.linewidth': 0.5,
    'lines.linewidth': 1.5,
    'patch.linewidth': 0.5,
    'xtick.major.width': 0.8,
    'ytick.major.width': 0.8,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'axes.axisbelow': True,
    'text.usetex': False,
    'mathtext.default': 'regular'
})

# Color scheme for methods
METHOD_COLORS = {
    'ConcurrentPruning': '#2E86AB',      # Blue
    'AdjacencyConsensus': '#A23B72',     # Purple
    'FullGraph': '#F18F01',              # Orange
    'CentralizedMST': '#28A745'          # Green
}

METHOD_LABELS = {
    'ConcurrentPruning': 'Concurrent Pruning (Proposed)',
    'AdjacencyConsensus': 'Adjacency Consensus',
    'FullGraph': 'Full Graph (No Pruning)',
    'CentralizedMST': 'Centralized MST'
}

SCENARIO_LABELS = {
    'A': 'Baseline',
    'B': 'Large Timestep',
    'C': 'Tight Radius',
    'D': 'High Robot Count',
    'E': 'Extreme Parameters'
}


def load_results(results_file: str) -> Dict:
    """Load experiment results from JSON file."""
    with open(results_file, 'r') as f:
        return json.load(f)


def plot_1_robot_trajectories(results: List[Dict], output_dir: Path):
    """Plot 1: Robot trajectories with pruning events for Scenario A."""
    print("Generating Plot 1: Robot Trajectories...")
    
    # Get one trial from ConcurrentPruning, Scenario A
    trial = next((r for r in results if r['method'] == 'ConcurrentPruning' 
                  and r['scenario'] == 'A' and r['success']), None)
    
    if not trial or 'positions_history' not in trial:
        print("  ⚠ No valid trajectory data found. Skipping Plot 1.")
        return
    
    positions_history = np.array(trial['positions_history'])  # Shape: (timesteps, n_robots, 2)
    pruning_events = trial.get('pruning_events', [])
    edge_counts_history = trial.get('edge_counts_history', [])
    
    fig, ax = plt.subplots(figsize=(8, 6))
    
    # Add flow field background (simulated - arrows showing general flow)
    # Create a simple flow field visualization
    workspace_x = 10.0  # Assume 10m workspace
    workspace_y = 10.0
    grid_points = 15
    grid_x = np.linspace(0.0, workspace_x, grid_points)
    grid_y = np.linspace(0.0, workspace_y, grid_points)
    gx, gy = np.meshgrid(grid_x, grid_y)
    
    # Simple flow field pointing toward goal (center-right area)
    goal_x, goal_y = 8.0, 5.0  # Assumed goal position
    flow_u = (goal_x - gx) * 0.1
    flow_v = (goal_y - gy) * 0.1
    
    ax.quiver(gx, gy, flow_u, flow_v, color="#B0C4DE", alpha=0.3, 
             width=0.003, scale=15, zorder=0)
    
    n_robots = positions_history.shape[1]
    # All robots in red color
    robot_color = 'red'
    
    # Plot initial fully connected graph edges
    initial_positions = positions_history[0]
    for i in range(n_robots):
        for j in range(i+1, n_robots):
            ax.plot([initial_positions[i, 0], initial_positions[j, 0]],
                   [initial_positions[i, 1], initial_positions[j, 1]],
                   color='#9b59b6', alpha=0.3, linewidth=1.0, linestyle='--', zorder=1)
    
    # Plot trajectories
    for i in range(n_robots):
        trajectory = positions_history[:, i, :]
        ax.plot(trajectory[:, 0], trajectory[:, 1], 
               color=robot_color, alpha=0.6, linewidth=0.8)
        
        # Mark start (circle)
        ax.plot(trajectory[0, 0], trajectory[0, 1], 'o', color=robot_color, 
               markersize=10, markeredgecolor='black', markeredgewidth=1.5, zorder=10)
        # Mark end (circle)
        ax.plot(trajectory[-1, 0], trajectory[-1, 1], 'o', color=robot_color, 
               markersize=10, markeredgecolor='black', markeredgewidth=1.5, zorder=10)
    
    # Plot final near-MST graph edges
    final_positions = positions_history[-1]
    # For near-MST, only show edges that would be in a sparse graph (approx n-1 edges)
    # Connect nearby robots only
    distances = []
    for i in range(n_robots):
        for j in range(i+1, n_robots):
            dist = np.linalg.norm(final_positions[i] - final_positions[j])
            distances.append((dist, i, j))
    distances.sort()
    # Take approximately n-1 to n+2 edges to show near-MST
    num_edges = min(n_robots + 1, len(distances))
    for k in range(num_edges):
        _, i, j = distances[k]
        ax.plot([final_positions[i, 0], final_positions[j, 0]],
               [final_positions[i, 1], final_positions[j, 1]],
               color='#9b59b6', alpha=0.3, linewidth=1.0, linestyle='-', zorder=2)
    
    # Mark goal
    ax.plot(goal_x, goal_y, marker='*', color='gold', markersize=20,
           markeredgecolor='darkorange', markeredgewidth=2, zorder=10)
    
    ax.set_xlim(0.0, workspace_x)
    ax.set_ylim(0.0, workspace_y)
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title('Robot Trajectories')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    # Custom legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor='red',
               markersize=8, label='Start', markeredgecolor='black'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='red',
               markersize=8, label='End', markeredgecolor='black'),
        Line2D([0], [0], color='#9b59b6', linestyle='--', linewidth=1.5,
               label='Initial (Fully Connected)'),
        Line2D([0], [0], color='#9b59b6', linestyle='-', linewidth=1.5,
               label='Final (Near-MST)'),
        Line2D([0], [0], marker='*', color='gold', markersize=12, label='Goal'),
    ]
    ax.legend(handles=legend_elements, loc='upper right', fontsize=8, framealpha=0.9)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_1_trajectories.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_1_trajectories.png")


def plot_2_edge_evolution(results: List[Dict], output_dir: Path):
    """Plot 2: Edge count evolution - all methods (Scenario A) and all scenarios (Concurrent Pruning)."""
    print("Generating Plot 2: Edge Evolution...")
    
    scenarios = ['A', 'B', 'C', 'D', 'E']
    methods = ['ConcurrentPruning', 'AdjacencyConsensus', 'FullGraph', 'CentralizedMST']
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # Subplot 1: All methods for Scenario A
    ax = axes[0]
    for method in methods:
        # Get successful trials for this method in Scenario A
        trials = [r for r in results if r['method'] == method 
                 and r['scenario'] == 'A' and r['success']]
        
        if not trials:
            continue
        
        # Average edge counts across trials
        max_len = max(len(t['edge_counts_history']) for t in trials)
        edge_counts_padded = np.array([
            np.pad(t['edge_counts_history'], (0, max_len - len(t['edge_counts_history'])), 
                  mode='edge') for t in trials
        ])
        mean_edges = np.mean(edge_counts_padded, axis=0)
        std_edges = np.std(edge_counts_padded, axis=0)
        
        time = np.arange(len(mean_edges))
        ax.plot(time, mean_edges, label=METHOD_LABELS[method], 
               color=METHOD_COLORS[method], linewidth=2)
    
    ax.set_xlabel('Time Step')
    ax.set_ylabel('Number of Edges')
    ax.set_title('All Methods (Scenario A: Baseline)')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, framealpha=0.9)
    
    # Subplot 2: ConcurrentPruning for all scenarios
    ax = axes[1]
    for scenario in scenarios:
        # Get successful trials for ConcurrentPruning in this scenario
        trials = [r for r in results if r['method'] == 'ConcurrentPruning' 
                 and r['scenario'] == scenario and r['success']]
        
        if not trials:
            continue
        
        # Average edge counts across trials
        max_len = max(len(t['edge_counts_history']) for t in trials)
        edge_counts_padded = np.array([
            np.pad(t['edge_counts_history'], (0, max_len - len(t['edge_counts_history'])), 
                  mode='edge') for t in trials
        ])
        mean_edges = np.mean(edge_counts_padded, axis=0)
        std_edges = np.std(edge_counts_padded, axis=0)
        
        time = np.arange(len(mean_edges))
        ax.plot(time, mean_edges, label=f'Scenario {scenario}: {SCENARIO_LABELS[scenario]}', 
               linewidth=2)
    
    ax.set_xlabel('Time Step')
    ax.set_ylabel('Number of Edges')
    ax.set_title('Concurrent Pruning Across Scenarios')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, framealpha=0.9)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_2_edge_evolution.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_2_edge_evolution.png")


def plot_3_lambda2_evolution_and_distribution(results: List[Dict], output_dir: Path):
    """Plot 3: Lambda2 evolution and distribution."""
    print("Generating Plot 3: Lambda2 Analysis...")
    
    scenarios = ['A', 'B', 'C', 'D', 'E']
    methods = ['ConcurrentPruning', 'AdjacencyConsensus', 'FullGraph', 'CentralizedMST']
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # Subplot 1: Lambda2 evolution for Scenario A only
    ax = axes[0]
    scenario = 'A'
    
    for method in methods:
        trials = [r for r in results if r['method'] == method and r['scenario'] == scenario and r['success']]
        
        if not trials:
            continue
        
        max_len = max(len(t['lambda2_history']) for t in trials)
        lambda2_padded = np.array([
            np.pad(t['lambda2_history'], (0, max_len - len(t['lambda2_history'])), 
                  mode='edge') for t in trials
        ])
        mean_lambda2 = np.mean(lambda2_padded, axis=0)
        std_lambda2 = np.std(lambda2_padded, axis=0)
        
        time = np.arange(len(mean_lambda2))
        ax.plot(time, mean_lambda2, label=METHOD_LABELS[method],
               color=METHOD_COLORS[method], linewidth=1.2)
    
    ax.axhline(y=0.2, color='red', linestyle='--', linewidth=1, alpha=0.6, label='Safety Threshold')
    ax.set_xlabel('Time Step')
    ax.set_ylabel(r'$\lambda_2$')
    ax.set_title(f'Scenario {scenario}: {SCENARIO_LABELS[scenario]}')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, framealpha=0.9)
    
    # Subplot 2: Lambda2 distribution (bar plot)
    ax_dist = axes[1]
    
    # Prepare data for bar plot
    means = []
    stds = []
    
    for method in methods:
        method_results = [r for r in results if r['method'] == method and r['success']]
        min_lambda2_values = [r['min_lambda2'] for r in method_results]
        means.append(np.mean(min_lambda2_values))
        stds.append(np.std(min_lambda2_values))
    
    # Create bar plot
    x = np.arange(len(methods))
    bars = ax_dist.bar(x, means, 
                       color=[METHOD_COLORS[m] for m in methods],
                       alpha=0.7, edgecolor='black', linewidth=0.8)
    
    # Add safety threshold line
    ax_dist.axhline(y=0.2, color='red', linestyle='--', linewidth=1, alpha=0.6, label='Safety Threshold')
    
    ax_dist.set_xticks(x)
    ax_dist.set_xticklabels([METHOD_LABELS[m] for m in methods], rotation=20, ha='right', fontsize=7)
    ax_dist.set_ylabel(r'Minimum $\lambda_2$')
    ax_dist.set_title(r'Min $\lambda_2$ Distribution Across All Scenarios')
    ax_dist.grid(True, alpha=0.3, axis='y')
    ax_dist.legend(fontsize=7)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_3_lambda2_analysis.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_3_lambda2_analysis.png")


def plot_4_control_effort(results: List[Dict], output_dir: Path):
    """Plot 4: Cumulative control effort over time."""
    print("Generating Plot 4: Control Effort...")
    
    methods = ['ConcurrentPruning', 'AdjacencyConsensus', 'FullGraph', 'CentralizedMST']
    
    fig, ax = plt.subplots(1, 1, figsize=(10, 6))
    
    # Only Scenario E
    scenario = 'E'
    
    for method in methods:
        trials = [r for r in results if r['method'] == method and r['scenario'] == scenario and r['success']]
        
        if not trials:
            continue
        
        # Check if control_effort_history exists and has data
        valid_trials = []
        for t in trials:
            if 'control_effort_history' in t and len(t['control_effort_history']) > 0:
                # Check if any values are non-zero
                if np.sum(np.abs(t['control_effort_history'])) > 1e-10:
                    valid_trials.append(t['control_effort_history'])
        
        if not valid_trials:
            # If no valid control_effort_history, skip this method
            continue
        
        # Find max length
        max_len = max(len(t) for t in valid_trials)
        
        # Pad all trials to same length
        control_padded = []
        for control_hist in valid_trials:
            if len(control_hist) < max_len:
                padded = np.pad(control_hist, (0, max_len - len(control_hist)), mode='edge')
            else:
                padded = np.array(control_hist)
            control_padded.append(padded)
        
        control_padded = np.array(control_padded)
        
        # Compute cumulative sum across time
        cumulative_control = np.cumsum(control_padded, axis=1)
        mean_control = np.mean(cumulative_control, axis=0)
        
        time = np.arange(len(mean_control))
        ax.plot(time, mean_control, label=METHOD_LABELS[method],
               color=METHOD_COLORS[method], linewidth=1.2)
    
    ax.set_xlabel('Time Step')
    ax.set_ylabel('Cumulative Control Effort')
    ax.set_title(f'Scenario {scenario}: {SCENARIO_LABELS[scenario]}')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9, framealpha=0.9)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_4_control_effort.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_4_control_effort.png")


def plot_5_success_rate_heatmap(results: List[Dict], output_dir: Path):
    """Plot 5: Success rate heatmap (scenarios x methods)."""
    print("Generating Plot 5: Success Rate Heatmap...")
    
    scenarios = ['A', 'B', 'C', 'D', 'E']
    methods = ['ConcurrentPruning', 'AdjacencyConsensus', 'FullGraph', 'CentralizedMST']
    
    # Compute success rates
    success_matrix = np.zeros((len(methods), len(scenarios)))
    
    for i, method in enumerate(methods):
        for j, scenario in enumerate(scenarios):
            trials = [r for r in results if r['method'] == method and r['scenario'] == scenario]
            if trials:
                success_rate = 100 * sum(r['success'] for r in trials) / len(trials)
                success_matrix[i, j] = success_rate
    
    fig, ax = plt.subplots(figsize=(8, 5))
    
    im = ax.imshow(success_matrix, cmap='RdYlGn', aspect='auto', vmin=0, vmax=100)
    
    # Set ticks
    ax.set_xticks(range(len(scenarios)))
    ax.set_yticks(range(len(methods)))
    ax.set_xticklabels([f'{s}: {SCENARIO_LABELS[s]}' for s in scenarios])
    ax.set_yticklabels([METHOD_LABELS[m] for m in methods])
    
    # Add text annotations
    for i in range(len(methods)):
        for j in range(len(scenarios)):
            text = ax.text(j, i, f'{success_matrix[i, j]:.0f}%',
                          ha="center", va="center", color="black", fontsize=9, fontweight='bold')
    
    ax.set_xlabel('Scenario')
    ax.set_ylabel('Method')
    ax.set_title(r'Success Rate: Connectivity Maintained ($\lambda_2$ > Threshold)', pad=15)
    
    # Colorbar
    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label('Success Rate (%)', rotation=270, labelpad=15)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_5_success_rate_heatmap.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_5_success_rate_heatmap.png")


def plot_6_min_lambda2_by_scenario(results: List[Dict], output_dir: Path):
    """Plot 6: Average minimum lambda2 vs scenario."""
    print("Generating Plot 6: Min Lambda2 by Scenario...")
    
    scenarios = ['A', 'B', 'C', 'D', 'E']
    methods = ['ConcurrentPruning', 'AdjacencyConsensus', 'FullGraph', 'CentralizedMST']
    
    fig, ax = plt.subplots(figsize=(7, 5))
    
    for method in methods:
        means = []
        stds = []
        
        for scenario in scenarios:
            trials = [r for r in results if r['method'] == method and r['scenario'] == scenario and r['success']]
            
            if trials:
                min_lambda2_values = [r['min_lambda2'] for r in trials]
                means.append(np.mean(min_lambda2_values))
                stds.append(np.std(min_lambda2_values))
            else:
                means.append(0)
                stds.append(0)
        
        x = range(len(scenarios))
        ax.plot(x, means, marker='o', markersize=5, linewidth=1.2,
               color=METHOD_COLORS[method], label=METHOD_LABELS[method])
    
    # Add safety threshold
    ax.axhline(y=0.2, color='red', linestyle='--', linewidth=1, alpha=0.6, label='Safety Threshold')
    
    ax.set_xticks(range(len(scenarios)))
    ax.set_xticklabels([f'{s}: {SCENARIO_LABELS[s]}' for s in scenarios])
    ax.set_xlabel('Scenario')
    ax.set_ylabel(r'Average Minimum $\lambda_2$')
    ax.set_title('Connectivity Safety Across Scenarios')
    ax.legend(fontsize=7, loc='best', framealpha=0.9)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_6_min_lambda2_by_scenario.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_6_min_lambda2_by_scenario.png")


def plot_7_pruning_efficiency(results: List[Dict], output_dir: Path):
    """Plot 7: Pruning efficiency by scenario."""
    print("Generating Plot 7: Pruning Efficiency...")
    
    scenarios = ['A', 'B', 'C', 'D', 'E']
    methods = ['ConcurrentPruning', 'AdjacencyConsensus', 'FullGraph', 'CentralizedMST']
    
    fig, ax = plt.subplots(figsize=(8, 5))
    
    x = np.arange(len(scenarios))
    width = 0.2
    
    for i, method in enumerate(methods):
        means = []
        stds = []
        
        for scenario in scenarios:
            trials = [r for r in results if r['method'] == method and r['scenario'] == scenario and r['success']]
            
            if trials:
                reduction_values = [r['edge_reduction_pct'] for r in trials]
                means.append(np.mean(reduction_values))
                stds.append(np.std(reduction_values))
            else:
                means.append(0)
                stds.append(0)
        
        offset = (i - 1.5) * width
        label = METHOD_LABELS[method].replace(' (Oracle)', '')
        ax.bar(x + offset, means, width, 
              label=label, color=METHOD_COLORS[method],
              alpha=0.7, edgecolor='black', linewidth=0.8)
    
    ax.set_xticks(x)
    ax.set_xticklabels([f'{s}: {SCENARIO_LABELS[s]}' for s in scenarios])
    ax.set_xlabel('Scenario')
    ax.set_ylabel('Edge Reduction (%)')
    ax.set_title('Pruning Efficiency Across Scenarios')
    ax.legend(fontsize=7, loc='best', framealpha=0.9)
    ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_7_pruning_efficiency.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_7_pruning_efficiency.png")


def plot_r1_trade_off_analysis(results: List[Dict], output_dir: Path):
    """Recommended Plot 1: Trade-off between edge reduction and connectivity (bar chart)."""
    print("Generating Recommended Plot 1: Trade-off Analysis...")
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    
    methods = ['ConcurrentPruning', 'AdjacencyConsensus', 'FullGraph', 'CentralizedMST']
    
    # Calculate statistics for each method
    edge_reduction_means = []
    edge_reduction_stds = []
    lambda2_means = []
    lambda2_stds = []
    
    for method in methods:
        method_results = [r for r in results if r['method'] == method and r['success']]
        
        if method_results:
            edge_reduction_means.append(np.mean([r['edge_reduction_pct'] for r in method_results]))
            edge_reduction_stds.append(np.std([r['edge_reduction_pct'] for r in method_results]))
            lambda2_means.append(np.mean([r['min_lambda2'] for r in method_results]))
            lambda2_stds.append(np.std([r['min_lambda2'] for r in method_results]))
        else:
            edge_reduction_means.append(0)
            edge_reduction_stds.append(0)
            lambda2_means.append(0)
            lambda2_stds.append(0)
    
    # Left plot: Edge Reduction
    x = range(len(methods))
    bars1 = ax1.bar(x, edge_reduction_means,
                    color=[METHOD_COLORS[m] for m in methods],
                    alpha=0.7, edgecolor='black', linewidth=0.8)
    
    ax1.set_xticks(x)
    ax1.set_xticklabels([METHOD_LABELS[m] for m in methods], rotation=20, ha='right', fontsize=7)
    ax1.set_ylabel('Edge Reduction (%)')
    ax1.set_title('Network Sparsity')
    ax1.grid(True, alpha=0.3, axis='y')
    ax1.set_ylim(0, max(edge_reduction_means) * 1.2)
    
    # Right plot: Min Lambda2
    bars2 = ax2.bar(x, lambda2_means,
                    color=[METHOD_COLORS[m] for m in methods],
                    alpha=0.7, edgecolor='black', linewidth=0.8)
    
    ax2.axhline(y=0.2, color='red', linestyle='--', linewidth=1, alpha=0.6, label='Safety Threshold')
    
    ax2.set_xticks(x)
    ax2.set_xticklabels([METHOD_LABELS[m] for m in methods], rotation=20, ha='right', fontsize=7)
    ax2.set_ylabel(r'Minimum $\lambda_2$')
    ax2.set_title('Connectivity Safety')
    ax2.grid(True, alpha=0.3, axis='y')
    ax2.legend(fontsize=7)
    
    fig.suptitle('Trade-off: Network Sparsity vs Connectivity Safety')
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_r1_tradeoff_analysis.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_r1_tradeoff_analysis.png")


def plot_r2_performance_distributions(results: List[Dict], output_dir: Path):
    """Recommended Plot 2: Performance distributions (bar plots)."""
    print("Generating Recommended Plot 2: Performance Distributions...")
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    
    methods = ['ConcurrentPruning', 'AdjacencyConsensus', 'FullGraph', 'CentralizedMST']
    
    # Left: Edge Reduction Distribution
    reduction_means = []
    reduction_stds = []
    
    for method in methods:
        method_results = [r for r in results if r['method'] == method and r['success']]
        reduction_values = [r['edge_reduction_pct'] for r in method_results]
        reduction_means.append(np.mean(reduction_values))
        reduction_stds.append(np.std(reduction_values))
    
    x = np.arange(len(methods))
    ax1.bar(x, reduction_means,
            color=[METHOD_COLORS[m] for m in methods],
            alpha=0.7, edgecolor='black', linewidth=0.8)
    
    ax1.set_xticks(x)
    ax1.set_xticklabels([METHOD_LABELS[m] for m in methods], rotation=20, ha='right', fontsize=7)
    ax1.set_ylabel('Edge Reduction (%)')
    ax1.set_title('Edge Reduction Distribution')
    ax1.grid(True, alpha=0.3, axis='y')
    
    # Right: Minimum Lambda2 Distribution
    lambda2_means = []
    lambda2_stds = []
    
    for method in methods:
        method_results = [r for r in results if r['method'] == method and r['success']]
        lambda2_values = [r['min_lambda2'] for r in method_results]
        lambda2_means.append(np.mean(lambda2_values))
        lambda2_stds.append(np.std(lambda2_values))
    
    ax2.bar(x, lambda2_means,
            color=[METHOD_COLORS[m] for m in methods],
            alpha=0.7, edgecolor='black', linewidth=0.8)
    
    ax2.axhline(y=0.2, color='red', linestyle='--', linewidth=1, alpha=0.6, label='Safety Threshold')
    
    ax2.set_xticks(x)
    ax2.set_xticklabels([METHOD_LABELS[m] for m in methods], rotation=20, ha='right', fontsize=7)
    ax2.set_ylabel(r'Minimum $\lambda_2$')
    ax2.set_title('Connectivity Safety Distribution')
    ax2.grid(True, alpha=0.3, axis='y')
    ax2.legend(fontsize=7)
    
    fig.suptitle('Method Performance Distributions')
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_r2_performance_distributions.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_r2_performance_distributions.png")


def plot_r3_runtime_efficiency(results: List[Dict], output_dir: Path):
    """Recommended Plot 3: Runtime comparison by method."""
    print("Generating Recommended Plot 3: Runtime Efficiency...")
    
    fig, ax = plt.subplots(figsize=(6, 4))
    
    methods = ['ConcurrentPruning', 'AdjacencyConsensus', 'FullGraph', 'CentralizedMST']
    
    # Calculate runtime statistics
    runtime_means = []
    runtime_stds = []
    
    for method in methods:
        method_results = [r for r in results if r['method'] == method and r['success']]
        runtimes = [r['elapsed_time'] for r in method_results]
        runtime_means.append(np.mean(runtimes))
        runtime_stds.append(np.std(runtimes))
    
    # Create bar plot
    x = np.arange(len(methods))
    ax.bar(x, runtime_means,
           color=[METHOD_COLORS[m] for m in methods],
           alpha=0.7, edgecolor='black', linewidth=0.8)
    
    ax.set_xticks(x)
    ax.set_xticklabels([METHOD_LABELS[m] for m in methods], rotation=20, ha='right', fontsize=7)
    ax.set_ylabel('Computation Time (seconds)')
    ax.set_title('Computational Efficiency Comparison')
    ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_r3_runtime_efficiency.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_r3_runtime_efficiency.png")


def plot_r4_pruning_events_by_scenario(results: List[Dict], output_dir: Path):
    """Recommended Plot 4: Average pruning events by scenario for pruning methods."""
    print("Generating Recommended Plot 4: Pruning Events by Scenario...")
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    scenarios = ['A', 'B', 'C', 'D', 'E']
    # Only show the 3 methods that actually perform pruning
    methods = ['ConcurrentPruning', 'AdjacencyConsensus', 'CentralizedMST']
    
    x = np.arange(len(scenarios))
    width = 0.25
    
    for i, method in enumerate(methods):
        means = []
        
        for scenario in scenarios:
            trials = [r for r in results if r['method'] == method and r['scenario'] == scenario and r['success']]
            
            if trials:
                # Get pruning events count from each trial
                pruning_counts = []
                for trial in trials:
                    if 'pruning_events' in trial and isinstance(trial['pruning_events'], list):
                        pruning_counts.append(len(trial['pruning_events']))
                    else:
                        pruning_counts.append(0)
                
                means.append(np.mean(pruning_counts) if pruning_counts else 0)
            else:
                means.append(0)
        
        offset = (i - 1) * width
        label = METHOD_LABELS[method].replace(' (Oracle)', '')
        ax.bar(x + offset, means, width,
               label=label, color=METHOD_COLORS[method],
               alpha=0.8, edgecolor='black', linewidth=0.8)
    
    ax.set_xticks(x)
    ax.set_xticklabels([f'{s}: {SCENARIO_LABELS[s]}' for s in scenarios], fontsize=9)
    ax.set_xlabel('Scenario', fontweight='bold', fontsize=11)
    ax.set_ylabel('Average Pruning Events', fontweight='bold', fontsize=11)
    ax.set_title('Pruning Efficiency: Average Pruning Events by Scenario', fontweight='bold', fontsize=12)
    ax.legend(fontsize=9, loc='best', framealpha=0.9)
    ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig(output_dir / 'plot_r4_pruning_events_by_scenario.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("  ✓ Saved: plot_r4_pruning_events_by_scenario.png")


def generate_summary_statistics(results: List[Dict], output_dir: Path):
    """Generate summary statistics table."""
    print("Generating Summary Statistics...")
    
    methods = ['ConcurrentPruning', 'AdjacencyConsensus', 'FullGraph', 'CentralizedMST']
    scenarios = ['A', 'B', 'C', 'D', 'E']
    
    output_file = output_dir / 'summary_statistics.txt'
    
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write("="*80 + "\n")
        f.write("EXPERIMENT SUMMARY STATISTICS\n")
        f.write("="*80 + "\n\n")
        
        for method in methods:
            f.write(f"\n{METHOD_LABELS[method]}\n")
            f.write("-"*80 + "\n")
            
            method_results = [r for r in results if r['method'] == method]
            success_results = [r for r in method_results if r['success']]
            
            f.write(f"Overall Success Rate: {100*len(success_results)/len(method_results):.1f}%\n")
            
            if success_results:
                f.write(f"Average Edge Reduction: {np.mean([r['edge_reduction_pct'] for r in success_results]):.2f}% ± {np.std([r['edge_reduction_pct'] for r in success_results]):.2f}%\n")
                f.write(f"Average Min λ₂: {np.mean([r['min_lambda2'] for r in success_results]):.4f} ± {np.std([r['min_lambda2'] for r in success_results]):.4f}\n")
                f.write(f"Average Runtime: {np.mean([r['elapsed_time'] for r in success_results]):.2f}s ± {np.std([r['elapsed_time'] for r in success_results]):.2f}s\n")
                
                f.write("\nBy Scenario:\n")
                for scenario in scenarios:
                    scenario_results = [r for r in success_results if r['scenario'] == scenario]
                    if scenario_results:
                        f.write(f"  {scenario} ({SCENARIO_LABELS[scenario]}): "
                               f"Success={len(scenario_results)}/{len([r for r in method_results if r['scenario'] == scenario])}, "
                               f"Edges={np.mean([r['edge_reduction_pct'] for r in scenario_results]):.1f}%, "
                               f"λ₂={np.mean([r['min_lambda2'] for r in scenario_results]):.3f}\n")
        
        f.write("\n" + "="*80 + "\n")
    
    print(f"  ✓ Saved: summary_statistics.txt")


def main():
    """Main function to generate all plots."""
    import sys
    
    # Get results file (latest by default)
    results_file = None
    
    if len(sys.argv) > 1:
        results_file = Path(sys.argv[1])
    else:
        # Find latest results file
        results_dir = Path(__file__).parent.parent
        result_files = list(results_dir.glob('comprehensive_results_*.json'))
        if result_files:
            results_file = max(result_files, key=lambda p: p.stat().st_mtime)
    
    if not results_file or not results_file.exists():
        print("ERROR: No results file found.")
        print("Usage: python generate_all_plots.py [path/to/results.json]")
        sys.exit(1)
    
    # Create output directory
    output_dir = results_file.parent / 'figures'
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print("\n" + "="*80)
    print("GENERATING ALL PLOTS")
    print("="*80)
    print(f"Results file: {results_file}")
    print(f"Output directory: {output_dir}")
    print("="*80 + "\n")
    
    # Load results
    print("Loading results...")
    data = load_results(str(results_file))
    results = data['results']
    print(f"  ✓ Loaded {len(results)} trial results\n")
    
    # Generate all plots
    print("Generating plots...\n")
    
    # User requested plots (7)
    plot_1_robot_trajectories(results, output_dir)
    plot_2_edge_evolution(results, output_dir)
    plot_3_lambda2_evolution_and_distribution(results, output_dir)
    plot_4_control_effort(results, output_dir)
    plot_5_success_rate_heatmap(results, output_dir)
    plot_6_min_lambda2_by_scenario(results, output_dir)
    plot_7_pruning_efficiency(results, output_dir)
    
    # Recommended plots (3)
    plot_r1_trade_off_analysis(results, output_dir)
    plot_r2_performance_distributions(results, output_dir)
    plot_r3_runtime_efficiency(results, output_dir)
    plot_r4_pruning_events_by_scenario(results, output_dir)
    
    # Summary statistics
    generate_summary_statistics(results, output_dir)
    
    print("\n" + "="*80)
    print("ALL PLOTS GENERATED SUCCESSFULLY!")
    print("="*80)
    print(f"Output directory: {output_dir}")
    print(f"Total plots: 11 PNG files")
    print("="*80)


if __name__ == '__main__':
    main()
