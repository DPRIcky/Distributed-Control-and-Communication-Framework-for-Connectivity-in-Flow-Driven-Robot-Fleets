"""Generate publication-quality figures for ACC 2026 revision.

Creates 8 figures validating theorems T1-T4:
1. Flow field and workspace setup
2. Robot trajectories with safety constraints (T1)
3. Edge pruning dynamics over time (T4)
4. Lambda2 evolution and connectivity (T3)
5. Consensus convergence characteristics
6. Goal convergence analysis (T2)
7. Control effort comparison
8. Method comparison across scenarios
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import seaborn as sns
from collections import defaultdict

# Set publication-quality parameters
plt.rcParams.update({
    'font.size': 10,
    'font.family': 'serif',
    'font.serif': ['Times New Roman'],
    'axes.labelsize': 11,
    'axes.titlesize': 12,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'legend.fontsize': 9,
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.1
})

# Color scheme for methods
METHOD_COLORS = {
    'hybrid': '#e74c3c',           # Red (our method)
    'full_graph': '#95a5a6',       # Gray
    'centralized_mst': '#3498db',  # Blue
    'random': '#f39c12',           # Orange
    'greedy_distance': '#2ecc71'   # Green
}

METHOD_LABELS = {
    'hybrid': 'Hybrid (NOVEL)',
    'full_graph': 'Full Graph',
    'centralized_mst': 'Centralized MST',
    'random': 'Random Pruning',
    'greedy_distance': 'Greedy Distance'
}


def load_batch_results(filepath):
    """Load batch simulation results from JSON."""
    with open(filepath, 'r') as f:
        data = json.load(f)
    return data


def figure1_flow_field_workspace():
    """Figure 1: Flow field visualization and workspace setup (matches GUI)."""
    from config.scenarios import get_scenario
    from config import VisualizationConfig
    from simulation import HybridUnderwaterSimulation
    from matplotlib.lines import Line2D
    
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111)
    
    # Create simulation with same settings as GUI
    sim_cfg, ctrl_cfg, cons_cfg, fp, _ = get_scenario('A')
    sim = HybridUnderwaterSimulation(sim_cfg, ctrl_cfg, cons_cfg)
    vis_config = VisualizationConfig()
    
    # Draw goal position with circle (like GUI)
    ax.scatter(*sim.goal_position, s=400, c='gold', marker='*',
              edgecolors='darkorange', linewidths=2.0, zorder=10, label='Goal')
    
    goal_circle = plt.Circle(
        sim.goal_position, 0.5, fill=False, color='gold',
        linestyle='--', alpha=0.5, linewidth=2
    )
    ax.add_patch(goal_circle)
    
    # Flow field visualization using quiver (like GUI)
    grid_x = np.linspace(0.0, sim.workspace[0], vis_config.flow_grid_points)
    grid_y = np.linspace(0.0, sim.workspace[1], vis_config.flow_grid_points)
    gx, gy = np.meshgrid(grid_x, grid_y)
    flow_grid_points = np.column_stack((gx.ravel(), gy.ravel()))
    initial_flow = np.array(
        [sim.flow.velocity(p, sim.time) for p in flow_grid_points]
    )
    ax.quiver(
        flow_grid_points[:, 0],
        flow_grid_points[:, 1],
        initial_flow[:, 0],
        initial_flow[:, 1],
        color="#B0C4DE",
        alpha=vis_config.flow_alpha,
        width=vis_config.flow_width,
        scale=vis_config.flow_scale,
        zorder=0,
    )
    
    # Plot robot initial positions
    positions = np.array([r.position for r in sim.robots])
    ax.scatter(positions[:, 0], positions[:, 1], s=vis_config.node_size, 
              c='#e74c3c', edgecolors='white', linewidths=1.0, zorder=10)
    
    # Add edges
    edges = sim.topology_manager.get_current_edges()
    edge_segments = [[positions[i], positions[j]] for i, j in edges]
    from matplotlib.collections import LineCollection
    lc = LineCollection(edge_segments, colors='#9b59b6', 
                       linewidths=vis_config.active_linewidth, zorder=3)
    ax.add_collection(lc)
    
    # Communication radius circle on one robot (2m for visualization)
    R_comm_display = 2.0
    circle = plt.Circle(positions[0], R_comm_display, 
                       fill=False, color='#e74c3c', linestyle='--', 
                       linewidth=1.5, alpha=0.5)
    ax.add_patch(circle)
    
    ax.set_xlim(0.0, sim.workspace[0])
    ax.set_ylim(0.0, sim.workspace[1])
    ax.set_xlabel('x (m)', fontweight='bold')
    ax.set_ylabel('y (m)', fontweight='bold')
    ax.set_title('Hybrid Consensus Pruning (Multi-Layer Robustness)', fontsize=12, fontweight='bold')
    
    # Custom legend matching GUI
    legend_elements = [
        Line2D([0], [0], color='#9b59b6', linewidth=2.5, label='Active Edges'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#e74c3c',
               markersize=8, label='Connected Robot'),
        Line2D([0], [0], marker='*', color='gold', markersize=12, label='Goal'),
        Line2D([0], [0], color='#e74c3c', linestyle='--',
              linewidth=1.5, label='R_comm = 2.0m')
    ]
    ax.legend(handles=legend_elements, loc='upper right', fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    return fig


def figure2_trajectories_safety(results_file):
    """Figure 2: Robot trajectories with safety constraints (Theorem T1)."""
    from config.scenarios import get_scenario
    from simulation import HybridUnderwaterSimulation
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    # Run one simulation to get trajectory data
    sim_cfg, ctrl_cfg, cons_cfg, fp, _ = get_scenario('A')
    sim_cfg.verbose = False
    sim = HybridUnderwaterSimulation(sim_cfg, ctrl_cfg, cons_cfg)
    
    # Track trajectories and topology snapshots
    trajectories = [[] for _ in range(sim.num_robots)]
    min_distances = []
    initial_edges = None
    final_edges = None
    
    for step in range(100):
        for i, robot in enumerate(sim.robots):
            trajectories[i].append(robot.position.copy())
        
        # Capture initial edges (step 0)
        if step == 0:
            initial_edges = set(sim.topology_manager.get_current_edges())
        
        sim.step()
        
        # Compute minimum inter-robot distance
        positions = np.array([r.position for r in sim.robots])
        dists = []
        for i in range(len(positions)):
            for j in range(i+1, len(positions)):
                dists.append(np.linalg.norm(positions[i] - positions[j]))
        min_distances.append(min(dists) if dists else 0)
    
    # Capture final edges (after t=5s)
    final_edges = set(sim.topology_manager.get_current_edges())
    
    # Left plot: Trajectories with connectivity overlay
    ax = axes[0]
    
    # Draw trajectories
    for i, traj in enumerate(trajectories):
        traj = np.array(traj)
        ax.plot(traj[:, 0], traj[:, 1], alpha=0.6, linewidth=1.5)
        ax.scatter(traj[0, 0], traj[0, 1], s=80, c='blue', 
                  edgecolors='white', linewidths=1, zorder=10, marker='o')
        ax.scatter(traj[-1, 0], traj[-1, 1], s=80, c='red', 
                  edgecolors='white', linewidths=1, zorder=10, marker='s')
    
    # Draw initial connectivity (at start positions) - dashed gray
    from matplotlib.collections import LineCollection
    initial_positions = np.array([traj[0] for traj in trajectories])
    initial_lines = []
    for edge in initial_edges:
        i, j = edge
        initial_lines.append([initial_positions[i], initial_positions[j]])
    lc_initial = LineCollection(initial_lines, colors='gray', linewidths=1.5, 
                                linestyles='dashed', alpha=0.4, zorder=1)
    ax.add_collection(lc_initial)
    
    # Draw final connectivity (at end positions) - solid green for kept, red for pruned
    final_positions = np.array([traj[-1] for traj in trajectories])
    kept_lines = []
    for edge in final_edges:
        i, j = edge
        kept_lines.append([final_positions[i], final_positions[j]])
    lc_kept = LineCollection(kept_lines, colors='green', linewidths=2, 
                            linestyles='solid', alpha=0.6, zorder=2)
    ax.add_collection(lc_kept)
    
    ax.scatter(*sim.goal_position, s=400, c='gold', marker='*',
              edgecolors='darkorange', linewidths=2, zorder=10)
    
    ax.set_xlabel('x (m)', fontweight='bold')
    ax.set_ylabel('y (m)', fontweight='bold')
    ax.set_title('Robot Trajectories (T1: Safety)', fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    # Add legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor='blue',
               markersize=8, label='Start'),
        Line2D([0], [0], marker='s', color='w', markerfacecolor='red',
               markersize=8, label='End (t=5s)'),
        Line2D([0], [0], marker='*', color='gold', markersize=12, label='Goal'),
        Line2D([0], [0], color='gray', linestyle='--', linewidth=1.5, 
               label='Initial Edges'),
        Line2D([0], [0], color='green', linestyle='-', linewidth=2, 
               label='Pruned Edges')
    ]
    ax.legend(handles=legend_elements, loc='upper right')
    
    # Right plot: Safety constraint (minimum distance)
    ax = axes[1]
    time = np.arange(len(min_distances)) * sim.dt
    ax.plot(time, min_distances, linewidth=2, color='#e74c3c', label='Min Distance')
    ax.axhline(y=ctrl_cfg.safety_distance, color='orange', linestyle='--', 
              linewidth=2, label=f'd_safe = {ctrl_cfg.safety_distance}m')
    
    ax.set_xlabel('Time (s)', fontweight='bold')
    ax.set_ylabel('Minimum Inter-Robot Distance (m)', fontweight='bold')
    ax.set_title('Safety Constraint Satisfaction (T1)', fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    plt.tight_layout()
    return fig


def figure3_pruning_dynamics(results_file=None):
    """Figure 3: Edge pruning dynamics over time (Theorem T4)."""
    from config.scenarios import get_scenario
    from baselines.baseline_simulation import BaselineSimulation
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    methods = ['hybrid', 'random', 'greedy_distance', 'centralized_mst']
    
    for idx, method in enumerate(methods):
        ax = axes[idx // 2, idx % 2]
        
        # Run simulation
        sim_cfg, ctrl_cfg, cons_cfg, fp, _ = get_scenario('A')
        sim_cfg.seed = 42
        sim_cfg.verbose = False
        sim = BaselineSimulation(sim_cfg, ctrl_cfg, cons_cfg, method)
        
        edge_history = []
        time_history = []
        pruning_times = []
        
        for step in range(100):
            sim.step()
            edge_history.append(len(sim.topology_manager.get_current_edges()))
            time_history.append(sim.time)
            
            # Check if pruning occurred
            if step > 0 and edge_history[-1] < edge_history[-2]:
                pruning_times.append(sim.time)
        
        # Plot edge count over time
        ax.plot(time_history, edge_history, linewidth=2, 
               color=METHOD_COLORS[method], label=METHOD_LABELS[method])
        
        # Mark pruning events
        for pt in pruning_times:
            ax.axvline(x=pt, color='red', alpha=0.3, linestyle=':', linewidth=1)
        
        ax.set_xlabel('Time (s)', fontweight='bold')
        ax.set_ylabel('Number of Edges', fontweight='bold')
        ax.set_title(f'{METHOD_LABELS[method]}', fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        # Add text annotation
        final_edges = edge_history[-1]
        pruning_count = len(pruning_times)
        ax.text(0.95, 0.95, f'Final: {final_edges} edges\nPruned: {pruning_count} times',
               transform=ax.transAxes, ha='right', va='top',
               bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    plt.suptitle('Edge Pruning Dynamics (T4: Distributed Correctness)', 
                fontsize=14, fontweight='bold', y=0.995)
    plt.tight_layout()
    return fig


def figure4_lambda2_evolution(results_file):
    """Figure 4: Lambda2 evolution showing connectivity maintenance (Theorem T3)."""
    from config.scenarios import get_scenario
    from baselines.baseline_simulation import BaselineSimulation
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    methods = ['hybrid', 'full_graph', 'centralized_mst', 'random', 'greedy_distance']
    
    # Left plot: Single scenario, all methods
    ax = axes[0]
    for method in methods:
        sim_cfg, ctrl_cfg, cons_cfg, fp, _ = get_scenario('A')
        sim_cfg.seed = 42
        sim_cfg.verbose = False
        sim = BaselineSimulation(sim_cfg, ctrl_cfg, cons_cfg, method)
        
        lambda2_history = []
        time_history = []
        
        for step in range(100):
            sim.step()
            
            # Get lambda2 from metrics
            if len(sim.metrics.lambda2_values) > 0:
                lambda2_history.append(sim.metrics.lambda2_values[-1])
                time_history.append(sim.time)
        
        ax.plot(time_history, lambda2_history, linewidth=2,
               color=METHOD_COLORS[method], label=METHOD_LABELS[method], alpha=0.8)
    
    ax.axhline(y=cons_cfg.lambda2_threshold, color='orange', linestyle='--',
              linewidth=2, label=f'Threshold = {cons_cfg.lambda2_threshold}')
    
    ax.set_xlabel('Time (s)', fontweight='bold')
    ax.set_ylabel('Algebraic Connectivity (λ₂)', fontweight='bold')
    ax.set_title('λ₂ Evolution - Scenario A (T3)', fontweight='bold')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=-0.5)
    
    # Right plot: Box plot of minimum lambda2 across scenarios
    ax = axes[1]
    
    # Load batch results
    data = load_batch_results(results_file)
    
    # Organize data for box plot
    plot_data = []
    labels = []
    positions = []
    pos = 0
    
    for method in methods:
        method_results = [r for r in data['results'] 
                         if r.get('method') == method and 'error' not in r]
        min_lambda2_values = [r['min_lambda2'] for r in method_results]
        
        plot_data.append(min_lambda2_values)
        labels.append(METHOD_LABELS[method].replace(' ', '\n'))
        positions.append(pos)
        pos += 1
    
    bp = ax.boxplot(plot_data, positions=positions, widths=0.6,
                    patch_artist=True, showfliers=True)
    
    # Color boxes
    for patch, method in zip(bp['boxes'], methods):
        patch.set_facecolor(METHOD_COLORS[method])
        patch.set_alpha(0.7)
    
    ax.axhline(y=cons_cfg.lambda2_threshold, color='orange', linestyle='--',
              linewidth=2, label=f'Threshold')
    
    ax.set_ylabel('Minimum λ₂ Achieved', fontweight='bold')
    ax.set_title('λ₂ Distribution Across All Scenarios (T3)', fontweight='bold')
    ax.set_xticks(positions)
    ax.set_xticklabels(labels, fontsize=8)
    ax.grid(True, alpha=0.3, axis='y')
    ax.legend()
    
    plt.tight_layout()
    return fig


def figure5_consensus_convergence():
    """Figure 5: Consensus convergence characteristics."""
    from config.scenarios import get_scenario
    from baselines.baseline_simulation import BaselineSimulation
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    # Left plot: Convergence iterations over time
    ax = axes[0]
    
    sim_cfg, ctrl_cfg, cons_cfg, fp, _ = get_scenario('A')
    sim_cfg.seed = 42
    sim_cfg.verbose = False
    sim = BaselineSimulation(sim_cfg, ctrl_cfg, cons_cfg, 'hybrid')
    
    consensus_iterations = []
    time_history = []
    
    # Initialize tracking
    if not hasattr(sim.pruning_manager, 'last_convergence_iterations'):
        sim.pruning_manager.last_convergence_iterations = 0
    
    previous_pruning_count = 0
    last_convergence_value = 0
    
    for step in range(100):
        result = sim.step()
        
        # Check if a new pruning event occurred
        current_pruning_count = len(sim.metrics.pruning_events)
        
        if current_pruning_count > previous_pruning_count:
            # New pruning event - show the consensus iterations that led to it
            if hasattr(sim.pruning_manager, 'last_convergence_iterations'):
                last_convergence_value = sim.pruning_manager.last_convergence_iterations
            consensus_iterations.append(last_convergence_value)
            previous_pruning_count = current_pruning_count
        else:
            # No new pruning - show 0 (consensus not currently running)
            consensus_iterations.append(0)
        
        time_history.append(sim.time)
    
    ax.plot(time_history, consensus_iterations, linewidth=2, color='#e74c3c', drawstyle='steps-post')
    ax.set_xlabel('Time (s)', fontweight='bold')
    ax.set_ylabel('Consensus Iterations', fontweight='bold')
    ax.set_title('Consensus Algorithm Iterations', fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    # Right plot: Pruning events and consensus convergence
    ax = axes[1]
    
    # Count pruning events over time
    pruning_times = [e['time'] for e in sim.metrics.pruning_events]
    pruning_count = list(range(1, len(pruning_times) + 1))
    
    ax.step(pruning_times, pruning_count, where='post', linewidth=2, 
           color='#e74c3c', label='Cumulative Pruning Events')
    ax.set_xlabel('Time (s)', fontweight='bold')
    ax.set_ylabel('Cumulative Pruning Events', fontweight='bold')
    ax.set_title('Distributed Consensus-Based Pruning Progress', fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # Add text annotation
    total_pruned = len(pruning_times)
    ax.text(0.95, 0.05, f'Total pruned: {total_pruned} edges',
           transform=ax.transAxes, ha='right', va='bottom',
           bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
           fontsize=10)
    
    plt.tight_layout()
    return fig


def figure6_goal_convergence(results_file):
    """Figure 6: Goal convergence analysis (Theorem T2)."""
    from config.scenarios import get_scenario
    from baselines.baseline_simulation import BaselineSimulation
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    # Left plot: Distance to goal over time
    ax = axes[0]
    
    methods = ['hybrid', 'full_graph']
    
    for method in methods:
        sim_cfg, ctrl_cfg, cons_cfg, fp, _ = get_scenario('A')
        sim_cfg.seed = 42
        sim_cfg.verbose = False
        sim = BaselineSimulation(sim_cfg, ctrl_cfg, cons_cfg, method)
        
        goal_distances = []
        time_history = []
        
        for step in range(100):
            sim.step()
            
            # Compute mean distance to goal
            positions = np.array([r.position for r in sim.robots])
            dists = [np.linalg.norm(pos - sim.goal_position) for pos in positions]
            goal_distances.append(np.mean(dists))
            time_history.append(sim.time)
        
        ax.plot(time_history, goal_distances, linewidth=2,
               color=METHOD_COLORS[method], label=METHOD_LABELS[method])
    
    ax.set_xlabel('Time (s)', fontweight='bold')
    ax.set_ylabel('Mean Distance to Goal (m)', fontweight='bold')
    ax.set_title('Goal Convergence (T2)', fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Right plot: Final goal distance distribution
    ax = axes[1]
    
    data = load_batch_results(results_file)
    
    methods = ['hybrid', 'full_graph', 'centralized_mst', 'random', 'greedy_distance']
    plot_data = []
    labels = []
    
    for method in methods:
        method_results = [r for r in data['results'] 
                         if r.get('method') == method and 'error' not in r]
        goal_dists = [r['goal_distance'] for r in method_results]
        plot_data.append(goal_dists)
        labels.append(METHOD_LABELS[method].replace(' ', '\n'))
    
    bp = ax.boxplot(plot_data, widths=0.6, patch_artist=True, showfliers=True)
    
    for patch, method in zip(bp['boxes'], methods):
        patch.set_facecolor(METHOD_COLORS[method])
        patch.set_alpha(0.7)
    
    ax.set_ylabel('Final Mean Distance to Goal (m)', fontweight='bold')
    ax.set_title('Goal Convergence Distribution (T2)', fontweight='bold')
    ax.set_xticklabels(labels, fontsize=8)
    ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    return fig


def figure7_control_effort():
    """Figure 7: Control effort comparison."""
    from config.scenarios import get_scenario
    from baselines.baseline_simulation import BaselineSimulation
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    methods = ['hybrid', 'full_graph', 'centralized_mst']
    
    # Left plot: Control effort over time
    ax = axes[0]
    
    for method in methods:
        sim_cfg, ctrl_cfg, cons_cfg, fp, _ = get_scenario('A')
        sim_cfg.seed = 42
        sim_cfg.verbose = False
        sim = BaselineSimulation(sim_cfg, ctrl_cfg, cons_cfg, method)
        
        control_history = []
        time_history = []
        
        for step in range(100):
            sim.step()
            
            # Get total control effort from metrics
            # Note: control for step N is stored in control_magnitudes[N-1] due to recording order
            if step > 0 and len(sim.metrics.control_magnitudes) > 0:
                # Use the previous entry which has the current step's control
                idx = min(step - 1, len(sim.metrics.control_magnitudes) - 1)
                total_control = np.sum(sim.metrics.control_magnitudes[idx])
            else:
                total_control = 0
            
            control_history.append(total_control)
            time_history.append(sim.time)
        
        ax.plot(time_history, control_history, linewidth=2,
               color=METHOD_COLORS[method], label=METHOD_LABELS[method])
    
    ax.set_xlabel('Time (s)', fontweight='bold')
    ax.set_ylabel('Total Control Effort', fontweight='bold')
    ax.set_title('Control Effort Over Time', fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Right plot: Edge count vs control effort trade-off
    ax = axes[1]
    
    edge_counts = []
    avg_controls = []
    
    for method in methods:
        sim_cfg, ctrl_cfg, cons_cfg, fp, _ = get_scenario('A')
        sim_cfg.seed = 42
        sim_cfg.verbose = False
        sim = BaselineSimulation(sim_cfg, ctrl_cfg, cons_cfg, method)
        
        controls = []
        
        for step in range(100):
            sim.step()
            # Get control effort (with proper offset for recording order)
            if step > 0 and len(sim.metrics.control_magnitudes) > 0:
                idx = min(step - 1, len(sim.metrics.control_magnitudes) - 1)
                total_control = np.sum(sim.metrics.control_magnitudes[idx])
            else:
                total_control = 0
            controls.append(total_control)
        
        final_edges = len(sim.topology_manager.get_current_edges())
        avg_control = np.mean(controls)
        
        edge_counts.append(final_edges)
        avg_controls.append(avg_control)
        
        ax.scatter(final_edges, avg_control, s=200, 
                  color=METHOD_COLORS[method], label=METHOD_LABELS[method],
                  edgecolors='black', linewidths=1.5, alpha=0.7)
    
    ax.set_xlabel('Final Edge Count', fontweight='bold')
    ax.set_ylabel('Average Control Effort', fontweight='bold')
    ax.set_title('Topology Sparsity vs Control Trade-off', fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig


def figure8_method_comparison(results_file):
    """Figure 8: Comprehensive method comparison across scenarios."""
    data = load_batch_results(results_file)
    
    fig = plt.figure(figsize=(14, 10))
    gs = GridSpec(2, 2, figure=fig, hspace=0.3, wspace=0.3)
    
    scenarios = ['A', 'B', 'C', 'D', 'E']
    methods = ['hybrid', 'full_graph', 'centralized_mst', 'random', 'greedy_distance']
    
    # Subplot 1: Success rate by scenario
    ax1 = fig.add_subplot(gs[0, 0])
    
    success_matrix = np.zeros((len(methods), len(scenarios)))
    for i, method in enumerate(methods):
        for j, scenario in enumerate(scenarios):
            scenario_results = [r for r in data['results'] 
                              if r.get('method') == method and r.get('scenario') == scenario
                              and 'error' not in r]
            if scenario_results:
                success_rate = 100 * sum(r['success'] for r in scenario_results) / len(scenario_results)
                success_matrix[i, j] = success_rate
    
    im1 = ax1.imshow(success_matrix, cmap='RdYlGn', aspect='auto', vmin=0, vmax=100)
    ax1.set_xticks(range(len(scenarios)))
    ax1.set_yticks(range(len(methods)))
    ax1.set_xticklabels(scenarios)
    ax1.set_yticklabels([METHOD_LABELS[m] for m in methods], fontsize=8)
    ax1.set_xlabel('Scenario', fontweight='bold')
    ax1.set_ylabel('Method', fontweight='bold')
    ax1.set_title('Success Rate (%) - Connectivity Maintained', fontweight='bold')
    
    # Add text annotations
    for i in range(len(methods)):
        for j in range(len(scenarios)):
            text = ax1.text(j, i, f'{success_matrix[i, j]:.0f}',
                          ha="center", va="center", color="black", fontsize=8)
    
    plt.colorbar(im1, ax=ax1, label='Success Rate (%)')
    
    # Subplot 2: Pruning efficiency (edges pruned)
    ax2 = fig.add_subplot(gs[0, 1])
    
    x = np.arange(len(scenarios))
    width = 0.15
    
    for i, method in enumerate(methods):
        pruning_means = []
        for scenario in scenarios:
            scenario_results = [r for r in data['results']
                              if r.get('method') == method and r.get('scenario') == scenario
                              and 'error' not in r]
            if scenario_results:
                avg_pruning = np.mean([r['pruning_events'] for r in scenario_results])
                pruning_means.append(avg_pruning)
            else:
                pruning_means.append(0)
        
        ax2.bar(x + i*width, pruning_means, width, 
               label=METHOD_LABELS[method], color=METHOD_COLORS[method], alpha=0.8)
    
    ax2.set_xlabel('Scenario', fontweight='bold')
    ax2.set_ylabel('Average Pruning Events', fontweight='bold')
    ax2.set_title('Pruning Efficiency by Scenario', fontweight='bold')
    ax2.set_xticks(x + width * 2)
    ax2.set_xticklabels(scenarios)
    ax2.legend(fontsize=7, ncol=2)
    ax2.grid(True, alpha=0.3, axis='y')
    
    # Subplot 3: Lambda2 comparison
    ax3 = fig.add_subplot(gs[1, 0])
    
    for method in methods:
        lambda2_by_scenario = []
        for scenario in scenarios:
            scenario_results = [r for r in data['results']
                              if r.get('method') == method and r.get('scenario') == scenario
                              and 'error' not in r]
            if scenario_results:
                avg_lambda2 = np.mean([r['min_lambda2'] for r in scenario_results])
                lambda2_by_scenario.append(avg_lambda2)
            else:
                lambda2_by_scenario.append(0)
        
        ax3.plot(scenarios, lambda2_by_scenario, marker='o', linewidth=2,
                markersize=8, color=METHOD_COLORS[method], label=METHOD_LABELS[method])
    
    ax3.set_xlabel('Scenario', fontweight='bold')
    ax3.set_ylabel('Average Min λ₂', fontweight='bold')
    ax3.set_title('Connectivity Strength by Scenario', fontweight='bold')
    ax3.legend(fontsize=7)
    ax3.grid(True, alpha=0.3)
    
    # Subplot 4: Overall performance radar
    ax4 = fig.add_subplot(gs[1, 1], projection='polar')
    
    # Metrics: Success, Pruning, Lambda2, Goal (normalized 0-1)
    angles = np.linspace(0, 2*np.pi, 4, endpoint=False).tolist()
    angles += angles[:1]
    
    metric_labels = ['Success\nRate', 'Pruning\nEvents', 'Min λ₂', 'Goal\nConv.']
    ax4.set_xticks(angles[:-1])
    ax4.set_xticklabels(metric_labels, fontsize=8)
    
    for method in methods:
        method_results = [r for r in data['results'] 
                         if r.get('method') == method and 'error' not in r]
        
        if method_results:
            success = sum(r['success'] for r in method_results) / len(method_results)
            pruning = np.mean([r['pruning_events'] for r in method_results]) / 50  # Normalize
            lambda2 = np.mean([r['min_lambda2'] for r in method_results]) / 5  # Normalize
            goal = 1 - np.mean([r['goal_distance'] for r in method_results]) / 10  # Normalize (closer = better)
            
            values = [success, pruning, lambda2, goal]
            values += values[:1]
            
            ax4.plot(angles, values, 'o-', linewidth=2, color=METHOD_COLORS[method],
                    label=METHOD_LABELS[method])
            ax4.fill(angles, values, alpha=0.15, color=METHOD_COLORS[method])
    
    ax4.set_ylim(0, 1)
    ax4.set_title('Overall Performance Profile', fontweight='bold', pad=20)
    ax4.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1), fontsize=7)
    ax4.grid(True)
    
    plt.suptitle('Comprehensive Method Comparison Across All Scenarios', 
                fontsize=14, fontweight='bold', y=0.995)
    
    return fig


def generate_all_figures(results_file, output_dir='figures'):
    """Generate all 8 figures and save to directory."""
    output_path = Path(output_dir)
    output_path.mkdir(exist_ok=True)
    
    print("="*70)
    print("GENERATING ACC 2026 FIGURES")
    print("="*70)
    
    figures = [
        ("Figure 1: Flow Field & Workspace", figure1_flow_field_workspace, []),
        ("Figure 2: Trajectories & Safety (T1)", figure2_trajectories_safety, [results_file]),
        ("Figure 3: Pruning Dynamics (T4)", figure3_pruning_dynamics, [results_file]),
        ("Figure 4: Lambda2 Evolution (T3)", figure4_lambda2_evolution, [results_file]),
        ("Figure 5: Consensus Convergence", figure5_consensus_convergence, []),
        ("Figure 6: Goal Convergence (T2)", figure6_goal_convergence, [results_file]),
        ("Figure 7: Control Effort", figure7_control_effort, []),
        ("Figure 8: Method Comparison", figure8_method_comparison, [results_file]),
    ]
    
    for i, (name, func, args) in enumerate(figures, 1):
        print(f"\nGenerating {name}...")
        try:
            fig = func(*args)
            filename = output_path / f"figure{i}_acc2026.png"
            fig.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"  [OK] Saved: {filename}")
            plt.close(fig)
        except Exception as e:
            print(f"  [ERROR] {e}")
            import traceback
            traceback.print_exc()
    
    print("\n" + "="*70)
    print(f"COMPLETE: All figures saved to {output_path.absolute()}")
    print("="*70)


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Generate ACC 2026 figures')
    parser.add_argument('--results', type=str, 
                       default='results/batch_results_20260202_102206.json',
                       help='Path to batch results JSON file')
    parser.add_argument('--output', type=str, default='figures',
                       help='Output directory for figures')
    
    args = parser.parse_args()
    
    generate_all_figures(args.results, args.output)
