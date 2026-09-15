"""Visualize robot trajectories with initial and final network topology.

Generates a figure showing:
1. Initial fully-connected network (dashed lines)
2. Final pruned network (solid lines)
3. Robot start/end positions (red circles)
4. Goal position (yellow star)
5. Trajectories during simulation
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyArrowPatch
import networkx as nx
from config import SimulationConfig, ControlConfig, VisualizationConfig
from critical_edge_pruning_simulation import CriticalEdgePruningSimulation


def find_most_central_node(graph):
    """Find the node that minimizes maximum distance to any other node.
    
    This gives a more balanced spanning tree than using a peripheral node.
    """
    if len(graph) == 0:
        return 0
    
    eccentricities = {}
    for node in graph.nodes():
        # Compute shortest paths from this node
        lengths = nx.single_source_shortest_path_length(graph, node)
        max_dist = max(lengths.values())
        eccentricities[node] = max_dist
    
    # Return node with minimum eccentricity (most central)
    return min(eccentricities, key=eccentricities.get)


def simulate_with_trajectory_recording(
    num_robots: int = 10,
    simulation_time: float = 10.0,
    communication_radius: float = 3.5,
    pruning_start_time: float = 2.0,
) -> dict:
    """Run simulation and record trajectory data.
    
    Args:
        num_robots: Number of robots
        simulation_time: Total simulation time in seconds
        communication_radius: Communication range
        pruning_start_time: When to start pruning
        
    Returns:
        Dictionary with trajectory and network data
    """
    
    # Create configs
    sim_config = SimulationConfig(
        num_robots=num_robots,
        communication_radius=communication_radius,
        dt=0.05,
        workspace_size=(12.0, 12.0),
        seed=42,
        verbose=False
    )
    
    control_config = ControlConfig(
        safety_distance=1.2,
        clf_gain=0.8,
        cbf_safety_gain=4.0,
        cbf_connectivity_gain=0.5,
        max_control_force=1.0
    )
    
    # Create simulation
    sim = CriticalEdgePruningSimulation(
        sim_config,
        control_config,
        enable_pruning=True,
        pruning_start_time=pruning_start_time,
        algorithm_interval=0.5,
        root_node=0  # Will use most central node in algorithm
    )
    sim.verbose = False  # Disable verbose output to prevent unicode encoding errors
    
    # Record data
    data = {
        'initial_positions': np.array([r.position.copy() for r in sim.robots]),
        'initial_edges': set(sim.current_edges()),
        'trajectories': {i: [sim.robots[i].position.copy()] for i in range(num_robots)},
        'time_steps': [0.0],
        'goal': np.array([sim_config.workspace_size[0] * 0.7, sim_config.workspace_size[1] * 0.5]),  # Approximate goal
    }
    
    # Find and use most central node as root for balanced spanning tree
    if data['initial_edges']:
        G_initial = nx.Graph()
        G_initial.add_nodes_from(range(num_robots))
        G_initial.add_edges_from(data['initial_edges'])
        central_root = find_most_central_node(G_initial)
        sim.root_node = central_root
        print(f"Using central node {central_root} as root for balanced spanning tree")
    
    # Run simulation
    max_steps = int(simulation_time / sim.dt)
    step = 0
    
    print(f"Running simulation for {simulation_time}s ({max_steps} steps)...")
    while step < max_steps:
        sim.step()
        step += 1
        
        # Record trajectories
        for i in range(num_robots):
            data['trajectories'][i].append(sim.robots[i].position.copy())
        
        data['time_steps'].append(sim.time)
        
        if step % 100 == 0:
            print(f"  Step {step}/{max_steps} (t={sim.time:.2f}s, edges={sim.total_edges})")
    
    # Record final state
    data['final_positions'] = np.array([r.position.copy() for r in sim.robots])
    
    # Get the optimized edges using the same method as plot_phases_2x2()
    if sim.distributed_algo is not None:
        try:
            # Build spanning tree by traversing parent-child relationships (from plot_results.py method)
            tree_edges_1based = set()
            for node in sim.distributed_algo.nodes:
                parent = sim.distributed_algo.node_state[node].parent
                if parent is not None and parent != node:
                    tree_edges_1based.add(tuple(sorted([node, parent])))
            
            # Get final pruned graph which includes tree edges + robustness edge
            pruned_graph = sim.distributed_algo.get_pruned_graph()
            
            # Convert all edges to 0-based indexing
            data['final_edges'] = set()
            for i, j in pruned_graph.edges():
                # Convert from 1-based (algorithm) to 0-based (visualization)
                i_0based = i - 1
                j_0based = j - 1
                data['final_edges'].add((min(i_0based, j_0based), max(i_0based, j_0based)))
                
        except Exception as e:
            # Fallback if anything fails
            data['final_edges'] = data['initial_edges']
    else:
        data['final_edges'] = data['initial_edges']
    data['initial_edge_count'] = len(data['initial_edges'])
    data['final_edge_count'] = len(data['final_edges'])
    
    print(f"Simulation complete: {data['initial_edge_count']} -> {data['final_edge_count']} edges")
    print(f"Final edges (all): {sorted(list(data['final_edges']))}")
    
    return data


def plot_trajectories_with_networks(data: dict, figsize: tuple = (12, 10)):
    """Plot robot trajectories with initial and final network overlays.
    
    Args:
        data: Dictionary from simulate_with_trajectory_recording()
        figsize: Figure size (width, height)
    """
    
    fig, ax = plt.subplots(figsize=figsize)
    
    # Plot initial network (dashed gray)
    for i, j in data['initial_edges']:
        pos_i = data['initial_positions'][i]
        pos_j = data['initial_positions'][j]
        ax.plot(
            [pos_i[0], pos_j[0]],
            [pos_i[1], pos_j[1]],
            'gray',
            linestyle='--',
            linewidth=1.0,
            alpha=0.4,
            zorder=1
        )
    
    # Plot final network (solid blue/purple)
    for i, j in data['final_edges']:
        pos_i = data['final_positions'][i]
        pos_j = data['final_positions'][j]
        ax.plot(
            [pos_i[0], pos_j[0]],
            [pos_i[1], pos_j[1]],
            'mediumpurple',
            linestyle='-',
            linewidth=2.0,
            alpha=0.7,
            zorder=2
        )
    
    # Plot trajectories
    colors = plt.cm.Reds(np.linspace(0.5, 1.0, len(data['trajectories'])))
    for robot_id, trajectory in data['trajectories'].items():
        traj_array = np.array(trajectory)
        ax.plot(
            traj_array[:, 0],
            traj_array[:, 1],
            color=colors[robot_id],
            linewidth=1.5,
            alpha=0.8,
            zorder=3
        )
    
    # Plot start positions (red circles)
    start_pos = data['initial_positions']
    ax.scatter(
        start_pos[:, 0],
        start_pos[:, 1],
        s=150,
        c='red',
        edgecolors='darkred',
        linewidth=2,
        marker='o',
        label='Start',
        zorder=5
    )
    
    # Plot end positions (red circles with black outline)
    end_pos = data['final_positions']
    ax.scatter(
        end_pos[:, 0],
        end_pos[:, 1],
        s=150,
        c='red',
        edgecolors='darkred',
        linewidth=2,
        marker='o',
        label='End',
        zorder=5
    )
    
    # Plot goal (yellow star)
    goal = data['goal']
    ax.scatter(
        goal[0],
        goal[1],
        s=400,
        c='gold',
        edgecolors='orange',
        linewidth=2,
        marker='*',
        label='Goal',
        zorder=6
    )
    
    # Labels and legend
    ax.set_xlabel('X Position (m)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y Position (m)', fontsize=12, fontweight='bold')
    ax.set_title('Robot Trajectories with Network Topology Evolution', fontsize=14, fontweight='bold')
    
    # Create custom legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='gray', linestyle='--', linewidth=2, alpha=0.4, label='Initial (Fully Connected)'),
        Line2D([0], [0], color='mediumpurple', linestyle='-', linewidth=2, alpha=0.7, label='Final (Near-MST)'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=10,
               markeredgecolor='darkred', markeredgewidth=2, label='Start'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=10,
               markeredgecolor='darkred', markeredgewidth=2, label='End'),
        Line2D([0], [0], marker='*', color='w', markerfacecolor='gold', markersize=15,
               markeredgecolor='orange', markeredgewidth=2, label='Goal'),
    ]
    ax.legend(handles=legend_elements, loc='upper left', fontsize=11, framealpha=0.95)
    
    # Grid and layout
    ax.grid(True, alpha=0.3, linestyle=':', linewidth=0.5)
    ax.set_aspect('equal')
    
    # Add statistics box
    stats_text = (
        f"Initial Edges: {data['initial_edge_count']}\n"
        f"Final Edges: {data['final_edge_count']}\n"
        f"Pruning: {(1 - data['final_edge_count']/data['initial_edge_count'])*100:.1f}%\n"
        f"Robots: {len(data['trajectories'])}"
    )
    ax.text(
        0.98, 0.02,
        stats_text,
        transform=ax.transAxes,
        fontsize=10,
        verticalalignment='bottom',
        horizontalalignment='right',
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
    )
    
    plt.tight_layout()
    return fig, ax


if __name__ == "__main__":
    print("\n" + "="*70)
    print("Robot Trajectories with Network Topology Evolution")
    print("="*70)
    
    # Run simulation and record data
    data = simulate_with_trajectory_recording(
        num_robots=10,
        simulation_time=10.0,
        communication_radius=3.5,
        pruning_start_time=2.0
    )
    
    # Plot results
    print("\nGenerating visualization...")
    fig, ax = plot_trajectories_with_networks(data)
    
    # Save figure
    output_path = Path(__file__).parent.parent / "Figures" / "robot_trajectories_with_pruning.png"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Figure saved to: {output_path}")
    
    # Display
    plt.show()
    
    print("\n" + "="*70)
    print("Visualization Complete!")
    print("="*70 + "\n")
