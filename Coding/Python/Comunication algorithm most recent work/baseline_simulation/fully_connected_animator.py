"""Fully Connected Animator - Visualization Module.

Standard animator for baseline simulation with dual-panel layout:
- Left: Main simulation with flow field
- Right: Metrics (edge count + distance to goal)

This animator matches the style of concurrent_pruning simulations and can be
easily extended for custom visualizations.
"""

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
import numpy as np

from config import VisualizationConfig


class FullyConnectedAnimator:
    """Standard animator for fully connected simulation.
    
    Provides dual-panel visualization matching concurrent_pruning style:
    - Left panel: Robot simulation with flow field
    - Right panel: Metrics tracking
    
    Attributes:
        sim: Simulation instance
        vis_config: Visualization configuration
        fig: Matplotlib figure
        ax: Main simulation axes
        ax_metrics: Metrics axes
        time_history: Time history buffer
        edge_count_history: Edge count history
        distance_history: Distance to goal history
    """
    
    def __init__(self, simulation, vis_config: VisualizationConfig):
        """Initialize animator.
        
        Args:
            simulation: FullyConnectedSimulation instance (or subclass)
            vis_config: Visualization configuration
        """
        self.sim = simulation
        self.vis_config = vis_config
        
        # History tracking
        self.time_history = []
        self.edge_count_history = []
        self.distance_history = []
        self.max_time_window = 100  # seconds
        
        # Create figure with subplots
        self.fig = plt.figure(figsize=(16, 7))
        
        # Main simulation plot (left, 60% width)
        self.ax = self.fig.add_subplot(121)
        
        # Metrics plot (right, 40% width)
        self.ax_metrics = self.fig.add_subplot(122)
        
        # Setup plots
        self._setup_main_plot()
        self._setup_metrics_plot()
        
    def _setup_main_plot(self):
        """Setup main simulation visualization."""
        sim = self.sim
        
        self.ax.set_xlim(0.0, sim.workspace[0])
        self.ax.set_ylim(0.0, sim.workspace[1])
        self.ax.set_title("Fully Connected Graph Simulation (CBF+CLF)", 
                         fontsize=12, fontweight='bold')
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        
        # Draw goal
        self.goal_marker = self.ax.scatter(
            *sim.goal_position, s=400, c='gold', marker='*',
            edgecolors='darkorange', linewidths=2.0, zorder=10, label='Goal'
        )
        
        goal_circle = plt.Circle(
            sim.goal_position, 0.5, fill=False, color='gold',
            linestyle='--', alpha=0.5, linewidth=2
        )
        self.ax.add_patch(goal_circle)
        
        # Robot nodes
        self.node_scatter = self.ax.scatter(
            [], [], s=self.vis_config.node_size, c="#e74c3c", 
            edgecolors="white", linewidths=1.0, zorder=6
        )
        self.labels = []
        
        # Flow field
        grid_x = np.linspace(0.0, sim.workspace[0], self.vis_config.flow_grid_points)
        grid_y = np.linspace(0.0, sim.workspace[1], self.vis_config.flow_grid_points)
        gx, gy = np.meshgrid(grid_x, grid_y)
        self.flow_grid_points = np.column_stack((gx.ravel(), gy.ravel()))
        initial_flow = np.array(
            [sim.flow.velocity(p, sim.time) for p in self.flow_grid_points]
        )
        self.quiver = self.ax.quiver(
            self.flow_grid_points[:, 0],
            self.flow_grid_points[:, 1],
            initial_flow[:, 0],
            initial_flow[:, 1],
            color="#B0C4DE",
            alpha=self.vis_config.flow_alpha,
            width=self.vis_config.flow_width,
            scale=self.vis_config.flow_scale,
            zorder=0,
        )
        
        # Edge collection (all edges in purple - active)
        self.active_lines = LineCollection(
            [], colors="#9b59b6", linewidths=self.vis_config.active_linewidth, zorder=3
        )
        self.ax.add_collection(self.active_lines)
        
        # Status text
        self.status = self.ax.text(
            0.02, 0.98, "", transform=self.ax.transAxes,
            ha="left", va="top", fontsize=9, family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="white", alpha=0.9),
        )
        
        # Info text
        self.info_text = self.ax.text(
            0.02, 0.02, "", transform=self.ax.transAxes,
            ha="left", va="bottom", fontsize=8, family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="lightyellow", alpha=0.95),
        )
        
        # Legend
        legend_elements = [
            Line2D([0], [0], color='#9b59b6', linewidth=2.5, label='Active Edges'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#e74c3c',
                   markersize=8, label='Robot'),
        ]
        self.ax.legend(handles=legend_elements, loc='upper right', fontsize=7, framealpha=0.9)
    
    def _setup_metrics_plot(self):
        """Setup metrics visualization."""
        self.ax_metrics.set_title("Metrics (Fully Connected Baseline)", fontsize=10, fontweight='bold')
        self.ax_metrics.set_xlabel("Time (s)")
        self.ax_metrics.set_ylabel("Edge Count", color='black')
        self.ax_metrics.grid(True, alpha=0.3)
        
        # Edge count line
        self.edge_count_line, = self.ax_metrics.plot(
            [], [], 'k-', linewidth=2.5, alpha=0.8, label='Edges'
        )
        
        # Average distance to goal (secondary axis)
        self.ax_distance = self.ax_metrics.twinx()
        self.ax_distance.set_ylabel("Avg Distance to Goal (m)", color='blue')
        self.distance_line, = self.ax_distance.plot(
            [], [], 'b-', linewidth=2.0, alpha=0.6, label='Avg Dist'
        )
        
        self.ax_metrics.set_xlim(0, 10)
        self.ax_metrics.set_ylim(0, 50)
        self.ax_distance.set_ylim(0, 20)
        
        # Legend
        lines1, labels1 = self.ax_metrics.get_legend_handles_labels()
        lines2, labels2 = self.ax_distance.get_legend_handles_labels()
        self.ax_metrics.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=8)
    
    def init(self):
        """Initialize animation."""
        self.node_scatter.set_offsets(np.empty((0, 2)))
        self.goal_marker.set_offsets([self.sim.goal_position])
        self.active_lines.set_segments([])
        self.status.set_text('')
        self.info_text.set_text('')
        return []
    
    def update(self, frame):
        """Update animation frame.
        
        Override this method in subclasses to add custom visualization behavior.
        
        Args:
            frame: Frame number
            
        Returns:
            list: List of artists to redraw
        """
        # Step simulation
        self.sim.step()
        state = self.sim.get_state()
        
        # Update time history
        current_time = self.sim.time
        self.time_history.append(current_time)
        self.edge_count_history.append(state['num_edges'])
        
        # Calculate average distance to goal
        distances_to_goal = np.linalg.norm(state['positions'] - state['goal'], axis=1)
        avg_dist = np.mean(distances_to_goal)
        self.distance_history.append(avg_dist)
        
        # Limit history
        if len(self.time_history) > self.max_time_window * 10:
            self.time_history.pop(0)
            self.edge_count_history.pop(0)
            self.distance_history.pop(0)
        
        # Update metrics plot
        self.edge_count_line.set_data(self.time_history, self.edge_count_history)
        self.distance_line.set_data(self.time_history, self.distance_history)
        
        # Adjust x-axis
        if self.time_history:
            max_time = max(self.time_history)
            self.ax_metrics.set_xlim(max(0, max_time - 30), max_time + 3)
        
        # Update main plot - robot positions
        self.node_scatter.set_offsets(state['positions'])
        
        # Update labels
        for label in self.labels:
            label.remove()
        self.labels = []
        for i, pos in enumerate(state['positions']):
            label = self.ax.text(pos[0], pos[1] + 0.3, str(i), ha='center', 
                               fontsize=8, color='white', weight='bold', zorder=7)
            self.labels.append(label)
        
        # Update edges
        edge_segments = []
        for i, j in state['edges']:
            edge_segments.append([state['positions'][i], state['positions'][j]])
        self.active_lines.set_segments(edge_segments)
        
        # Update flow field
        flow_velocities = np.array(
            [self.sim.flow.velocity(p, self.sim.time) for p in self.flow_grid_points]
        )
        self.quiver.set_UVC(flow_velocities[:, 0], flow_velocities[:, 1])
        
        # Update status text
        max_dist = np.max(distances_to_goal)
        status_lines = [
            f"Time: {state['time']:6.2f}s",
            f"Robots: {self.sim.num_robots}",
            f"Edges: {state['num_edges']}",
            f"Mode: Fully Connected",
        ]
        self.status.set_text("\n".join(status_lines))
        
        # Update info text
        info_lines = self._get_info_text(state, avg_dist, max_dist)
        self.info_text.set_text("\n".join(info_lines))
        
        return []
    
    def _get_info_text(self, state, avg_dist, max_dist):
        """Get info text lines.
        
        Override this method in subclasses to customize info display.
        
        Args:
            state: Current simulation state
            avg_dist: Average distance to goal
            max_dist: Maximum distance to goal
            
        Returns:
            list: List of info text lines
        """
        return [
            f"BASELINE: Fully Connected",
            f"No Pruning",
            f"",
            f"Avg Dist to Goal: {avg_dist:.2f}m",
            f"Max Dist to Goal: {max_dist:.2f}m",
        ]
    
    def animate(self):
        """Create and return animation.
        
        Returns:
            FuncAnimation: Matplotlib animation object
        """
        anim = FuncAnimation(
            self.fig,
            self.update,
            init_func=self.init,
            interval=self.vis_config.interval_ms,
            blit=False,
            cache_frame_data=False
        )
        return anim
