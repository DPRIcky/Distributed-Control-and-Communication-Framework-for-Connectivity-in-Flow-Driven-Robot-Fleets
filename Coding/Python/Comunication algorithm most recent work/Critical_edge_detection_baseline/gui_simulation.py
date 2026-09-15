"""GUI Simulation for Critical Edge Detection Baseline.

This simulation demonstrates the distributed critical edge detection algorithm
for dynamically pruning communication networks while preserving connectivity.

Features:
  - Real-time visualization of robot swarm with communication topology
  - Edge classification: Critical (red), MST (green), Other (gray)
  - MST convergence metrics and pruning progress
  - Distributed algorithm performance tracking
"""

import sys
from pathlib import Path
parent_dir = Path(__file__).parent.parent
sys.path.insert(0, str(parent_dir))

import io
import contextlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
import numpy as np
from typing import Tuple

from config import SimulationConfig, ControlConfig, VisualizationConfig

# Suppress import warnings
with contextlib.redirect_stdout(io.StringIO()):
    from Critical_edge_detection_baseline.simple_simulation import CriticalEdgeSimulation

Edge = Tuple[int, int]


class CriticalEdgeAnimator:
    """Animator for critical edge detection baseline simulation.
    
    Visualizes:
      - Robot positions and movement
      - Communication topology with edge classification
      - MST convergence metrics
      - Pruning progress and statistics
    """
    
    def __init__(self, sim, config=None):
        """Initialize animator with critical edge tracking.
        
        Args:
            sim: CriticalEdgeSimulation instance
            config: VisualizationConfig
        """
        self.sim = sim
        self.config = config or VisualizationConfig()
        
        # History tracking
        self.time_history = []
        self.edge_count_history = []
        self.pruned_count_history = []
        self.mst_updates_history = []
        self.critical_edges_history = []
        self.max_time_window = 100  # seconds
        
        # CBF monitoring
        self.max_critical_distance = 0.0
        self.cbf_violations = 0
        self.constraint_margin = sim.communication_radius * 0.95
        
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
        self.ax.set_title("Distributed Critical Edge Detection", fontsize=12, fontweight='bold')
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
            [], [], s=self.config.node_size, c="#1abc9c", 
            edgecolors="white", linewidths=1.0, zorder=6
        )
        self.labels = []
        
        # Flow field
        grid_x = np.linspace(0.0, sim.workspace[0], self.config.flow_grid_points)
        grid_y = np.linspace(0.0, sim.workspace[1], self.config.flow_grid_points)
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
            alpha=self.config.flow_alpha,
            width=self.config.flow_width,
            scale=self.config.flow_scale,
            zorder=0,
        )
        
        # Edge collections
        self.critical_lines = LineCollection(
            [], colors="#e74c3c", linewidths=1.2, zorder=5
        )
        self.other_lines = LineCollection(
            [], colors="#95a5a6", linewidths=0.8, alpha=0.2, zorder=2
        )
        
        self.ax.add_collection(self.critical_lines)
        self.ax.add_collection(self.other_lines)
        
        # Status text
        self.status = self.ax.text(
            0.02, 0.98, "", transform=self.ax.transAxes,
            ha="left", va="top", fontsize=9, family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="white", alpha=0.9),
        )
        
        # Algorithm info text
        self.algo_status = self.ax.text(
            0.02, 0.02, "", transform=self.ax.transAxes,
            ha="left", va="bottom", fontsize=8, family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="lightyellow", alpha=0.95),
        )
        
        # Legend
        legend_elements = [
            Line2D([0], [0], color='#e74c3c', linewidth=1.2, label='Critical Edges'),
            Line2D([0], [0], color='#95a5a6', linewidth=1.0, label='Other Edges'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#1abc9c',
                   markersize=8, label='Robot'),
        ]
        self.ax.legend(handles=legend_elements, loc='upper right', fontsize=7, framealpha=0.9)
    
    def _setup_metrics_plot(self):
        """Setup convergence metrics visualization."""
        self.ax_metrics.set_title("MST Convergence & Pruning Metrics", fontsize=10, fontweight='bold')
        self.ax_metrics.set_xlabel("Time (s)")
        self.ax_metrics.set_ylabel("Edge Count")
        self.ax_metrics.grid(True, alpha=0.3)
        
        # Edge count line
        self.edge_count_line, = self.ax_metrics.plot(
            [], [], 'b-', linewidth=2.5, alpha=0.8, label='Current Edges'
        )
        
        # Pruned count line
        self.pruned_line, = self.ax_metrics.plot(
            [], [], 'r-', linewidth=2.0, alpha=0.7, label='Total Pruned'
        )
        
        # Critical edges on secondary axis
        self.ax_critical = self.ax_metrics.twinx()
        self.ax_critical.set_ylabel("Critical Edges & Updates", color='#e74c3c')
        
        self.critical_line, = self.ax_critical.plot(
            [], [], 'o-', color='#e74c3c', linewidth=1.5, alpha=0.6, 
            markersize=4, label='Critical Edges'
        )
        
        self.updates_line, = self.ax_critical.plot(
            [], [], 's-', color='#9b59b6', linewidth=1.5, alpha=0.6, 
            markersize=4, label='MST Updates'
        )
        
        self.ax_metrics.set_xlim(0, 10)
        self.ax_metrics.set_ylim(0, 50)
        self.ax_critical.set_ylim(0, 100)
        
        # Combined legend
        lines1, labels1 = self.ax_metrics.get_legend_handles_labels()
        lines2, labels2 = self.ax_critical.get_legend_handles_labels()
        self.ax_metrics.legend(lines1 + lines2, labels1 + labels2, 
                              loc='upper left', fontsize=7, ncol=2)
    
    def _update_plot(self, frame):
        """Update visualization for one frame."""
        # Get edge lengths before step to check CBF constraints
        edge_lengths = self.sim._get_edge_lengths()
        
        # Check critical edge distances for CBF violations
        if self.sim.baseline.critical_edges and edge_lengths:
            for i, j in self.sim.baseline.critical_edges:
                edge_key = (min(i, j), max(i, j))
                if edge_key in edge_lengths:
                    dist = edge_lengths[edge_key]
                    self.max_critical_distance = max(self.max_critical_distance, dist)
                    if dist > self.constraint_margin:
                        self.cbf_violations += 1
        
        # Step simulation
        self.sim.step()
        
        # Update time history
        current_time = self.sim.time
        self.time_history.append(current_time)
        self.edge_count_history.append(self.sim.total_edges)
        self.pruned_count_history.append(self.sim.edges_pruned_count)
        
        # Get algorithm statistics
        stats = self.sim.baseline.get_statistics()
        self.mst_updates_history.append(stats['mst_updates'])
        self.critical_edges_history.append(stats['critical_edges_detected'])
        
        # Limit history
        if len(self.time_history) > self.max_time_window * 10:
            self.time_history.pop(0)
            self.edge_count_history.pop(0)
            self.pruned_count_history.pop(0)
            self.mst_updates_history.pop(0)
            self.critical_edges_history.pop(0)
        
        # Update metrics plot  
        self.edge_count_line.set_data(self.time_history, self.edge_count_history)
        self.pruned_line.set_data(self.time_history, self.pruned_count_history)
        self.critical_line.set_data(self.time_history, self.critical_edges_history)
        self.updates_line.set_data(self.time_history, self.mst_updates_history)
        # Adjust x-axis
        if self.time_history:
            max_time = max(self.time_history)
            self.ax_metrics.set_xlim(max(0, max_time - 30), max_time + 3)
        
        # Update main plot
        positions = np.array([r.position for r in self.sim.robots])
        self.node_scatter.set_offsets(positions)
        
        # Update labels
        for label in self.labels:
            label.remove()
        self.labels = []
        for i, pos in enumerate(positions):
            label = self.ax.text(pos[0], pos[1] + 0.3, str(i), ha='center', 
                               fontsize=8, color='white', weight='bold', zorder=7)
            self.labels.append(label)
        
        # Get edge information
        edge_lengths = self.sim._get_edge_lengths()
        critical_edges = self.sim.baseline.critical_edges
        
        # If we have exactly n-1 edges (MST size), all remaining edges are critical by definition
        if len(edge_lengths) == self.sim.num_robots - 1:
            critical_edges = set(edge_lengths.keys())
        
        # Classify edges - only highlight critical edges
        critical_segments = []
        other_segments = []
        
        for (i, j), length in edge_lengths.items():
            pos_i = positions[i]
            pos_j = positions[j]
            segment = (pos_i, pos_j)
            
            is_critical = ((i, j) in critical_edges or (j, i) in critical_edges)
            
            if is_critical:
                critical_segments.append(segment)
            else:
                other_segments.append(segment)
        
        self.critical_lines.set_segments(critical_segments)
        self.other_lines.set_segments(other_segments)
        
        # Update flow field
        flow_velocities = np.array(
            [self.sim.flow.velocity(p, self.sim.time) for p in self.flow_grid_points]
        )
        self.quiver.set_UVC(flow_velocities[:, 0], flow_velocities[:, 1])
        
        # Update status text
        status_lines = [
            f"Time: {self.sim.time:6.2f}s",
            f"Robots: {self.sim.num_robots}",
            f"Current Edges: {self.sim.total_edges}",
            f"Total Pruned: {self.sim.edges_pruned_count}",
            f"Frame: {frame}",
        ]
        self.status.set_text("\n".join(status_lines))
        
        # Update algorithm status
        cbf_status = "[OK] ACTIVE" if len(self.sim.baseline.critical_edges) > 0 else "[--] Waiting"
        violation_indicator = "[!]" if self.cbf_violations > 0 else "[OK]"
        
        algo_lines = [
            f"CRITICAL EDGE DETECTION ALGORITHM",
            f"MST Updates: {stats['mst_updates']}  |  DFS Rounds: {stats['total_rounds']}",
            f"Critical Edges: {stats['critical_edges_detected']}  |  Avg Rounds/Update: {stats['avg_rounds_per_update']:.2f}",
            f"Edge Reduction: {100 * (1 - self.sim.total_edges / max(1, self.sim.max_edges)):.1f}%",
            f"",
            f"CBF CONSTRAINT SYSTEM (Comm Radius: {self.sim.communication_radius:.1f}m)",
            f"Status: {cbf_status}  |  Constraint Margin: {self.constraint_margin:.3f}m",
            f"Max Critical Distance: {self.max_critical_distance:.4f}m  {violation_indicator} Violations: {self.cbf_violations}",
        ]
        self.algo_status.set_text("\n".join(algo_lines))
        
        return []
    
    def animate(self):
        """Create and return animation."""
        anim = FuncAnimation(
            self.fig,
            self._update_plot,
            interval=self.config.interval_ms,
            blit=False,
            cache_frame_data=False
        )
        return anim


def main():
    """Run GUI simulation with critical edge detection."""
    import sys
    
    print("\n" + "="*70)
    print("CRITICAL EDGE DETECTION BASELINE - GUI SIMULATION")
    print("="*70)
    
    # ========== CONFIGURATION ==========
    # Check for command line arguments
    if len(sys.argv) > 1:
        try:
            COMM_RADIUS = float(sys.argv[1])
        except ValueError:
            COMM_RADIUS = 2.0
    else:
        COMM_RADIUS = 2.0  # Default
    
    NUM_ROBOTS = 20
    UPDATE_FREQ = 10
    PREFER_CRITICAL = True
    # ===================================
    
    sim_config = SimulationConfig(
        num_robots=NUM_ROBOTS,
        communication_radius=COMM_RADIUS,
        dt=0.05,
        workspace_size=(10.0, 10.0),
        verbose=False,
        seed=42
    )
    control_config = ControlConfig()
    vis_config = VisualizationConfig(interval_ms=100)
    
    print(f"\nConfiguration:")
    print(f"  Robots: {NUM_ROBOTS}")
    print(f"  Communication Radius: {COMM_RADIUS}m")
    print(f"  MST Update Frequency: every {UPDATE_FREQ} steps")
    print(f"  Prefer Critical Edges: {PREFER_CRITICAL}")
    print(f"\nCBF Enhancement Features:")
    print(f"  [*] Distributed MST construction")
    print(f"  [*] Critical edge (bridge) detection")
    print(f"  [*] CBF constrains critical edges to stay within comm radius")
    print(f"  [*] Automatic communication pruning")
    print(f"  [*] O(n) convergence time")
    print(f"  [*] Real-time visualization")
    print(f"\nVisualization:")
    print(f"  Left: Robot simulation with edge classification")
    print(f"  Right: Convergence metrics and pruning progress")
    print(f"\nUsage:")
    print(f"  python gui_simulation.py [comm_radius]")
    print(f"  Examples:")
    print(f"    python gui_simulation.py 1.0    # Test with 1m radius (CBF constraints visible)")
    print(f"    python gui_simulation.py 2.0    # Test with 2m radius (optimal)")
    print(f"    python gui_simulation.py 3.0    # Test with 3m radius (stable)")
    print("="*70 + "\n")
    
    # Create simulation and animator
    sim = CriticalEdgeSimulation(
        sim_config=sim_config,
        control_config=control_config,
        update_frequency=UPDATE_FREQ,
        prefer_critical=PREFER_CRITICAL,
        verbose=False
    )
    
    animator = CriticalEdgeAnimator(sim, vis_config)
    anim = animator.animate()
    
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
