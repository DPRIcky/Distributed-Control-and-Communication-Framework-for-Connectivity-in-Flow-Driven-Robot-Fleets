"""Interactive matplotlib animation for hybrid underwater simulation."""

from typing import Iterable, List

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
import numpy as np

from config.visualization_config import VisualizationConfig
from core.types import Edge


class HybridAnimator:
    """Interactive matplotlib animation for underwater robotics simulation."""

    def __init__(self, sim, config: VisualizationConfig = None) -> None:
        """Initialize animator.
        
        Args:
            sim: HybridUnderwaterSimulation instance
            config: Visualization configuration
        """
        self.sim = sim
        self.config = config or VisualizationConfig()

        self.fig, self.ax = plt.subplots(figsize=self.config.figsize)
        self.ax.set_xlim(0.0, sim.workspace[0])
        self.ax.set_ylim(0.0, sim.workspace[1])
        self.ax.set_title("Hybrid Consensus Pruning (Multi-Layer Robustness)", fontsize=12, fontweight='bold')
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")

        # Draw goal position
        self.goal_marker = self.ax.scatter(
            *sim.goal_position, s=400, c='gold', marker='*',
            edgecolors='darkorange', linewidths=2.0, zorder=10, label='Goal'
        )

        goal_circle = plt.Circle(
            sim.goal_position, 0.5, fill=False, color='gold',
            linestyle='--', alpha=0.5, linewidth=2
        )
        self.ax.add_patch(goal_circle)

        self.node_scatter = self.ax.scatter(
            [], [], s=self.config.node_size, c="#e74c3c", edgecolors="white", linewidths=1.0
        )
        self.labels: List[plt.Text] = []

        # Flow field visualization
        grid_x = np.linspace(0.0, sim.workspace[0], self.config.flow_grid_points)
        grid_y = np.linspace(0.0, sim.workspace[1], self.config.flow_grid_points)
        gx, gy = np.meshgrid(grid_x, grid_y)
        self.flow_grid_points = np.column_stack((gx.ravel(), gy.ravel()))
        initial_flow = np.array(
            [self.sim.flow.velocity(p, self.sim.time) for p in self.flow_grid_points]
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

        # Edge visualization
        self.active_lines = LineCollection(
            [], colors="#9b59b6", linewidths=self.config.active_linewidth, zorder=3
        )
        self.highlight_lines = LineCollection(
            [], colors="#e74c3c", linewidths=self.config.highlight_linewidth, zorder=5
        )
        self.candidate_lines = LineCollection(
            [], colors="#f39c12", linewidths=self.config.candidate_linewidth,
            linestyles='dashdot', zorder=4
        )
        
        # GHS edge visualization (MST discovery)
        self.ghs_mst_lines = LineCollection(
            [], colors="#27ae60", linewidths=3.5, zorder=6, alpha=0.9,
            label='MST Edges (GHS)'
        )
        self.ghs_testing_lines = LineCollection(
            [], colors="#f1c40f", linewidths=1.0, linestyles='dotted', 
            zorder=2, alpha=0.4, label='Testing Edges (GHS)'
        )

        self.ax.add_collection(self.active_lines)
        self.ax.add_collection(self.candidate_lines)
        self.ax.add_collection(self.highlight_lines)
        self.ax.add_collection(self.ghs_mst_lines)
        self.ax.add_collection(self.ghs_testing_lines)

        self.status = self.ax.text(
            0.02,
            0.98,
            "",
            transform=self.ax.transAxes,
            ha="left",
            va="top",
            fontsize=9,
            family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="white", alpha=0.9),
        )

        # Create custom legend
        legend_elements = [
            Line2D([0], [0], color='#27ae60', linewidth=3.5, label='MST Edges (GHS)'),
            Line2D([0], [0], color='#f1c40f', linewidth=1.0, linestyle='dotted', label='Testing (GHS)', alpha=0.5),
            Line2D([0], [0], color='#9b59b6', linewidth=2.5, label='Active Edges'),
            Line2D([0], [0], color='#f39c12', linewidth=3.5, linestyle='dashdot', label='Candidate Edge'),
            Line2D([0], [0], color='#e74c3c', linewidth=4.5, label='Recently Pruned'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#e74c3c',
                   markersize=8, label='Connected Robot'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#ff8c00',
                   markersize=8, label='Isolated Robot'),
        ]
        self.ax.legend(handles=legend_elements, loc='upper right', fontsize=7, framealpha=0.9)

    def _build_segments(self, edges: Iterable[Edge]) -> List[np.ndarray]:
        """Build line segments from edge list.
        
        Args:
            edges: Iterable of edges
            
        Returns:
            List of line segments
        """
        positions = [robot.position for robot in self.sim.robots]
        segments: List[np.ndarray] = []
        for a, b in edges:
            segments.append(np.vstack((positions[a], positions[b])))
        return segments

    def _update_plot(self, _frame: int):
        """Update plot for one animation frame.
        
        Args:
            _frame: Frame number (unused)
            
        Returns:
            Updated artists
        """
        report = self.sim.step()
        positions = np.array([robot.position for robot in self.sim.robots])

        # Color robots based on connectivity tree status
        colors = []
        for robot in self.sim.robots:
            if robot.parent is None and robot.robot_id != 0:
                # No parent AND not the root = disconnected from tree
                colors.append('#ff8c00')  # Dark orange for isolated robots
            else:
                # Has parent OR is root = in connectivity tree
                colors.append('#e74c3c')  # Red for connected robots

        self.node_scatter.set_offsets(positions)
        self.node_scatter.set_color(colors)

        # Update labels
        if not self.labels:
            for idx, (x, y) in enumerate(positions):
                self.labels.append(
                    self.ax.text(x, y + 0.18, str(idx), ha="center", fontsize=9)
                )
        else:
            for label, (x, y) in zip(self.labels, positions):
                label.set_position((x, y + 0.18))

        self.active_lines.set_segments(self._build_segments(self.sim.current_edges()))
        
        # Update GHS edge visualization
        if hasattr(self.sim, 'ghs') and self.sim.ghs is not None:
            # Build MST edges (BRANCH edges)
            mst_segments = []
            # Build testing edges (BASIC edges)
            testing_segments = []
            
            for robot in self.sim.ghs.robots:
                for neighbor in robot.neighbors:
                    if neighbor > robot.robot_id:  # Draw each edge once
                        edge_state = robot.edge_states.get(neighbor)
                        pos1 = positions[robot.robot_id]
                        pos2 = positions[neighbor]
                        segment = np.vstack((pos1, pos2))
                        
                        if edge_state and edge_state.value == 'BRANCH':
                            mst_segments.append(segment)
                        elif edge_state and edge_state.value == 'BASIC':
                            testing_segments.append(segment)
            
            self.ghs_mst_lines.set_segments(mst_segments)
            self.ghs_testing_lines.set_segments(testing_segments)
        else:
            self.ghs_mst_lines.set_segments([])
            self.ghs_testing_lines.set_segments([])

        # Highlight candidate edge if in consensus
        if self.sim.consensus_active and self.sim.consensus_candidate:
            self.candidate_lines.set_segments(self._build_segments([self.sim.consensus_candidate]))
        else:
            self.candidate_lines.set_segments([])

        # Highlight recently pruned edge
        if report.get("decision") == "prune":
            self.highlight_lines.set_segments(self._build_segments([report["removed_edge"]]))
        else:
            self.highlight_lines.set_segments([])

        # Calculate metrics
        avg_distance_to_goal = np.mean([
            np.linalg.norm(robot.position - self.sim.goal_position)
            for robot in self.sim.robots
        ])

        robots_at_goal = sum(
            1 for robot in self.sim.robots
            if np.linalg.norm(robot.position - self.sim.goal_position) < 0.5
        )

        # Count isolated robots (not in connectivity tree)
        isolated_count = sum(
            1 for robot in self.sim.robots
            if robot.parent is None and robot.robot_id != 0
        )
        
        # Get GHS statistics if available
        ghs_stats = None
        if hasattr(self.sim, 'ghs') and self.sim.ghs is not None:
            ghs_stats = self.sim.ghs.get_statistics()

        # Build status text
        status_lines = [
            f"Time: {self.sim.time:4.2f}s | Edges: {len(self.sim.current_edges())} | Pruned: {len(self.sim.pruned_edges)}",
            f"Isolated: {isolated_count} | Avg dist to goal: {avg_distance_to_goal:.2f}m",
            f"Robots at goal: {robots_at_goal}/{self.sim.num_robots}",
        ]
        
        # Add GHS status if enabled
        if ghs_stats:
            status_lines.append(f"GHS: MST={ghs_stats['mst_edges']}, Removed={ghs_stats['removed_edges']}, Fragments={ghs_stats['fragments']}")
            if ghs_stats['converged']:
                status_lines.append(f"GHS: ✓ Converged")
        
        status_lines.extend([
            "",
            "NOTE: Isolated = No path to root in connectivity tree",
            "      (May still have edges, but not connected to root)",
            "─" * 50,
        ])

        decision = report.get("decision")
        if decision == "topology_unstable":
            status_lines.append(f"[!] TOPOLOGY UNSTABLE")
            status_lines.append(f"  Broken: {report.get('edges_broken', [])}")
            status_lines.append(f"  Formed: {report.get('edges_formed', [])}")
        elif decision == "waiting_for_stability":
            status_lines.append(f"[~] Waiting for stability: {report['stable_rounds']}/{report['threshold']}")
        elif decision == "building_history":
            status_lines.append(f"[+] Building history: {report['history_size']}/{report['needed']}")
        elif decision == "no_persistent_redundant_edges":
            status_lines.append(f"[OK] No persistent redundant edges")
        elif decision == "evaluating":
            status_lines.append(f"[?] Evaluating: {report['candidate']}")
            status_lines.append(f"  Rounds: {report['consensus_round']}/{report['needed_rounds']}")
            status_lines.append(f"  Redundant: {report.get('is_redundant', 'checking...')}")
        elif decision == "consensus_failed":
            status_lines.append(f"[X] FAILED: {report['failed_candidate']} NOT redundant")
            status_lines.append(f"  Searching for other candidates...")
        elif decision == "prune":
            status_lines.append(f"[PRUNED] {report['removed_edge']}")
        elif decision == "abort_topology_changed":
            status_lines.append(f"[X] ABORTED: Topology changed during consensus")

        self.status.set_text("\n".join(status_lines))

        # Update flow field
        flow_vectors = np.array(
            [self.sim.flow.velocity(p, self.sim.time) for p in self.flow_grid_points]
        )
        self.quiver.set_UVC(flow_vectors[:, 0], flow_vectors[:, 1])

        return (
            self.node_scatter,
            self.active_lines,
            self.candidate_lines,
            self.highlight_lines,
            self.ghs_mst_lines,
            self.ghs_testing_lines,
            self.status,
            *self.labels,
        )

    def _init_plot(self):
        """Initialize plot.
        
        Returns:
            Initial artists
        """
        positions = np.array([robot.position for robot in self.sim.robots])
        self.node_scatter.set_offsets(positions)
        self.labels = []
        for idx, (x, y) in enumerate(positions):
            self.labels.append(
                self.ax.text(x, y + 0.18, str(idx), ha="center", fontsize=9)
            )
        self.active_lines.set_segments(self._build_segments(self.sim.current_edges()))
        self.candidate_lines.set_segments([])
        self.highlight_lines.set_segments([])
        self.ghs_mst_lines.set_segments([])
        self.ghs_testing_lines.set_segments([])

        self.status.set_text("Initializing hybrid consensus + GHS...")

        flow_vectors = np.array(
            [self.sim.flow.velocity(p, self.sim.time) for p in self.flow_grid_points]
        )
        self.quiver.set_UVC(flow_vectors[:, 0], flow_vectors[:, 1])
        return (
            self.node_scatter,
            self.active_lines,
            self.candidate_lines,
            self.highlight_lines,
            self.ghs_mst_lines,
            self.ghs_testing_lines,
            self.status,
            *self.labels,
        )

    def animate(self) -> FuncAnimation:
        """Start animation loop.
        
        Returns:
            FuncAnimation object
        """
        animation = FuncAnimation(
            self.fig,
            self._update_plot,
            interval=self.config.interval_ms,
            blit=False,
            repeat=False,
            cache_frame_data=False,
            init_func=self._init_plot,
        )
        self._animation = animation
        return animation
