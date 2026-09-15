"""GUI Simulation for Concurrent Consensus + Pruning with DISTRIBUTED COORDINATION.

This simulation demonstrates the novel concurrent approach where edge pruning
happens DURING consensus convergence (not after) using distributed coordination:

Distributed Pruning Protocol (3 Phases):
  1. PROPOSAL: Each robot independently evaluates its incident edges
  2. NEGOTIATION: Both endpoint robots must agree to prune an edge
  3. CONFLICT RESOLUTION: Deterministic tie-breaking when multiple edges approved

Features:
  - Real-time visualization of robot swarm with communication topology
  - Convergence metrics (Lyapunov or Max Disagreement) per robot
  - Edge pruning events highlighted dynamically
  - Adaptive threshold phases: Conservative → Moderate → Aggressive
  - Fully distributed decision-making (no central coordinator)
"""

import sys
from pathlib import Path
parent_dir = Path(__file__).parent.parent
sys.path.insert(0, str(parent_dir))

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
import numpy as np
from typing import Set, Tuple

from config import SimulationConfig, ControlConfig, ConsensusConfig, VisualizationConfig
from simulation import HybridUnderwaterSimulation
from concurrent_pruning.concurrent_pruning_manager import ConcurrentPruningManager

Edge = Tuple[int, int]


class ConcurrentPruningSimulation(HybridUnderwaterSimulation):
    """Simulation using concurrent consensus + pruning with DISTRIBUTED COORDINATION.
    
    Implements three-phase distributed protocol:
      Phase 1: Each robot proposes edges to prune (local evaluation)
      Phase 2: Both endpoint robots must agree (bilateral negotiation)
      Phase 3: Deterministic tie-breaking if conflicts (no coordinator needed)
    """
    
    def __init__(self, sim_config, control_config, consensus_config, pruning_mode='lyapunov'):
        """Initialize with distributed concurrent pruning manager.
        
        Args:
            sim_config: Simulation configuration
            control_config: Control configuration
            consensus_config: Consensus configuration
            pruning_mode: 'lyapunov', 'max_disagreement', or 'hybrid'
        """  
        super().__init__(sim_config, control_config, consensus_config)
        
        # Replace pruning manager with concurrent version
        self.concurrent_manager = ConcurrentPruningManager(
            num_robots=self.num_robots,
            mode=pruning_mode,
            sigma=consensus_config.consensus_sigma,
            sample_time=consensus_config.consensus_sample_time,
            lambda2_threshold=consensus_config.lambda2_threshold,
            lambda2_safety_margin=consensus_config.lambda2_safety_margin,
            k_max=100
        )
        
        # Store custom threshold params from config if provided
        if hasattr(self, '_custom_threshold_params'):
            params = self._custom_threshold_params
            if 'epsilon_max' in params:
                self.concurrent_manager.threshold_scheduler.lyapunov_params['epsilon_max'] = params['epsilon_max']
            if 'epsilon_min' in params:
                self.concurrent_manager.threshold_scheduler.lyapunov_params['epsilon_min'] = params['epsilon_min']
            if 'mu_max' in params:
                self.concurrent_manager.threshold_scheduler.lambda2_params['mu_max'] = params['mu_max']
            if 'mu_min' in params:
                self.concurrent_manager.threshold_scheduler.lambda2_params['mu_min'] = params['mu_min']
        
        self.pruning_mode = pruning_mode
        self.concurrent_initialized = False
        self.concurrent_iteration = 0
        self.last_pruned_edge = None
        self.pruning_events = []
        
        # Track topology changes
        self.topology_stable_rounds = 0
        self.concurrent_stability_rounds = 10  # Renamed to avoid conflict with parent property
    
    def step(self) -> dict:
        """Execute one simulation step with concurrent pruning."""
        # Step 1-4: Robot movement and control
        self.tree_builder.build_tree(self.robots, self.topology_manager.neighbor_graph)
        
        for robot_id, robot in enumerate(self.robots):
            control_force = self.controller.compute_control(robot_id, self.robots)
            robot.step(self.flow, self.dt, self.time, self.workspace, control_force)
            self.metrics.record_control(robot_id, control_force)
        
        self.time += self.dt
        self.metrics.record_step(self, self.time)
        
        # Step 5: Update topology
        old_edges = set(self.current_edges())
        self.topology_manager.update_neighbor_graph(self.robots)
        new_edges = set(self.current_edges())
        
        # Check topology changes
        edges_broken = old_edges - new_edges
        edges_formed = new_edges - old_edges
        topology_changed = len(edges_broken) > 0 or len(edges_formed) > 0
        
        if topology_changed:
            # Reset on topology change
            self.topology_stable_rounds = 0
            self.concurrent_manager.reset()
            self.concurrent_initialized = False
            self.concurrent_iteration = 0
            
            if self.verbose:
                print(f"\n[t={self.time:5.2f}] Topology changed! Resetting concurrent pruning")
            
            return {
                "decision": "topology_unstable",
                "edges_broken": sorted(edges_broken),
                "edges_formed": sorted(edges_formed),
                "concurrent_iteration": self.concurrent_iteration
            }
        else:
            self.topology_stable_rounds += 1
        
        # Wait for stability
        if self.topology_stable_rounds < self.concurrent_stability_rounds:
            return {
                "decision": "waiting_for_stability",
                "stable_rounds": self.topology_stable_rounds,
                "threshold": self.concurrent_stability_rounds,
                "concurrent_iteration": self.concurrent_iteration
            }
        
        # Initialize concurrent manager if needed
        if not self.concurrent_initialized:
            positions = np.array([r.position for r in self.robots], dtype=float)
            self.concurrent_manager.initialize_robot_knowledge(positions, new_edges)
            self.concurrent_initialized = True
            
            if self.verbose:
                print(f"\n[t={self.time:5.2f}] Initialized concurrent pruning (mode: {self.pruning_mode})")
            
            return {
                "decision": "initializing_concurrent",
                "mode": self.pruning_mode,
                "concurrent_iteration": 0
            }
        
        # DISTRIBUTED CONCURRENT CONSENSUS + PRUNING STEP
        # Three-phase protocol:
        #   1. PROPOSAL: Each robot evaluates incident edges
        #   2. NEGOTIATION: Both endpoints must agree
        #   3. CONFLICT RESOLUTION: Deterministic tie-breaking
        positions = np.array([r.position for r in self.robots], dtype=float)
        
        # Enable debug every 20 iterations if verbose (shows distributed phases)
        debug_mode = self.verbose and (self.concurrent_iteration % 20 == 0)
        
        report = self.concurrent_manager.concurrent_step(
            positions=positions,
            current_edges=new_edges,
            debug=debug_mode
        )
        
        self.concurrent_iteration = report['iteration']
        
        # Check if edge was pruned
        if report['pruned_edge']:
            pruned_edge = report['pruned_edge']
            self.last_pruned_edge = pruned_edge
            
            # Actually prune the edge in topology
            self.topology_manager.prune_edge(pruned_edge, self.robots)
            
            # Record event
            self.pruning_events.append({
                'time': self.time,
                'edge': pruned_edge,
                'iteration': self.concurrent_iteration,
                'phase': report['phase']
            })
            
            if self.verbose:
                i, j = pruned_edge
                print(f"\n[t={self.time:5.2f}] CONCURRENT PRUNED {pruned_edge} "
                      f"(iter={self.concurrent_iteration}, phase={report['phase']}) | "
                      f"edges left={len(new_edges)-1}")

            self.metrics.record_pruning_event(self.time, pruned_edge, "concurrent")
            
            return {
                "decision": "concurrent_prune",
                "removed_edge": pruned_edge,
                "concurrent_iteration": self.concurrent_iteration,
                "phase": report['phase'],
                "edges_remaining": len(new_edges) - 1
            }
        
        # No pruning this step
        return {
            "decision": "concurrent_running",
            "concurrent_iteration": self.concurrent_iteration,
            "phase": report['phase']
        }


class ConcurrentAnimator:
    """Animator for distributed concurrent pruning simulation.
    
    Visualizes:
      - Robot positions and movement
      - Communication topology (edges)
      - Edge pruning events (highlighted)
      - Convergence metrics (Lyapunov/Disagreement per robot)
      - Distributed coordination phases
    """
    
    def __init__(self, sim, config=None):
        """Initialize animator with distributed concurrent tracking.
        
        Args:
            sim: ConcurrentPruningSimulation instance
            config: VisualizationConfig
        """
        self.sim = sim
        self.config = config or VisualizationConfig()
        
        # History tracking
        self.time_history = []
        self.lyapunov_history = {i: [] for i in range(sim.num_robots)}
        self.disagreement_history = {i: [] for i in range(sim.num_robots)}
        self.edge_count_history = []
        self.max_time_window = 100  # seconds
        
        # Create figure with subplots
        self.fig = plt.figure(figsize=(16, 7))
        
        # Main simulation plot (left, 60% width)
        self.ax = self.fig.add_subplot(121)
        
        # Convergence metrics plot (right, 40% width)
        self.ax_metrics = self.fig.add_subplot(122)
        
        # Setup plots
        self._setup_main_plot()
        self._setup_metrics_plot()
        
    def _setup_main_plot(self):
        """Setup main simulation visualization."""
        sim = self.sim
        
        self.ax.set_xlim(0.0, sim.workspace[0])
        self.ax.set_ylim(0.0, sim.workspace[1])
        self.ax.set_title(f"Distributed Concurrent Pruning (Mode: {sim.pruning_mode.upper()})", 
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
            [], [], s=self.config.node_size, c="#e74c3c", 
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
        self.active_lines = LineCollection(
            [], colors="#9b59b6", linewidths=self.config.active_linewidth, zorder=3
        )
        self.highlight_lines = LineCollection(
            [], colors="#e74c3c", linewidths=self.config.highlight_linewidth, zorder=5
        )
        
        self.ax.add_collection(self.active_lines)
        self.ax.add_collection(self.highlight_lines)
        
        # Status text
        self.status = self.ax.text(
            0.02, 0.98, "", transform=self.ax.transAxes,
            ha="left", va="top", fontsize=9, family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="white", alpha=0.9),
        )
        
        # Concurrent info text
        self.concurrent_status = self.ax.text(
            0.02, 0.02, "", transform=self.ax.transAxes,
            ha="left", va="bottom", fontsize=8, family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="lightyellow", alpha=0.95),
        )
        
        # Legend
        legend_elements = [
            Line2D([0], [0], color='#9b59b6', linewidth=2.5, label='Active Edges'),
            Line2D([0], [0], color='#e74c3c', linewidth=4.5, label='Recently Pruned'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#e74c3c',
                   markersize=8, label='Robot'),
        ]
        self.ax.legend(handles=legend_elements, loc='upper right', fontsize=7, framealpha=0.9)
    
    def _setup_metrics_plot(self):
        """Setup convergence metrics visualization."""
        self.ax_metrics.set_title(f"Concurrent Pruning Metrics", fontsize=10, fontweight='bold')
        self.ax_metrics.set_xlabel("Time (s)")
        self.ax_metrics.grid(True, alpha=0.3)
        
        colors = ['#e74c3c', '#2ecc71', '#3498db', '#f39c12', '#9b59b6', 
                  '#1abc9c', '#e67e22', '#34495e']
        
        # Choose what to plot based on mode
        if self.sim.pruning_mode in ['lyapunov', 'hybrid']:
            self.ax_metrics.set_ylabel("Local Lyapunov V_l(k)")
            self.metric_lines = []
            for i in range(self.sim.num_robots):
                line, = self.ax_metrics.plot([], [], linewidth=1.5, alpha=0.7,
                                            color=colors[i % len(colors)],
                                            label=f'V_{i+1}')
                self.metric_lines.append(line)
            self.ax_metrics.set_yscale('log')
            self.ax_metrics.legend(loc='upper right', fontsize=6, ncol=3)
        
        elif self.sim.pruning_mode == 'max_disagreement':
            self.ax_metrics.set_ylabel("Max Disagreement D_l(k)")
            self.metric_lines = []
            for i in range(self.sim.num_robots):
                line, = self.ax_metrics.plot([], [], linewidth=1.5, alpha=0.7,
                                            color=colors[i % len(colors)],
                                            label=f'D_{i+1}')
                self.metric_lines.append(line)
            self.ax_metrics.set_yscale('log')
            self.ax_metrics.legend(loc='upper right', fontsize=6, ncol=3)
        
        # Edge count subplot (secondary axis)
        self.ax_edges = self.ax_metrics.twinx()
        self.ax_edges.set_ylabel("Edge Count", color='black')
        self.edge_count_line, = self.ax_edges.plot(
            [], [], 'k-', linewidth=2.5, alpha=0.8, label='Edges'
        )
        
        # Initial edges line (reference)
        if hasattr(self.sim, 'current_edges'):
            initial_edges = len(self.sim.current_edges())
            self.ax_edges.axhline(y=initial_edges, color='gray', linestyle=':', linewidth=1.5)
        
        self.ax_metrics.set_xlim(0, 10)
        self.ax_metrics.set_ylim(0.001, 10)
        self.ax_edges.set_ylim(0, 30)
    
    def _update_plot(self, frame):
        """Update visualization for one frame."""
        # Step simulation
        report = self.sim.step()
        
        # Update time history
        current_time = self.sim.time
        self.time_history.append(current_time)
        self.edge_count_history.append(len(self.sim.current_edges()))
        
        # Update metrics from concurrent manager
        if self.sim.concurrent_initialized:
            if self.sim.pruning_mode in ['lyapunov', 'hybrid']:
                monitor = self.sim.concurrent_manager.lyapunov_monitor
                if monitor:
                    for robot_id in range(self.sim.num_robots):
                        if len(monitor.lyapunov_history[robot_id]) > 0:
                            V_l = monitor.lyapunov_history[robot_id][-1]
                            self.lyapunov_history[robot_id].append(V_l)
                        else:
                            self.lyapunov_history[robot_id].append(0.001)
            
            if self.sim.pruning_mode in ['max_disagreement', 'hybrid']:
                monitor = self.sim.concurrent_manager.disagreement_monitor
                if monitor:
                    for robot_id in range(self.sim.num_robots):
                        if len(monitor.disagreement_history[robot_id]) > 0:
                            D_l = monitor.disagreement_history[robot_id][-1]
                            self.disagreement_history[robot_id].append(D_l)
                        else:
                            self.disagreement_history[robot_id].append(0.001)
        else:
            # Not initialized yet, append small values
            for robot_id in range(self.sim.num_robots):
                self.lyapunov_history[robot_id].append(0.001)
                self.disagreement_history[robot_id].append(0.001)
        
        # Limit history
        if len(self.time_history) > self.max_time_window * 10:
            self.time_history.pop(0)
            self.edge_count_history.pop(0)
            for robot_id in range(self.sim.num_robots):
                self.lyapunov_history[robot_id].pop(0)
                self.disagreement_history[robot_id].pop(0)
        
        # Update metrics plot
        if self.sim.pruning_mode in ['lyapunov', 'hybrid']:
            for robot_id in range(self.sim.num_robots):
                self.metric_lines[robot_id].set_data(
                    self.time_history,
                    self.lyapunov_history[robot_id]
                )
        elif self.sim.pruning_mode == 'max_disagreement':
            for robot_id in range(self.sim.num_robots):
                self.metric_lines[robot_id].set_data(
                    self.time_history,
                    self.disagreement_history[robot_id]
                )
        
        # Update edge count
        self.edge_count_line.set_data(self.time_history, self.edge_count_history)
        
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
        
        # Update edges
        edges = self.sim.current_edges()
        active_segments = [(positions[i], positions[j]) for i, j in edges]
        self.active_lines.set_segments(active_segments)
        
        # Highlight recently pruned edge
        if self.sim.last_pruned_edge and frame % 20 < 10:
            highlight_segments = [self.sim.last_pruned_edge]
            # Check if edge endpoints still exist
            if all(0 <= idx < len(positions) for idx in self.sim.last_pruned_edge):
                i, j = self.sim.last_pruned_edge
                self.highlight_lines.set_segments([(positions[i], positions[j])])
        else:
            self.highlight_lines.set_segments([])
        
        # Update flow field
        flow_velocities = np.array(
            [self.sim.flow.velocity(p, self.sim.time) for p in self.flow_grid_points]
        )
        self.quiver.set_UVC(flow_velocities[:, 0], flow_velocities[:, 1])
        
        # Update status text
        decision = report.get('decision', 'unknown')
        status_lines = [
            f"Time: {self.sim.time:6.2f}s",
            f"Robots: {self.sim.num_robots}",
            f"Edges: {len(edges)}",
            f"Decision: {decision}",
        ]
        self.status.set_text("\n".join(status_lines))
        
        # Update concurrent status
        if self.sim.concurrent_initialized:
            phase = self.sim.concurrent_manager.threshold_scheduler.get_phase_name()
            iter_num = self.sim.concurrent_iteration
            pruned_total = len(self.sim.concurrent_manager.pruned_edges)
            
            # Get current threshold info
            scheduler = self.sim.concurrent_manager.threshold_scheduler
            progress = scheduler.current_iteration / max(scheduler.k_max, 1)
            
            concurrent_lines = [
                f"CONCURRENT MODE: {self.sim.pruning_mode.upper()}",
                f"Iteration: {iter_num}/{scheduler.k_max}",
                f"Progress: {progress*100:.1f}%",
                f"Phase: {phase}",
                f"Total Pruned: {pruned_total}",
                f"Edges Remaining: {len(edges)}",
            ]
            
            # Add mode-specific metrics
            if self.sim.pruning_mode in ['lyapunov', 'hybrid']:
                monitor = self.sim.concurrent_manager.lyapunov_monitor
                if monitor and len(monitor.lyapunov_history[0]) > 0:
                    avg_V = np.mean([monitor.lyapunov_history[i][-1] 
                                    for i in range(self.sim.num_robots)])
                    concurrent_lines.append(f"Avg Lyapunov: {avg_V:.3e}")
            
            if self.sim.pruning_mode in ['max_disagreement', 'hybrid']:
                monitor = self.sim.concurrent_manager.disagreement_monitor
                if monitor and len(monitor.disagreement_history[0]) > 0:
                    avg_D = np.mean([monitor.disagreement_history[i][-1] 
                                    for i in range(self.sim.num_robots)])
                    concurrent_lines.append(f"Avg Disagreement: {avg_D:.3e}")
            
            self.concurrent_status.set_text("\n".join(concurrent_lines))
        else:
            self.concurrent_status.set_text("CONCURRENT: Not initialized")
        
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
    """Run GUI simulation with concurrent pruning."""
    print("\n" + "="*70)
    print("CONCURRENT CONSENSUS + PRUNING - GUI SIMULATION")
    print("="*70)
    
    # ========== CONFIGURATION ==========
    VERBOSE_DEBUG = True     # Set to True to see detailed pruning decisions
    PRUNING_MODE = 'lyapunov'  # Options: 'lyapunov', 'max_disagreement', 'hybrid'
    NUM_ROBOTS = 10
    K_MAX = 150              # Maximum iterations for threshold scheduling
    
    # Pruning aggressiveness (higher = more conservative)
    EPSILON_MAX = 0.10       # Lyapunov: Require 10% decrease initially
    EPSILON_MIN = 0.01       # Lyapunov: Require 1% decrease at end
    LAMBDA2_MARGIN_MAX = 0.2 # Lambda2: Moderate safety margin initially
    LAMBDA2_MARGIN_MIN = 0.05 # Lambda2: Small safety margin at end
    # ===================================
    
    sim_config = SimulationConfig(
        num_robots=NUM_ROBOTS,
        communication_radius=3.5,
        verbose=VERBOSE_DEBUG
    )
    control_config = ControlConfig()
    consensus_config = ConsensusConfig()
    vis_config = VisualizationConfig(interval_ms=100)
    
    print(f"\nConfiguration:")
    print(f"  Robots: {NUM_ROBOTS}")
    print(f"  Pruning Mode: {PRUNING_MODE.upper()}")
    print(f"  Communication Radius: {sim_config.communication_radius}m")
    print(f"  Max Iterations: {K_MAX}")
    print(f"  Verbose Debug: {VERBOSE_DEBUG}")
    print(f"\nPruning Parameters:")
    print(f"  Epsilon (Lyapunov): {EPSILON_MAX:.2f} -> {EPSILON_MIN:.3f}")
    print(f"  Lambda2 Margin: {LAMBDA2_MARGIN_MAX:.2f} -> {LAMBDA2_MARGIN_MIN:.2f}")
    print(f"\nDistributed Coordination Features:")
    print(f"  ✓ PHASE 1: Each robot proposes edges (local evaluation)")
    print(f"  ✓ PHASE 2: Both endpoints negotiate (bilateral consensus)")
    print(f"  ✓ PHASE 3: Deterministic conflict resolution (no coordinator)")
    print(f"  ✓ Uses consensus estimates A^l(k) (not centralized A(k))")
    print(f"  ✓ Lyapunov/Disagreement safety constraints")
    print(f"  ✓ Adaptive: Conservative -> Moderate -> Aggressive transition")
    print(f"  ✓ Real-time visualization of distributed protocol")
    print(f"\nLeft plot: Robot simulation")
    print(f"Right plot: Convergence metrics + edge count")
    print("="*70 + "\n")
    
    # Create simulation and animator
    sim = ConcurrentPruningSimulation(
        sim_config, control_config, consensus_config, PRUNING_MODE
    )
    
    # Override k_max and custom threshold parameters
    sim.concurrent_manager.threshold_scheduler.k_max = K_MAX
    sim.concurrent_manager.threshold_scheduler.lyapunov_params.update({
        'epsilon_max': EPSILON_MAX,
        'epsilon_min': EPSILON_MIN,
        'alpha': 2.0
    })
    sim.concurrent_manager.threshold_scheduler.lambda2_params.update({
        'mu_max': LAMBDA2_MARGIN_MAX,
        'mu_min': LAMBDA2_MARGIN_MIN,
        'beta': 2.0
    })
    
    animator = ConcurrentAnimator(sim, vis_config)
    anim = animator.animate()
    
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
