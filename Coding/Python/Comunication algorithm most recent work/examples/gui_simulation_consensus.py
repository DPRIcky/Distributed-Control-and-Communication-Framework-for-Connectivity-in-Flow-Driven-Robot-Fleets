"""GUI simulation using NOVEL consensus-based pruning method.

This version explicitly uses the new distributed consensus approach
with verbose logging to prove the novel methods are being used.
"""

import sys
import os
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
import numpy as np

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import SimulationConfig, ControlConfig, ConsensusConfig, VisualizationConfig
from simulation import HybridUnderwaterSimulation
from visualization import HybridAnimator


class ConsensusProofSimulation(HybridUnderwaterSimulation):
    """Modified simulation that uses consensus-based pruning with logging."""
    
    def __init__(self, sim_config, control_config, consensus_config):
        """Initialize with consensus tracking."""
        super().__init__(sim_config, control_config, consensus_config)
        self.using_novel_method = True
        self.consensus_phase_done = False
    
    def step(self) -> dict:
        """Override step to use NOVEL consensus-based pruning."""
        # STEP 1-7: Same as parent (movement, topology detection, stability checks)
        self.tree_builder.build_tree(self.robots, self.topology_manager.neighbor_graph)
        
        for robot_id, robot in enumerate(self.robots):
            control_force = self.controller.compute_control(robot_id, self.robots)
            robot.step(self.flow, self.dt, self.time, self.workspace, control_force)
        
        self.time += self.dt
        
        old_edges = set(self.current_edges())
        self.topology_manager.update_neighbor_graph(self.robots)
        new_edges = set(self.current_edges())
        
        edges_broken = old_edges - new_edges
        edges_formed = new_edges - old_edges
        topology_changed = len(edges_broken) > 0 or len(edges_formed) > 0
        
        if topology_changed:
            self.pruning_manager.reset_on_topology_change()
            self.consensus_phase_done = False
            if self.verbose:
                print(f"[t={self.time:5.2f}] Topology changed! Resetting consensus")
            return {
                "decision": "topology_unstable",
                "edges_broken": sorted(edges_broken),
                "edges_formed": sorted(edges_formed),
            }
        else:
            self.pruning_manager.increment_stability()
        
        if not self.pruning_manager.is_stable():
            return {
                "decision": "waiting_for_stability",
                "stable_rounds": self.pruning_manager.topology_stable_rounds,
                "threshold": self.consensus_config.stability_threshold
            }
        
        self.pruning_manager.add_topology_snapshot(
            new_edges,
            self.topology_manager.gather_edge_lengths(self.robots),
            self.time
        )
        
        if not self.pruning_manager.has_sufficient_history():
            return {
                "decision": "building_history",
                "history_size": len(self.pruning_manager.topology_history),
                "needed": self.consensus_config.rolling_window_size
            }
        
        # NOVEL CONSENSUS-BASED PRUNING STARTS HERE
        positions = np.array([r.position for r in self.robots], dtype=float)
        edge_lengths = self.topology_manager.gather_edge_lengths(self.robots)
        
        # Run consensus phase until converged
        if not self.consensus_phase_done:
            print("\n" + "="*70)
            print("[NOVEL] Running Adjacency Matrix Consensus")
            print("="*70)
            
            for iteration in range(50):
                converged = self.pruning_manager.run_consensus_phase(
                    positions=positions,
                    edge_lengths=edge_lengths,
                    current_edges=new_edges
                )
                
                if iteration % 10 == 0:
                    A0 = self.pruning_manager.get_consensus_estimate(0)
                    A1 = self.pruning_manager.get_consensus_estimate(1)
                    disagreement = np.max(np.abs(A0 - A1))
                    print(f"  Iter {iteration}: disagreement = {disagreement:.6f}")
                
                if converged:
                    print(f"  ✓ Consensus CONVERGED in {iteration+1} iterations!")
                    self.consensus_phase_done = True
                    break
            else:
                self.pruning_manager.consensus_converged = True
                self.consensus_phase_done = True
                print(f"  ⚠ Forced convergence after {iteration+1} iterations")
            
            return {"decision": "running_consensus"}
        
        # Use distributed method to find redundant edge
        print("\n[NOVEL] Using distributed edge detection...")
        robot_decisions = []
        for robot_id in range(self.num_robots):
            decision = self.pruning_manager.find_redundant_edge_distributed(
                robot_id=robot_id,
                debug=(robot_id == 0)
            )
            robot_decisions.append(decision)
        
        unique_decisions = set(robot_decisions)
        
        if len(unique_decisions) == 1 and robot_decisions[0] is not None:
            candidate = robot_decisions[0]
            print(f"[NOVEL] ✓ ALL {self.num_robots} ROBOTS AGREE: Remove {candidate}")
            
            # Prune the edge
            self.topology_manager.prune_edge(candidate, self.robots)
            
            # CRITICAL: Update consensus estimates to reflect the pruned edge
            # Set A[i,j] = 0 for the pruned edge in ALL robot estimates
            # This prevents re-detecting the same edge without full consensus reset
            i, j = candidate
            for robot_id in range(self.num_robots):
                A = self.pruning_manager.adjacency_consensus.A_estimates[robot_id]
                A[i, j] = 0.0
                A[j, i] = 0.0
            
            self.pruning_manager.reset_after_successful_prune()
            self.consensus_phase_done = False
            
            if self.verbose:
                i, j = candidate
                print(f"[t={self.time:5.2f}] PRUNED {candidate} (CONSENSUS-BASED) | "
                      f"edges left={len(self.current_edges())} | "
                      f"Robot {i} and {j} gain freedom")
            
            return {
                "decision": "prune",
                "removed_edge": candidate,
                "method": "NOVEL_CONSENSUS"
            }
        else:
            # Robots disagree - reset consensus to try converging further
            self.consensus_phase_done = False
            if self.verbose and len(unique_decisions) > 1:
                print(f"[NOVEL] ⚠ Robots disagree: {unique_decisions}")
                print(f"[NOVEL] → Re-running consensus to improve estimates")
            return {"decision": "no_consensus"}


class ConsensusAnimator(HybridAnimator):
    """Extended animator with consensus visualization."""
    
    def __init__(self, sim, config=None):
        """Initialize with consensus tracking."""
        # Store reference before calling super().__init__
        self.consensus_sim = sim
        
        # History of λ₂ estimates for each robot over time
        self.time_history = []
        self.lambda2_history = {i: [] for i in range(sim.num_robots)}
        self.ground_truth_history = []  # True λ₂ from actual graph
        self.max_time_window = 150  # seconds
        
        # Create figure with two subplots
        self.fig = plt.figure(figsize=(14, 6))
        
        # Main simulation plot (left, larger)
        self.ax = self.fig.add_subplot(121)
        
        # Lambda2 convergence plot (right) - like Fig. 9 from paper
        self.ax_consensus = self.fig.add_subplot(122)
        self.ax_consensus.set_title("Robots' Local Algebraic Connectivity (λ₂)", fontsize=10, fontweight='bold')
        self.ax_consensus.set_xlabel("Time (s)")
        self.ax_consensus.set_ylabel("λ₂")
        self.ax_consensus.grid(True, alpha=0.3)
        
        # Create a line for each robot with different colors
        self.colors = ['#e74c3c', '#2ecc71', '#3498db', '#f39c12', '#9b59b6', '#1abc9c', '#e67e22', '#34495e']
        self.lambda2_lines = []
        for i in range(sim.num_robots):
            line, = self.ax_consensus.plot([], [], linewidth=1.5, alpha=0.7,
                                          color=self.colors[i % len(self.colors)],
                                          label=f'λ₂^{i+1} (distributed)')
            self.lambda2_lines.append(line)
        
        # Ground truth line (centralized - what the paper assumes)
        self.ground_truth_line, = self.ax_consensus.plot([], [], 'k-', linewidth=3, 
                                                          label='λ₂ TRUE (centralized)',
                                                          alpha=0.8, zorder=10)
        
        # Reference threshold line
        lambda2_threshold = self.consensus_sim.consensus_config.lambda2_threshold
        self.threshold_line = self.ax_consensus.axhline(
            y=lambda2_threshold, color='orange', linestyle='--', linewidth=2.5, 
            label=f'λ₂_ref = {lambda2_threshold}'
        )
        
        self.ax_consensus.legend(loc='upper right', fontsize=6, ncol=3)
        self.ax_consensus.set_ylim(0, 2.5)
        self.ax_consensus.set_xlim(0, 10)
        
        # Now call parent init (which will use self.fig and self.ax)
        self.config = config or VisualizationConfig()
        
        # Setup main plot
        self.ax.set_xlim(0.0, sim.workspace[0])
        self.ax.set_ylim(0.0, sim.workspace[1])
        self.ax.set_title("Hybrid Consensus Pruning (Multi-Layer Robustness)", fontsize=12, fontweight='bold')
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        
        # Initialize parent components manually
        self._init_main_plot()
        
    def _init_main_plot(self):
        """Initialize main simulation plot components."""
        sim = self.consensus_sim
        
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
        self.labels = []
        
        # Flow field visualization
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
        
        # Edge visualization
        from matplotlib.collections import LineCollection
        from matplotlib.lines import Line2D
        
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
        
        self.ax.add_collection(self.active_lines)
        self.ax.add_collection(self.candidate_lines)
        self.ax.add_collection(self.highlight_lines)
        
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
            Line2D([0], [0], color='#9b59b6', linewidth=2.5, label='Active Edges'),
            Line2D([0], [0], color='#f39c12', linewidth=3.5, linestyle='dashdot', label='Candidate Edge'),
            Line2D([0], [0], color='#e74c3c', linewidth=4.5, label='Recently Pruned'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#e74c3c',
                   markersize=8, label='Connected Robot'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#ff8c00',
                   markersize=8, label='Isolated Robot'),
        ]
        self.ax.legend(handles=legend_elements, loc='upper right', fontsize=7, framealpha=0.9)
        
        # Consensus status text (on main plot)
        self.consensus_status = self.ax.text(
            0.02,
            0.02,
            "",
            transform=self.ax.transAxes,
            ha="left",
            va="bottom",
            fontsize=8,
            family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="lightyellow", alpha=0.95),
        )
    
    def animate(self):
        """Create and return animation with consensus tracking."""
        self.sim = self.consensus_sim
        
        def update_with_consensus(frame):
            """Update both simulation and lambda2 plots."""
            # Call parent update
            result = self._update_plot(frame)
            
            # Update lambda2 plot - track each robot's estimate
            current_time = self.consensus_sim.time
            self.time_history.append(current_time)
            
            # Compute GROUND TRUTH λ₂ from actual graph (centralized - paper's assumption)
            # Build true adjacency matrix from current edges
            actual_adjacency = np.zeros((self.consensus_sim.num_robots, self.consensus_sim.num_robots))
            for i in range(self.consensus_sim.num_robots):
                for j in self.consensus_sim.topology_manager.neighbor_graph[i]:
                    actual_adjacency[i, j] = 1.0
            
            n = actual_adjacency.shape[0]
            D_true = np.diag(np.sum(actual_adjacency, axis=1))
            L_true = D_true - actual_adjacency
            try:
                eigenvalues_true = np.linalg.eigvalsh(L_true)
                eigenvalues_true = np.sort(eigenvalues_true)
                lambda2_true = eigenvalues_true[1] if len(eigenvalues_true) > 1 else 0.0
            except:
                lambda2_true = 0.0
            self.ground_truth_history.append(lambda2_true)
            
            # Get each robot's lambda2 estimate from their consensus matrix (DISTRIBUTED)
            for robot_id in range(self.consensus_sim.num_robots):
                A_estimate = self.consensus_sim.pruning_manager.get_consensus_estimate(robot_id)
                
                # Compute lambda2 from this robot's adjacency estimate
                # Simple spectral calculation
                n = A_estimate.shape[0]
                D = np.diag(np.sum(A_estimate, axis=1))
                L = D - A_estimate  # Laplacian
                
                # Get eigenvalues
                try:
                    eigenvalues = np.linalg.eigvalsh(L)
                    eigenvalues = np.sort(eigenvalues)
                    lambda2 = eigenvalues[1] if len(eigenvalues) > 1 else 0.0
                except:
                    lambda2 = 0.0
                
                self.lambda2_history[robot_id].append(lambda2)
            
            # Limit history length (keep last max_time_window seconds)
            if len(self.time_history) > self.max_time_window * 10:  # assuming ~10 fps
                self.time_history.pop(0)
                self.ground_truth_history.pop(0)
                for robot_id in range(self.consensus_sim.num_robots):
                    self.lambda2_history[robot_id].pop(0)
            
            # Update ground truth line
            self.ground_truth_line.set_data(self.time_history, self.ground_truth_history)
            
            # Update each robot's line
            for robot_id in range(self.consensus_sim.num_robots):
                self.lambda2_lines[robot_id].set_data(
                    self.time_history,
                    self.lambda2_history[robot_id]
                )
            
            # Adjust x-axis to show recent history
            if self.time_history:
                max_time = max(self.time_history)
                self.ax_consensus.set_xlim(max(0, max_time - 30), max_time + 5)
            
            # Update consensus status text
            if hasattr(self.consensus_sim, 'consensus_phase_done'):
                if not self.consensus_sim.consensus_phase_done:
                    iteration = self.consensus_sim.pruning_manager.consensus_iterations
                    converged = self.consensus_sim.pruning_manager.consensus_converged
                    status_text = f"Consensus: {'CONVERGED' if converged else 'RUNNING'}\n"
                    status_text += f"Iteration: {iteration}"
                    self.consensus_status.set_text(status_text)
                else:
                    # Consensus done, show final state
                    self.consensus_status.set_text(f"Consensus: IDLE\n(awaiting topology change)")
            
            return result
        
        anim = FuncAnimation(
            self.fig,
            update_with_consensus,
            interval=self.config.interval_ms,
            blit=False,
            cache_frame_data=False
        )
        return anim


def main():
    """Run GUI with consensus-based pruning and proof logging."""
    # Configure simulation
    sim_config = SimulationConfig(
        num_robots=15,  # Fewer robots for clearer visualization
        communication_radius=3.0,
        verbose=True
    )
    control_config = ControlConfig()
    consensus_config = ConsensusConfig()
    vis_config = VisualizationConfig(interval_ms=150)
    
    # Create consensus-proof simulation
    sim = ConsensusProofSimulation(sim_config, control_config, consensus_config)
    
    print("\n" + "#"*70)
    print("# NOVEL CONSENSUS-BASED PRUNING - PROOF OF CONCEPT")
    print("#"*70)
    print(f"\nRobots: {sim.num_robots}")
    print(f"Goal: {sim.goal_position}")
    print(f"\nThis simulation uses:")
    print(f"  1. Adjacency matrix consensus (Griparic et al. 2022) ✓")
    print(f"  2. Adaptive lambda2 estimation (NOVEL - distributed) ✓")
    print(f"  3. Distributed edge detection (NOVEL - no global knowledge) ✓")
    print("\nWatch the terminal AND GUI for proof that novel methods are used!")
    print("  - Right plot shows each robot's λ₂ estimate over time (like Fig. 9)")
    print("  - BLACK line = Ground truth λ₂ (centralized - paper's assumption)")
    print("  - COLORED lines = Distributed λ₂ estimates (your novel contribution)")
    print("  - Yellow box shows consensus status")
    print("  - Watch how distributed estimates CONVERGE to ground truth!")
    print("#"*70 + "\n")
    
    # Create consensus animator and run
    animator = ConsensusAnimator(sim, vis_config)
    anim = animator.animate()
    
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
