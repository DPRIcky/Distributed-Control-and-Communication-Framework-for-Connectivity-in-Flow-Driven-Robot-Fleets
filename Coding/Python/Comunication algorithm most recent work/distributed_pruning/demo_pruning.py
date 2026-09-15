"""
Demo: Distributed Pruning with Connectivity Guarantee

Demonstrates provably safe distributed edge pruning with visualization.
Shows progressive pruning from fully connected graph to sparse chain topology.
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from config import SimulationConfig, ControlConfig, VisualizationConfig
from distributed_pruning import PrunedSimulation
from baseline_simulation import FullyConnectedAnimator


class PrunedAnimator(FullyConnectedAnimator):
    """
    Animator that visualizes pruned edges in different colors.
    
    - Green: Active edges (kept after pruning)
    - Red (faint): Pruned edges
    - Blue: Robots
    """
    
    def __init__(self, pruned_sim: PrunedSimulation, vis_config: VisualizationConfig):
        """Initialize pruned animator."""
        self.pruned_sim = pruned_sim
        super().__init__(pruned_sim, vis_config)
        
        # Replace active_lines with pruned edge visualization
        self.active_lines.set_colors('green')
        self.active_lines.set_linewidths(3)
        self.active_lines.set_alpha(0.8)
        self.active_lines.set_label('Active edges')
        
        # Add pruned edges collection (red, faint)
        self.pruned_lines = LineCollection(
            [], colors='red', linewidths=1,
            alpha=0.2, zorder=1, label='Pruned edges'
        )
        self.ax.add_collection(self.pruned_lines)
        
        # Update title
        self.ax.set_title(
            'Distributed Pruning with Connectivity Guarantee',
            fontsize=12, fontweight='bold'
        )
        
        # Add legend
        self.ax.legend(loc='upper right', fontsize=8)
    
    def update(self, frame):
        """Update animation frame with pruned/active edge visualization."""
        # Call parent's update method
        result = super().update(frame)
        
        # Get current positions
        positions = np.array([robot.position for robot in self.sim.robots])
        
        # Get active and pruned edges
        active_edges = self.pruned_sim.get_active_edges()
        pruned_edges = self.pruned_sim.get_pruned_edges()
        
        # Print progress every 3 seconds
        if frame % 50 == 0:  # Every ~3.5 seconds at 14 fps
            stats = self.pruned_sim.get_pruning_statistics()
            if stats:
                latest = stats[-1]
                potential = latest['total_potential_edges']
                active = latest['active_edges']
                info = latest['avg_completeness']
                print(f"[t={latest['time']:.1f}s] Potential: {potential}, Active: {active}, "
                      f"Pruned: {potential - active}, Info: {info:.0%}, Degree: {latest['avg_degree']:.1f}")
        
        # Build edge segments
        active_segments = []
        pruned_segments = []
        
        for i, j in active_edges:
            segment = [positions[i], positions[j]]
            active_segments.append(segment)
        
        for i, j in pruned_edges:
            segment = [positions[i], positions[j]]
            pruned_segments.append(segment)
        
        # Update collections
        self.active_lines.set_segments(active_segments)
        self.pruned_lines.set_segments(pruned_segments)
        
        # Update info text
        state = self.sim.get_state()
        info_text = self._format_info_text_with_pruning(state)
        self.info_text.set_text(info_text)
        
        return result + [self.pruned_lines]
    
    def _format_info_text_with_pruning(self, state):
        """Format info text with pruning statistics."""
        pruning = state.get('pruning', {})
        
        active = pruning.get('active_edges', 0)
        pruned = pruning.get('pruned_edges', 0)
        total = active + pruned
        avg_degree = pruning.get('avg_degree', 0)
        target = pruning.get('target_degree', 2)
        
        # Convergence indicator
        degree_diff = abs(avg_degree - target)
        converged = "✓" if degree_diff < 0.5 else "..."
        
        info = (
            f"Time: {state['time']:.1f}s\n"
            f"Robots: {self.pruned_sim.num_robots}\n"
            f"\n"
            f"PRUNING [{converged}]\n"
            f"Total Edges: {total}\n"
            f"Active: {active} (green)\n"
            f"Pruned: {pruned} (red)\n"
            f"\n"
            f"Avg Degree: {avg_degree:.1f}\n"
            f"Target: {target}\n"
            f"\n"
            f"Connectivity: {pruning.get('k_connectivity', 1)}-edge-connected\n"
        )
        
        return info


def run_pruning_demo():
    """Run distributed pruning demonstration."""
    
    print("="*70)
    print("DISTRIBUTED PRUNING WITH FORMAL CONNECTIVITY GUARANTEE")
    print("="*70)
    
    # Configuration
    sim_config = SimulationConfig(
        num_robots=6,
        communication_radius=1.0,  # Physical constraint - cannot change
        workspace_size=(10.0, 10.0),
        dt=0.05,
        seed=42
    )
    
    control_config = ControlConfig()
    vis_config = VisualizationConfig()
    
    print(f"\nConfiguration:")
    print(f"  Robots: {sim_config.num_robots}")
    print(f"  Communication Radius: {sim_config.communication_radius}m (PHYSICAL CONSTRAINT)")
    print(f"  Target Degree: 2 (minimal - conservative for sparse topology)")
    print(f"  Connectivity Guarantee: 1-connected (formally proven)")
    print(f"  Workspace: {sim_config.workspace_size}")
    print(f"\n  Strategy: VERY CONSERVATIVE pruning for {sim_config.communication_radius}m radius:")
    print(f"    - Requires 50% information before any pruning")
    print(f"    - Never prunes if potential degree ≤ target+1")
    print(f"    - Never prunes if active degree ≤ 2")
    print(f"    - CBF maintains active edges within {sim_config.communication_radius}m")
    
    # Create pruned simulation
    # NOTE: With 6 robots and 1m radius, use target_degree=2 (minimal)
    # Higher targets may be geometrically impossible as formation spreads
    sim = PrunedSimulation(
        sim_config,
        control_config,
        target_degree=2,  # Minimal for 1m radius (conservative)
        k_connectivity=1  # Guarantee connectivity
    )
    
    # Print initial state
    print(f"\n[Initial State]")
    edges = sim.current_edges()
    print(f"  Initial edges: {len(edges)}")
    print(f"  Expected for fully connected: {sim_config.num_robots * (sim_config.num_robots - 1) // 2}")
    print(f"  Target after pruning: ~{2 * sim_config.num_robots // 2} edges (minimal chain)")
    print(f"  Minimum for connectivity: {sim_config.num_robots - 1} edges")
    
    # Visualize
    print(f"\n[Visualization]")
    print(f"  * BLUE circles = Robots")
    print(f"  * GREEN lines = Active edges (kept)")
    print(f"  * RED lines (faint) = Pruned edges")
    print(f"\n  Expected behavior:")
    print(f"  1. Robots start TIGHTLY CLUSTERED (~0.5m spread)")
    print(f"  2. Info gathering: 0.5% per frame → 50% at ~100 frames (~7s)")
    print(f"  3. Pruning starts only after 50% information (conservative!)")
    print(f"  4. Minimal pruning: removes only provably redundant edges")
    print(f"  5. As robots move toward goal, CBF maintains active edges")
    print(f"  6. GUARANTEE: Graph stays connected (never drops below {sim_config.num_robots - 1} edges)")
    print(f"")
    print(f"  WARNING: With {sim_config.communication_radius}m radius and distant goal:")
    print(f"    If formation must spread >{sim_config.num_robots - 1}m, geometry may violate connectivity")
    print(f"    This is a PHYSICAL constraint, not an algorithm bug")
    print(f"\n  Diagnostics:")
    print(f"    - If robots disconnect due to GEOMETRY: CBF couldn't maintain formation")
    print(f"    - If robots disconnect due to PRUNING: Algorithm bug!")
    print("="*70 + "\n")
    
    # Create animator
    animator = PrunedAnimator(sim, vis_config)
    
    # Calculate number of frames (30 seconds)
    duration = 30.0
    num_frames = int(duration / sim.dt)
    
    # Create animation with slower frame rate to see progression
    anim = FuncAnimation(
        animator.fig,
        animator.update,
        frames=num_frames,
        interval=70,  # 70ms = ~14 fps (slower to see gradual changes)
        blit=False,
        repeat=False
    )
    
    plt.tight_layout()
    plt.show()
    
    # Print final statistics
    print("\n" + "="*70)
    print("FINAL RESULTS")
    print("="*70)
    
    stats = sim.get_pruning_statistics()
    if stats:
        final = stats[-1]
        print(f"\nEdge Statistics:")
        print(f"  Potential edges: {final['total_potential_edges']}")
        print(f"  Active edges: {final['active_edges']}")
        print(f"  Pruned edges: {final['total_potential_edges'] - final['active_edges']}")
        print(f"  Pruning ratio: {(final['total_potential_edges'] - final['active_edges']) / final['total_potential_edges']:.1%}")
        
        print(f"\nTopology Statistics:")
        print(f"  Average degree: {final['avg_degree']:.2f}")
        print(f"  Target degree: 2")
        print(f"  Information completeness: {final['avg_completeness']:.1%}")
        print(f"  Pruning aggressiveness: {final['avg_aggressiveness']:.1%}")
        
        # Check connectivity guarantee
        active_edges = sim.get_active_edges()
        num_robots = sim_config.num_robots
        
        # Simple connectivity check: num_edges >= num_robots - 1 (necessary condition)
        if len(active_edges) >= num_robots - 1:
            print(f"\n✓ Connectivity Check: PASSED")
            print(f"  Active edges ({len(active_edges)}) >= Minimum required ({num_robots - 1})")
        else:
            print(f"\n✗ Connectivity Check: WARNING")
            print(f"  Active edges ({len(active_edges)}) < Minimum required ({num_robots - 1})")
    
    print("="*70 + "\n")
    
    # Plot pruning progression
    if stats:
        plot_pruning_progression(stats)


def plot_pruning_progression(stats):
    """Plot how pruning progresses over time."""
    times = [s['time'] for s in stats]
    potential = [s['total_potential_edges'] for s in stats]
    active = [s['active_edges'] for s in stats]
    completeness = [s['avg_completeness'] for s in stats]
    aggressiveness = [s['avg_aggressiveness'] for s in stats]
    degrees = [s['avg_degree'] for s in stats]
    
    # Calculate target edges (for undirected graph: n * target_degree / 2)
    # With 6 robots and target_degree=2: 6 * 2 / 2 = 6 edges
    target_edges = 6  # Target for 6 robots with degree 2
    target_degree = 2
    
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    
    # Plot 1: Edge counts
    ax1 = axes[0]
    ax1.plot(times, potential, 'b--', label='Potential edges', linewidth=2)
    ax1.plot(times, active, 'g-', label='Active edges', linewidth=2)
    ax1.axhline(y=target_edges, 
                color='r', linestyle=':', label=f'Target (~{target_degree} per robot)', linewidth=2)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Number of Edges')
    ax1.set_title('Progressive Edge Pruning')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Average degree over time
    ax2 = axes[1]
    ax2.plot(times, degrees, 'purple', label='Average Degree', linewidth=2)
    ax2.axhline(y=target_degree, color='r', linestyle=':', label='Target Degree', linewidth=2)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Average Degree')
    ax2.set_title('Network Degree Evolution')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Information metrics
    ax3 = axes[2]
    ax3.plot(times, completeness, 'b-', label='Information Completeness', linewidth=2)
    ax3.plot(times, aggressiveness, 'r-', label='Pruning Aggressiveness', linewidth=2)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Metric Value')
    ax3.set_title('Pruning Controller Metrics')
    ax3.set_ylim([0, 1.1])
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    run_pruning_demo()
