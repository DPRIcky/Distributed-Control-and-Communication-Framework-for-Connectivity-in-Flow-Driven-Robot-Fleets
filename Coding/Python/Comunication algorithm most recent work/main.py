"""Main entry point for hybrid underwater consensus pruning simulation.

Features:
    * Robots drift in underwater flow field with individual dynamics
    * CLF (Control Lyapunov Function) for goal-seeking behavior
    * CBF (Control Barrier Function) for safety and connectivity constraints
    * HYBRID decentralized consensus-based edge pruning with:
      - Topology stability detection
      - Rolling window validation
      - Interleaved consensus execution
      - Final safety checks before pruning

Two usage modes:
    python main.py text      # run text-only simulation
    python main.py gui       # run interactive animation
"""

from __future__ import annotations

import argparse

import matplotlib.pyplot as plt
import numpy as np

from config import SimulationConfig, ControlConfig, ConsensusConfig, VisualizationConfig
from simulation import HybridUnderwaterSimulation
from visualization import HybridAnimator


def run_text(num_robots: int, steps: int, verbose: bool, communication_radius: float = 2.5) -> None:
    """Run text-based simulation.
    
    Args:
        num_robots: Number of robots
        steps: Number of simulation steps
        verbose: Enable verbose output
        communication_radius: Communication radius
    """
    sim_config = SimulationConfig(
        num_robots=num_robots,
        communication_radius=communication_radius,
        verbose=verbose
    )
    control_config = ControlConfig()
    consensus_config = ConsensusConfig()
    
    sim = HybridUnderwaterSimulation(sim_config, control_config, consensus_config)
    
    print(f"Goal position: ({sim.goal_position[0]:.2f}, {sim.goal_position[1]:.2f})")
    print(f"Starting HYBRID simulation with {num_robots} robots")
    print(f"Stability threshold: {sim.stability_threshold} steps")
    print(f"Rolling window: {sim.rolling_window_size} snapshots")
    print(f"Consensus rounds per step: {sim.consensus_rounds_per_step}")
    print()

    for step_index in range(steps):
        report = sim.step()

        if step_index % 5 == 0 or report.get("decision") in ["prune", "abort_topology_changed", "topology_unstable"]:
            decision = report.get("decision", "none")
            print(f"[Step {step_index+1:03d}] t={sim.time:.2f}s | Decision: {decision}")
            if decision == "prune":
                print(f"  ✓ Pruned: {report['removed_edge']}, Edges left: {report['edges_remaining']}")
            elif decision == "topology_unstable":
                print(f"  ⚠ Broken: {report.get('edges_broken')}, Formed: {report.get('edges_formed')}")

    print("\n" + "="*60)
    print(f"Final pruned edges: {sorted(sim.pruned_edges)}")
    print(f"Final edge count: {len(sim.current_edges())}")


def run_gui(num_robots: int, verbose: bool, communication_radius: float = 2.5) -> None:
    """Run GUI-based simulation.
    
    Args:
        num_robots: Number of robots
        verbose: Enable verbose output
        communication_radius: Communication radius
    """
    sim_config = SimulationConfig(
        num_robots=num_robots,
        communication_radius=communication_radius,
        verbose=verbose
    )
    control_config = ControlConfig()
    consensus_config = ConsensusConfig()
    vis_config = VisualizationConfig()
    
    sim = HybridUnderwaterSimulation(sim_config, control_config, consensus_config)
    
    print(f"Goal position: ({sim.goal_position[0]:.2f}, {sim.goal_position[1]:.2f})")
    print(f"Starting HYBRID GUI with {num_robots} robots")
    print(f"Multi-layer robustness enabled:")
    print(f"  - Topology stability detection")
    print(f"  - Rolling window validation ({sim.rolling_window_size} snapshots)")
    print(f"  - Interleaved consensus ({sim.consensus_rounds_per_step} rounds/step)")
    print(f"  - Final safety checks before pruning")
    print()

    animator = HybridAnimator(sim, vis_config)
    anim = animator.animate()
    plt.tight_layout()
    plt.show()
    return anim


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments.
    
    Returns:
        Parsed arguments
    """
    parser = argparse.ArgumentParser(description="Hybrid underwater consensus pruning demo")
    parser.add_argument(
        "mode",
        choices={"text", "gui"},
        nargs="?",
        default="gui",
        help="text: console log, gui: interactive animation",
    )
    parser.add_argument(
        "--robots", type=int, default=8, help="number of robots (>=3, default 8)"
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=100,
        help="simulation steps for text mode (default 100)",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="suppress per-step console messages",
    )
    return parser.parse_args()


def main() -> None:
    """Main entry point."""
    args = parse_args()
    num_robots = max(3, args.robots)
    verbose = not args.quiet
    
    if args.mode == "gui":
        run_gui(num_robots, verbose)
    else:
        run_text(num_robots, steps=args.steps, verbose=verbose)


if __name__ == "__main__":
    main()
