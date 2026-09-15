"""GUI simulation example."""

import matplotlib.pyplot as plt

from config import SimulationConfig, ControlConfig, ConsensusConfig, VisualizationConfig
from simulation import HybridUnderwaterSimulation
from visualization import HybridAnimator


def main():
    """Run GUI-based simulation."""
    # Configure simulation
    sim_config = SimulationConfig(
        num_robots=10,
        communication_radius=3.0,
        verbose=True
    )
    control_config = ControlConfig()
    consensus_config = ConsensusConfig()
    vis_config = VisualizationConfig(interval_ms=100)
    
    # Create simulation
    sim = HybridUnderwaterSimulation(sim_config, control_config, consensus_config)
    
    print("Starting GUI simulation...")
    print(f"Robots: {sim.num_robots}")
    print(f"Goal: {sim.goal_position}")
    
    # Create animator and run
    animator = HybridAnimator(sim, vis_config)
    anim = animator.animate()
    
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
