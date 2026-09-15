"""Basic simulation example - text mode."""

from config import SimulationConfig, ControlConfig, ConsensusConfig
from simulation import HybridUnderwaterSimulation


def main():
    """Run basic text-mode simulation."""
    # Configure simulation
    sim_config = SimulationConfig(
        num_robots=8,
        communication_radius=2.5,
        verbose=True,
        seed=42  # For reproducibility
    )
    control_config = ControlConfig()
    consensus_config = ConsensusConfig()
    
    # Create simulation
    sim = HybridUnderwaterSimulation(sim_config, control_config, consensus_config)
    
    print(f"Starting simulation with {sim.num_robots} robots")
    print(f"Goal: {sim.goal_position}")
    print()
    
    # Run simulation
    for step_idx, report in enumerate(sim.iterate(100)):
        if step_idx % 10 == 0:
            print(f"Step {step_idx}: {report.get('decision')}")
            
    print(f"\nFinal edges: {len(sim.current_edges())}")
    print(f"Pruned: {len(sim.pruned_edges)}")


if __name__ == "__main__":
    main()
