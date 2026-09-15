"""Integration tests for full simulation."""

import numpy as np

from config import SimulationConfig, ControlConfig, ConsensusConfig
from simulation import HybridUnderwaterSimulation


def test_simulation_basic():
    """Test basic simulation initialization and step."""
    sim_config = SimulationConfig(
        num_robots=5,
        communication_radius=3.0,
        verbose=False,
        seed=42
    )
    control_config = ControlConfig()
    consensus_config = ConsensusConfig()
    
    sim = HybridUnderwaterSimulation(sim_config, control_config, consensus_config)
    
    # Check initialization
    assert len(sim.robots) == 5
    assert sim.num_robots == 5
    assert sim.communication_radius == 3.0
    
    # Run one step
    result = sim.step()
    assert "decision" in result
    assert sim.time > 0


def test_simulation_deterministic():
    """Test simulation is deterministic with fixed seed."""
    sim_config1 = SimulationConfig(num_robots=5, seed=42, verbose=False)
    sim1 = HybridUnderwaterSimulation(sim_config1, ControlConfig(), ConsensusConfig())
    
    sim_config2 = SimulationConfig(num_robots=5, seed=42, verbose=False)
    sim2 = HybridUnderwaterSimulation(sim_config2, ControlConfig(), ConsensusConfig())
    
    # Same seed should produce same initial positions
    for r1, r2 in zip(sim1.robots, sim2.robots):
        assert np.allclose(r1.position, r2.position)
    
    # And same trajectory
    for _ in range(10):
        sim1.step()
        sim2.step()
    
    for r1, r2 in zip(sim1.robots, sim2.robots):
        assert np.allclose(r1.position, r2.position, atol=1e-10)


def test_simulation_iteration():
    """Test simulation iteration interface."""
    sim_config = SimulationConfig(num_robots=5, seed=42, verbose=False)
    sim = HybridUnderwaterSimulation(sim_config, ControlConfig(), ConsensusConfig())
    
    steps = list(sim.iterate(10))
    assert len(steps) == 10
    assert all("decision" in step for step in steps)


if __name__ == "__main__":
    import pytest
    pytest.main([__file__, "-v"])
