"""Simulation scenario configurations for ACC 2026 experiments."""

from config import SimulationConfig, ControlConfig, ConsensusConfig


def get_scenario_A():
    """Scenario A: Baseline (moderate conditions).
    
    Returns:
        Tuple of (sim_config, control_config, consensus_config, flow_params, name)
    """
    sim_config = SimulationConfig(
        num_robots=10,
        communication_radius=3.0,  # Same as your working simulation
        workspace_size=(10.0, 10.0),
        dt=0.05,  # Same as default (not 0.2!)
        seed=42,
        verbose=False
    )
    
    control_config = ControlConfig()
    consensus_config = ConsensusConfig()
    
    # Moderate flow
    flow_params = {
        'base_flow': 0.3,  # m/s
        'flow_scale': 0.3
    }
    
    return sim_config, control_config, consensus_config, flow_params, "Scenario_A_Baseline"


def get_scenario_B():
    """Scenario B: Large timestep (testing temporal discretization).
    
    Returns:
        Tuple of (sim_config, control_config, consensus_config, flow_params, name)
    """
    sim_config = SimulationConfig(
        num_robots=10,
        communication_radius=3.5,  # Slightly larger to compensate
        workspace_size=(10.0, 10.0),
        dt=0.1,  # 2x larger than baseline!
        seed=42,
        verbose=False
    )
    
    # Tuned: Much stronger CBF gains for large timestep
    control_config = ControlConfig(
        cbf_connectivity_gain=1.5,  # Much higher
        cbf_safety_gain=8.0,  # Much higher
        max_control_force=2.0  # Increased authority
    )
    consensus_config = ConsensusConfig()
    
    flow_params = {
        'base_flow': 0.3,
        'flow_scale': 0.3
    }
    
    return sim_config, control_config, consensus_config, flow_params, "Scenario_B_HighCount"


def get_scenario_C():
    """Scenario C: Tight communication radius (testing sparse connectivity).
    
    Returns:
        Tuple of (sim_config, control_config, consensus_config, flow_params, name)
    """
    sim_config = SimulationConfig(
        num_robots=10,
        communication_radius=2.0,  # 33% smaller than baseline!
        workspace_size=(10.0, 10.0),
        dt=0.03,  # Smaller timestep to help maintain edges
        seed=42,
        verbose=False
    )
    
    # Tuned: Prioritize connectivity preservation
    control_config = ControlConfig(
        clf_gain=0.5,  # Reduced to prioritize connectivity
        cbf_connectivity_gain=2.0,  # Very high
        cbf_safety_gain=6.0,
        max_control_force=2.0  # High authority
    )
    consensus_config = ConsensusConfig()
    
    flow_params = {
        'base_flow': 0.3,
        'flow_scale': 0.3
    }
    
    return sim_config, control_config, consensus_config, flow_params, "Scenario_C_TightRadius"


def get_scenario_D():
    """Scenario D: High robot count (testing scalability).
    
    Returns:
        Tuple of (sim_config, control_config, consensus_config, flow_params, name)
    """
    sim_config = SimulationConfig(
        num_robots=20,  # 2x baseline!
        communication_radius=3.0,
        workspace_size=(15.0, 15.0),  # Larger workspace
        dt=0.05,
        seed=42,
        verbose=False
    )
    
    # Tuned: Handle many neighbors
    control_config = ControlConfig(
        cbf_connectivity_gain=1.0,  # Higher for many edges
        cbf_safety_gain=5.0,  # Higher for many neighbors
        max_control_force=1.8  # More authority
    )
    consensus_config = ConsensusConfig()
    
    flow_params = {
        'base_flow': 0.3,
        'flow_scale': 0.3,
        'diffusion_sigma': 0.02  # Increased 2.5x
    }
    
    return sim_config, control_config, consensus_config, flow_params, "Scenario_D_HighNoise"


def get_scenario_E():
    """Scenario E: Extreme case - tight radius with larger timestep.
    
    Returns:
        Tuple of (sim_config, control_config, consensus_config, flow_params, name)
    """
    sim_config = SimulationConfig(
        num_robots=12,
        communication_radius=2.2,  # Tight radius
        workspace_size=(10.0, 10.0),
        dt=0.08,  # Larger timestep (challenging combination!)
        seed=42,
        verbose=False
    )
    
    # Tuned: Aggressive connectivity preservation
    control_config = ControlConfig(
        clf_gain=0.4,  # Low to prioritize safety/connectivity
        cbf_connectivity_gain=2.5,  # Very aggressive
        cbf_safety_gain=7.0,  # Very high
        max_control_force=2.5  # Maximum authority
    )
    consensus_config = ConsensusConfig()
    
    flow_params = {
        'base_flow': 0.3,
        'flow_scale': 0.3
    }
    
    return sim_config, control_config, consensus_config, flow_params, "Scenario_E_Extreme"


# Scenario registry
SCENARIOS = {
    'A': get_scenario_A,
    'B': get_scenario_B,
    'C': get_scenario_C,
    'D': get_scenario_D,
    'E': get_scenario_E,
}


def get_scenario(name: str):
    """Get scenario configuration by name.
    
    Args:
        name: Scenario name ('A', 'B', 'C', 'D', or 'E')
        
    Returns:
        Tuple of (sim_config, control_config, consensus_config, flow_params, name)
        
    Raises:
        ValueError: If scenario name is invalid
    """
    if name not in SCENARIOS:
        raise ValueError(f"Unknown scenario: {name}. Choose from {list(SCENARIOS.keys())}")
    return SCENARIOS[name]()
