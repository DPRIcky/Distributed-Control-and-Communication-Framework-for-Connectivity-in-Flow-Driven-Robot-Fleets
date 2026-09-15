"""Consensus configuration parameters."""

from dataclasses import dataclass


@dataclass
class ConsensusConfig:
    """Configuration for hybrid consensus parameters."""

    # Legacy pruning parameters
    stability_threshold: int = 5
    rolling_window_size: int = 10
    consensus_rounds_per_step: int = 3
    total_consensus_rounds_needed: int = 50
    
    # New adjacency matrix consensus parameters (from Griparic et al. 2022)
    consensus_sigma: float = 1.0  # Trust function sensitivity (Eq. 23)
    consensus_sample_time: float = 0.2  # Discrete time step T_d (Eq. 34)
    consensus_convergence_epsilon: float = 1e-4  # Convergence threshold
    
    # Algebraic connectivity parameters
    lambda2_threshold: float = 0.1  # Minimum λ₂ to maintain
    lambda2_safety_margin: float = 0.05  # Recompute exact when within margin
    lambda2_max_incremental: int = 5  # Max incremental updates before exact
    
    # Edge quality threshold
    edge_quality_threshold: float = 0.01  # Min quality to consider edge present (lowered for weak consensus estimates)

    def __post_init__(self):
        """Validate configuration."""
        if self.stability_threshold < 1:
            raise ValueError("Stability threshold must be at least 1.")
        if self.rolling_window_size < 1:
            raise ValueError("Rolling window size must be at least 1.")
        if self.consensus_rounds_per_step < 1:
            raise ValueError("Consensus rounds per step must be at least 1.")
        
        # Validate new consensus parameters
        if self.consensus_sigma <= 0:
            raise ValueError("Consensus sigma must be positive.")
        if self.consensus_sample_time <= 0:
            raise ValueError("Consensus sample time must be positive.")
        if self.consensus_convergence_epsilon <= 0:
            raise ValueError("Convergence epsilon must be positive.")
        if self.lambda2_threshold < 0:
            raise ValueError("Lambda2 threshold must be non-negative.")
        if self.edge_quality_threshold < 0 or self.edge_quality_threshold > 1:
            raise ValueError("Edge quality threshold must be in [0, 1].")
