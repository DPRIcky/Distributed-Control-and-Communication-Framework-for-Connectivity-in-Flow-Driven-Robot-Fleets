"""Simulation configuration parameters."""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class SimulationConfig:
    """Configuration for main simulation parameters."""

    num_robots: int = 10
    communication_radius: float = 3.0
    dt: float = 0.05
    workspace_size: Tuple[float, float] = (10.0, 10.0)
    seed: Optional[int] = None
    verbose: bool = True
    goal_position: Optional[np.ndarray] = None

    def __post_init__(self):
        """Validate configuration."""
        if self.num_robots < 3:
            raise ValueError("Need at least 3 robots for cycle formation.")
        if self.dt <= 0:
            raise ValueError("Time step must be positive.")
        if self.communication_radius <= 0:
            raise ValueError("Communication radius must be positive.")
