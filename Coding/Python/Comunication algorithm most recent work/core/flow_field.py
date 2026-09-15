"""Underwater flow field model."""

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class FlowField:
    """Simple 2‑D current: constant drift plus gentle sinusoidal swirl."""

    base_vector: np.ndarray = field(
        default_factory=lambda: np.array([0.20, 0.05], dtype=float)
    )
    swirl_amplitude: float = 0.10
    swirl_scale: float = 5.5

    def velocity(self, position: np.ndarray, time: float) -> np.ndarray:
        """Compute flow velocity at the given position and time.
        
        Args:
            position: 2D position array
            time: Current simulation time
            
        Returns:
            2D velocity vector
        """
        swirl = self.swirl_amplitude * np.array(
            [
                math.sin((position[1] + time) / self.swirl_scale),
                math.cos((position[0] + 0.3 * time) / self.swirl_scale),
            ]
        )
        return self.base_vector + swirl
