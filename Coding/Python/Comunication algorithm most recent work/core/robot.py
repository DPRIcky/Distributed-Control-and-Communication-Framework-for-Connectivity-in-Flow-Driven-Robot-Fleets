"""Robot model with underwater dynamics."""

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from core.flow_field import FlowField
from core.types import Edge


@dataclass
class ChainRobot:
    """Single-integrator robot advected by the flow with mild diffusion."""

    robot_id: int
    position: np.ndarray
    rng: np.random.Generator
    diffusion: float = 0.008
    mobility: float = 0.45
    removed_edges: List[Edge] = field(default_factory=list)

    mass: float = field(default_factory=lambda: 2.0 + np.random.normal(0.0, 0.2))
    drag_coefficient: float = field(default_factory=lambda: 0.8 + np.random.normal(0.0, 0.1))
    cross_sectional_area: float = field(default_factory=lambda: 0.01 + np.random.normal(0.0, 0.002))

    velocity: np.ndarray = field(default_factory=lambda: np.zeros(2))
    parent: Optional[int] = None

    def __post_init__(self):
        """Initialize physical properties."""
        self.volume = self.mass / 1800.0

    def step(
        self,
        flow: FlowField,
        dt: float,
        time: float,
        bounds: np.ndarray,
        control_force: Optional[np.ndarray] = None
    ) -> None:
        """Update position with flow, diffusion, and control input.
        
        Args:
            flow: Flow field model
            dt: Time step
            time: Current simulation time
            bounds: Workspace boundaries
            control_force: Optional control input
        """
        flow_velocity = flow.velocity(self.position, time)

        if control_force is not None:
            self.velocity += control_force * dt

        agitation = self.rng.normal(0.0, self.diffusion, size=2)
        self.velocity += agitation * self.mobility

        self.velocity *= 0.85

        self.position = self.position + dt * (flow_velocity + self.velocity)
        self.position = np.clip(self.position, [0.0, 0.0], bounds)
        self.removed_edges.clear()
