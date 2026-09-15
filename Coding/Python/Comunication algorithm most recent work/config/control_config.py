"""Control configuration parameters."""

from dataclasses import dataclass


@dataclass
class ControlConfig:
    """Configuration for CLF/CBF control parameters."""

    safety_distance: float = 1.2
    clf_gain: float = 0.8
    cbf_safety_gain: float = 4.0
    cbf_connectivity_gain: float = 0.5
    max_control_force: float = 1.0

    def __post_init__(self):
        """Validate configuration."""
        if self.safety_distance <= 0:
            raise ValueError("Safety distance must be positive.")
        if self.max_control_force <= 0:
            raise ValueError("Max control force must be positive.")
