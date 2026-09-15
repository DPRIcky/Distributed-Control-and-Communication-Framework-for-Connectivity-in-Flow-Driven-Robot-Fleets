"""Visualization configuration parameters."""

from dataclasses import dataclass


@dataclass
class VisualizationConfig:
    """Configuration for visualization settings."""

    interval_ms: int = 100
    figsize: tuple = (10, 5)
    flow_grid_points: int = 10
    flow_alpha: float = 0.4
    flow_width: float = 0.0015
    flow_scale: float = 2.0
    
    node_size: int = 80
    active_linewidth: float = 2.5
    candidate_linewidth: float = 3.5
    highlight_linewidth: float = 4.5

    def __post_init__(self):
        """Validate configuration."""
        if self.interval_ms <= 0:
            raise ValueError("Interval must be positive.")
