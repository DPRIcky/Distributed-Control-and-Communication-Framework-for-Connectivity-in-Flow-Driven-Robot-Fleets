"""Baseline simulation module for fully connected graph simulation.

This module provides a standard, reusable baseline simulation environment that can be
easily imported and extended for testing different algorithms (GHS, consensus pruning, etc.).

Usage:
    from baseline_simulation import FullyConnectedSimulation, FullyConnectedAnimator
    
    sim = FullyConnectedSimulation(sim_config, control_config)
    animator = FullyConnectedAnimator(sim, vis_config)
    anim = animator.animate()
    plt.show()

Or extend it:
    class MyCustomSimulation(FullyConnectedSimulation):
        def step(self):
            # Your custom step logic
            super().step()
            # Additional processing
"""

from .fully_connected_simulation import FullyConnectedSimulation
from .fully_connected_animator import FullyConnectedAnimator

__all__ = ['FullyConnectedSimulation', 'FullyConnectedAnimator']
