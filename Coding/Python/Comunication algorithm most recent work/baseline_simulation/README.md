# Baseline Simulation Module

Standard, reusable baseline simulation environment for underwater robot swarms.

## Overview

This module provides a **clean, extensible baseline** for testing different algorithms (GHS, consensus pruning, etc.) with the exact same simulation environment. No need to rebuild the simulation from scratch each time.

## Features

✓ **Fully connected graph** - All edges within communication range maintained  
✓ **CBF/CLF control** - Collision avoidance + goal-seeking  
✓ **Flow field disturbance** - Sinusoidal flow for realism  
✓ **Dual-panel visualization** - Simulation + metrics  
✓ **Extensible design** - Override methods to add custom behavior  
✓ **Matches concurrent_pruning style** - Consistent environment for comparison  

## Quick Start

### Basic Usage

```python
from baseline_simulation import FullyConnectedSimulation, FullyConnectedAnimator
from config import SimulationConfig, ControlConfig, VisualizationConfig
import matplotlib.pyplot as plt

# Configure
sim_config = SimulationConfig(num_robots=10, communication_radius=3.5)
control_config = ControlConfig()
vis_config = VisualizationConfig(interval_ms=100)

# Create and run
sim = FullyConnectedSimulation(sim_config, control_config)
animator = FullyConnectedAnimator(sim, vis_config)
anim = animator.animate()
plt.show()
```

### Extending for Custom Algorithms

```python
from baseline_simulation import FullyConnectedSimulation, FullyConnectedAnimator

class MyGHSSimulation(FullyConnectedSimulation):
    """Custom simulation with GHS algorithm."""
    
    def __init__(self, sim_config, control_config):
        super().__init__(sim_config, control_config)
        # Add your custom initialization
        self.ghs_manager = MyGHSManager(...)
    
    def step(self):
        """Override to add GHS logic."""
        # Base simulation step (robots move, topology updates)
        super().step()
        
        # Your custom GHS step
        self.ghs_manager.execute_ghs_round()
        # Prune non-MST edges
        for edge in self.ghs_manager.rejected_edges:
            self.topology_manager.prune_edge(edge, self.robots)

class MyGHSAnimator(FullyConnectedAnimator):
    """Custom animator for GHS visualization."""
    
    def _get_info_text(self, state, avg_dist, max_dist):
        """Override to show GHS-specific info."""
        return [
            f"GHS MST Discovery",
            f"Iteration: {self.sim.ghs_manager.iteration}",
            f"MST Edges: {len(self.sim.ghs_manager.mst_edges)}",
            f"",
            f"Avg Dist: {avg_dist:.2f}m",
        ]

# Use your custom classes
sim = MyGHSSimulation(sim_config, control_config)
animator = MyGHSAnimator(sim, vis_config)
anim = animator.animate()
plt.show()
```

## Module Structure

```
baseline_simulation/
├── __init__.py                      # Package initialization
├── fully_connected_simulation.py   # Core simulation class
├── fully_connected_animator.py     # Visualization class
└── README.md                        # This file
```

## Classes

### `FullyConnectedSimulation`

Core simulation class with standard environment.

**Key Methods:**
- `__init__(sim_config, control_config)` - Initialize simulation
- `step()` - Execute one simulation step (override for custom behavior)
- `current_edges()` - Get list of active edges
- `get_state()` - Get current state dict for visualization

**Key Attributes:**
- `robots` - List of ChainRobot instances
- `topology_manager` - TopologyManager instance
- `controller` - HybridCLFCBFController instance
- `flow` - FlowField instance
- `time` - Current simulation time

### `FullyConnectedAnimator`

Standard animator with dual-panel layout matching concurrent_pruning style.

**Key Methods:**
- `__init__(simulation, vis_config)` - Initialize animator
- `animate()` - Create and return FuncAnimation
- `update(frame)` - Update one frame (override for custom viz)
- `_get_info_text(state, avg_dist, max_dist)` - Get info text (override for custom info)

**Panels:**
- **Left**: Robot simulation with flow field, edges, goal
- **Right**: Edge count + average distance to goal

## Configuration

Uses standard config classes:
- `SimulationConfig` - Robots, comm radius, workspace, etc.
- `ControlConfig` - CBF/CLF parameters
- `VisualizationConfig` - Animation interval, styling

## Examples

See `Minimum_Spanning_Tree_new_mwthond/gui_fully_connected.py` for a complete example.

## Design Philosophy

1. **Inheritance over reimplementation** - Extend base classes instead of copying code
2. **Override key methods** - `step()` for simulation logic, `update()` for visualization
3. **Consistent environment** - Same config, same layout, same style across all algorithms
4. **Easy comparison** - Run baseline and custom side-by-side with identical settings

## Future Extensions

This module can be extended for:
- GHS distributed MST discovery
- Reverse-delete pruning
- Consensus-based pruning
- Concurrent pruning algorithms
- Custom topology algorithms
- Any distributed algorithm requiring standard swarm environment

## Notes

- Matches the exact style and configuration of `concurrent_pruning/gui_simulation_concurrent.py`
- Uses the same dual-panel layout: simulation (left) + metrics (right)
- Flow field, robot labels, edge styling all consistent
- Communication radius: 3.5m (standard)
- Easy to compare different algorithms with identical baseline
