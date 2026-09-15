# Using FullyConnectedSimulation with Critical Edge Pruning

## Quick Start

### 1. **Import into your scripts folder**

```python
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from baseline_simulation import FullyConnectedSimulation, FullyConnectedAnimator
from config import SimulationConfig, ControlConfig, VisualizationConfig
```

### 2. **Extend with your custom pruning method**

```python
class MyCustomPruningSimulation(FullyConnectedSimulation):
    """Your custom implementation with critical edge detection."""
    
    def __init__(self, sim_config, control_config):
        super().__init__(sim_config, control_config)
        # Initialize your algorithm-specific attributes
        
    def step(self):
        """Override step() to add pruning logic."""
        super().step()  # Run parent step
        # Your custom pruning logic here
```

## Example Usage Patterns

### Pattern 1: Direct Extension (Recommended)

```python
from critical_edge_pruning_simulation import CriticalEdgePruningSimulation

# Create configurations
sim_config = SimulationConfig(num_robots=10, communication_radius=3.5)
control_config = ControlConfig()

# Create simulation with pruning
sim = CriticalEdgePruningSimulation(
    sim_config, 
    control_config,
    enable_pruning=True,
    pruning_start_time=1.0  # Start pruning after 1 second
)

# Run simulation
for _ in range(100):
    sim.step()
    state = sim.get_state()
    
    print(f"Edges: {state['num_edges']}, "
          f"Pruned: {state['total_pruned']}, "
          f"Critical: {len(state['critical_edges'])}")
```

### Pattern 2: Integrate with Distributed Algorithm

```python
from critical_edge_pruning_simulation import CriticalEdgePruningSimulation
from distributed_pruning_algorithm import DistributedPruningAlgorithm

sim = CriticalEdgePruningSimulation(sim_config, control_config)

# Run steps
for step in range(500):
    sim.step()
    
    # Extract current topology as networkx graph
    edges = sim.current_edges()
    positions = sim.get_state()['positions']
    
    # Run distributed algorithm on current topology
    if step % 20 == 0:  # Every 20 steps
        G = create_networkx_graph(edges, positions)  # Your conversion function
        algo = DistributedPruningAlgorithm(G, root=0)
        pruned_edges = algo.run_full_pipeline()
        
        # Apply pruning to simulation
        for i, j in sim.current_edges():
            if (i, j) not in pruned_edges:
                sim.topology_manager.neighbor_graph[i].discard(j)
                sim.topology_manager.neighbor_graph[j].discard(i)
```

### Pattern 3: Visualize with Real-Time Metrics

```python
from critical_edge_pruning_simulation import CriticalEdgePruningSimulation
from baseline_simulation import FullyConnectedAnimator
import matplotlib.pyplot as plt

sim = CriticalEdgePruningSimulation(sim_config, control_config)
animator = FullyConnectedAnimator(sim, vis_config)

# Get animation
anim = animator.animate()

# The animator will show:
# - Left panel: Robots + edges (critical edges in different color)
# - Right panel: Edge count metrics

plt.show()
```

## Key Methods & Attributes

### CriticalEdgePruningSimulation Methods

| Method | Purpose |
|--------|---------|
| `detect_critical_edges()` | Identify bridges in topology |
| `prune_non_critical_edges()` | Remove redundant edges |
| `_is_connected()` | Check if graph remains connected |
| `step()` | Run one simulation step with pruning |
| `get_state()` | Get state including pruning info |

### Available Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `critical_edges` | Set | Current bridge edges |
| `pruned_edges` | Set | Edges that have been removed |
| `enable_pruning` | bool | Whether pruning is active |
| `pruning_start_time` | float | When to start pruning (seconds) |
| `edge_pruning_threshold` | float | Distance threshold for pruning |
| `robots` | List[ChainRobot] | Robot instances |
| `topology_manager` | TopologyManager | Neighbor graph manager |

## Integration with Your Distributed Algorithm

To integrate with `distributed_pruning_algorithm.py`:

```python
import networkx as nx
from distributed_pruning_algorithm import DistributedPruningAlgorithm

def simulate_with_distributed_pruning(sim, duration=10.0, algorithm_interval=0.5):
    """Run simulation with periodic distributed pruning."""
    
    last_algorithm_run = 0
    
    while sim.time < duration:
        sim.step()
        
        # Run distributed algorithm periodically
        if sim.time - last_algorithm_run >= algorithm_interval:
            # Convert current topology to networkx graph
            edges = sim.current_edges()
            G = nx.Graph()
            G.add_nodes_from(range(sim.num_robots))
            G.add_edges_from(edges)
            
            # Run algorithm
            if nx.is_connected(G) and len(edges) > sim.num_robots - 1:
                algo = DistributedPruningAlgorithm(G, root=0)
                algo.run_full_pipeline()
                critical_edges = algo.get_critical_edges()
                
                # Prune non-critical edges
                for i, j in edges:
                    if (i, j) not in critical_edges and (j, i) not in critical_edges:
                        sim.topology_manager.neighbor_graph[i].discard(j)
                        sim.topology_manager.neighbor_graph[j].discard(i)
                        sim.pruned_edges.add((i, j) if i < j else (j, i))
            
            last_algorithm_run = sim.time
```

## Files Reference

| File | Purpose |
|------|---------|
| `baseline_simulation/fully_connected_simulation.py` | Base simulation class |
| `baseline_simulation/fully_connected_animator.py` | Visualization |
| `critical_edge_pruning_simulation.py` | Extended with pruning |
| `distributed_pruning_algorithm.py` | Your distributed algorithm |

## Tips

1. **Always call parent `step()`** - Ensure `super().step()` is called first to maintain dynamics
2. **Set pruning_start_time** - Let robots settle before pruning to avoid connectivity loss
3. **Monitor critical edges** - Check `get_state()['critical_edges']` to validate algorithm
4. **Test connectivity** - Use `_is_connected()` to verify graph remains valid
5. **Distance thresholds** - Tune `edge_pruning_threshold` based on your workspace and controller gains
