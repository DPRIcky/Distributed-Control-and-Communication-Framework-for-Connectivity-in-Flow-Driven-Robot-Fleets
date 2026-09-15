"""Debug script to test pruning behavior."""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from config import SimulationConfig, ControlConfig
from distributed_pruning import PrunedSimulation

# Configuration
sim_config = SimulationConfig(
    num_robots=10,
    communication_radius=10.0,
    workspace_size=(10.0, 10.0),
    dt=0.05,
    seed=42
)

control_config = ControlConfig()

# Create simulation
sim = PrunedSimulation(
    sim_config,
    control_config,
    target_degree=2,
    k_connectivity=1
)

print(f"\nInitial state:")
print(f"  Potential edges: {len(sim.current_edges())}")
print(f"  Active edges: {len(sim.active_edges)}")
print(f"  Information completeness: 0% (no 2-hop info yet)")

# Run iterations with detailed output
print(f"\nProgressive information gathering and pruning:")
print(f"  (Each timestep, robots learn 5% more about their neighbors' neighbors)")
for i in range(25):  # More iterations to see progression
    sim.step()
    
# Run iterations with detailed output
print(f"\nProgressive information gathering and pruning:")
print(f"  (Each timestep, robots learn 2% more about their neighbors' neighbors)")

# Print initial state (iteration 0)
print(f"\nIteration 0 (t=0.00s) - INITIAL STATE:")
print(f"  Potential edges: {len(sim.current_edges())}")
print(f"  Active edges: {len(sim.active_edges)} (all edges active)")
print(f"  Avg info: 0%, Avg aggr: 0%")

for i in range(50):  # Enough to see  full progression to 100%
    sim.step()
    
    # Print progress every 10 iterations
    if (i+1) % 10 == 0 or i < 3:
        stats = sim.get_pruning_statistics()
        if stats:
            latest = stats[-1]
            print(f"\nIteration {i+1} (t={latest['time']:.2f}s):")
            print(f"  Active edges: {latest['active_edges']}/{latest['total_potential_edges']}", end="")
            print(f", Pruned: {latest['total_potential_edges'] - latest['active_edges']}")
            print(f"  Avg degree: {latest['avg_degree']:.2f} (target: 2)")
            print(f"  Avg information: {latest['avg_completeness']:.0%} (should be ~{min(100, (i+1)*2)}%)")
            print(f"  Avg aggressiveness: {latest['avg_aggressiveness']:.0%}")

print(f"\n\nFinal state:")
print(f"  Active edges: {len(sim.active_edges)}")
print(f"  Target edges: ~{sim.num_robots - 1} (minimal spanning tree)")
