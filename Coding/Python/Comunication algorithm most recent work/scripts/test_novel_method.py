"""Quick test to verify novel distributed method is being used."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from config.scenarios import get_scenario
from baselines.baseline_simulation import BaselineSimulation

# Create simulation with hybrid method
sim_cfg, ctrl_cfg, cons_cfg, fp, n = get_scenario('A')
sim_cfg.verbose = True  # Enable verbose output
sim = BaselineSimulation(sim_cfg, ctrl_cfg, cons_cfg, 'hybrid')

print("Testing NOVEL distributed consensus method...")
print(f"Initial edges: {len(sim.topology_manager.get_current_edges())}")

# Run simulation
for i in range(100):  # Run longer
    sim.step()
    if i % 20 == 0:
        edges = len(sim.topology_manager.get_current_edges())
        print(f"Step {i}: {edges} edges")

# Summary
summary = sim.metrics.get_summary_statistics()
print(f"\nFinal edges: {summary['final_edges']}")
print(f"Pruning events: {summary['pruning_events']}")
print(f"Min lambda2: {summary['min_lambda2']:.3f}")
