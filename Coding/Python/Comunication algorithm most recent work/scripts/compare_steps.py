"""Compare hybrid method at different step counts."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from config.scenarios import get_scenario
from baselines.baseline_simulation import BaselineSimulation

print("Comparing hybrid method at different step counts...")
print("="*70)

for num_steps in [30, 50, 100]:
    sim_cfg, ctrl_cfg, cons_cfg, fp, n = get_scenario('A')
    sim = BaselineSimulation(sim_cfg, ctrl_cfg, cons_cfg, 'hybrid')
    
    for _ in range(num_steps):
        sim.step()
    
    summary = sim.metrics.get_summary_statistics()
    print(f"{num_steps:3d} steps: Edges={summary['final_edges']:2d}, "
          f"Pruned={summary['pruning_events']:2d}, "
          f"Lambda2={summary['min_lambda2']:.3f}")

print("="*70)
