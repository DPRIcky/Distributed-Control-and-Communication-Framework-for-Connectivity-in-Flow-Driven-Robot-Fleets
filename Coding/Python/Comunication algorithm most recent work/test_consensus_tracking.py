import sys
sys.path.insert(0, r'c:\Users\prajj\OneDrive - Arizona State University\ASU\PhD\Research\Coding\Python\Comunication algorithm most recent work')

from config.scenarios import get_scenario
from baselines.baseline_simulation import BaselineSimulation

sim_cfg, ctrl_cfg, cons_cfg, fp, _ = get_scenario('A')
sim_cfg.seed = 42
sim_cfg.verbose = False
sim = BaselineSimulation(sim_cfg, ctrl_cfg, cons_cfg, 'hybrid')

if not hasattr(sim.pruning_manager, 'last_convergence_iterations'):
    sim.pruning_manager.last_convergence_iterations = 0

print("Step | Time  | Consensus Iters | Pruning Events")
print("-----|-------|-----------------|---------------")

for step in range(30):
    sim.step()
    val = sim.pruning_manager.last_convergence_iterations if hasattr(sim.pruning_manager, 'last_convergence_iterations') else 0
    pruning_count = len(sim.metrics.pruning_events)
    print(f"{step:4d} | {sim.time:5.2f} | {val:15d} | {pruning_count:14d}")
