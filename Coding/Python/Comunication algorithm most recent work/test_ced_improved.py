from config.scenarios import get_scenario
from Critical_edge_detection_baseline.critical_edge_simulation import CriticalEdgeSimulation
import numpy as np

scenario = 'D'
seed = 1
max_steps = 150

sim_config, control_config, consensus_config, flow_params, scenario_name = get_scenario(scenario)
sim_config.seed = seed

# Use adaptive update frequency
update_freq = max(5, 10 - sim_config.num_robots // 5)

print(f'Testing IMPROVED CriticalEdgeDetection on Scenario D')
print(f'Config: {sim_config.num_robots} robots')
print(f'Update frequency: every {update_freq} steps (adaptive to network size)\n')

sim = CriticalEdgeSimulation(
    sim_config, control_config, consensus_config,
    update_frequency=update_freq,
    prefer_critical=True,
    verbose=False
)

print(f'Initial: {sum(len(neighbors) for neighbors in sim.topology_manager.neighbor_graph) // 2} edges')
print(f'Target: {sim.num_robots - 1} edges\n')

for step in range(max_steps):
    sim.step()
    current_edges = sum(len(neighbors) for neighbors in sim.topology_manager.neighbor_graph) // 2
    
    if step % 10 == 0 or current_edges <= sim.num_robots - 1:
        print(f'Step {step:3d}: {current_edges:3d} edges', end='')
        if current_edges <= sim.num_robots - 1:
            print(f' - CONVERGED!')
            break
        print()

if current_edges > sim.num_robots - 1:
    print(f'\nAfter {max_steps} steps: {current_edges} edges (improvement needed)')
