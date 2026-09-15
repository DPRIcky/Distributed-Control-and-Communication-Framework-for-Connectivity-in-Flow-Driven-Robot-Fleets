from config.scenarios import get_scenario
from Critical_edge_detection_baseline.critical_edge_simulation import CriticalEdgeSimulation
import numpy as np

scenario = 'D'
seed = 1
max_steps = 300

sim_config, control_config, consensus_config, flow_params, scenario_name = get_scenario(scenario)
sim_config.seed = seed

print(f'Testing CriticalEdgeDetection on Scenario D (High Robot Count)')
print(f'Config: {sim_config.num_robots} robots, {sim_config.communication_radius} radius\n')

sim = CriticalEdgeSimulation(
    sim_config, control_config, consensus_config,
    update_frequency=10,
    prefer_critical=True,
    verbose=False
)

print(f'Initial state:')
print(f'  Robots: {sim.num_robots}')
print(f'  Initial edges: {sum(len(neighbors) for neighbors in sim.topology_manager.neighbor_graph) // 2}')
print(f'  Target spanning tree edges: {sim.num_robots - 1}\n')

for step in range(min(150, max_steps)):
    sim.step()
    
    current_edges = sum(len(neighbors) for neighbors in sim.topology_manager.neighbor_graph) // 2
    
    # Print progress every 10 steps
    if step % 10 == 0 or current_edges <= sim.num_robots - 1:
        print(f'Step {step:3d}: edges={current_edges:3d}, reached_tree={current_edges <= sim.num_robots - 1}')
    
    # Stop if converged
    if current_edges <= sim.num_robots - 1:
        print(f'\n✓ Converged to spanning tree at step {step}!')
        break
else:
    print(f'\n✗ Did not converge after {max_steps} steps')
    print(f'Final edges: {current_edges}')
