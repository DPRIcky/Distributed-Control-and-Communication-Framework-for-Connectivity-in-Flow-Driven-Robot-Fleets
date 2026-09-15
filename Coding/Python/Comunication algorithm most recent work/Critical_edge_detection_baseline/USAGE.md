# Critical Edge Detection Baseline - Usage Examples

Quick reference guide for running simulations with the critical edge detection baseline.

## Quick Start

### 1. Run with Default Parameters
```bash
cd Critical_edge_detection_baseline
python run_simulation.py
```

### 2. Custom Robot Count
```bash
python run_simulation.py --robots 15
```

### 3. Longer Simulation
```bash
python run_simulation.py --steps 1000
```

### 4. Save Results
```bash
python run_simulation.py --save results/my_simulation.npz
```

### 5. Quiet Mode (No Output)
```bash
python run_simulation.py --quiet --save results/batch_run.npz
```

### 6. Verbose Mode (Detailed Output)
```bash
python run_simulation.py --verbose
```

## Common Usage Patterns

### Small Test Run
```bash
python run_simulation.py --robots 5 --steps 100
```

### Production Run (More Robots, Longer)
```bash
python run_simulation.py --robots 20 --steps 2000 --save results/production_run.npz
```

### Reproducible Run (Fixed Seed)
```bash
python run_simulation.py --seed 42 --save results/reproducible.npz
```

### Different Communication Radius
```bash
python run_simulation.py --comm-radius 5.0
```

### Larger Workspace
```bash
python run_simulation.py --workspace 20.0 20.0
```

### Different MST Update Frequency
```bash
# Update MST every 5 steps (more frequent)
python run_simulation.py --update-freq 5

# Update MST every 20 steps (less frequent)
python run_simulation.py --update-freq 20
```

### Without Critical Edge Priority
```bash
python run_simulation.py --no-prefer-critical
```

## Batch Processing

### Run Multiple Simulations
```bash
# Run with different robot counts
for n in 5 10 15 20; do
    python run_simulation.py --robots $n --quiet --save results/robots_${n}.npz
done

# Run with different communication radii
for r in 2.0 3.0 4.0 5.0; do
    python run_simulation.py --comm-radius $r --quiet --save results/radius_${r}.npz
done
```

## Python API Usage

### Using in Your Own Script

```python
import sys
sys.path.insert(0, '..')

from config import SimulationConfig, ControlConfig
from Critical_edge_detection_baseline.simple_simulation import CriticalEdgeSimulation

# Create configuration
sim_config = SimulationConfig(
    num_robots=10,
    communication_radius=3.0,
    dt=0.05,
    workspace_size=(10.0, 10.0),
    seed=42
)

control_config = ControlConfig()

# Create simulation
sim = CriticalEdgeSimulation(
    sim_config=sim_config,
    control_config=control_config,
    update_frequency=10,
    prefer_critical=True
)

# Run simulation
for step in range(500):
    sim.step()
    
    if step % 100 == 0:
        print(f"Step {step}: {sim.total_edges} edges")

# Print statistics
sim.print_statistics()
```

## Loading and Analyzing Saved Results

```python
import numpy as np
import matplotlib.pyplot as plt

# Load results
data = np.load('results/my_simulation.npz', allow_pickle=True)

# Extract data
time = data['time']
edges = data['edges']
mst_updates = data['mst_updates']
positions = data['positions']

# Plot edge count over time
plt.figure(figsize=(10, 6))
plt.plot(time, edges, 'b-', linewidth=2, label='Active edges')
plt.axhline(y=data['config'].item()['num_robots'] - 1, 
           color='r', linestyle='--', label='MST size')
plt.xlabel('Time (s)')
plt.ylabel('Number of Edges')
plt.title('Communication Graph Evolution')
plt.legend()
plt.grid(True)
plt.savefig('edge_evolution.png', dpi=300, bbox_inches='tight')
plt.show()

# Plot robot trajectories
plt.figure(figsize=(8, 8))
for robot_id in range(data['config'].item()['num_robots']):
    trajectory = positions[:, robot_id, :]
    plt.plot(trajectory[:, 0], trajectory[:, 1], alpha=0.6)
    plt.scatter(trajectory[0, 0], trajectory[0, 1], s=100, marker='o')  # Start
    plt.scatter(trajectory[-1, 0], trajectory[-1, 1], s=100, marker='s')  # End
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Robot Trajectories')
plt.grid(True)
plt.axis('equal')
plt.savefig('trajectories.png', dpi=300, bbox_inches='tight')
plt.show()
```

## Quick Tests

### Test Installation
```bash
python test_critical_baseline.py
```

### Quick Demo
```bash
python demo.py
```

### Simple Test Simulation
```bash
python simple_simulation.py
```

## Full Command Reference

```
usage: run_simulation.py [-h] [--robots ROBOTS] [--steps STEPS] [--dt DT] 
                         [--comm-radius COMM_RADIUS] 
                         [--workspace WORKSPACE WORKSPACE]
                         [--update-freq UPDATE_FREQ] [--no-prefer-critical] 
                         [--save SAVE] [--verbose] [--quiet] 
                         [--visualize] [--plot-interval PLOT_INTERVAL] 
                         [--seed SEED]

Arguments:
  --robots ROBOTS              Number of robots (default: 10)
  --steps STEPS                Number of simulation steps (default: 500)
  --dt DT                      Time step in seconds (default: 0.05)
  --comm-radius COMM_RADIUS    Communication radius (default: 3.0)
  --workspace WIDTH HEIGHT     Workspace size (default: [10.0, 10.0])
  --update-freq UPDATE_FREQ    MST update frequency (default: 10)
  --no-prefer-critical         Do not prioritize critical edges
  --save SAVE                  Save results to .npz file
  --verbose                    Print detailed output
  --quiet                      Minimal output
  --visualize                  Enable real-time visualization
  --plot-interval INTERVAL     Visualization update interval (default: 50)
  --seed SEED                  Random seed for reproducibility
```

## Tips

1. **Start small**: Test with fewer robots (5-8) and fewer steps (100-200) first
2. **Use seeds**: Use `--seed` for reproducible results
3. **Save results**: Always save important runs with `--save`
4. **Quiet mode**: Use `--quiet` for batch processing
5. **Update frequency**: Lower `--update-freq` for more frequent MST updates (higher accuracy, more computation)

## Troubleshooting

### Import errors
Make sure you're running from the correct directory:
```bash
cd Critical_edge_detection_baseline
python run_simulation.py
```

### Memory issues with large simulations
Reduce `--steps` or increase `--update-freq`

### Results not saving
Check that the results directory exists or use a simple filename:
```bash
python run_simulation.py --save results.npz
```
