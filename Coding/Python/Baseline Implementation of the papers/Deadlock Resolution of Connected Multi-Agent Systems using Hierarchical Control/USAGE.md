# Usage Guide

## Quick Start

### Installation

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Run a simulation:
```bash
python main.py --scenario apartment
```

## Command Line Arguments

```bash
python main.py [OPTIONS]
```

### Available Options:

- `--scenario {simple,apartment,crossing,narrow}`: Choose the scenario to simulate (default: apartment)
- `--max_time FLOAT`: Maximum simulation time in seconds (default: 30.0)
- `--no_planner`: Disable high-level planner for comparison
- `--no_bidding`: Disable bidding mechanism for edge deletion
- `--no_viz`: Disable real-time visualization
- `--dt FLOAT`: Time step in seconds (default: 0.1)

## Example Usage

### 1. Run apartment scenario with full hierarchical control:
```bash
python main.py --scenario apartment --max_time 40
```

### 2. Run without high-level planner (for comparison):
```bash
python main.py --scenario apartment --no_planner
```

This will show how the system gets stuck in deadlocks without the hierarchical planner.

### 3. Run crossing scenario:
```bash
python main.py --scenario crossing --max_time 50
```

### 4. Run narrow passage scenario (24 agents):
```bash
python main.py --scenario narrow --max_time 60
```

### 5. Run simple test scenario:
```bash
python main.py --scenario simple --max_time 20
```

## Understanding the Output

The simulation will display:
- Real-time visualization of agents (blue circles), leaders (red circles), and goals (green stars)
- Network connectivity shown as gray lines between agents
- Progress updates showing average distance to goal and average speed
- Final statistics including success rate and computation times

### Visual Elements:
- **Blue circles**: Regular agents
- **Red circles**: Leader agents (during deadlock resolution)
- **Green stars**: Goal locations
- **Gray lines**: Network connectivity edges
- **Black polygons**: Obstacles
- **Arrows**: Agent orientations

## Scenarios Description

### Simple (3 agents)
A basic test scenario with 3 agents and one obstacle. Useful for debugging and understanding the system behavior.

### Apartment (5 agents)
Warehouse/apartment environment where agents navigate through rooms with obstacles. Tests deadlock resolution around corners.

### Crossing (10 agents)
Two groups of agents cross paths diagonally. Tests inter-group interaction and deadlock avoidance.

### Narrow Passage (24 agents)
Four groups of 6 agents each exchange positions through narrow passages. Most challenging scenario testing scalability.

## Comparison Studies

To compare the hierarchical controller with baseline CBF-QP:

1. Run with hierarchical control:
```bash
python main.py --scenario apartment > results_hierarchical.txt
```

2. Run without high-level planner:
```bash
python main.py --scenario apartment --no_planner > results_baseline.txt
```

3. Compare the success rates and completion times

## Advanced Usage

### Creating Custom Scenarios

Add new scenarios in `scenarios/scenarios.py`:

```python
def create_custom_scenario():
    # Define initial positions [x, y, theta]
    init_positions = np.array([
        [x1, y1, theta1],
        [x2, y2, theta2],
        # ...
    ])

    # Define goals [x, y]
    goals = np.array([
        [gx1, gy1],
        [gx2, gy2],
        # ...
    ])

    # Create agents
    agents = []
    for i in range(len(init_positions)):
        agent = Agent(i, init_positions[i], goals[i])
        agents.append(agent)

    # Create obstacles
    obstacles = []
    obstacles.append(create_rectangle_obstacle(center, width, height))
    # ...

    return agents, obstacles
```

### Tuning Parameters

Key parameters can be adjusted in the code:

**In CBFQPController** ([src/control/cbf_qp_controller.py](src/control/cbf_qp_controller.py)):
- `v_max`: Maximum linear velocity (default: 0.5)
- `omega_max`: Maximum angular velocity (default: 1.0)
- `lambda_p`, `lambda_theta`: CLF convergence rates

**In HighLevelPlanner** ([src/planning/high_level_planner.py](src/planning/high_level_planner.py)):
- `u_min`: Deadlock detection threshold (default: 0.01)
- `d_min`: Minimum distance to use temporary goals (default: 0.5)

**In Agent** ([src/core/agent.py](src/core/agent.py)):
- `sensing_radius`: Communication radius (default: 1.7)
- `safe_distance`: Minimum safe distance between agents (default: 0.1)

## Troubleshooting

### QP solver issues
If OSQP solver fails, try:
```bash
pip install --upgrade cvxpy osqp
```

### Visualization issues
On some systems, interactive plotting may not work. Try:
```bash
python main.py --scenario apartment --no_viz
```

### Performance issues
For faster simulation (less accurate):
```bash
python main.py --scenario narrow --dt 0.2
```

## Theory Background

This implementation is based on the paper:

**Garg, K., Hamilton, S., & Fan, C. (2024)**. "Deadlock Resolution of Connected Multi-Agent Systems using Hierarchical Control." IEEE Conference on Decision and Control (CDC).

### Key Components:

1. **High-level Planner**: Assigns temporary goals to resolve deadlocks using leader-follower mechanism (Algorithm 1)

2. **Low-level Controller**: CBF-QP controller ensures:
   - Safety (collision avoidance)
   - Connectivity maintenance
   - Goal reaching performance

3. **Bidding Mechanism**: Intelligently removes redundant edges to improve performance while maintaining connectivity

## Performance Metrics

The simulation reports:
- **Success rate**: Percentage of agents reaching goals
- **Computation time**: Average time per control step
- **Total time**: Time to complete mission
- **Final distances**: Individual agent performance
