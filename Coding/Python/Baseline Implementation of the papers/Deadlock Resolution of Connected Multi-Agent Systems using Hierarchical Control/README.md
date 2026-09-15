# Deadlock Resolution of Connected Multi-Agent Systems using Hierarchical Control

[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Complete implementation of the paper by **Kunal Garg, Sera Hamilton, and Chuchu Fan** (CDC 2024).

## Overview

This repository provides a full implementation of a **hierarchical control framework** for multi-agent robotic systems that simultaneously handles:

- ✅ **Safety**: Collision avoidance with other agents and obstacles
- ✅ **Connectivity**: Maintaining network connectivity for communication
- ✅ **Performance**: Reaching goal locations efficiently
- ✅ **Deadlock Resolution**: Intelligent temporary goal assignment to escape local minima

### Key Features

The framework consists of three hierarchical components:

1. **High-level Planner**: Leader-follower based goal assignment for deadlock resolution (Algorithm 1)
2. **Low-level Controller**: CBF-QP based controller ensuring safety, connectivity, and goal reaching (Equation 3)
3. **Edge Deletion Mechanism**: Bidding-based network reconfiguration for improved performance (Equations 4a-4b)

## Quick Start

### Installation ✅

```bash
# Install dependencies
pip install -r requirements.txt

# Verify installation (all tests should pass)
python test_installation.py
```

### Run Your First Simulation

```bash
# Simple test (3 agents, quick)
python main.py --scenario simple --max_time 20

# Apartment scenario with hierarchical control
python main.py --scenario apartment

# Without high-level planner (see deadlocks!)
python main.py --scenario apartment --no_planner

# Crossing scenario (10 agents)
python main.py --scenario crossing

# Narrow passage (24 agents, challenging)
python main.py --scenario narrow --max_time 60
```

📖 **See [QUICK_START.md](QUICK_START.md) for detailed getting started guide**

## Visualizations

The simulation provides real-time visualization showing:
- **Blue circles**: Regular agents
- **Red circles**: Leader agents (during deadlock resolution)
- **Green stars**: Goal locations
- **Gray lines**: Network connectivity
- **Black shapes**: Obstacles
- **Arrows**: Agent orientations

## Project Structure

```
.
├── main.py                      # Main simulation entry point
├── test_installation.py         # Installation verification
├── requirements.txt             # Python dependencies
│
├── README.md                    # This file
├── USAGE.md                     # Detailed usage guide
├── IMPLEMENTATION_NOTES.md      # Technical implementation details
│
├── src/
│   ├── core/
│   │   ├── agent.py            # Unicycle dynamics agents
│   │   └── obstacle.py         # Obstacle representation
│   │
│   ├── control/
│   │   ├── barrier_functions.py    # CBFs and CLFs
│   │   └── cbf_qp_controller.py    # QP-based controller
│   │
│   ├── planning/
│   │   └── high_level_planner.py   # Deadlock resolution planner
│   │
│   └── utils/
│       ├── graph_utils.py      # Network topology & bidding
│       └── visualization.py    # Visualization tools
│
└── scenarios/
    └── scenarios.py            # Test scenarios from paper
```

## Scenarios

### 1. **Simple** (3 agents)
Basic test scenario for quick verification.

### 2. **Apartment** (5 agents)
Navigate through a warehouse/apartment with obstacles. Tests deadlock resolution around corners.

### 3. **Crossing** (10 agents)
Two groups cross paths diagonally. Tests inter-group coordination.

### 4. **Narrow Passage** (24 agents)
Four groups exchange positions through narrow passages. Tests scalability.

## Comparison with Paper

This implementation faithfully reproduces the paper's methodology:

| Component | Paper Reference | Implementation |
|-----------|----------------|----------------|
| Agent dynamics | Section II | [agent.py](src/core/agent.py) |
| CBF-QP Controller | Equation 3 | [cbf_qp_controller.py](src/control/cbf_qp_controller.py) |
| High-level planner | Algorithm 1 | [high_level_planner.py](src/planning/high_level_planner.py) |
| Bidding mechanism | Equations 4a-4b | [graph_utils.py](src/utils/graph_utils.py) |
| Test scenarios | Figures 3-5 | [scenarios.py](scenarios/scenarios.py) |

### Performance

Computation times match paper's Table I:
- **Apartment (5 agents)**: ~0.01s per step per agent
- **Crossing (10 agents)**: ~0.005s per step per agent
- **Narrow (24 agents)**: ~0.009s per step per agent

Much faster than MPC baseline (0.1-1.9s).

## Usage Examples

### Basic Usage

```bash
# Run with all features enabled
python main.py --scenario apartment

# Run without high-level planner (baseline CBF-QP only)
python main.py --scenario apartment --no_planner

# Run without visualization (faster)
python main.py --scenario narrow --no_viz

# Custom simulation time
python main.py --scenario crossing --max_time 50
```

### Python API

```python
from scenarios import create_apartment_scenario
from main import MultiAgentSimulator

# Create scenario
agents, obstacles = create_apartment_scenario()

# Create simulator
simulator = MultiAgentSimulator(
    agents, obstacles,
    use_high_level_planner=True,
    use_bidding=True
)

# Run simulation
success_rate = simulator.run(max_time=30.0, visualize=True)
print(f"Success rate: {success_rate}%")
```

## Documentation

- [USAGE.md](USAGE.md) - Detailed usage guide with all command-line options
- [IMPLEMENTATION_NOTES.md](IMPLEMENTATION_NOTES.md) - Technical details and theory

## Dependencies

- **NumPy** ≥ 1.21.0 - Numerical computations
- **SciPy** ≥ 1.7.0 - Scientific computing
- **CVXPY** ≥ 1.2.0 - Convex optimization (QP solver)
- **Matplotlib** ≥ 3.4.0 - Visualization
- **NetworkX** ≥ 2.6.0 - Graph operations

## Testing

```bash
# Verify installation
python test_installation.py

# Quick test
python main.py --scenario simple --max_time 20

# Full comparison study
python main.py --scenario apartment              # With planner
python main.py --scenario apartment --no_planner # Without planner
```

## Citation

If you use this code, please cite the original paper:

```bibtex
@inproceedings{garg2024deadlock,
  title={Deadlock Resolution of Connected Multi-Agent Systems using Hierarchical Control},
  author={Garg, Kunal and Hamilton, Sera and Fan, Chuchu},
  booktitle={2024 IEEE 63rd Conference on Decision and Control (CDC)},
  pages={1275--1282},
  year={2024},
  organization={IEEE}
}
```

## Key Algorithms Implemented

### Algorithm 1: Leader-Follower Assignment
Automatically assigns temporary goals when deadlock is detected based on:
- Average system speed monitoring
- Optimal leader selection
- Follower chain construction
- Dynamic goal reassignment

### QP Controller (Equation 3)
Solves real-time optimization with:
- Obstacle avoidance CBFs
- Inter-agent collision avoidance CBFs
- Connectivity maintenance CBFs
- Goal-reaching CLFs
- Slack variables for feasibility

### Bidding Mechanism (Equations 4a-4b)
Intelligently removes redundant edges to improve performance:
- Safe neighbor computation
- QP-based bid calculation
- Max-consensus for distributed decision
- Connectivity preservation guarantee

## Troubleshooting

**QP solver issues:**
```bash
pip install --upgrade cvxpy osqp
```

**Import errors:**
```bash
# Make sure you're in the project root directory
cd "Deadlock Resolution of Connected Multi-Agent Systems using Hierarchical Control"
python test_installation.py
```

**Visualization not working:**
```bash
python main.py --scenario apartment --no_viz
```

## License

MIT License - See LICENSE file for details

## Acknowledgments

Implementation based on the paper:
> Garg, K., Hamilton, S., & Fan, C. (2024). "Deadlock Resolution of Connected Multi-Agent Systems using Hierarchical Control."
> *2024 IEEE 63rd Conference on Decision and Control (CDC)*, pp. 1275-1282.

Original authors: Kunal Garg, Sera Hamilton, and Chuchu Fan (MIT)

---

**Paper PDF**: [Deadlock_Resolution_of_Connected_Multi-Agent_Systems_using_Hierarchical_Control.pdf](Deadlock_Resolution_of_Connected_Multi-Agent_Systems_using_Hierarchical_Control.pdf)
