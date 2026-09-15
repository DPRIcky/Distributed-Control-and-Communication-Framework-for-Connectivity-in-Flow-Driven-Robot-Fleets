# Implementation Notes

## Overview

This is a complete implementation of the paper "Deadlock Resolution of Connected Multi-Agent Systems using Hierarchical Control" by Garg, Hamilton, and Fan (CDC 2024).

## Implementation Structure

```
.
├── main.py                          # Main simulation entry point
├── requirements.txt                 # Python dependencies
├── README.md                        # Project overview
├── USAGE.md                        # Detailed usage guide
├── IMPLEMENTATION_NOTES.md         # This file
│
├── src/
│   ├── core/
│   │   ├── agent.py                # Agent with unicycle dynamics
│   │   └── obstacle.py             # Obstacle representation
│   │
│   ├── control/
│   │   ├── barrier_functions.py    # CBFs and CLFs
│   │   └── cbf_qp_controller.py    # Low-level QP controller (Eq. 3)
│   │
│   ├── planning/
│   │   └── high_level_planner.py   # Deadlock resolution (Algorithm 1)
│   │
│   └── utils/
│       ├── graph_utils.py          # Network topology and bidding (Eq. 4)
│       └── visualization.py        # Visualization tools
│
└── scenarios/
    └── scenarios.py                # Test scenarios
```

## Key Components Implemented

### 1. Agent Dynamics (Section II)
**File**: [src/core/agent.py](src/core/agent.py)

Unicycle dynamics:
- ẋᵢ = vᵢ cos(θᵢ)
- ẏᵢ = vᵢ sin(θᵢ)
- θ̇ᵢ = ωᵢ

### 2. Control Barrier Functions (Section III-B)
**File**: [src/control/barrier_functions.py](src/control/barrier_functions.py)

Implemented CBFs:
- **ObstacleBarrierPosition**: Bₗ,ₚ(pᵢ) = ½‖pᵢ - pₒ‖² - ½r²ₒᵦₛ
- **ObstacleBarrierAngle**: Bₗ,θ(θᵢ) = ½|θᵢ - θₒ|² - ½θ²ₜₕᵣₑₛₕₒₗₐ
- **AgentBarrierPosition**: Bᵢⱼ,ₚ = ½‖pᵢ - pⱼ‖² - (2dₘ)²
- **ConnectivityBarrier**: B̃ᵢⱼ,ₚ = -½‖pᵢ - pⱼ‖² + ½R²ₛ

### 3. Control Lyapunov Functions
**File**: [src/control/barrier_functions.py](src/control/barrier_functions.py)

Implemented CLFs:
- **LyapunovPosition**: Vₚ(pᵢ) = ½‖pᵢ - pᵍᵢ‖²
- **LyapunovAngle**: Vθ(θᵢ) = ½|θᵢ - θᵍᵢ|² - θ²ᵍ,ₜₕᵣₑₛ

### 4. CBF-QP Controller (Equation 3)
**File**: [src/control/cbf_qp_controller.py](src/control/cbf_qp_controller.py)

Solves the optimization problem:
```
min  ½z^T Q z + F^T z
s.t. CBF constraints (safety and connectivity)
     CLF constraints (goal reaching)
     Input constraints
```

With decision variables: z = [u, {αₗ,ₚ}, {αₗ,θ}, {αᵢⱼ,ₚ}, {α̃ᵢⱼ,ₚ}, δₚ, δθ]

### 5. High-Level Planner (Algorithm 1)
**File**: [src/planning/high_level_planner.py](src/planning/high_level_planner.py)

Implements:
- Deadlock detection: Σᵢ|ṗᵢ|/N < uₘᵢₙ
- Leader selection: lead = arg minᵢ ‖pᵢ - pᵍᵢ‖/‖ṗᵢ|ₚᵍᵢ,ₜₑₘₚ‖
- Follower assignment (Equation 2)
- Temporary goal assignment for leader
- Terminal goal re-assignment (goal swapping)

### 6. Bidding Mechanism (Based on Zavlanos & Pappas 2008)
**File**: [src/utils/graph_utils.py](src/utils/graph_utils.py)

Implements Algorithm 1 from "Distributed Connectivity Control of Mobile Networks":
- **Safe neighbor computation**: Sᵢ = {jᵢ ∈ Nᵢ : λ₂(L(Aᵢ \ {(i,jᵢ)})) > 0}
  - Uses local connectivity check via BFS
  - Verifies all neighbors remain reachable after edge removal
- **Selection function g(Sᵢ)**: Choose farthest safe neighbor
  - Prioritizes removing long-range edges
  - Distance-based metric for bid computation
- **Bid computation**: bᵢ = Kᵢ · (distance / sensing_radius)
  - Normalized by sensing radius for reasonable range
  - Kᵢ = 1 + ε for mutually beneficial edges
- **Max-consensus update**: Token-based distributed agreement
  - Binary tokens: 1 if agent has max bid among neighbors
  - Convergence when all tokens = 1
  - Edge removed when consensus reached

### 7. Graph Connectivity (Section II)
**File**: [src/utils/graph_utils.py](src/utils/graph_utils.py)

Implements:
- Adjacency matrix computation
- Laplacian matrix: L(A) = D - A
- Second smallest eigenvalue λ₂(L) for connectivity check
- Distributed connectivity assessment

## Test Scenarios

### 1. Simple Scenario
- **Agents**: 3
- **Purpose**: Basic testing and debugging

### 2. Apartment Scenario (Figure 3 in paper)
- **Agents**: 5
- **Obstacles**: Walls and rooms
- **Challenge**: Navigation through constrained spaces

### 3. Crossing Scenario (Figure 4 in paper)
- **Agents**: 10 (two groups of 5)
- **Challenge**: Inter-group interaction and crossing paths

### 4. Narrow Passage Scenario (Figure 5 in paper)
- **Agents**: 24 (four groups of 6)
- **Obstacles**: Four squares creating narrow passages
- **Challenge**: Scalability and complex deadlock scenarios

## Key Features

### ✅ Implemented from Paper:
1. Unicycle dynamics for agents
2. CBF-QP controller with safety and connectivity constraints
3. High-level leader-follower assignment (Algorithm 1)
4. Bidding-based edge deletion mechanism (Equations 4a-4b)
5. Deadlock detection and resolution
6. Connectivity maintenance via Laplacian eigenvalue
7. All three test scenarios from the paper

### 🔧 Implementation Details:

**Solver**: OSQP for quadratic programming
- Fast, suitable for real-time control
- Handles slack variables for CBF/CLF feasibility

**Numerical Integration**: Euler method
- Simple and stable for small time steps
- dt = 0.1s default (same as paper)

**Graph Updates**: NetworkX library
- Efficient graph operations
- Built-in connectivity checking

## Parameters (from paper)

| Parameter | Symbol | Default Value | Description |
|-----------|--------|---------------|-------------|
| Sensing radius | Rₛ | 1.7 | Communication range |
| Safe distance | dₘ | 0.1 | Minimum inter-agent distance |
| Obstacle safety | rₒᵦₛ | 0.1 | Minimum obstacle distance |
| Max velocity | vₘₐₓ | 0.5 | Maximum linear velocity |
| Max angular vel | ωₘₐₓ | 1.0 | Maximum angular velocity |
| Deadlock threshold | uₘᵢₙ | 0.01 | Speed threshold for deadlock |
| Min goal distance | dₘᵢₙ | 0.5 | Distance for temp goal use |
| Time step | dt | 0.1 | Simulation time step |

## Comparison with Paper

The implementation closely follows the paper with these considerations:

### Exact Matches:
- Agent dynamics (unicycle model)
- CBF/CLF formulations
- QP structure (Equation 3)
- Leader-follower assignment (Algorithm 1)
- Bidding mechanism (Equations 4a-4b)

### Practical Adaptations:
- **Paper**: Assumes continuous-time dynamics
- **Implementation**: Discrete-time with Euler integration

- **Paper**: Theoretical framework
- **Implementation**: Numerical stability considerations (epsilon values, bounds)

- **Paper**: Assumes ideal communication
- **Implementation**: Explicit neighbor information exchange

## Performance Characteristics

Based on paper's Table I:
- **Apartment (5 agents)**: ~0.01s per step per agent
- **Crossing (10 agents)**: ~0.005s per step per agent
- **Narrow (24 agents)**: ~0.009s per step per agent

Much faster than MPC baseline (0.1-1.9s reported in paper).

## Usage for Research

### Reproducing Paper Results:
```bash
# Run with hierarchical control
python main.py --scenario apartment

# Run without high-level planner (baseline CBF-QP)
python main.py --scenario apartment --no_planner

# Compare success rates and times
```

### Extensions:
1. **Custom scenarios**: Add to `scenarios/scenarios.py`
2. **Different dynamics**: Modify `src/core/agent.py`
3. **Advanced CBFs**: Extend `src/control/barrier_functions.py`
4. **Learning-based planners**: Replace `HighLevelPlanner`

## Limitations and Future Work

### Current Limitations:
1. **Heuristic goal assignment**: Leader's temporary goal uses sampling (not learning-based)
2. **Simple bidding**: Speed-based metric (could use more sophisticated criteria)
3. **Convex obstacles**: Barrier functions assume convex shapes

### Suggested Extensions (from paper's Section V):
1. Reinforcement learning for high-level goal assignment
2. Neural barrier certificates for complex obstacles
3. Time-varying obstacles and dynamic environments
4. Heterogeneous agent dynamics

## Dependencies

- **NumPy**: Numerical computations
- **SciPy**: Scientific computing utilities
- **CVXPY**: Convex optimization (QP solving)
- **Matplotlib**: Visualization
- **NetworkX**: Graph operations

## Testing

To verify the implementation:

```bash
# Test simple scenario (quick)
python main.py --scenario simple --max_time 20

# Test apartment scenario
python main.py --scenario apartment

# Compare with/without planner
python main.py --scenario crossing --no_planner
```

Expected behavior:
- **With planner**: Agents resolve deadlocks and reach goals
- **Without planner**: Agents get stuck in local minima

## References

Garg, K., Hamilton, S., & Fan, C. (2024). Deadlock Resolution of Connected Multi-Agent Systems using Hierarchical Control. IEEE Conference on Decision and Control (CDC), pp. 1275-1282.

## Contact

For questions or issues with this implementation, please refer to the original paper and its authors' contact information.
