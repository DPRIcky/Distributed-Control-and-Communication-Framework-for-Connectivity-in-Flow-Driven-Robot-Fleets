# UNIFIED ALGORITHM ENVIRONMENT - Quick Start Guide

## Overview

This unified environment allows you to run **Algorithms 1-6** sequentially within a robot communication network simulation. Each algorithm builds on the previous ones to progressively analyze and optimize the network topology.

## What Each Algorithm Does

### Algorithm 1: Distributed BFS Tree Construction
- **Purpose**: Build spanning trees from each node as root
- **Use Case**: Foundation for topology discovery
- **Complexity**: O(n + m)
- **Output**: BFS trees, distances, parent relationships
- **Key Metric**: Average reachable nodes

### Algorithm 2: Subtree Computation  
- **Purpose**: Compute subtree information from BFS trees
- **Use Case**: Distributed topology analysis
- **Complexity**: O(n + m)
- **Output**: Subtree sizes and members
- **Key Metric**: Complete trees percentage

### Algorithm 3: Bridge Detection (Critical Edges)
- **Purpose**: Identify edges whose removal disconnects the graph
- **Use Case**: Find critical infrastructure
- **Complexity**: O(m(n + m))
- **Output**: Set of critical edges (bridges)
- **Key Metric**: Redundancy ratio

### Algorithm 4: Edge Connectivity Computation
- **Purpose**: Calculate minimum edge cuts needed to disconnect each edge
- **Use Case**: Measure edge resilience  
- **Complexity**: O(m × network_flow)
- **Output**: Edge connectivity values k_e for each edge
- **Key Metric**: Average connectivity

### Algorithm 5: Redundant Edge Identification
- **Purpose**: Find non-critical edges that can be removed
- **Use Case**: Identify optimization opportunities
- **Complexity**: O(m)
- **Output**: Set of redundant edges
- **Key Metric**: Redundancy ratio

### Algorithm 6: Distributed Edge Pruning
- **Purpose**: Remove redundant edges while maintaining connectivity
- **Use Case**: Optimize network (minimize edges while keeping connected)
- **Complexity**: O(m)
- **Output**: Pruned graph with only critical edges
- **Key Metric**: Connectivity preserved

## How to Use

### Option 1: Command-Line (All Algorithms)

Run all algorithms on an 8-robot network:
```bash
cd critical_edge_detection_and_pruning_update/scripts
python unified_algorithm_environment.py --all
```

Run specific algorithm:
```bash
python unified_algorithm_environment.py --algorithm 3 --robots 12
```

### Option 2: Command-Line (Specific Scenarios)

**Scenario A: 8 robots, all algorithms**
```bash
python unified_algorithm_environment.py --all --robots 8 --seed 42
```

**Scenario B: Run only Algorithm 3 on 15 robots**
```bash
python unified_algorithm_environment.py -a 3 -r 15
```

**Scenario C: Run only Algorithm 6 (pruning) on 10 robots**  
```bash
python unified_algorithm_environment.py -a 6 -r 10
```

### Option 3: Interactive GUI

Real-time visualization with algorithm selection:
```bash
python gui_unified_algorithms.py
```

**Using the GUI:**
1. Click any algorithm button (1-6) to run it
2. Watch the network visualization update
3. View results in the "Latest Results" panel
4. Click "Run All" to execute algorithms 1-6 sequentially
5. Observe how critical (red) and redundant (orange) edges are identified

### Option 4: Python Script (Programmatic)

```python
from unified_algorithm_environment import UnifiedAlgorithmEnvironment

# Create environment
env = UnifiedAlgorithmEnvironment(num_robots=12, seed=42)

# Run specific algorithms
result1 = env.run_algorithm_1()
result2 = env.run_algorithm_2()
result3 = env.run_algorithm_3()
result4 = env.run_algorithm_4()
result5 = env.run_algorithm_5()
result6 = env.run_algorithm_6()

# Access results
print(f"Critical edges: {env.critical_edges}")
print(f"Redundant edges: {env.redundant_edges}")
print(f"Edges remaining: {len(env.graph.edges())}")
```

## Understanding the Output

### Console Output Example

```
======================================================================
ALGORITHM 3: Bridge Detection
======================================================================

Testing 24 edges for criticality...
  ✓ Edge (0, 1) is CRITICAL (bridge)
  ✓ Edge (3, 7) is CRITICAL (bridge)
  ...

======================================================================
Algorithm 3: Bridge Detection (Critical Edges)
======================================================================
Status: ✓ SUCCESS
Time: 0.0234s
Messages: 24
Communication Rounds: 2
  total_edges: 24
  critical_edges: 5
  non_critical_edges: 19
  redundancy_ratio: 0.79
  bridges: [(0, 1), (3, 7), ...]
```

### Key Metrics Explained

| Metric | Meaning |
|--------|---------|
| **Messages** | Number of message exchanges in distributed algorithm |
| **Communication Rounds** | Number of parallel communication phases |
| **Execution Time** | Time taken to run algorithm |
| **Redundancy Ratio** | Non-critical / total edges (0 = minimal, 1 = very redundant) |
| **Connectivity** | How many edge-disjoint paths exist between endpoints |
| **Critical Edges** | Edges that are bridges (removing disconnects graph) |

## Example: Complete Workflow

```bash
# 1. Run all algorithms on 10-robot network
python unified_algorithm_environment.py --all --robots 10

# 2. View final results
# The algorithm will show:
# - 10 robots in the network
# - After Algorithm 3: X edges are critical (bridges)
# - After Algorithm 5: Y edges are redundant  
# - After Algorithm 6: Z edges removed, graph still connected

# 3. Success indicators:
# ✓ Algorithm 6 connectivity preserved = SUCCESS
```

## Integration with Robot Simulation

The unified environment is designed to integrate with underwater robot simulations:

```python
from gui_simulation_consensus import ConsensusProofSimulation
from unified_algorithm_environment import UnifiedAlgorithmEnvironment

# Create robot simulation
robot_sim = ConsensusProofSimulation(sim_config, control_config, consensus_config)

# Extract current topology as graph
current_graph = robot_sim.topology_manager.get_network_graph()

# Run algorithms on current topology
algo_env = UnifiedAlgorithmEnvironment(num_robots=robot_sim.num_robots)
algo_env.graph = current_graph
algo_env.run_algorithm_3()

# Use results for intelligent pruning decisions
critical_edges = algo_env.critical_edges
```

## Performance Expectations

### Execution Time (Approximate)

| # Robots | Algo 1-2 | Algo 3 | Algo 4 | Algo 5-6 | Total |
|----------|----------|--------|--------|----------|-------|
| 8        | ~0.01s   | 0.05s  | 0.10s  | 0.02s    | 0.18s |
| 12       | ~0.02s   | 0.15s  | 0.35s  | 0.03s    | 0.55s |
| 20       | ~0.05s   | 0.50s  | 1.2s   | 0.05s    | 1.8s  |

### Message Complexity

- **Distributed algorithms** (1-2, 3): O(n) rounds, O(n+m) total messages
- **Centralized algorithms** (4): O(1) rounds, O(m²) messages for edge cuts
- **Pruning** (6): O(m) messages for edge removal

## Troubleshooting

### "No critical edges found"
- The network might have no bridges (all edges redundant)
- This is normal for highly connected networks
- Try with more robots or sparser network

### "Graph disconnected after pruning"
- Algorithm 6 failed safely (didn't remove edges)
- Review Algorithm 3 results - may have miscalculated critical edges
- Check Algorithm 4 connectivity values

### "ImportError: cannot import HybridUnderwaterSimulation"
- Robot simulation components not available
- GUI will still work with algorithm analysis only
- The unified_algorithm_environment.py works independently

### Slow performance
- Large number of robots increases complexity
- Algorithm 4 (O(m²)) is slowest for dense networks
- Try with fewer robots or sparser topology

## Files Reference

| File | Purpose |
|------|---------|
| `unified_algorithm_environment.py` | Core algorithm implementations (Algo 1-6) |
| `gui_unified_algorithms.py` | Interactive GUI with visualization |
| `distributed_edge_connectivity.py` | Original Algorithm 1-4 implementations |
| `algorithm_6_implementation.py` | Original Algorithm 6 implementation |

## Advanced Usage

### Custom Test Networks

```python
import networkx as nx
from unified_algorithm_environment import UnifiedAlgorithmEnvironment

env = UnifiedAlgorithmEnvironment(num_robots=0)  # No auto-generate

# Create custom graph
G = nx.Graph()
G.add_edges_from([(0,1), (1,2), (2,3), (3,0), (0,2)])  # Square with diagonal
env.graph = G
env.num_robots = G.number_of_nodes()

# Run algorithms
env.run_algorithm_3()
print(f"Critical edges: {env.critical_edges}")
```

### Performance Profiling

```python
import time
from unified_algorithm_environment import UnifiedAlgorithmEnvironment

env = UnifiedAlgorithmEnvironment(num_robots=20)

times = {}
for algo in range(1, 7):
    start = time.time()
    getattr(env, f'run_algorithm_{algo}')()
    times[algo] = time.time() - start

print("Algorithm runtimes:")
for algo, t in times.items():
    print(f"  Algorithm {algo}: {t:.4f}s")
```

## References

- **CDC 2024 Paper**: "A Distributed Method for Detecting Critical Edges and Increasing Edge Connectivity in Undirected Networks"
- **Griparic et al. 2022**: Consensus-based adjacency matrix estimation
- **Underwater Simulation**: CLF-CBF control with hybrid consensus pruning

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review algorithm output messages (often indicate the problem)
3. Verify network is connected after graph creation
4. Check that number of robots matches graph node count

---

**Last Updated**: February 2026
**Version**: 1.0
