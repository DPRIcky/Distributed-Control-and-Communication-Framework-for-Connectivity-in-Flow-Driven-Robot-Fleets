# Quick Reference Guide

## Installation

```bash
pip install -r requirements.txt
```

## Running the Implementation

### 1. Quick Test (Verify Installation)
```bash
python verify_implementation.py
```
Expected output: `✓ IMPLEMENTATION VERIFIED SUCCESSFULLY!`

### 2. Run Examples
```bash
python example_usage.py
```
Shows analysis of 5 different network topologies.

### 3. Run Full Test Suite
```bash
python test_algorithm.py
```
Executes 19 unit tests.

## Basic Usage Examples

### Example 1: Detect Bridges in a Network
```python
import networkx as nx
from distributed_edge_connectivity import DistributedEdgeConnectivity

# Create network
G = nx.path_graph(5)

# Run algorithm
algo = DistributedEdgeConnectivity(G)
results = algo.run()

# Get results
print("Critical edges:", results['critical_edges'])
print("Number of bridges:", results['bridge_count'])
```

### Example 2: Analyze Network Robustness
```python
import networkx as nx
from distributed_edge_connectivity import DistributedEdgeConnectivity

# Create network
G = nx.grid_2d_graph(3, 3)

# Run algorithm
algo = DistributedEdgeConnectivity(G)
algo.run()

# Get metrics
metrics = algo.get_robustness_metrics()
print(f"Vulnerability: {metrics['vulnerability']:.4f}")
print(f"Network edge connectivity: {metrics['network_edge_connectivity']}")
```

### Example 3: Find Redundant Paths
```python
import networkx as nx
from distributed_edge_connectivity import DistributedEdgeConnectivity

# Create network
G = nx.cycle_graph(5)

# Run algorithm
algo = DistributedEdgeConnectivity(G)
algo.run()

# Find redundant paths
redundancy = algo.identify_redundant_paths(0, 2)
print(f"Edge-disjoint paths: {redundancy['num_edge_disjoint_paths']}")
print(f"Paths: {redundancy['edge_disjoint_paths']}")
```

### Example 4: Compare Algorithms
```python
import networkx as nx
from distributed_edge_connectivity import (
    DistributedEdgeConnectivity,
    CentralizedEdgeConnectivity
)

G = nx.path_graph(5)

# Distributed
dist_algo = DistributedEdgeConnectivity(G)
dist_results = dist_algo.run()

# Centralized
cent_algo = CentralizedEdgeConnectivity(G)
cent_results = cent_algo.run()

# Compare
print(f"Distributed:  {dist_results['critical_edges']}")
print(f"Centralized:  {cent_results['critical_edges']}")
```

## API Quick Reference

### DistributedEdgeConnectivity

**Constructor:**
```python
algo = DistributedEdgeConnectivity(graph, max_rounds=100)
```

**Methods:**
```python
# Run algorithm
results = algo.run()

# Get robustness metrics  
metrics = algo.get_robustness_metrics()

# Find redundant paths
redundancy = algo.identify_redundant_paths(source, target)
```

**Results Dictionary:**
- `critical_edges`: Set of bridge edges
- `edge_connectivity`: Dict of edge connectivity values
- `bridge_count`: Number of critical edges
- `total_edges`: Total edges in graph
- `total_nodes`: Total nodes in graph

**Robustness Metrics:**
- `vulnerability`: Fraction of critical edges
- `average_edge_connectivity`: Mean edge connectivity
- `minimum_edge_connectivity`: Minimum connectivity
- `maximum_edge_connectivity`: Maximum connectivity
- `network_edge_connectivity`: Network's k value

## Understanding the Output

### Critical Edges
- **Bridge**: An edge whose removal disconnects the graph
- Example: In path 1-2-3-4-5, all edges are bridges
- **Significance**: Removal causes network partition

### Edge Connectivity
- **k(u,v)**: Minimum edges to remove to disconnect u and v
- **Network k**: Minimum k across all node pairs
- **Range**: 1 (bridge) to n-1 (complete graph)

### Vulnerability Score
- **0.0**: No critical edges (fully redundant)
- **1.0**: All edges are critical (path graph)
- **0.5**: Half the edges are critical

### Redundancy
- **1 path**: No alternative routes
- **2+ paths**: Has backup routes
- **Higher = More robust**

## Common Tasks

### Task: Find Network Bottlenecks
```python
algo = DistributedEdgeConnectivity(G)
results = algo.run()
bottlenecks = results['critical_edges']
```

### Task: Check Network Reliability
```python
algo = DistributedEdgeConnectivity(G)
algo.run()
metrics = algo.get_robustness_metrics()
if metrics['vulnerability'] < 0.3:
    print("Network is reliable")
```

### Task: Plan Redundancy
```python
algo = DistributedEdgeConnectivity(G)
algo.run()
for source in G.nodes():
    for target in G.nodes():
        if source < target:
            red = algo.identify_redundant_paths(source, target)
            if red['has_redundancy']:
                print(f"Path {source}-{target} has backup routes")
```

## Troubleshooting

### "ModuleNotFoundError: No module named 'networkx'"
```bash
pip install networkx
```

### "AttributeError: 'Graph' object has no attribute..."
- Make sure you're using NetworkX Graph, not other formats
- Check that the graph has nodes and edges

### "Timeout in algorithm"
- Large graphs may take time
- Increase `max_rounds` if needed
- Use centralized version for debugging

## Performance Tips

1. **Small graphs**: Use either algorithm
2. **Large graphs**: Use distributed algorithm
3. **Many queries**: Cache results
4. **Repeated analysis**: Pre-compute and store

## Which Algorithm to Use?

### Use Distributed When:
- Simulating real networks
- Studying message-passing systems
- Implementing in actual distributed system
- Analyzing network communication costs

### Use Centralized When:
- Validating results
- Analyzing static topology
- Quick one-time analysis
- Need maximum performance

## File Overview

| File | Purpose |
|------|---------|
| `distributed_edge_connectivity.py` | Core algorithm |
| `example_usage.py` | Usage examples |
| `test_algorithm.py` | Unit tests |
| `verify_implementation.py` | Quick check |
| `README.md` | Full documentation |
| `IMPLEMENTATION_SUMMARY.md` | Implementation details |
| `requirements.txt` | Dependencies |
| `QUICK_REFERENCE.md` | This file |

## Next Steps

1. **Run** `python verify_implementation.py`
2. **Explore** `python example_usage.py`
3. **Read** README.md for theory
4. **Integrate** into your project
5. **Test** with `python test_algorithm.py`

## Contact/Support

For algorithm questions, refer to:
- README.md for detailed information
- Code comments for implementation details
- Original CDC 2024 paper for theory

---

Happy analyzing! 🚀
