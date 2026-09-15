# Comm-Feasible Distributed Spanning-Tree Pruning

## Overview

This document describes the refactored distributed pruning algorithm that eliminates O(n) vector propagation, making it realistic for deployment in communication-constrained multi-robot systems.

## The Problem: "Pruning is a Communication Disaster"

The original 4-phase algorithm required each node to broadcast **full distance vectors** of size O(n) to all neighbors. For a network of n nodes:

| Metric | Original Algorithm |
|--------|-------------------|
| Message payload | O(n) per message |
| Total bandwidth per round | O(n² · |E|) bytes |
| Memory per node | O(n) |

This made the algorithm **infeasible** for real wireless networks with:
- Limited bandwidth (e.g., 250 kbps for ZigBee)
- Packet size limits (e.g., 127 bytes max for IEEE 802.15.4)
- Battery constraints on transmission power

## The Solution: δ-Only BFS

The refactored algorithm uses **scalar-only** distance propagation:

### Message Format
```
Message = (sender_id, delta, seq)
         = (2 bytes, 2 bytes, 2 bytes)
         = 6 bytes total
```

| Metric | Refactored Algorithm |
|--------|---------------------|
| Message payload | **O(1)** = 6 bytes |
| Total bandwidth per round | O(|E|) · 6 bytes |
| Memory per node | O(degree) |

### Protocol Summary

**Phase 1: δ-BFS (Distance to Root)**
- Root initializes δ_root = 0
- All other nodes initialize δ_i = ∞
- Nodes periodically broadcast (id, δ, seq) to neighbors
- On receive from neighbor j: if δ_j + 1 < δ_i, update δ_i and set parent = j
- Tie-break: prefer lower-ID parent for determinism

**Phase 3: Spanning Tree Construction**
- After δ-BFS converges, each node keeps edge to its parent
- Result: spanning tree rooted at the designated root node

### What Was Removed

**Phase 2 (Connectivity Assurance)** - REMOVED
- Required O(n) distance vectors to detect disconnected components
- Not feasible under O(1) communication constraints
- Future work: leader election or timeout-based approaches

**Phase 4 (Robustness Enhancement)** - REMOVED
- Required all-pairs distances for alternate path validation
- Would need O(n²) information globally
- Future work: local 2-hop analysis or probabilistic sampling

## Communication Simulation Features

The implementation includes realistic comm modeling:

```python
DistributedPruningAlgorithm(
    graph=G,
    root=1,
    p_drop=0.1,      # 10% packet drop probability
    delay_max=0.5,   # Max random delay (seconds)
    t_broadcast=1.0, # Broadcast period
    jitter=0.2       # Scheduling jitter
)
```

### Features
- **Packet drops**: Messages dropped with probability p_drop
- **Random delays**: Delivery delayed by Uniform(0, delay_max)
- **Async scheduling**: Nodes broadcast independently with jitter
- **Comm accounting**: Track messages sent, dropped, bytes transmitted

## Comparison Table

| Aspect | Original O(n) Vectors | Refactored δ-Only |
|--------|----------------------|-------------------|
| Payload size | n × 4 bytes | 6 bytes |
| For n=100 | 400 bytes | 6 bytes |
| For n=1000 | 4000 bytes | 6 bytes |
| IEEE 802.15.4 compatible | ❌ No (exceeds 127B limit) | ✅ Yes |
| Memory scalable | ❌ O(n) per node | ✅ O(degree) |
| Supports drops/delays | ❌ Not designed for | ✅ Built-in |
| Spanning tree | ✅ Yes | ✅ Yes |
| Robustness edge | ✅ Yes | ❌ Removed (needs redesign) |

## Usage Example

```python
from distributed_pruning_algorithm import DistributedPruningAlgorithm
import networkx as nx

# Create graph
G = nx.erdos_renyi_graph(20, 0.3)
G = nx.relabel_nodes(G, {i: i+1 for i in range(20)})

# Run comm-feasible pruning
alg = DistributedPruningAlgorithm(
    graph=G,
    root=1,
    p_drop=0.05,
    delay_max=0.2
)

kept_edges = alg.run(t_prune=15.0, t_stable=3.0)

# Get comm stats
stats = alg.summary_comm_stats()
print(f"Messages: {stats['messages_sent_total']}")
print(f"Bandwidth: {stats['bytes_per_second']:.1f} B/s")
```

## Future Work

To restore Phase 2 and Phase 4 functionality under O(1) comm constraints:

1. **Phase 2 Alternatives**:
   - Spanning forest detection via parent pointer analysis
   - Distributed leader election within components
   - Timeout-based connectivity inference

2. **Phase 4 Alternatives**:
   - Local 2-hop neighborhood analysis (limited but O(1))
   - Probabilistic robustness edge sampling
   - Separate bounded-message robustness protocol

## References

- Bellman-Ford distributed shortest paths
- Spanning tree construction via BFS
- IEEE 802.15.4 packet size constraints
