# Distributed Graph Pruning with Formal Connectivity Guarantee

This module implements **provably safe distributed edge pruning** for multi-robot systems. The algorithm maintains global connectivity while creating sparse communication topologies using only local 2-hop information.

## Key Features

✅ **Formal Proof**: Mathematically proven to preserve connectivity  
✅ **Fully Distributed**: No central coordinator required  
✅ **Low Computation**: O(d²) per robot, where d ≈ 3-10 (very fast!)  
✅ **Low Communication**: Only neighbor ID lists (O(d) integers per robot)  
✅ **Progressive**: Adapts pruning rate to information completeness  
✅ **Robust**: Handles dynamic topology changes  

---

## Mathematical Guarantee

### Theorem: Common Neighbor Criterion

**Statement:** If edge $(i,j)$ has at least $k$ common neighbors (i.e., $|N(i) \cap N(j)| \geq k$), then:
- For $k=1$: Edge is **not a bridge** → Graph stays connected when removed
- For $k=2$: Graph remains **2-edge-connected** (robust to single failure)

**Proof:** See `FORMAL_PROOF.md`

### Algorithm Invariant

```
INVARIANT: Never prune edge (i,j) unless |N(i) ∩ N(j)| ≥ k

This guarantees connectivity is preserved at every timestep.
```

---

## Algorithm Overview

### Progressive Pruning Strategy

The algorithm runs in **three phases** based on information completeness:

**Phase 1 (k=1 hop, <30% completeness):**
- Very conservative: prune 1-2 edges per robot
- Only prune obvious redundancies
- Graph remains 80-90% dense

**Phase 2 (k=2 hop partial, 30-70% completeness):**
- Moderate: prune 3-5 edges per robot
- Only prune proven-safe edges (with common neighbors)
- Graph becomes 50-70% sparse

**Phase 3 (k=2 hop complete, >70% completeness):**
- Aggressive: prune up to 50% of excess edges per step
- Rapid convergence to target degree (3-4 neighbors)
- Graph reaches target sparse chain topology

### Safety Rules

1. **Never prune** edges without 2-hop confirmation
2. **Never prune** edges that might be bridges
3. **Never prune** if it would make robot under-connected
4. **Always keep** potential bridges (when $N(i) \cap N(j) = \emptyset$)

---

## Usage

### Basic Example

```python
from distributed_pruning import PrunedSimulation
from config import SimulationConfig, ControlConfig

# Create configuration
sim_config = SimulationConfig(
    num_robots=10,
    communication_radius=10.0,
    workspace_size=(10.0, 10.0),
    dt=0.05
)
control_config = ControlConfig()

# Create pruned simulation
sim = PrunedSimulation(
    sim_config,
    control_config,
    target_degree=3,      # Target 3 neighbors (sparse chain)
    k_connectivity=1      # 1-connected (proven guarantee)
)

# Run simulation
for step in range(1000):
    sim.step()  # Automatically handles pruning + control

# Get results
active_edges = sim.get_active_edges()
pruned_edges = sim.get_pruned_edges()
stats = sim.get_pruning_statistics()
```

### Run Demo

```bash
cd distributed_pruning
python demo_pruning.py
```

This will:
1. Start with fully connected graph (45 edges for 10 robots)
2. Progressively prune to sparse chain (~15 edges)
3. Show visualization with active (green) and pruned (red) edges
4. Plot pruning progression and metrics

---

## Architecture

### Files

- `pruning_controller.py`: Core pruning algorithm for single robot
- `pruned_simulation.py`: Simulation integration with baseline
- `demo_pruning.py`: Demonstration with visualization
- `README.md`: This file
- `FORMAL_PROOF.md`: Mathematical proof of connectivity preservation

### Class Hierarchy

```
FullyConnectedSimulation (baseline)
    ↓ extends
PrunedSimulation
    ↓ uses
ProgressivePruningController (one per robot)
```

---

## Complexity Analysis

**Per Robot Per Timestep:**

| Operation | Complexity | Typical Values |
|-----------|------------|----------------|
| Neighbor intersection (N(i) ∩ N(j)) | O(d²) | ~25-100 ops (d=5-10) |
| Edge scoring | O(d) | ~5-10 ops |
| Sorting safe edges | O(d log d) | ~10-30 ops |
| **Total** | **O(d²)** | **~50-150 ops** |

**Communication:**
- Broadcast: List of d neighbor IDs
- Bandwidth: O(d log n) bits ≈ 20-50 bytes
- Frequency: Every timestep (or on-change)

**Storage:**
- Own neighbors: O(d) ≈ 5-10 IDs
- 2-hop info: O(d²) ≈ 25-100 IDs
- **Total per robot: ~200-500 bytes**

---

## Parameters

### `target_degree` (default: 3)
Target number of neighbors per robot for final sparse topology.
- 2: Minimal connectivity (tree-like)
- 3-4: **Recommended** (robust chain)
- 5+: Less sparse (more redundancy)

### `k_connectivity` (default: 1)
Number of edge-disjoint paths to maintain.
- 1: **Connected** (proven guarantee)
- 2: **2-connected** (robust to single edge failure)

### Information Completeness Thresholds

**Conservative (< 0.3):** Prune 1-2 edges max  
**Moderate (0.3-0.7):** Prune 3-5 edges  
**Aggressive (> 0.7):** Prune 50% of excess edges  

---

## Theoretical Guarantees

### What is Proven:

✅ **Connectivity:** Graph never disconnects (Theorem 3)  
✅ **Safety:** Only non-bridge edges are pruned (Theorem 2)  
✅ **Distributed:** Heterogeneous information doesn't break guarantee (Theorem 6)  
✅ **Progressive:** Valid at every timestep (Theorem 4)  

### What is NOT Proven:

❌ **Optimality:** May not achieve globally optimal sparse topology  
❌ **Convergence Rate:** No bound on time to reach target degree  
❌ **Byzantine Resilience:** Assumes honest robots (no false neighbor lists)  

---

## Comparison with Alternatives

| Method | Connectivity | Computation | Communication | Distributed |
|--------|--------------|-------------|---------------|-------------|
| **This (Common Neighbor)** | ✅ Proven | O(d²) | O(d) | ✅ Yes |
| Bridge Detection (DFS/BFS) | ✅ Exact | O(E) | O(E) | ❌ No |
| MST (Prim/Kruskal) | ✅ Tree | O(E log V) | O(E) | ❌ No |
| GHS Distributed MST | ✅ Tree | O(E log V) | O(E log V) | ✅ Yes (complex) |
| Random Pruning | ❌ None | O(1) | O(1) | ✅ Yes |
| Algebraic Connectivity | ❌ Heuristic | O(N³) | O(N²) | ❌ No |

**Key Advantage:** This method is the **only** approach that provides:
1. Formal connectivity guarantee
2. O(d²) local computation
3. Fully distributed operation
4. No matrix operations or graph traversals

---

## Limitations & Future Work

### Current Limitations:
1. **No Byzantine tolerance:** Trusts neighbor lists (could add verification)
2. **Heuristic optimality:** Not proven to find optimal sparse topology
3. **Static target degree:** Doesn't adapt based on local density

### Future Enhancements:
- [ ] Add 3-way handshake for neighbor verification (Byzantine tolerance)
- [ ] Adaptive target degree based on local connectivity requirements
- [ ] Integration with consensus algorithms (use disagreement metric)
- [ ] Formal convergence rate analysis
- [ ] Extension to directed graphs

---

## Citation

If you use this implementation, please cite:

```bibtex
@software{distributed_pruning_2026,
  title={Distributed Graph Pruning with Formal Connectivity Guarantee},
  author={Your Name},
  year={2026},
  url={https://github.com/your-repo}
}
```

---

## References

1. **Bridge Detection:** Tarjan, R. E. (1974). "A note on finding the bridges of a graph"
2. **Graph Connectivity:** West, D. B. (2001). "Introduction to Graph Theory"
3. **Distributed Algorithms:** Lynch, N. A. (1996). "Distributed Algorithms"
4. **GHS Algorithm:** Gallager, R. G., Humblet, P. A., & Spira, P. M. (1983). "A Distributed Algorithm for Minimum-Weight Spanning Trees"

---

## Contact

For questions or issues, please open an issue on GitHub or contact [your email].
