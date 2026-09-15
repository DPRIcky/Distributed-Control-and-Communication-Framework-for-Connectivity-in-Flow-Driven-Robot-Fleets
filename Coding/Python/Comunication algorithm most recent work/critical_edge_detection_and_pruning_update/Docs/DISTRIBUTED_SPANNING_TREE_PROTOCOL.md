# Distributed Spanning Tree Protocol for Edge Pruning

## Network Model

Let $G = (N, E)$ be a connected undirected graph where:

- **Nodes**: $N = \{1, 2, \ldots, n\}$ (robots in the network)
- **Edges**: $E \subseteq \{\{i, j\} \mid i \neq j\}$ (direct communication links)

Each node (robot) $i$ maintains only **local knowledge**:

$$N_i := \{j \in N \mid \{i, j\} \in E\}$$

No node has global knowledge of the full edge set $E$.

A designated root node $r \in N$ is chosen (e.g., $r = 1$).

---

## Objective

Compute a pruned edge set $E_T \subseteq E$ such that $(N, E_T)$ forms a **spanning tree**. This means:

1. **Connectivity**: $(N, E_T)$ is connected
2. **Acyclic**: $(N, E_T)$ contains no cycles  
3. **Minimal edges**: $|E_T| = n - 1$

**Key consequence**: In a tree, every edge is a **bridge** (critical edge). After pruning via this protocol, all remaining edges are guaranteed to be critical.

---

## Protocol Phases

### Phase 1: Distributed Root-Distance Computation

Each node $i$ computes its hop distance to the root $r$:

$$\delta_i := d(i, r)$$

This can be done distributively using a min-consensus procedure (e.g., neighbor-based distance-first logic or BFS-style wave):

1. Root $r$ initializes $\delta_r = 0$
2. Non-root nodes initialize $\delta_i = \infty$
3. In each communication round, nodes exchange distances with neighbors and update:
   $$\delta_i^{(\text{new})} = 1 + \min_{j \in N_i} \delta_j^{(\text{old})}$$
4. Upon convergence, each node shares $\delta_i$ with all neighbors (local exchange)

**Convergence**: In a connected graph, this completes in $O(D)$ rounds where $D$ is the graph diameter, or $O(n)$ if using finite-time methods.

### Phase 2: Parent Selection (Local Rule)

For each non-root node $i \neq r$, define the set of candidate parents:

$$C_i := \{j \in N_i \mid \delta_j = \delta_i - 1\}$$

Since $G$ is connected and distances are correct, $C_i \neq \emptyset$ for all $i \neq r$.

Each node selects a **deterministic parent** using a tie-breaking rule (e.g., minimum node ID):

$$p(i) := \min C_i, \quad \forall i \neq r$$

The root $r$ has no parent: $p(r) := \text{undefined}$.

**Local announcement**: Each node announces its parent choice $p(i)$ to all neighbors. This can be done as a broadcast of the single scalar $p(i)$ or as a 1-bit confirmation sent only to $p(i)$.

### Phase 3: Distributed Keep/Prune Handshake

For each undirected edge $\{i, j\} \in E$, define the keep decision:

$$K_{ij} = \begin{cases} 1, & \text{if } p(i) = j \text{ or } p(j) = i \\ 0, & \text{otherwise} \end{cases}$$

**Keep rule**: Edge $\{i, j\}$ is kept iff $K_{ij} = 1$; otherwise it is pruned.

The resulting pruned edge set is:

$$E_T := \{\{i, j\} \in E \mid K_{ij} = 1\}$$

**Local execution**: Each node $i$ independently decides for each incident edge $\{i, j\}$:
- Keep $\{i, j\}$ if $j = p(i)$ or $i = p(j)$
- Prune $\{i, j\}$ otherwise

No global consensus or synchronization is required for this decision.

---

## Correctness

### Lemma 1 (Strict Descent)

For any non-root node $i \neq r$:

$$\delta_{p(i)} = \delta_i - 1$$

**Proof**: By definition, $p(i)$ is chosen from $C_i$, which contains only neighbors $j$ with $\delta_j = \delta_i - 1$. ∎

### Lemma 2 (Connectivity)

For every non-root node $i \neq r$, edge $\{i, p(i)\} \in E_T$.

**Proof**: By definition of $K_{ij}$, since $p(i) = p(i)$ by reflexivity, edge $\{i, p(i)\}$ satisfies $K_{i,p(i)} = 1$. Thus $\{i, p(i)\} \in E_T$. By Lemma 1, following parent pointers from any node eventually reaches $r$, giving a path in $(N, E_T)$ to the root. ∎

### Lemma 3 (Acyclic)

The graph $(N, E_T)$ contains no cycles.

**Proof**: A directed cycle $i_1 \to i_2 \to \cdots \to i_1$ (where $\to$ denotes the parent relationship) would require $\delta_{i_1} > \delta_{i_2} > \cdots > \delta_{i_1}$, a contradiction. Thus no cycles exist. ∎

### Theorem (All Edges Critical)

After executing the protocol, $(N, E_T)$ is a spanning tree. Therefore, every edge in $E_T$ is a bridge (critical edge).

**Proof**: By Lemmas 2 and 3, $(N, E_T)$ is connected and acyclic. Since $|E_T| = n - 1$ (each non-root node contributes exactly one parent edge), it is a spanning tree. In any tree, every edge is a bridge. ∎

---

## Complexity Analysis

### Round Complexity

Assuming synchronous communication rounds:

**Phase 1 (Distance computation):**
- Standard BFS-style wave: $O(D)$ rounds, where $D$ is the graph diameter
- Finite-time consensus variant: $O(n)$ rounds (diameter-independent)

**Phase 2 (Share $\delta$ and select parent):**
- 1 round: Each node broadcasts $\delta_i$ to neighbors
- Local computation: Nodes compute $p(i)$

**Phase 3 (Announce parents and apply keep/prune rule):**
- 1 round: Each node broadcasts $p(i)$ to neighbors
- Local computation: Each node decides keep/prune per incident edge

**Total**: After distances converge, $2$ additional communication rounds suffice for pruning.

### Message Complexity

**Phase 1**: Depends on distance algorithm. Typical complexity is $O(|E|)$ messages per round and $O(D)$ rounds, totaling $O(D \cdot |E|)$ messages.

**Phase 2 ($\delta$-broadcast):**
- Each node sends $\delta_i$ (one scalar) to all neighbors
- Total: $2|E|$ messages (one each direction per edge)

**Phase 3 (parent-broadcast):**
- Each node sends $p(i)$ (one node ID) to all neighbors
- Total: $2|E|$ messages (or $n-1$ if using parent-only unicast)

**Overall**: $O(|E|)$ messages for Phases 2 and 3 combined.

### Bit Complexity

Both $\delta_i$ and $p(i)$ fit in $\lceil \log_2 n \rceil$ bits (hop count and node ID respectively).

**Per-edge per-round bit cost**: $O(\log n)$ bits

---

## Worked 8-Node Example

**Setup:**
- Root: $r = 1$
- Distances: $(\delta_1, \delta_2, \delta_3, \delta_4, \delta_5, \delta_6, \delta_7, \delta_8) = (0, 1, 2, 1, 2, 2, 3, 1)$
- Neighbor sets:
  - $N_1 = \{2, 4, 8\}$
  - $N_2 = \{1, 3, 4\}$
  - $N_3 = \{2, 4\}$
  - $N_4 = \{1, 2, 3, 5\}$
  - $N_5 = \{4, 6, 7\}$
  - $N_6 = \{5, 7, 8\}$
  - $N_7 = \{5, 6\}$
  - $N_8 = \{6, 1\}$

**Parent Selection** (using $p(i) = \min\{j \in N_i : \delta_j = \delta_i - 1\}$):

| Node $i$ | $\delta_i$ | $N_i$ | $C_i$ (candidates) | $p(i)$ | Kept Edge |
|:--------:|:----------:|:-------:|:------------------:|:------:|:---------:|
| 1 | 0 | {2,4,8} | — | — | — |
| 2 | 1 | {1,3,4} | {1} | 1 | {2,1} |
| 3 | 2 | {2,4} | {2,4} | 2 | {3,2} |
| 4 | 1 | {1,2,3,5} | {1} | 1 | {4,1} |
| 5 | 2 | {4,6,7} | {4} | 4 | {5,4} |
| 6 | 2 | {5,7,8} | {8} | 8 | {6,8} |
| 7 | 3 | {5,6} | {5,6} | 5 | {7,5} |
| 8 | 1 | {6,1} | {1} | 1 | {8,1} |

**Resulting Pruned Edge Set:**

$$E_T = \{\{1,2\}, \{2,3\}, \{1,4\}, \{4,5\}, \{5,7\}, \{1,8\}, \{8,6\}\}$$

**Verification:**
- $|E_T| = 7 = n - 1$ ✓
- Connected: All nodes reachable from root 1 via parent chains ✓
- Acyclic: No cycles (distances strictly decrease toward root) ✓
- All 7 edges are bridges in the spanning tree ✓

All other edges in the original graph $G$ (if any exist beyond $E_T$) are pruned by the keep/prune rule: $\{i,j\}$ is pruned iff $p(i) \neq j$ AND $p(j) \neq i$.

---

## Implementation Notes

### Message Exchange Sequence

After Phase 1 (distance computation) converges:

1. **Round N (distance share)**: Each node broadcasts $\delta_i$ to all neighbors; each node locally computes parent $p(i)$

2. **Round N+1 (parent share)**: Each node broadcasts $p(i)$ to all neighbors (or sends a unicast 1-bit "selected" message to $p(i)$); each node applies the keep/prune rule to all incident edges

3. **Edge state**: Each edge is marked locally by both endpoints; in practice, pruning is **asymmetric enforcement** (if both endpoints agree to prune, the edge is inactive; if either keeps it, it remains active until both agree it is pruned)

### Symmetric Pruning on Undirected Links

Since the graph is undirected and the keep rule is symmetric ($K_{ij} = K_{ji}$), both endpoints of edge $\{i,j\}$ will independently compute the same keep/prune decision:

$$K_{ij} = 1 \iff (p(i) = j \text{ or } p(j) = i) = (p(j) = i \text{ or } p(i) = j) = K_{ji}$$

Thus, no additional synchronization is required. Both endpoints autonomously agree on the status of their shared edge.

### Handling Node Failures (Optional Extension)

If a node or link fails after spanning tree formation, the protocol remains valid: the tree is no longer a spanning tree of the new graph, but CBF can detect the failure and re-trigger a new iteration of phase 1 (distance recomputation) to adapt the spanning tree to the new topology.

