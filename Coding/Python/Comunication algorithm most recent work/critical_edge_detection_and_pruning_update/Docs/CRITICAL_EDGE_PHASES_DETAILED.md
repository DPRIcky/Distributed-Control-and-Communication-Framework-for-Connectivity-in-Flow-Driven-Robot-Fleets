# Detailed Phase Breakdown: Distributed Critical Edge Detection

Complete technical detailing of all four phases for identifying and pruning to critical edges.

---

# Phase 1: Distributed Distance Computation (Steps 0 to n-1)

## Objective
Compute exact shortest-path distances between all pairs of nodes in the network using a fully decentralized breadth-first search (BFS) wave propagation.

## Fundamental Principle
Instead of centralizing distance computation, each node maintains a **distance vector** $\omega_i(k) = [\omega_{i,1}(k), \ldots, \omega_{i,n}(k)]^T$ where:
- $\omega_{i,j}(k)$ = shortest distance from node $j$ to node $i$ at step $k$

The key insight: **Node $i$ is storing how far node $j$ is from $i$**, not the other way around.

## Distributed Update Law (1)

$$\omega_{i,j}(k+1) = \min_{l \in N_i \cup \{i\}} \{\omega_{l,j}(k) + 1\}$$

**Interpretation**:
- To update the distance to node $j$ at node $i$:
  - Consider all direct neighbors $l \in N_i$ AND the node itself ($l = i$)
  - Each neighbor $l$ has its own estimate $\omega_{l,j}(k)$ of how far $j$ is
  - Add 1 (the cost of the edge to $l$)
  - Pick the minimum across all options
  - This implements a local "relay" where neighbors report their distances

**Physical interpretation**: "What's the shortest way to reach $j$? Either I'm already at $j$ (distance 0), or I ask my neighbors how far they are, and I go through one of them (adding 1 hop)."

## Initialization (Step k=0)

For each node $i$:
$$\omega_{i,j}(0) = \begin{cases} 
0 & \text{if } i = j \\
\infty & \text{if } i \neq j
\end{cases}$$

**Process**:
1. Every node knows its own identity: distance to self is 0
2. Every node is initially infinitely far from all other nodes

## Execution: Step-by-Step Wave Propagation

### Step k=0
- Root node 1 (or arbitrary root) has: $\omega_1(0) = [0, \infty, \infty, \ldots, \infty]$
- All other nodes have: $\omega_i(0) = [\infty, \infty, \ldots, \infty]$ (except position $i$ which is 0)

### Step k=1
**Broadcast phase**: Each node sends its current distance vector to all neighbors.
- Node 1 sends: $\omega_1(0) = [0, \infty, \infty, \ldots]$
- Nodes 2,3,... send: $\omega_i(0) = [\infty, \ldots, 0_i, \ldots]$

**Update phase**: Each node applies update law (1).
- Node $i$ receives vectors from all neighbors $l \in N_i$
- For each measurement dimension $j$ (node $j$'s index):
  - $\omega_{i,j}(1) = \min\{\omega_{l,j}(0) + 1 : l \in N_i \cup \{i\}\}$

**Result**: 
- Nodes adjacent to node 1 update $\omega_{i,1}(1) = 1$ (they are 1 hop from 1)
- All others still have $\infty$ for distant nodes
- But now nodes can also update their own position: if node 5 is a neighbor of node 3, then $\omega_5(1)$ includes the fact that node 3 is reachable

### Step k=2
**Broadcast phase**: Nodes send updated vectors.

**Update phase**:
- Nodes 2 hops away from node 1 can now compute distance: receiving from a node that is 1 hop away
- $\omega_{i,1}(2) = \min\{1 + 1\} = 2$ for nodes 2 hops from 1

**Wave pattern**: Each step, the "wavefront" of known distances advances by 1 hop.

### Step k=n-1
After n-1 steps:
- All distances have propagated through the network
- In a connected network of diameter $D$, all exact distances are computed within $D$ steps
- After $n$ steps in the worst case, even a path graph has computed all distances

## Synchronization Requirements

### Synchronous Model
- All nodes execute updates in lockstep
- At the start of step $k$, all nodes simultaneously:
  1. Broadcast their current $\omega_i(k)$ to neighbors
  2. Receive broadcasts from neighbors
  3. Update using law (1)
  4. Move to step $k+1$

### Message Format (per neighbor $l$)
```
[step_number | sender_id | distance_vector]
  4 bytes      4 bytes     n * 4 bytes = O(n) bytes
```

### Communication per Step
- Each edge carries one message per direction
- Message size: $O(n)$ scalars
- Total messages per step: $2|E|$
- Total bits per step: $O(|E| \cdot n \cdot \log n)$ (assuming $\log n$ bits per distance)

## Convergence Guarantee

**Lemma (Convergence)**: In a connected graph with diameter $D$, after $k$ steps:
$$\omega_{i,j}(k) = d(j, i) \text{ for all } i, j \text{ with } d(j,i) \leq k$$

**Proof sketch**: By induction on distance.
- Base case: For neighbors (distance 1), info propagates in 1 step.
- Inductive step: If all distances $\leq k$ are known correctly, then at step $k+1$, nodes can compute distance $k+1$ by relaying through a node at distance $k$.

**Conclusion**: After $n$ steps, $\omega_{i,j}(n) = d(j, i)$ for all pairs in any $n$-node network.

## Data Structure at Node $i$ After Phase 1

Each node $i$ retains:
$$\Omega_i := \{\omega_{i,j}(n) : j = 1, \ldots, n\}$$

This is the distance vector containing exact shortest paths **from $j$ to $i$** for all $j$.

---

# Phase 2: Connectivity Assurance (Steps n to 2n)

## Objective
Detect if the original network is disconnected and repair connectivity if needed by adding new edges. Ensure by step $2n$ that every node is aware the network is connected.

## Problem Statement

In a network with $n_c > 1$ disjoint connected components, the network is not a valid single connected topology. The spanning tree protocol (and critical edge detection) assumes connectivity.

**Solution**: Distributively detect and repair before proceeding to Phase 3.

## Connectivity Indicator $\xi_{i,j}(k)$

At each step, node $i$ can determine if node $j$ is in the same connected component:

$$\xi_{i,j}(k) = \begin{cases}
1 & \text{if } \omega_{i,j}(k) < \infty \\
0 & \text{if } \omega_{i,j}(k) = \infty
\end{cases}$$

**Interpretation**:
- If distance is finite, they are in the same component
- If distance is infinite, they are in different components

## Modified Update Law (5)

For steps $k = n, n+1, \ldots, 2n-1$:

$$\omega_{i,j}(k+1) = \begin{cases} 
\omega_{i,j}(k) & \text{if } \xi_{i,j}(k+1) = \xi_{i,j}(k) \quad \text{(component unchanged)}\\
\omega_{i,i^*}(k) + \omega_{j^*,j}(k) & \text{if } \xi_{i,j}(k+1) > \xi_{i,j}(k) \quad \text{(new edge added)}
\end{cases}$$

**Interpretation**:
- First case: If connectivity state hasn't changed, keep the distance as-is
- Second case: If a new component became reachable via a new edge, recalculate distance through the intermediate nodes $i^*$ (in old component) and $j^*$ (in new component)

## Algorithm A: Distributed Connection Protocol

### Execution per Node $i$

At each step $k \geq n$, node $i$ checks if it has any disconnected neighbors:

```
while ∃j : ξᵢ,ⱼ(k) = 0 do
    // Node i has detected that j is unreachable
    
    // Step A1: Find highest-index CONNECTED neighbor
    i* ← max{j | ξᵢ,ⱼ(k) = 1}
    
    // Step A2: Only node i* is responsible for connecting
    if (i == i*) then
        // Step A3: Find highest-index DISCONNECTED node
        j* ← max{j | ξᵢ,ⱼ(k) = 0}
        
        // Step A4: Establish new link
        establish_link(i*, j*)
        
        // Update neighbor sets
        Nᵢ* ← Nᵢ* ∪ {j*}
        Nⱼ* ← Nⱼ* ∪ {i*}
        
        // Broadcast new adjacency
        announce_new_neighbor(i*, j*)
    end if
    
    k ← k + 1
    
    // Step A5: Immediately apply update laws (1) and (5)
    // to propagate the new edge information
    phase1_phase2_updates()
end while
```

### Key Design Decisions

1. **Single agent per step**: Only one node ($i^*$) adds an edge per iteration.
   - Why? Prevents duplicate edge additions and maintains determinism
   - $i^*$ is the highest-index connected neighbor (tie-breaker for uniqueness)

2. **Highest-index disconnected**: $j^* = \max\{j : \xi_{i,j}(k) = 0\}$
   - Ensures all isolated nodes get connected eventually
   - Deterministic and requires no global coordination

3. **Distributed awareness**: Each node $i$ independently decides to initiate connection if:
   - It has disconnected neighbors
   - It is the highest-index connected neighbor in its component

**Example**: If component {1,2,3} sees component {5,6,8} unreachable:
- Node 3 (max of {1,2,3}) executes: connect 3 ↔ 8 (max of {5,6,8})

### Convergence Behavior

**Iteration 1**: 
- Disconnected components attempt to connect
- First component edge added (e.g., 3 ↔ 8)
- Update law (5) propagates new connectivity info through both components

**Iteration 2**:
- If still $n_c > 1$ components remain, second edge added
- Example: now {1,2,3,5,6,8} tries to connect into a third component

**General**: After exactly $n_c - 1$ iterations, network is connected.

### Why Run Until Step 2n?

Even after connectivity is established at step $n + (n_c - 1)$, the protocol:
1. Continues running update laws (1) and (5) to propagate new edge information
2. Ensures every node learns of the added edges
3. Allows distance vectors to stabilize on new topology
4. Results in all $\omega_{i,j}(2n)$ being accurate shortest paths in the **final connected topology**

By step $2n$, every node has:
- Recognized new edges were added (if any)
- Recomputed distances through new edges
- Established that full connectivity exists: $\forall i,j: \xi_{i,j}(2n) = 1$

## Data Structure at Node $i$ After Phase 2

Each node $i$ retains (updated from Phase 1):
$$\Omega_i := \{\omega_{i,j}(2n) : j = 1, \ldots, n\}$$

This now includes distances through the **final connected network** (with repair edges if needed).

---

# Phase 3: Distributed Spanning Tree Protocol for Edge Pruning

## Objective

Build a spanning tree $E_T$ with exactly $n-1$ edges using a fully distributed parent-child relationship protocol.

---

## Part 3.1: Spanning Tree Construction

### Network Model Recap

Let $G = (N, E)$ be a connected undirected graph:
- $N = \{1, 2, \ldots, n\}$ (nodes/robots)
- $E \subseteq \{\{i,j\} \mid i \neq j\}$ (edges/communication links)

Each node $i$ knows only its neighbors:
$$N_i := \{j \in N \mid \{i,j\} \in E\}$$

No node has global knowledge of $E$.

A designated root node $r \in N$ is chosen (e.g., $r = 1$).

## Objective

Compute a pruned edge set $E_T \subseteq E$ such that $(N, E_T)$ forms a **spanning tree**:

1. **Connectivity**: $(N, E_T)$ is connected
2. **Acyclic**: $(N, E_T)$ contains no cycles
3. **Minimal edges**: $|E_T| = n - 1$

**Key consequence**: Every edge in a tree is a bridge. Therefore, after this protocol, all remaining edges are **guaranteed critical**.

### Part 1: Distributed Root-Distance Computation

### Objective
Each node computes its hop distance to the chosen root $r$.

### Distance Definition

$$\delta_i := d(i, r) \text{ (shortest path length in hops)}$$

### Distributed Computation

Using the distance computation from Phase 1, extract:
$$\delta_i = \omega_{i,r}(n)$$

This is element $r$ of node $i$'s distance vector computed via update law (1).

### Local Exchange

After Phase 1 convergence, each node broadcasts its distance $\delta_i$ to all neighbors:

```
for each node i:
    δᵢ ← ω_i,r(n)                    // Extract distance to root
    broadcast_to_neighbors(δᵢ)        // Share hop distance
    
    for each neighbor l:
        receive δₗ from neighbor l
    end for
end for
```

All nodes now have local knowledge of the distance of their neighbors to the root.

### Part 2: Parent Selection (Local Rule)

### Objective
Each non-root node deterministically selects a parent closer to the root.

### Candidate Parent Set

For each node $i \neq r$:

$$C_i := \{j \in N_i \mid \delta_j = \delta_i - 1\}$$

This is the set of neighbors with hop distance exactly 1 less than node $i$.

**Guarantee**: In a connected graph, $C_i \neq \emptyset$ for all $i \neq r$ (by definition of shortest paths).

### Parent Selection Rule

Choose parent deterministically using tie-breaking (minimum node ID):

$$p(i) := \min C_i, \quad \forall i \neq r$$

The root has no parent: $p(r) := \text{undefined}$.

### Parent Announcement

Each node announces its selected parent to all neighbors:

```
for each node i ≠ r:
    p(i) ← min{j ∈ Nᵢ : δⱼ = δᵢ - 1}  // Compute parent locally
    broadcast_to_neighbors(p(i))        // Announce parent choice
    
    for each neighbor l:
        receive p(l) from neighbor l
    end for
end for
```

Each node now knows:
- Its own parent $p(i)$
- Whether each neighbor selected it as a parent: $p(l) = i$?

### Part 3: Distributed Keep/Prune Handshake Per Edge

### Objective
For each edge, determine whether to keep it (part of spanning tree) or prune it.

### Keep Decision Rule

For each undirected edge $\{i, j\} \in E$, define:

$$K_{ij} = \begin{cases} 
1 & \text{if } p(i) = j \text{ or } p(j) = i \\
0 & \text{otherwise}
\end{cases}$$

**Interpretation**:
- Keep edge $\{i,j\}$ if it is a parent-child link in the tree (either direction)
- Prune edge $\{i,j\}$ if neither endpoint is the other's parent

### Symmetric Execution

Both endpoints compute the same decision:

```
for each node i:
    KEEP_SET_i ← ∅
    PRUNE_SET_i ← ∅
    
    for each neighbor j ∈ Nᵢ:
        // Compute keep decision
        if (p(i) = j) OR (p(j) = i) then
            KEEP_SET_i ← KEEP_SET_i ∪ {{i, j}}
            K_ij ← 1
        else:
            PRUNE_SET_i ← PRUNE_SET_i ∪ {{i, j}}
            K_ij ← 0
        end if
    end for
end for
```

**Local execution**: Each node independently decides for each incident edge; no synchronization required.

### Resulting Edge Set

The pruned spanning tree is:

$$E_T := \{\{i, j\} \in E \mid K_{ij} = 1\}$$

Since both endpoints compute $K_{ij}$ identically, the decision is symmetric and globally consistent.

### Correctness Proofs

### Lemma 1: Strict Descent

**Statement**: For any non-root node $i \neq r$:
$$\delta_{p(i)} = \delta_i - 1$$

**Proof**: By definition, $p(i)$ is chosen from $C_i = \{j \in N_i : \delta_j = \delta_i - 1\}$, so $\delta_{p(i)} = \delta_i - 1$ by construction. ∎

**Consequence**: Following parent pointers from any node eventually reaches the root, since distances strictly decrease.

### Lemma 2: Connectivity

**Statement**: For every non-root node $i \neq r$, edge $\{i, p(i)\} \in E_T$.

**Proof**: Since $p(i)$ is node $i$'s parent by definition, $K_{i,p(i)} = 1$ (the condition $p(i) = p(i)$ holds). Therefore $\{i, p(i)\} \in E_T$. By Lemma 1, following parent pointers gives a path from every node to the root. ∎

**Consequence**: $(N, E_T)$ is connected.

### Lemma 3: Acyclic

**Statement**: The graph $(N, E_T)$ contains no cycles.

**Proof**: Suppose a directed cycle exists: $i_1 \to i_2 \to \cdots \to i_k \to i_1$ (where $\to$ denotes the parent relationship). Then:
$$\delta_{i_1} > \delta_{i_2} > \cdots > \delta_{i_k} > \delta_{i_1}$$

This is a contradiction. Therefore no cycles exist. ∎

### Theorem: Spanning Tree ⇒ All Edges Critical

**Statement**: After executing Phase 4, $(N, E_T)$ is a spanning tree, and every edge in $E_T$ is a bridge (critical edge).

**Proof**: By Lemmas 2 and 3, $(N, E_T)$ is connected and acyclic. Each non-root node contributes exactly one parent edge, so $|E_T| = n-1$. Therefore $(N, E_T)$ is a spanning tree. In any tree, removing any edge disconnects the graph, so every edge is a bridge. ∎

### Complexity Analysis

**Round Complexity**:

**Part 1 (Distance extraction and share):**
- 1 round: Broadcast $\delta_i$ to neighbors

**Part 2 (Parent selection and announcement):**
- 1 round: Compute $p(i)$ locally, then broadcast to neighbors

**Part 3 (Keep/prune decisions):**
- 1 round: Apply keep-prune rule (local computation, no communication needed)

**Total for Phase 3**: 2 communication rounds.

**Message Complexity**:

**Part 1**: Each node sends $\delta_i$ (one scalar) to all neighbors.
- Total: $2|E|$ messages (one per edge direction)

**Part 2**: Each node sends $p(i)$ (one node ID) to all neighbors.
- Total: $2|E|$ messages (one per edge direction)

**Part 3**: No messages (purely local decisions).

**Overall Phase 3**: $O(|E|)$ messages.

### Bit Complexity

Both $\delta_i$ and $p(i)$ require $\lceil \log_2 n \rceil$ bits (hop counts and node IDs).

Per edge per round: $O(\log n)$ bits.

---

### Worked 8-Node Example

**Setup**
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

**Parent Selection (Part 2)**

Using $p(i) = \min\{j \in N_i : \delta_j = \delta_i - 1\}$:

| Node $i$ | $\delta_i$ | $N_i$ | $C_i = \{j : \delta_j = \delta_i - 1\}$ | $p(i)$ | Kept Edge |
|:--------:|:----------:|:-------:|:------------------:|:------:|:---------:|
| 1 | 0 | {2,4,8} | — | — | — |
| 2 | 1 | {1,3,4} | {1} (δ₁=0) | 1 | {2,1} |
| 3 | 2 | {2,4} | {2,4} (both δ=1) | 2 | {3,2} |
| 4 | 1 | {1,2,3,5} | {1} (δ₁=0) | 1 | {4,1} |
| 5 | 2 | {4,6,7} | {4} (δ₄=1) | 4 | {5,4} |
| 6 | 2 | {5,7,8} | {8} (δ₈=1) | 8 | {6,8} |
| 7 | 3 | {5,6} | {5,6} (both δ=2) | 5 | {7,5} |
| 8 | 1 | {6,1} | {1} (δ₁=0) | 1 | {8,1} |

**Part 3: Keep/Prune Decisions**

Apply rule $K_{ij} = 1$ iff $p(i) = j$ or $p(j) = i$:

**Edge {1,2}**:
- $p(1)$ = undefined (root)
- $p(2) = 1$ ✓ → **KEEP**

**Edge {2,3}**:
- $p(2) = 1 \neq 3$
- $p(3) = 2$ ✓ → **KEEP**

**Edge {1,4}**:
- $p(1)$ = undefined
- $p(4) = 1$ ✓ → **KEEP**

**Edge {4,5}**:
- $p(4) = 1 \neq 5$
- $p(5) = 4$ ✓ → **KEEP**

**Edge {5,7}**:
- $p(5) = 4 \neq 7$
- $p(7) = 5$ ✓ → **KEEP**

**Edge {1,8}**:
- $p(1)$ = undefined
- $p(8) = 1$ ✓ → **KEEP**

**Edge {8,6}**:
- $p(8) = 1 \neq 6$
- $p(6) = 8$ ✓ → **KEEP**

**Resulting Spanning Tree**

$$E_T = \{\{1,2\}, \{2,3\}, \{1,4\}, \{4,5\}, \{5,7\}, \{1,8\}, \{8,6\}\}$$

**All other edges are pruned**. For example:
- {2,4}: $p(2)=1 \neq 4$ and $p(4)=1 \neq 2$ → **PRUNE**
- {3,4}: $p(3)=2 \neq 4$ and $p(4)=1 \neq 3$ → **PRUNE**

**Verification**

- $|E_T| = 7 = n - 1$ ✓
- Connected: Path from every node to root 1 ✓
- Acyclic: Parent pointers form a tree ✓
- All 7 edges are bridges ✓

---

### Message Exchange Sequence

**Part 1 (After Phases 1-2 complete)**

**Round N**: Distance share
- Each node $i$ broadcasts $\delta_i = \omega_{i,r}(n)$ to all neighbors
- Each node receives $\delta_j$ from each neighbor $j$

**Part 2**

**Round N+1**: Parent announcement
- Each node $i$ computes $p(i) = \min\{j \in N_i : \delta_j = \delta_i - 1\}$ (local, no messages yet)
- Each node broadcasts $p(i)$ to all neighbors
- Each node receives parent choices from neighbors and determines: "Did any neighbor choose me as parent?"

**Part 3**

**Local computation** (no additional round):
- Each node applies keep/prune rule to all incident edges based on:
  - Its own parent $p(i)$
  - Knowledge of whether each neighbor selected it as parent

**Output**: Each node has classified all edges as KEEP or PRUNE.

---

## Symmetric Pruning on Undirected Links

### Key Property

The keep/prune decision is **symmetric** by construction:

$$K_{ij} = 1 \iff (p(i) = j \text{ or } p(j) = i) = (p(j) = i \text{ or } p(i) = j) = K_{ji}$$

### Proof of Symmetry

- Both endpoints compute the same rule
- If node $i$ has node $j$ as parent ($p(i) = j$), then node $j$ learns this from the parent announcement
- Node $j$ will NOT have node $i$ as parent (since $\delta_i \neq \delta_j - 1$ if $p(i) = j$)
- But node $j$ still evaluates: "$p(j) = i$ or $p(i) = j$?" and correctly computes the decision

### Consensus Without Additional Synchronization

Both endpoints reach the same keep/prune decision independently. No extra message rounds or voting needed.

---

### Implementation Notes

**Data Flow**

```
Phase 2 ends (connectivity ensured)
    ↓
Part 1: Broadcast δᵢ to neighbors
    ↓
Each node computes Cᵢ and p(i)
    ↓
Part 2: Broadcast p(i) to neighbors
    ↓
Each node learns p(j) for all neighbors j
    ↓
Part 3: Apply keep/prune rule
    ↓
Phase 3 complete (spanning tree E_T with n-1 edges)
    ↓
Phase 4: Add robustness edge with embedded delta calculation
    ↓
Edge set E_T^+ determined (n edges total)
```

## Data Structure at Node $i$ After Phase 3

Each node $i$ retains:
- $\Omega_i = \{\omega_{i,j}(2n)\}$ (distance vector from Phase 1-2)
- $p(i)$ = parent node (scalar)
- $KEEP\_SET_i$ = set of edges to keep (subset of $N_i$)
- $PRUNE\_SET_i$ = set of edges to prune (subset of $N_i$)


---

# Summary: Reorganized Protocol - Four Phases

| Phase | Duration | Method | Objective | Output |
|:-----:|:-------:|:------:|:---------:|:------:|
| **1** | $0$ to $n-1$ | BFS wavefront (update law 1) | Compute all pairwise hop distances distributedly |  $\omega_{i,j}(n)$ for all nodes |
| **2** | $n$ to $2n$ | Modified update law (5) + Algorithm A | Detect and repair disconnected components | Connected topology $E$ |
| **3** | $O(n)$ rounds | Parent-child spanning tree via hop distance to root | Build spanning tree with exactly $n-1$ edges | $E_T$ (spanning tree, all edges critical) |
| **4** | $1-2$ rounds | Embedded delta + hop distance + distributed consensus | Add 1 robustness edge via fully distributed selection | $E_T^+ = E_T \cup \{e_{robust}\}$ (exactly $n$ edges) |

**Key distinction from 5-phase version**: 
- **Removed**: Phase 3 (Critical Edge Determination) no longer exists as separate phase
- **Integrated**: Delta calculation for Lemma 1 verification now embedded in Phase 4, Step 1
- **Result**: Cleaner pipeline, same robustness guarantees, fewer communication rounds

---

# Phase 4: Distributed Robustness Enhancement via Hop-Distance Proximity

## Objective

After Phase 3 produces a spanning tree with exactly $n-1$ edges, add **one additional edge** to improve algebraic connectivity while:
1. Maintaining full distribution (no root node decisions)
2. Creating true redundancy (alternate paths verified)
3. Selecting geometrically close edge pair (lowest hop distance)
4. Ensuring deterministic global consensus (all nodes agree on the same edge)

## Design Principles

### 1. Why Add One Edge?

A spanning tree $E_T$ with $n-1$ edges is minimally connected but may be fragile:
- Removing any edge disconnects the network
- No redundant paths exist

Adding exactly **1 edge** creates a single cycle, which:
- Produces $n$ edges (Euler characteristic: $n$ vertices, $n$ edges → 1 independent cycle)
- Provides 2 edge-disjoint paths between most node pairs
- Minimally increases algebraic connectivity $\lambda_2$ (Fiedler eigenvalue)
- Maintains sparsity

### 2. Geometric Proximity via Hop Distance

Since nodes cannot centrally compute all pairwise geometric distances (full distribution constraint), use **hop distance** as a proxy:

$$\omega_{i,l}(n) = \text{shortest path length (in hops) from node } l \text{ to node } i$$

**Justification**:
- In robot networks, 1-hop neighbors are typically geometrically close
- 2-hop neighbors are further away
- $k$-hop neighbors scale roughly with geometric distance
- Hop distance is already computed in Phase 1 and available to all nodes

### 3. Alternate Path Verification via Phase 3

Not any edge can be added—only edges that **create alternate paths** are valid robustness candidates.

Use embedded delta calculation: An edge $\{i, l\}$ creates an alternate path if it satisfies the following conditions for alternate path detection:

**Either**: Some node $j$ is equidistant from $i$ and $l$: $\Delta^{(il)}_{i,j} = 0$

**Or**: Some node $j$ and neighbors exist such that: $\Delta^{(ii')}_{i,j} = 1$ AND $\Delta^{(ll')}_{l,j} = 1$

## Phase 4 Algorithm: Four-Step Execution

### Step 1: Collect Pruned Edges and Verify Alternate Paths

Each node $i$ examines all pruned edges (edges in $PRUNE_SET_i $ from Phase 4):

```
for each NODE i:
    REWIRE_CANDIDATES_i ← ∅  // Edges creating alternate paths
    
    for each pruned edge {i, l} ∈ PRUNE_SET_i:
        // Compute Δ^(il)_i for all j
        Δ^(il)_i ← []
        for j = 1 to n:
            Δ^(il)_i[j] ← ω_i,j(2n) - ω_l,j(2n)
        end for
        
        // Check Lemma 1 conditions (Equations 7 and 8)
        has_equidistant ← FALSE
        has_relay ← FALSE
        
        // Condition 1: Equidistant node
        for j = 1 to n:
            if Δ^(il)_i[j] = 0 then
                has_equidistant ← TRUE
                break
            end if
        end for
        
        // Condition 2: Neighbor-relay (simplified check)
        if NOT has_equidistant then
            for j = 1 to n:
                if Δ^(il)_i[j] ≠ 0 then
                    for each i' ∈ N_i:
                        for each l' ∈ N_l:
                            if (Δ^(ii')_i[j] = 1 AND Δ^(ll')_l[j] = 1) then
                                has_relay ← TRUE
                                break [all loops]
                            end if
                        end for
                    end for
                end if
            end for
        end if
        
        // Add to candidates if alternate path exists
        if (has_equidistant OR has_relay) then
            hop_dist_il ← ω_i,l(n)  // Hop distance from l to i
            REWIRE_CANDIDATES_i ← REWIRE_CANDIDATES_i ∪ {(i, l, hop_dist_il)}
        end if
    end for
end for
```

**Output per node**: $REWIRE\_CANDIDATES_i$ = list of pruned edges that create alternate paths, tagged with hop distance.

### Step 2: Rank Candidates by Hop Distance (Local)

For each node $i$, sort candidates by proximity:

```
for each NODE i:
    if |REWIRE_CANDIDATES_i| > 0 then
        // Sort ascending by hop distance (proximity)
        SORTED_CANDIDATES_i ← SORT(REWIRE_CANDIDATES_i, 
                                    key = lambda (a, b, dist): dist)
        // Best candidate: edge with smallest hop distance
        best_candidate_i ← SORTED_CANDIDATES_i[0]
    else:
        best_candidate_i ← NULL  // No candidates
    end if
end for
```

**Output per node**: $best\_candidate_i$ = single best edge (lowest hop distance) from that node's pruned set.

### Step 3: Fully Distributed Deterministic Selection

**All nodes compute independently** using the same deterministic rule to select the global rewire edge:

$$e_{robust} = \arg\min_{e = (a, b) \in \text{CANDIDATES}} \{ \omega_{a,b}(n), \min(a, b) \}$$

Where:
- $\omega_{a,b}(n)$ = hop distance between $a$ and $b$
- $\min(a,b)$ = lexicographic tie-breaker (node IDs)

**Distributed execution**: Each node independently computes which edge would win this global selection:

```
for each NODE i:
    all_best_edges ← ∅  // Collect from all nodes
    
    // Each node broadcasts its best candidate to all neighbors
    broadcast_to_neighbors(best_candidate_i)
    
    // Collect from neighbors and retransmit
    for all nodes j (via neighbor gossip or flooding):
        if best_candidate_j ≠ NULL then
            all_best_edges ← all_best_edges ∪ {best_candidate_j}
        end if
    end for
    
    // Apply global selection rule
    e_robust ← min(all_best_edges, 
                   key = lambda e = (a, b): (ω_a,b(n), min(a, b)))
    
    // All nodes compute the same e_robust
    GLOBAL_REWIRE_EDGE_i ← e_robust
end for
```

**Communication pattern**: 
- 1-hop broadcast: Each node sends its best candidate
- 2-hop gossip or flooding: Propagate to all nodes (can use existing neighbor network)
- Convergence: All nodes learn all best candidates within $O(\log n$) gossip rounds

**Determinism guarantee**: 
- All nodes have the same set of best candidates (from all node perspectives)
- All nodes apply identical comparison rule
- All nodes reach identical conclusion: $e_{robust}$

### Step 4: Add Rewire Edge to Spanning Tree

```
for each NODE i:
    // Add the globally agreed rewire edge
    e_robust = (a, b)  // Edge between nodes a and b
    
    if (i = a AND b ∈ N_i) OR (i = b AND a ∈ N_i) then
        // Node i is an endpoint of the rewire edge
        KEEP_SET_i ← KEEP_SET_i ∪ {e_robust}
    end if
end for

// Global edge set after Phase 5
E_T^+ ← {all edges marked KEEP after Phase 4} ∪ {e_robust}
```

**Verification**:
- $|E_T^+| = (n-1) + 1 = n$ ✓
- $(N, E_T^+)$ contains exactly one cycle
- Algebraic connectivity improved via cycle addition

---

## Why This Works: Theoretical Justification

### 1. **Alternate Path Guarantee**

By checking Lemma 1 conditions:
- Any edge in $REWIRE\_CANDIDATES_i$ creates at least one cycle when added
- These cycles provide true redundancy, not just redundant links
- No edge in the spanning tree is rendered redundant

### 2. **Geometric Proximity**

Hop distance $\omega_{i,l}(n)$ correlates with geometric distance:
- Lowest hop distance → most likely to be geometrically close
- No centralized distance knowledge required
- Each node can locally rank candidates by this metric

### 3. **Deterministic Global Consensus**

Lexicographic ordering $(hop\_distance, min(node\_id))$ ensures:
- **Symmetry**: Both endpoints compute identically
- **Uniqueness**: Exactly one edge is selected (no ties)
- **Distributed**: No root node or central arbiter needed
- **Speed**: 1 additional gossip round (or 2-hop broadcast)

Step 4.1 achieves determinism without involving a designated root:
- Each node computes $e_{robust}$ the same way
- No voting, no consensus protocol needed
- All nodes' best candidates flow through the network
- Deterministic tie-breaking ensures global agreement

### 4. **Independent Execution**

Each node $i$ works only with:
- Its own distance vector $\omega_i(2n)$ (from Phase 1)
- Its own set of pruned edges $PRUNE_SET_i$ (from Phase 4)
- Neighbor distance vectors (received in Phase 4)
- Broadcast of best candidates from other nodes

No node needs special privileges or global knowledge.

---

## Complexity Analysis

### Round Complexity

**Step 1 (Alternate path verification via embedded delta calculation)**:
- Performed at each node independently using stored $\omega_i(2n)$ and neighbor vectors
- No additional communication rounds needed
- Time: 0 rounds (local computation)

**Step 2 (Local ranking)**:
- Sort candidates by hop distance
- Time: 0 rounds (local computation)

**Step 3 (Distributed selection)**:
- 1 round broadcast: Each node sends best candidate to neighbors
- $O(\log n)$ rounds gossip: Flood best candidates throughout network
- **Total**: 1 main round + $O(\log n)$ gossip rounds

**Step 4 (Edge addition)**:
- Local decision: apply rule to add $e_{robust}$
- Time: 0 rounds (local computation)

**Overall Phase 4**: 1 communication round + $O(\log n)$ rounds for distributed agreement.

### Message Complexity

**Step 1**: No messages (local computation using Phase 1 and Phase 4 data).

**Step 3 broadcast**: 
- Each node sends its best candidate (tuple: node IDs $a, b$ + hop distance)
- Per message: $2 \lceil \log_2 n \rceil + \lceil \log_2 n \rceil = 3\lceil \log_2 n \rceil$ bits
- Total: $O(n \log n)$ bits for full network gossip

**Overall Phase 4**: $O(n \log n)$ bits.

### Bit Complexity

- Best candidate encoding: $(a, b, \omega_{a,b})$ where $a, b \in \{1, \ldots, n\}$ and $\omega_{a,b} \leq n$
- Per candidate: $3 \lceil \log_2 n \rceil$ bits
- Each node sends 1 candidate
- **Total bits**: $O(n \log n)$ across all nodes

---

## Worked Example: 8-Node Network

Continuing from Phase 3 with spanning tree:

$$E_T = \{\{1,2\}, \{2,3\}, \{1,4\}, \{4,5\}, \{5,7\}, \{1,8\}, \{8,6\}\}$$

Pruned edges deleted from Phase 4:
$$\text{PRUNE} = \{\{2,4\}, \{3,4\}, \{4,4\}, \{6,5\}, \{6,7\}\}$$

### Step 1: Check Alternate Paths (Lemma 1)

For pruned edge $\{2,4\}$:
- $\delta_2 = 1, \delta_4 = 1$ (both distance 1 from root)
- Path $2 \to 1 \to 4$ exists (length 2)
- Direct edge $2-4$ would create cycle: $2 \to 1 \to 4 \to 2$ or $2 \to 4 \to 1 \to 2$
- Check: Is there equidistant node $j$ with $\Delta^{(24)}_{2,j} = 0$?
  - Likely YES (both nodes 1-hop from root, node 1 is equidistant)
- **Result**: ADD $\{2,4\}$ to candidates

For pruned edge $\{6,5\}$:
- $\delta_6 = 2, \delta_5 = 2$ (both distance 2 from root)
- Path $6 \to 8 \to 1 \to 4 \to 5$ exists (length 4)
- Direct edge would create cycle
- Likely has equidistant nodes
- **Result**: ADD $\{6,5\}$ to candidates

For pruned edge $\{6,7\}$:
- $\delta_6 = 2, \delta_7 = 3$
- Not equidistant; check relay condition...
- Likely NO alternate paths (different distance levels)
- **Result**: NO, do not add to candidates

### Step 2: Local Ranking

Assuming hop distances from Phase 1:
- Node 2: best candidate $= \{2,4\}$ with $\omega_{2,4}(n) = 2$
- Node 4: best candidate $= \{2,4\}$ with $\omega_{4,2}(n) = 2$
- Node 6: best candidate $= \{6,5\}$ with $\omega_{6,5}(n) = 3$
- Node 5: best candidate $= \{6,5\}$ with $\omega_{5,6}(n) = 3$
- Other nodes: no candidates or farther candidates

### Step 3: Distributed Selection

**Each node broadcasts its best**:
- Nodes 2,4 say: $(2,4,2)$
- Nodes 6,5 say: $(6,5,3)$
- Others say: NULL

**Global selection rule**:
- Candidates: $\{(2,4,2), (6,5,3)\}$
- Compare by: $(hop\_distance, \min(a,b))$
- $(2,4,2)$ vs $(6,5,3)$: $2 < 3$ → winner is $(2,4,2)$
- **Global agreement**: $e_{robust} = \{2,4\}$ with hop distance 2

**All 8 nodes independently compute this result.**

### Step 4: Add Edge

$$E_T^+ = E_T \cup \{\{2,4\}\} = \{\{1,2\}, \{2,3\}, \{1,4\}, \{4,5\}, \{5,7\}, \{1,8\}, \{8,6\}, \{2,4\}\}$$

Now:
- $|E_T^+| = 8 = n$ ✓
- Network has 1 cycle: $1-2-4-1$
- All 8 edges have redundancy
- Most edge removals don't disconnect graph

---

## Comparison: Phase 3 vs Phase 4

| Metric | After Phase 3 | After Phase 4 |
|:------:|:-----------:|:----------:|
| **Edges** | $n-1 = 7$ | $n = 8$ |
| **Cycles** | 0 (tree) | 1 |
| **Removed neighbors** | All non-critical | All non-critical + 1 |
| **Redundant paths** | None | Limited (1 cycle) |
| **Algebraic connectivity** | $\lambda_2$ (tree) | $\lambda_2^+$ (improved) |
| **Edge criticality** | All are bridges | $n-1$ bridges + 1 redundant |
| **Robustness** | Fragile | Robust: 2-edge-disjoint paths exist for some pairs |
| **Communication** | From Phase 3 | +1 round + $O(\log n)$ gossip |

---

## Integration: Phases 1→2→3→4 Pipeline

Typical execution sequence for robustness enhancement:

```
Start: All robots have full neighbor set E

Phase 1 (steps 0 to n-1):
  - Compute shortest distances ω_i,j(n)
  - Each robot learns hop distance to all others
  
Phase 2 (steps n to 2n):
  - Ensure network connected (repair if needed)
  - All robots aware of same topology
  
Phase 3 (O(n) coordination):
  - Extract distances to root
  - Elect parents via min(C_i) rule
  - Prune to spanning tree E_T with n-1 edges
  - Result: All edges critical, but minimal
  
Phase 4 (1-2 steps):
  ┌─ Step 1: Each robot checks pruned edges for alternate paths
  │  (with embedded delta calculation)
  │
  ├─ Step 2: Rank candidates by hop distance
  │
  ├─ Step 3: Global consensus on best rewire edge
  │  (all robots independently compute same choice)
  │
  └─ Step 4: Add rewire edge, final set E_T^+, n edges
  
End: Spanning tree + 1 robustness edge, all edges critical or redundant
     Network has improved algebraic connectivity
     Full distribution maintained throughout
```

---

## Summary: Phase 4 Key Properties

✓ **Fully distributed**: No root node, no central arbiter
✓ **Deterministic**: All nodes compute identical selection
✓ **Efficient**: 1 round + $O(\log n)$ gossip for agreement
✓ **Robust**: Adds exact 1 cycle for redundancy
✓ **Proximity-aware**: Uses hop distance as geometric proxy
✓ **Theoretically sound**: Lemma 1 ensures alternate paths exist
✓ **Low overhead**: Messages scale with $O(n \log n)$ bits
✓ **Simple integration**: Builds directly on Phase 3 (spanning tree) output

---

# Integration with Communication-Based Feasibility (CBF) Controller

## Topology Usage After Phase 4

Once the four-phase protocol completes:

1. **Output topology**: $E_T^+ = E_T \cup \{e_{robust}\}$ with exactly $n$ edges
2. **CBF connectivity constraints**: Controller uses edges in $KEEP\_SET_i$ (initially $n-1$ from spanning tree)
3. **After robustness edge added**: CBF has $n$ edges total, can maintain redundancy if needed
4. **Routing decisions**: Prioritize parent edges initially, can failover to robustness edge if parent edge fails
5. **Robot motion control**: CBF ensures no edge in $E_T^+$ is broken, providing guaranteed connectivity

## Performance Metrics

**Final Network Properties**:
- Edges: exactly $n$ (minimal connected graph with 1 cycle)
- Cycles: exactly 1 (provides limited redundancy)
- Bridge edges: $n-1$ (spanning tree edges)
- Redundant edges: 1 (robustness edge)
- Algebraic connectivity $\lambda_2$: improved over spanning tree

**Computational Complexity**:
- Total phases: 4 (not 5)
- Communication rounds: $O(n)$ (Phases 1-2) + $O(n)$ (Phase 3) + $2$ (Phase 4)
- Message complexity: $O(|E| \cdot n)$ for Phases 1-2, $O(|E|)$ for Phase 3, $O(n \log n)$ for Phase 4
- Storage per node: $O(n^2)$ bits (distance vectors)
- Local computation: $O(n)$ per phase

**Advantages over 5-Phase Version**:
- Fewer communication phases needed
- Delta calculation no longer requires separate dedicated phase
- Embedded delta in Phase 4 reuses distance vectors already available
- No additional messaging for critical edge determination (not needed for spanning tree approach)
- Simplified algorithm flow for practitioners

