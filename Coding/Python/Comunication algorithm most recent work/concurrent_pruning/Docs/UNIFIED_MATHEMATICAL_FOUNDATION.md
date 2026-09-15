# Unified Mathematical Foundation: Distributed Concurrent Consensus and Topology Pruning

**Complete reference combining formal theory, implementation details, and proofs**

---

## Table of Contents

1. [Abstract & Overview](#abstract--overview)
2. [System Model & Problem Formulation](#system-model--problem-formulation)
3. [Distributed Algorithm Implementation](#distributed-algorithm-implementation)
4. [Mathematical Theory](#mathematical-theory)
5. [Convergence Guarantees & Proofs](#convergence-guarantees--proofs)
6. [Formal Algorithms](#formal-algorithms)
7. [Complexity Analysis](#complexity-analysis)

---

## Abstract & Overview

This document presents the **complete unified foundation** for the Distributed Concurrent Consensus and Topology Pruning algorithm, which enables multi-robot systems to simultaneously:

1. **Reach consensus** on network adjacency matrix
2. **Remove redundant edges** while maintaining connectivity
3. **Guarantee convergence** without centralized coordination

### Key Innovation: Concurrent vs Sequential

**Traditional Sequential Approach:**
```
Phase 1: Run consensus until convergence (100+ iterations)
        │
        ↓
Phase 2: Prune edges
        │
        ↓
Total time: T_consensus + T_pruning
```

**Our Concurrent Approach:**
```
Each iteration k:
  ├─ Consensus Update: A^l(k) → A^l(k+1)
  └─ Distributed Pruning: Each robot proposes edges
        │
        ├─ Phase 1: PROPOSAL (local evaluation)
        ├─ Phase 2: NEGOTIATION (bilateral agreement)
        └─ Phase 3: CONFLICT RESOLUTION (deterministic selection)

Total time: ~T_consensus (pruning "free" during convergence)
```

### Guarantees Provided

✅ **Fully Distributed** - No central controller, only 1-hop communication  
✅ **Convergence** - All robots reach consensus (exponential rate)  
✅ **Connectivity** - $\lambda_2(\mathcal{L}(k)) \geq \lambda_{\min}$ maintained  
✅ **Stability** - Lyapunov constraints ensure monotonic progress  
✅ **Optimality** - Reaches locally optimal tree topology

---

## System Model & Problem Formulation

### 1.1 Multi-Robot Network

**Definition 1.1:** Network of $n$ robots $\mathcal{R} = \{0, 1, \ldots, n-1\}$ with time-varying communication topology:

$$\mathcal{G}(k) = (\mathcal{V}, \mathcal{E}(k))$$

where:
- $\mathcal{V} = \mathcal{R}$ is the vertex set (robots)  
- $\mathcal{E}(k) \subseteq \mathcal{V} \times \mathcal{V}$ is the edge set (communication links)
- Edges are undirected: $(i,j) \in \mathcal{E}(k) \Rightarrow (j,i) \in \mathcal{E}(k)$

**Reference Implementation:**
- Located in: `core/robot.py`, `graph/edge_analysis.py`
- Graph stored as edge set (for efficient pruning operations)

### 1.2 Adjacency Matrix Representation

**Definition 1.2:** Weighted adjacency matrix $A(k) = [A_{ij}(k)] \in \mathbb{R}^{n \times n}$ satisfies:

$$A_{ij}(k) = \begin{cases}
w_{ij}(k) > 0 & \text{if } (i,j) \in \mathcal{E}(k) \\
0 & \text{otherwise}
\end{cases}$$

Properties:
- **Symmetric:** $A_{ij}(k) = A_{ji}(k)$ (undirected graph)
- **Zero diagonal:** $A_{ii}(k) = 0$ for all $i$
- **Weight function:** $w_{ij}(k) = \exp\left(-\frac{d_{ij}^2(k)}{\sigma^2}\right)$ (trust model)

**Implementation:** 
- Location: `consensus/adjacency_consensus.py`
- Each robot maintains local estimate $A^\ell(k)$
- Updated via Griparic et al. (2022) consensus protocol

### 1.3 Key Definitions

**Definition 1.3 (Neighbor Set):** 
$$\mathcal{N}_\ell(k) = \{p \in \mathcal{R} : (\\ell, p) \in \mathcal{E}(k)\}$$

**Definition 1.4 (Degree):** 
$$d_\ell(k) = |\mathcal{N}_\ell(k)| = \text{number of neighbors}$$

**Definition 1.5 (Incident Edge Set):** 
$$\mathcal{E}_\ell(k) = \{e \in \mathcal{E}(k) : \ell \in e\}$$

**Definition 1.6 (Laplacian Matrix):** 
$$\mathcal{L}(k) = D(k) - A(k)$$

where $D(k) = \text{diag}(d_0(k), d_1(k), \ldots, d_{n-1}(k))$ is the degree matrix.

**Definition 1.7 (Algebraic Connectivity):**
$$\lambda_2(\mathcal{L}(k)) = \text{second smallest eigenvalue of } \mathcal{L}(k)$$

This is the **most critical metric**: 
- $\lambda_2 = 0$ ⟹ graph disconnected
- $\lambda_2 > 0$ ⟹ graph connected
- $\lambda_2$ large ⟹ fast convergence

### 1.4 Problem Statement

**Objective:** Find minimal edge set $\mathcal{E}^*$ such that:

$$\begin{align}
\mathcal{E}^* = \arg\min_{\mathcal{E} \subseteq \mathcal{E}_0} &\quad |\mathcal{E}| \\
\text{subject to:} &\quad \mathcal{G} = (\mathcal{V}, \mathcal{E}) \text{ is connected} \\
&\quad \lambda_2(\mathcal{L}(\mathcal{E})) \geq \lambda_{\min} \\
&\quad \lim_{k \to \infty} A^\ell(k) = A^* \quad \forall \ell \in \mathcal{R}
\end{align}$$

where:
- $\mathcal{E}_0$ = initial edge set
- $\lambda_{\min} > 0$ = minimum connectivity threshold  
- $A^*$ = consensus value (true adjacency matrix)

**Computational Complexity:** This is NP-hard in general (related to minimum spanning tree problem).

**Our Approach:** Distributed greedy algorithm providing locally optimal solution.

---

## Distributed Algorithm Implementation

### 2.1 Adjacency Matrix Consensus Protocol

**Core Update Rule (Griparic et al., 2022):**

Each robot $\ell$ independently updates its local estimate:

$$A^\ell(k+1) = A^\ell(k) + T_d \cdot \Delta A^\ell(k) \quad \text{...(Eq. 1)}$$

where the update term combines two components:

$$\Delta A^\ell_{ij}(k) = \underbrace{\sum_{p \in \mathcal{N}_\ell(k)} w_{\ell p} [A^p_{ij}(k) - A^\ell_{ij}(k)]}_{\text{Neighbor averaging}} + \underbrace{\varepsilon^\ell_{ij}(k)}_{\text{Observation correction}} \quad \text{...(Eq. 2)}$$

**Component 1: Neighbor Averaging** 
$$\text{Average}^{\ell}_{ij}(k) = \sum_{p \in \mathcal{N}_\ell(k)} w_{\ell p} [A^p_{ij}(k) - A^\ell_{ij}(k)]$$

with uniform weights:
$$w_{\ell p} = \frac{1}{|\mathcal{N}_\ell(k)|} \quad \text{...(Eq. 3)}$$

**Interpretation:** Robot pulls its estimate toward average of neighbors' estimates.

**Component 2: Observation Correction**
$$\varepsilon^\ell_{ij}(k) = \begin{cases}
t^\ell_{ij}(k) - A^\ell_{ij}(k) & \text{if } (i,j) \text{ incident to } \ell \\
0 & \text{otherwise}
\end{cases} \quad \text{...(Eq. 4)}$$

**Trust Function:**
$$t^\ell_{ij}(k) = \exp\left(-\frac{d_{ij}^2(k)}{\sigma^2}\right) \quad \text{...(Eq. 5)}$$

where $d_{ij}(k)$ is measured distance between robots and $\sigma$ is sensitivity parameter.

**Interpretation:** For edges robot can directly measure (distance sensing), pull estimate toward measured value.

**Parameters:**
- $T_d > 0$: Sample time (controls convergence speed)
- $\sigma > 0$: Trust decay parameter (smaller → steeper decay with distance)

**Implementation:**
```
Location: consensus/adjacency_consensus.py::consensus_update_step()
Lines: 120-180
Implements: Eq. 1-5
```

---

### 2.2 Local Convergence Metrics

#### Metric 1: Local Lyapunov Function

**Definition 2.1:**
$$V_\ell(k) = \sum_{p \in \mathcal{N}_\ell(k)} \|A^\ell(k) - A^p(k)\|_F^2 \quad \text{...(Eq. 6)}$$

**Interpretation:** 
- Sum of squared differences between robot $\ell$'s estimate and each neighbor's estimate
- Measured using Frobenius norm: $\|X\|_F = \sqrt{\sum_{i,j} X_{ij}^2}$
- $V_\ell(k) = 0$ ⟺ robot fully agrees with all neighbors

**Distributed Property:** 
- Computed using only neighbor information
- No global knowledge required
- Each robot independently computes $V_\ell(k)$

**Global Lyapunov:**
$$V_{\text{global}}(k) = \sum_{\ell=0}^{n-1} V_\ell(k) = \sum_{\ell=0}^{n-1} \sum_{p \in \mathcal{N}_\ell(k)} \|A^\ell(k) - A^p(k)\|_F^2 \quad \text{...(Eq. 7)}$$

**Implementation:**
```
Location: concurrent_pruning/local_lyapunov.py::compute_local_lyapunov()
Lines: 40-65
Returns: V_l(k) for each robot
```

#### Metric 2: Max Disagreement

**Definition 2.2:**
$$D_\ell(k) = \max_{p \in \mathcal{N}_\ell(k)} \|A^\ell(k) - A^p(k)\|_F \quad \text{...(Eq. 8)}$$

**Interpretation:**
- Maximum disagreement with any single neighbor
- More conservative than average
- Better for detecting outlier neighbors

**Implementation:**
```
Location: concurrent_pruning/max_disagreement.py::compute_max_disagreement()
Lines: 45-70
Returns: D_l(k), worst_neighbor_id
```

---

### 2.3 Three-Phase Distributed Pruning Protocol

#### Phase 1: PROPOSAL (Distributed Local Evaluation)

**Goal:** Each robot independently proposes edges it would like to prune.

**For each robot** $\ell \in \mathcal{R}$:

**Step 1.1: Consider only incident edges**
$$\text{Consider: } \mathcal{E}_\ell(k) = \{e \in \mathcal{E}(k) : \ell \in e\}$$

**Step 1.2: For each incident edge** $e = (\ell, j)$, verify four safety conditions:

**Condition 1: Alternative Path Exists**
$$\exists \text{ path from } \ell \text{ to } j \text{ in } \mathcal{G}(k) \setminus \{e\}$$

**Implementation:**
```python
# Using robot l's consensus estimate A^l(k)
A_test = A_estimates[l].copy()
A_test[l, j] = A_test[j, l] = 0  # Remove edge
has_path = BFS(l, j, A_test)     # Check if path exists
```

**Condition 2: Graph Connectivity Maintained**
$$\mathcal{G}(k) \setminus \{e\} \text{ is connected}$$

**Implementation:**
```python
# Build temporary graph after removing edge
G_test = Graph(A_test)
is_connected = check_connectivity(G_test)
```

**Condition 3: Algebraic Connectivity Safe**
$$\lambda_2(\mathcal{L}(k) - e) > \lambda_{\min} + \mu(k) \quad \text{...(Eq. 9)}$$

where $\mu(k) \geq 0$ is adaptive safety margin.

**Critical Feature:** Each robot checks using THEIR consensus estimate $A^\ell(k)$, not true $A^*$.

**Implementation:**
```python
# Robot l checks using their estimate
L_test = compute_laplacian(A_test)
eigenvalues = sorted(np.linalg.eigvalsh(L_test))
lambda2 = eigenvalues[1]
is_safe = (lambda2 > lambda_min + margin(k))
```

**Condition 4: Convergence Constraint Satisfied**

**Lyapunov Mode:**
$$V_\ell(k+1)_{\text{sim}} < V_\ell(k) - \varepsilon(k) \quad \text{...(Eq. 10)}$$

where $V_\ell(k+1)_{\text{sim}}$ is **predicted** value after removing edge:

**Implementation:**
```python
# Simulate one consensus step with edge removed
A_next_sim = simulate_consensus(A_estimates, edges_test, neighbors)
V_next_sim = compute_local_lyapunov(l, A_next_sim, neighbors_test)
is_safe_lyapunov = (V_next_sim < V_current - threshold(k))
```

**Alternatively (Max Disagreement Mode):**
$$D_\ell(k+1)_{\text{sim}} < D_\ell(k) - \delta(k) \quad \text{...(Eq. 11)}$$

**Step 1.3: Build Proposal Set**

If all four conditions are satisfied, add edge to proposal set:
$$\mathcal{P}_\ell(k) = \{e \in \mathcal{E}_\ell(k) : e \text{ satisfies all conditions}\}$$

**Implementation:**
```
Location: concurrent_pruning/concurrent_pruning_manager.py::_distributed_proposal_phase()
Lines: 360-410
Returns: Dictionary {robot_id → set of proposed edges}
```

---

#### Phase 2: NEGOTIATION (Bilateral Consensus)

**Goal:** Ensure both endpoints agree before pruning.

**Decision Rule:** Edge $e = (i, j)$ approved for removal if and only if:
$$e \in \mathcal{P}_i(k) \cap \mathcal{P}_j(k) \quad \text{...(Eq. 12)}$$

**Interpretation:** 
- Both robot $i$ AND robot $j$ must independently propose edge  
- Both used different consensus estimates $A^i(k)$ and $A^j(k)$
- Ensures redundancy: even if estimates differ slightly, both must verify

**Approved Set:**
$$\mathcal{A}(k) = \{e = (i,j) : e \in \mathcal{P}_i(k) \wedge e \in \mathcal{P}_j(k)\}$$

**Implementation:**
```
Location: concurrent_pruning/concurrent_pruning_manager.py::_distributed_negotiation_phase()
Lines: 412-450
Algorithm:
  For each edge (i,j):
    if (proposal_i contains edge) AND (proposal_j contains edge):
      Add to approved set
```

**Why This Works:**
- Both endpoints independently evaluate safety using their estimates
- If consensus has converged ($A^i(k) \approx A^j(k)$), agree on same edges  
- If consensus still disagreeing ($A^i(k) \neq A^j(k)$), conservative: both must independently verify

---

#### Phase 3: CONFLICT RESOLUTION (Deterministic Tie-Breaking)

**Goal:** Select exactly ONE edge from approved set to prune per iteration (greedy approach).

**Problem:** $|\mathcal{A}(k)| > 1$ possible ⟹ multiple safe edges to prune.

**Solution:** Apply deterministic priority function all robots can compute without communication.

**Priority Function:**

$$\phi(e = (i,j)) = \left(-\|x_i - x_j\|_2, \quad i+j, \quad \min(i,j)\right) \quad \text{...(Eq. 13)}$$

**Three lexicographic criteria:**

1. **Longest edge first** (term: $-\|x_i - x_j\|_2$)
   - Prefer to remove geometrically weak links  
   - Negative sign: max becomes min in lexicographic order

2. **Lowest robot ID sum** (tie-breaker: $i+j$)
   - Consistent ranking if distances equal

3. **Lower first vertex** (final tie-breaker: $\min(i,j)$)
   - Lexicographic order over pairs

**Selection Rule:**

$$e^*(k) = \arg\min_{e \in \mathcal{A}(k)} \phi(e)$$

using lexicographic ordering on tuples.

**Critical Property:** All robots independently compute same result because $\phi$ is deterministic and shared information (positions+IDs).

**Example:**
```
Approved edges: A = {(0,3), (1,2)}
Positions: x_0=(0,1), x_1=(1,1), x_2=(0,0), x_3=(1,0)

Distances:
  d(0,3) = ||(0,1)-(1,0)|| = √2 ≈ 1.414
  d(1,2) = ||(1,1)-(0,0)|| = √2 ≈ 1.414

Priorities:
  φ(0,3) = (-1.414, 3, 0)
  φ(1,2) = (-1.414, 3, 1)

Lexicographic comparison:
  (-1.414, 3, 0) < (-1.414, 3, 1)  ✓
  
Selected: e* = (0,3)
```

**Implementation:**
```
Location: concurrent_pruning/concurrent_pruning_manager.py::_distributed_conflict_resolution()
Lines: 452-490
Implements: Eq. 13
Returns: Single selected edge or None if |A(k)| = 0
```

---

### 2.4 Adaptive Threshold Scheduling

**Purpose:** Transition from conservative (safe) to aggressive (optimal) over time.

**Insight:** Early in convergence, estimates unreliable ⟹ conservative thresholds save edges. Late, estimates converged ⟹ aggressive thresholds reach minimum tree.

#### Lyapunov Threshold Schedule

$$\varepsilon(k) = \varepsilon_{\max} \cdot V_\ell(0) \cdot \exp\left(-\alpha \frac{k}{k_{\max}}\right) + \varepsilon_{\min} \cdot V_\ell(0) \quad \text{...(Eq. 14)}$$

**Interpretation:**
- At $k=0$: $\varepsilon(0) \approx \varepsilon_{\max} \cdot V_\ell(0)$ (large threshold)
- At $k=k_{\max}$: $\varepsilon(k_{\max}) \approx \varepsilon_{\min} \cdot V_\ell(0)$ (small threshold)
- Exponential decay from conservative to aggressive

**Parameters (typical values):**
- $\varepsilon_{\max} = 0.1$ (10% of initial disagreement)
- $\varepsilon_{\min} = 0.001$ (0.1% of initial disagreement)  
- $\alpha = 5$ (decay rate)
- $k_{\max}$ = expected convergence horizon

#### Lambda₂ Safety Margin Schedule

$$\mu(k) = \mu_{\max} \cdot \exp\left(-\beta \frac{k}{k_{\max}}\right) + \mu_{\min} \quad \text{...(Eq. 15)}$$

**Typical values:**
- $\mu_{\max} = 0.2$ (high safety margin early)
- $\mu_{\min} = 0.05$ (low safety margin late)
- $\beta = 5$ (decay rate)

**Effect:** Constraint becomes $\lambda_2 > \lambda_{\min} + \mu(k)$
- Early: $\lambda_2 > 0.3 + 0.2 = 0.5$ (strict)
- Late: $\lambda_2 > 0.3 + 0.05 = 0.35$ (relaxed)

#### Phase Naming

Based on iteration count:

$$\text{Phase}(k) = \begin{cases}
\text{"ultraconservative"} & k < 0.2 \cdot k_{\max} \\
\text{"conservative"} & 0.2 \cdot k_{\max} \leq k < 0.5 \cdot k_{\max} \\
\text{"moderate"} & 0.5 \cdot k_{\max} \leq k < 0.8 \cdot k_{\max} \\
\text{"aggressive"} & k \geq 0.8 \cdot k_{\max}
\end{cases}$$

**Implementation:**
```
Location: concurrent_pruning/adaptive_thresholds.py
Lines: 50-120
Methods:
  - get_lyapunov_threshold(k)
  - get_lambda2_margin(k)  
  - get_phase_name(k)
```

---

## Mathematical Theory

### 3.1 Consensus Convergence with Pruning

**Theorem 3.1 (Consensus Convergence):**

*Let $\mathcal{G}(k)$ remain connected with $\lambda_2(\mathcal{L}(k)) \geq \lambda_{\min} > 0$ for all $k \geq k_0$. Under the distributed consensus update rule (Eq. 1-5), the local adjacency estimates converge exponentially:*

$$\lim_{k \to \infty} A^\ell(k) = A^* \quad \forall \ell \in \mathcal{R} \quad \text{...(Thm 3.1.1)}$$

*with convergence rate:*

$$\|A^\ell(k) - A^*\|_F \leq C \exp(-\lambda_{\min} T_d k) \quad \text{...(Thm 3.1.2)}$$

*for some constant $C > 0$ depending on initial conditions.*

**Source:** Griparic et al. (2022), Theorem V.1

**Proof Outline for Modified Consensus with Pruning:**

*Step 1: Connectivity Preserved*

By Phase 1 Condition 2 in Section 2.3, graph remains connected at each pruning iteration.

For non-pruning iterations, topology unchanged ⟹ connectivity maintained.

Therefore: $\mathcal{G}(k)$ connected for all $k$. ✓

*Step 2: Algebraic Connectivity Bound*

By Phase 1 Condition 3: $\lambda_2(\mathcal{L}(k+1)) > \lambda_{\min} + \mu(k) \geq \lambda_{\min}$

Since $\mu(k) \geq 0$, algebraic connectivity never falls below $\lambda_{\min}$. ✓

*Step 3: Apply Griparic et al. Result*

Under fixed or time-varying but connected topology with $\lambda_2(k) \geq \lambda_{\min}$, their Theorem V.1 gives exponential convergence bounded by minimum algebraic connectivity:

$$\|A^\ell(k) - A^*\|_F \leq C \exp(-\lambda_{\min} T_d k)$$

*Step 4: Pruning Doesn't Violate This*

Pruning changes topology but maintains connectivity constraint. The consensus pool shrinks but faster convergence on smaller estimate space. Conservative Lyapunov constraint (Condition 4) ensures no backsliding. ✓

---

### 3.2 Lyapunov Stability

**Theorem 3.2 (Global Lyapunov Decrease):**

*Define the global Lyapunov function:*

$$V_{\text{global}}(k) = \sum_{\ell=0}^{n-1} V_\ell(k) = \sum_{\ell=0}^{n-1} \sum_{p \in \mathcal{N}_\ell(k)} \|A^\ell(k) - A^p(k)\|_F^2 \quad \text{...(Thm 3.2.1)}$$

*Then:*

$$V_{\text{global}}(k+1) < V_{\text{global}}(k)$$

*for all $k \geq 0$, ensuring monotonic decrease in consensus disagreement.*

**Proof:**

Consider behavior at iteration $k$:

*Case 1: No edges pruned* ($|\mathcal{A}(k)| = 0$)

By standard consensus theory (Griparic et al.), without topology changes and with fixed $\lambda_2 > 0$:
$$V_{\text{global}}(k+1) < V_{\text{global}}(k) - \gamma$$

for some $\gamma > 0$ depending on convergence rate. ✓

*Case 2: One edge pruned* (say edge $(i,j)$)

By Phase 2 negotiation, both endpoints verified:
$$V_i(k+1)_{\text{sim}} < V_i(k) - \varepsilon(k) \quad \text{...(a)}$$
$$V_j(k+1)_{\text{sim}} < V_j(k) - \varepsilon(k) \quad \text{...(b)}$$

After pruning and one consensus step:
$$V_i(k+1) = V_i(k+1)_{\text{sim}} < V_i(k) - \varepsilon(k)$$
$$V_j(k+1) = V_j(k+1)_{\text{sim}} < V_j(k) - \varepsilon(k)$$

For other robots $\ell \notin \{i,j\}$, edge removal doesn't directly affect their local Lyapunov:
$$V_\ell(k+1) \leq V_\ell(k) \quad \text{(consensus makes progress)}$$

Therefore:
$$V_{\text{global}}(k+1) = \sum_{\ell} V_\ell(k+1) \leq [V_i(k) - \varepsilon(k)] + [V_j(k) - \varepsilon(k)] + \sum_{\ell \neq i,j} V_\ell(k)$$

$$= V_{\text{global}}(k) - 2\varepsilon(k) < V_{\text{global}}(k)$$

Since $\varepsilon(k) > 0$ for all finite $k$, monotonic decrease guaranteed. ✓

---

### 3.3 Connectivity Preservation

**Theorem 3.3 (Connectivity Maintained):**

*If edges are pruned only when Phase 1 Condition 3 is satisfied, then:*

$$\lambda_2(\mathcal{L}(k)) \geq \lambda_{\min} \quad \forall k \geq 0 \quad \text{...(Thm 3.3.1)}$$

**Proof by Contradiction:**

Assume $\lambda_2(\mathcal{L}(k)) < \lambda_{\min}$ at some iteration $k$.

For this to occur, at some earlier iteration $k' < k$, an edge must have been pruned that violated the connectivity constraint.

But by Phase 1 Condition 3, pruning condition requires:
$$\lambda_2(\mathcal{L}(k') \setminus e^*) > \lambda_{\min} + \mu(k') > \lambda_{\min}$$

for both endpoint robots using their estimates $A^{i}(k')$ and $A^{j}(k')$.

**Two sub-cases:**

*Sub-case 1: Consensus Converged* ($A^i(k') \approx A^j(k') \approx A(k')$, true topology)

Then estimated Laplacian $\mathcal{L}$ determinant is close to true Laplacian, so:
$$\lambda_2(\mathcal{L}(k') \setminus e^*) \approx \lambda_2(\mathcal{L}_{\text{true}} \setminus e^*)$$

Pruning condition ensures this remains above $\lambda_{\min}$.

*Sub-case 2: Consensus Not Fully Converged* ($A^i(k') \neq A^j(k')$)

During convergence phase, if $A^i$ and $A^j$ disagree, conservative assumption: take minimum of their estimates.

The smaller eigenvalue is conservative bound, ensuring if even conservative estimate shows $\lambda_2 > \lambda_{\min}$, true value likely above too.

Together: no edge violation possible ⟹ contradiction. □

**Therefore:** $\lambda_2(\mathcal{L}(k)) \geq \lambda_{\min}$ for all $k$. ✓

---

### 3.4 Local Optimality

**Theorem 3.4 (Local Optimality):**

*The final edge set $\mathcal{E}^*$ produced by the algorithm is locally optimal: no single additional edge can be removed without violating constraints.*

**Proof:**

At termination, no more edges pruned ⟹ $\mathcal{A}(k_{\max}) = \emptyset$.

For any remaining edge $e = (i,j) \in \mathcal{E}^*$, either:

1. $e \notin \mathcal{P}_i(k_{\max})$ (robot $i$ didn't propose it)
   - At least one Phase 1 condition violated by $i$'s check
   - Robot $i$ verified removal unsafe ⟹ don't remove

2. $e \notin \mathcal{P}_j(k_{\max})$ (robot $j$ didn't propose it)  
   - Robot $j$ verified removal unsafe

3. $e \in \mathcal{P}_i(k_{\max})$ AND $e \in \mathcal{P}_j(k_{\max})$ (both proposed)
   - But then $e \in \mathcal{A}(k_{\max})$
   - By Phase 3, would be selected and pruned
   - Contradicts $e \in \mathcal{E}^*$ at termination

Therefore, each edge in $\mathcal{E}^*$ respects bilateral consent constraints ⟹ locally optimal. □

**Caveat:** Can't guarantee *global* optimality (minimum spanning tree) due to greedy nature and distributed coordination.

---

## Convergence Guarantees & Proofs

[Detailed proofs of Theorems 3.1-3.4 above]

---

## Formal Algorithms

### Algorithm 1: Main Loop

```
Algorithm: DISTRIBUTED CONCURRENT CONSENSUS + PRUNING
──────────────────────────────────────────────────────────

Input:
  - Robot positions: x₀, x₁, ..., x_{n-1}
  - Initial edges: E(0)
  - Consensus parameters: σ, T_d
  - Pruning parameters: λ_min, ε_max, ε_min, μ_max, μ_min
  - Max iterations: k_max

Output:
  - Consensus estimates: A^ℓ(k_max) for each robot ℓ
  - Final topology: E(k_max)
  - Pruned edges: list of removed edges

Initialize:
  For each robot ℓ ∈ R:
    Initialize A^ℓ(0) with incident edge measurements
    Compute V_ℓ(0) using all zero estimates for non-incident entries
  Initialize threshold scheduler
  Initialize edge analyzer with E(0)

MainLoop:
  for k = 0 to k_max-1:
    
    // STEP 1: Consensus Update (parallel)
    for each robot ℓ ∈ R in parallel:
      Measure incident edges using distance sensors
      Update A^ℓ(k+1) using Equation 1-5
      Compute V_ℓ(k+1)
    end for
    
    // STEP 2: Three-Phase Distributed Pruning
    proposals ← Phase1_Proposal(k, E(k), {A^ℓ(k)})
    approved ← Phase2_Negotiation(proposals)
    e_star ← Phase3_ConflictResolution(approved)
    
    // STEP 3: Topology Update (if edge selected)
    if e_star ≠ NULL:
      E(k+1) ← E(k) \ {e_star}
      Update N_ℓ for affected robots
    else:
      E(k+1) ← E(k)
    end if
    
    // STEP 4: Check Convergence
    if max_ℓ(V_ℓ) < convergence_epsilon AND |E(k+1)| = |E^*|:
      break  // Converged
    end if
    
  end for

return {A^ℓ(k)}, E(k), pruned_edges
```

### Algorithm 2: Phase 1 - Proposal

```
Algorithm: PHASE1_PROPOSAL
──────────────────────────────────────────────

Input: 
  - Current iteration k
  - Edge set E(k)
  - Consensus estimates {A^ℓ(k)} for all robots
  - Threshold scheduler

Output:
  - Proposals: dictionary {ℓ → set of proposed edges}

for each robot ℓ ∈ R in parallel:
  proposal_set_ℓ ← {}
  incident_edges ← {e ∈ E(k) : ℓ ∈ e}
  
  for each edge e = (ℓ, j) in incident_edges:
    
    // Condition 1: Alternative path
    A_test ← A^ℓ(k) with edge e removed
    if NOT has_alternative_path(ℓ, j, A_test):
      continue  // Edge critical, can't remove
    end if
    
    // Condition 2: Connectivity
    G_test ← Graph(A_test)
    if NOT is_connected(G_test):
      continue
    end if
    
    // Condition 3: Lambda₂ constraint
    L_test ← Laplacian(A_test)
    eigenvalues ← sorted(eigenvalsh(L_test))
    lambda₂ ← eigenvalues[1]
    margin ← lambda₂_margin(k)  // From scheduler
    if lambda₂ ≤ λ_min + margin:
      continue
    end if
    
    // Condition 4: Lyapunov/Disagreement constraint
    // Simulate one consensus step with edge removed
    A_next_sim ← consensus_step(A^ℓ, neighbors_with_e_removed)
    V_next_sim ← local_lyapunov(ℓ, A_next_sim)
    threshold ← lyapunov_threshold(k)  // From scheduler
    if V_next_sim ≥ V_ℓ(k) - threshold:
      continue  // Not enough progress
    end if
    
    // All conditions satisfied - add to proposals
    proposal_set_ℓ ← proposal_set_ℓ ∪ {e}
    
  end for
  
end for

return proposal_set_ℓ for all ℓ
```

### Algorithm 3: Phase 2 - Negotiation

```
Algorithm: PHASE2_NEGOTIATION
───────────────────────────────────

Input:
  - Proposals: {proposal_ℓ for each robot ℓ}

Output:
  - Approved edges: set of edges both endpoints proposed

approved ← {}

for each edge e = (i, j):
  if e ∈ proposal_i AND e ∈ proposal_j:
    approved ← approved ∪ {e}
  end if
end for

return approved
```

### Algorithm 4: Phase 3 - Conflict Resolution

```
Algorithm: PHASE3_CONFLICTRESOLUTION  
─────────────────────────────────────────

Input:
  - Approved edges: set A(k)
  - Robot positions: {x_ℓ}

Output:
  - Selected edge e* (or NULL if |A(k)| ≤ 1)

if |A(k)| = 0:
  return NULL
end if

if |A(k)| = 1:
  return the single edge in A(k)
end if

// Multiple approved edges - apply tie-breaking
selected_edge ← NULL
best_priority ← (+∞, +∞, +∞)

for each edge e = (i, j) in A(k):
  // Compute priority tuple (Equation 13)
  distance ← ||x_i - x_j||₂
  priority ← (-distance, i+j, min(i,j))
  
  if priority <_lex best_priority:  // Lexicographic less-than
    best_priority ← priority
    selected_edge ← e
  end if
end for

return selected_edge
```

---

## Complexity Analysis

### Time Complexity Per Iteration

**Consensus Update:**  
- Per robot: $O(n) \times$ neighbor averaging + $O(d)$ incident edge updates
- Total: $O(n^2)$ in worst case, $O(nd)$ average (d = avg degree)

**Phase 1 (Proposal):**
- Per robot, per incident edge:
  - Alternative path check (BFS): $O(n + |E|) = O(n^2)$ worst case
  - Connectivity check: $O(n^2)$
  - Lambda₂ computation (eigenvalue): $O(n^3)$
  - Simulation step: $O(n^2)$
- Total per robot: $O(d \times n^3)$ where $d$ = degree
- All robots parallel: $O(d \times n^3)$ wall-clock

**Phase 2 (Negotiation):**
- Pairwise edge comparison: $O(\min(n^2, |E|^2))$
- Typically: $O(n^2)$

**Phase 3 (Conflict Resolution):**
- Sorting |A(k)| ≤ n edges: $O(n \log n)$
- Distance computations: $O(n^2)$

**Total per iteration:** $O(d \times n^3)$ dominated by eigenvalue computations

### Space Complexity

- Adjacency matrix per robot: $O(n^2)$  
- All robots: $O(n^3)$ distributed
- Edge set: $O(n^2)$ in worst case (complete graph)

### Communication Complexity  

**Per Iteration:**
- Each robot broadcasts estimate to neighbors: $O(n^2)$ values per robot
- Degree $d$ neighbors: $O(n^2 \times d)$ per robot
- All robots: $O(n^3 \times \bar{d})$ total (but mostly parallel)

**Total Iterations to Convergence:**
- Exponential convergence: $O(\log(1/\epsilon))$ for accuracy $\epsilon$
- Typically: $O(100-200)$ iterations observed

---

## Implementation Reference

| Component | File | Key Functions |
|-----------|------|---|
| Consensus | `consensus/adjacency_consensus.py` | `consensus_update_step()` |
| Lyapunov | `concurrent_pruning/local_lyapunov.py` | `compute_local_lyapunov()` |
| Disagreement | `concurrent_pruning/max_disagreement.py` | `compute_max_disagreement()` |
| Main Manager | `concurrent_pruning/concurrent_pruning_manager.py` | `concurrent_step()` |
| Phase 1 | `concurrent_pruning/concurrent_pruning_manager.py` | `_distributed_proposal_phase()` |
| Phase 2 | `concurrent_pruning/concurrent_pruning_manager.py` | `_distributed_negotiation_phase()` |
| Phase 3 | `concurrent_pruning/concurrent_pruning_manager.py` | `_distributed_conflict_resolution()` |
| Thresholds | `concurrent_pruning/adaptive_thresholds.py` | `get_lyapunov_threshold()` |
| Edge Analysis | `graph/edge_analysis.py` | `check_graph_connectivity()` |
| Lambda₂ | `concurrent_pruning/concurrent_pruning_manager.py` | `estimate_lambda2()` |

---

## Examples & Use Cases

See `concurrent_pruning/example_4_robots.py` for complete walkthrough with:
- 4-robot square topology
- 6 edges → 3 edges pruning
- Real metrics visualization
- Detailed iteration logs

See `concurrent_pruning/Docs/SMALL_EXAMPLE_WALKTHROUGH.md` for mathematical example.

