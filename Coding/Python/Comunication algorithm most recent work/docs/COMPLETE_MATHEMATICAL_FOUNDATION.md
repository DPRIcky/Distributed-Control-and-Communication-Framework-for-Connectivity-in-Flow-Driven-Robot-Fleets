# Complete Mathematical Foundation for Distributed Consensus-Based Multi-Robot Control

**Document Purpose:** Step-by-step mathematical derivations for entire codebase  
**Date:** January 23, 2026

---

## Table of Contents

1. [Graph Theory Fundamentals](#1-graph-theory-fundamentals)
2. [Algebraic Connectivity (λ₂)](#2-algebraic-connectivity-λ₂)
3. [Adjacency Matrix Consensus](#3-adjacency-matrix-consensus)
4. [Distributed Edge Detection](#4-distributed-edge-detection)
5. [Control Barrier Functions (CBF)](#5-control-barrier-functions-cbf)
6. [Control Lyapunov Functions (CLF)](#6-control-lyapunov-functions-clf)
7. [Hybrid CLF-CBF Control](#7-hybrid-clf-cbf-control)
8. [Robot Dynamics and Flow Fields](#8-robot-dynamics-and-flow-fields)
9. [Complete System Integration](#9-complete-system-integration)
10. [Convergence Guarantees](#10-convergence-guarantees)

---

# 1. GRAPH THEORY FUNDAMENTALS

## 1.1 Basic Definitions

### **Graph Representation**

A communication network of $N$ robots is represented by an undirected graph:

$$\mathcal{G} = (\mathcal{V}, \mathcal{E})$$

where:
- $\mathcal{V} = \{1, 2, \ldots, N\}$ is the vertex set (robots)
- $\mathcal{E} \subseteq \mathcal{V} \times \mathcal{V}$ is the edge set (communication links)

### **Adjacency Matrix**

The adjacency matrix $\mathbf{A} \in \mathbb{R}^{N \times N}$ encodes edge connections:

$$\mathbf{A}_{ij} = \begin{cases}
1 & \text{if } (i,j) \in \mathcal{E} \\
0 & \text{otherwise}
\end{cases}$$

**Properties:**
1. **Symmetric:** $\mathbf{A}_{ij} = \mathbf{A}_{ji}$ (undirected graph)
2. **Zero diagonal:** $\mathbf{A}_{ii} = 0$ (no self-loops)
3. **Binary entries:** $\mathbf{A}_{ij} \in \{0, 1\}$

**Example (4 robots in a line):**
```
Topology: 0 — 1 — 2 — 3

A = [0  1  0  0]
    [1  0  1  0]
    [0  1  0  1]
    [0  0  1  0]
```

### **Degree Matrix**

The degree matrix $\mathbf{D} \in \mathbb{R}^{N \times N}$ is diagonal:

$$\mathbf{D}_{ii} = \sum_{j=1}^{N} \mathbf{A}_{ij} = \deg(i)$$

$$\mathbf{D}_{ij} = 0, \quad i \neq j$$

where $\deg(i)$ is the number of neighbors of robot $i$.

**Example (same 4-robot line):**
```
D = [1  0  0  0]    Robot 0: 1 neighbor
    [0  2  0  0]    Robot 1: 2 neighbors
    [0  0  2  0]    Robot 2: 2 neighbors
    [0  0  0  1]    Robot 3: 1 neighbor
```

### **Graph Laplacian**

The graph Laplacian $\mathbf{L} \in \mathbb{R}^{N \times N}$ is defined as:

$$\mathbf{L} = \mathbf{D} - \mathbf{A}$$

**Explicit form:**

$$\mathbf{L}_{ij} = \begin{cases}
\deg(i) & \text{if } i = j \\
-1 & \text{if } (i,j) \in \mathcal{E} \\
0 & \text{otherwise}
\end{cases}$$

**Example:**
```
L = D - A = [1  -1   0   0]
            [-1  2  -1   0]
            [0  -1   2  -1]
            [0   0  -1   1]
```

**Properties of Laplacian:**

1. **Symmetric:** $\mathbf{L}^\top = \mathbf{L}$

2. **Positive Semi-Definite:** $\mathbf{L} \succeq 0$
   - All eigenvalues $\lambda_i \geq 0$

3. **Row sums to zero:** 
   $$\sum_{j=1}^{N} \mathbf{L}_{ij} = 0, \quad \forall i$$
   
   Equivalently: $\mathbf{L} \mathbf{1} = \mathbf{0}$ where $\mathbf{1} = [1, 1, \ldots, 1]^\top$

4. **Eigenvalue structure:** 
   $$0 = \lambda_1 \leq \lambda_2 \leq \cdots \leq \lambda_N$$
   
   where $\lambda_1 = 0$ always with eigenvector $\mathbf{v}_1 = \frac{1}{\sqrt{N}}\mathbf{1}$

## 1.2 Connectivity and λ₂

### **Graph Connectivity**

A graph is **connected** if there exists a path between every pair of vertices.

**Mathematical Definition:**

$$\text{Graph } \mathcal{G} \text{ is connected} \iff \forall i, j \in \mathcal{V}, \exists \text{ path } i \leadsto j$$

### **Fiedler's Theorem**

**Theorem 1.1 (Fiedler, 1973):** 

For a graph Laplacian $\mathbf{L}$ with eigenvalues $0 = \lambda_1 \leq \lambda_2 \leq \cdots \leq \lambda_N$:

$$\text{Graph } \mathcal{G} \text{ is connected} \iff \lambda_2 > 0$$

**Proof Sketch:**

**Forward direction (⇒):** 
If $\mathcal{G}$ is connected, suppose $\lambda_2 = 0$. Then there exist two eigenvectors $\mathbf{v}_1, \mathbf{v}_2$ with eigenvalue 0. By properties of Laplacian, this implies the graph has at least 2 connected components (contradiction).

**Backward direction (⇐):**
If $\lambda_2 > 0$, then the null space of $\mathbf{L}$ is one-dimensional (span of $\mathbf{1}$), implying a single connected component.

### **Algebraic Connectivity**

The second smallest eigenvalue $\lambda_2$ is called **algebraic connectivity**:

$$\lambda_2(\mathcal{L}) = \text{algebraic connectivity}$$

**Physical Interpretation:**
- $\lambda_2 = 0$: Graph disconnected (at least 2 components)
- $\lambda_2$ small (>0): Weak connectivity (easy to disconnect)
- $\lambda_2$ large: Strong connectivity (robust to edge removal)

**Example:**
```
Complete graph K₄:        Line graph (4 nodes):
All pairs connected       0—1—2—3

λ₂ = 4 (very robust)      λ₂ ≈ 0.38 (fragile)
```

### **Rayleigh Quotient Characterization**

The Fiedler vector $\mathbf{v}_2$ (eigenvector for $\lambda_2$) satisfies:

$$\lambda_2 = \min_{\mathbf{x} \perp \mathbf{1}} \frac{\mathbf{x}^\top \mathbf{L} \mathbf{x}}{\mathbf{x}^\top \mathbf{x}}$$

Expanding the numerator:

$$\mathbf{x}^\top \mathbf{L} \mathbf{x} = \sum_{(i,j) \in \mathcal{E}} (x_i - x_j)^2$$

**Interpretation:** $\lambda_2$ measures how much you can "pull apart" nodes while maintaining graph structure.

---

# 2. ALGEBRAIC CONNECTIVITY (λ₂)

## 2.1 Direct Eigenvalue Computation

### **Eigenvalue Problem**

Given Laplacian $\mathbf{L}$, solve:

$$\mathbf{L} \mathbf{v} = \lambda \mathbf{v}$$

**Characteristic Equation:**

$$\det(\mathbf{L} - \lambda \mathbf{I}) = 0$$

**Standard Algorithm:**

1. Form $\mathbf{L} = \mathbf{D} - \mathbf{A}$
2. Compute eigenvalues: $\text{eig}(\mathbf{L}) = [\lambda_1, \lambda_2, \ldots, \lambda_N]$
3. Sort: $\lambda_1 \leq \lambda_2 \leq \cdots \leq \lambda_N$
4. Extract: $\lambda_2$ is second element

**Complexity:** $O(N^3)$ using QR algorithm

**Code mapping:** `lambda2_estimator.py` → `_compute_exact_lambda2()`

## 2.2 Cheeger's Inequality (Fast Lower Bound)

### **Cheeger Constant**

For a graph $\mathcal{G} = (\mathcal{V}, \mathcal{E})$, the **Cheeger constant** (isoperimetric number) is:

$$h(\mathcal{G}) = \min_{S \subset \mathcal{V}} \frac{|\partial S|}{\min(|S|, |\bar{S}|)}$$

where:
- $S \subset \mathcal{V}$ is a vertex subset
- $\bar{S} = \mathcal{V} \setminus S$ is the complement
- $|\partial S| = \{(i,j) \in \mathcal{E} : i \in S, j \in \bar{S}\}$ is the edge cut

**Physical meaning:** Minimum "bottleneck" when splitting graph into two parts.

### **Cheeger's Inequality**

**Theorem 2.1 (Cheeger, 1970):**

$$\frac{h(\mathcal{G})^2}{2} \leq \lambda_2 \leq 2h(\mathcal{G})$$

**Lower Bound (Conservative Estimate):**

$$\lambda_2 \geq \frac{h(\mathcal{G})^2}{2}$$

**Algorithm:**

```python
def cheeger_lower_bound(A):
    """
    Compute h(G)² / 2
    """
    N = A.shape[0]
    h_min = float('inf')
    
    # Try all possible cuts
    for subset_bits in range(1, 2^N - 1):
        S = {i for i in range(N) if (subset_bits >> i) & 1}
        S_bar = set(range(N)) - S
        
        # Count edge cut
        edge_cut = sum(A[i, j] for i in S for j in S_bar)
        
        # Cheeger ratio
        h = edge_cut / min(len(S), len(S_bar))
        h_min = min(h_min, h)
    
    return h_min**2 / 2
```

**Complexity:** $O(2^N \cdot N^2)$ (exponential, but can use heuristics)

**Code mapping:** `lambda2_estimator.py` → `_compute_cheeger_bound()`

## 2.3 Incremental Sensitivity (Fast Update)

### **Edge Perturbation**

When edge $(i,j)$ is added/removed, the Laplacian changes:

$$\mathbf{L}_{\text{new}} = \mathbf{L}_{\text{old}} + \Delta \mathbf{L}_{ij}$$

where:

$$\Delta \mathbf{L}_{ij} = \mathbf{e}_{ij} \mathbf{e}_{ij}^\top$$

with $\mathbf{e}_{ij} = \mathbf{e}_i - \mathbf{e}_j$ (difference of standard basis vectors).

### **First-Order Sensitivity**

The change in $\lambda_2$ is approximately:

$$\Delta \lambda_2 \approx \frac{\partial \lambda_2}{\partial l_{ij}}$$

**Theorem 2.2 (Eigenvalue Perturbation):**

For Laplacian eigenvalue $\lambda_k$ with eigenvector $\mathbf{v}_k$:

$$\frac{\partial \lambda_k}{\partial l_{ij}} = \mathbf{v}_k^\top \frac{\partial \mathbf{L}}{\partial l_{ij}} \mathbf{v}_k$$

For the Fiedler value:

$$\frac{\partial \lambda_2}{\partial l_{ij}} = (\mathbf{v}_{2,i} - \mathbf{v}_{2,j})^2$$

where $\mathbf{v}_2 = [\mathbf{v}_{2,1}, \mathbf{v}_{2,2}, \ldots, \mathbf{v}_{2,N}]^\top$ is the Fiedler vector.

**Algorithm:**

```python
def incremental_lambda2(L_old, lambda2_old, v2_old, edge_removed):
    """
    Fast update when removing edge
    """
    i, j = edge_removed
    
    # Sensitivity
    sensitivity = (v2_old[i] - v2_old[j])**2
    
    # Approximate new value
    lambda2_new = lambda2_old - sensitivity
    
    return lambda2_new
```

**Complexity:** $O(1)$ if Fiedler vector is cached

**Code mapping:** `lambda2_estimator.py` → `_compute_incremental_lambda2()`

## 2.4 Adaptive Mode Selection

### **Switching Logic**

Choose computation method based on graph state:

$$\text{Mode} = \begin{cases}
\text{Cheeger} & \text{if initial estimate or large change} \\
\text{Incremental} & \text{if } |\Delta \mathcal{E}| \leq 5 \text{ (small edge change)} \\
\text{Exact} & \text{if accuracy critical or verification needed}
\end{cases}$$

**Cost-Accuracy Tradeoff:**

| Method | Complexity | Accuracy | Use Case |
|--------|-----------|----------|----------|
| Cheeger | $O(N^2)$ | Lower bound (conservative) | Quick safety check |
| Incremental | $O(1)$ | Approximate (~5% error) | Pruning decisions |
| Exact | $O(N^3)$ | Exact | Verification, final check |

**Code mapping:** `lambda2_estimator.py` → `get_lambda2()` with `mode='auto'`

---

# 3. ADJACENCY MATRIX CONSENSUS

## 3.1 Consensus Problem Setup

### **Local Observations**

Each robot $l$ has:
- **Direct observation:** $\varepsilon^l_{ij}$ of edge $(i,j)$ if $i, j \in \mathcal{N}_l \cup \{l\}$
- **Estimate:** $\mathbf{A}^l(k) \in \mathbb{R}^{N \times N}$ at iteration $k$

**Goal:** All robots converge to true adjacency matrix:

$$\lim_{k \to \infty} \mathbf{A}^l(k) = \mathbf{A}^*, \quad \forall l$$

where $\mathbf{A}^*$ is the ground truth adjacency.

### **Why Consensus is Needed**

- **Problem:** Robot $l$ can only see edges involving its neighbors
- **Solution:** Share estimates with neighbors to reconstruct full graph
- **Benefit:** Distributed knowledge without centralized coordinator

## 3.2 Consensus Update Equation

### **Discrete-Time Consensus (Griparic et al. 2022)**

**Update Rule:**

$$\mathbf{A}^l(k+1) = \mathbf{A}^l(k) + T_d \cdot \Delta \mathbf{A}^l(k)$$

where:

$$\Delta \mathbf{A}^l(k) = \sum_{p \in \mathcal{N}_l} w_{lp}(k) \left[ \mathbf{A}^p(k) - \mathbf{A}^l(k) \right] + \varepsilon^l(k)$$

**Terms:**
- $T_d > 0$: Sample time (consensus step size), typically $T_d = 0.1$
- $\mathcal{N}_l$: Set of neighbors of robot $l$
- $w_{lp}(k)$: Trust weight between robots $l$ and $p$
- $\varepsilon^l(k)$: Direct observation error/innovation

### **Detailed Breakdown**

**Step 1: Neighbor averaging**

$$\text{Avg}_l(k) = \sum_{p \in \mathcal{N}_l} w_{lp}(k) \mathbf{A}^p(k)$$

Robot $l$ takes weighted average of neighbors' estimates.

**Step 2: Correction toward neighbors**

$$\text{Correction}_l(k) = \sum_{p \in \mathcal{N}_l} w_{lp}(k) \left[ \mathbf{A}^p(k) - \mathbf{A}^l(k) \right]$$

This is the "pull" toward consensus.

**Step 3: Add direct observation**

$$\Delta \mathbf{A}^l(k) = \text{Correction}_l(k) + \varepsilon^l(k)$$

Include new information from sensors.

**Step 4: Update estimate**

$$\mathbf{A}^l(k+1) = \mathbf{A}^l(k) + T_d \cdot \Delta \mathbf{A}^l(k)$$

### **Trust Function**

**Distance-Based Trust:**

$$t_{lp}(k) = \exp\left( -\frac{d_{lp}(k)^2}{\sigma^2} \right)$$

where:
- $d_{lp}(k) = \|\mathbf{p}_l(k) - \mathbf{p}_p(k)\|$ is distance between robots
- $\sigma > 0$ is trust bandwidth (typically $\sigma = 1.0$)

**Normalized Weights:**

$$w_{lp}(k) = \frac{t_{lp}(k)}{\sum_{q \in \mathcal{N}_l} t_{lq}(k)}$$

**Properties:**
- $\sum_{p \in \mathcal{N}_l} w_{lp}(k) = 1$ (convex combination)
- $w_{lp} \to 1$ as $d_{lp} \to 0$ (close robots trusted more)
- $w_{lp} \to 0$ as $d_{lp} \to \infty$ (far robots ignored)

**Code mapping:** `adjacency_consensus.py` → `_compute_trust_weights()`

## 3.3 Convergence Guarantee

### **Theorem 3.1 (Consensus Convergence - Griparic et al. 2022)**

Given:
1. Communication graph $\mathcal{G}_{\text{comm}}$ is connected
2. Trust weights $w_{lp}$ are symmetric: $w_{lp} = w_{pl}$
3. Step size $T_d$ is sufficiently small

Then:

$$\left\| \mathbf{A}^l(k) - \mathbf{A}^* \right\| \leq C \cdot \exp\left( -\lambda_2(\mathcal{L}_{\text{comm}}) \cdot T_d \cdot k \right)$$

where:
- $C$ depends on initial conditions
- $\lambda_2(\mathcal{L}_{\text{comm}})$ is algebraic connectivity of **communication graph**
- Convergence is **exponential** with rate $\lambda_2 \cdot T_d$

### **Proof Sketch**

**Step 1: Define consensus error**

$$\mathbf{E}^l(k) = \mathbf{A}^l(k) - \mathbf{A}^*$$

**Step 2: Error dynamics**

Subtracting $\mathbf{A}^*$ from both sides of update:

$$\mathbf{E}^l(k+1) = \mathbf{E}^l(k) + T_d \sum_{p \in \mathcal{N}_l} w_{lp} \left[ \mathbf{E}^p(k) - \mathbf{E}^l(k) \right]$$

(assuming $\varepsilon^l \to 0$ after initial observation)

**Step 3: Stack all errors**

$$\mathbf{e}(k) = \begin{bmatrix} \text{vec}(\mathbf{E}^1(k)) \\ \text{vec}(\mathbf{E}^2(k)) \\ \vdots \\ \text{vec}(\mathbf{E}^N(k)) \end{bmatrix}$$

**Step 4: Global dynamics**

$$\mathbf{e}(k+1) = (\mathbf{I} - T_d \mathbf{L}_{\text{comm}} \otimes \mathbf{I}) \mathbf{e}(k)$$

where $\otimes$ is Kronecker product.

**Step 5: Eigenvalue analysis**

Eigenvalues of $\mathbf{I} - T_d \mathbf{L}_{\text{comm}}$ are:

$$\rho_i = 1 - T_d \lambda_i$$

The largest eigenvalue (excluding $\lambda_1 = 0$) is:

$$\rho_{\max} = 1 - T_d \lambda_2$$

**Step 6: Convergence rate**

$$\|\mathbf{e}(k)\| \leq \rho_{\max}^k \|\mathbf{e}(0)\| = (1 - T_d \lambda_2)^k \|\mathbf{e}(0)\|$$

Using $(1-x)^k \approx e^{-kx}$ for small $x$:

$$\|\mathbf{e}(k)\| \leq C e^{-\lambda_2 T_d k}$$

**Code mapping:** Implemented in `adjacency_consensus.py` → `run_consensus_iterations()`

## 3.4 Practical Implementation

### **Initialization**

```python
# Each robot l initializes with direct observations
A_estimates[l] = np.zeros((N, N))

for i in neighbors_of[l]:
    for j in neighbors_of[l]:
        if distance(i, j) <= R_max:
            A_estimates[l][i, j] = 1.0
```

### **Iteration Loop**

```python
for iteration in range(max_iterations):
    for robot_id in range(N):
        # Get neighbors
        neighbors = get_neighbors(robot_id)
        
        # Compute trust weights
        weights = {}
        total_trust = 0
        for neighbor_id in neighbors:
            dist = distance(robot_id, neighbor_id)
            trust = exp(-(dist**2) / (sigma**2))
            weights[neighbor_id] = trust
            total_trust += trust
        
        # Normalize
        for neighbor_id in neighbors:
            weights[neighbor_id] /= total_trust
        
        # Consensus update
        correction = np.zeros((N, N))
        for neighbor_id in neighbors:
            correction += weights[neighbor_id] * (
                A_estimates[neighbor_id] - A_estimates[robot_id]
            )
        
        # Add direct observations
        direct_obs = observe_edges(robot_id)
        
        # Update
        A_estimates[robot_id] += T_d * (correction + direct_obs)
    
    # Check convergence
    if max_disagreement(A_estimates) < epsilon:
        break
```

**Code mapping:** `adjacency_consensus.py` → `consensus_update_step()`

---

# 4. DISTRIBUTED EDGE DETECTION

## 4.1 Edge Extraction from Consensus

### **Threshold-Based Detection**

After consensus converges, each robot $l$ has $\mathbf{A}^l \approx \mathbf{A}^*$.

**Extract edges:**

$$\mathcal{E}^l_{\text{est}} = \left\{ (i,j) : \mathbf{A}^l_{ij} > \theta_{\text{edge}}, \, i < j \right\}$$

where $\theta_{\text{edge}} = 0.1$ is a threshold.

**Why threshold?**
- Consensus may produce values slightly different from 0 or 1
- Threshold filters noise: $\mathbf{A}^l_{ij} = 0.98 \approx 1$ (edge exists)
- Values below threshold ignored: $\mathbf{A}^l_{ij} = 0.02 \approx 0$ (no edge)

**Code mapping:** `edge_analysis.py` → `extract_edges_from_consensus()`

## 4.2 Redundancy Check via BFS

### **Alternative Path Existence**

An edge $(i,j)$ is **redundant** if removing it doesn't disconnect $i$ from $j$.

**Mathematical condition:**

$$(i,j) \text{ redundant} \iff \exists \text{ path } i \leadsto j \text{ in } (\mathcal{V}, \mathcal{E} \setminus \{(i,j)\})$$

### **Breadth-First Search (BFS)**

**Algorithm:**

```
Input: Graph G, edge (i,j)
Output: True if alternative path exists

1. Remove edge (i,j) from G
2. BFS from node i
3. If node j is reached:
     return True  (alternative path exists)
   else:
     return False (edge is critical)
```

**Pseudocode:**

```python
def has_alternative_path(A, edge):
    i, j = edge
    N = A.shape[0]
    
    # Create adjacency without edge (i,j)
    A_test = A.copy()
    A_test[i, j] = 0
    A_test[j, i] = 0
    
    # BFS from i
    visited = {i}
    queue = [i]
    
    while queue:
        current = queue.pop(0)
        
        # Check if reached j
        if current == j:
            return True
        
        # Explore neighbors
        for neighbor in range(N):
            if A_test[current, neighbor] > 0.5 and neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)
    
    # If j not reached, no alternative path
    return False
```

**Complexity:** $O(N + m)$ where $m = |\mathcal{E}|$

**Code mapping:** `edge_analysis.py` → `has_alternative_path_from_consensus()`

## 4.3 Global Connectivity Check

### **Connected Graph Test**

After removing edge $(i,j)$, check if graph remains connected:

$$\mathcal{G} \setminus \{(i,j)\} \text{ connected?}$$

**BFS-Based Algorithm:**

```python
def is_connected(A):
    """Check if graph is connected using BFS"""
    N = A.shape[0]
    
    # BFS from node 0
    visited = {0}
    queue = [0]
    
    while queue:
        current = queue.pop(0)
        for neighbor in range(N):
            if A[current, neighbor] > 0.5 and neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)
    
    # Check if all nodes reached
    return len(visited) == N
```

**Alternative (Eigenvalue-Based):**

$$\text{Graph connected} \iff \lambda_2(\mathbf{L}) > 0$$

More expensive ($O(N^3)$) but exact.

**Code mapping:** `edge_analysis.py` → `check_connectivity_from_consensus()`

## 4.4 Redundancy Classification

### **Edge Categories**

For edge $(i,j)$:

1. **Critical:** No alternative path exists
   - Removing it disconnects graph
   - Must keep for connectivity

2. **Redundant:** Alternative path exists AND graph stays connected
   - Can be removed safely
   - Candidate for pruning

3. **Uncertain:** Borderline cases (near threshold)
   - Requires higher consensus accuracy

**Decision Logic:**

```python
def classify_edge(A, edge, lambda2_threshold=0.1):
    i, j = edge
    
    # Test 1: Alternative path?
    has_alt = has_alternative_path(A, edge)
    if not has_alt:
        return "CRITICAL"
    
    # Test 2: Graph stays connected?
    A_test = remove_edge(A, edge)
    if not is_connected(A_test):
        return "CRITICAL"
    
    # Test 3: Algebraic connectivity acceptable?
    lambda2_new = compute_lambda2(A_test)
    if lambda2_new < lambda2_threshold:
        return "CRITICAL"
    
    return "REDUNDANT"
```

**Code mapping:** `hybrid_pruning.py` → `find_redundant_edge_distributed()`

---

# 5. CONTROL BARRIER FUNCTIONS (CBF)

## 5.1 Safe Set and Barrier Functions

### **Safe Set Definition**

A **safe set** $\mathcal{C} \subseteq \mathbb{R}^n$ is defined implicitly by a barrier function $h: \mathbb{R}^n \to \mathbb{R}$:

$$\mathcal{C} = \{ \mathbf{x} \in \mathbb{R}^n : h(\mathbf{x}) \geq 0 \}$$

**Examples:**

1. **Collision avoidance:** Keep robots apart
   $$h_{ij}^{\text{safe}}(\mathbf{x}) = \|\mathbf{x}_i - \mathbf{x}_j\|^2 - d_{\min}^2$$
   Safe set: $\|\mathbf{x}_i - \mathbf{x}_j\| \geq d_{\min}$

2. **Connectivity maintenance:** Keep robots in communication range
   $$h_{ij}^{\text{conn}}(\mathbf{x}) = R_{\max}^2 - \|\mathbf{x}_i - \mathbf{x}_j\|^2$$
   Safe set: $\|\mathbf{x}_i - \mathbf{x}_j\| \leq R_{\max}$

### **Forward Invariance**

A set $\mathcal{C}$ is **forward invariant** under dynamics $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$ if:

$$\mathbf{x}(0) \in \mathcal{C} \implies \mathbf{x}(t) \in \mathcal{C}, \quad \forall t \geq 0$$

**Goal:** Design control $\mathbf{u}$ to ensure forward invariance.

## 5.2 CBF Condition

### **Definition (Ames et al. 2019)**

A continuously differentiable function $h: \mathbb{R}^n \to \mathbb{R}$ is a **Control Barrier Function (CBF)** for system $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$ if there exists $\alpha > 0$ such that:

$$\sup_{\mathbf{u} \in \mathcal{U}} \left[ \mathcal{L}_f h(\mathbf{x}) + \mathcal{L}_g h(\mathbf{x}) \mathbf{u} + \alpha h(\mathbf{x}) \right] \geq 0$$

for all $\mathbf{x}$ such that $h(\mathbf{x}) \geq 0$.

**Lie Derivatives:**
- $\mathcal{L}_f h(\mathbf{x}) = \nabla h(\mathbf{x})^\top f(\mathbf{x})$ (drift term)
- $\mathcal{L}_g h(\mathbf{x}) = \nabla h(\mathbf{x})^\top g(\mathbf{x})$ (control term)

For control-affine system $\dot{\mathbf{x}} = f(\mathbf{x}) + g(\mathbf{x})\mathbf{u}$.

### **Single-Integrator Simplification**

For our robot dynamics $\dot{\mathbf{x}}_i = \mathbf{u}_i$:
- $f(\mathbf{x}) = \mathbf{0}$ (no drift)
- $g(\mathbf{x}) = \mathbf{I}$ (direct control)

**CBF Condition becomes:**

$$\dot{h}(\mathbf{x}, \mathbf{u}) + \alpha h(\mathbf{x}) \geq 0$$

where:

$$\dot{h}(\mathbf{x}, \mathbf{u}) = \nabla h(\mathbf{x})^\top \dot{\mathbf{x}} = \nabla h(\mathbf{x})^\top \mathbf{u}$$

## 5.3 Safety Barrier (Collision Avoidance)

### **Barrier Function**

For robots $i$ and $j$:

$$h_{ij}^{\text{safe}}(\mathbf{x}) = \|\mathbf{x}_i - \mathbf{x}_j\|^2 - d_{\min}^2$$

**Safe set:** $\mathcal{C}_{ij}^{\text{safe}} = \{ \mathbf{x} : h_{ij}^{\text{safe}}(\mathbf{x}) \geq 0 \}$

Equivalently: $\|\mathbf{x}_i - \mathbf{x}_j\| \geq d_{\min}$

### **Time Derivative**

$$\dot{h}_{ij}^{\text{safe}} = \frac{d}{dt} \|\mathbf{x}_i - \mathbf{x}_j\|^2 = 2(\mathbf{x}_i - \mathbf{x}_j)^\top (\dot{\mathbf{x}}_i - \dot{\mathbf{x}}_j)$$

For single-integrator: $\dot{\mathbf{x}}_i = \mathbf{u}_i$, $\dot{\mathbf{x}}_j = \mathbf{u}_j$

$$\dot{h}_{ij}^{\text{safe}} = 2(\mathbf{x}_i - \mathbf{x}_j)^\top (\mathbf{u}_i - \mathbf{u}_j)$$

### **CBF Constraint**

$$\dot{h}_{ij}^{\text{safe}} + \beta_s h_{ij}^{\text{safe}} \geq 0$$

Substituting:

$$2(\mathbf{x}_i - \mathbf{x}_j)^\top (\mathbf{u}_i - \mathbf{u}_j) + \beta_s \left( \|\mathbf{x}_i - \mathbf{x}_j\|^2 - d_{\min}^2 \right) \geq 0$$

**Linear constraint in** $\mathbf{u}_i$ (if $\mathbf{u}_j$ is fixed or another robot's control).

**Code mapping:** `cbf_controller.py` → Safety barrier setup

## 5.4 Connectivity Barrier

### **Barrier Function**

For robots $i$ and $j$ that must stay connected:

$$h_{ij}^{\text{conn}}(\mathbf{x}) = R_{\max}^2 - \|\mathbf{x}_i - \mathbf{x}_j\|^2$$

**Safe set:** $\mathcal{C}_{ij}^{\text{conn}} = \{ \mathbf{x} : h_{ij}^{\text{conn}}(\mathbf{x}) \geq 0 \}$

Equivalently: $\|\mathbf{x}_i - \mathbf{x}_j\| \leq R_{\max}$

### **Time Derivative**

$$\dot{h}_{ij}^{\text{conn}} = \frac{d}{dt} \left( R_{\max}^2 - \|\mathbf{x}_i - \mathbf{x}_j\|^2 \right) = -2(\mathbf{x}_i - \mathbf{x}_j)^\top (\mathbf{u}_i - \mathbf{u}_j)$$

### **CBF Constraint**

$$\dot{h}_{ij}^{\text{conn}} + \beta_c h_{ij}^{\text{conn}} \geq 0$$

$$-2(\mathbf{x}_i - \mathbf{x}_j)^\top (\mathbf{u}_i - \mathbf{u}_j) + \beta_c \left( R_{\max}^2 - \|\mathbf{x}_i - \mathbf{x}_j\|^2 \right) \geq 0$$

**Code mapping:** `cbf_controller.py` → Connectivity barrier setup

## 5.5 Multi-Robot CBF Constraints

### **Robot $i$'s Perspective**

Robot $i$ must satisfy:

1. **Safety with all sensing neighbors:**
   $$\dot{h}_{ij}^{\text{safe}} + \beta_s h_{ij}^{\text{safe}} \geq 0, \quad \forall j \in \mathcal{N}(i)$$

2. **Connectivity with critical neighbors:**
   $$\dot{h}_{ij}^{\text{conn}} + \beta_c h_{ij}^{\text{conn}} \geq 0, \quad \forall j \in \mathcal{C}_i$$

where $\mathcal{C}_i$ is the set of critical neighbors (from pruned graph).

### **Stacked Constraints**

Define constraint vector:

$$\mathbf{A}_{\text{CBF}}^i \mathbf{u}_i \geq \mathbf{b}_{\text{CBF}}^i$$

where each row corresponds to one barrier constraint.

---

# 6. CONTROL LYAPUNOV FUNCTIONS (CLF)

## 6.1 Lyapunov Stability Review

### **Lyapunov Function**

A continuously differentiable function $V: \mathbb{R}^n \to \mathbb{R}$ is a **Lyapunov function** for system $\dot{\mathbf{x}} = f(\mathbf{x})$ if:

1. **Positive definite:** $V(\mathbf{x}) > 0$ for $\mathbf{x} \neq \mathbf{0}$, $V(\mathbf{0}) = 0$
2. **Radially unbounded:** $V(\mathbf{x}) \to \infty$ as $\|\mathbf{x}\| \to \infty$
3. **Decreasing along trajectories:** $\dot{V}(\mathbf{x}) \leq 0$

### **Lyapunov's Theorem**

If such $V$ exists, the equilibrium $\mathbf{x} = \mathbf{0}$ is **stable**.

If additionally $\dot{V}(\mathbf{x}) < 0$ for $\mathbf{x} \neq \mathbf{0}$, it is **asymptotically stable**.

## 6.2 Control Lyapunov Function (CLF)

### **Definition**

A function $V: \mathbb{R}^n \to \mathbb{R}$ is a **Control Lyapunov Function (CLF)** for system $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$ if there exists $\alpha > 0$ such that:

$$\inf_{\mathbf{u} \in \mathcal{U}} \left[ \mathcal{L}_f V(\mathbf{x}) + \mathcal{L}_g V(\mathbf{x}) \mathbf{u} + \alpha V(\mathbf{x}) \right] \leq 0$$

**Interpretation:** For any state $\mathbf{x}$, there exists a control $\mathbf{u}$ that makes $V$ decrease.

### **CLF Condition**

$$\dot{V}(\mathbf{x}, \mathbf{u}) + \alpha V(\mathbf{x}) \leq 0$$

This ensures exponential convergence:

$$V(t) \leq V(0) e^{-\alpha t} \to 0$$

## 6.3 Target Tracking CLF

### **Tracking Error**

Define error between robot $i$ and its target $\mathbf{x}_i^{\text{goal}}$:

$$\mathbf{e}_i = \mathbf{x}_i - \mathbf{x}_i^{\text{goal}}$$

### **CLF Candidate**

$$V_i(\mathbf{x}_i) = \frac{1}{2} \|\mathbf{e}_i\|^2 = \frac{1}{2} \|\mathbf{x}_i - \mathbf{x}_i^{\text{goal}}\|^2$$

**Properties:**
- $V_i(\mathbf{x}_i^{\text{goal}}) = 0$ (zero at goal)
- $V_i(\mathbf{x}_i) > 0$ for $\mathbf{x}_i \neq \mathbf{x}_i^{\text{goal}}$ (positive definite)

### **Time Derivative**

$$\dot{V}_i = \mathbf{e}_i^\top \dot{\mathbf{e}}_i = (\mathbf{x}_i - \mathbf{x}_i^{\text{goal}})^\top \dot{\mathbf{x}}_i$$

For single-integrator: $\dot{\mathbf{x}}_i = \mathbf{u}_i$

$$\dot{V}_i = (\mathbf{x}_i - \mathbf{x}_i^{\text{goal}})^\top \mathbf{u}_i$$

**Note:** If $\mathbf{x}_i^{\text{goal}}$ is static, $\dot{\mathbf{x}}_i^{\text{goal}} = \mathbf{0}$.

### **CLF Constraint**

$$\dot{V}_i + \alpha V_i \leq 0$$

$$(\mathbf{x}_i - \mathbf{x}_i^{\text{goal}})^\top \mathbf{u}_i + \frac{\alpha}{2} \|\mathbf{x}_i - \mathbf{x}_i^{\text{goal}}\|^2 \leq 0$$

**Linear constraint in** $\mathbf{u}_i$.

**Code mapping:** `clf_controller.py` → Target tracking CLF

## 6.4 Exploration Mode (No Target)

### **Passive Drift**

When no target is assigned:

$$\mathbf{x}_i^{\text{ref}} = \mathbf{x}_i$$

$$V_i = \|\mathbf{x}_i - \mathbf{x}_i\|^2 = 0$$

**CLF constraint becomes vacuous** (always satisfied).

Robot applies **only CBF constraints**, allowing drift with flow while maintaining safety and connectivity.

**Code mapping:** `hybrid_controller.py` → Scenario 1 (exploration)

---

# 7. HYBRID CLF-CBF CONTROL

## 7.1 Quadratic Program (QP) Formulation

### **Unified Control Problem**

For robot $i$, solve at each time step:

$$\begin{aligned}
\min_{\mathbf{u}_i, \gamma} \quad & \|\mathbf{u}_i\|^2 + \rho \gamma^2 \\
\text{subject to:} \quad & \dot{V}_i + \alpha V_i \leq \gamma \quad &\text{(CLF)} \\
& \dot{h}_{ij}^{\text{safe}} + \beta_s h_{ij}^{\text{safe}} \geq 0, \quad &\forall j \in \mathcal{N}(i) \quad \text{(Safety CBF)} \\
& \dot{h}_{ij}^{\text{conn}} + \beta_c h_{ij}^{\text{conn}} \geq 0, \quad &\forall j \in \mathcal{C}_i \quad \text{(Connectivity CBF)} \\
& \|\mathbf{u}_i\| \leq u_{\max} \quad &\text{(Control bound)}
\end{aligned}$$

**Variables:**
- $\mathbf{u}_i \in \mathbb{R}^2$: control input (to optimize)
- $\gamma \in \mathbb{R}$: CLF relaxation variable

**Parameters:**
- $\alpha > 0$: CLF convergence rate (e.g., $\alpha = 1.5$)
- $\beta_s > 0$: Safety CBF class-$\mathcal{K}$ gain (e.g., $\beta_s = 10.0$)
- $\beta_c > 0$: Connectivity CBF gain (e.g., $\beta_c = 5.0$)
- $\rho > 0$: Relaxation penalty (e.g., $\rho = 50$)
- $u_{\max}$: Maximum control magnitude (e.g., $u_{\max} = 0.4$ m/s)

### **Standard QP Form**

$$\begin{aligned}
\min_{\mathbf{z}} \quad & \frac{1}{2} \mathbf{z}^\top \mathbf{Q} \mathbf{z} + \mathbf{c}^\top \mathbf{z} \\
\text{s.t.} \quad & \mathbf{A} \mathbf{z} \leq \mathbf{b}
\end{aligned}$$

where $\mathbf{z} = [\mathbf{u}_i^\top, \gamma]^\top \in \mathbb{R}^3$.

**Cost Matrix:**

$$\mathbf{Q} = \begin{bmatrix} \mathbf{I}_{2 \times 2} & \mathbf{0} \\ \mathbf{0}^\top & \rho \end{bmatrix}$$

**Linear term:** $\mathbf{c} = \mathbf{0}$

**Constraint matrix:** Stack all constraints into $\mathbf{A}\mathbf{z} \leq \mathbf{b}$.

## 7.2 Constraint Construction

### **CLF Constraint (Converted to $\leq$ form)**

Original: $\dot{V}_i + \alpha V_i \leq \gamma$

For $V_i = \frac{1}{2}\|\mathbf{x}_i - \mathbf{x}_i^{\text{goal}}\|^2$:

$$(\mathbf{x}_i - \mathbf{x}_i^{\text{goal}})^\top \mathbf{u}_i + \frac{\alpha}{2}\|\mathbf{x}_i - \mathbf{x}_i^{\text{goal}}\|^2 - \gamma \leq 0$$

**Matrix form:**

$$\begin{bmatrix} (\mathbf{x}_i - \mathbf{x}_i^{\text{goal}})^\top & -1 \end{bmatrix} \begin{bmatrix} \mathbf{u}_i \\ \gamma \end{bmatrix} \leq -\frac{\alpha}{2}\|\mathbf{x}_i - \mathbf{x}_i^{\text{goal}}\|^2$$

### **Safety CBF Constraints (Converted to $\leq$ form)**

Original: $\dot{h}_{ij}^{\text{safe}} + \beta_s h_{ij}^{\text{safe}} \geq 0$

Negate: $-\dot{h}_{ij}^{\text{safe}} - \beta_s h_{ij}^{\text{safe}} \leq 0$

For $h_{ij}^{\text{safe}} = \|\mathbf{x}_i - \mathbf{x}_j\|^2 - d_{\min}^2$:

$$-2(\mathbf{x}_i - \mathbf{x}_j)^\top \mathbf{u}_i - \beta_s (\|\mathbf{x}_i - \mathbf{x}_j\|^2 - d_{\min}^2) \leq 0$$

(Assuming $\mathbf{u}_j = \mathbf{0}$ or treating neighbor control as disturbance)

**Matrix form:**

$$\begin{bmatrix} -2(\mathbf{x}_i - \mathbf{x}_j)^\top & 0 \end{bmatrix} \begin{bmatrix} \mathbf{u}_i \\ \gamma \end{bmatrix} \leq \beta_s (\|\mathbf{x}_i - \mathbf{x}_j\|^2 - d_{\min}^2)$$

### **Connectivity CBF Constraints**

Original: $\dot{h}_{ij}^{\text{conn}} + \beta_c h_{ij}^{\text{conn}} \geq 0$

Negate: $-\dot{h}_{ij}^{\text{conn}} - \beta_c h_{ij}^{\text{conn}} \leq 0$

For $h_{ij}^{\text{conn}} = R_{\max}^2 - \|\mathbf{x}_i - \mathbf{x}_j\|^2$:

$$2(\mathbf{x}_i - \mathbf{x}_j)^\top \mathbf{u}_i - \beta_c (R_{\max}^2 - \|\mathbf{x}_i - \mathbf{x}_j\|^2) \leq 0$$

**Matrix form:**

$$\begin{bmatrix} 2(\mathbf{x}_i - \mathbf{x}_j)^\top & 0 \end{bmatrix} \begin{bmatrix} \mathbf{u}_i \\ \gamma \end{bmatrix} \leq \beta_c (R_{\max}^2 - \|\mathbf{x}_i - \mathbf{x}_j\|^2)$$

### **Control Bound Constraints**

$\|\mathbf{u}_i\| \leq u_{\max}$ can be linearized as:

$$-u_{\max} \leq u_{i,x} \leq u_{\max}$$
$$-u_{\max} \leq u_{i,y} \leq u_{\max}$$

Or use second-order cone constraint (SOCP).

## 7.3 QP Solution

### **Solver Options**

1. **CVXPY (Python):**
   ```python
   import cvxpy as cp
   
   # Variables
   u = cp.Variable(2)
   gamma = cp.Variable()
   
   # Cost
   cost = cp.sum_squares(u) + rho * cp.square(gamma)
   
   # Constraints
   constraints = [
       # CLF
       (x_i - x_goal).T @ u + alpha/2 * cp.norm(x_i - x_goal)**2 <= gamma,
       
       # Safety CBF for all neighbors
       *[safety_constraint(u, j) for j in neighbors],
       
       # Connectivity CBF for critical neighbors
       *[connectivity_constraint(u, j) for j in critical_neighbors],
       
       # Control bounds
       cp.norm(u, 'inf') <= u_max
   ]
   
   # Solve
   prob = cp.Problem(cp.Minimize(cost), constraints)
   prob.solve(solver=cp.OSQP)
   
   u_opt = u.value
   ```

2. **OSQP (Operator Splitting QP):**
   - Fast, suitable for real-time control
   - Warmstart capability for sequential solves

3. **GUROBI (Commercial):**
   - Highly optimized
   - Requires license

**Code mapping:** `hybrid_controller.py` → `compute_control()`

## 7.4 Feasibility and Safety

### **Theorem 7.1 (Forward Invariance)**

If:
1. QP is feasible at $t = 0$
2. All barrier functions $h$ satisfy $h(\mathbf{x}(0)) \geq 0$

Then:
- QP remains feasible for all $t > 0$
- All barrier constraints satisfied: $h(\mathbf{x}(t)) \geq 0, \, \forall t$

**Proof sketch:**

By CBF condition, if $h(\mathbf{x}) \geq 0$ and $\dot{h} + \beta h \geq 0$, then:

$$h(t) \geq h(0) e^{-\beta t} \geq 0$$

(Exponential barrier function).

### **Relaxation Variable $\gamma$**

- If QP is infeasible (CLF conflicts with CBFs), $\gamma$ allows **soft CLF**
- Penalized by $\rho \gamma^2$ in cost
- Ensures feasibility while prioritizing safety (CBF hard, CLF soft)

**Trade-off:**
- Large $\rho$: Strong CLF enforcement (fast convergence to goal)
- Small $\rho$: Prioritize safety over goal reaching

**Code mapping:** Parameter tuning in simulation config

---

# 8. ROBOT DYNAMICS AND FLOW FIELDS

## 8.1 Single-Integrator Model (Control Design)

### **Dynamics**

$$\dot{\mathbf{x}}_i = \mathbf{u}_i$$

where:
- $\mathbf{x}_i = [x_i, y_i]^\top \in \mathbb{R}^2$: position of robot $i$
- $\mathbf{u}_i = [u_{i,x}, u_{i,y}]^\top \in \mathbb{R}^2$: control input (velocity command)

**Discrete-time integration:**

$$\mathbf{x}_i(k+1) = \mathbf{x}_i(k) + \Delta t \cdot \mathbf{u}_i(k)$$

where $\Delta t = 0.1$ s is the time step.

**Code mapping:** Used for CLF-CBF synthesis in `hybrid_controller.py`

## 8.2 Advection-Diffusion Model (Simulation)

### **Full Dynamics**

$$d\mathbf{x}_i(t) = \mathbf{u}_i(t) \, dt + \mathbf{f}_{\text{flow}}(\mathbf{x}_i, t) \, dt + \mathbf{F}_{\text{drag}}(\dot{\mathbf{x}}_i) \, dt + d\mathbf{W}_i(t)$$

**Terms:**

1. **Control input:** $\mathbf{u}_i(t) \, dt$

2. **Flow advection:** $\mathbf{f}_{\text{flow}}(\mathbf{x}_i, t) \, dt$

3. **Drag force:** $\mathbf{F}_{\text{drag}}(\dot{\mathbf{x}}_i) = -\gamma \dot{\mathbf{x}}_i \, dt$

4. **Brownian motion:** $d\mathbf{W}_i(t)$ (Wiener process)

### **Flow Field Model**

**Vortex-Based Flow:**

$$\mathbf{f}_{\text{flow}}(\mathbf{x}, t) = \mathbf{v}_{\text{bg}}(t) + \sum_{q=1}^{Q} \mathbf{v}_q(\mathbf{x})$$

where:

**Background current:**

$$\mathbf{v}_{\text{bg}}(t) = V_0 \begin{bmatrix} \cos(\omega t) \\ \sin(\omega t) \end{bmatrix}$$

(Rotating current with magnitude $V_0$ and frequency $\omega$)

**Gaussian vortex $q$:**

$$\mathbf{v}_q(\mathbf{x}) = \frac{\kappa_q}{2\pi r_q^2} \exp\left( -\frac{\|\mathbf{x} - \mathbf{c}_q\|^2}{r_q^2} \right) \begin{bmatrix} -(y - c_{q,y}) \\ (x - c_{q,x}) \end{bmatrix}$$

where:
- $\mathbf{c}_q = [c_{q,x}, c_{q,y}]^\top$: vortex center
- $\kappa_q$: vortex strength (positive = counterclockwise, negative = clockwise)
- $r_q$: vortex radius

**Turbulence (Sinusoidal):**

$$\mathbf{v}_{\text{turb}}(\mathbf{x}, t) = A_{\text{turb}} \begin{bmatrix} \sin(k_x x + \omega_x t) \\ \sin(k_y y + \omega_y t) \end{bmatrix}$$

**Code mapping:** `flow_field.py` → `FlowField` class

## 8.3 Hydrodynamic Forces

### **Drag Force**

$$\mathbf{F}_{\text{drag}} = -\frac{1}{2} \rho C_d A \|\mathbf{v}_{\text{rel}}\| \mathbf{v}_{\text{rel}}$$

where:
- $\rho$: fluid density (for water, $\rho = 1000$ kg/m³)
- $C_d$: drag coefficient (typically $C_d \approx 0.8$)
- $A$: frontal area (m²)
- $\mathbf{v}_{\text{rel}} = \dot{\mathbf{x}}_i - \mathbf{f}_{\text{flow}}$: relative velocity

**Simplified linear drag:**

$$\mathbf{F}_{\text{drag}} = -\gamma \dot{\mathbf{x}}_i$$

where $\gamma = \frac{1}{2} \rho C_d A$ is the drag coefficient.

### **Buoyancy (Vertical Motion)**

For 3D extension:

$$F_{\text{buoy}} = (\rho_{\text{water}} - \rho_{\text{robot}}) V g$$

where:
- $V$: robot volume
- $g$: gravitational acceleration

**Code mapping:** `robot.py` → `Robot` class with `apply_drag()`

## 8.4 Stochastic Diffusion

### **Brownian Motion**

$$d\mathbf{W}_i(t) = \sqrt{2D} \, d\mathbf{B}_i(t)$$

where:
- $\mathbf{B}_i(t)$: Standard Brownian motion
- $D$: Diffusion coefficient (m²/s)

**Discrete-time approximation (Euler-Maruyama):**

$$\mathbf{W}_i(k+1) = \mathbf{W}_i(k) + \sqrt{2D \Delta t} \, \boldsymbol{\xi}_i(k)$$

where $\boldsymbol{\xi}_i(k) \sim \mathcal{N}(\mathbf{0}, \mathbf{I})$ is Gaussian noise.

**Effect:** Robots gradually disperse due to turbulence and unmodeled disturbances.

**Code mapping:** `simulation_engine.py` → `step()` with diffusion

---

# 9. COMPLETE SYSTEM INTEGRATION

## 9.1 System Architecture

### **Layered Control Framework**

```
┌─────────────────────────────────────────┐
│  High Level: Mission Planning           │
│  (Target assignment, exploration)       │
└──────────────┬──────────────────────────┘
               │
┌──────────────▼──────────────────────────┐
│  Mid Level: Graph Management            │
│  - Adjacency Consensus                  │
│  - Distributed Edge Detection           │
│  - λ₂ Estimation                        │
│  - Unanimous Voting                     │
└──────────────┬──────────────────────────┘
               │
┌──────────────▼──────────────────────────┐
│  Low Level: CLF-CBF Control             │
│  - Safety barriers (collision avoid)    │
│  - Connectivity barriers (comms)        │
│  - Target tracking (CLF)                │
│  - QP solver                            │
└──────────────┬──────────────────────────┘
               │
┌──────────────▼──────────────────────────┐
│  Physical Layer: Robot Dynamics         │
│  - Flow field advection                 │
│  - Drag forces                          │
│  - Brownian diffusion                   │
└─────────────────────────────────────────┘
```

## 9.2 Algorithm Flow (Per Time Step)

### **Iteration $k$**

**Step 1: Neighbor Discovery**

Each robot $i$ broadcasts position $\mathbf{x}_i(k)$ and receives $\{\mathbf{x}_j(k)\}_{j \in \mathcal{N}_i}$.

**Neighbor set:**

$$\mathcal{N}_i(k) = \{ j : \|\mathbf{x}_i(k) - \mathbf{x}_j(k)\| \leq R_{\max} \}$$

---

**Step 2: Adjacency Consensus Update**

Each robot updates its adjacency estimate:

$$\mathbf{A}^i(k+1) = \mathbf{A}^i(k) + T_d \sum_{j \in \mathcal{N}_i} w_{ij}(k) \left[ \mathbf{A}^j(k) - \mathbf{A}^i(k) \right] + \varepsilon^i(k)$$

**Convergence check:**

$$\Delta_{\max}(k) = \max_{i,j} \| \mathbf{A}^i(k) - \mathbf{A}^j(k) \|_F$$

If $\Delta_{\max}(k) < \epsilon_{\text{consensus}}$, consensus achieved.

---

**Step 3: Distributed Edge Detection**

Each robot extracts edges:

$$\mathcal{E}^i = \{ (p,q) : \mathbf{A}^i_{pq}(k) > \theta_{\text{edge}} \}$$

Find redundant edge candidate:

```
FOR each edge (p,q) ∈ E^i:
    IF has_alternative_path(p, q, A^i):
        IF is_connected(A^i \ {(p,q)}):
            L_test = Laplacian(A^i \ {(p,q)})
            λ₂_test = compute_lambda2(L_test)
            IF λ₂_test ≥ λ_ref:
                candidate^i = (p, q)
                BREAK
```

---

**Step 4: Unanimous Voting**

Robots exchange candidates:

$$\mathcal{C}(k) = \{ c^1, c^2, \ldots, c^N \}$$

**Decision:**

$$\text{Decision}(k) = \begin{cases}
\text{PRUNE}(c) & \text{if } |\mathcal{C}(k)| = 1, \, c \neq \emptyset \\
\text{DISAGREE} & \text{if } |\mathcal{C}(k)| > 1 \\
\text{DONE} & \text{if } \mathcal{C}(k) = \{\emptyset\}
\end{cases}$$

If PRUNE, update:

$$\mathbf{A}^i_{pq}(k+1) = 0, \quad \mathbf{A}^i_{qp}(k+1) = 0$$

---

**Step 5: CLF-CBF Control**

Extract critical neighbors from pruned graph:

$$\mathcal{C}_i = \{ j : (i,j) \in \mathcal{E}^i_{\text{pruned}} \}$$

Solve QP:

$$\begin{aligned}
\min_{\mathbf{u}_i, \gamma} \quad & \|\mathbf{u}_i\|^2 + \rho \gamma^2 \\
\text{s.t.} \quad & \dot{V}_i + \alpha V_i \leq \gamma \\
& \dot{h}_{ij}^{\text{safe}} + \beta_s h_{ij}^{\text{safe}} \geq 0, \quad \forall j \in \mathcal{N}_i \\
& \dot{h}_{ij}^{\text{conn}} + \beta_c h_{ij}^{\text{conn}} \geq 0, \quad \forall j \in \mathcal{C}_i \\
& \|\mathbf{u}_i\| \leq u_{\max}
\end{aligned}$$

---

**Step 6: Dynamics Update**

Apply control and integrate:

$$\mathbf{x}_i(k+1) = \mathbf{x}_i(k) + \Delta t \left[ \mathbf{u}_i(k) + \mathbf{f}_{\text{flow}}(\mathbf{x}_i(k), t) + \mathbf{F}_{\text{drag}}(\dot{\mathbf{x}}_i) \right] + \sqrt{2D\Delta t} \, \boldsymbol{\xi}_i(k)$$

---

## 9.3 Parameter Summary

### **Graph Parameters**

| Parameter | Symbol | Typical Value | Description |
|-----------|--------|---------------|-------------|
| Communication range | $R_{\max}$ | 1.2 m | Maximum edge distance |
| Safety distance | $d_{\min}$ | 0.6 m | Collision avoidance |
| λ₂ threshold | $\lambda_{\text{ref}}$ | 0.1 | Connectivity safety margin |
| Edge threshold | $\theta_{\text{edge}}$ | 0.1 | Consensus edge detection |

### **Consensus Parameters**

| Parameter | Symbol | Typical Value | Description |
|-----------|--------|---------------|-------------|
| Sample time | $T_d$ | 0.1 s | Consensus step size |
| Trust bandwidth | $\sigma$ | 1.0 m | Distance-based trust |
| Convergence tol | $\epsilon_{\text{consensus}}$ | 0.01 | Agreement threshold |

### **CLF-CBF Parameters**

| Parameter | Symbol | Typical Value | Description |
|-----------|--------|---------------|-------------|
| CLF rate | $\alpha$ | 1.5 | Convergence speed |
| Safety CBF gain | $\beta_s$ | 10.0 | Collision barrier strength |
| Connectivity gain | $\beta_c$ | 5.0 | Communication barrier strength |
| Relaxation penalty | $\rho$ | 50.0 | CLF softness weight |
| Max control | $u_{\max}$ | 0.4 m/s | Speed limit |

### **Physical Parameters**

| Parameter | Symbol | Typical Value | Description |
|-----------|--------|---------------|-------------|
| Robot mass | $m$ | 2.0 kg | Inertia |
| Drag coefficient | $C_d$ | 0.8 | Hydrodynamic drag |
| Frontal area | $A$ | 0.01 m² | Cross-section |
| Diffusion coeff | $D$ | 0.001 m²/s | Brownian diffusion |
| Time step | $\Delta t$ | 0.1 s | Integration step |

---

# 10. CONVERGENCE GUARANTEES

## 10.1 Consensus Convergence

**Theorem 10.1 (Adjacency Consensus):**

Under assumptions:
1. Communication graph $\mathcal{G}_{\text{comm}}(t)$ remains connected
2. Trust weights symmetric and positive
3. Step size $T_d < \frac{2}{\lambda_N(\mathcal{L}_{\text{comm}})}$

The consensus error decays exponentially:

$$\|\mathbf{A}^i(k) - \mathbf{A}^*\| \leq C e^{-\lambda_2 T_d k}$$

**Convergence time to $\epsilon$ accuracy:**

$$k_{\epsilon} \geq \frac{1}{\lambda_2 T_d} \ln\left(\frac{C}{\epsilon}\right)$$

**Example:**
- $\lambda_2 = 0.5$, $T_d = 0.1$, $C = 1.0$, $\epsilon = 0.01$
- $k_{\epsilon} \geq \frac{1}{0.05} \ln(100) \approx 92$ iterations

## 10.2 Connectivity Preservation

**Theorem 10.2 (Safety via CBF):**

If:
1. Initial graph connected: $\lambda_2(\mathbf{L}(0)) > 0$
2. All robots satisfy connectivity CBF: $\dot{h}_{ij}^{\text{conn}} + \beta_c h_{ij}^{\text{conn}} \geq 0$
3. Pruning only when $\lambda_2(\mathbf{L} \setminus e) \geq \lambda_{\text{ref}}$

Then graph remains connected for all time:

$$\lambda_2(\mathbf{L}(t)) \geq \lambda_{\text{ref}} > 0, \quad \forall t \geq 0$$

**Proof:**
- CBF ensures $h_{ij}^{\text{conn}}(t) \geq 0 \implies \|\mathbf{x}_i - \mathbf{x}_j\| \leq R_{\max}$
- Pruning condition ensures $\lambda_2 \geq \lambda_{\text{ref}}$ after removal
- By Fiedler theorem, $\lambda_2 > 0 \implies$ connected

## 10.3 Target Convergence

**Theorem 10.3 (CLF Tracking):**

If:
1. Target $\mathbf{x}_i^{\text{goal}}$ is static
2. CLF constraint satisfied: $\dot{V}_i + \alpha V_i \leq \gamma$
3. QP is feasible

Then:

$$\|\mathbf{x}_i(t) - \mathbf{x}_i^{\text{goal}}\| \leq \|\mathbf{x}_i(0) - \mathbf{x}_i^{\text{goal}}\| e^{-\frac{\alpha}{2} t} + \frac{\sqrt{\rho}}{\alpha} \gamma^*$$

where $\gamma^*$ is optimal relaxation value.

**Asymptotic convergence:**

If $\gamma^* \to 0$ (no conflict between CLF and CBF), then:

$$\lim_{t \to \infty} \mathbf{x}_i(t) = \mathbf{x}_i^{\text{goal}}$$

## 10.4 Distributed Pruning Optimality

**Theorem 10.4 (Asymptotic MST Convergence):**

As robots disperse under flow and consensus converges, the total edge length of the pruned graph approaches that of the Minimum Spanning Tree (MST):

$$\lim_{t \to \infty} \sum_{(i,j) \in \mathcal{E}_{\text{pruned}}(t)} \|\mathbf{x}_i(t) - \mathbf{x}_j(t)\| = \sum_{(i,j) \in \text{MST}(t)} \|\mathbf{x}_i(t) - \mathbf{x}_j(t)\|$$

**Empirical validation:** Figure 4 in paper shows convergence.

---

## APPENDIX: Code-to-Math Mapping

### **File:** `graph/connectivity_tree.py`
- **Function:** `build_spanning_tree()`
- **Math:** Constructs tree from adjacency matrix using BFS

### **File:** `graph/lambda2_estimator.py`
- **Function:** `get_lambda2(A, mode='auto')`
- **Math:** Computes $\lambda_2(\mathbf{L})$ using exact, Cheeger, or incremental method

### **File:** `consensus/adjacency_consensus.py`
- **Function:** `consensus_update_step()`
- **Math:** Implements $\mathbf{A}^i(k+1) = \mathbf{A}^i(k) + T_d \Delta \mathbf{A}^i(k)$

### **File:** `consensus/hybrid_pruning.py`
- **Function:** `find_redundant_edge_distributed()`
- **Math:** Distributed edge detection using consensus estimate $\mathbf{A}^i$

### **File:** `controllers/cbf_controller.py`
- **Function:** `compute_control()`
- **Math:** Solves safety CBF: $\dot{h}_{ij}^{\text{safe}} + \beta_s h_{ij}^{\text{safe}} \geq 0$

### **File:** `controllers/clf_controller.py`
- **Function:** `compute_control()`
- **Math:** Solves tracking CLF: $\dot{V}_i + \alpha V_i \leq 0$

### **File:** `controllers/hybrid_controller.py`
- **Function:** `compute_control()`
- **Math:** Unified CLF-CBF QP

### **File:** `core/flow_field.py`
- **Function:** `get_flow_velocity()`
- **Math:** Computes $\mathbf{f}_{\text{flow}}(\mathbf{x}, t)$ with vortices

### **File:** `simulation/simulation_engine.py`
- **Function:** `step()`
- **Math:** Integrates dynamics with flow, drag, diffusion

---

**END OF MATHEMATICAL FOUNDATION**

---

This document provides complete mathematical foundations for the entire distributed consensus-based multi-robot control system, from graph theory to CLF-CBF control, with explicit mappings to code implementation.
