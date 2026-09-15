# Complete System Documentation: Underwater Multi-Robot Consensus Pruning

## **Table of Contents**

1. [System Overview](#1-system-overview)
2. [Mathematical Foundation](#2-mathematical-foundation)
3. [System Architecture](#3-system-architecture)
4. [Robot Dynamics & Physics](#4-robot-dynamics--physics)
5. [Control System (CLF-CBF)](#5-control-system-clf-cbf)
6. [Communication & Graph Topology](#6-communication--graph-topology)
7. [Distributed Consensus Algorithm](#7-distributed-consensus-algorithm)
8. [Complete System Flow](#8-complete-system-flow)
9. [Implementation Details](#9-implementation-details)
10. [Parameters & Tuning](#10-parameters--tuning)

---

## **1. System Overview**

### **1.1 Problem Statement**

**Scenario:** A swarm of $n$ underwater robots must:
1. Navigate from start positions to a goal region
2. Maintain connectivity through communication graph
3. Optimize network topology by removing redundant edges
4. Avoid collisions while moving through dynamic flow fields

**Challenges:**
- **Dynamic environment**: Underwater currents (constant drift + sinusoidal swirl)
- **Physical constraints**: Each robot has unique mass, drag, buoyancy
- **Communication limits**: Finite communication radius $R_{comm} = 2.5m$
- **Distributed coordination**: No central controller - all decisions through consensus
- **Safety requirements**: No collisions, maintain connectivity, preserve algebraic connectivity

---

### **1.2 System Components**

```
┌─────────────────────────────────────────────────────────────┐
│                     COMPLETE SYSTEM                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────┐      ┌──────────────┐    ┌─────────────┐ │
│  │   PHYSICS   │──────▶│   CONTROL    │────▶│  TOPOLOGY   │ │
│  │  (Dynamics) │      │  (CLF-CBF)   │    │  (Consensus)│ │
│  └─────────────┘      └──────────────┘    └─────────────┘ │
│         │                     │                    │        │
│         │                     │                    │        │
│    Flow Field           Goal-seeking         Edge Pruning  │
│    + Diffusion          + Safety             via Consensus │
│                                                             │
│  OUTPUT: Robot trajectories + optimized communication graph│
└─────────────────────────────────────────────────────────────┘
```

**Three Layers:**

1. **Physical Layer** (Bottom): Robot dynamics, underwater flow, environmental forces
2. **Control Layer** (Middle): CLF-CBF controllers for motion planning and safety
3. **Coordination Layer** (Top): Distributed consensus for topology optimization

---

## **2. Mathematical Foundation**

### **2.1 State Space**

**Robot state:**

$$
\mathbf{x}_i(t) = \begin{bmatrix} p_{i,x}(t) \\ p_{i,y}(t) \\ v_{i,x}(t) \\ v_{i,y}(t) \end{bmatrix} \in \mathbb{R}^4
$$

where:
- $\mathbf{p}_i = [p_{i,x}, p_{i,y}]^T$: Position in 2D workspace
- $\mathbf{v}_i = [v_{i,x}, v_{i,y}]^T$: Velocity

**System state:**

$$
\mathbf{X}(t) = [\mathbf{x}_1(t), \mathbf{x}_2(t), \ldots, \mathbf{x}_n(t)]^T \in \mathbb{R}^{4n}
$$

---

### **2.2 Communication Graph**

**Definition:**

$$
\mathcal{G}(t) = (\mathcal{V}, \mathcal{E}(t))
$$

where:
- $\mathcal{V} = \{0, 1, \ldots, n-1\}$: Robot set
- $\mathcal{E}(t) = \{(i,j) : \|\mathbf{p}_i(t) - \mathbf{p}_j(t)\| \leq R_{comm}\}$: Edge set

**Adjacency Matrix:**

$$
A_{ij}(t) = \begin{cases}
e^{-\frac{d_{ij}^2(t)}{\sigma^2}} & \text{if } (i,j) \in \mathcal{E}(t) \\
0 & \text{otherwise}
\end{cases}
$$

where $d_{ij}(t) = \|\mathbf{p}_i(t) - \mathbf{p}_j(t)\|$

**Graph Laplacian:**

$$
L(t) = D(t) - A(t)
$$

where $D(t) = \text{diag}(\sum_j A_{ij})$ is the degree matrix.

**Algebraic Connectivity:**

$$
\lambda_2(t) = \text{second smallest eigenvalue of } L(t)
$$

**Key property:** $\lambda_2 > 0 \iff$ graph is connected.

---

## **3. System Architecture**

### **3.1 Module Hierarchy**

```
simulation_engine.py
    ├── Robot Dynamics (core/robot.py)
    │   ├── ChainRobot class
    │   └── Physical properties (mass, drag, area)
    │
    ├── Flow Field (core/flow_field.py)
    │   └── Underwater current model
    │
    ├── Controllers (controllers/)
    │   ├── CLFController (goal-seeking)
    │   ├── CBFController (safety + connectivity)
    │   └── HybridCLFCBFController (combined)
    │
    ├── Graph Management (graph/)
    │   ├── TopologyManager (neighbor tracking)
    │   ├── ConnectivityTree (spanning tree)
    │   └── EdgeAnalyzer (redundancy detection)
    │
    └── Consensus (consensus/)
        ├── AdjacencyMatrixConsensus (Griparic et al. 2022)
        ├── HybridPruningManager (orchestration)
        └── Lambda2Manager (algebraic connectivity)
```

---

### **3.2 Data Flow**

```
Time step k:

1. SENSE:
   positions[k] = [p_0, p_1, ..., p_{n-1}]
   
2. COMMUNICATE:
   neighbor_graph[k] = build_from_distances(positions[k], R_comm)
   
3. CONTROL:
   for each robot i:
       u_i[k] = CLF(goal) + CBF(safety) + CBF(connectivity)
   
4. DYNAMICS:
   for each robot i:
       v_i[k+1] = v_i[k] + u_i[k]·dt + flow(p_i,t) + noise
       p_i[k+1] = p_i[k] + v_i[k+1]·dt
   
5. CONSENSUS (every N steps):
   if topology_stable:
       redundant_edge = distributed_consensus()
       if unanimous:
           prune(redundant_edge)
```

---

## **4. Robot Dynamics & Physics**

### **4.1 Continuous-Time Dynamics**

Each robot follows **first-order dynamics with flow advection:**

$$
\dot{\mathbf{p}}_i(t) = \mathbf{v}_{flow}(\mathbf{p}_i, t) + \mathbf{v}_i(t) + \boldsymbol{\xi}_i(t)
$$

$$
\dot{\mathbf{v}}_i(t) = \mathbf{u}_i(t) - \gamma \mathbf{v}_i(t)
$$

where:
- $\mathbf{v}_{flow}(\mathbf{p}, t)$: Flow field velocity at position $\mathbf{p}$
- $\mathbf{v}_i(t)$: Robot's self-propelled velocity
- $\boldsymbol{\xi}_i(t) \sim \mathcal{N}(0, \sigma_{diffusion}^2 \mathbf{I})$: Brownian motion (turbulence)
- $\mathbf{u}_i(t)$: Control input (thrust force)
- $\gamma = 0.85$: Velocity damping coefficient

---

### **4.2 Flow Field Model**

**Base current + sinusoidal swirl:**

$$
\mathbf{v}_{flow}(\mathbf{p}, t) = \mathbf{v}_{base} + A_{swirl} \begin{bmatrix}
\sin\left(\frac{p_y + t}{s_{swirl}}\right) \\
\cos\left(\frac{p_x + 0.3t}{s_{swirl}}\right)
\end{bmatrix}
$$

**Default parameters:**
- $\mathbf{v}_{base} = [0.20, 0.05]^T$ m/s (constant drift)
- $A_{swirl} = 0.10$ (swirl amplitude)
- $s_{swirl} = 5.5$ (swirl spatial scale)

**Physical interpretation:**
- Constant eastward current with weak northward component
- Time-varying vortical structures (eddies)
- Spatially heterogeneous flow

---

### **4.3 Physical Properties (Per Robot)**

Each robot has **randomized** physical characteristics:

| Property | Symbol | Distribution | Units |
|----------|--------|--------------|-------|
| Mass | $m_i$ | $\mathcal{N}(2.0, 0.2)$ | kg |
| Drag coefficient | $C_{d,i}$ | $\mathcal{N}(0.8, 0.1)$ | - |
| Cross-sectional area | $A_{i}$ | $\mathcal{N}(0.01, 0.002)$ | m² |
| Diffusion strength | $\sigma_{\xi,i}$ | 0.008 | m/s |
| Mobility | $\mu_i$ | 0.45 | - |

**Heterogeneity reason:** Real robots have manufacturing tolerances and different payloads.

---

### **4.4 Discrete-Time Update (Implementation)**

```python
def robot_step(p_i, v_i, u_i, dt, flow_field, time):
    """
    One timestep of robot dynamics.
    
    Args:
        p_i: Position (2D)
        v_i: Velocity (2D)
        u_i: Control input (2D)
        dt: Time step
        flow_field: FlowField object
        time: Current time
    
    Returns:
        p_i_new, v_i_new
    """
    # Flow velocity at current position
    v_flow = flow_field.velocity(p_i, time)
    
    # Control acceleration
    v_i = v_i + u_i * dt
    
    # Turbulence (Brownian motion)
    agitation = np.random.normal(0.0, diffusion, size=2)
    v_i = v_i + agitation * mobility
    
    # Damping
    v_i = v_i * 0.85
    
    # Position update
    p_i_new = p_i + dt * (v_flow + v_i)
    
    # Enforce workspace boundaries
    p_i_new = np.clip(p_i_new, [0, 0], workspace_bounds)
    
    return p_i_new, v_i
```

**Timestep:** $\Delta t = 0.05$ seconds (default)

---

## **5. Control System (CLF-CBF)**

### **5.1 Control Architecture**

**Hybrid controller combines three objectives:**

$$
\mathbf{u}_i(t) = \mathbf{u}_{CLF}^i(t) + \mathbf{u}_{CBF,safety}^i(t) + \mathbf{u}_{CBF,conn}^i(t)
$$

subject to: $\|\mathbf{u}_i(t)\| \leq u_{max} = 1.5$

---

### **5.2 Control Lyapunov Function (CLF) - Goal Seeking**

**Objective:** Drive robot toward goal position $\mathbf{p}_{goal}$.

**Lyapunov function:**

$$
V_{CLF}^i(\mathbf{p}_i) = \frac{1}{2} \|\mathbf{p}_i - \mathbf{p}_{goal}\|^2
$$

**Control law:**

$$
\mathbf{u}_{CLF}^i = -k_{CLF} \nabla_{\mathbf{p}_i} V_{CLF}^i = -k_{CLF} (\mathbf{p}_i - \mathbf{p}_{goal})
$$

**Gain modulation:**

$$
k_{CLF} = \begin{cases}
k_{nominal} = 0.3 & \text{if robot connected to tree} \\
0.15 \cdot k_{nominal} & \text{if robot disconnected}
\end{cases}
$$

**Rationale:** Disconnected robots move slowly to avoid creating more topology changes.

**Lyapunov guarantee:**

$$
\dot{V}_{CLF}^i \leq -k_{CLF} \|\mathbf{p}_i - \mathbf{p}_{goal}\|^2 < 0
$$

Thus: $\mathbf{p}_i(t) \to \mathbf{p}_{goal}$ as $t \to \infty$

---

### **5.3 Control Barrier Function (CBF) - Collision Avoidance**

**Objective:** Maintain safe distance $d_{safe} = 0.3m$ between robots.

**Barrier function** (for pair $(i,j)$):

$$
h_{safety}^{ij}(\mathbf{p}_i, \mathbf{p}_j) = \|\mathbf{p}_i - \mathbf{p}_j\|^2 - d_{safe}^2
$$

**Safe set:** $\mathcal{S}_{safety}^{ij} = \{\mathbf{p}: h_{safety}^{ij}(\mathbf{p}_i, \mathbf{p}_j) \geq 0\}$

**Control law** (when $h_{safety}^{ij} < \epsilon_{threshold}$):

$$
\mathbf{u}_{CBF,safety}^i = k_{safety} \cdot (-h_{safety}^{ij}) \cdot \frac{\mathbf{p}_i - \mathbf{p}_j}{\|\mathbf{p}_i - \mathbf{p}_j\|}
$$

**Aggregation over all neighbors:**

$$
\mathbf{u}_{CBF,safety}^i = \sum_{j \in \mathcal{N}_i} \mathbf{u}_{CBF,safety}^{ij}
$$

**Parameters:**
- $d_{safe} = 0.3m$ (safety distance)
- $k_{safety} = 1.0$ (repulsion gain)
- $\epsilon_{threshold} = 2 \cdot d_{safe} = 0.6m$ (activation distance)

**Barrier guarantee:**

$$
h_{safety}^{ij}(\mathbf{p}_i, \mathbf{p}_j) \geq 0 \implies \text{no collision}
$$

---

### **5.4 Control Barrier Function (CBF) - Connectivity Maintenance**

**Objective:** Keep robot within communication range of parent in connectivity tree.

**Barrier function** (robot $i$ with parent $p$):

$$
h_{conn}^{ip}(\mathbf{p}_i, \mathbf{p}_p) = d_{max}^2 - \|\mathbf{p}_i - \mathbf{p}_p\|^2
$$

where $d_{max} = 0.95 \cdot R_{comm} = 2.375m$ (with safety margin).

**Safe set:** $\mathcal{S}_{conn}^{ip} = \{\mathbf{p}: h_{conn}^{ip}(\mathbf{p}_i, \mathbf{p}_p) \geq 0\}$

**Control law** (when $h_{conn}^{ip} < 0.2$):

$$
\mathbf{u}_{CBF,conn}^i = k_{conn} \cdot (-h_{conn}^{ip}) \cdot \frac{\mathbf{p}_p - \mathbf{p}_i}{\|\mathbf{p}_p - \mathbf{p}_i\|}
$$

**Parameters:**
- $k_{conn} = 0.8$ (attraction gain)
- $R_{comm} = 2.5m$ (communication radius)

**Barrier guarantee:**

$$
h_{conn}^{ip}(\mathbf{p}_i, \mathbf{p}_p) \geq 0 \implies \text{parent link maintained}
$$

---

### **5.5 Control Magnitude Limiting**

**Saturation:**

$$
\mathbf{u}_i = \begin{cases}
\mathbf{u}_i^{desired} & \text{if } \|\mathbf{u}_i^{desired}\| \leq u_{max} \\
u_{max} \cdot \frac{\mathbf{u}_i^{desired}}{\|\mathbf{u}_i^{desired}\|} & \text{otherwise}
\end{cases}
$$

where $u_{max} = 1.5$ (maximum thrust).

---

## **6. Communication & Graph Topology**

### **6.1 Neighbor Graph Construction**

**Proximity-based communication:**

$$
(i, j) \in \mathcal{E}(t) \iff \|\mathbf{p}_i(t) - \mathbf{p}_j(t)\| \leq R_{comm}
$$

**Weighted adjacency matrix:**

$$
A_{ij}(t) = \begin{cases}
t_{ij}(t) = e^{-\frac{d_{ij}^2(t)}{\sigma^2}} & \text{if } d_{ij}(t) \leq R_{comm} \\
0 & \text{otherwise}
\end{cases}
$$

where:
- $\sigma = 1.0$ (trust function sensitivity)
- $t_{ij}(t) \in [0,1]$: Link quality (trust)

**Physical interpretation:**
- Closer robots → higher trust
- Far robots → lower trust
- Beyond $R_{comm}$ → no communication

---

### **6.2 Connectivity Tree (Spanning Tree)**

**Purpose:** Provides hierarchical structure for coordination.

**Construction:** Breadth-First Search (BFS) from root (Robot 0):

```
Algorithm: Build Connectivity Tree
Input: Robots, Neighbor Graph G
Output: Parent assignments

1. root = Robot 0
2. Queue Q = [root]
3. Visited = {root}
4. root.parent = None

5. while Q not empty:
6.     current = Q.dequeue()
7.     for neighbor in neighbors[current]:
8.         if neighbor not in Visited:
9.             neighbor.parent = current
10.            Q.enqueue(neighbor)
11.            Visited.add(neighbor)
```

**Properties:**
- **Acyclic:** No loops
- **Connected:** All robots reachable from root (if graph connected)
- **Parent assignment:** Each robot knows its parent
- **Used for:** CLF gain modulation, connectivity maintenance

---

### **6.3 Edge Redundancy Criteria**

An edge $(i,j)$ is **redundant** if removing it satisfies:

**Criterion 1: Alternative Path Exists**

$$
\exists \text{ path from } i \text{ to } j \text{ in } \mathcal{G} \setminus \{(i,j)\}
$$

**Criterion 2: Graph Remains Connected**

$$
\lambda_2(\mathcal{G} \setminus \{(i,j)\}) > 0
$$

**Criterion 3: Algebraic Connectivity Maintained**

$$
\lambda_2(\mathcal{G} \setminus \{(i,j)\}) \geq \lambda_{2,threshold} = 0.1
$$

**All three must hold** to prune edge $(i,j)$.

---

### **6.4 Topology Stability**

**Definition:** Topology is **stable** if no edges form/break for $T_{stable}$ consecutive timesteps.

$$
\mathcal{E}(t) = \mathcal{E}(t - \Delta t) = \cdots = \mathcal{E}(t - T_{stable} \cdot \Delta t)
$$

**Default:** $T_{stable} = 5$ timesteps (0.25 seconds).

**Reason:** Consensus requires stable topology to converge. If edges keep changing, consensus is aborted and restarted.

---

## **7. Distributed Consensus Algorithm**

### **7.1 Consensus Protocol (Griparic et al. 2022)**

**Goal:** All robots converge to same estimate $\mathbf{A}^*$ of adjacency matrix.

**Each robot $l$ maintains:**

$$
\mathbf{A}^l(k) \in \mathbb{R}^{n \times n}
$$

**Update rule:**

$$
\mathbf{A}^l(k+1) = \mathbf{A}^l(k) + T_d \cdot \Delta \mathbf{A}^l(k)
$$

where:

$$
\Delta \mathbf{A}_{ij}^l(k) = \underbrace{\sum_{p \in \mathcal{N}_l} w_{lp} \left( A_{ij}^p(k) - A_{ij}^l(k) \right)}_{\text{Consensus term}} + \underbrace{\varepsilon_{ij}^l(k)}_{\text{Observation term}}
$$

**Observation correction:**

$$
\varepsilon_{ij}^l(k) = \begin{cases}
t_{ij}^l(k) - a_{ij}^l(k) & \text{if edge } (i,j) \text{ incident to robot } l \\
0 & \text{otherwise}
\end{cases}
$$

where $t_{ij}^l(k) = e^{-\frac{d_{ij}^2(k)}{\sigma^2}}$ is the observed trust.

**Trust-based weights:**

$$
w_{lp} = \frac{t_{lp}}{\sum_{q \in \mathcal{N}_l} t_{lq}}
$$

(Normalized by total trust to neighbors)

**Parameters:**
- $T_d = 0.2$ (sample time)
- $\sigma = 1.0$ (trust sensitivity)
- $\varepsilon_{conv} = 10^{-4}$ (convergence threshold)

---

### **7.2 Convergence Guarantee**

**Theorem (Griparic et al. 2022):**

If communication graph is **connected**, then:

$$
\lim_{k \to \infty} \mathbf{A}^l(k) = \mathbf{A}^* \quad \forall l
$$

where $\mathbf{A}^*$ is the **true adjacency matrix**.

**Convergence rate:**

$$
\|\mathbf{A}^l(k) - \mathbf{A}^*\| \leq C \cdot \exp(-\lambda_2 \cdot T_d \cdot k)
$$

**Exponentially fast!** Typically converges in **20-50 iterations**.

---

### **7.3 Distributed Edge Detection**

After consensus, each robot **independently** detects redundant edges using **only its own estimate** $\mathbf{A}^l$:

```python
def find_redundant_edge_distributed(robot_id):
    """
    Find redundant edge using ONLY robot's consensus estimate.
    No global knowledge required!
    """
    # Get robot's own estimate (NOT global truth!)
    A_estimate = A_estimates[robot_id]  # n×n matrix
    
    # Extract edges from estimate
    edges = extract_edges(A_estimate, threshold=0.01)
    
    # For each edge, check if redundant
    candidates = []
    for edge in edges:
        # Alternative path check (graph search on A_estimate)
        has_alt_path = has_alternative_path(edge, A_estimate)
        
        if not has_alt_path:
            continue  # Not redundant
        
        # Connectivity check (remove edge from A_estimate)
        A_test = A_estimate.copy()
        A_test[i, j] = 0
        A_test[j, i] = 0
        
        is_connected = check_connectivity(A_test)
        
        if not is_connected:
            continue  # Disconnects graph
        
        # Lambda2 check
        lambda2 = compute_lambda2(A_test)
        
        if lambda2 < threshold:
            continue  # Algebraic connectivity too low
        
        # All checks passed!
        candidates.append((edge, A_test[i,j]))  # Store with strength
    
    # Return weakest redundant edge
    if candidates:
        candidates.sort(key=lambda x: x[1])  # Sort by strength
        return candidates[0][0]  # Weakest edge
    
    return None
```

**Key insight:** Since all $\mathbf{A}^l \approx \mathbf{A}^*$ after consensus, all robots find **same redundant edge**!

---

### **7.4 Unanimous Decision Protocol**

**After consensus:**

1. Each robot $l$ finds redundant edge: $c_l$
2. Robots exchange candidates: $\{c_0, c_1, \ldots, c_{n-1}\}$
3. **Unanimous check:**

$$
\text{Unanimous}(\mathcal{C}) = \begin{cases}
\text{TRUE} & \text{if } |\mathcal{C}| = 1 \text{ (all same)} \\
\text{FALSE} & \text{otherwise}
\end{cases}
$$

4. **Decision:**

$$
\text{Action} = \begin{cases}
\text{PRUNE}(c) & \text{if Unanimous}(\{c\}) \\
\text{ABORT} & \text{otherwise}
\end{cases}
$$

**Why unanimous?**
- **Safety:** Prevents split decisions
- **Consistency:** All robots prune same edge simultaneously
- **Correctness:** Guaranteed by consensus convergence

---

## **8. Complete System Flow**

### **8.1 High-Level Loop**

```
INITIALIZATION:
    1. Spawn n robots in cluster
    2. Generate random goal position
    3. Initialize communication graph
    4. Build connectivity tree

MAIN LOOP (each timestep Δt = 0.05s):
    
    ┌─────────────────────────────────────┐
    │  PHASE 1: ROBOT MOTION              │
    └─────────────────────────────────────┘
    
    1.1. Build connectivity tree (BFS from root)
    
    1.2. For each robot i:
         - Compute CLF control (goal-seeking)
         - Compute CBF safety control (collision avoidance)
         - Compute CBF connectivity control (parent maintenance)
         - Combine: u_i = CLF + CBF_safety + CBF_conn
         - Saturate: u_i = min(u_i, u_max)
    
    1.3. For each robot i:
         - Apply control: v_i += u_i * dt
         - Add flow: v_flow = flow_field(p_i, t)
         - Add noise: v_i += N(0, σ_diffusion)
         - Damp velocity: v_i *= 0.85
         - Update position: p_i += (v_flow + v_i) * dt
         - Enforce bounds: p_i = clip(p_i, workspace)
    
    ┌─────────────────────────────────────┐
    │  PHASE 2: TOPOLOGY UPDATE           │
    └─────────────────────────────────────┘
    
    2.1. Save old edges: E_old = current_edges()
    
    2.2. Update neighbor graph:
         For all pairs (i,j):
             d_ij = ||p_i - p_j||
             if d_ij <= R_comm:
                 A[i,j] = exp(-d_ij² / σ²)
             else:
                 A[i,j] = 0
    
    2.3. Detect topology changes:
         E_new = current_edges()
         broken = E_old \ E_new
         formed = E_new \ E_old
    
    2.4. If topology changed:
         - Reset consensus state
         - Reset stability counter
         - GOTO PHASE 1 (wait for stability)
    
    2.5. Else (topology stable):
         - Increment stability counter
    
    ┌─────────────────────────────────────┐
    │  PHASE 3: CONSENSUS & PRUNING       │
    └─────────────────────────────────────┘
    
    3.1. Check stability:
         if stability_counter < T_stable:
             GOTO PHASE 1 (not stable yet)
    
    3.2. Add to rolling window:
         history.append({
             'edges': E_new,
             'lengths': {(i,j): d_ij for (i,j) in E_new},
             'time': t
         })
    
    3.3. Check history:
         if len(history) < window_size:
             GOTO PHASE 1 (building history)
    
    3.4. If NOT in consensus:
         - Find persistent redundant edge (from history)
         - If found: START consensus
         - Else: GOTO PHASE 1 (no redundant edges)
    
    3.5. If in consensus:
         - Run consensus update:
             For each robot l:
                 - Get neighbors N_l
                 - Get observations: t_ij for edges incident to l
                 - Update: A^l += T_d * (consensus_term + observation_term)
         
         - Check convergence:
             max_change = max_l ||A^l(k) - A^l(k-1)||
             if max_change < ε_conv:
                 converged = TRUE
    
    3.6. If converged:
         - Each robot finds redundant edge from A^l
         - Exchange candidates
         - Check unanimity:
             if all robots agree:
                 PRUNE edge
                 Reset consensus
             else:
                 ABORT (mark edge as non-redundant)
                 Reset consensus
    
    3.7. Else (not converged):
         - If consensus_rounds >= max_rounds:
             ABORT (timeout)
         - Else:
             Continue consensus (GOTO PHASE 1)

END LOOP
```

---

### **8.2 Detailed Consensus Flow**

```
CONSENSUS PHASE (when topology stable):

┌───────────────────────────────────────────────────────┐
│ STEP 1: Initialization (Round k=0)                    │
└───────────────────────────────────────────────────────┘

For each robot l:
    A^l[l, :] = 0  (zero out own row)
    A^l[:, l] = 0  (zero out own column)
    
    For each neighbor j in N_l:
        d_lj = ||p_l - p_j||
        t_lj = exp(-d_lj² / σ²)
        A^l[l, j] = t_lj  (observe own edges)
        A^l[j, l] = t_lj  (symmetric)

Example (3 robots):
    Robot 0 sees neighbors {1, 2}:
        A^0 = [[0, 0.95, 0.87],
               [0, 0,    0   ],
               [0, 0,    0   ]]  (only knows own row)
    
    Robot 1 sees neighbors {0, 2}:
        A^1 = [[0, 0,    0   ],
               [0.95, 0, 0.92],
               [0, 0,    0   ]]  (only knows own row)
    
    Robot 2 sees neighbors {0, 1}:
        A^2 = [[0, 0,    0   ],
               [0, 0,    0   ],
               [0.87, 0.92, 0]]  (only knows own row)

┌───────────────────────────────────────────────────────┐
│ STEP 2: Consensus Iteration (Round k)                 │
└───────────────────────────────────────────────────────┘

For each robot l:
    
    # Consensus term: average with neighbors
    ΔA_consensus = 0
    for neighbor p in N_l:
        weight = t_lp / (sum of all t_lq)
        ΔA_consensus += weight * (A^p - A^l)
    
    # Observation term: correct own edges
    E = 0  (n×n matrix)
    for edge (i,j) incident to l:
        d_ij = ||p_i - p_j||  (measure current distance)
        t_ij_observed = exp(-d_ij² / σ²)
        E[i,j] = t_ij_observed - A^l[i,j]
        E[j,i] = t_ij_observed - A^l[j,i]
    
    # Update
    A^l = A^l + T_d * (ΔA_consensus + E)
    
    # Enforce constraints
    A^l = clip(A^l, 0, 1)  (valid range)
    A^l = (A^l + A^l.T) / 2  (symmetry)
    diag(A^l) = 0  (no self-loops)

Example (Round k=1):
    Robot 0 updates:
        Neighbors: {1, 2}
        Receives: A^1, A^2
        
        ΔA_consensus = 0.5 * (A^1 - A^0) + 0.5 * (A^2 - A^0)
        
        Row 0: [0, 0.95, 0.87] (already knows)
        Row 1: learns from A^1 → [0.95, 0, 0.92] (partially)
        Row 2: learns from A^2 → [0.87, 0.92, 0] (partially)
        
        A^0[k=1] = [[0,    0.95, 0.87],
                    [0.48, 0,    0.46],  ← partial info
                    [0.44, 0.46, 0   ]]  ← partial info

┌───────────────────────────────────────────────────────┐
│ STEP 3: Convergence Check (Every round)               │
└───────────────────────────────────────────────────────┘

max_change = max over all l: ||A^l(k) - A^l(k-1)||_∞

if max_change < ε_conv = 1e-4:
    CONVERGED = True
else:
    Continue consensus

Typical convergence: 20-50 iterations

┌───────────────────────────────────────────────────────┐
│ STEP 4: Redundant Edge Detection (After convergence)  │
└───────────────────────────────────────────────────────┘

For each robot l (independently):
    
    A_estimate = A^l  (use own estimate)
    
    # Extract edges
    edges = {(i,j) : A_estimate[i,j] > threshold}
    
    # Find redundant candidates
    candidates = []
    for (i,j) in edges:
        
        # Check 1: Alternative path?
        A_temp = A_estimate.copy()
        A_temp[i,j] = 0
        A_temp[j,i] = 0
        
        if not path_exists(i, j, A_temp):
            continue  # No alternative path
        
        # Check 2: Still connected?
        if not is_connected(A_temp):
            continue  # Graph disconnects
        
        # Check 3: Lambda2 sufficient?
        λ₂ = compute_lambda2(A_temp)
        if λ₂ < λ₂_threshold:
            continue  # Algebraic connectivity too low
        
        # All checks passed!
        candidates.append((i, j, A_estimate[i,j]))
    
    # Select weakest edge
    if candidates:
        candidates.sort(by strength)
        my_proposal = candidates[0]  # Weakest edge
    else:
        my_proposal = None

┌───────────────────────────────────────────────────────┐
│ STEP 5: Unanimous Voting (Final decision)             │
└───────────────────────────────────────────────────────┘

# All robots exchange proposals
proposals = {
    0: edge_0,
    1: edge_1,
    ...,
    n-1: edge_{n-1}
}

# Check unanimity
unique_proposals = set(proposals.values())

if len(unique_proposals) == 1:
    # UNANIMOUS!
    edge_to_prune = unique_proposals.pop()
    
    # Final safety check: topology still same?
    if current_edges() == edges_before_consensus:
        # PRUNE EDGE
        remove_edge(edge_to_prune)
        print(f"PRUNED {edge_to_prune}")
    else:
        # Topology changed during consensus
        print("ABORT: Topology changed")
else:
    # NOT UNANIMOUS
    print(f"DISAGREE: {len(unique_proposals)} different proposals")
    # Mark edge as non-redundant (blacklist)
```

---

### **8.3 Example Execution Timeline**

```
Time    Event                              Topology State          Consensus State
────────────────────────────────────────────────────────────────────────────────────
0.00    System starts                      8 edges, unstable       Inactive
0.05    Robots move toward goal            8 edges, unstable       Inactive
0.10    Edge (2,5) breaks                  7 edges, unstable       Inactive
0.15    Topology stable                    7 edges, stable (1/5)   Inactive
0.20    Topology stable                    7 edges, stable (2/5)   Inactive
0.25    Topology stable                    7 edges, stable (3/5)   Inactive
0.30    Topology stable                    7 edges, stable (4/5)   Inactive
0.35    Topology stable                    7 edges, stable (5/5)   Inactive
0.40    Stability achieved!                7 edges, stable         Check redundancy
0.45    Find redundant edge (0,3)          7 edges, stable         Start consensus
0.50    Consensus round 1                  7 edges, stable         Consensus (1/50)
0.55    Consensus round 2                  7 edges, stable         Consensus (2/50)
...
1.50    Consensus round 21                 7 edges, stable         Consensus (21/50)
1.55    CONVERGED! (max_change < 1e-4)     7 edges, stable         Voting phase
1.60    All robots propose (0,3)           7 edges, stable         Unanimous!
1.65    PRUNE edge (0,3)                   6 edges, unstable       Inactive (reset)
1.70    Topology stable                    6 edges, stable (1/5)   Inactive
...
```

---

## **9. Implementation Details**

### **9.1 Key Classes**

#### **ChainRobot**
```python
class ChainRobot:
    """Single robot with underwater dynamics."""
    
    Attributes:
        robot_id: int
        position: np.ndarray (2D)
        velocity: np.ndarray (2D)
        mass: float
        drag_coefficient: float
        cross_sectional_area: float
        parent: Optional[int]  # Parent in connectivity tree
        removed_edges: List[Edge]  # Pruned edges
    
    Methods:
        step(flow, dt, time, bounds, control_force)
            Updates position and velocity for one timestep
```

#### **FlowField**
```python
class FlowField:
    """Underwater current model."""
    
    Attributes:
        base_vector: np.ndarray  # Constant drift
        swirl_amplitude: float
        swirl_scale: float
    
    Methods:
        velocity(position, time) -> np.ndarray
            Computes flow velocity at given position and time
```

#### **HybridCLFCBFController**
```python
class HybridCLFCBFController:
    """Combined CLF-CBF controller."""
    
    Attributes:
        clf: CLFController
        cbf: CBFController
        goal_position: np.ndarray
        max_control: float
    
    Methods:
        compute_control(robot_id, robots) -> np.ndarray
            Computes total control force (CLF + CBF)
```

#### **AdjacencyMatrixConsensus**
```python
class AdjacencyMatrixConsensus:
    """Distributed consensus protocol (Griparic et al. 2022)."""
    
    Attributes:
        A_estimates: Dict[int, np.ndarray]  # Per-robot estimates
        num_robots: int
        sigma: float  # Trust sensitivity
        T_d: float  # Sample time
        epsilon_conv: float  # Convergence threshold
        consensus_converged: bool
    
    Methods:
        initialize_robot_knowledge(robot_id, neighbors, distances)
            Initialize robot's estimate with local observations
        
        consensus_update_step(robot_id, neighbors, observations) -> np.ndarray
            Execute one consensus update
        
        check_convergence() -> bool
            Check if all estimates converged
        
        get_adjacency_estimate(robot_id) -> np.ndarray
            Get robot's current estimate
```

#### **HybridPruningManager**
```python
class HybridPruningManager:
    """Orchestrates consensus-based pruning."""
    
    Attributes:
        adjacency_consensus: AdjacencyMatrixConsensus
        lambda2_manager: AdaptiveLambda2Manager
        edge_analyzer: EdgeAnalyzer
        consensus_active: bool
        consensus_candidate: Optional[Edge]
        consensus_round: int
        failed_candidates: Set[Edge]  # Blacklist
    
    Methods:
        find_redundant_edge_distributed(robot_id) -> Optional[Edge]
            Find redundant edge using robot's consensus estimate
        
        run_consensus_rounds(positions, edge_lengths, first_round) -> Optional[Edge]
            Execute consensus updates
        
        start_consensus(candidate: Edge)
            Begin consensus on candidate edge
        
        mark_candidate_failed() -> Edge
            Blacklist edge that failed consensus
```

#### **HybridUnderwaterSimulation**
```python
class HybridUnderwaterSimulation:
    """Main simulation engine."""
    
    Attributes:
        robots: List[ChainRobot]
        flow: FlowField
        controller: HybridCLFCBFController
        topology_manager: TopologyManager
        tree_builder: ConnectivityTree
        pruning_manager: HybridPruningManager
        time: float
    
    Methods:
        step() -> Dict
            Execute one simulation timestep (see Section 8.1)
        
        iterate(steps) -> Generator
            Run multiple steps
```

---

### **9.2 Configuration Parameters**

#### **Simulation Config**
```python
@dataclass
class SimulationConfig:
    num_robots: int = 8
    communication_radius: float = 2.5  # meters
    workspace_size: Tuple[float, float] = (20.0, 10.0)
    dt: float = 0.05  # seconds
    seed: Optional[int] = None
    goal_position: Optional[np.ndarray] = None
    verbose: bool = False
```

#### **Control Config**
```python
@dataclass
class ControlConfig:
    clf_gain: float = 0.3
    cbf_safety_gain: float = 1.0
    cbf_connectivity_gain: float = 0.8
    safety_distance: float = 0.3  # meters
    max_control_force: float = 1.5
```

#### **Consensus Config**
```python
@dataclass
class ConsensusConfig:
    # Topology stability
    stability_threshold: int = 5  # timesteps
    rolling_window_size: int = 10
    
    # Consensus parameters (Griparic et al. 2022)
    consensus_sigma: float = 1.0
    consensus_sample_time: float = 0.2  # T_d
    consensus_convergence_epsilon: float = 1e-4
    
    # Algebraic connectivity
    lambda2_threshold: float = 0.1
    lambda2_safety_margin: float = 0.05
    
    # Edge detection
    edge_quality_threshold: float = 0.01
    
    # Consensus rounds
    consensus_rounds_per_step: int = 3
    total_consensus_rounds_needed: int = 50
```

---

## **10. Parameters & Tuning**

### **10.1 Critical Parameters**

| Parameter | Symbol | Default | Impact | Tuning Guide |
|-----------|--------|---------|--------|--------------|
| **Communication radius** | $R_{comm}$ | 2.5 m | Graph density | Larger → more edges → slower pruning |
| **CLF gain** | $k_{CLF}$ | 0.3 | Goal-seeking speed | Larger → faster but less stable |
| **CBF safety gain** | $k_{safety}$ | 1.0 | Collision avoidance | Larger → stronger repulsion |
| **Safety distance** | $d_{safe}$ | 0.3 m | Minimum spacing | Smaller → denser formation |
| **Consensus sample time** | $T_d$ | 0.2 | Convergence speed | Larger → faster but less stable |
| **Consensus epsilon** | $\varepsilon_{conv}$ | $10^{-4}$ | Convergence precision | Smaller → more accurate |
| **Lambda2 threshold** | $\lambda_{2,min}$ | 0.1 | Connectivity strength | Larger → stricter pruning |
| **Trust sigma** | $\sigma$ | 1.0 | Link quality sensitivity | Smaller → more sensitive |
| **Stability threshold** | $T_{stable}$ | 5 steps | Topology steadiness | Larger → more conservative |

---

### **10.2 Trade-offs**

#### **CLF Gain ($k_{CLF}$)**

```
Low gain (0.1):
    ✓ Smooth trajectories
    ✓ Less oscillation
    ✗ Slow convergence to goal
    ✗ More time for pruning

High gain (0.5):
    ✓ Fast goal convergence
    ✗ Jerky motion
    ✗ Topology instability
    ✗ Frequent consensus aborts
```

#### **Communication Radius ($R_{comm}$)**

```
Small radius (1.5m):
    ✓ Sparse graph (fewer edges to prune)
    ✓ Fast consensus
    ✗ Frequent disconnections
    ✗ Weak robustness

Large radius (4.0m):
    ✓ Dense graph (high robustness)
    ✓ Rare disconnections
    ✗ Many redundant edges
    ✗ Slow pruning process
```

#### **Consensus Sample Time ($T_d$)**

```
Small T_d (0.05):
    ✓ Stable convergence
    ✓ Accurate estimates
    ✗ Slow convergence (more iterations)

Large T_d (0.5):
    ✓ Fast convergence (fewer iterations)
    ✗ Potential oscillations
    ✗ Numerical instability
```

#### **Lambda2 Threshold ($\lambda_{2,min}$)**

```
Low threshold (0.05):
    ✓ More aggressive pruning
    ✓ Sparser final graph
    ✗ Weaker connectivity
    ✗ Risk of disconnection

High threshold (0.2):
    ✓ Strong connectivity
    ✓ High robustness
    ✗ Conservative pruning
    ✗ Denser final graph
```

---

### **10.3 Recommended Settings by Scenario**

#### **Scenario 1: Fast Goal Convergence**
```python
SimulationConfig:
    num_robots = 8
    communication_radius = 3.0  # Larger for stability
    dt = 0.05

ControlConfig:
    clf_gain = 0.4  # Higher gain
    cbf_safety_gain = 1.2
    max_control_force = 2.0  # Allow higher thrust

ConsensusConfig:
    stability_threshold = 3  # Less strict
    consensus_sample_time = 0.3  # Faster consensus
```

#### **Scenario 2: Aggressive Pruning**
```python
SimulationConfig:
    communication_radius = 3.5  # Start with denser graph

ConsensusConfig:
    lambda2_threshold = 0.05  # Lower threshold
    edge_quality_threshold = 0.05  # Detect weaker edges
    stability_threshold = 7  # More conservative
```

#### **Scenario 3: High Robustness**
```python
ControlConfig:
    cbf_connectivity_gain = 1.2  # Stronger parent link

ConsensusConfig:
    lambda2_threshold = 0.15  # Higher minimum connectivity
    lambda2_safety_margin = 0.08  # Larger safety buffer
    stability_threshold = 10  # Very stable topology required
```

#### **Scenario 4: Large Swarm (n > 20)**
```python
SimulationConfig:
    num_robots = 30
    communication_radius = 2.0  # Smaller to limit graph size

ConsensusConfig:
    consensus_sample_time = 0.15  # Slower for stability
    total_consensus_rounds_needed = 100  # More rounds for larger n
    rolling_window_size = 15  # Longer history
```

---

### **10.4 Performance Metrics**

**Goal Achievement:**

$$
\text{Goal Error} = \frac{1}{n} \sum_{i=0}^{n-1} \|\mathbf{p}_i(T) - \mathbf{p}_{goal}\|
$$

**Connectivity Preservation:**

$$
\text{Connected Ratio} = \frac{\text{\# timesteps with } \lambda_2 > 0}{\text{Total timesteps}}
$$

**Pruning Efficiency:**

$$
\text{Edges Pruned} = |\mathcal{E}(0)| - |\mathcal{E}(T)|
$$

**Consensus Success Rate:**

$$
\text{Success Rate} = \frac{\text{\# successful prunings}}{\text{\# consensus attempts}}
$$

**Computation Time:**

$$
\text{Time per Step} = \text{Control} + \text{Dynamics} + \text{Consensus}
$$

Typical: ~5-10ms per step (Python), ~1-2ms (C++)

---

## **Summary**

This system integrates:

1. **Physical Layer**: Realistic underwater dynamics with flow fields and heterogeneous robots
2. **Control Layer**: CLF-CBF framework for safe goal-seeking and collision avoidance
3. **Coordination Layer**: Distributed consensus (Griparic et al. 2022) for topology optimization

**Key Innovations:**
- ✅ Fully distributed pruning (no global knowledge)
- ✅ Unanimous decision protocol
- ✅ Topology stability detection
- ✅ Multi-layer safety checks
- ✅ Real-time operation with dynamic topology

**Applications:**
- Underwater sensor networks
- Multi-AUV coordination
- Distributed environmental monitoring
- Resilient communication networks

---

**End of Documentation**
