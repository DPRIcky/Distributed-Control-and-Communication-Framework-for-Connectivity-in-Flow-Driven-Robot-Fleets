# Assumptions for Distributed Concurrent Consensus and Topology Pruning

**Concise documentation of key assumptions underlying the proposed method**

---

## 1. Network and Communication Assumptions

### A1.1: Graph Structure
- **Undirected communication graph**: $(i,j) \in \mathcal{E} \Leftrightarrow (j,i) \in \mathcal{E}$
- **Initial connectivity**: The initial graph $\mathcal{G}(0)$ is connected ($\lambda_2(\mathcal{L}(0)) > 0$)
- **No self-loops**: $A_{ii} = 0$ for all robots $i$

### A1.2: Communication Model
- **1-hop communication**: Each robot can exchange information with direct neighbors
- **Bilateral message passing**: Both endpoints of an edge can communicate for pruning negotiation
- **Ideal communication**: Messages are reliably delivered within the iteration (no delays/drops explicitly modeled)*

*Note: The focus is on the distributed decision-making logic; implementation with communication protocols (e.g., ROS, acoustic modems) is deferred to deployment.

### A1.3: Local Information Access
- **Local adjacency estimate**: Robot $\ell$ maintains $A^\ell(k)$
- **Neighbor awareness**: Robot $\ell$ knows its 1-hop neighbor set $\mathcal{N}_\ell(k)$
- **No global knowledge**: Robots do not have access to the full network state

---

## 2. Consensus Protocol Assumptions

### A2.1: Adjacency Matrix Consensus
- **Protocol**: Griparic et al. (2022) distributed adjacency matrix consensus
- **Convergence rate**: Exponential with rate $\alpha \in (0, 1)$ under fixed topology
- **Trust model**: Edge weights are distance-based: $w_{ij} = \exp(-d_{ij}^2 / \sigma^2)$

### A2.2: Convergence Under Pruning
- **Assumption**: Consensus continues to converge even as edges are removed, provided:
  - Graph remains connected ($\lambda_2 \geq \lambda_{\min}$)
  - Pruning rate is controlled by adaptive thresholds
  - Lyapunov constraints are satisfied ($V_\ell(k+1) < V_\ell(k) - \varepsilon(k)$)

---

## 3. Pruning Safety Assumptions

### A3.1: Connectivity Preservation
- **Lower bound**: $\lambda_2(\mathcal{L}(k)) \geq \lambda_{\min} + \mu(k)$
  - $\lambda_{\min}$: Minimum acceptable algebraic connectivity
  - $\mu(k)$: Adaptive safety margin (conservative → aggressive)
  
### A3.2: Redundancy Detection
- **Alternative paths**: Edge $(i,j)$ is redundant if an alternative path exists after removal
- **Local check**: Each robot verifies redundancy using local graph information

### A3.3: Lyapunov Stability
- **Local Lyapunov**: $V_\ell(k) = \sum_{p \in \mathcal{N}_\ell} \|A^\ell(k) - A^p(k)\|_F^2$
- **Monotonic decrease**: Pruning allowed only if $V_\ell(k+1) < V_\ell(k) - \varepsilon(k)$
- **Threshold scheduling**: $\varepsilon(k)$ adapts from conservative to aggressive

---

## 4. Distributed Coordination Assumptions

### A4.1: Pruning Decision Protocol
- **Phase 1 (Proposal)**: Each robot independently evaluates incident edges using local data
- **Phase 2 (Negotiation)**: Edge $(i,j)$ requires unanimous consent (both $i$ and $j$ propose it)
- **Phase 3 (Conflict Resolution)**: Deterministic tie-breaking rule applied consistently by all robots
  - Priority: Longest edge → Lowest ID sum → Lowest ID

### A4.2: Deterministic Execution
- **Assumption**: All robots execute the same algorithm with identical tie-breaking rules
- **Implication**: Given the same local information, all robots reach the same pruning decision
- **No randomization**: The protocol is fully deterministic (no probabilistic elements)

---

## 5. Robot Dynamics and Control Assumptions

### A5.1: Motion Model
- **Double integrator dynamics**: $\ddot{x}_i = u_i$ (position control)
- **Control synthesis**: Hybrid CLF-CBF quadratic program (QP) controller
- **Bounded control**: $\|u_i\| \leq u_{\max}$ (actuator limits)

### A5.2: Sensing and Localization
- **Relative positioning**: Each robot can measure relative positions of neighbors within communication range
- **Localization**: Robots know their own positions (onboard sensors/global reference)

---

## 6. Scalability and Complexity Assumptions

### A6.1: Computational Complexity
- **Per-robot computation**: $O(|\mathcal{N}_\ell| \cdot n^2)$ per iteration
  - Adjacency consensus update: $O(n^2)$
  - Local Lyapunov: $O(|\mathcal{N}_\ell| \cdot n^2)$
- **Assumption**: Robots have sufficient onboard computation (typical for modern autonomous systems)

### A6.2: Communication Complexity
- **Per-robot per-iteration**: $O(|\mathcal{N}_\ell| \cdot n^2)$ (exchange $n \times n$ matrices)
- **Pruning negotiation**: $O(|\mathcal{N}_\ell|)$ (local coordination messages)

---

## 7. Objective and Optimality Assumptions

### A7.1: Optimization Goal
- **Minimize edges** while maintaining:
  - Connectivity: $\lambda_2 \geq \lambda_{\min}$
  - Consensus convergence: exponential rate preserved
  - Control objective: robots reach formation/waypoints

### A7.2: Optimality
- **Local optimality**: The method converges to a locally optimal sparse topology
- **No global optimality claim**: Finding the globally optimal spanning tree is NP-hard in general
- **Practical performance**: Empirically achieves near-optimal sparsity (close to minimal spanning tree)

---

## 8. Environmental and Application Context

### A8.1: Target Application
- **Multi-robot coordination** (e.g., underwater swarms, aerial fleets)
- **Communication-constrained environments** (limited bandwidth, packet loss)
- **Dynamic scenarios**: Robots in motion while consensus and pruning occur concurrently

### A8.2: Time Scales
- **Consensus time scale**: Faster than robot motion (typical: 10-50 iterations)
- **Pruning time scale**: Adaptive thresholds allow faster pruning as consensus progresses
- **Control update rate**: Fast enough to track topology changes (e.g., 10-100 Hz)

---

## 9. Practical Implementation Considerations

### A9.1: Initialization
- **Initial topology**: Typically proximity-based (geometric graph) or fully connected
- **Initial consensus**: Each robot starts with local sensor measurements $A^\ell(0)$

### A9.2: Real-World Deployment
- **Communication delays**: Not explicitly modeled; robustness to delays is an open question
- **Packet loss**: Not explicitly handled; conservative thresholds provide some robustness
- **Measurement noise**: Adjacency weights are noisy; consensus protocol filters noise over time

---

## 10. Key Simplifications and Limitations

### S10.1: What is NOT Assumed
- ❌ **Central coordinator**: The method is fully distributed
- ❌ **Global clock**: Robots operate asynchronously (iteration-based synchronization assumed for analysis)
- ❌ **Perfect communication**: Practical implementation requires handling delays/losses
- ❌ **Static topology**: The method handles dynamic topologies (robots in motion)

### S10.2: Known Limitations
1. **Communication overhead**: $O(n^2)$ matrix exchange per neighbor per iteration
2. **Convergence time**: Depends on network size and initial connectivity
3. **Message passing idealization**: Real-world implementation requires robust communication layer

---

## References

**Adjacency Matrix Consensus:**
- Griparic et al. (2022), "Distributed Adjacency Matrix Estimation for Multi-Robot Systems"

**Lyapunov Stability:**
- Boyd et al. (2011), "Distributed Optimization and Statistical Learning via ADMM"

**CLF-CBF Control:**
- Ames et al. (2019), "Control Barrier Functions: Theory and Applications"

---

**Document Status:** Draft for ACC 2026 submission  
**Last Updated:** February 19, 2026
