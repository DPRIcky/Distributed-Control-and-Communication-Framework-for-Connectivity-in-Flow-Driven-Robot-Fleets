# THEOREM STATEMENTS + PROOF SKELETONS (T1–T4)
## ACC 2026 Theorem Package — Draft for Revision

---

## THEOREM 1: Robust Invariance for Connectivity and Safety

### Statement

Consider the multi-robot system with dynamics:
$$\dot{x}_i = f_{\text{flow}}(x_i, t) + u_i + d_i, \quad i \in V$$

subject to Assumptions A1–A4. Define the barrier functions:

**Safety barrier** (for all $j \in N(i)$):
$$h_{ij}^{\text{safe}} = \|x_i - x_j\|^2 - d_{\min}^2$$

**Connectivity barrier** (for all $j \in C_i$, where $C_i$ is the critical neighbor set):
$$h_{ij}^{\text{conn}} = R_{\max}^2 - \|x_i - x_j\|^2$$

If the control input $u_i$ at each time $t$ satisfies the CBF conditions:
$$\dot{h}_{ij}^{\text{safe}} + \alpha_s h_{ij}^{\text{safe}} \geq 0, \quad \forall j \in N(i)$$
$$\dot{h}_{ij}^{\text{conn}} + \alpha_c h_{ij}^{\text{conn}} \geq 0, \quad \forall j \in C_i$$

with class-$\mathcal{K}$ functions $\alpha_s, \alpha_c > 0$, then:

1. **Safety:** $h_{ij}^{\text{safe}}(0) \geq 0 \implies h_{ij}^{\text{safe}}(t) \geq 0, \, \forall t \geq 0$ (no collisions)
2. **Connectivity:** $h_{ij}^{\text{conn}}(0) \geq 0 \implies h_{ij}^{\text{conn}}(t) \geq 0, \, \forall t \geq 0$ (critical links preserved)

---

### Proof Outline (Bullet Points)

**Setup:** 
- Compute $\dot{h}_{ij}^{\text{safe}}$ using the relative dynamics identity from the unified model
- The key challenge is bounding the drift difference term $f_{\text{flow}}(x_i, t) - f_{\text{flow}}(x_j, t)$

**Step 1: Compute barrier derivative**
$$\dot{h}_{ij}^{\text{safe}} = \frac{d}{dt}\|x_i - x_j\|^2 - 0 = 2(x_i - x_j)^\top (\dot{x}_i - \dot{x}_j)$$
$$= 2(x_i - x_j)^\top \left[ (f_{\text{flow}}(x_i, t) - f_{\text{flow}}(x_j, t)) + (u_i - u_j) + (d_i - d_j) \right]$$

**Step 2: Bound the drift difference using Lipschitz assumption (A3)**
$$\|f_{\text{flow}}(x_i, t) - f_{\text{flow}}(x_j, t)\| \leq L\|x_i - x_j\|$$

By Cauchy-Schwarz:
$$(x_i - x_j)^\top (f_{\text{flow}}(x_i, t) - f_{\text{flow}}(x_j, t)) \geq -L\|x_i - x_j\|^2$$

**Step 3: Bound disturbances using A2**
$$(x_i - x_j)^\top (d_i - d_j) \geq -2\bar{d}\|x_i - x_j\|$$

**Step 4: Show feasibility of CBF constraint**

The CBF condition $\dot{h}_{ij}^{\text{safe}} + \alpha_s h_{ij}^{\text{safe}} \geq 0$ becomes:
$$2(x_i - x_j)^\top (u_i - u_j) \geq -2(x_i - x_j)^\top (f_{\text{flow}}(x_i, t) - f_{\text{flow}}(x_j, t)) - 2(x_i - x_j)^\top (d_i - d_j) - \alpha_s h_{ij}^{\text{safe}}$$

Using bounds from Steps 2–3:
$$2(x_i - x_j)^\top (u_i - u_j) \geq -2L\|x_i - x_j\|^2 - 4\bar{d}\|x_i - x_j\| - \alpha_s (\|x_i - x_j\|^2 - d_{\min}^2)$$

**Step 5: Verify sufficient control authority (A4)**

Under Assumption A4 ($u_{\max} > \bar{d} + LR_{\max}$), the right-hand side is achievable by choosing:
$$u_i - u_j = -\frac{(x_i - x_j)}{\|x_i - x_j\|} \cdot \text{(proportional gain)}$$

with gain magnitude $\leq 2u_{\max}$, ensuring the QP is feasible.

**Step 6: Invoke standard CBF invariance theorem**

From [Ames et al., 2019], if $\dot{h} + \alpha h \geq 0$ holds and $h(0) \geq 0$, then $h(t) \geq 0$ for all $t \geq 0$.

**Conclusion:** Both safety and connectivity sets are forward invariant. $\square$

---

**Key novelty vs. existing CBF work:** Most CBF connectivity papers assume *no external drift* or *constant drift that cancels*. Our contribution is showing that **Lipschitz spatially-varying flow** can be incorporated into the CBF framework by treating the flow gradient as a bounded disturbance-like term.

---

## THEOREM 2: CLF-Based Goal Convergence (Practical Stability)

### Statement

Consider robot $i$ with an assigned goal $x_i^{\text{goal}}$ and Lyapunov function:
$$V_i(x_i) = \|x_i - x_i^{\text{goal}}\|^2$$

Suppose the control $u_i$ is synthesized via the CLF-CBF QP:
$$\begin{aligned}
\min_{u_i, \gamma_i} \quad & \|u_i\|^2 + \lambda \gamma_i^2 \\
\text{s.t.} \quad & \dot{V}_i + \alpha V_i \leq \gamma_i \quad \text{(soft CLF)} \\
& \dot{h}_{ij}^{\text{safe}} + \alpha_s h_{ij}^{\text{safe}} \geq 0, \quad \forall j \in N(i) \quad \text{(hard CBF)} \\
& \dot{h}_{ij}^{\text{conn}} + \alpha_c h_{ij}^{\text{conn}} \geq 0, \quad \forall j \in C_i \quad \text{(hard CBF)} \\
& \|u_i\| \leq u_{\max}
\end{aligned}$$

with $\lambda \gg 1$ (large penalty on $\gamma_i$). Then:

1. **Feasibility:** The QP is feasible for all $t \geq 0$ (by Theorem 1, CBF constraints are satisfiable)
2. **Practical convergence:** Robot $i$ converges to a neighborhood of the goal:
$$\limsup_{t \to \infty} V_i(t) \leq \frac{\gamma^*}{\alpha}$$
where $\gamma^*$ is the steady-state relaxation variable value
3. **Exponential decay (outside residual set):** When $V_i$ is large, $\gamma_i \approx 0$ (due to high cost $\lambda$), and:
$$\dot{V}_i \leq -\alpha V_i \implies V_i(t) \leq V_i(0) e^{-\alpha t}$$

---

### Proof Outline (Bullet Points)

**Step 1: Compute CLF derivative**
$$\dot{V}_i = 2(x_i - x_i^{\text{goal}})^\top \dot{x}_i = 2(x_i - x_i^{\text{goal}})^\top (f_{\text{flow}}(x_i, t) + u_i + d_i)$$

**Step 2: Optimal control without constraints (pure CLF)**

If no CBF constraints were active, the optimal $u_i$ would be:
$$u_i^* = -\frac{\alpha}{2}(x_i - x_i^{\text{goal}}) - (f_{\text{flow}}(x_i, t) + d_i)$$
yielding $\dot{V}_i = -\alpha V_i$.

**Step 3: Effect of CBF constraints (projection onto feasible set)**

The QP solution $u_i^{\text{QP}}$ is the projection of $u_i^*$ onto the feasible region defined by CBF constraints. By convex optimization theory:
$$\|u_i^{\text{QP}}\|^2 \leq \|u_i^*\|^2$$
and the CLF relaxation $\gamma_i$ compensates for the mismatch.

**Step 4: Bound the relaxation variable**

The QP cost $\lambda \gamma_i^2$ penalizes large $\gamma_i$. In steady state near the goal, disturbances and flow are bounded:
$$|\gamma_i| \leq C_{\text{dist}} := 2\sqrt{V_i}(\|f_{\text{flow}}\| + \bar{d})$$

As $V_i \to 0$, $\gamma_i \to 0$.

**Step 5: Apply comparison lemma**

From $\dot{V}_i \leq -\alpha V_i + \gamma_i$ and $\gamma_i \leq C_{\text{dist}} \sqrt{V_i}$, we get:
$$\dot{V}_i \leq -\alpha V_i + C \sqrt{V_i}$$

By comparison principle, $V_i(t)$ converges to a residual ball of radius $O(C^2/\alpha^2)$.

**Conclusion:** Practical stability to goal neighborhood is guaranteed, with exponential convergence when far from goal. $\square$

---

**Key novelty:** Explicitly separates **hard CBF (connectivity/safety)** from **soft CLF (goal)**, resolving Reviewer #2's question "where is Lyapunov used?" Answer: CLF is the *objective* with relaxation; CBF is the *hard constraint*.

---

## THEOREM 3: Global Connectivity via Spanning Backbone Maintenance

### Statement

Let $C \subseteq E$ be the set of critical edges identified via the distributed procedure in [Venkateswaran et al., 2024]. Define the pruned graph:
$$G' = (V, E'), \quad E' \supseteq C$$

Suppose:
1. The initial graph $G_0$ is connected (Assumption A5)
2. Algorithm 1 ensures $C \subseteq E'(t)$ for all $t \geq 0$
3. Theorem 1 ensures all $(i,j) \in C$ satisfy $\|x_i(t) - x_j(t)\| \leq R_{\max}$ for all $t \geq 0$

Then the full communication graph $G_t = (V, E_t)$ (where $E_t = \{(i,j) : \|x_i - x_j\| \leq R_{\max}\}$) remains connected for all $t \geq 0$.

---

### Proof Outline (Bullet Points)

**Step 1: Critical edge characterization (Venkateswaran Lemma 1)**

By [24, Lemma 1], an edge $(i,j)$ is critical if and only if removing it disconnects the graph. Equivalently:
- The set $C$ forms a **spanning subgraph** of $G_0$
- $C$ is connected (removing any edge from $C$ disconnects the network)

**Step 2: Pruning preserves critical edges**

Algorithm 1 only removes edges from $E^- = E \setminus C$ (non-critical edges). By definition, removing any edge in $E^-$ leaves the graph connected.

**Step 3: Maintained critical edges stay within range**

From Theorem 1, the connectivity CBF ensures:
$$h_{ij}^{\text{conn}} = R_{\max}^2 - \|x_i - x_j\|^2 \geq 0, \quad \forall (i,j) \in C$$

Thus, all critical edges remain in $E_t$ (communication range).

**Step 4: Spanning subgraph connectivity implies full graph connectivity**

Graph theory fundamental: If $G'$ is a connected spanning subgraph of $G$, and $E' \subseteq E$, then $G$ is also connected.

Since $C \subseteq E'(t) \subseteq E_t$ and $C$ spans $V$:
$$G_t \text{ is connected} \iff G' = (V, C) \text{ is connected}$$

**Step 5: Induction over time**

- Base case ($t=0$): $G_0$ connected by A5
- Inductive step: If $G_t$ connected at $t$, then $C$ remains connected, and pruning/rewiring (Alg. 1) only affects non-critical edges → $G_{t+\Delta t}$ connected

**Conclusion:** Global connectivity is preserved for all $t \geq 0$. $\square$

---

**Key novelty:** Combines *distributed critical-edge detection* (graph theory) with *CBF-based range maintenance* (control), avoiding centralized MST construction while guaranteeing connectivity.

---

## THEOREM 4: Pruning Correctness (No False Disconnections)

### Statement

Algorithm 1 (Distance-First Pruning with Guards) never disconnects the communication graph. That is, if $G = (V, E)$ is connected before applying the algorithm, then the output graph $G' = (V, E')$ is also connected.

---

### Proof Outline (Bullet Points)

**Step 1: Alternative path guarantee**

For each edge $(i,j) \in E_{\text{batch}}$ (selected for pruning), the algorithm verifies:
- There exists a path $i \rightsquigarrow j$ of at most $H$ hops (default $H=2$)
- All edges in this path have length $< l_{ij}$

Thus, removing $(i,j)$ does **not** disconnect $i$ and $j$.

**Step 2: Degree guard prevents isolated nodes**

Before removing $(i,j)$, the algorithm checks:
$$\min\{\deg(i), \deg(j)\} \geq 2$$

This ensures that both endpoints still have at least one other neighbor after removal.

**Step 3: Vertex-disjoint batching (conflict-free)**

Edges in $E_{\text{batch}}$ are selected such that no two edges share a vertex. This prevents:
- Cascading failures (e.g., removing both $(i,j)$ and $(i,k)$ simultaneously could isolate $i$)
- Cyclic dependencies in alternative path validation

**Step 4: Next-step guard (range margin)**

All edges on the alternative path satisfy $l \leq R_{\max} - \epsilon$. This ensures that small drift does not immediately break the substitute path.

**Step 5: Recoup step (safety net)**

If any node's degree becomes $0$ after pruning (due to unexpected edge failures), Algorithm 1 restores its most recently removed edge.

**Step 6: Induction over batches**

- Base case: $G$ is connected before the first batch
- Inductive step: If $G$ is connected before batch $k$, then:
  - Each edge in batch $k$ has an alternative path (Step 1)
  - No node is isolated (Steps 2, 5)
  - $G$ remains connected after batch $k$

**Conclusion:** Algorithm 1 produces a connected graph $G'$ for all valid inputs. $\square$

---

**Key novelty:** Combines *distributed alternative-path checking* with *conflict-free batching*, achieving near-MST performance without centralized coordination.

---

## SUMMARY TABLE: THEOREM DEPENDENCIES

| Theorem | Depends On | Provides To |
|---------|-----------|-------------|
| **T1** | Assumptions A1–A4 | Foundation for T2, T3 (ensures CBF feasibility) |
| **T2** | T1 (CBF feasibility) | Goal convergence guarantee |
| **T3** | T1 (connectivity CBF), A5 (initial connectivity), [Venkateswaran Lemma 1] | Global connectivity proof |
| **T4** | Algorithm 1 guards | Correctness of pruning → feeds into T3 |

---

## NEXT STEPS (FOR FULL PAPER)

1. **Full proofs:** Expand bullet outlines into rigorous proofs (target: 1–2 pages per theorem in appendix)
2. **Lemma extraction:** Identify reusable sub-results (e.g., "Lipschitz flow → bounded relative drift")
3. **Simulation validation:** For each theorem, add 1–2 sentences in Section IV linking simulation results to theorem claims
4. **Cross-references:** Ensure every theorem is cited in the main text and matched to a figure

---

**End of Deliverable #3**
