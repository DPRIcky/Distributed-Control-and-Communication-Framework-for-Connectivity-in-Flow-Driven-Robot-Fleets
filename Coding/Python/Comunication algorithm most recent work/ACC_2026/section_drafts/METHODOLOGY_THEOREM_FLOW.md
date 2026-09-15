# Methodology Section Flow + Theorem Ordering (ACC 2026)

**Goal of this file:** Provide a paper-ready *flow* for Section III (Methodology), with short opening + bridge statements so the theorem dependencies are clean.

This structure matches the logical dependencies in the drafts:
- T1 is foundational (CBF invariance / feasibility).
- T2 depends on T1 (CLF–CBF QP feasibility + hard constraint satisfaction).
- T3 depends on T1 (maintaining a spanning backbone implies global connectivity).
- T4 depends on T3 (graph knowledge/agreement) and uses “quasi-static window” assumptions.

It also gives an option to merge T3 and T4.

---

## Section III: Methodology (recommended skeleton)

### III. Opening (pipeline overview, 4–6 sentences)

**Template (copy-ready):**
We propose a fully distributed framework that couples a graph layer with a control layer. The control layer enforces safety and preserves a selected set of communication links under spatially varying flow using control barrier functions (CBFs). The graph layer uses neighbor-to-neighbor message passing to identify a sparse backbone and prune redundant edges without centralized computation. Building on this evolving backbone, a CLF–CBF controller activates goal-seeking behavior only when targets are detected while maintaining safety and end-to-end connectivity. The following subsections formalize these components and state the resulting guarantees.

**Bridge to III.A:**
We first establish the key invariance guarantee: if certain CBF constraints are enforced, then safety and selected communication links remain satisfied for all time.

---

## III.A CBF layer: safety + link-maintenance (Theorem T1)

**Purpose:** Make T1 come *before* any theorem that uses it.

### III.A.0 Setup and modeling choice (what the theorem covers)

**Dynamics used for T1 (analysis model):** for each robot $i$,

$$\dot{x}_i(t)=f_{\mathrm{flow}}(x_i,t)+u_i(t)+d_i(t),\qquad x_i\in\mathbb{R}^2.$$

This is the reduced control-affine model used in the theorem draft; the flow is spatially varying so relative dynamics contain a non-canceling term $\Delta f_{ij}=f_{\mathrm{flow}}(x_i,t)-f_{\mathrm{flow}}(x_j,t)$.

**Assumptions referenced (match T1 draft A1–A4):**
- **A1:** $\|u_i(t)\|\le u_{\max}$.
- **A2:** $\|d_i(t)\|\le \bar d$.
- **A3:** flow is Lipschitz in space: $\|f_{\mathrm{flow}}(x_i,t)-f_{\mathrm{flow}}(x_j,t)\|\le L\|x_i-x_j\|$.
- **A4 (dominance / authority):** $u_{\max}>\bar d+LR_{\max}$.

**Local neighbor notation:**
- Proximity-based neighbors: $\mathcal{N}_i(t)=\{j:\|x_i-x_j\|\le R_{\max}\}$.
- “Must-keep” neighbors for connectivity: $\mathcal{C}_i\subseteq\mathcal{N}_i(0)$ (in the draft, $\mathcal{C}$ is the set of critical edges in the initial graph).

### III.A.1 Define the constraint sets and barrier functions

**Safety (collision avoidance):** enforce $\|x_i-x_j\|\ge d_{\min}$ for relevant pairs.

Barrier:
$$h_{ij}^{\mathrm{safe}}(x_i,x_j)=\|x_i-x_j\|^2-d_{\min}^2.$$

Safe set: $\mathcal{S}_{ij}^{\mathrm{safe}}=\{(x_i,x_j):h_{ij}^{\mathrm{safe}}\ge 0\}$.

**Connectivity (range maintenance on selected edges):** enforce $\|x_i-x_j\|\le R_{\max}$ for all $(i,j)\in\mathcal{C}$.

Barrier:
$$h_{ij}^{\mathrm{conn}}(x_i,x_j)=R_{\max}^2-\|x_i-x_j\|^2.$$

Safe set: $\mathcal{S}_{ij}^{\mathrm{conn}}=\{(x_i,x_j):h_{ij}^{\mathrm{conn}}\ge 0\}$.

### III.A.2 State the CBF inequalities (what each robot enforces)

Pick class-$\mathcal{K}$ functions (typically linear):
- $\alpha_s(s)=\gamma_s s$ for safety
- $\alpha_c(s)=\gamma_c s$ for connectivity

Then require, for each time $t$:

$$\dot h_{ij}^{\mathrm{safe}}+\alpha_s\big(h_{ij}^{\mathrm{safe}}\big)\ge 0,\quad \forall j\in\mathcal{N}_i(t)$$
$$\dot h_{ij}^{\mathrm{conn}}+\alpha_c\big(h_{ij}^{\mathrm{conn}}\big)\ge 0,\quad \forall j\in\mathcal{C}_i.$$

**Implementation note (paper clarity):** in the paper you can present these as constraints enforced by a local QP that minimally modifies a nominal input (e.g., exploration, formation keeping, CLF goal input), while respecting actuation bounds.

### III.A.3 Derivative calculations (the core technical step)

Let $z_{ij}=x_i-x_j$ and $r_{ij}=\|z_{ij}\|$.

**Safety barrier derivative:**
$$h_{ij}^{\mathrm{safe}}=\|z_{ij}\|^2-d_{\min}^2\quad\Rightarrow\quad \dot h_{ij}^{\mathrm{safe}}=2z_{ij}^\top(\dot x_i-\dot x_j).$$

Substitute dynamics:
$$\dot h_{ij}^{\mathrm{safe}}=2z_{ij}^\top\Big(\Delta f_{ij}+(u_i-u_j)+(d_i-d_j)\Big),\ \Delta f_{ij}:=f_{\mathrm{flow}}(x_i,t)-f_{\mathrm{flow}}(x_j,t).$$

**Connectivity barrier derivative:**
$$h_{ij}^{\mathrm{conn}}=R_{\max}^2-\|z_{ij}\|^2\quad\Rightarrow\quad \dot h_{ij}^{\mathrm{conn}}=-2z_{ij}^\top(\dot x_i-\dot x_j).$$

Same relative term appears, but with a sign flip, which is why the proofs are “identical up to sign.”

### III.A.4 Robust bounds for flow and disturbance (what replaces “flow cancels”)

This is the reviewer-critical step: spatially varying flow does not cancel, but it is bounded via Lipschitz continuity.

From **A3** (Lipschitz) and Cauchy–Schwarz,
$$\|\Delta f_{ij}\|\le L\|z_{ij}\|\ \Rightarrow\ z_{ij}^\top\Delta f_{ij}\ge -\|z_{ij}\|\,\|\Delta f_{ij}\|\ge -L\|z_{ij}\|^2.$$

From **A2** (bounded disturbances),
$$\|d_i-d_j\|\le \|d_i\|+\|d_j\|\le 2\bar d\ \Rightarrow\ z_{ij}^\top(d_i-d_j)\ge -2\bar d\,\|z_{ij}\|.$$

Plugging these into $\dot h$ yields an explicit *lower bound* on $\dot h_{ij}^{\mathrm{safe}}$ (and an analogous bound for $\dot h_{ij}^{\mathrm{conn}}$).

### III.A.5 Feasibility / why Assumption A4 appears

At the paper level, you do not need to reproduce the full “pick $u_i-u_j$ along $\hat r_{ij}$” construction; what matters is the narrative:

- The CBF constraints become affine constraints in $(u_i,u_j)$ once $x_i,x_j$ are fixed.
- The terms contributed by $\Delta f_{ij}$ and $(d_i-d_j)$ are bounded by **A2–A3**.
- **A4** ensures the admissible control set is large enough to counter worst-case relative drift over the maximum separation scale $R_{\max}$.

**One sentence you can use:** Assumption A4 is a standard “control authority dominates drift” condition ensuring that the robust CBF inequalities remain feasible despite worst-case flow gradients and disturbances.

### III.A.6 Theorem statement (paper-ready, aligned with T1 draft)

**Theorem T1 (Robust invariance of safety and link-maintenance):**
Consider $\dot x_i=f_{\mathrm{flow}}(x_i,t)+u_i+d_i$ under A1–A4. Define $h_{ij}^{\mathrm{safe}}$ for all neighbor pairs and $h_{ij}^{\mathrm{conn}}$ for all selected edges $(i,j)\in\mathcal{C}$. If the controls satisfy
$$\dot h_{ij}^{\mathrm{safe}}+\alpha_s(h_{ij}^{\mathrm{safe}})\ge 0,\ \forall (i,j)\in\mathcal{E}(t),\qquad \dot h_{ij}^{\mathrm{conn}}+\alpha_c(h_{ij}^{\mathrm{conn}})\ge 0,\ \forall (i,j)\in\mathcal{C},$$
then $h_{ij}^{\mathrm{safe}}(t)\ge 0$ and $h_{ij}^{\mathrm{conn}}(t)\ge 0$ for all $t\ge 0$ whenever they are satisfied initially.

### III.A.7 Proof blueprint (what to write in Section III vs Appendix)

**Recommended proof structure (matches the draft):**
1) Differentiate $\|x_i-x_j\|^2$ to get $\dot h$ in terms of $\Delta f_{ij}$, $u_i-u_j$, $d_i-d_j$.
2) Use A3 to bound the flow differential by $L\|x_i-x_j\|^2$.
3) Use A2 to bound the disturbance differential by $2\bar d\|x_i-x_j\|$.
4) Argue feasibility under A4 (either a short feasibility lemma or a one-paragraph bound).
5) Invoke the standard CBF forward-invariance result to conclude invariance.
6) Repeat the same argument for $h_{ij}^{\mathrm{conn}}$ (sign flip).

**What can be shortened in the main paper:** Steps (4)–(5) can be summarized with a short feasibility statement + citation to standard CBF invariance theorems; put the full inequality algebra in an appendix/supplement.

**Bridge statement into Theorem T1:**
The next theorem shows that, despite spatially varying flow and bounded disturbances, enforcing the CBF inequalities renders the safety and link-maintenance sets forward invariant.

**Bridge from T1 to III.B:**
With this invariance tool in place, we can safely talk about maintaining a *subset* of edges: once a backbone is identified, T1 guarantees those backbone edges remain in range.

---

## III.B Distributed backbone identification / agreement (Theorem T3)

**Purpose:** Show that the robots can agree on a common backbone edge set \(\mathcal{C}\) using only local communication.

**Content order (compact):**
1) State what information is exchanged (e.g., neighbor lists, or adjacency estimates \(\mathbf{A}^l(k)\)).
2) State the required “consensus window” assumption: graph remains connected and changes slowly during the iterations.
3) Define the extracted graph/edge set and the deterministic backbone selection rule.
4) State **Theorem T3** (distributed agreement on \(\mathcal{C}\) + enforcing T1 on \(\mathcal{C}\) implies global connectivity).

**Bridge from III.B to III.C:**
Once a backbone is available, we further reduce communication/control burden by pruning redundant edges, while ensuring that pruning never disconnects the network.

---

## III.C Distributed pruning/rewiring on redundant edges (Theorem T4)

**Purpose:** Guarantee your pruning algorithm (distance-first + guards + batching/recoup) does not disconnect the graph.

**Content order:**
1) Present the pruning rule and guards (alternative path, degree guard, next-step margin, critical guard).
2) Present the distributed execution details (batching / conflict avoidance / recoup).
3) State **Theorem T4** (connectivity preservation under pruning + finite termination + sparsity outcome; optionally MST-approx flavor if you keep it).

**Bridge from T4 to III.D:**
After the graph layer produces a sparse backbone, the control layer solves a local CLF–CBF program that activates goal-seeking only when needed, while always respecting the safety/connectivity invariants guaranteed by T1.

---

## III.D CLF–CBF controller for target engagement (Theorem T2)

**Purpose:** Show goal convergence (practical) while hard constraints remain satisfied.

**Content order:**
1) Define when the CLF is active (target detected / assigned) vs inactive (exploration).
2) Define the Lyapunov/CLF function \(V_i\) and the CLF inequality (soft via slack).
3) Write the per-robot CLF–CBF QP (or equivalent constrained minimization).
4) State and prove **Theorem T2** (QP feasibility + safety/connectivity from T1 + practical convergence).

**Closing sentence for Methodology (optional):**
Together, T1–T2 establish safe control under flow, while T3–T4 establish a distributed mechanism to identify and maintain a sparse, connectivity-preserving backbone.

---

# Option: merge Theorem 3 and 4 (recommended combined theorem)

If you want to reduce theorem count, you can merge T3 and T4 into a single result that reads more “systems” and less “modular,” while still keeping T1 and T2 separate.

## Combined theorem idea: “Distributed sparsification with guaranteed connectivity”

**Proposed combined statement (T3/4 merged):**
- Assumptions: (i) consensus/pruning window connectivity + quasi-static; (ii) unique IDs; (iii) consensus converges enough to yield a common extracted edge set; (iv) T1 conditions for enforcing link-maintenance on selected edges.
- Algorithmic components:
  1) distributed agreement on a common graph estimate / edge set;
  2) deterministic extraction of a backbone \(\mathcal{C}\);
  3) distance-first pruning of non-backbone edges with guards/batching/recoup.
- Conclusions:
  1) all robots agree on the same \(\mathcal{C}\) and the same prune decisions (unanimity);
  2) pruning steps never disconnect the graph (connectivity preserved throughout rounds);
  3) the maintained backbone edges remain within range for all time (by T1);
  4) therefore the physical communication graph remains connected for all time;
  5) the final graph is sparse / tree-like (with your chosen quantifier).

**Bridge text to introduce the merged theorem (copy-ready):**
We next combine the graph-agreement and pruning steps into a single guarantee: robots can distributively converge to a common sparse backbone and remove redundant edges without ever risking network fragmentation.

**Proof structure for merged theorem (1 paragraph roadmap):**
The proof proceeds by (i) showing edge-set agreement after consensus convergence; (ii) using determinism of the backbone/prune rules to conclude unanimous decisions; (iii) proving by induction that each pruning round preserves connectivity because only edges with valid alternative paths and guards are removed; and (iv) invoking Theorem T1 to ensure backbone edges remain in range, which implies the physical disk graph remains connected.

**Where to place this merged theorem in Section III:**
- Put it after introducing \(\mathcal{C}\subseteq\mathcal{E}\) and immediately before/after Algorithm 1. In practice:
  - Introduce agreement + define \(\mathcal{C}\)
  - State merged theorem
  - Present Algorithm 1
  - Provide proof sketch (full proof in appendix/supplement if needed)

---

## Minimal bridge sentences (quick pick list)

Use these as 1–2 liners to stitch subsections cleanly.

- **Intro → T1:** “We begin by establishing forward invariance of safety and selected communication constraints under spatially varying flow.”
- **After T1 → graph layer:** “Since we can guarantee link maintenance for any chosen edge set, we can now focus on how robots identify a sparse set to maintain using only local exchanges.”
- **T3 → T4:** “Having agreed on a common backbone, the next step is to prune redundant edges while ensuring no pruning action can disconnect the network.”
- **Graph layer → CLF–CBF:** “With a sparse backbone in place, we activate goal-seeking using a CLF, enforced through a CLF–CBF program that preserves the invariants already established.”
