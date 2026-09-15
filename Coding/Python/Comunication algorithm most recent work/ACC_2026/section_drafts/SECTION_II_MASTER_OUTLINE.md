# Section II: Problem Formulation - MASTER OUTLINE
## Para-by-Para Writing Guide

---

## OVERALL STRUCTURE

```
Section II: Background and Problem Formulation
├── [Opening: 1-2 sentence transition from Section I]
├── II.A: Dynamics
│   ├── II.A.1: Advection model
│   └── II.A.2: Control-affine model
├── II.B: Assumptions
├── II.C: Graph Notation and Constraint Sets
└── II.D: Problem Statement
```

---

## LOGICAL FLOW: WHAT COMES AFTER WHAT

### The Story Arc:
1. **Physical reality** → How robots naturally behave in water (advection-diffusion)
2. **Control model** → Simplified version for designing controllers (control-affine)
3. **Assumptions** → What we assume about the system (A1-A6)
4. **Definitions** → Precise mathematical notation (graphs, sets)
5. **Problem** → What we want to achieve (objectives + constraints)

---

## 📋 SECTION-BY-SECTION OUTLINE

---

## **OPENING (Optional, 1-2 sentences)**

**Purpose:** Transition from Section I to formal math

**What to write:**
- [ ] Brief statement that this section formalizes the problem
- [ ] NO literature review, NO motivation (that's in Section I)

**Template:**
> "We now formalize the problem. This section introduces the robot dynamics model, states the assumptions, defines the graph-theoretic notation, and presents the formal problem statement."

**Length:** 1-2 sentences MAX

---

## **II.A: DYNAMICS**

**Purpose:** State the motion model in a way that (i) matches the physical intuition and simulation, (ii) yields a tractable model for control design, and (iii) explicitly resolves the reviewer concerns about “two models” and “flow cancellation.”

### Copy-ready Dynamics text (paragraphs + equations)

#### II.A.1 Advection model

We model a fleet of $N$ robots operating in $\mathbb{R}^n$ (with $n=2$ used for visualization in our simulations). Physically, each robot experiences (i) advection due to a spatially/time-varying ambient flow and (ii) dispersion due to unresolved turbulence and small perturbations. Let $x_i(t)\in\mathbb{R}^n$ denote the position of robot $i$ and define the fleet center of mass
$$
\bar x(t)=\frac{1}{N}\sum_{i=1}^N x_i(t).
$$

**Simulation-accurate advection/dispersion with velocity damping.** The dynamics implemented in the simulation can be written in a damped-velocity form:
$$
dx_i(t)=\big(f_{\text{flow}}(x_i(t),t)+v_i(t)\big)dt+\sigma_{\text{diff}}\,dW_i(t),
$$
$$
\dot v_i(t)=u_i(t)-\gamma v_i(t),
$$
where $v_i(t)\in\mathbb{R}^n$ is the robot’s self-propelled velocity, $u_i(t)\in\mathbb{R}^n$ is the thrust (acceleration) input, $\gamma>0$ is a velocity damping coefficient, and $W_i(t)$ is a Wiener process capturing diffusion/turbulence (with intensity $\sigma_{\text{diff}}$).

**Advection-diffusion interpretation (group drift + spread).** Averaging the position dynamics gives
$$
d\bar x(t)=\left(\frac{1}{N}\sum_{i=1}^N f_{\text{flow}}(x_i(t),t)+\frac{1}{N}\sum_{i=1}^N v_i(t)\right)dt+\frac{\sigma_{\text{diff}}}{N}\sum_{i=1}^N dW_i(t).
$$
When the fleet remains spatially localized and the self-propelled velocities approximately balance (e.g., the formation has small net bias), this reduces to the familiar drift approximation
$$
\dot{\bar x}(t)\approx f_{\text{flow}}(\bar x(t),t),
$$
while individual robots disperse stochastically around the drifting center via the diffusion term.

**Flow model used in simulation (example).** In our current implementation (for $n=2$), the flow field is a base current plus a spatially varying swirl:
$$
f_{\text{flow}}(x,t)=v_{\text{base}}(t)+A_{\text{swirl}}
\begin{bmatrix}
\sin\!\left(\frac{x_2+t}{s_{\text{swirl}}}\right)\\
\cos\!\left(\frac{x_1+0.3t}{s_{\text{swirl}}}\right)
\end{bmatrix}.
$$

**Critical property (spatial variation).** For two robots $i$ and $j$, the flow difference is generally nonzero when $x_i\neq x_j$:
$$
f_{\text{flow}}(x_i,t)-f_{\text{flow}}(x_j,t)=A_{\text{swirl}}
\begin{bmatrix}
\sin\!\left(\frac{x_{i,2}+t}{s_{\text{swirl}}}\right)-\sin\!\left(\frac{x_{j,2}+t}{s_{\text{swirl}}}\right)\\
\cos\!\left(\frac{x_{i,1}+0.3t}{s_{\text{swirl}}}\right)-\cos\!\left(\frac{x_{j,1}+0.3t}{s_{\text{swirl}}}\right)
\end{bmatrix}\neq 0.
$$
Moreover, this flow is globally Lipschitz in $x$ with a constant on the order of $L_f\approx A_{\text{swirl}}/s_{\text{swirl}}$, which motivates Assumption A3.

**Bridge sentence (end of II.A.1):** For controller synthesis and theoretical analysis, we now abstract the above simulation dynamics into a compact control-affine model on $x_i$.

#### II.A.2 Control-affine model

The full simulation model above is second-order in the sense that $u_i$ acts through the damped velocity $v_i$. For analysis, we use a quasi-steady (single-integrator) approximation in which the velocity dynamics equilibrate rapidly relative to the position dynamics (e.g., $\gamma$ is sufficiently large over the time scales of interest). In this reduced model, we treat the effective self-propelled velocity as the control variable and absorb diffusion, residual velocity transients, and unmodeled effects into a bounded disturbance:
$$
\dot x_i(t)=f_{\text{flow}}(x_i(t),t)+u_i(t)+d_i(t),\qquad \|d_i(t)\|\le \bar d.
$$
Here $u_i(t)$ denotes the effective control velocity used for controller design (not the acceleration input of the full simulation model).

**Reviewer clarification (addresses “two models” and “flow cancels”):** We present two dynamics descriptions because they serve different purposes: the advection/dispersion model supports physical interpretation and simulation realism, while the control-affine model is used for controller design and theoretical analysis in Section III. Importantly, the flow term is evaluated at each robot’s position, so it is not a common-mode input that is identical for all robots. In relative coordinates $z_{ij}=x_i-x_j$,
$$
\dot z_{ij} = \big(f_{\text{flow}}(x_i,t)-f_{\text{flow}}(x_j,t)\big) + (u_i-u_j) + (d_i-d_j),
$$
so the flow contribution cancels only in the special case of spatially uniform flow. In our setting, spatial variation of $f_{\text{flow}}$ is explicitly bounded via a global Lipschitz constant (Assumption A1 in II.B), which is the quantity that enters the connectivity and invariance analysis.

**Relative-dynamics identity (used by barrier constraints).** The time derivative of the squared inter-robot distance can be written as
$$
\frac{d}{dt}\|x_i-x_j\|^2=2(x_i-x_j)^\top\Big(\big(f_{\text{flow}}(x_i,t)-f_{\text{flow}}(x_j,t)\big)+(u_i-u_j)+(d_i-d_j)\Big).
$$
This expression makes explicit that the flow contributes through the *difference term* $f_{\text{flow}}(x_i,t)-f_{\text{flow}}(x_j,t)$, which is bounded via Assumption A3 and appears directly in the safety/connectivity invariance conditions.

Finally, the choice of a two-dimensional environment is made solely for ease of visualization. The modeling and control formulation are dimension-agnostic and extend directly to three-dimensional settings.

### Optional: Remark 1 (if you want a dedicated box)
- **Placement:** Immediately after II.A.2 and before II.B.
- **Must say (2–4 sentences):** “We use the advection/dispersion description for physical interpretation and simulation fidelity; we use the control-affine model for control synthesis and theoretical analysis. The models are consistent because the latter is an abstraction that lumps drag/turbulence into a bounded disturbance while retaining position-dependent flow.”

---

## **II.B: ASSUMPTIONS**

**Purpose:** State the assumptions used in Theorems T1–T4 (safety/connectivity invariance, CLF goal convergence, distributed consensus, pruning correctness). To keep Section II compact and avoid repetition, the bounded-disturbance and Lipschitz-flow conditions are stated once in II.A and only referenced here.

### Copy-ready Assumptions text (compact, minimal numbering)

**A1 (Model regularity and bounded inputs).** The reduced (single-integrator) model uses the effective control velocity $u_i(t)$ and a bounded disturbance $d_i(t)$, and the ambient flow is spatially Lipschitz:
- $\|u_i(t)\|\le u_{\max}$ for all $i$ and $t$.
- $\|d_i(t)\|\le \bar d$ for all $i$ and $t$ (lumping diffusion, transient velocity dynamics, and uncertainties from the full simulation model).
- $\|f_{\text{flow}}(x_i,t)-f_{\text{flow}}(x_j,t)\|\le L_f\|x_i-x_j\|$ for all $x_i,x_j$ and $t$.
These bounds are introduced in II.A alongside the unified dynamics.

**A2 (Feasibility / control dominance).** Control authority dominates worst-case drift differential over the communication range and worst-case disturbance:
$$
u_{\max} > \bar d + L_f R_{\max}.
$$

**A3 (Safety–connectivity margin).** The minimum separation distance is strictly smaller than the communication range (with buffer):
$$
d_{\min} < R_{\max} - \varepsilon
$$
for some margin $\varepsilon>0$.

**A4 (Connectivity + identifiers).** The initial communication graph is connected, and each robot has a unique persistent identifier used to index neighbor messages and distributed adjacency/graph estimates consistently.

**Communication model (disk graph + local information).** At time $t$, an undirected edge $(i,j)$ exists if and only if $\|x_i(t)-x_j(t)\|\le R_{\max}$. Robots communicate only with current neighbors and use only neighbor-to-neighbor message passing (no centralized coordinator, no global state).

**Note (only needed for T3–T4).** During distributed consensus/pruning phases, we assume a standard time-scale separation: the graph remains connected long enough for information to propagate and the consensus iterations converge before the physical graph changes significantly.

---

## **II.C: GRAPH NOTATION AND CONSTRAINT SETS**

**Purpose:** Define all mathematical objects used in theorems

### Structure: 4 Definition Blocks

#### Block 1: Communication Graph
**What to write:**
- [ ] Define $\mathcal{G}_t = (\mathcal{V}, \mathcal{E}_t)$
- [ ] Define vertices $\mathcal{V} = \{1, ..., N\}$
- [ ] Define edges $\mathcal{E}_t = \{(i,j) : \|x_i - x_j\| \leq R_{\max}\}$
- [ ] Define degree $\deg(i,t) = |\mathcal{N}(i,t)|$
- [ ] Define connectivity: $\lambda_2(\mathcal{L}_t) > 0$

#### Block 2: Critical Edge Set
**What to write:**
- [ ] Define critical edge: removal disconnects graph
- [ ] Denote $\mathcal{C}_t \subseteq \mathcal{E}_t$ (critical edges)
- [ ] Define non-critical: $\mathcal{E}_t^- = \mathcal{E}_t \setminus \mathcal{C}_t$
- [ ] Explain: Critical edges = bridges, must keep

#### Block 3: Safe Set (Collision Avoidance)
**What to write:**
- [ ] Define pairwise: $\mathcal{C}_{\text{safe}}^{ij} = \{(x_i, x_j) : \|x_i - x_j\|^2 \geq d_{\min}^2\}$
- [ ] Define overall: $\mathcal{C}_{\text{safe}} = \bigcap_{i \neq j} \mathcal{C}_{\text{safe}}^{ij}$
- [ ] Explain: All robots maintain minimum separation

#### Block 4: Connectivity Set (Communication Maintenance)
**What to write:**
- [ ] Define pairwise: $\mathcal{C}_{\text{conn}}^{ij} = \{(x_i, x_j) : \|x_i - x_j\|^2 \leq R_{\max}^2\}$
- [ ] Define for critical pairs: $\mathcal{C}_{\text{conn}} = \bigcap_{(i,j) \in \mathcal{C}_t} \mathcal{C}_{\text{conn}}^{ij}$
- [ ] Explain: Critical neighbors stay in range

#### Block 5: Feasible Region
**What to write:**
- [ ] Define: $\mathcal{C}_{\text{feasible}} = \mathcal{C}_{\text{safe}} \cap \mathcal{C}_{\text{conn}}$
- [ ] Explain: Safe AND connected (the "sweet spot")
- [ ] Reference A6: ensures this set is non-empty

**See:** `04_Graph_and_Constraint_Definitions.md` for full LaTeX

---

## **II.D: PROBLEM STATEMENT**

**Purpose:** Formal statement of what we want to achieve

### Structure: Given-Design-Such That

#### Part 1: GIVEN (what we have)
**What to write:**
- [ ] $N$ robots with dynamics $\dot{x}_i = f_{\text{flow}}(x_i,t) + u_i + d_i$
- [ ] Subject to Assumptions A1–A4 (reference by number)
- [ ] Initially connected graph $\mathcal{G}_0$
- [ ] Range-limited communication
- [ ] Local information only (enumerate what robots know)
- [ ] Task: targets appear randomly, must be attended

**Format:** Bullet list

#### Part 2: DESIGN (what we create)
**What to write:**
- [ ] A distributed control framework with TWO layers:
  1. **Graph layer:** Identifies critical structure $\mathcal{C}_t$
  2. **Control layer:** Computes $u_i$ using only local info

**Format:** Short numbered list

#### Part 3: SUCH THAT (objectives)
**What to write:**
- [ ] **Safety:** $\|x_i - x_j\| \geq d_{\min}$ for all $i,j,t$
- [ ] **Connectivity:** $\lambda_2(\mathcal{L}_t) > 0$ for all $t$
- [ ] **Energy Efficiency:**
  - Exploit flow advection
  - Maintain only critical links
  - Activate control only when needed
- [ ] **Reconfiguration:** 
  - Detecting robot → reaches target
  - Others → maintain relay network
- [ ] **Distributed:** 
  - All decisions local
  - No global info

**Format:** Numbered objectives with sub-bullets

#### Part 4: Summary statement
**What to write:**
- [ ] One sentence capturing the core challenge
- [ ] Reference forward to Section III (solution) and Section IV (validation)

**See:** `05_Problem_Statement_Revised.md` for full LaTeX

---

## 📊 COMPLETE PARA-BY-PARA CHECKLIST

### Order of Writing:

1. ✅ **Keep existing II.A paragraphs 1-2** (advection-diffusion, already good)
2. ✏️ **Add II.A paragraph 3** (control-affine model with $d_i$, flow depends on $x_i$)
3. ✏️ **Optional: Add Remark 1 box** (two-model explanation)
4. ✏️ **Rewrite II.B** → Assumptions A1-A6 blocks (remove literature)
5. ✏️ **Add communication model paragraph** (disk model, at end of II.B)
6. ✏️ **NEW subsection II.C** → Graph definitions (4-5 blocks)
7. ✏️ **Rewrite II.D** → Problem statement (Given-Design-Such That)

---

## 🎯 KEY PRINCIPLES FOR WRITING

### DO:
✅ Use precise mathematical notation
✅ Reference assumption numbers (A1, A2, etc.)
✅ Use LaTeX equation environments
✅ Cross-reference forward to theorems ("proven in Theorem 1")
✅ Keep paragraphs focused (one idea per paragraph)

### DON'T:
❌ Repeat literature review (that's in Section I)
❌ Explain WHY the problem is important (already in intro)
❌ Compare to related work (already in intro)
❌ Justify design choices (that's in Section III)
❌ Show results (that's in Section IV)

---

## 📏 LENGTH ESTIMATES

| Subsection | Paragraphs | Equations | Pages |
|------------|-----------|-----------|-------|
| Opening | 0-1 | 0 | 0.05 |
| II.A | 3 | 4-5 | 0.5 |
| II.B | 7 (6 assumptions + comm) | 6 | 1.0 |
| II.C | 5 | 10 | 1.0 |
| II.D | 4 | 2-3 | 0.7 |
| **TOTAL** | ~20 | ~23 | **3.25** |

This is about right for a 6-page conference paper (Section II typically ~3 pages).

---

## 🔗 CROSS-REFERENCES TO TRACK

As you write, note where you need to add references:

- From A3 → to Theorem T1 (flow gradient in proof)
- From A4 → to Section III.B (QP feasibility)
- From A6 → to $\mathcal{C}_{\text{feasible}}$ definition
- From $\mathcal{C}_t$ → to Section III.A (how we identify it)
- From $\mathcal{C}_{\text{safe}}$, $\mathcal{C}_{\text{conn}}$ → to Section III.B (CBF construction)
- From Problem Statement → to Theorem T1, T2, T3, T4

---

## STATUS: ⬜ MASTER OUTLINE - Ready for Writing

**Next Steps:**
1. Review this master outline
2. Write each subsection following the para-by-para structure
3. Use the individual .md files for detailed LaTeX code
4. Integrate into ACC_2026.tex when satisfied
