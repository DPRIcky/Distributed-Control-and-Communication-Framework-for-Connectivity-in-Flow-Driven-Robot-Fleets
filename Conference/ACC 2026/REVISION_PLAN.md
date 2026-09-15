# ACC 2026 Paper Revision Plan
**Date:** January 22, 2026  
**Deadline:** End of this week (January 26, 2026)

## Executive Summary
The paper received rejection with valuable feedback from two reviewers. The main concerns are: (1) lack of mathematical rigor in problem formulation, (2) missing formal proofs and guarantees, (3) unclear novelty given flow affects all robots equally, and (4) insufficient experimental validation. This plan addresses all concerns systematically.

---

## Reviewer Comments Summary

### Reviewer 18 (Review ID: 10607) - Major Concerns
| # | Comment | Severity | Category |
|---|---------|----------|----------|
| 1 | Models in section II-A and II-B are confusing/different | High | Mathematical Formulation |
| 2 | Flow term is same for all robots → connectivity not challenging | **Critical** | Novelty/Contribution |
| 3 | Problem statement not in rigorous math | High | Mathematical Rigor |
| 4 | No proof showing proposed method solves the problem | **Critical** | Theoretical Foundation |

### Reviewer 25 (Review ID: 14961) - Constructive Feedback
| # | Comment | Severity | Category |
|---|---------|----------|----------|
| 1 | Need convergence or stability guarantees | High | Theoretical Foundation |
| 2 | Meaning of dmin on Page 4 unclear | Low | Presentation |
| 3 | Unclear where Lyapunov function is used | Medium | Theoretical Foundation |
| 4 | Need more relevant references in introduction | Medium | Literature Review |

---

## Required Revisions

### SECTION 1: Mathematical Rigor & Formulation

#### Task 1.1: Unify Robot Models (Reviewer 18, Comment 1)
**Current Issue:** Section II-A uses advection-diffusion model with stochastic Wiener process:
```
dxi(t) = ui(t)dt + fflow(x̄(t),t)dt + Fdrag(ẋi)dt + dWi(t)
```
Then simplifies to: `ẋi(t) = fflow(x̄(t),t) + ui(t) + di(t)` where `||di|| ≤ d̄`

Section II-B uses deterministic single-integrator: `ẋi = ui`

**Root Cause:** The progression from physical model → control model is unclear. Reviewers see contradiction.

**Action Items:**
- [x] IDENTIFIED: II-A is physical/simulation model; II-B is control synthesis model
- [ ] Add **Remark 1** after Eq. (1): "For control synthesis in Sec. III, we adopt the simplified model ẋi = ui, treating flow and disturbances as external forces handled by the barrier functions. The full dynamics (1) are used only in simulation validation (Sec. IV)."
- [ ] In Section II-B, add reference back: "This model is standard for distributed multi-robot control [12], [15]. The advection-diffusion dynamics from Sec. II-A are incorporated during simulation (Sec. IV)."
- [ ] Ensure notation consistency: xi(t) ∈ R², ui(t) ∈ R², ||ui|| ≤ umax throughout
- [ ] Add to notation table: distinction between "physical model" vs "control model"

**Timeline:** Day 1 (Jan 22) - 2 hours  
**Deliverable:** Clarified relationship between models with explicit remark in paper

---

#### Task 1.2: Formalize Problem Statement (Reviewer 18, Comment 3)
**Current Issue:** Section II-C "Problem Statement" is qualitative ("design a distributed control framework that satisfies...") without mathematical formulation

**What's Missing:** 
- No formal optimization problem
- No mathematical definition of connectivity constraint
- No formal objective function
- Lists objectives 1-3 in prose instead of mathematics

**Action Items:**
- [ ] **Add Problem Formulation Box** after current prose description:
  ```
  Problem 1 (Distributed Connectivity-Preserving Control):
  Given: N robots with dynamics ẋi = ui, i ∈ V = {1,...,N}
  State: X = {x1,...,xN} ∈ R^(2N)
  Control: U = {u1,...,uN}, ||ui|| ≤ umax
  Initial graph: G0 = (V, E0) connected
  
  Constraints:
    C1 (Connectivity): Gt remains connected ∀t ≥ 0
    C2 (Safety): ||xi - xj|| ≥ dmin ∀i,j, ∀t
    C3 (Communication): Edge (i,j) exists ⟺ ||xi - xj|| ≤ Rmax
    C4 (Distributed): Each robot i uses only {xj - xi}_{j∈N(i)} and local messages
  
  Objective: Minimize total control effort
    J = ∫₀ᵀ Σᵢ ||ui(t)||² dt
  
  while achieving target xᵍᵒᵃˡ when detected
  ```
- [ ] Reference this formal problem in Section III introduction
- [ ] Add remark: "Unlike centralized methods [17], we seek distributed solution using only local information"

**Timeline:** Day 2 (Jan 23) - 2 hours  
**Deliverable:** Formal Problem 1 box added to Section II-C

---

#### Task 1.3: Clarify dmin Parameter (Reviewer 25, Comment 2)
**Current Issue:** Page 4 states "A safety distance dmin = 0.6 m is enforced" but definition never given in problem setup (Sec II-B)

**Where it appears:**
- Mentioned in passing on page 2: "minimum pairwise separation distance dmin > 0 must be maintained"
- Section III-B: "safety barrier hsafe_ij = ||xi - xj||² - dmin²"
- Section IV: numerical value 0.6m given without justification

**Action Items:**
- [ ] In Section II-B after communication model, add: "**Collision Avoidance:** A minimum separation distance dmin > 0 is enforced to prevent physical collisions. Typically dmin < Rmax to allow neighbors to communicate while maintaining safe separation."
- [ ] In notation table, add: "dmin - Minimum allowable inter-robot distance (safety constraint)"
- [ ] In Section IV simulation setup, justify choice: "dmin = 0.6 m is set to half the communication range to balance safety and coordination"

**Timeline:** Day 1 (Jan 22) - 30 minutes  
**Deliverable:** Clear definition added in 3 places

---

### SECTION 2: Theoretical Foundation & Proofs

#### Task 2.1: Develop Formal Proofs (Reviewer 18, Comment 4; Reviewer 25, Comment 1)
**Current Issue:** No proof that method solves the problem  
**Required Proofs:**

**Proof 1: Connectivity Preservation**
- [ ] **Theorem 1:** Under the proposed hybrid control law, if $\lambda_2(\mathcal{L}(0)) > \lambda_{\min}$, then $\lambda_2(\mathcal{L}(t)) \geq \lambda_{\min}, \forall t \geq 0$
- [ ] Approach: Use CBF (Control Barrier Function) framework
- [ ] Show barrier function $h(\mathbf{x}) = \lambda_2(\mathcal{L}) - \lambda_{\min}$
- [ ] Prove $\dot{h}(\mathbf{x}) \geq -\alpha h(\mathbf{x})$ under control law

**Proof 2: Consensus Convergence**
- [ ] **Theorem 2:** The distributed adjacency consensus algorithm converges to global optimum in finite time
- [ ] Cite/prove: Connected graph → consensus convergence
- [ ] Prove: Pruning maintains connectivity (based on Theorem 1)

**Proof 3: Stability Analysis**
- [ ] **Theorem 3:** The closed-loop system is Lyapunov stable
- [ ] Define Lyapunov candidate: $V(\mathbf{x}) = \frac{1}{2}\sum_{i,j} \|\mathbf{p}_i - \mathbf{p}_j\|^2 a_{ij}$
- [ ] Show $\dot{V}(\mathbf{x}) \leq 0$ under control law
- [ ] Interpret: robots maintain formation while drifting

**Timeline:** Day 2-4 (Jan 23-25)  
**Deliverable:** New Section IV "Theoretical Analysis" with 3 formal theorems + proofs

---

#### Task 2.2: Clarify Lyapunov Function Usage (Reviewer 25, Comment 3)
**Current Issue:** Section III-B defines:
```
Vi(xi) = ||xi - xi_ref||²
```
and states "the inequality upon this candidate" but NEVER shows:
- What inequality? (presumably V̇i ≤ -αVi)
- Proof that V̇i ≤ 0
- Connection to stability
- How it's used in the QP

**It appears this is a CLF (Control Lyapunov Function) for the QP, NOT a Lyapunov proof!**

**Action Items:**
- [ ] **Option A (Recommended):** Add explicit subsection "III-B.1 CLF-CBF Formulation" explaining:
  ```
  The QP at each robot i solves:
    min ||ui||² + penalty terms
    s.t. V̇i + αVi ≤ 0  (CLF constraint)
         ḣsafe_ij + βs hsafe_ij ≥ 0  ∀j ∈ N(i)
         ḣconn_ij + βc hconn_ij ≥ 0  ∀j ∈ Ci
  ```
- [ ] Reference CLF-CBF-QP literature [25] for well-posedness
- [ ] Add Remark: "This does not guarantee global asymptotic stability but ensures forward invariance of safe set {h ≥ 0}"
- [ ] **Option B:** If want formal stability, need Theorem 3 proving closed-loop stability

**Timeline:** Day 3 (Jan 24) - 3 hours  
**Deliverable:** Clear CLF-CBF formulation section or formal stability theorem

---

### SECTION 3: Novelty & Contribution

#### Task 3.1: Address Flow Term Challenge (Reviewer 18, Comment 2)
**Current Issue:** Reviewer states: "Even for the model (1), the connectivity problem is not challenging in the sense that the flow term is the same for all the robots, which can thus be neglected when considering the relative positions among robots."

**This is WRONG but requires strong rebuttal!**

**Counter-Argument 1: Flow IS Spatially Varying**
- Current Section II-A states: `fflow(x̄(t),t)` - using CENTER OF MASS
- BUT actual flow model (Sec IV) has: `fflow(x,t) = vbg(t) + Σ vortices` → **position-dependent!**
- [ ] FIX Section II-A: Change fflow(x̄(t),t) → fflow(xi(t),t) to emphasize spatial variation
- [ ] Add explicit note: "The flow field fflow(xi,t) varies spatially due to vortices and eddies. Unlike uniform drift where relative dynamics are flow-independent, spatially varying flow creates differential advection that must be actively countered to maintain connectivity."
- [ ] Add Figure: Flow field vectors at different positions showing gradient
- [ ] Reference [5], [6] which also exploit spatial flow structure

**Counter-Argument 2: Energy-Limited Control is the Core Challenge**
- [ ] Revise Introduction paragraph 2: "The central challenge is to design strategies that enable the fleet to (i) expend **minimal** control effort..."
- [ ] Add: "Standard connectivity maintenance methods [12]-[16] assume continuous control actuation. In contrast, our energy-constrained setting requires **selective activation** only when connectivity is threatened."
- [ ] Emphasize in Problem Statement: Unlike [17] which assumes full control authority, we minimize ∫||u||² dt

**Counter-Argument 3: Distributed + No Global Information**
- [ ] Current intro mentions this but undersell it
- [ ] Strengthen: "Existing connectivity methods [14] require centralized eigenvalue computation, while distributed CBF approaches [15], [16] maintain all edges. Our contribution: distributed pruning + selective control on sparse structure."

**Timeline:** Day 2 (Jan 23) - 3 hours  
**Deliverable:** Revised Sections I, II-A with strengthened novelty claims

---

### SECTION 4: Literature Review & Positioning

#### Task 4.1: Expand Related Work (Reviewer 25, Comment 4)
**Action Items:**

**Add References (10-15 papers) in Following Areas:**

1. **Connectivity Maintenance (3-4 papers)**
   - [ ] Zavlanos & Pappas (2007) - Graph connectivity control
   - [ ] Sabattini et al. (2013) - Decentralized connectivity maintenance
   - [ ] Yang et al. (2010) - Connectivity preserving flocking

2. **Flow-Assisted Robotics (3-4 papers)**
   - [ ] Smith et al. (2011) - Glider path planning in currents
   - [ ] Inanc et al. (2005) - AUV navigation in ocean currents
   - [ ] Leonard & Graver (2001) - Underwater glider dynamics

3. **Energy-Efficient Multi-Robot Systems (2-3 papers)**
   - [ ] Choi et al. (2009) - Consensus with limited communication
   - [ ] Gao & Cai (2020) - Event-triggered coordination

4. **CBF/CLF Methods (2-3 papers)**
   - [ ] Ames et al. (2019) - Control barrier functions survey
   - [ ] Xu et al. (2015) - Connectivity maintenance via CBF

**Positioning Statement:**
- [ ] Add paragraph: "Unlike [refs], which assume unlimited actuation, our method..."
- [ ] Table 1: Comparison of our approach vs. existing methods (features matrix)

**Timeline:** Day 3-4 (Jan 24-25)  
**Deliverable:** Extended Section I-C "Related Work" + comparison table

---

### SECTION 5: Experimental Validation

#### Task 5.1: Design New Experiments
**Current Issue:** Need stronger validation of theoretical claims  
**Required Experiments:**

**Experiment 1: Connectivity Preservation**
- [ ] Setup: 10 robots, time-varying flow field, communication range = 50m
- [ ] Metric: Track $\lambda_2(t)$ over time
- [ ] Validation: Show $\lambda_2(t) \geq \lambda_{\min}$ always (proves Theorem 1)
- [ ] Vary: flow strength, initial configuration, $\lambda_{\min}$ threshold

**Experiment 2: Energy Efficiency**
- [ ] Baseline 1: Continuous control (always-on)
- [ ] Baseline 2: No control (pure drift, loses connectivity)
- [ ] Baseline 3: Event-triggered control without flow exploitation
- [ ] Metric: Total control effort $\int \|u(t)\| dt$ over mission
- [ ] Show: 60-80% energy savings vs. continuous control

**Experiment 3: Scalability**
- [ ] Fleet sizes: 5, 10, 20, 50 robots
- [ ] Metric: Communication overhead, computation time, success rate
- [ ] Show: Linear/polynomial scaling (distributed algorithm benefit)

**Experiment 4: Robustness**
- [ ] Vary: flow prediction error, communication delays, robot failures
- [ ] Metric: Connectivity maintenance success rate
- [ ] Show: Graceful degradation

**Experiment 5: Comparison with Existing Methods**
- [ ] Implement: Standard potential field method, centralized optimization
- [ ] Compare: Energy, connectivity robustness, scalability
- [ ] Highlight: Superiority in flow-dominated regimes

**Timeline:** Day 4-5 (Jan 25-26)  
**Deliverable:** Section VI "Simulation Results" with 5 experiment subsections

---

#### Task 5.2: Create Validation Figures
**Action Items:**
- [ ] Figure: $\lambda_2(t)$ time history for all experiments
- [ ] Figure: Energy consumption comparison (bar chart)
- [ ] Figure: Robot trajectories overlaid on flow field (show drift exploitation)
- [ ] Figure: Network topology evolution (snapshots)
- [ ] Table: Quantitative results summary

**Timeline:** Day 5 (Jan 26)  
**Deliverable:** 5-7 publication-quality figures

---

### SECTION 6: Code Implementation

#### Task 6.1: Implement Proofs in Code
**Action Items:**
- [ ] Add `tests/test_connectivity_preservation.py`: Verify Theorem 1 empirically
- [ ] Add `tests/test_consensus_convergence.py`: Verify Theorem 2
- [ ] Add `tests/test_stability.py`: Compute Lyapunov function, verify $\dot{V} \leq 0$

**Timeline:** Day 3-4 (Jan 24-25)  
**Deliverable:** 3 new test files with validation

---

#### Task 6.2: Create Baseline Comparisons
**Action Items:**
- [ ] File: `examples/baseline_continuous_control.py` (always-on control)
- [ ] File: `examples/baseline_pure_drift.py` (no control)
- [ ] File: `examples/baseline_event_triggered.py` (standard event-triggered)
- [ ] File: `examples/comparison_study.py` (runs all methods, generates figures)

**Timeline:** Day 4-5 (Jan 25-26)  
**Deliverable:** 4 new example files

---

## Implementation Schedule

### Day 1 (Jan 22, 2026) - Wednesday ✓
- [x] Review all reviewer comments (Task completed above)
- [ ] Create revision plan document (this file)
- [ ] Task 1.3: Define dmin (30 min)
- [ ] Task 1.1: Start model unification (2 hours)

### Day 2 (Jan 23, 2026) - Thursday
- [ ] Task 1.1: Complete model unification (2 hours)
- [ ] Task 1.2: Formalize problem statement (3 hours)
- [ ] Task 3.1: Revise novelty arguments (2 hours)
- [ ] Task 2.1: Start proofs - connectivity preservation (2 hours)

### Day 3 (Jan 24, 2026) - Friday
- [ ] Task 2.1: Complete all proofs (4 hours)
- [ ] Task 2.2: Clarify Lyapunov usage (1 hour)
- [ ] Task 4.1: Literature review - find and add references (3 hours)
- [ ] Task 6.1: Start proof validation code (1 hour)

### Day 4 (Jan 25, 2026) - Saturday
- [ ] Task 6.1: Complete proof validation code (2 hours)
- [ ] Task 4.1: Write related work section (2 hours)
- [ ] Task 5.1: Design and run experiments 1-3 (4 hours)
- [ ] Task 6.2: Start baseline implementations (2 hours)

### Day 5 (Jan 26, 2026) - Sunday
- [ ] Task 6.2: Complete baseline implementations (2 hours)
- [ ] Task 5.1: Complete experiments 4-5 (2 hours)
- [ ] Task 5.2: Create all validation figures (3 hours)
- [ ] Task 1-5: Integrate all changes into paper draft (3 hours)
- [ ] Final review and send to PI (1 hour)

---

## Deliverables Summary

### Documents to Send PI by Jan 26, 2026:
1. ✅ **This revision plan** (concrete list of required revisions)
2. **Updated paper draft** with:
   - Unified robot models (Section II)
   - Rigorous problem formulation (Section III)
   - New theoretical analysis section with proofs (Section IV)
   - Revised novelty/contribution claims (Section I)
   - Expanded related work (Section I-C)
   - New experimental results (Section VI)
   - Clear Lyapunov function usage
   - All reviewer comments addressed

3. **Code updates:**
   - Proof validation tests (`tests/test_connectivity_preservation.py`, etc.)
   - Baseline comparison implementations
   - Experiment scripts for new results

4. **Response to reviewers document:**
   - Point-by-point response to each comment
   - Mapping of changes to paper sections/equations

---

## Risk Mitigation

### High-Priority Items (Must Complete):
1. **Formal proofs** (Reviewer 18 Comment 4 is critical)
2. **Problem formulation** (Reviewer 18 Comment 3)
3. **Novelty clarification** (Reviewer 18 Comment 2)

### If Time Constraints:
- Complete high-priority items first (Days 2-3)
- Experimental validation can be subset (3 experiments minimum)
- Related work expansion can be 5 references minimum

### Backup Plan:
- If proofs complex, consider collaborative session with PI on Day 3
- If experiments take longer, prepare preliminary results + plan for additional validation

---

## Success Metrics
- [ ] All 8 reviewer comments explicitly addressed
- [ ] At least 3 formal theorems with complete proofs
- [ ] At least 3 experimental validations
- [ ] 10+ new relevant references
- [ ] Paper ready for resubmission to next venue (or journal)

---

## Notes for PI Discussion
- Most critical issue: **Reviewer 18's novelty concern** - need strong counter-argument
- Theoretical proofs will strengthen paper significantly (currently a weakness)
- Suggest targeting journal (e.g., IEEE T-RO, Automatica) for longer format if proofs extensive
- Alternative venues: IROS 2026 (July deadline), CDC 2026, or IEEE T-ASE

---

**Next Action:** Share this plan with PI for approval, then begin execution starting with high-priority theoretical tasks.
