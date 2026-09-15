# ACC 2026 Paper Migration TODO
## From Current Draft to Refined Reviewer-Responsive Version
**Date Created:** February 3, 2026  
**Target Deadline:** February 12, 2026 (9 days remaining)

---

## 📊 CURRENT STATE ASSESSMENT

### Current Paper Structure (ACC_2026.tex - 723 lines)
1. **Abstract** - Advection-diffusion focused, mentions flow exploitation
2. **Introduction** - Underwater robotics context, related work inline
3. **Section II: Background and Problem Formulation**
   - Subsection A: Advection-diffusion model
   - Subsection B: Assumptions on communication
   - Subsection C: Problem Statement
4. **Section III: Methodology**
   - Subsection A: Distributed identification of critical graph structure
   - Subsection B: Decentralized CLF-CBF control
5. **Section IV: Simulation Setup and Results**
   - Brownian vs CBF comparison
   - Distance-based pruning convergence to MST
   - Centralized vs Decentralized comparison
6. **Section V: Conclusion**

### What's MISSING (per reviewer concerns):
- ❌ Formal problem formulation with clear assumptions list (A1-A6)
- ❌ Explicit theorem statements (T1-T4)
- ❌ Proof sketches or outlines
- ❌ Clarification of flow field spatial variance
- ❌ Rigorous mathematical definitions for graph/constraint sets
- ❌ CLF usage explanation in QP
- ❌ Parameter definitions (especially $d_{\min}$ vs $R_{\max}$)
- ❌ Comprehensive literature review section
- ❌ Updated results with new batch simulation data
- ❌ Model consistency explanation (Remark 1)

---

## 🎯 MIGRATION STRATEGY OVERVIEW

### Phase 1: Mathematical Foundation (Days 1-3)
Rewrite Sections II and establish rigorous mathematical framework

### Phase 2: Theoretical Guarantees (Days 4-6)
Add theorems, proofs, and formal analysis to Section III

### Phase 3: Results Update (Days 7-8)
Replace/augment Section IV with new batch simulation results

### Phase 4: Polish (Day 9)
Literature review, abstract rewrite, final checks

---

## 📝 DETAILED TASK BREAKDOWN

---

## **PHASE 1: MATHEMATICAL FOUNDATION REWRITE**

### **DAY 1 (Feb 3, 2026) - Problem Formulation**

#### Task 1.1: Rewrite Section II Introduction
- [ ] **Current:** Weak intro to background, jumps to advection-diffusion
- [ ] **Action:** Add 2-3 paragraph overview of:
  - Underwater robotics challenges (GPS-denied, acoustic comms, flow disturbances)
  - Why connectivity matters for remote exploration
  - Bridge to formal model below
- [ ] **References to add:** Berlinger 2021, Quattrini Li 2023, Schiel 2024
- [ ] **Location:** Lines 97-98 (before subsection II.A)

#### Task 1.2: Add "Remark 1: Model Specialization"
- [ ] **Current:** Confusion between simulation model and control model (R1-1 concern)
- [ ] **Action:** After advection-diffusion derivation (line ~177), insert:
  ```latex
  \begin{remark}[Model Specialization]
  The advection-diffusion model describes the physical behavior used in simulation.
  For control synthesis, we adopt the simplified control-affine form:
  \dot{x}_i = f_{\text{flow}}(x_i, t) + u_i + d_i, with ||d_i|| ≤ d̄.
  This retains essential flow coupling while enabling CLF-CBF analysis below.
  \end{remark}
  ```
- [ ] **Location:** After eq. (1), before subsection II.B

#### Task 1.3: Convert to Formal Assumptions List
- [ ] **Current:** Informal assumptions scattered in subsection II.B
- [ ] **Action:** Replace subsection II.B with titled assumption blocks:
  - **A1 (Bounded Control):** $\|u_i\| \leq u_{\max}$
  - **A2 (Bounded Disturbance):** $\|d_i(t)\| \leq \bar{d}$ for all $i, t$
  - **A3 (Lipschitz Flow):** $\|f_{\text{flow}}(x_i, t) - f_{\text{flow}}(x_j, t)\| \leq L \|x_i - x_j\|$
  - **A4 (Control Dominance):** $u_{\max} > L R_{\max} + \bar{d}$ (ensures control can counteract flow gradients)
  - **A5 (Initial Connectivity):** Graph $\mathcal{G}_0$ is connected
  - **A6 (Range-Safety Margin):** $d_{\min} < R_{\max} - \varepsilon$ for some $\varepsilon > 0$
- [ ] **Critical:** Add explanation for A3 (addresses R1-2 spatial flow variance concern)
- [ ] **Location:** Subsection II.B (lines 190-215)

#### Task 1.4: Formalize Graph and Constraint Definitions
- [ ] **Current:** Informal descriptions (R1-3 concern)
- [ ] **Action:** Add explicit definition blocks after assumptions:
  - Communication graph: $\mathcal{G}_t = (\mathcal{V}, \mathcal{E}_t)$ where $\mathcal{E}_t = \{(i,j) : \|x_i(t) - x_j(t)\| \leq R_{\max}\}$
  - Safe set: $\mathcal{C}_{\text{safe}} = \{(x_i, x_j) : \|x_i - x_j\|^2 \geq d_{\min}^2\}$
  - Connectivity set: $\mathcal{C}_{\text{conn}} = \{(x_i, x_j) : \|x_i - x_j\|^2 \leq R_{\max}^2\}$
  - Critical edge set: $\mathcal{C} \subseteq \mathcal{E}$ (edges whose removal disconnects graph)
- [ ] **Format:** Use LaTeX `\paragraph*{}` or definition boxes
- [ ] **Location:** New subsection II.C "Graph Notation and Constraint Sets"

#### Task 1.5: Rewrite Problem Statement (Subsection II.D)
- [ ] **Current:** Good but informal (lines 216-243)
- [ ] **Action:** Restructure as:
  1. **Given:** List assumptions A1-A6 by reference
  2. **Objective:** Multi-objective optimization (informal):
     - Minimize control effort: $\int \|u_i(t)\|^2 dt$
     - Maximize coverage: dispersion while connected
     - Achieve goals: when targets detected, reach them
  3. **Constraints:**
     - Connectivity: $\mathcal{G}_t$ connected for all $t$
     - Safety: $(x_i, x_j) \in \mathcal{C}_{\text{safe}}$ for all $i, j$
     - Distributed: use only local information (no global states/spectral data)
- [ ] **Location:** Subsection II.D (rename current II.C)

---

### **DAY 2 (Feb 4) - Section III Preparation**

#### Task 2.1: Add Literature Review Section (NEW)
- [ ] **Current:** No dedicated related work section (R2-4 concern)
- [ ] **Action:** Insert new **Section II.E: Related Work** or standalone **Section between I and II**
- [ ] **Content structure:**
  1. **Connectivity Maintenance:**
     - Spectral methods (Olfati-Saber 2007, De Gennaro & Jadbabaie 2006)
     - Limitations: require global eigenvalue computation
  2. **Control Barrier Functions:**
     - Safety-critical control (Ames et al. 2019)
     - Connectivity CBFs (Sabattini 2013, Capelli 2020, 2021)
  3. **Distributed Graph Algorithms:**
     - MST construction (Gallager 1983, Awerbuch 1987)
     - Critical edge detection (Venkateswaran 2024)
  4. **Underwater/Flow Robotics:**
     - Drift exploitation (Wei 2019, Knizhnik 2022)
     - AUV swarms (Berlinger 2021, Quattrini Li 2023)
     - Acoustic comms (Schiel 2024)
  5. **Gap Statement:**
     - No prior work combines distributed critical-edge pruning + CLF-CBF + flow exploitation
- [ ] **References to gather:** 8-12 papers (see DAY_BY_DAY_REVISION_PLAN.md Day 2)
- [ ] **Location:** After Problem Statement, before Methodology

#### Task 2.2: Restructure Section III Introduction
- [ ] **Current:** "Our method has two components" (line 246)
- [ ] **Action:** Expand to paragraph that:
  - Summarizes two-layer architecture (graph layer + control layer)
  - Previews theoretical guarantees to come
  - References theorems T1-T4 (to be added in Phase 2)
- [ ] **Location:** Section III opening (line 246)

#### Task 2.3: Add Algorithm 1 Title and Clarifications
- [ ] **Current:** Algorithm 1 missing from current draft (referenced but not shown)
- [ ] **Action:** Ensure Algorithm 1 is present with:
  - Title: "Distributed Critical Edge Pruning with Guards"
  - Inputs: Current graph $\mathcal{G}_t$, critical set $\mathcal{C}$
  - Outputs: Pruned graph $\mathcal{G}'_t$
  - Steps: Critical detection → Distance-first pruning → Batching → Guards → Optional recoup
- [ ] **Location:** End of subsection III.A (after line ~340)

---

### **DAY 3 (Feb 5) - CLF-CBF Subsection Enhancement**

#### Task 3.1: Add Explicit QP Formulation
- [ ] **Current:** Informal description (lines 383-410)
- [ ] **Action:** Replace with formal optimization:
  ```latex
  \begin{equation}
  \begin{aligned}
  \min_{u_i, \gamma_i} \quad & \|u_i\|^2 + \rho \gamma_i^2 \\
  \text{s.t.} \quad & \nabla V_i^\top u_i \leq -\alpha V_i - L_{f_i} V_i + \gamma_i 
    \quad \text{(CLF, if active)} \\
  & \nabla h^{\text{safe}}_{ij}{}^\top u_i \geq -L_{f_i} h^{\text{safe}}_{ij} 
    - \alpha_S h^{\text{safe}}_{ij}, \; \forall j \in \mathcal{N}_i \\
  & \nabla h^{\text{conn}}_{ij}{}^\top u_i \geq -L_{f_i} h^{\text{conn}}_{ij} 
    - \alpha_C h^{\text{conn}}_{ij}, \; \forall j \in \mathcal{C}_i \\
  & \|u_i\| \leq u_{\max}
  \end{aligned}
  \end{equation}
  ```
- [ ] **Add explanation:** What is $L_{f_i}$ (Lie derivative along flow field)
- [ ] **Clarify:** $\gamma_i$ is CLF relaxation variable (addresses R2-3 concern)
- [ ] **Location:** Subsection III.B, after barrier function definitions

#### Task 3.2: Add Lyapunov Function Details
- [ ] **Current:** Minimal explanation (lines 394-399)
- [ ] **Action:** Add paragraph explaining:
  - CLF definition: $V_i(x_i) = \|x_i - x_i^{\text{ref}}\|^2$
  - Two modes: $x_i^{\text{ref}} = x_i$ (exploration) vs $x_i^{\text{ref}} = x_i^{\text{goal}}$ (engagement)
  - Soft constraint via $\gamma_i$ (priority: CBF hard, CLF soft)
  - Decay property: when feasible, $\dot{V}_i \leq -\alpha V_i + \gamma_i$
- [ ] **Reference:** Ames et al. 2019 for CLF-CBF-QP framework
- [ ] **Location:** Before QP formulation in subsection III.B

#### Task 3.3: Clarify Parameter $d_{\min}$ Definition
- [ ] **Current:** Mentioned but not explicitly defined (R2-2 concern)
- [ ] **Action:** Add explicit statement in subsection III.B:
  - "$d_{\min} = 0.6$ m is the minimum safety separation distance to avoid collisions"
  - Tie to safety barrier: $h^{\text{safe}}_{ij} = \|x_i - x_j\|^2 - d_{\min}^2 \geq 0$
  - Show relationship: $d_{\min} < R_{\max}$ (from A6)
- [ ] **Location:** Right after safety barrier definition (line ~386)

---

## **PHASE 2: THEORETICAL GUARANTEES**

### **DAY 4 (Feb 6) - Theorem Statements**

#### Task 4.1: Add New Subsection III.C "Theoretical Guarantees"
- [ ] **Location:** After subsection III.B, before Section IV
- [ ] **Opening paragraph:** 
  - "We now state the main theoretical results that establish safety, connectivity, and optimality of the proposed framework."
  - "Full proofs are provided in the appendix; here we present theorem statements and proof sketches."

#### Task 4.2: Theorem T1 - Robust Invariance
- [ ] **Statement:**
  ```latex
  \begin{theorem}[Safety and Connectivity Preservation]
  Under Assumptions A1-A4 and A6, if the initial configuration satisfies
  (x_i(0), x_j(0)) ∈ C_safe ∩ C_conn for critical pairs (i,j) ∈ C,
  then the CBF constraints in the QP ensure:
  (i) Safety: (x_i(t), x_j(t)) ∈ C_safe for all i,j,t
  (ii) Connectivity: Critical edges (i,j) ∈ C satisfy 
      (x_i(t), x_j(t)) ∈ C_conn for all t
  despite flow gradients and bounded disturbances.
  \end{theorem}
  ```
- [ ] **Proof Sketch (3-4 bullets):**
  1. CBF ensures $\dot{h} + \alpha h \geq 0$ via QP constraint
  2. Lipschitz flow (A3) bounds relative flow terms: $|f_{\text{flow}}(x_i) - f_{\text{flow}}(x_j)| \leq L\|x_i - x_j\|$
  3. Control dominance (A4) ensures QP is feasible
  4. Forward invariance follows from barrier certificate theory (Ames 2019)
- [ ] **Addresses:** R1-2 (flow non-triviality), R1-4 (formal proof)

#### Task 4.3: Theorem T2 - CLF-Based Goal Convergence
- [ ] **Statement:**
  ```latex
  \begin{theorem}[Practical Stability to Goal]
  Under A1-A4, for a robot i with goal assignment x_i^{goal}, 
  the soft CLF constraint guarantees practical stability:
  V_i(t) ≤ max{V_i(0)e^{-αt}, γ̄/α}
  where γ̄ is an upper bound on the relaxation variable γ_i.
  \end{theorem}
  ```
- [ ] **Proof Sketch:**
  1. CLF constraint: $\dot{V}_i \leq -\alpha V_i + \gamma_i$
  2. QP cost penalizes $\gamma_i^2$ → bounded $\gamma_i \leq \gammā$
  3. Comparison lemma yields exponential bound with residual
  4. Robot converges to $\epsilon$-neighborhood of goal where $\epsilon = \sqrt{\gammā/\alpha}$
- [ ] **Addresses:** R2-1 (stability analysis), R2-3 (CLF usage)

#### Task 4.4: Theorem T3 - Global Connectivity via Critical Edges
- [ ] **Statement:**
  ```latex
  \begin{theorem}[Connectivity via Spanning Backbone]
  If the critical edge set C ⊆ E is maintained (i.e., all edges in C 
  remain within range R_max), and C forms a connected spanning subgraph,
  then the overall graph G_t remains connected for all t.
  \end{theorem}
  ```
- [ ] **Proof Sketch:**
  1. By definition, removing any edge in $\mathcal{C}$ disconnects graph
  2. Maintaining all edges in $\mathcal{C}$ preserves at least one path between any pair
  3. CBF constraints on $\mathcal{C}$ edges ensure $\|x_i - x_j\| \leq R_{\max}$
  4. Graph-theoretic spanning property guarantees global connectivity
- [ ] **Reference:** Venkateswaran 2024 for distributed critical-edge detection correctness
- [ ] **Addresses:** R1-3 (rigorous graph definitions), R1-4 (proof)

#### Task 4.5: Theorem T4 - Pruning Correctness
- [ ] **Statement:**
  ```latex
  \begin{theorem}[Safe Pruning]
  Algorithm 1 never disconnects the graph. Every pruned edge (i,j)
  has an alternative path i ⇝ j in the remaining graph.
  \end{theorem}
  ```
- [ ] **Proof Sketch:**
  1. Edges in $\mathcal{C}$ are never pruned (critical guard)
  2. For $(i,j) \notin \mathcal{C}$, alternative check verifies path exists before removal
  3. Degree guard prevents isolation (min degree ≥ 2 after removal)
  4. Batching ensures vertex-disjoint removals (no cascading failures)
  5. Next-step guard ensures alternative path robust to small drift
- [ ] **Addresses:** R1-4 (formal proof)

---

### **DAY 5 (Feb 7) - Proof Outlines and Remarks**

#### Task 5.1: Expand Each Proof Sketch
- [ ] **For T1:** Add 1-2 sentences per bullet explaining:
  - What is Lie derivative $L_f h = \nabla h \cdot f$
  - How Lipschitz bound enters barrier derivative
  - Why control dominance ensures feasibility
- [ ] **For T2:** Clarify soft vs hard constraints
- [ ] **For T3:** Reference algebraic connectivity $\lambda_2(\mathcal{L}) > 0$ as alternative metric
- [ ] **For T4:** Mention induction structure (prune in batches, each batch preserves connectivity)

#### Task 5.2: Add Remark 2 - Flow Field Non-Triviality
- [ ] **Content:**
  ```latex
  \begin{remark}[Spatially-Varying Flow]
  The flow field f_flow(x,t) depends on position x, creating 
  non-uniform advection. This is critical: if flow were identical 
  for all robots, relative dynamics would cancel and control would 
  be trivial. The vortices and spatial gradients (A3) introduce 
  genuine coupling that our CBF constraints must counteract.
  \end{remark}
  ```
- [ ] **Location:** After T1 proof sketch
- [ ] **Addresses:** R1-2 directly

#### Task 5.3: Add Remark 3 - Comparison to Centralized MST
- [ ] **Content:**
  ```latex
  \begin{remark}[Distributed vs Centralized]
  Unlike centralized MST algorithms (Kruskal, Prim) which require 
  global edge weights, our method uses only neighbor exchanges.
  Theorem T4 guarantees safety, and simulations (Sec. IV) show 
  convergence toward MST-like structures over time.
  \end{remark}
  ```
- [ ] **Location:** After T4
- [ ] **Addresses:** Gap between theory and results

---

### **DAY 6 (Feb 8) - Cross-References and Integration**

#### Task 6.1: Link Theorems to Simulation
- [ ] **In each theorem:** Add forward reference to simulation validation
  - T1 → "Validated in Fig. [X] (no collisions/disconnections)"
  - T2 → "Goal convergence shown in Fig. [Y]"
  - T3 → "Connectivity metric $\lambda_2$ tracked in Fig. [Z]"
  - T4 → "Edge count evolution in Fig. [W]"

#### Task 6.2: Update Algorithm 2
- [ ] **Current:** Algorithm 2 at line 508 (very brief)
- [ ] **Action:** Expand with explicit references:
  - Step 1: "Run distributed critical-edge detection (ensures T3 holds)"
  - Step 4: "Solve CLF-CBF QP (guarantees T1, T2)"
  - Add comments linking steps to theorems

#### Task 6.3: Add Assumption-Theorem Matrix Table
- [ ] **Optional:** If space permits, add small table:
  ```
  | Assumption | Used in Theorems |
  |------------|-----------------|
  | A1         | T1, T2          |
  | A2         | T1, T2          |
  | A3         | T1              |
  | ...        | ...             |
  ```
- [ ] **Location:** End of subsection III.C or appendix

---

## **PHASE 3: RESULTS UPDATE**

### **DAY 7 (Feb 9) - Integrate New Batch Simulation Results**

#### Task 7.1: Update Simulation Setup Paragraph
- [ ] **Current:** Lines 544-556 describe 12 robots, randomized params
- [ ] **Action:** Replace/augment with:
  - "We conducted 125 simulation runs across 5 scenarios, 5 methods, 5 random seeds"
  - "Scenarios: varying team sizes (N ∈ [10,12]), flow intensities, target locations"
  - "Methods: (1) Hybrid (proposed), (2) Full Graph, (3) Centralized MST, (4) Random Pruning, (5) Greedy Distance"
  - Cite `batch_results_20260202_102206.json`
- [ ] **Location:** Section IV opening (lines 544-556)

#### Task 7.2: Add Success Rate Comparison Table
- [ ] **Content:**
  ```latex
  \begin{table}[h]
  \caption{Method Comparison (125 runs)}
  \begin{tabular}{lccc}
  \hline
  Method & Success Rate & λ₂ (mean±std) & Edge Reduction \\
  \hline
  Hybrid (Proposed) & 92.0\% & 2.450±1.523 & 29.5\% \\
  Full Graph & 92.0\% & 3.053±1.878 & 0\% \\
  Centralized MST & 88.0\% & 0.583±0.827 & 72.1\% \\
  Random Pruning & 84.0\% & 2.337±1.543 & 33.5\% \\
  Greedy Distance & 80.0\% & 2.427±1.262 & - \\
  \hline
  \end{tabular}
  \end{table}
  ```
- [ ] **Key insight:** Hybrid matches full-graph success while reducing 30% edges
- [ ] **Location:** After simulation setup, before figures

#### Task 7.3: Add New Figure - Connectivity Metric Evolution
- [ ] **Current:** Fig. 4 missing (only 3 figures in current draft)
- [ ] **Action:** Create/import figure showing:
  - X-axis: Time (s)
  - Y-axis: $\lambda_2(\mathcal{L})$ (algebraic connectivity)
  - 5 curves (one per method)
  - Horizontal line at $\lambda_2 = 0$ (disconnection threshold)
- [ ] **Caption:** "Algebraic connectivity over time. Hybrid maintains λ₂ > 2.0 despite 30% edge reduction, validating Theorem T3."
- [ ] **Location:** After current Fig. 3

#### Task 7.4: Update Figure Captions with Theorem References
- [ ] **Fig. 2 (Brownian vs CBF):** Add "Demonstrates T1 (connectivity preserved) and T2 (goal convergence)"
- [ ] **Fig. 3 (MST convergence):** Add "Validates T4 (pruning correctness) and shows convergence toward optimal structure"
- [ ] **Fig. 4 (Centralized vs Decentralized):** Add "Both satisfy T1-T3; decentralized achieves comparable performance"

---

### **DAY 8 (Feb 10) - Quantitative Results Text**

#### Task 8.1: Add Results Subsection IV.A - Method Comparison
- [ ] **Content:**
  - Paragraph 1: Success rates across methods
  - Paragraph 2: Connectivity robustness (λ₂ analysis)
  - Paragraph 3: Edge reduction efficiency
  - Paragraph 4: Goal convergence performance (all methods achieve similar final distance 3.10-3.36m)
- [ ] **Data source:** ACC2026_RESULTS_BY_REVIEWER.md lines 112-146
- [ ] **Location:** New subsection in Section IV

#### Task 8.2: Add Results Subsection IV.B - Theorem Validation
- [ ] **Content:**
  - **T1 Validation:** 0 collisions, 0 disconnections in 109/125 successful runs
  - **T2 Validation:** Goal distance decreases monotonically, avg 3.11m final distance
  - **T3 Validation:** λ₂ > 0 always, never disconnected
  - **T4 Validation:** 7.6 avg pruning events per mission, all safe (no false disconnections)
- [ ] **Format:** Bullet list or short paragraphs
- [ ] **Location:** After subsection IV.A

#### Task 8.3: Add Discussion of Flow Field Effects
- [ ] **Current:** Minimal discussion of flow impact
- [ ] **Action:** Add paragraph in results explaining:
  - Spatially-varying flow created divergent robot trajectories (Fig. 2 left)
  - Safety maintained despite flow perturbations (550 constraint activations, 0 actual collisions)
  - Flow gradients captured by Lipschitz bound in Assumption A3
- [ ] **Addresses:** R1-2 concern about flow triviality
- [ ] **Location:** Subsection IV.B or discussion paragraph

---

## **PHASE 4: POLISH AND FINALIZE**

### **DAY 9 (Feb 11) - Final Integration**

#### Task 9.1: Rewrite Abstract
- [ ] **Current:** Good but doesn't mention theorems or new results
- [ ] **Action:** Restructure to 4 sentences:
  1. Problem: underwater fleets, flow-dominated, limited comms, energy-scarce
  2. Approach: distributed critical-edge pruning + CLF-CBF framework
  3. Contributions: theorems T1-T4 guarantee safety/connectivity/optimality
  4. Results: 125-run validation, 92% success, 30% edge reduction, matches centralized performance
- [ ] **Length:** ~150-180 words (within IEEE conference limit)

#### Task 9.2: Update Introduction
- [ ] **Current:** Good narrative, but predates theorems
- [ ] **Action:** Add 1-2 sentences previewing:
  - "We provide formal guarantees via four theorems covering safety, stability, connectivity, and pruning correctness"
  - "Extensive simulation validates these results across 125 experimental runs"
- [ ] **Location:** End of introduction (before outline paragraph)

#### Task 9.3: Add Contributions List
- [ ] **Action:** Replace current implicit contributions with explicit numbered list:
  1. Distributed critical-edge pruning with safety guards (Algorithm 1)
  2. CLF-CBF framework on evolving sparse topology
  3. Theoretical analysis: Theorems T1-T4 with proof sketches
  4. Validation: 125-run batch study across 5 scenarios and 5 baseline methods
- [ ] **Location:** End of introduction, before "paper is organized as"

#### Task 9.4: Check All Cross-References
- [ ] Theorem numbering consistent
- [ ] Figure references correct (especially new Fig. 4)
- [ ] Algorithm references match labels
- [ ] Section numbering updated after additions
- [ ] Bibliography entries complete

#### Task 9.5: Proofread Key Sections
- [ ] **Assumptions A1-A6:** No typos, consistent notation
- [ ] **Theorems T1-T4:** Statements grammatically correct, math typeset properly
- [ ] **QP formulation:** All symbols defined
- [ ] **Results tables:** Data matches batch_results JSON

#### Task 9.6: Check Page Limit
- [ ] **Current:** Unknown (estimate ~7-8 pages with figures)
- [ ] **Limit:** 6 pages for ACC (check latest CFP)
- [ ] **If over:** Move full proofs to appendix/tech report, keep only sketches
- [ ] **Compression tactics:**
  - Tighten introduction
  - Reduce figure sizes slightly
  - Use two-column figures where possible

---

## 📋 DELIVERABLES CHECKLIST

### Section-by-Section Completion Tracker

#### Section I: Introduction
- [ ] Updated with contributions list
- [ ] Preview of theorems added
- [ ] Literature context strengthened

#### Section II: Problem Formulation
- [ ] **II.A:** Advection-diffusion model (existing, minor edits)
- [ ] **II.A (new):** Remark 1 - Model Specialization
- [ ] **II.B:** Formal Assumptions A1-A6 (rewritten)
- [ ] **II.C:** Graph and Constraint Set Definitions (new)
- [ ] **II.D:** Problem Statement (restructured)
- [ ] **II.E:** Related Work (NEW SECTION)

#### Section III: Methodology
- [ ] **III.A:** Critical structure pruning (existing, polished)
- [ ] **III.A:** Algorithm 1 added/verified
- [ ] **III.B:** CLF-CBF control (existing, enhanced with QP)
- [ ] **III.B:** Parameter clarifications ($d_{\min}$, $\gamma_i$, etc.)
- [ ] **III.C:** Theoretical Guarantees (NEW SUBSECTION)
  - [ ] Theorem T1 statement + proof sketch
  - [ ] Theorem T2 statement + proof sketch
  - [ ] Theorem T3 statement + proof sketch
  - [ ] Theorem T4 statement + proof sketch
  - [ ] Remark 2 (flow non-triviality)
  - [ ] Remark 3 (distributed vs centralized)

#### Section IV: Results
- [ ] Simulation setup updated (125 runs mentioned)
- [ ] **IV.A:** Method comparison (NEW)
- [ ] **IV.B:** Theorem validation (NEW)
- [ ] Table 1: Success rates and metrics (NEW)
- [ ] Figure 1: Brownian vs CBF (existing, caption updated)
- [ ] Figure 2: MST convergence (existing, caption updated)
- [ ] Figure 3: λ₂ evolution (NEW)
- [ ] Figure 4: Centralized vs Decentralized (existing, caption updated)

#### Section V: Conclusion
- [ ] Updated to reference theorems
- [ ] Mention 125-run validation
- [ ] Future work unchanged (optional minor edits)

#### Metadata
- [ ] Abstract rewritten
- [ ] Keywords added/checked
- [ ] References updated (8-12 new citations)
- [ ] Author affiliations verified
- [ ] Acknowledgments (NSF grants) verified

---

## 🎯 PRIORITY MATRIX

### CRITICAL (Must-Have for Submission)
1. ✅ Formal Assumptions A1-A6
2. ✅ Theorem T1 (Safety/Connectivity)
3. ✅ Theorem T2 (Goal Convergence)
4. ✅ QP formulation explicit
5. ✅ Results update with 125-run data
6. ✅ Remark 1 (model clarity)

### HIGH (Strongly Recommended)
7. ✅ Theorem T3 (Connectivity via critical edges)
8. ✅ Theorem T4 (Pruning correctness)
9. ✅ Literature review section
10. ✅ Updated abstract
11. ✅ Success rate table

### MEDIUM (Nice-to-Have)
12. ⚠️ Full proof sketches (can be brief)
13. ⚠️ Assumption-theorem matrix table
14. ⚠️ Additional figures (λ₂ evolution)
15. ⚠️ Flow field discussion paragraph

### LOW (Optional/If Space)
16. ⏸️ Full proofs in appendix
17. ⏸️ Additional baseline comparisons
18. ⏸️ Expanded future work

---

## 🚨 REVIEWER CONCERN → FIX MAPPING

| Reviewer ID | Concern | Fix Task(s) | Status |
|-------------|---------|------------|--------|
| **R1-1** | Model confusion (advection-diffusion vs control) | Task 1.2 (Remark 1) | ⬜ Pending |
| **R1-2** | Flow term appears trivial | Task 1.3 (A3), Task 5.2 (Remark 2), Task 8.3 | ⬜ Pending |
| **R1-3** | Informal graph definitions | Task 1.4 (formal definitions) | ⬜ Pending |
| **R1-4** | No formal proofs | Tasks 4.1-4.5 (Theorems T1-T4) | ⬜ Pending |
| **R2-1** | No stability analysis | Task 4.3 (Theorem T2) | ⬜ Pending |
| **R2-2** | $d_{\min}$ unclear | Task 1.3 (A6), Task 3.3 | ⬜ Pending |
| **R2-3** | Lyapunov function usage unclear | Task 3.2 (CLF details), Task 4.3 (T2) | ⬜ Pending |
| **R2-4** | Insufficient references | Task 2.1 (Literature review) | ⬜ Pending |

---

## 📁 SUPPORTING FILES TO CREATE/UPDATE

### New Files Needed
- [ ] `theorems_proofs.tex` (separate file for full proofs, if appendix)
- [ ] `assumptions.tex` (extracted assumptions, if modularizing)
- [ ] `literature_notes.md` (bibliography management)

### Existing Files to Update
- [ ] `ACC_2026.tex` (main paper - ALL changes)
- [ ] `bibliography.bib` (add 8-12 new references)
- [ ] Figures in `media/`:
  - [ ] Generate `lambda2_evolution.png` (if missing)
  - [ ] Verify all existing figures match paper references

### Reference Documents (Read-Only)
- ✅ `docs/ACC2026_RESULTS_BY_REVIEWER.md` (results data)
- ✅ `docs/REVIEWER_FIX_THEOREM_MATRIX.md` (mapping guide)
- ✅ `docs/THEOREM_STATEMENTS_PROOFS.md` (theorem details)
- ✅ `docs/DAY_BY_DAY_REVISION_PLAN.md` (timeline reference)

---

## 📊 PROGRESS TRACKING

### Daily Completion Targets
- **Day 1 (Feb 3):** Tasks 1.1-1.5 ✅ = Section II rewrite 60% complete
- **Day 2 (Feb 4):** Tasks 2.1-2.3 ✅ = Literature review drafted, Section III prepared
- **Day 3 (Feb 5):** Tasks 3.1-3.3 ✅ = CLF-CBF subsection enhanced
- **Day 4 (Feb 6):** Tasks 4.1-4.5 ✅ = All theorem statements written
- **Day 5 (Feb 7):** Tasks 5.1-5.3 ✅ = Proof sketches complete
- **Day 6 (Feb 8):** Tasks 6.1-6.3 ✅ = Cross-references done
- **Day 7 (Feb 9):** Tasks 7.1-7.4 ✅ = Results section updated
- **Day 8 (Feb 10):** Tasks 8.1-8.3 ✅ = Quantitative results added
- **Day 9 (Feb 11):** Tasks 9.1-9.6 ✅ = Final polish complete
- **Day 10 (Feb 12):** 🎉 FINAL REVIEW & SUBMISSION

---

## 🔧 TOOLS AND RESOURCES

### LaTeX Compilation
```bash
# Compile paper (run from ACC_2026 directory)
pdflatex ACC_2026.tex
bibtex ACC_2026
pdflatex ACC_2026.tex
pdflatex ACC_2026.tex
```

### Word Count Check
```bash
# Target: ~3500-4000 words for 6-page IEEE conference paper
texcount -1 -sum -merge ACC_2026.tex
```

### Figure Generation
- Results figures: Use `analyze_results.py` to regenerate from batch JSON
- Plots: `matplotlib` with IEEE style (`ieee.mplstyle`)

### Reference Management
- BibTeX file: `bibliography.bib`
- Citation style: `IEEEtran`
- Search: Google Scholar, IEEE Xplore, arXiv

---

## 🎓 QUALITY ASSURANCE

### Self-Review Questions (Before Final Submission)
1. ✅ Can a reader understand the problem without domain knowledge?
2. ✅ Are all assumptions clearly stated before use?
3. ✅ Does every theorem have a clear statement and some justification?
4. ✅ Are all figures referenced in text and clearly captioned?
5. ✅ Do results quantitatively validate each theorem?
6. ✅ Is the paper within page limit?
7. ✅ Are all references formatted correctly?
8. ✅ Is notation consistent throughout?

### External Checks
- [ ] Run through Grammarly or similar (grammar/clarity)
- [ ] Have advisor review Sections II.B (assumptions) and III.C (theorems)
- [ ] Verify all simulation data matches batch results JSON
- [ ] Check arxiv for very recent related papers (Jan-Feb 2026)

---

## 📝 NOTES AND CAVEATS

### Known Limitations to Address in Paper
1. **2D environment:** Acknowledge in Sec. IV, state 3D extension is straightforward
2. **Simplified disturbance model:** Bounded $d_i$ is abstraction; real underwater has complex turbulence
3. **Proof sketches only:** Full proofs in tech report/appendix (if space constrained)
4. **Simulation validation:** Hardware experiments future work

### Potential Reviewer Pushback Points (Prepare Responses)
- **"Proofs too informal"** → Point to detailed sketches + state full proofs available
- **"Limited hardware validation"** → Emphasize simulation realism (125 runs, realistic flow/comms)
- **"Comparison to Yang 2023 unclear"** → Fig. 4 directly compares, clarify in text
- **"Flow model over-simplified"** → Acknowledge, cite Berlinger/Quattrini Li for realism

---

## 🚀 QUICK START GUIDE

### If Starting Today (Feb 3, 2026)
1. **First 2 hours:** Complete Tasks 1.1-1.3 (Section II.A-B rewrite)
2. **Next 2 hours:** Complete Tasks 1.4-1.5 (formal definitions + problem statement)
3. **Evening:** Start Task 2.1 (literature review - gather 5 papers)
4. **Tomorrow:** Follow Day 2 schedule

### If Short on Time (Emergency 3-Day Plan)
- **Day 1:** All of Phase 1 (Tasks 1.1-3.3) → Foundation complete
- **Day 2:** All of Phase 2 (Tasks 4.1-6.3) → Theorems complete
- **Day 3:** Phases 3+4 combined (Tasks 7.1-9.6) → Results + polish

---

## ✅ FINAL SUBMISSION CHECKLIST

- [ ] PDF compiles without errors
- [ ] All figures embedded and visible
- [ ] Page limit satisfied (≤6 pages)
- [ ] References formatted (IEEEtran style)
- [ ] Author names/affiliations correct
- [ ] Abstract within word limit (~150-200 words)
- [ ] No TODO or \xyu{} comments remaining
- [ ] Acknowledgments section present
- [ ] Copyright notice (if required by ACC)
- [ ] Supplementary materials prepared (if any):
  - [ ] Extended proofs document
  - [ ] Simulation code repository link
  - [ ] Video demonstration link verified

---

## 📧 COMMUNICATION PLAN

### Advisor Check-Ins
- **Day 3 (Feb 5):** Share Section II draft (assumptions + problem formulation)
- **Day 6 (Feb 8):** Share Section III draft (with theorems)
- **Day 9 (Feb 11):** Share complete draft for final review

### Co-Author Reviews
- **Geoff Hollinger:** Review underwater robotics framing (Sec. I, II.A)
- **Xi Yu:** Review theoretical content (Sec. III.C theorems)
- **Both:** Final review on Day 10

---

**END OF MIGRATION TODO**

*This document is a living guide. Update task statuses as you progress. Good luck!* 🎯
