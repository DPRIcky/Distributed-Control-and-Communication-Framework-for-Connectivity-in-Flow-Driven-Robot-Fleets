# REVIEWER → FIX → THEOREM → SIMULATION ARTIFACT MATRIX
## ACC 2026 Revision Tracking Document

---

| **Reviewer ID** | **Specific Concern** | **Fix / Response** | **Theorem(s) Addressing** | **Simulation Artifact** | **Status** |
|----------------|---------------------|-------------------|-------------------------|------------------------|-----------|
| **R1-1** | Model confusion: paper uses different models (advection-diffusion vs. single-integrator) | **Unified model:** Single control-affine model $\dot{x}_i = f_{\text{flow}}(x_i, t) + u_i + d_i$ for all analysis. Simulation uses same model with concrete flow field instantiation. Add clear "Model Specialization" subsection. | N/A (foundational) | Section II-A revised with unified model; simulation parameters in Section IV explicitly show same dynamics | ✅ Locked |
| **R1-2** | Flow term appears trivial: "same flow for all robots" → cancels in relative dynamics | **Switch to spatially-varying flow:** $f_{\text{flow}}(x_i, t)$ depends on position $x_i$. Add Lipschitz assumption A3: $\|f_{\text{flow}}(x_i, t) - f_{\text{flow}}(x_j, t)\| \leq L\|x_i - x_j\|$. Show non-cancellation in relative dynamics identity. | **T1** (Robust CBF invariance with flow gradients) | Figure 2 shows flow field visualization with spatial variation; robot trajectories demonstrate non-trivial drift differences | ✅ Locked |
| **R1-3** | Statements not rigorous: informal descriptions of graph/constraints | **Formal definitions:** Add rigorous definitions for: <br>• Communication graph $G_t = (V, E_t)$ <br>• Safe set $\mathcal{C}_{\text{safe}} = \{(x_i, x_j) : \|x_i - x_j\|^2 \geq d_{\min}^2\}$ <br>• Connectivity set $\mathcal{C}_{\text{conn}} = \{(x_i, x_j) : \|x_i - x_j\|^2 \leq R_{\max}^2\}$ <br>• Critical edge set $C \subseteq E$ | **T1, T3** (use formal set definitions) | Algorithm 1 pseudocode with explicit set operations; Section III-A defines $C$ rigorously | ⏳ Draft ready |
| **R1-4** | No formal proofs provided | **Add theorem package T1–T4** with proof sketches (full proofs in appendix or tech report). Each theorem has: statement, assumptions, proof outline (2–4 bullets). | **T1, T2, T3, T4** (see detailed table below) | Simulation validates each theorem: T1→connectivity preserved (Fig. 2); T2→goal convergence (Fig. 5c/d); T3→graph connectivity metric (Fig. 4); T4→pruning correctness (Fig. 3) | ⏳ Drafting |
| **R2-1** | Stability/convergence not addressed | **Add CLF theorem (T2):** Prove practical stability toward goal under soft CLF constraint in QP. State explicitly: CLF drives convergence, CBF ensures safety. | **T2** (CLF-based goal convergence) | Figure 5c/d shows robot-target distance convergence; demonstrate bounded-input bounded-state stability | ⏳ Drafting |
| **R2-2** | $d_{\min}$ meaning unclear | **Clarify $d_{\min}$ definition:** "Minimum safety separation distance to avoid collisions." Tie to safety barrier $h_{ij}^{\text{safe}} = \|x_i - x_j\|^2 - d_{\min}^2 \geq 0$. State in Assumption A6: $d_{\min} < R_{\max} - \epsilon$. | **T1** (uses $d_{\min}$ in safety CBF) | Section II-B and IV simulation setup explicitly state $d_{\min} = 0.6$ m with $R_{\max} = 1.2$ m | ✅ Locked |
| **R2-3** | Lyapunov function usage unclear | **Explicit CLF in QP formulation:** Show CLF constraint $\dot{V}_i + \alpha V_i \leq \gamma$ in QP problem statement. Add plot of $V(t)$ vs. time showing decay. Clarify: **CLF is soft (relaxed via $\gamma$), CBF is hard (strict constraint)**. | **T2** (CLF theorem) | Add new figure: $V_i(t)$ trajectories for detecting robots showing exponential-like decay toward goal | 🔄 Simulation update needed |
| **R2-4** | Insufficient references | **Add 8–12 citations:** <br>• CBF theory foundations (Ames et al.) <br>• Underwater robotics in flows (Berlinger, Quattrini Li) <br>• Distributed connectivity (Sabattini, Capelli) <br>• Graph theory (Gallager MST) <br>• Critical edge detection (Venkateswaran et al.—already cited) | N/A (literature support) | References section expanded; literature cited in appropriate theorem statements and proofs | ⏳ Literature skim today |

---

## DETAILED THEOREM PACKAGE (T1–T4)

| **Theorem** | **Title** | **What It Proves** | **Key Assumptions Used** | **Proof Technique** | **Simulation Validation** |
|------------|-----------|-------------------|------------------------|-------------------|-------------------------|
| **T1** | Robust Invariance for Connectivity & Safety | Under CBF constraints, safe set $\mathcal{C}_{\text{safe}}$ and connectivity set $\mathcal{C}_{\text{conn}}$ are forward invariant despite flow gradients and disturbances | A1 (bounded control), A2 (bounded disturbance), A3 (Lipschitz flow), A4 (control dominance) | Barrier function derivative analysis: show $\dot{h}_{ij} + \alpha h_{ij} \geq 0$ is feasible using Lipschitz bound on $f_{\text{flow}}(x_i, t) - f_{\text{flow}}(x_j, t)$ | Figure 2b: no collisions ($d_{\min}$ preserved), no link breaks (connectivity preserved) throughout mission |
| **T2** | CLF-Based Goal Convergence | Robots with active goal assignment converge to within $\epsilon$-neighborhood of target; practical stability under soft CLF relaxation | A1, A2, A4, plus initial condition $V_i(0) < \infty$ | Lyapunov decrease: $\dot{V}_i \leq -\alpha V_i + \gamma$, with $\gamma$ bounded by QP cost weight → exponential convergence to residual ball | Figure 5c/d: robot-target distance decreases monotonically; minimum distance → 0 |
| **T3** | Global Connectivity via Spanning Backbone | If critical edge set $C \subseteq E$ is maintained and connected, then global graph $G_t$ remains connected for all $t \geq 0$ | A5 (initial connectivity), A6 (range-safety margin), plus pruning guards (Alg. 1) | Graph theory: spanning subgraph connectivity implies full graph connectivity; leverage distributed critical-edge detection (Venkateswaran et al. Lemma 1) | Figure 3: 11 edges for 12 robots = spanning tree; Figure 4: network never fragments (algebraic connectivity $\lambda_2(L) > 0$) |
| **T4** | Pruning Correctness (No False Disconnections) | Algorithm 1 never disconnects the graph; pruned edges are non-critical and have alternative paths | A6 (range margin), plus alternative-path condition, degree guard, next-step guard | Induction on batches: each batch removal preserves connectivity by (i) alternative path existence, (ii) vertex-disjoint batch (no cascading failures), (iii) recoup step as safety net | Figure 3: graph transitions from 6 to 4 edges without disconnection; Figure 4: error metric → 0 (pruned structure = MST, which is minimal connected) |

---

## CROSS-REFERENCE: REVIEWER → THEOREM → CODE/SIMULATION

| **Reviewer Concern** | **Addresses in Paper** | **Validates in Code** | **Shows in Simulation** |
|---------------------|----------------------|---------------------|------------------------|
| R1-2 (flow triviality) | T1 proof outline, Assumption A3 | `flow_field.py`: spatially-varying flow implementation | Figure 2: flow field arrows show position-dependent vectors |
| R2-1 (stability) | T2 statement + proof sketch | `hybrid_controller.py`: CLF constraint in QP | Figure 5c/d: distance convergence plots |
| R1-3 (rigor) | Formal definitions in Section II-B, T1/T3 statements | `topology.py`: graph operations with set notation | Algorithm 1 pseudocode matches implementation |
| R1-4 (no proof) | T1–T4 package with bullet proofs | `test_integration.py`: unit tests for theorem conditions | All figures validate theorem claims |

---

## PASS CONDITION ✅

**Can you paste this matrix into advisor meeting notes and immediately show:**
1. ✅ Which reviewer comment maps to which fix?
2. ✅ Which theorem addresses which concern?
3. ✅ Which simulation artifact validates each claim?

**Answer:** Yes. This table is **locked and ready for reuse** in paper revision and advisor discussions.

---

**End of Deliverable #2**
