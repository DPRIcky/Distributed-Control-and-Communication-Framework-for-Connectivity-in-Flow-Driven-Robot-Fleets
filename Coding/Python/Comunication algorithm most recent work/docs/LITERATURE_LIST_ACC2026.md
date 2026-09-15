# TARGETED LITERATURE LIST — ACC 2026 Revision
## 8–12 Key Papers with Usage Notes

---

## MUST-READ TODAY (Critical for Proofs)

### 1. **Control Barrier Functions: Theory and Applications**
**Citation:** A. D. Ames, S. Coogan, M. Egerstedt, G. Notomista, K. Sreenath, and P. Tabuada, "Control barrier functions: Theory and applications," in *2019 18th European Control Conference (ECC)*, 2019, pp. 3420–3431.

**What I'm using this for:**
- **Theorem T1 (Step 6):** Standard CBF invariance theorem: if $\dot{h} + \alpha h \geq 0$ and $h(0) \geq 0$, then $h(t) \geq 0$ for all $t \geq 0$
- **Section III-B:** Formal definition of CBF and class-$\mathcal{K}$ functions
- **Terminology:** "Hard CBF constraints" vs. "soft CLF constraints" (addresses R2-3)

**Status:** Already cited in current ACC draft (Ref [25])

---

### 2. **Connectivity Maintenance: Global and Optimized Approach through Control Barrier Functions**
**Citation:** B. Capelli and L. Sabattini, "Connectivity maintenance: Global and optimized approach through control barrier functions," in *2020 IEEE International Conference on Robotics and Automation (ICRA)*, 2020, pp. 5590–5596.

**What I'm using this for:**
- **Theorem T1:** CBF formulation for connectivity (distance-based range constraints)
- **Literature review (R2-4):** Centralized CBF connectivity methods (Fiedler value approach) — contrast with our distributed method
- **Proof technique:** How to handle communication range as a barrier constraint

**Status:** Already cited in current ACC draft (Ref [14])

---

### 3. **A Distributed Method for Detecting Critical Edges and Increasing Edge Connectivity in Undirected Networks**
**Citation:** D. B. Venkateswaran, Z. Qu, and A. Gusrialdi, "A distributed method for detecting critical edges and increasing edge connectivity in undirected networks," in *2024 IEEE 63rd Conference on Decision and Control (CDC)*, 2024, pp. 6951–6956.

**What I'm using this for:**
- **Theorem T3 (Step 1):** Lemma 1 from this paper: distributed critical-edge detection characterization
- **Theorem T4:** Justification for "critical edge set $C$ forms a spanning subgraph"
- **Section III-A:** Cite as foundation for our pruning method

**Status:** Already cited in current ACC draft (Ref [24])

---

## ADDITIONAL PAPERS FOR COMPLETENESS (R2-4: More References)

### 4. **Decentralized Connectivity Maintenance for Cooperative Control of Mobile Robotic Systems**
**Citation:** L. Sabattini, N. Chopra, and C. Secchi, "Decentralized connectivity maintenance for cooperative control of mobile robotic systems," *The International Journal of Robotics Research*, vol. 32, no. 12, pp. 1411–1423, 2013.

**What I'm using this for:**
- **Literature review:** Distributed connectivity maintenance (contrast with our approach)
- **Theorem T3 background:** Existing work on maintaining connectivity with local information
- **Section I introduction:** Cite as related work on distributed multi-robot coordination

**Status:** Already cited in current ACC draft (Ref [15])

---

### 5. **Decentralized Connectivity Maintenance with Time Delays Using Control Barrier Functions**
**Citation:** B. Capelli, H. Fouad, G. Beltrame, and L. Sabattini, "Decentralized connectivity maintenance with time delays using control barrier functions," in *2021 IEEE International Conference on Robotics and Automation (ICRA)*, 2021, pp. 1586–1592.

**What I'm using this for:**
- **Theorem T1:** Robustness analysis for CBF under delays (analogous to our disturbance term)
- **Literature review (R2-4):** Distributed pairwise CBF methods
- **Future work:** Mention time delays as extension (communication delays in underwater settings)

**Status:** Already cited in current ACC draft (Ref [16])

---

### 6. **Minimally Constrained Multi-Robot Coordination with Line-of-Sight Connectivity Maintenance**
**Citation:** Y. Yang, Y. Lyu, and W. Luo, "Minimally constrained multi-robot coordination with line-of-sight connectivity maintenance," *arXiv preprint arXiv:2303.04271*, 2023.

**What I'm using this for:**
- **Section I introduction:** Cite as closest related work (MST + CBF hybrid framework)
- **Comparison:** Our method is distributed MST construction vs. their centralized MST
- **Theorem T3:** Contrast with their approach (they assume global MST knowledge)

**Status:** Already cited in current ACC draft (Ref [17])

---

### 7. **A Distributed Algorithm for Minimum-Weight Spanning Trees**
**Citation:** R. G. Gallager, P. A. Humblet, and P. M. Spira, "A distributed algorithm for minimum-weight spanning trees," *ACM Transactions on Programming Languages and Systems (TOPLAS)*, vol. 5, no. 1, pp. 66–77, 1983.

**What I'm using this for:**
- **Section I introduction:** Classical distributed MST (GHS algorithm) — cite for "communication-intensive" claim
- **Literature review:** Contrast with our pruning approach (we avoid GHS complexity)
- **Theorem T4 discussion:** Mention that exact distributed MST is possible but costly

**Status:** Already cited in current ACC draft (Ref [21])

---

### 8. **Implicit Coordination for 3D Underwater Collective Behaviors in a Fish-Inspired Robot Swarm**
**Citation:** F. Berlinger, M. Gauci, and R. Nagpal, "Implicit coordination for 3d underwater collective behaviors in a fish-inspired robot swarm," *Science Robotics*, vol. 6, no. 50, p. eabd8668, 2021.

**What I'm using this for:**
- **Section I introduction (R2-4):** Recent underwater swarm robotics (adds credibility to application domain)
- **Section II-A:** Cite for stochastic dispersion behavior in underwater robots
- **Future work:** Mention implicit coordination as alternative to explicit CBF

**Status:** Already cited in current ACC draft (Ref [10])

---

### 9. **Diffusion by Continuous Movements**
**Citation:** G. I. Taylor, "Diffusion by continuous movements," *Proceedings of the London Mathematical Society*, vol. 2, no. 1, pp. 196–212, 1922.

**What I'm using this for:**
- **Section II-A:** Classic reference for advection-diffusion model
- **Assumption A2 justification:** Brownian motion model for turbulence
- **Historical context:** Foundational work on stochastic dispersion

**Status:** Already cited in current ACC draft (Ref [7])

---

### 10. **Consensus and Cooperation in Networked Multi-Agent Systems**
**Citation:** R. Olfati-Saber, J. A. Fax, and R. M. Murray, "Consensus and cooperation in networked multi-agent systems," *Proceedings of the IEEE*, vol. 95, no. 1, pp. 215–233, 2007.

**What I'm using this for:**
- **Section I introduction (R2-4):** Foundational multi-agent systems reference
- **Theorem T3 background:** Graph Laplacian and connectivity definitions
- **Literature review:** Cite for graph-theoretic methods in multi-robot control

**Status:** Already cited in current ACC draft (Ref [12])

---

## ADDITIONAL CITATIONS TO ADD (NEW)

### 11. **Lyapunov-Based Control of Robotic Systems**
**Citation:** M. Krstic, I. Kanellakopoulos, and P. V. Kokotovic, *Nonlinear and Adaptive Control Design*, Wiley-Interscience, 1995.

**What I'm using this for:**
- **Theorem T2 (Step 5):** Comparison lemma for Lyapunov stability analysis
- **Section III-B:** Justification for CLF-based controller design
- **R2-1 response:** Standard reference for Lyapunov-based control

**Status:** **NEW — add to references**

---

### 12. **Convex Optimization**
**Citation:** S. Boyd and L. Vandenberghe, *Convex Optimization*, Cambridge University Press, 2004.

**What I'm using this for:**
- **Theorem T2 (Step 3):** Projection onto convex feasible set (QP optimality conditions)
- **Section III-B:** Justification for QP formulation
- **Proof rigor:** Mathematical foundation for optimization-based control

**Status:** **NEW — add to references**

---

## LITERATURE READING PLAN (FOR TODAY)

### High Priority (Read First — 30 min each)

1. **Ames et al. 2019** → Extract exact CBF theorem statement for T1 proof
2. **Venkateswaran et al. 2024** → Get Lemma 1 for critical-edge characterization (T3)
3. **Capelli & Sabattini 2020** → Understand connectivity CBF formulation (T1)

### Medium Priority (Skim — 15 min each)

4. **Sabattini et al. 2013** → Distributed connectivity methods (literature review)
5. **Yang et al. 2023** → MST + CBF hybrid approach (comparison)
6. **Gallager et al. 1983** → GHS distributed MST (contrast our method)

### Low Priority (Reference Only — already familiar)

7. **Berlinger et al. 2021** → Underwater swarms (application context)
8. **Taylor 1922** → Diffusion model (classical reference)
9. **Olfati-Saber et al. 2007** → Multi-agent consensus (foundational)

### New Additions (Quick Lookup)

10. **Krstic et al. 1995** → Comparison lemma (T2 proof, Chapter 4)
11. **Boyd & Vandenberghe 2004** → Convex projection (T2 proof, Section 5.1)

---

## EXTRACTION CHECKLIST (What to Pull from Each Paper)

| **Paper** | **Extract** | **Use In** |
|----------|-----------|-----------|
| Ames 2019 | "If $\dot{h} + \alpha h \geq 0$ and $h(0) \geq 0$, then $h(t) \geq 0$" | T1 proof, Step 6 |
| Venkateswaran 2024 | "Lemma 1: An edge is critical iff removing it disconnects the graph" | T3 proof, Step 1 |
| Capelli 2020 | Connectivity barrier: $h = R_{\max}^2 - \|x_i - x_j\|^2$ | T1 statement |
| Krstic 1995 | "Comparison lemma for $\dot{V} \leq -\alpha V + \gamma$" | T2 proof, Step 5 |
| Boyd 2004 | "Projection onto convex set minimizes distance" | T2 proof, Step 3 |

---

## CITATION USAGE SUMMARY (WHERE TO ADD IN PAPER)

### Section I (Introduction)
- Add: Krstic 1995 (Lyapunov control), Boyd 2004 (optimization)
- Context: "Lyapunov-based methods [Krstic 1995] combined with optimization [Boyd 2004] form the foundation for CLF-CBF frameworks [Ames 2019]"

### Section II-A (Model)
- Already cited: Taylor 1922, Berlinger 2021
- Add context: "This advection-diffusion model is well-established [Taylor 1922] and aligns with observations in underwater robotics [Berlinger 2021]"

### Section III-A (Pruning)
- Already cited: Venkateswaran 2024, Gallager 1983
- Add context: "While exact distributed MST algorithms exist [Gallager 1983], they require extensive message passing. Our pruning method [Venkateswaran 2024] achieves similar results with minimal communication"

### Section III-B (CLF-CBF)
- Already cited: Ames 2019, Capelli 2020, Yang 2023
- Add: Krstic 1995, Boyd 2004
- Context: "The CLF-CBF framework [Ames 2019] leverages Lyapunov stability [Krstic 1995] and convex optimization [Boyd 2004] to guarantee safety and goal convergence"

### New Theorem Section (if added)
- T1: Cite Ames 2019, Capelli 2020
- T2: Cite Krstic 1995, Boyd 2004
- T3: Cite Venkateswaran 2024, Olfati-Saber 2007
- T4: Cite Gallager 1983 (as contrast)

---

## PASS CONDITION ✅

**Can you paste this list into your notes and immediately:**
1. ✅ See which papers to read today and in what order?
2. ✅ Know exactly what to extract from each paper?
3. ✅ Know where to cite each paper in the revision?

**Answer:** Yes. This literature list is **locked and ready for today's reading session**.

---

**End of Deliverable #4**
