# LITERATURE SEARCH KEYWORDS FOR NOVELTY VERIFICATION
## ACC 2026 Submission - Before Finalizing Theorem 1

**Date:** January 28, 2026  
**Purpose:** Identify potentially competing/similar work to ensure proper citation and novelty claims

---

## PRIMARY SEARCH STRINGS (Google Scholar)

### **Category 1: CBF + Spatially-Varying Drift**

**Search String 1 (Most Specific):**
```
"control barrier function" "spatially varying" (drift OR flow OR disturbance)
```
**What to look for:**  
Papers explicitly addressing CBF under non-uniform drift fields

---

**Search String 2:**
```
"control barrier function" "Lipschitz" flow multi-robot
```
**What to look for:**  
CBF papers that exploit Lipschitz continuity of drift terms

---

**Search String 3:**
```
CBF connectivity "time-varying disturbance" (robots OR agents)
```
**What to look for:**  
Works on connectivity maintenance under time-varying (possibly spatially varying) perturbations

---

### **Category 2: Multi-Robot Control in Flow Fields**

**Search String 4:**
```
multi-robot connectivity "ocean current" OR "flow field" control
```
**What to look for:**  
Marine/underwater robotics papers addressing connectivity under currents

---

**Search String 5:**
```
formation control "spatially varying disturbance" (safety OR collision avoidance)
```
**What to look for:**  
Formation control papers with non-uniform environmental forces

---

**Search String 6:**
```
"relative dynamics" "differential drift" multi-agent CBF
```
**What to look for:**  
Papers analyzing relative coordinates under differential disturbances

---

### **Category 3: Barrier Functions + Lipschitz Dynamics**

**Search String 7:**
```
"barrier function" "Lipschitz continuous" (drift OR dynamics)
```
**What to look for:**  
General CBF theory papers handling Lipschitz perturbations

---

**Search String 8:**
```
"control barrier function" "bounded disturbance" multi-robot connectivity
```
**What to look for:**  
Multi-robot CBF works with disturbance robustness

---

### **Category 4: Underwater/Marine Robotics**

**Search String 9:**
```
underwater (AUV OR robot) connectivity "ocean current" (CBF OR barrier)
```
**What to look for:**  
Marine robotics papers using barrier methods in currents

---

**Search String 10:**
```
"spatially varying flow" formation control underwater multi-robot
```
**What to look for:**  
Formation control in non-uniform aquatic environments

---

### **Category 5: Distributed Consensus + Graph Connectivity**

**Search String 11:**
```
distributed consensus "critical edge" detection multi-robot
```
**What to look for:**  
Distributed algorithms for identifying important graph edges

---

**Search String 12:**
```
"algebraic connectivity" CBF (maintenance OR preservation) multi-agent
```
**What to look for:**  
Connectivity maintenance using λ₂ and barrier methods

---

## SPECIFIC AUTHOR/GROUP SEARCHES

### **Known Relevant Authors (Check Recent Works)**

1. **Aaron Ames** (Caltech) - CBF pioneer
   - Search: `Aaron Ames "control barrier function" 2022..2025`
   - Why: May have extended CBF theory to spatially-varying settings

2. **Magnus Egerstedt** (UC Irvine) - Multi-robot connectivity expert
   - Search: `Magnus Egerstedt connectivity multi-robot barrier 2020..2025`
   - Why: Leader in connectivity maintenance for multi-agent systems

3. **Paulo Tabuada** (UCLA) - CBF theory
   - Search: `Paulo Tabuada barrier function disturbance 2020..2025`
   - Why: Deep theoretical CBF contributions

4. **Koushil Sreenath** (UC Berkeley) - CBF applications
   - Search: `Koushil Sreenath CBF multi-robot 2020..2025`
   - Why: Applied CBF work, may have multi-robot extensions

5. **Naomi Leonard** (Princeton) - Marine robotics, flow fields
   - Search: `Naomi Leonard ocean current multi-robot coordination 2018..2025`
   - Why: Expert in underwater swarms in ocean currents

6. **Spring Berman** (ASU/Santa Barbara) - Robotic swarms
   - Search: `Spring Berman connectivity disturbance multi-robot 2020..2025`
   - Why: Multi-robot systems with environmental perturbations

7. **Dimitra Panagou** (Michigan) - CBF multi-agent
   - Search: `Dimitra Panagou barrier function multi-agent 2020..2025`
   - Why: Multi-agent CBF research

8. **Michael Zavlanos** (Duke) - Graph connectivity control
   - Search: `Michael Zavlanos connectivity maintenance multi-robot 2020..2025`
   - Why: Leader in connectivity-constrained control

---

## CONFERENCE/JOURNAL TARGETED SEARCHES

### **High-Impact Venues (2020–2025)**

**IEEE CDC (Conference on Decision and Control):**
```
site:ieeexplore.ieee.org "CDC" "control barrier" connectivity 2020..2025
```

**ACC (American Control Conference):**
```
site:ieeexplore.ieee.org "ACC" multi-robot connectivity flow 2020..2025
```

**IEEE Transactions on Robotics:**
```
site:ieeexplore.ieee.org "T-RO" OR "Transactions on Robotics" CBF multi-agent 2020..2025
```

**Automatica:**
```
site:sciencedirect.com Automatica "barrier function" disturbance 2020..2025
```

**IEEE Transactions on Automatic Control:**
```
site:ieeexplore.ieee.org "TAC" "Transactions on Automatic Control" barrier connectivity 2020..2025
```

**ICRA (International Conference on Robotics and Automation):**
```
site:ieeexplore.ieee.org ICRA multi-robot connectivity ocean 2020..2025
```

---

## NEGATIVE SEARCH (What We Are NOT)

To sharpen our novelty claim, verify that these combinations return **few or no results**:

**Negative Search 1:**
```
"control barrier function" "spatially varying flow" multi-robot connectivity
```
**Expected:** Very few (<5) directly relevant papers

**Negative Search 2:**
```
CBF "Lipschitz drift" "non-uniform disturbance" multi-agent
```
**Expected:** Minimal overlap with our exact problem

**Negative Search 3:**
```
"relative dynamics" "flow differential" barrier function multi-robot
```
**Expected:** Likely zero exact matches

---

## KEY DIFFERENTIATION POINTS (FOR NOVELTY CLAIMS)

When reviewing papers, check if they address **ALL** of the following:

1. **Spatially-Varying Drift:**  
   - [ ] Does the paper explicitly model $f(x_i, t) \neq f(x_j, t)$ (no cancellation)?
   - [ ] Or do they assume uniform drift $f(x_i, t) = f(x_j, t) = v_0(t)$?

2. **CBF-Based Connectivity:**  
   - [ ] Do they use barrier functions for connectivity maintenance?
   - [ ] Or do they use potential functions, optimization, or MPC?

3. **Multi-Robot Setting:**  
   - [ ] Is it truly distributed (each robot has only local info)?
   - [ ] Or centralized/leader-follower?

4. **Lipschitz Assumption:**  
   - [ ] Do they bound flow gradients explicitly?
   - [ ] Or assume bounded $\|f(x, t)\|$ globally (weaker)?

5. **Simultaneous Safety + Connectivity:**  
   - [ ] Do they address both collision avoidance AND connectivity preservation?
   - [ ] Or only one of the two?

6. **Distributed Graph Consensus:**  
   - [ ] Do they use adjacency matrix consensus for critical edge detection?
   - [ ] Or centralized MST/Laplacian computation?

---

## LIKELY RELATED WORKS (PRELIMINARY LIST TO CHECK)

Based on our domain knowledge, these papers are **highly likely** to be relevant. Check them first:

### **CBF + Multi-Robot Connectivity**

1. **Ames et al. (2019)** - "Control Barrier Functions: Theory and Applications"  
   *IEEE CDC Tutorial* - Foundational CBF reference (likely cited, but doesn't address spatially-varying drift)

2. **Notomista et al. (2020)** - "Persistification of Robotic Tasks Using Control Barrier Functions"  
   *IEEE TAC* - CBF for task persistence (may have connectivity examples)

3. **Panagou & Kumar (2015)** - "Cooperative Visibility Maintenance for Leader-Follower Formations"  
   *IEEE TRO* - Connectivity via potential fields (not CBF, but related objective)

4. **Zavlanos & Pappas (2007)** - "Potential Fields for Maintaining Connectivity"  
   *IEEE TRO* - Classic connectivity control (not CBF-based)

5. **Sabattini et al. (2013)** - "Decentralized Connectivity Maintenance for Cooperative Control"  
   *IJRR* - Distributed connectivity (graph-theoretic, not CBF)

### **Marine/Underwater Robotics in Currents**

6. **Leonard & Graver (2001)** - "Model-Based Feedback Control of Autonomous Underwater Gliders"  
   *IEEE JOE* - Ocean current modeling (classic reference, no CBF)

7. **Hsieh et al. (2008)** - "Experimental Validation of Fish-Inspired Robotic Formations"  
   *Robotica* - Formation control in flow (experimental, no CBF theory)

8. **Fiorelli et al. (2006)** - "Multi-AUV Control and Adaptive Sampling in Monterey Bay"  
   *IEEE JOE* - Ocean sampling (flow-aware, but not CBF/connectivity theory)

9. **Chen et al. (2020)** - "Distributed Formation Control of Underwater Vehicles with Communication Constraints"  
   *Ocean Engineering* - UUV formation (potential field methods, check if CBF used)

### **Barrier Functions + Disturbances**

10. **Xu et al. (2015)** - "Robustness of Control Barrier Functions for Safety Critical Control"  
    *IFAC Analysis and Design of Hybrid Systems* - CBF under bounded disturbances (single robot, not multi-agent)

11. **Kolathaya & Ames (2017)** - "Input-to-State Safety with Control Barrier Functions"  
    *IEEE CSL* - ISS-CBF theory (disturbance robustness, but not spatially-varying)

12. **Lindemann & Dimarogonas (2017)** - "Control Barrier Functions for Signal Temporal Logic Tasks"  
    *IEEE CDC* - CBF + STL (may have disturbance models)

### **Distributed Consensus + Graph Theory**

13. **Venkateswaran et al. (2024)** - "Distributed Detection of Critical Edges..."  
    *IEEE TAC* (cited in our work) - Our baseline for critical edge detection

14. **Zelazo et al. (2011)** - "Decentralized Formation Control with Connectivity Maintenance"  
    *IEEE TAC* - Connectivity via Laplacian estimation (not CBF)

---

## SEARCH TIMELINE RECOMMENDATION

**Phase 1 (1–2 hours): Quick Scans**
- Run Search Strings 1–6 (most specific)
- Check titles/abstracts for exact matches
- Download any papers that claim "CBF + spatially-varying drift"

**Phase 2 (2–3 hours): Author Deep Dive**
- Check recent papers (2022–2025) from key authors
- Look for preprints on arXiv: `arxiv.org/search/?query=control+barrier+function+flow`

**Phase 3 (1 hour): Venue Sweeps**
- Browse CDC 2023, 2024 proceedings (Table of Contents)
- ACC 2024, 2025 (if available)
- IEEE TRO/TAC latest issues

**Phase 4 (30 min): Citation Forward/Backward**
- For any highly relevant paper found:
  - Check "Cited by" (Google Scholar) for follow-ups
  - Check references for foundational works

---

## RED FLAGS (Papers to Scrutinize Carefully)

If you find a paper with **any** of these titles, read it thoroughly:

- **"Spatially Varying Drift + CBF"** → Potential direct overlap
- **"Lipschitz Flow + Multi-Robot Connectivity"** → Very close to our problem
- **"Relative Dynamics + Barrier Functions"** → May have similar derivative computations
- **"Distributed Critical Edge Detection + CBF"** → Combines our two main components
- **"Ocean Current + Multi-Robot + Barrier"** → Applied version of our work

---

## NOVELTY STATEMENT REFINEMENT (After Search)

Once you complete the search, we can refine our novelty claims. Expected outcomes:

**Scenario A: No close matches found**  
→ Novelty claim: *"To the best of our knowledge, this is the first work to combine CBF-based safety/connectivity with Lipschitz spatially-varying flow in a distributed multi-robot setting."*

**Scenario B: Some CBF + flow papers exist (but different assumptions)**  
→ Novelty claim: *"While [Ref X] addresses flow fields, they assume globally bounded drift $\|f(x,t)\| \leq M$, which does not capture spatial variation. In contrast, our Lipschitz assumption $\|f(x_i,t) - f(x_j,t)\| \leq L\|x_i - x_j\|$ explicitly bounds the flow gradient, enabling tighter control authority conditions (A4)."*

**Scenario C: Related work on connectivity in currents (non-CBF)**  
→ Novelty claim: *"Prior work on multi-robot connectivity in ocean currents [Ref Y] uses potential field methods without formal barrier-based guarantees. Our CBF approach provides certified safety (Theorem 1) with explicit bounds on required control effort."*

---

## GOOGLE SCHOLAR SEARCH TIPS

1. **Use date filters:** Add `2020..2025` to focus on recent work
2. **Exclude patents:** Add `-patent` to search string
3. **Include arXiv:** Check `arxiv.org` separately for preprints
4. **Boolean operators:** Use `AND`, `OR`, `NOT` (uppercase) for precision
5. **Exact phrases:** Use quotes `"control barrier function"` for exact match
6. **Wildcard:** Use `*` for variations (e.g., `spatiall* vary*` matches "spatially varying")

---

## RECOMMENDED SEARCH ORDER (Prioritized)

**Priority 1 (Do First):**
- Search Strings 1, 2, 4, 7 (most specific to our problem)
- Author searches: Ames, Panagou, Zavlanos (2023–2025 only)

**Priority 2 (If Time Permits):**
- Search Strings 3, 5, 6, 8
- Conference proceedings: CDC 2024, ACC 2025

**Priority 3 (Comprehensive):**
- All remaining search strings
- Full author bibliographies
- Citation chaining

---

## DOCUMENTATION TEMPLATE (As You Search)

For each relevant paper found, record:

```markdown
### Paper Title
- **Authors:** 
- **Venue/Year:** 
- **Key Contribution:** 
- **Overlap with our work:** (High/Medium/Low)
- **Differences:** 
  - [ ] Assumptions (e.g., uniform drift vs. Lipschitz)
  - [ ] Methods (e.g., MPC vs. CBF)
  - [ ] Setting (e.g., single robot vs. multi-robot)
- **Citation action:** (Must cite / Should cite / Optional)
```

---

## FINAL CHECKLIST BEFORE SUBMITTING THEOREM 1

After completing the literature search:

- [ ] No paper addresses **exactly** our problem (Lipschitz flow + CBF + distributed multi-robot)
- [ ] All closely related works are cited in Introduction/Related Work
- [ ] Novelty claims are precise and defensible
- [ ] Differences from prior work are clearly articulated (e.g., in remarks after Theorem 1)
- [ ] Any overlapping assumptions are acknowledged (e.g., "Following [Ref], we assume...")

---

**Good luck with the search! Report back with findings and we'll refine the novelty positioning accordingly.**

---

**End of Keyword Document**
