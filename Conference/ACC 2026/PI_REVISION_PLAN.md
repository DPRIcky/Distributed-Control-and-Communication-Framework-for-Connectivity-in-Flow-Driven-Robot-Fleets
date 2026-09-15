# ACC 2026 Revision Plan for PI Review

**From:** Prajjwal Dutta  
**Date:** January 23, 2026  
**Re:** ACC 2026 Reviewer Comments - Concrete Revision List & Timeline

---

## Executive Summary

I have reviewed the ACC 2026 reviewer feedback in detail. The paper received **rejection with 8 specific comments** (4 major concerns from Reviewer 18, 4 constructive suggestions from Reviewer 25). 

**Good news:** I have already completed significant theoretical work in a separate document (DISTRIBUTED_VALIDATION.md) that addresses approximately **70% of the reviewer concerns**. With focused effort on transferring this work to the paper and adding new experiments, we can address all issues.

**Timeline estimate:** 2-3 weeks for complete revision (details below).

---

## Reviewer Comments Summary

### Reviewer 18 (ID: 10607) - Major Concerns ⚠️ REJECTION DRIVERS

| # | Comment | Severity | Our Status |
|---|---------|----------|------------|
| 1 | Models in Section II-A and II-B are different | High | ✅ Solution ready |
| 2 | **Flow term same for all robots → not challenging** | **CRITICAL** | ✅ Rebuttal ready |
| 3 | Problem statement not in rigorous math | High | ⚠️ Need to formalize |
| 4 | **No proof showing method solves the problem** | **CRITICAL** | ✅ **Theorems already written!** |

### Reviewer 25 (ID: 14961) - Constructive Feedback

| # | Comment | Severity | Our Status |
|---|---------|----------|------------|
| 1 | Need convergence or stability guarantees | High | ✅ Theorems ready |
| 2 | Meaning of dmin unclear (page 4) | Low | ⚠️ Simple fix |
| 3 | Unclear where Lyapunov function is used | Medium | ⚠️ Need to clarify |
| 4 | More relevant references needed | Medium | ⚠️ Need to add 10-15 |

---

## Key Finding: Existing Theoretical Work (DISTRIBUTED_VALIDATION.md)

**I have already completed substantial theoretical analysis** in a working document that I haven't shared yet. This document contains:

### ✅ **Three Formal Theorems with Proofs:**

1. **Theorem 1 (Safety/Connectivity Preservation):**
   - Statement: If robots prune edge e only when λ₂(L \ e) ≥ λ_ref, graph remains connected
   - Proof: Uses Fiedler theorem (λ₂ > 0 ⟺ connected)
   - **This directly addresses Reviewer 18's Comment 4!**

2. **Theorem 2 (Liveness):**
   - Statement: If consensus converges and redundant edges exist, algorithm will identify and prune them unanimously
   - Proof: Consensus guarantee → all robots extract same edges → unanimous decision
   - **Proves algorithm terminates correctly**

3. **Theorem 3 (Asymptotic Optimality):**
   - Statement: Algorithm produces sparse graph with m ≥ n-1 edges maximizing λ₂ subject to threshold
   - Proof sketch: Greedy pruning of longest edges is locally optimal
   - **Addresses efficiency concern**

### ✅ **Complete Distributed Algorithm:**
- Step-by-step pseudocode (initialization, consensus, edge detection, voting)
- Complexity analysis: O(n³) per pruning decision
- Convergence guarantees from Griparic et al. extended to our setting

### ✅ **Novelty Justification:**
- Explains why spatially-varying flow IS challenging (vortices create differential advection)
- Distinguishes energy-constrained selective activation vs continuous control
- Shows distributed solution without global knowledge

---

## Concrete Revision List

### **CATEGORY A: Transfer Existing Theory to Paper** (High Priority, ~10 hours)

#### 1. Add Section III-C "Theoretical Analysis" 
**Content:** Copy 3 theorems from DISTRIBUTED_VALIDATION.md with complete proofs  
**Addresses:** Reviewer 18 Comment 4 (CRITICAL), Reviewer 25 Comment 1  
**Effort:** 3 hours (mostly LaTeX formatting of existing content)  
**Status:** ✅ Content ready, needs formatting

#### 2. Add Formal Problem 1 Statement (Section II-C)
**Content:** Mathematical formulation with:
- State space: X ∈ R^(2N)
- Control constraints: U, ||ui|| ≤ umax
- Connectivity constraint (C1): Graph remains connected
- Safety (C2), Communication (C3), Distributed (C4)
- Objective: Minimize J = ∫||u||²dt

**Addresses:** Reviewer 18 Comment 3  
**Effort:** 2 hours  
**Status:** ⚠️ Need to write

#### 3. Fix Flow Spatial Variation (Section II-A)
**Change:** fflow(x̄(t),t) → fflow(xi(t),t)  
**Add:** "Flow field varies spatially due to vortices, creating differential advection that must be actively countered"  
**Addresses:** Reviewer 18 Comment 2 (CRITICAL - novelty challenge)  
**Effort:** 30 minutes  
**Status:** ⚠️ Quick fix

#### 4. Add Remark 1 Clarifying Models (After Eq. 1)
**Content:** "Section II-A is physical model for simulation; Section II-B is control design model. Standard separation in distributed robotics."  
**Addresses:** Reviewer 18 Comment 1  
**Effort:** 30 minutes  
**Status:** ⚠️ Quick fix

#### 5. Add CLF-CBF QP Formulation (Section III-B)
**Content:** Show explicit QP robots solve with CLF constraint V̇ + αV ≤ γ and CBF constraints  
**Addresses:** Reviewer 25 Comment 3  
**Effort:** 2 hours  
**Status:** ⚠️ Need to write

#### 6. Define dmin Parameter (Section II-B)
**Content:** "dmin > 0 is minimum separation distance for collision avoidance. Typically dmin < Rmax."  
**Addresses:** Reviewer 25 Comment 2  
**Effort:** 15 minutes  
**Status:** ⚠️ Trivial fix

---

### **CATEGORY B: Expand Literature Review** (Medium Priority, ~5 hours)

#### 7. Find and Add 10-15 New References
**Categories:**
- Underwater robotics & AUVs (3-4 papers): Leonard 2001, Paull 2014, Yuh 2000, etc.
- Distributed graph algorithms (3-4 papers): Mesbahi 2010, Zavlanos 2007, Ji 2007, etc.
- Energy-efficient multi-robot (2-3 papers): Gao 2020, Nowzari 2019, etc.
- CBF/CLF advanced theory (2-3 papers): Xu 2015, Cortés 2008, etc.

**Addresses:** Reviewer 25 Comment 4  
**Effort:** 3 hours (finding + reading abstracts + citing)  
**Status:** ⚠️ Need to find papers

#### 8. Expand Related Work Section (Section I)
**Content:** 
- Add 3-4 paragraphs positioning work against new references
- Create Table 1: Comparison with existing methods [14], [15], [17]
  - Columns: Distributed? Energy-Aware? Flow-Exploiting? Connectivity Guarantee?

**Addresses:** Better positioning of contributions  
**Effort:** 2 hours  
**Status:** ⚠️ Need to write

---

### **CATEGORY C: New Experimental Validation** (High Priority, ~20 hours)

Current experiments (Fig 2-5) show qualitative results and MST comparison. Need quantitative metrics:

#### 9. Quantitative Connectivity Metrics
**Experiment:** Track λ₂(Laplacian) over time for all simulation runs  
**Metric:** Show λ₂(t) ≥ λmin = 0.1 always (validates Theorem 1)  
**Baseline:** Compare with no-pruning (dense graph maintains higher λ₂)  
**Addresses:** Reviewer 25 Comment 1 (stability guarantees)  
**Effort:** 4 hours (code + plot + analysis)  
**Status:** ⚠️ Need to implement

#### 10. Energy Efficiency Comparison
**Experiment:** Measure total control effort E = ∫₀ᵀ Σᵢ||ui(t)||²dt  
**Baselines:**
1. Continuous control (always-on connectivity maintenance)
2. No pruning (CBF on all edges)
3. Pure drift (no control, loses connectivity)

**Expected Result:** 60-80% energy savings vs continuous control  
**Addresses:** Energy efficiency claim in contribution  
**Effort:** 6 hours (implement baselines + run experiments + plot)  
**Status:** ⚠️ Need to implement

#### 11. Scalability Analysis
**Experiment:** Vary fleet size N = 5, 10, 15, 20, 30, 50  
**Metrics:**
- Computation time per pruning decision
- Communication overhead (messages exchanged)
- Success rate (connectivity maintained)

**Expected Result:** Linear/polynomial scaling (distributed advantage)  
**Addresses:** Scalability of distributed approach  
**Effort:** 5 hours (parameter sweep + data collection + plot)  
**Status:** ⚠️ Need to implement

#### 12. Robustness Tests
**Experiment:** Test against realistic failures  
**Scenarios:**
1. Communication dropouts (10%, 20%, 30% packet loss)
2. Flow prediction error (±10%, ±20% uncertainty)
3. Robot failures (remove 1-2 robots mid-mission)

**Metric:** Graceful degradation vs catastrophic failure  
**Addresses:** Practical viability of method  
**Effort:** 5 hours (implement failure modes + run tests + plot)  
**Status:** ⚠️ Need to implement

---

### **CATEGORY D: Documentation** (Low Priority, ~3 hours)

#### 13. Point-by-Point Response to Reviewers
**Content:** Map each comment to specific changes:
```
Reviewer 18, Comment 1 → Fixed by Remark 1 (Sec II-A, line XX)
Reviewer 18, Comment 2 → Fixed by flow notation change (Sec II-A) + strengthened intro
Reviewer 18, Comment 3 → Fixed by Problem 1 (Sec II-C)
Reviewer 18, Comment 4 → Fixed by Theorems 1-3 (new Sec III-C)
Reviewer 25, Comment 1 → Fixed by Theorems 1-3 + Experiment 9
...
```

**Effort:** 2 hours  
**Status:** ⚠️ After all changes complete

#### 14. Update Figures and Captions
**Changes:**
- Existing Fig 2-5: Update captions to reference theorems
- New Fig 6: λ₂ over time (Experiment 9)
- New Fig 7: Energy comparison (Experiment 10)
- New Fig 8: Scalability results (Experiment 11)
- New Fig 9: Robustness tests (Experiment 12)
- New Table 1: Method comparison (Related work)

**Effort:** 1 hour  
**Status:** ⚠️ After experiments complete

---

## Proposed Timeline

### Week 1 (Jan 23-29): Theory & Problem Formulation
- **Days 1-2:** Transfer theorems to Section III-C (Item 1)
- **Day 3:** Add Problem 1 + fix flow notation + Remark 1 (Items 2-4)
- **Day 4:** Add CLF-CBF formulation + define dmin (Items 5-6)
- **Days 5-7:** Find references + expand related work (Items 7-8)

**Deliverable:** Draft with all theoretical content added

### Week 2 (Jan 30 - Feb 5): Experiments
- **Days 1-2:** Implement connectivity + energy experiments (Items 9-10)
- **Days 3-4:** Implement scalability + robustness experiments (Items 11-12)
- **Days 5-7:** Generate figures, analyze results, write new subsections

**Deliverable:** Complete experimental section

### Week 3 (Feb 6-12): Polish & Submit
- **Days 1-2:** Point-by-point response document (Item 13)
- **Day 3:** Update all figures and captions (Item 14)
- **Days 4-5:** Full paper review, consistency check, proofreading
- **Day 6:** Internal review (send to you for feedback)
- **Day 7:** Finalize based on your feedback

**Deliverable:** Submission-ready paper

---

## Resource Requirements

### Computational:
- Experiments 9-12 will require ~100-150 simulation runs
- Estimated compute time: 20-30 hours on workstation
- Can parallelize across multiple cores

### References:
- Need library access for 10-15 papers (IEEE Xplore, ACM Digital Library)
- Most should be accessible through ASU subscriptions

### Your Input Needed:
1. **Approval of timeline** - Is 3 weeks reasonable? Or should we target specific deadline?
2. **Experiment priorities** - Which experiments are most critical? (I recommend 9 & 10)
3. **Submission venue** - Resubmit to ACC 2027? Or target journal (T-RO, Automatica)?
4. **Theoretical depth** - Are Theorems 1-3 sufficient, or do you want more formal proofs?

---

## Risk Assessment

### Low Risk:
- ✅ Category A (theory transfer): Content exists, just needs formatting
- ✅ Items 3-6 (quick fixes): Straightforward edits

### Medium Risk:
- ⚠️ Category B (references): Finding good papers may take time
- ⚠️ Experiments 9-10: Implementation should be straightforward using existing code

### High Risk:
- ⚠️ Experiments 11-12 (scalability/robustness): May reveal unexpected issues that need addressing
- ⚠️ Reviewer 18 Comment 2 (novelty): Need strong rebuttal; may need additional argumentation

### Mitigation:
- Start with high-priority items (Theorems, Problem 1, Experiments 9-10)
- Can submit partial revision if time-constrained (defer scalability experiments)
- Schedule check-in meeting after Week 1 to assess progress

---

## Questions for Discussion

1. **Scope:** Should we aim for full revision (all items 1-14) or prioritize subset for faster turnaround?

2. **Venue:** 
   - Option A: ACC 2027 (submission ~Sept 2026) - allows leisurely revision
   - Option B: Journal (T-RO, Automatica, T-ASE) - longer format, more experimental validation expected
   - Option C: IROS/CDC 2026 (deadline ~March 2026) - aggressive timeline (4-6 weeks)

3. **Theoretical rigor:** Are the 3 theorems sufficient, or should I develop more formal stability analysis (e.g., global Lyapunov function for formation)?

4. **Experiments:** Which metrics are most important to reviewers in your opinion?

5. **Authorship:** Should we consider adding collaborators for experimental validation or theoretical refinement?

---

## Immediate Next Steps (Awaiting Your Approval)

1. **Today:** Send you this plan for feedback
2. **Upon approval:** Start with Item 1 (Section III-C theorems) - highest priority
3. **This weekend:** Complete Items 2-6 (theory transfer + quick fixes)
4. **Next week:** Begin experiments

Please let me know your thoughts on:
- Timeline feasibility
- Experiment priorities  
- Submission venue preference
- Any concerns about the proposed revisions

I'm confident we can address all reviewer concerns with the work already done plus focused effort on experiments and polish.

---

**Attachments:**
1. REVISION_PROGRESS_REPORT.md - Detailed analysis of what's done vs needed
2. QUICK_ACTION_ITEMS.md - Copy/paste LaTeX code for paper edits
3. DISTRIBUTED_VALIDATION.md - Existing theoretical work (theorems, proofs, algorithms)

Ready to proceed upon your approval.

Best regards,  
Prajjwal
