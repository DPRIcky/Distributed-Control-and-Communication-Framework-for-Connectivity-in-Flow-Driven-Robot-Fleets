# Email to PI - ACC 2026 Revision Plan

**Subject:** ACC 2026 Reviewer Comments Analysis & Revision Plan

---

Dear [PI Name],

I have reviewed the ACC 2026 reviewer feedback and analyzed our submitted paper in detail. Here is my concrete list of required revisions and proposed plan.

## Summary of Reviews

**Reviewer 18 (ID: 10607)** - 4 Major Concerns (REJECTION DRIVERS)
1. Models in Section II-A and II-B are confusing/different
2. **CRITICAL:** "Flow term is same for all robots → connectivity not challenging" (challenges novelty)
3. Problem statement not in rigorous math
4. **CRITICAL:** No proof showing proposed method solves the problem

**Reviewer 25 (ID: 14961)** - 4 Constructive Comments
1. Need convergence or stability guarantees
2. Meaning of dmin parameter unclear (page 4)
3. Unclear where Lyapunov function is used
4. Need more relevant references in introduction

## Root Cause Analysis

After reading our paper critically through the reviewers' eyes, I identified the fatal flaws:

### 🔴 CRITICAL ISSUES (Must Fix):
1. **No theorems or proofs** - Paper is purely algorithmic with simulation validation only
2. **Novelty challenged** - Reviewer claims our flow affects all robots equally (this is WRONG but we didn't explain spatial variation clearly)
3. **Mathematical rigor missing** - Problem statement is prose, not formal mathematics

### 🟡 HIGH PRIORITY:
4. Model notation inconsistency (II-A: stochastic model, II-B: control model)
5. Lyapunov function defined but never used in any proof
6. No stability/convergence analysis

## Detailed Revision Plan

I've created three documents in the Conference/ACC 2026 folder:
1. **REVISION_PLAN.md** - Detailed task breakdown with timeline
2. **REVISION_SUMMARY_UPDATED.md** - Comprehensive analysis with specific fixes

### Required Changes by Priority:

**PRIORITY 1: Add Formal Proofs (Days 2-4)**
- **Theorem 1:** Connectivity preservation under distributed pruning (Algorithm 1)
- **Theorem 2:** CLF-CBF safety and connectivity guarantees
- **Theorem 3:** Convergence to target when detected
- New Section III-C "Theoretical Analysis" with complete proofs

**PRIORITY 2: Address Novelty Challenge (Days 2-3)**
- Fix Section II-A: Change fflow(x̄,t) → fflow(xi,t) to emphasize spatial variation
- Add explicit statement about vortex-induced differential flow
- Strengthen introduction: energy-constrained selective activation vs continuous control
- Add comparison table with existing methods [14-17]

**PRIORITY 3: Mathematical Rigor (Day 2)**
- Add formal "Problem 1" box with state space, constraints, objective function
- Define dmin clearly in three places
- Clarify CLF-CBF QP formulation

**PRIORITY 4: Enhanced Experiments (Days 4-5)**
- Quantitative connectivity metrics (track λ₂(t))
- Energy comparison with baselines (∫||u||²dt)
- Scalability tests (N = 5, 10, 20, 50)
- Robustness tests (communication failures, flow uncertainty)

**PRIORITY 5: Expanded Related Work (Days 3-4)**
- Add 10-15 references on underwater robotics, distributed algorithms, CBF theory
- Expand Section I with proper positioning

## Implementation Timeline

**Day 1 (Wed Jan 22):** ✅ Completed
- Reviewed all feedback
- Created detailed revision plan

**Day 2 (Thu Jan 23):** Mathematical Formulation
- Model clarification (Remark 1)
- Formal Problem 1
- Flow novelty rebuttal
- Draft Theorem 1

**Day 3 (Fri Jan 24):** Theoretical Analysis
- Complete Theorems 2-3 with proofs
- CLF-CBF formulation
- Find new references

**Day 4 (Sat Jan 25):** Implementation & Experiments
- Code validation for theorems
- Baseline implementations
- Run connectivity + energy experiments
- Expanded related work

**Day 5 (Sun Jan 26):** Finalization
- Complete all experiments
- Integrate changes into paper
- Point-by-point response to reviewers
- **Deliver to you for review**

## Deliverables (Sunday Jan 26 EOD)

1. **Updated paper draft** with:
   - 3 new theorems with complete proofs
   - Formal problem formulation
   - Clarified models and Lyapunov usage
   - 4 new experiment subsections
   - Expanded related work (+10-15 refs)

2. **Point-by-point response** to all 8 reviewer comments

3. **Code updates:**
   - Test files validating theorems
   - Baseline comparison implementations
   - Updated experiment scripts

4. **Submission strategy recommendation**

## Questions for Discussion

1. **Scope:** This is a major revision (~40% rewrite). Should we:
   - Target ACC 2027 (Sept deadline)?
   - Submit to journal (T-RO, Automatica) for longer format?
   - Target IROS/CDC 2026 (March deadlines)?

2. **Proofs:** Theorems 1-2 are straightforward. Theorem 3 (stability) may need your input on the Lyapunov approach. Can we schedule 30-min call on Day 3 (Friday) if needed?

3. **Priority:** Are you okay with the 5-day timeline? Any adjustments needed?

## My Assessment

**Good news:** All issues are addressable. The core contribution (distributed pruning + CLF-CBF) is sound.

**Challenge:** Reviewer 18's novelty concern is serious but based on misunderstanding. Our flow IS spatially varying (vortices), and energy-constrained control IS novel. We just didn't emphasize this clearly enough.

**Confidence:** With formal proofs and enhanced experiments, this can become a strong paper. The fact that Reviewer 25 was positive suggests the work is valuable - we just need mathematical rigor.

I'm ready to start execution tomorrow. Please let me know if you'd like any adjustments to this plan.

Best regards,
Prajjwal

---

**Attached Documents:**
- REVISION_PLAN.md (detailed task list)
- REVISION_SUMMARY_UPDATED.md (comprehensive analysis)
