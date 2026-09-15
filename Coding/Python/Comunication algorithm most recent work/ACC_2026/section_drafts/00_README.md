# Section II Drafts - README
## ACC 2026 Paper Update Workflow

---

## PURPOSE OF THIS DIRECTORY

This folder contains markdown drafts for each update to the ACC 2026 paper. Each file represents a specific section or subsection that needs to be revised based on reviewer feedback.

**Workflow:**
1. Review the draft .md file
2. Edit in your own language / add your input
3. Refine the text until you're satisfied
4. Copy the final version into the LaTeX paper (ACC_2026.tex)
5. Mark the task as complete in PAPER_MIGRATION_TODO.md

---

## SECTION II FILES (CURRENT)

### File Structure:

| File | Purpose | Replaces/Inserts | Status |
|------|---------|-----------------|--------|
| `00_README.md` | This file - navigation guide | N/A | ✅ |
| `01_Section_II_Introduction.md` | New introduction paragraphs | Insert BEFORE subsection II.A | ⬜ Draft |
| `02_Remark1_Model_Specialization.md` | Clarify simulation vs. control model | Insert AFTER eq. (1) | ⬜ Draft |
| `03_Assumptions_A1_to_A6.md` | Formal assumptions list | REPLACE subsection II.B | ⬜ Draft |
| `04_Graph_and_Constraint_Definitions.md` | Rigorous graph/set definitions | NEW subsection II.C | ⬜ Draft |
| `05_Problem_Statement_Revised.md` | Formal problem statement | REPLACE current II.C (→becomes II.D) | ⬜ Draft |

---

## HOW TO USE THESE DRAFTS

### Step 1: Read the Draft
Each .md file has three main sections:
- **Draft Text (LaTeX Format):** Ready-to-paste LaTeX code
- **Plain Language Summary:** Explanation in simple terms
- **Integration Notes:** Where/how to insert into the paper

### Step 2: Review and Edit
- Check if the technical content is accurate
- Adjust language to match your writing style
- Add any missing context or examples from your work
- Verify all cross-references make sense

### Step 3: Prepare for Integration
- Copy the LaTeX code from the draft
- Make note of:
  - Which line numbers in ACC_2026.tex to modify
  - What references/labels need to be added
  - Any new figures or equations to prepare

### Step 4: Update the Paper
- Open `ACC_2026/ACC_2026.tex`
- Find the location indicated in "Integration Notes"
- Paste and format the new content
- Compile to check for errors
- Update cross-references

### Step 5: Mark Complete
- In `PAPER_MIGRATION_TODO.md`, check off the completed task
- Update the status in the file header (⬜ → ✅)

---

## SECTION II OVERVIEW

### What We're Changing:

**Current Structure:**
```
II. Background and Problem Formulation
  A. Advection-diffusion model
  B. Assumptions on communication and connectivity
  C. Problem Statement
```

**New Structure:**
```
II. Background and Problem Formulation
  [NEW: 2-3 paragraph introduction]
  A. Advection-diffusion model
     [NEW: Remark 1 - Model Specialization]
  B. Assumptions on Dynamics and Communication [REWRITTEN]
     - Assumption A1: Bounded Control
     - Assumption A2: Bounded Disturbance
     - Assumption A3: Lipschitz Flow (addresses R1-2!)
     - Assumption A4: Control Dominance
     - Assumption A5: Initial Connectivity
     - Assumption A6: Range-Safety Margin
  C. Graph Notation and Constraint Sets [NEW]
     - Communication graph definitions
     - Critical edge set
     - Safe set
     - Connectivity set
     - Feasible region
  D. Problem Statement [REVISED]
     - Given-Design-Such That structure
     - Explicit references to assumptions
     - Quantitative objectives
```

---

## REVIEWER CONCERNS → DRAFT FILE MAPPING

| Reviewer ID | Concern | Addressed in File |
|-------------|---------|------------------|
| R1-1 | Model confusion | `02_Remark1_Model_Specialization.md` |
| R1-2 | Flow appears trivial | `03_Assumptions_A1_to_A6.md` (A3) |
| R1-3 | Informal graph definitions | `04_Graph_and_Constraint_Definitions.md` |
| R1-4 | No formal proofs | (Section III drafts - coming next) |
| R2-2 | $d_{\min}$ unclear | `03_Assumptions_A1_to_A6.md` (A6) |
| General | Weak motivation | `01_Section_II_Introduction.md` |

---

## ESTIMATED TIMELINE

Based on PAPER_MIGRATION_TODO.md:

- **Day 1 (Feb 3):** Files 01-03 → Section II intro + Remark 1 + Assumptions
- **Day 2 (Feb 4):** Files 04-05 → Graph definitions + Problem statement
- **Day 3 (Feb 5):** Integration into ACC_2026.tex + LaTeX compilation check

Total time for Section II: ~3 days (part of Phase 1)

---

## LATEX COMPILATION TIPS

After integrating each draft:

```bash
# Navigate to ACC_2026 directory
cd "ACC_2026"

# Compile (run multiple times for cross-refs)
pdflatex ACC_2026.tex
pdflatex ACC_2026.tex

# Check for errors
# Look for lines starting with "!" in the output
```

Common issues:
- **Undefined labels:** Run `pdflatex` twice
- **Bibliography errors:** Need to run `bibtex` (we'll do this after adding references)
- **Missing packages:** Check preamble for `\usepackage{}` statements

---

## NEXT STEPS (AFTER SECTION II)

Once Section II is complete, we'll create similar drafts for:

1. **Section III-C: Theoretical Guarantees**
   - Theorem T1: Safety & Connectivity Invariance
   - Theorem T2: CLF-Based Goal Convergence
   - Theorem T3: Global Connectivity via Critical Edges
   - Theorem T4: Pruning Correctness

2. **Section III Enhancements**
   - Explicit QP formulation
   - CLF/CBF parameter explanations
   - Algorithm updates

3. **Section IV: Results Update**
   - Integrate 125-run batch data
   - Add comparison tables
   - Update figure captions with theorem references

4. **Front Matter**
   - Abstract rewrite
   - Introduction update with contributions list

---

## QUESTIONS OR ISSUES?

If something in the drafts is unclear:
1. Check the "Plain Language Summary" section
2. Review the cross-references to other documents (REVIEWER_FIX_THEOREM_MATRIX.md, etc.)
3. Check PAPER_MIGRATION_TODO.md for additional context
4. Add comments/notes directly in the .md file for discussion

---

## COLLABORATION NOTES

When working with co-authors:
- **Geoff Hollinger:** Focus on underwater robotics framing in file 01
- **Xi Yu:** Review theoretical rigor in files 03-04
- Mark files with status tags:
  - ⬜ Draft
  - 🔄 Under Review
  - ✅ Approved
  - 📝 Needs Revision

---

## BACKUP STRATEGY

Before making changes to ACC_2026.tex:
1. Make a backup copy: `ACC_2026_backup_Feb3.tex`
2. Use version control (git) if available
3. Save intermediate versions after each major section update

---

**Happy drafting!** 🚀

Remember: These are DRAFTS. You have full freedom to edit, refine, and personalize the language before integrating into the paper.
