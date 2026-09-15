# Comprehensive Experiment Plan
**Date:** February 19, 2026  
**Purpose:** Compare 4 different communication topology pruning methods across 5 challenging scenarios

---

## 🎯 Overview

This experiment systematically evaluates distributed communication pruning algorithms for underwater robot swarms operating in dynamic flow fields. We compare methods across varying conditions to understand trade-offs between:
- **Network sparsity** (edge reduction)
- **Connectivity safety** (algebraic connectivity λ₂)
- **Control efficiency** (computational cost, control effort)
- **Robustness** (performance across scenarios)

---

## 🤖 Methods Being Compared

### 1. **ConcurrentPruning** ⭐ (Proposed Method)
- **Type:** Distributed, Lyapunov-based pruning
- **Key Features:**
  - Each robot locally evaluates edges using Lyapunov analysis
  - Requires bilateral agreement (both endpoints must consent)
  - Deterministic tie-breaking for conflicts
  - Predicts future connectivity impact before pruning
- **Expected Behavior:** Balanced pruning with strong safety guarantees
- **Implementation:** `ConcurrentPruningSimulation` with mode='lyapunov'

### 2. **AdjacencyConsensus** (Baseline Distributed)
- **Type:** Distributed consensus-based pruning
- **Key Features:**
  - Uses adjacency matrix consensus
  - Rolling window stability detection
  - Interleaved consensus execution
  - Conservative pruning decisions
- **Expected Behavior:** Safe but slower pruning, higher edge retention
- **Implementation:** `HybridUnderwaterSimulation` (default consensus pruning)

### 3. **FullGraph** (No Pruning Baseline)
- **Type:** Baseline - maintain all edges
- **Key Features:**
  - No pruning performed
  - Maintains all communication links within range
  - Maximum connectivity, maximum communication cost
- **Expected Behavior:** Highest λ₂, highest edge count, highest cost
- **Implementation:** `BaselineSimulation` with method='full_graph'

### 4. **CentralizedMST** (Oracle Baseline)
- **Type:** Centralized optimal pruning
- **Key Features:**
  - Uses global knowledge to compute Minimum Spanning Tree
  - Kruskal's algorithm for optimal edge selection
  - Not implementable in distributed setting (oracle)
- **Expected Behavior:** Maximum pruning (minimal tree), potentially unsafe if distances change
- **Implementation:** `BaselineSimulation` with method='centralized_mst'

---

## 🌊 Scenarios

### Scenario A: **Baseline** (Moderate Conditions)
**Purpose:** Standard operating conditions
- **Robots:** 10
- **Communication Radius:** 3.0m
- **Workspace:** 10m × 10m
- **Timestep (dt):** 0.05s
- **Flow:** Moderate (0.3 m/s base, 0.3 swirl)
- **Challenge Level:** ⭐⭐ (Medium)
- **Expected:** All methods should perform well

### Scenario B: **Large Timestep** (Temporal Discretization Test)
**Purpose:** Test robustness to coarse temporal resolution
- **Robots:** 10
- **Communication Radius:** 3.5m (compensating)
- **Workspace:** 10m × 10m
- **Timestep (dt):** 0.1s (2× larger!)
- **Flow:** Moderate (0.3 m/s base, 0.3 swirl)
- **Control Tuning:** Much stronger CBF gains (1.5 connectivity, 8.0 safety)
- **Challenge Level:** ⭐⭐⭐ (High)
- **Expected:** Tests if methods handle rapid topology changes

### Scenario C: **Tight Communication Radius** (Sparse Connectivity)
**Purpose:** Test performance with limited range
- **Robots:** 10
- **Communication Radius:** 2.0m (33% smaller!)
- **Workspace:** 10m × 10m
- **Timestep (dt):** 0.03s (smaller to help maintain edges)
- **Flow:** Moderate (0.3 m/s base, 0.3 swirl)
- **Control Tuning:** Prioritize connectivity (2.0 gain)
- **Challenge Level:** ⭐⭐⭐⭐ (Very High)
- **Expected:** Tests pruning under naturally sparse graphs

### Scenario D: **High Robot Count** (Scalability Test)
**Purpose:** Test scalability to larger teams
- **Robots:** 20 (2× baseline!)
- **Communication Radius:** 3.0m
- **Workspace:** 15m × 15m (larger)
- **Timestep (dt):** 0.05s
- **Flow:** Moderate (0.3 m/s base, 0.3 swirl)
- **Challenge Level:** ⭐⭐⭐ (High)
- **Expected:** Tests computational scaling and consensus convergence

### Scenario E: **Extreme** (Combined Stress Test)
**Purpose:** Worst-case combined challenges
- **Robots:** 12
- **Communication Radius:** 2.2m (tight)
- **Workspace:** 10m × 10m
- **Timestep (dt):** 0.08s (large)
- **Flow:** Moderate (0.3 m/s base, 0.3 swirl)
- **Control Tuning:** Very aggressive connectivity preservation (2.5 gain, 7.0 safety)
- **Challenge Level:** ⭐⭐⭐⭐⭐ (Extreme)
- **Expected:** Tests robustness limits

---

## 📊 Metrics Collected

### Per-Simulation Metrics:
- **Edge Statistics:**
  - Initial edge count
  - Final edge count
  - Edges pruned
  - Edge reduction percentage
  - Pruning events count

- **Connectivity Metrics:**
  - Minimum λ₂ (algebraic connectivity)
  - Mean λ₂ over time
  - Final λ₂
  - Number of λ₂ threshold violations
  - Safety threshold: λ₂ > 0.2

- **Control Metrics:**
  - Total control effort (summed over all steps)
  - Mean control effort per step
  - Maximum control effort
  
- **Performance Metrics:**
  - Mean distance to goal
  - Minimum distance to goal
  - Computation time (elapsed seconds)
  - Steps executed
  - Success/failure status

### Time Series Data (for dynamic plots):
- Edge count evolution over time
- λ₂ evolution over time
- Control effort over time
- Robot positions (trajectories) over time
- Pruning events with timestamps

---

## 📈 Planned Visualizations

### **User-Requested Plots (7 plots):**

#### **Plot 1: Robot Trajectories with Pruning Events**
- **Type:** 2D trajectory plot (x vs y)
- **Data:** Position history for ConcurrentPruning
- **Features:** 
  - Robot paths shown as colored lines
  - Pruning events marked with symbols
  - Initial/final positions highlighted
  - Communication edges shown at key moments
- **Purpose:** Visualize spatial behavior and pruning decisions

#### **Plot 2: Edge Evolution Over Time**
- **Type:** 3 separate line plots (one per method comparison)
- **Data:** Edge count vs time for all 4 methods
- **Features:**
  - Time on x-axis, edge count on y-axis
  - One line per method with confidence bands
  - Shows rate of pruning
- **Purpose:** Compare pruning dynamics

#### **Plot 3: Algebraic Connectivity Evolution + Distribution**
- **Type:** Combined plot
  - **Top:** λ₂ vs time for all methods (with safety threshold line)
  - **Bottom:** λ₂ distribution (violin/box plots by method)
- **Data:** Lambda2 history
- **Purpose:** Show safety maintenance over time + final distributions

#### **Plot 4: Control Effort Over Time**
- **Type:** Line plot with confidence bands
- **Data:** Total control effort vs time for all methods
- **Features:** Cumulative or per-step control effort
- **Purpose:** Compare computational/actuation cost

#### **Plot 5: Success Rate Heatmap**
- **Type:** Heatmap (5 scenarios × 4 methods)
- **Data:** Percentage of trials maintaining λ₂ > threshold
- **Features:** Color-coded cells with percentages
- **Purpose:** Identify method robustness across conditions

#### **Plot 6: Average Minimum λ₂ by Scenario**
- **Type:** Line plot (scenarios on x-axis)
- **Data:** Mean of min λ₂ across trials for each scenario
- **Features:** One line per method, error bars showing std dev
- **Purpose:** Show connectivity safety across difficulty levels

#### **Plot 7: Pruning Efficiency by Scenario**
- **Type:** Grouped bar chart
- **Data:** Edge reduction percentage by scenario for each method
- **Features:** Bars grouped by scenario, colored by method
- **Purpose:** Compare pruning effectiveness across conditions

---

### **Recommended Plots from Analysis (from SPECIFIC_PLOT_RECOMMENDATIONS.md):**

#### **Plot R1: Trade-off Analysis ⭐⭐⭐ CRITICAL**
- **Type:** Scatter plot
- **Axes:** Edge Reduction (%) vs Minimum λ₂
- **Points:** Each trial colored by method
- **Features:** Confidence ellipses, ideal region highlighted
- **Purpose:** Show fundamental sparsity-safety trade-off

#### **Plot R2: Method Performance Distributions ⭐⭐⭐ ESSENTIAL**
- **Type:** Side-by-side violin plots
- **Left panel:** Edge Reduction (%) by method
- **Right panel:** Minimum λ₂ by method (with safety line)
- **Purpose:** Statistical comparison of main outcomes

#### **Plot R3: Edge Reduction vs Runtime**
- **Type:** Scatter plot
- **Axes:** Edge Reduction (%) vs Computation Time (s)
- **Purpose:** Show efficiency trade-off

#### **Plot R4: Temporal Dynamics (Combined) ⭐⭐⭐ CRITICAL**
- **Type:** 2-panel line plots
- **Top:** Edge count vs iteration (median + IQR bands)
- **Bottom:** λ₂ vs iteration (median + IQR bands + safety line)
- **Purpose:** Show HOW algorithms work dynamically

#### **Plot R5: Safety Reliability Curve**
- **Type:** Performance profile
- **Axes:** λ₂ threshold vs Success Rate (%)
- **Lines:** One per method
- **Purpose:** "Method X maintains λ₂ > 0.2 in 95% of trials"

#### **Plot R6: Cumulative Distribution Functions**
- **Type:** Empirical CDFs
- **Left:** Edge reduction CDF
- **Right:** Minimum λ₂ CDF
- **Purpose:** Statistical rigor

---

## 🔬 Experimental Design

### Trial Configuration:
- **Number of trials per configuration:** 10
- **Random seeds:** 1000, 1001, 1002, ..., 1009 (consistent across methods)
- **Maximum simulation steps:** 500 per trial
- **Early stopping conditions:**
  - Graph disconnected (λ₂ < 0.01 for >50 steps)
  - Converged to minimal tree (edges = robots - 1)

### Total Simulations:
```
4 methods × 5 scenarios × 10 trials = 200 simulations
Estimated time: ~15-30 minutes (depending on convergence)
```

### Output Files:
- **Primary:** `experiments/comprehensive_results_YYYYMMDD_HHMMSS.json`
- **Contains:**
  - Metadata (timestamp, configuration)
  - Individual trial results
  - Full time series data
  - Trajectory data

---

## 📋 Analysis Plan

### Statistical Tests:
1. **ANOVA** - Test if method choice significantly affects edge reduction
2. **Kruskal-Wallis** - Non-parametric test for λ₂ differences
3. **Post-hoc tests** - Pairwise comparisons between methods
4. **Effect sizes** - Cohen's d for practical significance

### Key Comparisons:
1. **ConcurrentPruning vs AdjacencyConsensus** - Distributed methods comparison
2. **ConcurrentPruning vs FullGraph** - Value of pruning
3. **ConcurrentPruning vs CentralizedMST** - Gap to oracle performance
4. **Across scenarios** - Robustness analysis

### Success Criteria:
- **For ConcurrentPruning to be considered successful:**
  - ✅ Higher edge reduction than AdjacencyConsensus
  - ✅ Similar or better λ₂ safety (>95% trials above threshold)
  - ✅ Graceful degradation across scenarios
  - ✅ Reasonable gap to CentralizedMST (not too far from oracle)
  - ✅ Lower cost than FullGraph (control effort, edges)

---

## ✅ Verification Checklist

Before running experiments, verify:

- [ ] All scenarios properly configured in `config/scenarios.py`
- [ ] All 4 methods correctly implemented
- [ ] Simulation tracking includes:
  - [ ] Robot positions over time
  - [ ] Control effort over time
  - [ ] Edge counts over time
  - [ ] Lambda2 over time
- [ ] Output directory exists: `experiments/`
- [ ] Random seeds are consistent across methods
- [ ] Success/failure criteria clearly defined
- [ ] Early stopping conditions set appropriately

---

## 🚀 Execution Plan

### Phase 1: Quick Validation (5 minutes)
```bash
python run_full_experiments.py --quick --scenarios A --methods ConcurrentPruning AdjacencyConsensus
```
- 2 trials, 100 steps, 1 scenario, 2 methods
- Verify: Code runs, metrics collected, files saved

### Phase 2: Full Experiments (20-30 minutes)
```bash
python run_full_experiments.py
```
- 10 trials × 5 scenarios × 4 methods = 200 simulations
- Output: `experiments/comprehensive_results_YYYYMMDD_HHMMSS.json`

### Phase 3: Generate All Plots (5 minutes)
```bash
python generate_all_plots.py experiments/comprehensive_results_YYYYMMDD_HHMMSS.json
```
- Creates all 13+ plots (7 user + 6+ recommended)
- Output: `experiments/figures/` directory

---

## 📝 Notes

- **Seed Consistency:** Using seeds 1000-1009 ensures same initial conditions for fair comparison
- **Scenario Difficulty:** Scenarios ordered roughly by difficulty (A=easiest, E=hardest)
- **Method Categories:** 
  - **Practical distributed:** ConcurrentPruning, AdjacencyConsensus
  - **Baselines:** FullGraph (lower bound), CentralizedMST (upper bound)
- **Safety Threshold:** λ₂ > 0.2 (more relaxed than default 0.3 to allow more pruning)

---

## 🎯 Expected Outcomes

### Hypothesis:
**ConcurrentPruning will achieve superior balance:** moderate edge reduction with high connectivity safety, outperforming AdjacencyConsensus in pruning effectiveness while matching safety guarantees.

### Key Questions:
1. Does Lyapunov prediction improve pruning decisions over consensus alone?
2. How much does ConcurrentPruning close the gap to optimal (CentralizedMST)?
3. Which scenarios expose the greatest differences between methods?
4. Is there a fundamental trade-off curve, or can smart pruning improve both metrics?

---

**Ready to proceed? Please review and confirm before starting experiments!** 🚀
