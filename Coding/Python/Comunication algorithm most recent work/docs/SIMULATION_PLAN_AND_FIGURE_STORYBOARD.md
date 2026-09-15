# SIMULATION PLAN & FIGURE STORYBOARD FOR ACC 2026
## Complete Blueprint for Results Section

**Date:** January 30, 2026  
**Purpose:** Finalized simulation plan with plots, metrics, and baseline methods  
**Status:** 📋 Planning Complete — Ready for Implementation

---

## TABLE OF CONTENTS

1. [Overview](#1-overview)
2. [Available Simulation Metrics](#2-available-simulation-metrics)
3. [Figure Storyboard (8 Figures)](#3-figure-storyboard-8-figures)
4. [Baseline Methods](#4-baseline-methods-to-implement)
5. [Key Performance Metrics](#5-key-performance-metrics-table)
6. [Simulation Scenarios](#6-simulation-scenarios)
7. [Figure-to-Theorem Mapping](#7-figure-to-theorem-mapping)
8. [Implementation Checklist](#8-implementation-checklist)

---

## 1. OVERVIEW

### 1.1 Objectives

This document defines the **complete simulation plan** to validate all theoretical contributions (T1-T4) and address all reviewer comments (R1-1 through R2-4) for the ACC 2026 revision.

### 1.2 Design Principles

- **Coverage:** Every theorem (T1, T2, T3, T4) validated by at least one figure
- **Clarity:** Every reviewer concern addressed with visual evidence
- **Rigor:** Quantitative metrics for all claims
- **Comparison:** Baseline methods to demonstrate novelty
- **Reproducibility:** Clear simulation scenarios and parameters

### 1.3 Simulation Capabilities

The codebase can track:
- ✅ **Topology Metrics**: Edge count, algebraic connectivity (λ₂), spanning tree structure
- ✅ **Consensus Metrics**: Convergence iterations, agreement error, unanimous decisions
- ✅ **Control Metrics**: Control effort, CLF evolution, CBF constraint satisfaction
- ✅ **Spatial Metrics**: Robot trajectories, inter-robot distances, goal convergence
- ✅ **Flow Field**: Spatially-varying velocity field with gradients
- ✅ **Temporal Metrics**: Pruning events, stability windows, consensus phases

---

## 2. AVAILABLE SIMULATION METRICS

### 2.1 Graph Topology Metrics

| **Metric** | **Symbol** | **Computation** | **Use Case** |
|-----------|-----------|----------------|--------------|
| Number of edges | $\|E(t)\|$ | `len(sim.current_edges())` | Track pruning progress |
| Algebraic connectivity | $\lambda_2(L)$ | `sim.lambda2_manager.get_lambda2(A)` | Validate T3 |
| Average degree | $\bar{d}$ | $2\|E\|/N$ | Network density |
| Diameter | $\text{diam}(G)$ | Longest shortest path | Communication delay |
| Clustering coefficient | $C$ | Local triangle density | Graph structure |

### 2.2 Consensus Metrics

| **Metric** | **Symbol** | **Computation** | **Use Case** |
|-----------|-----------|----------------|--------------|
| Consensus iterations | $k_{\text{conv}}$ | `sim.pruning_manager.consensus_iterations` | Validate convergence |
| Agreement error | $\epsilon_{\text{agree}}$ | $\max_{l,m} \|\mathbf{A}^l - \mathbf{A}^m\|_F$ | Consensus quality |
| Unanimous decision rate | $P_{\text{unan}}$ | Successful unanimous votes / Total attempts | Robustness |
| Edge estimate accuracy | $\|\mathbf{A}^l - \mathbf{A}^*\|$ | Compare estimate to ground truth | Validation |

### 2.3 Control & Safety Metrics

| **Metric** | **Symbol** | **Computation** | **Use Case** |
|-----------|-----------|----------------|--------------|
| Min inter-robot distance | $d_{\min}^{\text{actual}}$ | $\min_{i \neq j} \|x_i - x_j\|$ | Validate T1 safety |
| Max edge distance | $d_{\max}^{\text{edge}}$ | $\max_{(i,j) \in E} \|x_i - x_j\|$ | Validate T1 connectivity |
| Control magnitude | $\|u_i(t)\|$ | `np.linalg.norm(control)` | Energy analysis |
| Cumulative energy | $E_{\text{total}}$ | $\sum_t \|u_i(t)\|^2 \Delta t$ | Efficiency comparison |
| CLF value | $V_i(t)$ | $\|x_i - x_{\text{goal}}\|^2$ | Validate T2 |
| CLF relaxation | $\delta(t)$ | From QP solver | Practical stability |

### 2.4 Goal Convergence Metrics

| **Metric** | **Symbol** | **Computation** | **Use Case** |
|-----------|-----------|----------------|--------------|
| Distance to goal | $\|x_i - x_{\text{goal}}\|$ | Euclidean distance | T2 validation |
| Time to goal | $T_{\text{goal}}$ | First time $\|x_i - x_{\text{goal}}\| < \epsilon$ | Convergence rate |
| Mean goal distance | $\bar{d}_{\text{goal}}$ | Average over all robots | Overall performance |

### 2.5 Flow Field Metrics

| **Metric** | **Symbol** | **Computation** | **Use Case** |
|-----------|-----------|----------------|--------------|
| Flow magnitude | $\|f_{\text{flow}}(x)\|$ | From flow field model | Validate A3 |
| Flow gradient | $\|\nabla f_{\text{flow}}\|$ | Spatial variation | Non-cancellation proof |
| Flow alignment | $\langle v_i, f_{\text{flow}} \rangle$ | Dot product | Flow exploitation |
| Flow differential | $\|f_i - f_j\|$ | Between robot pairs | Lipschitz validation |

---

## 3. FIGURE STORYBOARD (8 Figures)

### **FIGURE 1: Problem Setup & Spatially-Varying Flow Field**

**Purpose:** Establish problem domain, validate non-trivial flow (addresses R1-2)  
**Validates:** Assumption A3 (Lipschitz flow), T1 foundation  
**Layout:** 2×2 subplot grid

#### Subplot 1(a): Flow Field Quiver Plot
- **X-axis:** x position (m)
- **Y-axis:** y position (m)
- **Content:** 
  - Quiver plot showing spatially-varying flow vectors across workspace
  - Color-coded by flow magnitude $\|f_{\text{flow}}(x,t)\|$
  - Overlay: Initial robot positions (red dots, clustered)
  - Overlay: Goal position (gold star)
- **Colorbar:** Flow speed (m/s)
- **Expected Result:** Clear spatial variation in flow direction and magnitude

#### Subplot 1(b): Flow Gradient Magnitude Heatmap
- **X-axis:** x position (m)
- **Y-axis:** y position (m)
- **Content:**
  - Heatmap of $\|\nabla f_{\text{flow}}(x,t)\|$ (spatial derivative)
  - Demonstrates non-zero gradients → non-cancellation in relative dynamics
- **Colorbar:** Gradient magnitude (1/s)
- **Expected Result:** Non-uniform gradient field, validates A3

#### Subplot 1(c): Initial Communication Graph
- **X-axis:** x position (m)
- **Y-axis:** y position (m)
- **Content:**
  - Network graph at t=0 (before pruning)
  - Nodes = robots, edges = communication links
  - Annotate: $|E_0| = $ [number], $\bar{d}_0 = $ [avg degree]
- **Expected Result:** Dense graph (near-complete if clustered)

#### Subplot 1(d): Flow Differential vs. Distance
- **X-axis:** Inter-robot distance $\|x_i - x_j\|$ (m)
- **Y-axis:** Flow differential $\|f_{\text{flow}}(x_i) - f_{\text{flow}}(x_j)\|$ (m/s)
- **Content:**
  - Scatter plot for all robot pairs
  - Overlay: Linear bound $y = L \cdot x$ (Lipschitz constant L)
  - Validates A3: $\|f_i - f_j\| \leq L \|x_i - x_j\|$
- **Expected Result:** Points lie below linear bound

**Implementation File:** `scripts/plot_figure1_flow_field.py`

---

### **FIGURE 2: Trajectory Evolution & Safety Preservation**

**Purpose:** Validate T1 (Robust Invariance), clarify $d_{\min}$ (addresses R2-2)  
**Validates:** Theorem T1 (CBF forward invariance)  
**Layout:** 2×2 subplot grid

#### Subplot 2(a): Robot Trajectories in Flow Field
- **X-axis:** x position (m)
- **Y-axis:** y position (m)
- **Content:**
  - 2D robot trajectories from start to goal
  - Color-coded by robot ID
  - Background: Flow field quiver (low alpha)
  - Start markers (circles), end markers (triangles)
  - Goal position (gold star)
- **Expected Result:** Trajectories navigate around each other, reach goal

#### Subplot 2(b): Minimum Inter-Robot Distance Over Time
- **X-axis:** Time (s)
- **Y-axis:** $\min_{i \neq j} \|x_i - x_j\|$ (m)
- **Content:**
  - Time series of minimum pairwise distance
  - **Critical line:** $d_{\min} = 0.6$ m (safety threshold) - RED DASHED
  - Shaded region below threshold (violation zone)
- **Expected Result:** **NEVER drops below $d_{\min}$** → proves T1 safety

#### Subplot 2(c): Maximum Communication Distance Over Time
- **X-axis:** Time (s)
- **Y-axis:** $\max_{(i,j) \in C(t)} \|x_i - x_j\|$ (m) [only for critical edges]
- **Content:**
  - Time series of maximum distance among critical edges
  - **Critical line:** $R_{\max} = 1.2$ m (connectivity threshold) - BLUE DASHED
  - Shaded region above threshold (violation zone)
- **Expected Result:** Critical edges stay within $R_{\max}$ → proves T1 connectivity

#### Subplot 2(d): Safety & Connectivity Constraint Satisfaction
- **X-axis:** Time (s)
- **Y-axis (left):** Number of safety violations (collisions)
- **Y-axis (right):** Number of connectivity violations (edge breaks)
- **Content:**
  - Dual y-axis plot
  - Bar chart or area plot for violations
- **Expected Result:** **ZERO violations** throughout simulation

**Implementation File:** `scripts/plot_figure2_trajectories_safety.py`

---

### **FIGURE 3: Edge Pruning Dynamics & Graph Evolution**

**Purpose:** Validate T4 (Pruning Correctness), demonstrate algorithm workflow  
**Validates:** Theorem T4 (no false disconnections), T3 (spanning backbone)  
**Layout:** 2×2 subplot grid

#### Subplot 3(a): Edge Count Evolution
- **X-axis:** Time (s)
- **Y-axis:** Number of edges $|E(t)|$
- **Content:**
  - Time series of edge count
  - Vertical markers at pruning events (red dashed lines)
  - Annotate: Initial $|E_0|$, Final $|E_f|$, Target MST = $N-1$
  - Show reduction rate: $(E_0 - E_f)/E_0 \times 100\%$
- **Expected Result:** Step-wise decrease from $E_0 \approx 30$ to $E_f \approx 11$ (for N=12)

#### Subplot 3(b): Graph Snapshots at Key Timepoints
- **Layout:** 2×2 grid of network graphs
  - **Top-left:** t=0 (initial dense graph)
  - **Top-right:** After 1st pruning event
  - **Bottom-left:** After 3rd pruning event
  - **Bottom-right:** Final converged graph
- **Content:**
  - Node positions, active edges (solid), pruned edges (dotted red)
  - Annotate each with $|E|$ and timestamp
- **Expected Result:** Progressive sparsification toward tree structure

#### Subplot 3(c): Pruned Edge Lengths Distribution
- **X-axis:** Pruning event number (chronological)
- **Y-axis:** Edge length (m)
- **Content:**
  - Bar chart of pruned edge lengths in order removed
  - Color gradient: Longer = darker (weaker links)
- **Expected Result:** Longest (weakest) edges pruned first → MST convergence

#### Subplot 3(d): Pruning Decision Timeline (Gantt Chart)
- **X-axis:** Time (s)
- **Y-axis:** Phase category
- **Content:**
  - Horizontal bars showing:
    - **Gray:** Stability windows (waiting for stable topology)
    - **Blue:** Consensus phases (distributed agreement)
    - **Green:** Validation phases
    - **Red markers:** Pruning events
  - Shows multi-layer robustness workflow
- **Expected Result:** Clear phase separation, no overlaps

**Implementation File:** `scripts/plot_figure3_pruning_dynamics.py`

---

### **FIGURE 4: Algebraic Connectivity & Network Health**

**Purpose:** Validate T3 (Global Connectivity via λ₂), demonstrate novel λ₂ estimation  
**Validates:** Theorem T3, Novel contribution (3-tier λ₂ estimator)  
**Layout:** 2×1 subplots + comparison table

#### Subplot 4(a): Algebraic Connectivity (λ₂) Evolution
- **X-axis:** Time (s)
- **Y-axis:** $\lambda_2(L(t))$ (algebraic connectivity)
- **Content:**
  - Time series of λ₂
  - **Critical line:** $\lambda_{2,\min} = 0.1$ (threshold) - RED DASHED
  - Vertical markers at pruning events
  - Shaded region below threshold (danger zone)
- **Expected Result:** **Always > $\lambda_{2,\min}$** → proves T3 connectivity

#### Subplot 4(b): λ₂ Estimation Methods Comparison
- **X-axis:** Time (s)
- **Y-axis:** λ₂ value
- **Content:**
  - Three overlaid curves:
    1. **Exact** (eigenvalue computation) - solid black
    2. **Cheeger bound** (lower bound) - dashed blue
    3. **Incremental** (perturbation update) - dotted green
  - Legend with computational costs: O(n³), O(n²), O(n)
- **Expected Result:** All methods agree closely, validates adaptive selection

#### Subplot 4(c): Computational Cost Breakdown (Table)
- **Content:**
  - Table showing:
    | Method | Complexity | Usage (%) | Avg. Error (%) |
    |--------|-----------|----------|---------------|
    | Exact | O(n³) | 15% | 0.0% |
    | Cheeger | O(n²) | 25% | 8.2% |
    | Incremental | O(n) | 60% | 2.1% |
  - Demonstrates computational efficiency of adaptive approach

**Implementation File:** `scripts/plot_figure4_lambda2_connectivity.py`

---

### **FIGURE 5: Consensus Convergence & Distributed Agreement**

**Purpose:** Validate adjacency matrix consensus (Griparic et al.), T3 foundation  
**Validates:** Consensus protocol, unanimous decision mechanism  
**Layout:** 2×2 subplot grid

#### Subplot 5(a): Consensus Agreement Error Over Iterations
- **X-axis:** Consensus iteration $k$
- **Y-axis:** $\max_{l,m} \|\mathbf{A}^l(k) - \mathbf{A}^m(k)\|_F$ (Frobenius norm)
- **Content:**
  - Time series of maximum disagreement between robots
  - **Convergence threshold:** $\epsilon_{\text{conv}} = 10^{-4}$ - RED DASHED
  - Overlay: Exponential fit $y = C e^{-\lambda_2 k}$
- **Expected Result:** Exponential decay to zero → validates consensus theory

#### Subplot 5(b): Adjacency Estimate Heatmaps (Before/After Consensus)
- **Layout:** 1×3 grid
  - **Left:** Robot 0's estimate $\mathbf{A}^0(k=0)$ (initial)
  - **Middle:** Robot 5's estimate $\mathbf{A}^5(k=0)$ (different initial)
  - **Right:** Converged $\mathbf{A}^l(k=k_{\text{conv}})$ (unanimous)
- **Content:**
  - 12×12 heatmaps (for N=12 robots)
  - Color scale: 0 (white) to 1 (dark blue)
  - Annotate: Frobenius norm difference
- **Expected Result:** Left ≠ Middle, but both → Right (consensus)

#### Subplot 5(c): Unanimous Decision Rate per Pruning Event
- **X-axis:** Pruning attempt number
- **Y-axis:** Unanimous agreement? (1=Yes, 0=No)
- **Content:**
  - Binary indicators showing success/failure
  - Annotate: Overall rate (e.g., 95% unanimous)
- **Expected Result:** >95% unanimous decisions

#### Subplot 5(d): Consensus Iteration Count Distribution
- **X-axis:** Number of iterations to convergence $k_{\text{conv}}$
- **Y-axis:** Frequency (number of pruning events)
- **Content:**
  - Histogram of convergence times across all pruning events
  - Annotate: Mean, std dev, max
- **Expected Result:** Most converge in 20-50 iterations

**Implementation File:** `scripts/plot_figure5_consensus_convergence.py`

---

### **FIGURE 6: Goal Convergence & CLF Evolution**

**Purpose:** Validate T2 (CLF-Based Goal Convergence), address R2-1 (stability)  
**Validates:** Theorem T2, addresses R2-3 (Lyapunov usage)  
**Layout:** 2×2 subplot grid

#### Subplot 6(a): Robot-to-Goal Distance Over Time
- **X-axis:** Time (s)
- **Y-axis:** $\|x_i(t) - x_{\text{goal}}\|$ (m)
- **Content:**
  - Individual curves for each robot (thin lines, color-coded)
  - Mean trajectory (thick black line)
  - Shaded region: Mean ± 1 std dev
  - **Goal region threshold:** $\epsilon = 0.5$ m - GOLD DASHED
- **Expected Result:** Monotonic decrease for robots in connectivity tree

#### Subplot 6(b): CLF Value Evolution (Detecting Robots)
- **X-axis:** Time (s)
- **Y-axis:** $V_i(t) = \|x_i - x_{\text{goal}}\|^2$ (CLF value)
- **Content:**
  - CLF for robots with active goal assignment (in tree)
  - Overlay: Theoretical bound $V(t) \leq V(0) e^{-\alpha t} + \gamma/\alpha$
  - Semilog scale (y-axis) to show exponential decay
- **Expected Result:** Exponential-like decay, validates T2

#### Subplot 6(c): Goal Region Arrival Times (CDF)
- **X-axis:** Time (s)
- **Y-axis:** Cumulative fraction of robots arrived
- **Content:**
  - CDF of arrival times into $\epsilon$-ball ($\epsilon = 0.5$ m)
  - Annotate: Median arrival time, 90th percentile
  - Compare: Proposed vs. Full Graph baseline
- **Expected Result:** 80-90% arrival within simulation time

#### Subplot 6(d): CLF Constraint Relaxation Variable
- **X-axis:** Time (s)
- **Y-axis:** $\delta(t)$ (relaxation variable from QP)
- **Content:**
  - Time series of relaxation penalty
  - Shows when CBF constraints conflict with CLF
  - Annotate: Mean, max values
- **Expected Result:** Bounded relaxation → practical stability (T2)

**Implementation File:** `scripts/plot_figure6_goal_convergence_clf.py`

---

### **FIGURE 7: Control Effort & Energy Efficiency**

**Purpose:** Demonstrate energy efficiency motivation (underwater robotics)  
**Validates:** Practical motivation, energy savings claim  
**Layout:** 2×2 subplot grid

#### Subplot 7(a): Control Magnitude Over Time (Comparison)
- **X-axis:** Time (s)
- **Y-axis:** $\|u_i(t)\|$ (control magnitude, m/s²)
- **Content:**
  - Two curves:
    1. **Proposed method** (pruned graph) - Blue
    2. **Full graph baseline** (no pruning) - Red
  - Mean ± std dev bands
  - Annotate: Mean reduction percentage
- **Expected Result:** 40-60% reduction after pruning

#### Subplot 7(b): Cumulative Control Energy (Per Robot)
- **X-axis:** Time (s)
- **Y-axis:** $E_i(t) = \int_0^t \|u_i(\tau)\|^2 d\tau$ (J)
- **Content:**
  - Cumulative energy for each robot (thin lines)
  - Mean cumulative energy (thick line)
  - Compare: Proposed vs. Full Graph
- **Expected Result:** Lower energy with pruned graph

#### Subplot 7(c): Control Effort Breakdown (Stacked Area)
- **X-axis:** Time (s)
- **Y-axis:** Control contribution (stacked, m/s²)
- **Content:**
  - Stacked area plot showing:
    1. **CLF contribution** (goal-seeking) - Green
    2. **CBF safety** (collision avoidance) - Red
    3. **CBF connectivity** (maintain parent link) - Blue
  - Shows temporal evolution of control priorities
- **Expected Result:** Connectivity dominates early, CLF increases as pruning occurs

#### Subplot 7(d): Flow Exploitation Metric
- **X-axis:** Time (s)
- **Y-axis:** $\langle v_i, f_{\text{flow}}(x_i) \rangle / (\|v_i\| \|f_{\text{flow}}\|)$ (alignment)
- **Content:**
  - Flow alignment for all robots (mean curve)
  - Alignment = 1 (fully with flow), -1 (against flow), 0 (perpendicular)
  - Annotate: Mean alignment value
- **Expected Result:** Positive alignment → robots exploit flow

**Implementation File:** `scripts/plot_figure7_control_energy.py`

---

### **FIGURE 8: Baseline Comparison & Ablation Study**

**Purpose:** Validate novelty and superiority over existing methods  
**Validates:** All theorems indirectly, demonstrates practical advantage  
**Layout:** 2×2 subplot grid

#### Subplot 8(a): Network Edge Count Comparison
- **X-axis:** Time (s)
- **Y-axis:** Number of edges $|E(t)|$
- **Content:**
  - Four methods overlaid:
    1. **Proposed (Hybrid Consensus)** - Blue solid
    2. **Full Graph (No Pruning)** - Red solid
    3. **Centralized MST (Oracle)** - Black dashed (ideal)
    4. **Random Pruning** - Orange dotted
  - Annotate: Final edge counts
- **Expected Result:** Proposed ≈ MST < Random < Full

#### Subplot 8(b): Connectivity Preservation Success Rate
- **X-axis:** Method
- **Y-axis:** Success rate (%)
- **Content:**
  - Bar chart showing percentage of time $\lambda_2(t) > \lambda_{2,\min}$
  - Four bars: Proposed, Full, MST, Random
- **Expected Result:** Proposed ≈ MST ≈ 100%, Random ≈ 60-80%, Full = 100% (but high cost)

#### Subplot 8(c): Total Consensus Overhead
- **X-axis:** Method
- **Y-axis:** Total consensus iterations (across all pruning events)
- **Content:**
  - Bar chart comparing computational overhead
  - Only Proposed and Random have consensus cost
  - Annotate: Average iterations per pruning event
- **Expected Result:** Proposed has overhead, but guarantees correctness

#### Subplot 8(d): Cumulative Control Energy Comparison
- **X-axis:** Method
- **Y-axis:** Total fleet energy $\sum_i E_i$ (J)
- **Content:**
  - Bar chart of total energy expenditure
  - Four bars: Proposed, Full, MST, Random
  - Error bars: std dev across Monte Carlo runs
- **Expected Result:** Proposed ≈ MST < Random < Full (30-50% savings vs. Full)

**Implementation File:** `scripts/plot_figure8_baseline_comparison.py`

---

## 4. BASELINE METHODS TO IMPLEMENT

### 4.1 Method 1: Full Graph (No Pruning)

**Description:** Maintain all initial communication edges without pruning

**Implementation:**
```python
class FullGraphBaseline:
    def __init__(self, sim_config):
        self.initial_edges = None
    
    def step(self, robots):
        # Keep all edges from initial topology
        if self.initial_edges is None:
            self.initial_edges = compute_all_edges(robots, R_max)
        return self.initial_edges  # Never prune
```

**Control:** Apply CBF to ALL edges (high control cost)

**Use Case:** Baseline for energy comparison, upper bound on connectivity

---

### 4.2 Method 2: Centralized MST (Oracle)

**Description:** Compute exact MST using global knowledge (Prim's algorithm)

**Implementation:**
```python
class CentralizedMSTBaseline:
    def compute_mst(self, robots):
        # Use global positions (oracle)
        distances = compute_all_distances(robots)
        mst_edges = prims_algorithm(distances)
        return mst_edges
```

**Control:** Apply CBF only to MST edges (optimal, but centralized)

**Use Case:** Ideal baseline showing best possible performance with global knowledge

**Algorithm:** Prim's MST or Kruskal's MST

---

### 4.3 Method 3: Random Pruning

**Description:** Remove random edges without consensus or safety checks

**Implementation:**
```python
class RandomPruningBaseline:
    def step(self, current_edges):
        # Randomly select and remove one edge
        candidate = random.choice(current_edges)
        # NO consensus, NO lambda2 check
        return current_edges - {candidate}
```

**Control:** Apply CBF to remaining edges

**Use Case:** Demonstrate importance of distributed coordination

**Expected:** Frequent disconnections, poor performance

---

### 4.4 Method 4: Distance-Only Pruning (Greedy)

**Description:** Prune longest edges greedily without connectivity verification

**Implementation:**
```python
class GreedyDistancePruning:
    def step(self, robots, current_edges):
        # Find longest edge
        longest_edge = max(current_edges, key=lambda e: distance(e))
        # Remove without checking lambda2
        return current_edges - {longest_edge}
```

**Control:** Apply CBF to remaining edges

**Use Case:** Show importance of λ₂ verification (T3)

**Expected:** May disconnect graph, violates T3

---

### 4.5 Baseline Comparison Matrix

| **Method** | **Knowledge** | **Consensus?** | **λ₂ Check?** | **Expected Performance** |
|-----------|--------------|---------------|--------------|------------------------|
| **Proposed** | Distributed | ✅ Yes | ✅ Yes | High efficiency, maintains connectivity |
| **Full Graph** | Local | ❌ No | ❌ No | High energy, always connected |
| **MST (Oracle)** | Global | ❌ No | ✅ Implicit | Optimal (ideal comparison) |
| **Random** | None | ❌ No | ❌ No | Poor, frequent disconnections |
| **Greedy Distance** | Local | ❌ No | ❌ No | Medium, may disconnect |

---

## 5. KEY PERFORMANCE METRICS TABLE

### 5.1 Safety & Connectivity Metrics

| **Category** | **Metric** | **Symbol** | **Expected Range** | **Theorem** |
|-------------|-----------|------------|-------------------|------------|
| **Safety** | Min inter-robot distance | $\min_{i \neq j} \\|x_i - x_j\\|$ | > $d_{\min} = 0.6$ m | T1 |
| **Safety** | Collision count | $N_{\text{coll}}$ | 0 | T1 |
| **Connectivity** | Algebraic connectivity | $\lambda_2(L)$ | > $\lambda_{\min} = 0.1$ | T3 |
| **Connectivity** | Graph connected? | Boolean | Always TRUE | T3, T4 |
| **Connectivity** | Max edge distance (critical) | $\max_{e \in C} d_e$ | < $R_{\max} = 1.2$ m | T1 |

### 5.2 Efficiency Metrics

| **Category** | **Metric** | **Symbol** | **Expected Range** | **Interpretation** |
|-------------|-----------|------------|-------------------|--------------------|
| **Topology** | Edge reduction rate | $(E_0 - E_f)/E_0$ | 70-90% | Sparsification success |
| **Topology** | Final edge count | $\|E_f\|$ | $N - 1$ to $N + 3$ | MST convergence |
| **Energy** | Control reduction vs. full | $\Delta E / E_{\text{full}}$ | 40-60% | Energy savings |
| **Energy** | Total fleet energy | $\sum_i E_i$ | 50-200 J | Absolute efficiency |
| **Consensus** | Convergence iterations (avg) | $\bar{k}_{\text{conv}}$ | 20-100 | Computational cost |

### 5.3 Convergence Metrics

| **Category** | **Metric** | **Symbol** | **Expected Range** | **Theorem** |
|-------------|-----------|------------|-------------------|------------|
| **Goal** | Time to goal ($\epsilon = 0.5$ m) | $T_{\text{goal}}$ | 80-150 steps | T2 |
| **Goal** | Final distance to goal | $\\|x_i^f - x_{\text{goal}}\\|$ | < 0.5 m (90% robots) | T2 |
| **Goal** | CLF decay rate | $\alpha$ (from fit) | 0.02-0.05 | T2 |
| **Consensus** | Agreement error (final) | $\epsilon_{\text{final}}$ | < $10^{-4}$ | T3 |

### 5.4 Robustness Metrics

| **Category** | **Metric** | **Symbol** | **Expected Range** | **Interpretation** |
|-------------|-----------|------------|-------------------|--------------------|
| **Decision** | Unanimous agreement rate | $P(\text{unanimous})$ | > 95% | Consensus quality |
| **Decision** | Failed pruning attempts | $N_{\text{fail}}$ | 0-2 | Robustness |
| **Topology** | Stability window duration | $T_{\text{stable}}$ | 5-10 steps | Multi-layer robustness |
| **Topology** | Graph disconnection events | $N_{\text{disconnect}}$ | 0 | T4 validation |

---

## 6. SIMULATION SCENARIOS

### 6.1 Scenario A: Baseline (Moderate Conditions)

**Purpose:** Standard validation scenario

**Parameters:**
- **Number of robots:** $N = 12$
- **Communication radius:** $R_{\max} = 1.2$ m
- **Flow magnitude:** $\|f_{\text{flow}}\| \approx 0.3$ m/s (moderate)
- **Disturbance:** $\sigma_{\text{diff}} = 0.008$ m/s (standard Brownian)
- **Workspace:** $10 \times 10$ m
- **Simulation time:** $T = 200$ s
- **Time step:** $\Delta t = 0.2$ s

**Expected Results:**
- Edge reduction: 75-85%
- All theorems validated
- Energy savings: 45-55% vs. full graph

**Use for:** Figures 1-7 (main results)

---

### 6.2 Scenario B: High Robot Count

**Purpose:** Test scalability

**Parameters:**
- **Number of robots:** $N = 20$ ← Increased
- **Communication radius:** $R_{\max} = 1.5$ m (scaled)
- **Flow magnitude:** $\|f_{\text{flow}}\| \approx 0.3$ m/s
- **Workspace:** $15 \times 15$ m (scaled)
- **Other:** Same as Scenario A

**Expected Results:**
- More consensus iterations (larger graph)
- Higher edge reduction (more redundancy initially)
- Scalability demonstration

**Use for:** Figure 8(c) - Consensus overhead analysis

---

### 6.3 Scenario C: Strong Flow

**Purpose:** Test robustness to environmental disturbance

**Parameters:**
- **Number of robots:** $N = 12$
- **Flow magnitude:** $\|f_{\text{flow}}\| \approx 0.6$ m/s ← DOUBLED
- **Flow gradient:** Higher Lipschitz constant $L = 0.5$ (stronger spatial variation)
- **Other:** Same as Scenario A

**Expected Results:**
- Higher control effort (fighting flow)
- Validates A3 (Lipschitz) and A4 (control dominance)
- Still maintains connectivity (robust to flow)

**Use for:** Figure 1(d) - Lipschitz validation with stronger gradients

---

### 6.4 Scenario D: High Stochastic Disturbance

**Purpose:** Test robustness to noise

**Parameters:**
- **Number of robots:** $N = 12$
- **Disturbance:** $\sigma_{\text{diff}} = 0.02$ m/s ← Increased 2.5× (high turbulence)
- **Flow magnitude:** $\|f_{\text{flow}}\| \approx 0.3$ m/s
- **Other:** Same as Scenario A

**Expected Results:**
- More jagged trajectories
- Slightly longer consensus times (noisy observations)
- Validates A2 (bounded disturbance)

**Use for:** Figure 2(b) - Safety validation under noise

---

### 6.5 Scenario E: Sparse Initial Graph

**Purpose:** Test with limited initial connectivity

**Parameters:**
- **Number of robots:** $N = 12$
- **Communication radius:** $R_{\max} = 1.0$ m ← Reduced (sparser)
- **Initial clustering:** Wider spread ($\sigma_{\text{cluster}} = 0.6$ m)
- **Other:** Same as Scenario A

**Expected Results:**
- Fewer initial edges (less pruning needed)
- May start closer to MST
- Validates T3 with near-minimal initial graph

**Use for:** Figure 3(a) - Show different starting topologies

---

### 6.6 Monte Carlo Runs

**For statistical significance:**
- **Runs per scenario:** 10-20 (different random seeds)
- **Reported metrics:** Mean ± std dev
- **Confidence intervals:** 95% (for Figure 8 comparisons)

---

## 7. FIGURE-TO-THEOREM MAPPING

### 7.1 Validation Matrix

| **Figure** | **Primary Theorem** | **Secondary Validation** | **Reviewer Comments** |
|-----------|-------------------|------------------------|---------------------|
| **Figure 1** | T1 (Assumptions) | A2, A3 (flow Lipschitz) | R1-2 (flow triviality) |
| **Figure 2** | T1 (CBF Invariance) | Safety & connectivity constraints | R2-2 ($d_{\min}$ meaning), R1-4 (proof) |
| **Figure 3** | T4 (Pruning) | T3 (spanning backbone) | R1-3 (rigor), R1-4 (proof) |
| **Figure 4** | T3 (Connectivity) | Novel λ₂ estimator | R1-4 (proof) |
| **Figure 5** | T3 (Consensus foundation) | Unanimous decision protocol | R1-3 (formal definitions) |
| **Figure 6** | T2 (CLF Convergence) | Practical stability | R2-1 (stability), R2-3 (Lyapunov) |
| **Figure 7** | Motivation | Energy efficiency | Application justification |
| **Figure 8** | All theorems | Novelty vs. baselines | Overall contribution |

### 7.2 Reviewer Comment Coverage

| **Reviewer ID** | **Concern** | **Addressed in Figure(s)** | **How** |
|----------------|------------|--------------------------|---------|
| **R1-1** | Model confusion | Fig 1(a,b) | Single unified model with flow field |
| **R1-2** | Flow triviality | Fig 1(b,d) | Spatially-varying flow, non-cancellation |
| **R1-3** | Lack of rigor | Fig 3, 5 | Formal graph operations, consensus protocol |
| **R1-4** | No proofs | Fig 2, 3, 4, 6 | Validation of all 4 theorems |
| **R2-1** | Stability unclear | Fig 6 | CLF evolution, goal convergence |
| **R2-2** | $d_{\min}$ meaning | Fig 2(b) | Explicit safety threshold visualization |
| **R2-3** | Lyapunov usage | Fig 6(b,d) | CLF value decay, relaxation variable |
| **R2-4** | Few references | N/A | Literature added in text |

---

## 8. IMPLEMENTATION CHECKLIST

### 8.1 Code Infrastructure

- [ ] **Metrics collection module** (`utils/metrics_collector.py`)
  - Track all metrics in Section 5
  - Export to CSV/JSON for plotting
  - Real-time computation during simulation

- [ ] **Baseline implementations** (`baselines/`)
  - [ ] `full_graph_baseline.py`
  - [ ] `centralized_mst_baseline.py`
  - [ ] `random_pruning_baseline.py`
  - [ ] `greedy_distance_baseline.py`

- [ ] **Scenario configurations** (`config/scenarios.py`)
  - [ ] Scenario A: Baseline
  - [ ] Scenario B: High robot count
  - [ ] Scenario C: Strong flow
  - [ ] Scenario D: High disturbance
  - [ ] Scenario E: Sparse initial graph

- [ ] **Plotting scripts** (`scripts/`)
  - [ ] `plot_figure1_flow_field.py`
  - [ ] `plot_figure2_trajectories_safety.py`
  - [ ] `plot_figure3_pruning_dynamics.py`
  - [ ] `plot_figure4_lambda2_connectivity.py`
  - [ ] `plot_figure5_consensus_convergence.py`
  - [ ] `plot_figure6_goal_convergence_clf.py`
  - [ ] `plot_figure7_control_energy.py`
  - [ ] `plot_figure8_baseline_comparison.py`

- [ ] **Batch simulation runner** (`scripts/run_all_scenarios.py`)
  - Monte Carlo loop
  - Parallel execution (multiprocessing)
  - Progress tracking

### 8.2 Data Management

- [ ] **Output directory structure**
  ```
  results/
    scenario_A/
      run_001/
        metrics.csv
        trajectories.npy
        topology_history.pkl
      run_002/
      ...
    scenario_B/
    ...
  ```

- [ ] **Logging**
  - Simulation parameters (JSON)
  - Console output (log files)
  - Error tracking

### 8.3 Validation Steps

- [ ] **Unit tests for metrics**
  - Test each metric computation
  - Validate against ground truth

- [ ] **Baseline verification**
  - MST: Compare against NetworkX MST
  - Full graph: Verify all edges maintained

- [ ] **Figure generation**
  - Test plotting functions on dummy data
  - Ensure consistent styling (fonts, colors)

### 8.4 Paper Integration

- [ ] **Figure captions** (draft in LaTeX)
- [ ] **Results section text** (one paragraph per figure)
- [ ] **Metrics table** (LaTeX tabular)
- [ ] **Baseline comparison discussion**

---

## 9. TIMELINE

### Day 1 (Jan 31): Infrastructure Setup
- Implement metrics collector
- Create baseline classes
- Set up scenario configurations

### Day 2 (Feb 1): Run Simulations
- Execute Scenario A (10 runs)
- Execute Scenarios B-E (5 runs each)
- Collect and organize data

### Day 3 (Feb 2): Generate Figures
- Create plots for Figures 1-4
- Create plots for Figures 5-8
- Iterate on styling

### Day 4 (Feb 3): Analysis & Write-up
- Compute statistics
- Write results section
- Draft figure captions

### Day 5 (Feb 4): Refinement
- Respond to any issues
- Finalize figures for paper
- Prepare supplementary materials

---

## 10. SUCCESS CRITERIA

### 10.1 Must-Have Results

✅ **All 4 theorems validated:**
- T1: Zero safety/connectivity violations (Fig 2)
- T2: Goal convergence with CLF decay (Fig 6)
- T3: λ₂ always > threshold (Fig 4)
- T4: Zero graph disconnections (Fig 3)

✅ **All reviewer comments addressed:**
- R1-2: Flow non-triviality shown (Fig 1)
- R2-2: $d_{\min}$ clearly defined (Fig 2b)
- R2-1: Stability demonstrated (Fig 6)
- R2-3: Lyapunov usage shown (Fig 6b,d)

✅ **Baseline superiority:**
- Energy: 40-60% savings vs. full graph
- Connectivity: Same as centralized MST
- Robustness: >95% unanimous decisions

### 10.2 Nice-to-Have Results

🎯 Edge reduction to exactly MST (N-1 edges)  
🎯 Consensus convergence in <50 iterations (avg)  
🎯 Zero failed pruning attempts  
🎯 Flow alignment >0.5 (strong exploitation)

---

## 11. NOTES & REMINDERS

### 11.1 Figure Quality Standards

- **Resolution:** Minimum 300 DPI for paper submission
- **Font size:** Minimum 8pt (readable when shrunk to column width)
- **Color scheme:** Colorblind-friendly palette
- **Line widths:** Minimum 1.5pt for main curves
- **Annotations:** Clear, concise, positioned to avoid overlap

### 11.2 Common Pitfalls to Avoid

⚠️ **Don't**: Show results from only 1 random seed (use Monte Carlo)  
⚠️ **Don't**: Claim "always works" without statistical evidence  
⚠️ **Don't**: Use log scale without justification  
⚠️ **Don't**: Overload figures with too many curves (max 5 per plot)  
⚠️ **Don't**: Forget to label axes, add legends, and annotate thresholds

### 11.3 Consistency Checks

- [ ] Same robot count (N=12) across Figures 1-7
- [ ] Same parameter values ($d_{\min}$, $R_{\max}$, etc.) across scenarios
- [ ] Consistent color scheme (robot IDs, method types)
- [ ] Time ranges aligned across temporal plots

---

## 12. REFERENCES FOR IMPLEMENTATION

### 12.1 Existing Code to Leverage

- **Flow field:** `core/flow_field.py` (already implements spatially-varying flow)
- **Lambda2 estimator:** `graph/lambda2_estimator.py` (3-tier adaptive method)
- **Consensus:** `consensus/adjacency_consensus.py` (Griparic protocol)
- **Pruning:** `consensus/hybrid_pruning.py` (multi-layer robustness)
- **Controllers:** `controllers/hybrid_controller.py` (CLF-CBF)
- **Simulation engine:** `simulation/simulation_engine.py` (main loop)
- **Animator:** `visualization/animator.py` (real-time viz)

### 12.2 External Libraries

- **Plotting:** `matplotlib` (all figures), `seaborn` (heatmaps, styling)
- **Graph algorithms:** `networkx` (MST baseline, BFS validation)
- **Numerical:** `numpy` (all computations), `scipy` (eigenvalues, optimization)
- **Data:** `pandas` (metrics tables), `pickle` (save/load states)

---

**END OF SIMULATION PLAN**

**Status:** ✅ Plan finalized and ready for implementation  
**Next Step:** Begin Day 1 infrastructure setup (Jan 31, 2026)

---

*Document prepared by: GitHub Copilot*  
*Date: January 30, 2026*  
*Version: 1.0 - Final*
