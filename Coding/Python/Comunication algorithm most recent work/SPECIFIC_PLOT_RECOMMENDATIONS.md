# Specific Plot Recommendations: What vs What

## Data Available (Per Trial)
From your experiment results, each trial has:
- **Scalars**: edge_reduction_pct, min_lambda2, mean_lambda2, final_lambda2, elapsed_time, pruning_events_count, iterations, initial_edges, final_edges
- **Time series**: edge_counts_history, lambda2_history, consensus_history
- **Categorical**: method (ConcurrentPruning, AdjacencyConsensus, CentralizedMST)
- **N trials**: 150 per method

---

## 🎯 Recommended Plots: Specific X vs Y

### CATEGORY 1: Trade-off Analysis (Most Important)

#### Plot 1.1: **Edge Reduction vs Connectivity** ⭐⭐⭐ CRITICAL
```
X-axis: Edge Reduction (%)
Y-axis: Minimum λ₂ (connectivity)
Points: Each trial (colored by method)
Style: Scatter plot with confidence ellipses

WHY: Shows the fundamental trade-off
- Top-right = ideal (high pruning + high connectivity)
- Your method should be middle-right
- Adjacency = top-left (low pruning, high connectivity)
- MST = bottom-right (high pruning, low connectivity)

INSIGHT: Which method best balances sparsity vs connectivity?
```

#### Plot 1.2: **Edge Reduction vs Runtime**
```
X-axis: Edge Reduction (%)
Y-axis: Computation Time (seconds)
Points: Each trial (colored by method)
Style: Scatter plot

WHY: Shows efficiency trade-off
- Bottom-right = ideal (fast + high pruning)
- MST should be bottom-right (fast, aggressive)
- Adjacency should be top-left (slow, no pruning)

INSIGHT: Is pruning worth the computational cost?
```

#### Plot 1.3: **Minimum λ₂ vs Runtime**
```
X-axis: Computation Time (seconds)
Y-axis: Minimum λ₂
Points: Each trial
Style: Scatter plot

WHY: Shows cost of safety
- Top-left = ideal (high connectivity, fast)
- Shows if spending more time improves safety

INSIGHT: Diminishing returns on computation?
```

---

### CATEGORY 2: Performance Distributions (Show Variability)

#### Plot 2.1: **Method vs Edge Reduction** ⭐⭐⭐ ESSENTIAL
```
X-axis: Method (categorical: 3 methods)
Y-axis: Edge Reduction (%)
Style: Violin plot or box plot

WHY: Primary outcome comparison
- Shows central tendency + spread
- Easy to see if methods differ significantly

INSIGHT: Which method prunes most? How consistent?
```

#### Plot 2.2: **Method vs Minimum λ₂** ⭐⭐⭐ ESSENTIAL
```
X-axis: Method
Y-axis: Minimum λ₂
Style: Violin plot with safety threshold line at y=0.3

WHY: Safety criterion
- Shows if methods maintain connectivity
- Violin shows risk of dropping below threshold

INSIGHT: Which method is safest? Any violations?
```

#### Plot 2.3: **Method vs Runtime**
```
X-axis: Method
Y-axis: Elapsed Time (seconds)
Style: Violin plot

WHY: Practical deployment consideration
- Shows computational burden
- Variance shows consistency

INSIGHT: Which is fastest? Most predictable?
```

#### Plot 2.4: **Method vs Pruning Events**
```
X-axis: Method
Y-axis: Number of Pruning Events
Style: Violin plot

WHY: Shows algorithm activity
- More events = more aggressive pruning
- Relates to convergence behavior

INSIGHT: How many decisions does each method make?
```

---

### CATEGORY 3: Temporal Dynamics (Show Evolution)

#### Plot 3.1: **Iteration vs Edge Count** ⭐⭐⭐ CRITICAL
```
X-axis: Iteration number
Y-axis: Number of Edges
Lines: One per method (median + IQR band)
Style: Line plot with confidence bands

WHY: Shows pruning dynamics
- Steep = aggressive pruning
- Gradual = conservative
- Flat = no pruning

INSIGHT: When does pruning happen? How fast?
```

#### Plot 3.2: **Iteration vs λ₂** ⭐⭐⭐ CRITICAL
```
X-axis: Iteration number
Y-axis: Algebraic Connectivity (λ₂)
Lines: One per method (median + IQR band)
Horizontal line: Safety threshold at y=0.3
Style: Line plot with confidence bands

WHY: Shows connectivity preservation over time
- Dips below 0.3 = safety violation
- Stable = good control
- Volatile = risky

INSIGHT: Do methods maintain safety? When are risky moments?
```

#### Plot 3.3: **Iteration vs Consensus Metric**
```
X-axis: Iteration number
Y-axis: Consensus Metric (log scale)
Lines: One per method
Style: Line plot (log-scale Y)

WHY: Shows convergence to agreement
- Exponential decay = good convergence
- Flat = no consensus

INSIGHT: How fast do robots agree?
```

#### Plot 3.4: **Iteration vs Pruning Rate**
```
X-axis: Iteration number
Y-axis: Edges Pruned Per Iteration
Lines: One per method
Style: Step plot or bar chart

WHY: Shows pruning activity over time
- Spikes = burst pruning
- Spread = gradual pruning

INSIGHT: Conservative initially or aggressive from start?
```

---

### CATEGORY 4: Cumulative/Statistical (Show Reliability)

#### Plot 4.1: **Edge Reduction Threshold vs Success Rate** ⭐⭐ IMPORTANT
```
X-axis: Edge Reduction Threshold (0% to 100%)
Y-axis: Percentage of Trials Achieving Threshold
Lines: One per method
Style: Performance profile (cumulative curve)

WHY: Shows reliability at different targets
- Steeper curve = consistent performance
- Higher curve = better performance

INSIGHT: "90% of runs achieve >X% reduction" guarantees
```

#### Plot 4.2: **λ₂ Threshold vs Safety Probability** ⭐⭐ IMPORTANT
```
X-axis: λ₂ Threshold value
Y-axis: Percentage of Trials ≥ Threshold
Lines: One per method
Vertical line: Safety requirement at x=0.3
Style: Performance profile

WHY: Shows safety guarantees
- y-value at x=0.3 = probability of meeting safety

INSIGHT: "Method X is safe 95% of time"
```

#### Plot 4.3: **Edge Reduction CDF**
```
X-axis: Edge Reduction (%)
Y-axis: Cumulative Probability (0 to 1)
Lines: One per method
Style: Empirical CDF

WHY: Statistical comparison
- Left curve dominates (achieves reduction at lower percentiles)
- Crossing curves = no clear winner

INSIGHT: Statistical significance of differences
```

#### Plot 4.4: **λ₂ CDF**
```
X-axis: Minimum λ₂
Y-axis: Cumulative Probability
Lines: One per method
Style: Empirical CDF

WHY: Distribution of safety margins
- Right-shifted = safer on average

INSIGHT: What's typical safety margin?
```

---

### CATEGORY 5: Multi-Metric Relationships (Advanced)

#### Plot 5.1: **Initial Edges vs Final Edges**
```
X-axis: Initial Edge Count
Y-axis: Final Edge Count
Points: Each trial (colored by method)
Diagonal line: y=x (no pruning)
Style: Scatter plot

WHY: Shows sensitivity to initial topology
- Points below y=x = pruning occurred
- Parallel to y=n-1 = aggressive pruning to MST

INSIGHT: Do methods adapt to initial density?
```

#### Plot 5.2: **Edge Reduction vs Iterations**
```
X-axis: Edge Reduction (%)
Y-axis: Number of Iterations to Converge
Points: Each trial (colored by method)
Style: Scatter plot

WHY: Shows convergence speed
- Bottom-right = fast + effective
- Top = needs many iterations

INSIGHT: Does more pruning take longer?
```

#### Plot 5.3: **Pruning Events vs Final λ₂**
```
X-axis: Number of Pruning Decisions Made
Y-axis: Final λ₂
Points: Each trial (colored by method)
Style: Scatter plot

WHY: Shows impact of decision count on safety
- Many prunes + high λ₂ = careful pruning
- Many prunes + low λ₂ = aggressive

INSIGHT: Quality vs quantity of decisions
```

---

### CATEGORY 6: Correlation Analysis

#### Plot 6.1: **Correlation Heatmap (Per Method)**
```
Grid: All metrics vs all metrics
Values: Correlation coefficients
Colors: Red (positive) to Blue (negative)
Style: Heatmap (one per method, or side-by-side)

Metrics to correlate:
- edge_reduction_pct
- min_lambda2
- mean_lambda2
- elapsed_time
- pruning_events_count
- iterations

WHY: Find hidden relationships
- Negative correlation edge_reduction vs lambda2 = trade-off
- Positive correlation time vs events = more work

INSIGHT: Which metrics are coupled?
```

---

## 🎯 My Top 5 Recommendations (Must-Have)

### For Paper Main Text:

**Figure 1: Fundamental Trade-off**
```
Plot 1.1: Edge Reduction (%) vs Minimum λ₂
- Shows your method achieves middle ground
- MST = aggressive but unsafe
- Adjacency = safe but ineffective
- Your method = optimal balance
```

**Figure 2: Performance Distributions**
```
Plot 2.1 + 2.2 side-by-side:
Left: Method vs Edge Reduction
Right: Method vs Minimum λ₂
- Shows statistical significance
- Violin plots show consistency
```

**Figure 3: Temporal Dynamics**
```
Plot 3.1 + 3.2 in same figure (2 subplots):
Top: Edge Count Evolution
Bottom: λ₂ Evolution (with safety line)
- Shows HOW algorithms work, not just outcomes
- Your method: gradual pruning, stable connectivity
```

**Figure 4: Reliability Analysis**
```
Plot 4.2: λ₂ Threshold vs Success Rate
- Shows practical safety guarantees
- "Our method maintains λ₂>0.3 in 98% of trials"
```

**Figure 5 (Optional): Efficiency**
```
Plot 1.2: Edge Reduction vs Runtime
- Shows computational cost vs benefit
- Your method: reasonable cost for good performance
```

---

## 📋 Selection Form

Check which plots you want (I'll generate them in order):

### Critical (Pick at least 2):
- [ ] **1.1**: Edge Reduction vs λ₂ (TRADE-OFF)
- [ ] **2.1**: Method vs Edge Reduction (MAIN RESULT)
- [ ] **2.2**: Method vs λ₂ (SAFETY)
- [ ] **3.1**: Iteration vs Edge Count (DYNAMICS)
- [ ] **3.2**: Iteration vs λ₂ (SAFETY DYNAMICS)

### Important (Pick 1-2):
- [ ] **1.2**: Edge Reduction vs Runtime (EFFICIENCY)
- [ ] **2.3**: Method vs Runtime (COST)
- [ ] **4.1**: Edge Reduction Success Rate (RELIABILITY)
- [ ] **4.2**: λ₂ Success Rate (SAFETY GUARANTEE)

### Supplementary (Pick if space):
- [ ] **1.3**: λ₂ vs Runtime
- [ ] **2.4**: Method vs Pruning Events
- [ ] **3.3**: Iteration vs Consensus
- [ ] **3.4**: Iteration vs Pruning Rate
- [ ] **4.3**: Edge Reduction CDF
- [ ] **4.4**: λ₂ CDF
- [ ] **5.1**: Initial vs Final Edges
- [ ] **5.2**: Edge Reduction vs Iterations
- [ ] **5.3**: Pruning Events vs λ₂
- [ ] **6.1**: Correlation Heatmap

---

## 💡 What Each Plot Will Tell You

| Plot | Main Insight | Why You Need It |
|------|--------------|-----------------|
| 1.1 | **Is there a trade-off?** | Core thesis of your work |
| 2.1 | **Does your method prune more?** | Prove effectiveness |
| 2.2 | **Is your method safe?** | Prove feasibility |
| 3.1 | **When does pruning happen?** | Understand dynamics |
| 3.2 | **Does connectivity stay safe?** | Show real-time safety |
| 1.2 | **Is the cost worth it?** | Practical deployment |
| 4.2 | **How reliable is safety?** | Engineering guarantees |

---

## Next Step

**Please tell me:**
1. Which plots from the list above (use plot numbers like "1.1, 2.1, 3.2")
2. Order of priority (which to generate first)
3. Any specific comparisons I missed that you're curious about?

Example response:
```
Generate these in order:
1. Plot 1.1 (Edge Reduction vs λ₂) - want to see trade-off
2. Plot 2.1 + 2.2 (Method comparisons) - main results
3. Plot 3.1 + 3.2 (Dynamics) - show process
4. Plot 4.2 (Safety reliability) - for discussion
```

Then I'll generate exactly those plots with the right insights!
