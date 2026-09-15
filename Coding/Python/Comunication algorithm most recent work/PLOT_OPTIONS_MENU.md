# Plot Options Menu - Choose What to Visualize

Based on your experimental data, here are all possible plot types organized by purpose. Choose which ones you want to generate.

---

## 📊 Available Data Points

From your JSON results, each method records:
- Initial/final edge counts
- Edge reduction percentage
- Pruning events (when, which edge)
- Lambda2 history (connectivity over time)
- Consensus metric history
- Runtime/computational cost
- Multiple trials (N=150) per method

---

## Plot Categories

### A. DISTRIBUTION PLOTS (Show Statistical Properties)

#### A1. **Box Plots** (Current - Basic)
```
What: Shows median, quartiles, outliers for each metric
When to use: Quick statistical summary
Pros: Standard, easy to understand
Cons: Hides distribution shape, misses multimodality
```
**Metrics available:**
- Edge reduction %
- Min/mean/final λ₂
- Runtime
- Pruning events count

#### A2. **Violin Plots** ⭐ RECOMMENDED
```
What: Box plot + full probability density
When to use: Want to see distribution shape
Pros: Shows bimodality, skewness, consistency
Cons: Slightly harder to interpret
```

#### A3. **Histogram + KDE**
```
What: Histogram bars with smoothed density curve
When to use: Detailed distribution analysis
Pros: Shows exact frequency bins
Cons: Can be cluttered with multiple methods
```

#### A4. **Ridgeline Plots**
```
What: Overlapping density curves (looks like mountain ridges)
When to use: Comparing 3+ methods elegantly
Pros: Beautiful, shows all distributions at once
Cons: Hard to read exact values
```

---

### B. COMPARISON PLOTS (Method vs Method)

#### B1. **Bar Charts with Error Bars** (Current - Basic)
```
What: Mean ± CI for each metric
When to use: Quick comparison of averages
Pros: Simple, clear
Cons: Hides variability, only shows summary stats
```

#### B2. **Grouped Bar Charts**
```
What: Multiple metrics side-by-side for each method
When to use: Compare multiple metrics simultaneously
Pros: Compact, good for 2-3 metrics
Cons: Gets cluttered with >3 metrics
```

#### B3. **Radar/Spider Charts**
```
What: Polygon showing performance across multiple dimensions
When to use: Multi-objective comparison
Pros: Shows trade-offs at a glance
Cons: Area interpretation can be misleading
```

#### B4. **Heat Map Comparison**
```
What: Color-coded matrix of methods × metrics
When to use: Many methods, many metrics
Pros: Compact, easy pattern recognition
Cons: Doesn't show variability
```

---

### C. TIME-SERIES PLOTS (Dynamics Over Iterations)

#### C1. **Line Plots with Confidence Bands** ⭐ RECOMMENDED
```
What: Median line + shaded IQR/CI region
When to use: Show convergence behavior
Pros: Shows dynamics + uncertainty
Cons: Can be cluttered with 4+ methods
```
**Available time series:**
- Edge count evolution
- λ₂ evolution
- Consensus metric evolution
- Cumulative pruning events

#### C2. **Stacked Area Chart**
```
What: Multiple time series stacked on top
When to use: Show cumulative contributions
Pros: Shows total + breakdown
Cons: Bottom series compressed
```

#### C3. **Step Plot**
```
What: Discrete jumps at pruning events
When to use: Emphasize discrete nature of pruning
Pros: Shows exact timing of events
Cons: Looks jagged
```

#### C4. **Animation/GIF**
```
What: Animated progression over iterations
When to use: Presentations, want to show dynamics
Pros: Engaging, shows process
Cons: Not for papers, needs video support
```

---

### D. CORRELATION PLOTS (Relationships Between Metrics)

#### D1. **Scatter Plots**
```
What: X-Y plot of two metrics
When to use: Investigate relationships
Pros: Shows correlations, clusters, outliers
Cons: Limited to 2D (can use color for 3rd)
```
**Interesting pairs:**
- Edge reduction % vs. min λ₂ (trade-off)
- Edge reduction % vs. runtime (efficiency)
- Pruning events vs. final connectivity
- Initial edges vs. final edges

#### D2. **Scatter with Confidence Ellipses** ⭐ RECOMMENDED
```
What: Scatter + statistical ellipses showing spread
When to use: Multi-objective comparison with uncertainty
Pros: Shows performance + consistency
Cons: Needs enough samples (N>20)
```

#### D3. **Hexbin Plots**
```
What: 2D histogram (hexagonal bins)
When to use: Dense scatter with 100+ points
Pros: Shows density where points overlap
Cons: Loses individual point info
```

#### D4. **Correlation Heatmap**
```
What: Matrix showing pairwise correlations
When to use: Explore many metric relationships
Pros: Comprehensive overview
Cons: Doesn't show raw data
```

---

### E. PERFORMANCE PROFILES (Success/Reliability)

#### E1. **Performance Profiles (Cumulative)** ⭐ RECOMMENDED
```
What: % of runs achieving each threshold
When to use: Assess reliability and robustness
Pros: Shows practical guarantees
Cons: Needs threshold definition
```
**Can analyze:**
- % achieving edge reduction targets (10%, 20%, ...)
- % maintaining λ₂ above safety threshold
- % completing within time budget

#### E2. **Survival Curves**
```
What: Kaplan-Meier style curves
When to use: Time-to-event analysis
Pros: Standard in reliability engineering
Cons: Similar to performance profiles
```

#### E3. **ROC-style Curves**
```
What: Trade-off curves (e.g., precision vs. recall)
When to use: Threshold-dependent performance
Pros: Shows full spectrum of trade-offs
Cons: Requires defining "positive" outcome
```

---

### F. CUMULATIVE DISTRIBUTION FUNCTIONS (CDFs)

#### F1. **Empirical CDFs** ⭐ RECOMMENDED
```
What: Cumulative probability for each value
When to use: Statistical comparison of methods
Pros: Shows dominance, easy to compare
Cons: Less intuitive than histograms
```
**For each metric:**
- Edge reduction CDF
- λ₂ CDF
- Runtime CDF

#### F2. **QQ Plots**
```
What: Quantile-quantile comparison
When to use: Check if distributions differ
Pros: Statistical rigor
Cons: Requires stats background to interpret
```

---

### G. SPECIALIZED PLOTS

#### G1. **Pareto Front**
```
What: Non-dominated solutions in multi-objective space
When to use: Optimization analysis
Pros: Shows optimal trade-offs
Cons: Requires clear objectives
```
**Objectives:**
- Minimize edges + Maximize λ₂
- Minimize edges + Minimize runtime

#### G2. **Taylor Diagram**
```
What: Polar plot showing correlation, std dev, RMSE
When to use: Model validation
Pros: Comprehensive performance summary
Cons: Complex interpretation
```

#### G3. **Bland-Altman Plot**
```
What: Difference vs. average plot
When to use: Agreement between methods
Pros: Shows systematic biases
Cons: Mainly for comparing two methods
```

#### G4. **Alluvial/Sankey Diagram**
```
What: Flow diagram showing state transitions
When to use: Visualize edge removal process
Pros: Beautiful, shows process flow
Cons: Complex to implement
```

---

## 🎯 Recommended Plot Combinations

### For Paper (4 figures max):

**Figure 1: Main Results**
- Type: **Violin plots** (A2)
- Metrics: Edge reduction %, Min λ₂, Runtime
- Why: Shows distribution shape + statistical significance

**Figure 2: Dynamics**
- Type: **Line plots with confidence bands** (C1)
- Metrics: Edge count, λ₂ evolution over iterations
- Why: Shows HOW algorithms behave, not just final states

**Figure 3: Trade-offs**
- Type: **Scatter with ellipses** (D2)
- Axes: Edge reduction % vs. Min λ₂
- Why: Shows multi-objective performance

**Figure 4: Reliability**
- Type: **Performance profiles** (E1)
- Thresholds: λ₂ > 0.3 (safety), edge reduction targets
- Why: Shows practical success rates

---

### For Detailed Analysis (8+ figures):

Add to above:

**Figure 5: CDFs**
- Type: **Empirical CDFs** (F1)
- Metrics: All key metrics
- Why: Statistical dominance proof

**Figure 6: Correlations**
- Type: **Correlation heatmap** (D4)
- Why: Understand metric relationships

**Figure 7: Pruning Activity**
- Type: **Step plots** (C3)
- Why: See when pruning happens

**Figure 8: Pareto Analysis**
- Type: **Pareto front** (G1)
- Why: Multi-objective optimality

---

## 📋 Selection Template

Please select plots using this format:

```
DISTRIBUTION PLOTS:
[ ] A1. Box plots
[X] A2. Violin plots ← (mark with X)
[ ] A3. Histogram + KDE
[ ] A4. Ridgeline plots

TIME-SERIES PLOTS:
[X] C1. Line plots with confidence bands
[ ] C2. Stacked area
[ ] C3. Step plots
[ ] C4. Animation

CORRELATION PLOTS:
[X] D2. Scatter with ellipses
[ ] D4. Correlation heatmap

PERFORMANCE PROFILES:
[X] E1. Performance profiles

CDFs:
[X] F1. Empirical CDFs

SPECIALIZED:
[ ] G1. Pareto front
```

---

## 💡 Questions to Help You Choose

1. **What's your main message?**
   - "Our method is faster" → Focus on runtime plots
   - "Our method balances trade-offs" → Multi-objective scatter
   - "Our method is reliable" → Performance profiles
   - "Our method converges smoothly" → Time-series

2. **Who's your audience?**
   - Theoreticians → CDFs, statistical rigor
   - Practitioners → Reliability, practical guarantees
   - General → Violin plots, trade-off scatter

3. **Page budget?**
   - 2-3 figures → Violin + Time-series + Scatter
   - 4-6 figures → Add Performance profiles + CDFs
   - 8+ figures → Full suite for appendix

4. **What surprised you in results?**
   - Adjacency consensus doesn't prune → Show distribution close to 0%
   - MST has low λ₂ → Show λ₂ near threshold in violin
   - Your method balanced → Show scatter with optimal region

---

## Next Steps

**Please tell me:**
1. Which plots you want (use X marks above)
2. Any specific comparisons you're interested in
3. Priority order (which to generate first)
4. Any custom plot ideas

Then I'll generate exactly what you need!
