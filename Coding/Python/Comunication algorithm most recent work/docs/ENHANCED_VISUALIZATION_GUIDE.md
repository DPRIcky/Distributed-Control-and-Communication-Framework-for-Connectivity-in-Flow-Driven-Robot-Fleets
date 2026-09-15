# Enhanced Visualization Guide

## Overview

The enhanced visualization module replaces basic bar charts and box plots with more insightful visualizations that reveal deeper patterns in experimental results.

## New Plot Types

### 1. **Time-Series Dynamics** (`dynamics_timeseries.png`)

**What it shows:**
- Evolution of metrics over iterations (not just final states)
- Rate of convergence and pruning activity
- Temporal patterns and stability

**Insights gained:**
- **When** pruning happens (early aggressive vs. gradual)
- **How fast** consensus is achieved
- **Stability** of connectivity over time
- **Pruning patterns** (burst vs. steady)

**Better than bar charts because:** Shows the full trajectory, not just endpoints. Reveals whether methods achieve results quickly or slowly.

---

### 2. **Trade-off Analysis** (`tradeoff_analysis.png`)

**What it shows:**
- Scatter plots with confidence ellipses
- Competing objectives: sparsity vs. connectivity, performance vs. cost
- Pareto-style frontiers

**Insights gained:**
- **Which method dominates** in multi-objective space
- **Consistency** (tight ellipses = low variance)
- **Trade-off relationships** between objectives
- **Optimal regions** of operation

**Better than separate bar charts because:** Shows correlations and trade-offs. One method might reduce more edges but at cost of connectivity.

---

### 3. **Violin Distributions** (`violin_distributions.png`)

**What it shows:**
- Full probability density of distributions
- Multimodality, skewness, outliers
- Quartiles overlaid on density

**Insights gained:**
- **Distribution shape**: Gaussian? Bimodal? Skewed?
- **Outlier patterns**: Are failures rare or common?
- **Consistency**: Narrow violins = consistent performance
- **Boundary effects**: Do methods cluster at limits?

**Better than box plots because:** Box plots hide distribution shape. Violin plots reveal if results are consistently good or highly variable with occasional good runs.

---

### 4. **Performance Profiles** (`performance_profiles.png`)

**What it shows:**
- Survival curve-style plots
- Percentage of runs achieving each threshold
- Robustness analysis

**Insights gained:**
- **Reliability**: What % of runs succeed?
- **Robustness to variations**: Does method work consistently?
- **Practical guarantees**: "95% of runs achieve X"
- **Risk assessment**: Probability of failure

**Better than bar charts because:** Bar charts show average; profiles show the full success spectrum. Critical for safety-critical systems.

---

### 5. **Cumulative Distribution Functions** (`cdf_comparison.png`)

**What it shows:**
- CDF curves for each metric
- Statistical dominance relationships
- Median, quartiles visible as curve features

**Insights gained:**
- **Stochastic dominance**: One method consistently better?
- **Percentile comparisons**: At any performance level, which is better?
- **Distribution differences**: Are methods fundamentally different?
- **Statistical significance**: Obvious from curve separation

**Better than histograms because:** CDFs are easier to compare visually. Crossing curves indicate no clear winner; separated curves show dominance.

---

### 6. **Correlation Heatmaps** (`correlation_heatmaps.png`)

**What it shows:**
- Correlation matrices between metrics
- Separate heatmap for each method
- Identifies coupling between objectives

**Insights gained:**
- **Metric dependencies**: Does reducing edges hurt connectivity?
- **Method characteristics**: Which method has tightest coupling?
- **Side effects**: Does optimizing X worsen Y?
- **Independent objectives**: Which metrics can improve separately?

**Better than scatter matrices because:** Compact, shows all relationships at once, easy to spot patterns.

---

## Usage

### Generate from existing results:

```bash
python scripts/regenerate_enhanced_plots.py experiments/results_section_20260219
```

### Generate during experiments:

The enhanced plots are now automatically generated when running:

```bash
python experiments/results_section_20260219/run_three_method_study.py
```

### Programmatic usage:

```python
from visualization.enhanced_comparison_plots import generate_all_enhanced_plots

generate_all_enhanced_plots(
    'path/to/results.json',
    output_dir='path/to/plots',
    methods=['ConcurrentPruning', 'AdjacencyConsensus']
)
```

## Installation

Install required dependencies:

```bash
pip install -r requirements.txt
```

New dependencies:
- `pandas>=1.3.0` - Data manipulation
- `seaborn>=0.11.0` - Statistical visualization
- `scipy>=1.7.0` - Statistical functions

## What's Wrong with Bar/Box Plots?

### Bar Charts:
- ❌ Hide variability (error bars don't show distribution shape)
- ❌ Don't show outliers clearly
- ❌ Can't see temporal dynamics
- ❌ Don't reveal correlations between metrics
- ❌ Summary statistics can be misleading

### Box Plots:
- ❌ Hide distribution shape (bimodal looks like unimodal)
- ❌ Don't show density (sparse vs. concentrated data)
- ❌ Can't assess normality
- ❌ Difficult to see small differences
- ❌ Don't show relationships between metrics

## Interpretation Guide

### For Paper/Publication:

1. **Main comparison**: Use **Performance Profiles** (shows reliability)
2. **Trade-offs**: Use **Trade-off Analysis** (shows multi-objective performance)
3. **Statistical validation**: Use **CDF Comparison** (shows dominance)
4. **Dynamics**: Use **Time-Series** (shows convergence behavior)

### For Debugging:

1. **Correlation Heatmaps**: Find unexpected dependencies
2. **Violin Distributions**: Spot bimodal failures
3. **Time-Series**: See when things go wrong
4. **Scatter + ellipses**: Identify outlier conditions

### For System Tuning:

1. **Performance Profiles**: Set safety margins
2. **Trade-off Analysis**: Choose operating point
3. **Time-Series**: Tune convergence speed
4. **Violin Distributions**: Understand worst-case

## Examples

### Good Performance Pattern:
- **Violin**: Narrow, symmetric, centered away from boundaries
- **Performance Profile**: High percentages at strict thresholds
- **CDF**: Left-shifted (for minimization) with steep rise
- **Time-Series**: Fast convergence with low variance
- **Trade-off**: Tight ellipse in optimal region

### Problem Pattern:
- **Violin**: Bimodal or very wide → inconsistent
- **Performance Profile**: Gradual slope → unreliable
- **CDF**: Spread out or flat regions → high variance
- **Time-Series**: Late convergence or oscillations
- **Trade-off**: Large ellipse or poor location

## Comparison Table

| Plot Type | Old (Bar/Box) | New (Enhanced) | Key Insight |
|-----------|--------------|----------------|-------------|
| Central tendency | ✓ | ✓ | Mean/median |
| Variability | △ (error bars) | ✓ | Full distribution |
| Outliers | △ | ✓ | Clearly visible |
| Distribution shape | ✗ | ✓ | Skew, modes |
| Temporal dynamics | ✗ | ✓ | Evolution |
| Correlations | ✗ | ✓ | Dependencies |
| Trade-offs | ✗ | ✓ | Multi-objective |
| Reliability | ✗ | ✓ | Success probability |
| Statistical dominance | ✗ | ✓ | Clear from CDFs |

**Legend:** ✓ = Yes, △ = Partial, ✗ = No

---

## Future Enhancements

Potential additions:
- **Ridge plots**: For comparing many conditions
- **Parallel coordinates**: For high-dimensional comparison
- **Animated GIFs**: Showing temporal evolution
- **Interactive plots**: Using Plotly for exploration
- **Uncertainty quantification**: Bayesian confidence regions
