# Quick Start: Enhanced Visualizations

## TL;DR - Show Me Better Plots!

```bash
# Install dependencies (if not already installed)
pip install -r requirements.txt

# Generate enhanced plots from existing results
python scripts/regenerate_enhanced_plots.py experiments/results_section_20260219

# For specific result file
python scripts/regenerate_enhanced_plots.py path/to/results.json
```

Your plots will be in `enhanced_plots/` directory with 6 new visualization types!

---

## What You Get

Instead of basic bar charts and box plots, you now get:

### 📈 1. **Time-Series Dynamics** 
Shows HOW algorithms behave over time, not just final results
- When does pruning happen?
- How fast is convergence?
- Is behavior stable?

### 🎯 2. **Trade-off Analysis**
Reveals multi-objective performance with scatter plots + confidence ellipses
- Which method is best overall?
- Edge reduction vs. connectivity preservation
- Performance vs. computational cost

### 🎻 3. **Violin Distributions**
Better than box plots - shows full distribution shape
- Bimodal? Skewed? Consistent?
- Where are the outliers?
- What's the typical vs. worst case?

### 📊 4. **Performance Profiles**
Reliability curves showing success rates
- What % of runs succeed?
- Robustness to variations
- Practical safety guarantees

### 📉 5. **Cumulative Distributions (CDFs)**
Best way to compare distributions statistically
- Which method dominates?
- How significant are differences?
- Percentile comparisons

### 🔥 6. **Correlation Heatmaps**
Find hidden relationships between metrics
- Does reducing edges hurt connectivity?
- Which metrics are coupled?
- Method-specific trade-offs

---

## Usage Options

### Option 1: Automatic (during experiments)

When you run experiments, enhanced plots are now generated automatically:

```bash
python experiments/results_section_20260219/run_three_method_study.py
```

Output structure:
```
run_20260219_143052/
├── raw/
│   └── concurrent_comparison_*.json
├── plots/
│   ├── comparison_bars_ci95.png        # Basic plots
│   ├── comparison_distributions.png
│   └── enhanced/                        # NEW: Enhanced plots
│       ├── dynamics_timeseries.png
│       ├── tradeoff_analysis.png
│       ├── violin_distributions.png
│       ├── performance_profiles.png
│       ├── cdf_comparison.png
│       └── correlation_heatmaps.png
└── summary_metrics.csv
```

### Option 2: Regenerate from existing results

If you already have result files:

```bash
# Automatically finds all result JSONs in directory
python scripts/regenerate_enhanced_plots.py experiments/results_section_20260219

# Or specify exact file
python scripts/regenerate_enhanced_plots.py path/to/specific_results.json

# Custom output location
python scripts/regenerate_enhanced_plots.py results.json --output-dir my_plots

# Filter specific methods
python scripts/regenerate_enhanced_plots.py results.json --methods ConcurrentPruning AdjacencyConsensus
```

### Option 3: Add to existing plot scripts

```bash
# Old way (basic plots only)
python concurrent_pruning/generate_comparison_plots.py results.json

# New way (basic + enhanced)
python concurrent_pruning/generate_comparison_plots.py results.json --enhanced
```

### Option 4: Programmatic

```python
from visualization.enhanced_comparison_plots import generate_all_enhanced_plots

# Generate all 6 plot types
generate_all_enhanced_plots(
    'path/to/results.json',
    output_dir='my_plots',
    methods=['ConcurrentPruning', 'AdjacencyConsensus', 'CentralizedMST']
)

# Or individual plots
from visualization.enhanced_comparison_plots import (
    plot_time_series_dynamics,
    plot_tradeoff_analysis,
    plot_violin_distributions,
    plot_performance_profiles,
    plot_cdf_comparison,
    plot_correlation_heatmap
)

results = load_results('results.json')['results']
plot_violin_distributions(results, 'my_violins.png')
```

---

## Troubleshooting

### "ModuleNotFoundError: No module named 'pandas'"

Install dependencies:
```bash
pip install pandas seaborn scipy
# Or
pip install -r requirements.txt
```

### "No result files found"

Make sure you're pointing to the right directory:
```bash
# List what files exist
ls experiments/results_section_20260219/run_*/raw/*.json

# Then use one of those
python scripts/regenerate_enhanced_plots.py experiments/results_section_20260219/run_20260219_143052/raw/concurrent_comparison_20260219_143241.json
```

### Plots look weird / cut off

Increase figure size or adjust settings in `visualization/enhanced_comparison_plots.py`:
```python
# Line ~23-32
plt.rcParams.update({
    'font.size': 11,  # Increase if text is too small
    'figure.dpi': 150,  # Increase for higher resolution
})
```

---

## For Your Paper

**Best plots for publication:**

1. **Main comparison**: Performance Profiles (shows reliability)
2. **Algorithm behavior**: Time-Series Dynamics (shows convergence)
3. **Trade-offs**: Trade-off Analysis scatter plot
4. **Statistical validation**: CDF Comparison

**Suggested figure captions:**

> **Figure X**: Performance profiles showing the percentage of trials achieving edge reduction and connectivity thresholds. Our method (red) achieves >90% reliability at strict thresholds.

> **Figure Y**: Multi-objective trade-off between network sparsity and connectivity preservation. Markers show individual trials; ellipses indicate 1-σ confidence regions.

> **Figure Z**: Time-series evolution of network metrics. Shaded regions represent inter-quartile range across N=150 trials.

---

## What's Wrong with Bar/Box Plots?

| Issue | Old Plots | Enhanced Plots |
|-------|-----------|----------------|
| Can't see dynamics | ❌ Static endpoints | ✅ Time evolution |
| Hide distribution shape | ❌ Box = single number | ✅ Violins show shape |
| Miss correlations | ❌ Separate axes | ✅ Scatter + heatmaps |
| No reliability info | ❌ Just average | ✅ Success probability |
| Hard to compare statistically | ❌ Need external tests | ✅ CDFs show dominance |
| Don't show trade-offs | ❌ Best in each metric | ✅ Multi-objective view |

---

## Examples

### Compare two result files:

```bash
# Generate plots for both
python scripts/regenerate_enhanced_plots.py experiment1/results.json --output-dir plots1
python scripts/regenerate_enhanced_plots.py experiment2/results.json --output-dir plots2

# Then compare side-by-side
```

### Focus on specific metrics:

Edit `enhanced_comparison_plots.py` to customize which plots are generated or modify metrics shown.

### Batch processing:

```bash
# Process all result directories
for dir in experiments/results_section_20260219/run_*/; do
    python scripts/regenerate_enhanced_plots.py "$dir/raw"
done
```

---

## Next Steps

1. ✅ Install dependencies: `pip install -r requirements.txt`
2. ✅ Generate enhanced plots from your latest results
3. ✅ Compare with old bar/box plots
4. ✅ Read [`docs/ENHANCED_VISUALIZATION_GUIDE.md`](ENHANCED_VISUALIZATION_GUIDE.md) for detailed interpretation guide
5. ✅ Use in your paper/presentations

---

## Need Help?

- **Detailed guide**: See `docs/ENHANCED_VISUALIZATION_GUIDE.md`
- **Code**: `visualization/enhanced_comparison_plots.py`
- **Examples**: Run script on your results and explore outputs
