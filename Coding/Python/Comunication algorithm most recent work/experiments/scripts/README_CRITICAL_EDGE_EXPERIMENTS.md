# Critical Edge Detection - Experimental Results Guide

## Quick Start

This guide shows you how to run comprehensive experiments comparing the **Critical Edge Detection** method with existing baselines.

### Step 1: Run Experiments

Generate comprehensive experimental results (5 methods × 5 scenarios × 5 trials):

```bash
cd experiments/scripts
python run_experiments_with_critical_edge.py
```

**Options:**
```bash
# Quick mode (2 trials, 300 steps) - 5 minutes
python run_experiments_with_critical_edge.py --quick

# Full experiments (5 trials, 500 steps) - 30-45 minutes
python run_experiments_with_critical_edge.py --trials 5 --steps 500

# Custom configuration
python run_experiments_with_critical_edge.py --trials 3 --steps 400 --output my_results.json
```

**Output:** 
- Results saved to: `experiments/comprehensive_results_with_critical_edge_YYYYMMDD_HHMMSS.json`
- Contains 125 total simulations (5 methods × 5 scenarios × 5 trials)

### Step 2: Generate Plots

Automatically finds the most recent results and generates all plots:

```bash
cd experiments/scripts
python generate_plots_with_critical_edge.py
```

**Generated Plots:**

1. **plot_0_summary_comparison.png** - 6-panel comparison of all metrics
   - Success rates, edge reduction, connectivity, control effort, violations
   
2. **plot_0a_scenario_heatmap.png** - Performance heatmaps by method × scenario
   - Min λ₂, edge reduction, success rate, control effort
   
3. **plot_1_edge_evolution.png** - Edge count over time
   - Scenario A (all methods)
   - Critical Edge Detection (all scenarios)
   - Scenario C and D comparisons
   
4. **plot_2_lambda2_analysis.png** - Connectivity analysis
   - λ₂ evolution for baseline and challenging scenarios
   - Min and mean λ₂ comparisons by scenario
   
5. **plot_3_efficiency.png** - Efficiency metrics
   - Edge reduction by scenario
   - Control effort by scenario
   - Pareto trade-off: sparsity vs connectivity
   - Pruning activity
   
6. **summary_statistics_with_critical_edge.txt** - Detailed numerical results

All files saved to: `experiments/figures/`

---

## Methods Compared

### 1. **Critical Edge Detection** ⭐ (NEW)
- **Type:** Distributed MST with bridge preservation
- **Key Features:**
  - Guarantees all critical edges (bridges) preserved
  - O(n) communication rounds
  - Fully decentralized
  - Proven correctness with formal proofs
- **Expected:** Balanced pruning with guaranteed connectivity preservation

### 2. **Concurrent Pruning** (Proposed Baseline)
- **Type:** Lyapunov-based distributed pruning
- **Key Features:**
  - Uses Lyapunov-based edge evaluation
  - Bilateral agreement for edge removal
  - Predicts future connectivity
- **Expected:** Strong safety guarantees with aggressive pruning

### 3. **Adjacency Consensus** (Distributed Baseline)
- **Type:** Consensus-based pruning
- **Key Features:**
  - Consensus on adjacency matrix
  - Conservative pruning
  - Higher edge retention
- **Expected:** Safe but slower pruning

### 4. **FullGraph** (No Pruning Baseline)
- **Type:** Baseline - maintain all edges
- **Key Features:**
  - No pruning performed
  - Maximum connectivity and cost
- **Expected:** Highest λ₂, most edges, highest control effort

### 5. **Centralized MST** (Oracle Baseline)
- **Type:** Centralized optimal using global knowledge
- **Key Features:**
  - Kruskal's algorithm with global knowledge
  - Maximum pruning (minimal tree)
  - Not implementable in distributed setting
- **Expected:** Minimum possible edges, potentially unsafe

---

## Test Scenarios

| Scenario | Name | Challenge | Robots | Radius | dt | Flow |
|----------|------|-----------|--------|--------|-----|------|
| A | Baseline | ⭐⭐ Medium | 10 | 3.0m | 0.05s | 0.3 m/s |
| B | Large Timestep | ⭐⭐⭐ High | 10 | 3.5m | 0.1s | 0.3 m/s |
| C | Tight Radius | ⭐⭐⭐⭐ Very High | 10 | 2.0m | 0.03s | 0.3 m/s |
| D | High Robot Count | ⭐⭐⭐ High | 20 | 3.0m | 0.05s | 0.3 m/s |
| E | Extreme Parameters | ⭐⭐⭐⭐⭐ Extreme | 15 | 2.5m | 0.08s | 0.5 m/s |

---

## Expected Results

### Critical Edge Detection Performance

Based on algorithmic properties:

| Metric | Expected Performance |
|--------|----------------------|
| **Success Rate** | 95-100% (guaranteed by spanning tree) |
| **Edge Reduction** | 60-90% depending on network structure |
| **Min λ₂** | 0.15-0.25 (safe connectivity) |
| **Mean λ₂** | 0.30-0.50 (very healthy) |
| **Control Effort** | Low-Medium (efficient pruning) |
| **Pruning Activity** | Moderate (periodic MST updates) |

### Comparison Summary

- **vs Concurrent Pruning:** Similar performance, different mechanisms
- **vs Adjacency Consensus:** More aggressive pruning, better efficiency
- **vs FullGraph:** Massive edge reduction while maintaining connectivity
- **vs Centralized MST:** Similar edge reduction, truly distributed (no oracle)

---

## Interpreting the Results

### Key Metrics

1. **Success Rate (%)** - Percentage of simulations that maintained connectivity
   - Target: >95%
   - Critical Edge Detection: Guaranteed as valid spanning tree

2. **Edge Reduction (%)** - Network sparsity achieved
   - Target: 60-90%
   - Critical Edge Detection expected: 70-85%

3. **Min λ₂ (Algebraic Connectivity)** - Worst-case connectivity during simulation
   - Safety Threshold: 0.2 (can be tuned)
   - Target: >0.15 (for safety margin)
   - Critical Edge Detection: 0.20-0.35

4. **Mean λ₂** - Average connectivity during simulation
   - Target: >0.3
   - Critical Edge Detection: 0.35-0.50

5. **Control Effort** - Cumulative control force applied
   - Lower is better (less actuation required)
   - Indicates network efficiency

6. **Pruning Events** - Number of edges removed
   - Higher indicates more aggressive pruning
   - Critical Edge Detection: Periodic (every 10 steps)

---

## Running Custom Experiments

### Variant 1: Compare only with Concurrent Pruning

Edit `run_experiments_with_critical_edge.py`:
```python
methods = [
    'ConcurrentPruning',
    'CriticalEdgeDetection'
]
```

### Variant 2: Focus on one scenario

Modify the experiment script to test specific scenarios:
```python
scenarios = ['C']  # Only test tight radius scenario
```

### Variant 3: Increase trial count for statistical significance

```bash
python run_experiments_with_critical_edge.py --trials 10  # 10 trials per config
```

---

## Troubleshooting

### Issue: "Graph not connected" errors
**Solution:** Increase communication radius in scenario definition or adjust control tuning

### Issue: Results file not found
**Solution:** Ensure you're running from `experiments/scripts/` directory

### Issue: Plots not generating
**Solution:** Check that results JSON file exists and contains successful trials

### Issue: Memory usage too high
**Solution:** Reduce `--trials` or `--steps`, or reduce the sample rate in plot generation (currently saves 1% of history)

---

## Publication-Ready Output

All plots are generated in publication-quality format:
- PNG files (300 DPI) for presentations
- Can be converted to PDF using ImageMagick or design tools
- Properly scaled fonts and colors
- Professional color scheme

---

## File Locations

```
Critical_edge_detection_baseline/
├── README.md                          # Algorithm documentation
├── distributed_critical_mst.py        # Core algorithm
├── critical_edge_baseline.py          # Baseline class
└── critical_edge_simulation.py        # Full simulation

experiments/
├── scripts/
│   ├── run_experiments_with_critical_edge.py      # Run experiments
│   ├── generate_plots_with_critical_edge.py       # Generate plots
│   └── README.md                                   # This file
├── figures/
│   ├── plot_0_summary_comparison.png
│   ├── plot_0a_scenario_heatmap.png
│   ├── plot_1_edge_evolution.png
│   ├── plot_2_lambda2_analysis.png
│   ├── plot_3_efficiency.png
│   └── summary_statistics_with_critical_edge.txt
└── comprehensive_results_with_critical_edge_*.json  # Raw results
```

---

## Citation

If publishing results using this framework, cite:

```bibtex
@misc{critical_edge_detection_experiments_2026,
  author = {Distributed Systems Research},
  title = {Distributed Critical Edge Detection for Multi-Robot Communication Networks},
  year = {2026},
  note = {Experimental framework comparing CED with baselines}
}
```

---

## Additional Resources

- [Critical Edge Detection README](../../Critical_edge_detection_baseline/README.md)
- [Experiment Plan](../EXPERIMENT_PLAN.md)
- [Original Plotting Guide](../PLOTTING_GUIDE.md)

---

**Last Updated:** February 24, 2026
**Status:** Production Ready ✓
