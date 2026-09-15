# Concurrent Pruning Batch Comparison

This directory contains scripts for running comprehensive batch comparisons of the Concurrent Pruning method against baseline approaches.

## Quick Start

### 1. Run Batch Comparison (125 trials)

```bash
# Default: 15 robots, density=0.7, 125 seeds
python concurrent_pruning/batch_comparison.py

# Custom configuration
python concurrent_pruning/batch_comparison.py --robots 20 --density 0.6 --seeds 125
```

**Expected runtime:** ~15-30 minutes for 125 seeds (depends on robot count)

**Output:** Results saved to `concurrent_pruning/results/concurrent_comparison_YYYYMMDD_HHMMSS.json`

### 2. Generate Comparison Plots

```bash
python concurrent_pruning/generate_comparison_plots.py concurrent_pruning/results/concurrent_comparison_YYYYMMDD_HHMMSS.json
```

**Output:** Figures saved to `concurrent_pruning/figures/`
- `comparison_boxplots.png` - Box plots comparing all metrics
- `comparison_timeseries.png` - Dynamics over iterations (median ± IQR)
- `comparison_summary.txt` - Detailed statistics table

---

## Methods Compared

| Method | Description | Expected Behavior |
|--------|-------------|-------------------|
| **ConcurrentPruning** | Distributed Lyapunov pruning with bilateral coordination | Balanced edge reduction + safety |
| **AdjacencyConsensus** | Prior distributed pruning using adjacency matrix consensus | Moderate pruning with consensus guarantees |
| **FullGraph** | No pruning - maintains all edges | Maximum connectivity, maximum edges |
| **CentralizedMST** | Oracle with global knowledge - computes optimal MST | Minimum edges (n-1), requires global info |

---

## Command-Line Options

### Batch Comparison (`batch_comparison.py`)

```bash
python concurrent_pruning/batch_comparison.py [OPTIONS]

Options:
  --robots N          Number of robots (default: 15)
  --density D         Edge density 0-1 (default: 0.7) 
  --seeds N           Number of random seeds (default: 125)
  --iterations N      Max iterations per trial (default: 100)
  --methods M1 M2     Methods to compare (default: all 4)
  --output DIR        Output directory (default: concurrent_pruning/results)
  --quiet             Suppress progress output
```

**Examples:**

```bash
# Quick test with 25 seeds
python concurrent_pruning/batch_comparison.py --seeds 25

# Large network testpython concurrent_pruning/batch_comparison.py --robots 30 --density 0.4 --seeds 100

# Only test ConcurrentPruning vs AdjacencyConsensus
python concurrent_pruning/batch_comparison.py --methods ConcurrentPruning AdjacencyConsensus

# Sparse topology
python concurrent_pruning/batch_comparison.py --robots 20 --density 0.3 --seeds 125
```

### Plot Generation (`generate_comparison_plots.py`)

```bash
python concurrent_pruning/generate_comparison_plots.py RESULTS_FILE [OPTIONS]

Arguments:
  RESULTS_FILE        Path to batch results JSON file (required)

Options:
  --output DIR        Output directory for figures (default: concurrent_pruning/figures)
```

**Example:**

```bash
python concurrent_pruning/generate_comparison_plots.py \
  concurrent_pruning/results/concurrent_comparison_20260218_143052.json \
  --output paper_figures/
```

---

## Understanding Results

### Box Plot Metrics

1. **Final Edge Count**
   - Lower is more efficient
  - ConcurrentPruning and AdjacencyConsensus should be between FullGraph and CentralizedMST

2. **Edge Reduction (%)**
   - Higher means more pruning
   - CentralizedMST achieves maximum (typically 50-70%)
   - ConcurrentPruning shows balanced reduction

3. **Minimum λ₂**
   - **Must stay above 0.3** (safety threshold)
   - FullGraph has highest λ₂ (most connected)
  - ConcurrentPruning and AdjacencyConsensus should maintain threshold
   - CentralizedMST operates near threshold

4. **Average λ₂**
   - Higher means better average connectivity
   - Shows overall network robustness

5. **Computation Time**
  - ConcurrentPruning and AdjacencyConsensus slower than CentralizedMST (distributed consensus overhead)
   - But provides distributed guarantee vs centralized oracle

6. **Pruning Events**
   - Number of edges pruned
  - ConcurrentPruning: gradual distributed pruning
  - AdjacencyConsensus: consensus-driven pruning with weaker edge removal
   - CentralizedMST: immediate optimal pruning

### Time Series Plots

- **Shaded regions** show 25th-75th percentiles (interquartile range)
- **Solid lines** show median across all seeds
- **Edge Pruning Dynamics:** Shows how edges are removed over time
- **Connectivity Evolution:** λ₂ must stay above orange threshold line
- **Consensus Convergence:** Log-scale, exponential decay expected

### Summary Table

- **Mean ± Std:** Average and standard deviation across all seeds
- **Min/Max:** Range of observed values
- Compare methods within each metric row

---

## Typical Results

### Example Output (15 robots, density=0.5, 125 seeds):

```
Method               Final Edges    Pruned (%)     Min λ₂         Time (s)
--------------------------------------------------------------------------------
ConcurrentPruning    32.4 ± 5.2    45.3 ± 8.1     0.42 ± 0.08    2.3 ± 0.5
FullGraph            58.7 ± 7.1     0.0 ± 0.0     1.85 ± 0.23    0.1 ± 0.02
CentralizedMST       14.0 ± 0.0    76.1 ± 0.0     0.31 ± 0.04    0.05 ± 0.01
```

**Interpretation:**
- **ConcurrentPruning:** Achieves 45% edge reduction while maintaining λ₂ > 0.3
- **FullGraph:** No pruning (baseline upper bound)
- **CentralizedMST:** Maximum pruning (theoretical lower bound), but requires global knowledge

---

## Scalability Testing

Test performance across different network sizes:

```bash
# Small network (8 robots)
python concurrent_pruning/batch_comparison.py --robots 8 --seeds 125

# Medium network (15 robots)  
python concurrent_pruning/batch_comparison.py --robots 15 --seeds 125

# Large network (25 robots)
python concurrent_pruning/batch_comparison.py --robots 25 --seeds 125

# Very large network (50 robots) - may take 1+ hour
python concurrent_pruning/batch_comparison.py --robots 50 --seeds 50
```

Then compare results across scales to analyze scalability.

---

## Density Testing

Test performance across topology densities:

```bash
# Sparse (density=0.3)
python concurrent_pruning/batch_comparison.py --density 0.3 --seeds 125

# Medium (density=0.5)
python concurrent_pruning/batch_comparison.py --density 0.5 --seeds 125

# Dense (density=0.7)
python concurrent_pruning/batch_comparison.py --density 0.7 --seeds 125
```

Expected: Sparser topologies have less room for pruning; denser topologies should show more edge reduction.

---

## Troubleshooting

### Out of Memory
- Reduce robot count: `--robots 10`
- Reduce seeds: `--seeds 50`
- Reduce iterations: `--iterations 50`

### Slow Execution
- Use `--quiet` flag to suppress verbose output
- Reduce seeds for quick testing: `--seeds 25`
- Run on compute cluster for large-scale tests

### Import Errors
```bash
# Make sure you're in the project root directory
cd "/path/to/Comunication algorithm most recent work"
python concurrent_pruning/batch_comparison.py
```

### Results Files
- Results are timestamped: `concurrent_comparison_YYYYMMDD_HHMMSS.json`
- Use the most recent file for plotting
- Or specify exact filename when generating plots

---

## Paper Figures

For publication-ready figures:

1. Run comprehensive tests:
```bash
python concurrent_pruning/batch_comparison.py --robots 15 --density 0.5 --seeds 125
```

2. Generate high-quality plots:
```bash
python concurrent_pruning/generate_comparison_plots.py \
  concurrent_pruning/results/concurrent_comparison_YYYYMMDD_HHMMSS.json \
  --output paper_figures/
```

3. Plots are saved at 300 DPI with publication-quality fonts

---

## Citation

If using these comparison scripts in research:

```bibtex
@inproceedings{concurrent_pruning_2026,
  title={Distributed Concurrent Consensus and Topology Pruning for Multi-Robot Networks},
  author={Your Name},
  booktitle={ACC 2026},
  year={2026}
}
```

---

## See Also

- [UNIFIED_MATHEMATICAL_FOUNDATION.md](Docs/UNIFIED_MATHEMATICAL_FOUNDATION.md) - Complete theory
- [example_4_robots.py](example_4_robots.py) - Small pedagogical example
- [SMALL_EXAMPLE_WALKTHROUGH.md](Docs/SMALL_EXAMPLE_WALKTHROUGH.md) - Step-by-step math
