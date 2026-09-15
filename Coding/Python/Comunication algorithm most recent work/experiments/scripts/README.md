# Experiment Scripts

This directory contains scripts for running comprehensive experiments.

## Files

- **`run_experiments.py`**: Main experiment runner

## Usage

### Quick Test (Recommended First)
Test that everything works with a small sample:
```bash
python experiments/scripts/run_experiments.py --quick
```
This runs:
- 2 trials
- 100 steps per trial
- Scenario A only
- All 4 methods
- **Takes ~2-3 minutes**

### Full Experiments
Run all experiments (4 methods × 5 scenarios × 10 trials = 200 simulations):
```bash
python experiments/scripts/run_experiments.py
```
**Takes ~20-30 minutes**

### Custom Configuration

Run specific methods:
```bash
python experiments/scripts/run_experiments.py --methods ConcurrentPruning AdjacencyConsensus
```

Run specific scenarios:
```bash
python experiments/scripts/run_experiments.py --scenarios A B C
```

Change number of trials:
```bash
python experiments/scripts/run_experiments.py --trials 5
```

Change max steps:
```bash
python experiments/scripts/run_experiments.py --steps 300
```

Specify output file:
```bash
python experiments/scripts/run_experiments.py --output experiments/my_results.json
```

## Output

Results are saved to:
```
experiments/comprehensive_results_YYYYMMDD_HHMMSS.json
```

The file contains:
- **Metadata**: Configuration, timestamp, total runs
- **Results**: List of all trial results with:
  - Edge statistics (initial, final, reduction %)
  - Connectivity metrics (min/mean/final λ₂)
  - Control effort (total, mean, max)
  - Time series data (edges, λ₂, control over time)
  - Robot trajectories (positions over time)
  - Pruning events with timestamps

## Monitoring Progress

The script prints progress in real-time:
```
ConcurrentPruning - Scenario A:
  ✓ Trial  1/10 | λ₂_min=0.234 | edges: 45→23 | Progress: 1/200 (0.5%) | ETA: 25.3min
  ✓ Trial  2/10 | λ₂_min=0.198 | edges: 47→25 | Progress: 2/200 (1.0%) | ETA: 24.8min
  ...
```

Icons:
- ✓ = Success (maintained connectivity)
- ✗ = Failure (disconnected graph)

## Troubleshooting

**Import errors:**
- Make sure you're in the root directory of the project
- The script will automatically add parent directories to Python path

**Simulation fails:**
- Check that all scenarios are properly configured in `config/scenarios.py`
- Verify simulation classes are correctly imported

**Out of memory:**
- Reduce `--trials` or `--steps`
- Run scenarios separately

**Taking too long:**
- Use `--quick` for testing
- Reduce `--steps` to 300 or less
- Run fewer scenarios at a time

## Next Steps

After experiments complete:
1. **Generate all plots**: Run the plotting script (see below)
2. **Review figures**: Check `experiments/figures/` directory
3. **Analyze results**: Review `experiments/figures/summary_statistics.txt`

---

## Generating Plots

After experiments are complete, generate all visualizations:

### Automatic (uses most recent results):
```bash
python experiments/scripts/generate_all_plots.py
```

### Manual (specify results file):
```bash
python experiments/scripts/generate_all_plots.py experiments/comprehensive_results_20260219_HHMMSS.json
```

### Custom output directory:
```bash
python experiments/scripts/generate_all_plots.py experiments/comprehensive_results_20260219_HHMMSS.json --output-dir my_figures/
```

This will generate:
- **10 main plots** (PNG + PDF format each):
  1. Robot Trajectories with pruning events
  2. Edge Count Evolution over time
  3. Lambda2 Evolution and Distribution
  4. Cumulative Control Effort
  5. Success Rate Heatmap
  6. Min Lambda2 by Scenario
  7. Pruning Efficiency by Scenario
  8. Trade-off Analysis (edge reduction vs connectivity)
  9. Performance Distributions (violin plots)
  10. Runtime Efficiency
- **1 summary statistics file** (TXT)

Output saved to: `experiments/figures/`
