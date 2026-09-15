# Iteration 010: Simulation 3 — Distance-Dependent Impairment

**Date:** 2026-03-04

## Summary

Added a distance-dependent packet-drop model to the simulation and ran a 3-condition × 5-seed comparison for N=20 to quantify how signal degradation with range affects δ-BFS tree convergence.

## Model Definition

For each send attempt from node `i` to neighbor `j`:

```
p_drop_eff(d) = clip(p0 + k * (d / R_comm)^alpha, 0, pmax)
```

where `d = ||x_i - x_j||` (Euclidean distance in meters), `R_comm` is the communication radius (default 2.0 m).

- Activated by `--dist_impair_model drop`
- Applied in the IID drop path; GE bursty loss not combined in Sim3 for clarity

### Presets

| Profile | p0   | k    | alpha | pmax | Notes |
|---------|------|------|-------|------|-------|
| `none`  | —    | —    | —     | —    | Standard iid p_drop=0.10 |
| `weak`  | 0.05 | 0.25 | 1.0   | 0.95 | Gradual linear degradation |
| `strong`| 0.05 | 0.90 | 2.0   | 0.95 | Sharp quadratic degradation |

**Interpretation at d = R_comm (max neighbor distance):**
- Weak: p_drop_eff = clip(0.05 + 0.25, 0.95) = 0.30 (30%)
- Strong: p_drop_eff = clip(0.05 + 0.90, 0.95) = 0.95 (capped at 95%)

## New CLI Flags

| Flag | Default | Description |
|------|---------|-------------|
| `--dist_impair_model` | `none` | `none` or `drop` |
| `--dist_profile` | `none` | `none`, `weak`, or `strong` |
| `--dist_p0` | `0.05` | Base drop prob at d=0 |
| `--dist_k` | `0.25` | Drop slope coefficient |
| `--dist_alpha` | `1.0` | Distance exponent |
| `--dist_pmax` | `0.95` | Maximum effective drop prob |

## New Metrics in epoch_summary.json

### `dist_model` block
```json
{
  "dist_impair_model": "drop",
  "dist_profile": "weak",
  "dist_p0": 0.05,
  "dist_k": 0.25,
  "dist_alpha": 1.0,
  "dist_pmax": 0.95
}
```

### `results` fields (new)
- `send_dist_mean_m` — mean Euclidean distance across all send attempts (m)
- `send_dist_p95_m` — 95th-percentile send distance (m)
- `p_drop_eff_mean` — mean effective drop probability across all sends
- `p_drop_eff_p95` — 95th-percentile effective drop probability

## Experiment Design

| Parameter | Value |
|-----------|-------|
| N | 20 |
| T_prune | 30 s |
| early_stop | true |
| n_seeds | 5 (seeds 1–5) |
| rate_cap_bps | inf (no rate limiting) |
| loss_model | iid |

Conditions run:

| Condition | Tag suffix | dist_impair_model | dist_profile |
|-----------|------------|-------------------|--------------|
| A: Baseline | `sim3_N20_baseline_s{seed}` | none | none |
| B: Weak | `sim3_N20_weak_s{seed}` | drop | weak |
| C: Strong | `sim3_N20_strong_s{seed}` | drop | strong |

## Commands

Run from: `holoocean_pruning_project/`

```powershell
# Step 1: Run all 15 experiments
python -m holoocean_runs.sweep_sim3 --N 20 --n_seeds 5 --T_prune 30 `
    --out_dir sweeps/sim3 --tag_prefix sim3

# Step 2: Plot comparison (3 panels)
python -m holoocean_runs.plot_sim3 sweeps/sim3/sim3_results.csv `
    --out sweeps/sim3/sim3_comparison.png --show
```

## Output directory structure

```
stress_results_holoocean/
  sim3_N20_baseline_s1/epoch_summary.json
  ...
  sim3_N20_strong_s5/epoch_summary.json

sweeps/sim3/
  sim3_results.csv
  sim3_comparison.png
```

## Results

All 15 runs completed successfully. Plot: `sweeps/sim3/sim3_comparison.png`

### Aggregate summary (5 seeds each)

| Condition | Success | t_tree_first (s) | t_tree_stable (s) | rx_Bps | TX drop rate | p_drop_eff mean |
|-----------|---------|-----------------|-------------------|--------|--------------|-----------------|
| baseline  | 5/5 (100%) | 3.10 ± 0.22 | 17.68 ± 4.79 | 539 | 9.9% | 10.0% (IID) |
| weak      | 5/5 (100%) | 3.74 ± 0.49 | 20.74 ± 5.74 | 444 | 24.5% | 24.97% |
| strong    | 5/5 (100%) | 7.64 ± 1.20 | 22.03 ± 1.76 | 212 | 62.9% | 62.37% |

*(t_tree_stable excludes seeds with no stable state by T_prune=30s: baseline s2, strong s1)*

### Per-seed data

| Condition | Seed | Success | t_first (s) | t_stable (s) | rx_Bps | drop_rate | p_eff_mean | p_eff_p95 |
|-----------|------|---------|-------------|--------------|--------|-----------|------------|-----------|
| baseline | 1 | ✓ | 3.05 | 20.05 | 514 | 10.7% | 10.0% | 10.0% |
| baseline | 2 | ✓ | 3.10 | — (30s) | 564 | 10.0% | 10.0% | 10.0% |
| baseline | 3 | ✓ | 2.83 | 17.25 | 562 | 9.7% | 10.0% | 10.0% |
| baseline | 4 | ✓ | 3.45 | 10.32 | 496 | 9.3% | 10.0% | 10.0% |
| baseline | 5 | ✓ | 3.05 | 23.12 | 560 | 9.8% | 10.0% | 10.0% |
| weak | 1 | ✓ | 3.32 | 26.93 | 477 | 24.4% | 24.95% | 29.2% |
| weak | 2 | ✓ | 4.33 | 17.85 | 393 | 23.8% | 24.98% | 29.0% |
| weak | 3 | ✓ | 3.08 | 18.52 | 473 | 24.0% | 25.11% | 29.3% |
| weak | 4 | ✓ | 4.07 | 27.93 | 521 | 23.9% | 24.89% | 28.9% |
| weak | 5 | ✓ | 3.90 | 12.47 | 354 | 26.1% | 24.92% | 29.4% |
| strong | 1 | ✓ | 8.08 | — (30s) | 228 | 62.3% | 62.01% | 87.6% |
| strong | 2 | ✓ | 8.97 | 22.53 | 188 | 62.1% | 62.16% | 86.7% |
| strong | 3 | ✓ | 7.03 | 21.17 | 212 | 64.5% | 63.25% | 89.8% |
| strong | 4 | ✓ | 5.65 | 24.57 | 246 | 62.0% | 62.47% | 87.5% |
| strong | 5 | ✓ | 8.47 | 19.87 | 188 | 63.6% | 61.97% | 89.5% |

### Key Findings

1. **100% convergence success across all conditions** — even under 63% observed drop rate (strong profile), δ-BFS forms a valid spanning tree in every trial. This is the primary result.

2. **First-tree time is the most sensitive metric**: baseline 3.1s → weak 3.7s (+21%) → strong 7.6s (+145%). The initial tree formation suffers most under heavy distance-dependent impairment because early topology exploration requires long-range messages that are hardest hit.

3. **Stable-tree convergence degrades more gracefully**: 17.7s → 20.7s → 22.0s. Once the first tree forms, refining to a stable state requires fewer long-range messages (neighbors are already known), so drop probability has less impact.

4. **rx_Bps drops sharply with impairment**: 539 → 444 → 212 B/s (strong is 39% of baseline), while tx_Bps stays high (621 → 587 → 573 B/s). This means the channel is saturated with retransmissions/overhead; efficiency (rx/tx ratio) degrades from 88% → 75% → 37%.

5. **p_drop_eff closely tracks the preset values**: weak mean ≈ 24.97% (expected ~25% at mean send distance), strong mean ≈ 62.4% (expected ~60–80% depending on distance distribution). The send distance p95 of 1.94m / R_comm=2.0m confirms robots routinely communicate near the edge of their comm radius.

6. **Seeds 2 (baseline) and 1 (strong) did not reach stable state before T_prune=30s**. These are outliers — the unstable network kept switching parents without stabilizing within the window. With T_stable=5s and T_prune=30s, there was only one opportunity for stable convergence if the tree first formed late.

### Interpretation for Paper

The δ-BFS protocol is highly robust to distance-dependent channel impairment. Even when 63% of packets are dropped (with drops concentrated on long-range links near R_comm), the protocol reliably converges to a correct spanning tree. The cost is a 2.5× increase in first-tree latency and a 60% reduction in received throughput, but the fundamental correctness guarantee is maintained. This suggests the protocol is well-suited for underwater swarm scenarios where acoustic propagation loss creates strong distance-dependent channel impairment.

## Files Created/Modified

- `holoocean_runs/run_pruning_epoch.py` — Added `--dist_impair_model`, `--dist_profile`, and 4 dist params; `_dist_drop_prob()`, `_resolve_dist_params()`; distance stats in results
- `holoocean_runs/sweep_sim3.py` — New: runs 3 conditions × N seeds, rebuilds CSV
- `holoocean_runs/plot_sim3.py` — New: 3-panel bar chart comparison
- `notes/iter_010.md` — This changelog
- `notes/grace_period_sim_notes.md` — Sim3 model and results section added
