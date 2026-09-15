# Iteration 008: Low-Cap Breakpoint Sweep (Rate Cap 6–240 bps)

**Date:** 2026-03-03

## Summary

Ran a fine-grained rate cap sweep at very low bps values (6–240) to find the exact breakpoint where the pruning protocol fails to converge. This follows up on Iter 007's coarse sweep (250–128k bps), which showed 100% success at all tested caps.

## Motivation

The Iter 007 sweep (caps: inf, 128k, 64k, 32k, 16k, 8k, 4k, 2k, 1k, 500, 250 bps) showed 100% success across all 10 seeds for every cap — even down to 250 bps. The protocol was more resilient than expected. A finer sweep at the low end was needed to locate the critical threshold.

## Command

```powershell
cd holoocean_pruning_project

python -m holoocean_runs.sweep_rate_cap --N 10 --n_seeds 3 --caps "6,12,18,24,30,36,42,48,60,72,96,120,144,192,240" --T_prune 30 --tag_prefix sweep --out_dir sweeps/breakpoint_N10
```

### Parameters
- **N:** 10 agents (fixed)
- **n_seeds:** 3 per cap
- **caps:** 6, 12, 18, 24, 30, 36, 42, 48, 60, 72, 96, 120, 144, 192, 240 bps
- **T_prune:** 30 s
- **tag_prefix:** `sweep` → output dirs: `sweep_N10_cap{cap}_s{seed}`

## Output Directory Structure

Results stored in `stress_results_holoocean/` with naming pattern:
```
sweep_N10_cap6_s1/
sweep_N10_cap6_s2/
sweep_N10_cap6_s3/
sweep_N10_cap12_s1/
...
sweep_N10_cap240_s3/
```

CSV summary: `sweeps/breakpoint_N10/sweep_N10_20260303_221945.csv`
Plot: `sweeps/breakpoint_N10/rate_cap_breakpoint.png`

## Results

### Full results table

| Cap (bps) | Success | Avg t_tree_first (s) | Avg t_stable (s) | Avg queue_delay_p95 (s) | Avg rx_Bps |
|-----------|---------|----------------------|------------------|-------------------------|------------|
| 240       | 3/3     | 4.35                 | 15.03            | 2.14                    | 146.0      |
| 192       | 3/3     | 5.19                 | 13.63            | 2.74                    | 122.2      |
| 144       | 3/3     | 5.66                 | 16.47            | 4.90                    | 108.6      |
| 120       | 3/3     | 6.56                 | 16.03            | 5.72                    | 89.0       |
| 96        | 3/3     | 7.64                 | 19.33            | 8.42                    | 75.2       |
| 72        | 3/3     | 9.41                 | 18.51            | 8.57                    | 52.1       |
| 60        | 3/3     | 9.2                  | 18.3             | 12.2                    | 47.9       |
| 48        | 3/3     | 11.47                | 22.67            | 12.48                   | 37.8       |
| 42        | 3/3     | 14.0                 | 20.6             | 11.5                    | 28.9       |
| 36        | 3/3     | 15.7                 | 23.1             | 14.0                    | 25.1       |
| 30        | 3/3     | 18.0                 | 23.9             | 15.1                    | 18.0       |
| 24        | 2/3     | 23.3                 | 28.5             | 18.3                    | 15.4       |
| 18        | 2/3     | 27.8                 | —                | 20.7                    | 10.4       |
| 12        | 0/3     | —                    | —                | 22.9                    | 3.2        |
| 6         | 0/3     | —                    | —                | 22.6                    | 1.1        |

### Sweep script bug

`sweep_rate_cap.py` skipped caps 6–60 because `subprocess.run()` returned non-zero exit codes (likely matplotlib or encoding error at script end), even when `epoch_summary.json` was produced correctly. Data was recovered by reading the JSON files directly into `sweep_N10_breakpoint_rebuilt.csv`.

### Breakpoint

The critical threshold for N=10 lies between **12 and 18 bps**:
- Caps ≥30 bps: 100% success (3/3)
- Caps 18–24 bps: marginal (2/3 seeds)
- Caps ≤12 bps: complete failure (0/3)

### Key observations

1. **Queue delay scales inversely with cap:** p95 delay grows from ~2s at 240 bps to ~23s at 6 bps
2. **Convergence time degrades gracefully:** t_tree_first increases from ~4.4s (240 bps) to ~28s (18 bps)
3. **Marginal zone at 18–24 bps:** partial success — protocol sometimes converges but unreliably
4. **Throughput compression:** rx_Bps drops from 146 (240 bps) to ~1 (6 bps)

## Files

- CSV: `sweeps/breakpoint_N10/sweep_N10_20260303_221945.csv`
- Plot: `sweeps/breakpoint_N10/rate_cap_breakpoint.png`
- Run dirs: `stress_results_holoocean/sweep_N10_cap*_s*/`

## Plotting

```powershell
python -m holoocean_runs.plot_rate_cap_sweep "sweeps/breakpoint_N10/*.csv" --out sweeps/breakpoint_N10/rate_cap_breakpoint.png --show
```

2x2 figure:
- (a) Success Rate vs cap
- (b) Queue Delay p95 vs cap
- (c) Received Throughput vs cap
- (d) Convergence Time vs cap — shows both `t_tree_first` (dashed) and `t_tree_stable_first` (solid) with SEM error bars

## Files Modified

- `holoocean_runs/plot_rate_cap_sweep.py` — Added `plot_convergence_time()`, changed layout from 1x3 to 2x2
- `notes/grace_period_sim_notes.md` — Added Iter 008 section with command, results, and plotting
- `notes/iter_008.md` — This changelog
