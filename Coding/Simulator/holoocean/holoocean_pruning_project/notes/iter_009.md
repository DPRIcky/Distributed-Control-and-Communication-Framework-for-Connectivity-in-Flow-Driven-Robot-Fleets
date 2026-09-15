# Iteration 009: N=20 Full Rate Cap Sweep (6 bps – 128k bps)

**Date:** 2026-03-03

## Summary

Sweep `rate_cap_bps` for N=20 across the full range (6–128k bps) using the same methodology as Iter 008 (N=10). After sweeps complete, rebuild CSVs using `rebuild_sweep_csv.py` to recover data lost due to the non-zero exit code bug in `sweep_rate_cap.py`.

## Commands

Run from: `holoocean_pruning_project/`

### Step 1 — Run sweeps

```powershell
# Coarse sweep: inf, 128k, 64k, ..., 250 bps
python -m holoocean_runs.sweep_rate_cap --N 20 --n_seeds 3 --T_prune 30 --tag_prefix sweep --out_dir sweeps/rate_cap_N20

# Fine sweep: 240 → 6 bps
python -m holoocean_runs.sweep_rate_cap --N 20 --n_seeds 3 `
    --caps "6,12,18,24,30,36,42,48,60,72,96,120,144,192,240" `
    --T_prune 30 --tag_prefix sweep --out_dir sweeps/breakpoint_N20
```

### Step 2 — Rebuild CSVs from epoch_summary.json

```powershell
# Coarse sweep rebuild
python -m holoocean_runs.rebuild_sweep_csv --N 20 --n_seeds 3 `
    --out sweeps/rate_cap_N20/sweep_N20_rebuilt.csv

# Fine sweep rebuild
python -m holoocean_runs.rebuild_sweep_csv --N 20 --n_seeds 3 `
    --caps "6,12,18,24,30,36,42,48,60,72,96,120,144,192,240" `
    --out sweeps/breakpoint_N20/sweep_N20_breakpoint_rebuilt.csv
```

### Step 3 — Combined N=10 + N=20 plot

```powershell
python -m holoocean_runs.plot_rate_cap_sweep `
    "sweeps/rate_cap_N10/*.csv" `
    "sweeps/breakpoint_N10/sweep_N10_breakpoint_rebuilt.csv" `
    "sweeps/rate_cap_N20/sweep_N20_rebuilt.csv" `
    "sweeps/breakpoint_N20/sweep_N20_breakpoint_rebuilt.csv" `
    --out sweeps/rate_cap_combined.png --show
```

## Output directory structure

```
stress_results_holoocean/
  sweep_N20_capinf_s1/
  sweep_N20_cap128000_s1/
  ...
  sweep_N20_cap6_s3/

sweeps/
  rate_cap_N20/sweep_N20_rebuilt.csv
  breakpoint_N20/sweep_N20_breakpoint_rebuilt.csv
  rate_cap_combined.png
```

## Results

### Full results table (N=20)

| Cap (bps) | Success | Avg t_tree_first (s) | Avg t_stable (s) | Avg queue_delay_p95 (s) | Avg rx_Bps |
|-----------|---------|----------------------|------------------|-------------------------|------------|
| inf       | 3/3     | 3.0                  | 18.6             | 0.00                    | 546.5      |
| 128000    | 3/3     | 2.8                  | 15.4             | 0.00                    | 484.6      |
| 64000     | 3/3     | 2.8                  | 15.4             | 0.00                    | 484.6      |
| 32000     | 3/3     | 2.8                  | 15.4             | 0.00                    | 484.6      |
| 16000     | 3/3     | 3.0                  | 14.3             | 0.00                    | 474.8      |
| 8000      | 3/3     | 2.8                  | 13.9             | 0.03                    | 475.3      |
| 4000      | 3/3     | 3.0                  | 17.3             | 0.08                    | 535.0      |
| 2000      | 3/3     | 3.1                  | 21.6             | 0.17                    | 528.1      |
| 1000      | 3/3     | 3.8                  | 21.2             | 0.35                    | 519.8      |
| 500       | 3/3     | 4.5                  | 19.9             | 0.76                    | 497.9      |
| 250       | 3/3     | 5.0                  | 21.1             | 4.72                    | 405.4      |
| 240       | 3/3     | 5.0                  | 21.3             | 5.14                    | 399.0      |
| 192       | 3/3     | 5.9                  | 18.8             | 6.24                    | 326.6      |
| 144       | 3/3     | 7.7                  | 20.4             | 8.47                    | 243.5      |
| 120       | 3/3     | 8.4                  | 20.1             | 9.21                    | 193.5      |
| 96        | 3/3     | 8.6                  | 20.4             | 10.56                   | 153.2      |
| 72        | 3/3     | 10.2                 | 23.3             | 15.04                   | 122.8      |
| 60        | 3/3     | 12.1                 | 25.6             | 15.47                   | 97.3       |
| 48        | 3/3     | 15.0                 | 21.2             | 15.26                   | 67.3       |
| 42        | 3/3     | 16.0                 | 26.4             | 18.20                   | 62.8       |
| 36        | 3/3     | 18.4                 | —                | 20.05                   | 53.5       |
| 30        | 3/3     | 20.6                 | 27.3             | 19.44                   | 37.9       |
| 24        | 3/3     | 27.2                 | —                | 20.32                   | 27.3       |
| 18        | 0/3     | —                    | —                | 20.93                   | 13.3       |
| 12        | 0/3     | —                    | —                | 22.61                   | 7.1        |
| 6         | 0/3     | —                    | —                | 23.28                   | 1.3        |

### Breakpoint

The critical threshold for N=20 lies between **18 and 24 bps**:
- Caps ≥24 bps: 100% success (3/3) — sharp cliff, no marginal zone
- Caps ≤18 bps: complete failure (0/3)

### Comparison with N=10

| | N=10 | N=20 |
|--|------|------|
| Breakpoint | 12–18 bps (marginal 18–24) | 18–24 bps (sharp cliff) |
| Baseline rx_Bps | ~190 B/s | ~520 B/s |
| t_tree_first at min working cap | ~28s (18 bps) | ~27s (24 bps) |
| Queue delay collapse | starts at ~30 bps | starts at ~250 bps |

N=20 has ~2.7× the baseline throughput, and its breakpoint shifted by only 1.3× in bps — suggesting the per-robot messaging rate, not total network traffic, is the limiting factor.

### Key observations

1. **Sharp cliff:** N=20 has a clean 0%→100% step at 24 bps (vs N=10's gradual 18–24 marginal zone)
2. **High-cap performance:** N=20 actually starts degrading queue delay earlier (at ~250 bps) than N=10 (~500 bps) due to higher baseline traffic
3. **Convergence ~same wall-clock at min cap:** Both N=10 and N=20 take ~27–28s to first form a tree at their minimum working cap
4. **t_stable missing for 24 and 36 bps:** Trees formed but didn't fully stabilize within T_prune=30s; use longer T_prune to capture stability time at these caps

## Files

- CSVs: `sweeps/rate_cap_N20/sweep_N20_rebuilt.csv`, `sweeps/breakpoint_N20/sweep_N20_breakpoint_rebuilt.csv`
- Combined plot: `sweeps/rate_cap_combined.png`
- Run dirs: `stress_results_holoocean/sweep_N20_cap*_s*/`

## Plotting

```powershell
python -m holoocean_runs.plot_rate_cap_sweep `
    "sweeps/rate_cap_N10/*.csv" `
    "sweeps/breakpoint_N10/sweep_N10_breakpoint_rebuilt.csv" `
    "sweeps/rate_cap_N20/sweep_N20_rebuilt.csv" `
    "sweeps/breakpoint_N20/sweep_N20_breakpoint_rebuilt.csv" `
    --out sweeps/rate_cap_combined.png --show
```

## Files Created/Modified

- `holoocean_runs/rebuild_sweep_csv.py` — Reusable CSV rebuilder from epoch_summary.json
- `sweeps/rate_cap_N20/sweep_N20_rebuilt.csv` — Coarse sweep results (33/33 rows)
- `sweeps/breakpoint_N20/sweep_N20_breakpoint_rebuilt.csv` — Fine sweep results (45/45 rows)
- `sweeps/rate_cap_combined.png` — Combined N=10 + N=20 2×2 plot
- `notes/grace_period_sim_notes.md` — N=20 results added
- `notes/iter_009.md` — This changelog
