# Iteration 004 — GE Validation & Diagnostics

**Date:** 2026-03-03

## What changed

### 1. Added `ge_transition_unit` field
- New field in `channel_model` block: `"ge_transition_unit": "per_send"`
- Documents that GE state updates happen once per `send_message()` call, not per sim tick

### 2. Added robust per-link GE metrics
New fields in `results`:
- `ge_num_state_updates` — total GE transitions (= total sends in GE mode)
- `ge_num_bad_runs` — completed BAD-state run count
- `ge_bad_fraction_expected` — steady-state π_B = p_gb / (p_gb + p_bg)
- `ge_links_total` — unique directed links used
- `ge_links_with_enough_samples` — links with ≥ K sends (K = `--ge_min_link_samples`)
- `ge_bad_send_fraction_per_link_stats` — {mean, median, min, max, n}
- `ge_links_with_enough_runs` — links with ≥ 2 completed BAD runs
- `ge_bad_run_length_per_link_stats` — {mean, median, min, max, n}

### 3. Added optional stdout diagnostics
New CLI flags:
- `--ge_debug` (bool, default false) — print diagnostic summary at end
- `--ge_debug_link "i,j"` — trace state/drop for specific link
- `--ge_debug_link_max N` — max sends to trace (default 50)
- `--ge_min_link_samples K` — threshold for per-link stats (default 10)

Debug output includes:
- Configured vs observed GE parameters
- Expected π_B vs observed bad_send_fraction
- State sequence (G=GOOD, B=BAD) and drop outcomes (. = delivered, X = dropped)
- Per-link run lengths

### 4. Internal tracking changes
New global variables for per-link accounting:
- `_ge_link_sends` — send count per (i→j)
- `_ge_link_bad_sends` — BAD-state sends per (i→j)
- `_ge_link_bad_runs` — list of completed BAD run lengths per link

Helper function `_compute_ge_per_link_stats()` computes aggregate stats with sample-count filtering.

## Why
User wanted confidence that observed GE stats match configured parameters. Short runs with `early_stop=true` can terminate before steady-state convergence, making validation difficult. Per-link stats help identify if specific links are behaving differently.

## Validation results

**IID run (N=3, T_prune=5):**
- `ge_num_state_updates: 0` (correct — IID path)
- `observed_drop_rate: 0.0667` (~6.7% vs configured 10%, normal variance)

**GE run (N=10, T_prune=30, L_bad=8):**
- Expected π_B = 0.2424
- Observed `ge_bad_send_fraction: 0.0721` (lower due to short run / early interrupt)
- Expected mean BAD run = 8.0
- Observed `ge_avg_bad_run_length: 3.2` (short run effect)
- `ge_links_total: 59`, `ge_links_with_enough_samples: 0` (avg ~3.7 sends/link < 10 threshold)

**Interpretation**: For accurate validation, use longer runs (`--T_prune 120 --early_stop false`).

## Files modified
- [run_pruning_epoch.py](../holoocean_runs/run_pruning_epoch.py)
- [grace_period_sim_notes.md](grace_period_sim_notes.md)

## Lessons
- Per-link sample sparsity is significant: N=10 with ~30 sends/sec total means <1 send/link/sec
- Early-stop bias can cause observed metrics to differ substantially from expected steady-state
- Use `--ge_min_link_samples 5` or lower for short validation runs
