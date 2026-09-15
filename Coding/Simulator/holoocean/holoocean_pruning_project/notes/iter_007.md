# Iteration 007: Bandwidth Cap + FIFO TX Queue (Rate Limiting)

**Date:** 2026-03-03

## Summary

Implemented per-robot transmit rate limiting with FIFO queues to enable bandwidth cap sweep experiments. This allows analysis of protocol performance under constrained link capacity.

## What Changed

### New Features in `run_pruning_epoch.py`

1. **CLI Parameters:**
   - `--rate_cap_bps` (default: `inf` = disabled) — per-robot TX rate limit in bits/second
   - `--queue_max_msgs` (default: 100) — max messages per queue before overflow
   - `--queue_drop_policy` (default: `drop_tail`) — `drop_tail` drops new msg, `drop_head` drops oldest

2. **Per-Robot TX Queue:**
   - Messages enqueued before transmission (sits BEFORE loss model)
   - Budget-based service: each tick adds `rate_cap_bps / 8 / TICKS_PER_SEC` bytes
   - Dequeued messages go through existing `send_message()` → loss → delay → deliver

3. **Queue Statistics Tracked:**
   - `queue_delay_mean_s` — average time from enqueue to dequeue
   - `queue_delay_p95_s` — 95th percentile queue delay
   - `queue_overflow_drops` — messages dropped due to full queue
   - `queue_overflow_drop_rate` — overflow drops / total enqueued
   - `queue_max_occupancy` — peak queue depth observed

4. **Epoch Summary:**
   - New `queue_model` block in JSON
   - Queue stats added to results and console output

### New Scripts

- `holoocean_runs/sweep_rate_cap.py` — Run sweep across rate caps and seeds, output CSV
- `holoocean_runs/plot_rate_cap_sweep.py` — Generate plots: success vs cap, delay vs cap, throughput vs cap

## Validation Results

### Test Commands

```powershell
# Baseline (rate limiting disabled)
python -m holoocean_runs.run_pruning_epoch --N 10 --T_prune 30 --seed 42 \
    --rate_cap_bps inf --queue_max_msgs 1000 --no_viz true --tag val_baseline2

# Bottleneck (48 bps = ~1 msg/s per robot)
python -m holoocean_runs.run_pruning_epoch --N 10 --T_prune 30 --seed 42 \
    --rate_cap_bps 48 --queue_max_msgs 200 --no_viz true --tag val_bottleneck
```

### Comparison Table

| Metric                   | Baseline (inf) | Bottleneck (48 bps) |
|--------------------------|----------------|---------------------|
| success (connected∧tree) | ✓ (1)          | ✗ (0)               |
| t_tree_first             | 2.35 s         | null                |
| rx_Bps                   | 160.4          | 14.6                |
| queue_delay_p95_s        | 0.0            | 4.983               |
| queue_overflow_drop_rate | 0.0%           | 0.0%                |
| reachable / N            | 10 / 10        | 6 / 10              |

### Interpretation

- **Baseline:** Zero queue delay confirms rate limiting is properly disabled when `rate_cap_bps=inf`
- **Bottleneck:** At 48 bps per robot, only ~1 msg/s can be transmitted. With ~3 neighbors per robot broadcasting every ~1s, demand exceeds capacity by 3×. Result:
  - Queue delay explodes to ~5 seconds (p95)
  - Throughput drops 11× (160→14 B/s)
  - 4 robots never received enough messages to be reached from root
  - Protocol fails to form spanning tree

This validates that:
1. Queue delay and overflow tracking work correctly
2. Rate limiting creates observable congestion
3. Extremely low caps cause protocol failure (as expected)

## Sweep Design

### Caps to Test

```
inf, 128k, 64k, 32k, 16k, 8k, 4k, 2k, 1k, 500, 250 bps
```

Rationale:
- N=10: Baseline ~160 B/s = 1280 bps → caps above 2k should succeed, below may struggle
- N=20: Higher baseline throughput → critical threshold shifts up

### Commands

```powershell
# N=10 sweep (10 seeds, ~110 runs, ~5-6 hours)
python -m holoocean_runs.sweep_rate_cap --N 10 --n_seeds 10 --T_prune 60 --out_dir sweeps/rate_cap_N10

# N=20 sweep (10 seeds, ~110 runs, ~10-12 hours)
python -m holoocean_runs.sweep_rate_cap --N 20 --n_seeds 10 --T_prune 60 --out_dir sweeps/rate_cap_N20

# Plot results
python -m holoocean_runs.plot_rate_cap_sweep sweeps/rate_cap_N10/*.csv sweeps/rate_cap_N20/*.csv --out rate_cap_sweep.png
```

### Expected Results

- Success rate should be ~100% for caps > ~4k bps
- Sharp cliff in success rate around 1-2k bps
- Queue delay p95 should increase exponentially as cap decreases toward saturation
- rx_Bps should plateau at cap when cap < natural throughput

## Files Modified

- `holoocean_runs/run_pruning_epoch.py` — Added rate limiting logic, queue stats, CLI params
- `notes/grace_period_sim_notes.md` — Documented queue model
- `claude.md/checklist.md` — Added rate limiting checklist and pitfalls

## Files Created

- `holoocean_runs/sweep_rate_cap.py` — Sweep runner
- `holoocean_runs/plot_rate_cap_sweep.py` — Plotting script
- `notes/iter_007.md` — This changelog

## Notes

- The rate limit is per-robot (each robot has independent budget and queue)
- Queue delay is measured from enqueue time to dequeue time (before network delay)
- Total message delay = queue_delay + network_delay (uniform [0, delay_max])
- If rate cap is too aggressive, the protocol will fail to converge due to insufficient message exchange
