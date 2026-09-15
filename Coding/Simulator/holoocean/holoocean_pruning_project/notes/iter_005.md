# Iteration 005 — Controller Tuning + Tie-break Fix + Per-Sender GE Stats

**Date:** 2026-03-03

## Summary

Three improvements in this iteration:
1. **Controller slowdown**: Reduced default k_goal, v_max, thruster_gain, thruster_clip for slower robot motion
2. **δ-BFS tie-break fix**: Replaced ID-bias (`snd < parent`) with distance-based tie-break to avoid star topology
3. **Per-sender GE stats**: Added aggregated GE statistics by sender to handle sparse per-link samples

## Changes

### 1. Controller Tuning

**Problem**: Robots moved too fast, causing connectivity issues and making tree dynamics hard to observe.

**Solution**: Changed defaults in `parse_args()`:
- `--k_goal`: 2.0 → 0.3
- `--v_max`: 2.0 → 0.6 m/s  
- `--thruster_gain`: 35.0 → 10.0
- `--thruster_clip`: 35.0 → 10.0

### 2. δ-BFS Tie-break Fix

**Problem**: Legacy tie-break `snd < parent[rcv]` made all nodes prefer lowest-ID neighbor, collapsing tree to star.

**Solution**: New `--tiebreak_mode` parameter with options:
- `distance` (default): prefer sender closest to receiver
- `id`: legacy ID-based (for comparison)
- `hash`: deterministic pseudo-random

**Hysteresis** to prevent parent flipping:
- `--hysteresis_margin` (default 0.2m): new parent must be this much closer
- `--hysteresis_count` (default 2): candidate must win K consecutive tie-breaks

**Implementation**:
- Modified `deliver_messages()` to accept positions, tiebreak_mode, hysteresis params
- Added global `_parent_hysteresis` dict to track consecutive wins per node
- Distance comparison uses `np.linalg.norm(pos_rcv - pos_snd)`

### 3. Per-Sender GE Stats

**Problem**: Per-link stats often empty because each directed link sees few sends in short runs.

**Solution**: Added `_compute_ge_per_sender_stats()` that aggregates all outgoing links per sender:
- Lower sample threshold (half of `--ge_min_link_samples`)
- More likely to produce useful stats

**New JSON fields**:
- `ge_senders_total`
- `ge_senders_with_enough_samples`
- `ge_bad_send_fraction_per_sender_stats`
- `ge_bad_run_length_per_sender_stats`

## Validation Results

**Test run**: N=10, T_prune=20s, loss_model=ge, L_bad=8, early_stop=false

**Tree topology (before - ID tie-break)**:
- All nodes parent=8 → star

**Tree topology (after - distance tie-break)**:
- Root 8 → children 0, 1, 2, 4 (4 direct)
- Node 0 → children 3, 9 (depth 2)
- Node 4 → children 5, 6, 7 (depth 2)
- Distributed tree structure ✓

**GE stats**:
- ge_senders_with_enough_samples: 10/10 (all senders)
- ge_bad_send_fraction_per_sender: mean=0.1946, median=0.2045
- ge_bad_run_length_per_sender: mean=5.64, median=5.875
- Expected π_B: 0.2857, observed: 0.197 (reasonable for 792 sends)

## Files Modified
- [run_pruning_epoch.py](../holoocean_runs/run_pruning_epoch.py)
- [grace_period_sim_notes.md](grace_period_sim_notes.md)

## Lessons Learned
- ID-based tie-break in distributed algorithms can cause unexpected topology collapse
- Distance-based tie-break requires position information but produces more natural topologies
- Hysteresis is important to prevent oscillation in distributed parent selection
- Per-sender aggregation is more robust than per-link stats for sparse communication patterns
