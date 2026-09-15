# Iteration 006: Visualization & Termination Fixes

**Date:** 2026-03-03

## Summary

Fixed three issues that appeared when scaling to N=20:
1. Matplotlib live figure not updating during simulation
2. Termination condition debugging and logging
3. Red edge visualization gating (building vs committed)

## Changes

### 1. Matplotlib Live Updates Fix

**Problem:** Figure only showed initial state and final state; no updates during run.

**Root cause:** Missing `plt.pause()` call in `refresh_viz()`. Without this, the GUI event loop never processes queued drawing commands.

**Fix in `refresh_viz()`:**
```python
try:
    fig.canvas.draw_idle()
    fig.canvas.flush_events()
    plt.pause(0.001)  # NEW: allows event loop to process updates
except Exception:
    pass
```

**New CLI flags:**
- `--no_viz` — disable matplotlib entirely
- `--debug_viz` — verbose viz debug output
- `--viz_interval` — wall-clock refresh period (default 0.35s)

### 2. Termination Tracking

**Problem:** Unclear why simulation ended (T_prune vs early_stop vs other).

**Solution:** Added termination tracking variables and new JSON fields.

**New tracking variables:**
- `termination_reason: str` — which condition ended the run
- `t_tree_first: float` — first time is_tree becomes true
- `t_tree_stable_first: float` — first time tree is stable for T_stable
- `t_goal_reached: float` — first time mean(d2goal) < 2.0m

**New JSON fields in `results`:**
- `termination_reason`: `"T_prune_reached"`, `"early_stop_stable"`, or `"keyboard_interrupt"`
- `t_tree_first`, `t_tree_stable_first`, `t_goal_reached`: timestamps or null

**Debug flag:**
- `--debug_term` — prints termination condition evaluation

### 3. Red Edge Gating

**Problem:** Red tree edges were drawn from continuously-updating candidate parent pointers, unclear which were committed.

**Solution:** Visual distinction:
- **Building** (`is_tree=False`): Salmon color, dashed lines
- **Committed** (`is_tree=True`): Crimson color, solid lines

**Code in `refresh_viz()`:**
```python
edge_color = "crimson" if is_tree else "salmon"
edge_style = "-" if is_tree else "--"
```

## Files Modified

- `holoocean_runs/run_pruning_epoch.py`
  - CLI: Added `--no_viz`, `--debug_viz`, `--debug_term`
  - `refresh_viz()`: Added `plt.pause(0.001)`, edge color/style gating
  - Main loop: Added termination tracking, debug output
  - Summary: Added new result fields
  - Banner: Added viz and debug info

- `notes/grace_period_sim_notes.md`
  - New sections: "Live Plotting in Scripts", "Termination Conditions", "Red Edge Visualization Gating"
  - New lessons learned #10-11

- `claude.md/checklist.md`
  - (to be updated)

## Validation Commands

### N=10 with viz enabled:
```bash
$env:PYTHONIOENCODING="utf-8"
cd "c:\Users\prajj\OneDrive - Arizona State University\ASU\PhD\Research\Coding\Simulator\holoocean\holoocean_pruning_project"
python -m holoocean_runs.run_pruning_epoch --N 10 --T_prune 30 --early_stop false --debug_viz true --debug_term true --tag iter006_n10
```

### N=20 with viz enabled:
```bash
$env:PYTHONIOENCODING="utf-8"
python -m holoocean_runs.run_pruning_epoch --N 20 --T_prune 60 --early_stop false --debug_viz true --debug_term true --tag iter006_n20
```

## Expected Results

1. **Matplotlib updates live** during simulation (not just at start/end)
2. **Console output** shows termination reason clearly
3. **epoch_summary.json** includes:
   - `termination_reason`: `"T_prune_reached"` (since early_stop=false)
   - `t_tree_first`: timestamp when tree first became valid
   - `t_goal_reached`: timestamp or null
4. **Tree edges** appear salmon/dashed while building, crimson/solid when committed

## Lessons Learned

1. Always include `plt.pause()` for live matplotlib updates in scripts
2. Track termination reason explicitly for debugging
3. Visual cues (color/style) help distinguish candidate vs committed state
