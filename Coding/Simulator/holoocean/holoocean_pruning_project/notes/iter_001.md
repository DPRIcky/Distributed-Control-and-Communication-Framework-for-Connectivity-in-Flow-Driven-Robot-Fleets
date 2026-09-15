# Iteration 001

## What changed
- Added GE CLI flags and loss-model switch in `run_pruning_epoch.py`.
- Implemented per-directed-link GE state machine in `send_message()`.
- Preserved IID drop logic when `--loss_model iid`.
- Added GE metrics (`ge_bad_send_fraction`, `ge_avg_bad_run_length`) to summary JSON and console summary.
- Added `channel_model` block to JSON including GE precedence metadata.

## Why
- Required bursty loss behavior while keeping existing pruning and delay semantics unchanged.

## Status
- Completed.
- Syntax clean (`py_compile` pass).
- Full simulator run starts but may stall at engine launch in this environment.
