# Recurring Pitfalls and Fixes

## Pitfalls
- Running script as a file path can break local package imports (`pruning`) because `sys.path` starts at `holoocean_runs/`.
- Hidden tab/space mix in nested blocks can trigger `IndentationError` at runtime even after patch review.
- Environment may miss required simulator dependencies (`numpy`, `matplotlib`, `networkx`, `holoocean`).
- Inserting `elif` via patch into deeply nested blocks is fragile; prefer rewriting the full `if/elif/else` block.
- PowerShell here-strings with inline Python f-strings break on curly braces; use a `.py` script file instead.
- `create_file` tool fails on existing files; use `replace_string_in_file` for updates.
- Unicode characters (δ, π) in stdout cause `UnicodeEncodeError` on Windows; set `$env:PYTHONIOENCODING="utf-8"`.
- Early-stop bias: short runs don't produce enough GE state updates for metrics to converge to expected steady-state.
- Per-link sample sparsity: with N agents at 1 Hz broadcast, each directed link sees ~1 send/sec → need long runs for per-link stats.
- ID-biased tie-break: `snd < parent[rcv]` makes all nodes prefer lowest-ID sender → star topology.
- Parent flap: without hysteresis, nodes switch parents rapidly when distances are similar → unstable tree.
- Fast robots: high k_goal/v_max makes robots reach goals before tree dynamics can be observed.
- Matplotlib not updating: missing `plt.pause()` after `draw_idle()` and `flush_events()` causes figure to only show at end.
- Termination debugging: unclear why simulation ended early; no logging of termination reason.
- Red edge ambiguity: drawing candidate parent pointers without visual distinction from committed tree.
- Rate cap too low: if `rate_cap_bps` is below required throughput, protocol may fail to converge.

## Fix Patterns
- Prefer module execution from project root:
  - `python -m holoocean_runs.run_pruning_epoch ...`
- After editing nested Python control flow, run:
  - `python -m py_compile holoocean_runs/run_pruning_epoch.py`
- If imports fail, install into the configured `.venv` before rerunning.
- Use `multi_replace_string_in_file` for atomic multi-edit operations to avoid intermediate broken states.
- For GE validation, use `--early_stop false --T_prune 60` or longer to reach steady-state.
- Use `--ge_min_link_samples 5` for short validation runs.
- Use `--tiebreak_mode distance` (default) to prevent star topology; avoid `--tiebreak_mode id`.
- Use `--hysteresis_margin 0.2 --hysteresis_count 2` to prevent parent flap.
- Use conservative controller defaults: `--k_goal 0.3 --v_max 0.6 --thruster_gain 10 --thruster_clip 10`.
- Per-sender GE stats aggregate all outbound links per sender → higher sample counts for sparse networks.
- For live matplotlib updates in scripts: always add `plt.pause(0.001)` after `draw_idle()` + `flush_events()`.
- Use `--debug_term true` to see which termination condition triggered when runs end unexpectedly.
- Use `--no_viz true` to disable matplotlib and speed up runs without visualization overhead.
- For rate limiting sweeps: start with `--rate_cap_bps inf` as baseline, then decrease.
- Monitor `queue_overflow_drops` and `queue_delay_p95_s` to detect congestion.

## GE-specific Checklist
- `loss_model=iid` preserves legacy `p_drop` path; GE metrics are zero.
- `loss_model=ge` uses per-directed-link state `(i->j)`.
- Transition happens **per send** (once per `send_message()` call), not per tick.
- `ge_transition_unit: "per_send"` is logged in `channel_model` block.
- Delay/enqueue path unchanged for non-dropped packets.
- Summary JSON includes:
  - Global: `ge_bad_send_fraction`, `ge_avg_bad_run_length`, `ge_num_state_updates`, `ge_num_bad_runs`
  - Expected: `ge_bad_fraction_expected` = π_B = p_gb / (p_gb + p_bg)
  - Per-link: `ge_bad_send_fraction_per_link_stats`, `ge_bad_run_length_per_link_stats`
- If `L_bad` is set, it overrides `p_bg` with `1/L_bad`.
- Debug flags: `--ge_debug`, `--ge_debug_link "i,j"`, `--ge_debug_link_max`, `--ge_min_link_samples`
- Per-sender stats: `ge_senders_with_enough_samples`, `ge_senders_bad_frac_mean/std`, `ge_senders_run_len_mean/std`

## Tie-break Checklist (Iter 005)
- `--tiebreak_mode distance` (default): prefer sender closest to root; prevents star topology.
- `--tiebreak_mode id`: legacy behavior; causes all nodes to prefer lowest-ID → star.
- `--tiebreak_mode hash`: deterministic pseudo-random using `hash((rcv,snd,parent))`.
- Hysteresis: candidate must beat current parent by `margin` for `count` consecutive rounds.
- `_parent_hysteresis[rcv] = (candidate, consecutive_wins)` tracks state.
- Positions dict must be passed to `deliver_messages()` for distance-based tie-break.

## Visualization Checklist (Iter 006)
- `plt.ion()` must be called before creating figure for interactive mode.
- `plt.show(block=False)` must be used (not blocking `plt.show()`) before sim loop.
- `fig.canvas.draw_idle()` + `fig.canvas.flush_events()` + `plt.pause(0.001)` together for live updates.
- Viz refresh is wall-clock throttled by `--viz_interval` (default 0.35s).
- `--no_viz true` disables matplotlib entirely; PNG still saved at end (if fig was created).
- `--debug_viz true` prints refresh timestamps for debugging.
- Tree edges: salmon/dashed while building (`is_tree=False`), crimson/solid when committed (`is_tree=True`).

## Termination Checklist (Iter 006)
- `termination_reason` field in JSON: `T_prune_reached`, `early_stop_stable`, `keyboard_interrupt`.
- `t_tree_first`: timestamp when `is_tree` first becomes true.
- `t_tree_stable_first`: timestamp when tree is first stable for T_stable.
- `t_goal_reached`: timestamp when mean(d2goal) < 2.0m (or null).
- `--debug_term true` prints condition evaluations during run.
- HoloOcean has no internal episode limit (ticks_per_episode not set).
- Early stop requires: `early_stop=true` AND `sim_t > 2.0` AND `T_stable elapsed` AND `all δ finite`.

## Rate Limiting Checklist (Iter 007)
- `--rate_cap_bps inf` (default): no rate limiting, identical to previous behavior.
- `--rate_cap_bps <value>`: per-robot TX queue with rate-limited service.
- `--queue_max_msgs 100` (default): max messages per robot's TX queue.
- `--queue_drop_policy drop_tail` (default): drop new messages when queue full.
- `--queue_drop_policy drop_head`: drop oldest message when queue full.
- Processing order: enqueue → service queue → send_message (loss) → deliver.
- Queue metrics: `queue_delay_mean_s`, `queue_delay_p95_s`, `queue_overflow_drops`, `queue_max_occupancy`.
- `queue_overflow_drop_rate` = overflow_drops / total_tx; independent from loss model drops.
- Budget accumulation: `budget += rate_cap_Bps * dt`; capped at 5× PAYLOAD_BYTES to prevent bursts.
- Very low caps may prevent protocol convergence (insufficient message throughput).
