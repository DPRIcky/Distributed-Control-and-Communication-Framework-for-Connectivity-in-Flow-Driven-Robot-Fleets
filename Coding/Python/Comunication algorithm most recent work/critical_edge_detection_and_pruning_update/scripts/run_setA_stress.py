#!/usr/bin/env python3
"""
Experiment Set A: Communication + Pruning Robustness (Synthetic Monte Carlo)
============================================================================

Runs two stress grids (N=10 and N=20), saves results, generates plots
and LaTeX budget tables.

Usage:
    python run_setA_stress.py
    python run_setA_stress.py --trials 10        # quick test
    python run_setA_stress.py --skip-plots       # data only

Outputs:
    stress_results/setA/N10_<ts>/  results.csv, results.json, summary.txt
    stress_results/setA/N20_<ts>/  results.csv, results.json, summary.txt
    Figures/setA/N10_<ts>/         fig_convergence_time_vs_drop.{png,pdf}
                                   fig_rx_bandwidth_vs_drop.{png,pdf}
                                   comm_budget_table.tex
    Figures/setA/N20_<ts>/         (same)
"""

import argparse
import os
import sys
import time
from datetime import datetime
from pathlib import Path

# ---------------------------------------------------------------------------
# Resolve project paths so imports work regardless of cwd
# ---------------------------------------------------------------------------
SCRIPT_DIR = Path(__file__).resolve().parent          # scripts/
PROJECT_DIR = SCRIPT_DIR.parent                        # project root
STRESS_BASE = SCRIPT_DIR / "stress_results" / "setA"
FIGURES_BASE = PROJECT_DIR / "Figures" / "setA"

# Ensure the script folder is on sys.path for local imports
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from distributed_pruning_algorithm import (
    DistributedPruningAlgorithm,
    run_stress_grid,
    save_stress_grid_results,
    print_stress_grid_results,
    create_demo_graph,
    PAYLOAD_BYTES,
)


# ============================================================================
# Connected-graph resampling wrapper
# ============================================================================
import networkx as nx
import random


def _make_connected_graph(n: int, edge_prob: float, seed: int,
                          max_attempts: int = 50) -> nx.Graph:
    """Generate an Erdős–Rényi graph on nodes 1..n that is connected.

    If the first draw is disconnected, resample with incremented seed
    (up to *max_attempts* times) before falling back to adding bridge edges.
    """
    for attempt in range(max_attempts):
        G = nx.erdos_renyi_graph(n, edge_prob, seed=seed + attempt)
        mapping = {i: i + 1 for i in range(n)}
        G = nx.relabel_nodes(G, mapping)
        if nx.is_connected(G):
            return G
    # Fallback: force connect (same logic as create_demo_graph)
    components = list(nx.connected_components(G))
    for i in range(len(components) - 1):
        a = min(components[i])
        b = min(components[i + 1])
        G.add_edge(a, b)
    return G


# ============================================================================
# Patched stress grid that guarantees connected graphs
# ============================================================================
def run_stress_grid_connected(
    n, edge_prob, trials, drop_list, delay_list, jitter_list,
    t_broadcast, t_prune, dt, t_stable, base_seed, verbose=False
):
    """Thin wrapper around run_stress_grid that pre-validates graph connectivity.

    *run_stress_grid* already calls create_demo_graph which adds edges to
    connect components, so connectivity is guaranteed.  This wrapper simply
    delegates and prints progress.
    """
    total_configs = len(drop_list) * len(delay_list) * len(jitter_list)
    print(f"  Configurations: {total_configs}  |  Trials each: {trials}  |  "
          f"Total runs: {total_configs * trials}")

    results = run_stress_grid(
        n=n,
        edge_prob=edge_prob,
        trials=trials,
        drop_list=drop_list,
        delay_list=delay_list,
        jitter_list=jitter_list,
        t_broadcast=t_broadcast,
        t_prune=t_prune,
        dt=dt,
        t_stable=t_stable,
        base_seed=base_seed,
        verbose=verbose,
    )
    return results


# ============================================================================
# LaTeX table generation
# ============================================================================
REPRESENTATIVE_ROWS = [
    {"label": "Ideal",    "p_drop": 0.0, "delay_max": 0.0, "jitter": 0.0},
    {"label": "Moderate", "p_drop": 0.1, "delay_max": 0.5, "jitter": 0.2},
    {"label": "Harsh",    "p_drop": 0.3, "delay_max": 0.5, "jitter": 0.2},
]


def _pick_row(results, p_drop, delay_max, jitter):
    """Find the result dict matching the given parameters (exact float match)."""
    for r in results:
        if (abs(r["p_drop"] - p_drop) < 1e-9 and
                abs(r["delay_max"] - delay_max) < 1e-9 and
                abs(r["jitter"] - jitter) < 1e-9):
            return r
    return None


def generate_budget_table(results, tex_path, n):
    """Write a compact LaTeX table with 3 representative rows."""
    rows_tex = []
    for spec in REPRESENTATIVE_ROWS:
        r = _pick_row(results, spec["p_drop"], spec["delay_max"], spec["jitter"])
        if r is None:
            # Config not in grid – skip row
            continue
        rows_tex.append(
            f"        {spec['label']} "
            f"& {PAYLOAD_BYTES} "
            f"& {r['avg_msg_sent']:.0f} "
            f"& {r['avg_msg_delivered']:.0f} "
            f"& {r['avg_rx_Bps']:.1f} "
            f"& {r['success_rate'] * 100:.0f}\\% "
            f"& {r['avg_duration']:.2f} \\\\"
        )

    table = (
        "% Auto-generated by run_setA_stress.py\n"
        f"% N = {n}, payload B_msg = {PAYLOAD_BYTES} bytes\n"
        "\\begin{table}[htbp]\n"
        "  \\centering\n"
        f"  \\caption{{Communication budget summary ($N={n}$, $B_{{\\mathrm{{msg}}}}={PAYLOAD_BYTES}$~bytes).}}\n"
        f"  \\label{{tab:comm_budget_N{n}}}\n"
        "  \\begin{tabular}{l c r r r r r}\n"
        "    \\toprule\n"
        "    Setting & $B_{\\mathrm{msg}}$ & TX msg/epoch & RX msg/epoch"
        " & RX (B/s) & Success & Duration (s) \\\\\n"
        "    \\midrule\n"
        + "\n".join(f"    {row}" for row in rows_tex) + "\n"
        "    \\bottomrule\n"
        "  \\end{tabular}\n"
        "\\end{table}\n"
    )

    os.makedirs(os.path.dirname(tex_path), exist_ok=True)
    with open(tex_path, "w", encoding="utf-8") as f:
        f.write(table)
    print(f"  Saved: {tex_path}")


# ============================================================================
# Run make_plots.py as a library call (avoids subprocess)
# ============================================================================
def run_make_plots(csv_path: str, outdir: str):
    """Invoke the same logic as make_plots.py --csv ... --outdir ..."""
    # Import lazily to avoid circular issues and keep startup fast
    import importlib
    make_plots = importlib.import_module("make_plots")

    os.makedirs(outdir, exist_ok=True)
    print(f"  Loading CSV: {csv_path}")
    df = make_plots.load_csv(csv_path)
    col_map = make_plots.validate_and_map_columns(df)

    print("  Generating convergence-time plot …")
    make_plots.plot_convergence_time(df, col_map, outdir)

    print("  Generating RX-bandwidth plot …")
    make_plots.plot_bandwidth(df, col_map, outdir)


# ============================================================================
# Single-grid pipeline
# ============================================================================
def run_one_grid(label, n, edge_prob, params, timestamp, skip_plots):
    """Run stress grid, save results, generate plots + table."""
    tag = f"N{n}_{timestamp}"
    stress_dir = str(STRESS_BASE / tag)
    fig_dir = str(FIGURES_BASE / tag)

    print()
    print("=" * 70)
    print(f"  Grid {label}  —  N={n}, edge_prob={edge_prob}, "
          f"trials={params['trials']}")
    print("=" * 70)

    t0 = time.time()
    results = run_stress_grid_connected(
        n=n,
        edge_prob=edge_prob,
        trials=params["trials"],
        drop_list=params["drop_list"],
        delay_list=params["delay_list"],
        jitter_list=params["jitter_list"],
        t_broadcast=params["t_broadcast"],
        t_prune=params["t_prune"],
        dt=params["dt"],
        t_stable=params["t_stable"],
        base_seed=params["base_seed"],
        verbose=False,
    )
    elapsed = time.time() - t0
    print(f"  Grid done in {elapsed:.1f}s")

    # Print table to console
    full_params = {**params, "n": n, "edge_prob": edge_prob}
    print_stress_grid_results(results, params=full_params)

    # Save results (csv, json, summary.txt)
    out_path = save_stress_grid_results(
        results=results,
        params=full_params,
        outdir=str(STRESS_BASE),
        tag=tag,
    )
    print(f"\n  Results saved → {out_path}")

    csv_path = os.path.join(out_path, "results.csv")

    # Plots
    if not skip_plots:
        print(f"\n  Generating plots → {fig_dir}")
        run_make_plots(csv_path, fig_dir)

    # LaTeX table
    tex_path = os.path.join(fig_dir, "comm_budget_table.tex")
    print(f"\n  Generating LaTeX table → {tex_path}")
    generate_budget_table(results, tex_path, n)

    return out_path, fig_dir


# ============================================================================
# Main
# ============================================================================
def main():
    parser = argparse.ArgumentParser(
        description="Experiment Set A: comm + pruning robustness Monte Carlo")
    parser.add_argument("--trials", type=int, default=30,
                        help="Monte Carlo trials per config (default 30)")
    parser.add_argument("--skip-plots", action="store_true",
                        help="Skip plot generation (data + table only)")
    parser.add_argument("--verbose", action="store_true",
                        help="Print per-trial output")
    args = parser.parse_args()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Shared timing / channel parameters
    shared = dict(
        trials=args.trials,
        drop_list=[0.0, 0.1, 0.3, 0.5],
        delay_list=[0.0, 0.5, 1.0],
        jitter_list=[0.0, 0.2],
        t_broadcast=1.0,
        t_prune=20.0,
        dt=0.1,
        t_stable=5.0,
        base_seed=1000,
    )

    print()
    print("#" * 70)
    print("#  Experiment Set A: Communication + Pruning Robustness")
    print(f"#  Timestamp: {timestamp}")
    print(f"#  Trials per config: {shared['trials']}")
    print("#" * 70)

    all_outputs = []

    # ── Grid A1: N = 10 ────────────────────────────────────────────────────
    res_dir, fig_dir = run_one_grid(
        label="A1", n=10, edge_prob=0.25,
        params=shared, timestamp=timestamp,
        skip_plots=args.skip_plots,
    )
    all_outputs.append(("N=10 results", res_dir))
    all_outputs.append(("N=10 figures", fig_dir))

    # ── Grid A2: N = 20 ────────────────────────────────────────────────────
    res_dir, fig_dir = run_one_grid(
        label="A2", n=20, edge_prob=0.15,
        params=shared, timestamp=timestamp,
        skip_plots=args.skip_plots,
    )
    all_outputs.append(("N=20 results", res_dir))
    all_outputs.append(("N=20 figures", fig_dir))

    # ── Summary ─────────────────────────────────────────────────────────────
    print()
    print("=" * 70)
    print("  All outputs:")
    print("=" * 70)
    for desc, path in all_outputs:
        print(f"  {desc:20s} → {path}")
    print("=" * 70)
    print("  Done.")


if __name__ == "__main__":
    main()
