#!/usr/bin/env python
"""
Paper-ready 2-panel rate-cap sweep figure (single-column width).

Panels:
  (a) Success rate vs TX rate cap  (N=10 & N=20)
  (b) Queue delay p95 vs TX rate cap  (N=10 & N=20, log scale)

Breakpoint bands and large fonts included.
The full 2×2 internal figure (rate_cap_combined.png) is unchanged.

Usage (from holoocean_pruning_project/):
    python -m holoocean_runs.plot_rate_cap_paper \\
        "sweeps/rate_cap_N10/*.csv" \\
        "sweeps/rate_cap_N20/sweep_N20_rebuilt.csv" \\
        "sweeps/breakpoint_N20/sweep_N20_breakpoint_rebuilt.csv" \\
        --out sweeps/rate_cap_paper.png
"""
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import numpy as np
import pandas as pd

from holoocean_runs.plot_rate_cap_sweep import (
    load_and_merge,
    aggregate_by_cap,
    add_breakpoint_band,
    _DELAY_FLOOR_MS,
)

# ── Typography & style ────────────────────────────────────────────────────────
FONT_TITLE  = 13
FONT_LABEL  = 12
FONT_TICK   = 11
FONT_LEGEND = 10
LINE_WIDTH  = 2.2
MARKER_SIZE = 7
CAP_SIZE    = 4


def _sem(std, n):
    return std / np.sqrt(np.maximum(n, 1))


def plot_success_paper(ax, agg: pd.DataFrame, label: str, color: str):
    x = np.arange(len(agg))
    y = agg["success_rate"].values
    yerr = _sem(agg["success_std"].values, agg["n_runs"].values)

    ax.errorbar(x, y, yerr=yerr, marker="o", capsize=CAP_SIZE,
                label=label, color=color,
                linewidth=LINE_WIDTH, markersize=MARKER_SIZE)
    ax.set_xticks(x)
    ax.set_xticklabels(agg["cap_label"].values, rotation=45, ha="right",
                       fontsize=FONT_TICK)
    ax.set_xlabel("TX rate cap (bps)", fontsize=FONT_LABEL)
    ax.set_ylabel("Success Rate", fontsize=FONT_LABEL)
    ax.set_ylim(-0.05, 1.12)
    ax.axhline(1.0, color="gray", linestyle="--", alpha=0.4, linewidth=1.0)
    ax.tick_params(axis="y", labelsize=FONT_TICK)
    ax.grid(True, alpha=0.3)


def plot_delay_paper(ax, agg: pd.DataFrame, label: str, color: str):
    x = np.arange(len(agg))
    y = agg["delay_p95_mean"].values * 1000  # s → ms
    yerr = agg["delay_p95_std"].values * 1000 / np.sqrt(
        np.maximum(agg["n_runs"].values, 1))

    y    = np.where(y < _DELAY_FLOOR_MS, _DELAY_FLOOR_MS, y)
    yerr = np.where(np.isnan(yerr) | (yerr < 0), 0, yerr)

    ax.errorbar(x, y, yerr=yerr, marker="s", capsize=CAP_SIZE,
                label=label, color=color,
                linewidth=LINE_WIDTH, markersize=MARKER_SIZE)
    ax.set_xticks(x)
    ax.set_xticklabels(agg["cap_label"].values, rotation=45, ha="right",
                       fontsize=FONT_TICK)
    ax.set_xlabel("TX rate cap (bps)", fontsize=FONT_LABEL)
    ax.set_ylabel("Queue Delay p95 (ms)", fontsize=FONT_LABEL)
    ax.set_yscale("log")
    ax.yaxis.set_major_formatter(mticker.LogFormatterSciNotation(labelOnlyBase=False))
    ax.set_ylim(bottom=_DELAY_FLOOR_MS * 0.5)
    ax.tick_params(axis="y", labelsize=FONT_TICK)
    ax.grid(True, alpha=0.3, which="both")


def main():
    parser = argparse.ArgumentParser(
        description="Paper-ready 2-panel rate-cap figure")
    parser.add_argument("csv_files", nargs="+",
                        help="Same CSV files as plot_rate_cap_sweep")
    parser.add_argument("--out", type=str, default="sweeps/rate_cap_paper.png")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    df = load_and_merge(args.csv_files)
    n_values = sorted(df["N"].dropna().unique())
    print(f"Loaded {len(df)} rows, N values: {n_values}")

    fig, axes = plt.subplots(1, 2, figsize=(10, 4.2))

    colors = plt.cm.tab10.colors
    agg_by_n = {}

    for i, n_val in enumerate(n_values):
        df_n = df[df["N"] == n_val]
        agg = aggregate_by_cap(df_n)
        agg_by_n[int(n_val)] = agg
        label  = f"N={int(n_val)}"
        color  = colors[i % len(colors)]
        plot_success_paper(axes[0], agg, label=label, color=color)
        plot_delay_paper(axes[1], agg, label=label, color=color)

    # Breakpoint bands
    for n_val, agg in agg_by_n.items():
        for ax in axes:
            add_breakpoint_band(ax, agg, n_val)

    axes[0].set_title("(a) Convergence Success Rate", fontsize=FONT_TITLE)
    axes[1].set_title("(b) Queue Delay p95", fontsize=FONT_TITLE)

    for ax in axes:
        ax.legend(loc="best", fontsize=FONT_LEGEND)

    fig.suptitle(
        r"$\delta$-BFS Protocol: Effect of TX Rate Cap on Convergence"
        "\n(N=10, N=20; 3 seeds; shaded = breakpoint region)",
        fontsize=11)

    plt.tight_layout(rect=[0, 0, 1, 0.91])

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=200, bbox_inches="tight")
    print(f"Saved: {out_path}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
