#!/usr/bin/env python
"""
Plot rate cap sweep results.
Generates: success rate vs cap, queue delay vs cap, rx_Bps vs cap, convergence time vs cap.

Usage:
    python -m holoocean_runs.plot_rate_cap_sweep "sweeps/rate_cap_N10/sweep_N10_*.csv"
    python -m holoocean_runs.plot_rate_cap_sweep "sweeps/rate_cap_N10/*.csv" \\
        "sweeps/rate_cap_N20/sweep_N20_rebuilt.csv" \\
        "sweeps/breakpoint_N20/sweep_N20_breakpoint_rebuilt.csv"
"""
import argparse
import glob
import math
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import numpy as np
import pandas as pd

# Breakpoint regions: {N: (last_failure_bps, first_success_bps)}
_BREAKPOINTS = {
    10: (12, 18),
    20: (18, 24),
}

_BREAKPOINT_COLORS = {
    10: "tab:orange",
    20: "tab:purple",
}


def expand_globs(patterns: list[str]) -> list[str]:
    """Expand glob patterns (needed on Windows where shell doesn't expand)."""
    expanded = []
    for p in patterns:
        matches = glob.glob(p)
        if matches:
            expanded.extend(matches)
        else:
            expanded.append(p)
    return expanded


def load_and_merge(csv_paths: list[str]) -> pd.DataFrame:
    """Load multiple CSVs and concatenate."""
    expanded = expand_globs(csv_paths)
    if not expanded:
        raise ValueError(f"No files found matching: {csv_paths}")

    dfs = []
    seen = set()
    for p in expanded:
        if not Path(p).exists():
            print(f"Warning: file not found: {p}")
            continue
        p_resolved = str(Path(p).resolve())
        if p_resolved in seen:
            print(f"Skipping duplicate file: {p}")
            continue
        seen.add(p_resolved)
        df = pd.read_csv(p)
        dfs.append(df)

    if not dfs:
        raise ValueError(f"No valid CSV files found in: {csv_paths}")

    merged = pd.concat(dfs, ignore_index=True)
    # Drop exact duplicate rows (same cap + seed + N) to guard against double-loaded files
    key_cols = [c for c in ["rate_cap_bps", "seed", "N"] if c in merged.columns]
    before = len(merged)
    merged = merged.drop_duplicates(subset=key_cols)
    after = len(merged)
    if before != after:
        print(f"Dropped {before - after} duplicate rows (same cap+seed+N)")
    return merged


def cap_to_numeric(cap) -> float:
    """Convert 'inf' string to np.inf for sorting/plotting."""
    if cap == "inf":
        return np.inf
    return float(cap)


def numeric_to_label(cap: float) -> str:
    """Human-readable cap label."""
    if math.isinf(cap):
        return "inf"
    if cap >= 1000:
        return f"{int(cap/1000)}k"
    return str(int(cap))


def aggregate_by_cap(df: pd.DataFrame) -> pd.DataFrame:
    """Aggregate metrics by rate_cap_bps."""
    df = df.copy()
    df["cap_numeric"] = df["rate_cap_bps"].apply(cap_to_numeric)

    agg = df.groupby("cap_numeric").agg({
        "success": ["mean", "std", "count"],
        "t_tree_first": ["mean", "std"],
        "t_tree_stable_first": ["mean", "std"],
        "rx_Bps": ["mean", "std"],
        "queue_delay_p95_s": ["mean", "std"],
        "queue_overflow_drop_rate": ["mean", "std"],
    }).reset_index()

    # Flatten column names
    agg.columns = [
        "cap_numeric",
        "success_rate", "success_std", "n_runs",
        "t_tree_first_mean", "t_tree_first_std",
        "t_stable_mean", "t_stable_std",
        "rx_Bps_mean", "rx_Bps_std",
        "delay_p95_mean", "delay_p95_std",
        "overflow_rate_mean", "overflow_rate_std",
    ]

    # Sort by cap descending (inf first, then high to low)
    agg = agg.sort_values("cap_numeric", ascending=False)
    agg["cap_label"] = agg["cap_numeric"].apply(numeric_to_label)

    return agg


def _find_cap_xpos(agg: pd.DataFrame, cap_bps: float) -> int | None:
    """Return the integer x-position (row index in sorted agg) of a given cap value."""
    caps = agg["cap_numeric"].values
    for i, c in enumerate(caps):
        if not math.isinf(c) and int(round(c)) == int(round(cap_bps)):
            return i
    return None


def add_breakpoint_band(ax, agg: pd.DataFrame, n_val: int, alpha: float = 0.12):
    """Shade the breakpoint region for the given N value."""
    if int(n_val) not in _BREAKPOINTS:
        return
    bp_lo, bp_hi = _BREAKPOINTS[int(n_val)]
    color = _BREAKPOINT_COLORS.get(int(n_val), "gray")

    lo_idx = _find_cap_xpos(agg, bp_lo)
    hi_idx = _find_cap_xpos(agg, bp_hi)

    if lo_idx is None or hi_idx is None:
        return

    x_left = min(lo_idx, hi_idx) - 0.5
    x_right = max(lo_idx, hi_idx) + 0.5
    ax.axvspan(x_left, x_right, color=color, alpha=alpha, zorder=0,
               label=f"N={n_val} breakpoint ({bp_lo}–{bp_hi} bps)")


def plot_success_rate(ax, agg: pd.DataFrame, label: str = None, color: str = None):
    """Plot success rate vs cap."""
    x = range(len(agg))
    y = agg["success_rate"].values
    yerr = agg["success_std"].values / np.sqrt(agg["n_runs"].values)  # SEM

    ax.errorbar(x, y, yerr=yerr, marker="o", capsize=3, label=label, color=color)
    ax.set_xticks(x)
    ax.set_xticklabels(agg["cap_label"].values, rotation=45, ha="right")
    ax.set_xlabel("TX rate cap (bps)")
    ax.set_ylabel("Success Rate")
    ax.set_ylim(-0.05, 1.05)
    ax.axhline(0.9, color="gray", linestyle="--", alpha=0.5, linewidth=0.8)
    ax.grid(True, alpha=0.3)


_DELAY_FLOOR_MS = 0.1  # floor for log scale (caps with ~0 delay)


def plot_delay_p95(ax, agg: pd.DataFrame, label: str = None, color: str = None):
    """Plot queue delay p95 vs cap (log scale, ms)."""
    x = range(len(agg))
    y = agg["delay_p95_mean"].values * 1000  # s → ms
    yerr = agg["delay_p95_std"].values * 1000 / np.sqrt(agg["n_runs"].values)

    # Floor zero/near-zero values so log scale works
    y = np.where(y < _DELAY_FLOOR_MS, _DELAY_FLOOR_MS, y)
    yerr = np.where(np.isnan(yerr) | (yerr < 0), 0, yerr)

    ax.errorbar(x, y, yerr=yerr, marker="s", capsize=3, label=label, color=color)
    ax.set_xticks(x)
    ax.set_xticklabels(agg["cap_label"].values, rotation=45, ha="right")
    ax.set_xlabel("TX rate cap (bps)")
    ax.set_ylabel("Queue Delay p95 (ms)")
    ax.set_yscale("log")
    ax.yaxis.set_major_formatter(mticker.LogFormatterSciNotation(labelOnlyBase=False))
    ax.set_ylim(bottom=_DELAY_FLOOR_MS * 0.5)
    ax.grid(True, alpha=0.3, which="both")


def plot_rx_throughput(ax, agg: pd.DataFrame, label: str = None, color: str = None):
    """Plot rx_Bps vs cap."""
    x = range(len(agg))
    y = agg["rx_Bps_mean"].values
    yerr = agg["rx_Bps_std"].values / np.sqrt(agg["n_runs"].values)

    ax.errorbar(x, y, yerr=yerr, marker="^", capsize=3, label=label, color=color)
    ax.set_xticks(x)
    ax.set_xticklabels(agg["cap_label"].values, rotation=45, ha="right")
    ax.set_xlabel("TX rate cap (bps)")
    ax.set_ylabel("Received Throughput (B/s)")
    ax.grid(True, alpha=0.3)


def plot_convergence_time(ax, agg: pd.DataFrame, label: str = None, color: str = None):
    """Plot t_tree_first and t_tree_stable_first vs cap."""
    x = list(range(len(agg)))
    sem = lambda std, n: std / np.sqrt(n)

    # t_tree_first (first time spanning tree forms)
    y1 = agg["t_tree_first_mean"].values
    yerr1 = sem(agg["t_tree_first_std"].values, agg["n_runs"].values)
    lbl_first = f"{label} first tree" if label else "First tree"
    ax.errorbar(x, y1, yerr=yerr1, marker="o", capsize=3, label=lbl_first,
                color=color, linestyle="--", alpha=0.7)

    # t_tree_stable_first (first time tree is stable)
    y2 = agg["t_stable_mean"].values
    yerr2 = sem(agg["t_stable_std"].values, agg["n_runs"].values)
    lbl_stable = f"{label} stable tree" if label else "Stable tree"
    ax.errorbar(x, y2, yerr=yerr2, marker="D", capsize=3, label=lbl_stable,
                color=color, linestyle="-")

    ax.set_xticks(x)
    ax.set_xticklabels(agg["cap_label"].values, rotation=45, ha="right")
    ax.set_xlabel("TX rate cap (bps)")
    ax.set_ylabel("Time (s)")
    ax.grid(True, alpha=0.3)


def main():
    parser = argparse.ArgumentParser(description="Plot rate cap sweep results")
    parser.add_argument("csv_files", nargs="+", help="CSV files from sweep_rate_cap.py")
    parser.add_argument("--labels", nargs="*", help="Labels for each CSV file group")
    parser.add_argument("--out", type=str, default="rate_cap_sweep_plots.png", help="Output file")
    parser.add_argument("--show", action="store_true", help="Show plot interactively")
    parser.add_argument("--no_breakpoint_bands", action="store_true",
                        help="Suppress breakpoint shading")
    args = parser.parse_args()

    # Load data (deduplication built into load_and_merge)
    df = load_and_merge(args.csv_files)
    print(f"Loaded {len(df)} rows")

    # Check if we have multiple N values
    n_values = sorted(df["N"].dropna().unique())
    print(f"N values: {n_values}")

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("Rate Cap Sweep Results", fontsize=12)
    axes = axes.flatten()  # [success, delay, throughput, convergence]

    colors = plt.cm.tab10.colors
    agg_by_n = {}

    if len(n_values) > 1:
        for i, n_val in enumerate(n_values):
            df_n = df[df["N"] == n_val]
            agg = aggregate_by_cap(df_n)
            agg_by_n[int(n_val)] = agg
            label = f"N={int(n_val)}"
            color = colors[i % len(colors)]
            plot_success_rate(axes[0], agg, label=label, color=color)
            plot_delay_p95(axes[1], agg, label=label, color=color)
            plot_rx_throughput(axes[2], agg, label=label, color=color)
            plot_convergence_time(axes[3], agg, label=label, color=color)

        # Add breakpoint bands (after all data so zorder=0 goes behind lines)
        if not args.no_breakpoint_bands:
            for n_val, agg in agg_by_n.items():
                for ax in axes:
                    add_breakpoint_band(ax, agg, n_val)

        for ax in axes:
            ax.legend(loc="best", fontsize=7)
    else:
        agg = aggregate_by_cap(df)
        n_val = int(n_values[0]) if n_values else 0
        agg_by_n[n_val] = agg
        plot_success_rate(axes[0], agg, label=f"N={n_val}")
        plot_delay_p95(axes[1], agg, label=f"N={n_val}")
        plot_rx_throughput(axes[2], agg, label=f"N={n_val}")
        plot_convergence_time(axes[3], agg, label=f"N={n_val}")

        if not args.no_breakpoint_bands:
            for ax in axes:
                add_breakpoint_band(ax, agg, n_val)

        axes[3].legend(loc="best", fontsize=7)

    axes[0].set_title("(a) Success Rate")
    axes[1].set_title("(b) Queue Delay p95")
    axes[2].set_title("(c) Received Throughput")
    axes[3].set_title("(d) Convergence Time")

    plt.tight_layout()

    out_path = Path(args.out)
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Saved: {out_path}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
