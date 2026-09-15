#!/usr/bin/env python
"""
Paper-ready 2-panel rate-cap figure v2 (IROS single-column width).

Output: figures/rate_cap_paper_v2.png  +  figures/rate_cap_paper_v2.pdf

Improvements over v1:
  - No suptitle (caption in LaTeX)
  - Subset x-axis ticks: [inf, 8k, 1k, 240, 96, 48, 24, 18, 12, 6]
  - Tick labels rotated 65 deg, 9 pt font
  - Breakpoints annotated directly on plot (no legend entries for shading)
  - Legends compact and placed to avoid covering curves
  - Delay y-label: "Queue delay p95 (ms)", log scale, y-min > 0
  - Font 10-11 pt, thicker lines/markers
  - Exports PNG (300 dpi) + PDF

Usage (from holoocean_pruning_project/):
    python -m holoocean_runs.plot_rate_cap_paper_v2 \\
        "sweeps/rate_cap_N10/*.csv" \\
        "sweeps/rate_cap_N20/sweep_N20_rebuilt.csv" \\
        "sweeps/breakpoint_N20/sweep_N20_breakpoint_rebuilt.csv"
"""
import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import numpy as np
import pandas as pd
from matplotlib.transforms import blended_transform_factory

from holoocean_runs.plot_rate_cap_sweep import (
    load_and_merge,
    aggregate_by_cap,
    _DELAY_FLOOR_MS,
)

# ── Design constants ──────────────────────────────────────────────────────────
FONT_LABEL  = 10
FONT_TICK   = 9
FONT_LEGEND = 9
LW          = 2.1
MARKER_SIZE = 6
CAP_SIZE    = 3.5

# Breakpoints (last failure → first success)
_BRKPTS = {10: (12, 18), 20: (18, 24)}
_BRKPT_COLORS = {10: "tab:orange", 20: "tab:purple"}

# Fixed colors for N=10 and N=20
_N_COLORS = {10: "tab:blue", 20: "tab:orange"}

# Desired x-axis subset: high caps first, then fine-grained low-cap region
SUBSET_CAPS  = [np.inf, 8000, 1000, 240, 96, 48, 24, 18, 12, 6]
SUBSET_XLBLS = [r"$\infty$", "8k", "1k", "240", "96", "48", "24", "18", "12", "6"]


# ── Helpers ───────────────────────────────────────────────────────────────────

def _filter_subset(agg: pd.DataFrame) -> pd.DataFrame:
    """Keep only requested x-ticks; preserves high→low order from SUBSET_CAPS."""
    rows = []
    for cap_val, cap_lbl in zip(SUBSET_CAPS, SUBSET_XLBLS):
        if np.isinf(cap_val):
            mask = np.isinf(agg["cap_numeric"].values)
        else:
            mask = np.abs(agg["cap_numeric"].values - float(cap_val)) < 1.0
        sub = agg[mask]
        if sub.empty:
            continue
        row = sub.iloc[0].to_dict()
        row["cap_label"] = cap_lbl
        rows.append(row)
    if not rows:
        return agg   # fallback: return full agg unchanged
    return pd.DataFrame(rows).reset_index(drop=True)


def _find_xpos(agg: pd.DataFrame, cap_bps: float):
    """Row position of cap_bps in agg, or None."""
    for i, c in enumerate(agg["cap_numeric"].values):
        if math.isinf(cap_bps) and math.isinf(float(c)):
            return i
        if not math.isinf(cap_bps) and not math.isinf(float(c)):
            if abs(float(c) - float(cap_bps)) < 1.0:
                return i
    return None


def _add_breakpoint(ax, agg: pd.DataFrame, n_val: int):
    """
    Shade breakpoint region and add a small italic text label at the bottom
    of the axes (using blended data-x / axes-fraction-y transform).
    No legend entry.
    """
    bp_lo, bp_hi = _BRKPTS.get(int(n_val), (None, None))
    if bp_lo is None:
        return
    color  = _BRKPT_COLORS.get(int(n_val), "gray")
    lo_idx = _find_xpos(agg, bp_lo)
    hi_idx = _find_xpos(agg, bp_hi)
    if lo_idx is None or hi_idx is None:
        return

    x_left = min(lo_idx, hi_idx) - 0.5
    x_right = max(lo_idx, hi_idx) + 0.5
    x_mid  = (x_left + x_right) / 2

    ax.axvspan(x_left, x_right, color=color, alpha=0.14, zorder=0)

    # Blended transform: x in data coords, y in axes fraction
    trans = blended_transform_factory(ax.transData, ax.transAxes)
    ax.text(x_mid, 0.04, f"N={n_val}",
            transform=trans, ha="center", va="bottom",
            fontsize=6.5, color=color, fontstyle="italic",
            alpha=0.95, zorder=2)


def _sem(std, n):
    return std / np.sqrt(np.maximum(n, 1))


def _apply_xticks(ax, agg: pd.DataFrame):
    x = np.arange(len(agg))
    ax.set_xticks(x)
    ax.set_xticklabels(agg["cap_label"].values,
                       rotation=65, ha="right", rotation_mode="anchor",
                       fontsize=FONT_TICK)
    ax.set_xlim(-0.6, len(agg) - 0.4)


# ── Panel renderers ───────────────────────────────────────────────────────────

def _plot_success(ax, agg: pd.DataFrame, label: str, color: str):
    x = np.arange(len(agg))
    y    = agg["success_rate"].values
    yerr = _sem(agg["success_std"].values, agg["n_runs"].values)
    ax.errorbar(x, y, yerr=yerr, marker="o", capsize=CAP_SIZE,
                label=label, color=color,
                linewidth=LW, markersize=MARKER_SIZE,
                markeredgewidth=0.7, markeredgecolor="white")


def _plot_delay(ax, agg: pd.DataFrame, label: str, color: str):
    x    = np.arange(len(agg))
    y    = agg["delay_p95_mean"].values * 1000        # s → ms
    yerr = _sem(agg["delay_p95_std"].values, agg["n_runs"].values) * 1000
    y    = np.where(y < _DELAY_FLOOR_MS, _DELAY_FLOOR_MS, y)
    yerr = np.where(np.isnan(yerr) | (yerr < 0), 0, yerr)
    ax.errorbar(x, y, yerr=yerr, marker="s", capsize=CAP_SIZE,
                label=label, color=color,
                linewidth=LW, markersize=MARKER_SIZE,
                markeredgewidth=0.7, markeredgecolor="white")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Paper-ready 2-panel rate-cap figure v2")
    parser.add_argument("csv_files", nargs="+",
                        help="Same CSVs as plot_rate_cap_paper")
    parser.add_argument("--out_dir", type=str, default="figures")
    parser.add_argument("--stem",    type=str, default="rate_cap_paper_v2",
                        help="Output filename stem (no extension)")
    parser.add_argument("--show",    action="store_true")
    args = parser.parse_args()

    df = load_and_merge(args.csv_files)
    n_values = sorted(df["N"].dropna().unique())
    print(f"Loaded {len(df)} rows — N values: {n_values}")

    fig, (ax_s, ax_d) = plt.subplots(1, 2, figsize=(6.5, 2.8),
                                      constrained_layout=True)

    agg_by_n: dict[int, pd.DataFrame] = {}
    for i, n_val in enumerate(n_values):
        n_int = int(n_val)
        agg_full = aggregate_by_cap(df[df["N"] == n_val])
        agg_sub  = _filter_subset(agg_full)
        agg_by_n[n_int] = agg_sub
        color = _N_COLORS.get(n_int, plt.cm.tab10.colors[i])
        _plot_success(ax_s, agg_sub, label=f"N={n_int}", color=color)
        _plot_delay(ax_d,   agg_sub, label=f"N={n_int}", color=color)

    # Shared x-axis configuration
    # Both N groups use the same SUBSET_CAPS so tick positions are identical.
    ref_agg = next(iter(agg_by_n.values()))
    for ax in (ax_s, ax_d):
        _apply_xticks(ax, ref_agg)
        ax.set_xlabel("TX rate cap (bps)", fontsize=FONT_LABEL, labelpad=2)

    # ── Success panel ────────────────────────────────────────────────────────
    ax_s.set_ylabel("Success rate", fontsize=FONT_LABEL)
    ax_s.set_ylim(-0.05, 1.13)
    ax_s.axhline(1.0, color="gray", linestyle="--", alpha=0.35, linewidth=0.9)
    ax_s.tick_params(axis="y", labelsize=FONT_TICK)
    ax_s.grid(True, alpha=0.3)
    ax_s.legend(loc="lower left", fontsize=FONT_LEGEND,
                framealpha=0.85, edgecolor="lightgray",
                handlelength=1.5, borderpad=0.5)
    for n_int in agg_by_n:
        _add_breakpoint(ax_s, ref_agg, n_int)

    # ── Delay panel ──────────────────────────────────────────────────────────
    ax_d.set_ylabel("Queue delay p95 (ms)", fontsize=FONT_LABEL)
    ax_d.set_yscale("log")
    # Use a readable log formatter; show 0.1, 1, 10, 100, 1000, 10000
    ax_d.yaxis.set_major_formatter(mticker.LogFormatterMathtext())
    ax_d.yaxis.set_major_locator(mticker.LogLocator(base=10.0, numticks=8))
    ax_d.yaxis.set_minor_formatter(mticker.NullFormatter())
    ax_d.set_ylim(bottom=_DELAY_FLOOR_MS * 0.5)
    ax_d.tick_params(axis="y", labelsize=FONT_TICK)
    ax_d.grid(True, alpha=0.3, which="major")
    ax_d.legend(loc="upper left", fontsize=FONT_LEGEND,
                framealpha=0.85, edgecolor="lightgray",
                handlelength=1.5, borderpad=0.5)
    for n_int in agg_by_n:
        _add_breakpoint(ax_d, ref_agg, n_int)

    # ── Save ─────────────────────────────────────────────────────────────────
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    for ext, dpi in [("png", 300), ("pdf", None)]:
        out_path = out_dir / f"{args.stem}.{ext}"
        kw = {"dpi": dpi} if dpi else {}
        fig.savefig(out_path, bbox_inches="tight", **kw)
        print(f"Saved: {out_path}")

    if args.show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
