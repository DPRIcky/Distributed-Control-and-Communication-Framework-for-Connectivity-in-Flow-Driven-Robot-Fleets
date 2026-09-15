#!/usr/bin/env python
"""
Plot Simulation 3: distance-dependent packet drop impairment comparison.
Generates a 1×3 (or 3-panel) figure comparing baseline vs weak vs strong.

Panels:
  (a) Success rate per condition
  (b) t_tree_stable_first (convergence to stable tree) per condition
  (c) Received throughput (rx_Bps) per condition

Usage (from holoocean_pruning_project/):
    python -m holoocean_runs.plot_sim3 sweeps/sim3/sim3_results.csv \\
        --out sweeps/sim3/sim3_comparison.png
"""
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

CONDITION_ORDER = ["baseline", "weak", "strong"]
CONDITION_LABELS = {
    "baseline": "Baseline\n(IID p=0.10)",
    "weak":     "Weak\n(dist, k=0.25)",
    "strong":   "Strong\n(dist, k=0.90)",
}
CONDITION_COLORS = {
    "baseline": "tab:blue",
    "weak":     "tab:orange",
    "strong":   "tab:red",
}


def load_data(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    # Ensure condition column is categorical with defined order
    df["condition"] = pd.Categorical(df["condition"], categories=CONDITION_ORDER, ordered=True)
    return df.sort_values("condition")


def aggregate(df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for cond in CONDITION_ORDER:
        grp = df[df["condition"] == cond]
        if grp.empty:
            continue
        n = len(grp)
        rows.append({
            "condition":         cond,
            "n_runs":            n,
            "success_rate":      grp["success"].mean(),
            "success_sem":       grp["success"].sem(),
            "t_stable_mean":     grp["t_tree_stable_first"].mean(),
            "t_stable_sem":      grp["t_tree_stable_first"].sem(),
            "t_first_mean":      grp["t_tree_first"].mean(),
            "t_first_sem":       grp["t_tree_first"].sem(),
            "rx_Bps_mean":       grp["rx_Bps"].mean(),
            "rx_Bps_sem":        grp["rx_Bps"].sem(),
            "drop_rate_mean":    grp["observed_drop_rate"].mean(),
            "drop_rate_sem":     grp["observed_drop_rate"].sem(),
            "p_drop_eff_mean":   grp["p_drop_eff_mean"].mean() if "p_drop_eff_mean" in grp else np.nan,
            "p_drop_eff_p95_mean": grp["p_drop_eff_p95"].mean() if "p_drop_eff_p95" in grp else np.nan,
            "send_dist_p95_mean":grp["send_dist_p95_m"].mean() if "send_dist_p95_m" in grp else np.nan,
        })
    return pd.DataFrame(rows)


def bar_panel(ax, agg: pd.DataFrame, y_col: str, yerr_col: str,
              ylabel: str, title: str, ymax=None):
    x = np.arange(len(agg))
    colors = [CONDITION_COLORS.get(c, "gray") for c in agg["condition"]]
    y = agg[y_col].values
    yerr = agg[yerr_col].values
    yerr = np.where(np.isnan(yerr), 0, yerr)

    bars = ax.bar(x, y, yerr=yerr, capsize=5, color=colors, alpha=0.85,
                  error_kw={"elinewidth": 1.5})
    ax.set_xticks(x)
    ax.set_xticklabels([CONDITION_LABELS.get(c, c) for c in agg["condition"]], fontsize=8)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    if ymax is not None:
        ax.set_ylim(0, ymax)
    ax.grid(True, axis="y", alpha=0.3)

    # Annotate bars with values
    for bar, val, err in zip(bars, y, yerr):
        if not np.isnan(val):
            ax.text(bar.get_x() + bar.get_width() / 2,
                    bar.get_height() + err + (ymax or max(y)) * 0.02,
                    f"{val:.2f}" if val < 10 else f"{val:.1f}",
                    ha="center", va="bottom", fontsize=7)


def bar_panel_paper(ax, agg: pd.DataFrame, y_col: str, yerr_col: str,
                    ylabel: str, title: str, ymax=None,
                    font_label=10, font_tick=9, font_annot=8.5,
                    capsize=5, elinewidth=1.8):
    """Paper-ready bar panel: larger fonts, thicker error bars, clean grid."""
    x = np.arange(len(agg))
    colors = [CONDITION_COLORS.get(c, "gray") for c in agg["condition"]]
    y = agg[y_col].values
    yerr = agg[yerr_col].values
    yerr = np.where(np.isnan(yerr), 0, yerr)

    bars = ax.bar(x, y, yerr=yerr, capsize=capsize, color=colors, alpha=0.85,
                  error_kw={"elinewidth": elinewidth, "capthick": elinewidth})
    ax.set_xticks(x)
    ax.set_xticklabels([CONDITION_LABELS.get(c, c) for c in agg["condition"]],
                       fontsize=font_tick)
    ax.set_ylabel(ylabel, fontsize=font_label)
    ax.set_title(title, fontsize=font_label, pad=4)

    # Set ylim with headroom so annotations clear the error-bar caps
    ymax_auto = float(np.nanmax(y + yerr)) * 1.22 if ymax is None else ymax
    ax.set_ylim(0, ymax_auto)

    ax.tick_params(axis="y", labelsize=font_tick)
    ax.grid(True, axis="y", alpha=0.3)

    # Value annotations: placed 3% of axis range above each error-bar cap
    offset = ymax_auto * 0.03
    for bar, val, err in zip(bars, y, yerr):
        if not np.isnan(val):
            ax.text(bar.get_x() + bar.get_width() / 2,
                    bar.get_height() + err + offset,
                    f"{val:.1f}" if val >= 10 else f"{val:.2f}",
                    ha="center", va="bottom", fontsize=font_annot)


def make_paper_figure(agg: pd.DataFrame, out_path: Path, show: bool = False):
    """
    Paper-ready 2-panel figure for single-column width.
    Panels: (a) t_tree_stable_first, (b) Received throughput.
    No suptitle; constrained_layout; PNG (300 dpi) + PDF.
    """
    FONT_LABEL = 10
    FONT_TICK  = 9
    FONT_ANNOT = 8.5

    fig, (ax_t, ax_r) = plt.subplots(1, 2, figsize=(6.5, 2.8),
                                      constrained_layout=True)

    # (a) Stable-tree convergence time
    bar_panel_paper(ax_t, agg,
                    y_col="t_stable_mean", yerr_col="t_stable_sem",
                    ylabel="Time (s)",
                    title="(a) Stable-tree convergence (mean\u202f\u00b1\u202fSEM)",
                    font_label=FONT_LABEL, font_tick=FONT_TICK, font_annot=FONT_ANNOT)

    # (b) Received throughput
    bar_panel_paper(ax_r, agg,
                    y_col="rx_Bps_mean", yerr_col="rx_Bps_sem",
                    ylabel="Received throughput (B/s)",
                    title="(b) Received throughput",
                    font_label=FONT_LABEL, font_tick=FONT_TICK, font_annot=FONT_ANNOT)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    for ext, dpi in [("png", 300), ("pdf", None)]:
        p = out_path.with_suffix(f".{ext}")
        kw = {"dpi": dpi} if dpi else {}
        fig.savefig(p, bbox_inches="tight", **kw)
        print(f"Saved: {p}")
    if show:
        plt.show()
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Plot Sim3 dist-impairment comparison")
    parser.add_argument("csv_file", help="CSV from sweep_sim3.py")
    parser.add_argument("--out", type=str, default="sweeps/sim3/sim3_comparison.png")
    parser.add_argument("--paper", type=str, default=None,
                        help="Stem for paper figure, e.g. figures/sim3_comparison_paper "
                             "(no extension; PNG+PDF generated).")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    df = load_data(args.csv_file)
    agg = aggregate(df)

    print(f"Loaded {len(df)} rows from {args.csv_file}")
    print(agg[["condition", "n_runs", "success_rate", "t_stable_mean", "rx_Bps_mean",
               "drop_rate_mean"]].to_string(index=False))

    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    fig.suptitle("Simulation 3: Distance-Dependent Impairment (N=20, T=30s, 5 seeds)",
                 fontsize=11)

    # (a) Success rate
    bar_panel(axes[0], agg,
              y_col="success_rate", yerr_col="success_sem",
              ylabel="Success Rate", title="(a) Success Rate", ymax=1.1)
    axes[0].axhline(1.0, color="gray", linestyle="--", alpha=0.4, linewidth=0.8)

    # (b) Stable tree convergence time
    bar_panel(axes[1], agg,
              y_col="t_stable_mean", yerr_col="t_stable_sem",
              ylabel="Time (s)",
              title="(b) t_tree_stable_first (mean ± SEM)")

    # (c) Received throughput
    bar_panel(axes[2], agg,
              y_col="rx_Bps_mean", yerr_col="rx_Bps_sem",
              ylabel="Received Throughput (B/s)",
              title="(c) Received Throughput")

    plt.tight_layout()
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Saved: {out_path}")

    if args.show:
        plt.show()
    plt.close()

    if args.paper is not None:
        paper_stem = Path(args.paper).with_suffix("")
        make_paper_figure(agg, paper_stem.with_suffix(".png"), show=args.show)


if __name__ == "__main__":
    main()
