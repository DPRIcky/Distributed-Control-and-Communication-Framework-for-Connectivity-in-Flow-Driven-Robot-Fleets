#!/usr/bin/env python
"""
GE channel figures for paper publication (no HoloOcean required).

Generates two figures from channel simulation:
  figures/ge_channel_paper.png    -- single-column CCDF (paper main text)
  figures/ge_channel_appendix.png -- 3-panel overview (appendix / supplement)

Usage (from holoocean_pruning_project/):
    python -m holoocean_runs.plot_ge_paper
    python -m holoocean_runs.plot_ge_paper --show
    python -m holoocean_runs.plot_ge_paper --paper_only
    python -m holoocean_runs.plot_ge_paper --appendix_only
    python -m holoocean_runs.plot_ge_paper --out_dir figures
"""
import argparse
import random
from pathlib import Path

import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np

# channel logic only -- no HoloOcean simulator
from holoocean_runs import run_pruning_epoch as _m

# ── Channel profiles ──────────────────────────────────────────────────────────
PROFILES = [
    {
        "label": r"IID ($p$=0.10)",
        "short": "IID",
        "loss_model": "iid",
        "p_drop":      0.10,
        "p_drop_good": 0.02,
        "p_drop_bad":  0.90,
        "p_gb":        0.05,
        "p_bg":        0.50,   # not used by IID path
        "color":       "tab:blue",
        "ls":          "-",
    },
    {
        "label": r"GE mild ($L_\mathrm{bad}$=3)",
        "short": "GE mild",
        "loss_model": "ge",
        "p_drop":      0.10,
        "p_drop_good": 0.02,
        "p_drop_bad":  0.90,
        "p_gb":        0.05,
        "p_bg":        1 / 3,
        "color":       "tab:orange",
        "ls":          "--",
    },
    {
        "label": r"GE strong ($L_\mathrm{bad}$=8)",
        "short": "GE strong",
        "loss_model": "ge",
        "p_drop":      0.10,
        "p_drop_good": 0.02,
        "p_drop_bad":  0.90,
        "p_gb":        0.04,
        "p_bg":        0.125,
        "color":       "tab:red",
        "ls":          "-",
    },
]

N_MESSAGES = 3000
N_RASTER   = 300
SEED       = 42


# ── Simulation helpers ────────────────────────────────────────────────────────

def simulate(prof: dict, n: int, seed: int) -> np.ndarray:
    """Return bool array: True = dropped, False = delivered."""
    random.seed(seed)
    np.random.seed(seed)
    _m._reset_counters()
    ge_state: dict = {}
    drops = []
    for t in range(n):
        before = len(_m._heap)
        _m.send_message(
            0, 1, 0, t, float(t),
            prof["p_drop"], 0.0,
            prof["loss_model"],
            prof["p_drop_good"], prof["p_drop_bad"],
            prof["p_gb"], prof["p_bg"],
            "GOOD", ge_state,
        )
        drops.append(len(_m._heap) == before)
    if prof["loss_model"] == "ge":
        _m._finalize_ge_bad_run_metrics()
    return np.array(drops, dtype=bool)


def burst_lengths(drops: np.ndarray) -> list:
    runs, cur = [], 0
    for d in drops:
        if d:
            cur += 1
        elif cur > 0:
            runs.append(cur)
            cur = 0
    if cur > 0:
        runs.append(cur)
    return runs


def ccdf(values: list) -> tuple:
    if not values:
        return np.array([0, 1]), np.array([1.0, 0.0])
    x = np.arange(1, max(values) + 2)
    total = len(values)
    y = np.array([sum(v >= k for v in values) / total for k in x])
    return x, y


def _draw_ccdf(ax, sim_data, font_label, font_tick, font_legend, lw,
               include_title=False,
               mean_alpha=0.70, mean_lw=1.1,
               overlay_text=True):
    """Shared CCDF drawing logic used by both figures.

    v2 options:
      mean_alpha   -- alpha for dotted mean-burst vertical lines (default 0.70)
      mean_lw      -- linewidth for dotted mean lines (default 1.1)
      overlay_text -- show floating 'dotted = mean burst' annotation (default True)
    """
    for prof, drops in zip(PROFILES, sim_data):
        runs = burst_lengths(drops)
        if not runs:
            continue
        x, y = ccdf(runs)
        ax.step(x, y, where="post", linewidth=lw,
                color=prof["color"], linestyle=prof["ls"], label=prof["label"])
        mean_len = float(np.mean(runs))
        ax.axvline(mean_len, color=prof["color"],
                   linestyle=":", linewidth=mean_lw, alpha=mean_alpha)

    ax.set_yscale("log")
    ax.set_xlim(left=1)
    ax.set_ylim(bottom=3e-4, top=2.0)
    ax.set_xlabel("Burst length $k$ (drops)", fontsize=font_label)
    ax.set_ylabel(r"$P(\mathrm{burst} \geq k)$", fontsize=font_label)
    ax.tick_params(labelsize=font_tick)
    ax.grid(True, alpha=0.25, which="both", linestyle="--", linewidth=0.55)
    ax.legend(fontsize=font_legend, loc="upper right",
              framealpha=0.88, edgecolor="lightgray", handlelength=1.8,
              borderpad=0.6)
    if include_title:
        ax.set_title("(a) CCDF of burst lengths\n(dotted = mean burst)",
                     fontsize=font_label, pad=4)
    # Subtle note about dotted lines when no title (v1 only)
    if overlay_text and not include_title:
        ax.text(0.97, 0.46, "dotted = mean burst",
                transform=ax.transAxes, fontsize=7.5, ha="right",
                va="bottom", color="dimgray", fontstyle="italic")


# ── Figure 1: paper (single-column CCDF) ─────────────────────────────────────

def make_paper_figure(sim_data: list, out_path: Path, show: bool = False):
    """
    Single-column CCDF figure for paper main text.
    No title -- use a LaTeX caption.
    """
    FONT_LABEL  = 11
    FONT_TICK   = 10
    FONT_LEGEND = 9
    LW          = 2.2

    fig, ax = plt.subplots(figsize=(3.5, 2.85))
    _draw_ccdf(ax, sim_data,
               font_label=FONT_LABEL, font_tick=FONT_TICK,
               font_legend=FONT_LEGEND, lw=LW,
               include_title=False)

    fig.tight_layout(pad=0.5)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"Saved paper figure: {out_path}")
    if show:
        plt.show()
    plt.close(fig)


# ── Figure 1b: paper v2 (clean CCDF, no overlay text) ────────────────────────

def make_paper_figure_v2(sim_data: list, out_path: Path, show: bool = False):
    """
    Single-column CCDF figure v2 for paper main text.

    Changes from v1:
      - 'dotted = mean burst' floating text removed (clean, unobstructed data)
      - Mean-burst dotted lines are lighter (alpha=0.40, lw=0.80)
      - constrained_layout=True prevents label clipping
      - Exports both PNG (300 dpi) and PDF (vector)
    Informational note for caption:
      Dotted vertical lines indicate the mean burst length per channel model.
    """
    FONT_LABEL  = 11
    FONT_TICK   = 10
    FONT_LEGEND = 9
    LW          = 2.2

    fig, ax = plt.subplots(figsize=(3.5, 2.85), constrained_layout=True)
    _draw_ccdf(ax, sim_data,
               font_label=FONT_LABEL, font_tick=FONT_TICK,
               font_legend=FONT_LEGEND, lw=LW,
               include_title=False,
               mean_alpha=0.40, mean_lw=0.80,
               overlay_text=False)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    for ext, dpi in [("png", 300), ("pdf", None)]:
        p = out_path.with_suffix(f".{ext}")
        kw = {"dpi": dpi} if dpi else {}
        fig.savefig(p, bbox_inches="tight", **kw)
        print(f"Saved paper v2 figure: {p}")
    if show:
        plt.show()
    plt.close(fig)


# ── Figure 1c: style-matched to rate_cap_paper_v2 ────────────────────────────

def make_stylematched_figure(sim_data: list, out_path: Path, show: bool = False):
    """
    GE burst-length CCDF styled to match rate_cap_paper_v2.png exactly.

    Style constants (from plot_rate_cap_paper_v2.py):
      FONT_LABEL=10, FONT_TICK=9, FONT_LEGEND=9
      LW=2.1, grid alpha=0.3 major-only
      Legend: framealpha=0.85, edgecolor='lightgray', handlelength=1.5, borderpad=0.5

    Design choices:
      - Legend loc='lower left' (empty region at small k, low P)
      - Mean-burst dotted lines kept at alpha=0.35, lw=0.75 (subtle reference)
      - No title, no overlay annotation text
      - Exports PNG (300 dpi) + PDF
    """
    FONT_LABEL  = 10
    FONT_TICK   = 9
    FONT_LEGEND = 9
    LW          = 2.1
    MEAN_ALPHA  = 0.35
    MEAN_LW     = 0.75

    fig, ax = plt.subplots(figsize=(3.5, 2.8), constrained_layout=True)
    for prof, drops in zip(PROFILES, sim_data):
        runs = burst_lengths(drops)
        if not runs:
            continue
        x, y = ccdf(runs)
        ax.step(x, y, where="post", linewidth=LW,
                color=prof["color"], linestyle=prof["ls"], label=prof["label"])
        ax.axvline(float(np.mean(runs)), color=prof["color"],
                   linestyle=":", linewidth=MEAN_LW, alpha=MEAN_ALPHA)

    ax.set_yscale("log")
    ax.set_xlim(left=1)
    ax.set_ylim(bottom=3e-4, top=2.0)
    ax.set_xlabel("Burst length $k$ (drops)", fontsize=FONT_LABEL)
    ax.set_ylabel(r"$P(\mathrm{burst} \geq k)$", fontsize=FONT_LABEL)
    ax.tick_params(labelsize=FONT_TICK)
    ax.grid(True, alpha=0.3, which="major")
    ax.legend(loc="lower left", fontsize=FONT_LEGEND,
              framealpha=0.85, edgecolor="lightgray",
              handlelength=1.5, borderpad=0.5)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    for ext, dpi in [("png", 300), ("pdf", None)]:
        p = out_path.with_suffix(f".{ext}")
        kw = {"dpi": dpi} if dpi else {}
        fig.savefig(p, bbox_inches="tight", **kw)
        print(f"Saved style-matched figure: {p}")
    if show:
        plt.show()
    plt.close(fig)


# ── Figure 2: appendix (3-panel, overlap-free) ───────────────────────────────

def make_appendix_figure(sim_data: list, out_path: Path, show: bool = False):
    """
    2-panel appendix/supplement figure (raster removed).
    Panels: (a) CCDF of burst lengths, (b) GE state diagram + parameter table.
    Uses constrained_layout=True to eliminate all overlap.
    """
    FONT_LABEL  = 10
    FONT_TICK   = 9
    FONT_LEGEND = 9
    LW          = 1.8

    fig = plt.figure(figsize=(7.5, 3.7), constrained_layout=True)
    gs  = gridspec.GridSpec(1, 2, figure=fig, width_ratios=[2.1, 1.7])
    ax_ccdf = fig.add_subplot(gs[0])
    ax_diag = fig.add_subplot(gs[1])

    # ── (a) CCDF ──────────────────────────────────────────────────────────────
    _draw_ccdf(ax_ccdf, sim_data,
               font_label=FONT_LABEL, font_tick=FONT_TICK,
               font_legend=FONT_LEGEND, lw=LW,
               include_title=True)

    # ── (b) GE state diagram + parameter table ───────────────────────────────
    ax_diag.axis("off")
    ax_diag.set_xlim(0, 1)
    ax_diag.set_ylim(0, 1)

    # Circles -- data coords in (0,1)×(0,1) space
    from matplotlib.patches import Circle
    CX_G, CX_B, CY   = 0.22, 0.78, 0.72
    R                 = 0.115
    ax_diag.add_patch(Circle((CX_G, CY), R, fill=True,
                             facecolor="#d4e8ff", edgecolor="steelblue",
                             linewidth=1.8, zorder=2))
    ax_diag.add_patch(Circle((CX_B, CY), R, fill=True,
                             facecolor="#ffd4d4", edgecolor="firebrick",
                             linewidth=1.8, zorder=2))
    ax_diag.text(CX_G, CY, "G", ha="center", va="center",
                 fontsize=10, fontweight="bold", color="steelblue", zorder=3)
    ax_diag.text(CX_B, CY, "B", ha="center", va="center",
                 fontsize=10, fontweight="bold", color="firebrick", zorder=3)

    # Arrows: curve above (G→B) and below (B→G) the circles
    AY_HI = CY + 0.07
    AY_LO = CY - 0.07
    ax_diag.annotate("",
                     xy=(CX_B - R, AY_HI), xytext=(CX_G + R, AY_HI),
                     arrowprops=dict(arrowstyle="->", color="steelblue", lw=1.4))
    ax_diag.text(0.50, CY + 0.15, r"$p_{gb}$",
                 ha="center", va="bottom", fontsize=9, color="steelblue")

    ax_diag.annotate("",
                     xy=(CX_G + R, AY_LO), xytext=(CX_B - R, AY_LO),
                     arrowprops=dict(arrowstyle="->", color="firebrick", lw=1.4))
    ax_diag.text(0.50, CY - 0.16, r"$p_{bg} = 1/L_\mathrm{bad}$",
                 ha="center", va="top", fontsize=8, color="firebrick")

    # Drop prob labels under circles
    ax_diag.text(CX_G, CY - R - 0.07, r"$p_\mathrm{drop}$=0.02",
                 ha="center", va="top", fontsize=7.5, color="steelblue")
    ax_diag.text(CX_B, CY - R - 0.07, r"$p_\mathrm{drop}$=0.90",
                 ha="center", va="top", fontsize=7.5, color="firebrick")

    # Parameter table (4 rows × 4 cols)
    HDR  = ["Profile", r"$p_{gb}$", r"$L_\mathrm{bad}$", "drop%"]
    ROWS = [
        ["IID",      "—",    "—", f"{sim_data[0].mean()*100:.0f}%"],
        ["GE mild",  "0.05", "3", f"{sim_data[1].mean()*100:.0f}%"],
        ["GE str.",  "0.04", "8", f"{sim_data[2].mean()*100:.0f}%"],
    ]
    COL_X = [0.02, 0.40, 0.60, 0.78]
    ty = 0.40
    for ci, h in enumerate(HDR):
        ax_diag.text(COL_X[ci], ty, h, ha="left", va="top",
                     fontsize=7.5, fontweight="bold")
    for row_data in ROWS:
        ty -= 0.09
        for ci, cell in enumerate(row_data):
            ax_diag.text(COL_X[ci], ty, cell, ha="left", va="top",
                         fontsize=7.5)

    ax_diag.set_title("(b) GE channel model", fontsize=FONT_LABEL, pad=5)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    print(f"Saved appendix figure: {out_path}")
    if show:
        plt.show()
    plt.close(fig)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="GE channel paper figures")
    parser.add_argument("--out_dir",       type=str, default="figures",
                        help="Output directory (relative to project root)")
    parser.add_argument("--paper",         type=str, default="ge_channel_paper.png",
                        help="Paper v1 figure filename")
    parser.add_argument("--paper_v2",      type=str, default=None,
                        help="Paper v2 figure stem, e.g. ge_channel_paper_v2 "
                             "(no extension; PNG+PDF generated). Set to generate v2.")
    parser.add_argument("--stylematched",  type=str, default=None,
                        help="Style-matched figure stem, e.g. ge_channel_paper_stylematched "
                             "(no extension; PNG+PDF generated).")
    parser.add_argument("--appendix",      type=str, default="ge_channel_appendix.png",
                        help="Appendix figure filename")
    parser.add_argument("--paper_only",    action="store_true",
                        help="Generate paper figure(s) only, skip appendix")
    parser.add_argument("--appendix_only", action="store_true",
                        help="Generate appendix figure only, skip paper figures")
    parser.add_argument("--show",          action="store_true",
                        help="Call plt.show() after saving")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)

    print(f"Simulating {N_MESSAGES} messages x {len(PROFILES)} profiles "
          f"(seed={SEED})...")
    sim_data = [simulate(prof, N_MESSAGES, SEED) for prof in PROFILES]
    for prof, drops in zip(PROFILES, sim_data):
        runs = burst_lengths(drops)
        mean_burst = float(np.mean(runs)) if runs else 0.0
        print(f"  {prof['short']:12s}  drop={drops.mean()*100:.1f}%  "
              f"mean_burst={mean_burst:.1f}  max_burst={max(runs) if runs else 0}")

    if not args.appendix_only:
        make_paper_figure(sim_data, out_dir / args.paper, show=args.show)
        if args.paper_v2 is not None:
            v2_stem = args.paper_v2
            # strip extension if user accidentally included one
            v2_stem = Path(v2_stem).stem
            make_paper_figure_v2(sim_data, out_dir / f"{v2_stem}.png",
                                 show=args.show)
        if args.stylematched is not None:
            sm_stem = Path(args.stylematched).stem
            make_stylematched_figure(sim_data, out_dir / f"{sm_stem}.png",
                                     show=args.show)

    if not args.paper_only:
        make_appendix_figure(sim_data, out_dir / args.appendix, show=args.show)


if __name__ == "__main__":
    main()
