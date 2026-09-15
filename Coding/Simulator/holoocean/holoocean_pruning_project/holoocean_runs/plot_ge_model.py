#!/usr/bin/env python
"""
GE channel model characterization figure (no HoloOcean required).

Generates sweeps/ge/ge_bursty_loss.png — a 3-panel paper-ready figure:
  (a) Drop-event raster: 300 consecutive messages × 3 channel models
      showing IID scatter vs GE burst clustering
  (b) CCDF of consecutive-drop run lengths
  (c) Theoretical drop probability vs link state (model diagram overlay)

Uses run_pruning_epoch.send_message() directly — identical path to simulation.

Usage (from holoocean_pruning_project/):
    python -m holoocean_runs.plot_ge_model --out sweeps/ge/ge_bursty_loss.png
"""
import argparse
import random
from pathlib import Path

import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

# ── simulation module (channel logic only, no HoloOcean) ─────────────────────
from holoocean_runs import run_pruning_epoch as _m

# ── Channel profiles ──────────────────────────────────────────────────────────
# All three are calibrated to ~10 % overall drop rate.
PROFILES = {
    "IID\n(p=0.10)": {
        "loss_model":   "iid",
        "p_drop":       0.10,
        "p_drop_good":  0.02,
        "p_drop_bad":   0.90,
        "p_gb":         0.05,
        "p_bg":         0.50,   # not used by IID
        "color":        "tab:blue",
    },
    "GE mild\n($L_{bad}$=3)": {
        "loss_model":   "ge",
        "p_drop":       0.10,   # unused for GE
        "p_drop_good":  0.02,
        "p_drop_bad":   0.90,
        "p_gb":         0.05,
        "p_bg":         1/3,    # L_bad = 3
        "color":        "tab:orange",
    },
    "GE strong\n($L_{bad}$=8)": {
        "loss_model":   "ge",
        "p_drop":       0.10,   # unused for GE
        "p_drop_good":  0.02,
        "p_drop_bad":   0.90,
        "p_gb":         0.04,
        "p_bg":         0.125,  # L_bad = 8
        "color":        "tab:red",
    },
}

N_MESSAGES = 2000   # for CCDF statistics
N_RASTER   = 300    # messages shown in raster panel
SEED       = 42


def simulate(profile: dict, n: int, seed: int) -> np.ndarray:
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
            profile["p_drop"],
            0.0,
            profile["loss_model"],
            profile["p_drop_good"],
            profile["p_drop_bad"],
            profile["p_gb"],
            profile["p_bg"],
            "GOOD",
            ge_state,
        )
        drops.append(len(_m._heap) == before)   # True = dropped
    if profile["loss_model"] == "ge":
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


def main():
    parser = argparse.ArgumentParser(description="GE model characterization figure")
    parser.add_argument("--out", type=str, default="sweeps/ge/ge_bursty_loss.png")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    # ── Simulate all profiles ───────────────────────────────────────────────
    profile_names = list(PROFILES.keys())
    results_full = {}
    results_raster = {}
    for name, prof in PROFILES.items():
        drops_full = simulate(prof, N_MESSAGES, SEED)
        results_full[name]   = drops_full
        results_raster[name] = drops_full[:N_RASTER]

    # ── Figure layout ────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(11, 4.5))
    gs = gridspec.GridSpec(1, 3, figure=fig, wspace=0.38)
    ax_raster = fig.add_subplot(gs[0])
    ax_ccdf   = fig.add_subplot(gs[1])
    ax_model  = fig.add_subplot(gs[2])

    # ── (a) Drop-event raster ────────────────────────────────────────────────
    n_profiles = len(profile_names)
    raster_img = np.zeros((n_profiles, N_RASTER))
    for row, name in enumerate(profile_names):
        raster_img[row] = results_raster[name].astype(float)

    ax_raster.imshow(raster_img, aspect="auto", cmap="binary",
                     vmin=0, vmax=1, interpolation="nearest",
                     extent=[0, N_RASTER, n_profiles - 0.5, -0.5])
    ax_raster.set_yticks(range(n_profiles))
    ax_raster.set_yticklabels(profile_names, fontsize=9)
    ax_raster.set_xlabel("Message index", fontsize=10)
    ax_raster.set_title("(a) Drop raster (black = lost)", fontsize=10)
    ax_raster.tick_params(axis="x", labelsize=8)

    # annotate observed drop rates
    for row, name in enumerate(profile_names):
        dr = results_full[name].mean()
        ax_raster.text(N_RASTER + 3, row, f"{dr*100:.1f}%",
                       va="center", ha="left", fontsize=8, color="dimgray")
    ax_raster.set_xlim(0, N_RASTER + 28)

    # ── (b) CCDF of burst run lengths ────────────────────────────────────────
    for name, prof in PROFILES.items():
        drops = results_full[name]
        runs = burst_lengths(drops)
        if runs:
            x, y = ccdf(runs)
            ax_ccdf.step(x, y, where="post", linewidth=2,
                         color=prof["color"], label=name.replace("\n", " "))
            ax_ccdf.axvline(np.mean(runs), color=prof["color"],
                            linestyle=":", alpha=0.6, linewidth=1)

    ax_ccdf.set_xlabel("Burst length $k$ (drops)", fontsize=10)
    ax_ccdf.set_ylabel("P(burst ≥ k)", fontsize=10)
    ax_ccdf.set_title("(b) CCDF of drop burst lengths", fontsize=10)
    ax_ccdf.set_yscale("log")
    ax_ccdf.set_xlim(left=1)
    ax_ccdf.set_ylim(bottom=1e-3)
    ax_ccdf.grid(True, alpha=0.3, which="both")
    ax_ccdf.legend(fontsize=8, loc="upper right")
    ax_ccdf.tick_params(labelsize=8)

    # ── (c) GE state-machine diagram + parameter table ───────────────────────
    ax_model.axis("off")

    # Draw two circles (GOOD / BAD states) with arrows
    circ_good = plt.Circle((0.25, 0.65), 0.14, fill=True,
                            facecolor="#d4e8ff", edgecolor="steelblue", linewidth=2)
    circ_bad  = plt.Circle((0.75, 0.65), 0.14, fill=True,
                            facecolor="#ffd4d4", edgecolor="firebrick", linewidth=2)
    ax_model.add_patch(circ_good)
    ax_model.add_patch(circ_bad)
    ax_model.text(0.25, 0.65, "GOOD", ha="center", va="center", fontsize=9,
                  fontweight="bold", color="steelblue")
    ax_model.text(0.75, 0.65, "BAD", ha="center", va="center", fontsize=9,
                  fontweight="bold", color="firebrick")

    # Arrows: GOOD → BAD
    ax_model.annotate("", xy=(0.61, 0.70), xytext=(0.39, 0.70),
                      arrowprops=dict(arrowstyle="->", color="steelblue", lw=1.5))
    ax_model.text(0.50, 0.76, r"$p_{gb}$", ha="center", va="bottom",
                  fontsize=9, color="steelblue")
    # BAD → GOOD
    ax_model.annotate("", xy=(0.39, 0.58), xytext=(0.61, 0.58),
                      arrowprops=dict(arrowstyle="->", color="firebrick", lw=1.5))
    ax_model.text(0.50, 0.52, r"$p_{bg} = 1/L_{bad}$", ha="center", va="top",
                  fontsize=9, color="firebrick")

    # Drop probabilities
    ax_model.text(0.25, 0.48, r"drop: $p_{good}$=0.02", ha="center", va="top",
                  fontsize=8, color="steelblue")
    ax_model.text(0.75, 0.48, r"drop: $p_{bad}$=0.90", ha="center", va="top",
                  fontsize=8, color="firebrick")

    # Parameter table
    rows = [
        ["Model", r"$p_{gb}$", r"$L_{bad}$", r"Drop%"],
        ["IID",   "—",         "—",           "10%"],
        ["GE mild",  "0.05", "3",  f"≈{100*(0.9/3)/(1/3):.0f}%*"],
        ["GE strong", "0.04", "8", f"≈{100*(0.9*0.04/(0.04+0.125)+0.02*0.125/(0.04+0.125)):.0f}%"],
    ]
    # Compute actual drop rates
    drop_rates = {name: results_full[name].mean() for name in profile_names}
    rows[2][3] = f"{drop_rates[profile_names[1]]*100:.1f}%"
    rows[3][3] = f"{drop_rates[profile_names[2]]*100:.1f}%"

    col_x = [0.03, 0.32, 0.55, 0.78]
    row_y0 = 0.38
    row_dy = 0.075
    for ri, row_data in enumerate(rows):
        y = row_y0 - ri * row_dy
        weight = "bold" if ri == 0 else "normal"
        for ci, cell in enumerate(row_data):
            ax_model.text(col_x[ci], y, cell, ha="left", va="top",
                          fontsize=8, fontweight=weight,
                          transform=ax_model.transAxes)
    ax_model.text(0.03, row_y0 - len(rows) * row_dy - 0.01,
                  "Dotted lines in (b) = mean burst length",
                  ha="left", va="top", fontsize=7, color="dimgray",
                  transform=ax_model.transAxes)

    ax_model.set_title("(c) GE channel model", fontsize=10)
    ax_model.set_xlim(0, 1)
    ax_model.set_ylim(0, 1)

    fig.suptitle("Gilbert-Elliott Channel Model: IID vs Bursty Loss\n"
                 "(N=2000 messages, seed=42; parameters as used in simulation)",
                 fontsize=10)

    plt.tight_layout(rect=[0, 0, 1, 0.93])

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=200, bbox_inches="tight")
    print(f"Saved: {out_path}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
