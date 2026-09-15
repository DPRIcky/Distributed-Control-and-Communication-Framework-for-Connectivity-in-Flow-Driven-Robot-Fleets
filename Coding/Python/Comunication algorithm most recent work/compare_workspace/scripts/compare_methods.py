#!/usr/bin/env python
"""
compare_methods.py
==================
IDEAL-communication comparison between:
  1) δ-BFS spanning-tree pruning  (our method)
  2) Adjacency-consensus pruning  (baseline)

Usage
-----
    cd compare_workspace
    python scripts/compare_methods.py

Dependencies: numpy, networkx  (+ whatever the baseline already needs)
"""

from __future__ import annotations

import sys
import time as _wt
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np

# ═══════════════════════════════════════════════════════════════════════════
# Path bootstrap — compare_workspace first (our packages), then parent
# workspace (config/, simulation/, core/, controllers/, …)
# ═══════════════════════════════════════════════════════════════════════════
_SCRIPT  = Path(__file__).resolve()
_COMPARE = _SCRIPT.parent.parent                  # compare_workspace/
_ROOT    = _COMPARE.parent                         # parent workspace

# Insert in reverse order so _COMPARE ends up at index 0
for _p in (_ROOT, _COMPARE):
    _s = str(_p)
    if _s not in sys.path:
        sys.path.insert(0, _s)

import networkx as nx                                             # noqa: E402
from pruning import DistributedPruningAlgorithm                   # noqa: E402
from consensus_baseline import run_adjacency_consensus_trial      # noqa: E402


# ═══════════════════════════════════════════════════════════════════════════
# Experiment knobs
# ═══════════════════════════════════════════════════════════════════════════
N_VALUES:    List[int]              = [10, 20]
TRIALS:      int                    = 10
COMM_RADIUS: float                  = 3.0
WORKSPACE:   Tuple[float, float]    = (10.0, 10.0)
BASE_SEED:   int                    = 1000

# δ-BFS knobs
T_PRUNE:  float = 20.0      # max epoch duration  (s)
DT_BFS:   float = 0.1       # sim time-step       (s)
T_STABLE: float = 5.0       # early-stop window   (s)

# Baseline knobs
DT_BL: float = 0.05         # integrator time-step (s)


# ═══════════════════════════════════════════════════════════════════════════
# Helpers: replicate the baseline's initial disk graph for δ-BFS
# ═══════════════════════════════════════════════════════════════════════════

def _baseline_positions(N: int, seed: int) -> np.ndarray:
    """Reproduce the *exact* positions that ``HybridUnderwaterSimulation``
    generates for a given *seed*.

    RNG consumption order (from ``simulation/simulation_engine.py``):
      1. goal_x = rng.uniform(…)
      2. goal_y = rng.uniform(…)
      3. per robot:  rng.integers(0, 2**32-1)   [per-robot seed]
                     rng.normal(0, 0.4, size=2)  [position offset]
    """
    rng = np.random.default_rng(seed)
    ws  = np.asarray(WORKSPACE, dtype=float)

    # burn the same draws the sim engine uses for the goal position
    rng.uniform(ws[0] * 0.5, ws[0] * 0.95)
    rng.uniform(ws[1] * 0.2, ws[1] * 0.8)

    cluster_center = np.array([ws[0] * 0.25, ws[1] * 0.5])
    pos = np.empty((N, 2))
    for i in range(N):
        rng.integers(0, 2**32 - 1)                       # per-robot seed
        pos[i] = np.clip(
            cluster_center + rng.normal(0.0, 0.4, size=2),
            0.0,
            ws,
        )
    return pos


def _disk_graph(pos: np.ndarray, r: float) -> nx.Graph:
    """Unit-disk graph from *pos* with radius *r*.

    If the graph happens to be disconnected (extremely rare with the
    clustered positions + large radius used here), the closest inter-
    component edges are added to guarantee connectivity.
    """
    N = len(pos)
    G = nx.Graph()
    G.add_nodes_from(range(N))
    for i in range(N):
        for j in range(i + 1, N):
            if np.linalg.norm(pos[i] - pos[j]) <= r:
                G.add_edge(i, j)

    # Bridge disconnected components (safety net)
    if not nx.is_connected(G):
        comps = list(nx.connected_components(G))
        for k in range(len(comps) - 1):
            best = min(
                ((a, b) for a in comps[k] for b in comps[k + 1]),
                key=lambda pair: np.linalg.norm(pos[pair[0]] - pos[pair[1]]),
            )
            G.add_edge(*best)
    return G


# ═══════════════════════════════════════════════════════════════════════════
# Per-trial runners (normalise metric dicts)
# ═══════════════════════════════════════════════════════════════════════════

def _trial_bfs(G: nx.Graph, N: int, seed: int) -> Dict[str, Any]:
    """Run one δ-BFS epoch and return normalised metrics."""
    alg = DistributedPruningAlgorithm(
        graph=G,
        root=0,
        p_drop=0.0,
        delay_max=0.0,
        t_broadcast=1.0,
        jitter=0.0,
        seed=seed,
    )
    _, dur, comm = alg.run_epoch(
        t_prune=T_PRUNE,
        dt=DT_BFS,
        t_stable=T_STABLE,
        verbose=False,
    )
    s = alg.get_statistics()
    return {
        "success":            s["connected"] and s["final_edges"] == N - 1,
        "convergence_time_s": dur,
        "tx_msgs":            comm["messages_sent_total"],
        "rx_msgs":            comm["messages_delivered_total"],
        "rx_payload_Bps":     comm["rx_bytes_per_second"],
        "final_edges":        s["final_edges"],
    }


def _trial_baseline(N: int, seed: int, max_steps: int) -> Dict[str, Any]:
    """Run one adjacency-consensus trial and return normalised metrics."""
    m = run_adjacency_consensus_trial(
        num_robots=N,
        communication_radius=COMM_RADIUS,
        max_steps=max_steps,
        dt=DT_BL,
        seed=seed,
        verbose=False,
        workspace_size=WORKSPACE,
    )
    connected = m["final_lambda2"] > 1e-9

    # Convergence time = time of last prune, or full sim_time if none
    conv_t = (
        m["pruning_events"][-1]["time"]
        if m["pruning_events"]
        else m["sim_time"]
    )
    return {
        "success":            connected,  # baseline won't reach N-1; connected is success here
        "convergence_time_s": conv_t,
        "tx_msgs":            m["tx_messages"],
        "rx_msgs":            m["rx_messages"],
        "rx_payload_Bps":     m["bytes_per_second"],  # tx_bytes/sim_time
        "final_edges":        m["final_edges"],
    }


# ═══════════════════════════════════════════════════════════════════════════
# LaTeX table generation
# ═══════════════════════════════════════════════════════════════════════════

def _latex_table(R: Dict[Tuple[int, str], List[Dict[str, Any]]]) -> str:
    """Return a complete LaTeX table string from the collected results."""
    L: List[str] = [
        r"\begin{table}[htbp]",
        r"  \centering",
        r"  \caption{IDEAL-communication comparison: "
        r"$\delta$-BFS vs.\ Adjacency Consensus}",
        r"  \label{tab:ideal-comparison}",
        r"  \begin{tabular}{ll rrrr}",
        r"    \toprule",
        r"    $N$ & Method & Success (\%)"
        r" & Conv.\ time (s) & Payload (B/s) & Final edges \\",
        r"    \midrule",
    ]

    for idx, N in enumerate(N_VALUES):
        for label, mk in [
            (r"$\delta$-BFS", "bfs"),
            (r"Adj-Consensus", "bl"),
        ]:
            T  = R[(N, mk)]
            sr = 100.0 * sum(t["success"] for t in T) / len(T)
            ct = np.array([t["convergence_time_s"] for t in T])
            bp = np.array([t["rx_payload_Bps"]     for t in T])
            fe = np.array([t["final_edges"]         for t in T], dtype=float)

            L.append(
                f"    {N} & {label} & {sr:.0f}"
                f" & ${ct.mean():.2f} \\pm {ct.std():.2f}$"
                f" & ${bp.mean():.0f} \\pm {bp.std():.0f}$"
                f" & {fe.mean():.1f} \\\\"
            )
        if idx < len(N_VALUES) - 1:
            L.append(r"    \midrule")

    L += [r"    \bottomrule", r"  \end{tabular}", r"\end{table}"]
    return "\n".join(L)


# ═══════════════════════════════════════════════════════════════════════════
# Plain-text summary table
# ═══════════════════════════════════════════════════════════════════════════

def _print_summary(R: Dict[Tuple[int, str], List[Dict[str, Any]]]) -> None:
    """Print a human-readable summary table to stdout."""
    print("\n" + "=" * 80)
    print("  Summary  (mean ± std)")
    print("=" * 80)
    hdr = (
        f"{'N':>3}  {'Method':>14}  {'Succ%':>5}"
        f"  {'Conv (s)':>16}  {'Payload (B/s)':>20}  {'Edges':>6}"
    )
    print(hdr)
    print("-" * 80)

    for N in N_VALUES:
        for label, mk in [("delta-BFS", "bfs"), ("Adj-Consensus", "bl")]:
            T  = R[(N, mk)]
            sr = 100.0 * sum(t["success"] for t in T) / len(T)
            ct = np.array([t["convergence_time_s"] for t in T])
            bp = np.array([t["rx_payload_Bps"]     for t in T])
            fe = np.array([t["final_edges"]         for t in T], dtype=float)
            print(
                f"{N:>3}  {label:>14}  {sr:>5.0f}"
                f"  {ct.mean():>7.2f} ± {ct.std():<6.2f}"
                f"  {bp.mean():>9.0f} ± {bp.std():<8.0f}"
                f"  {fe.mean():>6.1f}"
            )

    print("=" * 80)


# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def main() -> None:
    wall_t0 = _wt.perf_counter()

    R: Dict[Tuple[int, str], List[Dict[str, Any]]] = {}

    for N in N_VALUES:
        # Baseline needs enough steps to prune a dense graph:
        #   ~N*250 steps ≥ (initial_edges − N+1) prune cycles × ~20 steps/cycle
        max_steps_bl = max(3000, N * 250)
        seeds = [BASE_SEED + i for i in range(TRIALS)]

        for method_label, mk in [("delta-BFS", "bfs"), ("Adj-Consensus", "bl")]:
            R[(N, mk)] = []

            for ti, sd in enumerate(seeds):
                tag = (
                    f"N={N:>2}  {method_label:>14s}"
                    f"  trial {ti + 1:>2}/{TRIALS}  seed={sd}"
                )
                t1 = _wt.perf_counter()

                if mk == "bfs":
                    pos = _baseline_positions(N, sd)
                    G   = _disk_graph(pos, COMM_RADIUS)
                    m   = _trial_bfs(G, N, sd)
                else:
                    m = _trial_baseline(N, sd, max_steps_bl)

                elapsed = _wt.perf_counter() - t1
                status  = "OK" if m["success"] else "FAIL"
                print(
                    f"  {tag}  {status:>4s}  {elapsed:>5.1f}s"
                    f"  edges={m['final_edges']}"
                )
                R[(N, mk)].append(m)

    wall_total = _wt.perf_counter() - wall_t0

    # ── Plain-text summary ────────────────────────────────────────────
    _print_summary(R)
    print(f"  Total wall-clock time: {wall_total:.1f} s\n")

    # ── LaTeX table ───────────────────────────────────────────────────
    latex = _latex_table(R)
    print(latex)
    print()

    # ── Save LaTeX to file ────────────────────────────────────────────
    out_dir = Path(__file__).resolve().parent.parent / "results"
    out_dir.mkdir(exist_ok=True)
    tex_path = out_dir / "ideal_comparison.tex"
    tex_path.write_text(latex, encoding="utf-8")
    print(f"  LaTeX table saved to: {tex_path}\n")


if __name__ == "__main__":
    main()
