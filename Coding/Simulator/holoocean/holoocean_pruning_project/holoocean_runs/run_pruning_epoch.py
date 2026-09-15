#!/usr/bin/env python
"""
run_pruning_epoch.py — HoloOcean Multi-BlueROV2 + Online δ-BFS Pruning
========================================================================

Spawns N BlueROV2 agents in HoloOcean's SimpleUnderwater world, applies an
advection–diffusion current model, a CLF goal-seeking controller with
CBF-based connectivity maintenance on pruned tree edges, and runs one epoch
of the comm-feasible δ-BFS distributed pruning protocol **online** (the
neighbor graph is recomputed every tick from real sensor positions).

A side-by-side 3D Matplotlib figure shows the full communication graph
(left) and the pruned spanning tree (right) in real time.  Goal and edges
are also drawn inside the HoloOcean viewport if the API supports it.

Results are saved to:
    holoocean_pruning_project/stress_results_holoocean/<tag>/

Usage (from project root):
    cd ".../holoocean_pruning_project"
    python holoocean_runs/run_pruning_epoch.py --help
    python holoocean_runs/run_pruning_epoch.py --N 10 --R_comm 2.0 \\
        --spawn_mode chain --spacing 1.6 --min_hops 3 \\
        --p_drop 0.1 --delay_max 0.5 --sigma_v 0.05 \\
        --goal 20 0 -5 --k_goal 0.25 --v_max 0.6 --alpha_cbf 1.0
"""

# ═══════════════════════════════════════════════════════════════════════════════
# Imports
# ═══════════════════════════════════════════════════════════════════════════════
import argparse
import atexit
import collections
import csv
import heapq
import json
import math
import random
import statistics
import sys
import time as wall_time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

# Matplotlib — let the system pick the best interactive backend
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 – registers '3d' projection

# Confirm pruning package is importable (protocol is implemented inline below)
from pruning.distributed_pruning_algorithm import DistributedPruningAlgorithm
print("[init] pruning package OK:", DistributedPruningAlgorithm)

import holoocean
print("[init] holoocean OK, version:", getattr(holoocean, "__version__", "n/a"))


# ═══════════════════════════════════════════════════════════════════════════════
# Constants
# ═══════════════════════════════════════════════════════════════════════════════
PAYLOAD_BYTES = 6          # sender_id(2) + delta(2) + seq(2)
TICKS_PER_SEC = 60         # HoloOcean physics rate
GE_GOOD = "GOOD"
GE_BAD = "BAD"


# ═══════════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════════
def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="HoloOcean + online δ-BFS pruning epoch demo",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--N",         type=int,   default=3,     help="Number of BlueROV2 agents")
    p.add_argument("--root",      type=int,   default=0,     help="Root agent ID (0-indexed, ignored if root_mode=min_degree)")
    p.add_argument("--R_comm",    type=float, default=2.0,   help="Comm radius (m)")
    p.add_argument("--T_bcast",   type=float, default=1.0,   help="Broadcast period (s)")
    p.add_argument("--jitter",    type=float, default=0.2,   help="Broadcast jitter ± (s)")
    p.add_argument("--p_drop",    type=float, default=0.1,   help="Packet drop probability")
    p.add_argument("--loss_model", type=str, choices=["iid", "ge"], default="iid",
                   help="Packet loss model: iid Bernoulli drop or Gilbert-Elliott burst loss")
    p.add_argument("--p_drop_good", type=float, default=0.02,
                   help="GE model: drop probability while link is in GOOD state")
    p.add_argument("--p_drop_bad", type=float, default=0.9,
                   help="GE model: drop probability while link is in BAD state")
    p.add_argument("--p_gb", type=float, default=0.05,
                   help="GE model: transition probability P(GOOD->BAD) per send")
    p.add_argument("--p_bg", type=float, default=0.125,
                   help="GE model: transition probability P(BAD->GOOD) per send")
    p.add_argument("--L_bad", type=float, default=None,
                   help="GE convenience param: mean BAD run length in sends; sets p_bg=1/L_bad when provided")
    p.add_argument("--ge_init_state", type=str, choices=["GOOD", "BAD", "good", "bad"],
                   default="GOOD", help="GE model initial state for unseen directed links")
    p.add_argument("--ge_debug", type=lambda x: x.lower() in ("true", "1", "yes"),
                   default=False, metavar="BOOL",
                   help="Print GE diagnostic summary at end of run")
    p.add_argument("--ge_debug_link", type=str, default=None,
                   help="Print state/drop trace for this link 'i,j' (e.g. '2,5')")
    p.add_argument("--ge_debug_link_max", type=int, default=50,
                   help="Max number of sends to trace for --ge_debug_link")
    p.add_argument("--ge_min_link_samples", type=int, default=10,
                   help="Min sends per link to include in per-link stats")
    p.add_argument("--delay_max", type=float, default=0.5,   help="Max delivery delay (s)")
    # ── rate limiting / TX queue ──
    p.add_argument("--rate_cap_bps", type=float, default=float("inf"),
                   help="Per-robot TX rate cap (bits/s); inf or 0 = disabled")
    p.add_argument("--queue_max_msgs", type=int, default=100,
                   help="Max messages in per-robot TX queue (0 = unlimited)")
    p.add_argument("--queue_drop_policy", type=str, choices=["drop_tail", "drop_head"],
                   default="drop_tail",
                   help="Queue overflow policy: drop_tail (new msg) or drop_head (oldest msg)")
    # ── distance-dependent impairment (Simulation 3) ──
    p.add_argument("--dist_impair_model", type=str, choices=["none", "drop"], default="none",
                   help="Distance-dependent impairment: 'drop' => p_drop_eff=clip(p0+k*(d/R)^alpha,pmax)")
    p.add_argument("--dist_profile", type=str, choices=["none", "weak", "strong"], default="none",
                   help="Dist impairment preset; weak/strong override dist_p0/k/alpha/pmax")
    p.add_argument("--dist_p0",    type=float, default=0.05, help="Dist model: base drop prob at d=0")
    p.add_argument("--dist_k",     type=float, default=0.25, help="Dist model: drop slope coefficient")
    p.add_argument("--dist_alpha", type=float, default=1.0,  help="Dist model: distance exponent")
    p.add_argument("--dist_pmax",  type=float, default=0.95, help="Dist model: maximum drop prob")
    p.add_argument("--T_prune",   type=float, default=20.0,  help="Epoch length (s)")
    p.add_argument("--T_stable",  type=float, default=5.0,   help="Early-stop stable window (s)")
    p.add_argument("--early_stop", type=lambda x: x.lower() in ("true", "1", "yes"),
                   default=True, metavar="BOOL",
                   help="Stop early when δ stable for T_stable; set false to always run full T_prune")
    p.add_argument("--sigma_v",   type=float, default=0.05,  help="Diffusion velocity σ (m/s)")
    p.add_argument("--seed",      type=int,   default=42,    help="Random seed")
    p.add_argument("--viz_interval", type=float, default=2.0, help="Viz refresh period (wall-s); increase to reduce lag")
    p.add_argument("--no_viz", type=lambda x: x.lower() in ("true", "1", "yes"),
                   default=False, metavar="BOOL",
                   help="Disable matplotlib live plotting entirely (PNG still saved)")
    p.add_argument("--fast_viz", type=lambda x: x.lower() in ("true", "1", "yes"),
                   default=False, metavar="BOOL",
                   help="Skip plt.pause() in viz updates for faster sim (may reduce responsiveness)")
    p.add_argument("--debug_viz", type=lambda x: x.lower() in ("true", "1", "yes"),
                   default=False, metavar="BOOL",
                   help="Print verbose visualization debug messages")
    # ── auto-zoom for 3D live plot ──
    p.add_argument("--auto_zoom", type=lambda x: x.lower() in ("true", "1", "yes"),
                   default=True, metavar="BOOL",
                   help="Auto-zoom 3D axes to follow robots (disable for fixed limits)")
    p.add_argument("--zoom_margin", type=float, default=0.15,
                   help="Fractional margin on each side when auto-zooming")
    p.add_argument("--min_span", type=float, default=6.0,
                   help="Minimum axis span (m) to prevent over-zoom")
    p.add_argument("--zoom_smoothing", type=float, default=0.8,
                   help="EMA smoothing for auto-zoom limits (0=instant, 0.8=smooth)")
    # ── HUD overlay window ──
    p.add_argument("--hud", type=lambda x: x.lower() in ("true", "1", "yes"),
                   default=True, metavar="BOOL",
                   help="Show separate IROS_HUD matplotlib window for OBS overlay")
    p.add_argument("--hud_rx_window_s", type=float, default=3.0,
                   help="Rolling window (s) for throughput mean on HUD")
    p.add_argument("--hud_delay_window_s", type=float, default=10.0,
                   help="Rolling window (s) for p95 queue delay on HUD")
    p.add_argument("--hud_write_textfile", type=lambda x: x.lower() in ("true", "1", "yes"),
                   default=True, metavar="BOOL",
                   help="Write overlay.txt for OBS text source fallback")
    p.add_argument("--debug_term", type=lambda x: x.lower() in ("true", "1", "yes"),
                   default=False, metavar="BOOL",
                   help="Print termination condition evaluation debug messages")
    p.add_argument("--tag",       type=str,   default=None,  help="Output sub-folder tag")
    # ── flow / current args ──
    p.add_argument("--flow_mode", type=str, choices=["vortex", "const"],
                   default="vortex", help="Current model: vortex field or constant vector")
    p.add_argument("--const_current", type=float, nargs=3,
                   default=[0.5, 0.0, 0.0],
                   help="Constant current vector (m/s) when flow_mode=const")
    p.add_argument("--vortex_cx", type=float, default=0.0,
                   help="Vortex centre x-coord")
    p.add_argument("--vortex_cy", type=float, default=0.0,
                   help="Vortex centre y-coord")
    p.add_argument("--flow_strength_scale", type=float, default=1.0,
                   help="Multiplicative scale on vortex output")
    # ── spawn / topology args ──
    p.add_argument("--spawn_mode", type=str, choices=["cluster", "chain"],
                   default="chain", help="Spawn layout: chain (multi-hop) or cluster (random)")
    p.add_argument("--spacing",   type=float, default=1.6,
                   help="Inter-agent spacing for chain mode (must be < R_comm)")
    p.add_argument("--cluster_radius", type=float, default=3.0,
                   help="Spawn sampling half-extent for cluster mode: x,y ∈ [-R, R]")
    p.add_argument("--deg_root_max", type=int, default=4,
                   help="Max allowed degree for root node (0 = no limit)")
    p.add_argument("--min_hops",  type=int, default=3,
                   help="Required min eccentricity(root) — at least one node this many hops away")
    p.add_argument("--root_mode", type=str, choices=["fixed", "min_degree"],
                   default="min_degree",
                   help="Root selection: fixed (use --root) or min_degree (argmin degree)")
    # ── goal pursuit (A, B) ──
    p.add_argument("--goal",      type=float, nargs=3, default=[20.0, 0.0, -5.0],
                   help="Goal position [x, y, z]")
    p.add_argument("--goal_draw", type=lambda x: x.lower() in ("true", "1", "yes"),
                   default=True, metavar="BOOL",
                   help="Draw goal marker inside HoloOcean viewport")
    p.add_argument("--edge_draw", type=lambda x: x.lower() in ("true", "1", "yes"),
                   default=True, metavar="BOOL",
                   help="Draw comm/tree edges inside HoloOcean viewport")
    p.add_argument("--draw_lifetime", type=float, default=0.25,
                   help="Lifetime (s) for drawn edge primitives in HoloOcean")
    p.add_argument("--k_goal",    type=float, default=0.3,
                   help="CLF goal-seeking proportional gain (conservative: 0.3)")
    p.add_argument("--v_max",     type=float, default=0.6,
                   help="Max desired velocity magnitude (m/s) (conservative: 0.6)")
    # ── CBF connectivity + safety (C) ──
    p.add_argument("--alpha_cbf", type=float, default=1.0,
                   help="CBF class-K parameter α (connectivity)")
    p.add_argument("--d_safe",    type=float, default=1.5,
                   help="Minimum inter-agent safety distance (m) for collision-avoidance CBF")
    p.add_argument("--alpha_safe", type=float, default=2.0,
                   help="CBF class-K parameter α for safety (collision avoidance)")
    p.add_argument("--cbf_passes", type=int,  default=3,
                   help="Number of pairwise CBF projection passes per tick")
    p.add_argument("--cbf_enable", type=lambda x: x.lower() in ("true", "1", "yes"),
                   default=True, metavar="BOOL",
                   help="Enable CBF connectivity + safety maintenance")
    # ── thruster mapping (D) ──
    p.add_argument("--thruster_gain", type=float, default=10.0,
                   help="Gain multiplying desired velocity → thruster command (conservative: 10)")
    p.add_argument("--thruster_clip", type=float, default=10.0,
                   help="Per-thruster clamp magnitude (conservative: 10)")
    # ── δ-BFS tie-break policy ──
    p.add_argument("--tiebreak_mode", type=str, choices=["distance", "id", "hash"],
                   default="distance",
                   help="Tie-break mode when candidate δ == current δ: distance (prefer closer), id (prefer lower), hash (deterministic random)")
    p.add_argument("--hysteresis_margin", type=float, default=0.2,
                   help="Min distance advantage (m) to switch parent on tie (0=no margin, switch freely)")
    p.add_argument("--hysteresis_count", type=int, default=2,
                   help="Consecutive receptions required before switching parent on tie")
    return p.parse_args(argv)


# ═══════════════════════════════════════════════════════════════════════════════
# Flow-field  (advection: vortex)
# ═══════════════════════════════════════════════════════════════════════════════
def vortex_field(pos, cx: float = 0.0, cy: float = 0.0,
                 scale: float = 1.0):
    """Underwater vortex current centred at (*cx*, *cy*), strength 0.3 × *scale*."""
    x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
    if z > 0:
        return np.zeros(3)
    s = 0.3 * scale
    dx, dy = x - cx, y - cy
    r2 = dx * dx + dy * dy + 1e-5
    return np.array([-dy / r2 * s, dx / r2 * s, 0.01 * scale * np.cos(0.05 * r2)])


# ═══════════════════════════════════════════════════════════════════════════════
# Range-graph helpers
# ═══════════════════════════════════════════════════════════════════════════════
def build_range_graph(positions: list, R_comm: float
                      ) -> Tuple[Dict[int, Set[int]], int]:
    """Return (adjacency-dict, num_edges) for the R_comm-disk graph."""
    N = len(positions)
    adj: Dict[int, Set[int]] = {i: set() for i in range(N)}
    n_edges = 0
    for a in range(N):
        for b in range(a + 1, N):
            if np.linalg.norm(np.array(positions[a]) - np.array(positions[b])) <= R_comm:
                adj[a].add(b)
                adj[b].add(a)
                n_edges += 1
    return adj, n_edges


def _bfs_distances(adj: Dict[int, Set[int]], src: int) -> Dict[int, int]:
    """BFS from *src*, return {node: distance} (only reachable nodes)."""
    dist = {src: 0}
    queue = [src]
    while queue:
        u = queue.pop(0)
        for v in adj[u]:
            if v not in dist:
                dist[v] = dist[u] + 1
                queue.append(v)
    return dist


def _is_connected(adj: Dict[int, Set[int]], N: int) -> bool:
    return len(_bfs_distances(adj, 0)) == N


def _eccentricity(adj: Dict[int, Set[int]], node: int, N: int) -> int:
    """Max distance from *node* (returns 0 if graph disconnected from node)."""
    dist = _bfs_distances(adj, node)
    if len(dist) < N:
        return 0
    return max(dist.values())


# ═══════════════════════════════════════════════════════════════════════════════
# Spawn generators  (E — 3D compact cluster)
# ═══════════════════════════════════════════════════════════════════════════════
def _generate_chain_spawns(N: int, spacing: float = 1.6) -> list:
    """Place agents in a compact 3-D cluster (front/back, left/right,
    above/below) rather than a long line.

    Strategy: fill a 3-D grid of cells (each cell ≈ spacing apart) in a
    roughly cubic / spherical arrangement centred at (0, 0, -5).  A small
    random jitter is added so positions aren't perfectly on a lattice.
    The result is a tight ball-shaped swarm that still guarantees
    nearest-neighbour distance ≈ spacing  (< R_comm → connected).
    """
    # Determine grid side-length: smallest cube that fits N cells
    side = max(1, math.ceil(N ** (1.0 / 3.0)))
    # Generate candidate lattice positions centred at origin
    candidates = []
    for ix in range(side + 1):
        for iy in range(side + 1):
            for iz in range(side + 1):
                cx = (ix - side / 2.0) * spacing
                cy = (iy - side / 2.0) * spacing
                cz = (iz - side / 2.0) * spacing
                candidates.append((cx, cy, cz))
    # Sort by distance from origin so we pick the most central N cells
    candidates.sort(key=lambda p: p[0]**2 + p[1]**2 + p[2]**2)
    pts = []
    for cx, cy, cz in candidates[:N]:
        jitter = spacing * 0.15
        x = cx + random.gauss(0, jitter)
        y = cy + random.gauss(0, jitter)
        z = -5.0 + cz + random.gauss(0, jitter)
        pts.append([x, y, z])

    # Random yaw rotation + small translation so cluster isn't axis-aligned
    theta = random.uniform(0, 2 * math.pi)
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    tx = random.uniform(-2, 2)
    ty = random.uniform(-2, 2)
    rotated = []
    for x, y, z in pts:
        rx = cos_t * x - sin_t * y + tx
        ry = sin_t * x + cos_t * y + ty
        rotated.append([rx, ry, z])
    return rotated


def _generate_cluster_spawns(N: int, R_comm: float,
                             cluster_radius: float = 3.0,
                             min_sep: float = 0.6,
                             max_inner: int = 100) -> Optional[list]:
    """Try once to sample N positions in [-R, R]^2 that form a connected graph.
    Returns None on failure (caller retries)."""
    pts = []
    for _ in range(N):
        placed = False
        for _ in range(max_inner):
            p = [random.uniform(-cluster_radius, cluster_radius),
                 random.uniform(-cluster_radius, cluster_radius),
                 random.uniform(-6, -4)]
            if all(np.linalg.norm(np.array(p) - np.array(q)) >= min_sep
                   for q in pts):
                pts.append(p)
                placed = True
                break
        if not placed:
            return None
    return pts


def generate_spawn_positions(N: int, args) -> Tuple[list, int, Dict[int, Set[int]], int]:
    """Return (positions, root_id, adjacency, max_hop_from_root).

    Rejection-samples up to 200 attempts until the topology constraints
    (connected, min_hops, deg_root_max) are satisfied.
    """
    R_comm = args.R_comm
    min_hops = args.min_hops
    deg_root_max = args.deg_root_max
    root_mode = args.root_mode
    fixed_root = min(args.root, N - 1)
    max_attempts = 200

    best = None   # (positions, root, adj, ecc) — best relaxed match

    for attempt in range(1, max_attempts + 1):
        # ── sample positions ──────────────────────────────────────────────
        if args.spawn_mode == "chain":
            pts = _generate_chain_spawns(N, spacing=args.spacing)
        else:
            pts = _generate_cluster_spawns(N, R_comm,
                                           cluster_radius=args.cluster_radius)
            if pts is None:
                continue

        adj, n_edges = build_range_graph(pts, R_comm)

        # ── connectivity ──────────────────────────────────────────────────
        if not _is_connected(adj, N):
            continue

        # ── root selection ────────────────────────────────────────────────
        if root_mode == "min_degree":
            root = min(range(N), key=lambda i: len(adj[i]))
        else:
            root = fixed_root

        # ── eccentricity check ────────────────────────────────────────────
        ecc = _eccentricity(adj, root, N)

        # track best-so-far (for relaxed fallback)
        if best is None or ecc > best[3]:
            best = (pts, root, adj, ecc)

        if ecc < min_hops:
            continue

        # ── degree cap on root ────────────────────────────────────────────
        if deg_root_max > 0 and len(adj[root]) > deg_root_max:
            continue

        print(f"  Spawn accepted  (attempt {attempt}, root={root}, "
              f"deg(root)={len(adj[root])}, eccentricity={ecc}, edges={n_edges})")
        return pts, root, adj, ecc

    # ── fallback: relax constraints and use best sample ───────────────────
    pts, root, adj, ecc = best  # type: ignore[misc]
    print(f"  WARN: constraints not met in {max_attempts} attempts — using best")
    print(f"        root={root}, deg(root)={len(adj[root])}, eccentricity={ecc}")
    return pts, root, adj, ecc


# ═══════════════════════════════════════════════════════════════════════════════
# HoloOcean scenario builder
# ═══════════════════════════════════════════════════════════════════════════════
def build_config(N: int, spawn_locs: list) -> dict:
    """Return a HoloOcean scenario config for N BlueROV2 agents."""
    agents = []
    for i in range(N):
        agents.append({
            "agent_name": f"auv{i}",
            "agent_type": "BlueROV2",
            "sensors": [
                {"sensor_type": "LocationSensor"},
                {"sensor_type": "IMUSensor"},
                {"sensor_type": "DVLSensor"},
            ],
            "control_scheme": 0,
            "location": spawn_locs[i],
            "rotation": [0, 0, 0],
        })
    return {
        "name": "pruning_epoch_demo",
        "world": "SimpleUnderwater",
        "package_name": "Ocean",
        "main_agent": "auv0",
        "ticks_per_sec": TICKS_PER_SEC,
        "window_width": 1280,
        "window_height": 720,
        "current": {"vehicle_debugging": True},
        "agents": agents,
    }


# ═══════════════════════════════════════════════════════════════════════════════
# Goal-seeking CLF nominal controller  (B)
# ═══════════════════════════════════════════════════════════════════════════════
def compute_nominal_velocities(positions: Dict[int, np.ndarray],
                               goal: np.ndarray,
                               k_goal: float,
                               v_max: float) -> Dict[int, np.ndarray]:
    """v_des[i] = k_goal * (goal - pos[i]), clamped to v_max."""
    v_des: Dict[int, np.ndarray] = {}
    for i, pos in positions.items():
        v = k_goal * (goal - pos)
        norm = float(np.linalg.norm(v))
        if norm > v_max:
            v = v * (v_max / norm)
        v_des[i] = v
    return v_des


# ═══════════════════════════════════════════════════════════════════════════════
# CBF connectivity maintenance on pruned tree edges  (C)
# ═══════════════════════════════════════════════════════════════════════════════
def cbf_correct_velocities(v_des: Dict[int, np.ndarray],
                           positions: Dict[int, np.ndarray],
                           parent: Dict[int, Optional[int]],
                           root: int,
                           R_comm: float,
                           alpha_conn: float,
                           d_safe: float,
                           alpha_safe: float,
                           cbf_passes: int,
                           v_max: float,
                           neighbors: Optional[Dict[int, Set[int]]] = None
                           ) -> Tuple[Dict[int, np.ndarray], int]:
    """Project velocities to satisfy TWO CBF families:

    1. **Connectivity** (ALL current communication edges):
       h_conn = R_comm² − ‖pᵢ − pⱼ‖²  ≥ 0   (stay within comm range)
       Applied on the full neighbor graph so that the communication
       topology stays connected even before the δ-BFS spanning tree
       has fully converged.

    2. **Safety / collision avoidance** (ALL agent pairs):
       h_safe = ‖pᵢ − pⱼ‖² − d_safe²  ≥ 0   (keep minimum separation)

    Returns (corrected velocities, number of constraints that fired).
    """
    v = {i: v_des[i].copy() for i in v_des}
    total_violations = 0
    ids = sorted(v.keys())

    # Collect ALL unique communication edges for connectivity CBF.
    # This prevents the graph from becoming disconnected while the
    # δ-BFS is still propagating (agents without a parent yet would
    # otherwise have no connectivity constraint).
    conn_edges_set: Set[Tuple[int, int]] = set()
    if neighbors is not None:
        for i in ids:
            for j in neighbors.get(i, set()):
                if j > i:
                    conn_edges_set.add((i, j))
    else:
        # Fallback: use only tree edges (original behaviour)
        for i in parent:
            if i != root and parent[i] is not None:
                a, b = min(i, parent[i]), max(i, parent[i])
                conn_edges_set.add((a, b))
    conn_edges = list(conn_edges_set)

    # Collect ALL unique agent pairs for safety constraint
    all_pairs = []
    for a_idx in range(len(ids)):
        for b_idx in range(a_idx + 1, len(ids)):
            all_pairs.append((ids[a_idx], ids[b_idx]))

    d_safe_sq = d_safe * d_safe

    for _pass in range(cbf_passes):
        # ── 1) Connectivity CBF on comm edges (stay close) ────────────
        for (i, j) in conn_edges:
            e = positions[i] - positions[j]          # p_i - p_j
            e_sq = float(np.dot(e, e))
            h = R_comm * R_comm - e_sq               # positive = safe

            # ḣ + α·h ≥ 0  →  -2 eᵀ(vᵢ-vⱼ) + α·h ≥ 0
            lhs = -2.0 * float(np.dot(e, v[i] - v[j])) + alpha_conn * h

            if lhs < 0:
                total_violations += 1
                denom = 4.0 * e_sq + 1e-6
                lam = (-lhs) / denom
                correction = lam * e
                v[i] = v[i] - correction       # pull i toward j
                v[j] = v[j] + correction       # pull j toward i

        # ── 2) Safety CBF on ALL pairs (stay apart) ───────────────────
        for (i, j) in all_pairs:
            e = positions[i] - positions[j]          # p_i - p_j
            e_sq = float(np.dot(e, e))
            h = e_sq - d_safe_sq                     # positive = safe

            # ḣ + α·h ≥ 0  →  2 eᵀ(vᵢ-vⱼ) + α·h ≥ 0
            lhs = 2.0 * float(np.dot(e, v[i] - v[j])) + alpha_safe * h

            if lhs < 0:
                total_violations += 1
                denom = 4.0 * e_sq + 1e-6
                lam = (-lhs) / denom
                correction = lam * e
                v[i] = v[i] + correction       # push i away from j
                v[j] = v[j] - correction       # push j away from i

        # clamp after each pass
        for i in v:
            norm = float(np.linalg.norm(v[i]))
            if norm > v_max:
                v[i] = v[i] * (v_max / norm)

    return v, total_violations


# ═══════════════════════════════════════════════════════════════════════════════
# Thruster mapping: desired world velocity → 8 thrusters  (D)
# ═══════════════════════════════════════════════════════════════════════════════
def velocity_to_thrusters(v_world: np.ndarray,
                          gain: float = 35.0,
                          clip: float = 35.0) -> np.ndarray:
    """Map a 3-D desired world-frame velocity to an 8-element thruster command.

    BlueROV2 thruster layout (control_scheme=0), matching autonomous_control.py:
      Vertical thrusters   [0,1,2,3] — control depth (z-axis)
      Horizontal thrusters [4,5,6,7] — control forward (x) + lateral (y)
        Forward:  all four [4,5,6,7] get the same x-component
        Lateral:  [4,6] += y,  [5,7] -= y  (differential steering)
    """
    vx, vy, vz = float(v_world[0]), float(v_world[1]), float(v_world[2])
    cmd = np.zeros(8)

    # Vertical thrusters (0-3): depth / z-axis
    cmd[0:4] = gain * vz

    # Horizontal thrusters (4-7): forward / x-axis
    cmd[4:8] += gain * vx

    # Lateral / y-axis (differential on horizontal thrusters)
    lateral = gain * vy
    cmd[[4, 6]] += lateral
    cmd[[5, 7]] -= lateral

    return np.clip(cmd, -clip, clip)


# ═══════════════════════════════════════════════════════════════════════════════
# Online δ-BFS protocol  (implemented directly — NOT the offline black-box)
# ═══════════════════════════════════════════════════════════════════════════════

# ── per-epoch mutable state (reset in main) ──────────────────────────────────
_heap: List = []               # min-heap: (deliver_time, counter, rcv, snd, δ, seq)
_heap_ctr: int = 0             # unique tie-breaker for heap ordering
_tx: int = 0                   # TX attempted
_rx: int = 0                   # RX delivered
_drop: int = 0                 # dropped in channel
_tx_B: int = 0
_rx_B: int = 0
_ge_bad_sends: int = 0
_ge_total_sends: int = 0
_ge_bad_run_total_len: int = 0
_ge_bad_run_count: int = 0
_ge_bad_run_cur_len: Dict[Tuple[int, int], int] = {}
# Per-link GE tracking for robust stats
_ge_link_sends: Dict[Tuple[int, int], int] = {}
_ge_link_bad_sends: Dict[Tuple[int, int], int] = {}
_ge_link_bad_runs: Dict[Tuple[int, int], List[int]] = {}  # completed BAD run lengths
_ge_debug_trace: Dict[Tuple[int, int], List[Tuple[str, bool]]] = {}  # (state, dropped)

# Hysteresis tracking for parent switching
# Maps node_id → (candidate_parent, consecutive_win_count)
_parent_hysteresis: Dict[int, Tuple[int, int]] = {}

# ── TX queue for rate limiting ──
# Per-sender queue: sender_id → List[(enqueue_time, receiver, delta_val, seq_val)]
_tx_queues: Dict[int, List[Tuple[float, int, int, int]]] = {}
# Per-sender byte budget accumulator (fractional bytes carried over ticks)
_tx_budget: Dict[int, float] = {}
# Stats
_queue_overflow_drops: int = 0
_queue_delays: List[float] = []  # all observed queue delays (for p95)
_queue_max_occupancy: int = 0

# ── distance-impairment stats ──
_send_distances: List[float] = []    # euclidean distance at each send attempt
_dist_p_drop_effs: List[float] = []  # effective drop prob at each send attempt


def _reset_counters():
    global _heap, _heap_ctr, _tx, _rx, _drop, _tx_B, _rx_B
    global _ge_bad_sends, _ge_total_sends, _ge_bad_run_total_len, _ge_bad_run_count
    global _ge_bad_run_cur_len
    global _ge_link_sends, _ge_link_bad_sends, _ge_link_bad_runs, _ge_debug_trace
    global _parent_hysteresis
    global _tx_queues, _tx_budget, _queue_overflow_drops, _queue_delays, _queue_max_occupancy
    global _send_distances, _dist_p_drop_effs
    _heap = []
    _heap_ctr = 0
    _tx = _rx = _drop = 0
    _tx_B = _rx_B = 0
    _ge_bad_sends = 0
    _ge_total_sends = 0
    _ge_bad_run_total_len = 0
    _ge_bad_run_count = 0
    _ge_bad_run_cur_len = {}
    _ge_link_sends = {}
    _ge_link_bad_sends = {}
    _ge_link_bad_runs = {}
    _ge_debug_trace = {}
    _parent_hysteresis = {}
    _tx_queues = {}
    _tx_budget = {}
    _queue_overflow_drops = 0
    _queue_delays = []
    _queue_max_occupancy = 0
    _send_distances = []
    _dist_p_drop_effs = []


def _validate_probability(name: str, value: float):
    if value < 0.0 or value > 1.0:
        raise ValueError(f"{name} must be in [0, 1], got {value}")


def _ge_transition_state(cur_state: str, p_gb: float, p_bg: float) -> str:
    if cur_state == GE_GOOD:
        return GE_BAD if random.random() < p_gb else GE_GOOD
    return GE_GOOD if random.random() < p_bg else GE_BAD


def _finalize_ge_bad_run_metrics():
    """Close any open BAD runs at end of simulation for accurate accounting."""
    global _ge_bad_run_total_len, _ge_bad_run_count
    for link, run_len in _ge_bad_run_cur_len.items():
        if run_len > 0:
            _ge_bad_run_total_len += run_len
            _ge_bad_run_count += 1
            # Also record in per-link list
            if link not in _ge_link_bad_runs:
                _ge_link_bad_runs[link] = []
            _ge_link_bad_runs[link].append(run_len)


# ── Distance-dependent impairment helpers ─────────────────────────────────────
_DIST_PRESETS = {
    "weak":   {"p0": 0.05, "k": 0.25, "alpha": 1.0, "pmax": 0.95},
    "strong": {"p0": 0.05, "k": 0.90, "alpha": 2.0, "pmax": 0.95},
}


def _resolve_dist_params(args):
    """Return (p0, k, alpha, pmax) applying preset if active."""
    if args.dist_profile in _DIST_PRESETS:
        pr = _DIST_PRESETS[args.dist_profile]
        return pr["p0"], pr["k"], pr["alpha"], pr["pmax"]
    return args.dist_p0, args.dist_k, args.dist_alpha, args.dist_pmax


def _dist_drop_prob(d: float, R_comm: float, p0: float, k: float,
                    alpha: float, pmax: float) -> float:
    """p_drop_eff(d) = clip(p0 + k*(d/R_comm)^alpha, 0, pmax)"""
    if R_comm <= 0:
        return float(p0)
    return float(np.clip(p0 + k * (d / R_comm) ** alpha, 0.0, pmax))


# ═══════════════════════════════════════════════════════════════════════════════
# TX Queue for Rate Limiting
# ═══════════════════════════════════════════════════════════════════════════════
def _enqueue_msg(sender: int, t_now: float, receiver: int, delta_val: int, seq_val: int,
                 queue_max_msgs: int, drop_policy: str) -> bool:
    """
    Enqueue a message into sender's TX queue.
    Returns True if enqueued, False if dropped due to overflow.
    """
    global _queue_overflow_drops, _queue_max_occupancy

    if sender not in _tx_queues:
        _tx_queues[sender] = []

    queue = _tx_queues[sender]
    msg = (t_now, receiver, delta_val, seq_val)

    # Check if queue is full
    if queue_max_msgs > 0 and len(queue) >= queue_max_msgs:
        if drop_policy == "drop_tail":
            # Drop the new message
            _queue_overflow_drops += 1
            return False
        else:  # drop_head
            # Drop the oldest message (front of queue)
            queue.pop(0)
            _queue_overflow_drops += 1

    queue.append(msg)
    # Track max occupancy
    if len(queue) > _queue_max_occupancy:
        _queue_max_occupancy = len(queue)
    return True


def _service_tx_queues(t_now: float, dt: float, rate_cap_bps: float, N: int,
                       p_drop: float, delay_max: float,
                       loss_model: str, p_drop_good: float, p_drop_bad: float,
                       p_gb: float, p_bg: float, ge_init_state: str,
                       ge_link_state: Dict[Tuple[int, int], str],
                       ge_debug_link: Optional[Tuple[int, int]],
                       ge_debug_link_max: int):
    """
    Service each robot's TX queue based on rate budget.
    Dequeues messages and passes them to send_message() for loss/delay.
    """
    global _tx_budget, _queue_delays

    # Rate cap in bytes per second
    rate_cap_Bps = rate_cap_bps / 8.0
    budget_bytes_this_tick = rate_cap_Bps * dt

    for sender in range(N):
        if sender not in _tx_queues:
            continue
        queue = _tx_queues[sender]
        if not queue:
            continue

        # Accumulate budget
        if sender not in _tx_budget:
            _tx_budget[sender] = 0.0
        _tx_budget[sender] += budget_bytes_this_tick

        # Service queue while budget allows
        while queue and _tx_budget[sender] >= PAYLOAD_BYTES:
            enqueue_time, receiver, delta_val, seq_val = queue.pop(0)
            _tx_budget[sender] -= PAYLOAD_BYTES

            # Track queue delay
            q_delay = t_now - enqueue_time
            _queue_delays.append(q_delay)

            # Pass to send_message for loss/delay handling
            send_message(sender, receiver, delta_val, seq_val,
                         t_now, p_drop, delay_max,
                         loss_model, p_drop_good, p_drop_bad,
                         p_gb, p_bg, ge_init_state, ge_link_state,
                         ge_debug_link=ge_debug_link,
                         ge_debug_link_max=ge_debug_link_max)

        # Cap accumulated budget to prevent burst after idle period
        _tx_budget[sender] = min(_tx_budget[sender], PAYLOAD_BYTES * 5)


def _compute_queue_stats() -> Dict:
    """Compute queue statistics for epoch summary."""
    if not _queue_delays:
        return {
            "queue_delay_mean_s": 0.0,
            "queue_delay_p95_s": 0.0,
            "queue_overflow_drops": _queue_overflow_drops,
            "queue_max_occupancy": _queue_max_occupancy,
        }
    sorted_delays = sorted(_queue_delays)
    p95_idx = int(0.95 * len(sorted_delays))
    p95_idx = min(p95_idx, len(sorted_delays) - 1)
    return {
        "queue_delay_mean_s": round(statistics.mean(_queue_delays), 6),
        "queue_delay_p95_s": round(sorted_delays[p95_idx], 6),
        "queue_overflow_drops": _queue_overflow_drops,
        "queue_max_occupancy": _queue_max_occupancy,
    }


def _compute_ge_per_link_stats(min_samples: int = 10, min_runs: int = 2) -> Dict:
    """Compute per-link GE statistics with sample-count filtering."""
    links_total = len(_ge_link_sends)
    links_with_enough = 0
    bad_fracs: List[float] = []
    run_lengths_all: List[float] = []

    for link, sends in _ge_link_sends.items():
        if sends >= min_samples:
            links_with_enough += 1
            bad_sends = _ge_link_bad_sends.get(link, 0)
            bad_fracs.append(bad_sends / sends)
            # Collect run lengths for links with >= min_runs bad runs
            runs = _ge_link_bad_runs.get(link, [])
            if len(runs) >= min_runs:
                avg_run = sum(runs) / len(runs)
                run_lengths_all.append(avg_run)

    def stats_summary(vals: List[float]) -> Dict:
        if not vals:
            return {"mean": None, "median": None, "min": None, "max": None, "n": 0}
        return {
            "mean": round(statistics.mean(vals), 4),
            "median": round(statistics.median(vals), 4),
            "min": round(min(vals), 4),
            "max": round(max(vals), 4),
            "n": len(vals),
        }

    return {
        "ge_links_total": links_total,
        "ge_links_with_enough_samples": links_with_enough,
        "ge_min_link_samples": min_samples,
        "ge_bad_send_fraction_per_link_stats": stats_summary(bad_fracs),
        "ge_links_with_enough_runs": len(run_lengths_all),
        "ge_bad_run_length_per_link_stats": stats_summary(run_lengths_all),
    }


def _compute_ge_per_sender_stats(min_samples: int = 5) -> Dict:
    """Compute GE statistics aggregated by sender (all outgoing links combined).
    
    This provides better stats when per-link samples are sparse, since each
    sender's outgoing transmissions are pooled.
    """
    # Aggregate by sender
    sender_sends: Dict[int, int] = {}
    sender_bad_sends: Dict[int, int] = {}
    sender_bad_runs: Dict[int, List[int]] = {}
    
    for (snd, rcv), sends in _ge_link_sends.items():
        sender_sends[snd] = sender_sends.get(snd, 0) + sends
        sender_bad_sends[snd] = sender_bad_sends.get(snd, 0) + _ge_link_bad_sends.get((snd, rcv), 0)
        runs = _ge_link_bad_runs.get((snd, rcv), [])
        if snd not in sender_bad_runs:
            sender_bad_runs[snd] = []
        sender_bad_runs[snd].extend(runs)
    
    senders_total = len(sender_sends)
    senders_with_enough = 0
    bad_fracs: List[float] = []
    avg_run_lengths: List[float] = []
    
    for snd, sends in sender_sends.items():
        if sends >= min_samples:
            senders_with_enough += 1
            bad = sender_bad_sends.get(snd, 0)
            bad_fracs.append(bad / sends)
            
            runs = sender_bad_runs.get(snd, [])
            if len(runs) >= 1:
                avg_run_lengths.append(sum(runs) / len(runs))
    
    def stats_summary(vals: List[float]) -> Dict:
        if not vals:
            return {"mean": None, "median": None, "min": None, "max": None, "n": 0}
        return {
            "mean": round(statistics.mean(vals), 4),
            "median": round(statistics.median(vals), 4),
            "min": round(min(vals), 4),
            "max": round(max(vals), 4),
            "n": len(vals),
        }
    
    return {
        "ge_senders_total": senders_total,
        "ge_senders_with_enough_samples": senders_with_enough,
        "ge_bad_send_fraction_per_sender_stats": stats_summary(bad_fracs),
        "ge_bad_run_length_per_sender_stats": stats_summary(avg_run_lengths),
    }


def send_message(sender: int, receiver: int, delta_val: int,
                 seq_val: int, t_now: float,
                 p_drop: float, delay_max: float,
                 loss_model: str,
                 p_drop_good: float,
                 p_drop_bad: float,
                 p_gb: float,
                 p_bg: float,
                 ge_init_state: str,
                 ge_link_state: Dict[Tuple[int, int], str],
                 ge_debug_link: Optional[Tuple[int, int]] = None,
                 ge_debug_link_max: int = 50):
    """Enqueue one (sender→receiver) message with drop / delay model."""
    global _heap_ctr, _tx, _tx_B, _drop
    global _ge_bad_sends, _ge_total_sends, _ge_bad_run_total_len, _ge_bad_run_count
    _tx += 1
    _tx_B += PAYLOAD_BYTES

    drop_now = False
    if loss_model == "ge":
        _ge_total_sends += 1
        link = (sender, receiver)

        # Per-link send counter
        _ge_link_sends[link] = _ge_link_sends.get(link, 0) + 1

        prev_state = ge_link_state.get(link, ge_init_state)
        cur_state = _ge_transition_state(prev_state, p_gb=p_gb, p_bg=p_bg)
        ge_link_state[link] = cur_state

        if cur_state == GE_BAD:
            _ge_bad_sends += 1
            _ge_link_bad_sends[link] = _ge_link_bad_sends.get(link, 0) + 1
            _ge_bad_run_cur_len[link] = _ge_bad_run_cur_len.get(link, 0) + 1
        else:
            # Transition out of BAD: close current run
            bad_run_len = _ge_bad_run_cur_len.get(link, 0)
            if bad_run_len > 0:
                _ge_bad_run_total_len += bad_run_len
                _ge_bad_run_count += 1
                # Record per-link
                if link not in _ge_link_bad_runs:
                    _ge_link_bad_runs[link] = []
                _ge_link_bad_runs[link].append(bad_run_len)
                _ge_bad_run_cur_len[link] = 0

        p_drop_now = p_drop_bad if cur_state == GE_BAD else p_drop_good
        drop_now = p_drop_now > 0 and random.random() < p_drop_now

        # Debug trace for specific link
        if ge_debug_link is not None and link == ge_debug_link:
            if link not in _ge_debug_trace:
                _ge_debug_trace[link] = []
            if len(_ge_debug_trace[link]) < ge_debug_link_max:
                _ge_debug_trace[link].append((cur_state, drop_now))
    else:
        drop_now = p_drop > 0 and random.random() < p_drop

    if drop_now:
        _drop += 1
        return

    delay = random.uniform(0, delay_max) if delay_max > 0 else 0.0
    _heap_ctr += 1
    heapq.heappush(_heap,
                   (t_now + delay, _heap_ctr, receiver, sender, delta_val, seq_val))


def deliver_messages(t_now: float,
                     delta: Dict[int, float],
                     parent: Dict[int, Optional[int]],
                     last_seq_from: Dict[int, Dict[int, int]],
                     last_update: List[float],
                     positions: Dict[int, np.ndarray],
                     tiebreak_mode: str = "distance",
                     hysteresis_margin: float = 0.2,
                     hysteresis_count: int = 2):
    """Pop all messages due by *t_now* and apply the δ-BFS update rule.
    
    Tie-break policy (when candidate δ == current δ):
    - "distance": prefer sender closest to receiver (with margin + count hysteresis)
    - "id": prefer lower sender ID (legacy, causes star topology)
    - "hash": deterministic hash of (rcv, snd) for stable random tie-break
    
    Hysteresis prevents frequent parent flipping:
    - hysteresis_margin: new parent must be this much closer (distance mode)
    - hysteresis_count: candidate must win K consecutive times before switching
    """
    global _rx, _rx_B, _parent_hysteresis
    while _heap and _heap[0][0] <= t_now:
        _, _, rcv, snd, d_snd, sq = heapq.heappop(_heap)
        _rx += 1
        _rx_B += PAYLOAD_BYTES
        # stale / duplicate suppression
        if sq <= last_seq_from[rcv].get(snd, -1):
            continue
        last_seq_from[rcv][snd] = sq
        # δ-BFS relaxation
        candidate = d_snd + 1
        cur = delta[rcv]
        updated = False
        
        if candidate < cur:
            # Strict improvement: always accept
            delta[rcv] = candidate
            parent[rcv] = snd
            updated = True
            # Reset hysteresis since we made a δ improvement
            _parent_hysteresis.pop(rcv, None)
            
        elif candidate == cur and parent[rcv] != snd:
            # Tie: use configurable tie-break policy
            cur_parent = parent[rcv]
            should_switch = False
            
            if cur_parent is None:
                # No current parent: accept any valid sender
                should_switch = True
            elif tiebreak_mode == "distance":
                # Distance-based: prefer sender closer to receiver
                pos_rcv = positions.get(rcv)
                pos_snd = positions.get(snd)
                pos_cur = positions.get(cur_parent)
                
                if pos_rcv is not None and pos_snd is not None and pos_cur is not None:
                    dist_snd = float(np.linalg.norm(pos_rcv - pos_snd))
                    dist_cur = float(np.linalg.norm(pos_rcv - pos_cur))
                    
                    # Check if snd is better by margin
                    if dist_snd + hysteresis_margin < dist_cur:
                        # Clear advantage: check hysteresis count
                        hyst_entry = _parent_hysteresis.get(rcv, (None, 0))
                        if hyst_entry[0] == snd:
                            count = hyst_entry[1] + 1
                        else:
                            count = 1
                        _parent_hysteresis[rcv] = (snd, count)
                        
                        if count >= hysteresis_count:
                            should_switch = True
                            _parent_hysteresis.pop(rcv, None)
                    else:
                        # Not better enough: reset counter for this candidate
                        if _parent_hysteresis.get(rcv, (None, 0))[0] == snd:
                            _parent_hysteresis.pop(rcv, None)
                            
            elif tiebreak_mode == "id":
                # Legacy ID-based: prefer lower sender ID (causes star)
                if snd < cur_parent:
                    should_switch = True
                    
            elif tiebreak_mode == "hash":
                # Deterministic hash: stable pseudo-random tie-break
                hash_snd = hash((rcv, snd, "tiebreak")) % (2**32)
                hash_cur = hash((rcv, cur_parent, "tiebreak")) % (2**32)
                if hash_snd < hash_cur:
                    should_switch = True
            
            if should_switch:
                parent[rcv] = snd
                updated = True
                
        if updated:
            last_update[0] = t_now


def compute_neighbors(positions: Dict[int, np.ndarray],
                      R_comm: float) -> Dict[int, Set[int]]:
    """Pairwise distance check → neighbour sets."""
    ids = sorted(positions)
    nbrs: Dict[int, Set[int]] = {i: set() for i in ids}
    for a in range(len(ids)):
        for b in range(a + 1, len(ids)):
            i, j = ids[a], ids[b]
            if np.linalg.norm(positions[i] - positions[j]) <= R_comm:
                nbrs[i].add(j)
                nbrs[j].add(i)
    return nbrs


# ═══════════════════════════════════════════════════════════════════════════════
# Pruned-graph analysis
# ═══════════════════════════════════════════════════════════════════════════════
def analyse_pruned(N: int, root: int,
                   parent: Dict[int, Optional[int]]) -> dict:
    """BFS on pruned tree edges → connectivity / tree checks."""
    adj: Dict[int, Set[int]] = {i: set() for i in range(N)}
    edge_count = 0
    for i in range(N):
        if i != root and parent[i] is not None:
            p = parent[i]
            adj[i].add(p)
            adj[p].add(i)
            edge_count += 1
    visited = {root}
    queue = [root]
    while queue:
        n = queue.pop(0)
        for nb in adj[n]:
            if nb not in visited:
                visited.add(nb)
                queue.append(nb)
    reachable = len(visited)
    connected = reachable == N
    is_tree = connected and edge_count == N - 1
    return dict(reachable_from_root=reachable, connected=connected,
                is_tree=is_tree, pruned_edges=edge_count)


# ═══════════════════════════════════════════════════════════════════════════════
# HoloOcean in-viewport drawing helpers  (A, F)
# ═══════════════════════════════════════════════════════════════════════════════
_warned_draw = False


def _draw_goal_in_holo(env, goal: np.ndarray):
    """Draw a box + arrow at the goal location (persistent, lifetime=0)."""
    global _warned_draw
    try:
        env.draw_box(center=goal.tolist(),
                     extent=[0.5, 0.5, 0.5],
                     color=[0, 0, 0],
                     thickness=5.0,
                     lifetime=0)
        arrow_start = [float(goal[0]), float(goal[1]), float(goal[2]) + 2.0]
        env.draw_arrow(start=arrow_start,
                       end=goal.tolist(),
                       color=[0, 0, 0],
                       thickness=3.0,
                       lifetime=0)
    except (AttributeError, Exception):
        if not _warned_draw:
            print("  WARN: env.draw_box / draw_arrow unavailable — "
                  "in-viewport goal marker skipped")
            _warned_draw = True


def _draw_edges_in_holo(env, positions, neighbors, parent, root,
                        draw_lifetime: float):
    """Draw only pruned tree edges as RED lines inside HoloOcean."""
    global _warned_draw
    try:
        ids = sorted(positions)
        for i in ids:
            if i != root and parent[i] is not None:
                j = parent[i]
                env.draw_line(start=positions[i].tolist(),
                              end=positions[j].tolist(),
                              color=[255, 0, 0],
                              thickness=3.0,
                              lifetime=draw_lifetime)
    except (AttributeError, Exception):
        if not _warned_draw:
            print("  WARN: env.draw_line unavailable — "
                  "in-viewport edge drawing skipped")
            _warned_draw = True


# ═══════════════════════════════════════════════════════════════════════════════
# HUD overlay window  (separate matplotlib figure for OBS capture)
# ═══════════════════════════════════════════════════════════════════════════════

class HudState:
    """Lightweight mutable state for the IROS_HUD overlay window."""

    def __init__(self, rx_window_s: float = 3.0, delay_window_s: float = 10.0):
        # rolling throughput samples: (sim_t, cumulative bytes)
        self.rx_samples: collections.deque = collections.deque()
        self.tx_samples: collections.deque = collections.deque()
        self.rx_window_s = rx_window_s
        self.delay_window_s = delay_window_s
        # matplotlib artists (set by init_hud)
        self.fig = None
        self.ax = None
        self.txt_topo = None
        self.txt_comms = None
        self.txt_queue = None
        self.txt_channel = None
        self.txt_task = None
        self.bar_rx_ax = None
        self.bar_tx_ax = None
        self.bar_rx = None
        self.bar_tx = None
        # EMA scale for bar axes (slowly-updating max)
        self.rx_scale: float = 500.0
        self.tx_scale: float = 500.0
        self._scale_min: float = 50.0
        self._scale_ema_alpha: float = 0.92
        # overlay text file
        self.overlay_path: Optional[Path] = None
        self.last_txt_write: float = 0.0

    # ── rolling throughput helpers ─────────────────────────────────────────
    def _push(self, dq: collections.deque, sim_t: float, cum_bytes: int):
        dq.append((sim_t, cum_bytes))
        cutoff = sim_t - self.rx_window_s
        while dq and dq[0][0] < cutoff:
            dq.popleft()

    @staticmethod
    def _rolling(dq: collections.deque) -> float:
        if len(dq) < 2:
            return 0.0
        t0, b0 = dq[0]
        t1, b1 = dq[-1]
        dt = t1 - t0
        return (b1 - b0) / dt if dt > 0 else 0.0

    def push_rx_sample(self, sim_t: float, rx_bytes: int):
        self._push(self.rx_samples, sim_t, rx_bytes)

    def push_tx_sample(self, sim_t: float, tx_bytes: int):
        self._push(self.tx_samples, sim_t, tx_bytes)

    def rolling_rx_bps(self) -> float:
        return self._rolling(self.rx_samples)

    def rolling_tx_bps(self) -> float:
        return self._rolling(self.tx_samples)

    def update_scale(self, value: float, attr: str) -> float:
        """Update an EMA bar scale, returning the new scale."""
        cur = getattr(self, attr)
        target = max(self._scale_min, value * 1.3)
        new_scale = self._scale_ema_alpha * cur + (1 - self._scale_ema_alpha) * target
        # only push xlim changes when meaningful (>10% shift)
        if abs(new_scale - cur) / max(cur, 1e-9) > 0.10:
            setattr(self, attr, new_scale)
        return getattr(self, attr)


def init_hud(hud: HudState) -> HudState:
    """Create the IROS_HUD figure with text artists and two throughput bars."""
    plt.ion()
    fig = plt.figure("IROS_HUD", figsize=(5.0, 6.8))
    fig.patch.set_facecolor("#1a1a2e")

    # Main text axis (occupies upper 70%)
    ax = fig.add_axes([0.05, 0.30, 0.90, 0.68])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.set_facecolor("#1a1a2e")

    text_kw = dict(fontsize=11, fontfamily="monospace", color="#e0e0e0",
                   verticalalignment="top", transform=ax.transAxes)
    bbox_kw = dict(boxstyle="round,pad=0.35", facecolor="#16213e",
                   edgecolor="#0f3460", alpha=0.92)

    hud.txt_topo    = ax.text(0.03, 0.97, "", **text_kw, bbox=bbox_kw)
    hud.txt_comms   = ax.text(0.03, 0.72, "", **text_kw, bbox=bbox_kw)
    hud.txt_queue   = ax.text(0.03, 0.48, "", **text_kw, bbox=bbox_kw)
    hud.txt_channel = ax.text(0.03, 0.28, "", **text_kw, bbox=bbox_kw)
    hud.txt_task    = ax.text(0.03, 0.10, "", **text_kw, bbox=bbox_kw)

    # ── small bar axes (bottom 25%): RX and TX throughput ──
    bar_kw = dict(facecolor="#1a1a2e")
    ax_rx = fig.add_axes([0.12, 0.14, 0.76, 0.06], **bar_kw)
    ax_tx = fig.add_axes([0.12, 0.04, 0.76, 0.06], **bar_kw)
    for bax in (ax_rx, ax_tx):
        bax.set_yticks([])
        bax.tick_params(colors="#aaaaaa", labelsize=8)
        for spine in bax.spines.values():
            spine.set_color("#0f3460")
        bax.set_autoscale_on(False)   # prevent auto-rescale on set_width

    hud.bar_rx = ax_rx.barh([0], [0], color="#00b4d8", height=0.6)
    ax_rx.set_xlim(0, 500)
    ax_rx.set_ylim(-0.5, 0.5)
    ax_rx.set_title("RX B/s (rolling)", fontsize=8, color="#aaaaaa", pad=2)

    hud.bar_tx = ax_tx.barh([0], [0], color="#e9c46a", height=0.6)
    ax_tx.set_xlim(0, 500)
    ax_tx.set_ylim(-0.5, 0.5)
    ax_tx.set_title("TX B/s (rolling)", fontsize=8, color="#aaaaaa", pad=2)

    hud.fig = fig
    hud.ax = ax
    hud.bar_rx_ax = ax_rx
    hud.bar_tx_ax = ax_tx

    fig.canvas.manager.set_window_title("IROS_HUD")
    plt.show(block=False)
    fig.canvas.flush_events()
    return hud


def update_hud(hud: HudState, *,
               sim_t: float,
               N: int,
               neighbors: Dict[int, Set[int]],
               cur_stats: dict,
               root: int,
               tx: int, rx: int, drop: int,
               tx_B: int, rx_B: int,
               loss_model: str,
               p_drop: float,
               ge_bad_sends: int, ge_total_sends: int,
               ge_link_state: Dict,
               rate_limit_enabled: bool,
               rate_cap_bps: float,
               queue_overflow_drops: int,
               queue_max_occupancy: int,
               queue_delays: List[float],
               t_tree_first: Optional[float],
               t_goal_reached: Optional[float],
               mean_d2g: float,
               out_dir: Optional[Path] = None,
               write_textfile: bool = True):
    """Update HUD artists in-place (no ax.cla()). Cheap."""
    if hud.fig is None:
        return

    # ── push throughput samples ───────────────────────────────────────────
    hud.push_rx_sample(sim_t, rx_B)
    hud.push_tx_sample(sim_t, tx_B)
    rx_bps = hud.rolling_rx_bps()
    tx_bps = hud.rolling_tx_bps()

    # ── B1 topology block ────────────────────────────────────────────────
    e_disk = _count_undirected_edges(neighbors)
    e_tree = cur_stats["pruned_edges"]
    pruned = e_disk - e_tree
    reach  = cur_stats["reachable_from_root"]
    status = "TREE ✓" if cur_stats["is_tree"] else "BUILDING…"
    hud.txt_topo.set_text(
        f"── Topology ──\n"
        f"E_disk  = {e_disk}\n"
        f"E_tree  = {e_tree}\n"
        f"Pruned  = {pruned}\n"
        f"Reach   = {reach}/{N}\n"
        f"Status  = {status}")

    # ── B2 comms block ───────────────────────────────────────────────────
    obs_drop = drop / max(tx, 1)
    hud.txt_comms.set_text(
        f"── Comms ──\n"
        f"TX={tx}  RX={rx}  Drop={drop}\n"
        f"DropRate = {obs_drop:.1%}\n"
        f"TX_Bps({hud.rx_window_s:.0f}s)={tx_bps:.1f}  "
        f"RX_Bps={rx_bps:.1f}")

    # ── B3 queue block (text only; p95 as text, not bar) ─────────────
    if rate_limit_enabled:
        if queue_delays:
            sd = sorted(queue_delays[-500:])
            p95_ms = sd[min(int(0.95 * len(sd)), len(sd) - 1)] * 1000
        else:
            p95_ms = 0.0
        hud.txt_queue.set_text(
            f"── Queue / Rate Cap ──\n"
            f"cap = {rate_cap_bps:.0f} bps\n"
            f"q_p95 = {p95_ms:.1f} ms\n"
            f"q_ovf = {queue_overflow_drops}  q_max = {queue_max_occupancy}")
    else:
        p95_ms = 0.0
        hud.txt_queue.set_text("── Queue ──\nRate cap: disabled")

    # ── B4 channel block ─────────────────────────────────────────────────
    if loss_model == "ge":
        ge_bad_frac = ge_bad_sends / max(ge_total_sends, 1)
        links_bad = sum(1 for s in ge_link_state.values()
                        if s == GE_BAD)
        L = len(ge_link_state)
        hud.txt_channel.set_text(
            f"── Channel (GE) ──\n"
            f"BAD-send frac = {ge_bad_frac:.1%}\n"
            f"Links BAD = {links_bad}/{L}")
    else:
        hud.txt_channel.set_text(
            f"── Channel (IID) ──\n"
            f"p_drop = {p_drop}")

    # ── B5 task / convergence ────────────────────────────────────────────
    t_tree_str = f"{t_tree_first:.1f} s" if t_tree_first is not None else "—"
    t_goal_str = f"{t_goal_reached:.1f} s" if t_goal_reached is not None else "—"
    hud.txt_task.set_text(
        f"── Task ──\n"
        f"t = {sim_t:.1f} s\n"
        f"T_tree = {t_tree_str}\n"
        f"T_goal = {t_goal_str}\n"
        f"mean d→goal = {mean_d2g:.2f} m")

    # ── bar updates (width changes, scale is EMA-stable) ────────────
    hud.bar_rx[0].set_width(rx_bps)
    rx_scale = hud.update_scale(rx_bps, "rx_scale")
    hud.bar_rx_ax.set_xlim(0, rx_scale)

    hud.bar_tx[0].set_width(tx_bps)
    tx_scale = hud.update_scale(tx_bps, "tx_scale")
    hud.bar_tx_ax.set_xlim(0, tx_scale)

    # ── redraw ───────────────────────────────────────────────────────────
    try:
        hud.fig.canvas.draw_idle()
        hud.fig.canvas.flush_events()
    except Exception:
        pass

    # ── D) overlay.txt fallback ──────────────────────────────────────────
    if write_textfile and out_dir is not None:
        now = wall_time.time()
        if now - hud.last_txt_write >= 1.0:
            hud.last_txt_write = now
            tree_flag = 1 if cur_stats["is_tree"] else 0
            line = (f"t={sim_t:.1f} | TREE={tree_flag} | T_tree={t_tree_str}"
                    f" | E_disk={e_disk} E_tree={e_tree} pruned={pruned}"
                    f" reach={reach}/{N}"
                    f" | TX={tx} RX={rx} drop={drop}"
                    f" rate={obs_drop:.2%}"
                    f" | TX_Bps={tx_bps:.1f} RX_Bps={rx_bps:.1f}"
                    f" | q_p95_ms={p95_ms:.1f}"
                    f" | model={loss_model}\n")
            overlay_path = out_dir / "overlay.txt"
            try:
                overlay_path.write_text(line, encoding="utf-8")
            except Exception:
                pass


# ═══════════════════════════════════════════════════════════════════════════════
# Side-by-side 3D Matplotlib visualisation  (G — updated with goal)
# ═══════════════════════════════════════════════════════════════════════════════
def init_figure(positions: Dict[int, np.ndarray],
                goal: np.ndarray,
                margin: float = 8.0,
                auto_zoom: bool = True,
                zoom_margin: float = 0.15,
                min_span: float = 6.0):
    """Create a 1×2 figure with 3D subplots; return (fig, ax_l, ax_r, lims_state).

    *lims_state* is a mutable dict with keys ``xlim``, ``ylim``, ``zlim``
    that ``refresh_viz`` will update in-place when auto-zoom is active.
    """
    plt.ion()
    fig = plt.figure("IROS_TOPOLOGY_3D", figsize=(15, 6))
    # Set OS-level window title (constant — never overwritten)
    try:
        fig.canvas.manager.set_window_title("IROS_TOPOLOGY_3D")
    except Exception:
        pass
    ax_l = fig.add_subplot(1, 2, 1, projection="3d")
    ax_r = fig.add_subplot(1, 2, 2, projection="3d")
    coords = np.array(list(positions.values()))
    all_pts = np.vstack([coords, goal.reshape(1, 3)])
    xlim = (all_pts[:, 0].min() - margin, all_pts[:, 0].max() + margin)
    ylim = (all_pts[:, 1].min() - margin, all_pts[:, 1].max() + margin)
    zlim = (all_pts[:, 2].min() - margin, all_pts[:, 2].max() + margin)
    # If auto-zoom, compute initial tight limits from positions + goal
    if auto_zoom:
        xlim, ylim, zlim = _compute_auto_zoom_target(
            positions, zoom_margin, min_span, goal=goal)
    lims_state = {"xlim": xlim, "ylim": ylim, "zlim": zlim}
    fig.tight_layout(pad=3.0)
    plt.show(block=False)
    fig.canvas.flush_events()
    return fig, ax_l, ax_r, lims_state


def _compute_auto_zoom_target(
        positions: Dict[int, np.ndarray],
        zoom_margin: float,
        min_span: float,
        goal: Optional[np.ndarray] = None,
) -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
    """Compute target axis limits that tightly frame all robots (+ optional goal)."""
    coords = np.array(list(positions.values()))
    if goal is not None:
        coords = np.vstack([coords, goal.reshape(1, 3)])
    xmin, ymin, zmin = coords.min(axis=0)
    xmax, ymax, zmax = coords.max(axis=0)
    lims = []
    for lo, hi in ((xmin, xmax), (ymin, ymax), (zmin, zmax)):
        span = max(hi - lo, min_span)
        span *= (1.0 + 2.0 * zoom_margin)
        cx = (lo + hi) / 2.0
        lims.append((cx - span / 2.0, cx + span / 2.0))
    return tuple(lims)  # type: ignore[return-value]


def _count_undirected_edges(neighbors: Dict[int, set]) -> int:
    """Count unique undirected edges from adjacency dict."""
    return sum(1 for i in neighbors for j in neighbors[i] if j > i)


def refresh_viz(fig, ax_l, ax_r,
                positions, neighbors, delta, parent,
                root, lims_state, sim_t, stats_txt,
                goal: Optional[np.ndarray] = None,
                mean_d2g: float = 0.0,
                is_tree: bool = False,
                auto_zoom: bool = False,
                zoom_margin: float = 0.15,
                min_span: float = 6.0,
                zoom_smoothing: float = 0.8,
                debug_viz: bool = False,
                committed_parent: Optional[Dict[int, Optional[int]]] = None,
                t_tree_first: Optional[float] = None,
                N: int = 0,
                fade_duration: float = 3.0):
    """Clear + redraw both subplots.

    *auto_zoom*: when True the limits stored in *lims_state* are updated
    in-place with an EMA towards the tight robot bounding box.

    Right subplot (Deliverable A): overlays faint disk-graph edges behind
    bold red tree edges, with a metrics text box and optional post-tree
    fade of non-tree edges.
    """
    # ── auto-zoom: recompute & smooth limits ─────────────────────────────
    if auto_zoom:
        tgt_x, tgt_y, tgt_z = _compute_auto_zoom_target(
            positions, zoom_margin, min_span, goal=goal)
        alpha = zoom_smoothing
        prev_x, prev_y, prev_z = lims_state["xlim"], lims_state["ylim"], lims_state["zlim"]
        lims_state["xlim"] = (alpha * prev_x[0] + (1 - alpha) * tgt_x[0],
                              alpha * prev_x[1] + (1 - alpha) * tgt_x[1])
        lims_state["ylim"] = (alpha * prev_y[0] + (1 - alpha) * tgt_y[0],
                              alpha * prev_y[1] + (1 - alpha) * tgt_y[1])
        lims_state["zlim"] = (alpha * prev_z[0] + (1 - alpha) * tgt_z[0],
                              alpha * prev_z[1] + (1 - alpha) * tgt_z[1])
        if debug_viz:
            print(f"  auto_zoom: xlim={lims_state['xlim']} "
                  f"ylim={lims_state['ylim']} zlim={lims_state['zlim']}")

    xlim, ylim, zlim = lims_state["xlim"], lims_state["ylim"], lims_state["zlim"]

    ids = sorted(positions)
    coords = np.array([positions[i] for i in ids])
    colours = ["gold" if i == root else "dodgerblue" for i in ids]

    # ── common setup for both subplots ────────────────────────────────
    for ax in (ax_l, ax_r):
        ax.cla()
        ax.scatter(coords[:, 0], coords[:, 1], coords[:, 2],
                   c=colours, s=80, edgecolors="k", linewidths=0.6,
                   zorder=5, depthshade=False)
        # draw goal as black star
        if goal is not None:
            ax.scatter([goal[0]], [goal[1]], [goal[2]],
                       marker="*", c="black", s=250, zorder=6,
                       edgecolors="yellow", linewidths=1.0)
        for idx, i in enumerate(ids):
            d = delta[i]
            lbl = f"{i}" if d == float("inf") else f"{i} (δ{int(d)})"
            ax.text(coords[idx, 0], coords[idx, 1], coords[idx, 2] + 0.7,
                    lbl, fontsize=8, ha="center", va="bottom")
        ax.set_xlim(xlim); ax.set_ylim(ylim); ax.set_zlim(zlim)
        ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")

    # ── left subplot: full comm graph (unchanged) ──────────────────────
    n_comm = _count_undirected_edges(neighbors)
    for i in ids:
        for j in neighbors.get(i, set()):
            if j > i:
                ax_l.plot([positions[i][0], positions[j][0]],
                          [positions[i][1], positions[j][1]],
                          [positions[i][2], positions[j][2]],
                          color="gray", linewidth=0.9, alpha=0.45)
    tree_tag = "TREE ✓" if is_tree else "building…"
    ax_l.set_title(f"Comm Graph  ·  t = {sim_t:.1f} s",
                   fontsize=9, fontweight="bold")

    # ── right subplot: pruned tree with disk-edge underlay ─────────────
    # (A1) Compute disk-edge fade alpha (A3)
    disk_alpha_base = 0.13
    if t_tree_first is not None and fade_duration > 0:
        elapsed_since_tree = max(sim_t - t_tree_first, 0.0)
        fade_frac = min(elapsed_since_tree / fade_duration, 1.0)
        disk_alpha = disk_alpha_base * (1.0 - fade_frac)  # 0.13 -> 0.0
    else:
        disk_alpha = disk_alpha_base

    # Draw ALL disk-graph edges as faint underlay
    if disk_alpha > 0.005:
        for i in ids:
            for j in neighbors.get(i, set()):
                if j > i:
                    ax_r.plot([positions[i][0], positions[j][0]],
                              [positions[i][1], positions[j][1]],
                              [positions[i][2], positions[j][2]],
                              color="black", linewidth=0.6, alpha=disk_alpha,
                              zorder=1)

    # Choose which parent dict to draw (prefer committed when available)
    draw_parent = committed_parent if (committed_parent is not None and
                                        any(v is not None for v in committed_parent.values())) \
                  else parent

    # (A1) Draw pruned tree edges as bold red on top
    n_tree = 0
    edge_color = "crimson" if is_tree else "salmon"
    edge_style = "-" if is_tree else "--"
    for i in ids:
        if i != root and draw_parent.get(i) is not None:
            p = draw_parent[i]
            ax_r.plot([positions[i][0], positions[p][0]],
                      [positions[i][1], positions[p][1]],
                      [positions[i][2], positions[p][2]],
                      color=edge_color, linewidth=3.0, alpha=0.95,
                      linestyle=edge_style, zorder=3)
            n_tree += 1

    # (A2) Metrics / status overlay text box
    n_pruned = n_comm - n_tree
    if n_tree == 0 or (N > 0 and n_tree < N - 1):
        status_str = "BFS building…"
    elif is_tree:
        status_str = "TREE ✓"
    else:
        status_str = "converging…"
    overlay_text = (f"E_disk = {n_comm}\n"
                    f"E_tree = {n_tree}\n"
                    f"Pruned = {n_pruned}\n"
                    f"Status = {status_str}")
    ax_r.text2D(0.02, 0.96, overlay_text, transform=ax_r.transAxes,
                fontsize=9, fontfamily="monospace", verticalalignment="top",
                bbox=dict(boxstyle="round,pad=0.4", facecolor="white",
                          edgecolor="black", alpha=0.85),
                zorder=10)

    ax_r.set_title(f"Pruned Tree  ·  t = {sim_t:.1f} s",
                   fontsize=9, fontweight="bold")

    try:
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
    except Exception:
        pass  # non-interactive backend — PNG saved at end


# ═══════════════════════════════════════════════════════════════════════════════
# Unified cleanup
# ═══════════════════════════════════════════════════════════════════════════════
_cleanup_done: bool = False
_cleanup_refs: dict = {"env": None, "fig_topo": None, "fig_hud": None}


def cleanup(reason: str = "unknown") -> None:
    """Idempotent shutdown: close HoloOcean env + both matplotlib windows.

    Safe to call multiple times — runs at most once.  Registered with atexit
    as a safety net and called explicitly on every exit path.
    """
    global _cleanup_done
    if _cleanup_done:
        return
    _cleanup_done = True
    print(f"\n[cleanup] shutting down  (reason={reason})")

    # ── close HoloOcean environment ───────────────────────────────────────────
    env = _cleanup_refs.get("env")
    if env is not None:
        try:
            env.close()
        except Exception as _e:
            print(f"[cleanup] env.close() ignored: {_e}")
        _cleanup_refs["env"] = None

    # ── close 3D topology figure ──────────────────────────────────────────────
    fig_topo = _cleanup_refs.get("fig_topo")
    if fig_topo is not None:
        try:
            plt.close(fig_topo)
        except Exception:
            pass
        _cleanup_refs["fig_topo"] = None

    # ── close HUD figure ──────────────────────────────────────────────────────
    fig_hud = _cleanup_refs.get("fig_hud")
    if fig_hud is not None:
        try:
            plt.close(fig_hud)
        except Exception:
            pass
        _cleanup_refs["fig_hud"] = None

    # fallback — close any remaining matplotlib figures
    try:
        plt.close("all")
    except Exception:
        pass

    print("[cleanup] done.")


# ═══════════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════════
def main():
    args = parse_args()
    N     = args.N

    # Register cleanup as a safety-net for all exit paths
    atexit.register(cleanup, "atexit")

    args.ge_init_state = args.ge_init_state.upper()
    if args.L_bad is not None:
        if args.L_bad <= 0:
            raise ValueError(f"--L_bad must be > 0, got {args.L_bad}")
        args.p_bg = 1.0 / args.L_bad

    _validate_probability("--p_drop", args.p_drop)
    _validate_probability("--p_drop_good", args.p_drop_good)
    _validate_probability("--p_drop_bad", args.p_drop_bad)
    _validate_probability("--p_gb", args.p_gb)
    _validate_probability("--p_bg (effective)", args.p_bg)

    # Parse ge_debug_link into tuple if provided
    ge_debug_link_tuple: Optional[Tuple[int, int]] = None
    if args.ge_debug_link is not None:
        try:
            parts = args.ge_debug_link.split(",")
            ge_debug_link_tuple = (int(parts[0].strip()), int(parts[1].strip()))
        except (ValueError, IndexError):
            raise ValueError(f"--ge_debug_link must be 'i,j' format, got '{args.ge_debug_link}'")

    random.seed(args.seed)
    np.random.seed(args.seed)

    goal = np.asarray(args.goal, dtype=float)

    # ── banner ────────────────────────────────────────────────────────────────
    print("\n" + "=" * 70)
    print("  HoloOcean  +  Online δ-BFS Pruning  +  Goal Pursuit + CBF")
    print("=" * 70)
    print(f"  Agents         : {N}")
    print(f"  R_comm         : {args.R_comm} m")
    print(f"  Goal           : {args.goal}")
    print(f"  k_goal / v_max : {args.k_goal} / {args.v_max} m/s")
    print(f"  CBF            : {'ON' if args.cbf_enable else 'OFF'}"
          f"  α_conn={args.alpha_cbf}  α_safe={args.alpha_safe}"
          f"  d_safe={args.d_safe} m  passes={args.cbf_passes}")
    print(f"  thruster gain  : {args.thruster_gain}  clip={args.thruster_clip}")
    print(f"  T_bcast        : {args.T_bcast} s   jitter ±{args.jitter} s")
    print(f"  loss_model     : {args.loss_model}")
    print(f"  p_drop         : {args.p_drop}      delay_max : {args.delay_max} s")
    if args.rate_cap_bps != float("inf") and args.rate_cap_bps > 0:
        print(f"  rate_cap_bps   : {args.rate_cap_bps}  queue_max={args.queue_max_msgs}  policy={args.queue_drop_policy}")
    else:
        print(f"  rate_cap_bps   : disabled (inf)")
    if args.loss_model == "ge":
        l_bad_note = (f" (from L_bad={args.L_bad})" if args.L_bad is not None else "")
        print(f"  GE pG/pB       : {args.p_drop_good} / {args.p_drop_bad}")
        print(f"  GE p_gb/p_bg   : {args.p_gb} / {args.p_bg}{l_bad_note}")
        print(f"  GE init state  : {args.ge_init_state}")
    print(f"  T_prune        : {args.T_prune} s   T_stable  : {args.T_stable} s")
    print(f"  early_stop     : {'ON' if args.early_stop else 'OFF'}")
    print(f"  tiebreak_mode  : {args.tiebreak_mode}  margin={args.hysteresis_margin}m  count={args.hysteresis_count}")
    print(f"  sigma_v (diff) : {args.sigma_v} m/s")
    print(f"  flow_mode      : {args.flow_mode}")
    if args.flow_mode == "const":
        print(f"  const_current  : {args.const_current}")
    else:
        print(f"  vortex centre  : ({args.vortex_cx}, {args.vortex_cy})")
        print(f"  flow_scale     : {args.flow_strength_scale}")
    print(f"  spawn_mode     : {args.spawn_mode}")
    if args.spawn_mode == "chain":
        print(f"  spacing        : {args.spacing} m")
    else:
        print(f"  cluster_radius : {args.cluster_radius} m")
    print(f"  root_mode      : {args.root_mode}")
    print(f"  min_hops       : {args.min_hops}")
    print(f"  deg_root_max   : {args.deg_root_max}")
    print(f"  seed           : {args.seed}")
    print(f"  viz            : {'OFF' if args.no_viz else 'ON'}"
          f"  interval={args.viz_interval}s  fast={args.fast_viz}")
    print(f"  debug_term     : {args.debug_term}")
    print("=" * 70)

    # ── spawn positions + root selection ──────────────────────────────────────
    spawn_locs, root, init_adj, max_hop = generate_spawn_positions(N, args)
    print(f"\n  Root           : {root}")
    print(f"  deg(root)      : {len(init_adj[root])}")
    print(f"  Max hop (ecc)  : {max_hop}")
    print(f"  Init edges     : {sum(len(v) for v in init_adj.values()) // 2}")

    # ── protocol state ────────────────────────────────────────────────────────
    delta:   Dict[int, float]        = {i: (0.0 if i == root else float("inf")) for i in range(N)}
    parent:  Dict[int, Optional[int]]= {i: None for i in range(N)}
    seq:     Dict[int, int]          = {i: 0 for i in range(N)}
    last_seq_from: Dict[int, Dict[int, int]] = {i: {} for i in range(N)}
    next_bcast: Dict[int, float]     = {i: random.uniform(0, args.T_bcast) for i in range(N)}
    last_global_update: List[float]  = [0.0]        # mutable wrapper
    ge_link_state: Dict[Tuple[int, int], str] = {}

    _reset_counters()

    neighbors: Dict[int, Set[int]] = {i: set() for i in range(N)}
    positions: Dict[int, np.ndarray] = {}

    # ── output dir ────────────────────────────────────────────────────────────
    tag = args.tag or datetime.now().strftime("run-%H%M%S")
    project_root = Path(__file__).resolve().parent.parent
    out_dir = project_root / "stress_results_holoocean" / tag
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"\n  Output → {out_dir}\n")

    # ── CSV logger ────────────────────────────────────────────────────────────
    csv_path = out_dir / "positions.csv"
    csv_fp   = open(csv_path, "w", newline="")
    csv_w    = csv.writer(csv_fp)
    csv_w.writerow(["sim_time", "agent_id", "x", "y", "z"])

    # ── HoloOcean config ─────────────────────────────────────────────────────
    config      = build_config(N, spawn_locs=spawn_locs)
    agent_names = [f"auv{i}" for i in range(N)]
    dt          = 1.0 / TICKS_PER_SEC

    # ── visualisation (deferred to first tick) ────────────────────────────────
    fig = ax_l = ax_r = None
    lims_state = None
    last_viz_wall = 0.0

    # ── HUD overlay window ────────────────────────────────────────────────────
    hud_state: Optional[HudState] = None
    if args.hud and not args.no_viz:
        try:
            hud_state = HudState(rx_window_s=args.hud_rx_window_s,
                                 delay_window_s=args.hud_delay_window_s)
            hud_state = init_hud(hud_state)
            _cleanup_refs["fig_hud"] = hud_state.fig
            hud_state.overlay_path = out_dir / "overlay.txt" if args.hud_write_textfile else None
            print("  [HUD] IROS_HUD window created")
        except Exception as exc:
            print(f"  WARN: HUD window failed ({exc}); continuing without HUD")
            hud_state = None

    # Edge drawing uses sim-tick cadence (not wall clock) so lifetime matches
    edge_draw_every = max(1, int(args.draw_lifetime * TICKS_PER_SEC))
    edge_draw_lifetime_sim = edge_draw_every * dt  # exact sim-second lifetime
    next_edge_draw_tick = 0

    warned_currents = False
    epoch_done      = False
    sim_t           = 0.0
    prev_positions: Dict[int, np.ndarray] = {}   # for finite-diff speed
    spawn_positions: Dict[int, np.ndarray] = {}  # for displacement diagnostic
    cbf_violations_accum = 0                      # running CBF correction count

    # ── termination tracking ──────────────────────────────────────────────────
    termination_reason: Optional[str] = None
    t_tree_first: Optional[float] = None         # first time is_tree becomes true
    t_tree_stable_first: Optional[float] = None  # first time tree is stable for T_stable
    t_goal_reached: Optional[float] = None       # first time mean(d2goal) < goal_threshold
    goal_threshold = 2.0                         # threshold (m) to consider "goal reached"
    # Track when tree FIRST becomes valid (for gating red edges)
    tree_first_valid_tick: Optional[int] = None
    committed_parent: Dict[int, Optional[int]] = {i: None for i in range(N)}  # committed tree

    # seed positions from spawn config (used on tick 0 for currents)
    for i in range(N):
        positions[i] = np.asarray(config["agents"][i]["location"], dtype=float)
        prev_positions[i] = positions[i].copy()
        spawn_positions[i] = positions[i].copy()

    # compute initial neighbors from spawn positions so CBF has connectivity
    # constraints from the very first tick (otherwise neighbors is empty)
    neighbors = compute_neighbors(positions, args.R_comm)

    # precompute constant-current vector (if needed)
    const_current_vec = np.asarray(args.const_current, dtype=float)

    # ── rate limiting enabled check ───────────────────────────────────────────
    rate_limit_enabled = (args.rate_cap_bps > 0 and
                          args.rate_cap_bps != float("inf") and
                          not math.isinf(args.rate_cap_bps))

    # ── distance-dependent impairment setup ───────────────────────────────────
    dist_impair_active = (args.dist_impair_model != "none")
    _dist_p0, _dist_k, _dist_alpha, _dist_pmax = _resolve_dist_params(args)
    if dist_impair_active:
        print(f"  dist_impair_model={args.dist_impair_model}  profile={args.dist_profile}"
              f"  p0={_dist_p0}  k={_dist_k}  alpha={_dist_alpha}  pmax={_dist_pmax}")

    print("Launching HoloOcean …\n")


    try:
        with holoocean.make(scenario_cfg=config) as env:
            _cleanup_refs["env"] = env   # expose for cleanup()
            tick = 0
            t_wall_start = wall_time.time()

            # ── draw goal marker (persistent) ─────────────────────────────
            if args.goal_draw:
                _draw_goal_in_holo(env, goal)

            while not epoch_done:
                sim_t = tick * dt

                # ── advection + diffusion current (BEFORE tick) ───────────
                for i in range(N):
                    if args.flow_mode == "const":
                        v_flow = const_current_vec.copy()
                    else:
                        v_flow = vortex_field(positions[i],
                                              cx=args.vortex_cx,
                                              cy=args.vortex_cy,
                                              scale=args.flow_strength_scale)
                    v_diff = np.random.normal(0, args.sigma_v, size=3)
                    v_total = (v_flow + v_diff).tolist()
                    try:
                        env.set_ocean_currents(agent_names[i], v_total)
                    except (AttributeError, Exception):
                        if not warned_currents:
                            print("  WARN: set_ocean_currents unavailable — "
                                  "currents skipped")
                            warned_currents = True

                # ── goal-seeking + CBF + thruster command ─────────────────
                v_des = compute_nominal_velocities(
                    positions, goal, args.k_goal, args.v_max)

                if args.cbf_enable:
                    v_cmd, n_viol = cbf_correct_velocities(
                        v_des, positions, parent, root,
                        args.R_comm, args.alpha_cbf,
                        args.d_safe, args.alpha_safe,
                        args.cbf_passes, args.v_max,
                        neighbors=neighbors)
                    cbf_violations_accum += n_viol
                else:
                    v_cmd = v_des

                for i in range(N):
                    thruster_cmd = velocity_to_thrusters(
                        v_cmd[i], gain=args.thruster_gain,
                        clip=args.thruster_clip)
                    env.act(agent_names[i], thruster_cmd)

                states = env.tick()

                # ── read positions ────────────────────────────────────────
                for i in range(N):
                    prev_positions[i] = positions[i].copy()
                    try:
                        positions[i] = np.asarray(
                            states[agent_names[i]]["LocationSensor"], dtype=float)
                    except (KeyError, TypeError):
                        if tick == 0:
                            print(f"  WARN: no LocationSensor for auv{i}; "
                                  "using spawn position")
                        positions[i] = np.asarray(
                            config["agents"][i]["location"], dtype=float)

                # ── update neighbour graph from live positions ────────────
                neighbors = compute_neighbors(positions, args.R_comm)

                # ── broadcast step ────────────────────────────────────────
                for i in range(N):
                    if sim_t >= next_bcast[i]:
                        # only broadcast if this node has useful info
                        if delta[i] < float("inf"):
                            seq[i] += 1
                            for j in neighbors[i]:
                                if rate_limit_enabled:
                                    # Enqueue to TX queue for rate-limited sending
                                    _enqueue_msg(i, sim_t, j, int(delta[i]), seq[i],
                                                 args.queue_max_msgs,
                                                 args.queue_drop_policy)
                                else:
                                    # Direct send (no rate limiting)
                                    # Compute effective drop prob (with or without dist impairment)
                                    d_ij = float(np.linalg.norm(positions[i] - positions[j]))
                                    if dist_impair_active:
                                        p_drop_eff = _dist_drop_prob(
                                            d_ij, args.R_comm,
                                            _dist_p0, _dist_k, _dist_alpha, _dist_pmax)
                                    else:
                                        p_drop_eff = args.p_drop
                                    _send_distances.append(d_ij)
                                    _dist_p_drop_effs.append(p_drop_eff)
                                    send_message(i, j, int(delta[i]), seq[i],
                                                 sim_t, p_drop_eff, args.delay_max,
                                                 args.loss_model,
                                                 args.p_drop_good,
                                                 args.p_drop_bad,
                                                 args.p_gb,
                                                 args.p_bg,
                                                 args.ge_init_state,
                                                 ge_link_state,
                                                 ge_debug_link=ge_debug_link_tuple,
                                                 ge_debug_link_max=args.ge_debug_link_max)
                        jit = (random.uniform(-args.jitter, args.jitter)
                               if args.jitter > 0 else 0.0)
                        next_bcast[i] = sim_t + args.T_bcast + jit

                # ── service TX queues (rate limiting) ─────────────────────
                if rate_limit_enabled:
                    _service_tx_queues(sim_t, dt, args.rate_cap_bps, N,
                                       args.p_drop, args.delay_max,
                                       args.loss_model, args.p_drop_good,
                                       args.p_drop_bad, args.p_gb, args.p_bg,
                                       args.ge_init_state, ge_link_state,
                                       ge_debug_link_tuple,
                                       args.ge_debug_link_max)

                # ── deliver messages ──────────────────────────────────────
                deliver_messages(sim_t, delta, parent,
                                last_seq_from, last_global_update,
                                positions,
                                args.tiebreak_mode,
                                args.hysteresis_margin,
                                args.hysteresis_count)

                # ── epoch termination ─────────────────────────────────────
                if sim_t >= args.T_prune:
                    epoch_done = True
                    termination_reason = "T_prune_reached"
                    print(f"\n  ✓ Epoch ended — T_prune reached ({args.T_prune} s)")
                elif (args.early_stop and sim_t > 2.0 and
                      sim_t - last_global_update[0] >= args.T_stable):
                    # Only early-stop if ALL nodes have finite δ (fully converged)
                    all_finite = all(delta[i] < float("inf") for i in range(N))
                    if all_finite:
                        epoch_done = True
                        termination_reason = "early_stop_stable"
                        if t_tree_stable_first is None:
                            t_tree_stable_first = sim_t
                        print(f"\n  ✓ Epoch ended — stable for {args.T_stable} s "
                              f"(sim_t = {sim_t:.2f} s, all δ finite)")
                    elif args.debug_term and tick % (TICKS_PER_SEC * 5) == 0:
                        n_inf = sum(1 for i in range(N) if delta[i] >= float("inf"))
                        print(f"  [debug_term] T_stable elapsed but {n_inf}/{N} nodes "
                              f"still have δ=∞ — continuing")

                # ── init visualisation on first tick ──────────────────────
                if fig is None and tick == 0 and not args.no_viz:
                    try:
                        fig, ax_l, ax_r, lims_state = init_figure(
                            positions, goal,
                            auto_zoom=args.auto_zoom,
                            zoom_margin=args.zoom_margin,
                            min_span=args.min_span)
                        _cleanup_refs["fig_topo"] = fig  # expose for cleanup()
                        if args.debug_viz:
                            print(f"  [debug_viz] Figure initialized at tick {tick}")
                    except Exception as exc:
                        print(f"  WARN: live plot unavailable ({exc}); "
                              "PNG saved at end")

                # ── compute goal distances (reused by viz + diag) ─────────
                dists_to_goal = [float(np.linalg.norm(positions[i] - goal))
                                 for i in range(N)]
                mean_d2g = float(np.mean(dists_to_goal))
                max_d2g  = float(np.max(dists_to_goal))

                # ── track goal reached time ───────────────────────────────
                if t_goal_reached is None and mean_d2g < goal_threshold:
                    t_goal_reached = sim_t
                    if args.debug_term:
                        print(f"  [debug_term] Goal reached at t={sim_t:.2f}s "
                              f"(mean_d2g={mean_d2g:.2f} < {goal_threshold})")

                # ── check current tree status for viz labels ──────────────
                cur_stats = analyse_pruned(N, root, parent)

                # ── track first valid tree and commit edges ───────────────
                if t_tree_first is None and cur_stats["is_tree"]:
                    t_tree_first = sim_t
                    tree_first_valid_tick = tick
                    # Commit the current parent pointers as first valid tree
                    for i in range(N):
                        committed_parent[i] = parent[i]
                    if args.debug_term:
                        print(f"  [debug_term] Tree first valid at t={sim_t:.2f}s")
                elif cur_stats["is_tree"]:
                    # Update committed tree on each tick if tree is valid
                    for i in range(N):
                        committed_parent[i] = parent[i]

                # ── draw edges in HoloOcean (sim-tick cadence) ────────────
                if args.edge_draw and tick >= next_edge_draw_tick:
                    _draw_edges_in_holo(env, positions, neighbors,
                                        parent, root,
                                        edge_draw_lifetime_sim)
                    next_edge_draw_tick = tick + edge_draw_every

                # ── refresh Matplotlib visualisation (wall-clock throttled) ─
                w_now = wall_time.time()
                if not args.no_viz and w_now - last_viz_wall >= args.viz_interval:
                    stxt = f"tx={_tx} rx={_rx} drop={_drop}"
                    if args.loss_model == "ge" and _ge_total_sends > 0:
                        stxt += f" geBAD={_ge_bad_sends / _ge_total_sends:.2f}"
                    if fig is not None:
                        refresh_viz(fig, ax_l, ax_r, positions, neighbors,
                                    delta, parent, root,
                                    lims_state, sim_t, stxt,
                                    goal=goal, mean_d2g=mean_d2g,
                                    is_tree=cur_stats["is_tree"],
                                    auto_zoom=args.auto_zoom,
                                    zoom_margin=args.zoom_margin,
                                    min_span=args.min_span,
                                    zoom_smoothing=args.zoom_smoothing,
                                    debug_viz=args.debug_viz,
                                    committed_parent=committed_parent,
                                    t_tree_first=t_tree_first,
                                    N=N)
                        # Optional pause for GUI event loop (skip with --fast_viz)
                        if not args.fast_viz:
                            plt.pause(0.001)
                        if args.debug_viz:
                            print(f"  [debug_viz] t={sim_t:.2f}s refresh; "
                                  f"wall_dt={w_now - last_viz_wall:.3f}s")
                    # ── update HUD (same throttle cadence) ────────────
                    if hud_state is not None:
                        update_hud(hud_state,
                                   sim_t=sim_t, N=N,
                                   neighbors=neighbors,
                                   cur_stats=cur_stats,
                                   root=root,
                                   tx=_tx, rx=_rx, drop=_drop,
                                   tx_B=_tx_B, rx_B=_rx_B,
                                   loss_model=args.loss_model,
                                   p_drop=args.p_drop,
                                   ge_bad_sends=_ge_bad_sends,
                                   ge_total_sends=_ge_total_sends,
                                   ge_link_state=ge_link_state,
                                   rate_limit_enabled=rate_limit_enabled,
                                   rate_cap_bps=args.rate_cap_bps,
                                   queue_overflow_drops=_queue_overflow_drops,
                                   queue_max_occupancy=_queue_max_occupancy,
                                   queue_delays=_queue_delays,
                                   t_tree_first=t_tree_first,
                                   t_goal_reached=t_goal_reached,
                                   mean_d2g=mean_d2g,
                                   out_dir=out_dir,
                                   write_textfile=args.hud_write_textfile)
                    last_viz_wall = w_now

                # ── log positions every ~1 sim-second ─────────────────────
                if tick % TICKS_PER_SEC == 0:
                    for i in range(N):
                        p = positions[i]
                        csv_w.writerow([f"{sim_t:.3f}", i,
                                        f"{p[0]:.4f}", f"{p[1]:.4f}",
                                        f"{p[2]:.4f}"])

                # ── sanity + goal diagnostic every ~2 sim-seconds ─────────
                if tick > 0 and tick % (TICKS_PER_SEC * 2) == 0:
                    disps = [float(np.linalg.norm(positions[i] - spawn_positions[i]))
                             for i in range(N)]
                    mean_disp = np.mean(disps)
                    max_disp  = np.max(disps)
                    dvl_speeds = []
                    for i in range(N):
                        try:
                            dvl = np.asarray(
                                states[agent_names[i]]["DVLSensor"], dtype=float)
                            dvl_speeds.append(float(np.linalg.norm(dvl[:3])))
                        except (KeyError, TypeError, IndexError):
                            pass
                    dvl_str = (f"  DVL={np.mean(dvl_speeds):.3f} m/s"
                               if dvl_speeds else "")
                    print(f"  [diag t={sim_t:5.1f}s]  Δpos mean={mean_disp:.3f} "
                          f"max={max_disp:.3f}  "
                          f"d_goal mean={mean_d2g:.2f} max={max_d2g:.2f}"
                          f"{dvl_str}")

                # ── debug print every ~5 sim-seconds ──────────────────────
                if tick > 0 and tick % (TICKS_PER_SEC * 5) == 0:
                    d_str = [str(int(delta[i])) if delta[i] < 1e9 else "∞"
                             for i in range(N)]
                    dvl_speeds_5 = []
                    for i in range(N):
                        try:
                            dvl = np.asarray(
                                states[agent_names[i]]["DVLSensor"], dtype=float)
                            dvl_speeds_5.append(float(np.linalg.norm(dvl[:3])))
                        except (KeyError, TypeError, IndexError):
                            disp = positions[i] - prev_positions[i]
                            dvl_speeds_5.append(float(np.linalg.norm(disp)) / dt)
                    mean_spd = float(np.mean(dvl_speeds_5)) if dvl_speeds_5 else 0.0
                    print(f"  t={sim_t:6.1f}s  tx={_tx} rx={_rx} "
                          f"drop={_drop}  δ={d_str}")
                    if args.loss_model == "ge" and _ge_total_sends > 0:
                        print(
                            f"         GE bad-send fraction={_ge_bad_sends / _ge_total_sends:.3f}"
                        )
                    print(f"         mean_DVL={mean_spd:.3f} m/s  "
                          f"mean_d2goal={mean_d2g:.2f} m  "
                          f"CBF_corrections={cbf_violations_accum}")

                tick += 1

    except KeyboardInterrupt:
        print("\n  Interrupted by user.")
        termination_reason = "keyboard_interrupt"
    except Exception as _sim_exc:
        # HoloOcean crash / unexpected error — clean up immediately and exit
        print(f"\n  [error] Unexpected exception in sim loop: {_sim_exc}")
        termination_reason = "exception"
        cleanup(f"sim_exception: {type(_sim_exc).__name__}")
        csv_fp.close()
        sys.exit(1)
    finally:
        csv_fp.close()

    # Set default termination reason if not set (shouldn't happen)
    if termination_reason is None:
        termination_reason = "unknown"

    # ── drain remaining messages after env closes ─────────────────────────────
    deliver_messages(sim_t + args.delay_max + 1.0,
                     delta, parent, last_seq_from, last_global_update,
                     positions,
                     args.tiebreak_mode,
                     args.hysteresis_margin,
                     args.hysteresis_count)
    _finalize_ge_bad_run_metrics()

    # ── analyse pruned tree ───────────────────────────────────────────────────
    stats = analyse_pruned(N, root, parent)
    duration = sim_t
    drop_rate = _drop / _tx if _tx > 0 else 0.0
    ge_bad_send_fraction = (_ge_bad_sends / _ge_total_sends) if _ge_total_sends > 0 else 0.0
    ge_avg_bad_run_len = (_ge_bad_run_total_len / _ge_bad_run_count) if _ge_bad_run_count > 0 else 0.0

    # Compute expected steady-state BAD fraction: π_B = p_gb / (p_gb + p_bg)
    ge_bad_fraction_expected = 0.0
    if args.loss_model == "ge" and (args.p_gb + args.p_bg) > 0:
        ge_bad_fraction_expected = args.p_gb / (args.p_gb + args.p_bg)

    # Compute per-link GE stats
    ge_per_link_stats = _compute_ge_per_link_stats(
        min_samples=args.ge_min_link_samples,
        min_runs=2
    )

    # Compute per-sender GE stats (aggregated outgoing links - better for sparse samples)
    ge_per_sender_stats = _compute_ge_per_sender_stats(
        min_samples=max(5, args.ge_min_link_samples // 2)  # Lower threshold for aggregated
    )

    # Compute queue stats
    queue_stats = _compute_queue_stats()
    _total_offered = _tx + queue_stats["queue_overflow_drops"]
    queue_overflow_drop_rate = (queue_stats["queue_overflow_drops"] / _total_offered
                                if _total_offered > 0 else 0.0)

    # distance impairment stats
    if _send_distances:
        send_dist_mean_m  = float(np.mean(_send_distances))
        send_dist_p95_m   = float(np.percentile(_send_distances, 95))
        p_drop_eff_mean   = float(np.mean(_dist_p_drop_effs))
        p_drop_eff_p95    = float(np.percentile(_dist_p_drop_effs, 95))
    else:
        send_dist_mean_m = send_dist_p95_m = p_drop_eff_mean = p_drop_eff_p95 = 0.0

    # final distances to goal
    final_dists = [float(np.linalg.norm(positions[i] - goal)) for i in range(N)]

    summary = {
        "parameters": vars(args),
        "channel_model": {
            "loss_model": args.loss_model,
            "ge_transition_unit": "per_send",
            "iid_p_drop": args.p_drop,
            "ge_p_drop_good": args.p_drop_good,
            "ge_p_drop_bad": args.p_drop_bad,
            "ge_p_gb": args.p_gb,
            "ge_p_bg_effective": args.p_bg,
            "ge_L_bad": args.L_bad,
            "ge_init_state": args.ge_init_state,
            "ge_p_bg_precedence": "L_bad_overrides_p_bg" if args.L_bad is not None else "p_bg_used",
        },
        "queue_model": {
            "rate_limit_enabled": rate_limit_enabled,
            "rate_cap_bps": args.rate_cap_bps if rate_limit_enabled else None,
            "queue_max_msgs": args.queue_max_msgs,
            "queue_drop_policy": args.queue_drop_policy,
        },
        "dist_model": {
            "dist_impair_model": args.dist_impair_model,
            "dist_profile": args.dist_profile,
            "dist_p0": _dist_p0,
            "dist_k": _dist_k,
            "dist_alpha": _dist_alpha,
            "dist_pmax": _dist_pmax,
        },
        "topology": {
            "root":              root,
            "deg_root":          len(init_adj[root]),
            "max_hop_from_root": max_hop,
            "init_edges":        sum(len(v) for v in init_adj.values()) // 2,
        },
        "results": {
            "duration_s":         round(duration, 3),
            "termination_reason": termination_reason,
            "t_tree_first":       round(t_tree_first, 3) if t_tree_first is not None else None,
            "t_tree_stable_first": round(t_tree_stable_first, 3) if t_tree_stable_first is not None else None,
            "t_goal_reached":     round(t_goal_reached, 3) if t_goal_reached is not None else None,
            "pruned_edges":       stats["pruned_edges"],
            "connected":          stats["connected"],
            "is_tree":            stats["is_tree"],
            "reachable_from_root":stats["reachable_from_root"],
            "tx_msgs":            _tx,
            "rx_msgs":            _rx,
            "dropped_msgs":       _drop,
            "tx_bytes":           _tx_B,
            "rx_bytes":           _rx_B,
            "tx_Bps":             round(_tx_B / max(duration, 1e-3), 1),
            "rx_Bps":             round(_rx_B / max(duration, 1e-3), 1),
            "observed_drop_rate": round(drop_rate, 4),
            "ge_num_state_updates": _ge_total_sends,
            "ge_num_bad_runs": _ge_bad_run_count,
            "ge_bad_fraction_expected": round(ge_bad_fraction_expected, 4),
            "ge_bad_send_fraction": round(ge_bad_send_fraction, 4),
            "ge_avg_bad_run_length": round(ge_avg_bad_run_len, 4),
            **ge_per_link_stats,
            **ge_per_sender_stats,
            "cbf_corrections":    cbf_violations_accum,
            **queue_stats,
            "queue_overflow_drop_rate": round(queue_overflow_drop_rate, 6),
            "send_dist_mean_m":   round(send_dist_mean_m, 3),
            "send_dist_p95_m":    round(send_dist_p95_m, 3),
            "p_drop_eff_mean":    round(p_drop_eff_mean, 4),
            "p_drop_eff_p95":     round(p_drop_eff_p95, 4),
            "final_mean_d2goal":  round(float(np.mean(final_dists)), 3),
            "final_max_d2goal":   round(float(np.max(final_dists)), 3),
        },
        "node_state": {
            str(i): {
                "delta":  delta[i] if delta[i] < 1e9 else None,
                "parent": parent[i],
            }
            for i in range(N)
        },
        "timestamp": datetime.now().isoformat(),
    }

    # ── console summary ───────────────────────────────────────────────────────
    r = summary["results"]
    print("\n" + "=" * 70)
    print("  EPOCH SUMMARY")
    print("=" * 70)
    print(f"  Root (id)          : {root}   deg={len(init_adj[root])}   max_hop={max_hop}")
    print(f"  Duration           : {r['duration_s']} s")
    print(f"  Termination reason : {r['termination_reason']}")
    if r['t_tree_first'] is not None:
        print(f"  t_tree_first       : {r['t_tree_first']} s")
    if r['t_tree_stable_first'] is not None:
        print(f"  t_tree_stable_first: {r['t_tree_stable_first']} s")
    if r['t_goal_reached'] is not None:
        print(f"  t_goal_reached     : {r['t_goal_reached']} s")
    print(f"  Pruned edges       : {r['pruned_edges']}")
    print(f"  Connected          : {r['connected']}")
    print(f"  Is spanning tree   : {r['is_tree']}")
    print(f"  Reachable / N      : {r['reachable_from_root']} / {N}")
    print(f"  TX messages        : {r['tx_msgs']}")
    print(f"  RX messages        : {r['rx_msgs']}")
    print(f"  Dropped messages   : {r['dropped_msgs']}")
    print(f"  TX bandwidth       : {r['tx_Bps']} B/s")
    print(f"  RX bandwidth       : {r['rx_Bps']} B/s")
    print(f"  Observed drop rate : {r['observed_drop_rate'] * 100:.1f} %")
    if args.loss_model == "ge":
        print(f"  GE bad-send frac   : {r['ge_bad_send_fraction'] * 100:.1f} %")
        print(f"  GE avg BAD run len : {r['ge_avg_bad_run_length']:.3f} sends")
    if dist_impair_active:
        print(f"  Dist impair model  : {args.dist_impair_model}  profile={args.dist_profile}")
        print(f"  Send dist mean/p95 : {r['send_dist_mean_m']:.2f} / {r['send_dist_p95_m']:.2f} m")
        print(f"  p_drop_eff mean/p95: {r['p_drop_eff_mean']:.3f} / {r['p_drop_eff_p95']:.3f}")
    print(f"  CBF corrections    : {r['cbf_corrections']}")
    if rate_limit_enabled:
        print(f"  Queue delay mean   : {r['queue_delay_mean_s']*1000:.2f} ms")
        print(f"  Queue delay p95    : {r['queue_delay_p95_s']*1000:.2f} ms")
        print(f"  Queue overflow     : {r['queue_overflow_drops']} "
              f"({r['queue_overflow_drop_rate']*100:.2f}%)")
        print(f"  Queue max occupancy: {r['queue_max_occupancy']}")
    print(f"  Final mean d→goal  : {r['final_mean_d2goal']} m")
    print(f"  Final max  d→goal  : {r['final_max_d2goal']} m")
    print()
    for i in range(N):
        d = int(delta[i]) if delta[i] < 1e9 else "∞"
        print(f"  Node {i}:  δ = {d},  parent = {parent[i]}")
    print("=" * 70)

    # ── optional GE debug output ──────────────────────────────────────────────
    if args.ge_debug and args.loss_model == "ge":
        print("\n" + "-" * 70)
        print("  GE DIAGNOSTIC SUMMARY (--ge_debug)")
        print("-" * 70)
        exp_bad_len = 1.0 / args.p_bg if args.p_bg > 0 else float("inf")
        print(f"  Configured:  p_gb={args.p_gb:.4f}  p_bg={args.p_bg:.4f}")
        print(f"               p_drop_good={args.p_drop_good:.3f}  p_drop_bad={args.p_drop_bad:.3f}")
        print(f"  Expected π_B (steady-state BAD fraction): {ge_bad_fraction_expected:.4f}")
        print(f"  Expected mean BAD run length (1/p_bg):    {exp_bad_len:.2f} sends")
        print()
        print(f"  Observed GE stats:")
        print(f"    state_updates (total sends): {_ge_total_sends}")
        print(f"    bad_send_fraction:           {ge_bad_send_fraction:.4f}")
        print(f"    num_bad_runs:                {_ge_bad_run_count}")
        print(f"    avg_bad_run_length:          {ge_avg_bad_run_len:.3f} sends")
        print()
        print(f"  Per-link stats (min_samples={args.ge_min_link_samples}):")
        print(f"    links_total:        {ge_per_link_stats['ge_links_total']}")
        print(f"    links_with_enough:  {ge_per_link_stats['ge_links_with_enough_samples']}")
        bfrac_stats = ge_per_link_stats['ge_bad_send_fraction_per_link_stats']
        if bfrac_stats['n'] > 0:
            print(f"    bad_frac per link:  mean={bfrac_stats['mean']:.4f}  "
                  f"median={bfrac_stats['median']:.4f}  "
                  f"min={bfrac_stats['min']:.4f}  max={bfrac_stats['max']:.4f}")
        brun_stats = ge_per_link_stats['ge_bad_run_length_per_link_stats']
        if brun_stats['n'] > 0:
            print(f"    avg_bad_run/link:   mean={brun_stats['mean']:.3f}  "
                  f"median={brun_stats['median']:.3f}  "
                  f"min={brun_stats['min']:.3f}  max={brun_stats['max']:.3f}")
        print()
        print(f"  Per-sender stats (aggregated outgoing, lower threshold):")
        print(f"    senders_total:      {ge_per_sender_stats['ge_senders_total']}")
        print(f"    senders_with_enough: {ge_per_sender_stats['ge_senders_with_enough_samples']}")
        sfrac_stats = ge_per_sender_stats['ge_bad_send_fraction_per_sender_stats']
        if sfrac_stats['n'] > 0:
            print(f"    bad_frac/sender:    mean={sfrac_stats['mean']:.4f}  "
                  f"median={sfrac_stats['median']:.4f}  "
                  f"min={sfrac_stats['min']:.4f}  max={sfrac_stats['max']:.4f}")
        srun_stats = ge_per_sender_stats['ge_bad_run_length_per_sender_stats']
        if srun_stats['n'] > 0:
            print(f"    avg_bad_run/sender: mean={srun_stats['mean']:.3f}  "
                  f"median={srun_stats['median']:.3f}  "
                  f"min={srun_stats['min']:.3f}  max={srun_stats['max']:.3f}")
        print()
        early_note = "(early_stop=true)" if args.early_stop else "(full run)"
        print(f"  Run duration: {duration:.1f}s {early_note}")
        print("-" * 70)

    # ── optional per-link debug trace ─────────────────────────────────────────
    if ge_debug_link_tuple is not None and ge_debug_link_tuple in _ge_debug_trace:
        trace = _ge_debug_trace[ge_debug_link_tuple]
        print(f"\n  GE TRACE for link {ge_debug_link_tuple[0]}→{ge_debug_link_tuple[1]} "
              f"(first {len(trace)} sends):")
        state_str = "".join("B" if s == GE_BAD else "G" for s, _ in trace)
        drop_str = "".join("X" if d else "." for _, d in trace)
        print(f"    State: {state_str}")
        print(f"    Drop:  {drop_str}")
        # Print run lengths for this link
        link_runs = _ge_link_bad_runs.get(ge_debug_link_tuple, [])
        link_sends = _ge_link_sends.get(ge_debug_link_tuple, 0)
        link_bad = _ge_link_bad_sends.get(ge_debug_link_tuple, 0)
        print(f"    Total sends: {link_sends}, BAD sends: {link_bad}, "
              f"bad_frac: {link_bad/link_sends:.3f}" if link_sends > 0 else "")
        if link_runs:
            print(f"    BAD run lengths: {link_runs[:20]}{'...' if len(link_runs) > 20 else ''}")
        print()

    # ── save JSON summary ─────────────────────────────────────────────────────
    json_path = out_dir / "epoch_summary.json"
    with open(json_path, "w") as f:
        json.dump(summary, f, indent=2, default=str)
    print(f"\n  Saved  {json_path}")
    print(f"  Saved  {csv_path}")

    # ── save final figure PNG & interactive HTML ─────────────────────────────
    if fig is not None:
        # one last refresh so the PNG captures epoch-end state
        stxt = (f"tx={_tx} rx={_rx} drop={_drop}  |  "
                f"{'TREE ✓' if stats['is_tree'] else 'NOT tree'}")
        refresh_viz(fig, ax_l, ax_r, positions, neighbors,
                    delta, parent, root,
                    lims_state, sim_t, stxt,
                    goal=goal,
                    mean_d2g=float(np.mean(final_dists)),
                    is_tree=stats["is_tree"],
                    auto_zoom=args.auto_zoom,
                    zoom_margin=args.zoom_margin,
                    min_span=args.min_span,
                    zoom_smoothing=args.zoom_smoothing,
                    debug_viz=args.debug_viz,
                    committed_parent=committed_parent,
                    t_tree_first=t_tree_first,
                    N=N)
        png_path = out_dir / "final_graph.png"
        try:
            fig.savefig(str(png_path), dpi=150, bbox_inches="tight")
            print(f"  Saved  {png_path}")
        except Exception as exc:
            print(f"  WARN: PNG save failed ({exc})")

    # ── final HUD update + save HUD PNG ──────────────────────────────────────
    if hud_state is not None and hud_state.fig is not None:
        update_hud(hud_state,
                   sim_t=sim_t, N=N,
                   neighbors=neighbors,
                   cur_stats=stats,
                   root=root,
                   tx=_tx, rx=_rx, drop=_drop,
                   tx_B=_tx_B, rx_B=_rx_B,
                   loss_model=args.loss_model,
                   p_drop=args.p_drop,
                   ge_bad_sends=_ge_bad_sends,
                   ge_total_sends=_ge_total_sends,
                   ge_link_state=ge_link_state,
                   rate_limit_enabled=rate_limit_enabled,
                   rate_cap_bps=args.rate_cap_bps,
                   queue_overflow_drops=_queue_overflow_drops,
                   queue_max_occupancy=_queue_max_occupancy,
                   queue_delays=_queue_delays,
                   t_tree_first=t_tree_first,
                   t_goal_reached=t_goal_reached,
                   mean_d2g=float(np.mean(final_dists)),
                   out_dir=out_dir,
                   write_textfile=args.hud_write_textfile)
        try:
            hud_png = out_dir / "final_hud.png"
            hud_state.fig.savefig(str(hud_png), dpi=150, bbox_inches="tight",
                                   facecolor=hud_state.fig.get_facecolor())
            print(f"  Saved  {hud_png}")
        except Exception as exc:
            print(f"  WARN: HUD PNG save failed ({exc})")

    # ── interactive 3D HTML (Plotly) — zoomable / rotatable ───────────────────
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots

        ids = sorted(positions)
        coords = np.array([positions[i] for i in ids])
        colours = ["gold" if i == root else "dodgerblue" for i in ids]
        labels = []
        for i in ids:
            d = delta[i]
            lbl = f"Agent {i}" if d == float("inf") else f"Agent {i} (δ={int(d)})"
            labels.append(lbl)

        pfig = make_subplots(
            rows=1, cols=2,
            specs=[[{"type": "scatter3d"}, {"type": "scatter3d"}]],
            subplot_titles=["Comm Graph (all edges)", "Pruned Spanning Tree"],
            horizontal_spacing=0.02,
        )

        # ── helper: add nodes + goal to a subplot column ──────────────────
        def _add_nodes(col):
            pfig.add_trace(go.Scatter3d(
                x=coords[:, 0], y=coords[:, 1], z=coords[:, 2],
                mode="markers+text", text=labels, textposition="top center",
                textfont=dict(size=8),
                marker=dict(size=5, color=colours, line=dict(width=0.5, color="black")),
                name="Agents", showlegend=(col == 1),
            ), row=1, col=col)
            # Goal marker
            pfig.add_trace(go.Scatter3d(
                x=[goal[0]], y=[goal[1]], z=[goal[2]],
                mode="markers", marker=dict(size=8, color="black", symbol="diamond",
                                            line=dict(width=1, color="yellow")),
                name="Goal", showlegend=(col == 1),
            ), row=1, col=col)

        _add_nodes(1)
        _add_nodes(2)

        # ── comm edges (left) ─────────────────────────────────────────────
        for i in ids:
            for j in neighbors.get(i, set()):
                if j > i:
                    pfig.add_trace(go.Scatter3d(
                        x=[positions[i][0], positions[j][0], None],
                        y=[positions[i][1], positions[j][1], None],
                        z=[positions[i][2], positions[j][2], None],
                        mode="lines",
                        line=dict(color="gray", width=2),
                        showlegend=False,
                    ), row=1, col=1)

        # ── tree edges (right) ────────────────────────────────────────────
        for i in ids:
            if i != root and parent[i] is not None:
                p = parent[i]
                pfig.add_trace(go.Scatter3d(
                    x=[positions[i][0], positions[p][0], None],
                    y=[positions[i][1], positions[p][1], None],
                    z=[positions[i][2], positions[p][2], None],
                    mode="lines",
                    line=dict(color="crimson", width=4),
                    showlegend=False,
                ), row=1, col=2)

        pfig.update_layout(
            title=f"Epoch Result · N={N} · root={root} · "
                  f"{'TREE ✓' if stats['is_tree'] else 'NOT tree'} · "
                  f"d̄goal={float(np.mean(final_dists)):.2f} m",
            width=1500, height=700,
            margin=dict(l=0, r=0, t=40, b=0),
        )
        html_path = out_dir / "final_graph_interactive.html"
        pfig.write_html(str(html_path), include_plotlyjs="cdn")
        print(f"  Saved  {html_path}  (open in browser to zoom/rotate)")
    except ImportError:
        print("  WARN: plotly not installed — interactive HTML skipped")
        print("        Install with:  pip install plotly")
    except Exception as exc:
        print(f"  WARN: interactive HTML save failed ({exc})")

    # ── unified shutdown (closes HoloOcean env + both matplotlib windows) ─────
    print("\nDone.")
    cleanup("normal_exit")
    sys.exit(0)


# ═══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    main()
