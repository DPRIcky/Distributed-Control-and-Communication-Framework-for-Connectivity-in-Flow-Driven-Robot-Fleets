#!/usr/bin/env python
"""
Sweep rate_cap_bps for N=10 and N=20 across multiple seeds.
Outputs CSV files with success rate, convergence time, throughput, and queue stats.

Usage:
    python -m holoocean_runs.sweep_rate_cap --N 10 --n_seeds 10 --out_dir sweeps/rate_cap_N10
    python -m holoocean_runs.sweep_rate_cap --N 20 --n_seeds 10 --out_dir sweeps/rate_cap_N20
"""
import argparse
import csv
import json
import math
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path


# Default rate caps to sweep (bps). Includes inf (disabled) and log-spaced values.
DEFAULT_CAPS = [float("inf"), 128_000, 64_000, 32_000, 16_000, 8_000, 4_000, 2_000, 1_000, 500, 250]


def run_epoch(
    N: int,
    seed: int,
    rate_cap_bps: float,
    queue_max_msgs: int = 500,
    T_prune: float = 60.0,
    tag_prefix: str = "sweep",
    project_dir: Path = None,
) -> dict | None:
    """Run a single epoch and return the parsed epoch_summary.json."""
    cap_str = "inf" if math.isinf(rate_cap_bps) else str(int(rate_cap_bps))
    tag = f"{tag_prefix}_N{N}_cap{cap_str}_s{seed}"
    
    # Use project dir if provided, else current directory
    cwd = project_dir if project_dir else Path.cwd()
    
    cmd = [
        sys.executable, "-m", "holoocean_runs.run_pruning_epoch",
        "--N", str(N),
        "--seed", str(seed),
        "--rate_cap_bps", str(rate_cap_bps),
        "--queue_max_msgs", str(queue_max_msgs),
        "--T_prune", str(T_prune),
        "--no_viz", "true",
        "--tag", tag,
    ]
    
    print(f"  Running: N={N}, cap={cap_str}, seed={seed} ...", flush=True)
    
    try:
        # Run with inherited stdout/stderr so we can see HoloOcean output
        result = subprocess.run(cmd, cwd=cwd, timeout=300)
    except subprocess.TimeoutExpired:
        print("  -> TIMEOUT")
        return None
    except Exception as e:
        print(f"  -> ERROR: {e}")
        return None
    
    if result.returncode != 0:
        print(f"  -> FAILED (exit code {result.returncode})")
        return None
    
    # Find the output directory
    run_dir = cwd / "stress_results_holoocean" / tag
    summary_path = run_dir / "epoch_summary.json"
    
    if not summary_path.exists():
        print(f"  -> MISSING: {summary_path}")
        return None
    
    with open(summary_path, "r") as f:
        data = json.load(f)
    
    print("OK")
    return data


def extract_row(data: dict, rate_cap_bps: float, seed: int) -> dict:
    """Extract relevant fields from epoch_summary into a row dict."""
    r = data.get("results", {})
    params = data.get("parameters", {})
    
    connected = r.get("connected", False)
    is_tree = r.get("is_tree", False)
    success = 1 if (connected and is_tree) else 0
    
    return {
        "rate_cap_bps": rate_cap_bps if not math.isinf(rate_cap_bps) else "inf",
        "seed": seed,
        "success": success,
        "t_tree_first": r.get("t_tree_first"),
        "t_tree_stable_first": r.get("t_tree_stable_first"),
        "duration_s": r.get("duration_s"),
        "termination_reason": r.get("termination_reason"),
        "rx_Bps": r.get("rx_Bps"),
        "tx_Bps": r.get("tx_Bps"),
        "queue_delay_mean_s": r.get("queue_delay_mean_s"),
        "queue_delay_p95_s": r.get("queue_delay_p95_s"),
        "queue_overflow_drops": r.get("queue_overflow_drops"),
        "queue_overflow_drop_rate": r.get("queue_overflow_drop_rate"),
        "queue_max_occupancy": r.get("queue_max_occupancy"),
        "reachable_from_root": r.get("reachable_from_root"),
        "N": params.get("N"),
    }


def main():
    parser = argparse.ArgumentParser(description="Sweep rate_cap_bps for pruning experiments")
    parser.add_argument("--N", type=int, required=True, help="Number of agents (10 or 20)")
    parser.add_argument("--n_seeds", type=int, default=10, help="Number of seeds per cap")
    parser.add_argument("--seed_start", type=int, default=1, help="Starting seed")
    parser.add_argument("--caps", type=str, default=None, 
                        help="Comma-separated caps (e.g., 'inf,64000,32000'). Default: standard sweep")
    parser.add_argument("--queue_max_msgs", type=int, default=500, help="Queue size limit")
    parser.add_argument("--T_prune", type=float, default=60.0, help="Pruning timeout")
    parser.add_argument("--out_dir", type=str, default=None, help="Output directory for CSV")
    parser.add_argument("--tag_prefix", type=str, default="sweep", help="Tag prefix for runs")
    args = parser.parse_args()
    
    # Determine project directory (parent of holoocean_runs/)
    project_dir = Path(__file__).resolve().parent.parent
    
    # Parse caps
    if args.caps:
        caps = []
        for c in args.caps.split(","):
            c = c.strip()
            caps.append(float("inf") if c.lower() == "inf" else float(c))
    else:
        caps = DEFAULT_CAPS
    
    seeds = list(range(args.seed_start, args.seed_start + args.n_seeds))
    
    # Output directory (relative to project dir)
    if args.out_dir:
        out_dir = project_dir / args.out_dir
    else:
        out_dir = project_dir / "sweeps" / f"rate_cap_N{args.N}"
    out_dir.mkdir(parents=True, exist_ok=True)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = out_dir / f"sweep_N{args.N}_{timestamp}.csv"
    
    print(f"=" * 60)
    print(f"  Rate Cap Sweep: N={args.N}")
    print(f"  Project dir: {project_dir}")
    print(f"  Caps: {caps}")
    print(f"  Seeds: {seeds}")
    print(f"  Output: {csv_path}")
    print(f"=" * 60)
    
    rows = []
    
    for cap in caps:
        print(f"\n--- rate_cap_bps = {cap if not math.isinf(cap) else 'inf'} ---")
        for seed in seeds:
            data = run_epoch(
                N=args.N,
                seed=seed,
                rate_cap_bps=cap,
                queue_max_msgs=args.queue_max_msgs,
                T_prune=args.T_prune,
                tag_prefix=args.tag_prefix,
                project_dir=project_dir,
            )
            if data:
                row = extract_row(data, cap, seed)
                rows.append(row)
    
    # Write CSV
    if rows:
        fieldnames = list(rows[0].keys())
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        print(f"\n✓ Saved {len(rows)} rows to {csv_path}")
    else:
        print("\n✗ No results to save")


if __name__ == "__main__":
    main()
