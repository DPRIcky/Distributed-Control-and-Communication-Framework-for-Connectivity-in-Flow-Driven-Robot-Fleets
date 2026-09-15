#!/usr/bin/env python
"""
Sweep Simulation 3: distance-dependent packet drop impairment.
Runs 3 conditions × 5 seeds for N=20, then rebuilds a CSV from epoch_summary.json.

Conditions:
  baseline  -- dist_impair_model=none (standard IID p_drop=0.1)
  weak      -- dist_impair_model=drop --dist_profile weak
  strong    -- dist_impair_model=drop --dist_profile strong

Usage (from holoocean_pruning_project/):
    python -m holoocean_runs.sweep_sim3
    python -m holoocean_runs.sweep_sim3 --N 20 --n_seeds 5 --T_prune 30 \\
        --out_dir sweeps/sim3 --tag_prefix sim3
"""
import argparse
import csv
import json
import math
import os
import subprocess
import sys
from pathlib import Path

CONDITIONS = [
    {"name": "baseline", "args": ["--dist_impair_model", "none"]},
    {"name": "weak",     "args": ["--dist_impair_model", "drop", "--dist_profile", "weak"]},
    {"name": "strong",   "args": ["--dist_impair_model", "drop", "--dist_profile", "strong"]},
]

CSV_FIELDS = [
    "condition", "seed", "N",
    "success", "t_tree_first", "t_tree_stable_first", "duration_s",
    "termination_reason",
    "rx_Bps", "tx_Bps", "observed_drop_rate",
    "send_dist_mean_m", "send_dist_p95_m",
    "p_drop_eff_mean", "p_drop_eff_p95",
    "dist_impair_model", "dist_profile",
    "dist_p0", "dist_k", "dist_alpha", "dist_pmax",
]


def extract_row(data: dict, condition: str, seed: int) -> dict:
    r = data.get("results", {})
    params = data.get("parameters", {})
    dm = data.get("dist_model", {})
    connected = r.get("connected", False)
    is_tree = r.get("is_tree", False)
    return {
        "condition":              condition,
        "seed":                   seed,
        "N":                      params.get("N"),
        "success":                1 if (connected and is_tree) else 0,
        "t_tree_first":           r.get("t_tree_first"),
        "t_tree_stable_first":    r.get("t_tree_stable_first"),
        "duration_s":             r.get("duration_s"),
        "termination_reason":     r.get("termination_reason"),
        "rx_Bps":                 r.get("rx_Bps"),
        "tx_Bps":                 r.get("tx_Bps"),
        "observed_drop_rate":     r.get("observed_drop_rate"),
        "send_dist_mean_m":       r.get("send_dist_mean_m"),
        "send_dist_p95_m":        r.get("send_dist_p95_m"),
        "p_drop_eff_mean":        r.get("p_drop_eff_mean"),
        "p_drop_eff_p95":         r.get("p_drop_eff_p95"),
        "dist_impair_model":      dm.get("dist_impair_model"),
        "dist_profile":           dm.get("dist_profile"),
        "dist_p0":                dm.get("dist_p0"),
        "dist_k":                 dm.get("dist_k"),
        "dist_alpha":             dm.get("dist_alpha"),
        "dist_pmax":              dm.get("dist_pmax"),
    }


def run_epoch(cmd: list[str], timeout: int = 300) -> int:
    """Run an epoch subprocess; return exit code."""
    print(f"\n  Running: {' '.join(cmd)}")
    env = {**os.environ, "PYTHONIOENCODING": "utf-8"}
    result = subprocess.run(cmd, timeout=timeout, env=env)
    return result.returncode


def rebuild_csv(project_dir: Path, N: int, n_seeds: int, seed_start: int,
                tag_prefix: str, out_path: Path) -> list[dict]:
    """Read epoch_summary.json for all runs and return list of row dicts."""
    results_dir = project_dir / "stress_results_holoocean"
    rows = []
    missing = []
    for cond in CONDITIONS:
        for seed in range(seed_start, seed_start + n_seeds):
            tag = f"{tag_prefix}_N{N}_{cond['name']}_s{seed}"
            summary_path = results_dir / tag / "epoch_summary.json"
            if not summary_path.exists():
                missing.append(tag)
                continue
            with open(summary_path) as f:
                data = json.load(f)
            rows.append(extract_row(data, cond["name"], seed))

    if missing:
        print(f"\nMissing {len(missing)} runs:")
        for m in missing:
            print(f"  {m}")

    if rows:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with open(out_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        expected = len(CONDITIONS) * n_seeds
        print(f"\nCSV: {out_path}  ({len(rows)}/{expected} rows)")

    return rows


def main():
    parser = argparse.ArgumentParser(description="Sim3 distance-impairment sweep")
    parser.add_argument("--N",          type=int,   default=20)
    parser.add_argument("--n_seeds",    type=int,   default=5)
    parser.add_argument("--seed_start", type=int,   default=1)
    parser.add_argument("--T_prune",    type=float, default=30.0)
    parser.add_argument("--tag_prefix", type=str,   default="sim3")
    parser.add_argument("--out_dir",    type=str,   default="sweeps/sim3")
    parser.add_argument("--rebuild_only", action="store_true",
                        help="Skip running experiments; only rebuild CSV from existing JSONs")
    args = parser.parse_args()

    project_dir = Path(__file__).resolve().parent.parent
    out_path = project_dir / args.out_dir / "sim3_results.csv"

    if not args.rebuild_only:
        seeds = list(range(args.seed_start, args.seed_start + args.n_seeds))
        total = len(CONDITIONS) * len(seeds)
        done = 0
        failed = 0

        for cond in CONDITIONS:
            for seed in seeds:
                tag = f"{args.tag_prefix}_N{args.N}_{cond['name']}_s{seed}"
                cmd = [
                    sys.executable, "-m", "holoocean_runs.run_pruning_epoch",
                    "--N", str(args.N),
                    "--T_prune", str(args.T_prune),
                    "--seed", str(seed),
                    "--tag", tag,
                    "--no_viz", "true",
                    "--early_stop", "true",
                    *cond["args"],
                ]
                try:
                    rc = run_epoch(cmd, timeout=600)
                    done += 1
                    status = "OK" if rc == 0 else f"exit={rc}"
                except subprocess.TimeoutExpired:
                    status = "TIMEOUT"
                    failed += 1
                except Exception as e:
                    status = f"ERROR: {e}"
                    failed += 1
                print(f"  [{done}/{total}] {tag}: {status}")

        print(f"\nAll {total} runs attempted ({failed} failures).")

    # Always rebuild CSV from JSON files
    rows = rebuild_csv(project_dir, args.N, args.n_seeds, args.seed_start,
                       args.tag_prefix, out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
