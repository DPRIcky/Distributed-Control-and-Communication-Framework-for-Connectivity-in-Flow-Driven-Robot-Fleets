#!/usr/bin/env python
"""
Rebuild a sweep CSV from existing epoch_summary.json files on disk.

Use this when sweep_rate_cap.py silently skipped runs due to non-zero
subprocess exit codes (e.g., matplotlib/encoding errors at script end),
even though the simulation completed and wrote epoch_summary.json.

Usage:
    # Rebuild breakpoint sweep for N=20 (caps 6–240, 3 seeds)
    python -m holoocean_runs.rebuild_sweep_csv --N 20 --n_seeds 3 \\
        --caps "6,12,18,24,30,36,42,48,60,72,96,120,144,192,240" \\
        --out sweeps/breakpoint_N20/sweep_N20_rebuilt.csv

    # Rebuild coarse sweep for N=20 (default caps, 3 seeds)
    python -m holoocean_runs.rebuild_sweep_csv --N 20 --n_seeds 3 \\
        --out sweeps/rate_cap_N20/sweep_N20_rebuilt.csv
"""
import argparse
import csv
import json
import math
from pathlib import Path

DEFAULT_CAPS = [float("inf"), 128_000, 64_000, 32_000, 16_000, 8_000,
                4_000, 2_000, 1_000, 500, 250]


def cap_str(cap: float) -> str:
    return "inf" if math.isinf(cap) else str(int(cap))


def extract_row(data: dict, rate_cap_bps: float, seed: int) -> dict:
    r = data.get("results", {})
    params = data.get("parameters", {})
    connected = r.get("connected", False)
    is_tree = r.get("is_tree", False)
    return {
        "rate_cap_bps": "inf" if math.isinf(rate_cap_bps) else rate_cap_bps,
        "seed": seed,
        "success": 1 if (connected and is_tree) else 0,
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
    parser = argparse.ArgumentParser(description="Rebuild sweep CSV from epoch_summary.json files")
    parser.add_argument("--N", type=int, required=True, help="Number of agents")
    parser.add_argument("--n_seeds", type=int, default=3, help="Number of seeds")
    parser.add_argument("--seed_start", type=int, default=1, help="Starting seed")
    parser.add_argument("--caps", type=str, default=None,
                        help="Comma-separated caps (e.g. '6,12,48,inf'). Default: standard coarse caps")
    parser.add_argument("--tag_prefix", type=str, default="sweep", help="Tag prefix used during sweep")
    parser.add_argument("--out", type=str, required=True, help="Output CSV path")
    args = parser.parse_args()

    project_dir = Path(__file__).resolve().parent.parent
    results_dir = project_dir / "stress_results_holoocean"

    if args.caps:
        caps = []
        for c in args.caps.split(","):
            c = c.strip()
            caps.append(float("inf") if c.lower() == "inf" else float(c))
    else:
        caps = DEFAULT_CAPS

    seeds = list(range(args.seed_start, args.seed_start + args.n_seeds))

    rows = []
    missing = []
    for cap in caps:
        cs = cap_str(cap)
        for seed in seeds:
            tag = f"{args.tag_prefix}_N{args.N}_cap{cs}_s{seed}"
            summary_path = results_dir / tag / "epoch_summary.json"
            if not summary_path.exists():
                missing.append(tag)
                continue
            with open(summary_path) as f:
                data = json.load(f)
            rows.append(extract_row(data, cap, seed))

    if missing:
        print(f"Missing ({len(missing)}):")
        for m in missing:
            print(f"  {m}")

    if not rows:
        print("No rows found — check N, caps, seeds, and tag_prefix.")
        return

    out_path = project_dir / args.out
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    expected = len(caps) * len(seeds)
    print(f"Wrote {len(rows)}/{expected} rows to {out_path}")
    if len(rows) < expected:
        print(f"  ({expected - len(rows)} missing — runs may not have completed)")


if __name__ == "__main__":
    main()
