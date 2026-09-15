#!/usr/bin/env python
"""Quick smoke-test for the consensus_baseline package.

Run from the workspace root:

    python consensus_baseline/run_smoke_test.py          # short 200-step trial
    python consensus_baseline/run_smoke_test.py --steps 1000 --robots 10

or, after ``pip install -e .``:

    python -m consensus_baseline.run_smoke_test
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Ensure workspace root is importable
_ROOT = str(Path(__file__).resolve().parent.parent)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from consensus_baseline import run_adjacency_consensus_trial


def main() -> None:
    ap = argparse.ArgumentParser(description="Smoke-test consensus_baseline")
    ap.add_argument("--robots", type=int, default=6, help="number of robots")
    ap.add_argument("--radius", type=float, default=3.0, help="comm radius")
    ap.add_argument("--steps", type=int, default=200, help="max sim steps")
    ap.add_argument("--seed", type=int, default=42, help="RNG seed")
    ap.add_argument("--verbose", action="store_true", help="step-by-step log")
    args = ap.parse_args()

    print("=" * 60)
    print("consensus_baseline  —  smoke test")
    print("=" * 60)
    print(f"  robots  = {args.robots}")
    print(f"  radius  = {args.radius}")
    print(f"  steps   = {args.steps}")
    print(f"  seed    = {args.seed}")
    print()

    metrics = run_adjacency_consensus_trial(
        num_robots=args.robots,
        communication_radius=args.radius,
        max_steps=args.steps,
        seed=args.seed,
        verbose=args.verbose,
    )

    print()
    print("─" * 60)
    print("Results")
    print("─" * 60)
    for k, v in metrics.items():
        if k == "pruning_events":
            print(f"  {k:20s} : {len(v)} events")
            for ev in v:
                print(f"    t={ev['time']:.2f}  edge={ev['edge']}  "
                      f"remaining={ev['edges_remaining']}")
        elif isinstance(v, float):
            print(f"  {k:20s} : {v:.4f}")
        else:
            print(f"  {k:20s} : {v}")
    print("─" * 60)

    # Basic sanity assertions
    assert metrics["initial_edges"] >= metrics["final_edges"], \
        "final_edges should be <= initial_edges"
    assert metrics["tx_messages"] >= 0
    assert metrics["rx_messages"] >= 0
    assert metrics["tx_bytes"] >= 0
    assert metrics["final_lambda2"] >= 0, "lambda2 must be non-negative"

    print("\n ALL assertions passed.\n")


if __name__ == "__main__":
    main()
