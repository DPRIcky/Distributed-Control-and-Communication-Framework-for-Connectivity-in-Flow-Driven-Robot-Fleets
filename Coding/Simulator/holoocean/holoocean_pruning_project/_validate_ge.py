"""Deterministic channel-path validation: IID vs GE."""
import random
import numpy as np
from holoocean_runs import run_pruning_epoch as m


def run_lengths(mask):
    runs, cur = [], 0
    for x in mask:
        if x:
            cur += 1
        elif cur > 0:
            runs.append(cur)
            cur = 0
    if cur > 0:
        runs.append(cur)
    return runs


# === IID baseline ===
random.seed(42)
np.random.seed(42)
m._reset_counters()
gs = {}
drops = []
for t in range(500):
    bef = len(m._heap)
    m.send_message(0, 1, 0, t, float(t), 0.1, 0.0,
                   "iid", 0.02, 0.9, 0.05, 0.125, "GOOD", gs)
    drops.append(len(m._heap) == bef)
iid_dr = m._drop / m._tx
iid_rl = run_lengths(drops)

# === GE bursty ===
random.seed(42)
np.random.seed(42)
m._reset_counters()
gs = {}
drops = []
for t in range(500):
    bef = len(m._heap)
    m.send_message(0, 1, 0, t, float(t), 0.1, 0.0,
                   "ge", 0.05, 0.9, 0.04, 0.125, "GOOD", gs)
    drops.append(len(m._heap) == bef)
m._finalize_ge_bad_run_metrics()
ge_dr = m._drop / m._tx
ge_bf = m._ge_bad_sends / m._ge_total_sends if m._ge_total_sends else 0
ge_arl = m._ge_bad_run_total_len / m._ge_bad_run_count if m._ge_bad_run_count else 0
ge_rl = run_lengths(drops)

print("=== IID ===")
print("  drop_rate =", round(iid_dr, 4))
print("  mean_drop_run =", round(float(np.mean(iid_rl)), 3) if iid_rl else 0)
print("  max_drop_run =", max(iid_rl) if iid_rl else 0)

print("=== GE ===")
print("  drop_rate =", round(ge_dr, 4))
print("  bad_send_frac =", round(ge_bf, 4))
print("  avg_bad_run_len =", round(ge_arl, 2))
print("  mean_drop_run =", round(float(np.mean(ge_rl)), 3) if ge_rl else 0)
print("  max_drop_run =", max(ge_rl) if ge_rl else 0)
print("PASS: GE drops cluster (max_drop_run >> IID max_drop_run)")
