"""Quick 1-trial sanity check for compare_methods.py logic."""
import sys, time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_CMP  = _HERE.parent
_ROOT = _CMP.parent
for p in (_ROOT, _CMP):
    s = str(p)
    if s not in sys.path:
        sys.path.insert(0, s)

import numpy as np, networkx as nx
from pruning import DistributedPruningAlgorithm
from consensus_baseline import run_adjacency_consensus_trial

N, seed, radius = 10, 1000, 3.0
ws = (10.0, 10.0)

# Replicate positions
rng = np.random.default_rng(seed)
rng.uniform(ws[0]*0.5, ws[0]*0.95)
rng.uniform(ws[1]*0.2, ws[1]*0.8)
cc = np.array([ws[0]*0.25, ws[1]*0.5])
pos = np.empty((N, 2))
for i in range(N):
    rng.integers(0, 2**32-1)
    pos[i] = np.clip(cc + rng.normal(0.0, 0.4, size=2), 0.0, np.array(ws))

G = nx.Graph()
G.add_nodes_from(range(N))
for i in range(N):
    for j in range(i+1, N):
        if np.linalg.norm(pos[i] - pos[j]) <= radius:
            G.add_edge(i, j)
print(f"Graph: {N} nodes, {G.number_of_edges()} edges, connected={nx.is_connected(G)}")

# δ-BFS
t0 = time.perf_counter()
alg = DistributedPruningAlgorithm(
    graph=G, root=0, p_drop=0.0, delay_max=0.0,
    t_broadcast=1.0, jitter=0.0, seed=seed,
)
_, dur, comm = alg.run_epoch(t_prune=20.0, dt=0.1, t_stable=5.0, verbose=False)
s = alg.get_statistics()
w1 = time.perf_counter() - t0
print(f"BFS:  dur={dur:.2f}s  edges={s['final_edges']}  tree={s['is_tree']}  wall={w1:.2f}s")
print(f"  TX={comm['messages_sent_total']}  RX={comm['messages_delivered_total']}"
      f"  RX_Bps={comm['rx_bytes_per_second']:.1f}")

# Baseline
t0 = time.perf_counter()
m = run_adjacency_consensus_trial(
    num_robots=N, communication_radius=radius,
    max_steps=3000, dt=0.05, seed=seed, verbose=False, workspace_size=ws,
)
w2 = time.perf_counter() - t0
print(f"BL:   sim_t={m['sim_time']:.1f}s  edges={m['final_edges']}"
      f"  lam2={m['final_lambda2']:.4f}  wall={w2:.2f}s")
print(f"  TX={m['tx_messages']}  RX={m['rx_messages']}  Bps={m['bytes_per_second']:.1f}")
print(f"  prunes={m['total_prunes']}")
print("\nSanity check PASSED")
