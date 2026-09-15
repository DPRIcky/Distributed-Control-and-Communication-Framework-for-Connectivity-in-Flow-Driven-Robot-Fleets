#!/usr/bin/env python3
import networkx as nx
from distributed_pruning_algorithm import DistributedPruningAlgorithm

def test_default():
    # Create a simple graph: 3 nodes in a line
    G = nx.Graph()
    G.add_edges_from([(1, 2), (2, 3)])

    # Instantiate the algorithm with default parameters
    algo = DistributedPruningAlgorithm(G, root=1)

    # Run the algorithm for a short time without early stopping
    kept_edges = algo.run(t_prune=5.0, dt=0.1, t_stable=None, verbose=False)

    print(f"Kept edges: {kept_edges}")
    expected = {(1, 2), (2, 3)}
    if kept_edges == expected:
        print("SUCCESS: Default parameters work.")
    else:
        print(f"FAILURE: Expected {expected}, got {kept_edges}")

if __name__ == "__main__":
    test_default()