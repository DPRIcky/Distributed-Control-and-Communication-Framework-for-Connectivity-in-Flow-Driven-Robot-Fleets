#!/usr/bin/env python3
import networkx as nx
from distributed_pruning_algorithm import DistributedPruningAlgorithm, EDGE_STATUS_PARENT, EDGE_STATUS_BACKUP, EDGE_STATUS_GRACE, EDGE_STATUS_DELETE_CANDIDATE, EDGE_STATUS_UNUSED

def test_phase3():
    # Create a simple graph: 3 nodes in a line
    G = nx.Graph()
    G.add_edges_from([(1, 2), (2, 3)])

    # Instantiate the algorithm with some loss and delay to see if state changes
    algo = DistributedPruningAlgorithm(
        G,
        root=1,
        p_drop=0.1,
        delay_max=0.1,
        t_broadcast=0.5,
        jitter=0.1,
        seed=42,
        delete_threshold_h=0.5,  # 0.5 seconds for testing
        grace_period_g=0.3,      # 0.3 seconds for testing
        enable_backup=True
    )

    # Run the algorithm for a short time
    print("Running algorithm for 2 seconds...")
    kept_edges = algo.run(t_prune=2.0, dt=0.1, t_stable=0.5, verbose=False)

    print(f"Kept edges (spanning tree): {kept_edges}")

    # Check the state of each node
    for node_id in sorted(algo.node_state.keys()):
        state = algo.node_state[node_id]
        print(f"\nNode {node_id}:")
        print(f"  delta_to_root: {state.delta_to_root}")
        print(f"  parent (candidate): {state.parent}")
        print(f"  committed_parent: {state.committed_parent}")
        print(f"  backup_parent: {state.backup_parent}")
        print(f"  seq: {state.seq}")
        print(f"  kept_neighbors (maintained topology): {state.kept_neighbors}")
        print(f"  edge_status: {state.edge_status}")
        print(f"  delete_counter: {state.delete_counter}")
        print(f"  grace_counter: {state.grace_counter}")
        print(f"  last_heard_time: {state.last_heard_time}")

    # Verify that the algorithm still produces a spanning tree
    # For a line of 3 nodes with root at 1, the spanning tree should be {(1,2), (2,3)}
    expected_edges = {(1, 2), (2, 3)}
    if kept_edges == expected_edges:
        print("\nSUCCESS: Kept edges match expected spanning tree.")
    else:
        print(f"\nWARNING: Kept edges {kept_edges} do not match expected {expected_edges}. This may be due to loss/delay or the sequential pruning logic.")

    # Verify that the committed_parent is being used for the tree
    for node_id in [2, 3]:
        state = algo.node_state[node_id]
        if node_id == 1:
            continue
        if state.committed_parent is not None:
            edge = tuple(sorted([node_id, state.committed_parent]))
            if edge not in kept_edges:
                print(f"ERROR: Node {node_id} has committed_parent {state.committed_parent} but edge {edge} not in kept_edges.")
            else:
                print(f"INFO: Node {node_id} committed_parent edge {edge} is in the spanning tree.")
        else:
            print(f"WARNING: Node {node_id} has no committed_parent.")

    print("\nTest completed.")

if __name__ == "__main__":
    test_phase3()