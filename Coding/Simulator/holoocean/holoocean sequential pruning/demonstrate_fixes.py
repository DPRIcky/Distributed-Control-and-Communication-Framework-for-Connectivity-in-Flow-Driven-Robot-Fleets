#!/usr/bin/env python3
"""
Demonstration of the fixed DELETE_CANDIDATE and grace period behaviors
"""
import networkx as nx
from distributed_pruning_algorithm import DistributedPruningAlgorithm, EDGE_STATUS_DELETE_CANDIDATE, EDGE_STATUS_UNUSED, EDGE_STATUS_PARENT, EDGE_STATUS_GRACE

def demonstrate_delete_candidate():
    print("=" * 60)
    print("DEMONSTRATING DELETE_CANDIDATE -> UNUSED TRANSITION")
    print("=" * 60)

    # Create a simple graph: 2 nodes connected
    G = nx.Graph()
    G.add_edge(1, 2)

    algo = DistributedPruningAlgorithm(
        G,
        root=1,
        p_drop=0.0,
        delay_max=0.0,
        t_broadcast=1.0,
        jitter=0.0,
        seed=42,
        delete_threshold_h=0.3,  # 0.3 seconds for quick demo
        grace_period_g=10.0,     # Large to avoid interference
        enable_backup=False
    )

    state = algo.node_state[2]

    # Setup: make edge 2-1 unwanted (should be deleted)
    state.committed_parent = None  # Not committed to anyone
    state.backup_parent = None
    state.grace_counter[1] = 0.0   # No grace period

    print(f"Initial state - Edge 2-1:")
    print(f"  Status: {state.edge_status.get(1, 'UNUSED')}")
    print(f"  Delete counter: {state.delete_counter.get(1, -1.0)}")
    print(f"  Kept neighbors: {set(state.kept_neighbors)}")

    # Enter DELETE_CANDIDATE state
    print(f"\n--- t=0.0s: Edge enters DELETE_CANDIDATE state ---")
    algo._update_edge_status_for_node(2, 0.0)
    print(f"  Status: {state.edge_status.get(1, 'UNUSED')}")
    print(f"  Delete counter: {state.delete_counter.get(1, -1.0)} (timer started)")
    print(f"  Kept neighbors: {set(state.kept_neighbors)} (EDGE STILL KEPT)")

    # Still in DELETE_CANDIDATE (within threshold)
    print(f"\n--- t=0.2s: Still in DELETE_CANDIDATE (within threshold) ---")
    algo._update_edge_status_for_node(2, 0.2)
    print(f"  Status: {state.edge_status.get(1, 'UNUSED')}")
    print(f"  Delete counter: {state.delete_counter.get(1, -1.0)} (unchanged)")
    print(f"  Kept neighbors: {set(state.kept_neighbors)} (EDGE STILL KEPT)")

    # Transition to UNUSED (threshold exceeded)
    print(f"\n--- t=0.5s: Delete threshold exceeded -> UNUSED ---")
    algo._update_edge_status_for_node(2, 0.5)
    print(f"  Status: {state.edge_status.get(1, 'UNUSED')}")
    print(f"  Delete counter: {state.delete_counter.get(1, -1.0)} (reset)")
    print(f"  Kept neighbors: {set(state.kept_neighbors)} (EDGE REMOVED)")

def demonstrate_grace_period():
    print("\n" + "=" * 60)
    print("DEMONSTRATING PARENT SWITCH WITH NONZERO GRACE BEHAVIOR")
    print("=" * 60)

    # Create a simple graph: 3 nodes in line
    G = nx.Graph()
    G.add_edges_from([(1, 2), (2, 3)])

    algo = DistributedPruningAlgorithm(
        G,
        root=1,
        p_drop=0.0,
        delay_max=0.0,
        t_broadcast=1.0,
        jitter=0.0,
        seed=42,
        delete_threshold_h=10.0,  # Large to avoid interference
        grace_period_g=0.4,       # 0.4 seconds for quick demo
        enable_backup=False
    )

    state = algo.node_state[2]

    # Initial state: committed to parent 1
    state.committed_parent = 1
    state.parent = 1          # candidate also 1
    state.last_candidate = 1
    state.grace_counter[1] = 0.0

    print(f"Initial state - Node 2 committed to parent 1:")
    print(f"  Committed parent: {state.committed_parent}")
    print(f"  Candidate parent: {state.parent}")
    print(f"  Grace counter for edge 2-1: {state.grace_counter.get(1, 0.0)}")

    # Candidate parent changes to 3 (but we'll simulate this by manually setting parent)
    print(f"\n--- t=0.0s: Candidate parent changes from 1 to 3 ---")
    state.parent = 3  # New candidate
    # last_candidate is still 1

    print(f"  Committed parent: {state.committed_parent}")
    print(f"  Candidate parent: {state.parent}")
    print(f"  Last candidate: {state.last_candidate}")

    # Call update to start grace period
    algo._update_committed_parent(2, 0.0)
    print(f"\n--- After update at t=0.0s: Grace period started ---")
    print(f"  Committed parent: {state.committed_parent} (UNCHANGED)")
    print(f"  Grace counter for edge 2-1: {state.grace_counter.get(1, 0.0)} (STARTED)")
    print(f"  Last candidate: {state.last_candidate} (UPDATED)")

    # Still in grace period
    print(f"\n--- t=0.2s: Still within grace period ---")
    algo._update_committed_parent(2, 0.2)
    print(f"  Committed parent: {state.committed_parent} (STILL UNCHANGED)")
    print(f"  Grace counter for edge 2-1: {state.grace_counter.get(1, 0.0)} (STILL ACTIVE)")

    # Grace period expired
    print(f"\n--- t=0.5s: Grace period expired ---")
    algo._update_committed_parent(2, 0.5)
    print(f"  Committed parent: {state.committed_parent} (CHANGED TO 3)")
    print(f"  Grace counter for edge 2-1: {state.grace_counter.get(1, 0.0)} (CLEARED)")
    print(f"  Last candidate: {state.last_candidate} (REMAINS 3)")

if __name__ == "__main__":
    demonstrate_delete_candidate()
    demonstrate_grace_period()
    print("\n" + "=" * 60)
    print("DEMONSTRATION COMPLETE")
    print("=" * 60)