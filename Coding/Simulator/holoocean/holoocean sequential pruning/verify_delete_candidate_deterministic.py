#!/usr/bin/env python3
"""
Deterministic test for DELETE_CANDIDATE state machine logic in _update_edge_status_for_node
"""
import networkx as nx
from distributed_pruning_algorithm import DistributedPruningAlgorithm, EDGE_STATUS_DELETE_CANDIDATE, EDGE_STATUS_UNUSED, EDGE_STATUS_PARENT

def test_delete_candidate_deterministic():
    print("Testing DELETE_CANDIDATE state machine deterministically...")

    # Create a simple graph: 3 nodes in a line (1-2-3)
    G = nx.Graph()
    G.add_edges_from([(1, 2), (2, 3)])

    # Instantiate the algorithm
    algo = DistributedPruningAlgorithm(
        G,
        root=1,
        p_drop=0.0,
        delay_max=0.0,
        t_broadcast=1.0,  # Value doesn't matter for this test
        jitter=0.0,
        seed=42,
        delete_threshold_h=0.5,  # 0.5 seconds delete threshold
        grace_period_g=10.0,     # Large to avoid interference
        enable_backup=False
    )

    # Get node 2 state
    state = algo.node_state[2]

    # Initialize state to simulate: edge 2-1 should be deleted
    # We'll set up conditions where desired_status is UNUSED
    state.committed_parent = 3  # Committed to node 3, not node 1
    state.backup_parent = None  # No backup parent
    state.grace_counter[1] = 0.0  # No active grace period

    print(f"Initial state for node 2, edge to neighbor 1:")
    print(f"  committed_parent: {state.committed_parent}")
    print(f"  backup_parent: {state.backup_parent}")
    print(f"  grace_counter[1]: {state.grace_counter.get(1, 0.0)}")
    print(f"  delete_counter[1]: {state.delete_counter.get(1, -1.0)}")
    print(f"  edge_status[1]: {state.edge_status.get(1, EDGE_STATUS_UNUSED)}")
    print(f"  kept_neighbors: {set(state.kept_neighbors)}")

    # Verify initial conditions - should be UNUSED and not kept
    assert state.edge_status.get(1, EDGE_STATUS_UNUSED) == EDGE_STATUS_UNUSED
    assert 1 not in state.kept_neighbors

    # Now test the DELETE_CANDIDATE state machine

    # PHASE 1: Enter DELETE_CANDIDATE state (start delete timer)
    print(f"\n--- PHASE 1: Enter DELETE_CANDIDATE state ---")
    t_now = 1.0
    algo._update_edge_status_for_node(2, t_now)

    print(f"After _update_edge_status_for_node at t={t_now}:")
    print(f"  delete_counter[1]: {state.delete_counter.get(1, -1.0)}")
    print(f"  edge_status[1]: {state.edge_status.get(1, EDGE_STATUS_UNUSED)}")
    print(f"  kept_neighbors: {set(state.kept_neighbors)}")

    # Verify: edge should now be in DELETE_CANDIDATE state and kept
    assert state.delete_counter.get(1, -1.0) == 1.0  # Delete timer started at t_now
    assert state.edge_status.get(1, EDGE_STATUS_UNUSED) == EDGE_STATUS_DELETE_CANDIDATE
    assert 1 in state.kept_neighbors  # DELETE_CANDIDATE edges are kept
    print("  > Edge entered DELETE_CANDIDATE state and is kept")

    # PHASE 2: Stay in DELETE_CANDIDATE state (delete timer running)
    print(f"\n--- PHASE 2: Stay in DELETE_CANDIDATE state (timer running) ---")
    t_now = 1.2  # 0.2 seconds after entering DELETE_CANDIDATE
    algo._update_edge_status_for_node(2, t_now)

    print(f"After _update_edge_status_for_node at t={t_now}:")
    print(f"  delete_counter[1]: {state.delete_counter.get(1, -1.0)}")
    print(f"  edge_status[1]: {state.edge_status.get(1, EDGE_STATUS_UNUSED)}")
    print(f"  kept_neighbors: {set(state.kept_neighbors)}")

    # Verify: edge should still be in DELETE_CANDIDATE state and kept
    assert state.delete_counter.get(1, -1.0) == 1.0  # Delete counter unchanged (still start time)
    assert state.edge_status.get(1, EDGE_STATUS_UNUSED) == EDGE_STATUS_DELETE_CANDIDATE
    assert 1 in state.kept_neighbors  # Still kept
    print("  > Edge remains in DELETE_CANDIDATE state and is kept")

    # PHASE 3: Transition from DELETE_CANDIDATE to UNUSED (delete timer expired)
    print(f"\n--- PHASE 3: Transition to UNUSED (delete timer expired) ---")
    t_now = 1.6  # 0.6 seconds after entering DELETE_CANDIDATE (0.1 seconds after threshold)
    algo._update_edge_status_for_node(2, t_now)

    print(f"After _update_edge_status_for_node at t={t_now}:")
    print(f"  delete_counter[1]: {state.delete_counter.get(1, -1.0)}")
    print(f"  edge_status[1]: {state.edge_status.get(1, EDGE_STATUS_UNUSED)}")
    print(f"  kept_neighbors: {set(state.kept_neighbors)}")

    # Verify: edge should now be in UNUSED state and NOT kept
    assert state.delete_counter.get(1, -1.0) == -1.0  # Delete counter reset
    assert state.edge_status.get(1, EDGE_STATUS_UNUSED) == EDGE_STATUS_UNUSED
    assert 1 not in state.kept_neighbors  # UNUSED edges are removed from kept_neighbors
    print("  > Edge transitioned to UNUSED state and removed from kept_neighbors")

    # PHASE 4: Test that if we want to keep the edge, it doesn't enter DELETE_CANDIDATE
    print(f"\n--- PHASE 4: Edge that should be kept (PARENT) ---")
    # Make the edge a parent edge
    state.committed_parent = 1  # Now committed to node 1

    t_now = 2.0
    algo._update_edge_status_for_node(2, t_now)

    print(f"After _update_edge_status_for_node at t={t_now} (edge should be PARENT):")
    print(f"  delete_counter[1]: {state.delete_counter.get(1, -1.0)}")
    print(f"  edge_status[1]: {state.edge_status.get(1, EDGE_STATUS_UNUSED)}")
    print(f"  kept_neighbors: {set(state.kept_neighbors)}")

    # Verify: edge should be PARENT status and kept
    assert state.delete_counter.get(1, -1.0) == -1.0  # Delete counter reset
    assert state.edge_status.get(1, EDGE_STATUS_UNUSED) == EDGE_STATUS_PARENT
    assert 1 in state.kept_neighbors  # PARENT edges are kept
    print("  > Edge correctly set to PARENT state and kept")

    print("\n>>> All DELETE_CANDIDATE state machine tests passed! <<<")

if __name__ == "__main__":
    test_delete_candidate_deterministic()