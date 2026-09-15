#!/usr/bin/env python3
"""
Deterministic test for grace period logic in _update_committed_parent
"""
import networkx as nx
from distributed_pruning_algorithm import DistributedPruningAlgorithm

def test_grace_period_deterministic():
    print("Testing grace period logic deterministically...")

    # Create a simple graph: 3 nodes in a line (1-2-3) with root=1
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
        delete_threshold_h=10.0,  # Large to avoid interference
        grace_period_g=0.5,       # 0.5 seconds grace period
        enable_backup=False
    )

    # Get node 2 state
    state = algo.node_state[2]

    # Initialize state to simulate: node 2 currently committed to parent 1
    state.committed_parent = 1
    state.parent = 1  # candidate parent also 1 initially
    state.last_candidate = 1  # last candidate was also 1

    print(f"Initial state:")
    print(f"  committed_parent: {state.committed_parent}")
    print(f"  parent (candidate): {state.parent}")
    print(f"  last_candidate: {state.last_candidate}")
    print(f"  grace_counter: {dict(state.grace_counter)}")

    # Verify initial conditions
    assert state.committed_parent == 1
    assert state.parent == 1
    assert state.last_candidate == 1
    assert state.grace_counter.get(1, 0.0) == 0.0

    # Now simulate: candidate parent changes to 3 (but node 3 is not actually a neighbor of 2 in our graph!)
    # Let's fix the graph to make 3 a neighbor of 2
    G.add_edge(2, 3)  # Make sure 2 and 3 are connected
    state.neighbors = set(G.neighbors(2))  # Update neighbors

    # Manually set up the scenario: candidate parent changes to 3
    state.parent = 3  # New candidate parent
    # Note: last_candidate is still 1 from initialization

    print(f"\nAfter changing candidate parent to 3:")
    print(f"  committed_parent: {state.committed_parent}")
    print(f"  parent (candidate): {state.parent}")
    print(f"  last_candidate: {state.last_candidate}")

    # Call _update_committed_parent at time t=1.0
    t_now = 1.0
    algo._update_committed_parent(2, t_now)

    print(f"\nAfter _update_committed_parent at t={t_now}:")
    print(f"  committed_parent: {state.committed_parent}")
    print(f"  parent (candidate): {state.parent}")
    print(f"  last_candidate: {state.last_candidate}")
    print(f"  grace_counter: {dict(state.grace_counter)}")

    # Verify: grace period should have started for old parent (1)
    assert state.committed_parent == 1  # Should NOT change yet
    assert state.last_candidate == 3    # Should update last_candidate
    assert state.grace_counter.get(1, 0.0) == 1.0  # Should be set to t_now (1.0)
    print("  > Grace period started correctly (grace_counter set to 1.0)")
    print("  > committed_parent unchanged during grace period")

    # Call _update_committed_parent at time t=1.25 (still within grace period)
    t_now = 1.25
    algo._update_committed_parent(2, t_now)

    print(f"\nAfter _update_committed_parent at t={t_now}:")
    print(f"  committed_parent: {state.committed_parent}")
    print(f"  parent (candidate): {state.parent}")
    print(f"  last_candidate: {state.last_candidate}")
    print(f"  grace_counter: {dict(state.grace_counter)}")

    # Verify: still within grace period, so committed_parent should not change
    assert state.committed_parent == 1  # Should NOT change yet
    assert state.last_candidate == 3    # Should remain 3
    assert state.grace_counter.get(1, 0.0) == 1.0  # Grace counter should still be set
    print("  > Still in grace period - committed_parent unchanged")

    # Call _update_committed_parent at time t=1.6 (after grace period expired)
    t_now = 1.6
    algo._update_committed_parent(2, t_now)

    print(f"\nAfter _update_committed_parent at t={t_now}:")
    print(f"  committed_parent: {state.committed_parent}")
    print(f"  parent (candidate): {state.parent}")
    print(f"  last_candidate: {state.last_candidate}")
    print(f"  grace_counter: {dict(state.grace_counter)}")

    # Verify: grace period expired, so committed_parent should change to candidate (3)
    assert state.committed_parent == 3  # Should NOW change to candidate
    assert state.last_candidate == 3    # Should remain 3
    assert state.grace_counter.get(1, 0.0) == 0.0  # Grace counter should be cleared
    print("  > Grace period expired - committed_parent changed to new candidate")
    print("  > Grace counter cleared after change")

    print("\n>>> All grace period tests passed! <<<")

if __name__ == "__main__":
    test_grace_period_deterministic()