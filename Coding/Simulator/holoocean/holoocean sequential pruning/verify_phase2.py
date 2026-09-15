#!/usr/bin/env python3
import networkx as nx
from distributed_pruning_algorithm import DistributedPruningAlgorithm, NodeState, EDGE_STATUS_UNUSED

def test_node_state_fields():
    # Create a simple graph: 3 nodes in a line
    G = nx.Graph()
    G.add_edges_from([(1, 2), (2, 3)])

    # Instantiate the algorithm
    algo = DistributedPruningAlgorithm(G, root=1, p_drop=0.1, delay_max=0.1, t_broadcast=0.1, jitter=0.1, seed=42)

    # Check that we have node states for each node
    assert set(algo.node_state.keys()) == {1, 2, 3}

    for node_id, state in algo.node_state.items():
        # Check that the NodeState is of the correct type
        assert isinstance(state, NodeState)

        # Check that the original fields exist
        assert hasattr(state, 'node_id')
        assert hasattr(state, 'neighbors')
        assert hasattr(state, 'delta_to_root')
        assert hasattr(state, 'parent')
        assert hasattr(state, 'seq')
        assert hasattr(state, 'last_seq_from_neighbor')
        assert hasattr(state, 'last_delta_update_time')
        assert hasattr(state, 'kept_neighbors')

        # Check that the new fields exist
        assert hasattr(state, 'committed_parent')
        assert hasattr(state, 'backup_parent')
        assert hasattr(state, 'edge_status')
        assert hasattr(state, 'delete_counter')
        assert hasattr(state, 'grace_counter')
        assert hasattr(state, 'last_heard_time')

        # Check that the new fields are of the correct type
        assert state.committed_parent is None or isinstance(state.committed_parent, int)
        assert state.backup_parent is None or isinstance(state.backup_parent, int)
        assert isinstance(state.edge_status, dict)
        assert isinstance(state.delete_counter, dict)
        assert isinstance(state.grace_counter, dict)
        assert isinstance(state.last_heard_time, dict)

        # Check that the per-neighbor state is initialized for each neighbor
        for neighbor in state.neighbors:
            assert neighbor in state.edge_status
            assert neighbor in state.delete_counter
            assert neighbor in state.grace_counter
            assert neighbor in state.last_heard_time

            # Check initial values
            assert state.edge_status[neighbor] == EDGE_STATUS_UNUSED
            assert state.delete_counter[neighbor] == -1.0
            assert state.grace_counter[neighbor] == 0.0
            assert state.last_heard_time[neighbor] == 0.0

    print("All tests passed!")

if __name__ == "__main__":
    test_node_state_fields()