"""
Test harness for Distributed Pruning Algorithm.
Validates correctness on various network topologies.
"""

import networkx as nx
import sys
from pathlib import Path

# Add scripts directory to path
sys.path.insert(0, str(Path(__file__).parent))

from distributed_pruning_algorithm import DistributedPruningAlgorithm


def test_small_network():
    """Test on 4-node simple network."""
    print("\n" + "="*70)
    print("TEST 1: Small 4-Node Network")
    print("="*70)
    
    # Create a simple 4-node network
    G = nx.Graph()
    G.add_edges_from([(1, 2), (1, 3), (2, 3), (2, 4), (3, 4)])
    
    algo = DistributedPruningAlgorithm(G, root=1)
    algo.run_full_pipeline()
    algo.print_results()
    
    # Verify results
    stats = algo.get_statistics()
    assert stats['connected'], "Final graph should be connected"
    assert stats['final_edges'] >= 4 - 1, f"Should have at least {4-1} edges, got {stats['final_edges']}"
    assert stats['final_edges'] <= 4, f"Should have at most {4} edges, got {stats['final_edges']}"
    print("✓ TEST PASSED\n")


def test_8node_network():
    """Test on 8-node network (from documentation example)."""
    print("\n" + "="*70)
    print("TEST 2: 8-Node Network (Documentation Example)")
    print("="*70)
    
    # Create 8-node network from documentation
    G = nx.Graph()
    edges = [
        (1, 2), (1, 4), (1, 8),
        (2, 3), (2, 4),
        (3, 4),
        (4, 5),
        (5, 6), (5, 7),
        (6, 7), (6, 8),
    ]
    G.add_edges_from(edges)
    
    algo = DistributedPruningAlgorithm(G, root=1)
    algo.run_full_pipeline()
    algo.print_results()
    
    # Verify results
    stats = algo.get_statistics()
    assert stats['connected'], "Final graph should be connected"
    assert stats['final_edges'] == 8, f"Should have exactly 8 edges for 8-node network, got {stats['final_edges']}"
    print("✓ TEST PASSED\n")


def test_grid_network():
    """Test on 3x3 grid network."""
    print("\n" + "="*70)
    print("TEST 3: 3x3 Grid Network")
    print("="*70)
    
    # Create 3x3 grid
    G = nx.grid_2d_graph(3, 3)
    # Relabel nodes to integers 1..9
    mapping = {node: i+1 for i, node in enumerate(sorted(G.nodes()))}
    G = nx.relabel_nodes(G, mapping)
    
    algo = DistributedPruningAlgorithm(G, root=1)
    algo.run_full_pipeline()
    algo.print_results()
    
    # Verify results
    stats = algo.get_statistics()
    n = len(G)
    assert stats['nodes'] == n, f"Should have {n} nodes"
    assert stats['connected'], "Final graph should be connected"
    assert stats['final_edges'] == n, f"Should have exactly {n} edges for {n}-node network, got {stats['final_edges']}"
    assert stats['cycles'] == 1, f"Should have exactly 1 cycle, got {stats['cycles']}"
    print(f"✓ TEST PASSED (Grid {n} nodes, pruning ratio: {(1 - stats['final_edges'] / stats['original_edges'])*100:.1f}%)\n")


def test_connected_tree():
    """Test on tree (no cycles)."""
    print("\n" + "="*70)
    print("TEST 4: Tree Network (No Cycles)")
    print("="*70)
    
    # Create a simple tree
    T = nx.balanced_tree(2, 2)  # Binary tree of depth 2 = 7 nodes, 6 edges
    # Relabel nodes
    mapping = {node: i+1 for i, node in enumerate(sorted(T.nodes()))}
    T = nx.relabel_nodes(T, mapping)
    
    algo = DistributedPruningAlgorithm(T, root=1)
    algo.run_full_pipeline()
    algo.print_results()
    
    # Verify results
    stats = algo.get_statistics()
    n = len(T)
    assert stats['connected'], "Final graph should be connected"
    assert stats['final_edges'] >= n - 1, f"Should have at least {n-1} edges"
    assert stats['final_edges'] <= n, f"Should have at most {n} edges"
    print(f"✓ TEST PASSED (Tree {n} nodes, no original cycles)\n")


def test_random_network():
    """Test on random network."""
    print("\n" + "="*70)
    print("TEST 5: Random Network (10 nodes, p=0.3)")
    print("="*70)
    
    # Create random connected graph
    G = nx.gnp_random_graph(10, 0.3, seed=42)
    while not nx.is_connected(G):
        G = nx.gnp_random_graph(10, 0.3, seed=42)
    
    # Relabel nodes
    mapping = {node: i+1 for i, node in enumerate(sorted(G.nodes()))}
    G = nx.relabel_nodes(G, mapping)
    
    algo = DistributedPruningAlgorithm(G, root=1)
    algo.run_full_pipeline()
    algo.print_results()
    
    # Verify results
    stats = algo.get_statistics()
    n = len(G)
    assert stats['connected'], "Final graph should be connected"
    assert stats['final_edges'] == n, f"Should have exactly {n} edges for {n}-node network, got {stats['final_edges']}"
    pruning_ratio = (1 - stats['final_edges'] / stats['original_edges']) * 100
    print(f"✓ TEST PASSED (Random network, pruning ratio: {pruning_ratio:.1f}%)\n")


def test_distance_computation():
    """Verify Phase 1 distance computation accuracy."""
    print("\n" + "="*70)
    print("TEST 6: Distance Computation Accuracy (Phase 1)")
    print("="*70)
    
    # Create test network
    G = nx.Graph()
    G.add_edges_from([(1, 2), (2, 3), (3, 4)])  # Path graph
    
    algo = DistributedPruningAlgorithm(G, root=1)
    algo.run_phase1()
    
    # Check distances
    expected_distances = {
        1: {1: 0, 2: 1, 3: 2, 4: 3},
        2: {1: 1, 2: 0, 3: 1, 4: 2},
        3: {1: 2, 2: 1, 3: 0, 4: 1},
        4: {1: 3, 2: 2, 3: 1, 4: 0},
    }
    
    for node in G.nodes():
        for target in G.nodes():
            computed = algo.node_state[node].distance_vector[target]
            expected = expected_distances[node][target]
            assert computed == expected, \
                f"Distance from {node} to {target}: expected {expected}, got {computed}"
    
    print("✓ All distances correct")
    print("✓ TEST PASSED\n")


def test_spanning_tree_property():
    """Verify spanning tree has exactly n-1 edges before robustness."""
    print("\n" + "="*70)
    print("TEST 7: Spanning Tree Properties (Phase 3)")
    print("="*70)
    
    # Create network
    G = nx.complete_graph(6)
    # Relabel nodes
    mapping = {i: i+1 for i in range(6)}
    G = nx.relabel_nodes(G, mapping)
    
    algo = DistributedPruningAlgorithm(G, root=1)
    algo.run_phase1()
    algo.run_phase2()
    algo.run_phase3()
    
    # Count tree edges before Phase 4
    tree_edges = set()
    for node in algo.nodes:
        tree_edges.update(algo.node_state[node].keep_edges)
    
    n = len(G)
    print(f"Spanning tree edges: {len(tree_edges)}")
    print(f"Expected: {n-1}")
    
    assert len(tree_edges) == n - 1, \
        f"Spanning tree should have {n-1} edges, got {len(tree_edges)}"
    
    # Verify it forms a connected tree
    tree_graph = nx.Graph()
    tree_graph.add_nodes_from(algo.nodes)
    tree_graph.add_edges_from(tree_edges)
    
    assert nx.is_connected(tree_graph), "Spanning tree should be connected"
    assert nx.is_tree(tree_graph), "Spanning tree should have no cycles"
    
    print("✓ TEST PASSED\n")


def run_all_tests():
    """Run all tests."""
    print("\n" + "="*70)
    print("DISTRIBUTED PRUNING ALGORITHM - TEST SUITE")
    print("="*70)
    
    try:
        test_small_network()
        test_8node_network()
        test_grid_network()
        test_connected_tree()
        test_random_network()
        test_distance_computation()
        test_spanning_tree_property()
        
        print("\n" + "="*70)
        print("ALL TESTS PASSED ✓")
        print("="*70 + "\n")
        
    except AssertionError as e:
        print(f"\n✗ TEST FAILED: {e}\n")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ ERROR: {e}\n")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    run_all_tests()
