"""
Unit tests for the Distributed Edge Connectivity Algorithm
"""

import unittest
import networkx as nx
from distributed_edge_connectivity import (
    DistributedEdgeConnectivity, 
    CentralizedEdgeConnectivity,
    NodeState
)


class TestDistributedEdgeConnectivity(unittest.TestCase):
    """Test cases for the distributed algorithm"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Simple path: 1-2-3-4-5
        self.path_graph = nx.path_graph(5)
        self.path_graph = nx.relabel_nodes(self.path_graph, {i: i+1 for i in range(5)})
        
        # Graph with bottleneck
        self.bottleneck = nx.Graph()
        self.bottleneck.add_edges_from([(1, 2), (2, 3), (2, 4), (4, 5), (5, 6)])
        
        # Complete graph (no bridges)
        self.complete = nx.complete_graph(4)
        
        # Cycle graph
        self.cycle = nx.cycle_graph(5)
    
    def test_initialization(self):
        """Test algorithm initialization"""
        algo = DistributedEdgeConnectivity(self.path_graph)
        self.assertEqual(algo.n_nodes, 5)
        self.assertEqual(len(algo.node_states), 5)
        self.assertIsNotNone(algo.node_states)
    
    def test_path_graph_bridges(self):
        """Test bridge detection on path graph"""
        algo = DistributedEdgeConnectivity(self.path_graph)
        results = algo.run()
        
        # All edges in a path should be bridges
        expected_bridges = {
            (1, 2), (2, 3), (3, 4), (4, 5)
        }
        self.assertEqual(results['critical_edges'], expected_bridges)
    
    def test_complete_graph_no_bridges(self):
        """Test that complete graph has no bridges"""
        algo = DistributedEdgeConnectivity(self.complete)
        results = algo.run()
        
        # Complete graph should have no bridges
        self.assertEqual(len(results['critical_edges']), 0)
    
    def test_cycle_graph_no_bridges(self):
        """Test that cycle graph has no bridges"""
        algo = DistributedEdgeConnectivity(self.cycle)
        results = algo.run()
        
        # Cycle graph should have no bridges
        self.assertEqual(len(results['critical_edges']), 0)
    
    def test_bottleneck_bridges(self):
        """Test bridge detection on bottleneck graph"""
        algo = DistributedEdgeConnectivity(self.bottleneck)
        results = algo.run()
        
        # Edges (2,3) and (2,4) and (4,5) should be bridges
        # (2,3) connects to node 3 only
        # (4,5) and (5,6) form a tail
        expected_has_bridge = True
        self.assertTrue(len(results['critical_edges']) > 0)
    
    def test_edge_connectivity_computation(self):
        """Test edge connectivity calculation"""
        algo = DistributedEdgeConnectivity(self.path_graph)
        algo.run()
        metrics = algo.get_robustness_metrics()
        
        self.assertIn('average_edge_connectivity', metrics)
        self.assertIn('minimum_edge_connectivity', metrics)
        self.assertGreaterEqual(metrics['average_edge_connectivity'], 0)
    
    def test_robustness_metrics(self):
        """Test robustness metrics computation"""
        algo = DistributedEdgeConnectivity(self.path_graph)
        algo.run()
        metrics = algo.get_robustness_metrics()
        
        # Check all expected metrics are present
        expected_metrics = [
            'vulnerability',
            'critical_edges_ratio',
            'average_edge_connectivity',
            'minimum_edge_connectivity',
            'maximum_edge_connectivity',
            'network_edge_connectivity'
        ]
        
        for metric in expected_metrics:
            self.assertIn(metric, metrics)
    
    def test_redundant_paths(self):
        """Test redundancy analysis"""
        algo = DistributedEdgeConnectivity(self.cycle)
        redundancy = algo.identify_redundant_paths(0, 2)
        
        # In a cycle, there should be edge-disjoint paths
        self.assertIn('num_edge_disjoint_paths', redundancy)
        self.assertGreaterEqual(redundancy['num_edge_disjoint_paths'], 1)
    
    def test_disconnected_nodes_no_path(self):
        """Test handling of disconnected nodes"""
        disconnected = nx.Graph()
        disconnected.add_edges_from([(1, 2), (3, 4)])
        
        algo = DistributedEdgeConnectivity(disconnected)
        redundancy = algo.identify_redundant_paths(1, 3)
        
        # No path between disconnected components
        self.assertEqual(redundancy['num_edge_disjoint_paths'], 0)


class TestCentralizedEdgeConnectivity(unittest.TestCase):
    """Test cases for the centralized baseline"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.path_graph = nx.path_graph(5)
        self.path_graph = nx.relabel_nodes(self.path_graph, {i: i+1 for i in range(5)})
        self.complete = nx.complete_graph(4)
        self.cycle = nx.cycle_graph(5)
    
    def test_centralized_initialization(self):
        """Test centralized algorithm initialization"""
        algo = CentralizedEdgeConnectivity(self.path_graph)
        self.assertIsNotNone(algo.graph)
    
    def test_path_graph_bridges_centralized(self):
        """Test bridge detection using centralized method"""
        algo = CentralizedEdgeConnectivity(self.path_graph)
        results = algo.run()
        
        # All edges in path should be bridges
        expected_bridges = {
            (0, 1), (1, 2), (2, 3), (3, 4)
        }
        self.assertEqual(results['critical_edges'], expected_bridges)
    
    def test_complete_graph_no_bridges_centralized(self):
        """Test complete graph via centralized method"""
        algo = CentralizedEdgeConnectivity(self.complete)
        results = algo.run()
        
        # No bridges in complete graph
        self.assertEqual(len(results['critical_edges']), 0)
    
    def test_edge_connectivity_centralized(self):
        """Test edge connectivity via centralized method"""
        algo = CentralizedEdgeConnectivity(self.complete)
        results = algo.run()
        
        # In complete graph with 4 nodes, edge connectivity should be 2
        for edge, connectivty in results['edge_connectivity'].items():
            self.assertGreaterEqual(connectivty, 1)


class TestAlgorithmComparison(unittest.TestCase):
    """Test cases comparing distributed vs centralized"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.test_graphs = {
            'path': nx.path_graph(5),
            'cycle': nx.cycle_graph(5),
            'complete': nx.complete_graph(4)
        }
    
    def test_algorithms_find_same_bridges_path(self):
        """Test both algorithms find same bridges in path graph"""
        graph = self.test_graphs['path']
        
        dist_algo = DistributedEdgeConnectivity(graph)
        dist_results = dist_algo.run()
        
        cent_algo = CentralizedEdgeConnectivity(graph)
        cent_results = cent_algo.run()
        
        # Both should find same bridges (after normalizing edge representation)
        self.assertEqual(len(dist_results['critical_edges']), 
                        len(cent_results['critical_edges']))
    
    def test_algorithms_find_same_bridges_cycle(self):
        """Test both algorithms find same bridges in cycle graph"""
        graph = self.test_graphs['cycle']
        
        dist_algo = DistributedEdgeConnectivity(graph)
        dist_results = dist_algo.run()
        
        cent_algo = CentralizedEdgeConnectivity(graph)
        cent_results = cent_algo.run()
        
        # Both should find no bridges in cycle
        self.assertEqual(len(dist_results['critical_edges']), 0)
        self.assertEqual(len(cent_results['critical_edges']), 0)
    
    def test_both_algorithms_consistency(self):
        """Test consistency between algorithms"""
        for graph_name, graph in self.test_graphs.items():
            with self.subTest(graph=graph_name):
                dist_algo = DistributedEdgeConnectivity(graph)
                dist_metrics = dist_algo.run()
                
                cent_algo = CentralizedEdgeConnectivity(graph)
                cent_metrics = cent_algo.run()
                
                # Number of edges should be same
                self.assertEqual(
                    dist_metrics['total_edges'],
                    cent_metrics['total_edges']
                )
                
                # Number of nodes should be same
                self.assertEqual(
                    dist_metrics['total_nodes'],
                    cent_metrics['total_nodes']
                )


class TestNodeState(unittest.TestCase):
    """Test NodeState dataclass"""
    
    def test_node_state_creation(self):
        """Test NodeState initialization"""
        state = NodeState(node_id=1, neighbors={2, 3})
        
        self.assertEqual(state.node_id, 1)
        self.assertEqual(state.neighbors, {2, 3})
        self.assertEqual(state.distance, {})
        self.assertEqual(state.parent, {})
    
    def test_node_state_hashable(self):
        """Test that NodeState is hashable"""
        state1 = NodeState(node_id=1)
        state2 = NodeState(node_id=1)
        state3 = NodeState(node_id=2)
        
        # Should be able to add to set
        node_set = {state1, state2, state3}
        self.assertEqual(len(node_set), 2)  # state1 and state2 have same id


class TestNetworkRobustness(unittest.TestCase):
    """Test network robustness analysis"""
    
    def test_star_topology_vulnerability(self):
        """Test vulnerability analysis on star topology"""
        # Star graph: central node connected to all others
        star = nx.star_graph(5)
        
        algo = DistributedEdgeConnectivity(star)
        algo.run()
        metrics = algo.get_robustness_metrics()
        
        # All edges are critical in star
        self.assertGreater(metrics['vulnerability'], 0.8)
    
    def test_line_topology_vulnerability(self):
        """Test vulnerability analysis on line topology"""
        line = nx.path_graph(6)
        
        algo = DistributedEdgeConnectivity(line)
        algo.run()
        metrics = algo.get_robustness_metrics()
        
        # All edges are critical in line
        self.assertGreater(metrics['vulnerability'], 0.9)
    
    def test_mesh_topology_redundancy(self):
        """Test redundancy in mesh topology"""
        mesh = nx.grid_2d_graph(3, 3)
        
        algo = DistributedEdgeConnectivity(mesh)
        algo.run()
        metrics = algo.get_robustness_metrics()
        
        # Mesh should have low vulnerability
        self.assertLess(metrics['vulnerability'], 0.3)


def run_tests():
    """Run all tests"""
    unittest.main(verbosity=2)


if __name__ == '__main__':
    run_tests()
