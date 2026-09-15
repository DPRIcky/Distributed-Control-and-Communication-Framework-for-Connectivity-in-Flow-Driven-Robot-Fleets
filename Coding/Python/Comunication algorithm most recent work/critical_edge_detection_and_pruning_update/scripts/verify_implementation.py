#!/usr/bin/env python
"""Quick verification test for the implementation"""

import networkx as nx
from distributed_edge_connectivity import DistributedEdgeConnectivity, CentralizedEdgeConnectivity

def test_basic():
    """Basic test of the implementation"""
    print("=" * 60)
    print("QUICK VERIFICATION TEST")
    print("=" * 60)
    
    # Create a simple path graph: 0-1-2-3-4
    graph = nx.path_graph(5)
    print(f"\nTest Graph: Path with 5 nodes")
    print(f"Edges: {list(graph.edges())}")
    
    # Test distributed algorithm
    print("\n[1/2] Testing Distributed Algorithm...")
    try:
        dist_algo = DistributedEdgeConnectivity(graph)
        dist_results = dist_algo.run()
        print(f"✓ Distributed algorithm executed successfully")
        print(f"  - Critical edges: {dist_results['critical_edges']}")
        print(f"  - Number of bridges: {dist_results['bridge_count']}")
    except Exception as e:
        print(f"✗ Distributed algorithm failed: {e}")
        return False
    
    # Test centralized algorithm
    print("\n[2/2] Testing Centralized Baseline...")
    try:
        cent_algo = CentralizedEdgeConnectivity(graph)
        cent_results = cent_algo.run()
        print(f"✓ Centralized algorithm executed successfully")
        print(f"  - Critical edges: {cent_results['critical_edges']}")
        print(f"  - Number of bridges: {cent_results['bridge_count']}")
    except Exception as e:
        print(f"✗ Centralized algorithm failed: {e}")
        return False
    
    # Verification
    print("\n" + "=" * 60)
    print("VERIFICATION RESULTS")
    print("=" * 60)
    
    print(f"\nAll edges in path graph should be bridges:")
    print(f"  Expected: 4 bridges")
    print(f"  Distributed found: {dist_results['bridge_count']} bridges")
    print(f"  Centralized found: {cent_results['bridge_count']} bridges")
    
    # Test passed
    if (dist_results['bridge_count'] == 4 and 
        cent_results['bridge_count'] == 4):
        print("\n✓ IMPLEMENTATION VERIFIED SUCCESSFULLY!")
        return True
    else:
        print("\n✗ VERIFICATION FAILED - Bridge counts don't match expected")
        return False

if __name__ == "__main__":
    success = test_basic()
    exit(0 if success else 1)
