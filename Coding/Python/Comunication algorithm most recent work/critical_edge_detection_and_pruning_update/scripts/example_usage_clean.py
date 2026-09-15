"""
Example usage of the Distributed Pruning Algorithm.
Fast examples that demonstrate the 4-phase algorithm.
"""

import networkx as nx
from distributed_pruning_algorithm import DistributedPruningAlgorithm
import time


def example1_basic_10node():
    """Example 1: Basic usage with random 10-node network."""
    print("\n" + "="*70)
    print("EXAMPLE 1: Basic Usage - Random 10-Node Network")
    print("="*70)
    
    G = nx.gnp_random_graph(10, 0.3, seed=42)
    while not nx.is_connected(G):
        G = nx.gnp_random_graph(10, 0.3, seed=42)
    
    mapping = {i: i+1 for i in range(len(G))}
    G = nx.relabel_nodes(G, mapping)
    
    print(f"Network: {len(G)} nodes, {len(G.edges())} edges")
    
    algo = DistributedPruningAlgorithm(G, root=1)
    algo.run_full_pipeline()
    algo.print_results()


def example2_cube():
    """Example 2: Cube graph topology."""
    print("\n" + "="*70)
    print("EXAMPLE 2: Cube Graph (8 nodes)")
    print("="*70)
    
    G = nx.cubical_graph()
    mapping = {i: i+1 for i in range(len(G))}
    G = nx.relabel_nodes(G, mapping)
    
    print(f"Network: {len(G)} nodes, {len(G.edges())} edges (3D hypercube)")
    
    algo = DistributedPruningAlgorithm(G, root=1)
    algo.run_full_pipeline()
    algo.print_results()


def example3_custom():
    """Example 3: Custom network from edges."""
    print("\n" + "="*70)
    print("EXAMPLE 3: Custom Network Definition")
    print("="*70)
    
    edges = [
        (1, 2), (1, 3), (2, 3), (2, 4),
        (3, 4), (4, 5), (5, 6), (4, 6),
        (6, 1)  # Create loop
    ]
    
    G = nx.Graph()
    G.add_edges_from(edges)
    
    print(f"Network: {len(G)} nodes, {len(G.edges())} edges")
    
    algo = DistributedPruningAlgorithm(G, root=1)
    algo.run_full_pipeline()
    algo.print_results()


def example4_complete():
    """Example 4: Complete graph K6."""
    print("\n" + "="*70)
    print("EXAMPLE 4: Complete Graph K6")
    print("="*70)
    
    G = nx.complete_graph(6)
    mapping = {i: i+1 for i in range(len(G))}
    G = nx.relabel_nodes(G, mapping)
    
    print(f"Network: Complete K{len(G)}")
    
    algo = DistributedPruningAlgorithm(G, root=1)
    algo.run_full_pipeline()
    
    stats = algo.get_statistics()
    pruning = (1 - stats['final_edges'] / stats['original_edges']) * 100
    print(f"\nResult: {stats['original_edges']} → {stats['final_edges']} edges ({pruning:.0f}% pruning)")


def example5_comparison():
    """Example 5: Compare multiple topologies."""
    print("\n" + "="*70)
    print("EXAMPLE 5: Topology Comparison")
    print("="*70)
    
    topologies = {
        'Cycle': lambda n: nx.cycle_graph(n),
        'Path': lambda n: nx.path_graph(n),
        'Star': lambda n: nx.star_graph(n-1),
        'Wheel': lambda n: nx.wheel_graph(n),
    }
    
    print(f"\n{'Topology':<12} {'Nodes':<8} {'Original':<10} {'Final':<8} {'Pruning %':<12}")
    print("-" * 60)
    
    n = 7
    for name, builder in topologies.items():
        G = builder(n)
        mapping = {i: i+1 for i in range(len(G))}
        G = nx.relabel_nodes(G, mapping)
        
        algo = DistributedPruningAlgorithm(G, root=1)
        algo.run_full_pipeline()
        
        stats = algo.get_statistics()
        pruning = (1 - stats['final_edges'] / stats['original_edges']) * 100
        
        print(f"{name:<12} {len(G):<8} {stats['original_edges']:<10} {stats['final_edges']:<8} {pruning:>10.1f}%")


def main():
    """Run all examples."""
    print("\n" + "="*70)
    print("DISTRIBUTED PRUNING ALGORITHM - USAGE EXAMPLES (FAST)")
    print("="*70)
    
    start = time.time()
    
    try:
        example1_basic_10node()
        example2_cube()
        example3_custom()
        example4_complete()
        example5_comparison()
        
        elapsed = time.time() - start
        
        print("\n" + "="*70)
        print(f"✓ ALL EXAMPLES COMPLETED in {elapsed:.2f}s")
        print("="*70 + "\n")
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
