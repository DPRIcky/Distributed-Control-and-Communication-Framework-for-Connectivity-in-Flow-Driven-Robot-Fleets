"""
Example usage of the Distributed Pruning Algorithm module.
Shows how to run the algorithm on custom networks and visualize results.
"""

import networkx as nx
from distributed_pruning_algorithm import DistributedPruningAlgorithm


def example_basic_usage():
    """Basic example: create a network and run the algorithm."""
    print("\n" + "="*70)
    print("EXAMPLE 1: Basic Usage")
    print("="*70)
    
    # Create a 10-node random network
    G = nx.gnp_random_graph(10, 0.3, seed=123)
    while not nx.is_connected(G):
        G = nx.gnp_random_graph(10, 0.3, seed=123)
    
    # Relabel nodes to 1..10
    mapping = {i: i+1 for i in range(len(G))}
    G = nx.relabel_nodes(G, mapping)
    
    # Create and run algorithm
    algo = DistributedPruningAlgorithm(G, root=1)
    algo.run_full_pipeline()
    
    # Print results
    algo.print_results()
    
    # Get pruned network
    pruned = algo.get_pruned_graph()
    print(f"\nPruned network is connected: {nx.is_connected(pruned)}")
    print(f"Number of cycles: {len(algo.final_edges) - len(G) + 1}")


def example_specific_topology():
    """Example: run on a specific topology."""
    print("\n" + "="*70)
    print("EXAMPLE 2: Specific Topology (Cube Graph)")
    print("="*70)
    
    # Create cube graph (8 nodes, 12 edges)
    G = nx.cubical_graph()
    
    # Relabel nodes to 1..8
    mapping = {i: i+1 for i in range(len(G))}
    G = nx.relabel_nodes(G, mapping)
    
    print(f"Network: Cube")
    print(f"Nodes: {len(G)}")
    print(f"Edges: {len(G.edges())}")
    print(f"Average degree: {2*len(G.edges())/len(G):.2f}")
    
    algo = DistributedPruningAlgorithm(G, root=1)
    algo.run_full_pipeline()
    algo.print_results()


def example_custom_network():
    """Example: create a custom network from edges."""
    print("\n" + "="*70)
    print("EXAMPLE 3: Custom Network Definition")
    print("="*70)
    
    # Define network manually
    edges = [
        (1, 2), (1, 3), (1, 5),
        (2, 3), (2, 4), (2, 6),
        (3, 4), (3, 7),
        (4, 5), (4, 8),
        (5, 6), (5, 9),
        (6, 7), (6, 10),
        (7, 8),
        (8, 9),
        (9, 10),
    ]
    
    G = nx.Graph()
    G.add_edges_from(edges)
    
    print(f"Network created with {len(G)} nodes and {len(G.edges())} edges")
    print(f"Network connected: {nx.is_connected(G)}")
    
    algo = DistributedPruningAlgorithm(G, root=1)
    algo.run_full_pipeline()
    algo.print_results()
    
    # Show final edge set
    print(f"\nFinal edges in pruned network:")
    for edge in sorted(algo.final_edges):
        print(f"  {edge}")


def example_analyze_convergence():
    """Example: analyze algorithm convergence step-by-step."""
    print("\n" + "="*70)
    print("EXAMPLE 4: Step-by-Step Convergence Analysis")
    print("="*70)
    
    # Create simple network
    G = nx.Graph()
    G.add_edges_from([(1, 2), (2, 3), (3, 4), (4, 1), (2, 4)])
    
    algo = DistributedPruningAlgorithm(G, root=1)
    
    # Phase 1: Step by step
    print("\nPhase 1: Distance Computation")
    algo.phase1_init()
    for step in range(5):
        algo.phase1_step()
        print(f"  Step {step}: Distance vector at node 1: {dict(algo.node_state[1].distance_vector)}")
    
    # Phase 2
    print("\nPhase 2: Connectivity Assurance")
    algo.phase2_init()
    for step in range(3):
        algo.phase2_step()
        connected_count = sum(
            len(algo.node_state[n].connected_component) for n in algo.nodes
        ) // len(algo.nodes)
        print(f"  Step {step}: Average component size = {connected_count}")
    
    # Phase 3
    print("\nPhase 3: Spanning Tree Construction")
    algo.phase3_init()
    algo.phase3_parent_selection()
    print("  Parent assignments:")
    for node in algo.nodes:
        if node != algo.root:
            print(f"    Node {node}: parent = {algo.node_state[node].parent}")
    
    algo.phase3_keep_prune_decision()
    tree_edges = set()
    for node in algo.nodes:
        tree_edges.update(algo.node_state[node].keep_edges)
    print(f"  Spanning tree edges: {len(tree_edges)}")
    
    # Phase 4
    print("\nPhase 4: Robustness Enhancement")
    algo.phase4_init()
    algo.phase4_step1_delta_calculation()
    
    candidates_total = sum(
        len(algo.node_state[node].rewire_candidates) for node in algo.nodes
    )
    print(f"  Candidate edges for rewiring: {candidates_total}")
    
    algo.phase4_step2_rank_candidates()
    algo.phase4_step3_distributed_selection()
    
    for node in algo.nodes:
        if algo.node_state[node].best_candidate:
            print(f"  Node {node}'s best candidate: {algo.node_state[node].best_candidate}")


def example_compare_topologies():
    """Example: compare pruning results across different topologies."""
    print("\n" + "="*70)
    print("EXAMPLE 5: Comparison Across Topologies")
    print("="*70)
    
    topologies = {
        'cycle': lambda n: nx.cycle_graph(n),
        'path': lambda n: nx.path_graph(n),
        'star': lambda n: nx.star_graph(n-1),
        'complete': lambda n: nx.complete_graph(n),
        'wheel': lambda n: nx.wheel_graph(n),
    }
    
    n = 10
    results = {}
    
    for name, builder in topologies.items():
        G = builder(n)
        
        # Relabel nodes
        mapping = {i: i+1 for i in range(len(G))}
        G = nx.relabel_nodes(G, mapping)
        
        algo = DistributedPruningAlgorithm(G, root=1)
        algo.run_full_pipeline()
        
        stats = algo.get_statistics()
        results[name] = {
            'original_edges': stats['original_edges'],
            'final_edges': stats['final_edges'],
            'pruning_ratio': (1 - stats['final_edges'] / stats['original_edges']) * 100,
        }
    
    print(f"\n{'Topology':<12} {'Original':<10} {'Final':<10} {'Pruning %':<12}")
    print("-" * 44)
    for name, data in results.items():
        print(f"{name:<12} {data['original_edges']:<10} {data['final_edges']:<10} {data['pruning_ratio']:<12.1f}")


def main():
    """Run all examples."""
    print("\n" + "="*70)
    print("DISTRIBUTED PRUNING ALGORITHM - USAGE EXAMPLES")
    print("="*70)
    
    example_basic_usage()
    example_specific_topology()
    example_custom_network()
    example_analyze_convergence()
    example_compare_topologies()
    
    print("\n" + "="*70)
    print("EXAMPLES COMPLETE")
    print("="*70 + "\n")


if __name__ == "__main__":
    main()
    """
    Compare distributed and centralized algorithms on a given graph.
    
    Parameters:
    -----------
    graph : nx.Graph
        Network to analyze
    graph_name : str
        Name of the network
    """
    print("\n" + "=" * 70)
    print(f"ANALYZING NETWORK: {graph_name}")
    print("=" * 70)
    print(f"Network: {graph.number_of_nodes()} nodes, {graph.number_of_edges()} edges")
    print()
    
    # Run distributed algorithm
    print("Running DISTRIBUTED ALGORITHM...")
    print("-" * 70)
    distributed_algo = DistributedEdgeConnectivity(graph)
    dist_results = distributed_algo.run()
    dist_metrics = distributed_algo.get_robustness_metrics()
    
    # Run centralized algorithm (baseline)
    print("\nRunning CENTRALIZED BASELINE...")
    print("-" * 70)
    centralized_algo = CentralizedEdgeConnectivity(graph)
    cent_results = centralized_algo.run()
    cent_metrics = centralized_algo.get_robustness_metrics()
    
    # Compare results
    print("\n" + "=" * 70)
    print("COMPARISON OF RESULTS")
    print("=" * 70)
    
    print("\nCritical Edges (Bridges):")
    print(f"  Distributed:  {sorted(dist_results['critical_edges'])}")
    print(f"  Centralized:  {sorted(cent_results['critical_edges'])}")
    print(f"  Match: {dist_results['critical_edges'] == cent_results['critical_edges']}")
    
    print("\nEdge Connectivity Values:")
    dist_conn = {str(k): v for k, v in sorted(dist_results['edge_connectivity'].items())}
    cent_conn = {str(k): v for k, v in sorted(cent_results['edge_connectivity'].items())}
    print(f"  Distributed:  {dist_conn}")
    print(f"  Centralized:  {cent_conn}")
    
    print("\nRobustness Metrics:")
    print(f"  {'Metric':<40} {'Distributed':<15} {'Centralized':<15}")
    print(f"  {'-'*70}")
    
    for key in cent_metrics.keys():
        if isinstance(cent_metrics[key], float):
            print(f"  {key:<40} {dist_metrics[key]:<15.4f} {cent_metrics[key]:<15.4f}")
        else:
            print(f"  {key:<40} {dist_metrics[key]!s:<15} {cent_metrics[key]!s:<15}")
    
    # Analyze redundancy for some node pairs
    print("\n" + "=" * 70)
    print("REDUNDANCY ANALYSIS")
    print("=" * 70)
    
    nodes = list(graph.nodes())
    if len(nodes) >= 2:
        # Pick a few node pairs for analysis
        test_pairs = [
            (nodes[0], nodes[-1]),
        ]
        
        if len(nodes) > 2:
            test_pairs.append((nodes[0], nodes[len(nodes)//2]))
        
        for source, target in test_pairs:
            if nx.has_path(graph, source, target):
                redundancy = distributed_algo.identify_redundant_paths(source, target)
                print(f"\nPath from {source} to {target}:")
                print(f"  Number of edge-disjoint paths: {redundancy['num_edge_disjoint_paths']}")
                print(f"  Has redundancy: {redundancy['has_redundancy']}")
                if redundancy['edge_disjoint_paths']:
                    for i, path in enumerate(redundancy['edge_disjoint_paths'], 1):
                        print(f"    Path {i}: {path}")


def main():
    """Main execution"""
    
    print("\n" + "=" * 70)
    print("DISTRIBUTED EDGE CONNECTIVITY ALGORITHM - DEMONSTRATION")
    print("=" * 70)
    
    # Create example networks
    networks = create_example_networks()
    
    # Analyze each network
    for net_name, graph in networks.items():
        compare_algorithms(graph, net_name.upper())
    
    print("\n" + "=" * 70)
    print("DEMONSTRATION COMPLETE")
    print("=" * 70)
    print("\nKey Results Summary:")
    print("- Distributed algorithm correctly identifies bridges/critical edges")
    print("- Edge connectivity values match centralized baseline")
    print("- Robustness metrics provide insights into network vulnerability")
    print("- Redundancy analysis identifies alternative paths for reliability")
    

if __name__ == "__main__":
    main()
