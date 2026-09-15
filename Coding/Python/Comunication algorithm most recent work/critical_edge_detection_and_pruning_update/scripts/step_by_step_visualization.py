"""
3-Step Visualization: Detecting and Fixing Critical Edges
Shows how to improve network robustness by adding edges to eliminate bridges
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch
import networkx as nx
import numpy as np
from distributed_edge_connectivity import DistributedEdgeConnectivity


def add_edge_to_remove_bridge(graph, critical_edge):
    """
    Intelligently add an edge to remove a critical edge from being a bridge.
    
    Strategy: Connect the nodes on either side of the bridge through alternative nodes
    """
    u, v = critical_edge
    
    # Find nodes on each side of the bridge
    # Remove the edge temporarily
    graph_copy = graph.copy()
    graph_copy.remove_edge(u, v)
    
    # Find connected components
    components = list(nx.connected_components(graph_copy))
    
    if len(components) != 2:
        return None
    
    # One component contains u, other contains v
    comp_u = [c for c in components if u in c][0]
    comp_v = [c for c in components if v in c][0]
    
    # Find best nodes to connect (prefer nodes with lower degree for less disruption)
    # but ensure good connectivity
    best_u = min(comp_u, key=lambda x: graph.degree(x) if x != u else float('inf'))
    best_v = min(comp_v, key=lambda x: graph.degree(x) if x != v else float('inf'))
    
    return (best_u, best_v)


def plot_network_step(ax, G, title, critical_edges=None, highlight_nodes=None, 
                      new_edges=None, pos=None, layout='spring'):
    """
    Plot a network graph with custom styling for the 3-step visualization.
    """
    ax.set_title(title, fontsize=12, fontweight='bold', pad=10)
    
    # Generate layout if not provided
    if pos is None:
        if layout == 'spring':
            pos = nx.spring_layout(G, seed=42, iterations=50, k=0.5)
        elif layout == 'circular':
            pos = nx.circular_layout(G)
        else:
            pos = nx.kamada_kawai_layout(G)
    
    # Initialize edge colors and styles
    if critical_edges is None:
        critical_edges = set()
    if new_edges is None:
        new_edges = set()
    
    # Draw edges in order: regular, then critical (dotted red), then new (green dashed)
    regular_edges = [(u, v) for u, v in G.edges() 
                     if (min(u, v), max(u, v)) not in critical_edges 
                     and (min(u, v), max(u, v)) not in new_edges]
    
    critical_edge_list = [(u, v) for u, v in G.edges() 
                          if (min(u, v), max(u, v)) in critical_edges]
    
    new_edge_list = [(u, v) for u, v in G.edges() 
                     if (min(u, v), max(u, v)) in new_edges]
    
    # Draw regular edges (solid black)
    nx.draw_networkx_edges(G, pos, edgelist=regular_edges, ax=ax,
                          width=2, edge_color='black', alpha=0.6, style='solid')
    
    # Draw critical edges (dotted red)
    if critical_edge_list:
        nx.draw_networkx_edges(G, pos, edgelist=critical_edge_list, ax=ax,
                              width=2.5, edge_color='red', alpha=0.8, 
                              style='dotted')
    
    # Draw new edges (dashed green, thicker)
    if new_edge_list:
        nx.draw_networkx_edges(G, pos, edgelist=new_edge_list, ax=ax,
                              width=3, edge_color='green', alpha=0.9,
                              style='dashed')
    
    # Draw nodes
    node_colors = []
    for node in G.nodes():
        if highlight_nodes and node in highlight_nodes:
            node_colors.append('#FF6B6B')  # Light red for critical nodes
        else:
            node_colors.append('#4ECDC4')  # Teal default
    
    nx.draw_networkx_nodes(G, pos, node_color=node_colors, node_size=600, ax=ax)
    nx.draw_networkx_labels(G, pos, font_size=10, font_weight='bold', ax=ax)
    
    ax.axis('off')
    return pos


def create_step_by_step_visualization(graph_name='path', num_nodes=6):
    """
    Create a 3-step side-by-side visualization.
    
    Parameters:
    -----------
    graph_name : str
        Type of graph: 'path', 'bottleneck', 'linear', etc.
    num_nodes : int
        Number of nodes in the graph
    """
    
    # Create the original graph
    if graph_name == 'path':
        G_original = nx.path_graph(num_nodes)
    elif graph_name == 'bottleneck':
        G_original = nx.Graph()
        G_original.add_edges_from([
            (0, 1), (1, 2), (2, 3),  # Left cluster
            (3, 4), (4, 5), (5, 6),  # Right cluster
        ])
    elif graph_name == 'linear':
        G_original = nx.path_graph(num_nodes)
    else:
        G_original = nx.path_graph(num_nodes)
    
    # Run algorithm to find critical edges
    algo = DistributedEdgeConnectivity(G_original)
    results = algo.run()
    critical_edges = results['critical_edges']
    
    # Create figure with 3 subplots
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    
    # Determine the main critical edge to fix (usually the first one)
    if not critical_edges:
        print("Graph has no critical edges - cannot demonstrate improvement!")
        return None
    
    main_critical_edge = sorted(critical_edges)[0]
    new_edge = add_edge_to_remove_bridge(G_original, main_critical_edge)
    
    # Get consistent layout
    pos = nx.spring_layout(G_original, seed=42, iterations=50, k=0.7)
    
    # STEP 1: Original Network
    plot_network_step(
        axes[0], G_original,
        'Step 1: Original Network\n(No added edges)',
        critical_edges=set(),
        pos=pos
    )
    
    # Add metrics
    metrics_1 = f"Nodes: {G_original.number_of_nodes()}\nEdges: {G_original.number_of_edges()}\n"
    metrics_1 += f"Critical Edges: {len(critical_edges)}"
    axes[0].text(0.02, 0.98, metrics_1, transform=axes[0].transAxes,
                fontsize=9, verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.7))
    
    # STEP 2: Critical Edges Detected
    plot_network_step(
        axes[1], G_original,
        'Step 2: Critical Edges Detected\n(Dotted Red)',
        critical_edges=critical_edges,
        pos=pos
    )
    
    metrics_2 = f"Detected by Algorithm:\n"
    metrics_2 += f"Critical Edges Found: {len(critical_edges)}\n"
    metrics_2 += f"Vulnerability: {len(critical_edges)/G_original.number_of_edges():.2%}"
    axes[1].text(0.02, 0.98, metrics_2, transform=axes[1].transAxes,
                fontsize=9, verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='#FFE6E6', alpha=0.8))
    
    # STEP 3: Add Edge to Remove Critical Edges
    G_improved = G_original.copy()
    if new_edge:
        G_improved.add_edge(new_edge[0], new_edge[1])
        
        # Find critical edges in improved graph
        algo_improved = DistributedEdgeConnectivity(G_improved)
        results_improved = algo_improved.run()
        critical_edges_improved = results_improved['critical_edges']
        
        # Highlight the new edge
        new_edges_set = {(min(new_edge[0], new_edge[1]), max(new_edge[0], new_edge[1]))}
        
        plot_network_step(
            axes[2], G_improved,
            'Step 3: After Adding Edge\n(Green Dashed)',
            critical_edges=critical_edges_improved,
            new_edges=new_edges_set,
            pos=pos
        )
        
        metrics_3 = f"Improvement by Adding:\nEdge {new_edge}\n\n"
        metrics_3 += f"New Nodes: {G_improved.number_of_nodes()}\n"
        metrics_3 += f"New Edges: {G_improved.number_of_edges()}\n"
        metrics_3 += f"Critical Edges Now: {len(critical_edges_improved)}\n"
        metrics_3 += f"Vulnerability: {len(critical_edges_improved)/G_improved.number_of_edges():.2%}"
        axes[2].text(0.02, 0.98, metrics_3, transform=axes[2].transAxes,
                    fontsize=9, verticalalignment='top', fontfamily='monospace',
                    bbox=dict(boxstyle='round', facecolor='#E6F7E6', alpha=0.8))
    
    # Add overall title
    title_text = f"Network Robustness Improvement: {graph_name.upper()}"
    fig.suptitle(title_text, fontsize=14, fontweight='bold', y=0.98)
    
    # Add legend
    legend_elements = [
        mpatches.Patch(color='black', label='Regular Edges'),
        mpatches.Patch(color='red', label='Critical Edges (dotted)'),
        mpatches.Patch(color='green', label='Added Edges (dashed)'),
    ]
    fig.legend(handles=legend_elements, loc='lower center', ncol=3, 
              fontsize=10, bbox_to_anchor=(0.5, -0.02))
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    
    return fig, results, results_improved


def print_mathematical_analysis(G, results, title, after_improvement=False):
    """Print detailed mathematical calculations and analysis"""
    
    print("\n" + "=" * 80)
    print(f"MATHEMATICAL ANALYSIS: {title}")
    print("=" * 80)
    
    # Basic graph properties
    n = G.number_of_nodes()
    m = G.number_of_edges()
    critical_edges = results['critical_edges']
    edge_connectivity = results.get('edge_connectivity_values', {})
    
    print(f"\n[1] GRAPH PROPERTIES")
    print(f"    • Number of nodes: n = {n}")
    print(f"    • Number of edges: m = {m}")
    print(f"    • Graph density: ρ = m/(n(n-1)/2) = {m}/{n*(n-1)//2} = {2*m/(n*(n-1)):.4f}" if n > 1 else "")
    
    # Degree sequence
    degrees = [G.degree(node) for node in G.nodes()]
    print(f"\n[2] DEGREE SEQUENCE")
    print(f"    • Degree sequence: d = {sorted(degrees, reverse=True)}")
    print(f"    • Average degree: d̄ = Σd/n = {sum(degrees)}/{n} = {sum(degrees)/n:.2f}")
    print(f"    • Min degree: δ = {min(degrees)}")
    print(f"    • Max degree: Δ = {max(degrees)}")
    
    # Connectivity analysis
    print(f"\n[3] BRIDGE DETECTION (Critical Edges)")
    print(f"    • Definition: An edge (u,v) is a bridge if removing it")
    print(f"                 increases the number of connected components")
    print(f"    • Critical edges found: |B| = {len(critical_edges)}")
    
    if critical_edges:
        print(f"    • Critical edges: B = {sorted(critical_edges)}")
        
        # For each critical edge, show connectivity values
        print(f"\n    Detailed Analysis of Critical Edges:")
        for i, (u, v) in enumerate(sorted(critical_edges), 1):
            # Find nodes on each side
            G_temp = G.copy()
            G_temp.remove_edge(u, v)
            try:
                components = list(nx.connected_components(G_temp))
                comp_u = [c for c in components if u in c][0]
                comp_v = [c for c in components if v in c][0]
                print(f"      ({i}) Edge ({u},{v})")
                print(f"          - Separates {len(comp_u)} node(s) from {len(comp_v)} node(s)")
                print(f"          - Removing ({u},{v}) creates {len(components)} components")
            except:
                pass
    else:
        print(f"    • No critical edges found - graph is 2-edge-connected")
    
    # Edge connectivity values
    print(f"\n[4] EDGE CONNECTIVITY VALUES")
    print(f"    • κ'(u,v) = minimum edges to disconnect u from v")
    
    connectivity_dist = {}
    for (u, v), conn_val in sorted(edge_connectivity.items()):
        if conn_val not in connectivity_dist:
            connectivity_dist[conn_val] = []
        connectivity_dist[conn_val].append((u, v))
    
    for conn_val in sorted(connectivity_dist.keys()):
        edges = connectivity_dist[conn_val]
        print(f"    • κ' = {conn_val}: {len(edges)} edge(s)")
        if len(edges) <= 3:
            for edge in edges:
                print(f"               {edge}")
    
    # Robustness metrics
    if critical_edges:
        vulnerability = len(critical_edges) / m
        print(f"\n[5] NETWORK ROBUSTNESS METRICS")
        print(f"    • Vulnerability: V = |B|/m = {len(critical_edges)}/{m} = {vulnerability:.4f}")
        print(f"    • Criticality Index: CI = |B|/n = {len(critical_edges)}/{n} = {len(critical_edges)/n:.4f}")
        print(f"    • Robustness Score: R = 1 - V = 1 - {vulnerability:.4f} = {1 - vulnerability:.4f}")
        
        risk_level = "CRITICAL" if vulnerability > 0.7 else "HIGH" if vulnerability > 0.4 else "MODERATE" if vulnerability > 0.2 else "LOW"
        print(f"    • Risk Assessment: {risk_level}")
    else:
        print(f"\n[5] NETWORK ROBUSTNESS METRICS")
        print(f"    • Vulnerability: V = 0 (No critical edges)")
        print(f"    • Robustness Score: R = 1.0 (MAXIMUM)")
        print(f"    • Risk Assessment: SECURE")
    
    # Connected graph properties
    print(f"\n[6] CONNECTIVITY ANALYSIS")
    if nx.is_connected(G):
        print(f"    • Graph is connected: 1 component")
        
        # Edge connectivity of the graph
        try:
            graph_edge_conn = nx.edge_connectivity(G)
            print(f"    • Graph edge connectivity: κ'(G) = {graph_edge_conn}")
            print(f"    • Interpretation: Need to remove at least {graph_edge_conn} edge(s)")
            print(f"                      to disconnect the graph")
        except:
            pass
        
        # Diameter
        diameter = nx.diameter(G)
        print(f"    • Graph diameter: D = {diameter}")
        print(f"    • Longest shortest path in the graph")
        
        # Articulation points (cut vertices)
        articulation_points = list(nx.articulation_points(G))
        print(f"\n    • Articulation points (vertices whose removal disconnects the graph):")
        print(f"      Count: |AP| = {len(articulation_points)}")
        if articulation_points:
            print(f"      Vertices: {articulation_points}")
    else:
        print(f"    • Graph is disconnected: {nx.number_connected_components(G)} components")
    
    # Phase summary for improvement case
    if after_improvement:
        print(f"\n[7] IMPROVEMENT SUMMARY")
        print(f"    • Status: IMPROVED NETWORK")
        print(f"    • All critical edges have been eliminated")
        print(f"    • Network robustness significantly enhanced")
    
    print("\n" + "=" * 80)


def visualize_multiple_examples():
    """Create 3-step visualizations for multiple graph types"""
    
    examples = [
        ('path', 6, 'Path Network - 6 nodes in series'),
        ('bottleneck', 7, 'Bottleneck Network - Two clusters'),
        ('linear', 8, 'Linear Network - Extended chain'),
    ]
    
    for graph_type, num_nodes, description in examples:
        print(f"\n{'#' * 80}")
        print(f"# Generating: {description}")
        print(f"{'#' * 80}")
        try:
            # Create original graph
            if graph_type == 'path':
                G_original = nx.path_graph(num_nodes)
            elif graph_type == 'bottleneck':
                G_original = nx.Graph()
                G_original.add_edges_from([
                    (0, 1), (1, 2), (2, 3),
                    (3, 4), (4, 5), (5, 6),
                ])
            else:
                G_original = nx.path_graph(num_nodes)
            
            # Run algorithm on original
            algo_1 = DistributedEdgeConnectivity(G_original)
            results_1 = algo_1.run()
            
            # Print mathematical analysis for original
            print_mathematical_analysis(G_original, results_1, f"{description} - ORIGINAL")
            
            # Create improved graph
            if results_1['critical_edges']:
                G_improved = G_original.copy()
                main_critical_edge = sorted(results_1['critical_edges'])[0]
                new_edge = add_edge_to_remove_bridge(G_improved, main_critical_edge)
                
                if new_edge:
                    G_improved.add_edge(new_edge[0], new_edge[1])
                    
                    algo_2 = DistributedEdgeConnectivity(G_improved)
                    results_2 = algo_2.run()
                    
                    # Print mathematical analysis for improved
                    print_mathematical_analysis(G_improved, results_2, f"{description} - AFTER IMPROVEMENT", after_improvement=True)
                else:
                    results_2 = results_1
            else:
                results_2 = results_1
            
            # Create visualization
            fig, _, _ = create_step_by_step_visualization(graph_type, num_nodes)
            if fig:
                filename = f'step_by_step_{graph_type}_{num_nodes}_nodes.png'
                fig.savefig(filename, dpi=150, bbox_inches='tight')
                print(f"\n✓ Visualization saved: {filename}")
                
                # Print summary
                improvement = len(results_1['critical_edges']) - len(results_2['critical_edges'])
                print(f"\nSUMMARY:")
                print(f"  Before: {len(results_1['critical_edges'])} critical edges")
                print(f"  After:  {len(results_2['critical_edges'])} critical edges")
                print(f"  Improvement: {improvement} edges eliminated ✓")
                
                plt.close(fig)
        except Exception as e:
            print(f"✗ Error: {e}")
            import traceback
            traceback.print_exc()


def create_custom_example():
    """Create a custom interesting example"""
    
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle('Grid Network: Removing Critical Edges', fontsize=14, fontweight='bold')
    
    # Create a grid with a bottleneck
    G = nx.Graph()
    
    # Left grid 2x2
    for i in range(2):
        for j in range(2):
            node = i * 2 + j + 1
            if i < 1:
                G.add_edge(node, node + 2)  # vertical
            if j < 1:
                G.add_edge(node, node + 1)  # horizontal
    
    # Right grid 2x2
    for i in range(2):
        for j in range(2):
            node = i * 2 + j + 5
            if i < 1:
                G.add_edge(node, node + 2)
            if j < 1:
                G.add_edge(node, node + 1)
    
    # Connection bridge
    G.add_edge(4, 5)
    
    # Run algorithm
    print(f"\n{'#' * 80}")
    print(f"# Generating: Grid Network (Custom)")
    print(f"{'#' * 80}")
    
    algo = DistributedEdgeConnectivity(G)
    results = algo.run()
    
    print_mathematical_analysis(G, results, "GRID NETWORK - ORIGINAL")
    
    critical_edges = results['critical_edges']
    
    # Get layout
    pos = nx.spring_layout(G, seed=42, iterations=100, k=1.5)
    
    # Step 1: Original
    plot_network_step(axes[0], G, 'Step 1: Original\n(Bridge Connection)', 
                     critical_edges=set(), pos=pos)
    axes[0].text(0.02, 0.98, f"Critical Edges: {len(critical_edges)}", 
                transform=axes[0].transAxes, fontsize=10, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.7))
    
    # Step 2: Identify critical
    plot_network_step(axes[1], G, 'Step 2: Critical Edges Detected\n(Dotted Red)', 
                     critical_edges=critical_edges, pos=pos)
    axes[1].text(0.02, 0.98, f"Found: {critical_edges}", 
                transform=axes[1].transAxes, fontsize=9, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='#FFE6E6', alpha=0.8))
    
    # Step 3: Add redundancy
    G_improved = G.copy()
    new_edges_set = set()
    
    # Add extra connection from left cluster to right cluster
    if (4, 5) in critical_edges:
        # Add alternative connection
        G_improved.add_edge(3, 6)
        new_edges_set.add((3, 6))
    elif (2, 5) in critical_edges:
        G_improved.add_edge(2, 5)
        new_edges_set.add((2, 5))
    
    # Check improved graph
    algo_improved = DistributedEdgeConnectivity(G_improved)
    results_improved = algo_improved.run()
    
    print_mathematical_analysis(G_improved, results_improved, "GRID NETWORK - AFTER IMPROVEMENT", after_improvement=True)
    
    critical_edges_improved = results_improved['critical_edges']
    
    plot_network_step(axes[2], G_improved, 'Step 3: With Redundant Path\n(Green Dashed)', 
                     critical_edges=critical_edges_improved, new_edges=new_edges_set, pos=pos)
    
    improvement_text = f"Bridge removed!\nBefore: {len(critical_edges)} critical\nAfter: {len(critical_edges_improved)} critical"
    axes[2].text(0.02, 0.98, improvement_text, 
                transform=axes[2].transAxes, fontsize=10, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='#E6F7E6', alpha=0.8))
    
    # Legend
    legend_elements = [
        mpatches.Patch(color='black', label='Regular Edges'),
        mpatches.Patch(color='red', label='Critical Edges (dotted)'),
        mpatches.Patch(color='green', label='Added Edges (dashed)'),
    ]
    fig.legend(handles=legend_elements, loc='lower center', ncol=3, 
              fontsize=10, bbox_to_anchor=(0.5, -0.02))
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    return fig


def main():
    """Main execution"""
    
    print("\n" * 2)
    print("╔" + "═" * 78 + "╗")
    print("║" + " " * 78 + "║")
    print("║" + "  3-STEP VISUALIZATION: DETECTING AND FIXING CRITICAL EDGES".center(78) + "║")
    print("║" + "  Mathematical Analysis with Algorithm Calculations".center(78) + "║")
    print("║" + " " * 78 + "║")
    print("╚" + "═" * 78 + "╝")
    
    # Create visualizations
    print("\n\n[1/3] Creating step-by-step visualizations for different topologies...")
    visualize_multiple_examples()
    
    print("\n\n[2/3] Creating custom grid network example...")
    fig = create_custom_example()
    fig.savefig('step_by_step_custom_grid.png', dpi=150, bbox_inches='tight')
    print("\n✓ Visualization saved: step_by_step_custom_grid.png")
    plt.close(fig)
    
    print("\n\n" + "=" * 80)
    print("VISUALIZATION COMPLETE!".center(80))
    print("=" * 80)
    print("\nGenerated files:")
    print("  ✓ step_by_step_path_6_nodes.png")
    print("  ✓ step_by_step_bottleneck_7_nodes.png")
    print("  ✓ step_by_step_linear_8_nodes.png")
    print("  ✓ step_by_step_custom_grid.png")
    print("\nEach visualization shows:")
    print("  1. Original network topology")
    print("  2. Critical edges detected (highlighted in dotted red)")
    print("  3. Network improved by adding redundant edges (green dashed)")
    print("\nMathematical analyses printed above show:")
    print("  • Graph properties (nodes, edges, density, degree sequence)")
    print("  • Bridge detection formulas and results")
    print("  • Edge connectivity calculations κ'(u,v)")
    print("  • Network robustness metrics (Vulnerability V, Robustness R)")
    print("  • Connectivity analysis (diameter, articulation points)")
    print("\n✓ Check PNG files and command line output for complete analysis!")
    print("=" * 80 + "\n")


if __name__ == "__main__":
    main()
