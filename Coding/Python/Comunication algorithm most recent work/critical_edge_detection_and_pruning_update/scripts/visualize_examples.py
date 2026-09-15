"""
Visualization of Distributed Edge Connectivity Algorithm
Plots examples similar to those in the CDC 2024 paper
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import networkx as nx
import numpy as np
from distributed_edge_connectivity import DistributedEdgeConnectivity, CentralizedEdgeConnectivity


def plot_network(ax, G, title, critical_edges=None, highlight_nodes=None, layout='spring'):
    """
    Plot a network graph with optional highlighting of critical edges.
    
    Parameters:
    -----------
    ax : matplotlib axis
        Axis to plot on
    G : nx.Graph
        Network to plot
    title : str
        Title for the plot
    critical_edges : set
        Set of critical edges to highlight in red
    highlight_nodes : set
        Set of nodes to highlight
    layout : str
        Layout algorithm: 'spring', 'circular', 'kamada_kawai'
    """
    ax.set_title(title, fontsize=12, fontweight='bold')
    
    # Generate layout
    if layout == 'spring':
        pos = nx.spring_layout(G, seed=42, iterations=50)
    elif layout == 'circular':
        pos = nx.circular_layout(G)
    elif layout == 'kamada_kawai':
        pos = nx.kamada_kawai_layout(G)
    else:
        pos = nx.spring_layout(G, seed=42)
    
    # Draw all edges
    if critical_edges is None:
        critical_edges = set()
    
    # Separate regular and critical edges
    regular_edges = [(u, v) for u, v in G.edges() if (min(u, v), max(u, v)) not in critical_edges]
    critical_edge_list = [(u, v) for u, v in G.edges() if (min(u, v), max(u, v)) in critical_edges]
    
    # Draw regular edges (black)
    nx.draw_networkx_edges(G, pos, edgelist=regular_edges, ax=ax, 
                          width=2, edge_color='black', alpha=0.6)
    
    # Draw critical edges (red)
    if critical_edge_list:
        nx.draw_networkx_edges(G, pos, edgelist=critical_edge_list, ax=ax,
                              width=3, edge_color='red', alpha=0.9)
    
    # Draw nodes
    node_colors = []
    for node in G.nodes():
        if highlight_nodes and node in highlight_nodes:
            node_colors.append('#FF6B6B')  # Light red
        else:
            node_colors.append('#4ECDC4')  # Teal
    
    nx.draw_networkx_nodes(G, pos, node_color=node_colors, node_size=500, ax=ax)
    
    # Draw labels
    nx.draw_networkx_labels(G, pos, font_size=9, font_weight='bold', ax=ax)
    
    ax.axis('off')
    
    return pos


def create_example_networks():
    """Create example networks from typical research scenarios"""
    
    examples = {}
    
    # Example 1: Simple Communication Network (Path-like with some redundancy)
    G1 = nx.Graph()
    G1.add_edges_from([
        (1, 2), (2, 3), (3, 4), (4, 5),  # Main path
    ])
    examples['path_network'] = {
        'graph': G1,
        'title': 'Example 1: Path Network\n(High Vulnerability)',
        'description': 'Linear topology - all edges are critical'
    }
    
    # Example 2: Network with Bottleneck (typical for papers)
    G2 = nx.Graph()
    G2.add_edges_from([
        (1, 2), (2, 3), (3, 4),  # Left cluster
        (3, 5), (5, 6), (6, 7),  # Right cluster via bottleneck
        (2, 5)  # Alternative connection
    ])
    examples['bottleneck_network'] = {
        'graph': G2,
        'title': 'Example 2: Network with Bottleneck\n(Medium Vulnerability)',
        'description': 'Two clusters connected via specific edges'
    }
    
    # Example 3: Redundant Mesh Network
    G3 = nx.Graph()
    # Create a 3x3 mesh
    G3.add_edges_from([
        (1, 2), (1, 4),
        (2, 3), (2, 5),
        (3, 6),
        (4, 5), (4, 7),
        (5, 6), (5, 8),
        (6, 9),
        (7, 8), (8, 9)
    ])
    examples['mesh_network'] = {
        'graph': G3,
        'title': 'Example 3: Mesh Network\n(Low Vulnerability)',
        'description': '3x3 grid - highly redundant'
    }
    
    # Example 4: Star with Backup
    G4 = nx.Graph()
    G4.add_edges_from([
        (1, 5), (2, 5), (3, 5), (4, 5),  # Star connections
        (1, 2), (2, 3), (3, 4), (4, 1)   # Backup ring
    ])
    examples['star_backup_network'] = {
        'graph': G4,
        'title': 'Example 4: Star Network with Ring Backup\n(Medium Vulnerability)',
        'description': 'Central hub with backup connections'
    }
    
    # Example 5: Complex Real-World-like Network
    G5 = nx.Graph()
    G5.add_edges_from([
        (1, 2), (2, 3), (3, 4), (4, 5),
        (2, 6), (6, 7), (7, 8),
        (4, 9), (9, 10),
        (3, 6), (5, 8), (7, 9)
    ])
    examples['complex_network'] = {
        'graph': G5,
        'title': 'Example 5: Complex Network\n(Variable Vulnerability)',
        'description': 'Mixed topology with multiple paths'
    }
    
    return examples


def visualize_single_network(graph, title, example_name):
    """Create detailed visualization for a single network"""
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle(title, fontsize=14, fontweight='bold', y=0.98)
    
    # Left plot: Show network with critical edges
    algo = DistributedEdgeConnectivity(graph)
    results = algo.run()
    metrics = algo.get_robustness_metrics()
    
    plot_network(axes[0], graph, 'Network with Critical Edges Highlighted',
                critical_edges=results['critical_edges'], layout='spring')
    
    # Add legend for left plot
    red_line = mpatches.Patch(color='red', label='Critical Edges (Bridges)')
    black_line = mpatches.Patch(color='black', label='Regular Edges')
    axes[0].legend(handles=[red_line, black_line], loc='upper left', fontsize=9)
    
    # Right plot: Metrics visualization
    axes[1].axis('off')
    
    # Display metrics as text
    metrics_text = f"""
ALGORITHM RESULTS: {example_name}

Network Statistics:
  • Total Nodes: {results['total_nodes']}
  • Total Edges: {results['total_edges']}
  • Critical Edges (Bridges): {results['bridge_count']}

Robustness Metrics:
  • Vulnerability Score: {metrics['vulnerability']:.4f}
    (0 = robust, 1 = vulnerable)
  
  • Average Edge Connectivity: {metrics['average_edge_connectivity']:.4f}
  
  • Minimum Edge Connectivity: {metrics['minimum_edge_connectivity']}
    (Network k-connectivity)
  
  • Maximum Edge Connectivity: {metrics['maximum_edge_connectivity']}

Analysis:
"""
    
    # Add interpretation
    if metrics['vulnerability'] < 0.2:
        interpretation = "  ✓ ROBUST network with good redundancy"
    elif metrics['vulnerability'] < 0.5:
        interpretation = "  ~ MODERATE network, some bottlenecks"
    else:
        interpretation = "  ✗ VULNERABLE network, many critical edges"
    
    metrics_text += interpretation
    
    # Critical edges info
    if results['critical_edges']:
        metrics_text += f"\n\nCritical Edges:\n"
        for i, edge in enumerate(sorted(results['critical_edges']), 1):
            metrics_text += f"  {i}. Edge {edge}\n"
    else:
        metrics_text += "\n\nCritical Edges: NONE (Highly robust!)\n"
    
    axes[1].text(0.05, 0.95, metrics_text, transform=axes[1].transAxes,
                fontsize=10, verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))
    
    plt.tight_layout()
    
    return fig


def compare_all_examples():
    """Create comparison plot of all example networks"""
    
    examples = create_example_networks()
    
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('Distributed Edge Connectivity: Example Networks from CDC 2024 Paper',
                fontsize=16, fontweight='bold', y=0.995)
    
    axes = axes.flatten()
    
    results_summary = []
    
    for idx, (example_name, example_data) in enumerate(examples.items()):
        G = example_data['graph']
        
        # Run algorithm
        algo = DistributedEdgeConnectivity(G)
        results = algo.run()
        metrics = algo.get_robustness_metrics()
        
        # Plot network
        plot_network(axes[idx], G, example_data['title'],
                    critical_edges=results['critical_edges'], layout='spring')
        
        # Store for summary
        results_summary.append({
            'name': example_name,
            'nodes': results['total_nodes'],
            'edges': results['total_edges'],
            'bridges': results['bridge_count'],
            'vulnerability': metrics['vulnerability'],
            'connectivity': metrics['minimum_edge_connectivity']
        })
    
    # Use the 6th subplot for legend and summary
    axes[5].axis('off')
    
    summary_text = "SUMMARY TABLE\n" + "="*50 + "\n\n"
    summary_text += f"{'Network':<20} {'Nodes'} {'Edges'} {'Bridges'} {'Vuln.'}\n"
    summary_text += "-"*50 + "\n"
    
    for r in results_summary:
        summary_text += f"{r['name']:<20} {r['nodes']:<5} {r['edges']:<5} {r['bridges']:<7} {r['vulnerability']:.3f}\n"
    
    summary_text += "\n" + "="*50
    summary_text += "\n\nLegend:\n"
    summary_text += "RED edges = Critical edges (bridges)\n"
    summary_text += "BLACK edges = Regular edges\n"
    summary_text += "TEAL nodes = Network nodes\n"
    summary_text += "\nVulnerability: 0=Robust, 1=Vulnerable"
    
    axes[5].text(0.05, 0.95, summary_text, transform=axes[5].transAxes,
                fontsize=9, verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.3))
    
    plt.tight_layout()
    return fig, results_summary


def plot_robustness_comparison():
    """Create a comparison plot of robustness metrics across networks"""
    
    examples = create_example_networks()
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Network Robustness Analysis Across Examples',
                fontsize=14, fontweight='bold')
    
    example_names = []
    vulnerabilities = []
    connectivities = []
    bridge_ratios = []
    node_counts = []
    
    for example_name, example_data in examples.items():
        G = example_data['graph']
        algo = DistributedEdgeConnectivity(G)
        results = algo.run()
        metrics = algo.get_robustness_metrics()
        
        example_names.append(example_name.replace('_network', '').title())
        vulnerabilities.append(metrics['vulnerability'])
        connectivities.append(metrics['minimum_edge_connectivity'])
        bridge_ratios.append(results['bridge_count'] / results['total_edges'] if results['total_edges'] > 0 else 0)
        node_counts.append(results['total_nodes'])
    
    # Plot 1: Vulnerability comparison
    colors = ['red' if v > 0.6 else 'orange' if v > 0.3 else 'green' for v in vulnerabilities]
    axes[0, 0].bar(range(len(example_names)), vulnerabilities, color=colors, alpha=0.7)
    axes[0, 0].set_ylabel('Vulnerability Score', fontweight='bold')
    axes[0, 0].set_title('Network Vulnerability', fontweight='bold')
    axes[0, 0].set_xticks(range(len(example_names)))
    axes[0, 0].set_xticklabels(example_names, rotation=45, ha='right')
    axes[0, 0].set_ylim([0, 1])
    axes[0, 0].axhline(y=0.5, color='orange', linestyle='--', alpha=0.5, label='Moderate threshold')
    axes[0, 0].legend()
    axes[0, 0].grid(axis='y', alpha=0.3)
    
    # Plot 2: Edge connectivity
    axes[0, 1].bar(range(len(example_names)), connectivities, color='skyblue', alpha=0.7)
    axes[0, 1].set_ylabel('Edge Connectivity (k)', fontweight='bold')
    axes[0, 1].set_title('Network Edge Connectivity', fontweight='bold')
    axes[0, 1].set_xticks(range(len(example_names)))
    axes[0, 1].set_xticklabels(example_names, rotation=45, ha='right')
    axes[0, 1].grid(axis='y', alpha=0.3)
    
    # Plot 3: Bridge ratio
    axes[1, 0].bar(range(len(example_names)), bridge_ratios, color='coral', alpha=0.7)
    axes[1, 0].set_ylabel('Bridge Ratio', fontweight='bold')
    axes[1, 0].set_title('Fraction of Critical Edges', fontweight='bold')
    axes[1, 0].set_xticks(range(len(example_names)))
    axes[1, 0].set_xticklabels(example_names, rotation=45, ha='right')
    axes[1, 0].set_ylim([0, 1])
    axes[1, 0].grid(axis='y', alpha=0.3)
    
    # Plot 4: Vulnerability vs Connectivity scatter
    axes[1, 1].scatter(vulnerabilities, connectivities, s=200, alpha=0.6, c=range(len(example_names)), cmap='viridis')
    for i, name in enumerate(example_names):
        axes[1, 1].annotate(name, (vulnerabilities[i], connectivities[i]), 
                           fontsize=8, ha='right', va='bottom')
    axes[1, 1].set_xlabel('Vulnerability Score', fontweight='bold')
    axes[1, 1].set_ylabel('Edge Connectivity (k)', fontweight='bold')
    axes[1, 1].set_title('Vulnerability vs Connectivity', fontweight='bold')
    axes[1, 1].grid(alpha=0.3)
    axes[1, 1].set_xlim([-0.1, 1.1])
    
    plt.tight_layout()
    return fig


def main():
    """Main execution"""
    
    print("=" * 70)
    print("DISTRIBUTED EDGE CONNECTIVITY - VISUALIZATION")
    print("=" * 70)
    
    examples = create_example_networks()
    
    # Create individual plots for each example
    print("\nGenerating individual network plots...")
    for example_name, example_data in examples.items():
        print(f"  • {example_name}")
        G = example_data['graph']
        fig = visualize_single_network(G, example_data['title'], example_name.title())
        fig.savefig(f'{example_name}_visualization.png', dpi=150, bbox_inches='tight')
        plt.close(fig)
    
    # Create comparison plot
    print("\nGenerating comparison plot...")
    fig, summary = compare_all_examples()
    fig.savefig('all_examples_comparison.png', dpi=150, bbox_inches='tight')
    plt.close(fig)
    
    # Create robustness analysis plot
    print("Generating robustness analysis plot...")
    fig = plot_robustness_comparison()
    fig.savefig('robustness_analysis.png', dpi=150, bbox_inches='tight')
    plt.close(fig)
    
    print("\n" + "=" * 70)
    print("PLOTS GENERATED SUCCESSFULLY!")
    print("=" * 70)
    print("\nGenerated files:")
    print("  1. path_network_visualization.png")
    print("  2. bottleneck_network_visualization.png")
    print("  3. mesh_network_visualization.png")
    print("  4. star_backup_network_visualization.png")
    print("  5. complex_network_visualization.png")
    print("  6. all_examples_comparison.png")
    print("  7. robustness_analysis.png")
    print("\n✓ Open PNG files to view network visualizations!")
    
    # Display summary
    print("\n" + "=" * 70)
    print("ANALYSIS SUMMARY")
    print("=" * 70)
    for example_name, example_data in examples.items():
        G = example_data['graph']
        algo = DistributedEdgeConnectivity(G)
        results = algo.run()
        metrics = algo.get_robustness_metrics()
        
        print(f"\n{example_name.upper()}:")
        print(f"  Description: {example_data['description']}")
        print(f"  Nodes/Edges: {results['total_nodes']}/{results['total_edges']}")
        print(f"  Critical Edges: {results['bridge_count']}")
        print(f"  Vulnerability: {metrics['vulnerability']:.4f}")
        print(f"  Edge Connectivity: {metrics['minimum_edge_connectivity']}")

if __name__ == "__main__":
    main()
