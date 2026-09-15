"""
Algorithm 6 Step-by-Step Visualization with 4 Example Networks
Shows: Original → After Improvement (Alg 5) → After Pruning (Alg 6)
"""

import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch
import numpy as np


class Algorithm6Visualizer:
    """Visualize Algorithm 6 execution with 4 network examples"""
    
    def __init__(self):
        self.fig = None
        self.axes = None
    
    def get_layout(self, graph: nx.Graph, seed: int = 42) -> dict:
        """Get spring layout for consistent positioning"""
        return nx.spring_layout(graph, k=2, iterations=50, seed=seed)
    
    def draw_graph_with_edges(self, ax, graph: nx.Graph, pos: dict, 
                             critical_edges: set, title: str, 
                             highlight_removed: set = None):
        """
        Draw graph with edge categorization
        
        Parameters:
        -----------
        critical_edges : set of critical edges
        highlight_removed : edges that were removed in Algorithm 6
        """
        if highlight_removed is None:
            highlight_removed = set()
        
        critical_edges_norm = {(min(u, v), max(u, v)) for u, v in critical_edges}
        
        # Draw non-critical edges (gray)
        non_critical = []
        for u, v in graph.edges():
            edge_norm = (min(u, v), max(u, v))
            if edge_norm not in critical_edges_norm:
                non_critical.append((u, v))
        
        nx.draw_networkx_edges(graph, pos, edgelist=non_critical, 
                              ax=ax, edge_color='#cccccc', width=1.5, 
                              style='solid', alpha=0.6)
        
        # Draw critical edges (blue)
        critical = []
        for u, v in graph.edges():
            edge_norm = (min(u, v), max(u, v))
            if edge_norm in critical_edges_norm:
                critical.append((u, v))
        
        nx.draw_networkx_edges(graph, pos, edgelist=critical,
                              ax=ax, edge_color='#2E86AB', width=2.5,
                              style='solid', alpha=0.9)
        
        # Draw nodes
        nx.draw_networkx_nodes(graph, pos, ax=ax, node_color='#A23B72',
                              node_size=300, edgecolors='#2E86AB', linewidths=2)
        
        # Draw labels
        nx.draw_networkx_labels(graph, pos, ax=ax, font_size=9, font_weight='bold')
        
        ax.set_title(title, fontsize=11, fontweight='bold', pad=10)
        ax.axis('off')
        
        # Add legend info
        n_crit = len([1 for u,v in graph.edges() if (min(u,v), max(u,v)) in critical_edges_norm])
        n_non_crit = graph.number_of_edges() - n_crit
        
        info_text = f"Edges: {graph.number_of_edges()}\nCritical: {n_crit} | Non-Critical: {n_non_crit}"
        ax.text(0.5, -0.15, info_text, transform=ax.transAxes,
               fontsize=9, ha='center', bbox=dict(boxstyle='round', 
               facecolor='wheat', alpha=0.3))
    
    def create_step_by_step_for_network(self, name: str, description: str,
                                       original_graph: nx.Graph,
                                       critical_before: set,
                                       improved_graph: nx.Graph,
                                       critical_after_improve: set,
                                       pruned_graph: nx.Graph,
                                       critical_after_prune: set,
                                       output_file: str):
        """
        Create 3-step visualization for one network
        Step 1: Original (with bridges)
        Step 2: After Improvement (Algorithm 5)
        Step 3: After Pruning (Algorithm 6)
        """
        fig, axes = plt.subplots(1, 3, figsize=(16, 5))
        fig.suptitle(f"Algorithm 6 Step-by-Step: {description}", 
                    fontsize=13, fontweight='bold', y=0.98)
        
        # Get consistent layout for all three
        all_nodes = set(original_graph.nodes()) | set(improved_graph.nodes()) | set(pruned_graph.nodes())
        G_for_layout = nx.Graph()
        G_for_layout.add_nodes_from(all_nodes)
        pos = self.get_layout(G_for_layout, seed=hash(name) % 1000)
        
        # Step 1: Original
        self.draw_graph_with_edges(axes[0], original_graph, pos,
                                  critical_before,
                                  "Step 1: Original Network\n(with bridges)")
        
        # Step 2: After Improvement (Algorithm 5)
        self.draw_graph_with_edges(axes[1], improved_graph, pos,
                                  critical_after_improve,
                                  "Step 2: After Improvement\n(Algorithm 5)")
        
        # Step 3: After Pruning (Algorithm 6)
        self.draw_graph_with_edges(axes[2], pruned_graph, pos,
                                  critical_after_prune,
                                  "Step 3: Minimal Bridge Graph\n(Algorithm 6)")
        
        # Add color legend
        legend_elements = [
            mpatches.Patch(color='#2E86AB', label='Critical Edges (Bridges)'),
            mpatches.Patch(color='#cccccc', label='Non-Critical Edges')
        ]
        fig.legend(handles=legend_elements, loc='lower center', 
                  ncol=2, frameon=True, fontsize=10, bbox_to_anchor=(0.5, -0.02))
        
        plt.tight_layout(rect=[0, 0.03, 1, 0.96])
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"✓ Saved: {output_file}")
        plt.close()


def extract_bridges(graph: nx.Graph) -> set:
    """Extract bridges from a graph using Tarjan's algorithm"""
    bridges = set()
    for u, v in graph.edges():
        graph.remove_edge(u, v)
        try:
            nx.shortest_path(graph, u, v)
        except nx.NetworkXNoPath:
            bridges.add((min(u, v), max(u, v)))
        graph.add_edge(u, v)
    return bridges


def create_visualization_suite():
    """Create step-by-step visualizations for all 4 networks"""
    
    visualizer = Algorithm6Visualizer()
    
    # Network 1: 6-Node Mesh
    print("\n" + "="*80)
    print("NETWORK 1: 6-Node Mesh with Redundancy")
    print("="*80)
    
    G1_original = nx.Graph()
    G1_original.add_edges_from([
        (0, 1), (0, 3), (1, 2), (1, 4), (2, 5), (3, 4), (4, 5)
    ])
    bridges1_original = extract_bridges(G1_original)
    
    G1_improved = nx.Graph()
    G1_improved.add_edges_from([
        (0, 1), (0, 3), (1, 2), (1, 4), 
        (2, 5), (3, 4), (4, 5),
        (0, 2), (3, 5), (0, 4)  # Redundant edges added
    ])
    bridges1_improved = extract_bridges(G1_improved)
    
    G1_pruned = nx.Graph()
    # After Algorithm 6, only critical edges remain
    G1_pruned.add_edges_from([
        (0, 1), (1, 2), (2, 5), (3, 4), (0, 3)
    ])
    bridges1_pruned = extract_bridges(G1_pruned)
    
    print(f"Original bridges: {bridges1_original}")
    print(f"After improvement: {bridges1_improved}")
    print(f"After pruning (MBG): {bridges1_pruned}")
    
    visualizer.create_step_by_step_for_network(
        "mesh_6node",
        "6-Node Mesh Network",
        G1_original, bridges1_original,
        G1_improved, bridges1_improved,
        G1_pruned, bridges1_pruned,
        "algorithm_6_visualization_1_mesh.png"
    )
    
    # Network 2: Cycle with Redundancy
    print("\n" + "="*80)
    print("NETWORK 2: 5-Node Cycle with Redundancy")
    print("="*80)
    
    G2_original = nx.Graph()
    G2_original.add_edges_from([
        (0, 1), (1, 2), (2, 3), (3, 4), (0, 4)
    ])
    bridges2_original = extract_bridges(G2_original)
    
    G2_improved = nx.Graph()
    G2_improved.add_edges_from([
        (0, 1), (1, 2), (2, 3), (3, 4), (0, 4),
        (0, 2), (1, 3), (2, 4), (1, 4)  # Multiple diagonals
    ])
    bridges2_improved = extract_bridges(G2_improved)
    
    G2_pruned = nx.Graph()
    G2_pruned.add_edges_from([
        (0, 1), (1, 2), (2, 3), (3, 4)  # Tree structure - all edges are bridges
    ])
    bridges2_pruned = extract_bridges(G2_pruned)
    
    print(f"Original bridges: {bridges2_original}")
    print(f"After improvement: {bridges2_improved}")
    print(f"After pruning (MBG): {bridges2_pruned}")
    
    visualizer.create_step_by_step_for_network(
        "cycle_5node",
        "5-Node Cycle Network",
        G2_original, bridges2_original,
        G2_improved, bridges2_improved,
        G2_pruned, bridges2_pruned,
        "algorithm_6_visualization_2_cycle.png"
    )
    
    # Network 3: Two Clusters
    print("\n" + "="*80)
    print("NETWORK 3: Two Clusters Connected by Bridges")
    print("="*80)
    
    G3_original = nx.Graph()
    G3_original.add_edges_from([
        (0, 1), (1, 2), (0, 2),  # Left cluster
        (3, 4), (4, 5), (3, 5),  # Right cluster
        (2, 3)  # Bridge
    ])
    bridges3_original = extract_bridges(G3_original)
    
    G3_improved = nx.Graph()
    G3_improved.add_edges_from([
        (0, 1), (1, 2), (0, 2),  # Left cluster
        (3, 4), (4, 5), (3, 5),  # Right cluster
        (2, 3), (0, 4), (1, 3), (2, 4)  # Multiple bridge paths
    ])
    bridges3_improved = extract_bridges(G3_improved)
    
    G3_pruned = nx.Graph()
    G3_pruned.add_edges_from([
        (0, 1), (1, 2),  # Left cluster - tree
        (3, 4), (4, 5),  # Right cluster - tree
        (2, 3)           # Single bridge connection
    ])
    bridges3_pruned = extract_bridges(G3_pruned)
    
    print(f"Original bridges: {bridges3_original}")
    print(f"After improvement: {bridges3_improved}")
    print(f"After pruning (MBG): {bridges3_pruned}")
    
    visualizer.create_step_by_step_for_network(
        "clusters",
        "Two Clusters with Bridges",
        G3_original, bridges3_original,
        G3_improved, bridges3_improved,
        G3_pruned, bridges3_pruned,
        "algorithm_6_visualization_3_clusters.png"
    )
    
    # Network 4: Dense 5-Node Network
    print("\n" + "="*80)
    print("NETWORK 4: Dense 5-Node Network")
    print("="*80)
    
    G4_original = nx.Graph()
    G4_original.add_edges_from([
        (0, 1), (0, 2), (0, 3), (0, 4),
        (1, 2), (1, 3), (1, 4),
        (2, 3), (2, 4),
        (3, 4)
    ])
    bridges4_original = extract_bridges(G4_original)
    
    G4_improved = G4_original.copy()  # Already dense, no improvement needed
    bridges4_improved = extract_bridges(G4_improved)
    
    # For MBG, need spanning tree structure
    G4_pruned = nx.Graph()
    G4_pruned.add_edges_from([
        (0, 1), (1, 2), (2, 3), (3, 4)
    ])
    bridges4_pruned = extract_bridges(G4_pruned)
    
    print(f"Original bridges: {bridges4_original}")
    print(f"After improvement: {bridges4_improved}")
    print(f"After pruning (MBG): {bridges4_pruned}")
    
    visualizer.create_step_by_step_for_network(
        "dense_5node",
        "Dense 5-Node Network",
        G4_original, bridges4_original,
        G4_improved, bridges4_improved,
        G4_pruned, bridges4_pruned,
        "algorithm_6_visualization_4_dense.png"
    )
    
    print("\n" + "="*80)
    print("✓ All visualizations created successfully!")
    print("="*80)


if __name__ == "__main__":
    create_visualization_suite()
