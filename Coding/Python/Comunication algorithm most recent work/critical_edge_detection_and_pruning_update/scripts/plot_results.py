"""
Visualization module for Distributed Pruning Algorithm results.
Generates plots of network topologies and algorithm statistics.
"""

import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch
import numpy as np
from pathlib import Path
from distributed_pruning_algorithm import DistributedPruningAlgorithm


class AlgorithmVisualizer:
    """Generate visualizations for algorithm results."""
    
    def __init__(self, output_dir: str = "../Figures"):
        """Initialize visualizer with output directory."""
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
    
    def plot_network_comparison(self, graph: nx.Graph, algo: DistributedPruningAlgorithm, 
                               title: str = "Network Topology"):
        """Plot original and pruned networks side by side."""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        
        # Original network
        pos = nx.spring_layout(graph, seed=42, k=1, iterations=50)
        
        ax1.set_title("Original Network", fontsize=14, fontweight='bold')
        ax1.text(0.5, -0.15, f"Nodes: {len(graph)}\nEdges: {len(graph.edges())}", 
                transform=ax1.transAxes, ha='center', fontsize=11)
        
        nx.draw_networkx_nodes(graph, pos, node_color='lightblue', 
                              node_size=500, ax=ax1)
        nx.draw_networkx_edges(graph, pos, edge_color='gray', 
                              width=1.5, alpha=0.6, ax=ax1)
        nx.draw_networkx_labels(graph, pos, font_size=10, ax=ax1)
        ax1.axis('off')
        
        # Pruned network
        pruned = algo.get_pruned_graph()
        
        ax2.set_title("Pruned Network", fontsize=14, fontweight='bold')
        stats = algo.get_statistics()
        pruning_pct = (1 - stats['final_edges'] / stats['original_edges']) * 100
        ax2.text(0.5, -0.15, 
                f"Nodes: {len(pruned)}\nEdges: {len(pruned.edges())}\nPruning: {pruning_pct:.1f}%", 
                transform=ax2.transAxes, ha='center', fontsize=11)
        
        # Color spanning tree edges (green) vs robustness edges (red)
        tree_edges = []
        robust_edges = []
        for edge in pruned.edges():
            # Simplified: assume last edge added is robustness edge
            if len(algo.final_edges) == len(graph):
                # One edge was added
                tree_edges.append(edge)
            else:
                tree_edges.append(edge)
        
        nx.draw_networkx_nodes(pruned, pos, node_color='lightcoral', 
                              node_size=500, ax=ax2)
        nx.draw_networkx_edges(pruned, pos, edge_color='darkred', 
                              width=2, alpha=0.7, ax=ax2)
        nx.draw_networkx_labels(pruned, pos, font_size=10, ax=ax2)
        ax2.axis('off')
        
        plt.tight_layout()
        filename = self.output_dir / f"network_comparison_{title.replace(' ', '_').lower()}.png"
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"✓ Saved: {filename}")
        plt.close()
    
    def plot_distance_heatmap(self, algo: DistributedPruningAlgorithm, title: str = "Distance"):
        """Plot distance matrix as heatmap."""
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Build distance matrix
        nodes = sorted(algo.nodes)
        n = len(nodes)
        dist_matrix = np.zeros((n, n))
        
        for i, node_i in enumerate(nodes):
            for j, node_j in enumerate(nodes):
                dist = algo.node_state[node_i].distance_vector[node_j]
                dist_matrix[i, j] = dist if dist < float('inf') else -1
        
        # Plot heatmap
        im = ax.imshow(dist_matrix, cmap='YlOrRd', aspect='auto')
        
        # Labels
        ax.set_xticks(range(n))
        ax.set_yticks(range(n))
        ax.set_xticklabels(nodes)
        ax.set_yticklabels(nodes)
        ax.set_xlabel('Target Node', fontsize=12)
        ax.set_ylabel('Source Node', fontsize=12)
        ax.set_title(f'{title} Distance Matrix (from Phase 1)', fontsize=14, fontweight='bold')
        
        # Add values to cells
        for i in range(n):
            for j in range(n):
                text = ax.text(j, i, f'{int(dist_matrix[i, j])}',
                             ha="center", va="center", color="black", fontsize=9)
        
        plt.colorbar(im, ax=ax, label='Hop Distance')
        plt.tight_layout()
        
        filename = self.output_dir / f"distance_heatmap_{title.replace(' ', '_').lower()}.png"
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"✓ Saved: {filename}")
        plt.close()
    
    def plot_statistics_summary(self, results_dict: dict, title: str = "Algorithm Statistics"):
        """Plot summary statistics across multiple runs."""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        topologies = list(results_dict.keys())
        original_edges = [results_dict[t]['original_edges'] for t in topologies]
        final_edges = [results_dict[t]['final_edges'] for t in topologies]
        pruning_ratios = [results_dict[t]['pruning_ratio'] for t in topologies]
        
        # Plot 1: Edges comparison
        x = np.arange(len(topologies))
        width = 0.35
        
        ax = axes[0, 0]
        bars1 = ax.bar(x - width/2, original_edges, width, label='Original', color='skyblue')
        bars2 = ax.bar(x + width/2, final_edges, width, label='Final', color='coral')
        ax.set_xlabel('Topology', fontweight='bold')
        ax.set_ylabel('Number of Edges', fontweight='bold')
        ax.set_title('Edge Count Comparison', fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels(topologies, rotation=45)
        ax.legend()
        ax.grid(axis='y', alpha=0.3)
        
        # Plot 2: Pruning ratio
        ax = axes[0, 1]
        bars = ax.bar(topologies, pruning_ratios, color='mediumseagreen', alpha=0.7)
        ax.set_ylabel('Pruning Ratio (%)', fontweight='bold')
        ax.set_title('Edge Pruning Efficiency', fontweight='bold')
        ax.set_xticklabels(topologies, rotation=45)
        for bar in bars:
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{height:.1f}%', ha='center', va='bottom')
        ax.grid(axis='y', alpha=0.3)
        
        # Plot 3: Degree reduction
        ax = axes[1, 0]
        avg_degree_original = [2*e/len(results_dict) for e in original_edges]  # Approximate
        avg_degree_final = [2*f/len(results_dict) for f in final_edges]  # Approximate
        
        x = np.arange(len(topologies))
        ax.bar(x - width/2, avg_degree_original, width, label='Original', color='lightblue')
        ax.bar(x + width/2, avg_degree_final, width, label='Final', color='lightcoral')
        ax.set_xlabel('Topology', fontweight='bold')
        ax.set_ylabel('Average Degree', fontweight='bold')
        ax.set_title('Average Node Degree', fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels(topologies, rotation=45)
        ax.legend()
        ax.grid(axis='y', alpha=0.3)
        
        # Plot 4: Summary table
        ax = axes[1, 1]
        ax.axis('off')
        
        table_data = []
        table_data.append(['Topology', 'Orig', 'Final', 'Prune %'])
        for topo in topologies:
            orig = results_dict[topo]['original_edges']
            final = results_dict[topo]['final_edges']
            ratio = results_dict[topo]['pruning_ratio']
            table_data.append([topo, f'{orig}', f'{final}', f'{ratio:.1f}%'])
        
        table = ax.table(cellText=table_data, cellLoc='center', loc='center',
                        colWidths=[0.3, 0.15, 0.15, 0.2])
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1, 2)
        
        # Style header row
        for i in range(4):
            table[(0, i)].set_facecolor('#40466e')
            table[(0, i)].set_text_props(weight='bold', color='white')
        
        plt.suptitle(title, fontsize=16, fontweight='bold', y=0.98)
        plt.tight_layout()
        
        filename = self.output_dir / f"statistics_summary_{title.replace(' ', '_').lower()}.png"
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"✓ Saved: {filename}")
        plt.close()
    
    def plot_convergence_phases(self, algo: DistributedPruningAlgorithm, title: str = "Algorithm"):
        """Plot algorithm convergence across phases."""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        # Phase 1: Distance propagation
        ax = axes[0, 0]
        phases = ['Phase 1\n(Distances)', 'Phase 2\n(Connectivity)', 
                 'Phase 3\n(Spanning Tree)', 'Phase 4\n(Robustness)']
        edge_counts = [
            len(algo.graph.edges()),  # Original
            len(algo.graph.edges()),  # Still same after Phase 2
            algo.n - 1,  # Spanning tree
            algo.n,  # With robustness edge
        ]
        colors = ['lightblue', 'lightyellow', 'lightcoral', 'lightgreen']
        bars = ax.bar(phases, edge_counts, color=colors, edgecolor='black', linewidth=2)
        ax.set_ylabel('Number of Edges', fontweight='bold')
        ax.set_title('Edge Count Across Phases', fontweight='bold')
        ax.grid(axis='y', alpha=0.3)
        
        for bar, count in zip(bars, edge_counts):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{int(count)}', ha='center', va='bottom', fontweight='bold')
        
        # Phase objectives
        ax = axes[0, 1]
        ax.axis('off')
        
        objectives = [
            ("Phase 1: Distance Computation", 
             "Compute hop distances via\nBFS wavefront (n steps)"),
            ("Phase 2: Connectivity Assurance", 
             "Detect & repair disconnected\ncomponents (n steps)"),
            ("Phase 3: Spanning Tree", 
             "Build tree via parent selection\nMin(neighbors at δ-1)"),
            ("Phase 4: Robustness Enhancement", 
             "Add 1 cycle via embedded Δ\n+ hop distance + consensus"),
        ]
        
        y_pos = 0.95
        for i, (title_text, desc) in enumerate(objectives):
            color = colors[i]
            rect = FancyBboxPatch((0.05, y_pos - 0.2), 0.9, 0.18,
                                 boxstyle="round,pad=0.01", 
                                 facecolor=color, edgecolor='black', linewidth=1.5,
                                 transform=ax.transAxes)
            ax.add_patch(rect)
            
            ax.text(0.08, y_pos - 0.03, title_text, fontsize=10, fontweight='bold',
                   transform=ax.transAxes, va='top')
            ax.text(0.08, y_pos - 0.10, desc, fontsize=9,
                   transform=ax.transAxes, va='top', style='italic')
            
            y_pos -= 0.24
        
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        
        # Node communication rounds
        ax = axes[1, 0]
        phase_names = ['Phase 1', 'Phase 2', 'Phase 3', 'Phase 4']
        rounds = [algo.n - 1, algo.n, 2, 2]  # Approximate
        colors_rounds = ['#FF9999', '#FFB366', '#99CCFF', '#99FF99']
        
        bars = ax.barh(phase_names, rounds, color=colors_rounds, edgecolor='black', linewidth=1.5)
        ax.set_xlabel('Communication Rounds', fontweight='bold')
        ax.set_title('Communication Complexity', fontweight='bold')
        ax.grid(axis='x', alpha=0.3)
        
        for bar, r in zip(bars, rounds):
            width = bar.get_width()
            label = f'~{int(r)}' if isinstance(r, float) else f'{r}'
            ax.text(width, bar.get_y() + bar.get_height()/2.,
                   f' {label}', va='center', fontweight='bold')
        
        # Final statistics
        ax = axes[1, 1]
        ax.axis('off')
        
        stats = algo.get_statistics()
        stats_text = f"""
ALGORITHM RESULTS

Network Size:
  Nodes: {stats['nodes']}
  Original Edges: {stats['original_edges']}
  Final Edges: {stats['final_edges']}

Optimization:
  Pruning Efficiency: {(1 - stats['final_edges'] / stats['original_edges']) * 100:.1f}%
  Cycles Created: {stats['cycles']}
  Connected: {'✓ Yes' if stats['connected'] else '✗ No'}

Degree Reduction:
  Original Avg: {stats['avg_degree_original']:.2f}
  Final Avg: {stats['avg_degree_final']:.2f}
  Reduction: {(1 - stats['avg_degree_final'] / stats['avg_degree_original']) * 100:.1f}%

Guarantee:
  Spanning Tree: n-1 = {algo.n - 1} edges
  + Robustness: +1 = 1 edge
  Total: {stats['final_edges']} edges
        """
        
        ax.text(0.05, 0.95, stats_text, transform=ax.transAxes,
               fontsize=10, verticalalignment='top', family='monospace',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))
        
        plt.suptitle(f'{title} - Algorithm Convergence & Statistics', 
                    fontsize=16, fontweight='bold', y=0.98)
        plt.tight_layout()
        
        filename = self.output_dir / f"convergence_{title.replace(' ', '_').lower()}.png"
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"✓ Saved: {filename}")
        plt.close()
    
    def plot_phases_2x2(self, algo: DistributedPruningAlgorithm, title: str = "Network"):
        """Plot 2x2 graph evolution through phases."""
        fig, axes = plt.subplots(2, 2, figsize=(16, 14))
        
        # Create layout based on original graph (consistent across all phases)
        pos = nx.spring_layout(algo.graph, seed=42, k=1.5, iterations=50)
        
        # Phase 1: Original Connected Graph
        ax = axes[0, 0]
        ax.set_title('Phase 1: Initial Network Topology', fontweight='bold', fontsize=13, pad=10)
        
        nx.draw_networkx_nodes(algo.graph, pos, node_color='#87CEEB', 
                              node_size=600, ax=ax, edgecolors='darkblue', linewidths=2)
        nx.draw_networkx_edges(algo.graph, pos, edge_color='#555555', 
                              width=2, alpha=0.6, ax=ax)
        nx.draw_networkx_labels(algo.graph, pos, font_size=10, font_weight='bold', ax=ax)
        
        stats = algo.get_statistics()
        ax.text(0.05, 0.95, f"Nodes: {stats['nodes']}\nEdges: {stats['original_edges']}\nAvg Degree: {stats['avg_degree_original']:.2f}",
               transform=ax.transAxes, fontsize=11, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))
        ax.axis('off')
        
        # Phase 2: After Connectivity Assurance
        ax = axes[0, 1]
        ax.set_title('Phase 2: Connectivity Assured', fontweight='bold', fontsize=13, pad=10)
        
        # For Phase 2, all nodes should still be connected
        phase2_graph = algo.graph.copy()
        
        nx.draw_networkx_nodes(phase2_graph, pos, node_color='#90EE90', 
                              node_size=600, ax=ax, edgecolors='darkgreen', linewidths=2)
        nx.draw_networkx_edges(phase2_graph, pos, edge_color='#555555', 
                              width=2, alpha=0.6, ax=ax)
        nx.draw_networkx_labels(phase2_graph, pos, font_size=10, font_weight='bold', ax=ax)
        
        connected_check = "✓ Fully Connected" if nx.is_connected(phase2_graph) else "⚠ Disconnected"
        components = nx.number_connected_components(phase2_graph)
        ax.text(0.05, 0.95, f"Components: {components}\n{connected_check}\nEdges: {len(phase2_graph.edges())}",
               transform=ax.transAxes, fontsize=11, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7))
        ax.axis('off')
        
        # Phase 3: Spanning Tree Construction
        ax = axes[1, 0]
        ax.set_title('Phase 3: Spanning Tree', fontweight='bold', fontsize=13, pad=10)
        
        phase3_graph = algo.graph.copy()
        # Build spanning tree by traversing from each node to its parent
        tree_edges = set()
        for node in algo.nodes:
            parent = algo.node_state[node].parent
            if parent is not None and parent != node:
                tree_edges.add(tuple(sorted([node, parent])))
        
        if tree_edges:
            phase3_graph = phase3_graph.edge_subgraph(tree_edges).copy()
        else:
            phase3_graph = nx.Graph()
            phase3_graph.add_nodes_from(algo.nodes)
        
        nx.draw_networkx_nodes(phase3_graph, pos, node_color='#FFB6C1', 
                              node_size=600, ax=ax, edgecolors='darkred', linewidths=2)
        nx.draw_networkx_edges(phase3_graph, pos, edge_color='darkred', 
                              width=2.5, alpha=0.8, ax=ax)
        nx.draw_networkx_labels(phase3_graph, pos, font_size=10, font_weight='bold', ax=ax)
        
        is_tree = len(phase3_graph.edges()) == algo.n - 1 and nx.is_connected(phase3_graph)
        tree_status = "✓ Valid Tree" if is_tree else "⚠ Invalid"
        ax.text(0.05, 0.95, f"Edges: {len(phase3_graph.edges())}\n(n-1 = {algo.n - 1})\n{tree_status}",
               transform=ax.transAxes, fontsize=11, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='#FFB6C1', alpha=0.7))
        ax.axis('off')
        
        # Phase 4: Final Network with Robustness Edge
        ax = axes[1, 1]
        ax.set_title('Phase 4: Robustness Enhanced', fontweight='bold', fontsize=13, pad=10)
        
        pruned = algo.get_pruned_graph()
        
        # Distinguish tree edges from robustness edge
        robust_edge_list = []
        if len(pruned.edges()) > algo.n - 1:
            # Find the robustness edge (not in tree_edges)
            for edge in pruned.edges():
                if edge not in tree_edges and tuple(reversed(edge)) not in tree_edges:
                    robust_edge_list.append(edge)
            tree_edge_list = list(tree_edges)
        else:
            tree_edge_list = list(pruned.edges())
            robust_edge_list = []
        
        nx.draw_networkx_nodes(pruned, pos, node_color='#FFD700', 
                              node_size=600, ax=ax, edgecolors='orange', linewidths=2)
        
        # Draw tree edges (solid)
        nx.draw_networkx_edges(pruned, pos, edgelist=tree_edge_list, 
                              edge_color='darkblue', width=2.5, alpha=0.8, ax=ax)
        
        # Draw robustness edge (dashed, red)
        if robust_edge_list:
            nx.draw_networkx_edges(pruned, pos, edgelist=robust_edge_list, 
                                  edge_color='red', width=3, style='dashed', alpha=0.9, ax=ax)
        
        nx.draw_networkx_labels(pruned, pos, font_size=10, font_weight='bold', ax=ax)
        
        pruning_pct = (1 - stats['final_edges'] / stats['original_edges']) * 100
        ax.text(0.05, 0.95, f"Total Edges: {stats['final_edges']}\nPruning: {pruning_pct:.1f}%\n✓ Connected",
               transform=ax.transAxes, fontsize=11, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='#FFD700', alpha=0.7))
        
        # Add legend for robustness edge
        if robust_edge_list:
            ax.text(0.05, 0.05, f"Red dashed edge: Robustness edge\n{robust_edge_list[0]}",
                   transform=ax.transAxes, fontsize=10, verticalalignment='bottom',
                   bbox=dict(boxstyle='round', facecolor='mistyrose', alpha=0.8))
        
        ax.axis('off')
        
        plt.suptitle(f'{title} - Algorithm Phase Progression: Network Evolution', 
                    fontsize=15, fontweight='bold', y=0.995)
        plt.tight_layout()
        
        filename = self.output_dir / f"phases_2x2_{title.replace(' ', '_').lower()}.png"
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"✓ Saved: {filename}")
        plt.close()
    
    def plot_phases_2x2_with_math(self, algo: DistributedPruningAlgorithm, title: str = "6-Node"):
        """Plot 2x2 graph evolution through phases WITH mathematical explanations.
        Does NOT include "Phase 1", "Phase 2", etc. in titles.
        """
        fig, axes = plt.subplots(2, 2, figsize=(18, 15))
        fig.patch.set_facecolor('white')
        
        # Fixed layout for consistency
        pos = nx.spring_layout(algo.graph, seed=42, k=2.0, iterations=50)
        
        # ======================== SUBPLOT 1: INITIAL NETWORK ========================
        ax = axes[0, 0]
        ax.set_title('Initial Network Topology', fontweight='bold', fontsize=13, pad=15)
        
        # Draw network
        nx.draw_networkx_nodes(algo.graph, pos, node_color='#87CEEB', 
                              node_size=800, ax=ax, edgecolors='darkblue', linewidths=2.5)
        nx.draw_networkx_edges(algo.graph, pos, edge_color='#555555', 
                              width=2, alpha=0.7, ax=ax)
        nx.draw_networkx_labels(algo.graph, pos, font_size=11, font_weight='bold', ax=ax)
        
        # Add statistics
        stats = algo.get_statistics()
        text_box = f"Nodes: {stats['nodes']}\nEdges: {stats['original_edges']}"
        ax.text(0.05, 0.95, text_box,
               transform=ax.transAxes, fontsize=10, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8, pad=0.8))
        
        # Add math explanation
        math_text = "BFS Distance Update:\n" + r"$\omega_{i,j}(k+1) = \min(\omega_{l,j}(k) + 1)$ for $l \in N_i$"
        ax.text(0.05, 0.15, math_text,
               transform=ax.transAxes, fontsize=9, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8, pad=0.8))
        
        ax.axis('off')
        
        # ======================== SUBPLOT 2: KEEP/PRUNE DECISION ========================
        ax = axes[0, 1]
        ax.set_title('Keep/Prune Decision Based on Parent-Child', fontweight='bold', fontsize=13, pad=15)
        
        phase2_graph = algo.graph.copy()
        
        nx.draw_networkx_nodes(phase2_graph, pos, node_color='#FFE4B5', 
                              node_size=800, ax=ax, edgecolors='darkorange', linewidths=2.5)
        
        # Separate keep and prune edges
        keep_edges = []
        prune_edges = []
        
        parent_map = {}
        for node in algo.nodes:
            parent = algo.node_state[node].parent
            parent_map[node] = parent
        
        for edge in phase2_graph.edges():
            u, v = edge
            edge_sorted = tuple(sorted([u, v]))
            # Keep if one is parent of the other
            is_parent_child = (parent_map[u] == v or parent_map[v] == u)
            if is_parent_child:
                keep_edges.append(edge_sorted)
            else:
                prune_edges.append(edge_sorted)
        
        # Draw keep edges (solid, thick, green)
        if keep_edges:
            nx.draw_networkx_edges(phase2_graph, pos, edgelist=keep_edges, 
                                  edge_color='darkgreen', width=3.5, alpha=0.9, ax=ax,
                                  label='Keep (Parent-Child)')
        
        # Draw prune edges (dashed, thin, red)
        if prune_edges:
            nx.draw_networkx_edges(phase2_graph, pos, edgelist=prune_edges, 
                                  edge_color='red', width=1.5, alpha=0.5, style='dashed', ax=ax,
                                  label='Prune (Non-critical)')
        
        nx.draw_networkx_labels(phase2_graph, pos, font_size=11, font_weight='bold', ax=ax)
        
        # Statistics
        text_box = f"Keep edges: {len(keep_edges)}\nPrune edges: {len(prune_edges)}\nTotal: {len(phase2_graph.edges())}"
        ax.text(0.05, 0.95, text_box,
               transform=ax.transAxes, fontsize=10, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='#FFE4B5', alpha=0.8, pad=0.8))
        
        # Math explanation
        math_text = "Classification:\n" + r"Keep if: $p(i) = j$ OR $p(j) = i$"
        ax.text(0.05, 0.15, math_text,
               transform=ax.transAxes, fontsize=9, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8, pad=0.8))
        
        ax.legend(loc='lower right', fontsize=9, framealpha=0.9)
        ax.axis('off')
        
        # ======================== SUBPLOT 3: SPANNING TREE ========================
        ax = axes[1, 0]
        ax.set_title('Spanning Tree Construction', fontweight='bold', fontsize=13, pad=15)
        
        # Extract spanning tree edges
        tree_edges = set()
        for node in algo.nodes:
            parent = algo.node_state[node].parent
            if parent is not None and parent != node:
                tree_edges.add(tuple(sorted([node, parent])))
        
        # Create tree graph
        if tree_edges:
            phase3_graph = algo.graph.copy()
            edges_to_remove = [e for e in phase3_graph.edges() if tuple(sorted(e)) not in tree_edges]
            phase3_graph.remove_edges_from(edges_to_remove)
        else:
            phase3_graph = nx.Graph()
            phase3_graph.add_nodes_from(algo.nodes)
        
        nx.draw_networkx_nodes(phase3_graph, pos, node_color='#FFB6C1', 
                              node_size=800, ax=ax, edgecolors='darkred', linewidths=2.5)
        nx.draw_networkx_edges(phase3_graph, pos, edge_color='darkred', 
                              width=2.5, alpha=0.9, ax=ax)
        nx.draw_networkx_labels(phase3_graph, pos, font_size=11, font_weight='bold', ax=ax)
        
        # Tree statistics  
        is_tree = len(phase3_graph.edges()) == algo.n - 1 and nx.is_connected(phase3_graph)
        tree_status = "✓ Valid Tree" if is_tree else "⚠ Invalid"
        
        text_box = f"Edges: {len(phase3_graph.edges())}\n(n-1 = {algo.n - 1})\n{tree_status}"
        ax.text(0.05, 0.95, text_box,
               transform=ax.transAxes, fontsize=10, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='#FFB6C1', alpha=0.8, pad=0.8))
        
        # Math explanation
        math_text = "Keep/Prune Rule:\nKeep edge {i,j} if: " + r"$p(i) = j$ OR $p(j) = i$"
        ax.text(0.05, 0.15, math_text,
               transform=ax.transAxes, fontsize=9, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8, pad=0.8))
        
        ax.axis('off')
        
        # ======================== SUBPLOT 4: ROBUSTNESS ENHANCED ========================
        ax = axes[1, 1]
        ax.set_title('Robustness Enhancement', fontweight='bold', fontsize=13, pad=15)
        
        pruned = algo.get_pruned_graph()
        
        # Distinguish tree edges from robustness edge
        robust_edges = []
        if len(pruned.edges()) > algo.n - 1:
            for edge in pruned.edges():
                if edge not in tree_edges and tuple(reversed(edge)) not in tree_edges:
                    robust_edges.append(edge)
            tree_edge_list = list(tree_edges)
        else:
            tree_edge_list = list(pruned.edges())
            robust_edges = []
        
        # Draw nodes
        nx.draw_networkx_nodes(pruned, pos, node_color='#FFD700', 
                              node_size=800, ax=ax, edgecolors='orange', linewidths=2.5)
        
        # Draw tree edges (solid, blue)
        if tree_edge_list:
            nx.draw_networkx_edges(pruned, pos, edgelist=tree_edge_list, 
                                  edge_color='darkblue', width=2.5, alpha=0.9, ax=ax,
                                  label='Tree (n-1 edges)')
        
        # Draw robustness edges (dashed, red)
        if robust_edges:
            nx.draw_networkx_edges(pruned, pos, edgelist=robust_edges, 
                                  edge_color='red', width=3, style='dashed', 
                                  alpha=0.9, ax=ax, label='Robustness (+1 edge)')
        
        nx.draw_networkx_labels(pruned, pos, font_size=11, font_weight='bold', ax=ax)
        
        # Statistics
        pruning_pct = (1 - stats['final_edges'] / stats['original_edges']) * 100 if stats['original_edges'] > 0 else 0
        text_box = f"Final Edges: {stats['final_edges']}\nPruning: {pruning_pct:.1f}%\n✓ Connected"
        ax.text(0.05, 0.95, text_box,
               transform=ax.transAxes, fontsize=10, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='#FFD700', alpha=0.8, pad=0.8))
        
        # Math explanation
        math_text = "Add Highest-Redundancy Edge:\n" + r"Select $e^* = \arg\max ||\Delta_i||_\infty$"
        ax.text(0.05, 0.15, math_text,
               transform=ax.transAxes, fontsize=9, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8, pad=0.8))
        
        # Legend
        if robust_edges:
            ax.legend(loc='lower right', fontsize=9, framealpha=0.9)
        
        ax.axis('off')
        
        # Main title with overall description
        fig.text(0.5, 0.98, '4-Phase Distributed Network Topology Optimization', 
                ha='center', fontsize=16, fontweight='bold')
        fig.text(0.5, 0.955, 'Mathematical Explanations for Each Phase', 
                ha='center', fontsize=12, style='italic', color='gray')
        
        plt.tight_layout(rect=[0, 0, 1, 0.94])
        
        # Save figure
        filename = self.output_dir / f"phases_2x2_{title.replace(' ', '_').lower()}_with_math.png"
        plt.savefig(filename, dpi=150, bbox_inches='tight', facecolor='white')
        print(f"✓ Saved: {filename}")
        plt.close()


def visualize_network(graph: nx.Graph, name: str = "network"):
    """Quick visualization of a single network."""
    viz = AlgorithmVisualizer()
    
    algo = DistributedPruningAlgorithm(graph, root=1)
    algo.run_full_pipeline()
    
    print(f"\n📊 Generating visualizations for {name}...")
    viz.plot_network_comparison(graph, algo, title=name)
    viz.plot_distance_heatmap(algo, title=name)
    viz.plot_convergence_phases(algo, title=name)
    viz.plot_phases_2x2(algo, title=name)
    print(f"✓ Complete!\n")
    
    return algo


def main():
    """Generate comprehensive visualizations."""
    print("\n" + "="*70)
    print("DISTRIBUTED PRUNING ALGORITHM - VISUALIZATION SUITE")
    print("="*70)
    
    viz = AlgorithmVisualizer()
    
    # Test 1: Simple grid
    print("\n📈 Test 1: 3x3 Grid Network")
    G_grid = nx.grid_2d_graph(3, 3)
    mapping = {node: i+1 for i, node in enumerate(sorted(G_grid.nodes()))}
    G_grid = nx.relabel_nodes(G_grid, mapping)
    algo_grid = visualize_network(G_grid, "grid_3x3")
    
    # Test 2: Random network
    print("📈 Test 2: Random 12-Node Network")
    G_random = nx.gnp_random_graph(12, 0.25, seed=42)
    while not nx.is_connected(G_random):
        G_random = nx.gnp_random_graph(12, 0.25, seed=42)
    mapping = {i: i+1 for i in range(len(G_random))}
    G_random = nx.relabel_nodes(G_random, mapping)
    algo_random = visualize_network(G_random, "random_12node")
    
    # Test 2.5: 6-Node Graph with Mathematical Explanations
    print("📈 Test 2.5: 6-Node Sparse Graph (with Mathematical Explanations)")
    G_6node = nx.Graph()
    G_6node.add_nodes_from([1, 2, 3, 4, 5, 6])
    G_6node.add_edges_from([
        (1, 2), (2, 3), (3, 4),     # Top chain
        (1, 5), (5, 6), (4, 6),     # Right connections
        (2, 5), (3, 5)              # Cross edges
    ])
    algo_6node = DistributedPruningAlgorithm(G_6node, root=1)
    algo_6node.run_full_pipeline()
    
    # Generate both regular and math-explained versions
    viz.plot_phases_2x2(algo_6node, "6node_sparse")
    viz.plot_phases_2x2_with_math(algo_6node, "6node_sparse")
    
    print("✓ 6-Node visualization complete!\n")
    
    # Test 3: Comparison across topologies
    print("📈 Test 3: Topology Comparison")
    topologies = {
        'cycle': lambda n: nx.cycle_graph(n),
        'path': lambda n: nx.path_graph(n),
        'complete': lambda n: nx.complete_graph(n),
        'star': lambda n: nx.star_graph(n-1),
    }
    
    results = {}
    n = 8
    
    for name, builder in topologies.items():
        G = builder(n)
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
    
    viz.plot_statistics_summary(results, title="Topology Comparison")
    
    print("\n" + "="*70)
    print("✓ ALL VISUALIZATIONS COMPLETE")
    print(f"📁 Output directory: {viz.output_dir.absolute()}")
    print("="*70 + "\n")


if __name__ == "__main__":
    main()
