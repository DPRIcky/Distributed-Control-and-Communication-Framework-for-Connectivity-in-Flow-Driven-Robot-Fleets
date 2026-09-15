"""
QUICK DEMO: Real-Time Edge Pruning Visualization
=================================================

Simple example showing how the pruning works in real-time
"""

import networkx as nx
import matplotlib.pyplot as plt
import numpy as np

class SimplePruningDemo:
    """Demonstrate real-time pruning on a sample graph"""
    
    def __init__(self):
        # Create a more realistic network graph with some critical edges
        self.G = nx.Graph()
        self.G.add_edges_from([
            # Backbone (critical)
            (0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6),
            # Short cuts (redundant)
            (0, 2), (2, 4), (1, 3), (3, 5)
        ])
        
        self.pos = nx.spring_layout(self.G, seed=42)
    
    def find_critical_edges(self):
        """Find critical edges (bridges) in the graph"""
        G = self.G.copy()
        bridges = set()
        
        for u, v in G.edges():
            G.remove_edge(u, v)
            try:
                nx.shortest_path(G, u, v)
            except nx.NetworkXNoPath:
                bridges.add((min(u, v), max(u, v)))
            G.add_edge(u, v)
        
        return bridges
    
    def run_demo(self):
        """Run the pruning demo"""
        print("\n" + "="*70)
        print("REAL-TIME EDGE PRUNING DEMONSTRATION")
        print("="*70)
        
        # Find critical edges
        critical = self.find_critical_edges()
        print(f"\nGraph Analysis:")
        print(f"  • Total edges: {self.G.number_of_edges()}")
        print(f"  • Critical edges (bridges): {len(critical)}")
        print(f"  • Redundant edges: {self.G.number_of_edges() - len(critical)}")
        print(f"\nCritical edges: {critical}")
        
        # Create visualization with 4 steps
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle("Real-Time Edge Pruning to Minimal Bridge Graph (MBG)", 
                     fontsize=14, fontweight='bold')
        
        # Step 1: Original graph
        ax = axes[0, 0]
        ax.set_title("Step 1: Original Network", fontweight='bold')
        nx.draw_networkx_nodes(self.G, self.pos, ax=ax, node_color='#e74c3c', 
                              node_size=600, edgecolors='white', linewidths=2)
        
        # Draw all edges
        nx.draw_networkx_edges(self.G, self.pos, ax=ax, width=2, alpha=0.6)
        
        # Highlight critical edges
        nx.draw_networkx_edges(self.G, self.pos, edgelist=critical, ax=ax,
                              width=3, edge_color='red', alpha=0.9)
        
        nx.draw_networkx_labels(self.G, self.pos, ax=ax, font_size=10, font_weight='bold')
        ax.text(0.02, 0.98, f"Edges: {self.G.number_of_edges()}\nCritical: {len(critical)}", 
                   transform=ax.transAxes, fontsize=9, verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))
        ax.axis('off')
        
        # Step 2: Mark redundant edges
        ax = axes[0, 1]
        ax.set_title("Step 2: Identify Redundant Edges", fontweight='bold')
        nx.draw_networkx_nodes(self.G, self.pos, ax=ax, node_color='#e74c3c',
                              node_size=600, edgecolors='white', linewidths=2)
        
        # Draw critical edges in red
        nx.draw_networkx_edges(self.G, self.pos, edgelist=critical, ax=ax,
                              width=3, edge_color='red', alpha=0.9, label='Critical')
        
        # Draw redundant edges  
        redundant = set((min(u,v), max(u,v)) for u,v in self.G.edges()) - critical
        redundant_list = [(u, v) for u, v in self.G.edges() 
                         if (min(u,v), max(u,v)) in redundant]
        nx.draw_networkx_edges(self.G, self.pos, edgelist=redundant_list, ax=ax,
                              width=2, edge_color='orange', style='--', alpha=0.8,
                              label='Redundant')
        
        nx.draw_networkx_labels(self.G, self.pos, ax=ax, font_size=10, font_weight='bold')
        ax.legend(loc='upper left', fontsize=8)
        ax.text(0.02, 0.98, f"Total: {self.G.number_of_edges()}\n" + 
                   f"Critical: {len(critical)}\nRedundant: {len(redundant)}", 
                   transform=ax.transAxes, fontsize=9, verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
        ax.axis('off')
        
        # Step 3: Pruning in progress
        ax = axes[1, 0]
        ax.set_title("Step 3: Pruning in Progress (50%)", fontweight='bold')
        
        G_partial = self.G.copy()
        # Remove half the redundant edges
        redundant_list = list(redundant)
        for edge in redundant_list[:len(redundant_list)//2]:
            G_partial.remove_edge(edge[0], edge[1])
        
        nx.draw_networkx_nodes(G_partial, self.pos, ax=ax, node_color='#e74c3c',
                              node_size=600, edgecolors='white', linewidths=2)
        
        # Draw remaining edges
        remaining_edges = list(G_partial.edges())
        remaining_critical = [e for e in critical if G_partial.has_edge(e[0], e[1]) or 
                            G_partial.has_edge(e[1], e[0])]
        
        removed = set(redundant_list[:len(redundant_list)//2])
        remaining_redundant = [e for e in remaining_edges if (min(e[0], e[1]), max(e[0], e[1])) in 
                             (redundant - removed)]
        
        # Critical
        nx.draw_networkx_edges(G_partial, self.pos, edgelist=remaining_critical, ax=ax,
                              width=3, edge_color='red', alpha=0.9, label='Critical')
        # Still to remove
        nx.draw_networkx_edges(G_partial, self.pos, edgelist=remaining_redundant, ax=ax,
                              width=2, edge_color='orange', style='--', alpha=0.8, label='To remove')
        # Already removed
        removed_edges = [(u, v) for u, v in self.G.edges() if (min(u,v), max(u,v)) in removed]
        nx.draw_networkx_edges(self.G, self.pos, edgelist=removed_edges, ax=ax,
                              width=1, edge_color='gray', style=':', alpha=0.4, label='Removed')
        
        nx.draw_networkx_labels(G_partial, self.pos, ax=ax, font_size=10, font_weight='bold')
        ax.legend(loc='upper left', fontsize=8)
        ax.text(0.02, 0.98, f"Edges: {G_partial.number_of_edges()}\n" + 
                   f"Removed: {len(removed)}/{len(redundant)}\nProgress: 50%", 
                   transform=ax.transAxes, fontsize=9, verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
        ax.axis('off')
        
        # Step 4: Complete - MBG
        ax = axes[1, 1]
        ax.set_title("Step 4: Complete - Minimal Bridge Graph (MBG)!", fontweight='bold')
        
        G_final = self.G.copy()
        for edge in redundant:
            G_final.remove_edge(edge[0], edge[1])
        
        nx.draw_networkx_nodes(G_final, self.pos, ax=ax, node_color='#3498db',
                              node_size=600, edgecolors='white', linewidths=2)
        
        # Only critical edges remain
        final_edges = list(G_final.edges())
        nx.draw_networkx_edges(G_final, self.pos, edgelist=final_edges, ax=ax,
                              width=3.5, edge_color='red', alpha=0.95, label='MBG')
        
        nx.draw_networkx_labels(G_final, self.pos, ax=ax, font_size=10, font_weight='bold')
        ax.legend(loc='upper left', fontsize=8)
        ax.text(0.02, 0.98, f"✓ Complete!\nEdges: {G_final.number_of_edges()}\n" + 
                   f"All {len(redundant)} redundant\nedges removed!", 
                   transform=ax.transAxes, fontsize=9, verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.9))
        ax.axis('off')
        
        plt.tight_layout()
        
        print("\n" + "-"*70)
        print("PRUNING SEQUENCE:")
        print("-"*70)
        print(f"1. Original network has {self.G.number_of_edges()} edges")
        print(f"2. Identified {len(critical)} critical edges (red)")
        print(f"3. Marked {len(redundant)} redundant edges (orange) for removal")
        print(f"4. PRUNING STARTED: Removing edges one by one...")
        print(f"5. COMPLETE: Reached Minimal Bridge Graph with {len(critical)} edges")
        print(f"\nNetwork Reduction: {self.G.number_of_edges()} edges (critical: {len(critical)}) " + 
              f"({100*(1-len(critical)/self.G.number_of_edges()):.1f}% reduction)")
        print("\n" + "="*70)
        
        plt.show()


if __name__ == "__main__":
    demo = SimplePruningDemo()
    demo.run_demo()
