"""
Simple Test: Validate distributed_critical_mst module works correctly
Includes GUI simulation of MST construction process
"""

import tkinter as tk
from tkinter import ttk
import math
import time
import random
from distributed_critical_mst import build_mst, DistributedCriticalMST

print("="*80)
print("TEST: Distributed Critical MST Module")
print("="*80)

# Test 1: Simple complete graph
print("\nTest 1: Complete Graph K7")
print("-"*80)
n = 7
edges = [(i, j) for i in range(1, n+1) for j in range(i+1, n+1)]
print(f"Input: {len(edges)} edges")

mst = build_mst(n, edges, root_id=1, verbose=False)
print(f"MST: {len(mst)} edges")
assert len(mst) == n-1, f"Expected {n-1} edges, got {len(mst)}"
print("[OK] PASS")

# Test 2: Chain graph (all bridges)
print("\nTest 2: Chain Graph (All Bridges)")
print("-"*80)
n = 10
edges = [(i, i+1) for i in range(1, n)]
print(f"Input: {len(edges)} edges (all bridges)")

system = DistributedCriticalMST(n, edges, verbose=False)
system.build_dfs_mst(root_id=1)
system.prune_to_mst()
mst = system.get_mst_edges()

print(f"MST: {len(mst)} edges")
assert len(mst) == n-1, f"Expected {n-1} edges, got {len(mst)}"
assert mst == set(edges), "All bridges should be in MST"
print("[OK] PASS: All bridges preserved")

# Test 3: Graph with redundant edges
print("\nTest 3: Graph with Redundant Edges")
print("-"*80)
n = 6
edges = [(1,2), (2,3), (3,4), (4,5), (5,6), (1,4), (2,5), (3,6)]
print(f"Input: {len(edges)} edges")

mst = build_mst(n, edges, root_id=1, verbose=False)
print(f"MST: {len(mst)} edges")
print(f"Reduced by: {len(edges) - len(mst)} edges")
assert len(mst) == n-1, f"Expected {n-1} edges, got {len(mst)}"
print("[OK] PASS")

#Test 4: Verbose output test
print("\nTest 4: Verbose Output")
print("-"*80)
n = 5
edges = [(1,2), (2,3), (3,4), (4,5), (1,5)]
print("Running with verbose=True:\n")

mst = build_mst(n, edges, root_id=1, verbose=True)

# Test 5: O(n) complexity verification
print("\n\nTest 5: O(n) Complexity")
print("-"*80)
for n in [5, 10, 15, 20]:
    edges = [(i, j) for i in range(1, n+1) for j in range(i+1, n+1)]
    system = DistributedCriticalMST(n, edges, verbose=False)
    stats = system.build_dfs_mst(root_id=1)
    print(f"n={n:2d}: {stats['rounds']:2d} rounds (≤{n})", end="")
    assert stats['rounds'] <= n, f"Should be O(n), got {stats['rounds']} for n={n}"
    print(" [OK]")

print("\n" + "="*80)
print("ALL TESTS PASSED!")
print("="*80)
print("\nModule is ready to use:")
print("  from distributed_critical_mst import build_mst")
print("  mst_edges = build_mst(n, edges, root_id=1)")
print("="*80)


# ===================================================================
# GUI SIMULATION
# ===================================================================

class MSTSimulationGUI:
    """Interactive GUI showing MST construction process step-by-step."""
    
    def __init__(self, n, edges, root=tk.Tk()):
        self.root = root
        self.root.title("Distributed MST Construction - Live Simulation")
        self.root.geometry("1200x800")
        
        self.n = n
        self.initial_edges = edges
        self.current_step = 0
        self.is_playing = False
        self.animation_speed = 1000  # ms
        
        # Validate input graph connectivity
        adj = {i: [] for i in range(1, n + 1)}
        for u, v in edges:
            adj[u].append(v)
            adj[v].append(u)
        
        visited = {1}
        queue = [1]
        while queue:
            node = queue.pop(0)
            for neighbor in adj[node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        
        if len(visited) != n:
            raise ValueError(f"Input graph is not connected! Only {len(visited)}/{n} nodes reachable from node 1")
        
        print(f"\nGUI: Validated graph connectivity - all {n} nodes reachable")
        
        # Create simulation system
        self.system = DistributedCriticalMST(n, edges, verbose=False)
        
        # Verify all nodes have correct neighbors
        for node_id in range(1, n + 1):
            node = self.system.nodes[node_id]
            if not node.Ni:
                print(f"WARNING: Node {node_id} has no neighbors!")
        
        #Calculate node positions (circular layout)
        self.positions = self._calculate_positions()
        
        # Track MST construction history
        self.history = []
        self._build_history()
        
        # Create GUI
        self._create_widgets()
        self._update_display()
    
    def _calculate_positions(self):
        """Calculate node positions in a circle."""
        positions = {}
        center_x, center_y = 400, 350
        radius = 250
        
        for i in range(1, self.n + 1):
            angle = 2 * math.pi * (i - 1) / self.n - math.pi / 2
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            positions[i] = (x, y)
        
        return positions
    
    def _build_history(self):
        """Build step-by-step history of MST construction."""
        # Step 0: Initial graph
        self.history.append({
            'step': 0,
            'description': 'Initial Connected Graph',
            'edges': set(self.initial_edges),
            'mst_edges': set(),
            'exploring_node': None,
            'tree_nodes': set(),
            'stats': f'Nodes: {self.n}, Edges: {len(self.initial_edges)}'
        })
        
        # Manually run DFS and track each step
        for node in self.system.nodes.values():
            node.dfs_state = "UNVISITED"
            node.dfs_parent = None
            node.dfs_depth = 10**9
            node.dfs_tree_neighbors = set()
            node.dfs_exploring = None
        
        root = self.system.nodes[1]
        root.dfs_state = "EXPLORING"
        root.dfs_depth = 0
        
        tree_nodes = {1}
        mst_edges = set()
        round_num = 0
        max_rounds = self.n * 2  # Safety limit
        
        while len(tree_nodes) < self.n and round_num < max_rounds:
            round_num += 1
            
            # Track state before round
            exploring = [node.id for node in self.system.nodes.values() 
                        if node.dfs_state == "EXPLORING"]
            
            if not exploring:
                # No nodes exploring - Try to recover by restarting from IN_TREE nodes
                recovered = False
                for node in self.system.nodes.values():
                    if node.dfs_state == "IN_TREE":
                        unvisited_neighbors = [n for n in node.Ni 
                                             if self.system.nodes[n].dfs_state == "UNVISITED"]
                        if unvisited_neighbors:
                            node.dfs_state = "EXPLORING"
                            recovered = True
                            exploring = [node.id]
                            break
                
                if not recovered:
                    # No recovery possible - DFS is done or graph is disconnected
                    print(f"\nWARNING: DFS terminated early at round {round_num}")
                    print(f"  Visited {len(tree_nodes)}/{self.n} nodes")
                    print(f"  Tree nodes: {sorted(tree_nodes)}")
                    print(f"  Unreached: {sorted(set(range(1, self.n+1)) - tree_nodes)}")
                    break
            
            # Execute one round
            new_nodes = self.system._dfs_explore_round(prefer_critical=False)
            
            # Update MST edges
            for node in self.system.nodes.values():
                for neighbor_id in node.dfs_tree_neighbors:
                    edge = self.system.canonical_pair(node.id, neighbor_id)
                    mst_edges.add(edge)
            
            # Update tree nodes
            for node in self.system.nodes.values():
                if node.dfs_state in ["EXPLORING", "IN_TREE"]:
                    tree_nodes.add(node.id)
            
            # Record this step
            exploring_node = exploring[0] if exploring else None
            self.history.append({
                'step': round_num,
                'description': f'Round {round_num}: Node {exploring_node} exploring' if exploring_node else f'Round {round_num}',
                'edges': set(self.initial_edges),
                'mst_edges': mst_edges.copy(),
                'exploring_node': exploring_node,
                'tree_nodes': tree_nodes.copy(),
                'stats': f'MST Edges: {len(mst_edges)}/{self.n-1}, Nodes in Tree: {len(tree_nodes)}/{self.n}'
            })
        
        # Final step: Show complete MST
        self.history.append({
            'step': round_num + 1,
            'description': 'Complete MST (All edges pruned)',
            'edges': set(),  # No non-MST edges
            'mst_edges': mst_edges.copy(),
            'exploring_node': None,
            'tree_nodes': tree_nodes.copy(),
            'stats': f'MST Complete: {len(mst_edges)} edges, {round_num} rounds (O(n))'
        })
        
        # Validate that all nodes were visited
        if len(tree_nodes) < self.n:
            unreached = sorted(set(range(1, self.n+1)) - tree_nodes)
            print(f"\\n*** ERROR: DFS did not visit all nodes! ***")
            print(f"  Visited: {len(tree_nodes)}/{self.n}")
            print(f"  Unreached: {unreached}")
            raise RuntimeError(f"DFS failed to visit nodes: {unreached}")
        else:
            print(f"GUI: DFS completed successfully - all {self.n} nodes visited in {round_num} rounds")
    
    def _create_widgets(self):
        """Create GUI widgets."""
        # Top frame: Controls
        control_frame = tk.Frame(self.root, bg='#f0f0f0', height=80)
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)
        control_frame.pack_propagate(False)
        
        # Title
        title_label = tk.Label(control_frame, text="Distributed MST Construction", 
                               font=('Arial', 16, 'bold'), bg='#f0f0f0')
        title_label.pack(pady=5)
        
        # Button frame
        button_frame = tk.Frame(control_frame, bg='#f0f0f0')
        button_frame.pack()
        
        self.play_button = tk.Button(button_frame, text="▶ Play", command=self._play,
                                     font=('Arial', 12), width=10, bg='#4CAF50', fg='white')
        self.play_button.pack(side=tk.LEFT, padx=5)
        
        self.pause_button = tk.Button(button_frame, text="⏸ Pause", command=self._pause,
                                      font=('Arial', 12), width=10, bg='#FF9800', fg='white')
        self.pause_button.pack(side=tk.LEFT, padx=5)
        
        self.reset_button = tk.Button(button_frame, text="⏮ Reset", command=self._reset,
                                      font=('Arial', 12), width=10, bg='#2196F3', fg='white')
        self.reset_button.pack(side=tk.LEFT, padx=5)
        
        self.next_button = tk.Button(button_frame, text="Next →", command=self._next_step,
                                     font=('Arial', 12), width=10)
        self.next_button.pack(side=tk.LEFT, padx=5)
        
        self.prev_button = tk.Button(button_frame, text="← Prev", command=self._prev_step,
                                     font=('Arial', 12), width=10)
        self.prev_button.pack(side=tk.LEFT, padx=5)
        
        # Main canvas for graph
        self.canvas = tk.Canvas(self.root, bg='white', width=800, height=700)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Right panel: Info
        info_frame = tk.Frame(self.root, bg='#f9f9f9', width=300)
        info_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=10, pady=10)
        info_frame.pack_propagate(False)
        
        # Info labels
        info_title = tk.Label(info_frame, text="Algorithm Info", 
                             font=('Arial', 14, 'bold'), bg='#f9f9f9')
        info_title.pack(pady=10)
        
        self.step_label = tk.Label(info_frame, text="Step: 0", 
                                   font=('Arial', 12), bg='#f9f9f9', anchor='w')
        self.step_label.pack(fill=tk.X, padx=10, pady=5)
        
        self.desc_label = tk.Label(info_frame, text="", 
                                   font=('Arial', 10), bg='#f9f9f9', 
                                   wraplength=280, justify='left', anchor='w')
        self.desc_label.pack(fill=tk.X, padx=10, pady=5)
        
        self.stats_label = tk.Label(info_frame, text="", 
                                    font=('Arial', 10), bg='#f9f9f9', 
                                    wraplength=280, justify='left', anchor='w')
        self.stats_label.pack(fill=tk.X, padx=10, pady=5)
        
        # Legend
        legend_frame = tk.LabelFrame(info_frame, text="Legend", 
                                     font=('Arial', 12, 'bold'), bg='#f9f9f9')
        legend_frame.pack(fill=tk.X, padx=10, pady=20)
        
        legends = [
            ("MST Edge", "#2E7D32", "thick"),
            ("Non-MST Edge", "#CCCCCC", "thin"),
            ("In Tree", "#4CAF50", "circle"),
            ("Exploring", "#FF9800", "circle"),
            ("Unvisited", "#E0E0E0", "circle")
        ]
        
        for text, color, style in legends:
            frame = tk.Frame(legend_frame, bg='#f9f9f9')
            frame.pack(fill=tk.X, padx=10, pady=3)
            
            if style == "circle":
                canvas = tk.Canvas(frame, width=20, height=20, bg='#f9f9f9', 
                                  highlightthickness=0)
                canvas.pack(side=tk.LEFT, padx=5)
                canvas.create_oval(3, 3, 17, 17, fill=color, outline='black', width=2)
            else:
                canvas = tk.Canvas(frame, width=40, height=5, bg='#f9f9f9', 
                                  highlightthickness=0)
                canvas.pack(side=tk.LEFT, padx=5)
                width = 4 if style == "thick" else 1
                canvas.create_line(0, 2, 40, 2, fill=color, width=width)
            
            label = tk.Label(frame, text=text, font=('Arial', 9), bg='#f9f9f9')
            label.pack(side=tk.LEFT, padx=5)
        
        # Speed control
        speed_frame = tk.LabelFrame(info_frame, text="Animation Speed", 
                                    font=('Arial', 12, 'bold'), bg='#f9f9f9')
        speed_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.speed_var = tk.IntVar(value=1000)
        speed_slider = tk.Scale(speed_frame, from_=100, to=3000, 
                               orient=tk.HORIZONTAL, variable=self.speed_var,
                               label="Delay (ms)", bg='#f9f9f9',
                               command=self._update_speed)
        speed_slider.pack(fill=tk.X, padx=10, pady=5)
    
    def _update_speed(self, value):
        """Update animation speed."""
        self.animation_speed = int(value)
    
    def _draw_graph(self):
        """Draw the current state of the graph."""
        self.canvas.delete("all")
        
        state = self.history[self.current_step]
        
        # Draw edges (non-MST first, then MST on top)
        for edge in state['edges']:
            if edge not in state['mst_edges']:
                i, j = edge
                x1, y1 = self.positions[i]
                x2, y2 = self.positions[j]
                self.canvas.create_line(x1, y1, x2, y2, fill='#CCCCCC', width=2)
        
        # Draw MST edges
        for edge in state['mst_edges']:
            i, j = edge
            x1, y1 = self.positions[i]
            x2, y2 = self.positions[j]
            self.canvas.create_line(x1, y1, x2, y2, fill='#2E7D32', width=5)
        
        # Draw nodes
        for node_id in range(1, self.n + 1):
            x, y = self.positions[node_id]
            
            # Determine color
            if node_id == state['exploring_node']:
                color = '#FF9800'  # Orange for exploring
            elif node_id in state['tree_nodes']:
                color = '#4CAF50'  # Green for in tree
            else:
                color = '#E0E0E0'  # Gray for unvisited
            
            # Draw node
            radius = 25
            self.canvas.create_oval(x-radius, y-radius, x+radius, y+radius,
                                   fill=color, outline='black', width=3)
            
            # Draw node label
            self.canvas.create_text(x, y, text=str(node_id), 
                                   font=('Arial', 14, 'bold'), fill='white')
    
    def _update_display(self):
        """Update the display with current step."""
        self._draw_graph()
        
        state = self.history[self.current_step]
        self.step_label.config(text=f"Step: {self.current_step}/{len(self.history)-1}")
        self.desc_label.config(text=state['description'])
        self.stats_label.config(text=state['stats'])
        
        # Update button states
        self.prev_button.config(state=tk.NORMAL if self.current_step > 0 else tk.DISABLED)
        self.next_button.config(state=tk.NORMAL if self.current_step < len(self.history)-1 else tk.DISABLED)
    
    def _play(self):
        """Start animation."""
        self.is_playing = True
        self._animate()
    
    def _pause(self):
        """Pause animation."""
        self.is_playing = False
    
    def _reset(self):
        """Reset to beginning."""
        self.is_playing = False
        self.current_step = 0
        self._update_display()
    
    def _next_step(self):
        """Go to next step."""
        if self.current_step < len(self.history) - 1:
            self.current_step += 1
            self._update_display()
    
    def _prev_step(self):
        """Go to previous step."""
        if self.current_step > 0:
            self.current_step -= 1
            self._update_display()
    
    def _animate(self):
        """Animate the algorithm."""
        if self.is_playing and self.current_step < len(self.history) - 1:
            self.current_step += 1
            self._update_display()
            self.root.after(self.animation_speed, self._animate)
        else:
            self.is_playing = False
    
    def run(self):
        """Start the GUI."""
        self.root.mainloop()


def generate_random_connected_graph(n, extra_edges_ratio=0.3):
    """Generate a random connected graph.
    
    Args:
        n: Number of nodes
        extra_edges_ratio: Ratio of extra edges to add beyond spanning tree (0.0 to 1.0)
    
    Returns:
        List of edges (tuples)
    """
    edges = []
    
    # Create a random spanning tree to ensure connectivity
    nodes = list(range(1, n + 1))
    tree_nodes = [nodes[0]]  # Start with first node
    remaining = nodes[1:]
    
    # Build spanning tree by randomly connecting remaining nodes
    random.shuffle(remaining)
    for node in remaining:
        # Connect to random node already in tree
        parent = random.choice(tree_nodes)
        edges.append((min(parent, node), max(parent, node)))
        tree_nodes.append(node)
    
    # Add extra random edges for redundancy
    max_extra = int((n * (n - 1) // 2 - (n - 1)) * extra_edges_ratio)
    extra_count = random.randint(2, max(2, max_extra))
    
    edge_set = set(edges)
    attempts = 0
    while len(edges) - (n - 1) < extra_count and attempts < 100:
        u = random.randint(1, n)
        v = random.randint(1, n)
        if u != v:
            edge = (min(u, v), max(u, v))
            if edge not in edge_set:
                edges.append(edge)
                edge_set.add(edge)
        attempts += 1
    
    return edges


def launch_gui_simulation():
    """Launch the GUI simulation."""
    print("\n" + "="*80)
    print("LAUNCHING GUI SIMULATION")
    print("="*80)
    print("\nThis will open an interactive window showing:")
    print("  - Initial connected graph")
    print("  - Step-by-step DFS MST construction")
    print("  - Final MST with pruned edges")
    print("\nControls:")
    print("  ▶ Play: Auto-play animation")
    print("  ⏸ Pause: Stop animation")
    print("  ⏮ Reset: Go back to start")
    print("  Next/Prev: Step through manually")
    print("\nGenerating random connected graph...")
    print("="*80)
    
    # Generate random graph
    n = random.randint(6, 12)  # Random size between 6 and 12 nodes
    edges = generate_random_connected_graph(n, extra_edges_ratio=0.3)
    
    # Validate connectivity
    adj = {i: [] for i in range(1, n + 1)}
    for u, v in edges:
        adj[u].append(v)
        adj[v].append(u)
    
    visited = {1}
    queue = [1]
    while queue:
        node = queue.pop(0)
        for neighbor in adj[node]:
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)
    
    if len(visited) != n:
        print(f"ERROR: Generated graph is not connected!")
        print(f"  Reachable from node 1: {sorted(visited)}")
        print(f"  Unreachable: {sorted(set(range(1, n+1)) - visited)}")
        print("  Regenerating...")
        return launch_gui_simulation()  # Try again
    
    print(f"\nRandom Graph: {n} nodes, {len(edges)} edges")
    print(f"Redundant edges: {len(edges) - (n - 1)}")
    print(f"Edges: {sorted(edges)}")
    print("="*80)
    
    # Launch GUI
    root = tk.Tk()
    app = MSTSimulationGUI(n, edges, root)
    app.run()


if __name__ == "__main__":
    # Ask user if they want to see GUI
    print("\n" + "="*80)
    response = input("Launch GUI simulation? (y/n): ").strip().lower()
    if response == 'y':
        launch_gui_simulation()
    else:
        print("Skipping GUI simulation.")
        print("Run this file again and choose 'y' to see the visualization!")
    print("="*80)
