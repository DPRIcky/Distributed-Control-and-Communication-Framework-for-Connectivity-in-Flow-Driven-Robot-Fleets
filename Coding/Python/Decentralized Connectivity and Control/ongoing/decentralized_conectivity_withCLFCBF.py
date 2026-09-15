"""
Distributed Network Algorithms Implementation
Implementation of distributed algorithms for neighbor structure identification 
and connectivity assurance as described in the paper.
"""

import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from typing import Dict, Set, List, Tuple
import copy
import random
import matplotlib.animation as animation
from matplotlib.widgets import Slider
try:
    import cvxpy as cp
    CVXPY_AVAILABLE = True
except ImportError:
    CVXPY_AVAILABLE = False
    import scipy.optimize
    print("Warning: cvxpy not available, using scipy.optimize for CLF-CBF control")

class Node:
    """A node implementing distributed neighbor structure identification"""
    
    def __init__(self, node_id: int, position: np.ndarray):
        self.id = node_id
        self.position = position
        self.direct_neighbors = set()
        
        # ξi(k) - node i's estimate of its in-neighbors (binary vector)
        self.xi = {}  # Dictionary: node_id -> 0 or 1
        
        # ωi(k) - node i's estimate of neighbor structure (distance vector)  
        self.omega = {}  # Dictionary: node_id -> distance
        
        # Algorithm state
        self.k = 0
        
    def add_direct_neighbor(self, neighbor_id: int):
        """Add a direct neighbor"""
        self.direct_neighbors.add(neighbor_id)
    
    def initialize_algorithm(self, total_nodes: int):
        """Initialize ξ and ω according to equations (3)"""
        for j in range(1, total_nodes + 1):
            if j == self.id:
                self.xi[j] = 1  # ξi,i(0) = 1
                self.omega[j] = 0  # ωi,i(0) = 0
            else:
                self.xi[j] = 0  # ξi,j(0) = 0 for j ≠ i
                self.omega[j] = float('inf')  # ωi,j(0) = ∞ for j ≠ i
    
    def update_xi_step(self, all_nodes: Dict[int, 'Node']):
        """Update ξi,j(k+1) according to equation (1)"""
        new_xi = {}
        
        for j in self.xi.keys():
            # ξi,j(k+1) = max over l∈Ni∪i of ξl,j(k)
            max_val = self.xi[j]  # Start with own value (i in Ni∪i)
            
            # Check all neighbors
            for neighbor_id in self.direct_neighbors:
                if neighbor_id in all_nodes:
                    neighbor_xi_j = all_nodes[neighbor_id].xi.get(j, 0)
                    max_val = max(max_val, neighbor_xi_j)
            
            new_xi[j] = max_val
        
        return new_xi
    
    def update_omega_step(self, all_nodes: Dict[int, 'Node'], new_xi: Dict[int, int]):
        """Update ωi,j(k+1) according to equation (2)"""
        new_omega = {}
        
        for j in self.omega.keys():
            if new_xi[j] == self.xi[j]:
                # ωi,j(k+1) = ωi,j(k) if ξi,j(k+1) = ξi,j(k)
                new_omega[j] = self.omega[j]
            else:
                # ωi,j(k+1) = min over l∈Ni of (ωl,j(k) + 1) if ξi,j(k+1) > ξi,j(k)
                min_val = float('inf')
                
                for neighbor_id in self.direct_neighbors:
                    if neighbor_id in all_nodes:
                        neighbor_omega_j = all_nodes[neighbor_id].omega.get(j, float('inf'))
                        if neighbor_omega_j != float('inf'):
                            min_val = min(min_val, neighbor_omega_j + 1)
                
                new_omega[j] = min_val if min_val != float('inf') else float('inf')
        
        return new_omega
    
    def execute_one_step(self, all_nodes: Dict[int, 'Node']):
        """Execute one step of the distributed algorithm"""
        # Update ξ first
        new_xi = self.update_xi_step(all_nodes)
        
        # Update ω based on new ξ
        new_omega = self.update_omega_step(all_nodes, new_xi)
        
        # Apply updates
        self.xi = new_xi
        self.omega = new_omega
        self.k += 1
        
        return True
    
    def get_neighbor_sets(self) -> Dict[int, List[int]]:
        """Get N(p)_i for different values of p"""
        neighbor_sets = {}
        
        for node_id, distance in self.omega.items():
            if node_id != self.id and distance != float('inf') and distance > 0:
                p = int(distance)
                if p not in neighbor_sets:
                    neighbor_sets[p] = []
                neighbor_sets[p].append(node_id)
        
        return neighbor_sets
    
    def is_network_connected(self) -> bool:
        """Check if network is connected from this node's perspective"""
        # Network is connected if ξi,j(n) ≠ 0 for all j ≠ i
        for j, xi_val in self.xi.items():
            if j != self.id and xi_val == 0:
                return False
        return True
    
    def find_highest_connected_index(self) -> int:
        """Find i* = max{j | ξi,j(k) = 1}"""
        connected_nodes = [j for j, xi_val in self.xi.items() if xi_val == 1]
        return max(connected_nodes) if connected_nodes else -1
    
    def find_highest_unconnected_index(self) -> int:
        """Find j* = max{j | ξi,j(k) = 0}"""
        unconnected_nodes = [j for j, xi_val in self.xi.items() if xi_val == 0]
        return max(unconnected_nodes) if unconnected_nodes else -1
    
    def calculate_delta_measures(self, all_nodes: Dict[int, 'Node']) -> Dict[int, Dict[int, int]]:
        """Calculate Δ(il)_i measures for critical edge detection (Equation 6)"""
        delta_measures = {}
        
        # For each direct neighbor l
        for neighbor_l in self.direct_neighbors:
            delta_il = {}
            
            # For each node j in the network
            for j in range(1, 9):  # Nodes 1-8
                if j in all_nodes:
                    # Δ(il)_i,j = ωi,j(n) - ωl,j(n)
                    omega_i_j = self.omega.get(j, float('inf'))
                    omega_l_j = all_nodes[neighbor_l].omega.get(j, float('inf'))
                    
                    if omega_i_j != float('inf') and omega_l_j != float('inf'):
                        delta_il[j] = omega_i_j - omega_l_j
                    else:
                        delta_il[j] = 0  # Default for unreachable nodes
            
            delta_measures[neighbor_l] = delta_il
        
        return delta_measures
    
    def is_edge_critical(self, neighbor_l: int, all_nodes: Dict[int, 'Node']) -> bool:
        """Determine if edge (i,l) is critical using Theorem 2 (Equation 9)"""
        delta_measures = self.calculate_delta_measures(all_nodes)
        
        if neighbor_l not in delta_measures:
            return False
        
        delta_il = delta_measures[neighbor_l]
        
        # Check condition (9) for all nodes j
        for j in range(1, 9):
            if j in delta_il:
                # Condition 1: Δ(il)_i,j ≠ 0 (not equidistant)
                if delta_il[j] == 0:
                    return False  # Found equidistant node, not critical
                
                # Condition 2: Check for all adjacent nodes i' ∈ Ni and l' ∈ Nl
                # that {Δ(ii')_i,j, Δ(ll')_l,j} ≠ {1, 1}
                
                # Check all neighbors of node i (self)
                for i_prime in self.direct_neighbors:
                    if i_prime != neighbor_l:  # Don't check the edge we're testing
                        delta_ii_prime = all_nodes[self.id].calculate_delta_measures(all_nodes)
                        if i_prime in delta_ii_prime and j in delta_ii_prime[i_prime]:
                            delta_ii_prime_j = delta_ii_prime[i_prime][j]
                            
                            # Check all neighbors of node l
                            for l_prime in all_nodes[neighbor_l].direct_neighbors:
                                if l_prime != self.id:  # Don't check the edge we're testing
                                    delta_ll_prime = all_nodes[neighbor_l].calculate_delta_measures(all_nodes)
                                    if l_prime in delta_ll_prime and j in delta_ll_prime[l_prime]:
                                        delta_ll_prime_j = delta_ll_prime[l_prime][j]
                                        
                                        # If both deltas equal 1, then edge is not critical
                                        if delta_ii_prime_j == 1 and delta_ll_prime_j == 1:
                                            return False
        
        return True  # Edge is critical if all conditions of (9) are satisfied
    
    def find_critical_edges(self, all_nodes: Dict[int, 'Node']) -> Set[int]:
        """Find all critical edges for this node"""
        critical_neighbors = set()
        
        for neighbor_l in self.direct_neighbors:
            if self.is_edge_critical(neighbor_l, all_nodes):
                critical_neighbors.add(neighbor_l)
        
        return critical_neighbors
    
    def print_delta_measures(self, all_nodes: Dict[int, 'Node']):
        """Print Δ(il)_i measures for this node"""
        delta_measures = self.calculate_delta_measures(all_nodes)
        
        print(f"Node {self.id} - Delta measures:")
        for neighbor_l in sorted(delta_measures.keys()):
            delta_il = delta_measures[neighbor_l]
            delta_vector = [delta_il.get(j, 0) for j in range(1, 9)]
            print(f"  Δ({self.id}{neighbor_l})_{self.id} = {delta_vector}")
    
    def print_state(self):
        """Print current ξ and ω values"""
        xi_dict = {j: self.xi[j] for j in sorted(self.xi.keys())}
        omega_dict = {}
        for j in sorted(self.omega.keys()):
            val = self.omega[j]
            omega_dict[j] = val if val != float('inf') else '∞'
        
        print(f"Node {self.id}:")
        print(f"  ξ = {xi_dict}")
        print(f"  ω = {omega_dict}")
    
    def compute_edge_priority(self, neighbor_id: int, all_nodes: Dict[int, 'Node'], verbose: bool = False) -> float:
        """
        Compute priority for edge (self.id, neighbor_id) using Triangle Participation + Local Redundancy
        Higher priority = remove first
        
        Mathematical Formula:
        P(i,j) = R(i,j) × (1 + T(i,j))
        where:
        - R(i,j) = |CN(i,j)| / min(deg(i), deg(j))  [Local Redundancy]
        - T(i,j) = |CN(i,j)|                         [Triangle Count]
        - CN(i,j) = N(i) ∩ N(j)                      [Common Neighbors]
        """
        if neighbor_id not in self.direct_neighbors or neighbor_id not in all_nodes:
            return 0.0
        
        neighbor_node = all_nodes[neighbor_id]
        
        # Calculate common neighbors: CN(i,j) = N(i) ∩ N(j)
        my_neighbors = self.direct_neighbors
        neighbor_neighbors = neighbor_node.direct_neighbors
        common_neighbors = my_neighbors.intersection(neighbor_neighbors)
        
        # Triangle count: T(i,j) = |CN(i,j)|
        triangle_count = len(common_neighbors)
        
        # Degrees
        my_degree = len(my_neighbors)
        neighbor_degree = len(neighbor_neighbors)
        
        if verbose:
            print(f"    🔍 Edge ({self.id},{neighbor_id}) Analysis from Node {self.id}:")
            print(f"      My neighbors: {sorted(list(my_neighbors))}")
            print(f"      Neighbor's neighbors: {sorted(list(neighbor_neighbors))}")
            print(f"      Common neighbors: {sorted(list(common_neighbors))}")
            print(f"      Triangle count T({self.id},{neighbor_id}) = {triangle_count}")
            print(f"      My degree = {my_degree}, Neighbor degree = {neighbor_degree}")
        
        if my_degree == 0 or neighbor_degree == 0:
            if verbose:
                print(f"      ⚠️ Zero degree detected - priority = 0.0")
            return 0.0
        
        # Local redundancy: R(i,j) = |CN(i,j)| / min(deg(i), deg(j))
        redundancy = triangle_count / min(my_degree, neighbor_degree)
        
        # Combined priority: P(i,j) = R(i,j) × (1 + T(i,j))
        priority = redundancy * (1 + triangle_count)
        
        if verbose:
            print(f"      Redundancy R({self.id},{neighbor_id}) = {triangle_count}/{min(my_degree, neighbor_degree)} = {redundancy:.3f}")
            print(f"      Priority P({self.id},{neighbor_id}) = {redundancy:.3f} × (1 + {triangle_count}) = {priority:.3f}")
            print()
        
        return priority
    
    def get_local_edge_priorities(self, all_nodes: Dict[int, 'Node']) -> Dict[int, float]:
        """Get priority scores for all edges connected to this node"""
        priorities = {}
        for neighbor_id in self.direct_neighbors:
            priority = self.compute_edge_priority(neighbor_id, all_nodes)
            priorities[neighbor_id] = priority
        return priorities

class DistributedNetwork:
    """Network implementing distributed algorithms"""
    
    def __init__(self, create_default_graph=True):
        self.nodes: Dict[int, Node] = {}
        self.k = 0
        
        if create_default_graph:
            self.create_specific_graph()
            
            # Initialize all nodes
            for node in self.nodes.values():
                node.initialize_algorithm(8)
            
            print("Initial Graph Structure:")
            self.print_graph_structure()
            print("\nInitial State (k=0):")
            self.print_all_states()
    
    def create_specific_graph(self):
        """Create the specific graph structure"""
        positions = {
            1: np.array([1.0, 2.0]),
            2: np.array([0.0, 1.0]),
            3: np.array([1.0, 0.0]),
            4: np.array([2.0, 1.0]),
            5: np.array([3.0, 1.0]),
            6: np.array([4.0, 1.0]),
            7: np.array([4.0, 0.0]),
            8: np.array([5.0, 1.0])
        }
        
        for node_id, pos in positions.items():
            self.nodes[node_id] = Node(node_id, pos)
        
        edges = [
            (1, 2), (1, 4),
            (2, 3), (2, 4),
            (3, 4),
            (4, 5),
            (5, 6), (5, 7),
            (6, 7), (6, 8),
        ]
        
        for node1, node2 in edges:
            self.nodes[node1].add_direct_neighbor(node2)
            self.nodes[node2].add_direct_neighbor(node1)
    
    def print_graph_structure(self):
        """Print the graph structure"""
        print("Graph edges:")
        for node_id in sorted(self.nodes.keys()):
            neighbors = sorted(list(self.nodes[node_id].direct_neighbors))
            print(f"  Node {node_id}: neighbors = {neighbors}")
    
    def run_n_step_algorithm(self, n_steps: int = 8):
        """Run the n-step distributed algorithm"""
        print(f"\nRunning {n_steps}-step distributed algorithm:")
        print("=" * 60)
        
        for step in range(1, n_steps + 1):
            print(f"\n--- Step k = {step} ---")
            
            # All nodes execute one step simultaneously
            for node in self.nodes.values():
                node.execute_one_step(self.nodes)
            
            self.k = step
            
            # Print state after this step
            print(f"State after step {step}:")
            self.print_all_states()
            
            # Check connectivity
            connectivity_status = self.check_connectivity_status()
            print(f"Connectivity status: {connectivity_status}")
        
        return step
    
    def print_all_states(self):
        """Print ξ and ω for all nodes"""
        for node_id in sorted(self.nodes.keys()):
            self.nodes[node_id].print_state()
        print()
    
    def check_connectivity_status(self):
        """Check connectivity from all nodes' perspectives"""
        connected_count = 0
        for node in self.nodes.values():
            if node.is_network_connected():
                connected_count += 1
        
        if connected_count == len(self.nodes):
            return "All nodes see network as CONNECTED"
        elif connected_count == 0:
            return "All nodes see network as DISCONNECTED"
        else:
            return f"{connected_count}/{len(self.nodes)} nodes see network as connected"
    
    def is_network_connected(self) -> bool:
        """Check if the network is connected using BFS from any node"""
        if not self.nodes:
            return True
            
        # Start BFS from first node
        start_node = next(iter(self.nodes.keys()))
        visited = set()
        queue = [start_node]
        visited.add(start_node)
        
        while queue:
            current = queue.pop(0)
            for neighbor in self.nodes[current].direct_neighbors:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        
        # Network is connected if all nodes are reachable
        return len(visited) == len(self.nodes)
    
    def compute_bilateral_edge_priorities(self, non_critical_edges: Set[Tuple[int, int]]) -> Dict[Tuple[int, int], float]:
        """
        Compute bilateral consensus priorities for non-critical edges
        
        Algorithm:
        1. Each endpoint computes local priority
        2. Final priority = average of both endpoints
        3. Higher priority = remove first
        """
        bilateral_priorities = {}
        
        for edge in sorted(non_critical_edges):
            node1_id, node2_id = edge
            node1 = self.nodes[node1_id]
            node2 = self.nodes[node2_id]
            
            # Each node computes its local priority for this edge (quiet mode)
            priority_from_node1 = node1.compute_edge_priority(node2_id, self.nodes, verbose=False)
            priority_from_node2 = node2.compute_edge_priority(node1_id, self.nodes, verbose=False)
            
            # Bilateral consensus: average of both perspectives
            consensus_priority = (priority_from_node1 + priority_from_node2) / 2.0
            
            bilateral_priorities[edge] = consensus_priority
        
        return bilateral_priorities
    
    def select_edge_for_removal(self, non_critical_edges: Set[Tuple[int, int]]) -> Tuple[int, int]:
        """
        Select the edge with highest priority for removal using bilateral consensus
        """
        if not non_critical_edges:
            return None
        
        # Compute bilateral priorities
        priorities = self.compute_bilateral_edge_priorities(non_critical_edges)
        
        # Sort edges by priority (highest first)
        sorted_edges = sorted(priorities.items(), key=lambda x: x[1], reverse=True)
        
        print(f"\n🏆 PRIORITY RANKING (Higher = Remove First)")
        print("=" * 60)
        for rank, (edge, priority) in enumerate(sorted_edges, 1):
            status = "👑 SELECTED" if rank == 1 else f"  #{rank}"
            print(f"{status}: Edge {edge} → Priority: {priority:.4f}")
        
        # Select edge with highest priority
        selected_edge = sorted_edges[0][0]
        
        return selected_edge

    def print_neighbor_sets_CLEAN(self):
        """Print N(p)_i for all nodes - CLEAN VERSION"""
        print("\nNeighbor Sets N(p)_i:")
        print("=" * 40)
        
        for node_id in sorted(self.nodes.keys()):
            node = self.nodes[node_id]
            neighbor_sets = node.get_neighbor_sets()
            
            print(f"Node {node_id}:")
            if neighbor_sets:
                for p in sorted(neighbor_sets.keys()):
                    neighbors = sorted(list(neighbor_sets[p]))
                    print(f"  N({p})_{node_id} = {neighbors}")
            else:
                print(f"  No neighbor sets computed yet")
            print()
    
    def run_algorithm_a(self):
        """Run Algorithm A for ensuring connectivity"""
        print("\nRunning Algorithm A (Distributed Connection of Non-Connected Network):")
        print("=" * 70)
        
        max_iterations = 10
        iteration = 0
        
        while iteration < max_iterations:
            print(f"\n--- Algorithm A - Iteration {iteration + 1} ---")
            
            # Check if any node sees disconnected components
            nodes_needing_connection = []
            for node in self.nodes.values():
                if not node.is_network_connected():
                    nodes_needing_connection.append(node)
            
            if not nodes_needing_connection:
                print("✅ All nodes see the network as connected!")
                break
            
            print(f"Nodes seeing disconnected network: {[n.id for n in nodes_needing_connection]}")
            
            # Execute Algorithm A steps
            connections_made = []
            for node in nodes_needing_connection:
                # Find i* = max{j | ξi,j(k) = 1}
                i_star = node.find_highest_connected_index()
                
                # Check if this node is i*
                if node.id == i_star:
                    # Find j* = max{j | ξi,j(k) = 0}  
                    j_star = node.find_highest_unconnected_index()
                    
                    if j_star != -1:
                        print(f"Node {node.id}: Establishing link with Node {j_star}")
                        
                        # Add the edge
                        self.nodes[node.id].add_direct_neighbor(j_star)
                        self.nodes[j_star].add_direct_neighbor(node.id)
                        
                        # Update connectivity matrices
                        self.nodes[node.id].xi[j_star] = 1
                        self.nodes[j_star].xi[node.id] = 1
                        self.nodes[node.id].omega[j_star] = 1
                        self.nodes[j_star].omega[node.id] = 1
                        
                        connections_made.append((node.id, j_star))
            
            if connections_made:
                print(f"New connections established: {connections_made}")
                
                # Run a few steps of the distributed algorithm to propagate changes
                print("Propagating connectivity information...")
                for prop_step in range(3):
                    for node in self.nodes.values():
                        node.execute_one_step(self.nodes)
                
                # Print updated state
                print("Updated state:")
                self.print_all_states()
            
            iteration += 1
        
        if iteration >= max_iterations:
            print("⚠️ Algorithm A reached maximum iterations")
        
        return iteration
    
    def run_critical_edge_detection(self):
        """Run 2-step distributed critical edge detection algorithm"""
        print("\n" + "=" * 70)
        print("RUNNING CRITICAL EDGE DETECTION (2-STEP ALGORITHM)")
        print("=" * 70)
        
        # Step 1: Calculate delta measures for all nodes
        print("\nStep 1: Calculating Δ(il)_i measures...")
        print("-" * 50)
        
        for node_id in sorted(self.nodes.keys()):
            self.nodes[node_id].print_delta_measures(self.nodes)
        
        # Step 2: Identify critical edges
        print("\nStep 2: Identifying critical edges...")
        print("-" * 50)
        
        all_critical_edges = set()
        node_critical_edges = {}
        
        for node_id in sorted(self.nodes.keys()):
            node = self.nodes[node_id]
            critical_neighbors = node.find_critical_edges(self.nodes)
            node_critical_edges[node_id] = critical_neighbors
            
            print(f"Node {node_id} critical neighbors: {sorted(list(critical_neighbors))}")
            
            # Add to global set (ensure we don't double count)
            for neighbor in critical_neighbors:
                edge = tuple(sorted([node_id, neighbor]))
                all_critical_edges.add(edge)
        
        print(f"\nAll critical edges in the network: {sorted(list(all_critical_edges))}")
        return all_critical_edges, node_critical_edges
    
    def visualize_network(self, title: str = "Network Graph", critical_edges: Set[tuple] = None):
        """Visualize the current network with critical edges marked in red"""
        G = nx.Graph()
        
        # Add nodes
        for node_id, node in self.nodes.items():
            G.add_node(node_id, pos=node.position)
        
        # Add edges
        all_edges = []
        for node_id, node in self.nodes.items():
            for neighbor_id in node.direct_neighbors:
                if node_id < neighbor_id:  # Avoid duplicate edges
                    edge = (node_id, neighbor_id)
                    G.add_edge(node_id, neighbor_id)
                    all_edges.append(edge)
        
        # Get positions
        pos = {node_id: node.position for node_id, node in self.nodes.items()}
        
        # Create figure with two subplots
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        
        # Subplot 1: Complete graph (all black edges)
        ax1.set_title("Complete Network Graph", fontsize=14, fontweight='bold')
        
        # Draw all edges in black
        nx.draw_networkx_edges(G, pos, edgelist=all_edges, 
                             edge_color='black', width=2, alpha=0.7, ax=ax1)
        
        # Draw nodes with smaller size
        nx.draw_networkx_nodes(G, pos, node_color='lightblue', 
                              node_size=400, alpha=0.9, ax=ax1)  # Reduced from 800 to 400
        nx.draw_networkx_labels(G, pos, font_size=10, font_weight='bold', ax=ax1)  # Reduced font size
        
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        ax1.text(0.02, 0.98, f"Nodes: {len(G.nodes())}\nEdges: {len(G.edges())}", 
                transform=ax1.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # Subplot 2: Graph with critical edges highlighted
        ax2.set_title("Critical Edges (Bridges) in Red", fontsize=14, fontweight='bold')
        
        # Separate normal and critical edges
        normal_edges = []
        critical_edge_list = []
        
        if critical_edges:
            for edge in all_edges:
                if edge in critical_edges or tuple(reversed(edge)) in critical_edges:
                    critical_edge_list.append(edge)
                else:
                    normal_edges.append(edge)
        else:
            normal_edges = all_edges
        
        # Draw normal edges in black
        if normal_edges:
            nx.draw_networkx_edges(G, pos, edgelist=normal_edges, 
                                 edge_color='black', width=2, alpha=0.7, ax=ax2)
        
        # Draw critical edges in red with dashed lines
        if critical_edge_list:
            nx.draw_networkx_edges(G, pos, edgelist=critical_edge_list, 
                                 edge_color='red', width=3, alpha=0.9,
                                 style='--', ax=ax2)
        
        # Draw nodes with smaller size
        nx.draw_networkx_nodes(G, pos, node_color='lightblue', 
                              node_size=400, alpha=0.9, ax=ax2)  # Reduced from 800 to 400
        nx.draw_networkx_labels(G, pos, font_size=10, font_weight='bold', ax=ax2)  # Reduced font size
        
        # Add legend for subplot 2
        if critical_edge_list:
            from matplotlib.lines import Line2D
            legend_elements = [
                Line2D([0], [0], color='black', lw=2, label='Normal edges'),
                Line2D([0], [0], color='red', lw=3, linestyle='--', label='Critical edges (bridges)')
            ]
            ax2.legend(handles=legend_elements, loc='upper right')
        
        ax2.set_aspect('equal')
        ax2.grid(True, alpha=0.3)
        ax2.text(0.02, 0.98, f"Critical Edges: {len(critical_edge_list)}\n{sorted(critical_edge_list) if critical_edge_list else 'None'}", 
                transform=ax2.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.8))
        
        plt.tight_layout()
        plt.show()
        
        # Note: Critical edge removal animation disabled 
        # (Not relevant for iterative pruning since we never remove critical edges)
        
        return G
    
    # animate_edge_removal function removed - not needed for iterative pruning
    # (We never remove critical edges in our algorithm)
    
    def print_final_results(self):
        """Print final N and ω results"""
        print("\n" + "=" * 70)
        print(f"FINAL RESULTS: N(p) AND ω VALUES FOR ALL {len(self.nodes)} ROBOTS")
        print("=" * 70)
        
        # Get all node IDs
        node_ids = sorted(self.nodes.keys())
        
        # Print ω matrix
        print("\nFinal ω matrix (shortest path distances):")
        print("-" * 50)
        
        # Header
        print("From\\To", end="")
        for j in node_ids:
            print(f"\t{j}", end="")
        print()
        
        # Matrix rows
        for i in node_ids:
            print(f"{i}", end="")
            for j in node_ids:
                omega_val = self.nodes[i].omega.get(j, float('inf'))
                if omega_val == float('inf'):
                    print(f"\t∞", end="")
                else:
                    print(f"\t{omega_val:.0f}", end="")
            print()
        
        # Print neighbor sets
        self.print_neighbor_sets()
        
        # Print connectivity status
        print(f"Final connectivity status: {self.check_connectivity_status()}")
        
        # Run critical edge detection
        critical_edges, node_critical_edges = self.run_critical_edge_detection()
        
        # Show visualization with critical edges marked
        print("\nShowing network visualization with critical edges in red...")
        self.visualize_network("Final Network Structure with Critical Edges", critical_edges)
    
    def iterative_pruning_step_by_step(self):
        """
        Simple Iterative Pruning Algorithm:
        Step 1: Run distributed algorithm to find critical edges
        Step 2: Keep ALL critical edges, prune ONE non-critical edge
        Step 3: Re-run ENTIRE distributed algorithm (graph changed!)
        Step 4: Repeat until no non-critical edges remain
        """
        print("\n" + "=" * 70)
        print("STEP-BY-STEP ITERATIVE EDGE PRUNING")
        print("Algorithm: Keep critical edges, remove non-critical edges one by one")
        print("=" * 70)
        
        iteration = 0
        pruned_edges = []
        
        while True:
            iteration += 1
            print(f"\n--- ITERATION {iteration} ---")
            
            # Step 1: Re-run COMPLETE distributed algorithm (graph may have changed)
            print("Step 1: Running complete distributed algorithm...")
            
            # Re-initialize all nodes for the current graph
            num_nodes = len(self.nodes)
            for node in self.nodes.values():
                node.initialize_algorithm(num_nodes)
            
            # Run the distributed algorithm
            self.run_n_step_algorithm(n_steps=min(num_nodes + 2, 10))
            
            # Step 2: Find critical edges in current graph
            print("Step 2: Detecting critical edges in current graph...")
            critical_edges, _ = self.run_critical_edge_detection()
            critical_edges_set = set(tuple(sorted(edge)) for edge in critical_edges)
            
            # Get all current edges
            all_current_edges = set()
            for node_id, node in self.nodes.items():
                for neighbor_id in node.direct_neighbors:
                    if node_id < neighbor_id:
                        edge = (node_id, neighbor_id)
                        all_current_edges.add(edge)
            
            # Step 3: Identify non-critical edges (these can be safely removed)
            non_critical_edges = all_current_edges - critical_edges_set
            
            print(f"Current edges: {len(all_current_edges)} → {sorted(list(all_current_edges))}")
            print(f"Critical edges: {len(critical_edges_set)} → {sorted(list(critical_edges_set))}")
            print(f"Non-critical edges: {len(non_critical_edges)} → {sorted(list(non_critical_edges))}")
            
            # Step 4: Check termination condition
            if len(non_critical_edges) == 0:
                print("\n🎯 PRUNING COMPLETE!")
                print("All remaining edges are critical - graph has minimal connectivity")
                break
            
            # Step 5: Remove exactly ONE non-critical edge using sophisticated selection
            print("\n🔍 SELECTING EDGE FOR REMOVAL")
            print("Using Triangle Participation + Local Redundancy Algorithm")
            print("-" * 55)
            
            edge_to_prune = self.select_edge_for_removal(non_critical_edges)
            print(f"\n🎯 Removing edge: {edge_to_prune}")
            
            # Remove the edge from the graph
            node1, node2 = edge_to_prune
            self.nodes[node1].direct_neighbors.discard(node2)
            self.nodes[node2].direct_neighbors.discard(node1)
            
            pruned_edges.append(edge_to_prune)
            
            # Step 6: Verify connectivity is maintained
            if not self.check_connectivity_status():
                print("❌ ERROR: Graph became disconnected!")
                print("This should never happen if we only remove non-critical edges!")
                break
            
            print(f"✅ Non-critical edge {edge_to_prune} successfully removed")
            print(f"Graph remains connected with {len(all_current_edges) - 1} edges")
            print("⚠️  Graph structure changed - will re-run distributed algorithm next iteration")
            
            # Continue to next iteration (animation step removed)
            
            # Safety check
            if iteration > 50:
                print("⚠️ Maximum iterations reached")
                break
        
        # Final summary
        print(f"\n" + "="*70)
        print("ITERATIVE PRUNING SUMMARY")
        print("="*70)
        print(f"Total iterations: {iteration}")
        print(f"Non-critical edges removed: {len(pruned_edges)}")
        print(f"Removed edges: {sorted(pruned_edges)}")
        
        # Final graph analysis
        final_edges = 0
        for node in self.nodes.values():
            final_edges += len(node.direct_neighbors)
        final_edges //= 2
        
        num_nodes = len(self.nodes)
        min_possible = num_nodes - 1  # spanning tree minimum
        
        print(f"\nFinal graph properties:")
        print(f"  Nodes: {num_nodes}")
        print(f"  Final edges: {final_edges}")
        print(f"  Theoretical minimum (spanning tree): {min_possible}")
        print(f"  Extra edges: {final_edges - min_possible}")
        
        if final_edges == min_possible:
            print("🎯 Perfect! Graph is now a spanning tree (absolute minimum connectivity)")
        elif final_edges > min_possible:
            print(f"📊 Graph has {final_edges - min_possible} extra edges (some redundancy remains)")
        
        # Final verification: all remaining edges should be critical
        print("\nFinal verification: Running critical edge detection on pruned graph...")
        final_critical_edges, _ = self.run_critical_edge_detection()
        
        current_edges = set()
        for node_id, node in self.nodes.items():
            for neighbor_id in node.direct_neighbors:
                if node_id < neighbor_id:
                    current_edges.add((node_id, neighbor_id))
        
        final_critical_set = set(tuple(sorted(edge)) for edge in final_critical_edges)
        
        if current_edges == final_critical_set:
            print("✅ VERIFIED: All remaining edges are critical!")
        else:
            print("❌ WARNING: Some remaining edges are not critical")
            remaining_non_critical = current_edges - final_critical_set
            print(f"Non-critical edges still present: {sorted(list(remaining_non_critical))}")
        
        # Show final animation
        original_edges_count = final_edges + len(pruned_edges)
        self.animate_final_pruning_result(original_edges_count, final_edges, pruned_edges)
        
        return pruned_edges

    def iterative_pruning_simultaneous(self):
        """
        NEW SIMULTANEOUS BATCH PRUNING ALGORITHM WITH AUTOMATIC ANIMATION:
        - Shows continuous batch edge removal in a single window
        - No manual interaction needed - fully automatic
        - Real-time verification of the algorithm
        """
        print("\n" + "=" * 70)
        print("SIMULTANEOUS BATCH EDGE PRUNING - FAST MODE WITH ANIMATION")
        print("Algorithm: Keep critical edges, remove multiple non-critical edges per iteration")
        print("*** Watch automatic animation - no manual interaction needed!")
        print("=" * 70)
        
        # Initialize animation data
        self.batch_animation_data = {
            'iteration': 0,
            'pruned_edges': [],
            'current_state': 'detecting',  # 'detecting', 'selecting', 'removing', 'complete'
            'critical_edges': set(),
            'all_current_edges': set(),
            'non_critical_edges': set(),
            'edges_to_remove': [],
            'status_message': 'Starting batch pruning in 10x10 workspace (nodes start clustered)...',
            'max_iterations': 20,
            'original_edge_count': 0,
            # BROWNIAN MOTION PARAMETERS - 10x10 WORKSPACE
            'brownian_step_size': 0.15,  # Increased step size for more dramatic movement
            'boundary_limits': {'x_min': -5, 'x_max': 5, 'y_min': -5, 'y_max': 5},  # 10x10 workspace
            'frame_count': 0,  # Track animation frames for smooth movement
            'algorithm_step_interval': 30,  # Run algorithm logic every 30 frames (3 seconds)
            'last_algorithm_frame': 0,  # Track when we last ran algorithm logic
            'node_trails': {node_id: [] for node_id in self.nodes.keys()},  # Store recent positions for trails
            'trail_length': 25,  # Longer trails to show movement better
            # Enhanced movement parameters
            'node_velocities': {node_id: (0.0, 0.0) for node_id in self.nodes.keys()},  # Add momentum
            'velocity_persistence': 0.7,  # How much velocity carries over (0-1)
            'max_velocity': 0.2  # Maximum velocity per frame
        }
        
        # Calculate initial edge count
        for node_id, node in self.nodes.items():
            for neighbor_id in node.direct_neighbors:
                if node_id < neighbor_id:
                    self.batch_animation_data['all_current_edges'].add((node_id, neighbor_id))
        
        self.batch_animation_data['original_edge_count'] = len(self.batch_animation_data['all_current_edges'])
        
        print(f"*** Starting automatic animation with {self.batch_animation_data['original_edge_count']} initial edges...")
        print(f"🎯 INTERACTIVE DISTANCE MONITORING:")
        print(f"   • Use the slider at the bottom to select robot pairs")
        print(f"   • Real-time distance will be displayed in the top-right panel")
        print(f"   • Selected robots are highlighted in RED and BLUE in the animation")
        print(f"   • Total robot pairs available: {len(self.robot_pairs)}")
        
        # Set up matplotlib animation with subplots for network + bar chart
        import matplotlib.animation as animation
        
        self.fig = plt.figure(figsize=(20, 12))  # Wider figure for side-by-side layout with slider
        
        # Create subplots: main network plot (left) and statistics (right)
        self.ax_network = plt.subplot(2, 3, (1, 4))  # Network visualization (2 rows, left side)
        self.ax_stats = plt.subplot(2, 3, 2)    # Bar chart statistics (top right)
        self.ax_details = plt.subplot(2, 3, 5)  # Edge removal details (bottom right)
        self.ax_distance = plt.subplot(2, 3, 3)  # Distance display (middle right)
        
        # Add slider for robot pair selection
        self.ax_slider = plt.subplot(2, 3, 6)  # Bottom right for slider
        
        self.ax_network.set_aspect('equal')
        
        # Initialize robot pair selector (robot IDs for distance measurement)
        self.num_robots = len(self.nodes)
        self.robot_pairs = [(i, j) for i in range(1, self.num_robots + 1) 
                           for j in range(i + 1, self.num_robots + 1)]
        
        # Create slider for robot pair selection
        self.pair_slider = Slider(
            self.ax_slider, 
            'Robot Pair', 
            0, 
            len(self.robot_pairs) - 1, 
            valinit=0, 
            valfmt='%d',
            valstep=1
        )
        
        # Add callback for slider changes
        def slider_update(val):
            self.current_pair_index = int(val)
            # Update slider label to show current robot pair
            if self.current_pair_index < len(self.robot_pairs):
                robot1, robot2 = self.robot_pairs[self.current_pair_index]
                self.ax_slider.set_title(f'Robot Pair: {robot1} ↔ {robot2}', fontsize=10)
        
        self.pair_slider.on_changed(slider_update)
        
        # Initialize distance tracking
        self.current_pair_index = 0
        self.current_distance = 0.0
        
        # Get positions for consistent layout
        self.pos = {node_id: node.position for node_id, node in self.nodes.items()}
        
        # Create the animation with fast updates for smooth Brownian motion
        self.batch_anim = animation.FuncAnimation(
            self.fig, 
            self.update_batch_simulation, 
            interval=100,   # Update every 100ms for smooth motion (10 FPS)
            repeat=False,   # Don't repeat when done
            blit=False,     # Don't use blitting for simplicity
            cache_frame_data=False  # Disable caching to avoid warning
        )
        
        plt.tight_layout()
        plt.show()
        
        return self.batch_animation_data['pruned_edges']

    def update_node_positions_brownian(self, data):
        """
        Update node positions with ENHANCED Brownian motion (random walk with momentum)
        This simulates MORE REALISTIC robot movement while the algorithm is running!
        """
        step_size = data['brownian_step_size']
        bounds = data['boundary_limits']
        persistence = data['velocity_persistence']
        max_vel = data['max_velocity']
        
        for node_id, node in self.nodes.items():
            # Get current velocity (with momentum from previous frame)
            current_vel = data['node_velocities'][node_id]
            
            # Add random acceleration (more dramatic changes)
            random_accel_x = random.uniform(-step_size, step_size)
            random_accel_y = random.uniform(-step_size, step_size)
            
            # Update velocity with persistence (momentum) + random acceleration
            new_vel_x = current_vel[0] * persistence + random_accel_x
            new_vel_y = current_vel[1] * persistence + random_accel_y
            
            # Limit maximum velocity to prevent crazy speeds
            vel_magnitude = (new_vel_x**2 + new_vel_y**2)**0.5
            if vel_magnitude > max_vel:
                new_vel_x = (new_vel_x / vel_magnitude) * max_vel
                new_vel_y = (new_vel_y / vel_magnitude) * max_vel
            
            # Calculate new position based on velocity
            new_x = node.position[0] + new_vel_x
            new_y = node.position[1] + new_vel_y
            
            # Boundary handling with velocity reflection (more realistic bouncing)
            if new_x < bounds['x_min'] or new_x > bounds['x_max']:
                new_vel_x = -new_vel_x * 0.8  # Bounce with some energy loss
                new_x = node.position[0] + new_vel_x  # Recalculate position
                new_x = max(bounds['x_min'], min(bounds['x_max'], new_x))  # Clamp to bounds
            
            if new_y < bounds['y_min'] or new_y > bounds['y_max']:
                new_vel_y = -new_vel_y * 0.8  # Bounce with some energy loss
                new_y = node.position[1] + new_vel_y  # Recalculate position
                new_y = max(bounds['y_min'], min(bounds['y_max'], new_y))  # Clamp to bounds
            
            # Store updated velocity
            data['node_velocities'][node_id] = (new_vel_x, new_vel_y)
            
            # Update node position
            node.position = (new_x, new_y)
            
            # Store position in trail (for visual effect)
            if node_id not in data['node_trails']:
                data['node_trails'][node_id] = []
            
            data['node_trails'][node_id].append((new_x, new_y))
            
            # Keep only recent trail positions
            if len(data['node_trails'][node_id]) > data['trail_length']:
                data['node_trails'][node_id].pop(0)
            
        # Update the positions dictionary for NetworkX visualization
        self.pos = {node_id: node.position for node_id, node in self.nodes.items()}

    def update_distance_display(self):
        """
        Update the real-time distance display between selected robot pair
        """
        # Get current robot pair from slider
        self.current_pair_index = int(self.pair_slider.val)
        if self.current_pair_index < len(self.robot_pairs):
            robot1_id, robot2_id = self.robot_pairs[self.current_pair_index]
            
            # Calculate distance between the two robots
            pos1 = self.nodes[robot1_id].position
            pos2 = self.nodes[robot2_id].position
            self.current_distance = np.linalg.norm(pos1 - pos2)
            
            # Update distance display panel
            self.ax_distance.clear()
            self.ax_distance.set_title(f"📏 Robot Distance Monitor", fontweight='bold', fontsize=11)
            
            # Create a nice distance display
            display_text = f"Robot Pair: {robot1_id} ↔ {robot2_id}\n"
            display_text += f"Distance: {self.current_distance:.3f} units\n\n"
            
            # Add visual indicator
            if self.current_distance < 1.0:
                status_color = 'green'
                status = "🔗 CLOSE"
            elif self.current_distance < 2.0:
                status_color = 'orange'
                status = "⚠️ MEDIUM"
            else:
                status_color = 'red'
                status = "📏 FAR"
            
            display_text += f"Status: {status}"
            
            # Display the text with color coding
            self.ax_distance.text(0.05, 0.95, display_text, 
                                transform=self.ax_distance.transAxes,
                                fontsize=12, verticalalignment='top',
                                bbox=dict(boxstyle='round', facecolor=status_color, alpha=0.3))
            
            # Add position coordinates
            pos_text = f"Robot {robot1_id}: ({pos1[0]:.2f}, {pos1[1]:.2f})\n"
            pos_text += f"Robot {robot2_id}: ({pos2[0]:.2f}, {pos2[1]:.2f})"
            
            self.ax_distance.text(0.05, 0.45, pos_text,
                                transform=self.ax_distance.transAxes,
                                fontsize=10, verticalalignment='top',
                                fontfamily='monospace',
                                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.3))
            
            # Draw a simple distance visualization
            self.ax_distance.plot([0.1, 0.9], [0.25, 0.25], 'k-', linewidth=2)
            self.ax_distance.plot([0.1], [0.25], 'ro', markersize=8, label=f'Robot {robot1_id}')
            self.ax_distance.plot([0.9], [0.25], 'bo', markersize=8, label=f'Robot {robot2_id}')
            
            # Add distance as text on the line
            self.ax_distance.text(0.5, 0.3, f"{self.current_distance:.3f}", 
                                horizontalalignment='center', fontweight='bold')
            
            self.ax_distance.set_xlim(0, 1)
            self.ax_distance.set_ylim(0, 1)
            self.ax_distance.axis('off')

    def update_batch_simulation(self, frame):
        """
        Update function for batch pruning animation - called automatically
        WITH BROWNIAN MOTION FOR DYNAMIC NODES! 🎯
        """
        data = self.batch_animation_data
        data['frame_count'] += 1
        
        # 🎯 BROWNIAN MOTION UPDATE - Move all nodes randomly each frame
        self.update_node_positions_brownian(data)
        
        # Only run algorithm logic every N frames (for smooth motion visualization)
        should_run_algorithm = (data['frame_count'] - data['last_algorithm_frame']) >= data['algorithm_step_interval']
        
        # Update status for continuous motion visualization
        if not should_run_algorithm and data['current_state'] != 'complete':
            frames_until_next = data['algorithm_step_interval'] - (data['frame_count'] - data['last_algorithm_frame'])
            data['status_message'] = f'*** Robots spreading out in 10x10 workspace! (Algorithm step in {frames_until_next} frames)'
        
        # State machine for batch pruning algorithm
        if data['current_state'] == 'detecting' and should_run_algorithm:
            # Phase 1: Run critical edge detection
            data['status_message'] = f'Iteration {data["iteration"] + 1}: Detecting critical edges (nodes moving)...'
            data['last_algorithm_frame'] = data['frame_count']
            
            # Run distributed algorithm to identify critical edges
            critical_edges, _ = self.run_critical_edge_detection()
            data['critical_edges'] = set(critical_edges)
            
            # Get all current edges
            data['all_current_edges'] = set()
            for node_id, node in self.nodes.items():
                for neighbor_id in node.direct_neighbors:
                    if node_id < neighbor_id:
                        edge = (node_id, neighbor_id)
                        data['all_current_edges'].add(edge)
            
            # Identify non-critical edges
            data['non_critical_edges'] = data['all_current_edges'] - data['critical_edges']
            
            if len(data['non_critical_edges']) == 0:
                data['current_state'] = 'complete'
                data['status_message'] = 'PRUNING COMPLETE! Only critical edges remain.'
            else:
                data['current_state'] = 'selecting'
        
        elif data['current_state'] == 'selecting' and should_run_algorithm:
            # Phase 2: Select edges for batch removal
            data['status_message'] = f'Selecting batch of edges for removal (nodes moving)...'
            data['last_algorithm_frame'] = data['frame_count']
            
            # Calculate optimal batch size
            optimal_batch_size = max(1, min(6, len(data['non_critical_edges']) // 3))
            
            # Select edges for batch removal
            data['edges_to_remove'] = self.select_edges_for_batch_removal(
                data['non_critical_edges'], max_batch_size=optimal_batch_size)
            
            if not data['edges_to_remove']:
                data['current_state'] = 'complete'
            else:
                data['current_state'] = 'removing'
                data['status_message'] = f'Removing {len(data["edges_to_remove"])} edges simultaneously...'
        
        elif data['current_state'] == 'removing' and should_run_algorithm:
            # Phase 3: Remove the selected batch of edges
            data['last_algorithm_frame'] = data['frame_count']
            
            for edge_to_remove in data['edges_to_remove']:
                node1, node2 = edge_to_remove
                self.nodes[node1].direct_neighbors.discard(node2)
                self.nodes[node2].direct_neighbors.discard(node1)
                data['pruned_edges'].append(edge_to_remove)
            
            data['iteration'] += 1
            data['status_message'] = f'Removed {len(data["edges_to_remove"])} edges. Nodes continue moving...'
            
            # Check if we should continue
            if data['iteration'] >= data['max_iterations']:
                data['current_state'] = 'complete'
            else:
                data['current_state'] = 'detecting'
                data['edges_to_remove'] = []  # Clear for next iteration
        
        # Update distance display in real-time
        self.update_distance_display()
        
        # Draw the network based on current state
        self.draw_batch_network_state(data)
        
        # Stop animation when complete
        if data['current_state'] == 'complete':
            # Stop the animation if it exists
            if hasattr(self, 'batch_anim') and self.batch_anim is not None:
                self.batch_anim.event_source.stop()
            print(f"\n[SUCCESS] Batch pruning animation complete!")
            print(f"   Original edges: {data['original_edge_count']}")
            print(f"   Final edges: {len(data['all_current_edges'])}")
            print(f"   Edges removed: {len(data['pruned_edges'])}")
            print(f"   Iterations: {data['iteration']}")
            
            # CRITICAL: Final connectivity validation
            self.perform_final_connectivity_validation()

    def draw_batch_network_state(self, data):
        """
        Draw the current state of the network during batch pruning animation
        WITH COOL BAR CHART AND STATISTICS PANELS!
        """
        # Clear main subplots (but not distance display or slider)
        self.ax_network.clear()
        self.ax_stats.clear() 
        self.ax_details.clear()
        # Note: distance display and slider are updated separately
        
        # Create NetworkX graph for visualization
        G = nx.Graph()
        G.add_nodes_from(self.nodes.keys())
        
        # Add edges to the graph
        for edge in data['all_current_edges']:
            G.add_edge(edge[0], edge[1])
        
        # ==== MAIN NETWORK VISUALIZATION (LEFT SIDE) ====
        # Set title based on current state
        title = f"Simultaneous Batch Pruning - {data['status_message']}\n"
        title += f"Iteration {data['iteration']} | "
        title += f"Total: {len(data['all_current_edges'])} | "
        title += f"Critical: {len(data['critical_edges'])} | "
        title += f"Non-Critical: {len(data['non_critical_edges'])}"
        
        self.ax_network.set_title(title, fontsize=12, fontweight='bold')
        
        # 🏠 Draw 10x10 workspace boundaries (show the operating area)
        bounds = data['boundary_limits']
        
        # Draw workspace boundary rectangle
        boundary_x = [bounds['x_min'], bounds['x_max'], bounds['x_max'], bounds['x_min'], bounds['x_min']]
        boundary_y = [bounds['y_min'], bounds['y_min'], bounds['y_max'], bounds['y_max'], bounds['y_min']]
        self.ax_network.plot(boundary_x, boundary_y, color='black', linewidth=2, linestyle='--', alpha=0.5)
        
        # Draw initial 1-unit radius circle (where robots started)
        circle = plt.Circle((0, 0), 0.8, fill=False, color='lightgray', linestyle=':', linewidth=1, alpha=0.7)
        self.ax_network.add_patch(circle)
        
        # Add workspace labels
        self.ax_network.text(bounds['x_min'] - 0.5, bounds['y_max'], '10x10 Workspace', 
                            fontsize=9, alpha=0.7, rotation=90, verticalalignment='top')
        self.ax_network.text(0, -0.6, 'Initial\n1-unit radius', 
                            fontsize=8, alpha=0.6, horizontalalignment='center')
        
        # Set axis limits to show full workspace with some padding
        self.ax_network.set_xlim(bounds['x_min'] - 1, bounds['x_max'] + 1)
        self.ax_network.set_ylim(bounds['y_min'] - 1, bounds['y_max'] + 1)
        
        # 🎯 Draw Enhanced Brownian motion trails (showing dramatic node movement paths)
        trail_colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
        
        for i, (node_id, trail) in enumerate(data['node_trails'].items()):
            if len(trail) > 1:
                trail_x = [pos[0] for pos in trail]
                trail_y = [pos[1] for pos in trail]
                
                # Use different colors for different nodes
                trail_color = trail_colors[i % len(trail_colors)]
                
                # Create gradient effect for trail (recent positions more visible)
                for j in range(len(trail) - 1):
                    alpha = (j + 1) / len(trail) * 0.7  # More visible trails (0 to 0.7)
                    line_width = ((j + 1) / len(trail)) * 2 + 0.5  # Thicker recent segments
                    self.ax_network.plot([trail_x[j], trail_x[j+1]], [trail_y[j], trail_y[j+1]], 
                                       color=trail_color, alpha=alpha, linewidth=line_width)
        
        # Draw nodes (current positions) with highlighted selected pair
        node_colors = []
        node_sizes = []
        
        # Get selected robot pair from slider
        selected_pair = self.robot_pairs[int(self.pair_slider.val)]
        robot1_id, robot2_id = selected_pair
        
        for node_id in self.nodes.keys():
            if node_id == robot1_id:
                node_colors.append('red')  # First robot in pair
                node_sizes.append(800)
            elif node_id == robot2_id:
                node_colors.append('blue')  # Second robot in pair
                node_sizes.append(800)
            else:
                node_colors.append('lightblue')  # Regular robots
                node_sizes.append(600)
        
        nx.draw_networkx_nodes(G, self.pos, ax=self.ax_network, 
                              node_color=node_colors, node_size=node_sizes, alpha=0.9)
        
        # Draw a special line between the selected robot pair
        if robot1_id in self.pos and robot2_id in self.pos:
            pos1 = self.pos[robot1_id]
            pos2 = self.pos[robot2_id]
            self.ax_network.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], 
                               color='purple', linewidth=3, alpha=0.7, linestyle='--',
                               label=f'Measuring: {robot1_id}↔{robot2_id}')
        
        
        # Draw different types of edges with different colors
        if data['current_state'] != 'complete':
            # Critical edges (red) - never removed
            critical_edge_list = [(e[0], e[1]) for e in data['critical_edges']]
            if critical_edge_list:
                nx.draw_networkx_edges(G, self.pos, ax=self.ax_network, edgelist=critical_edge_list, 
                                      edge_color='red', width=3, alpha=0.8)
            
            # Non-critical edges (gray) - candidates for removal  
            non_critical_list = [(e[0], e[1]) for e in data['non_critical_edges'] 
                               if e not in data['edges_to_remove']]
            if non_critical_list:
                nx.draw_networkx_edges(G, self.pos, ax=self.ax_network, edgelist=non_critical_list, 
                                      edge_color='gray', width=1, alpha=0.6)
            
            # Edges being removed (orange/yellow) - highlighted
            if data['edges_to_remove']:
                edges_removing_list = [(e[0], e[1]) for e in data['edges_to_remove']]
                nx.draw_networkx_edges(G, self.pos, ax=self.ax_network, edgelist=edges_removing_list, 
                                      edge_color='orange', width=5, alpha=1.0)
        else:
            # Final state - all remaining edges are critical (red)
            all_edges_list = [(e[0], e[1]) for e in data['all_current_edges']]
            if all_edges_list:
                nx.draw_networkx_edges(G, self.pos, ax=self.ax_network, edgelist=all_edges_list, 
                                      edge_color='red', width=3, alpha=0.8)
        
        # Draw labels
        nx.draw_networkx_labels(G, self.pos, ax=self.ax_network, font_size=10, font_weight='bold')
        
        # Add legend
        from matplotlib.lines import Line2D
        legend_elements = []
        
        if data['current_state'] != 'complete':
            legend_elements = [
                Line2D([0], [0], color='red', lw=3, label=f'Critical Edges ({len(data["critical_edges"])})'),
                Line2D([0], [0], color='gray', lw=1, label=f'Non-Critical ({len(data["non_critical_edges"]) - len(data["edges_to_remove"])})'),
            ]
            if data['edges_to_remove']:
                legend_elements.append(
                    Line2D([0], [0], color='orange', lw=5, label=f'Being Removed ({len(data["edges_to_remove"])})')
                )
        else:
            legend_elements = [
                Line2D([0], [0], color='red', lw=3, label=f'Final Critical Edges ({len(data["all_current_edges"])})')
            ]
        
        self.ax_network.legend(handles=legend_elements, loc='upper right')
        self.ax_network.axis('off')
        
        # ==== BAR CHART STATISTICS (TOP RIGHT) ====
        self.ax_stats.set_title("*** Edge Statistics", fontweight='bold', fontsize=11)
        
        # Create bar chart data
        if data['current_state'] != 'complete':
            categories = ['Critical', 'Non-Critical\nRemaining', 'Being\nRemoved']
            counts = [
                len(data['critical_edges']), 
                len(data['non_critical_edges']) - len(data['edges_to_remove']),
                len(data['edges_to_remove'])
            ]
            colors = ['red', 'gray', 'orange']
        else:
            # Final state
            categories = ['Critical\n(Final)', 'Total\nRemoved']
            counts = [len(data['all_current_edges']), len(data['pruned_edges'])]
            colors = ['red', 'lightcoral']
        
        bars = self.ax_stats.bar(categories, counts, color=colors, alpha=0.7, edgecolor='black', linewidth=1)
        self.ax_stats.set_ylabel('Number of Edges', fontsize=10)
        
        # Add count labels on bars
        for bar, count in zip(bars, counts):
            if count > 0:
                self.ax_stats.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1, 
                        str(count), ha='center', va='bottom', fontweight='bold', fontsize=10)
        
        # Set y-axis limit with some padding (avoid identical limits)
        max_count = max(counts) if counts else 1
        if max_count == 0:
            self.ax_stats.set_ylim(0, 1)  # Default range when no data
        else:
            self.ax_stats.set_ylim(0, max_count * 1.3)
        
        # Add grid for easier reading
        self.ax_stats.grid(axis='y', alpha=0.3, linestyle='--')
        
        # ==== DETAILED PROGRESS PANEL (BOTTOM RIGHT) ====
        self.ax_details.set_title("*** Progress Details", fontweight='bold', fontsize=11)
        
        # Create detailed progress text
        progress_text = ""
        
        if data['current_state'] != 'complete':
            progress_text += f"🔄 Iteration: {data['iteration']}/{data['max_iterations']}\n"
            progress_text += f"📈 Original Edges: {data['original_edge_count']}\n"
            progress_text += f"📉 Edges Removed: {len(data['pruned_edges'])}\n"
            progress_text += f"*** Current Edges: {len(data['all_current_edges'])}\n"
            
            if data['edges_to_remove']:
                progress_text += f"\n>>> Removing This Iteration:\n"
                for i, edge in enumerate(data['edges_to_remove']):
                    progress_text += f"   • Edge {edge}\n"
                    if i >= 6:  # Limit to prevent overcrowding
                        progress_text += f"   ... and {len(data['edges_to_remove']) - 6} more\n"
                        break
            
            # Progress percentage
            if data['original_edge_count'] > 0:
                progress_pct = (len(data['pruned_edges']) / data['original_edge_count']) * 100
                progress_text += f"\n📈 Progress: {progress_pct:.1f}% pruned"
        else:
            # Final summary
            progress_text += f"*** PRUNING COMPLETE! ***\n\n"
            progress_text += f"*** Final Statistics:\n"
            progress_text += f"   Original: {data['original_edge_count']} edges\n"
            progress_text += f"   Final: {len(data['all_current_edges'])} edges\n"
            progress_text += f"   Removed: {len(data['pruned_edges'])} edges\n"
            progress_text += f"   Iterations: {data['iteration']}\n"
            
            if data['original_edge_count'] > 0:
                reduction_pct = (len(data['pruned_edges']) / data['original_edge_count']) * 100
                progress_text += f"   Reduction: {reduction_pct:.1f}%\n"
            
            progress_text += f"\n>>> Network connectivity preserved!"
        
        # Display the progress text
        self.ax_details.text(0.05, 0.95, progress_text, transform=self.ax_details.transAxes, 
                    fontsize=9, verticalalignment='top', fontfamily='monospace',
                    bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
        
        self.ax_details.axis('off')
        
        # Adjust layout for better spacing
        plt.tight_layout()

    def visualize_simultaneous_pruning_result(self, original_edges_count, final_edges_count, pruned_edges):
        """
        Visualize the result of simultaneous batch pruning
        """
        print(f"\n🎨 VISUALIZING SIMULTANEOUS PRUNING RESULTS")
        print("=" * 60)
        
        # Create visualization of the final minimal connectivity graph
        plt.figure(figsize=(12, 8))
        
        # Create NetworkX graph for final state
        G = nx.Graph()
        G.add_nodes_from(self.nodes.keys())
        
        # Add remaining edges (these should all be critical)
        remaining_edges = []
        for node_id, node in self.nodes.items():
            for neighbor_id in node.direct_neighbors:
                if node_id < neighbor_id:
                    edge = (node_id, neighbor_id)
                    remaining_edges.append(edge)
                    G.add_edge(node_id, neighbor_id)
        
        # Create layout
        pos = nx.spring_layout(G, seed=42, k=3, iterations=50)
        
        # Draw the network
        plt.subplot(1, 1, 1)
        plt.title(f"Simultaneous Batch Pruning Result\n"
                 f"Critical Edges Only: {final_edges_count} edges (Removed {len(pruned_edges)} edges)", 
                 fontsize=14, fontweight='bold')
        
        # Draw nodes
        nx.draw_networkx_nodes(G, pos, node_color='lightblue', 
                              node_size=800, alpha=0.9)
        
        # Draw remaining edges (all should be critical - in red)
        nx.draw_networkx_edges(G, pos, edgelist=remaining_edges, 
                              edge_color='red', width=3, alpha=0.8)
        
        # Draw labels
        nx.draw_networkx_labels(G, pos, font_size=12, font_weight='bold')
        
        # Add legend and statistics
        stats_text = (f"Original Edges: {original_edges_count}\n"
                     f"Final Edges: {final_edges_count}\n"
                     f"Edges Removed: {len(pruned_edges)}\n"
                     f"Reduction: {(len(pruned_edges)/original_edges_count)*100:.1f}%\n"
                     f"Method: Simultaneous Batch Pruning")
        
        plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes, 
                fontsize=10, verticalalignment='top', 
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # Add legend for edge colors
        from matplotlib.lines import Line2D
        legend_elements = [Line2D([0], [0], color='red', lw=3, label='Critical Edges (Remaining)')]
        plt.legend(handles=legend_elements, loc='upper right')
        
        plt.axis('off')
        plt.tight_layout()
        plt.show()
        
        print(f"✅ Visualization complete!")
        print(f"   All {final_edges_count} remaining edges are critical for connectivity")
        print(f"   Successfully removed {len(pruned_edges)} non-critical edges using batch pruning")

    def visualize_batch_iteration(self, iteration, edges_removed, critical_edges, all_edges, non_critical_edges):
        """
        Visualize each iteration of batch pruning showing which edges were removed
        """
        print(f"\n🎨 Visualizing Iteration {iteration} - Batch Edge Removal")
        
        # Create NetworkX graph
        G = nx.Graph()
        G.add_nodes_from(self.nodes.keys())
        
        # Add all edges to the graph
        for edge in all_edges:
            G.add_edge(edge[0], edge[1])
        
        # Create layout
        pos = nx.spring_layout(G, seed=42, k=2, iterations=30)
        
        # Create figure
        plt.figure(figsize=(14, 10))
        
        # Main plot
        plt.subplot(2, 2, (1, 3))
        plt.title(f"Iteration {iteration}: Batch Removal of {len(edges_removed)} Edges\n"
                 f"Total Edges: {len(all_edges)} | Critical: {len(critical_edges)} | "
                 f"Non-Critical: {len(non_critical_edges)}", 
                 fontsize=12, fontweight='bold')
        
        # Draw nodes
        nx.draw_networkx_nodes(G, pos, node_color='lightblue', 
                              node_size=600, alpha=0.8)
        
        # Draw different types of edges
        # Critical edges (red)
        critical_edge_list = [(e[0], e[1]) for e in critical_edges if e in all_edges]
        if critical_edge_list:
            nx.draw_networkx_edges(G, pos, edgelist=critical_edge_list, 
                                  edge_color='red', width=2, alpha=0.8)
        
        # Non-critical edges remaining (gray)
        remaining_non_critical = [(e[0], e[1]) for e in non_critical_edges if e not in edges_removed]
        if remaining_non_critical:
            nx.draw_networkx_edges(G, pos, edgelist=remaining_non_critical, 
                                  edge_color='gray', width=1, alpha=0.6)
        
        # Edges being removed this iteration (yellow/orange - highlighted)
        edges_to_remove_list = [(e[0], e[1]) for e in edges_removed]
        if edges_to_remove_list:
            nx.draw_networkx_edges(G, pos, edgelist=edges_to_remove_list, 
                                  edge_color='orange', width=4, alpha=1.0)
        
        # Draw labels
        nx.draw_networkx_labels(G, pos, font_size=10, font_weight='bold')
        
        # Add legend
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], color='red', lw=2, label=f'Critical Edges ({len(critical_edges)})'),
            Line2D([0], [0], color='gray', lw=1, label=f'Non-Critical Remaining ({len(remaining_non_critical)})'),
            Line2D([0], [0], color='orange', lw=4, label=f'Being Removed ({len(edges_removed)})')
        ]
        plt.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(0, 1))
        
        plt.axis('off')
        
        # Side panel - Statistics
        plt.subplot(2, 2, 2)
        plt.title("Iteration Statistics", fontweight='bold')
        
        # Create bar chart of edge types
        categories = ['Critical', 'Non-Critical\nRemaining', 'Being\nRemoved']
        counts = [len(critical_edges), len(remaining_non_critical), len(edges_removed)]
        colors = ['red', 'gray', 'orange']
        
        bars = plt.bar(categories, counts, color=colors, alpha=0.7)
        plt.ylabel('Number of Edges')
        
        # Add count labels on bars
        for bar, count in zip(bars, counts):
            if count > 0:
                plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1, 
                        str(count), ha='center', va='bottom', fontweight='bold')
        
        plt.ylim(0, max(counts) * 1.2 if counts else 1)
        
        # Side panel - Removed edges details
        plt.subplot(2, 2, 4)
        plt.title(f"Edges Removed This Iteration", fontweight='bold')
        
        if edges_removed:
            edge_text = "Removed Edges:\n"
            for i, edge in enumerate(edges_removed):
                edge_text += f"• Edge {edge}\n"
                if i >= 8:  # Limit display to prevent crowding
                    edge_text += f"... and {len(edges_removed) - 8} more"
                    break
        else:
            edge_text = "No edges removed"
        
        plt.text(0.1, 0.9, edge_text, transform=plt.gca().transAxes, 
                fontsize=10, verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
        
        plt.axis('off')
        
        plt.tight_layout()
        plt.show()
        
        # Pause to let user observe
        input(f"\nPress Enter to continue to next iteration...")
        plt.close('all')

    def select_edges_for_batch_removal(self, non_critical_edges: set, max_batch_size: int = 5) -> list:
        """
        Select multiple edges for simultaneous removal using conflict-free batch selection
        
        Strategy:
        1. Rank all edges by priority (highest first)
        2. Select non-conflicting edges (no shared nodes)
        3. CRITICAL: Validate that removal won't disconnect any nodes
        4. Return batch of edges for simultaneous removal
        """
        if not non_critical_edges:
            return []
        
        # Compute bilateral priorities (quietly)
        priorities = self.compute_bilateral_edge_priorities(non_critical_edges)
        
        # Sort edges by priority (highest first) 
        sorted_edges = sorted(priorities.items(), key=lambda x: x[1], reverse=True)
        
        # Batch selection with conflict avoidance AND connectivity validation
        selected_edges = []
        used_nodes = set()
        
        print(f"\n*** BATCH EDGE SELECTION WITH CONNECTIVITY VALIDATION (Max Batch: {max_batch_size})")
        print("=" * 75)
        
        for edge, priority in sorted_edges:
            node1, node2 = edge
            
            # Check for conflicts (shared nodes with already selected edges)
            if node1 in used_nodes or node2 in used_nodes:
                print(f"   ⚠️ Skipping Edge {edge} → Priority: {priority:.4f} (Node conflict)")
                continue
            
            # CRITICAL CONNECTIVITY CHECK: Will removing this edge (plus already selected edges) disconnect any nodes?
            test_batch = selected_edges + [edge]
            if not self.validate_batch_connectivity(test_batch):
                print(f"   [SKIP] Skipping Edge {edge} → Priority: {priority:.4f} (Would disconnect nodes!)")
                continue
            
            # No conflict and connectivity preserved - add to batch
            selected_edges.append(edge)
            used_nodes.add(node1)
            used_nodes.add(node2)
            
            status = "👑 SELECTED" if len(selected_edges) == 1 else f"  #{len(selected_edges)}"
            print(f"{status}: Edge {edge} → Priority: {priority:.4f} (Connectivity OK)")
            
            # Stop when we reach max batch size
            if len(selected_edges) >= max_batch_size:
                break
        
        # Show remaining edges that couldn't be selected due to conflicts
        remaining_edges = len(sorted_edges) - len(selected_edges)
        if remaining_edges > 0:
            print(f"   [INFO] {remaining_edges} edges skipped due to conflicts or connectivity constraints")
        
        print(f"\n*** BATCH SUMMARY: {len(selected_edges)} edges selected for simultaneous removal")
        print("[SUCCESS] All selected edges verified to preserve network connectivity")
        
        return selected_edges

    def validate_batch_connectivity(self, edges_to_remove: List[Tuple[int, int]]) -> bool:
        """
        CRITICAL CONNECTIVITY VALIDATION:
        Check if removing a batch of edges would disconnect any nodes from the network.
        
        Returns True if connectivity is preserved, False if any node would be isolated.
        """
        if not edges_to_remove:
            return True
        
        # Create a temporary copy of the network to test connectivity
        temp_neighbors = {}
        for node_id, node in self.nodes.items():
            temp_neighbors[node_id] = set(node.direct_neighbors)
        
        # Temporarily remove the batch of edges
        for edge in edges_to_remove:
            node1, node2 = edge
            if node2 in temp_neighbors[node1]:
                temp_neighbors[node1].discard(node2)
            if node1 in temp_neighbors[node2]:
                temp_neighbors[node2].discard(node1)
        
        # Check 1: No node should have zero neighbors (isolated)
        for node_id, neighbors in temp_neighbors.items():
            if len(neighbors) == 0:
                # This node would be completely isolated!
                return False
        
        # Check 2: Network must remain connected (all nodes reachable from node 0)
        if not self.is_network_connected_with_temp_topology(temp_neighbors):
            return False
        
        return True
    
    def is_network_connected_with_temp_topology(self, temp_neighbors: Dict[int, Set[int]]) -> bool:
        """
        Check if the network is connected using a temporary topology
        Uses BFS to verify all nodes are reachable from the first node
        """
        if not temp_neighbors:
            return True
        
        # Start BFS from the first node
        start_node = next(iter(temp_neighbors.keys()))
        visited = set()
        queue = [start_node]
        visited.add(start_node)
        
        while queue:
            current_node = queue.pop(0)
            
            # Visit all neighbors of current node
            for neighbor in temp_neighbors[current_node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        
        # All nodes should be reachable
        return len(visited) == len(temp_neighbors)

    def perform_final_connectivity_validation(self):
        """
        CRITICAL FINAL VALIDATION:
        Verify that no nodes are isolated and the network is fully connected
        """
        print(f"\n🔍 FINAL CONNECTIVITY VALIDATION")
        print("=" * 50)
        
        # Check 1: No isolated nodes
        isolated_nodes = []
        for node_id, node in self.nodes.items():
            if len(node.direct_neighbors) == 0:
                isolated_nodes.append(node_id)
        
        if isolated_nodes:
            print(f"❌ CRITICAL ERROR: {len(isolated_nodes)} nodes are ISOLATED!")
            print(f"   Isolated nodes: {isolated_nodes}")
            print("   This should never happen with proper critical edge detection!")
            return False
        else:
            print(f"✅ No isolated nodes - all {len(self.nodes)} nodes have connections")
        
        # Check 2: Network connectivity using BFS from node 0
        all_node_ids = set(self.nodes.keys())
        start_node = next(iter(all_node_ids))
        
        visited = set()
        queue = [start_node]
        visited.add(start_node)
        
        while queue:
            current_node = queue.pop(0)
            
            # Visit all neighbors
            for neighbor_id in self.nodes[current_node].direct_neighbors:
                if neighbor_id not in visited:
                    visited.add(neighbor_id)
                    queue.append(neighbor_id)
        
        # Check if all nodes are reachable
        unreachable_nodes = all_node_ids - visited
        
        if unreachable_nodes:
            print(f"❌ CONNECTIVITY ERROR: {len(unreachable_nodes)} nodes are UNREACHABLE!")
            print(f"   Unreachable nodes: {sorted(unreachable_nodes)}")
            print(f"   Reachable nodes: {sorted(visited)}")
            print("   The network is fragmented into disconnected components!")
            return False
        else:
            print(f"✅ Full connectivity verified - all {len(self.nodes)} nodes are reachable")
        
        # Check 3: Degree analysis
        min_degree = min(len(node.direct_neighbors) for node in self.nodes.values())
        max_degree = max(len(node.direct_neighbors) for node in self.nodes.values())
        avg_degree = sum(len(node.direct_neighbors) for node in self.nodes.values()) / len(self.nodes)
        
        print(f"\n📊 NETWORK STATISTICS:")
        print(f"   Minimum degree: {min_degree}")
        print(f"   Maximum degree: {max_degree}")  
        print(f"   Average degree: {avg_degree:.2f}")
        
        if min_degree == 0:
            print("   ⚠️  WARNING: Minimum degree is 0 (isolated nodes exist)")
        elif min_degree == 1:
            print("   ⚠️  WARNING: Some nodes have only 1 connection (vulnerable)")
        else:
            print("   ✅ All nodes have multiple connections (robust)")
        
        print(f"\n🎯 FINAL RESULT: Network connectivity {'PRESERVED' if not unreachable_nodes and not isolated_nodes else 'VIOLATED'}")
        
        return len(isolated_nodes) == 0 and len(unreachable_nodes) == 0

    def compute_bilateral_edge_priorities(self, non_critical_edges: set) -> dict:
        """
        Compute bilateral consensus priorities for non-critical edges
        (Quiet mode - no debug output for batch processing)
        """
        bilateral_priorities = {}
        
        for edge in sorted(non_critical_edges):
            node1_id, node2_id = edge
            node1 = self.nodes[node1_id]
            node2 = self.nodes[node2_id]
            
            # Each node computes its local priority for this edge (quiet mode)
            priority_from_node1 = node1.compute_edge_priority(node2_id, self.nodes, verbose=False)
            priority_from_node2 = node2.compute_edge_priority(node1_id, self.nodes, verbose=False)
            
            # Bilateral consensus: average of both perspectives
            consensus_priority = (priority_from_node1 + priority_from_node2) / 2.0
            
            bilateral_priorities[edge] = consensus_priority
        
        return bilateral_priorities
    
    def real_time_pruning_simulation(self):
        """
        Real-time simulation in a single window showing iterative pruning
        """
        import matplotlib.animation as animation
        
        print("\n" + "=" * 70)
        print("REAL-TIME ITERATIVE PRUNING SIMULATION")
        print("Single window real-time animation - no closing/opening windows")
        print("=" * 70)
        
        # Store initial state
        self.simulation_data = {
            'iteration': 0,
            'pruned_edges': [],
            'current_state': 'detecting',  # 'detecting', 'pruning', 'complete'
            'status_message': 'Starting simulation...',
            'edge_to_remove': None,
            'critical_edges': set(),
            'non_critical_edges': set(),
            'all_edges': set()
        }
        
        # Set up the figure and axis
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.ax.set_aspect('equal')
        
        # Get positions for consistent layout
        self.pos = {node_id: node.position for node_id, node in self.nodes.items()}
        
        # Create the animation
        self.anim = animation.FuncAnimation(
            self.fig, 
            self.update_simulation, 
            interval=1000,  # Update every 1 second
            repeat=False,   # Don't repeat when done
            blit=False      # Don't use blitting for simplicity
        )
        
        plt.tight_layout()
        plt.show()
        
        return self.simulation_data['pruned_edges']
    
    def update_simulation(self, frame):
        """Update function for the real-time animation"""
        data = self.simulation_data
        
        # Clear the axis
        self.ax.clear()
        
        if data['current_state'] == 'detecting':
            # Phase 1: Detect critical edges
            data['iteration'] += 1
            
            # Re-run distributed algorithm
            num_nodes = len(self.nodes)
            for node in self.nodes.values():
                node.initialize_algorithm(num_nodes)
            
            # Run distributed algorithm (simplified for animation)
            for _ in range(min(num_nodes + 2, 8)):
                for node in self.nodes.values():
                    node.execute_one_step(self.nodes)
            
            # Detect critical edges
            critical_edges, _ = self.run_critical_edge_detection()
            data['critical_edges'] = set(tuple(sorted(edge)) for edge in critical_edges)
            
            # Get all current edges
            data['all_edges'] = set()
            for node_id, node in self.nodes.items():
                for neighbor_id in node.direct_neighbors:
                    if node_id < neighbor_id:
                        edge = (node_id, neighbor_id)
                        data['all_edges'].add(edge)
            
            # Identify non-critical edges
            data['non_critical_edges'] = data['all_edges'] - data['critical_edges']
            
            if len(data['non_critical_edges']) == 0:
                data['current_state'] = 'complete'
                data['status_message'] = 'PRUNING COMPLETE! Only critical edges remain.'
            else:
                # Select edge to remove using our sophisticated algorithm
                edge_list = list(data['non_critical_edges'])
                if len(edge_list) == 1:
                    data['edge_to_remove'] = edge_list[0]
                else:
                    # Use bilateral consensus for multiple edges
                    data['edge_to_remove'] = self.select_edge_for_removal(data['non_critical_edges'])
                    
                data['current_state'] = 'pruning'
                data['status_message'] = f'Removing non-critical edge: {data["edge_to_remove"]}'
        
        elif data['current_state'] == 'pruning':
            # Phase 2: Remove the selected edge
            edge_to_remove = data['edge_to_remove']
            node1, node2 = edge_to_remove
            
            # Remove the edge
            self.nodes[node1].direct_neighbors.discard(node2)
            self.nodes[node2].direct_neighbors.discard(node1)
            
            data['pruned_edges'].append(edge_to_remove)
            data['status_message'] = f'Removed edge {edge_to_remove}. Re-detecting critical edges...'
            data['current_state'] = 'detecting'
        
        # Visualize current state
        self.draw_current_state()
        
        # Stop animation if complete
        if data['current_state'] == 'complete':
            self.anim.event_source.stop()
            print(f"\n🎯 Simulation complete! Removed {len(data['pruned_edges'])} edges")
            print(f"Pruned edges: {data['pruned_edges']}")
    
    def draw_current_state(self):
        """Draw the current state of the graph"""
        data = self.simulation_data
        
        # Create NetworkX graph for current edges
        G = nx.Graph()
        for node_id in self.nodes.keys():
            G.add_node(node_id)
        
        for edge in data['all_edges']:
            G.add_edge(edge[0], edge[1])
        
        # Set title based on current state
        if data['current_state'] == 'detecting':
            title = f"Iteration {data['iteration']}: DETECTING Critical Edges"
            title_color = 'blue'
        elif data['current_state'] == 'pruning':
            title = f"Iteration {data['iteration']}: REMOVING Non-Critical Edge"
            title_color = 'orange'
        else:  # complete
            title = f"SIMULATION COMPLETE - Minimal Graph Achieved"
            title_color = 'green'
        
        self.ax.set_title(title, fontsize=14, fontweight='bold', color=title_color)
        
        # Draw edges based on type
        if data['current_state'] != 'complete':
            # Draw non-critical edges in gray
            if data['non_critical_edges']:
                non_critical_list = list(data['non_critical_edges'])
                nx.draw_networkx_edges(G, self.pos, edgelist=non_critical_list,
                                     edge_color='gray', width=2, alpha=0.6, ax=self.ax)
            
            # Draw critical edges in red (dashed)
            if data['critical_edges']:
                critical_list = list(data['critical_edges'])
                nx.draw_networkx_edges(G, self.pos, edgelist=critical_list,
                                     edge_color='red', width=3, alpha=0.9,
                                     style='--', ax=self.ax)
            
            # Highlight edge to remove (if in pruning phase)
            if data['current_state'] == 'pruning' and data['edge_to_remove']:
                nx.draw_networkx_edges(G, self.pos, edgelist=[data['edge_to_remove']],
                                     edge_color='yellow', width=4, alpha=1.0, ax=self.ax)
        else:
            # Final state: all edges are critical
            if data['all_edges']:
                all_edges_list = list(data['all_edges'])
                nx.draw_networkx_edges(G, self.pos, edgelist=all_edges_list,
                                     edge_color='red', width=3, alpha=0.9,
                                     style='--', ax=self.ax)
        
        # Draw nodes
        node_color = 'lightgreen' if data['current_state'] == 'complete' else 'lightblue'
        nx.draw_networkx_nodes(G, self.pos, node_color=node_color,
                              node_size=400, alpha=0.9, ax=self.ax)
        nx.draw_networkx_labels(G, self.pos, font_size=10, font_weight='bold', ax=self.ax)
        
        # Add legend
        from matplotlib.lines import Line2D
        legend_elements = []
        
        if data['current_state'] != 'complete':
            legend_elements = [
                Line2D([0], [0], color='red', lw=3, linestyle='--', label='Critical edges (KEEP)'),
                Line2D([0], [0], color='gray', lw=2, label='Non-critical edges'),
            ]
            if data['current_state'] == 'pruning':
                legend_elements.append(
                    Line2D([0], [0], color='yellow', lw=4, label='Edge being removed')
                )
        else:
            legend_elements = [
                Line2D([0], [0], color='red', lw=3, linestyle='--', label='Final critical edges only')
            ]
        
        self.ax.legend(handles=legend_elements, loc='upper right', fontsize=10)
        self.ax.grid(True, alpha=0.3)
        
        # Status information
        status_text = f"Iteration: {data['iteration']}\n"
        status_text += f"Total edges: {len(data['all_edges'])}\n"
        status_text += f"Critical edges: {len(data['critical_edges'])}\n"
        status_text += f"Non-critical edges: {len(data['non_critical_edges'])}\n"
        status_text += f"Edges removed: {len(data['pruned_edges'])}\n\n"
        status_text += f"Status: {data['status_message']}"
        
        self.ax.text(0.02, 0.98, status_text,
                    transform=self.ax.transAxes, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9),
                    fontsize=9)
    
    def animate_final_pruning_result(self, original_edges_count, final_edges_count, pruned_edges):
        """Show final comparison animation"""
        print("\n🎬 Creating final comparison animation...")
        
        # This would show the original vs final graph
        final_critical_edges, _ = self.run_critical_edge_detection()
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        
        # Get positions
        pos = {node_id: node.position for node_id, node in self.nodes.items()}
        
        # LEFT: Show what we removed (conceptual - we can't recreate original)
        ax1.set_title(f"PRUNING SUMMARY\nRemoved {len(pruned_edges)} non-critical edges", 
                     fontsize=14, fontweight='bold')
        ax1.text(0.5, 0.5, f"Original edges: {original_edges_count}\n\nRemoved edges:\n{pruned_edges}\n\nFinal edges: {final_edges_count}", 
                ha='center', va='center', transform=ax1.transAxes,
                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8),
                fontsize=12)
        ax1.set_xlim(0, 1)
        ax1.set_ylim(0, 1)
        ax1.axis('off')
        
        # RIGHT: Show final minimal graph
        ax2.set_title("FINAL MINIMAL GRAPH\n(Only critical edges remain)", 
                     fontsize=14, fontweight='bold')
        
        # Create final graph
        G_final = nx.Graph()
        for node_id in self.nodes.keys():
            G_final.add_node(node_id)
        
        final_edges = []
        for node_id, node in self.nodes.items():
            for neighbor_id in node.direct_neighbors:
                if node_id < neighbor_id:
                    edge = (node_id, neighbor_id)
                    G_final.add_edge(node_id, neighbor_id)
                    final_edges.append(edge)
        
        # All remaining edges should be critical (draw in red)
        if final_edges:
            nx.draw_networkx_edges(G_final, pos, edgelist=final_edges, 
                                 edge_color='red', width=3, alpha=0.9,
                                 style='--', ax=ax2)
        
        # Draw nodes
        nx.draw_networkx_nodes(G_final, pos, node_color='lightcoral', 
                              node_size=400, alpha=0.9, ax=ax2)
        nx.draw_networkx_labels(G_final, pos, font_size=10, font_weight='bold', ax=ax2)
        
        # Legend
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], color='red', lw=3, linestyle='--', label='Critical edges (minimal connectivity)')
        ]
        ax2.legend(handles=legend_elements, loc='upper right')
        ax2.set_aspect('equal')
        ax2.grid(True, alpha=0.3)
        
        # Final stats
        num_nodes = len(self.nodes)
        spanning_tree_min = num_nodes - 1
        redundancy = len(final_edges) - spanning_tree_min
        
        ax2.text(0.02, 0.98, f"Nodes: {num_nodes}\nEdges: {len(final_edges)}\nSpanning tree min: {spanning_tree_min}\nRedundancy: {redundancy}", 
                transform=ax2.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.8))
        
        plt.tight_layout()
        plt.show()
        
        return final_edges
    
    def goal_seeking_simulation(self, goal_x, goal_y):
        """
        Goal-seeking simulation with connectivity preservation
        Closest robot moves toward goal, others maintain connectivity
        """
        import matplotlib.animation as animation
        
        print("\n" + "=" * 70)
        print("GOAL-SEEKING SIMULATION WITH CONNECTIVITY PRESERVATION")
        print(f"Goal location: ({goal_x}, {goal_y})")
        print("Closest robot moves toward goal while preserving network connectivity")
        print("=" * 70)
        
        # Set goal location
        self.goal_location = np.array([goal_x, goal_y])
        
        # Initialize simulation parameters
        self.goal_reached = False
        self.goal_tolerance = 0.3  # How close to consider goal reached
        self.max_movement_per_frame = 0.08  # Robot movement speed
        
        # Initialize critical edge tracking
        self.current_critical_edges = []
        self.current_non_critical_edges = []
        self.pruned_edges = []
        
        # Run initial critical edge detection
        print("🔍 Running initial critical edge detection...")
        initial_critical_edges, _ = self.run_critical_edge_detection()
        self.cached_critical_edges = initial_critical_edges
        print(f"✅ Found {len(initial_critical_edges)} critical edges: {sorted(list(initial_critical_edges))}")
        
        # Create figure with subplots
        fig = plt.figure(figsize=(18, 10))
        
        # Create grid layout: main network plot (left), statistics (top right), progress (bottom right)
        gs = fig.add_gridspec(2, 3, width_ratios=[2, 1, 1], height_ratios=[1, 1])
        
        # Main network plot
        ax_network = fig.add_subplot(gs[:, 0])
        ax_network.set_title("Goal-Seeking Robot Network with Connectivity Preservation", 
                            fontsize=14, fontweight='bold')
        
        # Statistics plot (top right)
        ax_stats = fig.add_subplot(gs[0, 1:])
        ax_stats.set_title("Network Statistics", fontsize=12, fontweight='bold')
        
        # Progress details (bottom right)
        ax_details = fig.add_subplot(gs[1, 1:])
        ax_details.set_title("Goal-Seeking Progress", fontsize=12, fontweight='bold')
        
        # Initialize data for animation
        self.animation_step = 0
        self.closest_robot_id = None
        self.goal_distance_history = []
        self.connectivity_history = []
        
        def animate_goal_seeking(frame):
            """Animation function for goal-seeking simulation"""
            for ax in [ax_network, ax_stats, ax_details]:
                ax.clear()
                
            # Find closest robot to goal
            min_distance = float('inf')
            closest_id = None
            for node_id, node in self.nodes.items():
                distance = np.linalg.norm(node.position - self.goal_location)
                if distance < min_distance:
                    min_distance = distance
                    closest_id = node_id
            
            self.closest_robot_id = closest_id
            
            # Check if goal is reached
            if min_distance <= self.goal_tolerance and not self.goal_reached:
                self.goal_reached = True
                print(f"\n🎯 GOAL REACHED! Robot {closest_id} reached the goal!")
            
            # Update robot positions if goal not reached
            if not self.goal_reached:
                self.update_positions_goal_seeking()
            
            # Record statistics
            self.goal_distance_history.append(min_distance)
            total_edges = sum(len(node.direct_neighbors) for node in self.nodes.values()) // 2
            self.connectivity_history.append(total_edges)
            
            # Draw main network
            ax_network.set_title("Goal-Seeking Robot Network with Connectivity Preservation", 
                                fontsize=14, fontweight='bold')
            
            # Set workspace limits with padding
            ax_network.set_xlim(-1, 11)
            ax_network.set_ylim(-1, 11)
            ax_network.grid(True, alpha=0.3)
            
            # Draw workspace boundary
            boundary = plt.Rectangle((0, 0), 10, 10, fill=False, linewidth=3, edgecolor='black', linestyle='--')
            ax_network.add_patch(boundary)
            
            # Draw goal location
            ax_network.scatter(goal_x, goal_y, c='gold', s=200, marker='*', 
                              edgecolors='orange', linewidth=2, zorder=10, label='Goal')
            ax_network.scatter(goal_x, goal_y, c='orange', s=50, marker='o', 
                              alpha=0.3, zorder=9)  # Goal radius indicator
            
            # Draw edges with critical/non-critical color coding
            max_comm_range = 1.2  # Same as in update function
            
            # Get current edge classifications
            current_critical = getattr(self, 'current_critical_edges', [])
            current_non_critical = getattr(self, 'current_non_critical_edges', [])
            pruned = getattr(self, 'pruned_edges', [])
            
            for node_id, node in self.nodes.items():
                for neighbor_id in node.direct_neighbors:
                    if node_id < neighbor_id:  # Draw each edge only once
                        neighbor_pos = self.nodes[neighbor_id].position
                        distance = np.linalg.norm(node.position - neighbor_pos)
                        edge = (node_id, neighbor_id)
                        
                        # Determine edge type and color
                        if edge in current_critical:
                            # Critical edges - must be preserved (RED)
                            color = 'red'
                            width = 2.5
                            alpha = 0.9
                            style = '-'
                        elif edge in current_non_critical:
                            # Non-critical edges - can be pruned (BLUE)
                            color = 'blue' 
                            width = 1.5
                            alpha = 0.6
                            style = '--'
                        else:
                            # Default (shouldn't happen but just in case)
                            color = 'gray'
                            width = 1.0
                            alpha = 0.4
                            style = ':'
                        
                        # Draw the edge
                        ax_network.plot([node.position[0], neighbor_pos[0]], 
                                       [node.position[1], neighbor_pos[1]], 
                                       color=color, linewidth=width, alpha=alpha, linestyle=style)
            
            # Draw communication range circles for visualization (optional - only for closest robot)
            if closest_id is not None:
                closest_pos = self.nodes[closest_id].position
                comm_circle = plt.Circle((closest_pos[0], closest_pos[1]), max_comm_range, 
                                       fill=False, linestyle=':', alpha=0.3, color='gray')
                ax_network.add_patch(comm_circle)
            
            # Draw nodes
            for node_id, node in self.nodes.items():
                color = 'red' if node_id == closest_id else 'lightblue'
                size = 300 if node_id == closest_id else 200
                ax_network.scatter(node.position[0], node.position[1], 
                                  c=color, s=size, alpha=0.8, edgecolors='black')
                ax_network.annotate(str(node_id), (node.position[0], node.position[1]), 
                                   xytext=(0, 0), textcoords='offset points', 
                                   ha='center', va='center', fontweight='bold')
            
            # Add updated legend
            from matplotlib.lines import Line2D
            legend_elements = [
                Line2D([0], [0], color='gold', marker='*', markersize=8, label='Goal', linestyle='None'),
                Line2D([0], [0], color='red', linewidth=2.5, label='Critical Edges (Must Keep)', linestyle='-'),
                Line2D([0], [0], color='blue', linewidth=1.5, label='Non-Critical Edges (Can Prune)', linestyle='--'),
                Line2D([0], [0], marker='o', color='red', markersize=8, label='Active Robot', linestyle='None'),
                Line2D([0], [0], marker='o', color='lightblue', markersize=6, label='Other Robots', linestyle='None')
            ]
            ax_network.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(0.98, 0.98))
            
            # Draw statistics
            ax_stats.set_title("Network Statistics", fontsize=12, fontweight='bold')
            if len(self.goal_distance_history) > 1:
                ax_stats.plot(self.goal_distance_history, 'r-', linewidth=2, label='Distance to Goal')
                ax_stats.plot(self.connectivity_history, 'b-', linewidth=2, label='Edge Count')
                ax_stats.set_xlabel('Animation Steps')
                ax_stats.set_ylabel('Values')
                ax_stats.legend()
                ax_stats.grid(True, alpha=0.3)
            else:
                ax_stats.text(0.5, 0.5, 'Collecting data...', ha='center', va='center', 
                             transform=ax_stats.transAxes, fontsize=12)
            
            # Draw progress details
            ax_details.set_title("Goal-Seeking Progress", fontsize=12, fontweight='bold')
            
            # Calculate edge statistics
            current_critical = getattr(self, 'current_critical_edges', [])
            current_non_critical = getattr(self, 'current_non_critical_edges', [])
            pruned = getattr(self, 'pruned_edges', [])
            
            details_text = f"Animation Step: {self.animation_step}\n"
            details_text += f"Active Robot: {closest_id}\n"
            details_text += f"Distance to Goal: {min_distance:.2f}\n"
            details_text += f"Goal Reached: {'YES' if self.goal_reached else 'NO'}\n\n"
            details_text += f"EDGE ANALYSIS:\n"
            details_text += f"Critical Edges: {len(current_critical)}\n"
            details_text += f"Non-Critical Edges: {len(current_non_critical)}\n"
            details_text += f"Pruned Edges: {len(pruned)}\n"
            details_text += f"Total Active: {len(current_critical) + len(current_non_critical)}\n\n"
            details_text += f"Network Connected: {'YES' if self.is_network_connected() else 'NO'}\n"
            details_text += f"Comm. Range: 2.8 units\n"
            details_text += f"Goal: ({goal_x:.1f}, {goal_y:.1f})"
            
            ax_details.text(0.05, 0.95, details_text, transform=ax_details.transAxes, 
                           verticalalignment='top', fontsize=10,
                           bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
            ax_details.axis('off')
            
            # Status message at bottom
            status_msg = "🎯 GOAL REACHED! Simulation complete!" if self.goal_reached else "🚀 Moving toward goal..."
            ax_details.text(0.5, 0.02, status_msg, transform=ax_details.transAxes, 
                           ha='center', va='bottom', fontsize=11, fontweight='bold',
                           color='green' if self.goal_reached else 'blue')
            
            self.animation_step += 1
            
            # Stop animation when goal is reached and after a few more frames
            if self.goal_reached and self.animation_step > len(self.goal_distance_history) + 20:
                return []
                
            return []
        
        # Create and run animation
        print("🎬 Starting goal-seeking animation...")
        print("Close the window to stop the simulation")
        
        anim = animation.FuncAnimation(fig, animate_goal_seeking, frames=1000, 
                                     interval=100, blit=False, repeat=False)
        
        plt.tight_layout()
        plt.show()
        
        # Final results
        print(f"\n{'=' * 70}")
        print("GOAL-SEEKING SIMULATION COMPLETE")
        if self.goal_distance_history:
            print(f"Final distance to goal: {self.goal_distance_history[-1]:.2f}")
        else:
            print("Final distance to goal: N/A (simulation ended early)")
        print(f"Goal reached: {'YES' if self.goal_reached else 'NO'}")
        print(f"Total animation steps: {self.animation_step}")
        print(f"Network remained connected: {'YES' if self.is_network_connected() else 'NO'}")
        print(f"{'=' * 70}")
        
        return self.goal_reached
    
    def calculate_constrained_position(self, robot_position, goal_position, neighbor_position, max_range):
        """
        Calculate optimal position for robot to move toward goal while staying within max_range of neighbor
        Returns the target position on the communication circle around the neighbor
        """
        # Direction toward goal
        direction_to_goal = goal_position - robot_position
        if np.linalg.norm(direction_to_goal) == 0:
            return robot_position
        
        direction_to_goal_norm = direction_to_goal / np.linalg.norm(direction_to_goal)
        
        # Circle parameters (around the neighbor)
        circle_center = neighbor_position
        radius = max_range * 0.95  # Stay slightly inside max range for safety
        
        # Find point on circle closest to the goal direction
        # Project goal direction onto circle around neighbor
        center_to_robot = robot_position - circle_center
        
        # If robot is already inside the circle, project goal direction onto circle
        robot_to_goal_from_center = direction_to_goal_norm
        
        # Calculate target position on circle boundary that's closest to goal direction
        target_position = circle_center + robot_to_goal_from_center * radius
        
        return target_position
    
    def compute_clf_goal_seeking(self, robot_position, goal_position):
        """
        Compute Control Lyapunov Function for goal-seeking
        CLF: V(x) = ||x - x_goal||^2
        CLF derivative: dV/dx = 2(x - x_goal)
        """
        error = robot_position - goal_position
        V = np.dot(error, error)  # ||x - x_goal||^2
        dV_dx = 2 * error  # Gradient of V
        return V, dV_dx
    
    def compute_cbf_connectivity(self, robot_position, neighbor_positions, max_range):
        """
        Compute Control Barrier Function for connectivity constraint
        CBF: h(x) = R^2 - ||x - x_neighbor||^2 for each neighbor
        CBF derivative: dh/dx = -2(x - x_neighbor)
        Returns constraints for maintaining connectivity
        """
        cbf_constraints = []
        
        for neighbor_pos in neighbor_positions:
            # Distance to neighbor
            diff = robot_position - neighbor_pos
            distance_sq = np.dot(diff, diff)
            
            # CBF: h(x) = R^2 - ||x - x_neighbor||^2
            # We want h(x) >= 0 to stay within communication range
            h = max_range**2 - distance_sq
            dh_dx = -2 * diff  # Gradient of h
            
            cbf_constraints.append({
                'h': h,
                'dh_dx': dh_dx,
                'neighbor_pos': neighbor_pos,
                'distance': np.sqrt(distance_sq),
                'max_range': max_range  # Include max_range for emergency braking logic
            })
        
        return cbf_constraints
    
    def solve_clf_cbf_qp(self, robot_position, goal_position, neighbor_positions, max_range):
        """
        Solve CLF-CBF Quadratic Program to find optimal control input
        
        Formulation:
        minimize: u^T u + gamma^2
        subject to: 
        - CLF constraint: dV/dx * u <= -alpha * V + gamma (goal reaching)
        - CBF constraint: dh/dx * u >= -beta * h (connectivity maintenance)
        - Control bounds: ||u|| <= u_max
        """
        try:
            # Compute CLF for goal-seeking
            V, dV_dx = self.compute_clf_goal_seeking(robot_position, goal_position)
            
            # Compute CBF for connectivity
            cbf_constraints = self.compute_cbf_connectivity(robot_position, neighbor_positions, max_range)
            
            if CVXPY_AVAILABLE:
                return self._solve_clf_cbf_cvxpy(V, dV_dx, cbf_constraints)
            else:
                return self._solve_clf_cbf_scipy(V, dV_dx, cbf_constraints)
                
        except Exception as e:
            print(f"CLF-CBF QP failed: {e}, using fallback control")
            # Fallback: simple proportional control toward goal
            direction_to_goal = goal_position - robot_position
            if np.linalg.norm(direction_to_goal) > 0:
                return 0.05 * direction_to_goal / np.linalg.norm(direction_to_goal)
            else:
                return np.zeros(2)
    
    def _solve_clf_cbf_cvxpy(self, V, dV_dx, cbf_constraints):
        """Solve CLF-CBF QP using cvxpy"""
        # Control input (2D velocity)
        u = cp.Variable(2)
        gamma = cp.Variable(1)  # Relaxation variable for CLF
        
        # Parameters
        alpha = 1.0  # CLF decay rate
        beta = 1.0   # CBF safety margin - strengthened for better constraint enforcement
        u_max = 0.03  # Maximum control input magnitude - reduced for better control
        
        # Objective: minimize control effort + relaxation penalty
        objective = cp.Minimize(cp.sum_squares(u) + 100 * cp.sum_squares(gamma))
        
        constraints = []
        
        # CLF constraint: dV/dx * u <= -alpha * V + gamma
        if V > 1e-6:  # Only apply if not already at goal
            constraints.append(dV_dx @ u <= -alpha * V + gamma)
            constraints.append(gamma >= 0)  # Relaxation variable is non-negative
        
        # CBF constraints: dh/dx * u >= -beta * h for each neighbor
        emergency_braking = False
        for cbf in cbf_constraints:
            if cbf['h'] > 0:  # Only apply if constraint is active
                # Check for emergency braking condition (95% of max range)
                if cbf['h'] < 0.05 * cbf.get('max_range', 1.2):  # Within 5% of communication limit
                    emergency_braking = True
                    # Hard constraint: force velocity away from neighbor
                    constraints.append(cbf['dh_dx'] @ u >= 0.1)  # Strong repulsive force
                else:
                    constraints.append(cbf['dh_dx'] @ u >= -beta * cbf['h'])
        
        # If emergency braking is active, reduce control authority significantly
        if emergency_braking:
            u_max = min(u_max, 0.01)  # Very limited movement during emergency
        
        # Control input bounds
        constraints.append(cp.norm(u, 2) <= u_max)
        
        # Solve the problem
        prob = cp.Problem(objective, constraints)
        prob.solve(solver=cp.ECOS, verbose=False)
        
        if prob.status == cp.OPTIMAL:
            return u.value
        else:
            print(f"CLF-CBF optimization failed with status: {prob.status}")
            return np.zeros(2)
    
    def _solve_clf_cbf_scipy(self, V, dV_dx, cbf_constraints):
        """Solve CLF-CBF QP using scipy (fallback when cvxpy not available)"""
        from scipy.optimize import minimize
        
        # Parameters
        alpha = 1.0
        beta = 1.0  # Strengthened CBF parameter
        u_max = 0.03  # Reduced for better control
        
        def objective(x):
            u = x[:2]
            gamma = x[2] if len(x) > 2 else 0
            return np.dot(u, u) + 100 * gamma**2
        
        # Constraints
        constraints = []
        
        # CLF constraint: dV/dx * u <= -alpha * V + gamma
        if V > 1e-6:
            def clf_constraint(x):
                u = x[:2]
                gamma = x[2] if len(x) > 2 else 0
                return -alpha * V + gamma - np.dot(dV_dx, u)
            constraints.append({'type': 'ineq', 'fun': clf_constraint})
            
            # Relaxation variable bounds
            bounds = [(-u_max, u_max), (-u_max, u_max), (0, 10)]
        else:
            bounds = [(-u_max, u_max), (-u_max, u_max)]
        
        # CBF constraints with emergency braking
        emergency_braking = False
        for cbf in cbf_constraints:
            if cbf['h'] > 0:
                # Check for emergency braking condition (95% of max range)
                if cbf['h'] < 0.05 * cbf.get('max_range', 1.2):  # Within 5% of communication limit
                    emergency_braking = True
                    # Hard constraint: force velocity away from neighbor
                    def emergency_cbf_constraint(x, dh_dx=cbf['dh_dx']):
                        u = x[:2]
                        return np.dot(dh_dx, u) - 0.1  # Strong repulsive force
                    constraints.append({'type': 'ineq', 'fun': emergency_cbf_constraint})
                else:
                    def cbf_constraint(x, dh_dx=cbf['dh_dx'], h=cbf['h']):
                        u = x[:2]
                        return np.dot(dh_dx, u) + beta * h
                    constraints.append({'type': 'ineq', 'fun': cbf_constraint})
        
        # If emergency braking is active, reduce control authority
        if emergency_braking:
            u_max = min(u_max, 0.01)
            bounds = [(-u_max, u_max), (-u_max, u_max)] if len(bounds) == 2 else [(-u_max, u_max), (-u_max, u_max), (0, 10)]
        
        # Initial guess
        x0 = np.zeros(len(bounds))
        
        # Solve
        result = minimize(objective, x0, method='SLSQP', bounds=bounds, constraints=constraints)
        
        if result.success:
            return result.x[:2]
        else:
            print(f"CLF-CBF scipy optimization failed: {result.message}")
            return np.zeros(2)
    
    def update_positions_goal_seeking(self):
        """Update robot positions with goal-seeking and critical edge preservation"""
        if self.closest_robot_id is None:
            return
        
        # STEP 1: Run distributed critical edge detection algorithm periodically 
        if not hasattr(self, 'algorithm_counter'):
            self.algorithm_counter = 0
            self.cached_critical_edges = set()  # Cache results
        
        self.algorithm_counter += 1
        
        # Run the expensive critical edge detection every 5 frames
        if self.algorithm_counter % 5 == 0:
            for node in self.nodes.values():
                node.execute_one_step(self.nodes)
            # Cache the critical edges from the algorithm
            all_critical_edges, node_critical_edges = self.run_critical_edge_detection()
            self.cached_critical_edges = all_critical_edges
        
        # STEP 2: Classify current edges using cached critical edges (every frame)
        critical_edges = []
        non_critical_edges = []
        max_communication_range = 1.2  # Reduced from 2.8 for more aggressive pruning
        
        for node_id, node in self.nodes.items():
            for neighbor_id in node.direct_neighbors:
                if node_id < neighbor_id:  # Count each edge only once
                    edge = (node_id, neighbor_id)
                    distance = np.linalg.norm(node.position - self.nodes[neighbor_id].position)
                    
                    # Use cached critical edges from the algorithm
                    is_algorithm_critical = edge in self.cached_critical_edges
                    
                    # An edge is critical if the algorithm identifies it as critical
                    # OR if removing it would exceed communication range for a critical path
                    if is_algorithm_critical:
                        critical_edges.append(edge)
                    else:
                        non_critical_edges.append(edge)
        
        # STEP 3: Aggressively prune non-critical edges that exceed communication range
        edges_to_remove = []
        for edge in non_critical_edges:
            node1, node2 = edge
            distance = np.linalg.norm(self.nodes[node1].position - self.nodes[node2].position)
            
            # More aggressive pruning - remove non-critical edges at 80% of max range
            if distance > max_communication_range * 0.8:
                edges_to_remove.append(edge)
        
        # Actually remove the edges from the network (with connectivity check)
        successfully_removed = []
        for edge in edges_to_remove:
            node1, node2 = edge
            
            # Temporarily remove the edge to test connectivity
            edge_existed = False
            if node2 in self.nodes[node1].direct_neighbors:
                self.nodes[node1].direct_neighbors.remove(node2)
                edge_existed = True
            if node1 in self.nodes[node2].direct_neighbors:
                self.nodes[node2].direct_neighbors.remove(node1)
            
            # Check if network is still connected
            if edge_existed and self.is_network_connected():
                # Network remains connected, keep the edge removed
                successfully_removed.append(edge)
            elif edge_existed:
                # Network would disconnect, restore the edge
                self.nodes[node1].direct_neighbors.add(node2)
                self.nodes[node2].direct_neighbors.add(node1)
        
        # Store current state for visualization (use REAL-TIME critical edge detection)
        actual_critical_edges = []
        actual_non_critical_edges = []
        
        # For each remaining edge, check if removing it would disconnect the network
        current_edges = []
        for node_id, node in self.nodes.items():
            for neighbor_id in node.direct_neighbors:
                if node_id < neighbor_id:  # Count each edge only once
                    current_edges.append((node_id, neighbor_id))
        
        # Test each edge to see if it's critical
        for edge in current_edges:
            node1, node2 = edge
            
            # Temporarily remove the edge
            was_connected_1 = node2 in self.nodes[node1].direct_neighbors
            was_connected_2 = node1 in self.nodes[node2].direct_neighbors
            
            if was_connected_1:
                self.nodes[node1].direct_neighbors.remove(node2)
            if was_connected_2:
                self.nodes[node2].direct_neighbors.remove(node1)
            
            # Check if network is still connected
            is_still_connected = self.is_network_connected()
            
            # Restore the edge
            if was_connected_1:
                self.nodes[node1].direct_neighbors.add(node2)
            if was_connected_2:
                self.nodes[node2].direct_neighbors.add(node1)
            
            # Classify the edge
            if not is_still_connected:
                # Removing this edge disconnects the network - it's CRITICAL
                actual_critical_edges.append(edge)
            else:
                # Network stays connected without this edge - it's NON-CRITICAL
                actual_non_critical_edges.append(edge)
        
        self.current_critical_edges = actual_critical_edges
        self.current_non_critical_edges = actual_non_critical_edges
        self.pruned_edges = successfully_removed
        
        # Debug output every 10 frames
        if hasattr(self, 'debug_counter'):
            self.debug_counter += 1
        else:
            self.debug_counter = 0
            
        if self.debug_counter % 10 == 0:
            print(f"🔍 DEBUG Frame {self.debug_counter}:")
            print(f"   Current edges in network: {current_edges}")
            print(f"   Classified as Critical: {actual_critical_edges}")
            print(f"   Classified as Non-Critical: {actual_non_critical_edges}")
            print(f"   Successfully removed: {successfully_removed}")
            print(f"   self.current_critical_edges = {self.current_critical_edges}")
            print("   " + "-"*50)
        
        # Initialize velocities if not present
        for node in self.nodes.values():
            if not hasattr(node, 'velocity'):
                node.velocity = np.random.normal(0, 0.02, 2)
        
        # STEP 4: Update robot positions with intelligent movement
        spring_strength = 0.06
        max_communication_range = 1.2  # Same as pruning logic
        
        for node_id, node in self.nodes.items():
            # Initialize velocities if not present
            if not hasattr(node, 'velocity'):
                node.velocity = np.random.normal(0, 0.02, 2)
            
            # CLF-CBF CONTROL for active robot, force-based for others
            if node_id == self.closest_robot_id:
                # === CLF-CBF CONTROL FOR ACTIVE ROBOT ===
                print(f"🤖 Using CLF-CBF control for active robot {node_id}")
                
                # Get neighbor positions for CBF constraints
                neighbor_positions = []
                for neighbor_id in node.direct_neighbors:
                    neighbor_positions.append(self.nodes[neighbor_id].position)
                
                # Solve CLF-CBF QP
                if len(neighbor_positions) > 0:
                    control_input = self.solve_clf_cbf_qp(
                        robot_position=node.position,
                        goal_position=self.goal_location,
                        neighbor_positions=neighbor_positions,
                        max_range=max_communication_range
                    )
                else:
                    # No neighbors - just go toward goal
                    direction_to_goal = self.goal_location - node.position
                    if np.linalg.norm(direction_to_goal) > 0:
                        control_input = 0.05 * direction_to_goal / np.linalg.norm(direction_to_goal)
                    else:
                        control_input = np.zeros(2)
                
                # Apply CLF-CBF control
                node.velocity = control_input
                
                # Debug output for CLF-CBF
                if hasattr(self, 'clf_cbf_debug_counter'):
                    self.clf_cbf_debug_counter += 1
                else:
                    self.clf_cbf_debug_counter = 0
                
                if self.clf_cbf_debug_counter % 30 == 0:  # Every 30 frames
                    dist_to_goal = np.linalg.norm(node.position - self.goal_location)
                    print(f"🎯 CLF-CBF Control Robot {node_id}:")
                    print(f"   Distance to goal: {dist_to_goal:.3f}")
                    print(f"   Control input: [{control_input[0]:.4f}, {control_input[1]:.4f}]")
                    print(f"   Connected neighbors: {len(neighbor_positions)}")
                    for i, neighbor_pos in enumerate(neighbor_positions):
                        dist = np.linalg.norm(node.position - neighbor_pos)
                        print(f"   Neighbor {i+1} distance: {dist:.3f} (max: {max_communication_range:.3f})")
            
            else:
                # === FORCE-BASED CONTROL FOR NON-ACTIVE ROBOTS ===
                # Base Brownian motion for all other robots
                brownian_force = np.random.normal(0, 0.025, 2)
                total_force = brownian_force
                
                # Smart connectivity forces based on CRITICAL EDGES ONLY
                for neighbor_id in node.direct_neighbors:
                    neighbor = self.nodes[neighbor_id]
                    direction = neighbor.position - node.position
                    distance = np.linalg.norm(direction)
                    
                    if distance > 0:
                        edge = tuple(sorted([node_id, neighbor_id]))
                        is_critical = edge in critical_edges
                        
                        if is_critical:
                            # STRONG force to maintain critical connections
                            if distance > max_communication_range * 0.8:
                                # Pull strongly toward critical neighbors when far
                                spring_force = (direction / distance) * spring_strength * 3.0
                                total_force += spring_force
                            elif distance < 0.6:
                                # Gentle repulsion if too close to maintain spreading
                                repulsion_force = -(direction / distance) * spring_strength * 0.3 * (0.6 - distance)
                                total_force += repulsion_force
                        else:
                            # WEAK force for non-critical connections - allow natural pruning
                            if distance > max_communication_range * 0.9:
                                spring_force = (direction / distance) * spring_strength * 0.5
                                total_force += spring_force
                
                # Spreading force - encourage robots to spread out intelligently
                spreading_force = np.zeros(2)
                for other_id, other_node in self.nodes.items():
                    if other_id != node_id:
                        direction_away = node.position - other_node.position
                        distance = np.linalg.norm(direction_away)
                        
                        # Only spread from very close robots (avoid clustering)
                        if distance > 0 and distance < 1.2:
                            edge = tuple(sorted([node_id, other_id]))
                            is_connected = other_id in node.direct_neighbors
                            is_critical = edge in critical_edges if is_connected else False
                            
                            # Stronger spreading if not critically connected
                            spread_strength = 0.015 if is_critical else 0.025
                            spreading_force += (direction_away / distance) * spread_strength * (1.2 - distance)
                
                total_force += spreading_force
                
                # Update velocity with momentum (force-based control)
                node.velocity = 0.75 * node.velocity + total_force
            
            # Apply velocity
            node.position += node.velocity
            
            # Boundary handling with reflection
            if node.position[0] <= 0 or node.position[0] >= 10:
                node.velocity[0] *= -0.9
                node.position[0] = np.clip(node.position[0], 0, 10)
            
            if node.position[1] <= 0 or node.position[1] >= 10:
                node.velocity[1] *= -0.9
                node.position[1] = np.clip(node.position[1], 0, 10)

def create_random_graph(num_nodes: int = 10, edge_probability: float = 0.4, preferred_type: str = 'random') -> DistributedNetwork:
    """Create a random connected graph starting in 1-unit radius, 10x10 workspace"""
    import random
    
    print(f"\nGenerating random connected graph with {num_nodes} nodes...")
    print(f"Initial radius: 1 unit (clustered start)")
    print(f"Workspace: 10x10 units (robots can spread out)")
    print(f"Preferred structure: {preferred_type}")
    print(f"Edge probability: {edge_probability}")
    print("-" * 60)
    
    # Create empty network (don't auto-initialize)
    network = DistributedNetwork(create_default_graph=False)
    
    # Generate positions in a circle within 1-unit radius (clustered start)
    positions = {}
    for i in range(1, num_nodes + 1):
        angle = 2 * np.pi * (i - 1) / num_nodes
        # Start clustered within 1-unit radius
        radius = 0.7 + random.uniform(-0.1, 0.1)  # Small random variation
        x = radius * np.cos(angle) + random.uniform(-0.1, 0.1)
        y = radius * np.sin(angle) + random.uniform(-0.1, 0.1)
        positions[i] = (x, y)
    
    # Add nodes
    for node_id in range(1, num_nodes + 1):
        network.nodes[node_id] = Node(node_id, np.array(positions[node_id]))
    
    # Strategy: Create different types of graph structures
    edges = []
    
    # Handle triangle_rich specially - use dedicated function
    if preferred_type == 'triangle_rich':
        print("🔺 Creating EXPLICIT triangle-rich graph...")
        return create_explicit_triangle_graph(num_nodes)
    
    # Handle complete graph specially - use dedicated function
    if preferred_type == 'complete':
        print("🔗 Creating COMPLETE graph...")
        return create_complete_graph(num_nodes)
    
    # Choose graph type based on preference
    if preferred_type == 'random':
        graph_types = ['path', 'star', 'tree', 'cycle_plus']
        chosen_type = random.choice(graph_types)
    else:
        chosen_type = preferred_type
    
    print(f"Creating {chosen_type}-like structure...")
    
    if chosen_type == 'path':
        # Create a path with some random connections
        nodes_list = list(range(1, num_nodes + 1))
        random.shuffle(nodes_list)
        for i in range(len(nodes_list) - 1):
            edges.append((nodes_list[i], nodes_list[i + 1]))
        
        # Add 1-3 random shortcuts
        for _ in range(random.randint(1, max(1, num_nodes // 4))):
            i, j = random.sample(nodes_list, 2)
            if (i, j) not in edges and (j, i) not in edges and abs(nodes_list.index(i) - nodes_list.index(j)) > 1:
                edges.append((i, j))
    
    elif chosen_type == 'star':
        # Create a star with additional connections
        center = random.randint(1, num_nodes)
        for i in range(1, num_nodes + 1):
            if i != center:
                edges.append((center, i))
        
        # Add some connections between leaf nodes
        leaf_nodes = [i for i in range(1, num_nodes + 1) if i != center]
        for _ in range(random.randint(1, max(1, len(leaf_nodes) // 3))):
            if len(leaf_nodes) >= 2:
                i, j = random.sample(leaf_nodes, 2)
                if (i, j) not in edges and (j, i) not in edges:
                    edges.append((i, j))
    
    elif chosen_type == 'cycle_plus':
        # Create a cycle with additional edges
        nodes_list = list(range(1, num_nodes + 1))
        random.shuffle(nodes_list)
        for i in range(len(nodes_list)):
            edges.append((nodes_list[i], nodes_list[(i + 1) % len(nodes_list)]))
        
        # Add some chords to the cycle
        for _ in range(random.randint(1, max(1, num_nodes // 3))):
            i, j = random.sample(nodes_list, 2)
            if (i, j) not in edges and (j, i) not in edges:
                edges.append((i, j))
    
    else:  # 'tree'
        # Create a random spanning tree using modified Kruskal-like approach
        nodes_list = list(range(1, num_nodes + 1))
        random.shuffle(nodes_list)
        
        in_tree = {nodes_list[0]}
        remaining = set(nodes_list[1:])
        
        while remaining:
            # Pick a random node already in tree
            tree_node = random.choice(list(in_tree))
            # Pick a random node not in tree
            new_node = random.choice(list(remaining))
            
            edges.append((tree_node, new_node))
            in_tree.add(new_node)
            remaining.remove(new_node)
        
        # Add some extra edges to make it more interesting
        for _ in range(random.randint(1, max(1, num_nodes // 4))):
            i, j = random.sample(range(1, num_nodes + 1), 2)
            if (i, j) not in edges and (j, i) not in edges:
                edges.append((i, j))
    
    # Add all edges to network
    for node1, node2 in edges:
        network.nodes[node1].add_direct_neighbor(node2)
        network.nodes[node2].add_direct_neighbor(node1)
    
    print(f"Generated {len(edges)} edges: {sorted(edges)}")
    
    # Initialize all nodes
    for node in network.nodes.values():
        node.initialize_algorithm(num_nodes)
    
    # Print initial state
    print("Initial Graph Structure:")
    network.print_graph_structure()
    print("\nInitial State (k=0):")
    network.print_all_states()
    
    # Verify connectivity
    if not network.check_connectivity_status():
        print("WARNING: Graph is not connected! Retrying...")
        return create_random_graph(num_nodes, edge_probability)
    
    print(f"✅ Successfully created connected random graph!")
    return network

def create_explicit_triangle_graph(num_nodes: int = 8) -> DistributedNetwork:
    """
    Create a graph with SIMPLE EXPLICIT triangles - absolutely guaranteed
    """
    print(f"\n🔺 CREATING SIMPLE TRIANGLE GRAPH")
    print(f"   Nodes: {num_nodes}")
    print("   Creating FOOLPROOF triangles...")
    print("=" * 50)
    
    # Create network
    network = DistributedNetwork(create_default_graph=False)
    
    # Create nodes
    for i in range(1, num_nodes + 1):
        angle = 2 * np.pi * (i - 1) / num_nodes
        x = 2 * np.cos(angle)
        y = 2 * np.sin(angle)
        network.nodes[i] = Node(i, np.array([x, y]))
    
    print("🔺 Creating triangle (1,2,3) - GUARANTEED triangle:")
    print("   Adding edge (1,2)")
    print("   Adding edge (2,3)")  
    print("   Adding edge (1,3)")
    print("   Result: Node 1 and Node 2 will have common neighbor 3!")
    
    # Create ONE SIMPLE triangle: (1,2,3)
    network.nodes[1].add_direct_neighbor(2)
    network.nodes[2].add_direct_neighbor(1)
    
    network.nodes[2].add_direct_neighbor(3)
    network.nodes[3].add_direct_neighbor(2)
    
    network.nodes[1].add_direct_neighbor(3)  # This completes the triangle!
    network.nodes[3].add_direct_neighbor(1)
    
    # Add a few more edges but not in triangles (to test the difference)
    if num_nodes >= 4:
        print("🔗 Adding non-triangle edge (1,4)")
        network.nodes[1].add_direct_neighbor(4)
        network.nodes[4].add_direct_neighbor(1)
    
    if num_nodes >= 5:
        print("🔗 Adding non-triangle edge (4,5)")
        network.nodes[4].add_direct_neighbor(5)
        network.nodes[5].add_direct_neighbor(4)
        
        print("🔺 Creating another triangle (3,4,5):")
        network.nodes[3].add_direct_neighbor(4)
        network.nodes[4].add_direct_neighbor(3)
        
        network.nodes[3].add_direct_neighbor(5)
        network.nodes[5].add_direct_neighbor(3)
    
    # Connect remaining nodes simply
    for i in range(6, num_nodes + 1):
        # Just connect to node 1 (no triangles)
        network.nodes[1].add_direct_neighbor(i)
        network.nodes[i].add_direct_neighbor(1)
    
    # Initialize algorithm
    for node in network.nodes.values():
        node.initialize_algorithm(len(network.nodes))
    
    print(f"\n✅ SIMPLE TRIANGLE GRAPH CREATED!")
    
    # MANUAL VERIFICATION
    print("\n🔍 MANUAL TRIANGLE VERIFICATION:")
    
    print("📊 Node neighbor lists:")
    for node_id in sorted(network.nodes.keys()):
        neighbors = sorted(list(network.nodes[node_id].direct_neighbors))
        print(f"   Node {node_id}: {neighbors}")
    
    print("\n🔺 Checking triangle (1,2,3):")
    if 1 in network.nodes and 2 in network.nodes and 3 in network.nodes:
        node1_neighbors = network.nodes[1].direct_neighbors
        node2_neighbors = network.nodes[2].direct_neighbors  
        node3_neighbors = network.nodes[3].direct_neighbors
        
        print(f"   Node 1 neighbors: {sorted(node1_neighbors)}")
        print(f"   Node 2 neighbors: {sorted(node2_neighbors)}")
        print(f"   Node 3 neighbors: {sorted(node3_neighbors)}")
        
        # Check edge (1,2) common neighbors
        common_12 = node1_neighbors.intersection(node2_neighbors)
        print(f"   Edge (1,2) common neighbors: {sorted(common_12)}")
        print(f"   Expected: should contain 3 = {'✅' if 3 in common_12 else '❌'}")
        
        # Check edge (2,3) common neighbors  
        common_23 = node2_neighbors.intersection(node3_neighbors)
        print(f"   Edge (2,3) common neighbors: {sorted(common_23)}")
        print(f"   Expected: should contain 1 = {'✅' if 1 in common_23 else '❌'}")
        
        # Check edge (1,3) common neighbors
        common_13 = node1_neighbors.intersection(node3_neighbors)
        print(f"   Edge (1,3) common neighbors: {sorted(common_13)}")
        print(f"   Expected: should contain 2 = {'✅' if 2 in common_13 else '❌'}")
    
    if num_nodes >= 5:
        print("\n� Checking triangle (3,4,5):")
        common_34 = network.nodes[3].direct_neighbors.intersection(network.nodes[4].direct_neighbors)
        common_45 = network.nodes[4].direct_neighbors.intersection(network.nodes[5].direct_neighbors)  
        common_35 = network.nodes[3].direct_neighbors.intersection(network.nodes[5].direct_neighbors)
        
        print(f"   Edge (3,4) common neighbors: {sorted(common_34)} (should contain 5)")
        print(f"   Edge (4,5) common neighbors: {sorted(common_45)} (should contain 3)")
        print(f"   Edge (3,5) common neighbors: {sorted(common_35)} (should contain 4)")
    
    return network

def create_complete_graph(num_nodes: int = 5) -> DistributedNetwork:
    """
    Create a COMPLETE GRAPH structure within 1-unit radius initial workspace
    Every node connects to every other node = MAXIMUM triangles
    Initial positions are clustered in center, then robots can spread out in 10x10 workspace
    """
    print(f"\n🔺 CREATING COMPLETE GRAPH (Maximum Triangles)")
    print(f"   Nodes: {num_nodes}")
    print("   Initial radius: 1 unit (clustered start)")
    print("   Workspace: 10x10 units (robots can spread out)")
    print("   Structure: Every node connects to every other node")
    print("   Result: MAXIMUM possible triangles!")
    print("=" * 70)
    
    # Create network
    network = DistributedNetwork(create_default_graph=False)
    
    # Create nodes in a circle pattern within 1-unit radius (clustered start)
    for i in range(1, num_nodes + 1):
        angle = 2 * np.pi * (i - 1) / num_nodes
        radius = 0.8  # Start clustered within 1-unit radius
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        network.nodes[i] = Node(i, np.array([x, y]))
    
    print("🔗 Creating COMPLETE GRAPH - connecting every node to every other node...")
    
    # Create COMPLETE GRAPH: connect every node to every other node
    total_edges = 0
    for i in range(1, num_nodes + 1):
        for j in range(i + 1, num_nodes + 1):  # Only connect each pair once
            # Add edge (i,j)
            network.nodes[i].add_direct_neighbor(j)
            network.nodes[j].add_direct_neighbor(i)
            total_edges += 1
            
            if total_edges <= 10:  # Show first few connections
                print(f"   ✅ Connected nodes {i} ↔ {j}")
    
    if total_edges > 10:
        print(f"   ... (and {total_edges - 10} more connections)")
    
    # Initialize algorithm
    for node in network.nodes.values():
        node.initialize_algorithm(len(network.nodes))
    
    print(f"\n✅ COMPLETE GRAPH CREATED!")
    print(f"   Total nodes: {num_nodes}")
    print(f"   Total edges: {total_edges}")
    print(f"   Expected edges: {num_nodes * (num_nodes - 1) // 2}")
    
    # Calculate expected triangles
    expected_triangles = num_nodes * (num_nodes - 1) * (num_nodes - 2) // 6
    print(f"   Expected triangles: {expected_triangles}")
    
    print(f"\n🔍 TRIANGLE VERIFICATION (Sample):")
    
    # Check first few edges for triangles
    sample_edges = [(1, 2), (1, 3), (2, 3)] if num_nodes >= 3 else []
    
    for node1_id, node2_id in sample_edges:
        if node1_id in network.nodes and node2_id in network.nodes:
            node1 = network.nodes[node1_id]
            node2 = network.nodes[node2_id]
            common_neighbors = node1.direct_neighbors.intersection(node2.direct_neighbors)
            
            print(f"   Edge ({node1_id},{node2_id}):")
            print(f"     Node {node1_id} neighbors: {sorted(node1.direct_neighbors)}")
            print(f"     Node {node2_id} neighbors: {sorted(node2.direct_neighbors)}")
            print(f"     Common neighbors: {sorted(common_neighbors)}")
            print(f"     Triangle count: {len(common_neighbors)}")
            
            if len(common_neighbors) > 0:
                redundancy = len(common_neighbors) / min(len(node1.direct_neighbors), len(node2.direct_neighbors))
                priority = redundancy * (1 + len(common_neighbors))
                print(f"     🎯 Expected Priority: {priority:.4f} (HIGH!)")
            print()
    
    return network

def create_triangle_rich_graph(num_nodes: int = 8) -> DistributedNetwork:
    """
    Create a graph rich in triangles for testing the Triangle Participation algorithm
    
    This creates multiple overlapping triangular structures:
    - Central hub with surrounding triangles
    - Interconnected triangular clusters
    - High triangle density for algorithm testing
    """
    print(f"\n🔺 CREATING TRIANGLE-RICH GRAPH")
    print(f"   Nodes: {num_nodes}")
    print(f"   Strategy: Multiple overlapping triangles")
    print("=" * 50)
    
    # Create network without default graph
    network = DistributedNetwork(create_default_graph=False)
    
    # Create nodes with positions for visualization
    positions = {}
    for i in range(1, num_nodes + 1):
        angle = 2 * np.pi * (i - 1) / num_nodes
        x = 2 * np.cos(angle) + np.random.normal(0, 0.1)
        y = 2 * np.sin(angle) + np.random.normal(0, 0.1)
        positions[i] = np.array([x, y])
        network.nodes[i] = Node(i, positions[i])
    
    # Create triangle-rich edge patterns
    edges = set()
    
    # Strategy: Create overlapping triangular clusters
    
    # Step 1: Create a central hub with first triangle cluster
    center = 1
    cluster_size = min(5, num_nodes - 1)  # Adaptive cluster size
    
    # Connect center to cluster_size nodes
    for i in range(2, min(cluster_size + 2, num_nodes + 1)):
        edges.add((center, i))
    
    # Create triangles around the center
    for i in range(2, min(cluster_size + 1, num_nodes)):
        next_node = i + 1 if i + 1 <= min(cluster_size + 1, num_nodes) else 2
        edges.add((i, next_node))
    
    # Step 2: Create additional triangular clusters for remaining nodes
    remaining_nodes = list(range(cluster_size + 2, num_nodes + 1))
    
    while len(remaining_nodes) >= 3:
        # Take 3 nodes and form a triangle
        triangle_nodes = remaining_nodes[:3]
        remaining_nodes = remaining_nodes[3:]
        
        # Form triangle
        edges.add((triangle_nodes[0], triangle_nodes[1]))
        edges.add((triangle_nodes[1], triangle_nodes[2]))
        edges.add((triangle_nodes[2], triangle_nodes[0]))
        
        # Connect this triangle to existing network (to maintain connectivity)
        # Connect to a random node from existing connected components
        existing_nodes = list(range(1, min(cluster_size + 2, num_nodes + 1)))
        bridge_target = random.choice(existing_nodes)
        edges.add((triangle_nodes[0], bridge_target))
    
    # Step 3: Handle remaining 1-2 nodes if any
    if len(remaining_nodes) > 0:
        for node in remaining_nodes:
            # Connect each remaining node to at least 2 existing nodes
            existing_nodes = list(range(1, node))
            if len(existing_nodes) >= 2:
                # Connect to 2 random existing nodes to create potential triangles
                targets = random.sample(existing_nodes, min(2, len(existing_nodes)))
                for target in targets:
                    edges.add((node, target))
            else:
                # Fallback: connect to node 1
                edges.add((node, 1))
    
    # Step 4: Add extra edges to increase triangle density
    extra_triangles = min(num_nodes // 3, 5)  # Add up to 5 extra triangular connections
    
    for _ in range(extra_triangles):
        # Pick 3 random nodes and try to form a triangle
        if num_nodes >= 3:
            triangle_candidates = random.sample(range(1, num_nodes + 1), 3)
            node_a, node_b, node_c = triangle_candidates
            
            # Add edges if they don't already exist
            potential_edges = [(node_a, node_b), (node_b, node_c), (node_c, node_a)]
            for edge in potential_edges:
                normalized_edge = tuple(sorted(edge))
                if normalized_edge not in edges:
                    edges.add(normalized_edge)
    
    # Add edges to the network
    for edge in edges:
        node1, node2 = edge
        if node1 in network.nodes and node2 in network.nodes:
            network.nodes[node1].add_direct_neighbor(node2)
            network.nodes[node2].add_direct_neighbor(node1)
    
    # Initialize algorithm for all nodes
    for node in network.nodes.values():
        node.initialize_algorithm(len(network.nodes))
    
    print(f"🔺 Triangle-rich graph created!")
    print(f"   Total edges: {len(edges)}")
    
    # Count triangles for verification
    triangle_count = 0
    for node1 in network.nodes:
        for node2 in network.nodes[node1].direct_neighbors:
            if node1 < node2:
                common = network.nodes[node1].direct_neighbors.intersection(
                    network.nodes[node2].direct_neighbors
                )
                triangle_count += len(common)
    
    print(f"   Triangle count: {triangle_count} triangles detected!")
    print(f"   Expected high priority scores for triangle edges")
    
    # Show the structure
    print("\n📊 Graph Structure:")
    for node_id in sorted(network.nodes.keys()):
        neighbors = sorted(list(network.nodes[node_id].direct_neighbors))
        degree = len(neighbors)
        print(f"   Node {node_id}: degree={degree}, neighbors = {neighbors}")
    
    # Verify all nodes are connected
    print("\n🔍 Connectivity Check:")
    isolated_nodes = []
    for node_id, node in network.nodes.items():
        if len(node.direct_neighbors) == 0:
            isolated_nodes.append(node_id)
    
    if isolated_nodes:
        print(f"   ⚠️  WARNING: Isolated nodes detected: {isolated_nodes}")
        # Connect isolated nodes to node 1
        for isolated in isolated_nodes:
            network.nodes[isolated].add_direct_neighbor(1)
            network.nodes[1].add_direct_neighbor(isolated)
            print(f"   🔗 Connected isolated node {isolated} to node 1")
    else:
        print(f"   ✅ All {num_nodes} nodes have at least one connection")
    
    return network

def compare_with_networkx(network: DistributedNetwork):
    """Compare our results with NetworkX bridge detection"""
    import networkx as nx
    
    print("\n" + "=" * 60)
    print("COMPARISON WITH NETWORKX BRIDGE DETECTION")
    print("=" * 60)
    
    # Create NetworkX graph
    G = nx.Graph()
    for node_id in network.nodes.keys():
        G.add_node(node_id)
    
    all_edges = []
    for node_id, node in network.nodes.items():
        for neighbor_id in node.direct_neighbors:
            if node_id < neighbor_id:
                edge = (node_id, neighbor_id)
                G.add_edge(node_id, neighbor_id)
                all_edges.append(edge)
    
    # Find bridges using NetworkX
    nx_bridges = list(nx.bridges(G))
    nx_bridges_normalized = [tuple(sorted(bridge)) for bridge in nx_bridges]
    
    # Get our critical edges
    critical_edges, _ = network.run_critical_edge_detection()
    our_critical_edges = [tuple(sorted(edge)) for edge in critical_edges]
    
    print(f"NetworkX bridges: {sorted(nx_bridges_normalized)}")
    print(f"Our critical edges: {sorted(our_critical_edges)}")
    
    # Compare results
    nx_set = set(nx_bridges_normalized)
    our_set = set(our_critical_edges)
    
    if nx_set == our_set:
        print("✅ PERFECT MATCH! Our algorithm correctly identified all bridges.")
    else:
        print("❌ MISMATCH detected:")
        print(f"   Bridges we missed: {sorted(list(nx_set - our_set))}")
        print(f"   False positives: {sorted(list(our_set - nx_set))}")
    
    return len(nx_set) == len(our_set) and nx_set == our_set

def get_user_input():
    """Get user input for number of nodes and graph type"""
    print("\n" + "=" * 60)
    print("INTERACTIVE CRITICAL EDGE DETECTION")
    print("=" * 60)
    
    # First choose between random and hardcoded graph
    print("Choose graph option:")
    print("1. Create random graph with customizable structure")
    print("2. Use specific 8-node hardcoded graph")
    
    while True:
        try:
            graph_option = int(input("Enter your choice [1-2]: "))
            if 1 <= graph_option <= 2:
                break
            else:
                print("Please enter 1 or 2.")
        except ValueError:
            print("Please enter a valid integer.")
    
    if graph_option == 1:
        # Get number of nodes
        while True:
            try:
                num_nodes = int(input("Enter the number of robots (nodes) [3-20]: "))
                if 3 <= num_nodes <= 20:
                    break
                else:
                    print("Please enter a number between 3 and 20.")
            except ValueError:
                print("Please enter a valid integer.")
        
        # Get graph type preference
        print("\nChoose graph structure:")
        print("1. Path-like (linear connections with few shortcuts)")
        print("2. Star-like (one central hub with satellites)")
        print("3. Tree-like (hierarchical branching structure)")
        print("4. Cycle-like (circular with some chords)")
        print("5. Random (let the algorithm choose)")
        print("6. Triangle-rich (multiple overlapping triangles)")
        print("7. Complete Graph (every node connected to every other - MAXIMUM triangles!)")
        
        while True:
            try:
                choice = int(input("Enter your choice [1-7]: "))
                if 1 <= choice <= 7:
                    break
                else:
                    print("Please enter a number between 1 and 7.")
            except ValueError:
                print("Please enter a valid integer.")
        
        graph_types = {
            1: ('path', 'Path-like structure'),
            2: ('star', 'Star-like structure'),
            3: ('tree', 'Tree-like structure'),
            4: ('cycle_plus', 'Cycle-like structure'),
            5: ('random', 'Random structure'),
            6: ('triangle_rich', 'Triangle-rich structure with multiple overlapping triangles'),
            7: ('complete', 'Complete graph with maximum triangles - like your example image!')
        }
        
        graph_type, description = graph_types[choice]
        
        # Get animation method preference
        print(f"\nChoose algorithm animation:")
        print("1. Traditional (1-by-1 edge removal) - Slower but step-by-step visualization")
        print("2. Simultaneous (Batch edge removal) - Much faster for large networks")
        print("3. Goal-Seeking (Robots move to goal while preserving connectivity) - NEW!")
        
        while True:
            try:
                pruning_choice = int(input("Enter your choice [1-3]: "))
                if 1 <= pruning_choice <= 3:
                    break
                else:
                    print("Please enter 1, 2, or 3.")
            except ValueError:
                print("Please enter a valid integer.")
        
        use_simultaneous_pruning = (pruning_choice == 2)
        use_goal_seeking = (pruning_choice == 3)
        
        if use_goal_seeking:
            pruning_method = "Goal-Seeking with Connectivity Preservation"
        elif use_simultaneous_pruning:
            pruning_method = "Simultaneous Batch Pruning"
        else:
            pruning_method = "Traditional 1-by-1 Pruning"
            
        print(f"Selected: {pruning_method}")
        
        # Calculate edge probability based on number of nodes
        if num_nodes <= 5:
            edge_prob = 0.3
        elif num_nodes <= 8:
            edge_prob = 0.2
        elif num_nodes <= 12:
            edge_prob = 0.15
        else:
            edge_prob = 0.1
    else:
        # Use hardcoded 8-node graph
        num_nodes = 8
        graph_type = 'hardcoded'
        description = 'Hardcoded 8-node structure'
        edge_prob = 0.0  # Not used for hardcoded graph
        # Set defaults for pruning options (no pruning/goal-seeking by default)
        use_simultaneous_pruning = False
        use_goal_seeking = False
        pruning_method = "Traditional 1-by-1 Pruning"
    
    return num_nodes, graph_type, description, edge_prob, graph_option, use_simultaneous_pruning, use_goal_seeking, pruning_method

def create_hardcoded_8_node_network():
    """Create the specific 8-node hardcoded graph for real-time simulation"""
    network = DistributedNetwork()
    
    # Define node positions
    positions = {
        1: np.array([1.0, 2.0]),
        2: np.array([0.0, 1.0]),
        3: np.array([1.0, 0.0]),
        4: np.array([2.0, 1.0]),
        5: np.array([3.0, 1.0]),
        6: np.array([4.0, 1.0]),
        7: np.array([4.0, 0.0]),
        8: np.array([5.0, 1.0])
    }
    
    # Create nodes
    for node_id, pos in positions.items():
        network.nodes[node_id] = Node(node_id, pos)
    
    # Define edges
    edges = [
        (1, 2), (1, 4),
        (2, 3), (2, 4),
        (3, 4),
        (4, 5),
        (5, 6), (5, 7),
        (6, 7), (6, 8),
    ]
    
    # Add edges to network
    for node1, node2 in edges:
        network.nodes[node1].add_direct_neighbor(node2)
        network.nodes[node2].add_direct_neighbor(node1)
    
    print(f"Created hardcoded 8-node network with {len(edges)} edges")
    return network

def main():
    """Main function with menu-driven interface"""
    import random
    import time
    
    while True:
        print("\n" + "=" * 70)
        print("DISTRIBUTED CRITICAL EDGE DETECTION SYSTEM")
        print("Real-time Critical Edge Detection for Multi-Robot Networks")
        print("=" * 70)
        print("Choose simulation type:")
        print("1. Basic critical edge detection")
        print("2. Step-by-step distributed algorithm")
        print("3. Random graph testing")
        print("4. NetworkX comparison")
        print("5. Final visualization")
        print("6. Real-time pruning simulation (NEW!)")
        print("7. Exit")
        
        try:
            choice = input("\nEnter your choice (1-7): ").strip()
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        
        if choice == '7':
            print("Thank you for using the Critical Edge Detection tool!")
            break
        
        if choice in ['1', '2', '3', '4', '5', '6']:
            # Option 6: Real-time pruning simulation
            if choice == '6':
                # Get user input
                num_nodes, preferred_type, description, edge_prob, graph_option, use_simultaneous_pruning, use_goal_seeking, pruning_method = get_user_input()
                
                # Use current time as seed for true randomness
                seed = int(time.time())
                random.seed(seed)
                np.random.seed(seed)
                
                print(f"\nGenerating network with:")
                print(f"  - Nodes (robots): {num_nodes}")
                print(f"  - Structure: {description}")
                print(f"  - Pruning method: {pruning_method}")
                if graph_option == 1:
                    print(f"  - Edge probability: {edge_prob}")
                print(f"  - Random seed: {seed}")
                
                # Create network
                if graph_option == 2:
                    # Use hardcoded 8-node graph
                    network = create_hardcoded_8_node_network()
                else:
                    # Create random graph
                    network = create_random_graph(num_nodes, edge_prob, preferred_type)
                
                # Run algorithm based on user choice
                if use_goal_seeking:
                    # Goal-seeking behavior
                    # Randomly generate goal within [0, 10] x [0, 10]
                    goal_x = np.random.uniform(0, 10)
                    goal_y = np.random.uniform(0, 10)
                    print(f"\n🎯 Starting goal-seeking simulation...")
                    print(f"Randomly generated goal location: ({goal_x:.2f}, {goal_y:.2f})")
                    print("Robots will move toward goal while preserving connectivity!")
                    network.run_n_step_algorithm(n_steps=min(num_nodes + 2, 15))
                    network.goal_seeking_simulation(goal_x, goal_y)
                elif use_simultaneous_pruning:
                    # Use new simultaneous pruning method
                    print("\n🚀 Starting simultaneous batch pruning...")
                    network.run_n_step_algorithm(n_steps=min(num_nodes + 2, 15))
                    network.iterative_pruning_simultaneous()
                else:
                    # Use existing real-time simulation (1-by-1)
                    print("\n🎬 Starting real-time pruning simulation (1-by-1)...")
                    print("This will show continuous edge removal in a single window!")
                    network.real_time_pruning_simulation()
                
                print(f"\n✅ {pruning_method} completed!")
                continue_simulation = input("\nWould you like to run another simulation? (y/n): ").lower().strip()
                if continue_simulation != 'y':
                    continue
            
            # All other options (1-5) - existing functionality
            else:
                # Get user input
                num_nodes, preferred_type, description, edge_prob, _ = get_user_input()
                
                # Use current time as seed for true randomness
                seed = int(time.time())
                random.seed(seed)
                np.random.seed(seed)
                
                print(f"\nGenerating network with:")
                print(f"  - Nodes (robots): {num_nodes}")
                print(f"  - Structure: {description}")
                if graph_option == 1:
                    print(f"  - Edge probability: {edge_prob}")
                print(f"  - Random seed: {seed}")
                
                # Create network with user preferences
                network = create_random_graph(num_nodes, edge_prob, preferred_type)
                
                # Run distributed algorithms
                print(f"\nRunning distributed algorithms...")
                network.run_n_step_algorithm(n_steps=min(num_nodes + 2, 15))
                
                # Execute based on choice
                if choice == '1':
                    # Basic critical edge detection
                    network.print_final_results()
                    match = compare_with_networkx(network)
                    if match:
                        print("✅ Test PASSED! Our algorithm correctly identified all critical edges.")
                    else:
                        print("❌ Test FAILED! There's a mismatch with NetworkX.")
                
                elif choice == '2':
                    # Step-by-step algorithm
                    network.print_step_by_step_algorithm()
                
                elif choice == '3':
                    # Random graph testing
                    network.print_final_results()
                    network.visualize_network("Random Graph Structure", [])
                
                elif choice == '4':
                    # NetworkX comparison
                    network.print_final_results()
                    match = compare_with_networkx(network)
                    
                elif choice == '5':
                    # Final visualization
                    critical_edges, _ = network.run_critical_edge_detection()
                    network.visualize_network("Critical Edge Detection Results", critical_edges)
                
                continue_simulation = input("\nWould you like to run another simulation? (y/n): ").lower().strip()
                if continue_simulation != 'y':
                    continue
        
        else:
            print("Invalid choice. Please select 1-7.")
            continue
    
    return None

if __name__ == "__main__":
    main()
