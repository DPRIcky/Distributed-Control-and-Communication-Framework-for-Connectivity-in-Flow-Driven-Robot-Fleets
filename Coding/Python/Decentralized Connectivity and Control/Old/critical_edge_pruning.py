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
        print("SIMULTANEOUS BATCH EDGE PRUNING - 🚀 FAST MODE WITH ANIMATION")
        print("Algorithm: Keep critical edges, remove multiple non-critical edges per iteration")
        print("🎬 Watch automatic animation - no manual interaction needed!")
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
            'status_message': 'Starting batch pruning algorithm...',
            'max_iterations': 20,
            'original_edge_count': 0
        }
        
        # Calculate initial edge count
        for node_id, node in self.nodes.items():
            for neighbor_id in node.direct_neighbors:
                if node_id < neighbor_id:
                    self.batch_animation_data['all_current_edges'].add((node_id, neighbor_id))
        
        self.batch_animation_data['original_edge_count'] = len(self.batch_animation_data['all_current_edges'])
        
        print(f"🎬 Starting automatic animation with {self.batch_animation_data['original_edge_count']} initial edges...")
        
        # Set up matplotlib animation with subplots for network + bar chart
        import matplotlib.animation as animation
        
        self.fig = plt.figure(figsize=(18, 10))  # Wider figure for side-by-side layout
        
        # Create subplots: main network plot (left) and statistics (right)
        self.ax_network = plt.subplot(1, 2, 1)  # Network visualization
        self.ax_stats = plt.subplot(2, 2, 2)    # Bar chart statistics  
        self.ax_details = plt.subplot(2, 2, 4)  # Edge removal details
        
        self.ax_network.set_aspect('equal')
        
        # Get positions for consistent layout
        self.pos = {node_id: node.position for node_id, node in self.nodes.items()}
        
        # Create the animation
        self.batch_anim = animation.FuncAnimation(
            self.fig, 
            self.update_batch_simulation, 
            interval=2000,  # Update every 2 seconds (slower to see each step)
            repeat=False,   # Don't repeat when done
            blit=False      # Don't use blitting for simplicity
        )
        
        plt.tight_layout()
        plt.show()
        
        return self.batch_animation_data['pruned_edges']

    def update_batch_simulation(self, frame):
        """
        Update function for batch pruning animation - called automatically
        """
        data = self.batch_animation_data
        
        # State machine for batch pruning algorithm
        if data['current_state'] == 'detecting':
            # Phase 1: Run critical edge detection
            data['status_message'] = f'Iteration {data["iteration"] + 1}: Detecting critical edges...'
            
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
        
        elif data['current_state'] == 'selecting':
            # Phase 2: Select edges for batch removal
            data['status_message'] = f'Selecting batch of edges for removal...'
            
            # Calculate optimal batch size
            optimal_batch_size = max(1, min(50, len(data['non_critical_edges']) // 3))
            
            # Select edges for batch removal
            data['edges_to_remove'] = self.select_edges_for_batch_removal(
                data['non_critical_edges'], max_batch_size=optimal_batch_size)
            
            if not data['edges_to_remove']:
                data['current_state'] = 'complete'
            else:
                data['current_state'] = 'removing'
                data['status_message'] = f'Removing {len(data["edges_to_remove"])} edges simultaneously...'
        
        elif data['current_state'] == 'removing':
            # Phase 3: Remove the selected batch of edges
            for edge_to_remove in data['edges_to_remove']:
                node1, node2 = edge_to_remove
                self.nodes[node1].direct_neighbors.discard(node2)
                self.nodes[node2].direct_neighbors.discard(node1)
                data['pruned_edges'].append(edge_to_remove)
            
            data['iteration'] += 1
            data['status_message'] = f'Removed {len(data["edges_to_remove"])} edges. Checking connectivity...'
            
            # Check if we should continue
            if data['iteration'] >= data['max_iterations']:
                data['current_state'] = 'complete'
            else:
                data['current_state'] = 'detecting'
                data['edges_to_remove'] = []  # Clear for next iteration
        
        # Draw the network based on current state
        self.draw_batch_network_state(data)
        
        # Stop animation when complete
        if data['current_state'] == 'complete':
            self.batch_anim.event_source.stop()
            print(f"\n✅ Batch pruning animation complete!")
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
        # Clear all subplots
        self.ax_network.clear()
        self.ax_stats.clear() 
        self.ax_details.clear()
        
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
        
        # Draw nodes
        nx.draw_networkx_nodes(G, self.pos, ax=self.ax_network, node_color='lightblue', 
                              node_size=600, alpha=0.8)
        
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
        self.ax_stats.set_title("🏆 Edge Statistics", fontweight='bold', fontsize=11)
        
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
        
        # Set y-axis limit with some padding
        max_count = max(counts) if counts else 1
        self.ax_stats.set_ylim(0, max_count * 1.3)
        
        # Add grid for easier reading
        self.ax_stats.grid(axis='y', alpha=0.3, linestyle='--')
        
        # ==== DETAILED PROGRESS PANEL (BOTTOM RIGHT) ====
        self.ax_details.set_title("📊 Progress Details", fontweight='bold', fontsize=11)
        
        # Create detailed progress text
        progress_text = ""
        
        if data['current_state'] != 'complete':
            progress_text += f"🔄 Iteration: {data['iteration']}/{data['max_iterations']}\n"
            progress_text += f"📈 Original Edges: {data['original_edge_count']}\n"
            progress_text += f"📉 Edges Removed: {len(data['pruned_edges'])}\n"
            progress_text += f"📊 Current Edges: {len(data['all_current_edges'])}\n"
            
            if data['edges_to_remove']:
                progress_text += f"\n🎯 Removing This Iteration:\n"
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
            progress_text += f"🎉 PRUNING COMPLETE!\n\n"
            progress_text += f"📊 Final Statistics:\n"
            progress_text += f"   Original: {data['original_edge_count']} edges\n"
            progress_text += f"   Final: {len(data['all_current_edges'])} edges\n"
            progress_text += f"   Removed: {len(data['pruned_edges'])} edges\n"
            progress_text += f"   Iterations: {data['iteration']}\n"
            
            if data['original_edge_count'] > 0:
                reduction_pct = (len(data['pruned_edges']) / data['original_edge_count']) * 100
                progress_text += f"   Reduction: {reduction_pct:.1f}%\n"
            
            progress_text += f"\n✅ Network connectivity preserved!"
        
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
        
        print(f"\n🏆 BATCH EDGE SELECTION WITH CONNECTIVITY VALIDATION (Max Batch: {max_batch_size})")
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
                print(f"   🚫 Skipping Edge {edge} → Priority: {priority:.4f} (Would disconnect nodes!)")
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
            print(f"   📊 {remaining_edges} edges skipped due to conflicts or connectivity constraints")
        
        print(f"\n🎯 BATCH SUMMARY: {len(selected_edges)} edges selected for simultaneous removal")
        print("✅ All selected edges verified to preserve network connectivity")
        
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

def create_random_graph(num_nodes: int = 10, edge_probability: float = 0.4, preferred_type: str = 'random') -> DistributedNetwork:
    """Create a random connected graph (not fully connected) for testing"""
    import random
    
    print(f"\nGenerating random connected graph with {num_nodes} nodes...")
    print(f"Preferred structure: {preferred_type}")
    print(f"Edge probability: {edge_probability}")
    print("-" * 50)
    
    # Create empty network (don't auto-initialize)
    network = DistributedNetwork(create_default_graph=False)
    
    # Generate positions in a circle with better spacing for larger graphs
    positions = {}
    for i in range(1, num_nodes + 1):
        angle = 2 * np.pi * (i - 1) / num_nodes
        # Adjust radius based on number of nodes for better visualization
        radius = max(3, num_nodes * 0.4) + random.uniform(-0.3, 0.3)
        x = radius * np.cos(angle) + random.uniform(-0.2, 0.2)
        y = radius * np.sin(angle) + random.uniform(-0.2, 0.2)
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
    Create a COMPLETE GRAPH structure like the example image
    Every node connects to every other node = MAXIMUM triangles
    """
    print(f"\n🔺 CREATING COMPLETE GRAPH (Maximum Triangles)")
    print(f"   Nodes: {num_nodes}")
    print("   Structure: Every node connects to every other node")
    print("   Result: MAXIMUM possible triangles!")
    print("=" * 60)
    
    # Create network
    network = DistributedNetwork(create_default_graph=False)
    
    # Create nodes in a circle pattern (like your image)
    for i in range(1, num_nodes + 1):
        angle = 2 * np.pi * (i - 1) / num_nodes
        radius = 3  # Spread them out more
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
        
        # Get pruning method preference
        print(f"\nChoose edge pruning method:")
        print("1. Traditional (1-by-1 edge removal) - Slower but step-by-step visualization")
        print("2. Simultaneous (Batch edge removal) - Much faster for large networks")
        
        while True:
            try:
                pruning_choice = int(input("Enter your choice [1-2]: "))
                if 1 <= pruning_choice <= 2:
                    break
                else:
                    print("Please enter 1 or 2.")
            except ValueError:
                print("Please enter a valid integer.")
        
        use_simultaneous_pruning = (pruning_choice == 2)
        pruning_method = "Simultaneous Batch Pruning" if use_simultaneous_pruning else "Traditional 1-by-1 Pruning"
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
    
    return num_nodes, graph_type, description, edge_prob, graph_option, use_simultaneous_pruning, pruning_method

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
                num_nodes, preferred_type, description, edge_prob, graph_option, use_simultaneous_pruning, pruning_method = get_user_input()
                
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
                if use_simultaneous_pruning:
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
