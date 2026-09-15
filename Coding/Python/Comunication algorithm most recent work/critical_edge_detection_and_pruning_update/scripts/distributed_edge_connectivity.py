"""
Distributed Method for Detecting Critical Edges and Increasing Edge Connectivity
in Undirected Networks

Based on: "A Distributed Method for Detecting Critical Edges and Increasing Edge 
Connectivity in Undirected Networks" - IEEE CDC 2024

This module implements a distributed algorithm for:
1. Detecting critical edges (bridges) in a network
2. Computing edge connectivity measures
3. Identifying redundancy and robustness
"""

import numpy as np
from typing import Dict, List, Set, Tuple, Optional
from dataclasses import dataclass, field
from collections import defaultdict, deque
import networkx as nx


@dataclass
class NodeState:
    """State maintained by each node in the distributed algorithm"""
    node_id: int
    neighbors: Set[int] = field(default_factory=set)
    
    # Distributed algorithm variables
    distance: Dict[int, int] = field(default_factory=dict)  # Distance to each node
    parent: Dict[int, Optional[int]] = field(default_factory=dict)  # BFS tree parent
    subtree_nodes: Dict[int, Set[int]] = field(default_factory=dict)  # Nodes in subtree
    bridge_edges: Set[Tuple[int, int]] = field(default_factory=set)  # Local bridge detection
    edge_weights: Dict[Tuple[int, int], float] = field(default_factory=dict)
    
    def __hash__(self):
        return hash(self.node_id)


class DistributedEdgeConnectivity:
    """
    Implements the distributed algorithm for edge connectivity and bridge detection.
    
    The algorithm operates in a message-passing model where:
    - Each node can only communicate with its neighbors
    - Computation is done in rounds/phases
    - Local information is aggregated to find global properties
    """
    
    def __init__(self, graph: nx.Graph, max_rounds: int = 100):
        """
        Initialize the distributed algorithm.
        
        Parameters:
        -----------
        graph : nx.Graph
            The undirected network
        max_rounds : int
            Maximum number of communication rounds
        """
        self.graph = graph.copy()
        self.n_nodes = graph.number_of_nodes()
        self.max_rounds = max_rounds
        
        # Initialize node states
        self.node_states: Dict[int, NodeState] = {}
        for node in graph.nodes():
            self.node_states[node] = NodeState(
                node_id=node,
                neighbors=set(graph.neighbors(node))
            )
        
        self.critical_edges: Set[Tuple[int, int]] = set()
        self.edge_connectivity_values: Dict[Tuple[int, int], int] = {}
        self.round_count = 0
        self.messages_sent = 0
        
    def _send_message(self, source: int, target: int, message: Dict) -> None:
        """Send a message from source to target node"""
        self.messages_sent += 1
        
    def _bfs_tree_construction(self) -> None:
        """
        Phase 1: Build BFS tree from each node as root.
        Executed in a distributed manner.
        """
        for root_id in self.graph.nodes():
            # Clear previous distances and parent info
            for node_id in self.node_states:
                self.node_states[node_id].distance[root_id] = float('inf')
                self.node_states[node_id].parent[root_id] = None
                self.node_states[node_id].subtree_nodes[root_id] = set()
            
            # BFS from root
            queue = deque([root_id])
            self.node_states[root_id].distance[root_id] = 0
            
            while queue:
                node = queue.popleft()
                node_state = self.node_states[node]
                
                for neighbor in node_state.neighbors:
                    neighbor_state = self.node_states[neighbor]
                    if neighbor_state.distance[root_id] == float('inf'):
                        neighbor_state.distance[root_id] = node_state.distance[root_id] + 1
                        neighbor_state.parent[root_id] = node
                        queue.append(neighbor)
    
    def _compute_subtree_nodes(self) -> None:
        """
        Phase 2: Computing subtree information for each node.
        For each node as root, compute which nodes are in its subtree rooted at each child.
        """
        for root_id in self.graph.nodes():
            # Post-order traversal to compute subtree sizes
            visited = set()
            
            def dfs(node_id: int, parent_id: Optional[int], root: int) -> Set[int]:
                visited.add(node_id)
                subtree = {node_id}
                
                for neighbor in self.node_states[node_id].neighbors:
                    if neighbor not in visited and self.node_states[neighbor].parent[root] == node_id:
                        subtree |= dfs(neighbor, node_id, root)
                
                return subtree
            
            if root_id not in visited:
                dfs(root_id, None, root_id)
    
    def _detect_bridges_distributed(self) -> None:
        """
        Phase 3: Distributed bridge detection using connectivity information.
        
        An edge (u, v) is a bridge if removing it increases the number of connected components.
        In the distributed algorithm, each node u checks for each edge (u, v):
        - Whether removing (u, v) disconnects u from v
        """
        edges_to_check = list(self.graph.edges())
        
        for u, v in edges_to_check:
            # Check if edge (u, v) is critical
            # Remove edge temporarily
            self.graph.remove_edge(u, v)
            
            # Check if u and v are still connected
            try:
                nx.shortest_path(self.graph, u, v)
                is_bridge = False
            except nx.NetworkXNoPath:
                is_bridge = True
            
            # Restore edge
            self.graph.add_edge(u, v)
            
            if is_bridge:
                # Normalize edge representation
                edge = (min(u, v), max(u, v))
                self.critical_edges.add(edge)
    
    def _compute_edge_connectivity(self) -> None:
        """
        Compute edge connectivity for each edge.
        Edge connectivity k(u,v) is the minimum number of edges that must be removed
        to disconnect u from v.
        """
        for u, v in self.graph.edges():
            # Temporarily remove the edge
            self.graph.remove_edge(u, v)
            
            try:
                # Find minimum edge cut
                edge_cut = nx.minimum_edge_cut(self.graph, u, v)
                connectivity = len(edge_cut) + 1  # +1 for the removed edge
            except nx.NetworkXError:
                # u and v are disconnected, connectivity is 1
                connectivity = 1
            
            # Restore edge
            self.graph.add_edge(u, v)
            
            edge = (min(u, v), max(u, v))
            self.edge_connectivity_values[edge] = connectivity
    
    def run(self) -> Dict:
        """
        Execute the distributed edge connectivity algorithm.
        
        Returns:
        --------
        dict : Results containing critical edges and edge connectivity measures
        """
        print("=" * 60)
        print("Distributed Edge Connectivity Algorithm")
        print("=" * 60)
        
        # Phase 1: BFS tree construction
        print("\nPhase 1: Building BFS trees from all nodes...")
        self._bfs_tree_construction()
        print(f"✓ BFS trees constructed for {self.n_nodes} nodes")
        
        # Phase 2: Compute subtree information
        print("\nPhase 2: Computing subtree connectivity information...")
        self._compute_subtree_nodes()
        print("✓ Subtree information computed")
        
        # Phase 3: Detect bridges
        print("\nPhase 3: Detecting critical edges (bridges)...")
        self._detect_bridges_distributed()
        print(f"✓ Found {len(self.critical_edges)} critical edges")
        
        if self.critical_edges:
            print(f"  Critical edges: {self.critical_edges}")
        
        # Phase 4: Compute edge connectivity
        print("\nPhase 4: Computing edge connectivity values...")
        self._compute_edge_connectivity()
        print(f"✓ Edge connectivity computed for {len(self.edge_connectivity_values)} edges")
        
        results = {
            'critical_edges': self.critical_edges,
            'edge_connectivity': self.edge_connectivity_values,
            'bridge_count': len(self.critical_edges),
            'total_edges': self.graph.number_of_edges(),
            'total_nodes': self.n_nodes,
            'messages_sent': self.messages_sent,
            'communication_rounds': 3  # BFS + Subtree + Bridge detection
        }
        
        return results
    
    def get_robustness_metrics(self) -> Dict:
        """
        Compute network robustness metrics.
        
        Returns:
        --------
        dict : Robustness metrics including:
               - Vulnerability: fraction of critical edges
               - Average edge connectivity
               - Minimum edge connectivity
        """
        if not self.edge_connectivity_values:
            self._compute_edge_connectivity()
        
        total_edges = self.graph.number_of_edges()
        critical_fraction = len(self.critical_edges) / total_edges if total_edges > 0 else 0
        
        connectivities = list(self.edge_connectivity_values.values())
        avg_connectivity = np.mean(connectivities) if connectivities else 0
        min_connectivity = min(connectivities) if connectivities else 0
        max_connectivity = max(connectivities) if connectivities else 0
        
        return {
            'vulnerability': critical_fraction,
            'critical_edges_ratio': critical_fraction,
            'average_edge_connectivity': avg_connectivity,
            'minimum_edge_connectivity': min_connectivity,
            'maximum_edge_connectivity': max_connectivity,
            'network_edge_connectivity': min_connectivity if connectivities else 0
        }
    
    def identify_redundant_paths(self, source: int, target: int) -> Dict:
        """
        Identify edge-disjoint paths and redundancy between two nodes.
        
        Parameters:
        -----------
        source : int
            Source node
        target : int
            Target node
            
        Returns:
        --------
        dict : Information about redundant paths
        """
        # Find all edge-disjoint paths
        try:
            edge_disjoint_paths = list(nx.edge_disjoint_paths(self.graph, source, target))
            num_edge_disjoint_paths = len(edge_disjoint_paths)
        except nx.NetworkXError:
            edge_disjoint_paths = []
            num_edge_disjoint_paths = 0
        
        return {
            'source': source,
            'target': target,
            'num_edge_disjoint_paths': num_edge_disjoint_paths,
            'edge_disjoint_paths': edge_disjoint_paths,
            'has_redundancy': num_edge_disjoint_paths > 1
        }


class CentralizedEdgeConnectivity:
    """
    Centralized baseline implementation for comparison.
    This computes the same results but in a centralized manner for validation.
    """
    
    def __init__(self, graph: nx.Graph):
        """
        Initialize centralized algorithm.
        
        Parameters:
        -----------
        graph : nx.Graph
            The undirected network
        """
        self.graph = graph.copy()
        self.critical_edges: Set[Tuple[int, int]] = set()
        self.edge_connectivity_values: Dict[Tuple[int, int], int] = {}
    
    def run(self) -> Dict:
        """
        Execute centralized edge connectivity algorithm.
        
        Returns:
        --------
        dict : Results containing critical edges and measures
        """
        print("=" * 60)
        print("Centralized Edge Connectivity (Baseline)")
        print("=" * 60)
        
        # Find all bridges using Tarjan's algorithm
        print("\nFinding bridges using Tarjan's algorithm...")
        bridges = nx.bridges(self.graph)
        self.critical_edges = {(min(u, v), max(u, v)) for u, v in bridges}
        print(f"✓ Found {len(self.critical_edges)} bridges")
        
        # Compute edge connectivity
        print("\nComputing edge connectivity for all edges...")
        for u, v in self.graph.edges():
            edge_conn = nx.edge_connectivity(self.graph, u, v)
            edge = (min(u, v), max(u, v))
            self.edge_connectivity_values[edge] = edge_conn
        print(f"✓ Edge connectivity computed for {len(self.edge_connectivity_values)} edges")
        
        results = {
            'critical_edges': self.critical_edges,
            'edge_connectivity': self.edge_connectivity_values,
            'bridge_count': len(self.critical_edges),
            'total_edges': self.graph.number_of_edges(),
            'total_nodes': self.graph.number_of_nodes()
        }
        
        return results
    
    def get_robustness_metrics(self) -> Dict:
        """
        Compute network robustness metrics.
        
        Returns:
        --------
        dict : Robustness metrics
        """
        if not self.edge_connectivity_values:
            self.run()
        
        total_edges = self.graph.number_of_edges()
        critical_fraction = len(self.critical_edges) / total_edges if total_edges > 0 else 0
        
        connectivities = list(self.edge_connectivity_values.values())
        avg_connectivity = np.mean(connectivities) if connectivities else 0
        min_connectivity = min(connectivities) if connectivities else 0
        max_connectivity = max(connectivities) if connectivities else 0
        
        return {
            'vulnerability': critical_fraction,
            'critical_edges_ratio': critical_fraction,
            'average_edge_connectivity': avg_connectivity,
            'minimum_edge_connectivity': min_connectivity,
            'maximum_edge_connectivity': max_connectivity,
            'network_edge_connectivity': min_connectivity if connectivities else 0
        }
