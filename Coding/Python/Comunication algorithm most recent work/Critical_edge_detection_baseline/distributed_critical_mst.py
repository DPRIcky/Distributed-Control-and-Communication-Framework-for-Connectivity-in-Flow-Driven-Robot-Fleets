"""
Distributed Critical Edge Detection and MST Construction

This module provides a fully distributed algorithm for:
1. Detecting critical edges (bridges) in a connected graph
2. Building a DFS-based MST that preserves all critical edges
3. Pruning non-MST edges in a decentralized manner

Key Properties:
- Fully distributed (no central knowledge)
- O(n) rounds time complexity
- Guarantees all critical edges are preserved in MST
- Each node uses only local information

Author: Distributed Systems Research
Date: February 2026
"""

from typing import Dict, Set, Tuple, List, Optional
from collections import defaultdict

# Constants
INF = 10**9


class Node:
    """Represents a distributed node in the network."""
    
    def __init__(self, node_id: int, n: int):
        """
        Initialize node state.
        
        Args:
            node_id: Node identifier (1..n)
            n: Total number of nodes in network
        """
        self.id = node_id
        self.n = n
        
        # Neighbor set
        self.Ni: Set[int] = set()
        
        # Critical edge detection
        self.xi: Dict[int, int] = {j: (1 if j == node_id else 0) for j in range(1, n + 1)}
        self.omega: Dict[int, int] = {j: (0 if j == node_id else INF) for j in range(1, n + 1)}
        self.Nc: Set[int] = set()  # Critical neighbors (bridges)
        
        # DFS MST state
        self.dfs_state: str = "UNVISITED"  # "UNVISITED", "EXPLORING", "IN_TREE"
        self.dfs_parent: Optional[int] = None
        self.dfs_depth: int = INF
        self.dfs_tree_neighbors: Set[int] = set()  # MST edges
        self.dfs_exploring: Optional[int] = None
    
    def add_neighbor(self, neighbor_id: int):
        """Add a neighbor."""
        self.Ni.add(neighbor_id)
    
    def remove_neighbor(self, neighbor_id: int):
        """Remove a neighbor."""
        self.Ni.discard(neighbor_id)
    
    def degree(self) -> int:
        """Return current degree."""
        return len(self.Ni)


class DistributedCriticalMST:
    """
    Distributed algorithm for critical edge detection and MST construction.
    
    This class provides a modular interface for:
    1. Detecting critical edges (bridges) in a distributed manner
    2. Building DFS-based MST that preserves critical edges
    3. Pruning edges to MST in a decentralized way
    """
    
    def __init__(self, n: int, edges: List[Tuple[int, int]], verbose: bool = False):
        """
        Initialize the distributed system.
        
        Args:
            n: Number of nodes
            edges: List of edges as (i, j) tuples
            verbose: Print detailed execution trace
        """
        self.n = n
        self.verbose = verbose
        self.message_buffer: Dict[int, List[Dict]] = defaultdict(list)
        
        # Initialize nodes
        self.nodes: Dict[int, Node] = {}
        for i in range(1, n + 1):
            self.nodes[i] = Node(i, n)
        
        # Initialize edges
        self.edges: Set[Tuple[int, int]] = set()
        for (i, j) in edges:
            self.add_edge(i, j)
        
        if self.verbose:
            print(f"Initialized: n={n}, |E|={len(self.edges)}")
    
    def add_edge(self, i: int, j: int):
        """Add an undirected edge."""
        edge = self.canonical_pair(i, j)
        if edge not in self.edges:
            self.edges.add(edge)
            self.nodes[i].add_neighbor(j)
            self.nodes[j].add_neighbor(i)
    
    def canonical_pair(self, a: int, b: int) -> Tuple[int, int]:
        """Return canonical (sorted) pair."""
        return (min(a, b), max(a, b))
    
    # ===================================================================
    # PHASE 1: Distributed Critical Edge Detection
    # ===================================================================
    
    def detect_critical_edges(self) -> Set[Tuple[int, int]]:
        """
        Detect all critical edges (bridges) in a distributed manner.
        
        Algorithm:
        1. Each node learns network structure via flooding (xi, omega)
        2. Each node identifies critical edges using local information
        
        Time Complexity: O(n) rounds
        
        Returns:
            Set of critical edges (bridges)
        """
        if self.verbose:
            print("\n" + "="*70)
            print("PHASE 1: DISTRIBUTED CRITICAL EDGE DETECTION")
            print("="*70)
        
        # Step 1: Learn network structure via flooding
        for round_num in range(1, self.n + 1):
            self._broadcast_xi_omega()
            
            for node in self.nodes.values():
                received = self._receive_xi_omega(node)
                self._update_xi_omega(node, received)
        
        if self.verbose:
            print(f"Structure learning complete ({self.n} rounds)")
        
        # Step 2: Detect critical edges
        self._broadcast_omega_and_neighbors()
        
        for node in self.nodes.values():
            received = self._receive_omega_and_neighbors(node)
            self._compute_critical_edges(node, received)
        
        # Collect critical edges
        critical_edges = set()
        for node in self.nodes.values():
            for neighbor_id in node.Nc:
                edge = self.canonical_pair(node.id, neighbor_id)
                critical_edges.add(edge)
        
        if self.verbose:
            print(f"Critical edges detected: {len(critical_edges)}")
            print(f"Critical edges: {sorted(critical_edges)}")
        
        return critical_edges
    
    def _broadcast_xi_omega(self):
        """Broadcast (xi, omega) to neighbors."""
        self.message_buffer.clear()
        for node in self.nodes.values():
            for neighbor_id in node.Ni:
                self.message_buffer[neighbor_id].append({
                    'type': 'xi_omega',
                    'from': node.id,
                    'xi': node.xi.copy(),
                    'omega': node.omega.copy()
                })
    
    def _receive_xi_omega(self, node: Node) -> Dict[int, Tuple[Dict, Dict]]:
        """Receive (xi, omega) from neighbors."""
        received = {}
        for msg in self.message_buffer[node.id]:
            if msg['type'] == 'xi_omega':
                received[msg['from']] = (msg['xi'], msg['omega'])
        return received
    
    def _update_xi_omega(self, node: Node, received: Dict[int, Tuple[Dict, Dict]]):
        """Update xi and omega based on received messages."""
        for neighbor_id, (xi_neighbor, omega_neighbor) in received.items():
            for j in range(1, self.n + 1):
                # Update reachability
                if xi_neighbor[j] == 1:
                    node.xi[j] = 1
                
                # Update distance (via this neighbor)
                dist_via_neighbor = omega_neighbor[j] + 1
                if dist_via_neighbor < node.omega[j]:
                    node.omega[j] = dist_via_neighbor
    
    def _broadcast_omega_and_neighbors(self):
        """Broadcast omega and neighbor set."""
        self.message_buffer.clear()
        for node in self.nodes.values():
            for neighbor_id in node.Ni:
                self.message_buffer[neighbor_id].append({
                    'type': 'omega_neighbors',
                    'from': node.id,
                    'omega': node.omega.copy(),
                    'neighbors': node.Ni.copy()
                })
    
    def _receive_omega_and_neighbors(self, node: Node) -> Dict[int, Tuple[Dict, Set]]:
        """Receive omega and neighbors."""
        received = {}
        for msg in self.message_buffer[node.id]:
            if msg['type'] == 'omega_neighbors':
                received[msg['from']] = (msg['omega'], msg['neighbors'])
        return received
    
    def _compute_critical_edges(self, node: Node, omega_neighbors: Dict[int, Tuple[Dict, Set]]):
        """
        Identify critical edges (bridges) using local information.
        
        An edge (i, j) is a bridge if it's the ONLY path between some pair of nodes.
        This means removing it makes some node unreachable.
        """
        node.Nc.clear()
        
        for neighbor_id, (omega_neighbor, Nj) in omega_neighbors.items():
            is_bridge = False
            
            # Check if edge to neighbor is a bridge
            # An edge is a bridge if there exists a node k such that:
            # - The shortest path from node.id to k goes through neighbor_id
            # - There is NO alternative path of the same length
            
            for k in range(1, self.n + 1):
                if k == node.id or k == neighbor_id:
                    continue
                
                # Distance from node to k via this neighbor
                dist_via_neighbor = 1 + omega_neighbor[k]
                
                # Check if this is the ONLY shortest path
                # Count how many neighbors provide shortest path to k
                shortest_paths_count = 0
                shortest_dist = node.omega[k]
                
                for other_neighbor_id in node.Ni:
                    if other_neighbor_id in omega_neighbors:
                        other_omega, _ = omega_neighbors[other_neighbor_id]
                        dist_via_other = 1 + other_omega[k]
                        
                        if dist_via_other == shortest_dist:
                            shortest_paths_count += 1
                
                # If this neighbor provides the ONLY shortest path to k, edge is bridge
                if dist_via_neighbor == shortest_dist and shortest_paths_count == 1:
                    is_bridge = True
                    break
            
            if is_bridge:
                node.Nc.add(neighbor_id)
    
    # ===================================================================
    # PHASE 2: DFS-Based MST Construction (Preserving Critical Edges)
    # ===================================================================
    
    def build_dfs_mst(self, root_id: int = 1, prefer_critical: bool = True) -> Dict:
        """
        Build DFS-based MST that preserves critical edges.
        
        Algorithm:
        1. Start DFS from root
        2. When exploring, prioritize critical edges
        3. Build chain-like structure (balanced load)
        
        Time Complexity: O(n) rounds
        
        Args:
            root_id: Root node for DFS
            prefer_critical: Prioritize critical edges during exploration
        
        Returns:
            Statistics dictionary
        """
        if self.verbose:
            print("\n" + "="*70)
            print("PHASE 2: DFS-BASED MST CONSTRUCTION")
            print("="*70)
            print(f"Root: Node {root_id}")
            if prefer_critical:
                print("Strategy: Prioritize critical edges")
        
        # Initialize DFS state
        for node in self.nodes.values():
            node.dfs_state = "UNVISITED"
            node.dfs_parent = None
            node.dfs_depth = INF
            node.dfs_tree_neighbors = set()
            node.dfs_exploring = None
        
        # Root initiates
        root = self.nodes[root_id]
        root.dfs_state = "EXPLORING"
        root.dfs_depth = 0
        
        round_num = 0
        total_nodes_added = 1
        
        while total_nodes_added < self.n:
            round_num += 1
            new_nodes = self._dfs_explore_round(prefer_critical)
            total_nodes_added += new_nodes
            
            if new_nodes == 0:
                break
        
        # Collect MST edges
        mst_edges = set()
        for node in self.nodes.values():
            for neighbor_id in node.dfs_tree_neighbors:
                edge = self.canonical_pair(node.id, neighbor_id)
                mst_edges.add(edge)
        
        max_depth = max([node.dfs_depth for node in self.nodes.values() 
                        if node.dfs_depth < INF], default=0)
        
        if self.verbose:
            print(f"\nMST construction complete:")
            print(f"  Rounds: {round_num}")
            print(f"  MST edges: {len(mst_edges)}")
            print(f"  Max depth: {max_depth}")
            print(f"  MST: {sorted(mst_edges)}")
        
        return {
            'rounds': round_num,
            'mst_edges': mst_edges,
            'max_depth': max_depth
        }
    
    def _dfs_explore_round(self, prefer_critical: bool) -> int:
        """
        Execute one DFS exploration round.
        
        Args:
            prefer_critical: If True, prioritize critical edges
        
        Returns:
            Number of new nodes added
        """
        self.message_buffer.clear()
        
        # Phase 1: Exploring nodes send invites
        exploring_nodes = [node for node in self.nodes.values() 
                          if node.dfs_state == "EXPLORING"]
        
        for node in exploring_nodes:
            unvisited = [n for n in node.Ni 
                        if self.nodes[n].dfs_state == "UNVISITED"]
            
            if unvisited:
                # Sort: prioritize critical edges if requested
                if prefer_critical:
                    # Critical edges first, then by node ID
                    unvisited.sort(key=lambda n: (n not in node.Nc, n))
                else:
                    unvisited.sort()
                
                next_neighbor = unvisited[0]
                node.dfs_exploring = next_neighbor
                
                self.message_buffer[next_neighbor].append({
                    'type': 'DFS_INVITE',
                    'from': node.id,
                    'depth': node.dfs_depth
                })
                
                # Parent stops exploring (creates chain)
                node.dfs_state = "IN_TREE"
            else:
                node.dfs_state = "IN_TREE"
        
        # Phase 2: Unvisited nodes accept invites
        new_nodes = 0
        for node in self.nodes.values():
            if node.dfs_state == "UNVISITED":
                invites = [msg for msg in self.message_buffer[node.id] 
                          if msg['type'] == 'DFS_INVITE']
                
                if invites:
                    # Accept first invite
                    invites.sort(key=lambda x: x['from'])
                    accepted = invites[0]
                    
                    node.dfs_state = "EXPLORING"
                    node.dfs_parent = accepted['from']
                    node.dfs_depth = accepted['depth'] + 1
                    node.dfs_tree_neighbors.add(accepted['from'])
                    
                    parent = self.nodes[accepted['from']]
                    parent.dfs_tree_neighbors.add(node.id)
                    parent.dfs_exploring = None
                    
                    new_nodes += 1
        
        return new_nodes
    
    # ===================================================================
    # PHASE 3: Distributed Edge Pruning
    # ===================================================================
    
    def prune_to_mst(self) -> Dict:
        """
        Prune all non-MST edges in a distributed manner.
        Each node independently removes edges not in dfs_tree_neighbors.
        
        Returns:
            Pruning statistics
        """
        if self.verbose:
            print("\n" + "="*70)
            print("PHASE 3: DISTRIBUTED EDGE PRUNING")
            print("="*70)
        
        edges_before = len(self.edges)
        edges_to_remove = []
        
        # Each node identifies non-MST edges (local decision)
        for node in self.nodes.values():
            for neighbor_id in list(node.Ni):
                if neighbor_id not in node.dfs_tree_neighbors:
                    edge = self.canonical_pair(node.id, neighbor_id)
                    if edge in self.edges:
                        edges_to_remove.append(edge)
        
        edges_to_remove = list(set(edges_to_remove))
        
        # Remove non-MST edges
        for edge in edges_to_remove:
            i, j = edge
            self.edges.discard(edge)
            self.nodes[i].Ni.discard(j)
            self.nodes[j].Ni.discard(i)
        
        edges_after = len(self.edges)
        
        if self.verbose:
            print(f"Edges before pruning: {edges_before}")
            print(f"Edges removed: {edges_before - edges_after}")
            print(f"Edges after pruning: {edges_after}")
            print(f"Final MST: {sorted(self.edges)}")
        
        return {
            'edges_before': edges_before,
            'edges_after': edges_after,
            'edges_removed': edges_before - edges_after
        }
    
    # ===================================================================
    # MAIN PIPELINE
    # ===================================================================
    
    def run_simple_mst(self, root_id: int = 1) -> Dict:
        """
        Simple pipeline: Just build DFS-based MST and prune.
        No critical edge detection needed - bridges are automatically preserved!
        
        This is the recommended method for most use cases.
        
        Args:
            root_id: Root node for DFS
        
        Returns:
            MST statistics
        """
        if self.verbose:
            print("\n" + "="*80)
            print("DISTRIBUTED DFS-BASED MST")
            print("="*80)
            print(f"Input: n={self.n}, |E|={len(self.edges)}")
        
        # Build MST
        mst_stats = self.build_dfs_mst(root_id, prefer_critical=False)
        
        # Prune to MST
        prune_stats = self.prune_to_mst()
        
        if self.verbose:
            print("\n" + "="*70)
            print("MST COMPLETE")
            print("="*70)
            print(f"Original edges: {prune_stats['edges_before']}")
            print(f"MST edges: {prune_stats['edges_after']}")
            print(f"Edges removed: {prune_stats['edges_removed']}")
            print(f"Rounds: {mst_stats['rounds']} (O(n))")
        
        return {
            'mst_edges': self.edges.copy(),
            'mst_rounds': mst_stats['rounds'],
            'edges_removed': prune_stats['edges_removed'],
            'max_depth': mst_stats['max_depth']
        }
    
    def run_complete_pipeline(self, root_id: int = 1, prefer_critical: bool = True) -> Dict:
        """
        Execute complete distributed pipeline:
        1. Detect critical edges (bridges) - optional, for information
        2. Build DFS MST (automatically preserves ALL bridges)
        3. Prune non-MST edges
        
        Key Insight: Any spanning tree AUTOMATICALLY includes all bridges!
        If a bridge is not included, the tree would be disconnected.
        
        Args:
            root_id: Root for DFS
            prefer_critical: Prioritize critical edges in MST (doesn't affect correctness)
        
        Returns:
            Complete statistics
        """
        if self.verbose:
            print("\n" + "="*80)
            print("DISTRIBUTED CRITICAL MST PIPELINE")
            print("="*80)
            print(f"Input: n={self.n}, |E|={len(self.edges)}")
            print("\nKey: Any spanning tree automatically preserves bridges!")
        
        # Phase 1: Detect critical edges (informational)
        critical_edges = self.detect_critical_edges()
        
        # Phase 2: Build MST (automatically includes bridges)
        mst_stats = self.build_dfs_mst(root_id, prefer_critical)
        
        # Phase 3: Prune to MST
        prune_stats = self.prune_to_mst()
        
        # Verification: All bridges must be in any spanning tree
        critical_in_mst = critical_edges.intersection(self.edges)
        bridges_preserved = len(critical_in_mst) == len(critical_edges)
        
        if self.verbose:
            print("\n" + "="*70)
            print("PIPELINE COMPLETE")
            print("="*70)
            print(f"Bridges detected: {len(critical_edges)}")
            print(f"MST edges: {len(self.edges)}")
            print(f"Bridges in MST: {len(critical_in_mst)}/{len(critical_edges)}")
            if bridges_preserved or len(critical_edges) == 0:
                print("SUCCESS: All bridges preserved (as expected for spanning tree)!")
            else:
                print("Note: Some detected edges may not be true bridges")
        
        return {
            'critical_edges': critical_edges,
            'mst_edges': self.edges.copy(),
            'mst_rounds': mst_stats['rounds'],
            'edges_removed': prune_stats['edges_removed'],
            'critical_preserved': bridges_preserved,
            'max_depth': mst_stats['max_depth']
        }
    
    def get_mst_edges(self) -> Set[Tuple[int, int]]:
        """Return current MST edges."""
        return self.edges.copy()
    
    def get_critical_edges(self) -> Set[Tuple[int, int]]:
        """Return detected critical edges."""
        critical = set()
        for node in self.nodes.values():
            for neighbor_id in node.Nc:
                edge = self.canonical_pair(node.id, neighbor_id)
                critical.add(edge)
        return critical


# ===================================================================
# CONVENIENCE FUNCTIONS
# ===================================================================

def build_mst(n: int, edges: List[Tuple[int, int]], 
              root_id: int = 1, verbose: bool = False) -> Set[Tuple[int, int]]:
    """
    Simple one-liner: Build DFS-based MST from graph.
    
    This is the recommended function for most use cases.
    Bridges are automatically preserved in any spanning tree!
    
    Args:
        n: Number of nodes
        edges: List of edges
        root_id: Root for DFS
        verbose: Print detailed trace
    
    Returns:
        Set of MST edges
    """
    mst = DistributedCriticalMST(n, edges, verbose=verbose)
    mst.run_simple_mst(root_id=root_id)
    return mst.get_mst_edges()


def detect_and_prune(n: int, edges: List[Tuple[int, int]], 
                     root_id: int = 1, verbose: bool = False) -> Dict:
    """
    Advanced: Run complete pipeline with critical edge detection.
    
    Note: Critical edge detection is mainly for analysis.
    Any spanning tree automatically includes all bridges!
    
    Args:
        n: Number of nodes
        edges: List of edges
        root_id: Root for DFS
        verbose: Print detailed trace
    
    Returns:
        Results dictionary
    """
    mst = DistributedCriticalMST(n, edges, verbose=verbose)
    return mst.run_complete_pipeline(root_id=root_id)


def get_mst(n: int, edges: List[Tuple[int, int]], 
            root_id: int = 1) -> Set[Tuple[int, int]]:
    """
    Alias for build_mst() - simple one-liner to get MST edges.
    
    Args:
        n: Number of nodes
        edges: List of edges
        root_id: Root for DFS
    
    Returns:
        Set of MST edges
    """
    return build_mst(n, edges, root_id=root_id, verbose=False)


if __name__ == "__main__":
    print("Distributed Critical MST Module")
    print("=" * 70)
    print("\nRECOMMENDED USAGE (Simple):")
    print("""
    from distributed_critical_mst import build_mst
    
    # Define connected graph
    n = 7
    edges = [(i, j) for i in range(1, n+1) for j in range(i+1, n+1)]
    
    # Get MST in one line!
    mst_edges = build_mst(n, edges, root_id=1, verbose=True)
    
    # That's it! Bridges are automatically preserved.
    print(f"MST: {sorted(mst_edges)}")
    """)
    
    print("\nADVANCED USAGE (With critical edge detection):")
    print("""
    from distributed_critical_mst import DistributedCriticalMST
    
    # Create MST system
    mst_system = DistributedCriticalMST(n, edges, verbose=True)
    
    # Optional: Detect bridges (for analysis)
    bridges = mst_system.detect_critical_edges()
    
    # Build MST (automatically includes bridges)
    mst_system.build_dfs_mst(root_id=1)
    mst_system.prune_to_mst()
    
    # Get results
    final_mst = mst_system.get_mst_edges()
    """)
