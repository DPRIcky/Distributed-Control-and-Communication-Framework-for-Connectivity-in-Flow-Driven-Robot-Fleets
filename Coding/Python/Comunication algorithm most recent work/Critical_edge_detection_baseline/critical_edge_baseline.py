"""Critical Edge Detection Baseline - Distributed MST with Bridge Preservation.

This baseline uses the distributed critical edge detection algorithm to:
1. Detect critical edges (bridges) in the communication graph
2. Build a DFS-based MST that preserves all critical edges
3. Prune non-MST edges in a distributed manner

Unlike centralized MST which requires global knowledge, this approach is
fully distributed and runs in O(n) rounds.
"""

import numpy as np
from typing import Dict, List, Optional, Set, Tuple
from collections import defaultdict

# Import types from core
try:
    from core.types import Edge
    from core.robot import ChainRobot
except ImportError:
    # Fallback for standalone testing
    Edge = Tuple[int, int]
    ChainRobot = None

# Handle both package and standalone imports
try:
    from .distributed_critical_mst import DistributedCriticalMST
except ImportError:
    from distributed_critical_mst import DistributedCriticalMST


class CriticalEdgeBaseline:
    """Distributed Critical Edge Detection Baseline.
    
    This baseline implements a distributed algorithm for MST construction
    that preserves all critical edges (bridges). Unlike centralized MST
    which requires global knowledge, this approach is fully distributed.
    
    Key Properties:
    - Fully distributed (no centralized oracle)
    - O(n) rounds time complexity
    - Automatically preserves all critical edges
    - Suitable for dynamic graphs
    """
    
    def __init__(
        self,
        num_robots: int,
        update_frequency: int = 10,
        prefer_critical: bool = True,
        verbose: bool = False
    ):
        """Initialize critical edge baseline.
        
        Args:
            num_robots: Number of robots in the system
            update_frequency: Update MST every N steps (default: 10)
            prefer_critical: Prioritize critical edges in MST construction
            verbose: Print detailed execution trace
        """
        self.num_robots = num_robots
        self.update_frequency = update_frequency
        self.prefer_critical = prefer_critical
        self.verbose = verbose
        
        # State tracking
        self.step_counter = 0
        self.current_mst: Set[Edge] = set()
        self.critical_edges: Set[Edge] = set()
        self.last_edge_set: Set[Edge] = set()
        
        # Statistics
        self.mst_updates = 0
        self.total_rounds = 0
        self.edges_pruned = 0
    
    def _edge_set_from_lengths(self, edge_lengths: Dict[Edge, float]) -> Set[Edge]:
        """Convert edge_lengths dict to edge set."""
        return set(edge_lengths.keys())
    
    def _has_topology_changed(self, current_edges: Set[Edge]) -> bool:
        """Check if topology has changed since last MST update."""
        return current_edges != self.last_edge_set
    
    def update_mst(self, edge_lengths: Dict[Edge, float]) -> Dict:
        """Update MST using distributed critical edge detection algorithm.
        
        This runs the full distributed pipeline:
        1. Detect critical edges (bridges)
        2. Build DFS-based MST
        3. Identify edges to prune
        
        Args:
            edge_lengths: Dictionary of current edge lengths
            
        Returns:
            Statistics about the MST update
        """
        if not edge_lengths:
            self.current_mst = set()
            self.critical_edges = set()
            return {'success': False, 'reason': 'no_edges'}
        
        # Convert 0-indexed edges to 1-indexed for DistributedCriticalMST
        edges_list_1indexed = [(i+1, j+1) for (i, j) in edge_lengths.keys()]
        
        # Create distributed MST system (expects 1-indexed nodes)
        mst_system = DistributedCriticalMST(
            n=self.num_robots,
            edges=edges_list_1indexed,
            verbose=self.verbose
        )
        
        # Run distributed pipeline
        if self.prefer_critical:
            # Full pipeline with critical edge detection
            results = mst_system.run_complete_pipeline(
                root_id=1,
                prefer_critical=True
            )
            # Convert critical edges back to 0-indexed
            self.critical_edges = {(i-1, j-1) for (i, j) in results['critical_edges']}
        else:
            # Simple DFS-based MST (faster)
            results = mst_system.run_simple_mst(root_id=1)
            self.critical_edges = set()
        
        # Convert MST edges back to 0-indexed
        self.current_mst = {(i-1, j-1) for (i, j) in results['mst_edges']}
        self.last_edge_set = set(edge_lengths.keys())
        self.mst_updates += 1
        self.total_rounds += results.get('mst_rounds', 0)
        
        return {
            'success': True,
            'mst_edges': len(self.current_mst),
            'critical_edges': len(self.critical_edges),
            'rounds': results.get('mst_rounds', 0),
            'edges_removed': len(edge_lengths) - len(self.current_mst)
        }
    
    def _verify_connectivity_after_removal(
        self, 
        edge_lengths: Dict[Edge, float],
        edge_to_remove: Edge
    ) -> bool:
        """Verify that removing an edge keeps the graph connected.
        
        Uses BFS to check if all nodes remain reachable after edge removal.
        
        Args:
            edge_lengths: Dictionary of current edge lengths  
            edge_to_remove: Edge to test removing
            
        Returns:
            True if graph stays connected, False if it would be disconnected
        """
        # Build adjacency list without the edge to remove
        adjacency = {i: set() for i in range(self.num_robots)}
        
        for (i, j) in edge_lengths.keys():
            if (i, j) != edge_to_remove and (j, i) != edge_to_remove:
                adjacency[i].add(j)
                adjacency[j].add(i)
        
        # BFS from node 0 to check connectivity
        if not adjacency[0]:
            return False  # Node 0 has no neighbors after removal
        
        visited = {0}
        queue = [0]
        
        while queue:
            node = queue.pop(0)
            for neighbor in adjacency[node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        
        # Check if all nodes were reached
        return len(visited) == self.num_robots
    
    def _is_graph_connected(self, edge_lengths: Dict[Edge, float]) -> bool:
        """Check if the current graph is fully connected.
        
        Args:
            edge_lengths: Dictionary of current edge lengths
            
        Returns:
            True if all robots are in one connected component, False otherwise
        """
        if not edge_lengths:
            return self.num_robots <= 1  # Single robot is trivially connected
        
        # Build adjacency list
        adjacency = {i: set() for i in range(self.num_robots)}
        for (i, j) in edge_lengths.keys():
            adjacency[i].add(j)
            adjacency[j].add(i)
        
        # BFS from node 0
        if not adjacency[0]:
            return False  # Node 0 is isolated
        
        visited = {0}
        queue = [0]
        
        while queue:
            node = queue.pop(0)
            for neighbor in adjacency[node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        
        # Check if all nodes were reached
        return len(visited) == self.num_robots
    
    def find_edge_to_prune(
        self,
        edge_lengths: Dict[Edge, float],
        robots: Optional[List] = None
    ) -> Optional[Edge]:
        """Find edge to prune using distributed critical edge detection.
        
        Strategy:
        1. Check if graph is connected (if not, don't prune to preserve connectivity)
        2. Periodically update MST using distributed algorithm
        3. Prune edges that are NOT in the current MST
        4. Prune longest non-MST edge first
        5. Never disconnect the graph
        
        Args:
            edge_lengths: Dictionary of current edge lengths
            robots: List of robots (optional)
            
        Returns:
            Edge to prune, or None if no safe pruning available or graph disconnected/too small
        """
        self.step_counter += 1
        current_edges = self._edge_set_from_lengths(edge_lengths)
        
        # SAFETY: If graph is currently disconnected, don't prune anything
        # This preserves whatever connectivity remains
        if not self._is_graph_connected(edge_lengths):
            return None
        
        # Don't prune if graph is already minimal
        if len(edge_lengths) <= self.num_robots - 1:
            return None
        
        # Check if we need to update MST
        should_update = (
            self.step_counter % self.update_frequency == 0 or
            self._has_topology_changed(current_edges) or
            len(self.current_mst) == 0
        )
        
        if should_update:
            if self.verbose:
                print(f"\n[CriticalEdge] Updating MST at step {self.step_counter}")
            
            update_results = self.update_mst(edge_lengths)
            
            if self.verbose and update_results['success']:
                print(f"[CriticalEdge] MST has {update_results['mst_edges']} edges")
                print(f"[CriticalEdge] Found {update_results['critical_edges']} critical edges")
        
        # Find non-MST edges
        non_mst_edges = []
        for edge, length in edge_lengths.items():
            # Normalize edge (ensure consistent ordering)
            normalized_edge = tuple(sorted(edge))
            
            # Check if edge is in MST (try both orderings)
            in_mst = (
                edge in self.current_mst or
                (edge[1], edge[0]) in self.current_mst or
                normalized_edge in self.current_mst or
                (normalized_edge[1], normalized_edge[0]) in self.current_mst
            )
            
            if not in_mst:
                non_mst_edges.append((edge, length))
        
        # If no non-MST edges, we're at minimum connectivity
        if not non_mst_edges:
            return None
        
        # Sort by length (longest first) and try to prune while maintaining connectivity
        non_mst_edges.sort(key=lambda x: x[1], reverse=True)
        
        for edge_to_prune, _ in non_mst_edges:
            # Verify this edge can be safely pruned without disconnecting
            if self._verify_connectivity_after_removal(edge_lengths, edge_to_prune):
                self.edges_pruned += 1
                
                if self.verbose:
                    print(f"[CriticalEdge] Pruning non-MST edge {edge_to_prune}")
                
                return edge_to_prune
        
        # No non-MST edge can be pruned without disconnecting
        return None
    
    def get_name(self) -> str:
        """Get baseline name for reporting.
        
        Returns:
            Baseline identifier string
        """
        return "CriticalEdgeMST"
    
    def get_statistics(self) -> Dict:
        """Get statistics about the baseline performance.
        
        Returns:
            Dictionary containing performance metrics
        """
        return {
            'name': self.get_name(),
            'mst_updates': self.mst_updates,
            'total_rounds': self.total_rounds,
            'edges_pruned': self.edges_pruned,
            'current_mst_size': len(self.current_mst),
            'critical_edges_detected': len(self.critical_edges),
            'avg_rounds_per_update': (
                self.total_rounds / self.mst_updates if self.mst_updates > 0 else 0
            )
        }
    
    def reset(self):
        """Reset baseline state."""
        self.step_counter = 0
        self.current_mst = set()
        self.critical_edges = set()
        self.last_edge_set = set()
        self.mst_updates = 0
        self.total_rounds = 0
        self.edges_pruned = 0


# Convenience function for one-shot use
def get_critical_edge_mst(
    num_robots: int,
    edge_lengths: Dict[Edge, float],
    prefer_critical: bool = True,
    verbose: bool = False
) -> Set[Edge]:
    """One-shot function to get MST using critical edge detection.
    
    Args:
        num_robots: Number of robots
        edge_lengths: Dictionary of edge lengths
        prefer_critical: Prioritize critical edges
        verbose: Print detailed trace
        
    Returns:
        Set of MST edges
    """
    baseline = CriticalEdgeBaseline(
        num_robots=num_robots,
        prefer_critical=prefer_critical,
        verbose=verbose
    )
    baseline.update_mst(edge_lengths)
    return baseline.current_mst


if __name__ == "__main__":
    print("Critical Edge Baseline Module")
    print("=" * 70)
    print("\nThis module provides a distributed baseline for MST construction")
    print("that preserves critical edges (bridges).")
    print("\nUSAGE:")
    print("""
    from critical_edge_baseline import CriticalEdgeBaseline
    
    # Initialize baseline
    baseline = CriticalEdgeBaseline(num_robots=10)
    
    # In simulation loop
    edge_to_prune = baseline.find_edge_to_prune(edge_lengths)
    if edge_to_prune:
        # Prune the edge
        prune_edge(edge_to_prune)
    
    # Get statistics
    stats = baseline.get_statistics()
    print(stats)
    """)
