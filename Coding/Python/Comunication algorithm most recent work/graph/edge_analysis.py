"""Edge analysis utilities for redundancy and connectivity checks."""

from typing import Dict, Set, Optional

import numpy as np

from core.types import Edge


class EdgeAnalyzer:
    """Analyzes edge redundancy and graph properties."""

    def __init__(self, num_robots: int):
        """Initialize edge analyzer.
        
        Args:
            num_robots: Number of robots in the system
        """
        self.num_robots = num_robots

    def has_alternative_path(self, edge: Edge, edge_set: Set[Edge]) -> bool:
        """Check if alternative path exists between edge endpoints without using the edge.
        
        This checks GRAPH connectivity, not tree connectivity to a specific root.
        
        Args:
            edge: Edge to test
            edge_set: Set of all current edges
            
        Returns:
            True if path exists without the edge
        """
        a, b = edge

        # Build adjacency from edge set (excluding the candidate edge)
        adj: Dict[int, Set[int]] = {i: set() for i in range(self.num_robots)}
        for e in edge_set:
            if e == edge:
                continue
            u, v = e
            adj[u].add(v)
            adj[v].add(u)

        # BFS to find ANY path from a to b
        visited = {a}
        queue = [a]

        while queue:
            node = queue.pop(0)
            if node == b:
                return True
            for neighbor in adj[node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)

        return False

    def check_graph_connectivity(self, edge_set: Set[Edge]) -> bool:
        """Check if the graph is fully connected (all robots can reach each other).
        
        Args:
            edge_set: Set of edges in the graph
            
        Returns:
            True if graph is fully connected
        """
        if not edge_set:
            return False

        # Build adjacency
        adj: Dict[int, Set[int]] = {i: set() for i in range(self.num_robots)}
        for u, v in edge_set:
            adj[u].add(v)
            adj[v].add(u)

        # BFS from robot 0 - can we reach everyone?
        visited = {0}
        queue = [0]

        while queue:
            node = queue.pop(0)
            for neighbor in adj[node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)

        # All robots must be reachable
        return len(visited) == self.num_robots

    # ------------------------------------------------------------------ #
    # Distributed methods using consensus estimates (NO GLOBAL KNOWLEDGE)
    # ------------------------------------------------------------------ #

    def has_alternative_path_from_consensus(
        self,
        robot_id: int,
        edge: Edge,
        A_estimate: np.ndarray,
        threshold: float = 0.1
    ) -> bool:
        """Check if alternative path exists using robot's local consensus estimate.
        
        DISTRIBUTED: Uses consensused adjacency matrix (not global edge set).
        After consensus convergence, all robots compute same answer.
        
        Args:
            robot_id: Robot performing the check
            edge: Edge to test (a, b)
            A_estimate: Robot's adjacency matrix estimate (from consensus)
            threshold: Minimum link quality to consider edge present
            
        Returns:
            True if alternative path exists (safe to remove edge)
        """
        a, b = edge
        n = A_estimate.shape[0]
        
        # Build adjacency from consensus estimate (excluding candidate edge)
        adj: Dict[int, Set[int]] = {i: set() for i in range(n)}
        
        for i in range(n):
            for j in range(i + 1, n):
                # Edge exists if quality > threshold
                if A_estimate[i, j] > threshold:
                    # Exclude the edge we're testing
                    if (i, j) != edge and (j, i) != edge:
                        adj[i].add(j)
                        adj[j].add(i)
        
        # BFS from a to b
        if a not in adj or b not in adj:
            return False
        
        visited = {a}
        queue = [a]
        
        while queue:
            node = queue.pop(0)
            if node == b:
                return True
            for neighbor in adj[node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        
        return False

    def check_connectivity_from_consensus(
        self,
        robot_id: int,
        A_estimate: np.ndarray,
        threshold: float = 0.1
    ) -> bool:
        """Check graph connectivity using robot's consensus estimate.
        
        DISTRIBUTED: Each robot computes independently using local estimate.
        After consensus, all robots get same answer.
        
        Args:
            robot_id: Robot performing the check
            A_estimate: Robot's adjacency matrix estimate (from consensus)
            threshold: Minimum link quality to consider edge present
            
        Returns:
            True if graph is connected
        """
        n = A_estimate.shape[0]
        
        # Build adjacency from estimate
        adj: Dict[int, Set[int]] = {i: set() for i in range(n)}
        
        for i in range(n):
            for j in range(i + 1, n):
                if A_estimate[i, j] > threshold:
                    adj[i].add(j)
                    adj[j].add(i)
        
        # BFS from node 0 to check reachability
        visited = {0}
        queue = [0]
        
        while queue:
            node = queue.pop(0)
            for neighbor in adj[node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        
        # All nodes must be reachable
        return len(visited) == n

    def extract_edges_from_consensus(
        self,
        A_estimate: np.ndarray,
        threshold: float = 0.1
    ) -> Set[Edge]:
        """Extract edge set from adjacency matrix estimate.
        
        Converts continuous adjacency estimates to binary edge set.
        
        Args:
            A_estimate: Adjacency matrix estimate (n×n)
            threshold: Minimum quality to consider edge present
            
        Returns:
            Set of edges where A_ij > threshold
        """
        n = A_estimate.shape[0]
        edges: Set[Edge] = set()
        
        for i in range(n):
            for j in range(i + 1, n):
                if A_estimate[i, j] > threshold:
                    edges.add((i, j))
        
        return edges
