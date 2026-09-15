"""Centralized MST Baseline - Oracle with global knowledge."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from typing import Dict, List, Optional, Set
from core.types import Edge
from core.robot import ChainRobot


class CentralizedMSTBaseline:
    """Centralized baseline using global knowledge to maintain MST + safety margin.
    
    This is an idealized oracle approach that has access to all robot positions
    and edge lengths, computes the MST centrally, and only maintains edges that
    are in the MST or needed for redundancy. Serves as a lower bound for edge count.
    """
    
    def __init__(self, num_robots: int, safety_factor: float = 1.2):
        """Initialize centralized MST baseline.
        
        Args:
            num_robots: Number of robots in the system
            safety_factor: Keep MST edges × safety_factor for redundancy
        """
        self.num_robots = num_robots
        self.safety_factor = safety_factor
    
    def compute_mst(self, edge_lengths: Dict[Edge, float]) -> Set[Edge]:
        """Compute Minimum Spanning Tree using Kruskal's algorithm.
        
        Args:
            edge_lengths: Dictionary mapping edges to their lengths
            
        Returns:
            Set of edges in the MST
        """
        if not edge_lengths:
            return set()
        
        # Sort edges by length
        sorted_edges = sorted(edge_lengths.items(), key=lambda x: x[1])
        
        # Union-find data structure
        parent = list(range(self.num_robots))
        rank = [0] * self.num_robots
        
        def find(x):
            if parent[x] != x:
                parent[x] = find(parent[x])
            return parent[x]
        
        def union(x, y):
            px, py = find(x), find(y)
            if px == py:
                return False
            if rank[px] < rank[py]:
                parent[px] = py
            elif rank[px] > rank[py]:
                parent[py] = px
            else:
                parent[py] = px
                rank[px] += 1
            return True
        
        # Kruskal's algorithm
        mst_edges = set()
        for edge, length in sorted_edges:
            i, j = edge
            if union(i, j):
                mst_edges.add(edge)
                if len(mst_edges) == self.num_robots - 1:
                    break
        
        return mst_edges
    
    def find_edge_to_prune(
        self,
        edge_lengths: Dict[Edge, float],
        robots: Optional[List[ChainRobot]] = None
    ) -> Optional[Edge]:
        """Find edge to prune using centralized MST knowledge.
        
        Strategy: Prune the longest edge that is NOT in the MST and not
        needed for safety redundancy.
        
        Args:
            edge_lengths: Dictionary of current edge lengths
            robots: List of robots (optional, for additional logic)
            
        Returns:
            Edge to prune, or None if no safe pruning available
        """
        if len(edge_lengths) <= self.num_robots - 1:
            # Already at minimum connectivity (spanning tree)
            return None
        
        # Compute MST
        mst_edges = self.compute_mst(edge_lengths)
        
        # Target number of edges: MST + safety margin
        target_edges = int((self.num_robots - 1) * self.safety_factor)
        
        if len(edge_lengths) <= target_edges:
            return None
        
        # Find non-MST edges sorted by length (longest first)
        non_mst_edges = [(edge, length) for edge, length in edge_lengths.items() 
                         if edge not in mst_edges]
        
        if not non_mst_edges:
            return None
        
        # Prune longest non-MST edge
        non_mst_edges.sort(key=lambda x: x[1], reverse=True)
        return non_mst_edges[0][0]
    
    def get_name(self) -> str:
        """Get baseline name for reporting.
        
        Returns:
            Baseline identifier string
        """
        return "CentralizedMST"
