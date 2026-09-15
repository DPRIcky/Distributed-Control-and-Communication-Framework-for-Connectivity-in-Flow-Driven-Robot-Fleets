"""Greedy Distance Baseline - Prune longest edge without λ₂ check."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from typing import Dict, Optional
from core.types import Edge


class GreedyDistanceBaseline:
    """Baseline that greedily prunes the longest edge without checking connectivity.
    
    This represents a simple heuristic approach that doesn't verify algebraic
    connectivity (λ₂) before pruning. Shows importance of connectivity metrics.
    """
    
    def __init__(self, num_robots: int, prune_threshold: float = 2.5):
        """Initialize greedy distance baseline.
        
        Args:
            num_robots: Number of robots in the system
            prune_threshold: Only prune edges longer than this threshold (meters)
        """
        self.num_robots = num_robots
        self.prune_threshold = prune_threshold
        self.min_edges = num_robots - 1
    
    def find_edge_to_prune(
        self,
        edge_lengths: Dict[Edge, float],
        **kwargs
    ) -> Optional[Edge]:
        """Greedily prune the longest edge above threshold.
        
        Args:
            edge_lengths: Dictionary of current edge lengths
            
        Returns:
            Longest edge above threshold, or None
        """
        if len(edge_lengths) <= self.min_edges:
            return None
        
        # Find longest edge
        if not edge_lengths:
            return None
        
        longest_edge = max(edge_lengths.items(), key=lambda x: x[1])
        edge, length = longest_edge
        
        # Only prune if above threshold
        if length > self.prune_threshold:
            return edge
        
        return None
    
    def get_name(self) -> str:
        """Get baseline name for reporting.
        
        Returns:
            Baseline identifier string
        """
        return "GreedyDistance"
