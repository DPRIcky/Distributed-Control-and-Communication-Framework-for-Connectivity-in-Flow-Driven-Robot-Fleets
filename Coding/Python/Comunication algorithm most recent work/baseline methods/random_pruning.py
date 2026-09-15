"""Random Pruning Baseline - Naive random edge removal."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from typing import Dict, Optional
from core.types import Edge


class RandomPruningBaseline:
    """Baseline that randomly prunes edges without considering topology.
    
    This demonstrates what happens without intelligent pruning - serves as
    a lower performance bound showing the importance of connectivity-aware pruning.
    """
    
    def __init__(self, num_robots: int, prune_probability: float = 0.1, seed: int = 42):
        """Initialize random pruning baseline.
        
        Args:
            num_robots: Number of robots in the system
            prune_probability: Probability of pruning an edge each time step
            seed: Random seed for reproducibility
        """
        self.num_robots = num_robots
        self.prune_probability = prune_probability
        self.rng = np.random.default_rng(seed)
        self.min_edges = num_robots - 1  # Never go below spanning tree
    
    def find_edge_to_prune(
        self,
        edge_lengths: Dict[Edge, float],
        **kwargs
    ) -> Optional[Edge]:
        """Randomly select an edge to prune.
        
        Args:
            edge_lengths: Dictionary of current edge lengths
            
        Returns:
            Random edge to prune, or None based on probability
        """
        if len(edge_lengths) <= self.min_edges:
            return None
        
        # Random decision to prune
        if self.rng.random() > self.prune_probability:
            return None
        
        # Select random edge
        edges = list(edge_lengths.keys())
        return edges[self.rng.integers(0, len(edges))]
    
    def get_name(self) -> str:
        """Get baseline name for reporting.
        
        Returns:
            Baseline identifier string
        """
        return "RandomPruning"
