"""Full Graph Baseline - No pruning (upper bound on connectivity)."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from typing import Optional
from core.types import Edge


class FullGraphBaseline:
    """Baseline that never prunes edges - maintains full communication graph.
    
    This serves as an upper bound for connectivity metrics and shows the cost
    of maintaining all edges (communication overhead, control effort).
    """
    
    def __init__(self, num_robots: int):
        """Initialize full graph baseline.
        
        Args:
            num_robots: Number of robots in the system
        """
        self.num_robots = num_robots
    
    def should_prune_edge(self, *args, **kwargs) -> bool:
        """Always return False - never prune.
        
        Returns:
            False (never prune any edge)
        """
        return False
    
    def find_edge_to_prune(self, *args, **kwargs) -> Optional[Edge]:
        """Return None - no edge should be pruned.
        
        Returns:
            None (no pruning)
        """
        return None
    
    def get_name(self) -> str:
        """Get baseline name for reporting.
        
        Returns:
            Baseline identifier string
        """
        return "FullGraph"
