"""Graph topology management."""

from typing import Dict, List, Set

import numpy as np

from core.robot import ChainRobot
from core.types import Edge


class TopologyManager:
    """Manages communication graph topology."""

    def __init__(self, num_robots: int, communication_radius: float):
        """Initialize topology manager.
        
        Args:
            num_robots: Number of robots in the system
            communication_radius: Maximum communication range
        """
        self.num_robots = num_robots
        self.communication_radius = communication_radius
        self.neighbor_graph: List[Set[int]] = [set() for _ in range(num_robots)]
        self.pruned_edges: Set[Edge] = set()

    def update_neighbor_graph(self, robots: List[ChainRobot]) -> None:
        """Build adjacency list from robot positions.
        
        Args:
            robots: List of all robots
        """
        graph: List[Set[int]] = [set() for _ in range(self.num_robots)]
        
        for i in range(self.num_robots):
            for j in range(i + 1, self.num_robots):
                key = (min(i, j), max(i, j))
                if key in self.pruned_edges:
                    continue
                    
                distance = self._distance(robots[i], robots[j])
                if distance <= self.communication_radius:
                    graph[i].add(j)
                    graph[j].add(i)
                    
        self.neighbor_graph = graph

    def get_current_edges(self) -> List[Edge]:
        """Extract edge list from adjacency graph.
        
        Returns:
            List of edges as (i, j) tuples where i < j
        """
        edges: List[Edge] = []
        for i in range(self.num_robots):
            for j in self.neighbor_graph[i]:
                if j > i:
                    edges.append((i, j))
        return edges

    def gather_edge_lengths(self, robots: List[ChainRobot]) -> Dict[Edge, float]:
        """Compute lengths of all current edges.
        
        Args:
            robots: List of all robots
            
        Returns:
            Dictionary mapping edges to their lengths
        """
        lengths: Dict[Edge, float] = {}
        for i in range(self.num_robots):
            for j in self.neighbor_graph[i]:
                if j <= i:
                    continue
                key = (min(i, j), max(i, j))
                if key in self.pruned_edges:
                    continue
                lengths[key] = self._distance(robots[i], robots[j])
        return lengths

    def prune_edge(self, edge: Edge, robots: List[ChainRobot]) -> None:
        """Remove an edge from the graph.
        
        Args:
            edge: Edge to remove
            robots: List of all robots
        """
        a, b = edge
        
        # Remove from neighbor graph
        if b in self.neighbor_graph[a]:
            self.neighbor_graph[a].discard(b)
        if a in self.neighbor_graph[b]:
            self.neighbor_graph[b].discard(a)
            
        # Add to pruned set
        self.pruned_edges.add(edge)
        
        # Update robot records
        robots[a].removed_edges.append(edge)
        robots[b].removed_edges.append((b, a))

    @staticmethod
    def _distance(robot1: ChainRobot, robot2: ChainRobot) -> float:
        """Compute Euclidean distance between two robots.
        
        Args:
            robot1: First robot
            robot2: Second robot
            
        Returns:
            Distance between robots
        """
        return float(np.linalg.norm(robot1.position - robot2.position))
