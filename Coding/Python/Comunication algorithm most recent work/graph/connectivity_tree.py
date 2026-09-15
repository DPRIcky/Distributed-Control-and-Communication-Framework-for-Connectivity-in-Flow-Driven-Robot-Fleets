"""Connectivity tree builder using BFS."""

from typing import List, Set

import numpy as np

from core.robot import ChainRobot


class ConnectivityTree:
    """BFS-based connectivity tree builder."""

    def __init__(self, num_robots: int, workspace_size: tuple):
        """Initialize connectivity tree builder.
        
        Args:
            num_robots: Number of robots
            workspace_size: Size of workspace (width, height)
        """
        self.num_robots = num_robots
        self.workspace_size = workspace_size

    def build_tree(
        self,
        robots: List[ChainRobot],
        neighbor_graph: List[Set[int]]
    ) -> None:
        """Build tree structure via BFS from closest robot to cluster center.
        
        Args:
            robots: List of all robots
            neighbor_graph: Adjacency list representation
        """
        # Reset all parent assignments
        for robot in robots:
            robot.parent = None

        # Find robot closest to cluster center as root
        cluster_center = np.array([self.workspace_size[0] * 0.25, self.workspace_size[1] * 0.5])
        distances_to_center = [
            np.linalg.norm(robot.position - cluster_center)
            for robot in robots
        ]
        root_id = int(np.argmin(distances_to_center))

        # BFS to build tree
        visited = {root_id}
        queue = [root_id]

        while queue:
            current_id = queue.pop(0)
            for neighbor_id in neighbor_graph[current_id]:
                if neighbor_id not in visited:
                    robots[neighbor_id].parent = current_id
                    visited.add(neighbor_id)
                    queue.append(neighbor_id)
