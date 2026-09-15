"""
Obstacle representation for the environment.
"""

import numpy as np
from typing import List, Tuple


class Obstacle:
    """Represents a static obstacle in the environment."""

    def __init__(self, vertices: np.ndarray):
        """
        Initialize obstacle.

        Args:
            vertices: Nx2 array of obstacle vertices (in order)
        """
        self.vertices = vertices
        self.center = np.mean(vertices, axis=0)

    def distance_to_point(self, point: np.ndarray) -> Tuple[float, np.ndarray]:
        """
        Compute minimum distance from a point to the obstacle.

        Args:
            point: [x, y] query point

        Returns:
            distance: Minimum distance to obstacle
            closest_point: Closest point on obstacle boundary
        """
        min_dist = np.inf
        closest_point = None

        # Check distance to each edge
        n_vertices = len(self.vertices)
        for i in range(n_vertices):
            p1 = self.vertices[i]
            p2 = self.vertices[(i + 1) % n_vertices]

            dist, cp = self._distance_to_segment(point, p1, p2)
            if dist < min_dist:
                min_dist = dist
                closest_point = cp

        return min_dist, closest_point

    @staticmethod
    def _distance_to_segment(point: np.ndarray, p1: np.ndarray, p2: np.ndarray) -> Tuple[float, np.ndarray]:
        """
        Compute distance from point to line segment.

        Args:
            point: Query point
            p1, p2: Segment endpoints

        Returns:
            distance: Distance to segment
            closest_point: Closest point on segment
        """
        # Vector from p1 to p2
        v = p2 - p1
        # Vector from p1 to point
        w = point - p1

        # Parametric position along segment
        c1 = np.dot(w, v)
        if c1 <= 0:
            # Closest to p1
            return np.linalg.norm(point - p1), p1

        c2 = np.dot(v, v)
        if c1 >= c2:
            # Closest to p2
            return np.linalg.norm(point - p2), p2

        # Closest to interior point
        b = c1 / c2
        closest_point = p1 + b * v
        return np.linalg.norm(point - closest_point), closest_point

    def is_inside(self, point: np.ndarray) -> bool:
        """
        Check if a point is inside the obstacle using ray casting algorithm.

        Args:
            point: [x, y] query point

        Returns:
            True if point is inside obstacle
        """
        x, y = point
        n = len(self.vertices)
        inside = False

        p1x, p1y = self.vertices[0]
        for i in range(1, n + 1):
            p2x, p2y = self.vertices[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside


def create_rectangle_obstacle(center: np.ndarray, width: float, height: float) -> Obstacle:
    """
    Create a rectangular obstacle.

    Args:
        center: [x, y] center of rectangle
        width: Width of rectangle
        height: Height of rectangle

    Returns:
        Obstacle object
    """
    half_w = width / 2
    half_h = height / 2
    vertices = np.array([
        [center[0] - half_w, center[1] - half_h],
        [center[0] + half_w, center[1] - half_h],
        [center[0] + half_w, center[1] + half_h],
        [center[0] - half_w, center[1] + half_h]
    ])
    return Obstacle(vertices)


def create_wall_obstacle(p1: np.ndarray, p2: np.ndarray, thickness: float = 0.1) -> Obstacle:
    """
    Create a wall obstacle between two points.

    Args:
        p1, p2: Wall endpoints
        thickness: Wall thickness

    Returns:
        Obstacle object
    """
    # Direction vector
    direction = p2 - p1
    direction = direction / np.linalg.norm(direction)

    # Perpendicular vector
    perpendicular = np.array([-direction[1], direction[0]])
    offset = perpendicular * thickness / 2

    vertices = np.array([
        p1 - offset,
        p2 - offset,
        p2 + offset,
        p1 + offset
    ])
    return Obstacle(vertices)
