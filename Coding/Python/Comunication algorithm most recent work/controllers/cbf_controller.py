"""Control Barrier Function controller."""

from typing import List, Optional

import numpy as np

from config.control_config import ControlConfig
from core.robot import ChainRobot


class CBFController:
    """Control Barrier Function for safety and connectivity."""

    def __init__(self, config: ControlConfig, communication_radius: float):
        """Initialize CBF controller.
        
        Args:
            config: Control configuration
            communication_radius: Maximum communication range
        """
        self.safety_distance = config.safety_distance
        self.cbf_safety_gain = config.cbf_safety_gain
        self.cbf_connectivity_gain = config.cbf_connectivity_gain
        self.communication_radius = communication_radius

    def compute_safety_control(
        self,
        robot_id: int,
        robot: ChainRobot,
        all_robots: List[ChainRobot]
    ) -> np.ndarray:
        """Compute collision avoidance control.
        
        Args:
            robot_id: ID of the robot
            robot: The robot to control
            all_robots: List of all robots
            
        Returns:
            Safety control force
        """
        control = np.zeros(2)

        for other_id, other_robot in enumerate(all_robots):
            if other_id == robot_id:
                continue

            diff = robot.position - other_robot.position
            dist = np.linalg.norm(diff)

            if dist < 1e-6:
                diff = robot.rng.normal(0.0, 0.1, size=2)
                dist = np.linalg.norm(diff)

            if dist < self.safety_distance * 2.0:
                barrier_value = dist**2 - self.safety_distance**2
                if barrier_value < 0.1:
                    direction = diff / dist
                    safety_control = self.cbf_safety_gain * (-barrier_value) * direction
                    control += safety_control

        return control

    def compute_connectivity_control(
        self,
        robot: ChainRobot,
        parent_robot: Optional[ChainRobot],
        critical_edges: Optional[dict] = None,
        robot_id: Optional[int] = None,
        all_robots: Optional[List[ChainRobot]] = None
    ) -> np.ndarray:
        """Compute connectivity maintenance control for critical edges.
        
        Args:
            robot: The robot to control
            parent_robot: The parent robot (for tree-based connectivity)
            critical_edges: Dict mapping robot_id to list of critical neighbor IDs
            robot_id: Current robot's ID
            all_robots: List of all robots
            
        Returns:
            Connectivity control force (sum of all critical edge constraints)
        """
        control = np.zeros(2)
        
        # First: maintain critical edges if provided
        if critical_edges is not None and robot_id is not None and all_robots is not None:
            critical_neighbors = critical_edges.get(robot_id, [])
            
            for neighbor_id in critical_neighbors:
                if neighbor_id >= len(all_robots):
                    continue
                    
                neighbor_robot = all_robots[neighbor_id]
                diff_to_neighbor = robot.position - neighbor_robot.position
                dist_to_neighbor = np.linalg.norm(diff_to_neighbor)
                
                if dist_to_neighbor > 1e-6:
                    # Keep critical edges at 95% of communication radius
                    max_distance = self.communication_radius * 0.95
                    barrier_value = max_distance**2 - dist_to_neighbor**2
                    
                    # Use lower threshold for earlier activation (be more reactive)
                    # Activate when at 85% of comm radius, trigger hard at 95%
                    soft_threshold = (self.communication_radius * 0.85)**2 - (self.communication_radius * 0.95)**2
                    
                    if barrier_value < soft_threshold:
                        # Proportional control that increases as we approach the limit
                        # When dist=0.85r: barrier_value = 0 (no control)
                        # When dist=0.95r: barrier_value = -soft_threshold (max control)
                        normalized_violation = 1.0 - max(0, barrier_value) / soft_threshold
                        
                        direction_to_neighbor = -diff_to_neighbor / dist_to_neighbor
                        # Use exponential scaling for more aggressive control near limit
                        control_magnitude = self.cbf_connectivity_gain * (normalized_violation ** 1.5)
                        critical_control = control_magnitude * direction_to_neighbor
                        control += critical_control * 0.5  # Scale down when multiple constraints
        
        # Second: maintain parent connection (tree-based backup)
        if parent_robot is not None:
            diff_to_parent = robot.position - parent_robot.position
            dist_to_parent = np.linalg.norm(diff_to_parent)

            if dist_to_parent > 1e-6:
                max_distance = self.communication_radius * 0.95
                barrier_value = max_distance**2 - dist_to_parent**2

                # Same enhanced logic for parent connection
                soft_threshold = (self.communication_radius * 0.85)**2 - (self.communication_radius * 0.95)**2
                
                if barrier_value < soft_threshold:
                    normalized_violation = 1.0 - max(0, barrier_value) / soft_threshold
                    direction_to_parent = -diff_to_parent / dist_to_parent
                    control_magnitude = self.cbf_connectivity_gain * (normalized_violation ** 1.5)
                    connectivity_control = control_magnitude * direction_to_parent
                    control += connectivity_control

        return control
