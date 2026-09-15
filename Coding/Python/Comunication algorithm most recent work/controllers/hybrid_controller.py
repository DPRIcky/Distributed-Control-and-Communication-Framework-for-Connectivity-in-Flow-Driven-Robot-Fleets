"""Hybrid CLF-CBF controller."""

from typing import List

import numpy as np

from config.control_config import ControlConfig
from controllers.clf_controller import CLFController
from controllers.cbf_controller import CBFController
from core.robot import ChainRobot


class HybridCLFCBFController:
    """Unified CLF-CBF controller combining goal-seeking and safety."""

    def __init__(self, config: ControlConfig, communication_radius: float, goal_position: np.ndarray):
        """Initialize hybrid controller.
        
        Args:
            config: Control configuration
            communication_radius: Maximum communication range
            goal_position: Goal position for all robots
        """
        self.clf = CLFController(config)
        self.cbf = CBFController(config, communication_radius)
        self.goal_position = goal_position
        self.max_control = config.max_control_force

    def compute_control(
        self,
        robot_id: int,
        robots: List[ChainRobot],
        critical_edges: dict = None
    ) -> np.ndarray:
        """Compute combined CLF-CBF control input.
        
        Args:
            robot_id: ID of the robot to control
            robots: List of all robots
            critical_edges: Dict mapping robot_id to list of critical neighbor robot_ids
            
        Returns:
            Combined control force
        """
        robot = robots[robot_id]
        control = np.zeros(2)

        # CLF: Goal-seeking
        has_parent = robot.parent is not None
        is_root = (robot_id == 0)
        clf_control = self.clf.compute_control(
            robot.position,
            self.goal_position,
            has_parent,
            is_root
        )
        control += clf_control

        # CBF Safety: Collision avoidance
        safety_control = self.cbf.compute_safety_control(robot_id, robot, robots)
        control += safety_control

        # CBF Connectivity: Maintain critical edges (spanning tree edges)
        parent_robot = robots[robot.parent] if robot.parent is not None else None
        connectivity_control = self.cbf.compute_connectivity_control(
            robot, 
            parent_robot,
            critical_edges=critical_edges,
            robot_id=robot_id,
            all_robots=robots
        )
        control += connectivity_control

        # Limit control magnitude
        control_magnitude = np.linalg.norm(control)
        if control_magnitude > self.max_control:
            control = control * (self.max_control / control_magnitude)

        return control
