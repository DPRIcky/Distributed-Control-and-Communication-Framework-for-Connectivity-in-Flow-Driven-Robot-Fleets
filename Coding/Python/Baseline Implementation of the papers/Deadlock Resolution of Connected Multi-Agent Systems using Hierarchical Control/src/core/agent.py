"""
Agent class implementing unicycle dynamics for multi-agent system.
"""

import numpy as np
from typing import Tuple, List


class Agent:
    """
    Agent with unicycle dynamics: x_dot = v*cos(theta), y_dot = v*sin(theta), theta_dot = omega
    """

    def __init__(self, agent_id: int, initial_state: np.ndarray, goal: np.ndarray,
                 sensing_radius: float = 1.7, safe_distance: float = 0.1):
        """
        Initialize agent.

        Args:
            agent_id: Unique identifier for the agent
            initial_state: [x, y, theta] initial state
            goal: [x_g, y_g] goal location
            sensing_radius: Sensing radius Rs
            safe_distance: Minimum safe distance dm
        """
        self.id = agent_id
        self.state = initial_state.copy()  # [x, y, theta]
        self.goal = goal.copy()  # [x_g, y_g]
        self.goal_temp = goal.copy()  # Temporary goal (can be updated by high-level planner)

        self.sensing_radius = sensing_radius
        self.safe_distance = safe_distance

        # Control inputs
        self.control = np.array([0.0, 0.0])  # [v, omega]
        self.prev_control = np.array([0.0, 0.0])

        # Leader-follower information
        self.leader_id = None
        self.is_leader = False

        # Neighbors and network information
        self.neighbors = set()
        self.safe_neighbors = set()  # Neighbors that can be removed without breaking connectivity

        # Bidding information
        self.bid_value = 0.0
        self.bid_neighbor = None

    @property
    def position(self) -> np.ndarray:
        """Get agent position [x, y]."""
        return self.state[:2]

    @property
    def theta(self) -> float:
        """Get agent orientation."""
        return self.state[2]

    def dynamics(self, control: np.ndarray) -> np.ndarray:
        """
        Compute state derivative for unicycle dynamics.

        Args:
            control: [v, omega] control input

        Returns:
            state_dot: [x_dot, y_dot, theta_dot]
        """
        v, omega = control
        x_dot = v * np.cos(self.theta)
        y_dot = v * np.sin(self.theta)
        theta_dot = omega

        return np.array([x_dot, y_dot, theta_dot])

    def update(self, control: np.ndarray, dt: float):
        """
        Update agent state using Euler integration.

        Args:
            control: [v, omega] control input
            dt: Time step
        """
        self.prev_control = self.control.copy()
        self.control = control.copy()

        # Euler integration
        state_dot = self.dynamics(control)
        self.state = self.state + state_dot * dt

        # Normalize theta to [-pi, pi]
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))

    def distance_to_agent(self, other: 'Agent') -> float:
        """Compute distance to another agent."""
        return np.linalg.norm(self.position - other.position)

    def distance_to_point(self, point: np.ndarray) -> float:
        """Compute distance to a point."""
        return np.linalg.norm(self.position - point)

    def distance_to_goal(self) -> float:
        """Compute distance to goal."""
        return np.linalg.norm(self.position - self.goal)

    def distance_to_temp_goal(self) -> float:
        """Compute distance to temporary goal."""
        return np.linalg.norm(self.position - self.goal_temp)

    def is_at_goal(self, threshold: float = 0.1) -> bool:
        """Check if agent has reached its goal."""
        return self.distance_to_goal() < threshold

    def get_velocity(self) -> np.ndarray:
        """Get agent velocity [vx, vy]."""
        v = self.control[0]
        return np.array([v * np.cos(self.theta), v * np.sin(self.theta)])

    def get_speed(self) -> float:
        """Get agent speed."""
        return np.linalg.norm(self.get_velocity())

    def set_temporary_goal(self, goal_temp: np.ndarray):
        """Set temporary goal for deadlock resolution."""
        self.goal_temp = goal_temp.copy()

    def reset_temporary_goal(self):
        """Reset temporary goal to actual goal."""
        self.goal_temp = self.goal.copy()

    def __repr__(self) -> str:
        return f"Agent {self.id}: pos={self.position}, goal={self.goal}"
