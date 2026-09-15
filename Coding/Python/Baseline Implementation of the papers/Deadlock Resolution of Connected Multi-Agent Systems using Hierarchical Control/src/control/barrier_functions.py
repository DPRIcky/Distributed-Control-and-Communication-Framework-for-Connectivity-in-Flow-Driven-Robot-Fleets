"""
Control Barrier Functions (CBFs) and Control Lyapunov Functions (CLFs).
"""

import numpy as np
from typing import Tuple, Optional


class BarrierFunction:
    """Base class for barrier functions."""

    def __init__(self):
        pass

    def evaluate(self, state: np.ndarray) -> float:
        """Evaluate barrier function."""
        raise NotImplementedError

    def grad_state(self, state: np.ndarray) -> np.ndarray:
        """Gradient with respect to state."""
        raise NotImplementedError


class ObstacleBarrierPosition:
    """
    Barrier function for obstacle avoidance (position).
    B_l,p(p_i) = 0.5 * min_{p_o in O_l} ||p_i - p_o||^2 - 0.5 * r_obs^2
    """

    def __init__(self, obstacle_distance: float, obstacle_point: np.ndarray, r_obs: float):
        """
        Args:
            obstacle_distance: Distance to closest point on obstacle
            obstacle_point: Closest point on obstacle
            r_obs: Safe distance from obstacle
        """
        self.obstacle_distance = obstacle_distance
        self.obstacle_point = obstacle_point
        self.r_obs = r_obs

    def evaluate(self, position: np.ndarray) -> float:
        """Evaluate barrier function."""
        dist_sq = np.linalg.norm(position - self.obstacle_point) ** 2
        return 0.5 * dist_sq - 0.5 * self.r_obs ** 2

    def lie_derivative_f(self, state: np.ndarray) -> float:
        """
        Lie derivative along drift dynamics (zero for unicycle).

        Args:
            state: [x, y, theta]
        """
        return 0.0

    def lie_derivative_g(self, state: np.ndarray) -> np.ndarray:
        """
        Lie derivative along control dynamics.

        Args:
            state: [x, y, theta]

        Returns:
            [L_g1 B, L_g2 B] derivatives w.r.t. [v, omega]
        """
        position = state[:2]
        theta = state[2]

        # Gradient of B w.r.t. position
        grad_p = position - self.obstacle_point

        # Dynamics: p_dot = [v*cos(theta), v*sin(theta)]
        # L_g1 B = grad_p^T * [cos(theta), sin(theta)]
        # L_g2 B = 0 (omega doesn't affect position directly)

        Lg1 = grad_p[0] * np.cos(theta) + grad_p[1] * np.sin(theta)
        Lg2 = 0.0

        return np.array([Lg1, Lg2])


class ObstacleBarrierAngle:
    """
    Barrier function for steering away from obstacle.
    B_l,θ(θ_i) = 0.5 * |θ_i - θ_o|^2 - 0.5 * θ_threshold^2
    """

    def __init__(self, theta: float, theta_obstacle: float, theta_threshold: float):
        """
        Args:
            theta: Current orientation
            theta_obstacle: Desired orientation away from obstacle
            theta_threshold: Threshold angle
        """
        self.theta = theta
        self.theta_obstacle = theta_obstacle
        self.theta_threshold = theta_threshold

    def evaluate(self, theta: float) -> float:
        """Evaluate barrier function."""
        angle_diff = self._angle_difference(theta, self.theta_obstacle)
        return 0.5 * angle_diff ** 2 - 0.5 * self.theta_threshold ** 2

    def lie_derivative_f(self, state: np.ndarray) -> float:
        """Lie derivative along drift dynamics."""
        return 0.0

    def lie_derivative_g(self, state: np.ndarray) -> np.ndarray:
        """
        Lie derivative along control dynamics.

        Args:
            state: [x, y, theta]
        """
        theta = state[2]
        angle_diff = self._angle_difference(theta, self.theta_obstacle)

        # L_g1 B = 0 (v doesn't affect theta)
        # L_g2 B = angle_diff (since theta_dot = omega)

        return np.array([0.0, angle_diff])

    @staticmethod
    def _angle_difference(angle1: float, angle2: float) -> float:
        """Compute signed angle difference."""
        diff = angle1 - angle2
        return np.arctan2(np.sin(diff), np.cos(diff))


class AgentBarrierPosition:
    """
    Barrier function for inter-agent collision avoidance (position).
    B_ij,p = 0.5 * ||p_i - p_j||^2 - (2*d_m)^2
    """

    def __init__(self, safe_distance: float):
        """
        Args:
            safe_distance: Minimum safe distance between agents (d_m)
        """
        self.safe_distance = safe_distance

    def evaluate(self, pos_i: np.ndarray, pos_j: np.ndarray) -> float:
        """Evaluate barrier function."""
        dist_sq = np.linalg.norm(pos_i - pos_j) ** 2
        return 0.5 * dist_sq - 0.5 * (2 * self.safe_distance) ** 2

    def lie_derivative(self, state_i: np.ndarray, state_j: np.ndarray,
                      control_j: np.ndarray) -> Tuple[float, np.ndarray]:
        """
        Compute Lie derivatives.

        Args:
            state_i: [x_i, y_i, theta_i]
            state_j: [x_j, y_j, theta_j]
            control_j: [v_j, omega_j]

        Returns:
            Lf_B: Lie derivative along drift
            Lg_B: Lie derivative along control [L_g1, L_g2]
        """
        pos_i = state_i[:2]
        pos_j = state_j[:2]
        theta_i = state_i[2]
        theta_j = state_j[2]

        # Relative position
        rel_pos = pos_i - pos_j

        # Agent j's velocity
        v_j, omega_j = control_j
        vel_j = np.array([v_j * np.cos(theta_j), v_j * np.sin(theta_j)])

        # Lf_B = (p_i - p_j)^T * (0 - vel_j) = -rel_pos^T * vel_j
        Lf_B = -np.dot(rel_pos, vel_j)

        # Lg_B for agent i's control [v_i, omega_i]
        # Agent i's velocity direction
        Lg1 = rel_pos[0] * np.cos(theta_i) + rel_pos[1] * np.sin(theta_i)
        Lg2 = 0.0  # omega doesn't directly affect position

        return Lf_B, np.array([Lg1, Lg2])


class ConnectivityBarrier:
    """
    Barrier function for maintaining connectivity.
    B_ij,p = -0.5 * ||p_i - p_j||^2 + 0.5 * R_s^2
    """

    def __init__(self, sensing_radius: float):
        """
        Args:
            sensing_radius: Communication/sensing radius R_s
        """
        self.sensing_radius = sensing_radius

    def evaluate(self, pos_i: np.ndarray, pos_j: np.ndarray) -> float:
        """Evaluate barrier function."""
        dist_sq = np.linalg.norm(pos_i - pos_j) ** 2
        return -0.5 * dist_sq + 0.5 * self.sensing_radius ** 2

    def lie_derivative(self, state_i: np.ndarray, state_j: np.ndarray,
                      control_j: np.ndarray) -> Tuple[float, np.ndarray]:
        """
        Compute Lie derivatives.

        Args:
            state_i: [x_i, y_i, theta_i]
            state_j: [x_j, y_j, theta_j]
            control_j: [v_j, omega_j]

        Returns:
            Lf_B: Lie derivative along drift
            Lg_B: Lie derivative along control
        """
        pos_i = state_i[:2]
        pos_j = state_j[:2]
        theta_i = state_i[2]
        theta_j = state_j[2]

        # Relative position
        rel_pos = pos_i - pos_j

        # Agent j's velocity
        v_j, omega_j = control_j
        vel_j = np.array([v_j * np.cos(theta_j), v_j * np.sin(theta_j)])

        # Lf_B = -(p_i - p_j)^T * (0 - vel_j) = rel_pos^T * vel_j
        Lf_B = np.dot(rel_pos, vel_j)

        # Lg_B for agent i's control
        Lg1 = -(rel_pos[0] * np.cos(theta_i) + rel_pos[1] * np.sin(theta_i))
        Lg2 = 0.0

        return Lf_B, np.array([Lg1, Lg2])


class LyapunovPosition:
    """
    Control Lyapunov Function for goal reaching (position).
    V_p(p_i) = 0.5 * ||p_i - p_g||^2
    """

    def __init__(self):
        pass

    def evaluate(self, pos: np.ndarray, goal: np.ndarray) -> float:
        """Evaluate Lyapunov function."""
        return 0.5 * np.linalg.norm(pos - goal) ** 2

    def lie_derivative(self, state: np.ndarray, goal: np.ndarray) -> Tuple[float, np.ndarray]:
        """
        Compute Lie derivatives.

        Args:
            state: [x, y, theta]
            goal: [x_g, y_g]

        Returns:
            Lf_V: Lie derivative along drift
            Lg_V: Lie derivative along control
        """
        pos = state[:2]
        theta = state[2]

        # Gradient w.r.t. position
        grad_p = pos - goal

        # Lf_V = 0 (no drift)
        Lf_V = 0.0

        # Lg_V
        Lg1 = grad_p[0] * np.cos(theta) + grad_p[1] * np.sin(theta)
        Lg2 = 0.0

        return Lf_V, np.array([Lg1, Lg2])


class LyapunovAngle:
    """
    Control Lyapunov Function for goal orientation.
    V_θ(θ_i) = 0.5 * |θ_i - θ_g|^2 - θ_g_threshold^2
    """

    def __init__(self, theta_threshold: float = 0.1):
        """
        Args:
            theta_threshold: Acceptable orientation error
        """
        self.theta_threshold = theta_threshold

    def evaluate(self, theta: float, theta_goal: float) -> float:
        """Evaluate Lyapunov function."""
        angle_diff = self._angle_difference(theta, theta_goal)
        return 0.5 * angle_diff ** 2 - self.theta_threshold ** 2

    def lie_derivative(self, state: np.ndarray, theta_goal: float) -> Tuple[float, np.ndarray]:
        """
        Compute Lie derivatives.

        Args:
            state: [x, y, theta]
            theta_goal: Desired orientation

        Returns:
            Lf_V: Lie derivative along drift
            Lg_V: Lie derivative along control
        """
        theta = state[2]
        angle_diff = self._angle_difference(theta, theta_goal)

        # Lf_V = 0
        Lf_V = 0.0

        # Lg_V
        Lg1 = 0.0
        Lg2 = angle_diff

        return Lf_V, np.array([Lg1, Lg2])

    @staticmethod
    def _angle_difference(angle1: float, angle2: float) -> float:
        """Compute signed angle difference."""
        diff = angle1 - angle2
        return np.arctan2(np.sin(diff), np.cos(diff))
