"""
CBF-QP based low-level controller.
"""

import numpy as np
import cvxpy as cp
from typing import List, Dict, Set, Tuple, Optional
from src.core.agent import Agent
from src.core.obstacle import Obstacle
from src.control.barrier_functions import (
    ObstacleBarrierPosition, ObstacleBarrierAngle,
    AgentBarrierPosition, ConnectivityBarrier,
    LyapunovPosition, LyapunovAngle
)


class CBFQPController:
    """
    Low-level CBF-QP controller for multi-agent system.
    Solves QP (3) from the paper.
    """

    def __init__(self, sensing_radius: float = 1.7, safe_distance: float = 0.1,
                 r_obs: float = 0.1, v_max: float = 0.5, omega_max: float = 1.0):
        """
        Initialize CBF-QP controller.

        Args:
            sensing_radius: Communication radius R_s
            safe_distance: Minimum safe distance d_m
            r_obs: Safe distance from obstacles
            v_max: Maximum linear velocity
            omega_max: Maximum angular velocity
        """
        self.sensing_radius = sensing_radius
        self.safe_distance = safe_distance
        self.r_obs = r_obs
        self.v_max = v_max
        self.omega_max = omega_max

        # Lyapunov and barrier function objects
        self.lyap_pos = LyapunovPosition()
        self.lyap_angle = LyapunovAngle(theta_threshold=0.15)

        # CBF/CLF rate parameters
        self.lambda_p = 1.0  # CLF rate for position
        self.lambda_theta = 1.0  # CLF rate for angle

    def compute_control(self, agent: Agent, neighbors: List[Agent],
                       obstacles: List[Obstacle], neighbor_controls: Dict[int, np.ndarray],
                       connectivity_neighbors: Set[int]) -> np.ndarray:
        """
        Compute control input for an agent using CBF-QP.

        Args:
            agent: The agent to control
            neighbors: List of neighboring agents
            obstacles: List of obstacles in sensing range
            neighbor_controls: Dictionary mapping neighbor ID to their control inputs
            connectivity_neighbors: Set of neighbor IDs to maintain connectivity with

        Returns:
            control: [v, omega] control input
        """
        # Decision variables
        v = cp.Variable()
        omega = cp.Variable()
        u = cp.vstack([v, omega])

        # Alpha variables for CBFs (slack variables)
        alpha_obs_p = {}
        alpha_obs_theta = {}
        alpha_agent_p = {}
        alpha_conn_p = {}

        # Slack variables for CLF
        delta_p = cp.Variable()
        delta_theta = cp.Variable()

        # Cost function weights
        w_v = 1.0  # Weight on linear velocity
        w_omega = 1.0  # Weight on angular velocity
        w_alpha = 1.0  # Penalty on negative alpha (encourages positive alpha)
        w_delta = 100.0  # Penalty on positive delta (encourages negative delta)
        w_reg = 0.01  # Regularization to prevent unboundedness

        # Objective: minimize ||u - u_prev||^2 - sum(alphas) + delta_p + delta_theta
        # Split into individual terms to avoid dimension issues
        u_prev = agent.prev_control
        cost = w_v * cp.square(v - u_prev[0]) + w_omega * cp.square(omega - u_prev[1])
        cost += w_delta * (delta_p + delta_theta)
        # Add regularization to keep problem bounded
        cost += w_reg * (cp.square(delta_p) + cp.square(delta_theta))

        constraints = []

        # Control input constraints
        constraints.append(v >= -self.v_max)
        constraints.append(v <= self.v_max)
        constraints.append(omega >= -self.omega_max)
        constraints.append(omega <= self.omega_max)

        # ===== Obstacle CBF constraints =====
        theta_threshold = 0.3
        for obs_idx, obstacle in enumerate(obstacles):
            dist, closest_point = obstacle.distance_to_point(agent.position)

            if dist < self.sensing_radius:
                # Position barrier
                obs_barrier_p = ObstacleBarrierPosition(dist, closest_point, self.r_obs)
                B_obs_p = obs_barrier_p.evaluate(agent.position)

                if B_obs_p < 1.0:  # Only apply constraint if close to obstacle
                    Lf_B = obs_barrier_p.lie_derivative_f(agent.state)
                    Lg_B = obs_barrier_p.lie_derivative_g(agent.state)

                    alpha_obs_p[obs_idx] = cp.Variable()
                    # Bound alpha to reasonable range [0.1, 10.0]
                    constraints.append(alpha_obs_p[obs_idx] >= 0.1)
                    constraints.append(alpha_obs_p[obs_idx] <= 10.0)
                    constraints.append(Lf_B + Lg_B @ u >= -alpha_obs_p[obs_idx] * B_obs_p)
                    cost += -w_alpha * alpha_obs_p[obs_idx]

                # Angle barrier (steering away from obstacle)
                # Angle to obstacle
                rel_vec = closest_point - agent.position
                theta_obs = np.arctan2(rel_vec[1], rel_vec[0])

                obs_barrier_theta = ObstacleBarrierAngle(agent.theta, theta_obs, theta_threshold)
                B_obs_theta = obs_barrier_theta.evaluate(agent.theta)

                if B_obs_theta < 0.5:
                    Lf_B = obs_barrier_theta.lie_derivative_f(agent.state)
                    Lg_B = obs_barrier_theta.lie_derivative_g(agent.state)

                    alpha_obs_theta[obs_idx] = cp.Variable()
                    constraints.append(alpha_obs_theta[obs_idx] >= 0.1)
                    constraints.append(alpha_obs_theta[obs_idx] <= 10.0)
                    constraints.append(Lf_B + Lg_B @ u >= -alpha_obs_theta[obs_idx] * B_obs_theta)
                    cost += -w_alpha * alpha_obs_theta[obs_idx]

        # ===== Agent collision avoidance CBF constraints =====
        agent_barrier = AgentBarrierPosition(self.safe_distance)
        for neighbor in neighbors:
            if neighbor.id == agent.id:
                continue

            B_agent = agent_barrier.evaluate(agent.position, neighbor.position)

            if B_agent < 2.0:  # Only apply if agents are reasonably close
                neighbor_control = neighbor_controls.get(neighbor.id, np.zeros(2))
                Lf_B, Lg_B = agent_barrier.lie_derivative(agent.state, neighbor.state, neighbor_control)

                alpha_agent_p[neighbor.id] = cp.Variable()
                constraints.append(alpha_agent_p[neighbor.id] >= 0.1)
                constraints.append(alpha_agent_p[neighbor.id] <= 10.0)
                constraints.append(Lf_B + Lg_B @ u >= -alpha_agent_p[neighbor.id] * B_agent)
                cost += -w_alpha * alpha_agent_p[neighbor.id]

        # ===== Connectivity CBF constraints =====
        conn_barrier = ConnectivityBarrier(self.sensing_radius)
        for neighbor_id in connectivity_neighbors:
            # Find the neighbor
            neighbor = next((n for n in neighbors if n.id == neighbor_id), None)
            if neighbor is None:
                continue

            B_conn = conn_barrier.evaluate(agent.position, neighbor.position)

            if B_conn > -1.0:  # Only apply if within sensing radius
                neighbor_control = neighbor_controls.get(neighbor.id, np.zeros(2))
                Lf_B, Lg_B = conn_barrier.lie_derivative(agent.state, neighbor.state, neighbor_control)

                alpha_conn_p[neighbor_id] = cp.Variable()
                constraints.append(alpha_conn_p[neighbor_id] >= 0.1)
                constraints.append(alpha_conn_p[neighbor_id] <= 10.0)
                constraints.append(Lf_B + Lg_B @ u >= -alpha_conn_p[neighbor_id] * B_conn)
                cost += -w_alpha * alpha_conn_p[neighbor_id]

        # ===== CLF constraints for goal reaching =====
        # Use temporary goal if set, otherwise use actual goal
        goal = agent.goal_temp

        # Position CLF
        V_p = self.lyap_pos.evaluate(agent.position, goal)
        if V_p > 0.01:  # Only apply if not at goal
            Lf_V, Lg_V = self.lyap_pos.lie_derivative(agent.state, goal)
            constraints.append(Lf_V + Lg_V @ u <= -self.lambda_p * V_p + delta_p)

        # Angle CLF (point towards goal)
        goal_angle = np.arctan2(goal[1] - agent.position[1], goal[0] - agent.position[0])
        V_theta = self.lyap_angle.evaluate(agent.theta, goal_angle)
        if V_theta > 0.01:
            Lf_V, Lg_V = self.lyap_angle.lie_derivative(agent.state, goal_angle)
            constraints.append(Lf_V + Lg_V @ u <= -self.lambda_theta * V_theta + delta_theta)

        # ===== Solve QP =====
        problem = cp.Problem(cp.Minimize(cost), constraints)

        try:
            problem.solve(solver=cp.OSQP, verbose=False, eps_abs=1e-4, eps_rel=1e-4, max_iter=10000)

            if problem.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                control = np.array([v.value, omega.value])
                # Clip to bounds as safety measure
                control[0] = np.clip(control[0], -self.v_max, self.v_max)
                control[1] = np.clip(control[1], -self.omega_max, self.omega_max)
                return control
            else:
                # If infeasible or unbounded, use safe nominal control
                return self._compute_safe_nominal_control(agent, obstacles, neighbors)

        except Exception as e:
            print(f"Agent {agent.id}: QP solve failed with error {e}, using safe nominal control")
            return self._compute_safe_nominal_control(agent, obstacles, neighbors)

    def _compute_safe_nominal_control(self, agent: Agent, obstacles: List[Obstacle],
                                      neighbors: List[Agent]) -> np.ndarray:
        """
        Compute safe nominal control with obstacle and agent avoidance.
        Used as fallback when QP fails.
        """
        goal = agent.goal_temp
        direction = goal - agent.position
        dist_to_goal = np.linalg.norm(direction)

        if dist_to_goal < 0.01:
            return np.zeros(2)

        # Desired direction toward goal
        direction = direction / dist_to_goal

        # Check for obstacles and apply repulsive force
        repulsive_force = np.zeros(2)
        for obstacle in obstacles:
            dist, closest_point = obstacle.distance_to_point(agent.position)
            if dist < self.r_obs + 0.8:  # Larger danger zone (increased from 0.5 to 0.8)
                # Repulsive vector away from obstacle
                rep_vec = agent.position - closest_point
                rep_dist = np.linalg.norm(rep_vec)
                if rep_dist > 0.01:
                    rep_vec = rep_vec / rep_dist
                    # Stronger repulsion when closer
                    strength = max(0, (self.r_obs + 0.8 - dist) / (self.r_obs + 0.8))
                    # Increased repulsion strength with power function
                    repulsive_force += 2.0 * (strength ** 2) * rep_vec

        # Check for nearby agents and add repulsion
        for neighbor in neighbors:
            dist = agent.distance_to_agent(neighbor)
            if dist < 2 * self.safe_distance + 0.3:  # Danger zone
                rep_vec = agent.position - neighbor.position
                rep_dist = np.linalg.norm(rep_vec)
                if rep_dist > 0.01:
                    rep_vec = rep_vec / rep_dist
                    strength = max(0, (2 * self.safe_distance + 0.3 - dist) / (2 * self.safe_distance + 0.3))
                    repulsive_force += strength * 0.5 * rep_vec

        # Combine attractive (goal) and repulsive (obstacles/agents) forces
        combined_direction = direction + repulsive_force
        combined_norm = np.linalg.norm(combined_direction)

        if combined_norm > 0.01:
            combined_direction = combined_direction / combined_norm
        else:
            # If completely blocked, try to move perpendicular
            combined_direction = np.array([-direction[1], direction[0]])

        # Compute desired heading
        desired_theta = np.arctan2(combined_direction[1], combined_direction[0])
        theta_error = np.arctan2(np.sin(desired_theta - agent.theta),
                                np.cos(desired_theta - agent.theta))

        # Slow down near obstacles or other agents
        speed_factor = 1.0
        min_obstacle_dist = min([obstacle.distance_to_point(agent.position)[0] for obstacle in obstacles], default=np.inf)
        if min_obstacle_dist < self.r_obs + 0.3:
            speed_factor = min(speed_factor, min_obstacle_dist / (self.r_obs + 0.3))

        for neighbor in neighbors:
            dist = agent.distance_to_agent(neighbor)
            if dist < 2 * self.safe_distance + 0.3:
                speed_factor = min(speed_factor, dist / (2 * self.safe_distance + 0.3))

        # Compute control with more aggressive speed
        v_nom = min(0.3 * speed_factor, dist_to_goal)  # Increased from 0.15 to 0.3
        omega_nom = np.clip(3.0 * theta_error, -self.omega_max, self.omega_max)  # Increased from 2.0 to 3.0

        return np.array([v_nom, omega_nom])

    def compute_control_without_connectivity(self, agent: Agent, neighbors: List[Agent],
                                            obstacles: List[Obstacle],
                                            neighbor_controls: Dict[int, np.ndarray],
                                            excluded_neighbor: Optional[int] = None) -> np.ndarray:
        """
        Compute control without connectivity constraint for a specific neighbor.
        Used for bidding mechanism.

        Args:
            agent: The agent
            neighbors: List of neighbors
            obstacles: List of obstacles
            neighbor_controls: Neighbor control inputs
            excluded_neighbor: Neighbor ID to exclude from connectivity

        Returns:
            control: Computed control input
        """
        # Get current connectivity neighbors
        current_connectivity = agent.neighbors.copy()

        # Remove excluded neighbor
        if excluded_neighbor is not None and excluded_neighbor in current_connectivity:
            current_connectivity.remove(excluded_neighbor)

        return self.compute_control(agent, neighbors, obstacles, neighbor_controls, current_connectivity)
