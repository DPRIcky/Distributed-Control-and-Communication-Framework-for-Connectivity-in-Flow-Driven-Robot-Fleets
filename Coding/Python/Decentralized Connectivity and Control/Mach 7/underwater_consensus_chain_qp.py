"""Decentralized underwater consensus pruning with CLF-CBF QP control.

This version implements proper CLF-CBF QP optimization for each robot instead of
gradient-based control. Each robot solves its own QP locally in a decentralized manner.

Features:
    * Robots drift in underwater flow field with individual dynamics
    * CLF-CBF QP: Each robot solves a quadratic program with:
      - Objective: Minimize control effort ||u||^2
      - CLF constraint: Lyapunov decrease for goal-seeking
      - CBF constraints: Safety (collision avoidance) and connectivity
    * Distributed consensus-based edge pruning
    * Multi-layer robustness for dynamic topologies

Requirements:
    pip install cvxpy numpy matplotlib

Usage:
    python underwater_consensus_chain_qp.py gui --robots 6 --steps 100
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Set, Tuple

import cvxpy as cp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
import numpy as np

from distributed_consensus_pruning import ConsensusPruningSimulation

Edge = Tuple[int, int]


@dataclass
class FlowField:
    """Simple 2-D current: constant drift plus gentle sinusoidal swirl."""

    base_vector: np.ndarray = field(
        default_factory=lambda: np.array([0.20, 0.05], dtype=float)
    )
    swirl_amplitude: float = 0.10
    swirl_scale: float = 5.5

    def velocity(self, position: np.ndarray, time: float) -> np.ndarray:
        swirl = self.swirl_amplitude * np.array(
            [
                math.sin((position[1] + time) / self.swirl_scale),
                math.cos((position[0] + 0.3 * time) / self.swirl_scale),
            ]
        )
        return self.base_vector + swirl


@dataclass
class ChainRobot:
    """Single-integrator robot advected by the flow with mild diffusion."""

    robot_id: int
    position: np.ndarray
    rng: np.random.Generator
    diffusion: float = 0.008
    mobility: float = 0.45
    removed_edges: List[Edge] = field(default_factory=list)

    mass: float = field(default_factory=lambda: 2.0 + np.random.normal(0.0, 0.2))
    drag_coefficient: float = field(default_factory=lambda: 0.8 + np.random.normal(0.0, 0.1))
    cross_sectional_area: float = field(default_factory=lambda: 0.01 + np.random.normal(0.0, 0.002))

    velocity: np.ndarray = field(default_factory=lambda: np.zeros(2))
    parent: Optional[int] = None

    def __post_init__(self):
        """Initialize physical properties."""
        self.volume = self.mass / 1800.0

    def step(self, flow: FlowField, dt: float, time: float, bounds: np.ndarray, control_force: Optional[np.ndarray] = None) -> None:
        """Update position with flow, diffusion, and control input."""
        flow_velocity = flow.velocity(self.position, time)

        if control_force is not None:
            self.velocity += control_force * dt

        agitation = self.rng.normal(0.0, self.diffusion, size=2)
        self.velocity += agitation * self.mobility

        self.velocity *= 0.85

        self.position = self.position + dt * (flow_velocity + self.velocity)
        self.position = np.clip(self.position, [0.0, 0.0], bounds)
        self.removed_edges.clear()


class QPUnderwaterSimulation:
    """Consensus pruning with CLF-CBF QP control for each robot."""

    def __init__(
        self,
        num_robots: int = 10,
        communication_radius: float = 3.0,
        workspace: Tuple[float, float] = (10.0, 10.0),
        dt: float = 0.05,
        seed: Optional[int] = None,
        goal_position: Optional[np.ndarray] = None,
        safety_distance: float = 0.35,
        # QP-specific parameters
        clf_alpha: float = 0.5,  # CLF decay rate
        cbf_gamma_safety: float = 2.0,  # CBF safety class-K function param
        cbf_gamma_connectivity: float = 1.5,  # CBF connectivity class-K function param
        max_control: float = 1.0,  # Maximum control input
        # Hybrid consensus parameters
        stability_threshold: int = 5,
        rolling_window_size: int = 10,
        consensus_rounds_per_step: int = 3,
        verbose: bool = True,
    ):
        self.num_robots = num_robots
        self.communication_radius = communication_radius
        self.workspace = np.array(workspace, dtype=float)
        self.dt = dt
        self.safety_distance = safety_distance

        # QP parameters
        self.clf_alpha = clf_alpha
        self.cbf_gamma_safety = cbf_gamma_safety
        self.cbf_gamma_connectivity = cbf_gamma_connectivity
        self.max_control = max_control

        # Hybrid consensus parameters
        self.stability_threshold = stability_threshold
        self.rolling_window_size = rolling_window_size
        self.consensus_rounds_per_step = consensus_rounds_per_step
        self.verbose = verbose

        self.time = 0.0
        self.rng = np.random.default_rng(seed)
        self.flow = FlowField()

        if goal_position is None:
            self.goal_position = self.rng.random(2) * self.workspace
        else:
            self.goal_position = np.array(goal_position, dtype=float)

        # Initialize robots in a compact connected cluster
        self.robots: List[ChainRobot] = []
        # Choose a random center point in the workspace, avoiding edges
        cluster_center = self.rng.random(2) * (self.workspace - 4.0) + 2.0
        # Compact cluster radius (50% of communication radius for tighter formation)
        cluster_radius = self.communication_radius * 0.5

        for i in range(num_robots):
            # Generate position within cluster radius from center
            angle = self.rng.random() * 2 * np.pi
            radius = self.rng.random() * cluster_radius
            offset = np.array([radius * np.cos(angle), radius * np.sin(angle)])
            pos = cluster_center + offset
            # Ensure position is within workspace bounds
            pos = np.clip(pos, [0.5, 0.5], self.workspace - 0.5)
            robot = ChainRobot(robot_id=i, position=pos, rng=self.rng)
            self.robots.append(robot)

        # Goal-reaching parameters
        self.goal_tolerance = 0.5  # Distance threshold to consider robot at goal
        self.goal_reached = False

        # Connectivity tracking
        self.neighbor_graph: Dict[int, Set[int]] = {i: set() for i in range(num_robots)}
        self.pruned_edges: Set[Edge] = set()

        # Hybrid consensus state
        self.topology_stable_rounds = 0
        self.topology_history: List[Dict] = []
        self.consensus_active = False
        self.consensus_candidate: Optional[Edge] = None
        self.consensus_round = 0
        self.total_consensus_rounds_needed = 50
        self.failed_candidates: Set[Edge] = set()

        self.consensus = ConsensusPruningSimulation(
            num_nodes=num_robots, verbose=False
        )

        self._update_neighbor_graph()
        self._build_connectivity_tree()

    # ------------------------------------------------------------------ #
    # Graph utilities
    # ------------------------------------------------------------------ #
    def _edge_key(self, a: int, b: int) -> Edge:
        return (a, b) if a < b else (b, a)

    def check_goal_reached(self) -> bool:
        """Check if all robots have reached within tolerance of the goal."""
        for robot in self.robots:
            dist_to_goal = np.linalg.norm(robot.position - self.goal_position)
            if dist_to_goal > self.goal_tolerance:
                return False
        return True

    def _update_neighbor_graph(self) -> None:
        """Update neighbor graph based on current positions."""
        for i in range(self.num_robots):
            self.neighbor_graph[i].clear()

        for i in range(self.num_robots):
            for j in range(i + 1, self.num_robots):
                dist = np.linalg.norm(self.robots[i].position - self.robots[j].position)
                if dist <= self.communication_radius:
                    edge = self._edge_key(i, j)
                    if edge not in self.pruned_edges:
                        self.neighbor_graph[i].add(j)
                        self.neighbor_graph[j].add(i)

    def current_edges(self) -> Set[Edge]:
        """Return current communication edges."""
        edges = set()
        for i, neighbors in self.neighbor_graph.items():
            for j in neighbors:
                if i < j:
                    edges.add((i, j))
        return edges

    def _gather_edge_lengths(self) -> Dict[Edge, float]:
        """Gather edge lengths for consensus protocol."""
        edge_lengths = {}
        for i in range(self.num_robots):
            for j in self.neighbor_graph[i]:
                if i < j:
                    edge = (i, j)
                    dist = np.linalg.norm(self.robots[i].position - self.robots[j].position)
                    edge_lengths[edge] = dist
        return edge_lengths

    # ------------------------------------------------------------------ #
    # Connectivity tree and CLF-CBF QP control
    # ------------------------------------------------------------------ #
    def _build_connectivity_tree(self) -> None:
        """Build connectivity tree rooted at robot 0 using BFS."""
        for robot in self.robots:
            robot.parent = None

        visited = {0}
        queue = [0]

        while queue:
            node = queue.pop(0)
            for neighbor in self.neighbor_graph[node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    self.robots[neighbor].parent = node
                    queue.append(neighbor)

    def _compute_clf_cbf_qp_control(self, robot_id: int) -> np.ndarray:
        """Compute CLF-CBF QP control for a single robot.

        Solves the following QP:
            minimize    ||u||^2
            subject to  Lf V + Lg V · u <= -alpha * V              (CLF constraint)
                        Lf h_safety + Lg h_safety · u >= -gamma(h)  (CBF safety)
                        Lf h_conn + Lg h_conn · u >= -gamma(h)      (CBF connectivity)
                        ||u|| <= u_max                               (Control limits)
        """
        robot = self.robots[robot_id]

        # Define control variable (2D) and slack variable for CLF relaxation
        u = cp.Variable(2)
        delta = cp.Variable(1, nonneg=True)  # Slack variable for CLF constraint

        # Objective: Minimize control effort + penalty for CLF violation
        # High penalty (p=1000) encourages satisfying CLF when possible
        p_clf = 1000.0  # CLF slack penalty weight
        objective = cp.Minimize(cp.sum_squares(u) + p_clf * delta)

        constraints = []

        # ============================================================
        # CLF: Control Lyapunov Function for goal-seeking (with slack)
        # ============================================================
        # V(x) = ||x - x_goal||^2
        # Lf V = 0 (no drift in our model, only flow which we treat as disturbance)
        # Lg V = 2 * (x - x_goal)^T

        error_to_goal = robot.position - self.goal_position
        V = np.dot(error_to_goal, error_to_goal)  # Lyapunov function value

        # Only apply CLF if robot is in tree or is root
        if robot.parent is not None or robot_id == 0:
            if V > 0.01:  # Only if not at goal
                Lg_V = 2.0 * error_to_goal  # Gradient of V
                # Relaxed CLF constraint: Lg V · u <= -alpha * V + delta
                # (We want V_dot <= -alpha * V, but allow violation via delta when needed for safety)
                clf_constraint = Lg_V @ u <= -self.clf_alpha * V + delta
                constraints.append(clf_constraint)

        # ============================================================
        # CBF Safety: Collision avoidance with other robots
        # ============================================================
        # h_safety(x_i, x_j) = ||x_i - x_j||^2 - d_safe^2
        # We want h >= 0 (stay safe)
        # Lf h = 0 (no drift)
        # Lg h = 2 * (x_i - x_j)^T
        # CBF constraint: Lg h · u >= -gamma * h

        for other_id, other_robot in enumerate(self.robots):
            if other_id == robot_id:
                continue

            diff = robot.position - other_robot.position
            dist_sq = np.dot(diff, diff)

            # Barrier function: h = dist^2 - d_safe^2
            h_safety = dist_sq - self.safety_distance**2

            # Only enforce if getting close to unsafe region
            if h_safety < 2.0 * self.safety_distance**2:
                if dist_sq < 1e-6:
                    # Handle collision case - add random perturbation
                    diff = self.rng.normal(0.0, 0.1, size=2)
                    dist_sq = np.dot(diff, diff)
                    h_safety = dist_sq - self.safety_distance**2

                # Lg h = 2 * (x_i - x_j)
                Lg_h_safety = 2.0 * diff

                # Class-K function: gamma(h) = cbf_gamma_safety * h
                gamma_h = self.cbf_gamma_safety * h_safety

                # CBF constraint: Lg h · u >= -gamma(h)
                cbf_safety_constraint = Lg_h_safety @ u >= -gamma_h
                constraints.append(cbf_safety_constraint)

        # ============================================================
        # CBF Connectivity: Maintain connection to parent
        # ============================================================
        # h_conn(x_i, x_parent) = d_max^2 - ||x_i - x_parent||^2
        # We want h >= 0 (stay connected)
        # Lf h = 0
        # Lg h = -2 * (x_i - x_parent)^T
        # CBF constraint: Lg h · u >= -gamma * h

        if robot.parent is not None:
            parent_robot = self.robots[robot.parent]
            diff_to_parent = robot.position - parent_robot.position
            dist_to_parent_sq = np.dot(diff_to_parent, diff_to_parent)

            # Maximum allowed distance (95% of communication radius)
            d_max = self.communication_radius * 0.95
            d_max_sq = d_max**2

            # Barrier function: h = d_max^2 - dist^2
            h_conn = d_max_sq - dist_to_parent_sq

            # Only enforce if getting close to max distance
            if h_conn < 0.5 * d_max_sq:
                if dist_to_parent_sq > 1e-6:
                    # Lg h = -2 * (x_i - x_parent)
                    Lg_h_conn = -2.0 * diff_to_parent

                    # Class-K function: gamma(h) = cbf_gamma_connectivity * h
                    gamma_h = self.cbf_gamma_connectivity * h_conn

                    # CBF constraint: Lg h · u >= -gamma(h)
                    cbf_conn_constraint = Lg_h_conn @ u >= -gamma_h
                    constraints.append(cbf_conn_constraint)

        # ============================================================
        # Control input bounds
        # ============================================================
        # Box constraints (QP-compatible): -u_max <= u_i <= u_max for each dimension
        # This is more conservative than ||u|| <= u_max but compatible with OSQP
        constraints.append(u[0] >= -self.max_control)
        constraints.append(u[0] <= self.max_control)
        constraints.append(u[1] >= -self.max_control)
        constraints.append(u[1] <= self.max_control)

        # ============================================================
        # Solve QP
        # ============================================================
        prob = cp.Problem(objective, constraints)

        try:
            prob.solve(solver=cp.OSQP, verbose=False, warm_start=True)

            if prob.status == cp.OPTIMAL or prob.status == cp.OPTIMAL_INACCURATE:
                control = u.value
                if control is None:
                    control = np.zeros(2)
            else:
                # QP infeasible - use relaxed solution or zero control
                if self.verbose and robot_id == 0:
                    print(f"[QP] Robot {robot_id}: {prob.status}, using zero control")
                control = np.zeros(2)
        except Exception as e:
            if self.verbose and robot_id == 0:
                print(f"[QP] Robot {robot_id} exception: {e}, using zero control")
            control = np.zeros(2)

        return control

    # ------------------------------------------------------------------ #
    # Consensus pruning helper methods (same as hybrid version)
    # ------------------------------------------------------------------ #
    def _has_alternative_path(self, edge: Edge, edge_set: Set[Edge]) -> bool:
        """Check if there's an alternative path between edge endpoints using BFS."""
        a, b = edge
        remaining_edges = edge_set - {edge}

        adj: Dict[int, Set[int]] = {i: set() for i in range(self.num_robots)}
        for u, v in remaining_edges:
            adj[u].add(v)
            adj[v].add(u)

        visited = {a}
        queue = [a]

        while queue:
            node = queue.pop(0)
            if node == b:
                return True
            for neighbor in adj[node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)

        return False

    def _check_graph_connectivity(self, edge_set: Set[Edge]) -> bool:
        """Check if the graph is fully connected using BFS from robot 0."""
        if not edge_set:
            return False

        adj: Dict[int, Set[int]] = {i: set() for i in range(self.num_robots)}
        for u, v in edge_set:
            adj[u].add(v)
            adj[v].add(u)

        visited = {0}
        queue = [0]

        while queue:
            node = queue.pop(0)
            for neighbor in adj[node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)

        return len(visited) == self.num_robots

    def _find_persistent_redundant_edge(self, debug: bool = False) -> Optional[Edge]:
        """Find edge that's redundant in current topology and recent history."""
        if len(self.topology_history) < self.rolling_window_size:
            return None

        current_snap = self.topology_history[-1]
        current_edges = current_snap['edges']
        current_lengths = current_snap['lengths']

        if not current_edges:
            return None

        candidate_edges = current_edges - self.failed_candidates

        if not candidate_edges:
            if debug:
                print(f"[DEBUG] No candidate edges (all in blacklist)")
            return None

        candidates = []
        rejected = {}

        for edge in candidate_edges:
            has_alt_path = self._has_alternative_path(edge, current_edges)

            if not has_alt_path:
                if debug:
                    rejected[edge] = "No alternative path"
                continue

            graph_stays_connected = self._check_graph_connectivity(current_edges - {edge})

            if not graph_stays_connected:
                if debug:
                    rejected[edge] = "Graph disconnects"
                continue

            redundant_count = 0
            for snap in self.topology_history[-5:]:
                if edge in snap['edges']:
                    if self._has_alternative_path(edge, snap['edges']):
                        if self._check_graph_connectivity(snap['edges'] - {edge}):
                            redundant_count += 1

            if redundant_count < 3:
                if debug:
                    rejected[edge] = f"Unstable ({redundant_count}/5 snapshots)"
                continue

            edge_length = current_lengths.get(edge, 0.0)
            candidates.append((edge, edge_length))

        if debug and rejected:
            print(f"\n[DEBUG] Rejected {len(rejected)} edges:")
            for edge, reason in list(rejected.items())[:8]:
                print(f"  {edge}: {reason}")

        if not candidates:
            return None

        candidates.sort(key=lambda x: -x[1])

        if debug:
            print(f"[DEBUG] Found {len(candidates)} redundant candidates")
            print(f"[DEBUG] Selected: {candidates[0][0]} (length={candidates[0][1]:.2f})")

        return candidates[0][0]

    # ------------------------------------------------------------------ #
    # Main simulation step
    # ------------------------------------------------------------------ #
    def step(self) -> Dict[str, Optional[object]]:
        """Execute one simulation step with CLF-CBF QP control and consensus pruning."""
        # STEP 0: Check if goal is reached
        if self.check_goal_reached():
            if not self.goal_reached:
                self.goal_reached = True
                if self.verbose:
                    print(f"\n[t={self.time:5.2f}] *** GOAL REACHED! All robots within {self.goal_tolerance}m of goal ***")
            return {"decision": "goal_reached"}

        # STEP 1: Update connectivity tree
        self._build_connectivity_tree()

        # STEP 2: Compute and apply CLF-CBF QP control for each robot (DECENTRALIZED)
        for robot_id, robot in enumerate(self.robots):
            control_force = self._compute_clf_cbf_qp_control(robot_id)
            robot.step(self.flow, self.dt, self.time, self.workspace, control_force)

        self.time += self.dt

        # STEP 3: Update neighbor graph
        old_edges = set(self.current_edges())
        self._update_neighbor_graph()
        new_edges = set(self.current_edges())

        # STEP 4: Detect topology changes
        broken = old_edges - new_edges
        formed = new_edges - old_edges

        if broken or formed:
            if self.consensus_active and self.verbose:
                print(f"[t={self.time:5.2f}] Topology changed! Broken: {broken}, Formed: {formed}")
                print(f"  Aborting consensus (was at round {self.consensus_round})")

            self.consensus_active = False
            self.consensus_candidate = None
            self.consensus_round = 0
            self.topology_stable_rounds = 0
            return {
                "decision": "topology_changed",
                "broken_edges": broken,
                "formed_edges": formed
            }

        # STEP 5: Increment stability counter
        self.topology_stable_rounds += 1

        # STEP 6: Add topology snapshot
        self.topology_history.append({
            'edges': new_edges.copy(),
            'lengths': self._gather_edge_lengths(),
            'time': self.time
        })
        if len(self.topology_history) > self.rolling_window_size:
            self.topology_history.pop(0)

        # STEP 7: Check if enough history
        if self.topology_stable_rounds < self.stability_threshold:
            return {
                "decision": "waiting_for_stability",
                "stable_rounds": self.topology_stable_rounds,
                "needed": self.stability_threshold
            }

        if len(self.topology_history) < self.rolling_window_size:
            return {
                "decision": "building_history",
                "history_size": len(self.topology_history),
                "needed": self.rolling_window_size
            }

        # STEP 8: Find persistently redundant edge
        if not self.consensus_active:
            candidate = self._find_persistent_redundant_edge(debug=True)

            if candidate is None:
                return {"decision": "no_persistent_redundant_edges"}

            self.consensus_active = True
            self.consensus_candidate = candidate
            self.consensus_round = 0
            if self.verbose:
                print(f"[t={self.time:5.2f}] Starting consensus on edge {candidate}")

        # STEP 9: Run distributed consensus
        positions = np.array([r.position for r in self.robots], dtype=float)
        edge_lengths = self._gather_edge_lengths()

        if self.consensus_round == 0:
            self.consensus.load_external_state(positions, edge_lengths)
        else:
            self.consensus.positions = positions
            for edge in list(self.consensus.edge_lengths.keys()):
                if edge in edge_lengths:
                    self.consensus.edge_lengths[edge] = edge_lengths[edge]

        removed_edge = None
        for _ in range(self.consensus_rounds_per_step):
            report = self.consensus.step()
            if report.get('decision') == 'prune':
                removed_edge = report.get('removed_edge')
                break

        self.consensus_round += self.consensus_rounds_per_step

        # STEP 10: Check if consensus reached
        if removed_edge == self.consensus_candidate:
            final_edges = set(self.current_edges())
            if final_edges == new_edges:
                edge_to_prune = self.consensus_candidate
                a, b = edge_to_prune

                if b in self.neighbor_graph[a]:
                    self.neighbor_graph[a].discard(b)
                if a in self.neighbor_graph[b]:
                    self.neighbor_graph[b].discard(a)
                self.pruned_edges.add(edge_to_prune)
                self.robots[a].removed_edges.append(edge_to_prune)
                self.robots[b].removed_edges.append((b, a))

                self.consensus_active = False
                self.consensus_candidate = None
                self.consensus_round = 0
                self.topology_stable_rounds = 0
                self.topology_history.clear()
                self.failed_candidates.clear()

                if self.verbose:
                    print(f"[t={self.time:5.2f}] PRUNED {edge_to_prune} | "
                          f"edges left={len(self.current_edges())} | "
                          f"Robot {a} and {b} gain freedom")

                return {
                    "decision": "prune",
                    "removed_edge": edge_to_prune,
                    "edges_remaining": len(self.current_edges())
                }
            else:
                if self.verbose:
                    print(f"[t={self.time:5.2f}] ABORT: Topology changed during consensus")
                self.consensus_active = False
                self.consensus_candidate = None
                self.consensus_round = 0
                self.topology_stable_rounds = 0
                return {
                    "decision": "abort_topology_changed",
                    "candidate": self.consensus_candidate
                }
        elif self.consensus_round >= self.total_consensus_rounds_needed:
            failed_edge = self.consensus_candidate

            if self.verbose:
                print(f"[t={self.time:5.2f}] NO CONSENSUS on {failed_edge} after {self.consensus_round} rounds")
                print(f"  Edge is NOT redundant, blacklisting and searching for other candidates")

            self.failed_candidates.add(failed_edge)

            self.consensus_active = False
            self.consensus_candidate = None
            self.consensus_round = 0

            return {
                "decision": "consensus_failed",
                "reason": "edge_not_redundant",
                "failed_candidate": failed_edge
            }

        return {
            "decision": "evaluating",
            "candidate": self.consensus_candidate,
            "consensus_round": self.consensus_round,
            "needed_rounds": self.total_consensus_rounds_needed,
            "removed_edge": removed_edge
        }

    def iterate(self, steps: int) -> Iterable[Dict[str, Optional[object]]]:
        for _ in range(steps):
            yield self.step()


# ------------------------------------------------------------------ #
# GUI Visualization
# ------------------------------------------------------------------ #
class QPVisualization:
    """Interactive GUI for CLF-CBF QP simulation."""

    def __init__(self, sim: QPUnderwaterSimulation):
        self.sim = sim
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_xlim(0, sim.workspace[0])
        self.ax.set_ylim(0, sim.workspace[1])
        self.ax.set_aspect('equal')
        self.ax.set_title('Decentralized CLF-CBF QP Control with Consensus Pruning', fontsize=14, weight='bold')

        # Flow field visualization
        grid_x = np.linspace(0.0, sim.workspace[0], 10)
        grid_y = np.linspace(0.0, sim.workspace[1], 10)
        X, Y = np.meshgrid(grid_x, grid_y)
        U = np.zeros_like(X)
        V = np.zeros_like(Y)

        for i in range(len(grid_x)):
            for j in range(len(grid_y)):
                pos = np.array([X[j, i], Y[j, i]])
                vel = sim.flow.velocity(pos, 0.0)
                U[j, i] = vel[0]
                V[j, i] = vel[1]

        self.quiver = self.ax.quiver(
            X, Y, U, V,
            color="#B0C4DE",
            alpha=0.4,
            width=0.0015,
            zorder=0,
        )

        # Robot visualization
        self.robot_scatter = self.ax.scatter([], [], s=150, c='blue', marker='o', zorder=5, edgecolors='black', linewidths=1.5)
        self.goal_scatter = self.ax.scatter([sim.goal_position[0]], [sim.goal_position[1]],
                                          s=200, c='gold', marker='*', zorder=4, edgecolors='black', linewidths=2)

        # Edge visualization
        self.active_lines = LineCollection([], colors="#9b59b6", linewidths=2.5, zorder=3)
        self.candidate_lines = LineCollection([], colors="#f39c12", linewidths=3.5, linestyles='dashdot', zorder=4)
        self.highlight_lines = LineCollection([], colors="#e74c3c", linewidths=4.5, zorder=5)

        self.ax.add_collection(self.active_lines)
        self.ax.add_collection(self.candidate_lines)
        self.ax.add_collection(self.highlight_lines)

        # Status text
        self.status_text = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            verticalalignment='top', fontsize=10,
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
            family='monospace'
        )

    def _update_plot(self, frame: int) -> tuple:
        """Update plot for animation."""
        report = self.sim.step()

        # Check if goal reached - stop animation
        if report.get('decision') == 'goal_reached':
            print(f"\n{'='*60}")
            print(f"SIMULATION COMPLETED - GOAL REACHED!")
            print(f"Time: {self.sim.time:.2f}s")
            print(f"Total edges pruned: {len(self.sim.pruned_edges)}")
            print(f"Final edges: {len(self.sim.current_edges())}")
            print(f"{'='*60}\n")
            plt.close(self.fig)
            return (self.robot_scatter, self.active_lines, self.candidate_lines,
                    self.highlight_lines, self.status_text)

        # Update robot positions
        positions = np.array([r.position for r in self.sim.robots])
        self.robot_scatter.set_offsets(positions)

        # Color robots by connectivity
        colors = []
        for robot in self.sim.robots:
            if robot.parent is None and robot.robot_id != 0:
                colors.append('orange')  # Isolated
            else:
                colors.append('blue')  # Connected
        self.robot_scatter.set_color(colors)

        # Update edges
        active_edges = []
        for i, neighbors in self.sim.neighbor_graph.items():
            for j in neighbors:
                if i < j:
                    active_edges.append([self.sim.robots[i].position, self.sim.robots[j].position])
        self.active_lines.set_segments(active_edges if active_edges else [])

        # Update candidate edge
        if self.sim.consensus_active and self.sim.consensus_candidate:
            a, b = self.sim.consensus_candidate
            candidate_edge = [[self.sim.robots[a].position, self.sim.robots[b].position]]
            self.candidate_lines.set_segments(candidate_edge)
        else:
            self.candidate_lines.set_segments([])

        # Update status
        decision = report.get('decision', 'unknown')
        status_lines = [
            f"Time: {self.sim.time:.2f}s | Robots: {self.sim.num_robots} | Edges: {len(self.sim.current_edges())}",
            f"Control: CLF-CBF QP (decentralized)",
            f"Pruned total: {len(self.sim.pruned_edges)}",
            ""
        ]

        # Calculate average distance to goal
        avg_dist_to_goal = np.mean([np.linalg.norm(r.position - self.sim.goal_position) for r in self.sim.robots])
        status_lines.append(f"Avg distance to goal: {avg_dist_to_goal:.2f}m")

        if decision == "prune":
            status_lines.append(f"[PRUNED] {report['removed_edge']}")
        elif decision == "topology_changed":
            status_lines.append(f"[!] Topology changed")
        elif decision == "waiting_for_stability":
            status_lines.append(f"[...] Stabilizing: {report['stable_rounds']}/{report['needed']}")
        elif decision == "building_history":
            status_lines.append(f"[+] Building history: {report['history_size']}/{report['needed']}")
        elif decision == "no_persistent_redundant_edges":
            status_lines.append(f"[OK] No persistent redundant edges")
        elif decision == "evaluating":
            status_lines.append(f"[?] Evaluating: {report['candidate']}")
            status_lines.append(f"  Rounds: {report['consensus_round']}/{report['needed_rounds']}")
        elif decision == "consensus_failed":
            status_lines.append(f"[X] FAILED: {report['failed_candidate']} NOT redundant")

        self.status_text.set_text('\n'.join(status_lines))

        return (self.robot_scatter, self.active_lines, self.candidate_lines,
                self.highlight_lines, self.status_text)

    def run(self, interval: int = 50):
        """Run the animation."""
        anim = FuncAnimation(
            self.fig, self._update_plot,
            interval=interval, blit=False, cache_frame_data=False
        )
        plt.tight_layout()
        plt.show()


# ------------------------------------------------------------------ #
# Main
# ------------------------------------------------------------------ #
def main():
    parser = argparse.ArgumentParser(description="Decentralized CLF-CBF QP control with consensus pruning")
    parser.add_argument("mode", nargs='?', default="text", choices=["gui", "text"],
                       help="Run mode: 'gui' for visualization or 'text' for console")
    parser.add_argument("--robots", type=int, default=6, help="Number of robots")
    parser.add_argument("--steps", type=int, default=100, help="Number of simulation steps (text mode)")
    parser.add_argument("--seed", type=int, default=None, help="Random seed")
    args = parser.parse_args()

    print(f"Goal position: ({np.random.rand() * 10:.2f}, {np.random.rand() * 10:.2f})")
    print(f"Starting {'QP GUI' if args.mode == 'gui' else 'QP TEXT'} with {args.robots} robots")
    print("CLF-CBF QP control enabled:")
    print("  - Each robot solves QP locally (decentralized)")
    print("  - CLF constraint for goal-seeking")
    print("  - CBF constraints for safety and connectivity")
    print("  - Distributed consensus-based pruning")
    print()

    sim = QPUnderwaterSimulation(
        num_robots=args.robots,
        seed=args.seed,
        verbose=True
    )

    if args.mode == "gui":
        viz = QPVisualization(sim)
        viz.run()
    else:
        for i, report in enumerate(sim.iterate(args.steps)):
            if i % 10 == 0:
                print(f"Step {i}: {report.get('decision', 'unknown')}")


if __name__ == "__main__":
    main()
