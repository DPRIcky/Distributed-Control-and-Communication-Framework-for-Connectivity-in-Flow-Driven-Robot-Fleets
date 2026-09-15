"""Standalone underwater environment and Brownian robot simulation module.

This module factors out the underwater flow field and robot dynamics so that
alternative communication or control strategies can be explored independently
from the legacy simulation. It provides:
    * UnderwaterEnvironment with currents, vortices, and turbulence
    * UnderwaterRobot that experiences Brownian motion plus environmental forces
    * UnderwaterSimulation helper that advances a swarm of robots

Example:
    python underwater_environment_brownian.py  # launches a simple animation
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Iterable, Optional, Set

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
import numpy as np

from distributed_consensus_pruning import ConsensusPruningSimulation


class UnderwaterEnvironment:
    """Environment with steady currents, vortices, and turbulence."""

    def __init__(
        self,
        workspace_size: Tuple[float, float] = (10.0, 10.0),
        flow_strength: float = 1.0,
    ) -> None:
        self.workspace_size = workspace_size
        self.simulation_time = 0.0
        self.flow_strength = flow_strength

        # Water properties
        self.water_density = 1025.0  # kg/m^3
        self.water_viscosity = 1.002e-3  # Pa*s
        self.gravity = 9.81  # m/s^2

        # Base current parameters
        self.base_current_base = np.array([0.35, 0.18], dtype=float)
        self.base_current = self.base_current_base * self.flow_strength
        self.current_velocity = self.base_current.copy()
        self.current_direction_change_rate = 0.01  # rad/s

        # Vortex parameters (center positions, strengths, radii)
        self.vortex_centers = [
            np.array([3.0, 7.0], dtype=float),
            np.array([7.0, 3.0], dtype=float),
        ]
        self.vortex_strengths_base = np.array([1.5, -1.2], dtype=float)
        self.vortex_strengths = (
            self.vortex_strengths_base * self.flow_strength
        ).tolist()
        self.vortex_radii = [2.0, 1.8]

        # Turbulence parameters
        self.turbulence_intensity_base = 0.05
        self.turbulence_intensity = self.turbulence_intensity_base * abs(
            self.flow_strength
        )
        self.turbulence_scale = 1.0

    def update_time(self, dt: float) -> None:
        """Advance the environment clock and update slowly varying currents."""
        self.simulation_time += dt
        angle_change = self.current_direction_change_rate * self.simulation_time
        rotation_matrix = np.array(
            [
                [math.cos(angle_change), -math.sin(angle_change)],
                [math.sin(angle_change), math.cos(angle_change)],
            ]
        )
        base_current = self.base_current_base * self.flow_strength
        self.current_velocity = rotation_matrix @ base_current

    def get_flow_field(self, position: np.ndarray) -> np.ndarray:
        """Compute the instantaneous water velocity at a position."""
        total_flow = self.current_velocity.copy()

        for strength, radius, center in zip(
            self.vortex_strengths, self.vortex_radii, self.vortex_centers
        ):
            r_vec = position - center
            r = np.linalg.norm(r_vec)
            if r < 1e-6:
                continue
            decay = math.exp(-(r / radius) ** 2)
            perp = np.array([-r_vec[1], r_vec[0]]) / r
            vortex_velocity = (strength / (2.0 * math.pi * r)) * perp * decay
            total_flow += vortex_velocity

        turbulence_x = self.turbulence_intensity * math.sin(
            2.0 * math.pi * position[0] / self.turbulence_scale
            + self.simulation_time * 0.5
        )
        turbulence_y = self.turbulence_intensity * math.cos(
            2.0 * math.pi * position[1] / self.turbulence_scale
            + self.simulation_time * 0.3
        )
        return total_flow + np.array([turbulence_x, turbulence_y])

    def calculate_drag_force(
        self, robot: "UnderwaterRobot", relative_velocity: np.ndarray
    ) -> np.ndarray:
        """Quadratic drag proportional to the square of relative speed."""
        speed = np.linalg.norm(relative_velocity)
        if speed < 1e-6:
            return np.zeros(2)
        drag_magnitude = (
            0.5
            * self.water_density
            * robot.drag_coefficient
            * robot.cross_sectional_area
            * speed**2
        )
        drag_direction = -relative_velocity / speed
        return drag_magnitude * drag_direction

    def calculate_buoyancy_force(self, robot: "UnderwaterRobot") -> np.ndarray:
        """Net upward or downward force due to buoyancy."""
        buoyant_force = self.water_density * robot.volume * self.gravity
        weight = robot.mass * self.gravity
        net_vertical = (buoyant_force - weight) * 0.1  # scaled for 2D simulation
        return np.array([0.0, net_vertical])

    def draw_flow_field(self, ax, resolution: int = 18) -> None:
        """Render a quiver plot of the flow field for visual context."""
        x_range = np.linspace(0.0, self.workspace_size[0], resolution)
        y_range = np.linspace(0.0, self.workspace_size[1], resolution)
        X, Y = np.meshgrid(x_range, y_range)
        U = np.zeros_like(X)
        V = np.zeros_like(Y)

        for i in range(resolution):
            for j in range(resolution):
                pos = np.array([X[i, j], Y[i, j]])
                flow = self.get_flow_field(pos)
                U[i, j] = flow[0]
                V[i, j] = flow[1]

        ax.quiver(
            X,
            Y,
            U,
            V,
            color="cyan",
            alpha=0.35,
            scale=15,
            width=0.004,
            minlength=0.1,
        )
        for center in self.vortex_centers:
            ax.scatter(center[0], center[1], c="orange", s=40, marker="x", alpha=0.7)


@dataclass
class UnderwaterRobot:
    """Robot model influenced by Brownian noise and environment forces."""

    robot_id: int
    position: np.ndarray
    rng: np.random.Generator
    brownian_intensity: float
    mass: float = field(default_factory=lambda: 2.0 + np.random.normal(0.0, 0.2))
    drag_coefficient: float = field(
        default_factory=lambda: 0.8 + np.random.normal(0.0, 0.1)
    )
    cross_sectional_area: float = field(
        default_factory=lambda: 0.01 + np.random.normal(0.0, 0.002)
    )
    neighbors: Set[int] = field(default_factory=set)
    removed_edges: List[Tuple[int, int]] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.position = self.position.astype(float)
        # Relative velocity with respect to the surrounding water
        init_std = 0.04 if self.brownian_intensity > 0 else 0.0
        self.relative_velocity = self.rng.normal(0.0, init_std, size=2)
        self.acceleration = np.zeros(2)
        self.volume = self.mass / 1800.0  # slightly buoyant
        self.world_velocity = np.zeros(2)
        self.path: List[np.ndarray] = [self.position.copy()]

    def step(
        self,
        environment: UnderwaterEnvironment,
        dt: float,
        control_input: Optional[np.ndarray] = None,
    ) -> None:
        """Advance the robot state by one integration step."""
        flow_velocity = environment.get_flow_field(self.position)
        drag_force = environment.calculate_drag_force(self, self.relative_velocity)
        buoyancy_force = environment.calculate_buoyancy_force(self)
        if control_input is None:
            control_input = np.zeros(2)
        control_force = control_input * self.mass

        if self.brownian_intensity > 0:
            brownian_force = (
                self.rng.normal(0.0, self.brownian_intensity, size=2) * self.mass
            )
        else:
            brownian_force = np.zeros(2)

        total_force = drag_force + buoyancy_force + brownian_force + control_force
        self.acceleration = total_force / self.mass
        self.relative_velocity += self.acceleration * dt
        # Additional random velocity kick (Brownian agitation)
        if self.brownian_intensity > 0:
            self.relative_velocity += self.rng.normal(
                0.0, self.brownian_intensity * 0.15, size=2
            )

        # Limit relative speed for numerical stability
        max_rel_speed = 1.5
        speed = np.linalg.norm(self.relative_velocity)
        if speed > max_rel_speed:
            self.relative_velocity *= max_rel_speed / speed

        world_velocity = flow_velocity + self.relative_velocity
        self.world_velocity = world_velocity

        # Combine advective transport with Brownian displacement
        if self.brownian_intensity > 0:
            brownian_displacement = self.rng.normal(
                0.0, self.brownian_intensity * math.sqrt(dt) * 0.25, size=2
            )
        else:
            brownian_displacement = np.zeros(2)
        self.position += world_velocity * dt + brownian_displacement

        self.path.append(self.position.copy())
        if len(self.path) > 300:
            self.path.pop(0)

    def enforce_workspace_bounds(self, workspace: Tuple[float, float]) -> None:
        """Reflect robots from the environment boundaries."""
        for axis, limit in enumerate(workspace):
            if self.position[axis] < 0.0:
                self.position[axis] = 0.0
                self.relative_velocity[axis] = abs(self.relative_velocity[axis]) * 0.5
            elif self.position[axis] > limit:
                self.position[axis] = limit
                self.relative_velocity[axis] = -abs(self.relative_velocity[axis]) * 0.5


@dataclass
class ProposalState:
    """Track the distributed agreement process for a single edge removal."""

    edge: Tuple[int, int]
    proposer: int
    length: float
    informed_nodes: Set[int] = field(default_factory=set)
    frontier: Set[int] = field(default_factory=set)
    steps_elapsed: int = 0


class DistributedGraphPruner:
    """
    Generate random connected graphs and manage multi-hop consensus-based pruning.

    Each iteration draws a new connected graph with a random number of nodes
    (within user-provided bounds). Robots exchange neighbor knowledge one hop at
    a time. Whenever a redundant edge is detected, a length-based consensus
    protocol selects the longest candidate edge for removal while freezing the
    rest of the topology until agreement is reached.
    """

    def __init__(
        self,
        min_nodes: int = 5,
        max_nodes: int = 20,
        workspace_size: Tuple[float, float] = (10.0, 10.0),
        rng: Optional[np.random.Generator] = None,
    ) -> None:
        if min_nodes < 2 or max_nodes < min_nodes:
            raise ValueError("Node bounds must satisfy 2 <= min_nodes <= max_nodes.")
        self.min_nodes = min_nodes
        self.max_nodes = max_nodes
        self.workspace_size = workspace_size
        self.rng = rng or np.random.default_rng()

        self.nodes: List[int] = []
        self.positions: Optional[np.ndarray] = None
        self.graph: Dict[int, Set[int]] = {}
        self.edge_lengths: Dict[Tuple[int, int], float] = {}
        self.initial_edge_lengths: Dict[Tuple[int, int], float] = {}
        self.knowledge: Dict[int, Dict[Tuple[int, int], float]] = {}
        self.proposal: Optional[ProposalState] = None
        self.completed_removals: List[Tuple[int, int]] = []
        self.knowledge_round: int = 0

    # ------------------------------------------------------------------ #
    # Graph generation helpers
    # ------------------------------------------------------------------ #
    def _add_edge(self, node_a: int, node_b: int) -> None:
        """Insert an undirected edge and store its geometric length."""
        if node_a == node_b:
            return
        key = (min(node_a, node_b), max(node_a, node_b))
        if key in self.edge_lengths:
            return
        distance = float(
            np.linalg.norm(self.positions[node_a] - self.positions[node_b])
        )
        self.graph[node_a].add(node_b)
        self.graph[node_b].add(node_a)
        self.edge_lengths[key] = distance

    def _generate_positions(self, num_nodes: int) -> np.ndarray:
        """Sample robot positions within the workspace for edge weighting."""
        spans = np.array(self.workspace_size, dtype=float)
        return self.rng.random((num_nodes, 2)) * spans

    def _build_random_connected_graph(self) -> None:
        """Create a random connected graph with stochastic extra edges."""
        num_nodes = int(self.rng.integers(self.min_nodes, self.max_nodes + 1))
        self.nodes = list(range(num_nodes))
        self.positions = self._generate_positions(num_nodes)
        self.graph = {node: set() for node in self.nodes}
        self.edge_lengths = {}
        self.knowledge = {}
        self.proposal = None
        self.completed_removals = []

        # Ensure connectivity via a random spanning tree
        for node in self.nodes[1:]:
            parent = int(self.rng.integers(0, node))
            self._add_edge(node, parent)

        # Stochastically add additional edges to create cycles
        candidate_pairs: List[Tuple[int, int]] = []
        for i in self.nodes:
            for j in range(i + 1, num_nodes):
                if j not in self.graph[i]:
                    candidate_pairs.append((i, j))

        self.rng.shuffle(candidate_pairs)
        target_extra_edges = max(1, num_nodes // 2)
        extra_edges_added = 0
        for i, j in candidate_pairs:
            if extra_edges_added >= target_extra_edges:
                break
            probability = 0.25 + 0.75 * self.rng.random()
            if self.rng.random() < probability:
                self._add_edge(i, j)
                extra_edges_added += 1

        # If no extra edges were added (degenerate draw), force one
        if extra_edges_added == 0 and candidate_pairs:
            i, j = candidate_pairs[0]
            self._add_edge(i, j)

        self.initial_edge_lengths = dict(self.edge_lengths)
        self._initialize_knowledge()

    def _initialize_knowledge(self) -> None:
        """Seed each robot's knowledge with its direct incident edges."""
        self.knowledge = {}
        for node in self.nodes:
            local_edges: Dict[Tuple[int, int], float] = {}
            for neighbor in self.graph[node]:
                edge_key = (min(node, neighbor), max(node, neighbor))
                local_edges[edge_key] = self.edge_lengths[edge_key]
            self.knowledge[node] = local_edges

    # Public API ------------------------------------------------------- #
    def start_new_iteration(self) -> None:
        """Reset state and draw a fresh random connected communication graph."""
        self._build_random_connected_graph()
        self.knowledge_round = 0

    # ------------------------------------------------------------------ #
    # Knowledge propagation and consensus mechanics
    # ------------------------------------------------------------------ #
    def _propagate_knowledge_one_hop(self) -> bool:
        """
        Share knowledge with immediate neighbors.

        Returns
        -------
        bool
            True if any robot learned a previously unknown edge.
        """
        learned = False
        snapshot = {node: dict(edges) for node, edges in self.knowledge.items()}
        updated: Dict[int, Dict[Tuple[int, int], float]] = {
            node: dict(edges) for node, edges in self.knowledge.items()
        }

        for node in self.nodes:
            combined = updated[node]
            for neighbor in self.graph[node]:
                neighbor_edges = snapshot[neighbor]
                for edge_key, length in neighbor_edges.items():
                    if edge_key not in combined:
                        combined[edge_key] = length
                        learned = True
        self.knowledge = updated
        return learned

    def _build_local_graph(
        self, local_edges: Dict[Tuple[int, int], float]
    ) -> Dict[int, Dict[int, float]]:
        adjacency: Dict[int, Dict[int, float]] = {}
        for (node_a, node_b), length in local_edges.items():
            adjacency.setdefault(node_a, {})[node_b] = length
            adjacency.setdefault(node_b, {})[node_a] = length
        return adjacency

    def _has_strictly_shorter_path(
        self,
        adjacency: Dict[int, Dict[int, float]],
        start: int,
        goal: int,
        threshold: float,
    ) -> bool:
        """
        Check if a path exists from start to goal using only edges shorter than threshold.

        This captures the idea that the communication graph can prune an edge if
        every hop in the alternative route is cheaper than the candidate edge.
        """
        if start not in adjacency or goal not in adjacency:
            return False

        visited: Set[int] = {start}
        queue: List[int] = [start]

        while queue:
            node = queue.pop(0)
            for neighbor, weight in adjacency.get(node, {}).items():
                if weight >= threshold - 1e-9:
                    continue
                if neighbor == goal:
                    return True
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        return False

    def _detect_redundant_edges(self) -> Dict[Tuple[int, int], List[int]]:
        """
        Identify edges that have a strictly shorter alternative path.

        Returns
        -------
        Dict[Tuple[int, int], List[int]]
            Mapping from candidate edge to the list of robots that detected it.
        """
        candidates: Dict[Tuple[int, int], List[int]] = {}
        for node in self.nodes:
            local_edges = self.knowledge.get(node, {})
            if not local_edges:
                continue
            adjacency = self._build_local_graph(local_edges)
            for edge_key, direct_length in local_edges.items():
                if edge_key not in self.edge_lengths:
                    # Edge already pruned; stale knowledge.
                    continue
                if self._has_strictly_shorter_path(
                    adjacency, edge_key[0], edge_key[1], direct_length
                ):
                    candidates.setdefault(edge_key, []).append(node)
        return candidates

    def _select_proposal(
        self, candidates: Dict[Tuple[int, int], List[int]]
    ) -> Optional[ProposalState]:
        """Choose the longest redundant edge and seed a consensus request."""
        if not candidates:
            return None

        def candidate_key(edge_key: Tuple[int, int]) -> Tuple[float, int, int]:
            length = self.edge_lengths.get(edge_key, -math.inf)
            return (length, -len(candidates[edge_key]), -max(edge_key))

        chosen_edge = max(candidates.keys(), key=candidate_key)
        length = self.edge_lengths[chosen_edge]
        proposer_candidates = sorted(candidates[chosen_edge])
        proposer = proposer_candidates[0] if proposer_candidates else chosen_edge[0]
        proposal = ProposalState(
            edge=chosen_edge,
            proposer=proposer,
            length=length,
            informed_nodes={proposer},
            frontier={proposer},
        )
        return proposal

    def _advance_proposal(self) -> None:
        """Propagate the active proposal one hop further through the network."""
        if self.proposal is None:
            return

        current_frontier = set(self.proposal.frontier)
        new_frontier: Set[int] = set()
        informed = set(self.proposal.informed_nodes)

        for node in current_frontier:
            informed.add(node)
            for neighbor in self.graph[node]:
                if neighbor not in informed:
                    new_frontier.add(neighbor)

        self.proposal.frontier = new_frontier
        self.proposal.informed_nodes = informed
        self.proposal.steps_elapsed += 1

    def _finalize_proposal(self) -> Tuple[int, int]:
        """Remove the agreed edge and update all knowledge bases."""
        assert self.proposal is not None
        edge = self.proposal.edge
        node_a, node_b = edge
        if node_b in self.graph[node_a]:
            self.graph[node_a].remove(node_b)
        if node_a in self.graph[node_b]:
            self.graph[node_b].remove(node_a)
        self.edge_lengths.pop(edge, None)

        for node in self.nodes:
            self.knowledge[node].pop(edge, None)

        self.completed_removals.append(edge)
        self.proposal = None
        return edge

    def step(self) -> Dict[str, Optional[object]]:
        """
        Advance the multi-hop communication protocol by one synchronous round.

        Returns a dictionary that summarizes what occurred during this round:
        - 'round': current hop count
        - 'knowledge_updated': whether any robot learned new edges
        - 'proposal': information about a freshly announced proposal (if any)
        - 'proposal_progress': status of an ongoing proposal
        - 'removed_edge': edge removed after consensus completion (if any)
        """
        if not self.nodes:
            raise RuntimeError(
                "No active graph. Call start_new_iteration() before stepping."
            )

        self.knowledge_round += 1
        report: Dict[str, Optional[object]] = {
            "round": self.knowledge_round,
            "knowledge_updated": None,
            "proposal": None,
            "proposal_progress": None,
            "removed_edge": None,
        }

        knowledge_updated = self._propagate_knowledge_one_hop()
        report["knowledge_updated"] = knowledge_updated

        if self.proposal is None:
            candidates = self._detect_redundant_edges()
            self.proposal = self._select_proposal(candidates)
            if self.proposal is not None:
                report["proposal"] = {
                    "edge": self.proposal.edge,
                    "length": self.proposal.length,
                    "proposer": self.proposal.proposer,
                    "supporters": candidates.get(self.proposal.edge, []),
                }
        else:
            self._advance_proposal()
            report["proposal_progress"] = {
                "edge": self.proposal.edge,
                "informed": len(self.proposal.informed_nodes),
                "steps": self.proposal.steps_elapsed,
            }
            if (
                len(self.proposal.informed_nodes) == len(self.nodes)
                and not self.proposal.frontier
            ):
                removed = self._finalize_proposal()
                report["removed_edge"] = removed

        return report

    def run_until_converged(
        self,
        max_rounds: int = 200,
    ) -> List[Dict[str, Optional[object]]]:
        """
        Execute rounds until no further redundant edges are detected or limits hit.

        Parameters
        ----------
        max_rounds : int
            Safety cap on the number of synchronous hop updates.

        Returns
        -------
        List[Dict[str, Optional[object]]]
            Chronicle of protocol events for post-analysis.
        """
        history: List[Dict[str, Optional[object]]] = []
        for _ in range(max_rounds):
            round_report = self.step()
            history.append(round_report)

            if round_report.get("removed_edge") is not None:
                # Allow the topology to settle before searching for new proposals.
                continue

            if self.proposal is not None:
                # Still negotiating an edge removal.
                continue

            # Stop when the graph is a tree or knowledge has saturated with no candidates.
            if len(self.edge_lengths) <= max(0, len(self.nodes) - 1):
                break

            knowledge_sizes = {len(edges) for edges in self.knowledge.values()}
            if len(knowledge_sizes) == 1 and not round_report.get("knowledge_updated"):
                # Everyone holds the same knowledge; if no proposal emerged, nothing else to prune.
                break

        return history

class UnderwaterSimulation:
    """Helper that evolves multiple robots in the shared environment."""

    def __init__(
        self,
        num_robots: int = 20,
        workspace_size: Tuple[float, float] = (10.0, 10.0),
        dt: float = 0.1,
        brownian_intensity: float = 0.0,
        flow_strength: float = 1.0,
        communication_radius: float = 2.0,
        safety_distance: float = 0.3,
        cbf_gain_safety: float = 6.0,
        cbf_gain_comm: float = 3.0,
        max_control_acc: float = 1.0,
        seed: Optional[int] = None,
    ) -> None:
        self.environment = UnderwaterEnvironment(
            workspace_size=workspace_size, flow_strength=flow_strength
        )
        self.dt = dt
        self.num_robots = num_robots
        self.flow_strength = flow_strength
        self.communication_radius = communication_radius
        self.safety_distance = safety_distance
        self.cbf_gain_safety = cbf_gain_safety
        self.cbf_gain_comm = cbf_gain_comm
        self.max_control_acc = max_control_acc
        base_rng = np.random.default_rng(seed)

        self.robots: List[UnderwaterRobot] = []
        cluster_center = np.array(
            [workspace_size[0] * 0.15, workspace_size[1] * 0.15], dtype=float
        )
        workspace_array = np.array(workspace_size, dtype=float)
        for robot_id in range(num_robots):
            robot_rng = np.random.default_rng(base_rng.integers(0, 2**32 - 1))
            start = cluster_center + base_rng.normal(0.0, 0.25, size=2)
            start = np.clip(start, [0.0, 0.0], workspace_array)
            robot = UnderwaterRobot(
                robot_id=robot_id,
                position=start,
                rng=robot_rng,
                brownian_intensity=brownian_intensity,
            )
            if flow_strength == 0.0:
                buoyancy_factor = 1.0
            else:
                buoyancy_jitter = 0.05 + base_rng.normal(0.0, 0.02)
                buoyancy_factor = 1.0 + buoyancy_jitter
            robot.volume = (
                robot.mass / self.environment.water_density * buoyancy_factor
            )
            self.robots.append(robot)

        # Start with a fully connected network over the initial cluster
        full_neighbors = [set(range(num_robots)) - {i} for i in range(num_robots)]
        self.neighbor_graph = full_neighbors
        for idx, robot in enumerate(self.robots):
            robot.neighbors = set(full_neighbors[idx])

        self.full_edge_set: Set[Tuple[int, int]] = {
            (min(i, j), max(i, j))
            for i in range(num_robots)
            for j in range(i + 1, num_robots)
        }
        self.last_communication_cycle: List[Dict[str, Tuple[int, int]]] = []
        self.communication_history: List[List[Dict[str, Tuple[int, int]]]] = []
        pruner_rng = np.random.default_rng(base_rng.integers(0, 2**32 - 1))
        self.graph_pruner = ConsensusPruningSimulation(
            num_nodes=self.num_robots, rng=pruner_rng, verbose=False
        )
        self.pruned_edges_permanent: Set[Tuple[int, int]] = set()
        self.consensus_history: List[Dict[str, Optional[object]]] = []

    def step(self) -> None:
        """Advance the full simulation by one time step."""
        self.environment.update_time(self.dt)
        control_inputs = [self.compute_cbf_control(idx) for idx in range(self.num_robots)]
        for idx, robot in enumerate(self.robots):
            robot.step(self.environment, self.dt, control_input=control_inputs[idx])
            robot.enforce_workspace_bounds(self.environment.workspace_size)
        for robot in self.robots:
            robot.removed_edges.clear()
        self.update_neighbor_graph()
        self.perform_consensus_pruning()

    def iterate(self, steps: int) -> Iterable[List[np.ndarray]]:
        """Yield robot positions over a number of steps."""
        for _ in range(steps):
            self.step()
            yield [robot.position.copy() for robot in self.robots]

    # ------------------------------------------------------------------ #
    # Communication and topology management
    # ------------------------------------------------------------------ #
    def _distance(self, idx_a: int, idx_b: int) -> float:
        """Euclidean distance between two robots."""
        return float(
            np.linalg.norm(self.robots[idx_a].position - self.robots[idx_b].position)
        )

    def compute_cbf_control(self, idx: int) -> np.ndarray:
        """
        Compute a simple CBF-inspired acceleration to maintain safety and connectivity.
        Robots are treated as single-integrator agents whose control input
        directly adjusts their velocity relative to the surrounding water.
        """
        if self.num_robots <= 1:
            return np.zeros(2)

        robot = self.robots[idx]
        control = np.zeros(2)

        for j, other in enumerate(self.robots):
            if j == idx:
                continue
            diff = robot.position - other.position
            dist = np.linalg.norm(diff)
            if dist < 1e-6:
                continue
            direction = diff / dist

            # Safety barrier: keep robots farther than safety_distance
            if dist < self.safety_distance:
                penetration = self.safety_distance - dist
                control += (
                    self.cbf_gain_safety
                    * (penetration / max(dist, 1e-3))
                    * direction
                )

            # Connectivity barrier: prevent the link from exceeding comm radius
            if self.communication_radius > 0 and dist > self.communication_radius:
                excess = dist - self.communication_radius
                control -= (
                    self.cbf_gain_comm
                    * (excess / max(self.communication_radius, 1e-3))
                    * direction
                )

        norm = np.linalg.norm(control)
        if norm > self.max_control_acc and norm > 0:
            control = control / norm * self.max_control_acc
        return control

    def update_neighbor_graph(self) -> None:
        """Recompute neighbors based on the current positions."""
        graph: List[Set[int]] = [set() for _ in range(self.num_robots)]
        for i in range(self.num_robots):
            for j in range(i + 1, self.num_robots):
                if self.communication_radius <= 0:
                    continue
                distance = self._distance(i, j)
                if distance <= self.communication_radius:
                    edge_key = (min(i, j), max(i, j))
                    if edge_key in self.pruned_edges_permanent:
                        continue
                    graph[i].add(j)
                    graph[j].add(i)

        self.neighbor_graph = graph
        for idx, robot in enumerate(self.robots):
            robot.neighbors = set(graph[idx])

    def _select_removal(
        self, candidate_a: int, candidate_b: int, shared_neighbor: int
    ) -> int:
        """Choose which neighbor should drop the shared edge."""
        dist_a = self._distance(candidate_a, shared_neighbor)
        dist_b = self._distance(candidate_b, shared_neighbor)

        if dist_a > dist_b + 1e-9:
            return candidate_a
        if dist_b > dist_a + 1e-9:
            return candidate_b
        # Tie-breaker: remove from the robot with higher id for determinism
        return max(candidate_a, candidate_b)

    def _prune_neighbor_edge(self, edge: Tuple[int, int]) -> None:
        """Remove an undirected edge from the neighbor graph and cache."""
        a, b = edge
        if b in self.neighbor_graph[a]:
            self.neighbor_graph[a].remove(b)
        if a in self.neighbor_graph[b]:
            self.neighbor_graph[b].remove(a)
        self.robots[a].neighbors.discard(b)
        self.robots[b].neighbors.discard(a)
        self.robots[a].removed_edges.append((a, b))
        self.robots[b].removed_edges.append((b, a))
        self.pruned_edges_permanent.add(edge)

    def perform_consensus_pruning(self, max_rounds: int = 200) -> None:
        """Run the decentralized consensus pruner on the current neighbor graph."""
        if self.num_robots <= 1:
            return

        edge_lengths: Dict[Tuple[int, int], float] = {}
        for i in range(self.num_robots):
            for j in self.neighbor_graph[i]:
                if j <= i:
                    continue
                key = (min(i, j), max(i, j))
                if key in self.pruned_edges_permanent:
                    continue
                edge_lengths[key] = self._distance(i, j)

        if not edge_lengths:
            return

        positions = np.array([robot.position for robot in self.robots], dtype=float)
        history, final_edges = self.graph_pruner.prune_graph_from_state(
            positions, edge_lengths, max_rounds=max_rounds
        )

        removed_edges = set(edge_lengths.keys()) - set(final_edges.keys())
        if removed_edges:
            for edge in removed_edges:
                sorted_edge = (min(edge), max(edge))
                self._prune_neighbor_edge(sorted_edge)
            self.consensus_history = history

    # Visualization helpers -------------------------------------------------
    def _init_animation(self, ax):
        ax.set_xlim(0.0, self.environment.workspace_size[0])
        ax.set_ylim(0.0, self.environment.workspace_size[1])
        ax.set_xlabel("X position (m)")
        ax.set_ylabel("Y position (m)")
        ax.set_title("Underwater Robots with Brownian Motion")
        return ax

    def animate(self, frames: int = 300, interval_ms: int = 50) -> None:
        """Display a simple animation of the robots inside the environment."""
        fig, ax = plt.subplots(figsize=(6, 6))
        self._init_animation(ax)

        # Draw flow field background once at t=0 for spatial context
        self.environment.draw_flow_field(ax)

        scatter = ax.scatter(
            [robot.position[0] for robot in self.robots],
            [robot.position[1] for robot in self.robots],
            s=80,
            c="royalblue",
            alpha=0.8,
        )
        trails: List[plt.Line2D] = []
        for robot in self.robots:
            (trail,) = ax.plot([], [], linewidth=1.0, alpha=0.4)
            trails.append(trail)

        # Communication edges
        edge_collection = LineCollection(
            [], colors="orange", linewidths=1.5, alpha=0.6, zorder=1
        )
        ax.add_collection(edge_collection)

        def build_edge_segments():
            segments = []
            for i in range(self.num_robots):
                for j in self.neighbor_graph[i]:
                    if j > i:
                        pi = self.robots[i].position
                        pj = self.robots[j].position
                        segments.append([pi, pj])
            return segments

        def init_func():
            scatter.set_offsets([robot.position for robot in self.robots])
            for robot, trail in zip(self.robots, trails):
                history = np.array(robot.path[-50:])
                if history.size:
                    trail.set_data(history[:, 0], history[:, 1])
                else:
                    trail.set_data([], [])
            edge_collection.set_segments(build_edge_segments())
            return [scatter, edge_collection, *trails]

        def update(_frame: int):
            self.step()
            scatter.set_offsets([robot.position for robot in self.robots])
            for robot, trail in zip(self.robots, trails):
                history = np.array(robot.path[-50:])
                if history.size:
                    trail.set_data(history[:, 0], history[:, 1])
                else:
                    trail.set_data([], [])
            edge_collection.set_segments(build_edge_segments())
            return [scatter, edge_collection, *trails]

        self._animation = FuncAnimation(
            fig,
            update,
            frames=frames,
            init_func=init_func,
            interval=interval_ms,
            blit=False,
            repeat=True,
        )
        plt.tight_layout()
        plt.show()


def main() -> None:
    """Run a demo showcasing Brownian motion inside the underwater environment."""
    print("=" * 55)
    print("UNDERWATER BROWNIAN SWARM SIMULATION")
    print("=" * 55)

    def prompt_float(
        message: str, default: float, minimum: float, maximum: Optional[float] = None
    ) -> float:
        while True:
            try:
                raw = input(message).strip()
            except EOFError:
                raw = ""

            if not raw:
                return default

            try:
                value = float(raw)
            except ValueError:
                print("Please enter a valid number.")
                continue

            if value < minimum:
                print(f"Value must be at least {minimum}.")
                continue

            if maximum is not None and value > maximum:
                print(f"Value must be no more than {maximum}.")
                continue

            return value

    while True:
        try:
            user_entry = input(
                "Enter the number of robots (nodes) [3-20]: "
            ).strip()
        except EOFError:
            user_entry = ""

        if not user_entry:
            print("Please enter an integer between 3 and 20.")
            continue

        try:
            num_robots = int(user_entry)
        except ValueError:
            print("Please enter a valid integer.")
            continue

        if 3 <= num_robots <= 20:
            break
        print("Please enter a number between 3 and 20.")

    flow_strength = prompt_float(
        "Enter flow strength (0 disables flow) [default 1.0]: ", default=1.0, minimum=0.0
    )
    brownian_intensity = prompt_float(
        "Enter Brownian intensity (0 disables noise) [default 0.0]: ",
        default=0.0,
        minimum=0.0,
    )
    communication_radius = prompt_float(
        "Enter communication radius (must be > 0) [default 2.0]: ",
        default=2.0,
        minimum=0.1,
    )
    safety_distance = prompt_float(
        "Enter safety distance (< communication radius) [default 0.3]: ",
        default=0.3,
        minimum=0.05,
        maximum=communication_radius * 0.9,
    )

    simulation = UnderwaterSimulation(
        num_robots=num_robots,
        brownian_intensity=brownian_intensity,
        flow_strength=flow_strength,
        communication_radius=communication_radius,
        safety_distance=safety_distance,
        seed=42,
    )
    simulation.animate(frames=400, interval_ms=60)


if __name__ == "__main__":
    main()
