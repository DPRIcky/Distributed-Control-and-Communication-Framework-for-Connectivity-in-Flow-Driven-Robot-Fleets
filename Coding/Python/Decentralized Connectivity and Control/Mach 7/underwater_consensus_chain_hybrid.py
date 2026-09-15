"""Hybrid flow-driven underwater consensus pruning with multi-layer robustness.

Features:
    * Robots drift in underwater flow field with individual dynamics
    * CLF (Control Lyapunov Function) for goal-seeking behavior
    * CBF (Control Barrier Function) for safety and connectivity constraints
    * HYBRID decentralized consensus-based edge pruning with:
      - Topology stability detection
      - Rolling window validation
      - Interleaved consensus execution
      - Final safety checks before pruning

Two usage modes:
    python underwater_consensus_chain_hybrid.py         # run text-only simulation
    python underwater_consensus_chain_hybrid.py gui     # run interactive animation
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Set, Tuple

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
import numpy as np

from distributed_consensus_pruning import ConsensusPruningSimulation

Edge = Tuple[int, int]


@dataclass
class FlowField:
    """Simple 2‑D current: constant drift plus gentle sinusoidal swirl."""

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


class HybridUnderwaterSimulation:
    """Hybrid consensus pruning with multi-layer robustness against topology changes."""

    def __init__(
        self,
        num_robots: int = 10,
        communication_radius: float = 3.0,
        dt: float = 0.05,
        workspace_size: Tuple[float, float] = (10.0, 10.0),
        seed: Optional[int] = None,
        verbose: bool = True,
        goal_position: Optional[np.ndarray] = None,
        safety_distance: float = 1.2,
        clf_gain: float = 0.8,
        cbf_safety_gain: float = 4.0,
        cbf_connectivity_gain: float = 0.5,
        # Hybrid-specific parameters
        stability_threshold: int = 5,
        rolling_window_size: int = 10,
        consensus_rounds_per_step: int = 3,
    ) -> None:
        if num_robots < 3:
            raise ValueError("Need at least 3 robots for cycle formation.")

        self.num_robots = num_robots
        self.communication_radius = communication_radius
        self.dt = dt
        self.workspace = np.array(workspace_size, dtype=float)
        self.time = 0.0
        self.verbose = verbose

        base_rng = np.random.default_rng(seed)

        # Goal-seeking and safety parameters
        if goal_position is not None:
            self.goal_position = goal_position
        else:
            goal_x = base_rng.uniform(workspace_size[0] * 0.5, workspace_size[0] * 0.95)
            goal_y = base_rng.uniform(workspace_size[1] * 0.2, workspace_size[1] * 0.8)
            self.goal_position = np.array([goal_x, goal_y])

        self.safety_distance = safety_distance
        self.clf_gain = clf_gain
        self.cbf_safety_gain = cbf_safety_gain
        self.cbf_connectivity_gain = cbf_connectivity_gain

        # Initialize robots
        cluster_center = np.array([workspace_size[0] * 0.25, workspace_size[1] * 0.5])
        self.robots: List[ChainRobot] = []
        for robot_id in range(num_robots):
            rng = np.random.default_rng(base_rng.integers(0, 2**32 - 1))
            start = cluster_center + base_rng.normal(0.0, 0.4, size=2)
            start = np.clip(start, [0.0, 0.0], self.workspace)
            self.robots.append(ChainRobot(robot_id=robot_id, position=start, rng=rng))

        self.flow = FlowField()
        self.neighbor_graph: List[Set[int]] = [set() for _ in range(num_robots)]
        self.pruned_edges: Set[Edge] = set()

        # Hybrid-specific state
        self.stability_threshold = stability_threshold
        self.rolling_window_size = rolling_window_size
        self.consensus_rounds_per_step = consensus_rounds_per_step

        self.topology_stable_rounds = 0
        self.topology_history: List[Dict] = []
        self.consensus_active = False
        self.consensus_candidate: Optional[Edge] = None
        self.consensus_round = 0
        self.total_consensus_rounds_needed = 50  # Need 50 rounds for agreement (increased from 10)
        self.failed_candidates: Set[Edge] = set()  # Track edges that failed consensus

        self.consensus = ConsensusPruningSimulation(
            num_nodes=num_robots, verbose=False  # Disable verbose for cleaner output
        )

        self._update_neighbor_graph()
        self._build_connectivity_tree()

    # ------------------------------------------------------------------ #
    # Geometry and graph helpers
    # ------------------------------------------------------------------ #
    def _distance(self, i: int, j: int) -> float:
        return float(np.linalg.norm(self.robots[i].position - self.robots[j].position))

    def _update_neighbor_graph(self) -> None:
        graph: List[Set[int]] = [set() for _ in range(self.num_robots)]
        for i in range(self.num_robots):
            for j in range(i + 1, self.num_robots):
                key = (min(i, j), max(i, j))
                if key in self.pruned_edges:
                    continue
                if self._distance(i, j) <= self.communication_radius:
                    graph[i].add(j)
                    graph[j].add(i)
        self.neighbor_graph = graph

    def _gather_edge_lengths(self) -> Dict[Edge, float]:
        lengths: Dict[Edge, float] = {}
        for i in range(self.num_robots):
            for j in self.neighbor_graph[i]:
                if j <= i:
                    continue
                key = (min(i, j), max(i, j))
                if key in self.pruned_edges:
                    continue
                lengths[key] = self._distance(i, j)
        return lengths

    def current_edges(self) -> List[Edge]:
        edges: List[Edge] = []
        for i in range(self.num_robots):
            for j in self.neighbor_graph[i]:
                if j > i:
                    edges.append((i, j))
        return edges

    # ------------------------------------------------------------------ #
    # Connectivity tree and CLF-CBF control
    # ------------------------------------------------------------------ #
    def _build_connectivity_tree(self) -> None:
        """Build a tree structure for connectivity maintenance using BFS."""
        for robot in self.robots:
            robot.parent = None

        cluster_center = np.array([self.workspace[0] * 0.25, self.workspace[1] * 0.5])
        distances_to_center = [np.linalg.norm(robot.position - cluster_center) for robot in self.robots]
        root_id = int(np.argmin(distances_to_center))

        visited = {root_id}
        queue = [root_id]

        while queue:
            current_id = queue.pop(0)
            for neighbor_id in self.neighbor_graph[current_id]:
                if neighbor_id not in visited:
                    self.robots[neighbor_id].parent = current_id
                    visited.add(neighbor_id)
                    queue.append(neighbor_id)

    def _compute_clf_cbf_control(self, robot_id: int) -> np.ndarray:
        """Compute CLF-CBF control for goal-seeking with safety and connectivity."""
        robot = self.robots[robot_id]
        control = np.zeros(2)

        # CLF: Control Lyapunov Function for goal-seeking
        error_to_goal = robot.position - self.goal_position
        distance_to_goal = np.linalg.norm(error_to_goal)

        if robot.parent is None and robot_id != 0:
            clf_gain_adjusted = self.clf_gain * 0.15
        else:
            clf_gain_adjusted = self.clf_gain

        if distance_to_goal > 0.1:
            clf_control = -clf_gain_adjusted * error_to_goal
            control += clf_control

        # CBF Safety: Collision avoidance with other robots
        for other_id, other_robot in enumerate(self.robots):
            if other_id == robot_id:
                continue

            diff = robot.position - other_robot.position
            dist = np.linalg.norm(diff)

            if dist < 1e-6:
                diff = self.robots[robot_id].rng.normal(0.0, 0.1, size=2)
                dist = np.linalg.norm(diff)

            if dist < self.safety_distance * 2.0:
                barrier_value = dist**2 - self.safety_distance**2
                if barrier_value < 0.1:
                    direction = diff / dist
                    safety_control = self.cbf_safety_gain * (-barrier_value) * direction
                    control += safety_control

        # CBF Connectivity: Maintain connection to parent
        if robot.parent is not None:
            parent_robot = self.robots[robot.parent]
            diff_to_parent = robot.position - parent_robot.position
            dist_to_parent = np.linalg.norm(diff_to_parent)

            if dist_to_parent > 1e-6:
                max_distance = self.communication_radius * 0.95
                barrier_value = max_distance**2 - dist_to_parent**2

                if barrier_value < 0.2:
                    direction_to_parent = -diff_to_parent / dist_to_parent
                    connectivity_control = self.cbf_connectivity_gain * (-barrier_value) * direction_to_parent
                    control += connectivity_control

        # Limit control magnitude
        control_magnitude = np.linalg.norm(control)
        max_control = 1.0
        if control_magnitude > max_control:
            control = control * (max_control / control_magnitude)

        return control

    # ------------------------------------------------------------------ #
    # Hybrid pruning methods
    # ------------------------------------------------------------------ #
    def _has_alternative_path(self, edge: Edge, edge_set: Set[Edge]) -> bool:
        """Check if alternative path exists between edge endpoints without using the edge.

        This checks GRAPH connectivity, not tree connectivity to a specific root.
        """
        a, b = edge

        # Build adjacency from edge set (excluding the candidate edge)
        adj: Dict[int, Set[int]] = {i: set() for i in range(self.num_robots)}
        for e in edge_set:
            if e == edge:
                continue
            u, v = e
            adj[u].add(v)
            adj[v].add(u)

        # BFS to find ANY path from a to b
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
        """Check if the graph is fully connected (all robots can reach each other)."""
        if not edge_set:
            return False

        # Build adjacency
        adj: Dict[int, Set[int]] = {i: set() for i in range(self.num_robots)}
        for u, v in edge_set:
            adj[u].add(v)
            adj[v].add(u)

        # BFS from robot 0 - can we reach everyone?
        visited = {0}
        queue = [0]

        while queue:
            node = queue.pop(0)
            for neighbor in adj[node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)

        # All robots must be reachable
        return len(visited) == self.num_robots

    def _find_persistent_redundant_edge(self, debug: bool = False) -> Optional[Edge]:
        """Find edge that's redundant in current topology and recent history."""
        if len(self.topology_history) < self.rolling_window_size:
            return None

        # Use CURRENT topology (most recent snapshot)
        current_snap = self.topology_history[-1]
        current_edges = current_snap['edges']
        current_lengths = current_snap['lengths']

        if not current_edges:
            return None

        # Filter out edges that already failed consensus
        candidate_edges = current_edges - self.failed_candidates

        if not candidate_edges:
            if debug:
                print(f"[DEBUG] No candidate edges (all in blacklist)")
            return None

        # Find edges that are CURRENTLY redundant
        candidates = []
        rejected = {}  # Track why edges were rejected

        for edge in candidate_edges:
            # CRITICAL CHECK 1: Does alternative path exist in current topology?
            has_alt_path = self._has_alternative_path(edge, current_edges)

            if not has_alt_path:
                if debug:
                    rejected[edge] = "No alternative path"
                continue

            # CRITICAL CHECK 2: Does graph remain connected after removing this edge?
            graph_stays_connected = self._check_graph_connectivity(current_edges - {edge})

            if not graph_stays_connected:
                if debug:
                    rejected[edge] = "Graph disconnects"
                continue

            # OPTIONAL CHECK: Verify redundancy is stable across recent history
            # Count how many recent snapshots this edge was redundant in
            redundant_count = 0
            for snap in self.topology_history[-5:]:  # Check last 5 snapshots
                if edge in snap['edges']:
                    if self._has_alternative_path(edge, snap['edges']):
                        if self._check_graph_connectivity(snap['edges'] - {edge}):
                            redundant_count += 1

            # Only consider if redundant in at least 3 out of last 5 snapshots
            if redundant_count < 3:
                if debug:
                    rejected[edge] = f"Unstable ({redundant_count}/5 snapshots)"
                continue

            # Get current edge length
            edge_length = current_lengths.get(edge, 0.0)

            # All checks passed - this edge is redundant and safe to remove
            candidates.append((edge, edge_length))

        if debug and rejected:
            print(f"\n[DEBUG] Rejected {len(rejected)} edges:")
            for edge, reason in list(rejected.items())[:8]:  # Show first 8
                print(f"  {edge}: {reason}")

        if not candidates:
            return None

        # Sort by edge length (prune longest edges first - they're weakest)
        candidates.sort(key=lambda x: -x[1])

        if debug:
            print(f"[DEBUG] Found {len(candidates)} redundant candidates")
            print(f"[DEBUG] Selected: {candidates[0][0]} (length={candidates[0][1]:.2f})")

        return candidates[0][0]

    def _validate_single_edge_pruning(
        self,
        candidate: Edge,
        positions: np.ndarray,
        edge_lengths: Dict[Edge, float],
        first_time: bool = False
    ) -> Dict[str, object]:
        """Run limited consensus rounds to validate a specific edge can be pruned.

        Args:
            first_time: If True, load fresh state. If False, continue from previous state.
        """
        # If this is the first validation for this candidate, load fresh state
        if first_time:
            self.consensus.load_external_state(positions, edge_lengths)

        # Run a few consensus steps (continue from current state)
        removed_edge = None
        for _ in range(self.consensus_rounds_per_step):
            report = self.consensus.step()
            if report.get('decision') == 'prune':
                removed_edge = report.get('removed_edge')
                break

        # Check if the candidate edge was removed
        agreed = (removed_edge == candidate)

        return {
            'agreed': agreed,
            'removed_edge': removed_edge,
            'report': report
        }

    # ------------------------------------------------------------------ #
    # Main simulation step with hybrid logic
    # ------------------------------------------------------------------ #
    def step(self) -> Dict[str, Optional[object]]:
        """Execute one simulation step with hybrid consensus pruning."""
        # STEP 1: Update connectivity tree
        self._build_connectivity_tree()

        # STEP 2: Compute and apply CLF-CBF control for each robot (ROBOTS MOVE)
        for robot_id, robot in enumerate(self.robots):
            control_force = self._compute_clf_cbf_control(robot_id)
            robot.step(self.flow, self.dt, self.time, self.workspace, control_force)

        self.time += self.dt

        # STEP 3: Detect topology changes
        old_edges = set(self.current_edges())
        self._update_neighbor_graph()
        new_edges = set(self.current_edges())

        edges_broken = old_edges - new_edges
        edges_formed = new_edges - old_edges

        topology_changed = len(edges_broken) > 0 or len(edges_formed) > 0

        # STEP 4: Handle topology changes (reset consensus if topology changed)
        if topology_changed:
            self.topology_stable_rounds = 0
            # Clear failed candidates - topology changed so previous failures may no longer apply
            self.failed_candidates.clear()

            if self.consensus_active:
                if self.verbose:
                    print(f"[t={self.time:5.2f}] Topology changed! "
                          f"Broken: {edges_broken}, Formed: {edges_formed}")
                    print(f"  Aborting consensus (was at round {self.consensus_round})")
                self.consensus_active = False
                self.consensus_candidate = None
                self.consensus_round = 0
            return {
                "decision": "topology_unstable",
                "edges_broken": sorted(edges_broken),
                "edges_formed": sorted(edges_formed),
                "stable_rounds": 0
            }
        else:
            self.topology_stable_rounds += 1

        # STEP 5: Wait for topology to stabilize
        if self.topology_stable_rounds < self.stability_threshold:
            return {
                "decision": "waiting_for_stability",
                "stable_rounds": self.topology_stable_rounds,
                "threshold": self.stability_threshold
            }

        # STEP 6: Add to rolling window
        self.topology_history.append({
            'edges': new_edges,
            'lengths': self._gather_edge_lengths(),
            'timestamp': self.time
        })
        if len(self.topology_history) > self.rolling_window_size:
            self.topology_history.pop(0)

        # STEP 7: If not enough history, wait
        if len(self.topology_history) < self.rolling_window_size:
            return {
                "decision": "building_history",
                "history_size": len(self.topology_history),
                "needed": self.rolling_window_size
            }

        # STEP 8: Find persistently redundant edge (if not already in consensus)
        if not self.consensus_active:
            candidate = self._find_persistent_redundant_edge(debug=True)

            if candidate is None:
                return {"decision": "no_persistent_redundant_edges"}

            # Start consensus on this candidate
            self.consensus_active = True
            self.consensus_candidate = candidate
            self.consensus_round = 0
            if self.verbose:
                print(f"[t={self.time:5.2f}] Starting consensus on edge {candidate}")

        # STEP 9: Run DISTRIBUTED consensus with dynamic position updates
        positions = np.array([r.position for r in self.robots], dtype=float)
        edge_lengths = self._gather_edge_lengths()

        # If first round for this candidate, initialize consensus state
        if self.consensus_round == 0:
            self.consensus.load_external_state(positions, edge_lengths)
        else:
            # Update positions and edge lengths to handle robot drift
            # This is realistic - in real deployment, robots sense updated distances
            self.consensus.positions = positions
            for edge in list(self.consensus.edge_lengths.keys()):
                if edge in edge_lengths:
                    self.consensus.edge_lengths[edge] = edge_lengths[edge]

        # Run distributed consensus steps
        removed_edge = None
        for _ in range(self.consensus_rounds_per_step):
            report = self.consensus.step()
            if report.get('decision') == 'prune':
                removed_edge = report.get('removed_edge')
                break

        self.consensus_round += self.consensus_rounds_per_step

        # STEP 10: Check if distributed consensus reached agreement
        # Consensus can succeed in just 1-3 rounds, so check immediately!
        if removed_edge == self.consensus_candidate:
            # SUCCESS! Distributed consensus agreed to prune our candidate
            # Final safety check: is topology STILL the same?
            final_edges = set(self.current_edges())
            if final_edges == new_edges:
                # Safe to prune!
                edge_to_prune = self.consensus_candidate
                a, b = edge_to_prune

                # Remove edge
                if b in self.neighbor_graph[a]:
                    self.neighbor_graph[a].discard(b)
                if a in self.neighbor_graph[b]:
                    self.neighbor_graph[b].discard(a)
                self.pruned_edges.add(edge_to_prune)
                self.robots[a].removed_edges.append(edge_to_prune)
                self.robots[b].removed_edges.append((b, a))

                # Reset state
                self.consensus_active = False
                self.consensus_candidate = None
                self.consensus_round = 0
                self.topology_stable_rounds = 0  # Reset after pruning
                self.topology_history.clear()  # Clear history for fresh start
                self.failed_candidates.clear()  # Clear blacklist after successful prune

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
                # Topology changed during final check, abort!
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
            # Consensus rounds completed but NO AGREEMENT - edge is NOT redundant
            # Give up on this candidate and look for others
            failed_edge = self.consensus_candidate

            if self.verbose:
                print(f"[t={self.time:5.2f}] NO CONSENSUS on {failed_edge} after {self.consensus_round} rounds")
                print(f"  Edge is NOT redundant, blacklisting and searching for other candidates")

            # Add to blacklist so we don't try it again
            self.failed_candidates.add(failed_edge)

            # Reset consensus but keep topology history
            self.consensus_active = False
            self.consensus_candidate = None
            self.consensus_round = 0
            # Don't reset topology_stable_rounds - we're still stable
            # Don't clear topology_history - we can use it to find another edge

            return {
                "decision": "consensus_failed",
                "reason": "edge_not_redundant",
                "failed_candidate": failed_edge
            }

        # Still evaluating
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


# ---------------------------------------------------------------------- #
# Visualisation
# ---------------------------------------------------------------------- #
class HybridAnimator:
    def __init__(self, sim: HybridUnderwaterSimulation, interval_ms: int = 100) -> None:
        self.sim = sim
        self.interval = interval_ms

        self.fig, self.ax = plt.subplots(figsize=(10, 5))
        self.ax.set_xlim(0.0, sim.workspace[0])
        self.ax.set_ylim(0.0, sim.workspace[1])
        self.ax.set_title("Hybrid Consensus Pruning (Multi-Layer Robustness)", fontsize=12, fontweight='bold')
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")

        # Draw goal position
        self.goal_marker = self.ax.scatter(
            *sim.goal_position, s=400, c='gold', marker='*',
            edgecolors='darkorange', linewidths=2.0, zorder=10, label='Goal'
        )

        goal_circle = plt.Circle(
            sim.goal_position, 0.5, fill=False, color='gold',
            linestyle='--', alpha=0.5, linewidth=2
        )
        self.ax.add_patch(goal_circle)

        self.node_scatter = self.ax.scatter(
            [], [], s=80, c="#e74c3c", edgecolors="white", linewidths=1.0
        )
        self.labels: List[plt.Text] = []

        # Sparser, lighter flow field visualization
        grid_x = np.linspace(0.0, sim.workspace[0], 10)  # Reduced from 16 to 10
        grid_y = np.linspace(0.0, sim.workspace[1], 10)  # Reduced from 16 to 10
        gx, gy = np.meshgrid(grid_x, grid_y)
        self.flow_grid_points = np.column_stack((gx.ravel(), gy.ravel()))
        initial_flow = np.array(
            [self.sim.flow.velocity(p, self.sim.time) for p in self.flow_grid_points]
        )
        self.quiver = self.ax.quiver(
            self.flow_grid_points[:, 0],
            self.flow_grid_points[:, 1],
            initial_flow[:, 0],
            initial_flow[:, 1],
            color="#B0C4DE",  # Lighter blue (LightSteelBlue)
            alpha=0.4,  # More transparent (was 0.7)
            width=0.0015,  # Thinner arrows (was 0.002)
            scale=2.0,  # Smaller arrows (was 1.5)
            zorder=0,  # Behind everything
        )

        # Thicker, more visible edges with proper z-ordering
        self.active_lines = LineCollection([], colors="#9b59b6", linewidths=2.5, zorder=3)  # Thicker purple
        self.highlight_lines = LineCollection([], colors="#e74c3c", linewidths=4.5, zorder=5)  # Thicker red
        self.candidate_lines = LineCollection([], colors="#f39c12", linewidths=3.5, linestyles='dashdot', zorder=4)  # Thicker orange

        self.ax.add_collection(self.active_lines)
        self.ax.add_collection(self.candidate_lines)
        self.ax.add_collection(self.highlight_lines)

        self.status = self.ax.text(
            0.02,
            0.98,
            "",
            transform=self.ax.transAxes,
            ha="left",
            va="top",
            fontsize=9,
            family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="white", alpha=0.9),
        )

        # Create custom legend
        from matplotlib.lines import Line2D
        from matplotlib.patches import Patch

        legend_elements = [
            Line2D([0], [0], color='#9b59b6', linewidth=2.5, label='Active Edges'),
            Line2D([0], [0], color='#f39c12', linewidth=3.5, linestyle='dashdot', label='Candidate Edge'),
            Line2D([0], [0], color='#e74c3c', linewidth=4.5, label='Recently Pruned'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#e74c3c',
                   markersize=8, label='Connected Robot'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#ff8c00',
                   markersize=8, label='Isolated Robot'),
        ]
        self.ax.legend(handles=legend_elements, loc='upper right', fontsize=7, framealpha=0.9)

    def _build_segments(self, edges: Iterable[Edge]) -> List[np.ndarray]:
        positions = [robot.position for robot in self.sim.robots]
        segments: List[np.ndarray] = []
        for a, b in edges:
            segments.append(np.vstack((positions[a], positions[b])))
        return segments

    def _update_plot(self, _frame: int):
        report = self.sim.step()
        positions = np.array([robot.position for robot in self.sim.robots])

        # Color robots based on connectivity tree status
        # RED: In connectivity tree (connected to root)
        # ORANGE: Isolated from tree (no path to root through current edges)
        colors = []
        for robot in self.sim.robots:
            if robot.parent is None and robot.robot_id != 0:
                # No parent AND not the root = disconnected from tree
                colors.append('#ff8c00')  # Dark orange for isolated robots
            else:
                # Has parent OR is root = in connectivity tree
                colors.append('#e74c3c')  # Red for connected robots

        self.node_scatter.set_offsets(positions)
        self.node_scatter.set_color(colors)

        if not self.labels:
            for idx, (x, y) in enumerate(positions):
                self.labels.append(
                    self.ax.text(x, y + 0.18, str(idx), ha="center", fontsize=9)
                )
        else:
            for label, (x, y) in zip(self.labels, positions):
                label.set_position((x, y + 0.18))

        self.active_lines.set_segments(self._build_segments(self.sim.current_edges()))

        # Highlight candidate edge if in consensus
        if self.sim.consensus_active and self.sim.consensus_candidate:
            self.candidate_lines.set_segments(self._build_segments([self.sim.consensus_candidate]))
        else:
            self.candidate_lines.set_segments([])

        # Highlight recently pruned edge
        if report.get("decision") == "prune":
            self.highlight_lines.set_segments(self._build_segments([report["removed_edge"]]))
        else:
            self.highlight_lines.set_segments([])

        # Calculate metrics
        avg_distance_to_goal = np.mean([
            np.linalg.norm(robot.position - self.sim.goal_position)
            for robot in self.sim.robots
        ])

        robots_at_goal = sum(
            1 for robot in self.sim.robots
            if np.linalg.norm(robot.position - self.sim.goal_position) < 0.5
        )

        # Count isolated robots (not in connectivity tree)
        isolated_count = sum(
            1 for robot in self.sim.robots
            if robot.parent is None and robot.robot_id != 0
        )

        # Build status text
        status_lines = [
            f"Time: {self.sim.time:4.2f}s | Edges: {len(self.sim.current_edges())} | Pruned: {len(self.sim.pruned_edges)}",
            f"Isolated: {isolated_count} | Avg dist to goal: {avg_distance_to_goal:.2f}m",
            f"Robots at goal: {robots_at_goal}/{self.sim.num_robots}",
            "",
            "NOTE: Isolated = No path to root in connectivity tree",
            "      (May still have edges, but not connected to root)",
            "─" * 50,
        ]

        decision = report.get("decision")
        if decision == "topology_unstable":
            status_lines.append(f"[!] TOPOLOGY UNSTABLE")
            status_lines.append(f"  Broken: {report.get('edges_broken', [])}")
            status_lines.append(f"  Formed: {report.get('edges_formed', [])}")
        elif decision == "waiting_for_stability":
            status_lines.append(f"[~] Waiting for stability: {report['stable_rounds']}/{report['threshold']}")
        elif decision == "building_history":
            status_lines.append(f"[+] Building history: {report['history_size']}/{report['needed']}")
        elif decision == "no_persistent_redundant_edges":
            status_lines.append(f"[OK] No persistent redundant edges")
        elif decision == "evaluating":
            status_lines.append(f"[?] Evaluating: {report['candidate']}")
            status_lines.append(f"  Rounds: {report['consensus_round']}/{report['needed_rounds']}")
            status_lines.append(f"  Redundant: {report.get('is_redundant', 'checking...')}")
        elif decision == "consensus_failed":
            status_lines.append(f"[X] FAILED: {report['failed_candidate']} NOT redundant")
            status_lines.append(f"  Searching for other candidates...")
        elif decision == "prune":
            status_lines.append(f"[PRUNED] {report['removed_edge']}")
        elif decision == "abort_topology_changed":
            status_lines.append(f"[X] ABORTED: Topology changed during consensus")

        self.status.set_text("\n".join(status_lines))

        flow_vectors = np.array(
            [self.sim.flow.velocity(p, self.sim.time) for p in self.flow_grid_points]
        )
        self.quiver.set_UVC(flow_vectors[:, 0], flow_vectors[:, 1])

        return (
            self.node_scatter,
            self.active_lines,
            self.candidate_lines,
            self.highlight_lines,
            self.status,
            *self.labels,
        )

    def _init_plot(self):
        positions = np.array([robot.position for robot in self.sim.robots])
        self.node_scatter.set_offsets(positions)
        self.labels = []
        for idx, (x, y) in enumerate(positions):
            self.labels.append(
                self.ax.text(x, y + 0.18, str(idx), ha="center", fontsize=9)
            )
        self.active_lines.set_segments(self._build_segments(self.sim.current_edges()))
        self.candidate_lines.set_segments([])
        self.highlight_lines.set_segments([])

        self.status.set_text("Initializing hybrid consensus...")

        flow_vectors = np.array(
            [self.sim.flow.velocity(p, self.sim.time) for p in self.flow_grid_points]
        )
        self.quiver.set_UVC(flow_vectors[:, 0], flow_vectors[:, 1])
        return (
            self.node_scatter,
            self.active_lines,
            self.candidate_lines,
            self.highlight_lines,
            self.status,
            *self.labels,
        )

    def animate(self) -> FuncAnimation:
        animation = FuncAnimation(
            self.fig,
            self._update_plot,
            interval=self.interval,
            blit=False,
            repeat=False,
            cache_frame_data=False,
            init_func=self._init_plot,
        )
        self._animation = animation
        return animation


# ---------------------------------------------------------------------- #
# CLI entry point
# ---------------------------------------------------------------------- #
def run_text(num_robots: int, steps: int, verbose: bool) -> None:
    sim = HybridUnderwaterSimulation(
        num_robots=num_robots, communication_radius=2.5, verbose=verbose
    )
    print(f"Goal position: ({sim.goal_position[0]:.2f}, {sim.goal_position[1]:.2f})")
    print(f"Starting HYBRID simulation with {num_robots} robots")
    print(f"Stability threshold: {sim.stability_threshold} steps")
    print(f"Rolling window: {sim.rolling_window_size} snapshots")
    print(f"Consensus rounds per step: {sim.consensus_rounds_per_step}")
    print()

    for step_index in range(steps):
        report = sim.step()

        if step_index % 5 == 0 or report.get("decision") in ["prune", "abort_topology_changed", "topology_unstable"]:
            decision = report.get("decision", "none")
            print(f"[Step {step_index+1:03d}] t={sim.time:.2f}s | Decision: {decision}")
            if decision == "prune":
                print(f"  ✓ Pruned: {report['removed_edge']}, Edges left: {report['edges_remaining']}")
            elif decision == "topology_unstable":
                print(f"  ⚠ Broken: {report.get('edges_broken')}, Formed: {report.get('edges_formed')}")

    print("\n" + "="*60)
    print(f"Final pruned edges: {sorted(sim.pruned_edges)}")
    print(f"Final edge count: {len(sim.current_edges())}")


def run_gui(num_robots: int, verbose: bool) -> None:
    sim = HybridUnderwaterSimulation(
        num_robots=num_robots, communication_radius=2.5, verbose=verbose
    )
    print(f"Goal position: ({sim.goal_position[0]:.2f}, {sim.goal_position[1]:.2f})")
    print(f"Starting HYBRID GUI with {num_robots} robots")
    print(f"Multi-layer robustness enabled:")
    print(f"  - Topology stability detection")
    print(f"  - Rolling window validation ({sim.rolling_window_size} snapshots)")
    print(f"  - Interleaved consensus ({sim.consensus_rounds_per_step} rounds/step)")
    print(f"  - Final safety checks before pruning")
    print()

    animator = HybridAnimator(sim, interval_ms=100)
    anim = animator.animate()
    plt.tight_layout()
    plt.show()
    return anim


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Hybrid underwater consensus pruning demo")
    parser.add_argument(
        "mode",
        choices={"text", "gui"},
        nargs="?",
        default="gui",
        help="text: console log, gui: interactive animation",
    )
    parser.add_argument(
        "--robots", type=int, default=8, help="number of robots (>=3, default 8)"
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=100,
        help="simulation steps for text mode (default 100)",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="suppress per-step console messages",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    num_robots = max(3, args.robots)
    verbose = not args.quiet
    if args.mode == "gui":
        run_gui(num_robots, verbose)
    else:
        run_text(num_robots, steps=args.steps, verbose=verbose)


if __name__ == "__main__":
    main()
