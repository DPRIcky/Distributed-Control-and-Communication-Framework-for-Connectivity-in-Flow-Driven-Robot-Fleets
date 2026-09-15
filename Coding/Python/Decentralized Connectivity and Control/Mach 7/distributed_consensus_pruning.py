"""Distributed redundant edge pruning on random connected graphs.

This module implements the communication and topology-management idea
described in the user specification: Every iteration draws a fresh connected
graph with 5-20 nodes, robots learn about their neighborhood hop-by-hop, and
when a cycle is detected they immediately decide whether to prune the longer
edge. Only one candidate edge is processed per round; every other edge is
treated as critical during that decision. Once the edge is removed the updated
knowledge is retained and subsequent rounds continue until no further pruning is
possible.
"""


from __future__ import annotations

import math
import heapq
from typing import Dict, Iterable, List, Optional, Set, Tuple

import numpy as np

Edge = Tuple[int, int]


def _edge_key(a: int, b: int) -> Edge:
    """Return a canonical undirected edge representation."""
    return (a, b) if a < b else (b, a)


class ConsensusPruningSimulation:
    """
    Manage multi-hop knowledge propagation and length-based edge pruning.

    Usage pattern:
        sim = ConsensusPruningSimulation()
        sim.start_new_iteration()
        history = sim.run_until_converged()
    """

    def __init__(
        self,
        min_nodes: int = 5,
        max_nodes: int = 20,
        num_nodes: Optional[int] = None,
        rng: Optional[np.random.Generator] = None,
        verbose: bool = False,
    ) -> None:
        if num_nodes is not None:
            if num_nodes < 3:
                raise ValueError("Number of nodes must be at least 3.")
            self.min_nodes = num_nodes
            self.max_nodes = num_nodes
            self.fixed_num_nodes: Optional[int] = num_nodes
        else:
            if min_nodes < 3 or max_nodes < min_nodes:
                raise ValueError("Node bounds must satisfy 3 <= min_nodes <= max_nodes.")
            self.min_nodes = min_nodes
            self.max_nodes = max_nodes
            self.fixed_num_nodes = None
        self.rng = rng or np.random.default_rng()
        self.verbose = verbose

        self.nodes: List[int] = []
        self.positions: Optional[np.ndarray] = None
        self.graph: Dict[int, Set[int]] = {}
        self.edge_lengths: Dict[Edge, float] = {}
        self.initial_edge_lengths: Dict[Edge, float] = {}
        self.knowledge: Dict[int, Dict[Edge, float]] = {}
        self.knowledge_round: int = 0
        self.completed_prunes: List[Edge] = []
        self.processed_edges: Set[Edge] = set()
        self.local_candidates: List[Optional[Dict[str, object]]] = []
        self.proposal_messages: List[Optional[Dict[str, object]]] = []
        self.proposal_history: Dict[Tuple[int, Edge], Dict[str, int]] = {}

    # ------------------------------------------------------------------ #
    # Random graph generation
    # ------------------------------------------------------------------ #
    def set_verbose(self, enabled: bool) -> None:
        """Toggle real-time console logging."""
        self.verbose = enabled

    def _add_edge(self, a: int, b: int) -> None:
        if a == b:
            return
        edge = _edge_key(a, b)
        if edge in self.edge_lengths:
            return
        distance = float(np.linalg.norm(self.positions[a] - self.positions[b]))
        self.graph[a].add(b)
        self.graph[b].add(a)
        self.edge_lengths[edge] = distance

    def _generate_positions(self, n: int) -> np.ndarray:
        return self.rng.random((n, 2))

    def _build_random_connected_graph(self) -> None:
        if self.fixed_num_nodes is not None:
            n_nodes = self.fixed_num_nodes
        else:
            n_nodes = int(self.rng.integers(self.min_nodes, self.max_nodes + 1))
        self.nodes = list(range(n_nodes))
        self.num_robots = n_nodes
        self.positions = self._generate_positions(n_nodes)
        self.graph = {node: set() for node in self.nodes}
        self.edge_lengths = {}
        self.initial_edge_lengths = {}
        self.knowledge = {}
        self.completed_prunes = []

        # Random spanning tree for connectivity.
        for node in self.nodes[1:]:
            parent = int(self.rng.integers(0, node))
            self._add_edge(node, parent)

        # Sprinkle additional edges to create cycles.
        potential_edges: List[Edge] = []
        for i in self.nodes:
            for j in range(i + 1, n_nodes):
                if j not in self.graph[i]:
                    potential_edges.append((i, j))
        self.rng.shuffle(potential_edges)
        target_extra = max(1, n_nodes // 2)
        added = 0
        for u, v in potential_edges:
            if added >= target_extra:
                break
            if self.rng.random() < 0.5:
                self._add_edge(u, v)
                added += 1

        if added == 0 and potential_edges:
            self._add_edge(*potential_edges[0])

        self.initial_edge_lengths = dict(self.edge_lengths)
        self._prime_local_knowledge()
        self.local_candidates = [None for _ in self.nodes]
        self.proposal_messages = [None for _ in self.nodes]
        self.proposal_history = {}
        if self.verbose:
            print(
                f"[init] generated graph with {n_nodes} nodes, "
                f"{len(self.edge_lengths)} edges "
                f"(extra edges added: {added})"
            )

    def _prime_local_knowledge(self) -> None:
        self.knowledge = {}
        for node in self.nodes:
            local = {}
            for neighbor in self.graph[node]:
                edge = _edge_key(node, neighbor)
                local[edge] = self.edge_lengths[edge]
            self.knowledge[node] = local
        self.knowledge_round = 0
        self.processed_edges = set()
        self.local_candidates = [None for _ in self.nodes]
        self.proposal_messages = [None for _ in self.nodes]
        self.proposal_history = {}

    # ------------------------------------------------------------------ #
    # Knowledge propagation and detection
    # ------------------------------------------------------------------ #
    def _propagate_one_hop(self) -> bool:
        """Each robot shares its current knowledge with one-hop neighbors."""
        learned = False
        snapshot = {node: dict(edges) for node, edges in self.knowledge.items()}
        updated = {node: dict(edges) for node, edges in self.knowledge.items()}
        for node in self.nodes:
            combined = updated[node]
            for neighbor in self.graph[node]:
                for edge, length in snapshot[neighbor].items():
                    if edge not in combined:
                        combined[edge] = length
                        learned = True
        self.knowledge = updated
        return learned

    def _build_local_adjacency(
        self, edges: Dict[Edge, float]
    ) -> Dict[int, Dict[int, float]]:
        adjacency: Dict[int, Dict[int, float]] = {}
        for (a, b), length in edges.items():
            adjacency.setdefault(a, {})[b] = length
            adjacency.setdefault(b, {})[a] = length
        return adjacency

    def _shortest_path_without_edge(
        self,
        adjacency: Dict[int, Dict[int, float]],
        start: int,
        goal: int,
        forbidden: Edge,
    ) -> Optional[Tuple[float, List[Edge], List[float]]]:
        if start not in adjacency or goal not in adjacency:
            return None
        block = _edge_key(*forbidden)
        distances: Dict[int, float] = {start: 0.0}
        parents: Dict[int, Optional[int]] = {start: None}
        frontier: List[Tuple[float, int]] = [(0.0, start)]
        while frontier:
            dist, node = heapq.heappop(frontier)
            if dist > distances.get(node, math.inf) + 1e-9:
                continue
            if node == goal:
                path_edges: List[Edge] = []
                path_lengths: List[float] = []
                current = node
                while parents[current] is not None:
                    prev = parents[current]
                    edge = _edge_key(current, prev)
                    path_edges.append(edge)
                    weight = adjacency[prev][current]
                    path_lengths.append(weight)
                    current = prev
                path_edges.reverse()
                path_lengths.reverse()
                return dist, path_edges, path_lengths
            for nbr, weight in adjacency.get(node, {}).items():
                edge = _edge_key(node, nbr)
                if edge == block:
                    continue
                candidate = dist + weight
                if candidate + 1e-9 < distances.get(nbr, math.inf):
                    distances[nbr] = candidate
                    parents[nbr] = node
                    heapq.heappush(frontier, (candidate, nbr))
        return None

    def _nan_min(self, a: Optional[float], b: Optional[float]) -> Optional[float]:
        if a is None or (isinstance(a, float) and math.isnan(a)):
            return b
        if b is None or (isinstance(b, float) and math.isnan(b)):
            return a
        return min(a, b)

    def _is_nan(self, value: Optional[float]) -> bool:
        return value is None or (isinstance(value, float) and math.isnan(value))

    def _update_message_priority(self, message: Dict[str, object]) -> None:
        direct = float(message.get("direct_length", 0.0))
        alt_candidates: List[float] = []
        alt_max = message.get("alternative_max")
        alt_path = message.get("alternative_path")
        if not self._is_nan(alt_max):
            alt_candidates.append(float(alt_max))
        if not self._is_nan(alt_path):
            alt_candidates.append(float(alt_path))
        alt_value = min(alt_candidates) if alt_candidates else direct
        margin = direct - alt_value
        support_size = len(message.get("support", []))
        message["priority"] = (direct, margin, support_size, -message["proposer"])

    def _create_message_from_candidate(self, robot_id: int, candidate: Dict[str, object]) -> Dict[str, object]:
        message = {
            "proposer": robot_id,
            "edge": candidate["edge"],
            "direct_length": candidate["direct_length"],
            "alternative_path": candidate.get("alternative_path"),
            "alternative_max": candidate.get("alternative_max"),
            "support": {robot_id},
        }
        self._update_message_priority(message)
        return message

    def _clone_message(self, message: Dict[str, object]) -> Dict[str, object]:
        cloned = {
            "proposer": message["proposer"],
            "edge": message["edge"],
            "direct_length": message.get("direct_length"),
            "alternative_path": message.get("alternative_path"),
            "alternative_max": message.get("alternative_max"),
            "support": set(message.get("support", [])),
        }
        self._update_message_priority(cloned)
        return cloned

    def _same_proposal(self, msg_a: Dict[str, object], msg_b: Dict[str, object]) -> bool:
        return msg_a.get("proposer") == msg_b.get("proposer") and msg_a.get("edge") == msg_b.get("edge")

    def _compare_messages(self, msg_a: Dict[str, object], msg_b: Dict[str, object]) -> int:
        if msg_a["priority"] > msg_b["priority"]:
            return 1
        if msg_a["priority"] < msg_b["priority"]:
            return -1
        return 0

    def _compute_local_candidate(
        self,
        robot_id: int,
        adjacency: Dict[int, Dict[int, float]],
    ) -> Optional[Dict[str, object]]:
        local_edges = self.knowledge.get(robot_id, {})
        best_candidate: Optional[Dict[str, object]] = None
        for edge, direct_length in local_edges.items():
            if edge not in self.edge_lengths:
                continue
            result = self._shortest_path_without_edge(adjacency, edge[0], edge[1], edge)
            if result is None:
                continue
            path_length, _, path_lengths = result
            if not path_lengths:
                continue
            alt_max = max(path_lengths)
            alt_value = alt_max if not self._is_nan(alt_max) else path_length
            # MODIFIED: Remove length comparison - prune ANY redundant edge (with alternative path)
            # Original: if alt_value + 1e-6 >= direct_length: continue
            # Now we prune based on redundancy (alternative exists), not length optimization
            margin = direct_length - alt_value if alt_value < direct_length else 0.0
            candidate = {
                "edge": edge,
                "direct_length": direct_length,
                "alternative_path": path_length,
                "alternative_max": alt_max,
                "priority": (direct_length, margin, 1, -robot_id),
            }
            if best_candidate is None or candidate["priority"] > best_candidate["priority"]:
                best_candidate = candidate
        return best_candidate

    def _evaluate_edge_for_robot(
        self,
        robot_id: int,
        edge: Edge,
        adjacency: Dict[int, Dict[int, float]],
    ) -> Tuple[bool, Optional[float], Optional[float]]:
        if edge not in self.knowledge.get(robot_id, {}):
            return False, None, None
        result = self._shortest_path_without_edge(adjacency, edge[0], edge[1], edge)
        if result is None:
            return False, None, None
        path_length, _, path_lengths = result
        if not path_lengths:
            return False, path_length, None
        alt_max = max(path_lengths)
        return True, path_length, alt_max

    def _select_best_message_for_robot(
        self,
        robot_id: int,
        adjacency: Dict[int, Dict[int, float]],
    ) -> Optional[Dict[str, object]]:
        candidates: List[Dict[str, object]] = []
        local_candidate = self.local_candidates[robot_id]
        if local_candidate is not None:
            candidates.append(self._create_message_from_candidate(robot_id, local_candidate))
        if self.proposal_messages and self.proposal_messages[robot_id] is not None:
            candidates.append(self._clone_message(self.proposal_messages[robot_id]))
        for neighbor in self.graph[robot_id]:
            if self.proposal_messages and self.proposal_messages[neighbor] is not None:
                candidates.append(self._clone_message(self.proposal_messages[neighbor]))

        best_message: Optional[Dict[str, object]] = None
        for message in candidates:
            if best_message is None:
                best_message = message
                continue
            comparison = self._compare_messages(message, best_message)
            if comparison > 0:
                best_message = message
            elif comparison == 0 and self._same_proposal(message, best_message):
                best_message["support"].update(message.get("support", []))
                best_message["alternative_path"] = self._nan_min(
                    best_message.get("alternative_path"), message.get("alternative_path")
                )
                best_message["alternative_max"] = self._nan_min(
                    best_message.get("alternative_max"), message.get("alternative_max")
                )
                self._update_message_priority(best_message)

        if best_message is None:
            return None

        feasible, alt_path, alt_max = self._evaluate_edge_for_robot(robot_id, best_message["edge"], adjacency)
        if feasible:
            if robot_id not in best_message["support"]:
                best_message["support"].add(robot_id)
            best_message["alternative_path"] = self._nan_min(best_message.get("alternative_path"), alt_path)
            best_message["alternative_max"] = self._nan_min(best_message.get("alternative_max"), alt_max)
            self._update_message_priority(best_message)
        else:
            if robot_id in best_message["support"]:
                best_message["support"].discard(robot_id)
            self._update_message_priority(best_message)
        return best_message

    def _apply_edge_removal(self, edge: Edge) -> None:
        """Remove an edge from the graph and update knowledge caches."""
        a, b = edge
        if b in self.graph[a]:
            self.graph[a].remove(b)
        if a in self.graph[b]:
            self.graph[b].remove(a)
        self.edge_lengths.pop(edge, None)
        for node in self.nodes:
            self.knowledge[node].pop(edge, None)
        self.completed_prunes.append(edge)
        self.processed_edges.add(edge)
        self.proposal_history = {
            key: value for key, value in self.proposal_history.items() if key[1] != edge
        }
        if self.verbose:
            print(
                f"[apply] removed {edge}; remaining edges={len(self.edge_lengths)} "
                f"removed total={len(self.completed_prunes)}"
            )

    # ------------------------------------------------------------------ #
    # Public iteration interface
    # ------------------------------------------------------------------ #
    def start_new_iteration(self) -> None:
        self._build_random_connected_graph()

    def step(self) -> Dict[str, Optional[object]]:
        if not self.nodes:
            raise RuntimeError("No active graph. Call start_new_iteration() first.")

        self.knowledge_round += 1
        report: Dict[str, Optional[object]] = {
            "round": self.knowledge_round,
            "knowledge_updated": None,
            "proposal": None,
            "removed_edge": None,
            "decision": None,
            "affected_nodes": [],
        }

        knowledge_updated = self._propagate_one_hop()
        report["knowledge_updated"] = knowledge_updated

        adjacency_cache = [
            self._build_local_adjacency(self.knowledge.get(robot, {}))
            for robot in self.nodes
        ]

        self.local_candidates = [
            self._compute_local_candidate(robot, adjacency_cache[robot])
            for robot in self.nodes
        ]

        if self.verbose:
            for robot, candidate in enumerate(self.local_candidates):
                if candidate is None:
                    continue
                alt_max = candidate.get("alternative_max")
                alt_path = candidate.get("alternative_path")
                alt_max_str = "nan" if self._is_nan(alt_max) else f"{float(alt_max):.3f}"
                alt_path_str = "nan" if self._is_nan(alt_path) else f"{float(alt_path):.3f}"
                print(
                    f"[round {self.knowledge_round}] robot {robot} candidate {candidate['edge']} "
                    f"(len={candidate['direct_length']:.3f}, alt_max={alt_max_str}, alt_sum={alt_path_str})"
                )

        new_messages: List[Optional[Dict[str, object]]] = [None for _ in self.nodes]
        tracking: Dict[Tuple[int, Edge], Dict[str, object]] = {}

        for robot in self.nodes:
            best_message = self._select_best_message_for_robot(robot, adjacency_cache[robot])
            new_messages[robot] = best_message
            if best_message is None:
                continue
            key = (best_message["proposer"], best_message["edge"])
            entry = tracking.setdefault(
                key,
                {
                    "message": best_message,
                    "supporters": set(),
                    "priority": best_message["priority"],
                    "alternative_path": best_message.get("alternative_path"),
                    "alternative_max": best_message.get("alternative_max"),
                },
            )
            entry["supporters"].update(best_message.get("support", set()))
            entry["priority"] = best_message["priority"]
            entry["message"] = best_message
            entry["alternative_path"] = self._nan_min(
                entry.get("alternative_path"), best_message.get("alternative_path")
            )
            entry["alternative_max"] = self._nan_min(
                entry.get("alternative_max"), best_message.get("alternative_max")
            )

        active_keys = set(tracking.keys())
        self.proposal_history = {
            key: value for key, value in self.proposal_history.items() if key in active_keys
        }

        best_commit: Optional[Tuple[Tuple[int, Edge], Dict[str, object]]] = None
        best_commit_priority: Optional[Tuple[float, float, int, int]] = None

        for key, data in tracking.items():
            support_size = len(data["supporters"])
            history = self.proposal_history.get(key)
            if history is None:
                history = {"support_size": support_size, "stable_rounds": 1}
            else:
                if history["support_size"] == support_size:
                    history["stable_rounds"] += 1
                else:
                    history = {"support_size": support_size, "stable_rounds": 1}
            self.proposal_history[key] = history

            if self.verbose:
                print(
                    f"[round {self.knowledge_round}] proposal {(key[0], key[1])} "
                    f"support={support_size} stable={history['stable_rounds']}"
                )

            proposer, edge = key
            candidate = self.local_candidates[proposer]
            if candidate is None or candidate["edge"] != edge:
                continue
            # Reduced to 1 stable round for dynamic topologies
            if history["stable_rounds"] >= 1 and support_size > 0:
                priority = data["priority"]
                if best_commit_priority is None or priority > best_commit_priority:
                    best_commit = (key, data)
                    best_commit_priority = priority

        if best_commit is not None:
            key, data = best_commit
            proposer, edge = key
            supporters = sorted(data["supporters"])
            message = data["message"]
            if self.verbose:
                print(
                    f"[round {self.knowledge_round}] robot {proposer} consensus reached; pruning {edge}"
                )
            self._apply_edge_removal(edge)
            report["proposal"] = {
                "edge": edge,
                "length": message["direct_length"],
                "alternative_path": data.get("alternative_path"),
                "alternative_max": data.get("alternative_max"),
                "proposer": proposer,
                "supporters": supporters,
            }
            report["decision"] = "prune"
            report["removed_edge"] = edge
            report["affected_nodes"] = supporters
            self.proposal_messages = [None for _ in self.nodes]
            self.proposal_history.clear()
        else:
            self.proposal_messages = new_messages
            if tracking:
                best_key = max(tracking.keys(), key=lambda k: tracking[k]["priority"])
                data = tracking[best_key]
                message = data["message"]
                supporters = sorted(data["supporters"])
                report["proposal"] = {
                    "edge": message["edge"],
                    "length": message["direct_length"],
                    "alternative_path": data.get("alternative_path"),
                    "alternative_max": data.get("alternative_max"),
                    "proposer": message["proposer"],
                    "supporters": supporters,
                }
                report["decision"] = "evaluate"
                report["affected_nodes"] = supporters
            else:
                report["decision"] = "none"

        return report

    def _should_stop(self, report: Dict[str, Optional[object]]) -> bool:
        """Return True when no further pruning opportunities remain."""
        decision = report.get("decision")
        if decision in {"evaluate"}:
            return False
        if decision == "prune":
            return False
        if len(self.nodes) <= 1:
            return True
        if len(self.edge_lengths) <= len(self.nodes) - 1:
            return True
        knowledge_sizes = {len(edges) for edges in self.knowledge.values()}
        if len(knowledge_sizes) == 1 and not report.get("knowledge_updated"):
            return True
        return False

    def _capture_snapshot(self, report: Dict[str, Optional[object]]) -> Dict[str, object]:
        """Collect visualization-friendly data after each round."""
        if self.positions is None:
            raise RuntimeError("Robot positions are undefined; run start_new_iteration() first.")

        snapshot: Dict[str, object] = {
            "round": report["round"],
            "positions": np.array(self.positions, copy=True),
            "edges_active": list(sorted(self.edge_lengths.keys())),
            "edges_removed": list(self.completed_prunes),
            "focus_edge": None,
            "highlight_edge": None,
            "highlight_nodes": list(report.get("affected_nodes", []) or []),
            "recently_removed": report.get("removed_edge"),
            "knowledge_updated": bool(report.get("knowledge_updated")),
            "decision": report.get("decision"),
            "alternative_path": None,
            "alternative_max": None,
        }

        proposal_data = report.get("proposal")
        if proposal_data:
            focus_edge = proposal_data["edge"]
            snapshot["alternative_path"] = proposal_data.get("alternative_path")
            snapshot["alternative_max"] = proposal_data.get("alternative_max")
            snapshot["focus_edge"] = focus_edge
            if report.get("decision") != "prune":
                snapshot["highlight_edge"] = focus_edge

        if report.get("removed_edge") is not None:
            snapshot["highlight_edge"] = None
            snapshot["focus_edge"] = report["removed_edge"]

        return snapshot

    def load_external_state(
        self,
        positions: np.ndarray,
        edge_lengths: Dict[Edge, float],
    ) -> None:
        """Populate the internal state from an externally supplied graph."""
        num_nodes = positions.shape[0]
        self.nodes = list(range(num_nodes))
        self.num_robots = num_nodes
        self.positions = np.array(positions, dtype=float)
        normalized_edges = {
            _edge_key(*edge): float(length) for edge, length in edge_lengths.items()
        }
        self.edge_lengths = normalized_edges
        self.initial_edge_lengths = dict(normalized_edges)
        self.graph = {node: set() for node in self.nodes}
        for (a, b) in normalized_edges.keys():
            self.graph[a].add(b)
            self.graph[b].add(a)

        self._prime_local_knowledge()
        self.local_candidates = [None for _ in self.nodes]
        self.proposal_messages = [None for _ in self.nodes]
        self.proposal_history = {}

    def prune_graph_from_state(
        self,
        positions: np.ndarray,
        edge_lengths: Dict[Edge, float],
        max_rounds: int = 200,
    ) -> Tuple[List[Dict[str, Optional[object]]], Dict[Edge, float]]:
        """
        Run the consensus pruning protocol on an externally provided graph.

        Parameters
        ----------
        positions : np.ndarray
            Node layout (used only for reporting/visualisation).
        edge_lengths : Dict[Edge, float]
            Mapping of undirected edges to their lengths.
        max_rounds : int
            Maximum number of hop-based communication rounds to simulate.

        Returns
        -------
        Tuple[List[Dict[str, Optional[object]]], Dict[Edge, float]]
            Round-by-round reports and the final surviving edge set with lengths.
        """
        self.load_external_state(positions, edge_lengths)
        history, _ = self.run_with_snapshots(max_rounds=max_rounds)
        return history, dict(self.edge_lengths)

    def _simulate(
        self, max_rounds: int
    ) -> Iterable[Tuple[Dict[str, Optional[object]], Dict[str, object]]]:
        """Yield per-round reports and snapshots until convergence or limit."""
        for _ in range(max_rounds):
            report = self.step()
            snapshot = self._capture_snapshot(report)
            yield report, snapshot
            if self._should_stop(report):
                break

    def run_until_converged(
        self,
        max_rounds: int = 200,
    ) -> List[Dict[str, Optional[object]]]:
        """Execute synchronous rounds until no further pruning is possible."""
        history: List[Dict[str, Optional[object]]] = []
        for report, _ in self._simulate(max_rounds):
            history.append(report)
        return history

    def run_with_snapshots(
        self,
        max_rounds: int = 200,
    ) -> Tuple[
        List[Dict[str, Optional[object]]],
        List[Dict[str, object]],
    ]:
        """Run the protocol and return both reports and visualization snapshots."""
        history: List[Dict[str, Optional[object]]] = []
        snapshots: List[Dict[str, object]] = []
        for report, snapshot in self._simulate(max_rounds):
            history.append(report)
            snapshots.append(snapshot)
        return history, snapshots

    def run_batch(
        self,
        iterations: int,
        max_rounds: int = 200,
    ) -> Iterable[Dict[str, object]]:
        for _ in range(max(0, iterations)):
            self.start_new_iteration()
            history, _ = self.run_with_snapshots(max_rounds=max_rounds)
            yield {
                "num_nodes": len(self.nodes),
                "initial_edges": dict(self.initial_edge_lengths),
                "final_edges": dict(self.edge_lengths),
                "removed_edges": list(self.completed_prunes),
                "history": history,
            }


def animate_single_run(
    seed: Optional[int] = None,
    max_rounds: int = 200,
    interval_ms: int = 700,
    repeat: bool = False,
    figsize: Tuple[float, float] = (6.0, 6.0),
    verbose: bool = True,
    num_nodes: Optional[int] = None,
):
    """
    Launch a matplotlib animation visualizing one pruning iteration.

    Parameters
    ----------
    seed : Optional[int]
        Seed for the random generator controlling both graph construction and layout.
    max_rounds : int
        Maximum number of synchronous communication rounds to display.
    interval_ms : int
        Delay (in milliseconds) between animation frames.
    repeat : bool
        Whether the animation should loop after finishing.
    figsize : Tuple[float, float]
        Size of the matplotlib figure in inches.
    verbose : bool
        When True, print per-round decisions and candidate edges to stdout.
    num_nodes : Optional[int]
        Fix the number of robots/nodes for the generated graph. If omitted, the
        simulator samples a value between min_nodes and max_nodes.

    Returns
    -------
    matplotlib.animation.FuncAnimation
        The animation object (also displayed via plt.show()).
    """

    from matplotlib import pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.collections import LineCollection

    rng = np.random.default_rng(seed)
    if num_nodes is not None and num_nodes < 3:
        raise ValueError("Number of nodes must be at least 3.")
    sim = ConsensusPruningSimulation(rng=rng, verbose=verbose, num_nodes=num_nodes)
    sim.start_new_iteration()
    history, snapshots = sim.run_with_snapshots(max_rounds=max_rounds)
    if not snapshots:
        raise RuntimeError("Simulation completed without any rounds; increase max_rounds.")

    positions = snapshots[0]["positions"]
    node_count = positions.shape[0]

    fig, ax = plt.subplots(figsize=figsize)
    ax.set_aspect("equal")
    margin = 0.1
    ax.set_xlim(-margin, 1.0 + margin)
    ax.set_ylim(-margin, 1.0 + margin)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title("Distributed Edge Pruning")

    node_scatter = ax.scatter(
        positions[:, 0],
        positions[:, 1],
        s=180,
        color="steelblue",
        edgecolors="white",
        linewidths=1.0,
        zorder=3,
    )

    labels = []
    for idx, (x, y) in enumerate(positions):
        labels.append(
            ax.text(x, y + 0.035, str(idx), ha="center", va="bottom", fontsize=9, zorder=4)
        )

    active_lines = LineCollection([], colors="tab:blue", linewidths=2.2, zorder=1)
    removed_lines = LineCollection(
        [], colors="0.75", linewidths=1.5, linestyles="dashed", zorder=0
    )
    highlight_lines = LineCollection([], colors="tab:red", linewidths=3.5, zorder=2)

    ax.add_collection(removed_lines)
    ax.add_collection(active_lines)
    ax.add_collection(highlight_lines)

    status_text = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=10,
        family="monospace",
        bbox=dict(boxstyle="round,pad=0.4", facecolor="white", alpha=0.8),
    )

    def build_segments(edge_list: Iterable[Edge]) -> List[np.ndarray]:
        segments: List[np.ndarray] = []
        for a, b in edge_list:
            if a < 0 or b < 0 or a >= num_nodes or b >= num_nodes:
                continue
            segments.append(np.vstack((positions[a], positions[b])))
        return segments

    def update(frame_idx: int):
        snapshot = snapshots[frame_idx]

        active_lines.set_segments(build_segments(snapshot["edges_active"]))
        removed_lines.set_segments(build_segments(snapshot["edges_removed"]))

        highlight_edge = snapshot.get("highlight_edge")
        focus_edge = snapshot.get("focus_edge")
        highlight_lines.set_segments(
            build_segments([highlight_edge]) if highlight_edge is not None else []
        )

        colors = []
        highlight_nodes = set(snapshot.get("highlight_nodes", []))
        endpoint_nodes: Set[int] = set()
        if focus_edge is not None:
            endpoint_nodes.update(focus_edge)
        for node_id in range(node_count):
            color = "steelblue"
            if node_id in highlight_nodes:
                color = "orange"
            if node_id in endpoint_nodes:
                color = "crimson"
            colors.append(color)
        node_scatter.set_facecolors(colors)

        status_lines = [
            f"Round: {snapshot['round']}",
            f"Active edges: {len(snapshot['edges_active'])}",
            f"Removed edges: {len(snapshot['edges_removed'])}",
        ]
        if focus_edge is not None:
            status_lines.append(f"Focus edge: {focus_edge}")
            alt_max = snapshot.get("alternative_max")
            if alt_max is not None and not math.isnan(alt_max):
                status_lines.append(f"Alt max: {alt_max:.3f}")
            alt_path = snapshot.get("alternative_path")
            if alt_path is not None and not math.isnan(alt_path):
                status_lines.append(f"Alt sum: {alt_path:.3f}")
        elif snapshot["knowledge_updated"]:
            status_lines.append("Knowledge expanding")
        decision = snapshot.get("decision")
        if decision:
            status_lines.append(f"Decision: {decision}")
        status_text.set_text("\n".join(status_lines))

        return (
            node_scatter,
            active_lines,
            removed_lines,
            highlight_lines,
            status_text,
        )

    animation = FuncAnimation(
        fig,
        update,
        frames=len(snapshots),
        interval=interval_ms,
        blit=False,
        repeat=repeat,
    )

    plt.tight_layout()
    plt.show()
    return animation


__all__ = [
    "ConsensusPruningSimulation",
    "animate_single_run",
]


if __name__ == "__main__":
    try:
        raw_input_value = input(
            "Enter the number of robots (>=3) [press Enter for random]: "
        ).strip()
    except EOFError:
        raw_input_value = ""

    chosen_nodes: Optional[int] = None
    if raw_input_value:
        try:
            parsed = int(raw_input_value)
            if parsed < 3:
                print("Number must be at least 3. Falling back to random selection.")
            else:
                chosen_nodes = parsed
        except ValueError:
            print("Invalid input. Falling back to random selection.")

    animate_single_run(num_nodes=chosen_nodes)
