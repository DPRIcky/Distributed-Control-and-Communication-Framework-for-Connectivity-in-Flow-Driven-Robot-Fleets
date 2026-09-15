"""
Graph utilities for connectivity maintenance and bidding mechanism.
"""

import numpy as np
import networkx as nx
from typing import List, Set, Dict, Tuple, Optional
from src.core.agent import Agent


class GraphManager:
    """Manages network topology and connectivity."""

    def __init__(self, sensing_radius: float = 1.7):
        """
        Initialize graph manager.

        Args:
            sensing_radius: Communication radius
        """
        self.sensing_radius = sensing_radius
        self.graph = nx.Graph()

    def update_graph(self, agents: List[Agent]):
        """
        Update graph topology based on agent positions.

        Args:
            agents: List of agents
        """
        # Clear existing edges
        self.graph.clear()

        # Add nodes
        for agent in agents:
            self.graph.add_node(agent.id, pos=agent.position)

        # Add edges based on sensing radius
        for i, agent_i in enumerate(agents):
            for j, agent_j in enumerate(agents[i + 1:], start=i + 1):
                dist = agent_i.distance_to_agent(agent_j)
                if dist <= self.sensing_radius:
                    self.graph.add_edge(agent_i.id, agent_j.id, weight=dist)

    def is_connected(self) -> bool:
        """Check if graph is connected."""
        return nx.is_connected(self.graph)

    def get_neighbors(self, agent_id: int) -> Set[int]:
        """Get neighbors of an agent."""
        if agent_id in self.graph:
            return set(self.graph.neighbors(agent_id))
        return set()

    def compute_safe_neighbors(self, agent_id: int) -> Set[int]:
        """
        Compute set of safe neighbors for an agent.
        Safe neighbor: removing edge to this neighbor doesn't break connectivity.

        Args:
            agent_id: Agent ID

        Returns:
            Set of safe neighbor IDs
        """
        safe_neighbors = set()
        neighbors = self.get_neighbors(agent_id)

        for neighbor_id in neighbors:
            # Temporarily remove edge
            self.graph.remove_edge(agent_id, neighbor_id)

            # Check if still connected locally (BFS from agent_id can reach all its other neighbors)
            if self._is_locally_connected(agent_id, neighbors - {neighbor_id}):
                safe_neighbors.add(neighbor_id)

            # Restore edge
            self.graph.add_edge(agent_id, neighbor_id)

        return safe_neighbors

    def _is_locally_connected(self, agent_id: int, neighbors: Set[int]) -> bool:
        """
        Check if agent can reach all specified neighbors through the graph.

        Args:
            agent_id: Agent ID
            neighbors: Set of neighbor IDs to check reachability

        Returns:
            True if all neighbors are reachable
        """
        if not neighbors:
            return True

        for neighbor_id in neighbors:
            if not nx.has_path(self.graph, agent_id, neighbor_id):
                return False

        return True

    def get_second_smallest_eigenvalue(self) -> float:
        """
        Compute second smallest eigenvalue of Laplacian matrix (algebraic connectivity).

        Returns:
            Second smallest eigenvalue (0 if not connected)
        """
        if len(self.graph.nodes) < 2:
            return 0.0

        try:
            laplacian = nx.laplacian_matrix(self.graph).todense()
            eigenvalues = np.linalg.eigvalsh(laplacian)
            eigenvalues.sort()
            return eigenvalues[1]  # Second smallest
        except:
            return 0.0

    def remove_edge(self, agent_i: int, agent_j: int):
        """Remove edge from graph."""
        if self.graph.has_edge(agent_i, agent_j):
            self.graph.remove_edge(agent_i, agent_j)

    def get_adjacency_matrix(self) -> np.ndarray:
        """Get adjacency matrix of the graph."""
        n = len(self.graph.nodes)
        adj_matrix = np.zeros((n, n))

        node_list = sorted(self.graph.nodes())
        for i, node_i in enumerate(node_list):
            for j, node_j in enumerate(node_list):
                if self.graph.has_edge(node_i, node_j):
                    adj_matrix[i, j] = 1

        return adj_matrix


class BiddingMechanism:
    """
    Bidding-based edge deletion mechanism based on Zavlanos & Pappas 2008.
    Algorithm 1: Distributed Connectivity Control of Mobile Networks
    """

    def __init__(self, epsilon: float = 0.01):
        """
        Initialize bidding mechanism.

        Args:
            epsilon: Tiebreak parameter for mutually beneficial edges (K_i = 1 + epsilon)
        """
        self.epsilon = epsilon
        self.token = {}  # Token for each agent (binary: 0 or 1)
        self.converged = False

    def compute_safe_neighbors_with_connectivity(self, agent_id: int,
                                                  graph: GraphManager) -> Set[int]:
        """
        Compute safe neighbors S_i for agent i using local connectivity check.

        From Zavlanos & Pappas 2008, Section III-A:
        S_i = {j_i ∈ N_i : λ₂(L(A_i \ {(i,j_i)})) > 0}

        Args:
            agent_id: Agent ID
            graph: Graph manager

        Returns:
            Set of safe neighbor IDs
        """
        safe_neighbors = set()
        neighbors = graph.get_neighbors(agent_id)

        if not neighbors:
            return safe_neighbors

        # For each neighbor, check if removing edge maintains local connectivity
        for neighbor_id in neighbors:
            # Temporarily remove edge
            edge_existed = graph.graph.has_edge(agent_id, neighbor_id)
            if edge_existed:
                graph.graph.remove_edge(agent_id, neighbor_id)

            # Check local connectivity: all neighbors reachable from agent_id
            is_safe = True
            remaining_neighbors = neighbors - {neighbor_id}

            if remaining_neighbors:
                # Use BFS to check if all remaining neighbors are reachable
                try:
                    for other_neighbor in remaining_neighbors:
                        if not nx.has_path(graph.graph, agent_id, other_neighbor):
                            is_safe = False
                            break
                except:
                    is_safe = False

            # Restore edge
            if edge_existed:
                graph.graph.add_edge(agent_id, neighbor_id)

            if is_safe:
                safe_neighbors.add(neighbor_id)

        return safe_neighbors

    def compute_bid_for_auction(self, agent: Agent, safe_neighbors: Set[int],
                                agents: List[Agent], graph: GraphManager) -> Tuple[Optional[int], float]:
        """
        Compute bid for edge deletion auction based on Zavlanos & Pappas 2008.

        Selection function g(S_i): Choose neighbor to remove based on distance
        Bid value: Distance to selected neighbor (farther = higher bid)

        Args:
            agent: The agent
            safe_neighbors: Set of safe neighbor IDs from compute_safe_neighbors_with_connectivity
            agents: List of all agents
            graph: Graph manager

        Returns:
            (selected_neighbor_id, bid_value)
        """
        if not safe_neighbors:
            return None, -1.0

        # Selection function g(S_i): select farthest safe neighbor
        # This prioritizes removing long-range edges that may be less useful
        max_distance = -1.0
        selected_neighbor = None

        for neighbor_id in safe_neighbors:
            neighbor_agent = next((a for a in agents if a.id == neighbor_id), None)
            if neighbor_agent is not None:
                dist = agent.distance_to_agent(neighbor_agent)
                if dist > max_distance:
                    max_distance = dist
                    selected_neighbor = neighbor_id

        if selected_neighbor is None:
            return None, -1.0

        # Compute bid value (distance-based)
        # Normalize by sensing radius to keep bids in reasonable range
        bid_value = max_distance / graph.sensing_radius

        # Check if edge is mutually beneficial to remove
        # If both agents want to remove the same edge, apply K_i = 1 + epsilon bonus
        neighbor_agent = next((a for a in agents if a.id == selected_neighbor), None)
        if neighbor_agent is not None:
            # Check if neighbor also selected this agent
            if (hasattr(neighbor_agent, 'bid_neighbor') and
                neighbor_agent.bid_neighbor == agent.id and
                hasattr(agent, 'bid_neighbor') and
                agent.bid_neighbor == selected_neighbor):
                K_i = 1.0 + self.epsilon
            else:
                K_i = 1.0
        else:
            K_i = 1.0

        bid_value *= K_i

        return selected_neighbor, bid_value

    def max_consensus_update(self, agents: List[Agent], graph: GraphManager) -> Optional[Tuple[int, int]]:
        """
        Perform max-consensus to determine winning bid and remove edge.
        Based on Algorithm 1 from Zavlanos & Pappas 2008.

        Each agent maintains a token (binary 0 or 1):
        - Token = 1 if agent has the maximum bid among its neighbors
        - Token = 0 otherwise

        Convergence: All tokens = 1 (max-consensus reached)

        Args:
            agents: List of all agents
            graph: Graph manager

        Returns:
            (agent_i, agent_j) tuple of edge to remove, or None
        """
        # Initialize tokens if first call
        if not self.token:
            for agent in agents:
                self.token[agent.id] = 0

        # Find global maximum bid
        max_bid = -1.0
        winning_agent_id = None
        winning_neighbor_id = None

        for agent in agents:
            if hasattr(agent, 'bid_value') and agent.bid_value > max_bid:
                max_bid = agent.bid_value
                winning_agent_id = agent.id
                if hasattr(agent, 'bid_neighbor'):
                    winning_neighbor_id = agent.bid_neighbor

        # Update tokens based on max-consensus
        all_converged = True
        for agent in agents:
            neighbors = graph.get_neighbors(agent.id)

            # Check if agent has maximum bid among neighbors
            agent_bid = agent.bid_value if hasattr(agent, 'bid_value') else -1.0
            is_max = True

            for neighbor_id in neighbors:
                neighbor_agent = next((a for a in agents if a.id == neighbor_id), None)
                if neighbor_agent is not None:
                    neighbor_bid = neighbor_agent.bid_value if hasattr(neighbor_agent, 'bid_value') else -1.0
                    if neighbor_bid > agent_bid:
                        is_max = False
                        break

            # Update token
            if is_max and agent_bid > 0:
                self.token[agent.id] = 1
            else:
                self.token[agent.id] = 0
                all_converged = False

        # If converged and there's a valid winning bid, remove edge
        if all_converged and max_bid > 0 and winning_agent_id is not None and winning_neighbor_id is not None:
            # Verify edge can still be safely removed
            safe_neighbors = self.compute_safe_neighbors_with_connectivity(winning_agent_id, graph)
            if winning_neighbor_id in safe_neighbors:
                print(f"Max-consensus: Removing edge ({winning_agent_id}, {winning_neighbor_id}) with bid {max_bid:.3f}")
                graph.remove_edge(winning_agent_id, winning_neighbor_id)

                # Reset tokens for next auction round
                self.token = {}

                return (winning_agent_id, winning_neighbor_id)
            else:
                # Edge is no longer safe to remove, reset
                self.token = {}

        return None
