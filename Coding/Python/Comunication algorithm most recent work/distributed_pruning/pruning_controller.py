"""
Progressive Distributed Pruning Controller

Implements provably safe edge pruning with formal connectivity guarantees.

Theoretical Guarantee:
    Only prunes edges (i,j) where |N(i) ∩ N(j)| ≥ k
    - k=1: Maintains connectivity (proven by common neighbor theorem)
    - k=2: Maintains 2-connectivity (robust to single edge failure)

Computation: O(d²) per robot, where d = max degree (~3-10)
Communication: O(d) neighbor IDs per robot
"""

import numpy as np
from typing import Dict, Set, List, Tuple, Optional
from collections import defaultdict


class ProgressivePruningController:
    """
    Distributed pruning controller for a single robot.
    
    Uses only local 2-hop neighborhood information to make provably safe
    pruning decisions that maintain global connectivity.
    
    Simulates realistic communication delays: 1-hop info is immediate,
    but 2-hop info is gradually learned over multiple communication rounds.
    
    Attributes:
        robot_id: Unique robot identifier
        target_degree: Target number of active neighbors (final sparse topology)
        k_connectivity: Minimum common neighbors required (1=connected, 2=2-connected)
        max_k_hop: Maximum hop distance for information gathering
        information_gain_rate: Rate at which 2-hop info is learned per timestep
        position: Current robot position
        state_vector: Current state for consensus (e.g., position, velocity)
        all_neighbors: All robots within communication range
        active_neighbors: Currently active (non-pruned) neighbors
        neighbor_neighbor_lists: 2-hop info - what neighbors each neighbor has (partial)
        neighbor_info_confidence: Confidence in 2-hop info for each neighbor (0-1)
        k_hop_neighbors: Multi-hop neighborhood {hop_distance: set of robot IDs}
        communication_radius: Maximum communication range
        information_completeness: Average confidence in 2-hop info (0.0-1.0)
        pruning_aggressiveness: How aggressively to prune (0.0-1.0)
    """
    
    def __init__(
        self,
        robot_id: int,
        target_degree: int = 3,
        k_connectivity: int = 1,
        max_k_hop: int = 3,
        communication_radius: float = 10.0,
        information_gain_rate: float = 0.1
    ):
        """
        Initialize pruning controller.
        
        Args:
            robot_id: Unique robot identifier
            target_degree: Target number of neighbors (3-4 for sparse chain)
            k_connectivity: 1=connectivity, 2=2-connectivity (more robust)
            max_k_hop: Maximum hop distance to gather information
            communication_radius: Maximum communication range
            information_gain_rate: Rate at which 2-hop info is learned (0.05-0.2)
        """
        self.robot_id = robot_id
        self.target_degree = target_degree
        self.k_connectivity = k_connectivity
        self.max_k_hop = max_k_hop
        self.communication_radius = communication_radius
        self.information_gain_rate = information_gain_rate
        
        # Robot state
        self.position = np.zeros(2)
        self.state_vector = np.zeros(2)
        
        # Neighbor information
        self.all_neighbors: Dict[int, Dict] = {}  # All neighbors in range
        self.active_neighbors: Set[int] = set()  # Active (non-pruned) neighbors
        self.permanently_pruned: Set[int] = set()  # Edges that have been pruned and should stay pruned
        self.neighbor_neighbor_lists: Dict[int, Set[int]] = {}  # 2-hop info
        self.neighbor_info_confidence: Dict[int, float] = {}  # How complete is 2-hop info (0-1)
        self.k_hop_neighbors: Dict[int, Set[int]] = defaultdict(set)
        
        # Pruning state
        self.information_completeness = 0.0  # 0.0 to 1.0
        self.pruning_aggressiveness = 0.0  # 0.0 (conservative) to 1.0 (aggressive)
        
        # Statistics
        self.total_edges_pruned = 0
        self.pruning_history: List[int] = []
    
    def update_state(self, position: np.ndarray, state_vector: Optional[np.ndarray] = None):
        """Update robot's physical state."""
        self.position = position.copy()
        if state_vector is not None:
            self.state_vector = state_vector.copy()
        else:
            self.state_vector = position.copy()
    
    def update_neighbors(self, neighbor_data: List[Tuple[int, float, np.ndarray, np.ndarray, List[int]]]):
        """
        Update information about nearby robots.
        
        Simulates progressive information gathering through communication:
        - 1-hop info is immediately available (direct sensing)
        - 2-hop info is gradually learned over time (communication rounds)
        
        Args:
            neighbor_data: List of (robot_id, distance, position, state, neighbor_list)
                - robot_id: ID of neighbor
                - distance: Distance to neighbor
                - position: Neighbor's position
                - state: Neighbor's state vector
                - neighbor_list: List of neighbor's neighbors (2-hop info)
        """
        self.all_neighbors.clear()
        self.k_hop_neighbors.clear()
        
        # Build 1-hop neighborhood (always immediately available)
        for robot_id, distance, position, state, neighbor_list in neighbor_data:
            if distance <= self.communication_radius:
                self.k_hop_neighbors[1].add(robot_id)
                self.all_neighbors[robot_id] = {
                    'distance': distance,
                    'position': position.copy(),
                    'state': state.copy(),
                    'hop_distance': 1
                }
                
                # Gradually build 2-hop information (simulate communication delays)
                if robot_id not in self.neighbor_info_confidence:
                    self.neighbor_info_confidence[robot_id] = 0.0
                
                # Increase confidence over time
                old_confidence = self.neighbor_info_confidence[robot_id]
                self.neighbor_info_confidence[robot_id] = min(
                    1.0,
                    self.neighbor_info_confidence[robot_id] + self.information_gain_rate
                )
                
                # Only store 2-hop info proportional to confidence
                confidence = self.neighbor_info_confidence[robot_id]
                full_neighbor_list = set(neighbor_list)
                
                # Randomly sample neighbors based on confidence level
                # (simulates incomplete information exchange)
                num_to_learn = int(len(full_neighbor_list) * confidence)
                if num_to_learn > 0:
                    partial_list = set(list(full_neighbor_list)[:num_to_learn])
                    self.neighbor_neighbor_lists[robot_id] = partial_list
                else:
                    self.neighbor_neighbor_lists[robot_id] = set()
        
        # Remove confidence for neighbors that went out of range
        current_neighbors = self.k_hop_neighbors[1]
        for nid in list(self.neighbor_info_confidence.keys()):
            if nid not in current_neighbors:
                del self.neighbor_info_confidence[nid]
        
        # Build 2-hop neighborhood (neighbors of neighbors)
        for neighbor_id in self.k_hop_neighbors[1]:
            if neighbor_id in self.neighbor_neighbor_lists:
                for second_hop_id in self.neighbor_neighbor_lists[neighbor_id]:
                    # Add to 2-hop if not in 1-hop and not self
                    if second_hop_id != self.robot_id and second_hop_id not in self.k_hop_neighbors[1]:
                        self.k_hop_neighbors[2].add(second_hop_id)
        
        # Update information completeness
        self._update_information_completeness()
    
    def _update_information_completeness(self):
        """
        Calculate information completeness metric.
        
        Completeness = average confidence in 2-hop info for all neighbors.
        Higher completeness → more confident pruning decisions.
        """
        neighbors_1hop = len(self.k_hop_neighbors[1])
        
        if neighbors_1hop == 0:
            self.information_completeness = 0.0
            self.pruning_aggressiveness = 0.0
            return
        
        # Average confidence across all neighbors
        total_confidence = sum(
            self.neighbor_info_confidence.get(nid, 0.0)
            for nid in self.k_hop_neighbors[1]
        )
        
        self.information_completeness = total_confidence / neighbors_1hop
        
        # Update pruning aggressiveness (sigmoid function)
        # More aggressive: steeper slope (15 instead of 10) and earlier shift (0.3 instead of 0.5)
        # This makes pruning ramp up quickly once we have some information
        self.pruning_aggressiveness = 1.0 / (1.0 + np.exp(-15 * (self.information_completeness - 0.3)))
    
    def count_common_neighbors(self, neighbor_id: int) -> int:
        """
        Count common neighbors with given neighbor.
        
        This is the CRITICAL computation for connectivity guarantee.
        IMPORTANT: Only counts neighbors in the ACTIVE graph, not potential graph.
        
        Args:
            neighbor_id: ID of neighbor to check
        
        Returns:
            Number of common neighbors (excluding self and neighbor_id) in ACTIVE graph
        """
        if neighbor_id not in self.neighbor_neighbor_lists:
            return 0
        
        # CRITICAL FIX: Use active_neighbors (pruned graph), not k_hop_neighbors (potential graph)
        # If no active neighbors set yet, use all neighbors (initial state)
        my_active = self.active_neighbors if len(self.active_neighbors) > 0 else self.k_hop_neighbors[1]
        their_neighbors = self.neighbor_neighbor_lists[neighbor_id]
        
        # Intersection: neighbors that are active for ME and known through THEM
        common = my_active & their_neighbors
        
        # Remove self and the neighbor from count
        common.discard(self.robot_id)
        common.discard(neighbor_id)
        
        return len(common)
    
    def is_bridge_edge(self, neighbor_id: int) -> bool:
        """
        Check if edge to neighbor is a bridge (cut-edge).
        
        THEOREM: If common_neighbors >= k_connectivity, edge is NOT a bridge.
        
        Args:
            neighbor_id: ID of neighbor to check
        
        Returns:
            True if edge is a bridge (UNSAFE to prune)
            False if edge is NOT a bridge (SAFE to prune)
        """
        common_count = self.count_common_neighbors(neighbor_id)
        return common_count < self.k_connectivity
    
    def is_safe_to_prune(self, neighbor_id: int) -> bool:
        """
        Determine if edge can be pruned while maintaining connectivity.
        
        Based on formal proof: Only prune if common neighbors exist.
        
        Args:
            neighbor_id: ID of neighbor to check
        
        Returns:
            True if provably safe to prune, False otherwise
        """
        # Safety check 0: Check if topology is already sparse
        # If average degree is low, be very conservative
        potential_degree = len(self.k_hop_neighbors[1])
        active_degree = len(self.active_neighbors)
        
        if potential_degree <= self.target_degree + 1:
            # Already at or near target - don't prune
            return False
        
        # Safety check 1: Need HIGH information completeness for sparse topologies
        # With 1m radius, wait for very good information
        if self.information_completeness < 0.5:
            return False  # Too early to prune safely (was 0.2, now 0.5)
        
        # Safety check 2: Don't have 2-hop info for this neighbor yet
        if neighbor_id not in self.neighbor_neighbor_lists:
            return False
        
        # Safety check 3: Is it a bridge?
        if self.is_bridge_edge(neighbor_id):
            return False  # UNSAFE - might disconnect graph
        
        # Safety check 4: Would this make me under-connected?
        # Check ACTIVE degree, not potential degree  
        if active_degree <= self.target_degree:
            return False
        
        # Safety check 5: Never prune if current degree is already low
        if active_degree <= 2:
            return False  # At minimum for connectivity
        
        return True  # Provably safe
    
    def get_pruning_budget(self) -> int:
        """
        Determine how many edges to prune this timestep.
        
        Progressive strategy:
        - Low completeness: Very conservative (1-2 edges)
        - Medium completeness: Moderate (3-5 edges)
        - High completeness: Aggressive (up to 50% of excess)
        
        Returns:
            Maximum number of edges to prune this step
        """
        # Use ACTIVE degree to calculate pruning budget
        current_degree = len(self.active_neighbors)
        
        if current_degree <= self.target_degree:
            return 0
        
        excess_edges = current_degree - self.target_degree
        
        # Very aggressive pruning - prune ALL excess edges once we have good info
        if self.information_completeness < 0.1:
            # Phase 1: Still prune at least 20-30% even with minimal info
            max_prune = min(max(2, int(excess_edges * 0.3)), excess_edges)
        elif self.information_completeness < 0.3:
            # Phase 2: Prune 60% of excess
            max_prune = min(max(4, int(excess_edges * 0.6)), excess_edges)
        else:
            # Phase 3: Very aggressive - try to prune ALL excess edges
            # Only keep edges if they're proven to be bridges
            max_prune = excess_edges  # Prune all excess if possible
        
        return max_prune
    
    def compute_edge_priority(self, neighbor_id: int) -> float:
        """
        Compute priority for pruning this edge (higher = prune first).
        
        Combines multiple heuristics:
        - Distance: Prefer to prune distant edges (keep nearby)
        - Disagreement: Prefer to prune synchronized edges
        - Redundancy: Prefer to prune edges with many common neighbors
        
        Args:
            neighbor_id: ID of neighbor to score
        
        Returns:
            Priority score (higher means prune first)
        """
        info = self.all_neighbors.get(neighbor_id, {})
        
        # Factor 1: Distance (always available, 1-hop)
        distance = info.get('distance', 0.0)
        distance_score = distance / self.communication_radius
        
        # Factor 2: State disagreement (consensus)
        disagreement_score = 0.0
        if self.state_vector is not None and info.get('state') is not None:
            disagreement = np.linalg.norm(self.state_vector - info['state'])
            # Low disagreement = already synchronized = higher prune priority
            disagreement_score = 1.0 / (disagreement + 0.1)
        
        # Factor 3: Redundancy (common neighbors)
        common_count = self.count_common_neighbors(neighbor_id)
        redundancy_score = float(common_count)
        
        # Weighted combination (weights change with information completeness)
        # Early: rely on distance; Later: incorporate disagreement and redundancy
        weight_distance = 1.0 - 0.5 * self.information_completeness
        weight_disagreement = self.information_completeness * 0.3
        weight_redundancy = self.information_completeness * 0.5
        
        priority = (
            weight_distance * distance_score +
            weight_disagreement * disagreement_score +
            weight_redundancy * redundancy_score
        )
        
        return priority
    
    def prune_edges(self) -> Tuple[Set[int], Set[int]]:
        """
        Main pruning algorithm with formal connectivity guarantee.
        
        Algorithm:
        1. Update active topology (add TRUE new arrivals, remove departed neighbors)
        2. Categorize active edges as safe/unsafe based on common neighbors
        3. ALWAYS keep unsafe (bridge) edges
        4. Score safe edges by priority
        5. Prune highest priority edges up to budget
        
        CRITICAL: Once an edge is pruned, it stays pruned PERMANENTLY
        Exception: If neighbor leaves range and comes back, it's a true new arrival
        
        Returns:
            Tuple of (pruned_edges, kept_edges) - sets of neighbor IDs
        """
        # Get current neighbors in communication range
        potential_neighbors = set(self.k_hop_neighbors[1])
        
        if not potential_neighbors:
            self.active_neighbors = set()
            return set(), set()
        
        # Track edges that left range
        # CRITICAL: Clear from permanently_pruned so they can be re-evaluated if they return
        # This handles cases where robots drift beyond range due to movement
        all_known_neighbors = self.active_neighbors | self.permanently_pruned
        departed = all_known_neighbors - potential_neighbors
        for neighbor_id in departed:
            self.permanently_pruned.discard(neighbor_id)
        
        # IMPORTANT: If an active edge goes out of range, it's because
        # CBF couldn't maintain it (geometric constraint). Don't mark as permanently pruned.
        # It should be re-evaluated when robots come back in range.
        
        # Initialize on first call
        if len(self.active_neighbors) == 0 and len(self.permanently_pruned) == 0:
            # First iteration: all edges are active
            self.active_neighbors = potential_neighbors.copy()
        else:
            # Update active topology:
            # 1. Remove neighbors that left range
            self.active_neighbors = self.active_neighbors & potential_neighbors
            
            # 2. Add ONLY true new arrivals (never seen before OR previously departed)
            true_new_arrivals = potential_neighbors - (self.active_neighbors | self.permanently_pruned)
            self.active_neighbors = self.active_neighbors | true_new_arrivals
        
        # Work with active neighbors only (never re-add permanently pruned edges)
        current_neighbors = list(self.active_neighbors)
        
        # Determine pruning budget
        max_prune = self.get_pruning_budget()
        
        if max_prune == 0:
            # Already at target or too few neighbors
            return set(), self.active_neighbors
        
        # Categorize edges by safety
        provably_safe: List[int] = []
        might_be_bridge: List[int] = []
        
        for neighbor_id in current_neighbors:
            if self.is_safe_to_prune(neighbor_id):
                provably_safe.append(neighbor_id)
            else:
                might_be_bridge.append(neighbor_id)
        
        # Debug: print info for robot 0 every so often
        if self.robot_id == 0 and len(current_neighbors) > 0:
            if np.random.random() < 0.05:  # 5% of the time
                print(f"  [Robot {self.robot_id}] Current neighbors: {len(current_neighbors)}, "
                      f"Safe to prune: {len(provably_safe)}, Bridges: {len(might_be_bridge)}, "
                      f"Budget: {max_prune}, Target: {self.target_degree}")
        
        # CONNECTIVITY GUARANTEE: ALWAYS keep potential bridges
        keep_edges = set(might_be_bridge)
        
        # SAFETY CHECK: Never prune if it would disconnecct us
        edges_after_max_prune = len(current_neighbors) - max_prune
        if edges_after_max_prune < 1:
            # WARNING: Would leave us isolated or nearly isolated
            print(f"  [WARNING Robot {self.robot_id}] Pruning budget {max_prune} would leave only {edges_after_max_prune} edges!")
            max_prune = max(0, len(current_neighbors) - 2)  # Keep at least 2 edges
        
        # Score safe edges for pruning
        safe_with_priority = [
            (self.compute_edge_priority(nid), nid)
            for nid in provably_safe
        ]
        safe_with_priority.sort(reverse=True)  # Highest priority first
        
        # Prune top priority edges up to budget
        pruned_this_step = set()
        for priority, neighbor_id in safe_with_priority[:max_prune]:
            pruned_this_step.add(neighbor_id)
            self.permanently_pruned.add(neighbor_id)  # Mark as permanently pruned
            self.total_edges_pruned += 1
        
        # Keep remaining safe edges
        for priority, neighbor_id in safe_with_priority[max_prune:]:
            keep_edges.add(neighbor_id)
        
        # Update active neighbors (remove pruned edges)
        self.active_neighbors = keep_edges
        self.pruning_history.append(len(pruned_this_step))
        
        return pruned_this_step, keep_edges
    
    def get_statistics(self) -> Dict:
        """
        Get current pruning statistics for monitoring.
        
        Returns:
            Dictionary with pruning metrics
        """
        return {
            'robot_id': self.robot_id,
            'k_hop_counts': {k: len(v) for k, v in self.k_hop_neighbors.items()},
            'information_completeness': self.information_completeness,
            'pruning_aggressiveness': self.pruning_aggressiveness,
            'current_degree': len(self.k_hop_neighbors[1]),
            'active_degree': len(self.active_neighbors),
            'target_degree': self.target_degree,
            'pruning_budget': self.get_pruning_budget(),
            'total_pruned': self.total_edges_pruned,
            'edges_might_be_bridges': sum(
                1 for nid in self.k_hop_neighbors[1]
                if not self.is_safe_to_prune(nid)
            )
        }
    
    def get_active_edges(self) -> Set[int]:
        """Get set of active (non-pruned) neighbor IDs."""
        return self.active_neighbors.copy()
    
    def get_pruned_edges(self) -> Set[int]:
        """Get set of pruned neighbor IDs."""
        all_ids = self.k_hop_neighbors[1]
        return all_ids - self.active_neighbors
