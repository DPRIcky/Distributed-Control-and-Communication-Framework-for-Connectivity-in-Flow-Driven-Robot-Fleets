"""Distributed adjacency matrix consensus protocol.

Implements the consensus algorithm from:
Griparic, K., et al. (2022). "Consensus-based distributed connectivity control 
in multi-agent systems." IEEE Transactions on Network Science and Engineering.

Mathematical Foundation:
    Each robot l maintains local estimate A^l(k) of global adjacency matrix.
    
    Update rule (Eq. 34 from paper):
        A^l(k+1) = A^l(k) + T_d · ΔA^l(k)
    
    where:
        ΔA^l_ij(k) = Σ_{p∈N_l} [w_lp · (A^p_ij(k) - A^l_ij(k))] + ε^l_ij(k)
    
    Observation correction (Eq. 24):
        ε^l_ij(k) = t^l_ij(k) - a^l_ij(k)  if (i,j) incident to l
                  = 0                       otherwise
    
    Trust function (Eq. 23):
        t^l_ij(k) = exp(-(d_ij(k))² / σ²)
    
    Convergence guarantee (Theorem V.1):
        lim_{k→∞} A^l(k) = A*  for all robots l
        with rate: ||A^l(k) - A*|| ≤ C·exp(-λ₂·T_d·k)
"""

from typing import Dict, Set, Tuple

import numpy as np


class AdjacencyMatrixConsensus:
    """Distributed consensus protocol for adjacency matrix estimation.
    
    Each robot maintains a local estimate of the global adjacency matrix
    and updates it through consensus with neighbors plus local observations.
    
    Attributes:
        num_robots: Number of robots in the system
        sigma: Trust function sensitivity parameter
        T_d: Sample time for discrete consensus updates
        epsilon_conv: Convergence threshold for detecting consensus
        A_estimates: Dictionary mapping robot_id to its n×n adjacency estimate
        consensus_converged: Boolean tracking global convergence status
        convergence_round: Number of consensus rounds executed
    """

    def __init__(
        self,
        num_robots: int,
        sigma: float = 1.0,
        sample_time: float = 0.2,
        convergence_epsilon: float = 1e-4
    ):
        """Initialize consensus protocol.
        
        Args:
            num_robots: Number of robots in the system
            sigma: Sensitivity parameter for Gaussian trust function
            sample_time: Discrete time step T_d for consensus updates
            convergence_epsilon: Threshold for detecting convergence
        """
        self.num_robots = num_robots
        self.sigma = sigma
        self.T_d = sample_time
        self.epsilon_conv = convergence_epsilon
        
        # Each robot maintains n×n adjacency matrix estimate
        self.A_estimates: Dict[int, np.ndarray] = {}
        for robot_id in range(num_robots):
            self.A_estimates[robot_id] = np.zeros((num_robots, num_robots))
        
        # Track convergence
        self.consensus_converged = False
        self.convergence_round = 0
        self.previous_estimates: Dict[int, np.ndarray] = {}

    def initialize_robot_knowledge(
        self,
        robot_id: int,
        neighbors: Set[int],
        neighbor_distances: Dict[int, float]
    ) -> None:
        """Initialize robot's adjacency estimate with local observations.
        
        Args:
            robot_id: ID of the robot
            neighbors: Set of neighbor robot IDs
            neighbor_distances: Map of neighbor_id -> distance
        """
        A_local = self.A_estimates[robot_id]
        
        for neighbor_id in neighbors:
            distance = neighbor_distances.get(neighbor_id, 0.0)
            quality = self._trust_function(distance)
            
            # Symmetric initialization
            A_local[robot_id, neighbor_id] = quality
            A_local[neighbor_id, robot_id] = quality

    def _trust_function(self, distance: float) -> float:
        """Compute link quality using Gaussian trust function.
        
        Implements Eq. 23 from Griparic et al. (2022):
            t_ij = exp(-(d_ij)² / σ²)
        
        Args:
            distance: Communication quality metric (e.g., actual distance)
            
        Returns:
            Link quality in [0, 1]
        """
        return np.exp(-(distance ** 2) / (self.sigma ** 2))

    def consensus_update_step(
        self,
        robot_id: int,
        neighbors: Set[int],
        direct_observations: Dict[Tuple[int, int], float]
    ) -> np.ndarray:
        """Execute one consensus update step for a robot.
        
        Implements Eq. 34 from Griparic et al. (2022):
            A^l(k+1) = A^l(k) + T_d · ΔA^l(k)
        
        where ΔA^l(k) = consensus_term + observation_term
        
        Args:
            robot_id: ID of the robot performing update
            neighbors: Current 1-hop neighbors
            direct_observations: Map of (i,j) -> distance for directly observed links
            
        Returns:
            Updated adjacency estimate for this robot
        """
        A_local = self.A_estimates[robot_id]
        n = self.num_robots
        
        # Save previous for convergence check
        self.previous_estimates[robot_id] = A_local.copy()
        
        # Consensus term: weighted average with neighbors
        # ΔA_consensus = Σ_{p∈N_l} w_lp · (A^p - A^l)
        delta_consensus = np.zeros((n, n))
        
        if neighbors:
            # Weight: 1/|N_l| for uniform averaging
            weight = 1.0 / len(neighbors)
            
            for neighbor_id in neighbors:
                A_neighbor = self.A_estimates[neighbor_id]
                delta_consensus += weight * (A_neighbor - A_local)
        
        # Observation term: correct for directly measured links
        # ε^l_ij = t_ij - a^l_ij if (i,j) incident to l, else 0
        E = np.zeros((n, n))
        
        for (i, j), distance in direct_observations.items():
            # Only observe edges incident to this robot
            if i == robot_id or j == robot_id:
                observed_quality = self._trust_function(distance)
                
                # Observation error
                E[i, j] = observed_quality - A_local[i, j]
                E[j, i] = observed_quality - A_local[j, i]
        
        # Update: A(k+1) = A(k) + T_d · (ΔA_consensus + E)
        A_new = A_local + self.T_d * (delta_consensus + E)
        
        # Enforce constraints
        A_new = np.clip(A_new, 0, 1)  # Valid range [0, 1]
        A_new = (A_new + A_new.T) / 2  # Symmetry
        np.fill_diagonal(A_new, 0)  # No self-loops
        
        self.A_estimates[robot_id] = A_new
        return A_new

    def check_convergence(self) -> bool:
        """Check if consensus has converged across all robots.
        
        Convergence criterion: max_{l} ||A^l(k) - A^l(k-1)|| < ε
        
        Returns:
            True if consensus converged
        """
        self.convergence_round += 1
        
        # Need at least one round of updates
        if not self.previous_estimates:
            return False
        
        # Minimum rounds for convergence
        if self.convergence_round < 20:
            return False
        
        # Check if estimates stopped changing
        max_change = 0.0
        for robot_id in range(self.num_robots):
            if robot_id in self.previous_estimates:
                A_prev = self.previous_estimates[robot_id]
                A_curr = self.A_estimates[robot_id]
                change = np.max(np.abs(A_curr - A_prev))
                max_change = max(max_change, change)
        
        self.consensus_converged = max_change < self.epsilon_conv
        return self.consensus_converged

    def get_adjacency_estimate(self, robot_id: int) -> np.ndarray:
        """Get the adjacency matrix estimate for a specific robot.
        
        Args:
            robot_id: Robot to query
            
        Returns:
            n×n adjacency matrix estimate
        """
        return self.A_estimates[robot_id].copy()

    def get_consensus_status(self) -> Dict[str, any]:
        """Get overall consensus status metrics.
        
        Returns:
            Dictionary with convergence status and statistics
        """
        # Compute agreement across robots
        if self.num_robots > 1:
            estimates = [self.A_estimates[i] for i in range(self.num_robots)]
            reference = estimates[0]
            max_disagreement = max(
                np.max(np.abs(A - reference)) for A in estimates[1:]
            )
        else:
            max_disagreement = 0.0
        
        return {
            'converged': self.consensus_converged,
            'round': self.convergence_round,
            'max_disagreement': max_disagreement,
            'epsilon_threshold': self.epsilon_conv
        }

    def reset(self) -> None:
        """Reset consensus state (for new topology or after significant change)."""
        for robot_id in range(self.num_robots):
            self.A_estimates[robot_id] = np.zeros((self.num_robots, self.num_robots))
        
        self.consensus_converged = False
        self.convergence_round = 0
        self.previous_estimates.clear()
