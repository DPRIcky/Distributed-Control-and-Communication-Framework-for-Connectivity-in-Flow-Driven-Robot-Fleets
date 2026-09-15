"""Local Lyapunov function for distributed consensus monitoring.

Mathematical Foundation:
    Local Lyapunov for robot l:
        V_l(k) = Σ_{p∈N_l} ||A^l(k) - A^p(k)||_F^2
    
    Key property: If V_l(k+1) < V_l(k) - ε for all robots l,
                  then global consensus is preserved.
    
    Reference: Distributed optimization theory (Boyd et al., 2011)
"""

from typing import Dict, Set, Tuple
import numpy as np


class LocalLyapunovMonitor:
    """Monitor local Lyapunov function for distributed pruning decisions.
    
    Each robot computes its local Lyapunov function using only information
    from 1-hop neighbors. No global/central knowledge required.
    
    Attributes:
        num_robots: Number of robots in system
        lyapunov_history: Track V_l(k) over time per robot
        initial_lyapunov: V_l(0) for normalization
    """
    
    def __init__(self, num_robots: int):
        """Initialize local Lyapunov monitor.
        
        Args:
            num_robots: Number of robots in the system
        """
        self.num_robots = num_robots
        self.lyapunov_history: Dict[int, list] = {i: [] for i in range(num_robots)}
        self.initial_lyapunov: Dict[int, float] = {}
    
    def compute_local_lyapunov(
        self,
        robot_id: int,
        A_estimates: Dict[int, np.ndarray],
        neighbors: Set[int]
    ) -> float:
        """Compute local Lyapunov function for robot l.
        
        Implements: V_l(k) = Σ_{p∈N_l} ||A^l(k) - A^p(k)||_F^2
        
        Args:
            robot_id: Robot computing the Lyapunov function
            A_estimates: Dictionary mapping robot_id -> adjacency matrix estimate
            neighbors: Set of 1-hop neighbor robot IDs
            
        Returns:
            Local Lyapunov value V_l(k)
        """
        if robot_id not in A_estimates:
            raise ValueError(f"Robot {robot_id} not in A_estimates")
        
        A_local = A_estimates[robot_id]
        V_local = 0.0
        
        # Sum squared Frobenius norm difference with each neighbor
        for neighbor_id in neighbors:
            if neighbor_id not in A_estimates:
                continue
            
            A_neighbor = A_estimates[neighbor_id]
            diff = A_local - A_neighbor
            squared_norm = np.sum(diff ** 2)  # Frobenius norm squared
            V_local += squared_norm
        
        # Store in history
        self.lyapunov_history[robot_id].append(V_local)
        
        # Store initial value for normalization
        if robot_id not in self.initial_lyapunov and V_local > 0:
            self.initial_lyapunov[robot_id] = V_local
        
        return V_local
    
    def simulate_lyapunov_after_edge_removal(
        self,
        robot_id: int,
        edge_to_remove: Tuple[int, int],
        A_estimates: Dict[int, np.ndarray],
        current_neighbors: Set[int],
        positions: np.ndarray,
        sigma: float = 1.0,
        T_d: float = 0.2,
        num_steps: int = 1
    ) -> float:
        """Simulate what local Lyapunov would be after removing an edge.
        
        This simulates one consensus update step with the edge removed,
        then computes the resulting Lyapunov function.
        
        Args:
            robot_id: Robot making the decision
            edge_to_remove: Edge (i, j) being considered for removal
            A_estimates: Current adjacency estimates
            current_neighbors: Current neighbors of robot_id
            positions: Robot positions for distance computation
            sigma: Trust function parameter
            T_d: Consensus sample time
            num_steps: Number of simulated consensus steps
            
        Returns:
            Predicted V_l(k+1) after edge removal
        """
        i, j = edge_to_remove
        
        # Determine new neighbor set after edge removal
        if robot_id == i:
            neighbors_after = current_neighbors - {j}
        elif robot_id == j:
            neighbors_after = current_neighbors - {i}
        else:
            neighbors_after = current_neighbors.copy()
        
        # If no neighbors left, Lyapunov becomes undefined (should not prune)
        if len(neighbors_after) == 0:
            return float('inf')
        
        # Simulate consensus updates with modified neighbor set
        A_simulated = A_estimates[robot_id].copy()
        n = A_simulated.shape[0]
        
        for _ in range(max(1, num_steps)):
            # Consensus term: weighted average with remaining neighbors
            delta_consensus = np.zeros((n, n))
            weight = 1.0 / len(neighbors_after)
            
            for neighbor_id in neighbors_after:
                A_neighbor = A_estimates[neighbor_id]
                delta_consensus += weight * (A_neighbor - A_simulated)
            
            # Observation term (only for edges incident to this robot)
            E = np.zeros((n, n))
            for neighbor_id in neighbors_after:
                distance = np.linalg.norm(positions[robot_id] - positions[neighbor_id])
                observed_quality = np.exp(-(distance ** 2) / (sigma ** 2))
                
                # Observation error
                E[robot_id, neighbor_id] = observed_quality - A_simulated[robot_id, neighbor_id]
                E[neighbor_id, robot_id] = observed_quality - A_simulated[neighbor_id, robot_id]
            
            # Simulated update: A(k+1) = A(k) + T_d * (delta_consensus + E)
            A_simulated = A_simulated + T_d * (delta_consensus + E)
            
            # Enforce constraints
            A_simulated = np.clip(A_simulated, 0, 1)
            A_simulated = (A_simulated + A_simulated.T) / 2
            np.fill_diagonal(A_simulated, 0)
        
        # Compute Lyapunov with simulated estimate
        V_simulated = 0.0
        for neighbor_id in neighbors_after:
            # Simulate neighbor's update too (approximate)
            A_neighbor_sim = A_estimates[neighbor_id].copy()
            diff = A_simulated - A_neighbor_sim
            V_simulated += np.sum(diff ** 2)
        
        return V_simulated
    
    def check_pruning_condition(
        self,
        robot_id: int,
        edge_to_remove: Tuple[int, int],
        A_estimates: Dict[int, np.ndarray],
        current_neighbors: Set[int],
        positions: np.ndarray,
        epsilon_threshold: float,
        sigma: float = 1.0,
        T_d: float = 0.2,
        num_steps: int = 1
    ) -> Tuple[bool, Dict[str, float]]:
        """Check if edge can be removed without disrupting consensus.
        
        Pruning condition: V_l(k) - V_l(k+1) > ε
        (Local Lyapunov must continue decreasing)
        
        Args:
            robot_id: Robot making decision
            edge_to_remove: Edge being considered
            A_estimates: Current estimates
            current_neighbors: Current neighbors
            positions: Robot positions
            epsilon_threshold: Required decrease threshold
            sigma: Trust function parameter
            T_d: Sample time
            num_steps: Number of simulated consensus steps
            
        Returns:
            (can_prune, metrics_dict)
        """
        # Compute current Lyapunov
        V_current = self.compute_local_lyapunov(robot_id, A_estimates, current_neighbors)
        
        # Simulate Lyapunov after edge removal
        V_after = self.simulate_lyapunov_after_edge_removal(
            robot_id=robot_id,
            edge_to_remove=edge_to_remove,
            A_estimates=A_estimates,
            current_neighbors=current_neighbors,
            positions=positions,
            sigma=sigma,
            T_d=T_d,
            num_steps=num_steps
        )
        
        # Check if Lyapunov decreases sufficiently
        decrease = V_current - V_after
        can_prune = decrease > epsilon_threshold
        
        metrics = {
            'V_current': V_current,
            'V_after': V_after,
            'decrease': decrease,
            'threshold': epsilon_threshold,
            'relative_decrease': decrease / V_current if V_current > 0 else 0.0
        }
        
        return can_prune, metrics
    
    def get_convergence_status(self, robot_id: int, window_size: int = 10) -> Dict[str, float]:
        """Get convergence statistics for a robot.
        
        Args:
            robot_id: Robot to analyze
            window_size: Number of recent iterations to analyze
            
        Returns:
            Convergence statistics dictionary
        """
        if robot_id not in self.lyapunov_history:
            return {}
        
        history = self.lyapunov_history[robot_id]
        if len(history) < 2:
            return {'iterations': len(history)}
        
        # Recent values
        recent = history[-window_size:] if len(history) >= window_size else history
        
        # Compute rate of decrease
        if len(recent) > 1:
            rate = (recent[0] - recent[-1]) / len(recent)
        else:
            rate = 0.0
        
        return {
            'iterations': len(history),
            'current_value': history[-1],
            'initial_value': self.initial_lyapunov.get(robot_id, 0.0),
            'decrease_rate': rate,
            'recent_mean': np.mean(recent),
            'recent_std': np.std(recent)
        }
    
    def reset(self):
        """Reset all stored history."""
        for robot_id in range(self.num_robots):
            self.lyapunov_history[robot_id] = []
        self.initial_lyapunov.clear()
