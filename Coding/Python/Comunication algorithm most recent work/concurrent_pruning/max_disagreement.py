"""Max disagreement metric for distributed consensus monitoring.

Mathematical Foundation:
    Max disagreement for robot l:
        D_l(k) = max_{p∈N_l} ||A^l(k) - A^p(k)||_F
    
    Key property: More conservative than Lyapunov.
                  Focuses on worst-case neighbor convergence.
    
    Advantage: Simpler computation, intuitive interpretation.
"""

from typing import Dict, Set, Tuple, Optional
import numpy as np


class MaxDisagreementMonitor:
    """Monitor max disagreement metric for distributed pruning decisions.
    
    Each robot computes maximum disagreement with its neighbors.
    More conservative than Lyapunov: focuses on slowest-converging neighbor.
    
    Attributes:
        num_robots: Number of robots in system
        disagreement_history: Track D_l(k) over time per robot
        initial_disagreement: D_l(0) for normalization
        worst_neighbor_history: Track which neighbor has max disagreement
    """
    
    def __init__(self, num_robots: int):
        """Initialize max disagreement monitor.
        
        Args:
            num_robots: Number of robots in the system
        """
        self.num_robots = num_robots
        self.disagreement_history: Dict[int, list] = {i: [] for i in range(num_robots)}
        self.initial_disagreement: Dict[int, float] = {}
        self.worst_neighbor_history: Dict[int, list] = {i: [] for i in range(num_robots)}
    
    def compute_max_disagreement(
        self,
        robot_id: int,
        A_estimates: Dict[int, np.ndarray],
        neighbors: Set[int]
    ) -> Tuple[float, Optional[int]]:
        """Compute max disagreement with neighbors.
        
        Implements: D_l(k) = max_{p∈N_l} ||A^l(k) - A^p(k)||_F
        
        Args:
            robot_id: Robot computing the disagreement
            A_estimates: Dictionary mapping robot_id -> adjacency matrix estimate
            neighbors: Set of 1-hop neighbor robot IDs
            
        Returns:
            (D_l(k), worst_neighbor_id)
        """
        if robot_id not in A_estimates:
            raise ValueError(f"Robot {robot_id} not in A_estimates")
        
        if len(neighbors) == 0:
            return 0.0, None
        
        A_local = A_estimates[robot_id]
        max_disagreement = 0.0
        worst_neighbor = None
        
        # Find maximum Frobenius norm difference across all neighbors
        for neighbor_id in neighbors:
            if neighbor_id not in A_estimates:
                continue
            
            A_neighbor = A_estimates[neighbor_id]
            diff = A_local - A_neighbor
            frobenius_norm = np.sqrt(np.sum(diff ** 2))
            
            if frobenius_norm > max_disagreement:
                max_disagreement = frobenius_norm
                worst_neighbor = neighbor_id
        
        # Store in history
        self.disagreement_history[robot_id].append(max_disagreement)
        self.worst_neighbor_history[robot_id].append(worst_neighbor)
        
        # Store initial value for normalization
        if robot_id not in self.initial_disagreement and max_disagreement > 0:
            self.initial_disagreement[robot_id] = max_disagreement
        
        return max_disagreement, worst_neighbor
    
    def compute_average_disagreement(
        self,
        robot_id: int,
        A_estimates: Dict[int, np.ndarray],
        neighbors: Set[int]
    ) -> float:
        """Compute average disagreement with neighbors.
        
        Alternative metric: D̄_l(k) = (1/|N_l|) Σ_{p∈N_l} ||A^l(k) - A^p(k)||_F
        
        Args:
            robot_id: Robot computing the disagreement
            A_estimates: Dictionary mapping robot_id -> adjacency matrix estimate
            neighbors: Set of 1-hop neighbor robot IDs
            
        Returns:
            Average disagreement D̄_l(k)
        """
        if robot_id not in A_estimates or len(neighbors) == 0:
            return 0.0
        
        A_local = A_estimates[robot_id]
        total_disagreement = 0.0
        
        for neighbor_id in neighbors:
            if neighbor_id not in A_estimates:
                continue
            
            A_neighbor = A_estimates[neighbor_id]
            diff = A_local - A_neighbor
            frobenius_norm = np.sqrt(np.sum(diff ** 2))
            total_disagreement += frobenius_norm
        
        return total_disagreement / len(neighbors) if len(neighbors) > 0 else 0.0
    
    def simulate_disagreement_after_edge_removal(
        self,
        robot_id: int,
        edge_to_remove: Tuple[int, int],
        A_estimates: Dict[int, np.ndarray],
        current_neighbors: Set[int],
        positions: np.ndarray,
        sigma: float = 1.0,
        T_d: float = 0.2
    ) -> float:
        """Simulate max disagreement after removing an edge.
        
        Args:
            robot_id: Robot making the decision
            edge_to_remove: Edge (i, j) being considered for removal
            A_estimates: Current adjacency estimates
            current_neighbors: Current neighbors of robot_id
            positions: Robot positions for distance computation
            sigma: Trust function parameter
            T_d: Consensus sample time
            
        Returns:
            Predicted D_l(k+1) after edge removal
        """
        i, j = edge_to_remove
        
        # Determine new neighbor set after edge removal
        if robot_id == i:
            neighbors_after = current_neighbors - {j}
        elif robot_id == j:
            neighbors_after = current_neighbors - {i}
        else:
            neighbors_after = current_neighbors.copy()
        
        # If no neighbors left, disagreement becomes undefined
        if len(neighbors_after) == 0:
            return float('inf')
        
        # Simulate consensus update with modified neighbor set
        A_local = A_estimates[robot_id].copy()
        n = A_local.shape[0]
        
        # Consensus term
        delta_consensus = np.zeros((n, n))
        weight = 1.0 / len(neighbors_after)
        
        for neighbor_id in neighbors_after:
            A_neighbor = A_estimates[neighbor_id]
            delta_consensus += weight * (A_neighbor - A_local)
        
        # Observation term
        E = np.zeros((n, n))
        for neighbor_id in neighbors_after:
            distance = np.linalg.norm(positions[robot_id] - positions[neighbor_id])
            observed_quality = np.exp(-(distance ** 2) / (sigma ** 2))
            
            E[robot_id, neighbor_id] = observed_quality - A_local[robot_id, neighbor_id]
            E[neighbor_id, robot_id] = observed_quality - A_local[neighbor_id, robot_id]
        
        # Simulated update
        A_simulated = A_local + T_d * (delta_consensus + E)
        A_simulated = np.clip(A_simulated, 0, 1)
        A_simulated = (A_simulated + A_simulated.T) / 2
        np.fill_diagonal(A_simulated, 0)
        
        # Compute max disagreement with simulated estimate
        max_disagreement = 0.0
        for neighbor_id in neighbors_after:
            A_neighbor_sim = A_estimates[neighbor_id].copy()
            diff = A_simulated - A_neighbor_sim
            frobenius_norm = np.sqrt(np.sum(diff ** 2))
            max_disagreement = max(max_disagreement, frobenius_norm)
        
        return max_disagreement
    
    def check_pruning_condition(
        self,
        robot_id: int,
        edge_to_remove: Tuple[int, int],
        A_estimates: Dict[int, np.ndarray],
        current_neighbors: Set[int],
        positions: np.ndarray,
        delta_threshold: float,
        absolute_threshold: float,
        sigma: float = 1.0,
        T_d: float = 0.2
    ) -> Tuple[bool, Dict[str, float]]:
        """Check if edge can be removed using max disagreement criterion.
        
        Two conditions:
        1. Relative: D_l(k) - D_l(k+1) > δ (must improve)
        2. Absolute: D_l(k+1) < D_threshold (consensus quality good enough)
        
        Args:
            robot_id: Robot making decision
            edge_to_remove: Edge being considered
            A_estimates: Current estimates
            current_neighbors: Current neighbors
            positions: Robot positions
            delta_threshold: Required improvement threshold
            absolute_threshold: Maximum acceptable disagreement
            sigma: Trust function parameter
            T_d: Sample time
            
        Returns:
            (can_prune, metrics_dict)
        """
        # Compute current disagreement
        D_current, worst_neighbor = self.compute_max_disagreement(
            robot_id, A_estimates, current_neighbors
        )
        
        # Simulate disagreement after edge removal
        D_after = self.simulate_disagreement_after_edge_removal(
            robot_id=robot_id,
            edge_to_remove=edge_to_remove,
            A_estimates=A_estimates,
            current_neighbors=current_neighbors,
            positions=positions,
            sigma=sigma,
            T_d=T_d
        )
        
        # Check both conditions
        improvement = D_current - D_after
        relative_check = improvement > delta_threshold
        absolute_check = D_after < absolute_threshold
        
        can_prune = relative_check and absolute_check
        
        metrics = {
            'D_current': D_current,
            'D_after': D_after,
            'improvement': improvement,
            'delta_threshold': delta_threshold,
            'absolute_threshold': absolute_threshold,
            'relative_check': relative_check,
            'absolute_check': absolute_check,
            'worst_neighbor': worst_neighbor
        }
        
        return can_prune, metrics
    
    def get_convergence_status(
        self, 
        robot_id: int, 
        window_size: int = 10
    ) -> Dict[str, float]:
        """Get convergence statistics for a robot.
        
        Args:
            robot_id: Robot to analyze
            window_size: Number of recent iterations to analyze
            
        Returns:
            Convergence statistics dictionary
        """
        if robot_id not in self.disagreement_history:
            return {}
        
        history = self.disagreement_history[robot_id]
        if len(history) < 2:
            return {'iterations': len(history)}
        
        recent = history[-window_size:] if len(history) >= window_size else history
        
        # Compute convergence rate
        if len(recent) > 1:
            rate = (recent[0] - recent[-1]) / len(recent)
        else:
            rate = 0.0
        
        return {
            'iterations': len(history),
            'current_value': history[-1],
            'initial_value': self.initial_disagreement.get(robot_id, 0.0),
            'improvement_rate': rate,
            'recent_mean': np.mean(recent),
            'recent_std': np.std(recent),
            'percent_converged': (1 - history[-1] / self.initial_disagreement.get(robot_id, 1.0)) * 100
        }
    
    def reset(self):
        """Reset all stored history."""
        for robot_id in range(self.num_robots):
            self.disagreement_history[robot_id] = []
            self.worst_neighbor_history[robot_id] = []
        self.initial_disagreement.clear()
