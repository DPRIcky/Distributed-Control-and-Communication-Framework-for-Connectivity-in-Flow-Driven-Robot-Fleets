"""Metrics collection for simulation analysis."""

import numpy as np
from typing import Dict, List, Optional
import json


class MetricsCollector:
    """Collects and stores simulation metrics for analysis."""
    
    def __init__(self, num_robots: int):
        """Initialize metrics collector.
        
        Args:
            num_robots: Number of robots in the simulation
        """
        self.num_robots = num_robots
        self.reset()
    
    def reset(self):
        """Reset all metrics."""
        # Temporal data (time series)
        self.time_series = []
        self.edge_counts = []
        self.lambda2_values = []
        self.min_distances = []
        self.max_edge_distances = []
        self.control_magnitudes = []  # List of arrays (per robot)
        self.clf_values = []  # List of arrays (per robot)
        self.goal_distances = []  # List of arrays (per robot)
        
        # Topology data
        self.edge_history = []
        self.pruning_events = []
        
        # Consensus data
        self.consensus_iterations_history = []
        self.consensus_errors = []
        self.unanimous_decisions = []
        
        # Robot positions
        self.position_history = []
        
        # Flow field data
        self.flow_vectors = []
        self.flow_magnitudes = []
        
        # Safety violations
        self.safety_violations = []
        self.connectivity_violations = []
    
    def record_step(self, sim, time: float):
        """Record metrics for one simulation step.
        
        Args:
            sim: HybridUnderwaterSimulation instance
            time: Current simulation time
        """
        # Time
        self.time_series.append(time)
        
        # Topology
        edges = sim.current_edges()
        self.edge_counts.append(len(edges))
        self.edge_history.append(list(edges))
        
        # Lambda2 (if available)
        try:
            # Get adjacency matrix from topology manager
            positions = np.array([r.position for r in sim.robots])
            A = self._compute_adjacency_matrix(edges, self.num_robots, sim.communication_radius, positions)
            
            # Compute Laplacian and check connectivity
            L = self._compute_laplacian(A)
            eigenvalues = np.linalg.eigvalsh(L)
            
            # λ₂ is the second smallest eigenvalue
            # λ₁ should be 0 (or very close due to numerical precision)
            # λ₂ > 0 indicates connected graph
            # λ₂ ≈ 0 indicates disconnected graph
            if len(eigenvalues) > 1:
                lambda2 = eigenvalues[1]
                # Clean up numerical noise: values < 1e-10 are effectively 0
                if abs(lambda2) < 1e-10:
                    lambda2 = 0.0
                # Clamp any negative values (numerical errors) to 0
                elif lambda2 < 0:
                    lambda2 = 0.0
                self.lambda2_values.append(float(lambda2))
            else:
                self.lambda2_values.append(0.0)
        except Exception:
            self.lambda2_values.append(np.nan)
        
        # Positions
        positions = np.array([r.position for r in sim.robots])
        self.position_history.append(positions.copy())
        
        # Safety: minimum inter-robot distance
        min_dist = float('inf')
        safety_violations = 0
        d_min = 0.6  # Safety threshold
        
        for i in range(self.num_robots):
            for j in range(i+1, self.num_robots):
                dist = np.linalg.norm(positions[i] - positions[j])
                min_dist = min(min_dist, dist)
                if dist < d_min:
                    safety_violations += 1
        
        self.min_distances.append(float(min_dist))
        self.safety_violations.append(safety_violations)
        
        # Connectivity: max distance for active edges
        max_edge_dist = 0.0
        connectivity_violations = 0
        R_max = sim.communication_radius
        
        for (i, j) in edges:
            dist = np.linalg.norm(positions[i] - positions[j])
            max_edge_dist = max(max_edge_dist, dist)
            if dist > R_max:
                connectivity_violations += 1
        
        self.max_edge_distances.append(float(max_edge_dist))
        self.connectivity_violations.append(connectivity_violations)
        
        # Goal distances (per robot)
        goal_dists = np.array([
            np.linalg.norm(r.position - sim.goal_position) 
            for r in sim.robots
        ])
        self.goal_distances.append(goal_dists.copy())
        
        # CLF values (per robot)
        clf_vals = goal_dists ** 2
        self.clf_values.append(clf_vals.copy())
        
        # Flow field at robot positions
        flow_vecs = np.array([
            sim.flow.velocity(r.position, time) 
            for r in sim.robots
        ])
        self.flow_vectors.append(flow_vecs.copy())
        self.flow_magnitudes.append(np.linalg.norm(flow_vecs, axis=1))
        
        # Control magnitudes (initialized to zeros, will be filled by record_control)
        self.control_magnitudes.append(np.zeros(self.num_robots))
    
    def record_control(self, robot_id: int, control: np.ndarray):
        """Record control input for a robot.
        
        Args:
            robot_id: ID of the robot
            control: Control vector
        """
        if len(self.control_magnitudes) > 0:
            self.control_magnitudes[-1][robot_id] = np.linalg.norm(control)
    
    def record_pruning_event(self, time: float, edge, reason: str = "consensus"):
        """Record edge pruning event.
        
        Args:
            time: Time of pruning
            edge: Edge that was pruned
            reason: Reason for pruning
        """
        self.pruning_events.append({
            'time': float(time),
            'edge': list(edge) if isinstance(edge, tuple) else edge,
            'reason': reason
        })
    
    def record_consensus(self, iterations: int, unanimous: bool, error: float = 0.0):
        """Record consensus outcome.
        
        Args:
            iterations: Number of iterations to converge
            unanimous: Whether unanimous agreement was reached
            error: Final consensus error
        """
        self.consensus_iterations_history.append(iterations)
        self.unanimous_decisions.append(unanimous)
        self.consensus_errors.append(float(error))
    
    def _compute_adjacency_matrix(self, edges, num_robots, R_max, positions):
        """Compute adjacency matrix from edges.
        
        Args:
            edges: Set of edges
            num_robots: Number of robots
            R_max: Communication radius
            positions: Robot positions
            
        Returns:
            Adjacency matrix
        """
        A = np.zeros((num_robots, num_robots))
        for (i, j) in edges:
            dist = np.linalg.norm(positions[i] - positions[j])
            weight = np.exp(-dist**2 / (R_max**2))
            A[i, j] = weight
            A[j, i] = weight
        return A
    
    def _compute_laplacian(self, A):
        """Compute Laplacian matrix from adjacency.
        
        Args:
            A: Adjacency matrix
            
        Returns:
            Laplacian matrix
        """
        D = np.diag(np.sum(A, axis=1))
        L = D - A
        return L
    
    def get_summary_statistics(self) -> Dict:
        """Compute summary statistics.
        
        Returns:
            Dictionary of summary statistics
        """
        # Count graph disconnections (λ₂ ≤ threshold)
        lambda2_threshold = 0.01  # Effectively 0 considering numerical precision
        disconnections = sum(1 for l2 in self.lambda2_values if not np.isnan(l2) and l2 <= lambda2_threshold)
        
        return {
            'total_steps': len(self.time_series),
            'total_time': self.time_series[-1] if self.time_series else 0.0,
            'initial_edges': self.edge_counts[0] if self.edge_counts else 0,
            'final_edges': self.edge_counts[-1] if self.edge_counts else 0,
            'edge_reduction_pct': (1 - self.edge_counts[-1]/self.edge_counts[0])*100 if self.edge_counts and self.edge_counts[0] > 0 else 0,
            'pruning_events': len(self.pruning_events),
            'min_distance_ever': float(np.min(self.min_distances)) if self.min_distances else 0,
            'safety_violations_total': sum(self.safety_violations),
            'connectivity_violations_total': sum(self.connectivity_violations),
            'graph_disconnections': disconnections,  # NEW: Track when graph fragments
            'mean_lambda2': float(np.nanmean(self.lambda2_values)) if self.lambda2_values else 0,
            'min_lambda2': float(np.nanmin(self.lambda2_values)) if self.lambda2_values else 0,
            'final_goal_distance_mean': float(np.mean(self.goal_distances[-1])) if self.goal_distances else 0,
        }
    
    def save(self, filepath: str):
        """Save metrics to JSON.
        
        Args:
            filepath: Path to save JSON file
        """
        # Convert numpy arrays to lists for JSON serialization
        data = {
            'time_series': [float(t) for t in self.time_series],
            'edge_counts': [int(e) for e in self.edge_counts],
            'lambda2_values': [float(l) if not np.isnan(l) else None for l in self.lambda2_values],
            'min_distances': [float(d) for d in self.min_distances],
            'max_edge_distances': [float(d) for d in self.max_edge_distances],
            'safety_violations': [int(v) for v in self.safety_violations],
            'connectivity_violations': [int(v) for v in self.connectivity_violations],
            'pruning_events': self.pruning_events,
            'consensus_iterations_history': [int(i) for i in self.consensus_iterations_history],
            'unanimous_decisions': [bool(u) for u in self.unanimous_decisions],
            'summary_statistics': self.get_summary_statistics(),
        }
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
    
    def save_arrays(self, filepath: str):
        """Save numpy arrays (positions, controls, etc.).
        
        Args:
            filepath: Path to save NPZ file
        """
        np.savez(filepath,
                 positions=np.array(self.position_history),
                 goal_distances=np.array(self.goal_distances),
                 clf_values=np.array(self.clf_values),
                 flow_vectors=np.array(self.flow_vectors),
                 flow_magnitudes=np.array(self.flow_magnitudes),
                 control_magnitudes=np.array(self.control_magnitudes))
