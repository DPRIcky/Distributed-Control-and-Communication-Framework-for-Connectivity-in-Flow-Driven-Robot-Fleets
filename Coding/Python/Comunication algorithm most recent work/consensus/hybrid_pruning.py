"""Hybrid pruning manager with multi-layer robustness."""

from typing import Dict, List, Optional, Set

import numpy as np

from config.consensus_config import ConsensusConfig
from core.types import Edge
from graph.edge_analysis import EdgeAnalyzer
from consensus.adjacency_consensus import AdjacencyMatrixConsensus
from graph.lambda2_estimator import AdaptiveLambda2Manager
from distributed_consensus_pruning import ConsensusPruningSimulation


class HybridPruningManager:
    """Multi-layer robustness consensus pruning manager."""

    def __init__(self, config: ConsensusConfig, num_robots: int):
        """Initialize hybrid pruning manager.
        
        Args:
            config: Consensus configuration
            num_robots: Number of robots in system
        """
        self.config = config
        self.num_robots = num_robots
        
        # Topology stability tracking
        self.topology_stable_rounds = 0
        self.topology_history: List[Dict] = []
        
        # Consensus state
        self.consensus_active = False
        self.consensus_candidate: Optional[Edge] = None
        self.consensus_round = 0
        self.failed_candidates: Set[Edge] = set()
        
        # Edge analyzer for redundancy checks
        self.edge_analyzer = EdgeAnalyzer(num_robots)
        
        # Distributed consensus simulation
        self.consensus_sim = ConsensusPruningSimulation(
            num_nodes=num_robots,
            verbose=False
        )
        
        # Adjacency matrix consensus (Griparic et al., 2022)
        self.adjacency_consensus = AdjacencyMatrixConsensus(
            num_robots=num_robots,
            sigma=config.consensus_sigma,
            sample_time=config.consensus_sample_time,
            convergence_epsilon=config.consensus_convergence_epsilon
        )
        
        # Max consensus iterations before forcing convergence
        # For online operation with moving robots, use practical limit
        self.max_consensus_iterations = 50  # Reduced for online operation
        
        # Adaptive lambda2 estimation
        self.lambda2_manager = AdaptiveLambda2Manager(
            num_robots=num_robots,
            lambda2_threshold=config.lambda2_threshold,
            safety_margin=config.lambda2_safety_margin,
            max_incremental_updates=5
        )
        
        # Consensus convergence tracking
        self.consensus_converged = False
        self.consensus_iterations = 0

    def reset_on_topology_change(self) -> None:
        """Reset state when topology changes."""
        self.topology_stable_rounds = 0
        self.failed_candidates.clear()
        
        if self.consensus_active:
            self.consensus_active = False
            self.consensus_candidate = None
            self.consensus_round = 0
        
        # Reset consensus convergence
        self.consensus_converged = False
        self.consensus_iterations = 0

    def increment_stability(self) -> None:
        """Increment stability counter."""
        self.topology_stable_rounds += 1

    def add_topology_snapshot(
        self,
        edges: Set[Edge],
        lengths: Dict[Edge, float],
        timestamp: float
    ) -> None:
        """Add snapshot to rolling window.
        
        Args:
            edges: Current edges
            lengths: Edge lengths
            timestamp: Current time
        """
        self.topology_history.append({
            'edges': edges,
            'lengths': lengths,
            'timestamp': timestamp
        })
        
        # Maintain window size
        if len(self.topology_history) > self.config.rolling_window_size:
            self.topology_history.pop(0)

    def is_stable(self) -> bool:
        """Check if topology is stable."""
        return self.topology_stable_rounds >= self.config.stability_threshold

    def has_sufficient_history(self) -> bool:
        """Check if sufficient history collected."""
        return len(self.topology_history) >= self.config.rolling_window_size
    
    def run_consensus_phase(
        self,
        positions: np.ndarray,
        edge_lengths: Dict[Edge, float],
        current_edges: Set[Edge]
    ) -> bool:
        """Run adjacency matrix consensus until convergence.
        
        This implements the distributed topology estimation from Griparic et al. (2022).
        Each robot maintains a local estimate of the global adjacency matrix and updates
        it via consensus with neighbors.
        
        Args:
            positions: Robot positions (n x 2)
            edge_lengths: Current edge lengths
            current_edges: Current communication graph edges
            
        Returns:
            True if consensus converged, False otherwise
        """
        # Compute pairwise distances for all robots
        n = self.num_robots
        distances = np.zeros((n, n))
        for i in range(n):
            for j in range(i+1, n):
                dist = np.linalg.norm(positions[i] - positions[j])
                distances[i, j] = dist
                distances[j, i] = dist
        
        # Build neighbor sets for each robot
        neighbors_per_robot: Dict[int, Set[int]] = {i: set() for i in range(n)}
        for edge in current_edges:
            i, j = edge
            neighbors_per_robot[i].add(j)
            neighbors_per_robot[j].add(i)
        
        # Each robot performs consensus update
        for robot_id in range(n):
            # Build direct observations for this robot
            direct_observations: Dict[tuple[int, int], float] = {}
            
            # Robot observes all its incident edges
            for neighbor_id in neighbors_per_robot[robot_id]:
                edge_key = (robot_id, neighbor_id)
                reverse_key = (neighbor_id, robot_id)
                
                # Get distance
                distance = distances[robot_id, neighbor_id]
                
                # Add both directions
                direct_observations[edge_key] = distance
                direct_observations[reverse_key] = distance
            
            # Perform consensus update for this robot
            self.adjacency_consensus.consensus_update_step(
                robot_id=robot_id,
                neighbors=neighbors_per_robot[robot_id],
                direct_observations=direct_observations
            )
        
        # Check convergence
        converged = self.adjacency_consensus.check_convergence()
        
        self.consensus_iterations += 1
        
        if converged:
            self.consensus_converged = True
            return True
        
        # Check max iterations - also mark as converged for practical purposes
        if self.consensus_iterations >= self.max_consensus_iterations:
            self.consensus_converged = True
            return True
        
        return False
    
    def get_consensus_estimate(self, robot_id: int) -> np.ndarray:
        """Get robot's current adjacency matrix estimate.
        
        Args:
            robot_id: Robot requesting estimate
            
        Returns:
            n x n adjacency matrix estimate
        """
        return self.adjacency_consensus.A_estimates[robot_id]

    def find_persistent_redundant_edge(self, debug: bool = False) -> Optional[Edge]:
        """Find edge that's redundant in current topology and recent history.
        
        LEGACY METHOD - Uses global knowledge (edge_set parameter).
        For distributed approach, use find_redundant_edge_distributed().
        
        Args:
            debug: Enable debug output
            
        Returns:
            Redundant edge or None
        """
        if debug:
            print("[LEGACY] Using find_persistent_redundant_edge (global knowledge)")
        
        if len(self.topology_history) < self.config.rolling_window_size:
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
            has_alt_path = self.edge_analyzer.has_alternative_path(edge, current_edges)

            if not has_alt_path:
                if debug:
                    rejected[edge] = "No alternative path"
                continue

            # CRITICAL CHECK 2: Does graph remain connected after removing this edge?
            graph_stays_connected = self.edge_analyzer.check_graph_connectivity(
                current_edges - {edge}
            )

            if not graph_stays_connected:
                if debug:
                    rejected[edge] = "Graph disconnects"
                continue

            # OPTIONAL CHECK: Verify redundancy is stable across recent history
            # Count how many recent snapshots this edge was redundant in
            redundant_count = 0
            for snap in self.topology_history[-5:]:  # Check last 5 snapshots
                if edge in snap['edges']:
                    if self.edge_analyzer.has_alternative_path(edge, snap['edges']):
                        if self.edge_analyzer.check_graph_connectivity(snap['edges'] - {edge}):
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
    
    def find_redundant_edge_distributed(
        self,
        robot_id: int,
        debug: bool = False
    ) -> Optional[Edge]:
        """Find redundant edge using ONLY robot's local consensus estimate.
        
        This is the fully distributed approach from Griparic et al. (2022).
        Robot uses its converged adjacency matrix estimate to make pruning decision.
        
        NOVEL: Uses adjacency matrix consensus + adaptive lambda2 estimation.
        
        Args:
            robot_id: Robot making the decision
            debug: Enable debug output
            
        Returns:
            Redundant edge or None
        """
        if debug:
            print(f"[NOVEL] Using find_redundant_edge_distributed (consensus-based)")
        
        if not self.consensus_converged:
            if debug:
                print(f"[Robot {robot_id}] Consensus not converged yet")
            return None
        
        # Get this robot's estimate of the adjacency matrix
        A_estimate = self.get_consensus_estimate(robot_id)
        
        # Extract edges from consensus estimate
        edges_from_consensus = self.edge_analyzer.extract_edges_from_consensus(
            A_estimate,
            threshold=self.config.edge_quality_threshold
        )
        
        if not edges_from_consensus:
            return None
        
        # Filter out blacklisted edges
        candidate_edges = edges_from_consensus - self.failed_candidates
        
        if not candidate_edges:
            if debug:
                print(f"[Robot {robot_id}] No candidate edges (all blacklisted)")
            return None
        
        # Find edges that are redundant according to consensus estimate
        candidates = []
        rejected = {}
        
        for edge in candidate_edges:
            # Check alternative path using consensus
            has_alt_path = self.edge_analyzer.has_alternative_path_from_consensus(
                robot_id=robot_id,
                edge=edge,
                A_estimate=A_estimate,
                threshold=self.config.edge_quality_threshold
            )
            
            if not has_alt_path:
                if debug:
                    rejected[edge] = "No alternative path (consensus)"
                continue
            
            # Check connectivity using consensus (remove edge from estimate)
            A_test = A_estimate.copy()
            i, j = edge
            A_test[i, j] = 0.0
            A_test[j, i] = 0.0
            
            is_connected = self.edge_analyzer.check_connectivity_from_consensus(
                robot_id=robot_id,
                A_estimate=A_test,
                threshold=self.config.edge_quality_threshold
            )
            
            if not is_connected:
                if debug:
                    rejected[edge] = "Graph disconnects (consensus)"
                continue
            
            # Check lambda2 is above threshold
            lambda2_value = self.lambda2_manager.get_lambda2(A_test, mode='auto')
            
            if debug:
                print(f"  Lambda2 check: {lambda2_value:.6f} (threshold: {self.config.lambda2_threshold})")
            
            if lambda2_value < self.config.lambda2_threshold:
                if debug:
                    rejected[edge] = f"Lambda2 too low ({lambda2_value:.4f})"
                continue
            
            # Estimate edge strength from consensus
            i, j = edge
            edge_strength = A_estimate[i, j]
            
            # All checks passed
            candidates.append((edge, edge_strength))
        
        if debug and rejected:
            print(f"\n[Robot {robot_id}] Rejected {len(rejected)} edges:")
            for edge, reason in list(rejected.items())[:5]:
                print(f"  {edge}: {reason}")
        
        if not candidates:
            return None
        
        # Sort by edge strength (remove weakest edges first)
        candidates.sort(key=lambda x: x[1])
        
        if debug:
            print(f"[Robot {robot_id}] Found {len(candidates)} redundant candidates")
            print(f"[Robot {robot_id}] Selected: {candidates[0][0]} (strength={candidates[0][1]:.3f})")
        
        return candidates[0][0]

    def start_consensus(self, candidate: Edge) -> None:
        """Start consensus on a candidate edge.
        
        Args:
            candidate: Edge to evaluate
        """
        self.consensus_active = True
        self.consensus_candidate = candidate
        self.consensus_round = 0

    def run_consensus_rounds(
        self,
        positions: np.ndarray,
        edge_lengths: Dict[Edge, float],
        first_round: bool
    ) -> Optional[Edge]:
        """Run distributed consensus steps.
        
        Args:
            positions: Robot positions
            edge_lengths: Current edge lengths
            first_round: Whether this is the first round for this candidate
            
        Returns:
            Removed edge if consensus reached, None otherwise
        """
        # If first round for this candidate, initialize consensus state
        if first_round:
            self.consensus_sim.load_external_state(positions, edge_lengths)
        else:
            # Update positions and edge lengths to handle robot drift
            self.consensus_sim.positions = positions
            for edge in list(self.consensus_sim.edge_lengths.keys()):
                if edge in edge_lengths:
                    self.consensus_sim.edge_lengths[edge] = edge_lengths[edge]

        # Run distributed consensus steps
        removed_edge = None
        for _ in range(self.config.consensus_rounds_per_step):
            report = self.consensus_sim.step()
            if report.get('decision') == 'prune':
                removed_edge = report.get('removed_edge')
                break

        self.consensus_round += self.config.consensus_rounds_per_step
        return removed_edge

    def mark_candidate_failed(self) -> Edge:
        """Mark current candidate as failed and reset.
        
        Returns:
            The failed candidate edge
        """
        failed_edge = self.consensus_candidate
        self.failed_candidates.add(failed_edge)
        self.consensus_active = False
        self.consensus_candidate = None
        self.consensus_round = 0
        return failed_edge

    def reset_after_successful_prune(self) -> None:
        """Reset state after successful edge pruning."""
        self.consensus_active = False
        self.consensus_candidate = None
        self.consensus_round = 0
        self.topology_stable_rounds = 0
        self.topology_history.clear()
        self.failed_candidates.clear()
        
        # Reset consensus state - topology changed
        self.consensus_converged = False
        self.consensus_iterations = 0
        # Note: adjacency_consensus and lambda2_manager will be updated on next consensus phase

    def abort_consensus(self) -> None:
        """Abort current consensus."""
        self.consensus_active = False
        self.consensus_candidate = None
        self.consensus_round = 0
        self.topology_stable_rounds = 0
        
        # Note: Do NOT reset consensus_converged - may still be valid
