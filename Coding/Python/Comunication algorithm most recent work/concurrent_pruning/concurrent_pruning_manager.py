"""Concurrent consensus and pruning manager with DISTRIBUTED COORDINATION.

This module orchestrates simultaneous adjacency consensus and edge pruning
with Lyapunov-constrained or max-disagreement-constrained decisions.

Key Innovation:
    Traditional: [Consensus → Convergence → Pruning] (sequential)
    Concurrent: [Consensus + Pruning] (simultaneous with safety guarantees)

DISTRIBUTED COORDINATION PROTOCOL:
    Phase 1 - PROPOSAL: Each robot independently evaluates its incident edges
              - Uses only local consensus estimate A^l(k)
              - Checks: alternative path, connectivity, λ2, Lyapunov/disagreement
    
    Phase 2 - NEGOTIATION: Both endpoint robots must agree to prune an edge
              - Bilateral consensus protocol
              - Edge (i,j) pruned only if BOTH i and j proposed it
    
    Phase 3 - CONFLICT RESOLUTION: Deterministic tie-breaking when multiple edges approved
              - Rule 1: Prune longest edge (weakest link)
              - Rule 2: If tie, prune edge with lowest robot ID sum
              - Rule 3: If still tie, prune edge with lower first vertex ID
              - All robots independently compute same result

Note: Message passing is assumed ideal (instantaneous, reliable). The focus is on
      distributed decision-making logic, not communication protocols.
"""

import sys
from pathlib import Path
from typing import Dict, Set, Tuple, Optional, List
import numpy as np

# Add parent directory to path for imports
parent_dir = Path(__file__).parent.parent
if str(parent_dir) not in sys.path:
    sys.path.insert(0, str(parent_dir))

try:
    from consensus.adjacency_consensus import AdjacencyMatrixConsensus
    from graph.edge_analysis import EdgeAnalyzer
    from graph.lambda2_estimator import AdaptiveLambda2Manager
    from core.types import Edge
except ImportError:
    # Fallback for different import contexts
    import os
    import importlib.util
    
    # Try to import from parent directory
    parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, parent_path)
    
    from consensus.adjacency_consensus import AdjacencyMatrixConsensus
    from graph.edge_analysis import EdgeAnalyzer
    from graph.lambda2_estimator import AdaptiveLambda2Manager
    
    # Define Edge type if import fails
    try:
        from core.types import Edge
    except ImportError:
        Edge = Tuple[int, int]

# Try relative imports first (when used as package), fall back to absolute
try:
    from .local_lyapunov import LocalLyapunovMonitor
    from .max_disagreement import MaxDisagreementMonitor
    from .adaptive_thresholds import AdaptiveThresholdScheduler
except ImportError:
    # Fallback for direct execution
    from local_lyapunov import LocalLyapunovMonitor
    from max_disagreement import MaxDisagreementMonitor
    from adaptive_thresholds import AdaptiveThresholdScheduler


class ConcurrentPruningManager:
    """Manage concurrent consensus and topology pruning with DISTRIBUTED COORDINATION.
    
    This manager runs adjacency matrix consensus while simultaneously
    evaluating and pruning redundant edges using distributed decision-making:
    
    1. Each robot independently proposes edges to prune
    2. Both endpoint robots must agree (bilateral negotiation)
    3. Conflicts resolved through deterministic tie-breaking rules
    
    Attributes:
        num_robots: Number of robots
        mode: 'lyapunov', 'max_disagreement', or 'hybrid'
        adjacency_consensus: Consensus protocol manager
        lyapunov_monitor: Local Lyapunov function tracking (if applicable)
        disagreement_monitor: Max disagreement tracking (if applicable)
        threshold_scheduler: Adaptive threshold management
        edge_analyzer: Edge redundancy and connectivity checks
        lambda2_manager: Algebraic connectivity estimation
    """
    
    def __init__(
        self,
        num_robots: int,
        mode: str = 'lyapunov',
        sigma: float = 1.0,
        sample_time: float = 0.2,
        lambda2_threshold: float = 0.3,
        lambda2_safety_margin: float = 0.1,
        k_max: int = 100,
        lyapunov_sim_steps: int = 1
    ):
        """Initialize concurrent pruning manager.
        
        Args:
            num_robots: Number of robots in system
            mode: Pruning constraint mode ('lyapunov', 'max_disagreement', 'hybrid')
            sigma: Trust function parameter
            sample_time: Consensus sample time T_d
            lambda2_threshold: Minimum algebraic connectivity
            lambda2_safety_margin: Safety margin for λ2
            k_max: Expected convergence horizon
            lyapunov_sim_steps: Simulated consensus steps for Lyapunov pruning check
        """
        self.num_robots = num_robots
        self.mode = mode
        self.sigma = sigma
        self.T_d = sample_time
        self.lambda2_threshold = lambda2_threshold
        self.lyapunov_sim_steps = lyapunov_sim_steps
        
        # Adjacency matrix consensus
        self.adjacency_consensus = AdjacencyMatrixConsensus(
            num_robots=num_robots,
            sigma=sigma,
            sample_time=sample_time,
            convergence_epsilon=1e-4
        )
        
        # Monitoring systems
        self.lyapunov_monitor = None
        self.disagreement_monitor = None
        
        if mode in ['lyapunov', 'hybrid']:
            self.lyapunov_monitor = LocalLyapunovMonitor(num_robots)
        
        if mode in ['max_disagreement', 'hybrid']:
            self.disagreement_monitor = MaxDisagreementMonitor(num_robots)
        
        # Threshold scheduler
        self.threshold_scheduler = AdaptiveThresholdScheduler(
            k_max=k_max,
            mode=mode,
            lambda2_params={'mu_max': lambda2_safety_margin, 'mu_min': 0.05}
        )
        
        # Edge analysis
        self.edge_analyzer = EdgeAnalyzer(num_robots)
        self.lambda2_manager = AdaptiveLambda2Manager(
            num_robots=num_robots,
            lambda2_threshold=lambda2_threshold,
            safety_margin=lambda2_safety_margin
        )
        
        # State tracking
        self.pruned_edges: List[Edge] = []
        self.current_iteration = 0
        self.pruning_history: List[Dict] = []
    
    def initialize_robot_knowledge(
        self,
        positions: np.ndarray,
        current_edges: Set[Edge]
    ):
        """Initialize each robot's local adjacency estimate.
        
        Args:
            positions: Robot positions (n x 2)
            current_edges: Initial communication edges
        """
        # Build neighbor sets and distances
        neighbors_per_robot: Dict[int, Set[int]] = {i: set() for i in range(self.num_robots)}
        edge_distances: Dict[Edge, float] = {}
        
        for edge in current_edges:
            i, j = edge
            neighbors_per_robot[i].add(j)
            neighbors_per_robot[j].add(i)
            
            distance = np.linalg.norm(positions[i] - positions[j])
            edge_distances[edge] = distance
        
        # Initialize each robot's estimate
        for robot_id in range(self.num_robots):
            neighbor_distances = {}
            for neighbor_id in neighbors_per_robot[robot_id]:
                edge = (min(robot_id, neighbor_id), max(robot_id, neighbor_id))
                neighbor_distances[neighbor_id] = edge_distances.get(edge, 0.0)
            
            self.adjacency_consensus.initialize_robot_knowledge(
                robot_id=robot_id,
                neighbors=neighbors_per_robot[robot_id],
                neighbor_distances=neighbor_distances
            )
    
    def concurrent_step(
        self,
        positions: np.ndarray,
        current_edges: Set[Edge],
        debug: bool = False
    ) -> Dict:
        """Execute one concurrent consensus + pruning step with DISTRIBUTED COORDINATION.
        
        Distributed Protocol:
        1. Update adjacency estimates (consensus step) - each robot independently
        2. Compute convergence metrics (Lyapunov/disagreement) - each robot locally
        3. PROPOSAL: Each robot evaluates its incident edges and proposes candidates
        4. NEGOTIATION: Both endpoint robots must agree to prune an edge
        5. CONFLICT RESOLUTION: Select one edge from multiple approved using deterministic rules
        6. Execute pruning if an edge was selected
        
        Args:
            positions: Current robot positions (n x 2)
            current_edges: Current communication edges
            debug: Enable debug output
            
        Returns:
            Step report dictionary
        """
        report = {
            'iteration': self.current_iteration,
            'edges_before': len(current_edges),
            'pruned_edge': None,
            'edges_after': len(current_edges),
            'phase': self.threshold_scheduler.get_phase_name(),
            'metrics': {}
        }
        
        # Build neighbor information
        neighbors_per_robot = self._build_neighbor_sets(current_edges)
        distances = self._compute_pairwise_distances(positions)
        
        # STEP 1: Consensus update for all robots
        for robot_id in range(self.num_robots):
            direct_observations = self._get_direct_observations(
                robot_id, neighbors_per_robot[robot_id], distances
            )
            
            self.adjacency_consensus.consensus_update_step(
                robot_id=robot_id,
                neighbors=neighbors_per_robot[robot_id],
                direct_observations=direct_observations
            )
        
        # STEP 2: Compute convergence metrics
        A_estimates = self.adjacency_consensus.A_estimates
        
        if self.mode in ['lyapunov', 'hybrid'] and self.lyapunov_monitor:
            lyapunov_values = {}
            for robot_id in range(self.num_robots):
                V_l = self.lyapunov_monitor.compute_local_lyapunov(
                    robot_id, A_estimates, neighbors_per_robot[robot_id]
                )
                lyapunov_values[robot_id] = V_l
            report['metrics']['lyapunov'] = lyapunov_values
            
            # Set initial values for threshold scheduler
            if self.current_iteration == 0:
                self.threshold_scheduler.set_initial_values(lyapunov_values=lyapunov_values)
        
        if self.mode in ['max_disagreement', 'hybrid'] and self.disagreement_monitor:
            disagreement_values = {}
            for robot_id in range(self.num_robots):
                D_l, worst = self.disagreement_monitor.compute_max_disagreement(
                    robot_id, A_estimates, neighbors_per_robot[robot_id]
                )
                disagreement_values[robot_id] = D_l
            report['metrics']['disagreement'] = disagreement_values
            
            if self.current_iteration == 0:
                self.threshold_scheduler.set_initial_values(disagreement_values=disagreement_values)
        
        # STEP 3: DISTRIBUTED PRUNING PROTOCOL (3 phases)
        #   Phase 1: Each robot proposes edges
        #   Phase 2: Both endpoints negotiate (must both agree)
        #   Phase 3: Conflict resolution if multiple edges approved
        candidate_edge, pruning_metrics = self._find_pruneable_edge(
            positions=positions,
            current_edges=current_edges,
            neighbors_per_robot=neighbors_per_robot,
            debug=debug
        )
        
        # STEP 4: Execute pruning if an edge was selected
        if candidate_edge:
            # Handle both edge directions (i,j) or (j,i)
            edge_to_remove = None
            for edge in current_edges:
                if (edge == candidate_edge or 
                    edge == (candidate_edge[1], candidate_edge[0])):
                    edge_to_remove = edge
                    break
            
            if edge_to_remove:
                current_edges.remove(edge_to_remove)
                self.pruned_edges.append(candidate_edge)
                report['pruned_edge'] = candidate_edge
                report['edges_after'] = len(current_edges)
                report['pruning_metrics'] = pruning_metrics
                
                if debug:
                    print(f"[Iteration {self.current_iteration}] Pruned edge {candidate_edge}")
                    print(f"  Phase: {report['phase']}")
                    print(f"  Metrics: {pruning_metrics}")
            elif debug:
                print(f"[Warning] Edge {candidate_edge} not found in current_edges set")
        
        # Update iteration counters
        self.current_iteration += 1
        self.threshold_scheduler.increment()
        
        # Store history
        self.pruning_history.append(report)
        
        return report
    
    def _find_pruneable_edge(
        self,
        positions: np.ndarray,
        current_edges: Set[Edge],
        neighbors_per_robot: Dict[int, Set[int]],
        debug: bool = False
    ) -> Tuple[Optional[Edge], Dict]:
        """Find an edge that can be safely pruned using DISTRIBUTED COORDINATION.
        
        Distributed Protocol:
        1. PROPOSAL: Each robot independently proposes edges to prune
        2. NEGOTIATION: Both endpoints must agree to prune an edge
        3. CONFLICT RESOLUTION: Select one edge from multiple approved proposals
        
        Tests each edge against:
        1. Lyapunov/disagreement constraint
        2. λ2 connectivity constraint
        3. Redundancy check (alternative path exists)
        
        Args:
            positions: Robot positions
            current_edges: Current edges
            neighbors_per_robot: Neighbor sets
            debug: Enable debug output
            
        Returns:
            (pruneable_edge, metrics) or (None, {})
        """
        # PHASE 1: PROPOSAL - Each robot independently proposes edges
        proposals = self._distributed_proposal_phase(
            positions, current_edges, neighbors_per_robot, debug
        )
        
        if debug and proposals:
            print(f"\n[PHASE 1: PROPOSALS] {len(proposals)} proposals from robots:")
            for robot_id, robot_proposals in proposals.items():
                if robot_proposals:
                    print(f"  Robot {robot_id}: {len(robot_proposals)} edges proposed")
        
        # PHASE 2: NEGOTIATION - Both endpoints must agree
        approved_edges = self._distributed_negotiation_phase(
            proposals, positions, current_edges, neighbors_per_robot, debug
        )
        
        if debug and approved_edges:
            print(f"\n[PHASE 2: NEGOTIATION] {len(approved_edges)} edges approved by both endpoints:")
            for edge_data in approved_edges:
                print(f"  Edge {edge_data['edge']}")
        
        # PHASE 3: CONFLICT RESOLUTION - Select one edge if multiple approved
        selected_edge, metrics = self._distributed_conflict_resolution(
            approved_edges, positions, debug
        )
        
        if debug and selected_edge:
            print(f"\n[PHASE 3: CONFLICT RESOLUTION] Selected edge {selected_edge}")
        
        return selected_edge, metrics
    
    def _distributed_proposal_phase(
        self,
        positions: np.ndarray,
        current_edges: Set[Edge],
        neighbors_per_robot: Dict[int, Set[int]],
        debug: bool = False
    ) -> Dict[int, List[Dict]]:
        """Phase 1: Each robot independently proposes edges to prune.
        
        DISTRIBUTED: Each robot operates independently using only:
        - Its local adjacency estimate A^l(k)
        - Its 1-hop neighbor information
        - Its local Lyapunov/disagreement metrics
        
        Args:
            positions: Robot positions
            current_edges: Current edges
            neighbors_per_robot: Neighbor sets
            debug: Enable debug output
            
        Returns:
            Dict mapping robot_id -> list of proposed edge data
        """
        proposals = {robot_id: [] for robot_id in range(self.num_robots)}
        A_estimates = self.adjacency_consensus.A_estimates
        
        # Each robot independently evaluates its INCIDENT edges
        for robot_id in range(self.num_robots):
            # Get edges incident to this robot
            incident_edges = [edge for edge in current_edges 
                            if edge[0] == robot_id or edge[1] == robot_id]
            
            if debug and incident_edges:
                print(f"\n  Robot {robot_id} evaluating {len(incident_edges)} incident edges")
            
            # Robot evaluates each incident edge independently
            for edge in incident_edges:
                i, j = edge
                
                # Skip if already pruned
                if edge in self.pruned_edges or (j, i) in self.pruned_edges:
                    continue
                
                # Robot checks using ITS OWN consensus estimate A^robot_id(k)
                proposal_valid, proposal_metrics = self._robot_evaluates_edge(
                    robot_id=robot_id,
                    edge=edge,
                    positions=positions,
                    current_edges=current_edges,
                    neighbors=neighbors_per_robot[robot_id]
                )
                
                if proposal_valid:
                    proposals[robot_id].append({
                        'edge': edge,
                        'proposer': robot_id,
                        'metrics': proposal_metrics
                    })
        
        return proposals
    
    def _robot_evaluates_edge(
        self,
        robot_id: int,
        edge: Edge,
        positions: np.ndarray,
        current_edges: Set[Edge],
        neighbors: Set[int]
    ) -> Tuple[bool, Dict]:
        """Single robot evaluates if an edge can be pruned (from its perspective).
        
        DISTRIBUTED: Uses only robot_id's local information:
        - A^robot_id(k): Its adjacency consensus estimate
        - N_robot_id: Its 1-hop neighbors
        - V_robot_id or D_robot_id: Its local convergence metric
        
        Args:
            robot_id: Robot making the evaluation
            edge: Edge to evaluate
            positions: Robot positions
            current_edges: Current edge set
            neighbors: Robot's neighbors
            
        Returns:
            (can_prune, metrics)
        """
        i, j = edge
        A_estimate = self.adjacency_consensus.A_estimates[robot_id]
        metrics = {}
        
        # Check 1: Alternative path exists (using robot's consensus estimate)
        has_alt_path = self.edge_analyzer.has_alternative_path(edge, current_edges)
        if not has_alt_path:
            return False, {'reason': 'no_alternative_path'}
        
        # Check 2: Graph stays connected (using robot's consensus estimate)
        edges_after = current_edges - {edge}
        stays_connected = self.edge_analyzer.check_graph_connectivity(edges_after)
        if not stays_connected:
            return False, {'reason': 'would_disconnect'}
        
        # Check 3: Lambda2 safety (robot computes from ITS estimate)
        A_test = A_estimate.copy()
        A_test[i, j] = 0.0
        A_test[j, i] = 0.0
        
        lambda2_margin = self.threshold_scheduler.get_lambda2_margin()
        lambda2_local = self.lambda2_manager.get_lambda2(A_test, mode='auto')
        
        if lambda2_local <= (self.lambda2_threshold + lambda2_margin):
            metrics['lambda2_local'] = lambda2_local
            metrics['lambda2_required'] = self.lambda2_threshold + lambda2_margin
            return False, {'reason': 'lambda2_unsafe', **metrics}
        
        metrics['lambda2_local'] = lambda2_local
        
        # Check 4: Lyapunov/disagreement constraint (robot's local metric)
        if self.mode in ['lyapunov', 'hybrid'] and self.lyapunov_monitor:
            can_prune, constraint_metrics = self._check_lyapunov_constraint(
                robot_id, edge, positions, neighbors
            )
            metrics.update(constraint_metrics)
            if not can_prune:
                return False, {'reason': 'lyapunov_constraint_failed', **metrics}
        
        if self.mode in ['max_disagreement', 'hybrid'] and self.disagreement_monitor:
            can_prune, constraint_metrics = self._check_disagreement_constraint(
                robot_id, edge, positions, neighbors
            )
            metrics.update(constraint_metrics)
            if not can_prune:
                return False, {'reason': 'disagreement_constraint_failed', **metrics}
        
        # All checks passed from this robot's perspective
        return True, metrics
    
    def _distributed_negotiation_phase(
        self,
        proposals: Dict[int, List[Dict]],
        positions: np.ndarray,
        current_edges: Set[Edge],
        neighbors_per_robot: Dict[int, Set[int]],
        debug: bool = False
    ) -> List[Dict]:
        """Phase 2: Bilateral negotiation - both endpoints must agree to prune an edge.
        
        DISTRIBUTED PROTOCOL:
        - An edge (i,j) can only be pruned if BOTH robots i and j proposed it
        - This ensures unanimous consent without central coordinator
        - Like a distributed consensus protocol
        
        Args:
            proposals: Proposals from each robot
            positions: Robot positions
            current_edges: Current edges
            neighbors_per_robot: Neighbor sets
            debug: Enable debug output
            
        Returns:
            List of approved edge proposals with combined metrics
        """
        approved_edges = []
        
        # Build map of edge -> proposing robots
        edge_proposers = {}
        for robot_id, robot_proposals in proposals.items():
            for proposal in robot_proposals:
                edge = proposal['edge']
                edge_normalized = (min(edge[0], edge[1]), max(edge[0], edge[1]))
                
                if edge_normalized not in edge_proposers:
                    edge_proposers[edge_normalized] = {
                        'proposers': set(),
                        'metrics': {}
                    }
                
                edge_proposers[edge_normalized]['proposers'].add(robot_id)
                edge_proposers[edge_normalized]['metrics'][f'robot_{robot_id}'] = proposal['metrics']
        
        # Check bilateral agreement
        for edge, data in edge_proposers.items():
            i, j = edge
            proposers = data['proposers']
            
            # DISTRIBUTED NEGOTIATION: Both endpoints must agree
            if i in proposers and j in proposers:
                # Calculate combined metrics (conservative: use minimum lambda2)
                metrics_i = data['metrics'].get(f'robot_{i}', {})
                metrics_j = data['metrics'].get(f'robot_{j}', {})
                
                combined_metrics = {
                    'edge': edge,
                    'agreed_by': [i, j],
                    'lambda2_robot_i': metrics_i.get('lambda2_local', 0),
                    'lambda2_robot_j': metrics_j.get('lambda2_local', 0),
                    'lambda2_consensus': min(
                        metrics_i.get('lambda2_local', 0),
                        metrics_j.get('lambda2_local', 0)
                    ),
                    'edge_length': np.linalg.norm(positions[i] - positions[j])
                }
                
                approved_edges.append(combined_metrics)
                
                if debug:
                    print(f"    Edge {edge}: APPROVED by both robots {i} and {j}")
            elif debug and proposers:
                print(f"    Edge {edge}: REJECTED (only proposed by {proposers}, need both)")
        
        return approved_edges
    
    def _distributed_conflict_resolution(
        self,
        approved_edges: List[Dict],
        positions: np.ndarray,
        debug: bool = False
    ) -> Tuple[Optional[Edge], Dict]:
        """Phase 3: Distributed conflict resolution when multiple edges are approved.
        
        DISTRIBUTED TIE-BREAKING RULES (deterministic, no central coordinator):
        1. Prune LONGEST edge (weakest link, deterministic based on geometry)
        2. If tie: Prune edge with LOWEST robot ID sum (deterministic lexicographic order)
        3. If still tie: Prune edge with LOWER first vertex ID
        
        All robots can independently compute the same result using the same rule.
        
        Args:
            approved_edges: List of approved edge proposals
            positions: Robot positions
            debug: Enable debug output
            
        Returns:
            (selected_edge, metrics) or (None, {})
        """
        if not approved_edges:
            return None, {}
        
        if len(approved_edges) == 1:
            edge_data = approved_edges[0]
            return edge_data['edge'], edge_data
        
        # CONFLICT: Multiple edges approved - apply deterministic tie-breaking
        if debug:
            print(f"    CONFLICT: {len(approved_edges)} edges approved, applying tie-breaking rules")
        
        # Sort by: (1) Length DESC, (2) Robot ID sum ASC, (3) First vertex ASC
        def tie_breaking_key(edge_data):
            edge = edge_data['edge']
            i, j = edge
            return (
                -edge_data['edge_length'],      # Longest edge first (negative for DESC)
                i + j,                           # Lowest ID sum (for determinism)
                min(i, j)                        # Lowest ID (further tie-break)
            )
        
        approved_edges.sort(key=tie_breaking_key)
        selected = approved_edges[0]
        
        if debug:
            print(f"    TIE-BREAKING: Selected edge {selected['edge']} (length={selected['edge_length']:.3f})")
            if len(approved_edges) > 1:
                print(f"    Rejected alternatives:")
                for alt in approved_edges[1:]:
                    print(f"      Edge {alt['edge']} (length={alt['edge_length']:.3f})")
        
        return selected['edge'], selected
    
    def _check_lyapunov_constraint(
        self,
        robot_id: int,
        edge: Edge,
        positions: np.ndarray,
        neighbors: Set[int]
    ) -> Tuple[bool, Dict]:
        """Check if Lyapunov constraint allows pruning."""
        A_estimates = self.adjacency_consensus.A_estimates
        V_current = self.lyapunov_monitor.compute_local_lyapunov(
            robot_id, A_estimates, neighbors
        )
        
        epsilon = self.threshold_scheduler.get_lyapunov_threshold(robot_id, V_current)
        
        can_prune, metrics = self.lyapunov_monitor.check_pruning_condition(
            robot_id=robot_id,
            edge_to_remove=edge,
            A_estimates=A_estimates,
            current_neighbors=neighbors,
            positions=positions,
            epsilon_threshold=epsilon,
            sigma=self.sigma,
            T_d=self.T_d,
            num_steps=self.lyapunov_sim_steps
        )
        
        return can_prune, metrics
    
    def _check_disagreement_constraint(
        self,
        robot_id: int,
        edge: Edge,
        positions: np.ndarray,
        neighbors: Set[int]
    ) -> Tuple[bool, Dict]:
        """Check if max disagreement constraint allows pruning."""
        A_estimates = self.adjacency_consensus.A_estimates
        D_current, _ = self.disagreement_monitor.compute_max_disagreement(
            robot_id, A_estimates, neighbors
        )
        
        thresholds = self.threshold_scheduler.get_disagreement_thresholds(robot_id, D_current)
        
        can_prune, metrics = self.disagreement_monitor.check_pruning_condition(
            robot_id=robot_id,
            edge_to_remove=edge,
            A_estimates=A_estimates,
            current_neighbors=neighbors,
            positions=positions,
            delta_threshold=thresholds['delta'],
            absolute_threshold=thresholds['absolute'],
            sigma=self.sigma,
            T_d=self.T_d
        )
        
        return can_prune, metrics
    
    def _build_neighbor_sets(self, edges: Set[Edge]) -> Dict[int, Set[int]]:
        """Build neighbor sets from edge set."""
        neighbors = {i: set() for i in range(self.num_robots)}
        for i, j in edges:
            neighbors[i].add(j)
            neighbors[j].add(i)
        return neighbors
    
    def _compute_pairwise_distances(self, positions: np.ndarray) -> np.ndarray:
        """Compute all pairwise distances."""
        n = self.num_robots
        distances = np.zeros((n, n))
        for i in range(n):
            for j in range(i+1, n):
                dist = np.linalg.norm(positions[i] - positions[j])
                distances[i, j] = dist
                distances[j, i] = dist
        return distances
    
    def _get_direct_observations(
        self,
        robot_id: int,
        neighbors: Set[int],
        distances: np.ndarray
    ) -> Dict[Tuple[int, int], float]:
        """Get direct observations for a robot."""
        observations = {}
        for neighbor_id in neighbors:
            observations[(robot_id, neighbor_id)] = distances[robot_id, neighbor_id]
            observations[(neighbor_id, robot_id)] = distances[robot_id, neighbor_id]
        return observations
    
    def _build_adjacency_from_edges(self, edges: Set[Edge]) -> np.ndarray:
        """Build binary adjacency matrix from edge set."""
        A = np.zeros((self.num_robots, self.num_robots))
        for i, j in edges:
            A[i, j] = 1.0
            A[j, i] = 1.0
        return A
    
    def get_summary(self) -> Dict:
        """Get summary of concurrent pruning session.
        
        Returns:
            Summary statistics dictionary
        """
        return {
            'total_iterations': self.current_iteration,
            'total_edges_pruned': len(self.pruned_edges),
            'pruned_edges': self.pruned_edges,
            'mode': self.mode,
            'final_phase': self.threshold_scheduler.get_phase_name()
        }
    
    def reset(self):
        """Reset all state for new run."""
        self.adjacency_consensus.reset()
        if self.lyapunov_monitor:
            self.lyapunov_monitor.reset()
        if self.disagreement_monitor:
            self.disagreement_monitor.reset()
        self.threshold_scheduler.reset()
        self.pruned_edges.clear()
        self.current_iteration = 0
        self.pruning_history.clear()


if __name__ == '__main__':
    print("=" * 70)
    print("CONCURRENT PRUNING MANAGER - Quick Demo")
    print("=" * 70)
    print()
    print("This is a module file. For full demos, run:")
    print("  python demo_concurrent_pruning.py")
    print("  python quick_test.py")
    print()
    print("Running quick example...")
    print()
    
    # Quick demo
    num_robots = 5
    positions = np.array([
        [0, 0],
        [1, 0],
        [1, 1],
        [0, 1],
        [0.5, 0.5]
    ])
    
    edges = {(0, 1), (1, 2), (2, 3), (3, 0), (0, 4), (1, 4), (2, 4), (3, 4)}
    
    print(f"Setup: {num_robots} robots, {len(edges)} edges")
    print(f"Minimum needed: {num_robots - 1} edges")
    print()
    
    # Test Lyapunov mode
    manager = ConcurrentPruningManager(
        num_robots=num_robots,
        mode='lyapunov',
        k_max=30
    )
    
    manager.initialize_robot_knowledge(positions, edges.copy())
    
    edges_working = edges.copy()
    print("Running concurrent consensus + pruning (Lyapunov mode)...")
    for i in range(20):
        report = manager.concurrent_step(positions, edges_working, debug=False)
        if report['pruned_edge']:
            print(f"  [Iter {i}] Pruned {report['pruned_edge']} -> {len(edges_working)} edges remaining")
    
    summary = manager.get_summary()
    print()
    print(f"Summary:")
    print(f"  Iterations: {summary['total_iterations']}")
    print(f"  Edges pruned: {summary['total_edges_pruned']}")
    print(f"  Final topology: {len(edges_working)} edges")
    print()
    print("✓ Demo complete!")
