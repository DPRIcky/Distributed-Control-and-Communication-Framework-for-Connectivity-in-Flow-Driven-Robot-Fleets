"""Main hybrid underwater simulation engine."""

import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional

import numpy as np

from config import SimulationConfig, ControlConfig, ConsensusConfig
from core import FlowField, ChainRobot
from core.types import Edge
from controllers import HybridCLFCBFController
from graph import TopologyManager, ConnectivityTree
from consensus import HybridPruningManager
from utils import MetricsCollector

# Import DynamicGHS from Minimum_Spanning_Tree_new_mwthond (optional)
try:
    mst_path = Path(__file__).parent.parent / "Minimum_Spanning_Tree_new_mwthond"
    sys.path.insert(0, str(mst_path))
    from demo_reverse_delete import DynamicGHS, EdgeState
    DYNAMIC_GHS_AVAILABLE = True
except ImportError:
    DYNAMIC_GHS_AVAILABLE = False
    DynamicGHS = None
    EdgeState = None


class HybridUnderwaterSimulation:
    """Hybrid consensus pruning with multi-layer robustness against topology changes."""

    def __init__(
        self,
        sim_config: Optional[SimulationConfig] = None,
        control_config: Optional[ControlConfig] = None,
        consensus_config: Optional[ConsensusConfig] = None
    ) -> None:
        """Initialize simulation with modular configuration.
        
        Args:
            sim_config: Simulation configuration
            control_config: Control configuration  
            consensus_config: Consensus configuration
        """
        # Use default configs if not provided
        self.sim_config = sim_config or SimulationConfig()
        self.control_config = control_config or ControlConfig()
        self.consensus_config = consensus_config or ConsensusConfig()

        # Extract frequently used values
        self.num_robots = self.sim_config.num_robots
        self.communication_radius = self.sim_config.communication_radius
        self.dt = self.sim_config.dt
        self.workspace = np.array(self.sim_config.workspace_size, dtype=float)
        self.verbose = self.sim_config.verbose
        
        self.time = 0.0

        # Initialize RNG
        base_rng = np.random.default_rng(self.sim_config.seed)

        # Set goal position
        if self.sim_config.goal_position is not None:
            self.goal_position = self.sim_config.goal_position
        else:
            goal_x = base_rng.uniform(
                self.sim_config.workspace_size[0] * 0.5,
                self.sim_config.workspace_size[0] * 0.95
            )
            goal_y = base_rng.uniform(
                self.sim_config.workspace_size[1] * 0.2,
                self.sim_config.workspace_size[1] * 0.8
            )
            self.goal_position = np.array([goal_x, goal_y])

        # Initialize robots
        cluster_center = np.array([
            self.sim_config.workspace_size[0] * 0.25,
            self.sim_config.workspace_size[1] * 0.5
        ])
        self.robots: List[ChainRobot] = []
        for robot_id in range(self.num_robots):
            rng = np.random.default_rng(base_rng.integers(0, 2**32 - 1))
            start = cluster_center + base_rng.normal(0.0, 0.4, size=2)
            start = np.clip(start, [0.0, 0.0], self.workspace)
            self.robots.append(ChainRobot(robot_id=robot_id, position=start, rng=rng))

        # Initialize subsystems
        self.flow = FlowField()
        
        self.topology_manager = TopologyManager(
            self.num_robots,
            self.communication_radius
        )
        
        self.tree_builder = ConnectivityTree(
            self.num_robots,
            self.sim_config.workspace_size
        )
        
        self.controller = HybridCLFCBFController(
            self.control_config,
            self.communication_radius,
            self.goal_position
        )
        
        self.pruning_manager = HybridPruningManager(
            self.consensus_config,
            self.num_robots
        )
        
        # Initialize metrics collector
        self.metrics = MetricsCollector(self.num_robots)

        # Initialize topology
        self.topology_manager.update_neighbor_graph(self.robots)
        self.tree_builder.build_tree(self.robots, self.topology_manager.neighbor_graph)
        
        # Initialize Dynamic GHS for MST discovery with progressive pruning
        self.ghs_enabled = True  # Toggle GHS on/off
        self.ghs_recompute_interval = 15.0  # Re-run GHS every 15 seconds
        self.ghs_step_interval = 0.05  # Run GHS step every 0.05s
        self.last_ghs_step_time = 0.0
        
        if self.ghs_enabled and DYNAMIC_GHS_AVAILABLE:
            self.ghs = DynamicGHS(self.num_robots, self.ghs_recompute_interval)
            positions = np.array([r.position for r in self.robots])
            self.ghs.initialize(positions, self.communication_radius)
            print(f"[GHS] Initialized with re-computation every {self.ghs_recompute_interval}s")
        else:
            self.ghs = None

    @property
    def neighbor_graph(self):
        """Compatibility property for neighbor graph."""
        return self.topology_manager.neighbor_graph

    @property
    def pruned_edges(self):
        """Compatibility property for pruned edges."""
        return self.topology_manager.pruned_edges
    
    @property
    def stability_threshold(self):
        """Compatibility property for stability threshold."""
        return self.consensus_config.stability_threshold
    
    @property
    def rolling_window_size(self):
        """Compatibility property for rolling window size."""
        return self.consensus_config.rolling_window_size
    
    @property
    def consensus_rounds_per_step(self):
        """Compatibility property for consensus rounds per step."""
        return self.consensus_config.consensus_rounds_per_step
    
    @property
    def consensus_active(self):
        """Compatibility property for consensus active state."""
        return self.pruning_manager.consensus_active
    
    @property
    def consensus_candidate(self):
        """Compatibility property for consensus candidate."""
        return self.pruning_manager.consensus_candidate
    
    @property
    def consensus_round(self):
        """Compatibility property for consensus round."""
        return self.pruning_manager.consensus_round

    def current_edges(self) -> List[Edge]:
        """Get current edges in the graph."""
        return self.topology_manager.get_current_edges()

    def step(self) -> Dict[str, Optional[object]]:
        """Execute one simulation step with hybrid consensus pruning.
        
        Returns:
            Dictionary containing step results and decision information
        """
        # STEP 0: Update GHS edge weights with current positions (BEFORE robots move)
        if self.ghs_enabled and self.ghs is not None:
            positions = np.array([r.position for r in self.robots])
            self.ghs.update_edges(positions, self.communication_radius)
            
            # Check if time for GHS re-computation
            if self.ghs.check_recomputation(self.time):
                if self.verbose:
                    print(f"\n[GHS] Re-computing MST at t={self.time:.2f}s (run #{self.ghs.current_run})")
                self.ghs.restart_computation()
            
            # Run GHS step at lower frequency
            if self.time - self.last_ghs_step_time >= self.ghs_step_interval:
                if not self.ghs.converged:
                    self.ghs.step()
                self.last_ghs_step_time = self.time
                
                # Print GHS progress periodically
                if self.ghs.round % 20 == 0 and self.verbose:
                    stats = self.ghs.get_statistics()
                    print(f"[GHS Round {stats['round']}] MST: {stats['mst_edges']}, "
                          f"Active: {stats['total_edges']}, Removed: {stats['removed_edges']}, "
                          f"Fragments: {stats['fragments']}")
                    
                # Check convergence
                if self.ghs.converged and self.verbose:
                    stats = self.ghs.get_statistics()
                    print(f"[GHS] ✓ Converged! MST: {stats['mst_edges']} edges, "
                          f"Removed: {stats['removed_edges']} non-MST edges")
        
        # STEP 1: Update connectivity tree
        self.tree_builder.build_tree(self.robots, self.topology_manager.neighbor_graph)

        # STEP 2: Compute and apply CLF-CBF control for each robot (ROBOTS MOVE)
        for robot_id, robot in enumerate(self.robots):
            control_force = self.controller.compute_control(robot_id, self.robots)
            robot.step(self.flow, self.dt, self.time, self.workspace, control_force)
            
            # Record control magnitude
            self.metrics.record_control(robot_id, control_force)

        self.time += self.dt
        
        # Record metrics for this step
        self.metrics.record_step(self, self.time)

        # STEP 3: Detect topology changes
        old_edges = set(self.current_edges())
        self.topology_manager.update_neighbor_graph(self.robots)
        new_edges = set(self.current_edges())

        edges_broken = old_edges - new_edges
        edges_formed = new_edges - old_edges

        topology_changed = len(edges_broken) > 0 or len(edges_formed) > 0

        # STEP 4: Handle topology changes (reset consensus if topology changed)
        if topology_changed:
            self.pruning_manager.reset_on_topology_change()

            if self.verbose:
                print(f"[t={self.time:5.2f}] Topology changed! "
                      f"Broken: {edges_broken}, Formed: {edges_formed}")
                if self.pruning_manager.consensus_active:
                    print(f"  Aborting consensus (was at round {self.pruning_manager.consensus_round})")
                    
            return {
                "decision": "topology_unstable",
                "edges_broken": sorted(edges_broken),
                "edges_formed": sorted(edges_formed),
                "stable_rounds": 0
            }
        else:
            self.pruning_manager.increment_stability()

        # STEP 5: Wait for topology to stabilize
        if not self.pruning_manager.is_stable():
            return {
                "decision": "waiting_for_stability",
                "stable_rounds": self.pruning_manager.topology_stable_rounds,
                "threshold": self.consensus_config.stability_threshold
            }

        # STEP 6: Add to rolling window
        self.pruning_manager.add_topology_snapshot(
            new_edges,
            self.topology_manager.gather_edge_lengths(self.robots),
            self.time
        )

        # STEP 7: If not enough history, wait
        if not self.pruning_manager.has_sufficient_history():
            return {
                "decision": "building_history",
                "history_size": len(self.pruning_manager.topology_history),
                "needed": self.consensus_config.rolling_window_size
            }

        # STEP 7.5: Run adjacency matrix consensus TO COMPLETION (NOVEL distributed method)
        # This allows each robot to estimate the graph topology locally
        current_edges_set = set(self.current_edges())
        if current_edges_set and not self.pruning_manager.consensus_converged:
            # Build position array and edge lengths for consensus
            positions = np.array([r.position for r in self.robots])
            edge_lengths = self.topology_manager.gather_edge_lengths(self.robots)
            
            if self.verbose:
                print(f"[NOVEL] Running adjacency matrix consensus to completion...")
            
            # Run consensus phase to completion (multiple iterations)
            for iteration in range(50):
                converged = self.pruning_manager.run_consensus_phase(
                    positions=positions,
                    edge_lengths=edge_lengths,
                    current_edges=current_edges_set
                )
                
                if converged:
                    if self.verbose:
                        print(f"[NOVEL] Consensus CONVERGED in {iteration+1} iterations")
                    # Store the iteration count for tracking
                    self.pruning_manager.last_convergence_iterations = iteration + 1
                    break
            else:
                # Force convergence after max iterations
                self.pruning_manager.consensus_converged = True
                self.pruning_manager.last_convergence_iterations = 50
                if self.verbose:
                    print(f"[NOVEL] Forced convergence after 50 iterations")

        # STEP 8: Find redundant edge using NOVEL distributed method
        if not self.pruning_manager.consensus_active:
            # NOVEL: Each robot independently decides using local consensus estimate
            # Check if all robots agree (truly distributed decision)
            if self.verbose:
                print(f"[NOVEL] Querying all {self.num_robots} robots for redundant edges...")
            
            robot_decisions = []
            for robot_id in range(self.num_robots):
                decision = self.pruning_manager.find_redundant_edge_distributed(
                    robot_id=robot_id,
                    debug=(self.verbose and robot_id == 0)  # Only show debug for robot 0
                )
                robot_decisions.append(decision)
            
            unique_decisions = set(robot_decisions)
            
            # Check if all robots agree on the same edge
            if len(unique_decisions) == 1 and robot_decisions[0] is not None:
                candidate = robot_decisions[0]
                if self.verbose:
                    print(f"[NOVEL] ✓ ALL {self.num_robots} ROBOTS AGREE: Prune edge {candidate}")
                
                # DIRECTLY PRUNE - all robots agree, no need for additional consensus
                self.topology_manager.pruned_edges.add(candidate)
                
                # CRITICAL: Update consensus estimates to reflect the pruned edge
                # Set A[i,j] = 0 for the pruned edge in ALL robot estimates
                i, j = candidate
                for robot_id in range(self.num_robots):
                    A = self.pruning_manager.adjacency_consensus.A_estimates[robot_id]
                    A[i, j] = 0.0
                    A[j, i] = 0.0
                
                # Record pruning event
                self.metrics.record_pruning_event(self.time, candidate, "novel_consensus")
                
                # Reset consensus for next pruning decision
                self.pruning_manager.consensus_converged = False
                self.pruning_manager.consensus_iterations = 0
                
                if self.verbose:
                    print(f"[t={self.time:5.2f}] PRUNED {candidate} (NOVEL) | "
                          f"edges left={len(self.current_edges())}")
                
                return {
                    "decision": "prune",
                    "removed_edge": candidate,
                    "method": "NOVEL_CONSENSUS",
                    "edges_remaining": len(self.current_edges())
                }
            else:
                # Robots disagree or no edge found
                if self.verbose:
                    if None in unique_decisions and len(unique_decisions) == 1:
                        print(f"[NOVEL] No redundant edges found by any robot")
                    else:
                        print(f"[NOVEL] Robots disagree on pruning: {unique_decisions}")
                        print(f"[NOVEL] Resetting consensus for better convergence")
                
                # Reset consensus to try again
                self.pruning_manager.consensus_converged = False
                self.pruning_manager.consensus_iterations = 0
                
                return {
                    "decision": "no_unanimous_agreement",
                    "unique_decisions": len(unique_decisions)
                }
        
        # No consensus active and no agreement - continue simulation
        return {"decision": "no_action"}

    def iterate(self, steps: int) -> Iterable[Dict[str, Optional[object]]]:
        """Run multiple simulation steps.
        
        Args:
            steps: Number of steps to run
            
        Yields:
            Step results for each iteration
        """
        for _ in range(steps):
            yield self.step()
