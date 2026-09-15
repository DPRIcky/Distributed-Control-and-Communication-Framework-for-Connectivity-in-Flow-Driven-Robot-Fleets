"""Modified simulation that uses baseline pruning methods."""

from typing import Optional
import numpy as np

from simulation import HybridUnderwaterSimulation

# Import from local modules (handles spaces in folder name)
from .full_graph import FullGraphBaseline
from .centralized_mst import CentralizedMSTBaseline
from .random_pruning import RandomPruningBaseline
from .greedy_distance import GreedyDistanceBaseline


class BaselineSimulation(HybridUnderwaterSimulation):
    """Simulation that replaces consensus pruning with a baseline method."""
    
    def __init__(
        self,
        sim_config,
        control_config,
        consensus_config,
        baseline_method: str = 'hybrid'
    ):
        """Initialize simulation with baseline pruning method.
        
        Args:
            sim_config: Simulation configuration
            control_config: Control configuration
            consensus_config: Consensus configuration
            baseline_method: One of 'hybrid', 'full_graph', 'centralized_mst',
                           'random', 'greedy_distance'
        """
        super().__init__(sim_config, control_config, consensus_config)
        
        self.baseline_method = baseline_method
        
        # Initialize baseline pruner if not using hybrid
        if baseline_method == 'full_graph':
            self.baseline_pruner = FullGraphBaseline(self.num_robots)
        elif baseline_method == 'centralized_mst':
            self.baseline_pruner = CentralizedMSTBaseline(self.num_robots)
        elif baseline_method == 'random':
            self.baseline_pruner = RandomPruningBaseline(
                self.num_robots,
                seed=sim_config.seed
            )
        elif baseline_method == 'greedy_distance':
            self.baseline_pruner = GreedyDistanceBaseline(self.num_robots)
        elif baseline_method == 'hybrid':
            self.baseline_pruner = None  # Use default consensus pruning
        else:
            raise ValueError(f"Unknown baseline method: {baseline_method}")
    
    def step(self):
        """Override step to use baseline pruning instead of consensus."""
        if self.baseline_method == 'hybrid':
            # Use original hybrid consensus pruning
            return super().step()
        
        # Execute motion and control (same as parent)
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

        # STEP 3: Update topology
        self.topology_manager.update_neighbor_graph(self.robots)
        
        # STEP 4: Baseline pruning (instead of consensus)
        edge_lengths = self.topology_manager.gather_edge_lengths(self.robots)
        
        if edge_lengths:
            edge_to_prune = self.baseline_pruner.find_edge_to_prune(
                edge_lengths=edge_lengths,
                robots=self.robots
            )
            
            if edge_to_prune is not None:
                # Prune the edge
                self.topology_manager.pruned_edges.add(edge_to_prune)
                if self.verbose:
                    print(f"[{self.baseline_method.upper()}] Pruned edge {edge_to_prune}")
                
                # Record pruning event
                self.metrics.record_pruning_event(
                    self.time,
                    edge_to_prune,
                    self.baseline_method
                )
        
        return {"decision": "baseline_step_complete"}
    
    def get_method_name(self) -> str:
        """Get the pruning method name.
        
        Returns:
            Method name string
        """
        if self.baseline_method == 'hybrid':
            return "HybridConsensus"
        return self.baseline_pruner.get_name()
