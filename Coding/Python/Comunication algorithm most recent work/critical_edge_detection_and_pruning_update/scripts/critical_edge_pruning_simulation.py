"""Critical Edge Detection and Pruning Simulation.

Extends FullyConnectedSimulation with distributed edge connectivity analysis
and intelligent pruning based on critical edge detection.

This integrates the distributed pruning algorithm with the baseline simulation 
framework, running the 4-phase algorithm on the robot network topology.
"""

import sys
import os
from pathlib import Path

# Add parent directories to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import numpy as np
import networkx as nx
from typing import List, Set, Tuple

from config import SimulationConfig, ControlConfig, VisualizationConfig
from baseline_simulation import FullyConnectedSimulation, FullyConnectedAnimator
from graph import TopologyManager
from distributed_pruning_algorithm import DistributedPruningAlgorithm


class CriticalEdgePruningSimulation(FullyConnectedSimulation):
    """Simulation with distributed critical edge detection and intelligent pruning.
    
    This class extends FullyConnectedSimulation to integrate the 4-phase distributed
    pruning algorithm:
    1. Phase 1: Distance Computation via distributed BFS
    2. Phase 2: Connectivity Assurance
    3. Phase 3: Spanning Tree Construction
    4. Phase 4: Robustness Enhancement
    
    Attributes:
        critical_edges: Set of edges identified as critical
        pruned_edges: Edges that have been removed
        distributed_algo: Current DistributedPruningAlgorithm instance
        algorithm_interval: How often to run the algorithm (seconds)
        last_algorithm_run: Timestamp of last algorithm execution
    """
    
    def __init__(self, sim_config: SimulationConfig, control_config: ControlConfig, 
                 enable_pruning: bool = True, pruning_start_time: float = 2.0,
                 algorithm_interval: float = 0.5, root_node: int = 0):
        """Initialize critical edge pruning simulation with distributed algorithm.
        
        Args:
            sim_config: Simulation configuration
            control_config: Control configuration
            enable_pruning: Whether to enable edge pruning
            pruning_start_time: When to start pruning (simulation time in seconds)
            algorithm_interval: How often to run distributed algorithm (seconds)
            root_node: Root node for spanning tree (default 0)
        """
        super().__init__(sim_config, control_config)
        
        self.enable_pruning = enable_pruning
        self.pruning_start_time = pruning_start_time
        self.algorithm_interval = algorithm_interval
        self.root_node = root_node
        
        # Pruning state
        self.critical_edges: Set[Tuple[int, int]] = set()
        self.pruned_edges: Set[Tuple[int, int]] = set()
        
        # Distributed algorithm state
        self.distributed_algo = None
        self.last_algorithm_run = -algorithm_interval
        self.algorithm_phase = 0
        self.algorithm_converged = False
        
        if self.verbose:
            print(f"\n[Distributed Critical Edge Pruning Simulation Initialized]")
            print(f"  Enable Pruning: {enable_pruning}")
            print(f"  Pruning Start Time: {pruning_start_time}s")
            print(f"  Algorithm Interval: {algorithm_interval}s")
            print(f"  Root Node: {root_node}")
    
    def _create_networkx_graph(self) -> nx.Graph:
        """Convert current robot topology to NetworkX graph.
        
        Maps robot indices to node IDs (0-based to 1-based for compatibility
        with DistributedPruningAlgorithm).
        
        Returns:
            nx.Graph: Network representation of current topology
        """
        G = nx.Graph()
        
        # Add nodes (use 1-based indexing for algorithm)
        for i in range(self.num_robots):
            G.add_node(i + 1)
        
        # Add edges from current topology
        edges = self.current_edges()
        for i, j in edges:
            # Convert to 1-based indexing
            G.add_edge(i + 1, j + 1)
        
        return G
    
    def _apply_pruning_from_algorithm(self):
        """Extract critical edges from algorithm and apply pruning to topology.
        
        Takes the pruned graph from the distributed algorithm and removes
        non-critical edges from the robot topology.
        """
        if self.distributed_algo is None:
            return
        
        try:
            # Get critical edges from algorithm (1-based)
            pruned_graph = self.distributed_algo.get_pruned_graph()
            algorithm_edges = set()
            for i, j in pruned_graph.edges():
                # Convert back to 0-based indexing
                algorithm_edges.add((min(i-1, j-1), max(i-1, j-1)))
            
            # Find edges to prune (current edges not in algorithm result)
            current_edges = set()
            for i, j in self.current_edges():
                current_edges.add((min(i, j), max(i, j)))
            
            edges_to_remove = current_edges - algorithm_edges
            
            # Remove non-critical edges from topology
            for i, j in edges_to_remove:
                if (i, j) not in self.pruned_edges and (j, i) not in self.pruned_edges:
                    self.topology_manager.neighbor_graph[i].discard(j)
                    self.topology_manager.neighbor_graph[j].discard(i)
                    self.pruned_edges.add((i, j) if i < j else (j, i))
                    
                    if self.verbose:
                        print(f"  [t={self.time:.2f}s] Pruned edge ({i}, {j}) via distributed algorithm")
            
            # Extract and store critical edges
            self.critical_edges = algorithm_edges
            self.algorithm_converged = True
            
        except Exception as e:
            if self.verbose:
                print(f"  Error applying pruning: {e}")
            import traceback
            if self.verbose:
                traceback.print_exc()
    
    def run_distributed_algorithm(self):
        """Execute the 4-phase distributed pruning algorithm on current topology.
        
        Converts robot topology to NetworkX graph, runs the distributed pruning
        algorithm, and applies pruning decisions back to the robot network.
        """
        if not self.enable_pruning or self.time < self.pruning_start_time:
            return
        
        # Check if enough time has passed since last run
        if self.time - self.last_algorithm_run < self.algorithm_interval:
            return
        
        self.last_algorithm_run = self.time
        
        # Convert current topology to NetworkX graph
        G = self._create_networkx_graph()
        
        # Check connectivity
        if not nx.is_connected(G):
            if self.verbose:
                print(f"  [t={self.time:.2f}s] Network disconnected - skipping algorithm")
            return
        
        # Check if we have enough edges for pruning
        if len(G.edges()) <= len(G.nodes()) - 1:
            if self.verbose:
                print(f"  [t={self.time:.2f}s] Network is already a tree - no pruning needed")
            return
        
        try:
            if self.verbose:
                print(f"\n[t={self.time:.2f}s] Running Distributed Pruning Algorithm")
                print(f"  Topology: {len(G.nodes())} nodes, {len(G.edges())} edges")
                print(f"  Root node: {self.root_node + 1}")
            
            # Create and run algorithm
            self.distributed_algo = DistributedPruningAlgorithm(G.copy(), root=self.root_node + 1)
            self.distributed_algo.run_full_pipeline()
            
            # Get statistics
            stats = self.distributed_algo.get_statistics()
            pruning_pct = (1 - stats['final_edges'] / stats['original_edges']) * 100 if stats['original_edges'] > 0 else 0
            
            if self.verbose:
                print(f"  Result: {stats['original_edges']} → {stats['final_edges']} edges")
                print(f"  Pruning: {pruning_pct:.1f}%")
                print(f"  Connected: {'✓ Yes' if stats['connected'] else '✗ No'}")
            
            # Apply pruning to robot topology
            self._apply_pruning_from_algorithm()
            
        except Exception as e:
            if self.verbose:
                print(f"  Error running algorithm: {e}")
    
    def detect_critical_edges(self) -> Set[Tuple[int, int]]:
        """Detect critical edges (bridges) in current topology.

        An edge is critical if removing it disconnects the graph.

        Returns:
            Set of (i, j) tuples representing critical edges (i < j)
        """
        critical = set()
        edges = self.current_edges()
        
        if len(edges) <= len(self.robots) - 1:
            # Tree (no redundancy) - all edges are critical
            return set(edges)
        
        # Check each edge - remove it and test connectivity
        for edge in edges:
            i, j = edge
            
            # Temporarily remove edge from topology
            original_neighbors_i = self.topology_manager.neighbor_graph[i].copy()
            original_neighbors_j = self.topology_manager.neighbor_graph[j].copy()
            
            self.topology_manager.neighbor_graph[i].discard(j)
            self.topology_manager.neighbor_graph[j].discard(i)
            
            # Check if graph is still connected
            if not self._is_connected():
                critical.add(edge)
            
            # Restore edge
            self.topology_manager.neighbor_graph[i] = original_neighbors_i
            self.topology_manager.neighbor_graph[j] = original_neighbors_j
        
        return critical
    
    def _is_connected(self) -> bool:
        """Check if current topology is connected using BFS.
        
        Returns:
            bool: True if graph is connected
        """
        if self.num_robots == 0:
            return True
        
        visited = set()
        queue = [0]
        visited.add(0)
        
        while queue:
            node = queue.pop(0)
            for neighbor in self.topology_manager.neighbor_graph[node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        
        return len(visited) == self.num_robots
    
    def step(self):
        """Execute one simulation step with distributed edge pruning.
        
        Overrides parent step() to add 4-phase distributed algorithm execution.
        """
        # Run parent step (control + dynamics)
        super().step()
        
        # Run distributed pruning algorithm periodically
        self.run_distributed_algorithm()
    
    def get_state(self):
        """Get current simulation state for visualization.
        
        Returns:
            dict: State with additional pruning information
        """
        state = super().get_state()
        
        # Add pruning-specific information
        state['critical_edges'] = list(self.critical_edges)
        state['pruned_edges'] = list(self.pruned_edges)
        state['total_pruned'] = len(self.pruned_edges)
        
        return state


if __name__ == "__main__":
    """Test critical edge pruning simulation."""
    import matplotlib.pyplot as plt
    
    # Create configurations
    sim_config = SimulationConfig(
        num_robots=10,
        communication_radius=3.5,
        dt=0.05,
        workspace_size=(12.0, 12.0),
        seed=42,
        verbose=True
    )
    
    control_config = ControlConfig(
        safety_distance=1.2,
        clf_gain=0.8,
        cbf_safety_gain=4.0,
        cbf_connectivity_gain=0.5,
        max_control_force=1.0
    )
    
    vis_config = VisualizationConfig()
    
    # Create and run simulation
    print("\n" + "="*60)
    print("CRITICAL EDGE PRUNING SIMULATION - TEST RUN")
    print("="*60)
    
    sim = CriticalEdgePruningSimulation(
        sim_config, 
        control_config,
        enable_pruning=True,
        pruning_start_time=1.0  # Start pruning after 1 second
    )
    
    animator = FullyConnectedAnimator(sim, vis_config)
    
    # Run animation
    print("\nRunning simulation with edge pruning... (close window to exit)")
    anim = animator.animate()
    plt.show()
    
    # Print final statistics
    print("\n" + "="*60)
    print("SIMULATION COMPLETE")
    print("="*60)
    print(f"Total simulation time: {sim.time:.2f}s")
    print(f"Final edges: {sim.total_edges}")
    print(f"Max edges: {sim.max_edges}")
    print(f"Pruned edges: {len(sim.pruned_edges)}")
    print(f"Critical edges detected: {len(sim.critical_edges)}")
    print("="*60 + "\n")
