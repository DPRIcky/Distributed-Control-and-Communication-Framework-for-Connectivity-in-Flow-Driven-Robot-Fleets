"""Simple Critical Edge Detection Simulation using baseline_simulation infrastructure.

This file provides a simple wrapper that applies the distributed critical edge
detection algorithm on top of the existing baseline simulation infrastructure.
"""

import sys
import os
from pathlib import Path

# Add parent directory to path - same as GUI simulation pattern
parent_dir = Path(__file__).parent.parent
sys.path.insert(0, str(parent_dir))

from baseline_simulation import FullyConnectedSimulation
from Critical_edge_detection_baseline.critical_edge_baseline import CriticalEdgeBaseline


class CriticalEdgeSimulation(FullyConnectedSimulation):
    """Simulation with distributed critical edge detection for pruning.
    
    This extends FullyConnectedSimulation and adds distributed MST-based pruning
    using the critical edge detection algorithm.
    """
    
    def __init__(
        self,
        sim_config,
        control_config,
        update_frequency: int = 10,
        prefer_critical: bool = True,
        verbose: bool = False
    ):
        """Initialize critical edge simulation.
        
        Args:
            sim_config: Simulation configuration
            control_config: Control configuration
            update_frequency: Update MST every N steps
            prefer_critical: Prioritize critical edges in MST
            verbose: Print detailed execution trace
        """
        # Initialize parent (fully connected simulation)
        super().__init__(sim_config, control_config)
        
        # Initialize critical edge baseline
        self.baseline = CriticalEdgeBaseline(
            num_robots=self.num_robots,
            update_frequency=update_frequency,
            prefer_critical=prefer_critical,
            verbose=verbose or sim_config.verbose
        )
        
        # Track pruned edges
        self.pruned_edges = set()
        self.edges_pruned_count = 0
    
    def step(self):
        """Execute one simulation step with critical edge pruning."""
        # Step 1: Update neighbor graph (rebuilds from scratch based on comm radius and permanently pruned edges)
        self.topology_manager.update_neighbor_graph(self.robots)
        
        # Step 2: Build connectivity tree
        self.tree_builder.build_tree(self.robots, self.topology_manager.neighbor_graph)
        
        # Step 3: Update MST BEFORE control to ensure critical edges are available
        # This ensures CBF has access to critical edges during control computation
        edge_lengths = self._get_edge_lengths()
        if edge_lengths and len(edge_lengths) > self.num_robots - 1:
            # Only trigger MST update if we have more than minimum edges
            # The find_edge_to_prune method handles MST update logic
            # We call it here to update critical edges but ignore pruning decision for now
            self.baseline.find_edge_to_prune(edge_lengths)
        elif edge_lengths and len(self.baseline.current_mst) == 0:
            # Initialize MST if not yet set
            self.baseline.find_edge_to_prune(edge_lengths)
        
        # Step 4: Convert critical edges to neighbor dict format for controller
        # IMPORTANT: Only include critical edges that are actually in the current topology
        # (i.e., not pruned and within communication range)
        critical_edges_dict = None
        if self.baseline.critical_edges and edge_lengths:
            critical_edges_dict = {i: [] for i in range(self.num_robots)}
            for i, j in self.baseline.critical_edges:
                # Check if edge is actually in current topology
                edge_key_1 = (min(i, j), max(i, j))
                edge_key_2 = (max(i, j), min(i, j))
                
                # Only add to dict if edge exists in current topology
                if edge_key_1 in edge_lengths or edge_key_2 in edge_lengths:
                    critical_edges_dict[i].append(j)
                    critical_edges_dict[j].append(i)
        
        # Step 5: Compute control and move robots
        for robot_id, robot in enumerate(self.robots):
            control_force = self.controller.compute_control(
                robot_id, 
                self.robots,
                critical_edges=critical_edges_dict
            )
            robot.step(self.flow, self.dt, self.time, self.workspace, control_force)
        
        # Update time
        self.time += self.dt
        
        # Step 6: Apply critical edge detection and pruning
        # Re-compute edge lengths after robot movement
        edge_lengths = self._get_edge_lengths()
        
        if edge_lengths:
            edge_to_prune = self.baseline.find_edge_to_prune(edge_lengths)
            
            if edge_to_prune is not None:
                # Properly prune the edge using TopologyManager (not by direct modification)
                self.topology_manager.prune_edge(edge_to_prune, self.robots)
                self.pruned_edges.add(edge_to_prune)
                self.edges_pruned_count += 1
                
                if self.verbose:
                    print(f"[t={self.time:.2f}] Pruned edge {edge_to_prune}")
        
        # Update statistics
        num_edges = sum(len(neighbors) for neighbors in self.topology_manager.neighbor_graph) // 2
        self.total_edges = num_edges
        self.max_edges = max(self.max_edges, num_edges)
    
    def _get_edge_lengths(self):
        """Get current edge lengths as dictionary.
        
        Returns:
            Dictionary mapping (i, j) tuples to edge lengths
        """
        edge_lengths = {}
        
        for i in range(self.num_robots):
            for j in self.topology_manager.neighbor_graph[i]:
                if i < j:  # Avoid duplicates
                    # Calculate distance
                    dist = np.linalg.norm(
                        self.robots[i].position - self.robots[j].position
                    )
                    edge_lengths[(i, j)] = dist
        
        return edge_lengths
    
    def get_method_name(self):
        """Get method name for reporting."""
        return self.baseline.get_name()
    
    def print_statistics(self):
        """Print simulation statistics."""
        stats = self.baseline.get_statistics()
        
        print("\n" + "=" * 70)
        print("CRITICAL EDGE DETECTION SIMULATION STATISTICS")
        print("=" * 70)
        print(f"Simulation time: {self.time:.2f}s")
        print(f"Current edges: {self.total_edges}")
        print(f"Max edges seen: {self.max_edges}")
        print(f"Total edges pruned: {self.edges_pruned_count}")
        print(f"\nMST Algorithm Statistics:")
        print(f"  MST updates: {stats['mst_updates']}")
        print(f"  Total DFS rounds: {stats['total_rounds']}")
        print(f"  Avg rounds/update: {stats['avg_rounds_per_update']:.2f}")
        print(f"  Current MST size: {stats['current_mst_size']}")
        print(f"  Critical edges detected: {stats['critical_edges_detected']}")
        print("=" * 70)


# Import numpy for distance calculation
import numpy as np


if __name__ == "__main__":
    from config import SimulationConfig, ControlConfig
    
    print("=" * 70)
    print("CRITICAL EDGE DETECTION BASELINE - SIMPLE DEMO")
    print("=" * 70)
    
    # Create configurations
    sim_config = SimulationConfig(
        num_robots=8,
        communication_radius=3.0,
        dt=0.05,
        workspace_size=(10.0, 10.0),
        verbose=True,
        seed=42
    )
    
    control_config = ControlConfig()
    
    # Create simulation
    sim = CriticalEdgeSimulation(
        sim_config=sim_config,
        control_config=control_config,
        update_frequency=10,
        prefer_critical=True,
        verbose=True
    )
    
    print("\nRunning simulation for 100 steps...")
    
    # Run simulation
    for step in range(100):
        sim.step()
        
        if step % 20 == 0:
            print(f"Step {step:3d} | t={sim.time:6.2f}s | Edges={sim.total_edges:3d}")
    
    # Print final statistics
    sim.print_statistics()
    
    print("\n" + "=" * 70)
    print("SIMULATION COMPLETE")
    print("=" * 70)
