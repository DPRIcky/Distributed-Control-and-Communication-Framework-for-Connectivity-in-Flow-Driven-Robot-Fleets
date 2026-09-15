"""Fully Connected Simulation - Core Module.

Standard baseline simulation with:
- Fully connected graph (all edges within communication range)
- CBF (Control Barrier Function) for collision avoidance + connectivity
- CLF (Control Lyapunov Function) for goal-seeking
- Flow field disturbance
- No edge pruning (baseline for comparison)

This is the foundation class that can be extended for custom algorithms.
"""

import sys
import os
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from typing import List

from config import SimulationConfig, ControlConfig
from core import FlowField, ChainRobot
from controllers import HybridCLFCBFController
from graph import TopologyManager, ConnectivityTree


class FullyConnectedSimulation:
    """Standard baseline simulation with fully connected graph - no pruning.
    
    This class provides a clean, reusable simulation environment that matches
    the style and configuration of concurrent_pruning simulations.
    
    Attributes:
        sim_config: Simulation configuration
        control_config: Control configuration
        num_robots: Number of robots
        communication_radius: Communication range
        dt: Time step
        workspace: Workspace size [width, height]
        time: Current simulation time
        goal_position: Goal position [x, y]
        robots: List of robot instances
        flow: Flow field instance
        topology_manager: Manages neighbor graph
        tree_builder: Builds connectivity tree
        controller: Hybrid CLF/CBF controller
        total_edges: Current edge count
        max_edges: Maximum edges seen
    """
    
    def __init__(self, sim_config: SimulationConfig, control_config: ControlConfig):
        """Initialize fully connected simulation.
        
        Args:
            sim_config: Simulation configuration
            control_config: Control configuration
        """
        # Store configs
        self.sim_config = sim_config
        self.control_config = control_config
        
        # Extract frequently used values
        self.num_robots = sim_config.num_robots
        self.communication_radius = sim_config.communication_radius
        self.dt = sim_config.dt
        self.workspace = np.array(sim_config.workspace_size, dtype=float)
        self.time = 0.0
        self.verbose = sim_config.verbose
        
        # Initialize RNG
        self.rng = np.random.default_rng(sim_config.seed)
        
        # Set goal position
        if sim_config.goal_position is not None:
            self.goal_position = sim_config.goal_position
        else:
            # Random goal in right half of workspace
            goal_x = self.rng.uniform(
                sim_config.workspace_size[0] * 0.6,
                sim_config.workspace_size[0] * 0.9
            )
            goal_y = self.rng.uniform(
                sim_config.workspace_size[1] * 0.3,
                sim_config.workspace_size[1] * 0.7
            )
            self.goal_position = np.array([goal_x, goal_y])
        
        # Initialize robots in a cluster on the left
        cluster_center = np.array([
            sim_config.workspace_size[0] * 0.2,
            sim_config.workspace_size[1] * 0.5
        ])
        self.robots: List[ChainRobot] = []
        for robot_id in range(self.num_robots):
            # Add small random offset from cluster center
            start = cluster_center + self.rng.normal(0.0, 0.5, size=2)
            start = np.clip(start, [0.0, 0.0], self.workspace)
            robot_rng = np.random.default_rng(self.rng.integers(0, 2**32 - 1))
            self.robots.append(ChainRobot(robot_id=robot_id, position=start, rng=robot_rng))
        
        # Initialize flow field (sinusoidal disturbance)
        self.flow = FlowField()
        
        # Initialize topology manager (finds neighbors within communication range)
        self.topology_manager = TopologyManager(
            self.num_robots,
            self.communication_radius
        )
        
        # Initialize connectivity tree builder (for spanning tree-based consensus)
        self.tree_builder = ConnectivityTree(
            self.num_robots,
            sim_config.workspace_size
        )
        
        # Initialize hybrid controller (CBF + CLF)
        self.controller = HybridCLFCBFController(
            control_config,
            self.communication_radius,
            self.goal_position
        )
        
        # Initialize topology
        self.topology_manager.update_neighbor_graph(self.robots)
        self.tree_builder.build_tree(self.robots, self.topology_manager.neighbor_graph)
        
        # Statistics
        self.total_edges = 0
        self.max_edges = 0
        
        if self.verbose:
            print(f"\n[Baseline Simulation Initialized]")
            print(f"  Robots: {self.num_robots}")
            print(f"  Communication Radius: {self.communication_radius:.1f}m")
            print(f"  Goal: ({self.goal_position[0]:.1f}, {self.goal_position[1]:.1f})")
            print(f"  Workspace: {sim_config.workspace_size}")
    
    def step(self):
        """Execute one simulation step.
        
        Override this method in subclasses to add custom behavior (e.g., GHS, pruning).
        """
        # Update neighbor graph based on current positions
        self.topology_manager.update_neighbor_graph(self.robots)
        
        # Build connectivity tree (for information routing if needed)
        self.tree_builder.build_tree(self.robots, self.topology_manager.neighbor_graph)
        
        # Compute control for each robot (CBF + CLF)
        for robot_id, robot in enumerate(self.robots):
            # CBF: Maintains safe distance from neighbors (collision avoidance + connectivity)
            # CLF: Guides robot towards goal
            # Combined: QP solver balances both objectives
            control_force = self.controller.compute_control(robot_id, self.robots)
            
            # Step robot dynamics with flow disturbance
            robot.step(self.flow, self.dt, self.time, self.workspace, control_force)
        
        # Update time
        self.time += self.dt
        
        # Update statistics (count edges, avoiding duplicates)
        num_edges = sum(len(neighbors) for neighbors in self.topology_manager.neighbor_graph) // 2
        self.total_edges = num_edges
        self.max_edges = max(self.max_edges, num_edges)
    
    def current_edges(self):
        """Get current edges as list of (i, j) tuples.
        
        Returns:
            list: List of (i, j) tuples representing edges where i < j
        """
        edges = []
        for robot_id in range(self.num_robots):
            for neighbor_id in self.topology_manager.neighbor_graph[robot_id]:
                if robot_id < neighbor_id:  # Avoid duplicates
                    edges.append((robot_id, neighbor_id))
        return edges
    
    def get_state(self):
        """Get current simulation state for visualization.
        
        Returns:
            dict: State dictionary with:
                - time: Current time
                - positions: Robot positions (n×2 array)
                - velocities: Robot velocities (n×2 array)
                - goal: Goal position
                - edges: List of edge tuples
                - num_edges: Edge count
                - communication_radius: Comm radius
        """
        positions = np.array([robot.position for robot in self.robots])
        velocities = np.array([robot.velocity for robot in self.robots])
        
        return {
            'time': self.time,
            'positions': positions,
            'velocities': velocities,
            'goal': self.goal_position,
            'edges': self.current_edges(),
            'num_edges': self.total_edges,
            'communication_radius': self.communication_radius
        }


if __name__ == "__main__":
    """Test simulation demonstration."""
    from config import SimulationConfig, ControlConfig, VisualizationConfig
    from baseline_simulation import FullyConnectedAnimator
    import matplotlib.pyplot as plt
    
    # Create configurations
    sim_config = SimulationConfig(
        num_robots=8,
        communication_radius=3.0,
        dt=0.05,
        workspace_size=(10.0, 10.0),
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
    print("FULLY CONNECTED BASELINE SIMULATION - TEST RUN")
    print("="*60)
    
    sim = FullyConnectedSimulation(sim_config, control_config)
    animator = FullyConnectedAnimator(sim, vis_config)
    
    # Run animation
    print("\nRunning animation... (close window to exit)")
    anim = animator.animate()
    plt.show()
    
    # Print final statistics
    print("\n" + "="*60)
    print("SIMULATION COMPLETE")
    print("="*60)
    print(f"Total simulation time: {sim.time:.2f}s")
    print(f"Final edges: {sim.total_edges}")
    print(f"Max edges: {sim.max_edges}")
    print(f"Final robot positions:")
    state = sim.get_state()
    for i, pos in enumerate(state['positions']):
        print(f"  Robot {i}: ({pos[0]:.2f}, {pos[1]:.2f})")
    print("="*60 + "\n")
