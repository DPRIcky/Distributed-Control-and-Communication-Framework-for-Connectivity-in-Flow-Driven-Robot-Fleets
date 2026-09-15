"""
Pruned Simulation with Distributed Edge Pruning

Extends baseline simulation with provably safe distributed pruning.
Maintains connectivity while creating sparse communication topologies.
"""

import sys
import os
import numpy as np
from typing import List, Dict, Set, Tuple

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from baseline_simulation.fully_connected_simulation import FullyConnectedSimulation
from config import SimulationConfig, ControlConfig
from .pruning_controller import ProgressivePruningController


class PrunedSimulation(FullyConnectedSimulation):
    """
    Simulation with distributed pruning that maintains connectivity.
    
    Extends FullyConnectedSimulation to add progressive edge pruning
    with formal guarantees.
    
    Attributes:
        pruning_controllers: Dict mapping robot_id to PruningController
        target_degree: Target number of neighbors per robot
        k_connectivity: Connectivity level (1=connected, 2=2-connected)
        active_edges: Current set of active (non-pruned) edges
        pruning_statistics: History of pruning stats over time
    """
    
    def __init__(
        self,
        sim_config: SimulationConfig,
        control_config: ControlConfig,
        target_degree: int = 3,
        k_connectivity: int = 1
    ):
        """
        Initialize pruned simulation.
        
        Args:
            sim_config: Simulation configuration
            control_config: Control configuration
            target_degree: Target number of neighbors (3-4 for sparse chain)
            k_connectivity: 1=connected, 2=2-connected
        """
        # Initialize base simulation
        super().__init__(sim_config, control_config)
        
        # Pruning parameters
        self.target_degree = target_degree
        self.k_connectivity = k_connectivity
        
        # Create pruning controller for each robot
        self.pruning_controllers: Dict[int, ProgressivePruningController] = {}
        for i in range(self.num_robots):
            self.pruning_controllers[i] = ProgressivePruningController(
                robot_id=i,
                target_degree=target_degree,
                k_connectivity=k_connectivity,
                max_k_hop=3,
                communication_radius=self.communication_radius,
                information_gain_rate=0.005  # Very slow: 0.5% per step, takes 200 steps (~30s at 15fps)
            )
        
        # Active edges (after pruning) - Start with ALL edges (no pruning yet)
        # Robots will gradually prune as they build  confidence
        self.active_edges: Set[Tuple[int, int]] = set()
        
        # Statistics tracking
        self.pruning_statistics: List[Dict] = []
        self.iteration_count = 0
        
        print(f"\n[Pruned Simulation Initialized]")
        print(f"  Robots: {self.num_robots}")
        print(f"  Target Degree: {target_degree}")
        print(f"  Connectivity Level: {k_connectivity}-connected")
        print(f"  Communication Radius: {self.communication_radius}m")
        print(f"  Information Gain Rate: 0.5% per timestep")
        print(f"  -> Takes ~200 timesteps to reach 100% information")
        print(f"  -> At 15 fps animation, that's ~13 seconds of gradual learning")
        print(f"  Goal: ({self.goal_position[0]:.1f}, {self.goal_position[1]:.1f})")
    
    def _gather_neighbor_data(self, robot_idx: int) -> List[Tuple]:
        """
        Gather neighbor information for a robot.
        
        Args:
            robot_idx: Index of robot to gather neighbors for
        
        Returns:
            List of (neighbor_id, distance, position, state, neighbor_list)
        """
        robot = self.robots[robot_idx]
        neighbor_data = []
        
        for j, other in enumerate(self.robots):
            if j == robot_idx:
                continue
            
            # Check if in communication range
            distance = np.linalg.norm(robot.position - other.position)
            
            if distance <= self.communication_radius * 1.2:  # Slightly beyond range for 2-hop
                # CRITICAL: Get this robot's ACTIVE neighbor list (pruned topology), not potential
                # This prevents false positives where we think alternate paths exist through pruned edges
                other_controller = self.pruning_controllers[j]
                active_neighbors = other_controller.get_active_edges()
                
                # Bootstrapping: if robot hasn't started pruning yet, use potential neighbors
                if len(active_neighbors) == 0:
                    other_neighbors = [
                        k for k, third in enumerate(self.robots)
                        if k != j and np.linalg.norm(other.position - third.position) <= self.communication_radius
                    ]
                else:
                    other_neighbors = list(active_neighbors)
                
                neighbor_data.append((
                    j,  # neighbor_id
                    distance,
                    other.position,
                    other.position,  # Use position as state for now
                    other_neighbors  # Their ACTIVE neighbor list (or potential if not pruned yet)
                ))
        
        return neighbor_data
    
    def update_pruning(self):
        """
        Execute one timestep of distributed pruning.
        
        Each robot independently:
        1. Gathers 2-hop neighbor information (gradually over time)
        2. Identifies safe-to-prune edges (only with sufficient confidence)
        3. Prunes edges based on priority and budget
        """
        self.iteration_count += 1
        
        # Step 1: Each robot updates its neighbor information
        for i, robot in enumerate(self.robots):
            controller = self.pruning_controllers[i]
            
            # Update robot state
            controller.update_state(robot.position, robot.position)
            
            # Gather neighbor data (includes 2-hop info)
            neighbor_data = self._gather_neighbor_data(i)
            
            # Update controller's knowledge
            controller.update_neighbors(neighbor_data)
        
        # Step 2: Each robot independently prunes edges
        for i in range(self.num_robots):
            controller = self.pruning_controllers[i]
            pruned, kept = controller.prune_edges()
        
        # Step 3: Build active edge set (union of all robots' active edges)
        # If no robot has pruned yet (initial state), keep all edges
        self.active_edges.clear()
        
        # Check if any robot has started pruning
        any_pruning = any(len(c.active_neighbors) > 0 for c in self.pruning_controllers.values())
        
        if not any_pruning:
            # Initial state: keep all edges
            for i in range(self.num_robots):
                for j in range(i+1, self.num_robots):
                    dist = np.linalg.norm(
                        self.robots[i].position - self.robots[j].position
                    )
                    if dist <= self.communication_radius:
                        self.active_edges.add((i, j))
        else:
            # Normal operation: union of active edges
            for i in range(self.num_robots):
                controller = self.pruning_controllers[i]
                for j in controller.get_active_edges():
                    edge = (min(i, j), max(i, j))
                    self.active_edges.add(edge)
        
        # Step 4: Collect statistics
        step_stats = {
            'time': self.time,
            'iteration': self.iteration_count,
            'total_potential_edges': len(self.current_edges()),
            'active_edges': len(self.active_edges),
            'avg_degree': np.mean([len(c.get_active_edges()) if len(c.active_neighbors) > 0 else len(c.k_hop_neighbors.get(1, set())) for c in self.pruning_controllers.values()]),
            'avg_completeness': np.mean([c.information_completeness for c in self.pruning_controllers.values()]),
            'avg_aggressiveness': np.mean([c.pruning_aggressiveness for c in self.pruning_controllers.values()]),
        }
        self.pruning_statistics.append(step_stats)
        
        # CONNECTIVITY CHECK: Warn if graph might be disconnected
        min_edges_required = self.num_robots - 1
        potential_edges = len(self.current_edges())
        
        if len(self.active_edges) < min_edges_required:
            print(f"\n  [CRITICAL t={self.time:.2f}s] DISCONNECTION DETECTED!")
            print(f"    Active edges: {len(self.active_edges)} < Minimum: {min_edges_required}")
            print(f"    Potential edges: {potential_edges}")
            print(f"    This violates connectivity guarantee!")
            
            # Check if disconnection is due to geometric constraints
            if potential_edges < min_edges_required:
                print(f"    ROOT CAUSE: Robots spread beyond {self.communication_radius}m")
                print(f"    CBF cannot maintain connectivity - geometric constraint violated")
                print(f"    Formation is too spread out for {self.communication_radius}m radius!\n")
    
    def step(self):
        """
        Execute one simulation timestep with pruning.
        
        Overrides parent step() to integrate pruning.
        """
        # Update pruning (decides which edges are active)
        self.update_pruning()
        
        # Update topology to only use active edges
        self._update_topology_with_pruning()
        
        # Run standard simulation step
        super().step()
    
    def _update_topology_with_pruning(self):
        """Update topology manager to only consider active edges.
        
        CRITICAL: Updates topology_manager.pruned_edges so CBF controller
        knows which edges to IGNORE (not maintain).
        """
        # Get all potential edges (within communication range)
        all_potential_edges = set()
        for i in range(self.num_robots):
            for j in range(i+1, self.num_robots):
                dist = np.linalg.norm(
                    self.robots[i].position - self.robots[j].position
                )
                if dist <= self.communication_radius:
                    all_potential_edges.add((i, j))
        
        # Calculate which edges are pruned (potential but not active)
        pruned_edges = all_potential_edges - self.active_edges
        
        # CRITICAL: Update topology manager's pruned_edges
        # This tells CBF controller which edges to ignore
        self.topology_manager.pruned_edges = pruned_edges
        
        # Update neighbor graph will be called by parent's step()
        # It will respect pruned_edges and only build graph with active edges
    
    def current_edges(self) -> Dict[Tuple[int, int], float]:
        """
        Get current edges within communication range (before pruning).
        
        Returns:
            Dictionary mapping (i,j) to distance
        """
        edges = {}
        for i in range(self.num_robots):
            for j in range(i+1, self.num_robots):
                dist = np.linalg.norm(
                    self.robots[i].position - self.robots[j].position
                )
                if dist <= self.communication_radius:
                    edges[(i, j)] = dist
        return edges
    
    def get_active_edges(self) -> Set[Tuple[int, int]]:
        """Get current active (non-pruned) edges."""
        return self.active_edges.copy()
    
    def get_pruned_edges(self) -> Set[Tuple[int, int]]:
        """Get current pruned edges."""
        all_edges = set(self.current_edges().keys())
        return all_edges - self.active_edges
    
    def get_pruning_statistics(self) -> List[Dict]:
        """Get history of pruning statistics."""
        return self.pruning_statistics.copy()
    
    def get_state(self) -> Dict:
        """
        Get current simulation state including pruning info.
        
        Extends parent get_state() with pruning statistics.
        """
        base_state = super().get_state()
        
        # Add pruning info
        base_state['pruning'] = {
            'active_edges': len(self.active_edges),
            'pruned_edges': len(self.get_pruned_edges()),
            'avg_degree': np.mean([len(c.get_active_edges()) for c in self.pruning_controllers.values()]),
            'target_degree': self.target_degree,
            'k_connectivity': self.k_connectivity,
        }
        
        return base_state
    
    def print_statistics(self):
        """Print current statistics including pruning info."""
        super().print_statistics()
        
        if self.pruning_statistics:
            latest = self.pruning_statistics[-1]
            print(f"\n[Pruning Statistics]")
            print(f"  Potential Edges: {latest['total_potential_edges']}")
            print(f"  Active Edges: {latest['active_edges']}")
            print(f"  Pruned: {latest['total_potential_edges'] - latest['active_edges']}")
            print(f"  Avg Degree: {latest['avg_degree']:.1f} (target: {self.target_degree})")
            print(f"  Info Completeness: {latest['avg_completeness']:.1%}")
            print(f"  Pruning Aggressiveness: {latest['avg_aggressiveness']:.1%}")
