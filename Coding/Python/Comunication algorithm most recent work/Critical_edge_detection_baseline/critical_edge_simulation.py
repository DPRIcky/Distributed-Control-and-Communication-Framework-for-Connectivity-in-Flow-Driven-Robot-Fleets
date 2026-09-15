"""Critical Edge Detection Baseline Simulation.

This simulation runner integrates the distributed critical edge detection
algorithm with the robot control framework.

Features:
- Distributed MST construction with critical edge preservation
- Robot formation control with CBF/CLF
- Dynamic topology management
- Real-time metrics and visualization support
"""

import numpy as np
from typing import List, Dict, Optional
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from config import SimulationConfig, ControlConfig
    from core import FlowField, ChainRobot
    from controllers import HybridCLFCBFController
    from graph import TopologyManager, ConnectivityTree
    from simulation import SimulationMetrics
    SIMULATION_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Could not import from main framework: {e}")
    print("Running in standalone mode with limited functionality.")
    SimulationConfig = None
    ControlConfig = None
    SIMULATION_AVAILABLE = False
    
    # Minimal fallback for SimulationMetrics
    class SimulationMetrics:
        def __init__(self):
            self.lambda2_values = []
            self.control_magnitudes = []
            self.pruning_events = []
        
        def record_control(self, robot_id, control_force):
            if not self.control_magnitudes:
                self.control_magnitudes.append([])
            self.control_magnitudes[-1].append(np.linalg.norm(control_force) if hasattr(control_force, '__len__') else control_force)
        
        def record_step(self, sim, time):
            # Try to compute lambda2 from the graph
            if hasattr(sim, 'topology_manager') and hasattr(sim.topology_manager, 'neighbor_graph'):
                try:
                    from scipy.sparse import csr_matrix
                    from scipy.sparse.linalg import eigsh
                    
                    graph = sim.topology_manager.neighbor_graph
                    if graph and len(graph) > 1:
                        # Build adjacency matrix
                        n = len(graph)
                        row, col, data = [], [], []
                        for i, neighbors in enumerate(graph):
                            for j in neighbors:
                                if i < j:
                                    row.extend([i, j])
                                    col.extend([j, i])
                                    data.extend([1, 1])
                        
                        if row:
                            A = csr_matrix((data, (row, col)), shape=(n, n))
                            # Compute Laplacian
                            D = csr_matrix((A.sum(axis=1).A1, (range(n), range(n))))
                            L = D - A
                            # Get second eigenvalue
                            try:
                                eigenvalues = eigsh(L, k=2, which='SM', return_eigenvectors=False)
                                lambda2 = eigenvalues[1] if len(eigenvalues) > 1 else 0.0
                            except:
                                lambda2 = 0.0
                        else:
                            lambda2 = 0.0
                    else:
                        lambda2 = 0.0
                except Exception:
                    lambda2 = 0.0
            else:
                lambda2 = 0.0
            
            self.lambda2_values.append(lambda2)
            if not self.control_magnitudes:
                self.control_magnitudes.append([1.0])
        
        def record_pruning_event(self, time, edge, method):
            self.pruning_events.append({'time': time, 'edge': edge, 'method': method})

try:
    from .critical_edge_baseline import CriticalEdgeBaseline
except ImportError:
    from critical_edge_baseline import CriticalEdgeBaseline


class CriticalEdgeSimulation:
    """Simulation using distributed critical edge detection for communication pruning.
    
    This simulation integrates the CriticalEdgeBaseline with robot dynamics,
    flow fields, and collision avoidance.
    
    Key Features:
    - Distributed MST construction (no centralized oracle)
    - Critical edge preservation (bridges are never pruned)
    - O(n) rounds for MST computation
    - Compatible with baseline comparison framework
    """
    
    def __init__(
        self,
        sim_config,
        control_config,
        consensus_config=None,
        update_frequency: int = 10,
        prefer_critical: bool = True,
        verbose: bool = False
    ):
        """Initialize critical edge simulation.
        
        Args:
            sim_config: Simulation configuration
            control_config: Control configuration
            consensus_config: Consensus configuration (optional, for compatibility)
            update_frequency: Update MST every N steps
            prefer_critical: Prioritize critical edges in MST
            verbose: Print detailed execution trace
        """
        # Store configs
        self.sim_config = sim_config
        self.control_config = control_config
        self.consensus_config = consensus_config
        
        # Extract frequently used values
        self.num_robots = sim_config.num_robots
        self.communication_radius = sim_config.communication_radius
        self.dt = sim_config.dt
        self.workspace = np.array(sim_config.workspace_size, dtype=float)
        self.time = 0.0
        self.verbose = verbose or sim_config.verbose
        
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
        
        # Initialize robots
        self.robots = self._initialize_robots()
        
        # Initialize flow field
        self.flow = FlowField()
        
        # Initialize topology manager
        self.topology_manager = TopologyManager(
            num_robots=self.num_robots,
            communication_radius=self.communication_radius
        )
        self.topology_manager.update_neighbor_graph(self.robots)
        
        # Initialize connectivity tree
        self.tree_builder = ConnectivityTree(
            num_robots=self.num_robots,
            workspace_size=tuple(self.workspace)
        )
        
        # Initialize controller
        self.controller = HybridCLFCBFController(
            config=control_config,
            communication_radius=self.communication_radius,
            goal_position=self.goal_position
        )
        
        # Initialize Critical Edge Baseline
        self.baseline = CriticalEdgeBaseline(
            num_robots=self.num_robots,
            update_frequency=update_frequency,
            prefer_critical=prefer_critical,
            verbose=self.verbose
        )
        
        # Initialize metrics
        self.metrics = SimulationMetrics()
        
        # Tracking
        self.total_edges = 0
        self.max_edges = 0
        self.step_count = 0
    
    def _initialize_robots(self) -> List[ChainRobot]:
        """Initialize robot positions and velocities.
        
        Returns:
            List of initialized robots
        """
        robots = []
        positions = self._generate_initial_positions()
        
        for i in range(self.num_robots):
            robot_rng = np.random.default_rng(self.rng.integers(0, 2**32 - 1))
            robot = ChainRobot(
                robot_id=i,
                position=positions[i],
                rng=robot_rng
            )
            robots.append(robot)
        
        return robots
    
    def _generate_initial_positions(self) -> np.ndarray:
        """Generate initial robot positions.
        
        Returns:
            Array of shape (num_robots, 2) with positions
        """
        # Start in left cluster
        positions = np.zeros((self.num_robots, 2))
        
        center_x = self.workspace[0] * 0.2
        center_y = self.workspace[1] * 0.5
        spread = min(self.workspace) * 0.15
        
        for i in range(self.num_robots):
            angle = 2 * np.pi * i / self.num_robots
            radius = spread * (0.3 + 0.7 * self.rng.random())
            
            positions[i] = [
                center_x + radius * np.cos(angle),
                center_y + radius * np.sin(angle)
            ]
        
        return positions
    
    def step(self):
        """Execute one simulation step.
        
        Steps:
        1. Update connectivity tree
        2. Update MST before control (ensures critical edges available)
        3. Compute and apply control with critical edge constraints
        4. Update robot states
        5. Update topology after movement
        6. Prune edges using critical edge detection
        7. Record metrics
        """
        self.step_count += 1
        
        # STEP 1: Update connectivity tree
        self.tree_builder.build_tree(
            self.robots,
            self.topology_manager.neighbor_graph
        )
        
        # STEP 2: Update MST BEFORE control (get latest critical edges)
        # This ensures CBF can use critical edges during control computation
        edge_lengths = self.topology_manager.gather_edge_lengths(self.robots)
        if edge_lengths and len(edge_lengths) > self.num_robots - 1:
            # Only trigger MST update if we have more than minimum edges
            self.baseline.find_edge_to_prune(edge_lengths=edge_lengths, robots=self.robots)
        elif edge_lengths and len(self.baseline.current_mst) == 0:
            # Initialize MST if not yet set
            self.baseline.find_edge_to_prune(edge_lengths=edge_lengths, robots=self.robots)
        
        # STEP 3: Convert critical edges to neighbor dict format for controller
        # IMPORTANT: Only include critical edges that are actually in the current topology
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
        
        # STEP 4: Compute and apply control for each robot
        for robot_id, robot in enumerate(self.robots):
            control_force = self.controller.compute_control(
                robot_id, 
                self.robots,
                critical_edges=critical_edges_dict
            )
            robot.step(self.flow, self.dt, self.time, self.workspace, control_force)
            
            # Record control magnitude
            self.metrics.record_control(robot_id, control_force)
        
        self.time += self.dt
        
        # STEP 5: Update topology after robot movement
        self.topology_manager.update_neighbor_graph(self.robots)
        
        # STEP 6: Critical edge detection and pruning
        # Re-compute edge lengths after robot movement
        edge_lengths = self.topology_manager.gather_edge_lengths(self.robots)
        
        if edge_lengths:
            edge_to_prune = self.baseline.find_edge_to_prune(
                edge_lengths=edge_lengths,
                robots=self.robots
            )
            
            if edge_to_prune is not None:
                # Prune the edge
                self.topology_manager.pruned_edges.add(edge_to_prune)
                
                if self.verbose:
                    print(f"[t={self.time:.2f}] Pruned edge {edge_to_prune}")
                
                # Record pruning event
                self.metrics.record_pruning_event(
                    self.time,
                    edge_to_prune,
                    "CriticalEdgeMST"
                )
        
        # STEP 5: Record metrics
        self.metrics.record_step(self, self.time)
        
        # Update edge tracking
        current_edges = len(self.topology_manager.get_current_edges())
        self.total_edges = current_edges
        self.max_edges = max(self.max_edges, current_edges)
        
        return {"decision": "step_complete"}
    
    def run(self, max_steps: int = 1000, max_time: Optional[float] = None):
        """Run simulation for specified duration.
        
        Args:
            max_steps: Maximum number of steps
            max_time: Maximum simulation time (seconds)
        """
        print("=" * 70)
        print("CRITICAL EDGE DETECTION BASELINE SIMULATION")
        print("=" * 70)
        print(f"Robots: {self.num_robots}")
        print(f"Communication radius: {self.communication_radius}")
        print(f"Update frequency: {self.baseline.update_frequency}")
        print(f"Prefer critical: {self.baseline.prefer_critical}")
        print("=" * 70)
        
        for step in range(max_steps):
            self.step()
            
            # Check termination conditions
            if max_time and self.time >= max_time:
                break
            
            # Print progress
            if step % 100 == 0:
                stats = self.baseline.get_statistics()
                print(f"Step {step:4d} | t={self.time:6.2f}s | "
                      f"Edges={self.total_edges:3d} | "
                      f"MST updates={stats['mst_updates']:3d} | "
                      f"Total rounds={stats['total_rounds']:4d}")
        
        print("\n" + "=" * 70)
        print("SIMULATION COMPLETE")
        print("=" * 70)
        self.print_statistics()
    
    def print_statistics(self):
        """Print simulation statistics."""
        stats = self.baseline.get_statistics()
        
        print("\nCritical Edge Baseline Statistics:")
        print("-" * 70)
        print(f"  Total steps: {self.step_count}")
        print(f"  Simulation time: {self.time:.2f}s")
        print(f"  Current edges: {self.total_edges}")
        print(f"  Max edges: {self.max_edges}")
        print(f"  MST updates: {stats['mst_updates']}")
        print(f"  Total DFS rounds: {stats['total_rounds']}")
        print(f"  Avg rounds/update: {stats['avg_rounds_per_update']:.2f}")
        print(f"  Total edges pruned: {stats['edges_pruned']}")
        print(f"  Current MST size: {stats['current_mst_size']}")
        print(f"  Critical edges found: {stats['critical_edges_detected']}")
        print("-" * 70)
    
    def get_method_name(self) -> str:
        """Get method name for reporting.
        
        Returns:
            Method identifier string
        """
        return self.baseline.get_name()
    
    def get_current_edges(self) -> int:
        """Get current edge count.
        
        Returns:
            Number of active edges
        """
        return len(self.topology_manager.get_current_edges())
    
    def get_metrics(self):
        """Get simulation metrics.
        
        Returns:
            Metrics object
        """
        return self.metrics


if __name__ == "__main__":
    print("Critical Edge Detection Simulation Module")
    print("=" * 70)
    print("\nThis module provides a simulation runner for the critical edge")
    print("detection baseline integrated with robot dynamics.")
    print("\nUSAGE:")
    print("""
    from config import SimulationConfig, ControlConfig
    from critical_edge_simulation import CriticalEdgeSimulation
    
    # Create configs
    sim_config = SimulationConfig(num_robots=10)
    control_config = ControlConfig()
    
    # Create simulation
    sim = CriticalEdgeSimulation(sim_config, control_config)
    
    # Run simulation
    sim.run(max_steps=1000)
    
    # Get statistics
    sim.print_statistics()
    """)
