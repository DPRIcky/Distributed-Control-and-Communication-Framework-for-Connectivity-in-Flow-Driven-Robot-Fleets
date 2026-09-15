"""
Distributed Network Algorithms Implementation
Implementation of distributed algorithms for neighbor structure identification 
and connectivity assurance as described in the paper.
"""

import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from typing import Dict, Set, List, Tuple
import copy
import random
try:
    import cvxpy as cp
    CVXPY_AVAILABLE = True
except ImportError:
    CVXPY_AVAILABLE = False
    import scipy.optimize
    print("Warning: cvxpy not available, using scipy.optimize for CLF-CBF control")

class Node:
    """A node implementing distributed neighbor structure identification with underwater dynamics"""
    
    def __init__(self, node_id: int, position: np.ndarray):
        self.id = node_id
        self.position = position
        self.direct_neighbors = set()
        
        # ξi(k) - node i's estimate of its in-neighbors (binary vector)
        self.xi = {}  # Dictionary: node_id -> 0 or 1
        
        # ωi(k) - node i's estimate of neighbor structure (distance vector)  
        self.omega = {}  # Dictionary: node_id -> distance
        
        # Algorithm state
        self.k = 0
        
        # === UNDERWATER DYNAMICS PROPERTIES ===
        # Robot physical properties
        self.mass = 2.0  # kg (underwater robot mass)
        self.volume = 0.002  # m³ (robot volume for buoyancy)
        self.drag_coefficient = 0.8  # Drag coefficient for underwater movement
        self.cross_sectional_area = 0.01  # m² (frontal area for drag calculation)
        
        # Dynamic state variables
        self.velocity = np.zeros(2)  # Current velocity [vx, vy]
        self.acceleration = np.zeros(2)  # Current acceleration
        
        # Environmental interaction history
        self.flow_history = []  # Track flow field influence over time
        self.drag_history = []  # Track drag forces over time
        
    def add_direct_neighbor(self, neighbor_id: int):
        """Add a direct neighbor"""
        self.direct_neighbors.add(neighbor_id)
    
    def initialize_algorithm(self, total_nodes: int):
        """Initialize ξ and ω according to equations (3)"""
        for j in range(1, total_nodes + 1):
            if j == self.id:
                self.xi[j] = 1  # ξi,i(0) = 1
                self.omega[j] = 0  # ωi,i(0) = 0
            else:
                self.xi[j] = 0  # ξi,j(0) = 0 for j ≠ i
                self.omega[j] = float('inf')  # ωi,j(0) = ∞ for j ≠ i
    
    def update_xi_step(self, all_nodes: Dict[int, 'Node']):
        """Update ξi,j(k+1) according to equation (1)"""
        new_xi = {}
        
        for j in self.xi.keys():
            # ξi,j(k+1) = max over l∈Ni∪i of ξl,j(k)
            max_val = self.xi[j]  # Start with own value (i in Ni∪i)
            
            # Check all neighbors
            for neighbor_id in self.direct_neighbors:
                if neighbor_id in all_nodes:
                    neighbor_xi_j = all_nodes[neighbor_id].xi.get(j, 0)
                    max_val = max(max_val, neighbor_xi_j)
            
            new_xi[j] = max_val
        
        return new_xi
    
    def update_omega_step(self, all_nodes: Dict[int, 'Node'], new_xi: Dict[int, int]):
        """Update ωi,j(k+1) according to equation (2)"""
        new_omega = {}
        
        for j in self.omega.keys():
            if new_xi[j] == self.xi[j]:
                # ωi,j(k+1) = ωi,j(k) if ξi,j(k+1) = ξi,j(k)
                new_omega[j] = self.omega[j]
            else:
                # ωi,j(k+1) = min over l∈Ni of (ωl,j(k) + 1) if ξi,j(k+1) > ξi,j(k)
                min_val = float('inf')
                
                for neighbor_id in self.direct_neighbors:
                    if neighbor_id in all_nodes:
                        neighbor_omega_j = all_nodes[neighbor_id].omega.get(j, float('inf'))
                        if neighbor_omega_j != float('inf'):
                            min_val = min(min_val, neighbor_omega_j + 1)
                
                new_omega[j] = min_val if min_val != float('inf') else float('inf')
        
        return new_omega
    
    def execute_one_step(self, all_nodes: Dict[int, 'Node']):
        """Execute one step of the distributed algorithm"""
        # Update ξ first
        new_xi = self.update_xi_step(all_nodes)
        
        # Update ω based on new ξ
        new_omega = self.update_omega_step(all_nodes, new_xi)
        
        # Apply updates
        self.xi = new_xi
        self.omega = new_omega
        self.k += 1
        
        return True
    
    def get_neighbor_sets(self) -> Dict[int, List[int]]:
        """Get N(p)_i for different values of p"""
        neighbor_sets = {}
        
        for node_id, distance in self.omega.items():
            if node_id != self.id and distance != float('inf') and distance > 0:
                p = int(distance)
                if p not in neighbor_sets:
                    neighbor_sets[p] = []
                neighbor_sets[p].append(node_id)
        
        return neighbor_sets
    
    def is_network_connected(self) -> bool:
        """Check if network is connected from this node's perspective"""
        # Network is connected if ξi,j(n) ≠ 0 for all j ≠ i
        for j, xi_val in self.xi.items():
            if j != self.id and xi_val == 0:
                return False
        return True
    
    def find_highest_connected_index(self) -> int:
        """Find i* = max{j | ξi,j(k) = 1}"""
        connected_nodes = [j for j, xi_val in self.xi.items() if xi_val == 1]
        return max(connected_nodes) if connected_nodes else -1
    
    def find_highest_unconnected_index(self) -> int:
        """Find j* = max{j | ξi,j(k) = 0}"""
        unconnected_nodes = [j for j, xi_val in self.xi.items() if xi_val == 0]
        return max(unconnected_nodes) if unconnected_nodes else -1
    
    def calculate_delta_measures(self, all_nodes: Dict[int, 'Node']) -> Dict[int, Dict[int, int]]:
        """Calculate Δ(il)_i measures for critical edge detection (Equation 6)"""
        delta_measures = {}
        
        # For each direct neighbor l
        for neighbor_l in self.direct_neighbors:
            delta_il = {}
            
            # For each node j in the network
            for j in range(1, 9):  # Nodes 1-8
                if j in all_nodes:
                    # Δ(il)_i,j = ωi,j(n) - ωl,j(n)
                    omega_i_j = self.omega.get(j, float('inf'))
                    omega_l_j = all_nodes[neighbor_l].omega.get(j, float('inf'))
                    
                    if omega_i_j != float('inf') and omega_l_j != float('inf'):
                        delta_il[j] = omega_i_j - omega_l_j
                    else:
                        delta_il[j] = 0  # Default for unreachable nodes
            
            delta_measures[neighbor_l] = delta_il
        
        return delta_measures
    
    def is_edge_critical(self, neighbor_l: int, all_nodes: Dict[int, 'Node']) -> bool:
        """Determine if edge (i,l) is critical using Theorem 2 (Equation 9)"""
        delta_measures = self.calculate_delta_measures(all_nodes)
        
        if neighbor_l not in delta_measures:
            return False
        
        delta_il = delta_measures[neighbor_l]
        
        # Check condition (9) for all nodes j
        for j in range(1, 9):
            if j in delta_il:
                # Condition 1: Δ(il)_i,j ≠ 0 (not equidistant)
                if delta_il[j] == 0:
                    return False  # Found equidistant node, not critical
                
                # Condition 2: Check for all adjacent nodes i' ∈ Ni and l' ∈ Nl
                # that {Δ(ii')_i,j, Δ(ll')_l,j} ≠ {1, 1}
                
                # Check all neighbors of node i (self)
                for i_prime in self.direct_neighbors:
                    if i_prime != neighbor_l:  # Don't check the edge we're testing
                        delta_ii_prime = all_nodes[self.id].calculate_delta_measures(all_nodes)
                        if i_prime in delta_ii_prime and j in delta_ii_prime[i_prime]:
                            delta_ii_prime_j = delta_ii_prime[i_prime][j]
                            
                            # Check all neighbors of node l
                            for l_prime in all_nodes[neighbor_l].direct_neighbors:
                                if l_prime != self.id:  # Don't check the edge we're testing
                                    delta_ll_prime = all_nodes[neighbor_l].calculate_delta_measures(all_nodes)
                                    if l_prime in delta_ll_prime and j in delta_ll_prime[l_prime]:
                                        delta_ll_prime_j = delta_ll_prime[l_prime][j]
                                        
                                        # If both deltas equal 1, then edge is not critical
                                        if delta_ii_prime_j == 1 and delta_ll_prime_j == 1:
                                            return False
        
        return True  # Edge is critical if all conditions of (9) are satisfied
    
    def find_critical_edges(self, all_nodes: Dict[int, 'Node']) -> Set[int]:
        """Find all critical edges for this node"""
        critical_neighbors = set()
        
        for neighbor_l in self.direct_neighbors:
            if self.is_edge_critical(neighbor_l, all_nodes):
                critical_neighbors.add(neighbor_l)
        
        return critical_neighbors
    
    def print_delta_measures(self, all_nodes: Dict[int, 'Node']):
        """Print Δ(il)_i measures for this node"""
        delta_measures = self.calculate_delta_measures(all_nodes)
        
        print(f"Node {self.id} - Delta measures:")
        for neighbor_l in sorted(delta_measures.keys()):
            delta_il = delta_measures[neighbor_l]
            delta_vector = [delta_il.get(j, 0) for j in range(1, 9)]
            print(f"  Δ({self.id}{neighbor_l})_{self.id} = {delta_vector}")
    
    def print_state(self):
        """Print current ξ and ω values"""
        xi_dict = {j: self.xi[j] for j in sorted(self.xi.keys())}
        omega_dict = {}
        for j in sorted(self.omega.keys()):
            val = self.omega[j]
            omega_dict[j] = val if val != float('inf') else 'inf'
        
        print(f"Node {self.id}:")
        print(f"  xi = {xi_dict}")
        print(f"  omega = {omega_dict}")
    
    def compute_edge_priority(self, neighbor_id: int, all_nodes: Dict[int, 'Node'], verbose: bool = False) -> float:
        """
        Compute priority for edge (self.id, neighbor_id) using Triangle Participation + Local Redundancy
        Higher priority = remove first
        
        Mathematical Formula:
        P(i,j) = R(i,j) × (1 + T(i,j))
        where:
        - R(i,j) = |CN(i,j)| / min(deg(i), deg(j))  [Local Redundancy]
        - T(i,j) = |CN(i,j)|                         [Triangle Count]
        - CN(i,j) = N(i) ∩ N(j)                      [Common Neighbors]
        """
        if neighbor_id not in self.direct_neighbors or neighbor_id not in all_nodes:
            return 0.0
        
        neighbor_node = all_nodes[neighbor_id]
        
        # Calculate common neighbors: CN(i,j) = N(i) ∩ N(j)
        my_neighbors = self.direct_neighbors
        neighbor_neighbors = neighbor_node.direct_neighbors
        common_neighbors = my_neighbors.intersection(neighbor_neighbors)
        
        # Triangle count: T(i,j) = |CN(i,j)|
        triangle_count = len(common_neighbors)
        
        # Degrees
        my_degree = len(my_neighbors)
        neighbor_degree = len(neighbor_neighbors)
        
        if verbose:
            print(f"    🔍 Edge ({self.id},{neighbor_id}) Analysis from Node {self.id}:")
            print(f"      My neighbors: {sorted(list(my_neighbors))}")
            print(f"      Neighbor's neighbors: {sorted(list(neighbor_neighbors))}")
            print(f"      Common neighbors: {sorted(list(common_neighbors))}")
            print(f"      Triangle count T({self.id},{neighbor_id}) = {triangle_count}")
            print(f"      My degree = {my_degree}, Neighbor degree = {neighbor_degree}")
        
        if my_degree == 0 or neighbor_degree == 0:
            if verbose:
                print(f"      ⚠️ Zero degree detected - priority = 0.0")
            return 0.0
        
        # Local redundancy: R(i,j) = |CN(i,j)| / min(deg(i), deg(j))
        redundancy = triangle_count / min(my_degree, neighbor_degree)
        
        # Combined priority: P(i,j) = R(i,j) × (1 + T(i,j))
        priority = redundancy * (1 + triangle_count)
        
        if verbose:
            print(f"      Redundancy R({self.id},{neighbor_id}) = {triangle_count}/{min(my_degree, neighbor_degree)} = {redundancy:.3f}")
            print(f"      Priority P({self.id},{neighbor_id}) = {redundancy:.3f} × (1 + {triangle_count}) = {priority:.3f}")
            print()
        
        return priority
    
    def get_local_edge_priorities(self, all_nodes: Dict[int, 'Node']) -> Dict[int, float]:
        """Get priority scores for all edges connected to this node"""
        priorities = {}
        for neighbor_id in self.direct_neighbors:
            priority = self.compute_edge_priority(neighbor_id, all_nodes)
            priorities[neighbor_id] = priority
        return priorities


class UnderwaterEnvironment:
    """
    Underwater Environment Simulator for Multi-Robot Systems
    Simulates realistic underwater dynamics including:
    - Flow fields (currents, vortices, turbulence)
    - Viscous drag forces
    - Buoyancy effects
    - Water density variations
    - Acoustic propagation effects
    """
    
    def __init__(self, workspace_size=(10, 10), simulation_time=0.0):
        self.workspace_size = workspace_size
        self.simulation_time = simulation_time
        
        # === WATER PROPERTIES ===
        self.water_density = 1025.0  # kg/m³ (seawater density)
        self.water_viscosity = 1.002e-3  # Pa·s (dynamic viscosity of water at 20°C)
        self.gravity = 9.81  # m/s² (gravitational acceleration)
        
        # === FLOW FIELD PARAMETERS ===
        # Primary current (steady flow)
        self.current_velocity = np.array([0.2, 0.1])  # m/s [vx, vy] - gentle current
        self.current_direction_change_rate = 0.01  # How fast current direction changes
        
        # Vortex parameters (rotating flow structures)
        self.vortex_centers = [
            np.array([3.0, 7.0]),  # Vortex 1 center
            np.array([7.0, 3.0])   # Vortex 2 center
        ]
        self.vortex_strengths = [1.5, -1.2]  # Circulation strengths (+ = CCW, - = CW)
        self.vortex_radii = [2.0, 1.8]  # Effective radii
        
        # Turbulence parameters
        self.turbulence_intensity = 0.05  # Random flow fluctuation intensity
        self.turbulence_scale = 1.0  # Spatial scale of turbulent eddies
        
        # === ACOUSTIC PROPAGATION ===
        # Underwater acoustic communication is affected by distance and environment
        self.sound_speed = 1500.0  # m/s (speed of sound in water)
        self.acoustic_absorption = 0.1  # dB/m (frequency-dependent absorption)
        
        # === ENVIRONMENTAL VARIATION ===
        self.depth_layers = {
            'surface': {'depth': 0, 'density': 1020, 'current_factor': 1.2},
            'mid': {'depth': 5, 'density': 1025, 'current_factor': 1.0},
            'deep': {'depth': 8, 'density': 1030, 'current_factor': 0.8}
        }
        
    def update_time(self, dt):
        """Update simulation time and time-dependent environmental factors"""
        self.simulation_time += dt
        
        # Update current direction with slow oscillation
        angle_change = self.current_direction_change_rate * self.simulation_time
        rotation_matrix = np.array([
            [np.cos(angle_change), -np.sin(angle_change)],
            [np.sin(angle_change), np.cos(angle_change)]
        ])
        base_current = np.array([0.2, 0.1])
        self.current_velocity = rotation_matrix @ base_current
    
    def get_flow_field(self, position: np.ndarray) -> np.ndarray:
        """
        Calculate flow field velocity at given position
        Combines steady current, vortices, and turbulence
        """
        x, y = position
        total_flow = np.copy(self.current_velocity)
        
        # Add vortex effects
        for i, center in enumerate(self.vortex_centers):
            r_vec = position - center  # Vector from vortex center to position
            r = np.linalg.norm(r_vec)
            
            if r > 0.1:  # Avoid singularity at vortex center
                # Velocity induced by vortex: v = (Γ/(2πr)) * perpendicular_to_r
                strength = self.vortex_strengths[i]
                radius = self.vortex_radii[i]
                
                # Gaussian decay with distance
                decay_factor = np.exp(-(r/radius)**2)
                
                # Perpendicular velocity (rotate r_vec by 90°)
                perp_velocity = np.array([-r_vec[1], r_vec[0]]) / r
                vortex_velocity = (strength / (2 * np.pi * r)) * perp_velocity * decay_factor
                
                total_flow += vortex_velocity
        
        # Add turbulence (spatially correlated noise)
        turbulence_x = self.turbulence_intensity * np.sin(2*np.pi*x/self.turbulence_scale + self.simulation_time*0.5)
        turbulence_y = self.turbulence_intensity * np.cos(2*np.pi*y/self.turbulence_scale + self.simulation_time*0.3)
        turbulence = np.array([turbulence_x, turbulence_y])
        
        total_flow += turbulence
        
        return total_flow
    
    def calculate_drag_force(self, node: Node, relative_velocity: np.ndarray) -> np.ndarray:
        """
        Calculate viscous drag force on robot
        F_drag = -0.5 * ρ * Cd * A * |v| * v
        """
        if np.linalg.norm(relative_velocity) < 1e-6:
            return np.zeros(2)
        
        speed = np.linalg.norm(relative_velocity)
        drag_magnitude = 0.5 * self.water_density * node.drag_coefficient * node.cross_sectional_area * speed**2
        
        # Drag opposes motion
        drag_direction = -relative_velocity / speed
        drag_force = drag_magnitude * drag_direction
        
        return drag_force
    
    def calculate_buoyancy_force(self, node: Node) -> np.ndarray:
        """
        Calculate buoyancy force (simplified to vertical component)
        F_buoyancy = ρ_water * V * g - m * g
        """
        buoyant_force = self.water_density * node.volume * self.gravity
        weight = node.mass * self.gravity
        
        # Net vertical force (positive = upward)
        net_buoyancy = buoyant_force - weight
        
        # In 2D, we'll apply a small upward/downward bias
        return np.array([0, net_buoyancy * 0.1])  # Scaled down for 2D simulation
    
    def get_acoustic_range_factor(self, distance: float) -> float:
        """
        Calculate acoustic communication range factor based on underwater propagation
        Accounts for absorption and spreading losses
        """
        if distance <= 0:
            return 1.0
        
        # Spreading loss: 20*log10(r)
        spreading_loss = 20 * np.log10(distance)
        
        # Absorption loss: α * r
        absorption_loss = self.acoustic_absorption * distance
        
        # Total loss in dB
        total_loss_db = spreading_loss + absorption_loss
        
        # Convert to linear scale (range reduction factor)
        range_factor = 10**(-total_loss_db/20)
        
        return max(range_factor, 0.1)  # Minimum 10% of original range
    
    def get_environmental_layer(self, y_position: float) -> dict:
        """Get environmental properties based on depth (y-position)"""
        if y_position <= 3:
            return self.depth_layers['surface']
        elif y_position <= 7:
            return self.depth_layers['mid']
        else:
            return self.depth_layers['deep']
    
    def visualize_flow_field(self, ax, resolution=20):
        """Visualize the flow field with arrows"""
        x_range = np.linspace(0, self.workspace_size[0], resolution)
        y_range = np.linspace(0, self.workspace_size[1], resolution)
        X, Y = np.meshgrid(x_range, y_range)
        
        U = np.zeros_like(X)
        V = np.zeros_like(Y)
        
        for i in range(resolution):
            for j in range(resolution):
                pos = np.array([X[i,j], Y[i,j]])
                flow = self.get_flow_field(pos)
                U[i,j] = flow[0]
                V[i,j] = flow[1]
        
        # Plot flow field with arrows
        ax.quiver(X, Y, U, V, alpha=0.5, scale=15, width=0.002, color='cyan')
        
        # Plot vortex centers
        for i, center in enumerate(self.vortex_centers):
            strength = self.vortex_strengths[i]
            color = 'red' if strength > 0 else 'blue'
            ax.scatter(center[0], center[1], c=color, s=100, alpha=0.7, 
                      marker='o' if strength > 0 else 'x', 
                      label=f'Vortex {i+1} ({"CCW" if strength > 0 else "CW"})')


class DistributedNetwork:
    """Network implementing distributed algorithms with underwater dynamics"""
    
    def __init__(self, create_default_graph=True, enable_underwater_physics=True):
        self.nodes: Dict[int, Node] = {}
        self.k = 0
        
        # === UNDERWATER ENVIRONMENT INTEGRATION ===
        self.enable_underwater_physics = enable_underwater_physics
        if self.enable_underwater_physics:
            self.underwater_env = UnderwaterEnvironment(workspace_size=(10, 10))
            print("UNDERWATER Environment Enabled")
            print(f"   Water density: {self.underwater_env.water_density} kg/m³")
            print(f"   Current velocity: {self.underwater_env.current_velocity} m/s")
            print(f"   Vortices: {len(self.underwater_env.vortex_centers)} active")
            print(f"   Acoustic propagation: Enabled")
        else:
            self.underwater_env = None
            print("AIR Environment (no underwater physics)")
        
        # Time step for physics integration
        self.dt = 0.1  # seconds per animation frame
        
        if create_default_graph:
            self.create_specific_graph()
            
            # Initialize all nodes
            for node in self.nodes.values():
                node.initialize_algorithm(8)
                # Initialize underwater physics properties if enabled
                if self.enable_underwater_physics:
                    self.initialize_underwater_dynamics(node)
            
            print("\nInitial Graph Structure:")
            self.print_graph_structure()
            print("\nInitial State (k=0):")
            self.print_all_states()
    
    def initialize_underwater_dynamics(self, node: Node):
        """Initialize underwater physics properties for a node"""
        # Randomize some physical properties for diversity
        base_mass = 2.0
        node.mass = base_mass + np.random.normal(0, 0.2)  # 2±0.2 kg
        node.volume = node.mass / 1800  # Slightly less dense than water for slight buoyancy
        node.drag_coefficient = 0.8 + np.random.normal(0, 0.1)  # 0.8±0.1
        node.cross_sectional_area = 0.01 + np.random.normal(0, 0.002)  # 0.01±0.002 m²
        
        # Initialize velocity and acceleration
        node.velocity = np.random.normal(0, 0.05, 2)  # Small initial random velocity
        node.acceleration = np.zeros(2)
        
        print(f"   Robot {node.id}: mass={node.mass:.2f}kg, drag_coeff={node.drag_coefficient:.2f}")
    
    def apply_underwater_dynamics(self, node: Node, control_force: np.ndarray):
        """
        Apply underwater physics to update node position and velocity
        Uses proper physics integration with environmental forces
        """
        if not self.enable_underwater_physics:
            # Simple kinematic update (original behavior)
            node.velocity = control_force
            node.position += node.velocity
            return
        
        # === UNDERWATER PHYSICS INTEGRATION ===
        
        # 1. Get environmental flow field at current position
        flow_velocity = self.underwater_env.get_flow_field(node.position)
        
        # 2. Calculate relative velocity (robot velocity relative to water)
        relative_velocity = node.velocity - flow_velocity
        
        # 3. Calculate forces acting on the robot
        forces = []
        
        # Control force (thrust from robot actuators)
        thrust_force = control_force * 20.0  # Scale up for realistic thrust
        forces.append(("Thrust", thrust_force))
        
        # Drag force (opposes relative motion through water)
        drag_force = self.underwater_env.calculate_drag_force(node, relative_velocity)
        forces.append(("Drag", drag_force))
        
        # Buoyancy force (vertical only in 2D)
        buoyancy_force = self.underwater_env.calculate_buoyancy_force(node)
        forces.append(("Buoyancy", buoyancy_force))
        
        # Flow force (water pushes robot along with current) - REDUCED MULTIPLIER to reduce flow influence
        flow_force = self.underwater_env.water_density * node.volume * flow_velocity * 1.0  # REDUCED from 2.0 to 0.5
        forces.append(("Flow", flow_force))
        
        # 4. Sum all forces
        total_force = np.sum([force for _, force in forces], axis=0)
        
        # 5. Calculate acceleration (F = ma)
        node.acceleration = total_force / node.mass
        
        # 6. Integrate velocity only (position updated in main loop)
        node.velocity += node.acceleration * self.dt
        
        # Store debug info about forces applied
        
        # 7. Record force history for visualization
        if len(node.flow_history) > 50:  # Keep last 50 time steps
            node.flow_history.pop(0)
            node.drag_history.pop(0)
        node.flow_history.append(flow_velocity)
        node.drag_history.append(drag_force)
        
        # 8. Apply velocity damping to prevent unrealistic speeds
        max_speed = 2.0  # m/s maximum realistic underwater robot speed
        current_speed = np.linalg.norm(node.velocity)
        if current_speed > max_speed:
            node.velocity = node.velocity * (max_speed / current_speed)
    
    def get_effective_communication_range(self, node1: Node, node2: Node, base_range: float) -> float:
        """
        Calculate effective communication range considering underwater acoustics
        """
        if not self.enable_underwater_physics:
            return base_range
        
        distance = np.linalg.norm(node1.position - node2.position)
        acoustic_factor = self.underwater_env.get_acoustic_range_factor(distance)
        
        # Environmental factors (depth, density variations)
        depth_factor1 = self.underwater_env.get_environmental_layer(node1.position[1])['current_factor']
        depth_factor2 = self.underwater_env.get_environmental_layer(node2.position[1])['current_factor']
        depth_factor = (depth_factor1 + depth_factor2) / 2
        
        effective_range = base_range * acoustic_factor * depth_factor
        return max(effective_range, base_range * 0.3)  # Minimum 30% of base range
    
    def create_specific_graph(self):
        """Create the specific graph structure"""
        positions = {
            1: np.array([1.0, 2.0]),
            2: np.array([0.0, 1.0]),
            3: np.array([1.0, 0.0]),
            4: np.array([2.0, 1.0]),
            5: np.array([3.0, 1.0]),
            6: np.array([4.0, 1.0]),
            7: np.array([4.0, 0.0]),
            8: np.array([5.0, 1.0])
        }
        
        for node_id, pos in positions.items():
            self.nodes[node_id] = Node(node_id, pos)
        
        edges = [
            (1, 2), (1, 4),
            (2, 3), (2, 4),
            (3, 4),
            (4, 5),
            (5, 6), (5, 7),
            (6, 7), (6, 8),
        ]
        
        for node1, node2 in edges:
            self.nodes[node1].add_direct_neighbor(node2)
            self.nodes[node2].add_direct_neighbor(node1)
    
    def print_graph_structure(self):
        """Print the graph structure"""
        print("Graph edges:")
        for node_id in sorted(self.nodes.keys()):
            neighbors = sorted(list(self.nodes[node_id].direct_neighbors))
            print(f"  Node {node_id}: neighbors = {neighbors}")
    
    def run_n_step_algorithm(self, n_steps: int = 8):
        """Run the n-step distributed algorithm"""
        print(f"\nRunning {n_steps}-step distributed algorithm:")
        print("=" * 60)
        
        for step in range(1, n_steps + 1):
            print(f"\n--- Step k = {step} ---")
            
            # All nodes execute one step simultaneously
            for node in self.nodes.values():
                node.execute_one_step(self.nodes)
            
            self.k = step
            
            # Print state after this step
            print(f"State after step {step}:")
            self.print_all_states()
            
            # Check connectivity
            connectivity_status = self.check_connectivity_status()
            print(f"Connectivity status: {connectivity_status}")
        
        return step
    
    def print_all_states(self):
        """Print ξ and ω for all nodes"""
        for node_id in sorted(self.nodes.keys()):
            self.nodes[node_id].print_state()
        print()
    
    def check_connectivity_status(self):
        """Check connectivity from all nodes' perspectives"""
        connected_count = 0
        for node in self.nodes.values():
            if node.is_network_connected():
                connected_count += 1
        
        if connected_count == len(self.nodes):
            return "All nodes see network as CONNECTED"
        elif connected_count == 0:
            return "All nodes see network as DISCONNECTED"
        else:
            return f"{connected_count}/{len(self.nodes)} nodes see network as connected"
    
    def is_network_connected(self) -> bool:
        """Check if the network is connected using BFS from any node"""
        if not self.nodes:
            return True
            
        # Start BFS from first node
        start_node = next(iter(self.nodes.keys()))
        visited = set()
        queue = [start_node]
        visited.add(start_node)
        
        while queue:
            current = queue.pop(0)
            for neighbor in self.nodes[current].direct_neighbors:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        
        # Network is connected if all nodes are reachable
        return len(visited) == len(self.nodes)
    
    def compute_bilateral_edge_priorities(self, non_critical_edges: Set[Tuple[int, int]]) -> Dict[Tuple[int, int], float]:
        """
        Compute bilateral consensus priorities for non-critical edges
        
        Algorithm:
        1. Each endpoint computes local priority
        2. Final priority = average of both endpoints
        3. Higher priority = remove first
        """
        bilateral_priorities = {}
        
        for edge in sorted(non_critical_edges):
            node1_id, node2_id = edge
            node1 = self.nodes[node1_id]
            node2 = self.nodes[node2_id]
            
            # Each node computes its local priority for this edge (quiet mode)
            priority_from_node1 = node1.compute_edge_priority(node2_id, self.nodes, verbose=False)
            priority_from_node2 = node2.compute_edge_priority(node1_id, self.nodes, verbose=False)
            
            # Bilateral consensus: average of both perspectives
            consensus_priority = (priority_from_node1 + priority_from_node2) / 2.0
            
            bilateral_priorities[edge] = consensus_priority
        
        return bilateral_priorities
    
    def select_edge_for_removal(self, non_critical_edges: Set[Tuple[int, int]]) -> Tuple[int, int]:
        """
        Select the edge with highest priority for removal using bilateral consensus
        """
        if not non_critical_edges:
            return None
        
        # Compute bilateral priorities
        priorities = self.compute_bilateral_edge_priorities(non_critical_edges)
        
        # Sort edges by priority (highest first)
        sorted_edges = sorted(priorities.items(), key=lambda x: x[1], reverse=True)
        
        print(f"\n🏆 PRIORITY RANKING (Higher = Remove First)")
        print("=" * 60)
        for rank, (edge, priority) in enumerate(sorted_edges, 1):
            status = "👑 SELECTED" if rank == 1 else f"  #{rank}"
            print(f"{status}: Edge {edge} -> Priority: {priority:.4f}")
        
        # Select edge with highest priority
        selected_edge = sorted_edges[0][0]
        
        return selected_edge

    def print_neighbor_sets_CLEAN(self):
        """Print N(p)_i for all nodes - CLEAN VERSION"""
        print("\nNeighbor Sets N(p)_i:")
        print("=" * 40)
        
        for node_id in sorted(self.nodes.keys()):
            node = self.nodes[node_id]
            neighbor_sets = node.get_neighbor_sets()
            
            print(f"Node {node_id}:")
            if neighbor_sets:
                for p in sorted(neighbor_sets.keys()):
                    neighbors = sorted(list(neighbor_sets[p]))
                    print(f"  N({p})_{node_id} = {neighbors}")
            else:
                print(f"  No neighbor sets computed yet")
            print()
    
    def run_algorithm_a(self):
        """Run Algorithm A for ensuring connectivity"""
        print("\nRunning Algorithm A (Distributed Connection of Non-Connected Network):")
        print("=" * 70)
        
        max_iterations = 10
        iteration = 0
        
        while iteration < max_iterations:
            print(f"\n--- Algorithm A - Iteration {iteration + 1} ---")
            
            # Check if any node sees disconnected components
            nodes_needing_connection = []
            for node in self.nodes.values():
                if not node.is_network_connected():
                    nodes_needing_connection.append(node)
            
            if not nodes_needing_connection:
                print("✅ All nodes see the network as connected!")
                break
            
            print(f"Nodes seeing disconnected network: {[n.id for n in nodes_needing_connection]}")
            
            # Execute Algorithm A steps
            connections_made = []
            for node in nodes_needing_connection:
                # Find i* = max{j | ξi,j(k) = 1}
                i_star = node.find_highest_connected_index()
                
                # Check if this node is i*
                if node.id == i_star:
                    # Find j* = max{j | ξi,j(k) = 0}  
                    j_star = node.find_highest_unconnected_index()
                    
                    if j_star != -1:
                        print(f"Node {node.id}: Establishing link with Node {j_star}")
                        
                        # Add the edge
                        self.nodes[node.id].add_direct_neighbor(j_star)
                        self.nodes[j_star].add_direct_neighbor(node.id)
                        
                        # Update connectivity matrices
                        self.nodes[node.id].xi[j_star] = 1
                        self.nodes[j_star].xi[node.id] = 1
                        self.nodes[node.id].omega[j_star] = 1
                        self.nodes[j_star].omega[node.id] = 1
                        
                        connections_made.append((node.id, j_star))
            
            if connections_made:
                print(f"New connections established: {connections_made}")
                
                # Run a few steps of the distributed algorithm to propagate changes
                print("Propagating connectivity information...")
                for prop_step in range(3):
                    for node in self.nodes.values():
                        node.execute_one_step(self.nodes)
                
                # Print updated state
                print("Updated state:")
                self.print_all_states()
            
            iteration += 1
        
        if iteration >= max_iterations:
            print("⚠️ Algorithm A reached maximum iterations")
        
        return iteration
    
    def run_critical_edge_detection(self):
        """Run 2-step distributed critical edge detection algorithm"""
        print("\n" + "=" * 70)
        print("RUNNING CRITICAL EDGE DETECTION (2-STEP ALGORITHM)")
        print("=" * 70)
        
        # Step 1: Calculate delta measures for all nodes
        print("\nStep 1: Calculating Δ(il)_i measures...")
        print("-" * 50)
        
        for node_id in sorted(self.nodes.keys()):
            self.nodes[node_id].print_delta_measures(self.nodes)
        
        # Step 2: Identify critical edges
        print("\nStep 2: Identifying critical edges...")
        print("-" * 50)
        
        all_critical_edges = set()
        node_critical_edges = {}
        
        for node_id in sorted(self.nodes.keys()):
            node = self.nodes[node_id]
            critical_neighbors = node.find_critical_edges(self.nodes)
            node_critical_edges[node_id] = critical_neighbors
            
            print(f"Node {node_id} critical neighbors: {sorted(list(critical_neighbors))}")
            
            # Add to global set (ensure we don't double count)
            for neighbor in critical_neighbors:
                edge = tuple(sorted([node_id, neighbor]))
                all_critical_edges.add(edge)
        
        print(f"\nAll critical edges in the network: {sorted(list(all_critical_edges))}")
        return all_critical_edges, node_critical_edges
    
    def visualize_network(self, title: str = "Network Graph", critical_edges: Set[tuple] = None):
        """Visualize the current network with critical edges marked in red"""
        G = nx.Graph()
        
        # Add nodes
        for node_id, node in self.nodes.items():
            G.add_node(node_id, pos=node.position)
        
        # Add edges
        all_edges = []
        for node_id, node in self.nodes.items():
            for neighbor_id in node.direct_neighbors:
                if node_id < neighbor_id:  # Avoid duplicate edges
                    edge = (node_id, neighbor_id)
                    G.add_edge(node_id, neighbor_id)
                    all_edges.append(edge)
        
        # Get positions
        pos = {node_id: node.position for node_id, node in self.nodes.items()}
        
        # Create figure with two subplots
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        
        # Subplot 1: Complete graph (all black edges)
        ax1.set_title("Complete Network Graph", fontsize=14, fontweight='bold')
        
        # Draw all edges in black
        nx.draw_networkx_edges(G, pos, edgelist=all_edges, 
                             edge_color='black', width=2, alpha=0.7, ax=ax1)
        
        # Draw nodes with smaller size
        nx.draw_networkx_nodes(G, pos, node_color='lightblue', 
                              node_size=400, alpha=0.9, ax=ax1)  # Reduced from 800 to 400
        nx.draw_networkx_labels(G, pos, font_size=10, font_weight='bold', ax=ax1)  # Reduced font size
        
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        ax1.text(0.02, 0.98, f"Nodes: {len(G.nodes())}\nEdges: {len(G.edges())}", 
                transform=ax1.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # Subplot 2: Graph with critical edges highlighted
        ax2.set_title("Critical Edges (Bridges) in Red", fontsize=14, fontweight='bold')
        
        # Separate normal and critical edges
        normal_edges = []
        critical_edge_list = []
        
        if critical_edges:
            for edge in all_edges:
                if edge in critical_edges or tuple(reversed(edge)) in critical_edges:
                    critical_edge_list.append(edge)
                else:
                    normal_edges.append(edge)
        else:
            normal_edges = all_edges
        
        # Draw normal edges in black
        if normal_edges:
            nx.draw_networkx_edges(G, pos, edgelist=normal_edges, 
                                 edge_color='black', width=2, alpha=0.7, ax=ax2)
        
        # Draw critical edges in red with dashed lines
        if critical_edge_list:
            nx.draw_networkx_edges(G, pos, edgelist=critical_edge_list, 
                                 edge_color='red', width=3, alpha=0.9,
                                 style='--', ax=ax2)
        
        # Draw nodes with smaller size
        nx.draw_networkx_nodes(G, pos, node_color='lightblue', 
                              node_size=400, alpha=0.9, ax=ax2)  # Reduced from 800 to 400
        nx.draw_networkx_labels(G, pos, font_size=10, font_weight='bold', ax=ax2)  # Reduced font size
        
        # Add legend for subplot 2
        if critical_edge_list:
            from matplotlib.lines import Line2D
            legend_elements = [
                Line2D([0], [0], color='black', lw=2, label='Normal edges'),
                Line2D([0], [0], color='red', lw=3, linestyle='--', label='Critical edges (bridges)')
            ]
            ax2.legend(handles=legend_elements, loc='upper right')
        
        ax2.set_aspect('equal')
        ax2.grid(True, alpha=0.3)
        ax2.text(0.02, 0.98, f"Critical Edges: {len(critical_edge_list)}\n{sorted(critical_edge_list) if critical_edge_list else 'None'}", 
                transform=ax2.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.8))
        
        plt.tight_layout()
        plt.show()
        
        # Note: Critical edge removal animation disabled 
        # (Not relevant for iterative pruning since we never remove critical edges)
        
        return G
    
    # animate_edge_removal function removed - not needed for iterative pruning
    # (We never remove critical edges in our algorithm)
    
    def print_final_results(self):
        """Print final N and ω results"""
        print("\n" + "=" * 70)
        print(f"FINAL RESULTS: N(p) AND ω VALUES FOR ALL {len(self.nodes)} ROBOTS")
        print("=" * 70)
        
        # Get all node IDs
        node_ids = sorted(self.nodes.keys())
        
        # Print ω matrix
        print("\nFinal ω matrix (shortest path distances):")
        print("-" * 50)
        
        # Header
        print("From\\To", end="")
        for j in node_ids:
            print(f"\t{j}", end="")
        print()
        
        # Matrix rows
        for i in node_ids:
            print(f"{i}", end="")
            for j in node_ids:
                omega_val = self.nodes[i].omega.get(j, float('inf'))
                if omega_val == float('inf'):
                    print(f"\t∞", end="")
                else:
                    print(f"\t{omega_val:.0f}", end="")
            print()
        
        # Print neighbor sets
        self.print_neighbor_sets()
        
        # Print connectivity status
        print(f"Final connectivity status: {self.check_connectivity_status()}")
        
        # Run critical edge detection
        critical_edges, node_critical_edges = self.run_critical_edge_detection()
        
        # Show visualization with critical edges marked
        print("\nShowing network visualization with critical edges in red...")
        self.visualize_network("Final Network Structure with Critical Edges", critical_edges)
    
    def iterative_pruning_step_by_step(self):
        """
        Simple Iterative Pruning Algorithm:
        Step 1: Run distributed algorithm to find critical edges
        Step 2: Keep ALL critical edges, prune ONE non-critical edge
        Step 3: Re-run ENTIRE distributed algorithm (graph changed!)
        Step 4: Repeat until no non-critical edges remain
        """
        print("\n" + "=" * 70)
        print("STEP-BY-STEP ITERATIVE EDGE PRUNING")
        print("Algorithm: Keep critical edges, remove non-critical edges one by one")
        print("=" * 70)
        
        iteration = 0
        pruned_edges = []
        
        while True:
            iteration += 1
            print(f"\n--- ITERATION {iteration} ---")
            
            # Step 1: Re-run COMPLETE distributed algorithm (graph may have changed)
            print("Step 1: Running complete distributed algorithm...")
            
            # Re-initialize all nodes for the current graph
            num_nodes = len(self.nodes)
            for node in self.nodes.values():
                node.initialize_algorithm(num_nodes)
            
            # Run the distributed algorithm
            self.run_n_step_algorithm(n_steps=min(num_nodes + 2, 10))
            
            # Step 2: Find critical edges in current graph
            print("Step 2: Detecting critical edges in current graph...")
            critical_edges, _ = self.run_critical_edge_detection()
            critical_edges_set = set(tuple(sorted(edge)) for edge in critical_edges)
            
            # Get all current edges
            all_current_edges = set()
            for node_id, node in self.nodes.items():
                for neighbor_id in node.direct_neighbors:
                    if node_id < neighbor_id:
                        edge = (node_id, neighbor_id)
                        all_current_edges.add(edge)
            
            # Step 3: Identify non-critical edges (these can be safely removed)
            non_critical_edges = all_current_edges - critical_edges_set
            
            print(f"Current edges: {len(all_current_edges)} -> {sorted(list(all_current_edges))}")
            print(f"Critical edges: {len(critical_edges_set)} -> {sorted(list(critical_edges_set))}")
            print(f"Non-critical edges: {len(non_critical_edges)} -> {sorted(list(non_critical_edges))}")
            
            # Step 4: Check termination condition
            if len(non_critical_edges) == 0:
                print("\n🎯 PRUNING COMPLETE!")
                print("All remaining edges are critical - graph has minimal connectivity")
                break
            
            # Step 5: Remove exactly ONE non-critical edge using sophisticated selection
            print("\n🔍 SELECTING EDGE FOR REMOVAL")
            print("Using Triangle Participation + Local Redundancy Algorithm")
            print("-" * 55)
            
            edge_to_prune = self.select_edge_for_removal(non_critical_edges)
            print(f"\n🎯 Removing edge: {edge_to_prune}")
            
            # Remove the edge from the graph
            node1, node2 = edge_to_prune
            self.nodes[node1].direct_neighbors.discard(node2)
            self.nodes[node2].direct_neighbors.discard(node1)
            
            pruned_edges.append(edge_to_prune)
            
            # Step 6: Verify connectivity is maintained
            if not self.check_connectivity_status():
                print("❌ ERROR: Graph became disconnected!")
                print("This should never happen if we only remove non-critical edges!")
                break
            
            print(f"✅ Non-critical edge {edge_to_prune} successfully removed")
            print(f"Graph remains connected with {len(all_current_edges) - 1} edges")
            print("⚠️  Graph structure changed - will re-run distributed algorithm next iteration")
            
            # Continue to next iteration (animation step removed)
            
            # Safety check
            if iteration > 50:
                print("⚠️ Maximum iterations reached")
                break
        
        # Final summary
        print(f"\n" + "="*70)
        print("ITERATIVE PRUNING SUMMARY")
        print("="*70)
        print(f"Total iterations: {iteration}")
        print(f"Non-critical edges removed: {len(pruned_edges)}")
        print(f"Removed edges: {sorted(pruned_edges)}")
        
        # Final graph analysis
        final_edges = 0
        for node in self.nodes.values():
            final_edges += len(node.direct_neighbors)
        final_edges //= 2
        
        num_nodes = len(self.nodes)
        min_possible = num_nodes - 1  # spanning tree minimum
        
        print(f"\nFinal graph properties:")
        print(f"  Nodes: {num_nodes}")
        print(f"  Final edges: {final_edges}")
        print(f"  Theoretical minimum (spanning tree): {min_possible}")
        print(f"  Extra edges: {final_edges - min_possible}")
        
        if final_edges == min_possible:
            print("🎯 Perfect! Graph is now a spanning tree (absolute minimum connectivity)")
        elif final_edges > min_possible:
            print(f"📊 Graph has {final_edges - min_possible} extra edges (some redundancy remains)")
        
        # Final verification: all remaining edges should be critical
        print("\nFinal verification: Running critical edge detection on pruned graph...")
        final_critical_edges, _ = self.run_critical_edge_detection()
        
        current_edges = set()
        for node_id, node in self.nodes.items():
            for neighbor_id in node.direct_neighbors:
                if node_id < neighbor_id:
                    current_edges.add((node_id, neighbor_id))
        
        final_critical_set = set(tuple(sorted(edge)) for edge in final_critical_edges)
        
        if current_edges == final_critical_set:
            print("✅ VERIFIED: All remaining edges are critical!")
        else:
            print("❌ WARNING: Some remaining edges are not critical")
            remaining_non_critical = current_edges - final_critical_set
            print(f"Non-critical edges still present: {sorted(list(remaining_non_critical))}")
        
        # Show final animation
        original_edges_count = final_edges + len(pruned_edges)
        self.animate_final_pruning_result(original_edges_count, final_edges, pruned_edges)
        
        return pruned_edges

    def iterative_pruning_simultaneous(self):
        """
        NEW SIMULTANEOUS BATCH PRUNING ALGORITHM WITH AUTOMATIC ANIMATION:
        - Shows continuous batch edge removal in a single window
        - No manual interaction needed - fully automatic
        - Real-time verification of the algorithm
        """
        print("\n" + "=" * 70)
        print("SIMULTANEOUS BATCH EDGE PRUNING - FAST MODE WITH ANIMATION")
        print("Algorithm: Keep critical edges, remove multiple non-critical edges per iteration")
        print("*** Watch automatic animation - no manual interaction needed!")
        print("=" * 70)
        
        # Initialize animation data
        self.batch_animation_data = {
            'iteration': 0,
            'pruned_edges': [],
            'current_state': 'detecting',  # 'detecting', 'selecting', 'removing', 'complete'
            'critical_edges': set(),
            'all_current_edges': set(),
            'non_critical_edges': set(),
            'edges_to_remove': [],
            'status_message': 'Starting batch pruning in 10x10 workspace (nodes start clustered)...',
            'max_iterations': 20,
            'original_edge_count': 0,
            # BROWNIAN MOTION PARAMETERS - 10x10 WORKSPACE
            'brownian_step_size': 0.15,  # Increased step size for more dramatic movement
            'boundary_limits': {'x_min': -5, 'x_max': 5, 'y_min': -5, 'y_max': 5},  # 10x10 workspace
            'frame_count': 0,  # Track animation frames for smooth movement
            'algorithm_step_interval': 30,  # Run algorithm logic every 30 frames (3 seconds)
            'last_algorithm_frame': 0,  # Track when we last ran algorithm logic
            'node_trails': {node_id: [] for node_id in self.nodes.keys()},  # Store recent positions for trails
            'trail_length': 25,  # Longer trails to show movement better
            # Enhanced movement parameters
            'node_velocities': {node_id: (0.0, 0.0) for node_id in self.nodes.keys()},  # Add momentum
            'velocity_persistence': 0.7,  # How much velocity carries over (0-1)
            'max_velocity': 0.2  # Maximum velocity per frame
        }
        
        # Calculate initial edge count
        for node_id, node in self.nodes.items():
            for neighbor_id in node.direct_neighbors:
                if node_id < neighbor_id:
                    self.batch_animation_data['all_current_edges'].add((node_id, neighbor_id))
        
        self.batch_animation_data['original_edge_count'] = len(self.batch_animation_data['all_current_edges'])
        
        print(f"*** Starting automatic animation with {self.batch_animation_data['original_edge_count']} initial edges...")
        
        # Set up matplotlib animation with subplots for network + bar chart
        import matplotlib.animation as animation
        
        self.fig = plt.figure(figsize=(18, 10))  # Wider figure for side-by-side layout
        
        # Create subplots: main network plot (left) and statistics (right)
        self.ax_network = plt.subplot(1, 2, 1)  # Network visualization
        self.ax_stats = plt.subplot(2, 2, 2)    # Bar chart statistics  
        self.ax_details = plt.subplot(2, 2, 4)  # Edge removal details
        
        self.ax_network.set_aspect('equal')
        
        # Get positions for consistent layout
        self.pos = {node_id: node.position for node_id, node in self.nodes.items()}
        
        # Create the animation with fast updates for smooth Brownian motion
        self.batch_anim = animation.FuncAnimation(
            self.fig, 
            self.update_batch_simulation, 
            interval=100,   # Update every 100ms for smooth motion (10 FPS)
            repeat=False,   # Don't repeat when done
            blit=False,     # Don't use blitting for simplicity
            cache_frame_data=False  # Disable caching to avoid warning
        )
        
        plt.tight_layout()
        plt.show()
        
        return self.batch_animation_data['pruned_edges']

    def update_node_positions_brownian(self, data):
        """
        Update node positions with ENHANCED Brownian motion (random walk with momentum)
        This simulates MORE REALISTIC robot movement while the algorithm is running!
        """
        step_size = data['brownian_step_size']
        bounds = data['boundary_limits']
        persistence = data['velocity_persistence']
        max_vel = data['max_velocity']
        
        for node_id, node in self.nodes.items():
            # Get current velocity (with momentum from previous frame)
            current_vel = data['node_velocities'][node_id]
            
            # Add random acceleration (more dramatic changes)
            random_accel_x = random.uniform(-step_size, step_size)
            random_accel_y = random.uniform(-step_size, step_size)
            
            # Update velocity with persistence (momentum) + random acceleration
            new_vel_x = current_vel[0] * persistence + random_accel_x
            new_vel_y = current_vel[1] * persistence + random_accel_y
            
            # Limit maximum velocity to prevent crazy speeds
            vel_magnitude = (new_vel_x**2 + new_vel_y**2)**0.5
            if vel_magnitude > max_vel:
                new_vel_x = (new_vel_x / vel_magnitude) * max_vel
                new_vel_y = (new_vel_y / vel_magnitude) * max_vel
            
            # Calculate new position based on velocity
            new_x = node.position[0] + new_vel_x
            new_y = node.position[1] + new_vel_y
            
            # Boundary handling with velocity reflection (more realistic bouncing)
            if new_x < bounds['x_min'] or new_x > bounds['x_max']:
                new_vel_x = -new_vel_x * 0.8  # Bounce with some energy loss
                new_x = node.position[0] + new_vel_x  # Recalculate position
                new_x = max(bounds['x_min'], min(bounds['x_max'], new_x))  # Clamp to bounds
            
            if new_y < bounds['y_min'] or new_y > bounds['y_max']:
                new_vel_y = -new_vel_y * 0.8  # Bounce with some energy loss
                new_y = node.position[1] + new_vel_y  # Recalculate position
                new_y = max(bounds['y_min'], min(bounds['y_max'], new_y))  # Clamp to bounds
            
            # Store updated velocity
            data['node_velocities'][node_id] = (new_vel_x, new_vel_y)
            
            # Update node position
            node.position = (new_x, new_y)
            
            # Store position in trail (for visual effect)
            if node_id not in data['node_trails']:
                data['node_trails'][node_id] = []
            
            data['node_trails'][node_id].append((new_x, new_y))
            
            # Keep only recent trail positions
            if len(data['node_trails'][node_id]) > data['trail_length']:
                data['node_trails'][node_id].pop(0)
            
        # Update the positions dictionary for NetworkX visualization
        self.pos = {node_id: node.position for node_id, node in self.nodes.items()}

    def update_batch_simulation(self, frame):
        """
        Update function for batch pruning animation - called automatically
        WITH BROWNIAN MOTION FOR DYNAMIC NODES! 🎯
        """
        data = self.batch_animation_data
        data['frame_count'] += 1
        
        # 🎯 BROWNIAN MOTION UPDATE - Move all nodes randomly each frame
        self.update_node_positions_brownian(data)
        
        # Only run algorithm logic every N frames (for smooth motion visualization)
        should_run_algorithm = (data['frame_count'] - data['last_algorithm_frame']) >= data['algorithm_step_interval']
        
        # Update status for continuous motion visualization
        if not should_run_algorithm and data['current_state'] != 'complete':
            frames_until_next = data['algorithm_step_interval'] - (data['frame_count'] - data['last_algorithm_frame'])
            data['status_message'] = f'*** Robots spreading out in 10x10 workspace! (Algorithm step in {frames_until_next} frames)'
        
        # State machine for batch pruning algorithm
        if data['current_state'] == 'detecting' and should_run_algorithm:
            # Phase 1: Run critical edge detection
            data['status_message'] = f'Iteration {data["iteration"] + 1}: Detecting critical edges (nodes moving)...'
            data['last_algorithm_frame'] = data['frame_count']
            
            # Run distributed algorithm to identify critical edges
            critical_edges, _ = self.run_critical_edge_detection()
            data['critical_edges'] = set(critical_edges)
            
            # Get all current edges
            data['all_current_edges'] = set()
            for node_id, node in self.nodes.items():
                for neighbor_id in node.direct_neighbors:
                    if node_id < neighbor_id:
                        edge = (node_id, neighbor_id)
                        data['all_current_edges'].add(edge)
            
            # Identify non-critical edges
            data['non_critical_edges'] = data['all_current_edges'] - data['critical_edges']
            
            if len(data['non_critical_edges']) == 0:
                data['current_state'] = 'complete'
                data['status_message'] = 'PRUNING COMPLETE! Only critical edges remain.'
            else:
                data['current_state'] = 'selecting'
        
        elif data['current_state'] == 'selecting' and should_run_algorithm:
            # Phase 2: Select edges for batch removal
            data['status_message'] = f'Selecting batch of edges for removal (nodes moving)...'
            data['last_algorithm_frame'] = data['frame_count']
            
            # Calculate optimal batch size
            optimal_batch_size = max(1, min(6, len(data['non_critical_edges']) // 3))
            
            # Select edges for batch removal
            data['edges_to_remove'] = self.select_edges_for_batch_removal(
                data['non_critical_edges'], max_batch_size=optimal_batch_size)
            
            if not data['edges_to_remove']:
                data['current_state'] = 'complete'
            else:
                data['current_state'] = 'removing'
                data['status_message'] = f'Removing {len(data["edges_to_remove"])} edges simultaneously...'
        
        elif data['current_state'] == 'removing' and should_run_algorithm:
            # Phase 3: Remove the selected batch of edges
            data['last_algorithm_frame'] = data['frame_count']
            
            for edge_to_remove in data['edges_to_remove']:
                node1, node2 = edge_to_remove
                self.nodes[node1].direct_neighbors.discard(node2)
                self.nodes[node2].direct_neighbors.discard(node1)
                data['pruned_edges'].append(edge_to_remove)
            
            data['iteration'] += 1
            data['status_message'] = f'Removed {len(data["edges_to_remove"])} edges. Nodes continue moving...'
            
            # Check if we should continue
            if data['iteration'] >= data['max_iterations']:
                data['current_state'] = 'complete'
            else:
                data['current_state'] = 'detecting'
                data['edges_to_remove'] = []  # Clear for next iteration
        
        # Draw the network based on current state
        self.draw_batch_network_state(data)
        
        # Stop animation when complete
        if data['current_state'] == 'complete':
            # Stop the animation if it exists
            if hasattr(self, 'batch_anim') and self.batch_anim is not None:
                self.batch_anim.event_source.stop()
            print(f"\n[SUCCESS] Batch pruning animation complete!")
            print(f"   Original edges: {data['original_edge_count']}")
            print(f"   Final edges: {len(data['all_current_edges'])}")
            print(f"   Edges removed: {len(data['pruned_edges'])}")
            print(f"   Iterations: {data['iteration']}")
            
            # CRITICAL: Final connectivity validation
            self.perform_final_connectivity_validation()

    def draw_batch_network_state(self, data):
        """
        Draw the current state of the network during batch pruning animation
        WITH COOL BAR CHART AND STATISTICS PANELS!
        """
        # Clear all subplots
        self.ax_network.clear()
        self.ax_stats.clear() 
        self.ax_details.clear()
        
        # Create NetworkX graph for visualization
        G = nx.Graph()
        G.add_nodes_from(self.nodes.keys())
        
        # Add edges to the graph
        for edge in data['all_current_edges']:
            G.add_edge(edge[0], edge[1])
        
        # ==== MAIN NETWORK VISUALIZATION (LEFT SIDE) ====
        # Set title based on current state
        title = f"Simultaneous Batch Pruning - {data['status_message']}\n"
        title += f"Iteration {data['iteration']} | "
        title += f"Total: {len(data['all_current_edges'])} | "
        title += f"Critical: {len(data['critical_edges'])} | "
        title += f"Non-Critical: {len(data['non_critical_edges'])}"
        
        self.ax_network.set_title(title, fontsize=12, fontweight='bold')
        
        # 🏠 Draw 10x10 workspace boundaries (show the operating area)
        bounds = data['boundary_limits']
        
        # Draw workspace boundary rectangle
        boundary_x = [bounds['x_min'], bounds['x_max'], bounds['x_max'], bounds['x_min'], bounds['x_min']]
        boundary_y = [bounds['y_min'], bounds['y_min'], bounds['y_max'], bounds['y_max'], bounds['y_min']]
        self.ax_network.plot(boundary_x, boundary_y, color='black', linewidth=2, linestyle='--', alpha=0.5)
        
        # Draw initial 1-unit radius circle (where robots started)
        circle = plt.Circle((0, 0), 0.8, fill=False, color='lightgray', linestyle=':', linewidth=1, alpha=0.7)
        self.ax_network.add_patch(circle)
        
        # Add workspace labels
        self.ax_network.text(bounds['x_min'] - 0.5, bounds['y_max'], '10x10 Workspace', 
                            fontsize=9, alpha=0.7, rotation=90, verticalalignment='top')
        self.ax_network.text(0, -0.6, 'Initial\n1-unit radius', 
                            fontsize=8, alpha=0.6, horizontalalignment='center')
        
        # Set axis limits to show full workspace with some padding
        self.ax_network.set_xlim(bounds['x_min'] - 1, bounds['x_max'] + 1)
        self.ax_network.set_ylim(bounds['y_min'] - 1, bounds['y_max'] + 1)
        
        # 🎯 Draw Enhanced Brownian motion trails (showing dramatic node movement paths)
        trail_colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
        
        for i, (node_id, trail) in enumerate(data['node_trails'].items()):
            if len(trail) > 1:
                trail_x = [pos[0] for pos in trail]
                trail_y = [pos[1] for pos in trail]
                
                # Use different colors for different nodes
                trail_color = trail_colors[i % len(trail_colors)]
                
                # Create gradient effect for trail (recent positions more visible)
                for j in range(len(trail) - 1):
                    alpha = (j + 1) / len(trail) * 0.7  # More visible trails (0 to 0.7)
                    line_width = ((j + 1) / len(trail)) * 2 + 0.5  # Thicker recent segments
                    self.ax_network.plot([trail_x[j], trail_x[j+1]], [trail_y[j], trail_y[j+1]], 
                                       color=trail_color, alpha=alpha, linewidth=line_width)
        
        # Draw nodes (current positions)
        nx.draw_networkx_nodes(G, self.pos, ax=self.ax_network, node_color='lightblue', 
                              node_size=600, alpha=0.9)
        
        # Draw different types of edges with different colors
        if data['current_state'] != 'complete':
            # Critical edges (red) - never removed
            critical_edge_list = [(e[0], e[1]) for e in data['critical_edges']]
            if critical_edge_list:
                nx.draw_networkx_edges(G, self.pos, ax=self.ax_network, edgelist=critical_edge_list, 
                                      edge_color='red', width=3, alpha=0.8)
            
            # Non-critical edges (gray) - candidates for removal  
            non_critical_list = [(e[0], e[1]) for e in data['non_critical_edges'] 
                               if e not in data['edges_to_remove']]
            if non_critical_list:
                nx.draw_networkx_edges(G, self.pos, ax=self.ax_network, edgelist=non_critical_list, 
                                      edge_color='gray', width=1, alpha=0.6)
            
            # Edges being removed (orange/yellow) - highlighted
            if data['edges_to_remove']:
                edges_removing_list = [(e[0], e[1]) for e in data['edges_to_remove']]
                nx.draw_networkx_edges(G, self.pos, ax=self.ax_network, edgelist=edges_removing_list, 
                                      edge_color='orange', width=5, alpha=1.0)
        else:
            # Final state - all remaining edges are critical (red)
            all_edges_list = [(e[0], e[1]) for e in data['all_current_edges']]
            if all_edges_list:
                nx.draw_networkx_edges(G, self.pos, ax=self.ax_network, edgelist=all_edges_list, 
                                      edge_color='red', width=3, alpha=0.8)
        
        # Draw labels
        nx.draw_networkx_labels(G, self.pos, ax=self.ax_network, font_size=10, font_weight='bold')
        
        # Add legend
        from matplotlib.lines import Line2D
        legend_elements = []
        
        if data['current_state'] != 'complete':
            legend_elements = [
                Line2D([0], [0], color='red', lw=3, label=f'Critical Edges ({len(data["critical_edges"])})'),
                Line2D([0], [0], color='gray', lw=1, label=f'Non-Critical ({len(data["non_critical_edges"]) - len(data["edges_to_remove"])})'),
            ]
            if data['edges_to_remove']:
                legend_elements.append(
                    Line2D([0], [0], color='orange', lw=5, label=f'Being Removed ({len(data["edges_to_remove"])})')
                )
        else:
            legend_elements = [
                Line2D([0], [0], color='red', lw=3, label=f'Final Critical Edges ({len(data["all_current_edges"])})')
            ]
        
        self.ax_network.legend(handles=legend_elements, loc='upper right')
        self.ax_network.axis('off')
        
        # ==== BAR CHART STATISTICS (TOP RIGHT) ====
        self.ax_stats.set_title("*** Edge Statistics", fontweight='bold', fontsize=11)
        
        # Create bar chart data
        if data['current_state'] != 'complete':
            categories = ['Critical', 'Non-Critical\nRemaining', 'Being\nRemoved']
            counts = [
                len(data['critical_edges']), 
                len(data['non_critical_edges']) - len(data['edges_to_remove']),
                len(data['edges_to_remove'])
            ]
            colors = ['red', 'gray', 'orange']
        else:
            # Final state
            categories = ['Critical\n(Final)', 'Total\nRemoved']
            counts = [len(data['all_current_edges']), len(data['pruned_edges'])]
            colors = ['red', 'lightcoral']
        
        bars = self.ax_stats.bar(categories, counts, color=colors, alpha=0.7, edgecolor='black', linewidth=1)
        self.ax_stats.set_ylabel('Number of Edges', fontsize=10)
        
        # Add count labels on bars
        for bar, count in zip(bars, counts):
            if count > 0:
                self.ax_stats.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1, 
                        str(count), ha='center', va='bottom', fontweight='bold', fontsize=10)
        
        # Set y-axis limit with some padding (avoid identical limits)
        max_count = max(counts) if counts else 1
        if max_count == 0:
            self.ax_stats.set_ylim(0, 1)  # Default range when no data
        else:
            self.ax_stats.set_ylim(0, max_count * 1.3)
        
        # Add grid for easier reading
        self.ax_stats.grid(axis='y', alpha=0.3, linestyle='--')
        
        # ==== DETAILED PROGRESS PANEL (BOTTOM RIGHT) ====
        self.ax_details.set_title("*** Progress Details", fontweight='bold', fontsize=11)
        
        # Create detailed progress text
        progress_text = ""
        
        if data['current_state'] != 'complete':
            progress_text += f"🔄 Iteration: {data['iteration']}/{data['max_iterations']}\n"
            progress_text += f"📈 Original Edges: {data['original_edge_count']}\n"
            progress_text += f"📉 Edges Removed: {len(data['pruned_edges'])}\n"
            progress_text += f"*** Current Edges: {len(data['all_current_edges'])}\n"
            
            if data['edges_to_remove']:
                progress_text += f"\n>>> Removing This Iteration:\n"
                for i, edge in enumerate(data['edges_to_remove']):
                    progress_text += f"   • Edge {edge}\n"
                    if i >= 6:  # Limit to prevent overcrowding
                        progress_text += f"   ... and {len(data['edges_to_remove']) - 6} more\n"
                        break
            
            # Progress percentage
            if data['original_edge_count'] > 0:
                progress_pct = (len(data['pruned_edges']) / data['original_edge_count']) * 100
                progress_text += f"\n📈 Progress: {progress_pct:.1f}% pruned"
        else:
            # Final summary
            progress_text += f"*** PRUNING COMPLETE! ***\n\n"
            progress_text += f"*** Final Statistics:\n"
            progress_text += f"   Original: {data['original_edge_count']} edges\n"
            progress_text += f"   Final: {len(data['all_current_edges'])} edges\n"
            progress_text += f"   Removed: {len(data['pruned_edges'])} edges\n"
            progress_text += f"   Iterations: {data['iteration']}\n"
            
            if data['original_edge_count'] > 0:
                reduction_pct = (len(data['pruned_edges']) / data['original_edge_count']) * 100
                progress_text += f"   Reduction: {reduction_pct:.1f}%\n"
            
            progress_text += f"\n>>> Network connectivity preserved!"
        
        # Display the progress text
        self.ax_details.text(0.05, 0.95, progress_text, transform=self.ax_details.transAxes, 
                    fontsize=9, verticalalignment='top', fontfamily='monospace',
                    bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
        
        self.ax_details.axis('off')
        
        # Adjust layout for better spacing
        plt.tight_layout()

    def visualize_simultaneous_pruning_result(self, original_edges_count, final_edges_count, pruned_edges):
        """
        Visualize the result of simultaneous batch pruning
        """
        print(f"\n🎨 VISUALIZING SIMULTANEOUS PRUNING RESULTS")
        print("=" * 60)
        
        # Create visualization of the final minimal connectivity graph
        plt.figure(figsize=(12, 8))
        
        # Create NetworkX graph for final state
        G = nx.Graph()
        G.add_nodes_from(self.nodes.keys())
        
        # Add remaining edges (these should all be critical)
        remaining_edges = []
        for node_id, node in self.nodes.items():
            for neighbor_id in node.direct_neighbors:
                if node_id < neighbor_id:
                    edge = (node_id, neighbor_id)
                    remaining_edges.append(edge)
                    G.add_edge(node_id, neighbor_id)
        
        # Create layout
        pos = nx.spring_layout(G, seed=42, k=3, iterations=50)
        
        # Draw the network
        plt.subplot(1, 1, 1)
        plt.title(f"Simultaneous Batch Pruning Result\n"
                 f"Critical Edges Only: {final_edges_count} edges (Removed {len(pruned_edges)} edges)", 
                 fontsize=14, fontweight='bold')
        
        # Draw nodes
        nx.draw_networkx_nodes(G, pos, node_color='lightblue', 
                              node_size=800, alpha=0.9)
        
        # Draw remaining edges (all should be critical - in red)
        nx.draw_networkx_edges(G, pos, edgelist=remaining_edges, 
                              edge_color='red', width=3, alpha=0.8)
        
        # Draw labels
        nx.draw_networkx_labels(G, pos, font_size=12, font_weight='bold')
        
        # Add legend and statistics
        stats_text = (f"Original Edges: {original_edges_count}\n"
                     f"Final Edges: {final_edges_count}\n"
                     f"Edges Removed: {len(pruned_edges)}\n"
                     f"Reduction: {(len(pruned_edges)/original_edges_count)*100:.1f}%\n"
                     f"Method: Simultaneous Batch Pruning")
        
        plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes, 
                fontsize=10, verticalalignment='top', 
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # Add legend for edge colors
        from matplotlib.lines import Line2D
        legend_elements = [Line2D([0], [0], color='red', lw=3, label='Critical Edges (Remaining)')]
        plt.legend(handles=legend_elements, loc='upper right')
        
        plt.axis('off')
        plt.tight_layout()
        plt.show()
        
        print(f"✅ Visualization complete!")
        print(f"   All {final_edges_count} remaining edges are critical for connectivity")
        print(f"   Successfully removed {len(pruned_edges)} non-critical edges using batch pruning")

    def visualize_batch_iteration(self, iteration, edges_removed, critical_edges, all_edges, non_critical_edges):
        """
        Visualize each iteration of batch pruning showing which edges were removed
        """
        print(f"\n🎨 Visualizing Iteration {iteration} - Batch Edge Removal")
        
        # Create NetworkX graph
        G = nx.Graph()
        G.add_nodes_from(self.nodes.keys())
        
        # Add all edges to the graph
        for edge in all_edges:
            G.add_edge(edge[0], edge[1])
        
        # Create layout
        pos = nx.spring_layout(G, seed=42, k=2, iterations=30)
        
        # Create figure
        plt.figure(figsize=(14, 10))
        
        # Main plot
        plt.subplot(2, 2, (1, 3))
        plt.title(f"Iteration {iteration}: Batch Removal of {len(edges_removed)} Edges\n"
                 f"Total Edges: {len(all_edges)} | Critical: {len(critical_edges)} | "
                 f"Non-Critical: {len(non_critical_edges)}", 
                 fontsize=12, fontweight='bold')
        
        # Draw nodes
        nx.draw_networkx_nodes(G, pos, node_color='lightblue', 
                              node_size=600, alpha=0.8)
        
        # Draw different types of edges
        # Critical edges (red)
        critical_edge_list = [(e[0], e[1]) for e in critical_edges if e in all_edges]
        if critical_edge_list:
            nx.draw_networkx_edges(G, pos, edgelist=critical_edge_list, 
                                  edge_color='red', width=2, alpha=0.8)
        
        # Non-critical edges remaining (gray)
        remaining_non_critical = [(e[0], e[1]) for e in non_critical_edges if e not in edges_removed]
        if remaining_non_critical:
            nx.draw_networkx_edges(G, pos, edgelist=remaining_non_critical, 
                                  edge_color='gray', width=1, alpha=0.6)
        
        # Edges being removed this iteration (yellow/orange - highlighted)
        edges_to_remove_list = [(e[0], e[1]) for e in edges_removed]
        if edges_to_remove_list:
            nx.draw_networkx_edges(G, pos, edgelist=edges_to_remove_list, 
                                  edge_color='orange', width=4, alpha=1.0)
        
        # Draw labels
        nx.draw_networkx_labels(G, pos, font_size=10, font_weight='bold')
        
        # Add legend
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], color='red', lw=2, label=f'Critical Edges ({len(critical_edges)})'),
            Line2D([0], [0], color='gray', lw=1, label=f'Non-Critical Remaining ({len(remaining_non_critical)})'),
            Line2D([0], [0], color='orange', lw=4, label=f'Being Removed ({len(edges_removed)})')
        ]
        plt.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(0, 1))
        
        plt.axis('off')
        
        # Side panel - Statistics
        plt.subplot(2, 2, 2)
        plt.title("Iteration Statistics", fontweight='bold')
        
        # Create bar chart of edge types
        categories = ['Critical', 'Non-Critical\nRemaining', 'Being\nRemoved']
        counts = [len(critical_edges), len(remaining_non_critical), len(edges_removed)]
        colors = ['red', 'gray', 'orange']
        
        bars = plt.bar(categories, counts, color=colors, alpha=0.7)
        plt.ylabel('Number of Edges')
        
        # Add count labels on bars
        for bar, count in zip(bars, counts):
            if count > 0:
                plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1, 
                        str(count), ha='center', va='bottom', fontweight='bold')
        
        plt.ylim(0, max(counts) * 1.2 if counts else 1)
        
        # Side panel - Removed edges details
        plt.subplot(2, 2, 4)
        plt.title(f"Edges Removed This Iteration", fontweight='bold')
        
        if edges_removed:
            edge_text = "Removed Edges:\n"
            for i, edge in enumerate(edges_removed):
                edge_text += f"• Edge {edge}\n"
                if i >= 8:  # Limit display to prevent crowding
                    edge_text += f"... and {len(edges_removed) - 8} more"
                    break
        else:
            edge_text = "No edges removed"
        
        plt.text(0.1, 0.9, edge_text, transform=plt.gca().transAxes, 
                fontsize=10, verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
        
        plt.axis('off')
        
        plt.tight_layout()
        plt.show()
        
        # Pause to let user observe
        input(f"\nPress Enter to continue to next iteration...")
        plt.close('all')

    def select_edges_for_batch_removal(self, non_critical_edges: set, max_batch_size: int = 5) -> list:
        """
        Select multiple edges for simultaneous removal using conflict-free batch selection
        
        Strategy:
        1. Rank all edges by priority (highest first)
        2. Select non-conflicting edges (no shared nodes)
        3. CRITICAL: Validate that removal won't disconnect any nodes
        4. Return batch of edges for simultaneous removal
        """
        if not non_critical_edges:
            return []
        
        # Compute bilateral priorities (quietly)
        priorities = self.compute_bilateral_edge_priorities(non_critical_edges)
        
        # Sort edges by priority (highest first) 
        sorted_edges = sorted(priorities.items(), key=lambda x: x[1], reverse=True)
        
        # Batch selection with conflict avoidance AND connectivity validation
        selected_edges = []
        used_nodes = set()
        
        print(f"\n*** BATCH EDGE SELECTION WITH CONNECTIVITY VALIDATION (Max Batch: {max_batch_size})")
        print("=" * 75)
        
        for edge, priority in sorted_edges:
            node1, node2 = edge
            
            # Check for conflicts (shared nodes with already selected edges)
            if node1 in used_nodes or node2 in used_nodes:
                print(f"   WARNING: Skipping Edge {edge} -> Priority: {priority:.4f} (Node conflict)")
                continue
            
            # CRITICAL CONNECTIVITY CHECK: Will removing this edge (plus already selected edges) disconnect any nodes?
            test_batch = selected_edges + [edge]
            if not self.validate_batch_connectivity(test_batch):
                print(f"   [SKIP] Skipping Edge {edge} -> Priority: {priority:.4f} (Would disconnect nodes!)")
                continue
            
            # No conflict and connectivity preserved - add to batch
            selected_edges.append(edge)
            used_nodes.add(node1)
            used_nodes.add(node2)
            
            status = "👑 SELECTED" if len(selected_edges) == 1 else f"  #{len(selected_edges)}"
            print(f"{status}: Edge {edge} -> Priority: {priority:.4f} (Connectivity OK)")
            
            # Stop when we reach max batch size
            if len(selected_edges) >= max_batch_size:
                break
        
        # Show remaining edges that couldn't be selected due to conflicts
        remaining_edges = len(sorted_edges) - len(selected_edges)
        if remaining_edges > 0:
            print(f"   [INFO] {remaining_edges} edges skipped due to conflicts or connectivity constraints")
        
        print(f"\n*** BATCH SUMMARY: {len(selected_edges)} edges selected for simultaneous removal")
        print("[SUCCESS] All selected edges verified to preserve network connectivity")
        
        return selected_edges

    def validate_batch_connectivity(self, edges_to_remove: List[Tuple[int, int]]) -> bool:
        """
        CRITICAL CONNECTIVITY VALIDATION:
        Check if removing a batch of edges would disconnect any nodes from the network.
        
        Returns True if connectivity is preserved, False if any node would be isolated.
        """
        if not edges_to_remove:
            return True
        
        # Create a temporary copy of the network to test connectivity
        temp_neighbors = {}
        for node_id, node in self.nodes.items():
            temp_neighbors[node_id] = set(node.direct_neighbors)
        
        # Temporarily remove the batch of edges
        for edge in edges_to_remove:
            node1, node2 = edge
            if node2 in temp_neighbors[node1]:
                temp_neighbors[node1].discard(node2)
            if node1 in temp_neighbors[node2]:
                temp_neighbors[node2].discard(node1)
        
        # Check 1: No node should have zero neighbors (isolated)
        for node_id, neighbors in temp_neighbors.items():
            if len(neighbors) == 0:
                # This node would be completely isolated!
                return False
        
        # Check 2: Network must remain connected (all nodes reachable from node 0)
        if not self.is_network_connected_with_temp_topology(temp_neighbors):
            return False
        
        return True
    
    def is_network_connected_with_temp_topology(self, temp_neighbors: Dict[int, Set[int]]) -> bool:
        """
        Check if the network is connected using a temporary topology
        Uses BFS to verify all nodes are reachable from the first node
        """
        if not temp_neighbors:
            return True
        
        # Start BFS from the first node
        start_node = next(iter(temp_neighbors.keys()))
        visited = set()
        queue = [start_node]
        visited.add(start_node)
        
        while queue:
            current_node = queue.pop(0)
            
            # Visit all neighbors of current node
            for neighbor in temp_neighbors[current_node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        
        # All nodes should be reachable
        return len(visited) == len(temp_neighbors)

    def perform_final_connectivity_validation(self):
        """
        CRITICAL FINAL VALIDATION:
        Verify that no nodes are isolated and the network is fully connected
        """
        print(f"\n🔍 FINAL CONNECTIVITY VALIDATION")
        print("=" * 50)
        
        # Check 1: No isolated nodes
        isolated_nodes = []
        for node_id, node in self.nodes.items():
            if len(node.direct_neighbors) == 0:
                isolated_nodes.append(node_id)
        
        if isolated_nodes:
            print(f"❌ CRITICAL ERROR: {len(isolated_nodes)} nodes are ISOLATED!")
            print(f"   Isolated nodes: {isolated_nodes}")
            print("   This should never happen with proper critical edge detection!")
            return False
        else:
            print(f"✅ No isolated nodes - all {len(self.nodes)} nodes have connections")
        
        # Check 2: Network connectivity using BFS from node 0
        all_node_ids = set(self.nodes.keys())
        start_node = next(iter(all_node_ids))
        
        visited = set()
        queue = [start_node]
        visited.add(start_node)
        
        while queue:
            current_node = queue.pop(0)
            
            # Visit all neighbors
            for neighbor_id in self.nodes[current_node].direct_neighbors:
                if neighbor_id not in visited:
                    visited.add(neighbor_id)
                    queue.append(neighbor_id)
        
        # Check if all nodes are reachable
        unreachable_nodes = all_node_ids - visited
        
        if unreachable_nodes:
            print(f"❌ CONNECTIVITY ERROR: {len(unreachable_nodes)} nodes are UNREACHABLE!")
            print(f"   Unreachable nodes: {sorted(unreachable_nodes)}")
            print(f"   Reachable nodes: {sorted(visited)}")
            print("   The network is fragmented into disconnected components!")
            return False
        else:
            print(f"✅ Full connectivity verified - all {len(self.nodes)} nodes are reachable")
        
        # Check 3: Degree analysis
        min_degree = min(len(node.direct_neighbors) for node in self.nodes.values())
        max_degree = max(len(node.direct_neighbors) for node in self.nodes.values())
        avg_degree = sum(len(node.direct_neighbors) for node in self.nodes.values()) / len(self.nodes)
        
        print(f"\n📊 NETWORK STATISTICS:")
        print(f"   Minimum degree: {min_degree}")
        print(f"   Maximum degree: {max_degree}")  
        print(f"   Average degree: {avg_degree:.2f}")
        
        if min_degree == 0:
            print("   ⚠️  WARNING: Minimum degree is 0 (isolated nodes exist)")
        elif min_degree == 1:
            print("   ⚠️  WARNING: Some nodes have only 1 connection (vulnerable)")
        else:
            print("   ✅ All nodes have multiple connections (robust)")
        
        print(f"\n🎯 FINAL RESULT: Network connectivity {'PRESERVED' if not unreachable_nodes and not isolated_nodes else 'VIOLATED'}")
        
        return len(isolated_nodes) == 0 and len(unreachable_nodes) == 0

    def compute_bilateral_edge_priorities(self, non_critical_edges: set) -> dict:
        """
        Compute bilateral consensus priorities for non-critical edges
        (Quiet mode - no debug output for batch processing)
        """
        bilateral_priorities = {}
        
        for edge in sorted(non_critical_edges):
            node1_id, node2_id = edge
            node1 = self.nodes[node1_id]
            node2 = self.nodes[node2_id]
            
            # Each node computes its local priority for this edge (quiet mode)
            priority_from_node1 = node1.compute_edge_priority(node2_id, self.nodes, verbose=False)
            priority_from_node2 = node2.compute_edge_priority(node1_id, self.nodes, verbose=False)
            
            # Bilateral consensus: average of both perspectives
            consensus_priority = (priority_from_node1 + priority_from_node2) / 2.0
            
            bilateral_priorities[edge] = consensus_priority
        
        return bilateral_priorities
    
    def real_time_pruning_simulation(self):
        """
        Real-time simulation in a single window showing iterative pruning
        """
        import matplotlib.animation as animation
        
        print("\n" + "=" * 70)
        print("REAL-TIME ITERATIVE PRUNING SIMULATION")
        print("Single window real-time animation - no closing/opening windows")
        print("=" * 70)
        
        # Store initial state
        self.simulation_data = {
            'iteration': 0,
            'pruned_edges': [],
            'current_state': 'detecting',  # 'detecting', 'pruning', 'complete'
            'status_message': 'Starting simulation...',
            'edge_to_remove': None,
            'critical_edges': set(),
            'non_critical_edges': set(),
            'all_edges': set()
        }
        
        # Set up the figure and axis
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.ax.set_aspect('equal')
        
        # Get positions for consistent layout
        self.pos = {node_id: node.position for node_id, node in self.nodes.items()}
        
        # Create the animation
        self.anim = animation.FuncAnimation(
            self.fig, 
            self.update_simulation, 
            interval=1000,  # Update every 1 second
            repeat=False,   # Don't repeat when done
            blit=False      # Don't use blitting for simplicity
        )
        
        plt.tight_layout()
        plt.show()
        
        return self.simulation_data['pruned_edges']
    
    def update_simulation(self, frame):
        """Update function for the real-time animation"""
        data = self.simulation_data
        
        # Clear the axis
        self.ax.clear()
        
        if data['current_state'] == 'detecting':
            # Phase 1: Detect critical edges
            data['iteration'] += 1
            
            # Re-run distributed algorithm
            num_nodes = len(self.nodes)
            for node in self.nodes.values():
                node.initialize_algorithm(num_nodes)
            
            # Run distributed algorithm (simplified for animation)
            for _ in range(min(num_nodes + 2, 8)):
                for node in self.nodes.values():
                    node.execute_one_step(self.nodes)
            
            # Detect critical edges
            critical_edges, _ = self.run_critical_edge_detection()
            data['critical_edges'] = set(tuple(sorted(edge)) for edge in critical_edges)
            
            # Get all current edges
            data['all_edges'] = set()
            for node_id, node in self.nodes.items():
                for neighbor_id in node.direct_neighbors:
                    if node_id < neighbor_id:
                        edge = (node_id, neighbor_id)
                        data['all_edges'].add(edge)
            
            # Identify non-critical edges
            data['non_critical_edges'] = data['all_edges'] - data['critical_edges']
            
            if len(data['non_critical_edges']) == 0:
                data['current_state'] = 'complete'
                data['status_message'] = 'PRUNING COMPLETE! Only critical edges remain.'
            else:
                # Select edge to remove using our sophisticated algorithm
                edge_list = list(data['non_critical_edges'])
                if len(edge_list) == 1:
                    data['edge_to_remove'] = edge_list[0]
                else:
                    # Use bilateral consensus for multiple edges
                    data['edge_to_remove'] = self.select_edge_for_removal(data['non_critical_edges'])
                    
                data['current_state'] = 'pruning'
                data['status_message'] = f'Removing non-critical edge: {data["edge_to_remove"]}'
        
        elif data['current_state'] == 'pruning':
            # Phase 2: Remove the selected edge
            edge_to_remove = data['edge_to_remove']
            node1, node2 = edge_to_remove
            
            # Remove the edge
            self.nodes[node1].direct_neighbors.discard(node2)
            self.nodes[node2].direct_neighbors.discard(node1)
            
            data['pruned_edges'].append(edge_to_remove)
            data['status_message'] = f'Removed edge {edge_to_remove}. Re-detecting critical edges...'
            data['current_state'] = 'detecting'
        
        # Visualize current state
        self.draw_current_state()
        
        # Stop animation if complete
        if data['current_state'] == 'complete':
            self.anim.event_source.stop()
            print(f"\n🎯 Simulation complete! Removed {len(data['pruned_edges'])} edges")
            print(f"Pruned edges: {data['pruned_edges']}")
    
    def draw_current_state(self):
        """Draw the current state of the graph"""
        data = self.simulation_data
        
        # Create NetworkX graph for current edges
        G = nx.Graph()
        for node_id in self.nodes.keys():
            G.add_node(node_id)
        
        for edge in data['all_edges']:
            G.add_edge(edge[0], edge[1])
        
        # Set title based on current state
        if data['current_state'] == 'detecting':
            title = f"Iteration {data['iteration']}: DETECTING Critical Edges"
            title_color = 'blue'
        elif data['current_state'] == 'pruning':
            title = f"Iteration {data['iteration']}: REMOVING Non-Critical Edge"
            title_color = 'orange'
        else:  # complete
            title = f"SIMULATION COMPLETE - Minimal Graph Achieved"
            title_color = 'green'
        
        self.ax.set_title(title, fontsize=14, fontweight='bold', color=title_color)
        
        # Draw edges based on type
        if data['current_state'] != 'complete':
            # Draw non-critical edges in gray
            if data['non_critical_edges']:
                non_critical_list = list(data['non_critical_edges'])
                nx.draw_networkx_edges(G, self.pos, edgelist=non_critical_list,
                                     edge_color='gray', width=2, alpha=0.6, ax=self.ax)
            
            # Draw critical edges in red (dashed)
            if data['critical_edges']:
                critical_list = list(data['critical_edges'])
                nx.draw_networkx_edges(G, self.pos, edgelist=critical_list,
                                     edge_color='red', width=3, alpha=0.9,
                                     style='--', ax=self.ax)
            
            # Highlight edge to remove (if in pruning phase)
            if data['current_state'] == 'pruning' and data['edge_to_remove']:
                nx.draw_networkx_edges(G, self.pos, edgelist=[data['edge_to_remove']],
                                     edge_color='yellow', width=4, alpha=1.0, ax=self.ax)
        else:
            # Final state: all edges are critical
            if data['all_edges']:
                all_edges_list = list(data['all_edges'])
                nx.draw_networkx_edges(G, self.pos, edgelist=all_edges_list,
                                     edge_color='red', width=3, alpha=0.9,
                                     style='--', ax=self.ax)
        
        # Draw nodes
        node_color = 'lightgreen' if data['current_state'] == 'complete' else 'lightblue'
        nx.draw_networkx_nodes(G, self.pos, node_color=node_color,
                              node_size=400, alpha=0.9, ax=self.ax)
        nx.draw_networkx_labels(G, self.pos, font_size=10, font_weight='bold', ax=self.ax)
        
        # Add legend
        from matplotlib.lines import Line2D
        legend_elements = []
        
        if data['current_state'] != 'complete':
            legend_elements = [
                Line2D([0], [0], color='red', lw=3, linestyle='--', label='Critical edges (KEEP)'),
                Line2D([0], [0], color='gray', lw=2, label='Non-critical edges'),
            ]
            if data['current_state'] == 'pruning':
                legend_elements.append(
                    Line2D([0], [0], color='yellow', lw=4, label='Edge being removed')
                )
        else:
            legend_elements = [
                Line2D([0], [0], color='red', lw=3, linestyle='--', label='Final critical edges only')
            ]
        
        self.ax.legend(handles=legend_elements, loc='upper right', fontsize=10)
        self.ax.grid(True, alpha=0.3)
        
        # Status information
        status_text = f"Iteration: {data['iteration']}\n"
        status_text += f"Total edges: {len(data['all_edges'])}\n"
        status_text += f"Critical edges: {len(data['critical_edges'])}\n"
        status_text += f"Non-critical edges: {len(data['non_critical_edges'])}\n"
        status_text += f"Edges removed: {len(data['pruned_edges'])}\n\n"
        status_text += f"Status: {data['status_message']}"
        
        self.ax.text(0.02, 0.98, status_text,
                    transform=self.ax.transAxes, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9),
                    fontsize=9)
    
    def animate_final_pruning_result(self, original_edges_count, final_edges_count, pruned_edges):
        """Show final comparison animation"""
        print("\n🎬 Creating final comparison animation...")
        
        # This would show the original vs final graph
        final_critical_edges, _ = self.run_critical_edge_detection()
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        
        # Get positions
        pos = {node_id: node.position for node_id, node in self.nodes.items()}
        
        # LEFT: Show what we removed (conceptual - we can't recreate original)
        ax1.set_title(f"PRUNING SUMMARY\nRemoved {len(pruned_edges)} non-critical edges", 
                     fontsize=14, fontweight='bold')
        ax1.text(0.5, 0.5, f"Original edges: {original_edges_count}\n\nRemoved edges:\n{pruned_edges}\n\nFinal edges: {final_edges_count}", 
                ha='center', va='center', transform=ax1.transAxes,
                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8),
                fontsize=12)
        ax1.set_xlim(0, 1)
        ax1.set_ylim(0, 1)
        ax1.axis('off')
        
        # RIGHT: Show final minimal graph
        ax2.set_title("FINAL MINIMAL GRAPH\n(Only critical edges remain)", 
                     fontsize=14, fontweight='bold')
        
        # Create final graph
        G_final = nx.Graph()
        for node_id in self.nodes.keys():
            G_final.add_node(node_id)
        
        final_edges = []
        for node_id, node in self.nodes.items():
            for neighbor_id in node.direct_neighbors:
                if node_id < neighbor_id:
                    edge = (node_id, neighbor_id)
                    G_final.add_edge(node_id, neighbor_id)
                    final_edges.append(edge)
        
        # All remaining edges should be critical (draw in red)
        if final_edges:
            nx.draw_networkx_edges(G_final, pos, edgelist=final_edges, 
                                 edge_color='red', width=3, alpha=0.9,
                                 style='--', ax=ax2)
        
        # Draw nodes
        nx.draw_networkx_nodes(G_final, pos, node_color='lightcoral', 
                              node_size=400, alpha=0.9, ax=ax2)
        nx.draw_networkx_labels(G_final, pos, font_size=10, font_weight='bold', ax=ax2)
        
        # Legend
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], color='red', lw=3, linestyle='--', label='Critical edges (minimal connectivity)')
        ]
        ax2.legend(handles=legend_elements, loc='upper right')
        ax2.set_aspect('equal')
        ax2.grid(True, alpha=0.3)
        
        # Final stats
        num_nodes = len(self.nodes)
        spanning_tree_min = num_nodes - 1
        redundancy = len(final_edges) - spanning_tree_min
        
        ax2.text(0.02, 0.98, f"Nodes: {num_nodes}\nEdges: {len(final_edges)}\nSpanning tree min: {spanning_tree_min}\nRedundancy: {redundancy}", 
                transform=ax2.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.8))
        
        plt.tight_layout()
        plt.show()
        
        return final_edges
    
    def goal_seeking_simulation(self, goal_x, goal_y):
        """
        Goal-seeking simulation with connectivity preservation
        Closest robot moves toward goal, others maintain connectivity
        """
        import matplotlib.animation as animation
        
        print("\n" + "=" * 70)
        print("GOAL-SEEKING SIMULATION WITH CONNECTIVITY PRESERVATION")
        print(f"Goal location: ({goal_x}, {goal_y})")
        print("Closest robot moves toward goal while preserving network connectivity")
        print("=" * 70)
        
        # Set goal location
        self.goal_location = np.array([goal_x, goal_y])
        
        # Initialize simulation parameters
        self.goal_reached = False
        self.goal_tolerance = 0.3  # How close to consider goal reached
        self.max_movement_per_frame = 0.08  # Robot movement speed
        
        # Initialize critical edge tracking
        self.current_critical_edges = []
        self.current_non_critical_edges = []
        self.pruned_edges = []
        
        # Run initial critical edge detection
        print("🔍 Running initial critical edge detection...")
        initial_critical_edges, _ = self.run_critical_edge_detection()
        self.cached_critical_edges = initial_critical_edges
        print(f"✅ Found {len(initial_critical_edges)} critical edges: {sorted(list(initial_critical_edges))}")
        
        # Run initial edge optimization to identify proximity-based connections
        print("\n🔄 Running initial edge optimization to strengthen connectivity...")
        initial_changes = self.run_decentralized_edge_optimization()
        if initial_changes['added']:
            print(f"🔗 Added {len(initial_changes['added'])} proximity-based edges: {initial_changes['added']}")
        
        # Create figure with subplots
        fig = plt.figure(figsize=(18, 10))
        
        # Create grid layout: main network plot (left), statistics (top right), progress (bottom right)
        gs = fig.add_gridspec(2, 3, width_ratios=[2, 1, 1], height_ratios=[1, 1])
        
        # Main network plot
        ax_network = fig.add_subplot(gs[:, 0])
        ax_network.set_title("Goal-Seeking Robot Network with Connectivity Preservation", 
                            fontsize=14, fontweight='bold')
        
        # Statistics plot (top right)
        ax_stats = fig.add_subplot(gs[0, 1:])
        ax_stats.set_title("Network Statistics", fontsize=12, fontweight='bold')
        
        # Progress details (bottom right)
        ax_details = fig.add_subplot(gs[1, 1:])
        ax_details.set_title("Goal-Seeking Progress", fontsize=12, fontweight='bold')
        
        # Initialize data for animation
        self.animation_step = 0
        self.closest_robot_id = None
        self.goal_distance_history = []
        self.connectivity_history = []
        
        def animate_goal_seeking(frame):
            """Animation function for goal-seeking simulation"""
            for ax in [ax_network, ax_stats, ax_details]:
                ax.clear()
                
            # Find closest robot to goal
            min_distance = float('inf')
            closest_id = None
            for node_id, node in self.nodes.items():
                distance = np.linalg.norm(node.position - self.goal_location)
                if distance < min_distance:
                    min_distance = distance
                    closest_id = node_id
            
            self.closest_robot_id = closest_id
            
            # DEBUG: Print animation info every 30 frames to reduce spam
            if frame % 30 == 0:
                print(f"\n🐛 DEBUG Frame {frame}:")
                print(f"   Goal: {self.goal_location}")
                print(f"   Closest robot: {closest_id}, distance: {min_distance:.3f}")
                print(f"   Goal tolerance: {self.goal_tolerance}")
                print(f"   Goal reached: {self.goal_reached}")
                for node_id, node in self.nodes.items():
                    print(f"   Robot {node_id} at {node.position}")
            
            # Check if goal is reached
            if min_distance <= self.goal_tolerance and not self.goal_reached:
                self.goal_reached = True
                print(f"\n🎯 GOAL REACHED! Robot {closest_id} reached the goal!")
            
            # Update robot positions if goal not reached
            if not self.goal_reached:
                if frame % 30 == 0:
                    print(f"   -> Calling update_positions_goal_seeking()...")
                self.update_positions_goal_seeking()
                
                # Run decentralized edge optimization periodically (every 10 frames)
                if frame % 10 == 0:
                    self.run_decentralized_edge_optimization()
            else:
                if frame % 30 == 0:
                    print(f"   -> Goal reached, no position update")
            
            # Record statistics
            self.goal_distance_history.append(min_distance)
            total_edges = sum(len(node.direct_neighbors) for node in self.nodes.values()) // 2
            self.connectivity_history.append(total_edges)
            
            # Draw main network
            environment_type = "Underwater" if self.enable_underwater_physics else "Surface"
            ax_network.set_title(f"{environment_type} Goal-Seeking Robot Network with Connectivity Preservation", 
                                fontsize=14, fontweight='bold')
            
            # Set workspace limits with padding
            ax_network.set_xlim(-1, 11)
            ax_network.set_ylim(-1, 11)
            ax_network.grid(True, alpha=0.3)
            
            # Draw workspace boundary
            boundary_color = 'navy' if self.enable_underwater_physics else 'black'
            boundary_style = '-' if self.enable_underwater_physics else '--'
            boundary = plt.Rectangle((0, 0), 10, 10, fill=False, linewidth=3, 
                                   edgecolor=boundary_color, linestyle=boundary_style)
            ax_network.add_patch(boundary)
            
            # === UNDERWATER ENVIRONMENT VISUALIZATION ===
            if self.enable_underwater_physics and hasattr(self, 'underwater_env'):
                # Add underwater background color
                underwater_bg = plt.Rectangle((0, 0), 10, 10, fill=True, 
                                            facecolor='lightcyan', alpha=0.2, zorder=0)
                ax_network.add_patch(underwater_bg)
                
                # Visualize flow field every few frames to avoid performance issues
                if frame % 5 == 0:  # Update flow field every 5 frames
                    self.underwater_env.visualize_flow_field(ax_network, resolution=15)
                
                # Add depth indicators (horizontal lines for depth layers)
                depth_colors = ['lightblue', 'blue', 'darkblue']
                depth_levels = [3, 7, 10]
                depth_labels = ['Surface', 'Mid-depth', 'Deep']
                for i, (depth, color, label) in enumerate(zip(depth_levels, depth_colors, depth_labels)):
                    if depth <= 10:
                        ax_network.axhline(y=depth, color=color, alpha=0.3, linestyle=':', linewidth=1)
                        if frame % 30 == 0:  # Show labels occasionally
                            ax_network.text(0.2, depth-0.2, label, fontsize=8, color=color, alpha=0.7)
            
            # Draw goal location
            goal_color = 'gold' if not self.enable_underwater_physics else 'yellow'
            ax_network.scatter(goal_x, goal_y, c=goal_color, s=200, marker='*', 
                              edgecolors='orange', linewidth=2, zorder=10, label='Goal')
            ax_network.scatter(goal_x, goal_y, c='orange', s=50, marker='o', 
                              alpha=0.3, zorder=9)  # Goal radius indicator
            
            # Draw edges with critical/non-critical color coding
            max_comm_range = 1.2  # Same as in update function
            
            # Get current edge classifications
            current_critical = getattr(self, 'current_critical_edges', [])
            current_non_critical = getattr(self, 'current_non_critical_edges', [])
            pruned = getattr(self, 'pruned_edges', [])
            
            for node_id, node in self.nodes.items():
                for neighbor_id in node.direct_neighbors:
                    if node_id < neighbor_id:  # Draw each edge only once
                        neighbor_pos = self.nodes[neighbor_id].position
                        distance = np.linalg.norm(node.position - neighbor_pos)
                        edge = (node_id, neighbor_id)
                        
                        # Determine edge type and color
                        if edge in current_critical:
                            # Critical edges - must be preserved (RED)
                            color = 'red'
                            width = 2.5
                            alpha = 0.9
                            style = '-'
                        elif edge in current_non_critical:
                            # Non-critical edges - can be pruned (BLUE)
                            color = 'blue' 
                            width = 1.5
                            alpha = 0.6
                            style = '--'
                        else:
                            # Default (shouldn't happen but just in case)
                            color = 'gray'
                            width = 1.0
                            alpha = 0.4
                            style = ':'
                        
                        # Draw the edge
                        ax_network.plot([node.position[0], neighbor_pos[0]], 
                                       [node.position[1], neighbor_pos[1]], 
                                       color=color, linewidth=width, alpha=alpha, linestyle=style)
            
            # Draw communication range circles for visualization (optional - only for closest robot)
            if closest_id is not None:
                closest_pos = self.nodes[closest_id].position
                comm_circle = plt.Circle((closest_pos[0], closest_pos[1]), max_comm_range, 
                                       fill=False, linestyle=':', alpha=0.3, color='gray')
                ax_network.add_patch(comm_circle)
            
            # Draw nodes with underwater enhancements
            for node_id, node in self.nodes.items():
                is_active = node_id == closest_id
                
                # Base robot visualization
                if self.enable_underwater_physics:
                    # Underwater robot colors (more blue-tinted)
                    color = 'darkred' if is_active else 'steelblue'
                    edge_color = 'white' if is_active else 'navy'
                    
                    # Show velocity trails for underwater movement - direction only with small arrows
                    if hasattr(node, 'velocity') and np.linalg.norm(node.velocity) > 0.01:
                        # FIXED: Force arrows to point in positive directions (rightward/upward)
                        # Create display velocity with positive components to ensure arrows point inward
                        display_velocity = np.array([abs(node.velocity[0]), abs(node.velocity[1])])
                        
                        # Normalize velocity to unit vector and use fixed small length
                        vel_magnitude = np.linalg.norm(display_velocity)
                        vel_direction = display_velocity / vel_magnitude
                        arrow_length = 0.3  # Fixed small arrow length
                        ax_network.arrow(node.position[0], node.position[1], 
                                       vel_direction[0] * arrow_length, vel_direction[1] * arrow_length,
                                       head_width=0.08, head_length=0.06, fc='orange', ec='orange', 
                                       alpha=0.7, linewidth=1.2)
                    
                    # Show drag force trails - direction only with small arrows
                    if hasattr(node, 'drag_history') and len(node.drag_history) > 0:
                        recent_drag = node.drag_history[-1]
                        if np.linalg.norm(recent_drag) > 0.01:
                            # Normalize drag to unit vector and use fixed small length
                            drag_magnitude = np.linalg.norm(recent_drag)
                            drag_direction = recent_drag / drag_magnitude
                            arrow_length = 0.25  # Fixed small arrow length
                            ax_network.arrow(node.position[0], node.position[1],
                                           drag_direction[0] * arrow_length, drag_direction[1] * arrow_length,
                                           head_width=0.06, head_length=0.04, fc='red', ec='red',
                                           alpha=0.5, linewidth=1, linestyle='--')
                    
                    # Depth-based size variation (deeper = slightly smaller due to perspective)
                    depth_factor = min(node.position[1] / 10, 1.0)  # Normalize depth
                    size_base = 300 if is_active else 200
                    size = size_base * (0.8 + 0.2 * (1 - depth_factor))
                    
                else:
                    # Surface/air environment
                    color = 'red' if is_active else 'lightblue'
                    edge_color = 'black'
                    size = 300 if is_active else 200
                
                # Draw the robot
                ax_network.scatter(node.position[0], node.position[1], 
                                  c=color, s=size, alpha=0.8, edgecolors=edge_color, linewidth=2)
                
                # Robot ID label
                label_color = 'white' if self.enable_underwater_physics else 'black'
                ax_network.annotate(str(node_id), (node.position[0], node.position[1]), 
                                   xytext=(0, 0), textcoords='offset points', 
                                   ha='center', va='center', fontweight='bold', color=label_color)
                
                # Show underwater-specific information occasionally
                if self.enable_underwater_physics and frame % 20 == 0 and is_active:
                    if hasattr(node, 'velocity'):
                        speed = np.linalg.norm(node.velocity)
                        ax_network.annotate(f'{speed:.2f} m/s', 
                                          (node.position[0], node.position[1] + 0.3), 
                                          ha='center', va='center', fontsize=8, 
                                          color='navy', alpha=0.7)
            
            # Add updated legend with environment-specific elements
            from matplotlib.lines import Line2D
            
            if self.enable_underwater_physics:
                legend_elements = [
                    Line2D([0], [0], color='yellow', marker='*', markersize=8, label='Goal', linestyle='None'),
                    Line2D([0], [0], color='red', linewidth=2.5, label='Critical Edges', linestyle='-'),
                    Line2D([0], [0], color='blue', linewidth=1.5, label='Non-Critical Edges', linestyle='--'),
                    Line2D([0], [0], marker='o', color='darkred', markersize=8, label='Active AUV', linestyle='None'),
                    Line2D([0], [0], marker='o', color='steelblue', markersize=6, label='Other AUVs', linestyle='None'),
                    Line2D([0], [0], color='cyan', marker='>', markersize=6, label='Flow Field', linestyle='None'),
                    Line2D([0], [0], color='orange', marker='>', markersize=6, label='Velocity', linestyle='None'),
                    Line2D([0], [0], color='red', marker='>', markersize=4, label='Drag Force', linestyle='--')
                ]
            else:
                legend_elements = [
                    Line2D([0], [0], color='gold', marker='*', markersize=8, label='Goal', linestyle='None'),
                    Line2D([0], [0], color='red', linewidth=2.5, label='Critical Edges (Must Keep)', linestyle='-'),
                    Line2D([0], [0], color='blue', linewidth=1.5, label='Non-Critical Edges (Can Prune)', linestyle='--'),
                    Line2D([0], [0], marker='o', color='red', markersize=8, label='Active Robot', linestyle='None'),
                    Line2D([0], [0], marker='o', color='lightblue', markersize=6, label='Other Robots', linestyle='None')
                ]
            ax_network.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(0.98, 0.98))
            
            # Draw statistics
            ax_stats.set_title("Network Statistics", fontsize=12, fontweight='bold')
            if len(self.goal_distance_history) > 1:
                ax_stats.plot(self.goal_distance_history, 'r-', linewidth=2, label='Distance to Goal')
                ax_stats.plot(self.connectivity_history, 'b-', linewidth=2, label='Edge Count')
                ax_stats.set_xlabel('Animation Steps')
                ax_stats.set_ylabel('Values')
                ax_stats.legend()
                ax_stats.grid(True, alpha=0.3)
            else:
                ax_stats.text(0.5, 0.5, 'Collecting data...', ha='center', va='center', 
                             transform=ax_stats.transAxes, fontsize=12)
            
            # Draw progress details
            ax_details.set_title("Goal-Seeking Progress", fontsize=12, fontweight='bold')
            
            # Calculate edge statistics
            current_critical = getattr(self, 'current_critical_edges', [])
            current_non_critical = getattr(self, 'current_non_critical_edges', [])
            pruned = getattr(self, 'pruned_edges', [])
            
            details_text = f"Animation Step: {self.animation_step}\n"
            details_text += f"Active Robot: {closest_id}\n"
            details_text += f"Distance to Goal: {min_distance:.2f}\n"
            details_text += f"Goal Reached: {'YES' if self.goal_reached else 'NO'}\n\n"
            details_text += f"EDGE ANALYSIS:\n"
            details_text += f"Critical Edges: {len(current_critical)}\n"
            details_text += f"Non-Critical Edges: {len(current_non_critical)}\n"
            details_text += f"Pruned Edges: {len(pruned)}\n"
            details_text += f"Total Active: {len(current_critical) + len(current_non_critical)}\n\n"
            details_text += f"Network Connected: {'YES' if self.is_network_connected() else 'NO'}\n"
            details_text += f"Comm. Range: 2.8 units\n"
            details_text += f"Goal: ({goal_x:.1f}, {goal_y:.1f})"
            
            ax_details.text(0.05, 0.95, details_text, transform=ax_details.transAxes, 
                           verticalalignment='top', fontsize=10,
                           bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
            ax_details.axis('off')
            
            # Status message at bottom
            status_msg = "🎯 GOAL REACHED! Simulation complete!" if self.goal_reached else "🚀 Moving toward goal..."
            ax_details.text(0.5, 0.02, status_msg, transform=ax_details.transAxes, 
                           ha='center', va='bottom', fontsize=11, fontweight='bold',
                           color='green' if self.goal_reached else 'blue')
            
            self.animation_step += 1
            
            # Stop animation when goal is reached and after a few more frames
            if self.goal_reached and self.animation_step > len(self.goal_distance_history) + 20:
                return []
                
            return []
        
        # Create and run animation
        print("🎬 Starting goal-seeking animation...")
        print("Close the window to stop the simulation")
        
        anim = animation.FuncAnimation(fig, animate_goal_seeking, frames=1000, 
                                     interval=100, blit=False, repeat=False)
        
        plt.tight_layout()
        plt.show()
        
        # Final results
        print(f"\n{'=' * 70}")
        print("GOAL-SEEKING SIMULATION COMPLETE")
        if self.goal_distance_history:
            print(f"Final distance to goal: {self.goal_distance_history[-1]:.2f}")
        else:
            print("Final distance to goal: N/A (simulation ended early)")
        print(f"Goal reached: {'YES' if self.goal_reached else 'NO'}")
        print(f"Total animation steps: {self.animation_step}")
        print(f"Network remained connected: {'YES' if self.is_network_connected() else 'NO'}")
        print(f"{'=' * 70}")
        
        return self.goal_reached
    
    def calculate_constrained_position(self, robot_position, goal_position, neighbor_position, max_range):
        """
        Calculate optimal position for robot to move toward goal while staying within max_range of neighbor
        Returns the target position on the communication circle around the neighbor
        """
        # Direction toward goal
        direction_to_goal = goal_position - robot_position
        if np.linalg.norm(direction_to_goal) == 0:
            return robot_position
        
        direction_to_goal_norm = direction_to_goal / np.linalg.norm(direction_to_goal)
        
        # Circle parameters (around the neighbor)
        circle_center = neighbor_position
        radius = max_range * 0.95  # Stay slightly inside max range for safety
        
        # Find point on circle closest to the goal direction
        # Project goal direction onto circle around neighbor
        center_to_robot = robot_position - circle_center
        
        # If robot is already inside the circle, project goal direction onto circle
        robot_to_goal_from_center = direction_to_goal_norm
        
        # Calculate target position on circle boundary that's closest to goal direction
        target_position = circle_center + robot_to_goal_from_center * radius
        
        return target_position
    
    def compute_clf_goal_seeking(self, robot_position, goal_position):
        """
        Compute Control Lyapunov Function for goal-seeking
        CLF: V(x) = ||x - x_goal||^2
        CLF derivative: dV/dx = 2(x - x_goal)
        """
        error = robot_position - goal_position
        V = np.dot(error, error)  # ||x - x_goal||^2
        dV_dx = 2 * error  # Gradient of V
        return V, dV_dx
    
    def compute_cbf_connectivity(self, robot_position, neighbor_positions, max_range):
        """
        Compute Control Barrier Function for both connectivity and safety constraints
        Connectivity CBF: h_conn(x) = R_max^2 - ||x - x_neighbor||^2 (maintain connectivity)
        Safety CBF: h_safe(x) = ||x - x_neighbor||^2 - R_min^2 (maintain minimum distance)
        Returns constraints for maintaining both connectivity and safety
        """
        cbf_constraints = []
        min_safe_distance = 1.0  # INCREASED minimum safety distance between robots for better visibility
        
        for neighbor_pos in neighbor_positions:
            # Distance to neighbor
            diff = robot_position - neighbor_pos
            distance_sq = np.dot(diff, diff)
            distance = np.sqrt(distance_sq)
            
            # === CONNECTIVITY CBF (Maximum Distance Constraint) ===
            # CBF: h_conn(x) = R_max^2 - ||x - x_neighbor||^2
            # We want h_conn(x) >= 0 to stay within communication range
            h_conn = max_range**2 - distance_sq
            dh_conn_dx = -2 * diff  # Gradient of h_conn
            
            cbf_constraints.append({
                'h': h_conn,
                'dh_dx': dh_conn_dx,
                'neighbor_pos': neighbor_pos,
                'distance': distance,
                'max_range': max_range,
                'type': 'connectivity'
            })
            
            # === SAFETY CBF (Minimum Distance Constraint) ===
            # CBF: h_safe(x) = ||x - x_neighbor||^2 - R_min^2
            # We want h_safe(x) >= 0 to stay away from neighbor
            h_safe = distance_sq - min_safe_distance**2
            dh_safe_dx = 2 * diff  # Gradient of h_safe
            
            cbf_constraints.append({
                'h': h_safe,
                'dh_dx': dh_safe_dx,
                'neighbor_pos': neighbor_pos,
                'distance': distance,
                'min_safe_distance': min_safe_distance,
                'type': 'safety'
            })
        
        return cbf_constraints
    
    def solve_clf_cbf_qp(self, robot_position, goal_position, neighbor_positions, max_range):
        """
        Solve CLF-CBF Quadratic Program to find optimal control input
        
        Formulation:
        minimize: u^T u + gamma^2
        subject to: 
        - CLF constraint: dV/dx * u <= -alpha * V + gamma (goal reaching)
        - CBF constraint: dh/dx * u >= -beta * h (connectivity maintenance)
        - Control bounds: ||u|| <= u_max
        """
        try:
            # Compute CLF for goal-seeking
            V, dV_dx = self.compute_clf_goal_seeking(robot_position, goal_position)
            
            # Compute CBF for connectivity
            cbf_constraints = self.compute_cbf_connectivity(robot_position, neighbor_positions, max_range)
            
            if CVXPY_AVAILABLE:
                return self._solve_clf_cbf_cvxpy(V, dV_dx, cbf_constraints)
            else:
                return self._solve_clf_cbf_scipy(V, dV_dx, cbf_constraints)
                
        except Exception as e:
            print(f"CLF-CBF QP failed: {e}, using enhanced fallback control")
            
            # Enhanced fallback with connectivity awareness
            control_input = np.zeros(2)
            
            # Goal-seeking component with INVERSE DISTANCE WEIGHTING (stronger when closer to goal)
            direction_to_goal = goal_position - robot_position
            goal_distance = np.linalg.norm(direction_to_goal)
            
            if goal_distance > 0:
                # TERMINAL GUIDANCE: Exponentially stronger goal-seeking when very close
                if goal_distance < 2.0:  # Within 2 unit of goal - TERMINAL PHASE
                    # Inverse exponential weighting: closer = much stronger
                    terminal_strength = 5.0 * (1.0 / max(goal_distance, 0.1))  # Minimum distance 0.1 to avoid division by zero
                    goal_component = terminal_strength * direction_to_goal / goal_distance
                    print(f"TERMINAL GUIDANCE ACTIVE: distance={goal_distance:.3f}, strength={terminal_strength:.3f}")
                else:
                    # Normal goal-seeking for distances > 1.0
                    goal_component = 0.8 * direction_to_goal / goal_distance
                
                control_input += goal_component
            
            # VERY STRONG connectivity preservation component
            for neighbor_pos in neighbor_positions:
                neighbor_diff = robot_position - neighbor_pos  
                neighbor_distance = np.linalg.norm(neighbor_diff)
                
                # Safety constraint: maintain minimum distance
                min_safe_distance = 1.0  # INCREASED for better visibility
                if neighbor_distance < min_safe_distance * 1.8:  # Within 80% buffer of minimum distance
                    # Strong repulsion to maintain safety distance
                    safety_strength = 0.80 * (min_safe_distance * 1.8 - neighbor_distance) / (min_safe_distance * 0.8)
                    safety_component = safety_strength * neighbor_diff / neighbor_distance
                    control_input += safety_component
                    print(f"⚠️  Fallback SAFETY control: distance={neighbor_distance:.3f}, repulsion={safety_strength:.3f}")
                
                # Connectivity constraint: if approaching communication limit, pull toward neighbor
                elif neighbor_distance > max_range * 0.70:  # Within 30% of limit - EARLIER trigger
                    connectivity_strength = 0.50 * (neighbor_distance / max_range - 0.70) / 0.30  # Scales 0->0.50 - STRONGER
                    connectivity_component = -connectivity_strength * neighbor_diff / neighbor_distance
                    control_input += connectivity_component
                    print(f"⚠️  Fallback CONNECTIVITY control: distance={neighbor_distance:.3f}, pull={connectivity_strength:.3f}")
            
            # Limit total control magnitude
            control_magnitude = np.linalg.norm(control_input)
            if control_magnitude > 0.35:  # INCREASED limit
                control_input = 0.35 * control_input / control_magnitude
                
            return control_input
    
    def _solve_clf_cbf_cvxpy(self, V, dV_dx, cbf_constraints):
        """Solve CLF-CBF QP using cvxpy"""
        # Control input (2D velocity)
        u = cp.Variable(2)
        gamma = cp.Variable(1)  # Relaxation variable for CLF
        
        # Parameters - SIGNIFICANTLY INCREASED to overcome strong opposing flow
        # TERMINAL GUIDANCE: Make alpha distance-dependent for stronger goal-seeking when close
        goal_distance = np.sqrt(V)  # V = ||x - goal||^2, so sqrt(V) = distance to goal
        if goal_distance < 1.0:  # Within 1 unit of goal - TERMINAL PHASE
            alpha = 6.0 + (1.0 / max(goal_distance, 0.1))  # Exponentially stronger when closer
            print(f"TERMINAL GUIDANCE ACTIVE in QP: distance={goal_distance:.3f}, alpha={alpha:.3f}")
        else:
            alpha = 1.0  # Normal CLF decay rate for far distances
        
        beta = 7.0  # MASSIVELY INCREASED CBF safety margin for dominant safety constraints
        u_max = 2.5  # NEARLY DOUBLED maximum control input magnitude to overcome opposing underwater forces
        
        # Objective: prioritize safety over goal-seeking
        # Add penalty for being close to safety violations
        safety_penalty = 0
        for cbf in cbf_constraints:
            if cbf['type'] == 'safety' and cbf['h'] < 2.0:  # Penalize being within 2x safe distance
                safety_penalty += 1000 * (2.0 - cbf['h'])**2  # Heavy penalty for proximity
        
        objective = cp.Minimize(cp.sum_squares(u) + 5 * cp.sum_squares(gamma) + safety_penalty)  # REDUCED gamma penalty from 10 to 5 to allow stronger goal seeking
        
        constraints = []
        
        # CLF constraint: dV/dx * u <= -alpha * V + gamma
        if V > 1e-6:  # Only apply if not already at goal
            constraints.append(dV_dx @ u <= -alpha * V + gamma)
            constraints.append(gamma >= 0)  # Relaxation variable is non-negative
        
        # CBF constraints: handle both connectivity and safety constraints
        emergency_braking = False
        for cbf in cbf_constraints:
            if cbf['type'] == 'connectivity':
                # Connectivity constraint: only apply if constraint is active (h > 0)
                if cbf['h'] > 0:
                    # Check for emergency braking condition (70% of max range - MUCH EARLIER)
                    if cbf['h'] < 0.30 * cbf.get('max_range', 2.5)**2:  # Within 30% of communication limit squared
                        emergency_braking = True
                        # VERY STRONG constraint: force velocity away from neighbor
                        constraints.append(cbf['dh_dx'] @ u >= 0.50)  # MUCH STRONGER repulsive force
                    else:
                        constraints.append(cbf['dh_dx'] @ u >= -beta * cbf['h'])
                        
            elif cbf['type'] == 'safety':
                # Safety constraint: ALWAYS apply, especially when h is small (robots too close)
                safety_beta = beta * 5.0  # 5X strength for safety constraints - MAXIMUM priority
                if cbf['h'] < 1.5 * cbf.get('min_safe_distance', 1.0)**2:  # Trigger much earlier for safety
                    emergency_braking = True
                    # MAXIMUM safety constraint: force velocity away from neighbor
                    constraints.append(cbf['dh_dx'] @ u >= 2.00)  # MAXIMUM repulsive force - doubled
                    print(f"🚨 EMERGENCY SAFETY: h={cbf['h']:.3f}, distance={cbf['distance']:.3f}, min_safe=1.0")
                else:
                    # Even regular safety constraints are very strong
                    constraints.append(cbf['dh_dx'] @ u >= -safety_beta * cbf['h'])
        
        # If emergency braking is active, allow maximum movement (CVXPY version)
        if emergency_braking:
            u_max = min(u_max, 0.80)  # SIGNIFICANTLY INCREASED maximum control authority for underwater environment
        
        # Control input bounds
        constraints.append(cp.norm(u, 2) <= u_max)
        
        # Solve the problem
        prob = cp.Problem(objective, constraints)
        prob.solve(solver=cp.ECOS, verbose=False)
        
        if prob.status == cp.OPTIMAL:
            return u.value
        else:
            print(f"CLF-CBF optimization failed with status: {prob.status}")
            return np.zeros(2)
    
    def _solve_clf_cbf_scipy(self, V, dV_dx, cbf_constraints):
        """Solve CLF-CBF QP using scipy (fallback when cvxpy not available)"""
        from scipy.optimize import minimize
        
        # Parameters
        alpha = 0.5  # INCREASED CLF decay rate - more aggressive goal seeking to overcome underwater forces
        beta = 10.0  # MASSIVELY INCREASED CBF parameter for dominant safety constraints
        u_max = 0.80  # SIGNIFICANTLY INCREASED for better control authority against underwater forces
        
        def objective(x):
            u = x[:2]
            gamma = x[2] if len(x) > 2 else 0
            return np.dot(u, u) + 100 * gamma**2
        
        # Constraints
        constraints = []
        
        # CLF constraint: dV/dx * u <= -alpha * V + gamma
        if V > 1e-6:
            def clf_constraint(x):
                u = x[:2]
                gamma = x[2] if len(x) > 2 else 0
                return -alpha * V + gamma - np.dot(dV_dx, u)
            constraints.append({'type': 'ineq', 'fun': clf_constraint})
            
            # Relaxation variable bounds
            bounds = [(-u_max, u_max), (-u_max, u_max), (0, 10)]
        else:
            bounds = [(-u_max, u_max), (-u_max, u_max)]
        
        # CBF constraints with emergency braking - handle both connectivity and safety
        emergency_braking = False
        for cbf in cbf_constraints:
            if cbf['type'] == 'connectivity':
                # Connectivity constraint: only apply if constraint is active (h > 0)
                if cbf['h'] > 0:
                    # Check for emergency braking condition (70% of max range - MUCH EARLIER)
                    if cbf['h'] < 0.30 * cbf.get('max_range', 2.5)**2:  # Within 30% of communication limit squared
                        emergency_braking = True
                        # VERY STRONG constraint: force velocity away from neighbor
                        def emergency_cbf_constraint(x, dh_dx=cbf['dh_dx']):
                            u = x[:2]
                            return np.dot(dh_dx, u) - 0.50  # MUCH STRONGER repulsive force
                        constraints.append({'type': 'ineq', 'fun': emergency_cbf_constraint})
                    else:
                        def cbf_constraint(x, dh_dx=cbf['dh_dx'], h=cbf['h']):
                            u = x[:2]
                            return np.dot(dh_dx, u) + beta * h
                        constraints.append({'type': 'ineq', 'fun': cbf_constraint})
                        
            elif cbf['type'] == 'safety':
                # Safety constraint: ALWAYS apply, especially when h is small (robots too close)
                safety_beta = beta * 5.0  # 5X strength for safety constraints - MAXIMUM priority
                if cbf['h'] < 1.5 * cbf.get('min_safe_distance', 1.0)**2:  # Trigger much earlier for safety
                    emergency_braking = True
                    # MAXIMUM safety constraint: force velocity away from neighbor
                    def emergency_safety_constraint(x, dh_dx=cbf['dh_dx']):
                        u = x[:2]
                        return np.dot(dh_dx, u) - 2.00  # MAXIMUM repulsive force - doubled
                    constraints.append({'type': 'ineq', 'fun': emergency_safety_constraint})
                    print(f"🚨 EMERGENCY SAFETY (scipy): h={cbf['h']:.3f}, distance={cbf['distance']:.3f}, min_safe=1.0")
                else:
                    # Even regular safety constraints are very strong
                    def safety_constraint(x, dh_dx=cbf['dh_dx'], h=cbf['h'], sb=safety_beta):
                        u = x[:2]
                        return np.dot(dh_dx, u) + sb * h
                    constraints.append({'type': 'ineq', 'fun': safety_constraint})
        
        # If emergency braking is active, allow maximum movement (SciPy version)
        if emergency_braking:
            u_max = min(u_max, 0.80)  # SIGNIFICANTLY INCREASED maximum control authority for underwater environment
            bounds = [(-u_max, u_max), (-u_max, u_max)] if len(bounds) == 2 else [(-u_max, u_max), (-u_max, u_max), (0, 10)]
        
        # Initial guess
        x0 = np.zeros(len(bounds))
        
        # Solve
        result = minimize(objective, x0, method='SLSQP', bounds=bounds, constraints=constraints)
        
        if result.success:
            return result.x[:2]
        else:
            print(f"CLF-CBF scipy optimization failed: {result.message}")
            return np.zeros(2)
    
    # ===== DECENTRALIZED EDGE OPTIMIZATION METHODS =====
    
    def discover_k_hop_neighbors(self, robot_id, k_max=2):
        """
        Discover k-hop neighbors with position estimates
        Returns: {neighbor_id: {'position': np.array, 'hops': int}}
        """
        print(f"🔍 Robot {robot_id} discovering {k_max}-hop neighbors...")
        print(f"   Direct neighbors: {list(self.nodes[robot_id].direct_neighbors)}")
        
        discovered = {}
        queue = [(robot_id, 0, self.nodes[robot_id].position)]  # (id, hops, position)
        visited = {robot_id}
        
        while queue:
            current_id, hops, pos = queue.pop(0)
            
            if hops > 0:  # Don't include self
                discovered[current_id] = {'position': pos, 'hops': hops}
                print(f"   Found: Robot {current_id} at {hops} hops, pos={pos}")
            
            if hops < k_max:
                # Explore neighbors
                for neighbor_id in self.nodes[current_id].direct_neighbors:
                    if neighbor_id not in visited:
                        visited.add(neighbor_id)
                        neighbor_pos = self.nodes[neighbor_id].position
                        queue.append((neighbor_id, hops + 1, neighbor_pos))
        
        print(f"   Total discovered: {len(discovered)} robots")
        return discovered
    
    def compute_edge_addition_score(self, robot_id, target_id, k_hop_neighbors):
        """
        Compute score for adding edge between robot_id and target_id
        Mathematical formulation:
        S_add(i,j) = α * distance_score + β * connectivity_score + γ * efficiency_score
        
        Returns: float score (higher = better)
        """
        robot_pos = self.nodes[robot_id].position
        target_info = k_hop_neighbors.get(target_id)
        
        if not target_info:
            return 0.0
        
        target_pos = target_info['position']
        distance = np.linalg.norm(robot_pos - target_pos)
        
        # Distance-based score: S_distance = (R_max - d_ij) / R_max
        if distance > 1.2:  # Beyond communication range
            return 0.0
        
        distance_score = (1.2 - distance) / 1.2
        
        # Connectivity improvement score based on common neighbors
        robot_neighbors = self.nodes[robot_id].direct_neighbors
        target_neighbors = self.nodes[target_id].direct_neighbors if target_id in self.nodes else set()
        
        # Triangle formation: more common neighbors = better connectivity
        common_neighbors = len(robot_neighbors.intersection(target_neighbors))
        connectivity_score = 1.0 + common_neighbors * 0.5
        
        # Path efficiency: shorter multi-hop paths are better
        hop_penalty = 1.0 / (1.0 + target_info['hops'] - 1)
        
        # Network clustering improvement
        clustering_bonus = 0.0
        if common_neighbors > 0:
            clustering_bonus = common_neighbors / max(len(robot_neighbors), len(target_neighbors), 1)
        
        # Final score: weighted combination
        α, β, γ, δ = 0.4, 0.3, 0.2, 0.1  # Weights
        total_score = (α * distance_score + 
                      β * connectivity_score + 
                      γ * hop_penalty + 
                      δ * clustering_bonus)
        
        return total_score
    
    def compute_edge_removal_score(self, robot_id, neighbor_id):
        """
        Compute score for removing edge between robot_id and neighbor_id
        Mathematical formulation:
        S_remove(i,j) = α * (d_ij/R_max) + β * redundancy_score + γ * (1/betweenness)
        
        Returns: float score (higher = more likely to remove)
        """
        if neighbor_id not in self.nodes:
            return 0.0
            
        robot_pos = self.nodes[robot_id].position
        neighbor_pos = self.nodes[neighbor_id].position
        distance = np.linalg.norm(robot_pos - neighbor_pos)
        
        # Distance penalty: longer edges preferred for removal
        distance_score = min(distance / 1.2, 1.0)
        
        # Check if removal preserves connectivity (critical edge detection)
        connectivity_preserved = self.has_alternative_path(robot_id, neighbor_id)
        
        if not connectivity_preserved:
            return 0.0  # Don't remove critical edges
        
        # Redundancy score: more alternative paths = higher removal score
        alternative_paths = self.count_alternative_paths(robot_id, neighbor_id, max_hops=3)
        redundancy_score = min(alternative_paths / 2.0, 1.0)
        
        # Edge betweenness centrality (approximation)
        betweenness_score = 1.0 / (1.0 + self.approximate_edge_betweenness(robot_id, neighbor_id))
        
        # Final score: weighted combination
        α, β, γ = 0.4, 0.4, 0.2
        total_score = α * distance_score + β * redundancy_score + γ * betweenness_score
        
        return total_score
    
    def has_alternative_path(self, robot_id, target_id, max_hops=3):
        """
        Check if alternative path exists between robot_id and target_id
        excluding direct edge (BFS-based connectivity check)
        """
        if target_id not in self.nodes:
            return False
        
        # BFS without using direct edge
        queue = [(robot_id, 0)]
        visited = {robot_id}
        
        while queue:
            current_id, hops = queue.pop(0)
            
            if hops >= max_hops:
                continue
            
            for neighbor_id in self.nodes[current_id].direct_neighbors:
                # Skip direct edge we're testing for removal
                if current_id == robot_id and neighbor_id == target_id:
                    continue
                if current_id == target_id and neighbor_id == robot_id:
                    continue
                    
                if neighbor_id == target_id and hops > 0:  # Found alternative path
                    return True
                
                if neighbor_id not in visited:
                    visited.add(neighbor_id)
                    queue.append((neighbor_id, hops + 1))
        
        return False
    
    def count_alternative_paths(self, robot_id, target_id, max_hops=3):
        """
        Count number of alternative paths between robot_id and target_id
        (excluding direct edge)
        """
        if target_id not in self.nodes:
            return 0
        
        paths_found = 0
        
        # Count 2-hop paths through intermediate nodes
        for intermediate_id in self.nodes:
            if intermediate_id in [robot_id, target_id]:
                continue
            
            # Check if path exists: robot_id -> intermediate -> target_id
            if (intermediate_id in self.nodes[robot_id].direct_neighbors and 
                target_id in self.nodes[intermediate_id].direct_neighbors):
                paths_found += 1
        
        return paths_found
    
    def approximate_edge_betweenness(self, robot_id, neighbor_id):
        """
        Approximate edge betweenness centrality
        (how many shortest paths pass through this edge)
        """
        if neighbor_id not in self.nodes:
            return 0.0
        
        # Simple approximation: count how many nodes are closer to robot_id through neighbor_id
        betweenness = 0.0
        robot_pos = self.nodes[robot_id].position
        neighbor_pos = self.nodes[neighbor_id].position
        
        for other_id, other_node in self.nodes.items():
            if other_id in [robot_id, neighbor_id]:
                continue
            
            # Distance from other node to robot directly vs through neighbor
            direct_dist = np.linalg.norm(other_node.position - robot_pos)
            via_neighbor_dist = (np.linalg.norm(other_node.position - neighbor_pos) + 
                               np.linalg.norm(neighbor_pos - robot_pos))
            
            # If path through neighbor is shorter, this edge has higher betweenness
            if via_neighbor_dist < direct_dist:
                betweenness += 1.0
        
        return betweenness
    
    def is_network_connected_without_edge(self, robot_id, neighbor_id):
        """
        Check if removing edge (robot_id, neighbor_id) would keep network connected
        Uses BFS from any node to verify all nodes are reachable
        """
        if len(self.nodes) <= 2:
            return False  # Can't remove any edge from 2-node network
        
        # Temporarily remove the edge for connectivity test
        temp_removed = False
        if neighbor_id in self.nodes[robot_id].direct_neighbors:
            self.nodes[robot_id].direct_neighbors.discard(neighbor_id)
            self.nodes[neighbor_id].direct_neighbors.discard(robot_id)
            temp_removed = True
        
        # BFS connectivity test from first node
        start_node = next(iter(self.nodes.keys()))
        visited = set()
        queue = [start_node]
        visited.add(start_node)
        
        while queue:
            current = queue.pop(0)
            for neighbor in self.nodes[current].direct_neighbors:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        
        is_connected = len(visited) == len(self.nodes)
        
        # Restore the temporarily removed edge
        if temp_removed:
            self.nodes[robot_id].direct_neighbors.add(neighbor_id)
            self.nodes[neighbor_id].direct_neighbors.add(robot_id)
        
        return is_connected

    def optimize_edges_decentralized(self, robot_id, k_max=2):
        """
        Main decentralized edge optimization for a single robot
        
        NEW APPROACH: Edge swapping for shorter connections
        - Find closest k-hop neighbor within communication range
        - If closer than current connections, swap edges simultaneously
        - Ensures connectivity preservation through simultaneous add/remove
        """
        changes = {'added': [], 'removed': []}
        
        # Phase 1: Multi-hop neighbor discovery
        k_hop_neighbors = self.discover_k_hop_neighbors(robot_id, k_max)
        
        print(f"🤖 Robot {robot_id}: Found {len(k_hop_neighbors)} k-hop neighbors: {list(k_hop_neighbors.keys())}")
        
        if len(k_hop_neighbors) == 0:
            return changes
        
        # Phase 2: Find shortest distance improvement opportunities
        current_neighbors = list(self.nodes[robot_id].direct_neighbors)
        robot_pos = self.nodes[robot_id].position
        
        # Find longest current connection
        longest_current_distance = 0.0
        longest_current_neighbor = None
        
        print(f"🔍 Robot {robot_id} current connections:")
        for neighbor_id in current_neighbors:
            distance = np.linalg.norm(robot_pos - self.nodes[neighbor_id].position)
            print(f"   -> Current neighbor {neighbor_id}: distance={distance:.3f}")
            if distance > longest_current_distance:
                longest_current_distance = distance
                longest_current_neighbor = neighbor_id
        
        # Find shortest potential new connection
        shortest_new_distance = float('inf')
        shortest_new_neighbor = None
        
        print(f"🔍 Robot {robot_id} evaluating potential edges:")
        for target_id, info in k_hop_neighbors.items():
            if target_id not in current_neighbors:  # Not already connected
                distance = np.linalg.norm(robot_pos - info['position'])
                print(f"   -> Potential neighbor {target_id}: distance={distance:.3f}, hops={info['hops']}")
                
                # Must be within communication range
                if distance <= 1.2 and distance < shortest_new_distance:
                    shortest_new_distance = distance
                    shortest_new_neighbor = target_id
        
        # Phase 3: Edge swapping decision
        if (shortest_new_neighbor and longest_current_neighbor and 
            shortest_new_distance < longest_current_distance and 
            len(current_neighbors) > 0):  # Ensure we don't disconnect
            
            improvement = longest_current_distance - shortest_new_distance
            print(f"🎯 Robot {robot_id} edge swap opportunity:")
            print(f"   Remove: {robot_id}-{longest_current_neighbor} (dist: {longest_current_distance:.3f})")
            print(f"   Add: {robot_id}-{shortest_new_neighbor} (dist: {shortest_new_distance:.3f})")
            print(f"   Improvement: {improvement:.3f}")
            
            # Minimum improvement threshold (very small to encourage swaps)
            improvement_threshold = 0.05
            
            if improvement > improvement_threshold:
                # CRITICAL: Check network connectivity BEFORE edge removal
                print(f"🔒 Robot {robot_id} checking connectivity before edge swap...")
                
                # Add new edge temporarily 
                self.nodes[robot_id].direct_neighbors.add(shortest_new_neighbor)
                if shortest_new_neighbor in self.nodes:
                    self.nodes[shortest_new_neighbor].direct_neighbors.add(robot_id)
                
                # Test: can we remove the old edge without breaking connectivity?
                if self.is_network_connected_without_edge(robot_id, longest_current_neighbor):
                    # SAFE TO REMOVE: Network stays connected
                    print(f"✅ Robot {robot_id} connectivity preserved - proceeding with edge swap")
                    
                    # Remove old longer edge
                    self.nodes[robot_id].direct_neighbors.discard(longest_current_neighbor)
                    if longest_current_neighbor in self.nodes:
                        self.nodes[longest_current_neighbor].direct_neighbors.discard(robot_id)
                    
                    changes['added'].append((robot_id, shortest_new_neighbor))
                    changes['removed'].append((robot_id, longest_current_neighbor))
                    
                    print(f"🎉 Robot {robot_id} successfully swapped edges!")
                    print(f"   Added: {robot_id}↔{shortest_new_neighbor} (distance: {shortest_new_distance:.3f})")
                    print(f"   Removed: {robot_id}↔{longest_current_neighbor} (distance: {longest_current_distance:.3f})")
                    
                else:
                    # UNSAFE: Removing edge would disconnect network
                    print(f"WARNING: Robot {robot_id} edge removal would break connectivity - reverting")
                    
                    # Revert the temporary edge addition
                    self.nodes[robot_id].direct_neighbors.discard(shortest_new_neighbor)
                    if shortest_new_neighbor in self.nodes:
                        self.nodes[shortest_new_neighbor].direct_neighbors.discard(robot_id)
                
                return changes
                changes['added'].append((robot_id, shortest_new_neighbor))
                
                # Step 2: Remove longer edge (connectivity is preserved)
                self.nodes[robot_id].direct_neighbors.discard(longest_current_neighbor)
                if longest_current_neighbor in self.nodes:
                    self.nodes[longest_current_neighbor].direct_neighbors.discard(robot_id)
                changes['removed'].append((robot_id, longest_current_neighbor))
                
                print(f"✅ Robot {robot_id} performed edge swap:")
                print(f"   🔗 Added: {robot_id}↔{shortest_new_neighbor} (dist: {shortest_new_distance:.3f})")
                print(f"   🔓 Removed: {robot_id}↔{longest_current_neighbor} (dist: {longest_current_distance:.3f})")
                
        else:
            print(f"❌ Robot {robot_id}: No beneficial edge swaps found")
            if not shortest_new_neighbor:
                print(f"   - No k-hop neighbors within 1.2 range")
            elif not longest_current_neighbor:
                print(f"   - No current connections to improve")
            elif shortest_new_distance >= longest_current_distance:
                print(f"   - Best new: {shortest_new_distance:.3f}, current longest: {longest_current_distance:.3f}")
        
        return changes
    
    def run_decentralized_edge_optimization(self):
        """
        Run edge optimization for all robots in a decentralized manner
        
        This simulates each robot independently running the optimization algorithm
        with only local k-hop knowledge, mimicking realistic distributed execution.
        """
        print("\n🔄 Running Decentralized Edge Optimization (k=2 hop discovery)...")
        all_changes = {'added': [], 'removed': []}
        
        # Randomize order to simulate asynchronous execution
        robot_ids = list(self.nodes.keys())
        random.shuffle(robot_ids)
        
        edge_changes_made = False
        
        for robot_id in robot_ids:
            changes = self.optimize_edges_decentralized(robot_id, k_max=2)
            all_changes['added'].extend(changes['added'])
            all_changes['removed'].extend(changes['removed'])
            
            if changes['added'] or changes['removed']:
                edge_changes_made = True
        
        if edge_changes_made:
            print(f"✅ Edge optimization complete: {len(all_changes['added'])} edges added, {len(all_changes['removed'])} edges removed")
            
            # Print current network statistics
            total_edges = sum(len(node.direct_neighbors) for node in self.nodes.values()) // 2
            avg_degree = total_edges * 2 / len(self.nodes)
            print(f"📊 Network stats: {total_edges} edges, avg degree: {avg_degree:.2f}")
        else:
            print("✅ No edge changes needed - network is optimally configured")
        
        return all_changes

    def update_positions_goal_seeking(self):
        """Update robot positions with goal-seeking and critical edge preservation"""
        if self.closest_robot_id is None:
            return
        
        # DEBUG: Print function entry
        if not hasattr(self, 'debug_position_counter'):
            self.debug_position_counter = 0
        self.debug_position_counter += 1
        
        if self.debug_position_counter % 10 == 0:
            print(f"🔧 DEBUG: update_positions_goal_seeking() called (count: {self.debug_position_counter})")
            print(f"   Closest robot: {self.closest_robot_id}")
            print(f"   Before update positions:")
            for node_id, node in self.nodes.items():
                print(f"     Robot {node_id}: {node.position}")
        
        # STEP 1: Run distributed critical edge detection algorithm periodically 
        if not hasattr(self, 'algorithm_counter'):
            self.algorithm_counter = 0
            self.cached_critical_edges = set()  # Cache results
        
        self.algorithm_counter += 1
        
        # Run the expensive critical edge detection every 5 frames
        if self.algorithm_counter % 5 == 0:
            for node in self.nodes.values():
                node.execute_one_step(self.nodes)
            # Cache the critical edges from the algorithm
            all_critical_edges, node_critical_edges = self.run_critical_edge_detection()
            self.cached_critical_edges = all_critical_edges
        
        # STEP 2: Classify current edges using cached critical edges (every frame)
        critical_edges = []
        non_critical_edges = []
        max_communication_range = 2.5  # INCREASED significantly to make safety distances more visible
        
        for node_id, node in self.nodes.items():
            for neighbor_id in node.direct_neighbors:
                if node_id < neighbor_id:  # Count each edge only once
                    edge = (node_id, neighbor_id)
                    distance = np.linalg.norm(node.position - self.nodes[neighbor_id].position)
                    
                    # Use cached critical edges from the algorithm
                    is_algorithm_critical = edge in self.cached_critical_edges
                    
                    # An edge is critical if the algorithm identifies it as critical
                    # OR if removing it would exceed communication range for a critical path
                    if is_algorithm_critical:
                        critical_edges.append(edge)
                    else:
                        non_critical_edges.append(edge)
        
        # STEP 3: Aggressively prune non-critical edges - MUCH MORE AGGRESSIVE
        edges_to_remove = []
        
        # Strategy 1: Remove ALL non-critical edges (let connectivity check decide what's safe)
        for edge in non_critical_edges:
            node1, node2 = edge
            distance = np.linalg.norm(self.nodes[node1].position - self.nodes[node2].position)
            
            # AGGRESSIVE: Try to remove ALL non-critical edges, regardless of distance
            # The connectivity check below will keep only the necessary ones
            edges_to_remove.append(edge)
            
            # Optional: Print which edges we're trying to remove for debugging
            if len(edges_to_remove) <= 3 and self.debug_position_counter % 20 == 0:  # Limit debug output
                print(f"   🔄 Attempting to prune non-critical edge {edge}, distance={distance:.2f}")
        
        if self.debug_position_counter % 20 == 0:  # Print summary less frequently
            print(f"   📊 Attempting to prune {len(edges_to_remove)} of {len(non_critical_edges)} non-critical edges")
        
        # Actually remove the edges from the network (with connectivity check)
        successfully_removed = []
        for edge in edges_to_remove:
            node1, node2 = edge
            
            # Temporarily remove the edge to test connectivity
            edge_existed = False
            if node2 in self.nodes[node1].direct_neighbors:
                self.nodes[node1].direct_neighbors.remove(node2)
                edge_existed = True
            if node1 in self.nodes[node2].direct_neighbors:
                self.nodes[node2].direct_neighbors.remove(node1)
            
            # Check if network is still connected
            if edge_existed and self.is_network_connected():
                # Network remains connected, keep the edge removed
                successfully_removed.append(edge)
                if self.debug_position_counter % 20 == 0:  # Less frequent output
                    print(f"   ✅ Successfully pruned edge {edge}")
            elif edge_existed:
                # Network would disconnect, restore the edge
                self.nodes[node1].direct_neighbors.add(node2)
                self.nodes[node2].direct_neighbors.add(node1)
                if self.debug_position_counter % 20 == 0:  # Less frequent output
                    print(f"   ❌ Cannot prune edge {edge} - would disconnect network")
        
        if self.debug_position_counter % 20 == 0:  # Print summary less frequently
            print(f"   🎯 PRUNING RESULT: Successfully removed {len(successfully_removed)} of {len(edges_to_remove)} attempted edges")
        
        # Store current state for visualization (use REAL-TIME critical edge detection)
        actual_critical_edges = []
        actual_non_critical_edges = []
        
        # For each remaining edge, check if removing it would disconnect the network
        current_edges = []
        for node_id, node in self.nodes.items():
            for neighbor_id in node.direct_neighbors:
                if node_id < neighbor_id:  # Count each edge only once
                    current_edges.append((node_id, neighbor_id))
        
        # Test each edge to see if it's critical
        for edge in current_edges:
            node1, node2 = edge
            
            # Temporarily remove the edge
            was_connected_1 = node2 in self.nodes[node1].direct_neighbors
            was_connected_2 = node1 in self.nodes[node2].direct_neighbors
            
            if was_connected_1:
                self.nodes[node1].direct_neighbors.remove(node2)
            if was_connected_2:
                self.nodes[node2].direct_neighbors.remove(node1)
            
            # Check if network is still connected
            is_still_connected = self.is_network_connected()
            
            # Restore the edge
            if was_connected_1:
                self.nodes[node1].direct_neighbors.add(node2)
            if was_connected_2:
                self.nodes[node2].direct_neighbors.add(node1)
            
            # Classify the edge
            if not is_still_connected:
                # Removing this edge disconnects the network - it's CRITICAL
                actual_critical_edges.append(edge)
            else:
                # Network stays connected without this edge - it's NON-CRITICAL
                actual_non_critical_edges.append(edge)
        
        self.current_critical_edges = actual_critical_edges
        self.current_non_critical_edges = actual_non_critical_edges
        self.pruned_edges = successfully_removed
        
        # Debug output every 10 frames
        if hasattr(self, 'debug_counter'):
            self.debug_counter += 1
        else:
            self.debug_counter = 0
            
        if self.debug_counter % 10 == 0:
            print(f"🔍 DEBUG Frame {self.debug_counter}:")
            print(f"   Current edges in network: {current_edges}")
            print(f"   Classified as Critical: {actual_critical_edges}")
            print(f"   Classified as Non-Critical: {actual_non_critical_edges}")
            print(f"   Successfully removed: {successfully_removed}")
            print(f"   self.current_critical_edges = {self.current_critical_edges}")
            print("   " + "-"*50)
        
        # Initialize velocities if not present
        for node in self.nodes.values():
            if not hasattr(node, 'velocity'):
                node.velocity = np.random.normal(0, 0.02, 2)
        
        # STEP 4: Update robot positions with intelligent movement
        spring_strength = 0.15  # INCREASED for stronger connectivity preservation
        max_communication_range = 2.5  # INCREASED significantly to make safety distances more visible
        
        for node_id, node in self.nodes.items():
            # Initialize velocities if not present
            if not hasattr(node, 'velocity'):
                node.velocity = np.random.normal(0, 0.02, 2)
            
            # Get neighbor positions for CBF constraints FIRST
            neighbor_positions = []
            # For complete graph, consider ALL other robots as neighbors for safety
            for other_id in self.nodes.keys():
                if other_id != node_id:  # Don't include self
                    neighbor_positions.append(self.nodes[other_id].position)
            
            # === CLF-CBF CONTROL FOR ALL ROBOTS ===
            # DEBUG: Only print occasionally to avoid spam
            if hasattr(self, 'clf_print_counter'):
                self.clf_print_counter += 1
            else:
                self.clf_print_counter = 0
            
            if self.clf_print_counter % 50 == 0:  # Print every 50 calls instead of every call
                print(f"🤖 Using CLF-CBF control for robot {node_id}")
                # Debug CBF constraint counts
                connectivity_constraints = sum(1 for cbf in self.compute_cbf_connectivity(node.position, neighbor_positions, max_communication_range) if cbf['type'] == 'connectivity')
                safety_constraints = sum(1 for cbf in self.compute_cbf_connectivity(node.position, neighbor_positions, max_communication_range) if cbf['type'] == 'safety')
                print(f"   Applying {connectivity_constraints} connectivity + {safety_constraints} safety CBF constraints")
            
            # Different goal behavior: active robot goes to goal, others spread out
            if node_id == self.closest_robot_id:
                # Active robot: goal-seeking with connectivity constraints
                goal_position = self.goal_location
            else:
                # Non-active robots: maintain position or move slightly away from neighbors
                # Instead of clustering at network center, use a distributed approach
                
                # Calculate repulsion from other robots to encourage spreading
                repulsion_vector = np.zeros(2)
                for other_id, other_node in self.nodes.items():
                    if other_id != node_id:
                        diff = node.position - other_node.position
                        distance = np.linalg.norm(diff)
                        if distance > 0 and distance < 1.5:  # Within influence range
                            # Stronger repulsion when closer
                            repulsion_strength = (1.5 - distance) / 1.5
                            repulsion_vector += repulsion_strength * diff / distance
                
                # Goal is current position plus small repulsion (stay put but spread out)
                if np.linalg.norm(repulsion_vector) > 0:
                    goal_position = node.position + 1.0 * repulsion_vector / np.linalg.norm(repulsion_vector)
                else:
                    goal_position = node.position  # Stay in place if no repulsion needed
            
            # Solve CLF-CBF QP for ALL robots
            if len(neighbor_positions) > 0:
                control_input = self.solve_clf_cbf_qp(
                    robot_position=node.position,
                    goal_position=goal_position,
                    neighbor_positions=neighbor_positions,
                    max_range=max_communication_range
                )
            else:
                # No neighbors - just go toward goal with increased speed to overcome underwater forces
                direction_to_goal = goal_position - node.position
                if np.linalg.norm(direction_to_goal) > 0:
                    control_input = 0.40 * direction_to_goal / np.linalg.norm(direction_to_goal)  # SIGNIFICANTLY INCREASED
                else:
                    control_input = np.zeros(2)
            
            # Apply CLF-CBF control
            node.velocity = control_input
            
            # CRITICAL SAFETY OVERRIDE: Direct repulsion if any robot is too close
            safety_override = np.zeros(2)
            min_safe_distance = 1.0
            safety_violation = False
            
            # Check safety with ALL other robots, not just direct neighbors
            for other_id, other_node in self.nodes.items():
                if other_id != node_id:  # Don't check against self
                    neighbor_pos = other_node.position
                    diff = node.position - neighbor_pos
                    distance = np.linalg.norm(diff)
                    
                    if distance < min_safe_distance * 1.5:  # Safety zone violation
                        safety_violation = True
                        # VERY STRONG direct repulsion - bypasses optimization
                        repulsion_strength = 0.8 * (min_safe_distance * 1.5 - distance) / (min_safe_distance * 1.5)
                        if distance > 0:
                            safety_override += repulsion_strength * diff / distance
                        print(f"🚨 DIRECT SAFETY OVERRIDE: Robot {node_id} from {other_id}, dist={distance:.3f}, repulsion={repulsion_strength:.3f}")
            
            # Override control if safety is violated
            if safety_violation:
                # Limit the override to reasonable magnitude
                override_magnitude = np.linalg.norm(safety_override)
                if override_magnitude > 0.5:
                    safety_override = 0.5 * safety_override / override_magnitude
                
                # Replace CLF-CBF control with direct safety control
                node.velocity = safety_override
                print(f"🚨 SAFETY CONTROL OVERRIDE: Robot {node_id}, safety_vel=[{safety_override[0]:.4f}, {safety_override[1]:.4f}]")
            
            # Debug output for CLF-CBF
            if hasattr(self, 'clf_cbf_debug_counter'):
                self.clf_cbf_debug_counter += 1
            else:
                self.clf_cbf_debug_counter = 0
            
            if self.clf_cbf_debug_counter % 30 == 0:  # Every 30 frames
                dist_to_goal = np.linalg.norm(node.position - goal_position)
                print(f"🎯 CLF-CBF Control Robot {node_id}:")
                print(f"   Distance to goal: {dist_to_goal:.3f}")
                print(f"   Control input: [{control_input[0]:.4f}, {control_input[1]:.4f}]")
                print(f"   Connected neighbors: {len(neighbor_positions)}")
                for i, neighbor_pos in enumerate(neighbor_positions):
                    dist = np.linalg.norm(node.position - neighbor_pos)
                    safety_violation = "SAFETY VIOLATION!" if dist < 1.0 else ""
                    connectivity_risk = "CONNECTIVITY RISK!" if dist > max_communication_range * 0.8 else ""
                    print(f"   Neighbor {i+1} distance: {dist:.3f} (max: {max_communication_range:.3f}, min_safe: 1.0) {safety_violation} {connectivity_risk}")
            
            # Apply underwater dynamics or simple position update
            if self.enable_underwater_physics:
                # All robots now use CLF-CBF control with underwater physics
                self.apply_underwater_dynamics(node, control_input)
                
                # Update position after applying physics
                old_position = node.position.copy()
                # FIXED: Use positive velocity components for movement (same as arrow visualization)
                movement_velocity = np.array([abs(node.velocity[0]), abs(node.velocity[1])])
                node.position += movement_velocity * self.dt
                position_change = np.linalg.norm(node.position - old_position)
                
                # Debug position updates
                if node_id <= 2 and self.debug_position_counter % 20 == 0:
                    control_mag = np.linalg.norm(control_input)
                    print(f"Robot {node_id}: pos_change={position_change:.6f}, vel_mag={np.linalg.norm(node.velocity):.6f}, control_mag={control_mag:.6f}")
                
                # Update environmental time
                if node_id == 1:  # Update time only once per frame
                    self.underwater_env.update_time(self.dt)
            else:
                # Original simple dynamics - FIXED: Use positive velocity components
                movement_velocity = np.array([abs(node.velocity[0]), abs(node.velocity[1])])
                node.position += movement_velocity
            
            # Boundary handling with realistic underwater physics
            if self.enable_underwater_physics:
                # Softer boundary conditions for underwater environment
                boundary_damping = 0.7
                if node.position[0] <= 0:
                    node.velocity[0] = abs(node.velocity[0]) * boundary_damping  # Bounce back
                    node.position[0] = 0.1
                elif node.position[0] >= 10:
                    node.velocity[0] = -abs(node.velocity[0]) * boundary_damping
                    node.position[0] = 9.9
                    
                if node.position[1] <= 0:
                    node.velocity[1] = abs(node.velocity[1]) * boundary_damping
                    node.position[1] = 0.1
                elif node.position[1] >= 10:
                    node.velocity[1] = -abs(node.velocity[1]) * boundary_damping
                    node.position[1] = 9.9
            else:
                # Original boundary handling with reflection
                if node.position[0] <= 0 or node.position[0] >= 10:
                    node.velocity[0] *= -0.9
                    node.position[0] = np.clip(node.position[0], 0, 10)
                
                if node.position[1] <= 0 or node.position[1] >= 10:
                    node.velocity[1] *= -0.9
                    node.position[1] = np.clip(node.position[1], 0, 10)
        
        # DEBUG: Print positions after update
        if self.debug_position_counter % 10 == 0:
            print(f"   After update positions:")
            for node_id, node in self.nodes.items():
                velocity_mag = np.linalg.norm(node.velocity) if hasattr(node, 'velocity') else 0
                print(f"     Robot {node_id}: {node.position}, vel_mag: {velocity_mag:.4f}")
            print("   " + "-"*40)

def create_random_graph(num_nodes: int = 10, edge_probability: float = 0.4, preferred_type: str = 'random', enable_underwater: bool = True) -> DistributedNetwork:
    """Create a random connected graph starting in 1-unit radius, 10x10 workspace with underwater physics"""
    import random
    
    environment_type = "UNDERWATER" if enable_underwater else "SURFACE"
    print(f"\nGenerating {environment_type} random connected graph with {num_nodes} nodes...")
    print(f"Initial radius: 1 unit (clustered start)")
    print(f"Workspace: 10x10 units (robots can spread out)")
    print(f"Preferred structure: {preferred_type}")
    print(f"Edge probability: {edge_probability}")
    print(f"Physics: {'Advanced underwater dynamics' if enable_underwater else 'Simple kinematic'}")
    print("-" * 70)
    
    # Create empty network with underwater physics option
    network = DistributedNetwork(create_default_graph=False, enable_underwater_physics=enable_underwater)
    
    # Generate positions in a circle within 1-unit radius (clustered start)
    positions = {}
    for i in range(1, num_nodes + 1):
        angle = 2 * np.pi * (i - 1) / num_nodes
        # Start clustered within 1-unit radius
        radius = 0.7 + random.uniform(-0.1, 0.1)  # Small random variation
        x = radius * np.cos(angle) + random.uniform(-0.1, 0.1)
        y = radius * np.sin(angle) + random.uniform(-0.1, 0.1)
        positions[i] = (x, y)
    
    # Add nodes
    for node_id in range(1, num_nodes + 1):
        network.nodes[node_id] = Node(node_id, np.array(positions[node_id]))
    
    # Strategy: Create different types of graph structures
    edges = []
    
    # Handle triangle_rich specially - use dedicated function
    if preferred_type == 'triangle_rich':
        print("🔺 Creating EXPLICIT triangle-rich graph...")
        return create_explicit_triangle_graph(num_nodes)
    
    # Handle complete graph specially - use dedicated function
    if preferred_type == 'complete':
        print("🔗 Creating COMPLETE graph...")
        return create_complete_graph(num_nodes)
    
    # Choose graph type based on preference
    if preferred_type == 'random':
        graph_types = ['path', 'star', 'tree', 'cycle_plus']
        chosen_type = random.choice(graph_types)
    else:
        chosen_type = preferred_type
    
    print(f"Creating {chosen_type}-like structure...")
    
    if chosen_type == 'path':
        # Create a path with some random connections
        nodes_list = list(range(1, num_nodes + 1))
        random.shuffle(nodes_list)
        for i in range(len(nodes_list) - 1):
            edges.append((nodes_list[i], nodes_list[i + 1]))
        
        # Add 1-3 random shortcuts
        for _ in range(random.randint(1, max(1, num_nodes // 4))):
            i, j = random.sample(nodes_list, 2)
            if (i, j) not in edges and (j, i) not in edges and abs(nodes_list.index(i) - nodes_list.index(j)) > 1:
                edges.append((i, j))
    
    elif chosen_type == 'star':
        # Create a star with additional connections
        center = random.randint(1, num_nodes)
        for i in range(1, num_nodes + 1):
            if i != center:
                edges.append((center, i))
        
        # Add some connections between leaf nodes
        leaf_nodes = [i for i in range(1, num_nodes + 1) if i != center]
        for _ in range(random.randint(1, max(1, len(leaf_nodes) // 3))):
            if len(leaf_nodes) >= 2:
                i, j = random.sample(leaf_nodes, 2)
                if (i, j) not in edges and (j, i) not in edges:
                    edges.append((i, j))
    
    elif chosen_type == 'cycle_plus':
        # Create a cycle with additional edges
        nodes_list = list(range(1, num_nodes + 1))
        random.shuffle(nodes_list)
        for i in range(len(nodes_list)):
            edges.append((nodes_list[i], nodes_list[(i + 1) % len(nodes_list)]))
        
        # Add some chords to the cycle
        for _ in range(random.randint(1, max(1, num_nodes // 3))):
            i, j = random.sample(nodes_list, 2)
            if (i, j) not in edges and (j, i) not in edges:
                edges.append((i, j))
    
    else:  # 'tree'
        # Create a random spanning tree using modified Kruskal-like approach
        nodes_list = list(range(1, num_nodes + 1))
        random.shuffle(nodes_list)
        
        in_tree = {nodes_list[0]}
        remaining = set(nodes_list[1:])
        
        while remaining:
            # Pick a random node already in tree
            tree_node = random.choice(list(in_tree))
            # Pick a random node not in tree
            new_node = random.choice(list(remaining))
            
            edges.append((tree_node, new_node))
            in_tree.add(new_node)
            remaining.remove(new_node)
        
        # Add some extra edges to make it more interesting
        for _ in range(random.randint(1, max(1, num_nodes // 4))):
            i, j = random.sample(range(1, num_nodes + 1), 2)
            if (i, j) not in edges and (j, i) not in edges:
                edges.append((i, j))
    
    # Add all edges to network
    for node1, node2 in edges:
        network.nodes[node1].add_direct_neighbor(node2)
        network.nodes[node2].add_direct_neighbor(node1)
    
    print(f"Generated {len(edges)} edges: {sorted(edges)}")
    
    # Initialize all nodes
    for node in network.nodes.values():
        node.initialize_algorithm(num_nodes)
    
    # Print initial state
    print("Initial Graph Structure:")
    network.print_graph_structure()
    print("\nInitial State (k=0):")
    network.print_all_states()
    
    # Verify connectivity
    if not network.check_connectivity_status():
        print("WARNING: Graph is not connected! Retrying...")
        return create_random_graph(num_nodes, edge_probability)
    
    print(f"✅ Successfully created connected random graph!")
    return network

def create_explicit_triangle_graph(num_nodes: int = 8) -> DistributedNetwork:
    """
    Create a graph with SIMPLE EXPLICIT triangles - absolutely guaranteed
    """
    print(f"\n🔺 CREATING SIMPLE TRIANGLE GRAPH")
    print(f"   Nodes: {num_nodes}")
    print("   Creating FOOLPROOF triangles...")
    print("=" * 50)
    
    # Create network
    network = DistributedNetwork(create_default_graph=False)
    
    # Create nodes
    for i in range(1, num_nodes + 1):
        angle = 2 * np.pi * (i - 1) / num_nodes
        x = 2 * np.cos(angle)
        y = 2 * np.sin(angle)
        network.nodes[i] = Node(i, np.array([x, y]))
    
    print("🔺 Creating triangle (1,2,3) - GUARANTEED triangle:")
    print("   Adding edge (1,2)")
    print("   Adding edge (2,3)")  
    print("   Adding edge (1,3)")
    print("   Result: Node 1 and Node 2 will have common neighbor 3!")
    
    # Create ONE SIMPLE triangle: (1,2,3)
    network.nodes[1].add_direct_neighbor(2)
    network.nodes[2].add_direct_neighbor(1)
    
    network.nodes[2].add_direct_neighbor(3)
    network.nodes[3].add_direct_neighbor(2)
    
    network.nodes[1].add_direct_neighbor(3)  # This completes the triangle!
    network.nodes[3].add_direct_neighbor(1)
    
    # Add a few more edges but not in triangles (to test the difference)
    if num_nodes >= 4:
        print("🔗 Adding non-triangle edge (1,4)")
        network.nodes[1].add_direct_neighbor(4)
        network.nodes[4].add_direct_neighbor(1)
    
    if num_nodes >= 5:
        print("🔗 Adding non-triangle edge (4,5)")
        network.nodes[4].add_direct_neighbor(5)
        network.nodes[5].add_direct_neighbor(4)
        
        print("🔺 Creating another triangle (3,4,5):")
        network.nodes[3].add_direct_neighbor(4)
        network.nodes[4].add_direct_neighbor(3)
        
        network.nodes[3].add_direct_neighbor(5)
        network.nodes[5].add_direct_neighbor(3)
    
    # Connect remaining nodes simply
    for i in range(6, num_nodes + 1):
        # Just connect to node 1 (no triangles)
        network.nodes[1].add_direct_neighbor(i)
        network.nodes[i].add_direct_neighbor(1)
    
    # Initialize algorithm
    for node in network.nodes.values():
        node.initialize_algorithm(len(network.nodes))
    
    print(f"\n✅ SIMPLE TRIANGLE GRAPH CREATED!")
    
    # MANUAL VERIFICATION
    print("\n🔍 MANUAL TRIANGLE VERIFICATION:")
    
    print("📊 Node neighbor lists:")
    for node_id in sorted(network.nodes.keys()):
        neighbors = sorted(list(network.nodes[node_id].direct_neighbors))
        print(f"   Node {node_id}: {neighbors}")
    
    print("\n🔺 Checking triangle (1,2,3):")
    if 1 in network.nodes and 2 in network.nodes and 3 in network.nodes:
        node1_neighbors = network.nodes[1].direct_neighbors
        node2_neighbors = network.nodes[2].direct_neighbors  
        node3_neighbors = network.nodes[3].direct_neighbors
        
        print(f"   Node 1 neighbors: {sorted(node1_neighbors)}")
        print(f"   Node 2 neighbors: {sorted(node2_neighbors)}")
        print(f"   Node 3 neighbors: {sorted(node3_neighbors)}")
        
        # Check edge (1,2) common neighbors
        common_12 = node1_neighbors.intersection(node2_neighbors)
        print(f"   Edge (1,2) common neighbors: {sorted(common_12)}")
        print(f"   Expected: should contain 3 = {'✅' if 3 in common_12 else '❌'}")
        
        # Check edge (2,3) common neighbors  
        common_23 = node2_neighbors.intersection(node3_neighbors)
        print(f"   Edge (2,3) common neighbors: {sorted(common_23)}")
        print(f"   Expected: should contain 1 = {'✅' if 1 in common_23 else '❌'}")
        
        # Check edge (1,3) common neighbors
        common_13 = node1_neighbors.intersection(node3_neighbors)
        print(f"   Edge (1,3) common neighbors: {sorted(common_13)}")
        print(f"   Expected: should contain 2 = {'✅' if 2 in common_13 else '❌'}")
    
    if num_nodes >= 5:
        print("\n� Checking triangle (3,4,5):")
        common_34 = network.nodes[3].direct_neighbors.intersection(network.nodes[4].direct_neighbors)
        common_45 = network.nodes[4].direct_neighbors.intersection(network.nodes[5].direct_neighbors)  
        common_35 = network.nodes[3].direct_neighbors.intersection(network.nodes[5].direct_neighbors)
        
        print(f"   Edge (3,4) common neighbors: {sorted(common_34)} (should contain 5)")
        print(f"   Edge (4,5) common neighbors: {sorted(common_45)} (should contain 3)")
        print(f"   Edge (3,5) common neighbors: {sorted(common_35)} (should contain 4)")
    
    return network

def create_complete_graph(num_nodes: int = 5) -> DistributedNetwork:
    """
    Create a COMPLETE GRAPH structure within 1-unit radius initial workspace
    Every node connects to every other node = MAXIMUM triangles
    Initial positions are clustered in center, then robots can spread out in 10x10 workspace
    """
    print(f"\n🔺 CREATING COMPLETE GRAPH (Maximum Triangles)")
    print(f"   Nodes: {num_nodes}")
    print("   Initial radius: 1 unit (clustered start)")
    print("   Workspace: 10x10 units (robots can spread out)")
    print("   Structure: Every node connects to every other node")
    print("   Result: MAXIMUM possible triangles!")
    print("=" * 70)
    
    # Create network
    network = DistributedNetwork(create_default_graph=False)
    
    # Create nodes in a circle pattern within 1-unit radius (clustered start)
    for i in range(1, num_nodes + 1):
        angle = 2 * np.pi * (i - 1) / num_nodes
        radius = 0.8  # Start clustered within 1-unit radius
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        network.nodes[i] = Node(i, np.array([x, y]))
    
    print("🔗 Creating COMPLETE GRAPH - connecting every node to every other node...")
    
    # Create COMPLETE GRAPH: connect every node to every other node
    total_edges = 0
    for i in range(1, num_nodes + 1):
        for j in range(i + 1, num_nodes + 1):  # Only connect each pair once
            # Add edge (i,j)
            network.nodes[i].add_direct_neighbor(j)
            network.nodes[j].add_direct_neighbor(i)
            total_edges += 1
            
            if total_edges <= 10:  # Show first few connections
                print(f"   ✅ Connected nodes {i} ↔ {j}")
    
    if total_edges > 10:
        print(f"   ... (and {total_edges - 10} more connections)")
    
    # Initialize algorithm
    for node in network.nodes.values():
        node.initialize_algorithm(len(network.nodes))
    
    print(f"\n✅ COMPLETE GRAPH CREATED!")
    print(f"   Total nodes: {num_nodes}")
    print(f"   Total edges: {total_edges}")
    print(f"   Expected edges: {num_nodes * (num_nodes - 1) // 2}")
    
    # Calculate expected triangles
    expected_triangles = num_nodes * (num_nodes - 1) * (num_nodes - 2) // 6
    print(f"   Expected triangles: {expected_triangles}")
    
    print(f"\n🔍 TRIANGLE VERIFICATION (Sample):")
    
    # Check first few edges for triangles
    sample_edges = [(1, 2), (1, 3), (2, 3)] if num_nodes >= 3 else []
    
    for node1_id, node2_id in sample_edges:
        if node1_id in network.nodes and node2_id in network.nodes:
            node1 = network.nodes[node1_id]
            node2 = network.nodes[node2_id]
            common_neighbors = node1.direct_neighbors.intersection(node2.direct_neighbors)
            
            print(f"   Edge ({node1_id},{node2_id}):")
            print(f"     Node {node1_id} neighbors: {sorted(node1.direct_neighbors)}")
            print(f"     Node {node2_id} neighbors: {sorted(node2.direct_neighbors)}")
            print(f"     Common neighbors: {sorted(common_neighbors)}")
            print(f"     Triangle count: {len(common_neighbors)}")
            
            if len(common_neighbors) > 0:
                redundancy = len(common_neighbors) / min(len(node1.direct_neighbors), len(node2.direct_neighbors))
                priority = redundancy * (1 + len(common_neighbors))
                print(f"     🎯 Expected Priority: {priority:.4f} (HIGH!)")
            print()
    
    return network

def create_triangle_rich_graph(num_nodes: int = 8) -> DistributedNetwork:
    """
    Create a graph rich in triangles for testing the Triangle Participation algorithm
    
    This creates multiple overlapping triangular structures:
    - Central hub with surrounding triangles
    - Interconnected triangular clusters
    - High triangle density for algorithm testing
    """
    print(f"\n🔺 CREATING TRIANGLE-RICH GRAPH")
    print(f"   Nodes: {num_nodes}")
    print(f"   Strategy: Multiple overlapping triangles")
    print("=" * 50)
    
    # Create network without default graph
    network = DistributedNetwork(create_default_graph=False)
    
    # Create nodes with positions for visualization
    positions = {}
    for i in range(1, num_nodes + 1):
        angle = 2 * np.pi * (i - 1) / num_nodes
        x = 2 * np.cos(angle) + np.random.normal(0, 0.1)
        y = 2 * np.sin(angle) + np.random.normal(0, 0.1)
        positions[i] = np.array([x, y])
        network.nodes[i] = Node(i, positions[i])
    
    # Create triangle-rich edge patterns
    edges = set()
    
    # Strategy: Create overlapping triangular clusters
    
    # Step 1: Create a central hub with first triangle cluster
    center = 1
    cluster_size = min(5, num_nodes - 1)  # Adaptive cluster size
    
    # Connect center to cluster_size nodes
    for i in range(2, min(cluster_size + 2, num_nodes + 1)):
        edges.add((center, i))
    
    # Create triangles around the center
    for i in range(2, min(cluster_size + 1, num_nodes)):
        next_node = i + 1 if i + 1 <= min(cluster_size + 1, num_nodes) else 2
        edges.add((i, next_node))
    
    # Step 2: Create additional triangular clusters for remaining nodes
    remaining_nodes = list(range(cluster_size + 2, num_nodes + 1))
    
    while len(remaining_nodes) >= 3:
        # Take 3 nodes and form a triangle
        triangle_nodes = remaining_nodes[:3]
        remaining_nodes = remaining_nodes[3:]
        
        # Form triangle
        edges.add((triangle_nodes[0], triangle_nodes[1]))
        edges.add((triangle_nodes[1], triangle_nodes[2]))
        edges.add((triangle_nodes[2], triangle_nodes[0]))
        
        # Connect this triangle to existing network (to maintain connectivity)
        # Connect to a random node from existing connected components
        existing_nodes = list(range(1, min(cluster_size + 2, num_nodes + 1)))
        bridge_target = random.choice(existing_nodes)
        edges.add((triangle_nodes[0], bridge_target))
    
    # Step 3: Handle remaining 1-2 nodes if any
    if len(remaining_nodes) > 0:
        for node in remaining_nodes:
            # Connect each remaining node to at least 2 existing nodes
            existing_nodes = list(range(1, node))
            if len(existing_nodes) >= 2:
                # Connect to 2 random existing nodes to create potential triangles
                targets = random.sample(existing_nodes, min(2, len(existing_nodes)))
                for target in targets:
                    edges.add((node, target))
            else:
                # Fallback: connect to node 1
                edges.add((node, 1))
    
    # Step 4: Add extra edges to increase triangle density
    extra_triangles = min(num_nodes // 3, 5)  # Add up to 5 extra triangular connections
    
    for _ in range(extra_triangles):
        # Pick 3 random nodes and try to form a triangle
        if num_nodes >= 3:
            triangle_candidates = random.sample(range(1, num_nodes + 1), 3)
            node_a, node_b, node_c = triangle_candidates
            
            # Add edges if they don't already exist
            potential_edges = [(node_a, node_b), (node_b, node_c), (node_c, node_a)]
            for edge in potential_edges:
                normalized_edge = tuple(sorted(edge))
                if normalized_edge not in edges:
                    edges.add(normalized_edge)
    
    # Add edges to the network
    for edge in edges:
        node1, node2 = edge
        if node1 in network.nodes and node2 in network.nodes:
            network.nodes[node1].add_direct_neighbor(node2)
            network.nodes[node2].add_direct_neighbor(node1)
    
    # Initialize algorithm for all nodes
    for node in network.nodes.values():
        node.initialize_algorithm(len(network.nodes))
    
    print(f"🔺 Triangle-rich graph created!")
    print(f"   Total edges: {len(edges)}")
    
    # Count triangles for verification
    triangle_count = 0
    for node1 in network.nodes:
        for node2 in network.nodes[node1].direct_neighbors:
            if node1 < node2:
                common = network.nodes[node1].direct_neighbors.intersection(
                    network.nodes[node2].direct_neighbors
                )
                triangle_count += len(common)
    
    print(f"   Triangle count: {triangle_count} triangles detected!")
    print(f"   Expected high priority scores for triangle edges")
    
    # Show the structure
    print("\n📊 Graph Structure:")
    for node_id in sorted(network.nodes.keys()):
        neighbors = sorted(list(network.nodes[node_id].direct_neighbors))
        degree = len(neighbors)
        print(f"   Node {node_id}: degree={degree}, neighbors = {neighbors}")
    
    # Verify all nodes are connected
    print("\n🔍 Connectivity Check:")
    isolated_nodes = []
    for node_id, node in network.nodes.items():
        if len(node.direct_neighbors) == 0:
            isolated_nodes.append(node_id)
    
    if isolated_nodes:
        print(f"   ⚠️  WARNING: Isolated nodes detected: {isolated_nodes}")
        # Connect isolated nodes to node 1
        for isolated in isolated_nodes:
            network.nodes[isolated].add_direct_neighbor(1)
            network.nodes[1].add_direct_neighbor(isolated)
            print(f"   🔗 Connected isolated node {isolated} to node 1")
    else:
        print(f"   ✅ All {num_nodes} nodes have at least one connection")
    
    return network

def compare_with_networkx(network: DistributedNetwork):
    """Compare our results with NetworkX bridge detection"""
    import networkx as nx
    
    print("\n" + "=" * 60)
    print("COMPARISON WITH NETWORKX BRIDGE DETECTION")
    print("=" * 60)
    
    # Create NetworkX graph
    G = nx.Graph()
    for node_id in network.nodes.keys():
        G.add_node(node_id)
    
    all_edges = []
    for node_id, node in network.nodes.items():
        for neighbor_id in node.direct_neighbors:
            if node_id < neighbor_id:
                edge = (node_id, neighbor_id)
                G.add_edge(node_id, neighbor_id)
                all_edges.append(edge)
    
    # Find bridges using NetworkX
    nx_bridges = list(nx.bridges(G))
    nx_bridges_normalized = [tuple(sorted(bridge)) for bridge in nx_bridges]
    
    # Get our critical edges
    critical_edges, _ = network.run_critical_edge_detection()
    our_critical_edges = [tuple(sorted(edge)) for edge in critical_edges]
    
    print(f"NetworkX bridges: {sorted(nx_bridges_normalized)}")
    print(f"Our critical edges: {sorted(our_critical_edges)}")
    
    # Compare results
    nx_set = set(nx_bridges_normalized)
    our_set = set(our_critical_edges)
    
    if nx_set == our_set:
        print("✅ PERFECT MATCH! Our algorithm correctly identified all bridges.")
    else:
        print("❌ MISMATCH detected:")
        print(f"   Bridges we missed: {sorted(list(nx_set - our_set))}")
        print(f"   False positives: {sorted(list(our_set - nx_set))}")
    
    return len(nx_set) == len(our_set) and nx_set == our_set

def get_user_input():
    """Get user input for number of nodes and graph type"""
    print("\n" + "=" * 60)
    print("INTERACTIVE CRITICAL EDGE DETECTION")
    print("=" * 60)
    
    # First choose between random and hardcoded graph
    print("Choose graph option:")
    print("1. Create random graph with customizable structure")
    print("2. Use specific 8-node hardcoded graph")
    
    while True:
        try:
            graph_option = int(input("Enter your choice [1-2]: "))
            if 1 <= graph_option <= 2:
                break
            else:
                print("Please enter 1 or 2.")
        except ValueError:
            print("Please enter a valid integer.")
    
    if graph_option == 1:
        # Get number of nodes
        while True:
            try:
                num_nodes = int(input("Enter the number of robots (nodes) [3-20]: "))
                if 3 <= num_nodes <= 20:
                    break
                else:
                    print("Please enter a number between 3 and 20.")
            except ValueError:
                print("Please enter a valid integer.")
        
        # Get graph type preference
        print("\nChoose graph structure:")
        print("1. Path-like (linear connections with few shortcuts)")
        print("2. Star-like (one central hub with satellites)")
        print("3. Tree-like (hierarchical branching structure)")
        print("4. Cycle-like (circular with some chords)")
        print("5. Random (let the algorithm choose)")
        print("6. Triangle-rich (multiple overlapping triangles)")
        print("7. Complete Graph (every node connected to every other - MAXIMUM triangles!)")
        
        while True:
            try:
                choice = int(input("Enter your choice [1-7]: "))
                if 1 <= choice <= 7:
                    break
                else:
                    print("Please enter a number between 1 and 7.")
            except ValueError:
                print("Please enter a valid integer.")
        
        graph_types = {
            1: ('path', 'Path-like structure'),
            2: ('star', 'Star-like structure'),
            3: ('tree', 'Tree-like structure'),
            4: ('cycle_plus', 'Cycle-like structure'),
            5: ('random', 'Random structure'),
            6: ('triangle_rich', 'Triangle-rich structure with multiple overlapping triangles'),
            7: ('complete', 'Complete graph with maximum triangles - like your example image!')
        }
        
        graph_type, description = graph_types[choice]
        
        # Get animation method preference
        print(f"\nChoose algorithm animation:")
        print("1. Traditional (1-by-1 edge removal) - Slower but step-by-step visualization")
        print("2. Simultaneous (Batch edge removal) - Much faster for large networks")
        print("3. Goal-Seeking (Robots move to goal while preserving connectivity) - NEW!")
        
        while True:
            try:
                pruning_choice = int(input("Enter your choice [1-3]: "))
                if 1 <= pruning_choice <= 3:
                    break
                else:
                    print("Please enter 1, 2, or 3.")
            except ValueError:
                print("Please enter a valid integer.")
        
        use_simultaneous_pruning = (pruning_choice == 2)
        use_goal_seeking = (pruning_choice == 3)
        
        if use_goal_seeking:
            pruning_method = "Goal-Seeking with Connectivity Preservation"
        elif use_simultaneous_pruning:
            pruning_method = "Simultaneous Batch Pruning"
        else:
            pruning_method = "Traditional 1-by-1 Pruning"
            
        print(f"Selected: {pruning_method}")
        
        # Calculate edge probability based on number of nodes
        if num_nodes <= 5:
            edge_prob = 0.3
        elif num_nodes <= 8:
            edge_prob = 0.2
        elif num_nodes <= 12:
            edge_prob = 0.15
        else:
            edge_prob = 0.1
    else:
        # Use hardcoded 8-node graph
        num_nodes = 8
        graph_type = 'hardcoded'
        description = 'Hardcoded 8-node structure'
        edge_prob = 0.0  # Not used for hardcoded graph
        # Set defaults for pruning options (no pruning/goal-seeking by default)
        use_simultaneous_pruning = False
        use_goal_seeking = False
        pruning_method = "Traditional 1-by-1 Pruning"
    
    return num_nodes, graph_type, description, edge_prob, graph_option, use_simultaneous_pruning, use_goal_seeking, pruning_method

def create_hardcoded_8_node_network(enable_underwater=True):
    """Create the specific 8-node hardcoded graph for real-time simulation with underwater physics"""
    network = DistributedNetwork(create_default_graph=False, enable_underwater_physics=enable_underwater)
    
    print(f"\n{'UNDERWATER' if enable_underwater else 'SURFACE'} 8-Node Network Creation")
    
    # Define node positions (adjusted for underwater scenario)
    positions = {
        1: np.array([1.0, 2.0]),
        2: np.array([0.0, 1.0]),
        3: np.array([1.0, 0.0]),
        4: np.array([2.0, 1.0]),
        5: np.array([3.0, 1.0]),
        6: np.array([4.0, 1.0]),
        7: np.array([4.0, 0.0]),
        8: np.array([5.0, 1.0])
    }
    
    # Create nodes
    for node_id, pos in positions.items():
        network.nodes[node_id] = Node(node_id, pos)
    
    # Define edges
    edges = [
        (1, 2), (1, 4),
        (2, 3), (2, 4),
        (3, 4),
        (4, 5),
        (5, 6), (5, 7),
        (6, 7), (6, 8),
    ]
    
    # Add edges to network
    for node1, node2 in edges:
        network.nodes[node1].add_direct_neighbor(node2)
        network.nodes[node2].add_direct_neighbor(node1)
    
    print(f"Created hardcoded 8-node network with {len(edges)} edges")
    return network

def main():
    """Main function with menu-driven interface"""
    import random
    import time
    
    while True:
        print("\n" + "=" * 70)
        print("DISTRIBUTED CRITICAL EDGE DETECTION SYSTEM")
        print("Real-time Critical Edge Detection for Multi-Robot Networks")
        print("=" * 70)
        print("Choose simulation type:")
        print("1. Basic critical edge detection")
        print("2. Step-by-step distributed algorithm")
        print("3. Random graph testing")
        print("4. NetworkX comparison")
        print("5. Final visualization")
        print("6. Real-time pruning simulation (NEW!)")
        print("7. Exit")
        print("8. Test edge optimization (DEBUG)")
        
        try:
            choice = input("\nEnter your choice (1-8): ").strip()
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        
        if choice == '7':
            print("Thank you for using the Critical Edge Detection tool!")
            break
            
        if choice == '8':
            # Test edge optimization with a network that has close robots
            print("\n🧪 Testing Edge Optimization with Very Close Robot Positions...")
            network = DistributedNetwork(create_default_graph=False)
            
            # Create extreme test case: robots 4 and 5 very close (0.3 units)
            # Robot 4 currently connected to distant robot 2 (2.5 units away)
            # Algorithm should swap 4-2 edge for 4-5 edge
            positions = {
                1: np.array([1.0, 2.0]),  # Position 1
                2: np.array([0.0, 1.0]),  # Position 2 (far from 4) 
                3: np.array([1.0, 0.0]),  # Position 3
                4: np.array([2.0, 1.0]),  # Position 4
                5: np.array([2.3, 1.0]),  # Position 5 (VERY close to 4: distance=0.3)
                6: np.array([4.0, 1.0]),  # Position 6
                7: np.array([4.0, 0.0]),  # Position 7
                8: np.array([5.0, 1.0]),  # Position 8
            }
            
            # Create nodes
            for node_id, pos in positions.items():
                network.nodes[node_id] = Node(node_id, pos)
            
            # Create simple test: robot 4 has bridge robot 3 connecting to robot 5
            # Path: Robot 4 -> Robot 3 -> Robot 5 (2 hops)
            # Robot 4 also connected to distant robot 2 (long edge to be swapped)  
            edges = [
                (1, 2),              # Robot 1 connects to 2
                (2, 4),              # Robot 2 connects to 4 (LONG connection: distance=2.0)
                (3, 4), (3, 5),      # Robot 3 bridges 4 and 5 (4->3->5 path, 2 hops)
                (5, 6), (6, 7), (7, 8) # Other connections
            ]
            
            # Add edges
            for node1, node2 in edges:
                network.nodes[node1].add_direct_neighbor(node2)
                network.nodes[node2].add_direct_neighbor(node1)
            
            print("Initial test network:")
            network.print_graph_structure()
            
            # Show distances for key robots
            pos4 = network.nodes[4].position
            pos2 = network.nodes[2].position
            pos5 = network.nodes[5].position
            
            dist_4_2 = np.linalg.norm(pos4 - pos2)
            dist_4_5 = np.linalg.norm(pos4 - pos5)
            
            print(f"\n📏 Key distances:")
            print(f"   Robot 4 ↔ Robot 2 (current): {dist_4_2:.3f}")
            print(f"   Robot 4 ↔ Robot 5 (potential): {dist_4_5:.3f}")
            print(f"   Expected swap: 4-5 distance < 4-2 distance? {dist_4_5 < dist_4_2}")
            
            print("\nTesting edge optimization with ANIMATION...")
            
            # Show initial network visually
            plt.figure(figsize=(12, 8))
            plt.subplot(1, 2, 1)
            plot_network_state(network, "Initial Network (Before Edge Optimization)")
            
            # Run the edge optimization
            print("🔄 Running Edge Optimization...")
            changes = network.run_decentralized_edge_optimization()
            
            # Show final network visually  
            plt.subplot(1, 2, 2)
            plot_network_state(network, "Final Network (After Edge Optimization)")
            
            plt.tight_layout()
            plt.show()
            
            print(f"\n✅ Edge optimization complete! Changes made: {changes}")
            
            print("\nFinal test network:")
            network.print_graph_structure()
            continue
        
        if choice in ['1', '2', '3', '4', '5', '6']:
            # Option 6: Real-time pruning simulation
            if choice == '6':
                # Get user input
                num_nodes, preferred_type, description, edge_prob, graph_option, use_simultaneous_pruning, use_goal_seeking, pruning_method = get_user_input()
                
                # Use current time as seed for true randomness
                seed = int(time.time())
                random.seed(seed)
                np.random.seed(seed)
                
                print(f"\nGenerating network with:")
                print(f"  - Nodes (robots): {num_nodes}")
                print(f"  - Structure: {description}")
                print(f"  - Pruning method: {pruning_method}")
                if graph_option == 1:
                    print(f"  - Edge probability: {edge_prob}")
                print(f"  - Random seed: {seed}")
                
                # Create network
                if graph_option == 2:
                    # Use hardcoded 8-node graph
                    network = create_hardcoded_8_node_network(enable_underwater=True)
                else:
                    # Create random graph
                    network = create_random_graph(num_nodes, edge_prob, preferred_type, enable_underwater=True)
                
                # Run algorithm based on user choice
                if use_goal_seeking:
                    # Goal-seeking behavior
                    # Randomly generate goal within [0, 10] x [0, 10]
                    goal_x = np.random.uniform(0, 10)
                    goal_y = np.random.uniform(0, 10)
                    print(f"\n🎯 Starting goal-seeking simulation...")
                    print(f"Randomly generated goal location: ({goal_x:.2f}, {goal_y:.2f})")
                    print("Robots will move toward goal while preserving connectivity!")
                    # COMMENTED OUT: Don't prune before goal-seeking to maintain all robots
                    # network.run_n_step_algorithm(n_steps=min(num_nodes + 2, 15))
                    network.goal_seeking_simulation(goal_x, goal_y)
                elif use_simultaneous_pruning:
                    # Use new simultaneous pruning method
                    print("\n🚀 Starting simultaneous batch pruning...")
                    network.run_n_step_algorithm(n_steps=min(num_nodes + 2, 15))
                    network.iterative_pruning_simultaneous()
                else:
                    # Use existing real-time simulation (1-by-1)
                    print("\n🎬 Starting real-time pruning simulation (1-by-1)...")
                    print("This will show continuous edge removal in a single window!")
                    network.real_time_pruning_simulation()
                
                print(f"\n✅ {pruning_method} completed!")
                continue_simulation = input("\nWould you like to run another simulation? (y/n): ").lower().strip()
                if continue_simulation != 'y':
                    continue
            
            # All other options (1-5) - existing functionality
            else:
                # Get user input
                num_nodes, preferred_type, description, edge_prob, _ = get_user_input()
                
                # Use current time as seed for true randomness
                seed = int(time.time())
                random.seed(seed)
                np.random.seed(seed)
                
                print(f"\nGenerating network with:")
                print(f"  - Nodes (robots): {num_nodes}")
                print(f"  - Structure: {description}")
                if graph_option == 1:
                    print(f"  - Edge probability: {edge_prob}")
                print(f"  - Random seed: {seed}")
                
                # Create network with user preferences
                network = create_random_graph(num_nodes, edge_prob, preferred_type, enable_underwater=True)
                
                # Run distributed algorithms
                print(f"\nRunning distributed algorithms...")
                network.run_n_step_algorithm(n_steps=min(num_nodes + 2, 15))
                
                # Execute based on choice
                if choice == '1':
                    # Basic critical edge detection
                    network.print_final_results()
                    match = compare_with_networkx(network)
                    if match:
                        print("✅ Test PASSED! Our algorithm correctly identified all critical edges.")
                    else:
                        print("❌ Test FAILED! There's a mismatch with NetworkX.")
                
                elif choice == '2':
                    # Step-by-step algorithm
                    network.print_step_by_step_algorithm()
                
                elif choice == '3':
                    # Random graph testing
                    network.print_final_results()
                    network.visualize_network("Random Graph Structure", [])
                
                elif choice == '4':
                    # NetworkX comparison
                    network.print_final_results()
                    match = compare_with_networkx(network)
                    
                elif choice == '5':
                    # Final visualization
                    critical_edges, _ = network.run_critical_edge_detection()
                    network.visualize_network("Critical Edge Detection Results", critical_edges)
                
                continue_simulation = input("\nWould you like to run another simulation? (y/n): ").lower().strip()
                if continue_simulation != 'y':
                    continue
        
        else:
            print("Invalid choice. Please select 1-7.")
            continue
    
    return None

def plot_network_state(network, title):
    """Plot the current state of the network with robot positions and connections"""
    positions = {}
    for node_id, node in network.nodes.items():
        positions[node_id] = node.position
    
    # Plot robot positions
    for node_id, pos in positions.items():
        plt.scatter(pos[0], pos[1], s=200, c='blue', alpha=0.7)
        plt.text(pos[0]+0.1, pos[1]+0.1, f'R{node_id}', fontsize=12, fontweight='bold')
    
    # Plot connections
    for node_id, node in network.nodes.items():
        pos1 = positions[node_id]
        for neighbor_id in node.direct_neighbors:
            if neighbor_id > node_id:  # Avoid duplicate lines
                pos2 = positions[neighbor_id]
                distance = np.linalg.norm(pos1 - pos2)
                # Color-code by distance: red=long, green=short
                color = 'red' if distance > 1.5 else 'green' if distance < 1.0 else 'orange'
                plt.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], 
                        color=color, linewidth=2, alpha=0.7)
                
                # Add distance labels
                mid_x, mid_y = (pos1[0] + pos2[0])/2, (pos1[1] + pos2[1])/2
                plt.text(mid_x, mid_y, f'{distance:.2f}', fontsize=9, 
                        bbox=dict(boxstyle="round,pad=0.2", facecolor='white', alpha=0.8))
    
    plt.title(title, fontsize=14, fontweight='bold')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')

if __name__ == "__main__":
    main()
