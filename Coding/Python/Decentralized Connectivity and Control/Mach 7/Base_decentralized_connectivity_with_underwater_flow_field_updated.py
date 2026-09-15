"""
Distributed Network Algorithms Implementation
Implementation of distributed algorithms for neighbor structure identification 
and connectivity assurance as described in the paper.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
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

# ============================================================================
# CENTRALIZED MLCCST CBF INTEGRATION
# ============================================================================

# Centralized algorithm parameters (from original file)
CENTRALIZED_NUM_ROBOTS = 40
CENTRALIZED_TIME_STEP = 0.1
CENTRALIZED_MAX_SPEED = 0.25  # Reduced for better connectivity
CENTRALIZED_BROWNIAN_INTENSITY = 0.02  # Reduced
CENTRALIZED_COMM_RADIUS = 1.0  # Increased for better connectivity
CENTRALIZED_MIN_SAFETY_DISTANCE = 0.15
CENTRALIZED_MAX_ITERATIONS = 550
CENTRALIZED_BROWNIAN_PHASE_DURATION = 50  # Reduced
CENTRALIZED_RETRACTION_START_ITERATION = 400
CENTRALIZED_FORMATION_RADIUS = 0.8
CENTRALIZED_GOAL_AREA_RADIUS = 0.4
CENTRALIZED_GOAL_DETECTION_RADIUS = 0.5
CENTRALIZED_RETRACTION_RADIUS = 1.5

# CLF-CBF Parameters - More goal-oriented balance
CLF_GAIN = 1.2  # Increased for stronger goal attraction
CBF_SAFETY_GAIN = 2.0
CBF_CONNECTIVITY_GAIN = 2.5  # Reduced from 4.0 for less aggressive connectivity
CONNECTIVITY_THRESHOLD = 0.85  # Reduced from 0.9 for more flexibility

# Enhanced expansion parameters
MIN_ORIGIN_NEIGHBORS = 2
MAX_ORIGIN_NEIGHBORS = 3
EXPANSION_RATE = 0.5
OPTIMAL_CHAIN_SPACING = CENTRALIZED_COMM_RADIUS * 0.75
MAX_ROBOTS_PER_CHAIN = 12
MIN_ROBOTS_TO_START_EXPANSION = 6
CONNECTIVITY_CHECK_INTERVAL = 10
EXPANSION_AGGRESSIVENESS = 0.8

# Goal balancing parameters
GOAL_BALANCE_THRESHOLD = 3
FORCED_BALANCE_INTERVAL = 20

class CentralizedRobot:
    """Robot class for centralized MLCCST CBF algorithm"""
    def __init__(self, robot_id, initial_position):
        self.id = robot_id
        self.position = np.array(initial_position, dtype=float)
        self.velocity = np.zeros(2)
        self.role = "follower"
        self.parent = None
        self.children = []
        self.is_connected_to_origin = False
        self.distance_to_goal1 = float('inf')
        self.distance_to_goal2 = float('inf')
        self.distance_to_origin = float('inf')
        self.assigned_goal = None
        self.should_stay_near_origin = False
        self.chain_distance = 0
        self.is_chain_leader = False
        self.chain_position = -1
        self.is_chain_active = False
        self.expansion_weight = 1.0
        self.connectivity_priority = 0
        self.last_connected_time = 0
        self.is_expansion_candidate = False
        self.preferred_goal = None
        
        # New attributes for goal area behavior
        self.is_at_goal = False
        self.goal_area_position = -1  # Position index in goal area formation
        self.goal_arrival_time = 0
        
        # CLF-CBF specific attributes
        self.target_position = None
        self.lyapunov_value = 0.0
        self.barrier_values = []

        # Goal preference tracking
        self.goal_preference_score = 0.0  # Positive = Goal 1, Negative = Goal 2
        self.assignment_locked = False  # Prevent reassignment once in chain
    
    def apply_brownian_motion(self, intensity):
        random_force = np.random.randn(2) * intensity
        self.velocity += random_force
    
    def apply_control_force(self, force):
        self.velocity += force
        speed = np.linalg.norm(self.velocity)
        if speed > CENTRALIZED_MAX_SPEED:
            self.velocity = self.velocity * (CENTRALIZED_MAX_SPEED / speed)
    
    def update_position(self):
        self.position += self.velocity * CENTRALIZED_TIME_STEP
        self.velocity *= 0.8
    
    def distance_to(self, other_robot):
        return np.linalg.norm(self.position - other_robot.position)
    
    def distance_to_point(self, point):
        return np.linalg.norm(self.position - point)

class CLFCBFController:
    """Control Lyapunov Function and Control Barrier Function controller"""
    def __init__(self, obstacles=None):
        self.obstacles = obstacles or []
    
    def control_lyapunov_function(self, robot, target_position):
        """CLF computation with goal area awareness"""
        if target_position is None:
            return 0.0, np.zeros(2)
        
        error = robot.position - target_position
        lyapunov_value = 0.5 * np.dot(error, error)
        lyapunov_gradient = error
        
        robot.lyapunov_value = lyapunov_value
        return lyapunov_value, lyapunov_gradient
    
    def control_barrier_function_safety(self, robot, other_robot):
        """CBF for collision avoidance"""
        distance_vector = robot.position - other_robot.position
        distance = np.linalg.norm(distance_vector)
        
        safety_distance = CENTRALIZED_MIN_SAFETY_DISTANCE * 2
        barrier_value = distance**2 - safety_distance**2
        
        if distance > 1e-6:
            barrier_gradient = 2 * distance_vector
        else:
            barrier_gradient = np.random.randn(2) * 0.1
        
        return barrier_value, barrier_gradient
    
    def control_barrier_function_connectivity(self, robot, parent_robot):
        """Enhanced CBF for connectivity"""
        if parent_robot is None:
            return 1.0, np.zeros(2)
        
        distance_vector = robot.position - parent_robot.position
        distance = np.linalg.norm(distance_vector)
        
        # More flexible for robots at goals
        if robot.is_at_goal:
            max_distance = CENTRALIZED_COMM_RADIUS * 0.95  # Very flexible for goal area robots
        elif robot.is_expansion_candidate or robot.expansion_weight > 1.5:
            max_distance = CENTRALIZED_COMM_RADIUS * 0.9
        else:
            max_distance = CENTRALIZED_COMM_RADIUS * CONNECTIVITY_THRESHOLD
            
        barrier_value = max_distance**2 - distance**2
        
        if distance > 1e-6:
            barrier_gradient = -2 * distance_vector
        else:
            barrier_gradient = np.zeros(2)
        
        return barrier_value, barrier_gradient
    
    def compute_clf_cbf_control(self, robot, target_position, other_robots, parent_robot):
        """Enhanced CLF-CBF control computation"""
        # CLF computation
        clf_value, clf_grad = self.control_lyapunov_function(robot, target_position)
        
        # Dynamic CLF gain based on goal proximity and role
        if not robot.is_connected_to_origin:
            clf_gain = CLF_GAIN * 0.1
        elif robot.is_at_goal:
            clf_gain = CLF_GAIN * 0.4  # Moderate for goal area positioning
        elif robot.is_chain_leader:
            distance_to_goal = robot.distance_to_goal1
            
            # More aggressive CLF gain for leaders
            if distance_to_goal < CENTRALIZED_GOAL_DETECTION_RADIUS:
                clf_gain = CLF_GAIN * 1.2  # Strong near goal
            elif distance_to_goal < OPTIMAL_CHAIN_SPACING:
                clf_gain = CLF_GAIN * 1.8  # Very strong in approach
            else:
                clf_gain = CLF_GAIN * 2.2  # Maximum for distant leaders
        elif robot.is_chain_active:
            clf_gain = CLF_GAIN * 1.4  # Strong for chain members
        else:
            clf_gain = CLF_GAIN * 0.6  # Moderate for followers
            
        control = -clf_gain * clf_grad
        
        # CONNECTIVITY CBF - reduced intervention
        if parent_robot is not None:
            cbf_val, cbf_grad = self.control_barrier_function_connectivity(robot, parent_robot)
            
            # Less aggressive connectivity intervention
            if robot.is_at_goal:
                intervention_threshold = 0.05  # Very lenient for goal robots
                critical_threshold = 0.01
                connectivity_gain = CBF_CONNECTIVITY_GAIN * 0.3
            elif robot.is_chain_leader:
                goal_distance = robot.distance_to_goal1
                
                if goal_distance < CENTRALIZED_GOAL_DETECTION_RADIUS:
                    intervention_threshold = 0.06  # Lenient near goal
                    critical_threshold = 0.02
                    connectivity_gain = CBF_CONNECTIVITY_GAIN * 0.4
                else:
                    intervention_threshold = 0.08  # More lenient
                    critical_threshold = 0.03
                    connectivity_gain = CBF_CONNECTIVITY_GAIN * 0.5
            elif robot.is_chain_active:
                intervention_threshold = 0.1  # Lenient for chain members
                critical_threshold = 0.04
                connectivity_gain = CBF_CONNECTIVITY_GAIN * 0.6
            else:
                intervention_threshold = 0.15  # Most lenient for followers
                critical_threshold = 0.06
                connectivity_gain = CBF_CONNECTIVITY_GAIN * 0.7
            
            if cbf_val < intervention_threshold:
                grad_norm = np.linalg.norm(cbf_grad)
                if grad_norm > 1e-6:
                    connectivity_correction = connectivity_gain * (-cbf_val) * cbf_grad / grad_norm
                    control += connectivity_correction
                    
                    # Only override CLF in critical situations
                    if cbf_val < critical_threshold:
                        if robot.is_chain_leader:
                            # Blend rather than override for leaders
                            control = 0.6 * control + 0.4 * connectivity_correction
                        else:
                            # Stronger override for non-leaders
                            control = 0.4 * control + 0.6 * connectivity_correction
        
        # Safety CBF (robot-robot collision avoidance)
        for other_robot in other_robots:
            if other_robot.id != robot.id:
                cbf_val, cbf_grad = self.control_barrier_function_safety(robot, other_robot)
                if cbf_val < 0.1:
                    grad_norm = np.linalg.norm(cbf_grad)
                    if grad_norm > 1e-6:
                        safety_correction = CBF_SAFETY_GAIN * (-cbf_val) * cbf_grad / grad_norm
                        control += safety_correction
        
        # Limit control magnitude
        control_magnitude = np.linalg.norm(control)
        if control_magnitude > CENTRALIZED_MAX_SPEED:
            control = control * (CENTRALIZED_MAX_SPEED / control_magnitude)
        
        return control

class Obstacle:
    """Simple obstacle class for centralized algorithm"""
    def __init__(self, position, radius):
        self.position = np.array(position, dtype=float)
        self.radius = radius

class SingleGoalMLCCSTController:
    """Single-Goal Multi-Level Chain Coordination with CLF-CBF control"""
    def __init__(self, initial_positions, goal, obstacles=None, underwater_env=None):
        # Initialize robots
        self.robots = [CentralizedRobot(i, pos) for i, pos in enumerate(initial_positions)]
        self.num_robots = len(self.robots)
        self.goal = np.array(goal, dtype=float)
        self.obstacles = obstacles or []
        self.origin = np.mean(initial_positions, axis=0)
        
        # CLF-CBF Controller
        self.controller = CLFCBFController(self.obstacles)
        
        # Underwater environment
        self.underwater_env = underwater_env
        
        # Goal area formation (circular formation around goal)
        self.goal_formation_positions = self._create_goal_formation(self.goal)
        
        # State variables
        self.goal_assignment_count = 0
        self.step_count = 0
        
        # Metrics tracking
        self.connectivity_history = []
        self.position_history = [[] for _ in range(self.num_robots)]
        
        # Initialize robot parameters
        for robot in self.robots:
            robot.distance_to_origin = np.linalg.norm(robot.position - self.origin)
            robot.distance_to_goal1 = np.linalg.norm(robot.position - self.goal)  # Use goal1 field for single goal
            robot.assigned_goal = 1  # All robots assigned to single goal
    
    def _create_goal_formation(self, goal_center, num_positions=12):
        """Create circular formation positions around goal"""
        positions = []
        for i in range(num_positions):
            angle = 2 * np.pi * i / num_positions
            offset = CENTRALIZED_GOAL_AREA_RADIUS * np.array([np.cos(angle), np.sin(angle)])
            positions.append(goal_center + offset)
        return positions
    
    def _build_tree_structure(self):
        """Build tree structure connecting robots to origin with stronger connectivity"""
        # Reset tree structure
        for robot in self.robots:
            robot.parent = None
            robot.children.clear()
            robot.is_connected_to_origin = False
            robot.distance_to_origin = float('inf')
        
        # Find robots near origin - more generous radius
        origin_robots = []
        for robot in self.robots:
            dist = np.linalg.norm(robot.position - self.origin)
            if dist < CENTRALIZED_COMM_RADIUS * 1.2:  # More generous
                robot.is_connected_to_origin = True
                robot.distance_to_origin = dist
                origin_robots.append(robot)
        
        # Use BFS to build tree from origin robots
        queue = origin_robots.copy()
        processed = set(robot.id for robot in origin_robots)
        
        while queue:
            current_robot = queue.pop(0)
            
            # Find unconnected robots in communication range
            for other_robot in self.robots:
                if (other_robot.id not in processed and 
                    current_robot.distance_to(other_robot) < CENTRALIZED_COMM_RADIUS):
                    
                    other_robot.parent = current_robot
                    current_robot.children.append(other_robot)
                    other_robot.is_connected_to_origin = True
                    other_robot.distance_to_origin = current_robot.distance_to_origin + current_robot.distance_to(other_robot)
                    
                    queue.append(other_robot)
                    processed.add(other_robot.id)
    
    def _manage_chain_formation(self):
        """Enhanced chain formation management for single goal"""
        # Reset chain properties
        for robot in self.robots:
            robot.is_chain_leader = False
            robot.is_chain_active = False
            robot.chain_position = -1
            robot.expansion_weight = 1.0
            robot.connectivity_priority = 0
            robot.is_expansion_candidate = False
        
        # Identify potential chain leaders - robots closer to goal
        potential_leaders = []
        for robot in self.robots:
            if robot.is_connected_to_origin and not robot.is_at_goal:
                goal_distance = robot.distance_to_goal1
                
                # Prioritize robots closer to goal but not too close
                if goal_distance > CENTRALIZED_GOAL_DETECTION_RADIUS * 1.5:
                    priority = 1.0 / (goal_distance + 0.1)
                    potential_leaders.append((robot, priority))
        
        # Sort by priority and select fewer leaders for better connectivity
        potential_leaders.sort(key=lambda x: x[1], reverse=True)
        
        # Assign chain leaders - more leaders for faster progress
        assigned_leaders = 0
        max_leaders = min(8, len(self.robots) // 6)  # More leaders
        for robot, priority in potential_leaders:
            if assigned_leaders >= max_leaders:
                break
            
            # More permissive leadership criteria
            if len(robot.children) <= 2:
                robot.is_chain_leader = True
                robot.is_chain_active = True
                robot.expansion_weight = 2.0  # Strong expansion
                robot.connectivity_priority = 10
                assigned_leaders += 1
        
        # Identify chain members - more permissive assignment
        for robot in self.robots:
            if not robot.is_chain_leader and robot.is_connected_to_origin:
                # Check if robot is supporting a chain
                supporting_leader = False
                for other_robot in self.robots:
                    if (other_robot.is_chain_leader and 
                        robot.distance_to(other_robot) < CENTRALIZED_COMM_RADIUS * 0.9):  # More generous range
                        supporting_leader = True
                        break
                
                if supporting_leader and not robot.is_at_goal:
                    robot.is_chain_active = True
                    robot.expansion_weight = 1.8  # More aggressive expansion
                    robot.connectivity_priority = 7
    
    def _compute_target_positions(self):
        """Enhanced target position computation for single goal"""
        for robot in self.robots:
            if robot.is_at_goal:
                # Assign formation positions in goal area
                formation_positions = self.goal_formation_positions
                
                # Find nearest available formation position
                best_position = min(formation_positions, 
                                  key=lambda pos: np.linalg.norm(robot.position - pos))
                robot.target_position = best_position
                
            elif robot.is_chain_leader:
                # Chain leaders move more aggressively toward goal
                direction_to_goal = self.goal - robot.position
                distance_to_goal = np.linalg.norm(direction_to_goal)
                
                if distance_to_goal > CENTRALIZED_GOAL_DETECTION_RADIUS * 1.2:
                    # Move more aggressively toward goal
                    direction = direction_to_goal / distance_to_goal
                    move_distance = min(CENTRALIZED_COMM_RADIUS * 0.8, distance_to_goal * 0.7)  # More aggressive
                    robot.target_position = robot.position + direction * move_distance
                else:
                    robot.target_position = self.goal.copy()
                    
            elif robot.is_chain_active:
                # Chain members bridge between parent and goal direction
                if robot.parent:
                    # Compute direction from parent toward goal
                    parent_to_goal = self.goal - robot.parent.position
                    parent_goal_distance = np.linalg.norm(parent_to_goal)
                    
                    if parent_goal_distance > 0:
                        direction = parent_to_goal / parent_goal_distance
                        # Position along the path from parent toward goal
                        target_distance = min(CENTRALIZED_COMM_RADIUS * 0.8, 
                                            parent_goal_distance * 0.6)  # More aggressive
                        robot.target_position = robot.parent.position + direction * target_distance
                    else:
                        robot.target_position = robot.position.copy()
                else:
                    robot.target_position = robot.position.copy()
                    
            else:
                # Non-active robots: bias toward goal while maintaining connectivity
                if robot.is_connected_to_origin and robot.parent:
                    # Blend parent attraction with goal attraction
                    parent_direction = robot.parent.position - robot.position
                    goal_direction = self.goal - robot.position
                    
                    parent_distance = np.linalg.norm(parent_direction)
                    goal_distance = np.linalg.norm(goal_direction)
                    
                    if parent_distance > CENTRALIZED_COMM_RADIUS * 0.8:
                        # Too far from parent - move toward parent
                        parent_direction = parent_direction / parent_distance
                        robot.target_position = robot.position + parent_direction * 0.3
                    else:
                        # Close to parent - bias toward goal
                        if goal_distance > 0:
                            goal_direction = goal_direction / goal_distance
                            # Small bias toward goal
                            robot.target_position = robot.position + goal_direction * 0.2
                        else:
                            robot.target_position = robot.position.copy()
                else:
                    # Disconnected robots - try to reconnect while biasing toward goal
                    if len(self.robots) > 0:
                        # Find nearest connected robot
                        connected_robots = [r for r in self.robots if r.is_connected_to_origin]
                        if connected_robots:
                            nearest_connected = min(connected_robots, 
                                                  key=lambda r: np.linalg.norm(r.position - robot.position))
                            reconnect_direction = nearest_connected.position - robot.position
                            if np.linalg.norm(reconnect_direction) > 0:
                                reconnect_direction = reconnect_direction / np.linalg.norm(reconnect_direction)
                                robot.target_position = robot.position + reconnect_direction * 0.4
                            else:
                                robot.target_position = robot.position.copy()
                        else:
                            robot.target_position = robot.position.copy()
                    else:
                        robot.target_position = robot.position.copy()
    
    def step(self, iteration):
        """Single step of the single-goal MLCCST algorithm"""
        self.step_count = iteration
        
        # Build tree structure
        self._build_tree_structure()
        
        # Update distances to goal
        for robot in self.robots:
            robot.distance_to_goal1 = np.linalg.norm(robot.position - self.goal)
            # Check if robot reached goal
            if robot.distance_to_goal1 < CENTRALIZED_GOAL_DETECTION_RADIUS:
                if not robot.is_at_goal:
                    robot.is_at_goal = True
                    robot.goal_arrival_time = iteration
        
        # Manage chain formation
        self._manage_chain_formation()
        
        # Compute target positions
        self._compute_target_positions()
        
        # Apply CLF-CBF control with stronger connectivity emphasis
        for robot in self.robots:
            control = self.controller.compute_clf_cbf_control(
                robot, robot.target_position, self.robots, robot.parent
            )
            
            # Apply underwater flow field effects if environment is available
            if self.underwater_env:
                flow_effect = self.underwater_env.get_flow_field(robot.position) * 0.05  # Reduced flow effect
                control += flow_effect
            
            robot.apply_control_force(control)
        
        # Apply Brownian motion - reduced for better connectivity
        if iteration < CENTRALIZED_BROWNIAN_PHASE_DURATION:
            for robot in self.robots:
                if not robot.is_at_goal:  # Don't disturb robots at goal
                    robot.apply_brownian_motion(CENTRALIZED_BROWNIAN_INTENSITY)
        
        # Update positions
        for robot in self.robots:
            robot.update_position()
        
        # Record positions
        for i, robot in enumerate(self.robots):
            self.position_history[i].append(robot.position.copy())
        
        # Calculate and record connectivity
        connectivity_ratio = self._calculate_connectivity()
        self.connectivity_history.append(connectivity_ratio)
        
        return connectivity_ratio
    
    def _calculate_connectivity(self):
        """Calculate network connectivity ratio"""
        connected_count = sum(1 for robot in self.robots if robot.is_connected_to_origin)
        return connected_count / self.num_robots
    
    def get_positions(self):
        """Get current robot positions"""
        return np.array([robot.position for robot in self.robots])
    
    def get_connectivity_metrics(self):
        """Get comprehensive connectivity metrics for single goal"""
        connectivity_ratio = self._calculate_connectivity()
        at_goal = sum(1 for r in self.robots if r.is_at_goal)
        
        return {
            'connectivity_ratio': connectivity_ratio,
            'connected_robots': sum(1 for r in self.robots if r.is_connected_to_origin),
            'assigned_robots': self.num_robots,  # All assigned to single goal
            'at_goal': at_goal,
            'chain_leaders': sum(1 for r in self.robots if r.is_chain_leader),
            'chain_active': sum(1 for r in self.robots if r.is_chain_active)
        }

# ============================================================================
# CENTRALIZED SIMULATION FUNCTIONS
# ============================================================================

def create_clustered_formation(num_robots, center, cluster_radius=0.6):
    """Create clustered initial formation"""
    positions = []
    
    # Create clusters
    num_clusters = max(3, num_robots // 8)
    robots_per_cluster = num_robots // num_clusters
    remaining_robots = num_robots % num_clusters
    
    for i in range(num_clusters):
        # Cluster center
        angle = 2 * np.pi * i / num_clusters
        cluster_center = center + cluster_radius * np.array([np.cos(angle), np.sin(angle)])
        
        # Robots in this cluster
        cluster_size = robots_per_cluster + (1 if i < remaining_robots else 0)
        
        for j in range(cluster_size):
            if cluster_size == 1:
                robot_pos = cluster_center
            else:
                sub_angle = 2 * np.pi * j / cluster_size
                sub_radius = 0.15 + 0.1 * np.random.random()
                offset = sub_radius * np.array([np.cos(sub_angle), np.sin(sub_angle)])
                robot_pos = cluster_center + offset
            
            positions.append(robot_pos)
    
    return positions

def generate_single_random_goal(bounds):
    """Generate single random goal position"""
    x_min, x_max, y_min, y_max = bounds
    
    goal = np.array([
        np.random.uniform(x_min + 1.0, x_max - 1.0),
        np.random.uniform(y_min + 1.0, y_max - 1.0)
    ])
    
    return goal

def generate_random_obstacles(bounds, num_obstacles=5, min_radius=0.1, max_radius=0.3):
    """Generate random obstacles"""
    x_min, x_max, y_min, y_max = bounds
    obstacles = []
    
    for _ in range(num_obstacles):
        position = np.array([
            np.random.uniform(x_min + 0.5, x_max - 0.5),
            np.random.uniform(y_min + 0.5, y_max - 0.5)
        ])
        radius = np.random.uniform(min_radius, max_radius)
        obstacles.append(Obstacle(position, radius))
    
    return obstacles

def run_centralized_mlccst_simulation(underwater_env):
    """Run centralized MLCCST simulation in underwater environment with real-time animation"""
    print("Starting Centralized MLCCST CBF Simulation (Single Goal) in Underwater Environment...")
    print("=" * 80)
    
    # Use same bounds as underwater environment
    bounds = [-5, 5, -5, 5]
    
    # Generate single goal and obstacles
    goal = generate_single_random_goal(bounds)
    obstacles = generate_random_obstacles(bounds, num_obstacles=3)
    
    print(f"Goal: ({goal[0]:.2f}, {goal[1]:.2f})")
    print(f"Number of obstacles: {len(obstacles)}")
    
    # Create initial formation at origin
    origin = np.array([0.0, 0.0])
    initial_positions = create_clustered_formation(CENTRALIZED_NUM_ROBOTS, origin)
    
    # Initialize controller
    controller = SingleGoalMLCCSTController(
        initial_positions=initial_positions,
        goal=goal,
        obstacles=obstacles,
        underwater_env=underwater_env
    )
    
    print(f"Initialized {CENTRALIZED_NUM_ROBOTS} robots in clustered formation")
    print(f"Communication radius: {CENTRALIZED_COMM_RADIUS}")
    print(f"Safety distance: {CENTRALIZED_MIN_SAFETY_DISTANCE}")
    print("Starting animated simulation...")
    print()
    
    # Set up the animated visualization (3-panel layout, removed connectivity plot)
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle("Centralized MLCCST CBF - Single Goal Coordination", fontsize=16)
    
    # Animation state variables
    iteration = 0
    connectivity_history = []
    robot_trajectories = [[] for _ in range(CENTRALIZED_NUM_ROBOTS)]
    clf_distance_history = []  # Store CLF data (average distance to goal)
    
    def animate_centralized_mlccst(frame):
        nonlocal iteration, connectivity_history, clf_distance_history
        
        if iteration >= CENTRALIZED_MAX_ITERATIONS:
            return
        
        # Step the controller
        connectivity_ratio = controller.step(iteration)
        connectivity_history.append(connectivity_ratio)
        
        # Calculate and store CLF (average distance to goal)
        distances_to_goal = []
        for robot in controller.robots:
            distance = np.linalg.norm(robot.position - goal)
            distances_to_goal.append(distance)
        avg_distance_to_goal = np.mean(distances_to_goal)
        clf_distance_history.append(avg_distance_to_goal)
        
        # Record robot trajectories
        positions = controller.get_positions()
        for i in range(CENTRALIZED_NUM_ROBOTS):
            robot_trajectories[i].append(positions[i].copy())
        
        # Get current metrics
        metrics = controller.get_connectivity_metrics()
        
        # Clear all axes
        ax1.clear()
        ax2.clear()
        ax3.clear()
        ax4.clear()
        
        # ===== MAIN ROBOT VISUALIZATION (ax1) =====
        ax1.set_xlim(-6, 6)
        ax1.set_ylim(-6, 6)
        ax1.set_title(f"Robot Swarm - Iteration {iteration}")
        ax1.grid(True, alpha=0.3)
        
        # Draw underwater flow field (light background vectors)
        x_flow = np.linspace(-5, 5, 8)
        y_flow = np.linspace(-5, 5, 8)
        X_flow, Y_flow = np.meshgrid(x_flow, y_flow)
        flow_vectors = np.array([[underwater_env.get_flow_field(np.array([x, y])) for x in x_flow] for y in y_flow])
        U_flow = flow_vectors[:, :, 0]
        V_flow = flow_vectors[:, :, 1]
        ax1.quiver(X_flow, Y_flow, U_flow, V_flow, alpha=0.3, scale=8, color='lightblue', width=0.003)
        
        # Draw goal
        ax1.scatter(*goal, color='red', s=400, marker='*', edgecolors='darkred', linewidth=3, label='Goal', zorder=10)
        
        # Draw goal area
        goal_circle = plt.Circle(goal, CENTRALIZED_GOAL_DETECTION_RADIUS, fill=False, color='red', linestyle='--', alpha=0.7)
        ax1.add_patch(goal_circle)
        
        # Draw obstacles
        for obs in obstacles:
            circle = plt.Circle(obs.position, obs.radius, fill=True, color='gray', alpha=0.7)
            ax1.add_patch(circle)
        
        # Draw robots with different colors based on status
        for robot in controller.robots:
            color = 'lightgray'  # Default
            marker = 'o'
            size = 80
            
            if robot.is_at_goal:
                color = 'gold'
                marker = 's'  # Square for robots at goal
                size = 120
            elif robot.is_chain_leader:
                color = 'red'
                marker = '^'  # Triangle for leaders
                size = 100
            elif robot.is_chain_active:
                color = 'orange'
                size = 90
            elif robot.is_connected_to_origin:
                color = 'lightgreen'
            else:
                color = 'lightcoral'  # Disconnected robots
            
            # Draw robot
            ax1.scatter(*robot.position, color=color, s=size, marker=marker, 
                       edgecolors='black', linewidth=1, alpha=0.8)
            
            # Draw connectivity lines to parent - thicker for better visibility
            if robot.parent and robot.is_connected_to_origin:
                ax1.plot([robot.position[0], robot.parent.position[0]], 
                        [robot.position[1], robot.parent.position[1]], 
                        'g-', alpha=0.6, linewidth=2)
        
        ax1.legend(loc='upper right')
        
        # ===== ROBOT STATUS PIE CHART (ax2) =====
        ax2.set_title("Robot Status Distribution")
        
        status_counts = {
            'At Goal': metrics['at_goal'],
            'Chain Leaders': metrics['chain_leaders'], 
            'Chain Active': metrics['chain_active'],
            'Connected': metrics['connected_robots'] - metrics['chain_leaders'] - metrics['chain_active'],
            'Disconnected': CENTRALIZED_NUM_ROBOTS - metrics['connected_robots']
        }
        
        # Filter out zero values for cleaner pie chart
        filtered_counts = {k: v for k, v in status_counts.items() if v > 0}
        
        if filtered_counts:
            colors = ['gold', 'red', 'orange', 'lightgreen', 'lightcoral'][:len(filtered_counts)]
            wedges, texts, autotexts = ax2.pie(filtered_counts.values(), labels=filtered_counts.keys(), 
                                              autopct='%1.0f', colors=colors, startangle=90)
            for autotext in autotexts:
                autotext.set_color('black')
                autotext.set_fontweight('bold')
        
        # ===== ROBOT TRAJECTORIES (ax3) =====
        ax3.set_xlim(-6, 6)
        ax3.set_ylim(-6, 6)
        ax3.set_title("Robot Movement Trajectories")
        ax3.grid(True, alpha=0.3)
        
        # Draw goal
        ax3.scatter(*goal, color='red', s=300, marker='*', label='Goal', zorder=10)
        goal_circle = plt.Circle(goal, CENTRALIZED_GOAL_DETECTION_RADIUS, fill=False, color='red', linestyle='--', alpha=0.5)
        ax3.add_patch(goal_circle)
        
        # Draw trajectories for robots that have moved
        for i, trajectory in enumerate(robot_trajectories):
            if len(trajectory) > 2:  # Only show meaningful trajectories
                robot = controller.robots[i]
                if robot.is_at_goal:
                    color = 'gold'
                    alpha = 0.8
                    width = 2
                elif robot.is_chain_leader:
                    color = 'red'
                    alpha = 0.7
                    width = 2
                elif robot.is_chain_active:
                    color = 'orange'
                    alpha = 0.6
                    width = 1.5
                else:
                    color = 'lightblue'
                    alpha = 0.4
                    width = 1
                
                traj_array = np.array(trajectory)
                ax3.plot(traj_array[:, 0], traj_array[:, 1], color=color, alpha=alpha, linewidth=width)
                
                # Mark current position
                ax3.scatter(*robot.position, color=color, s=40, alpha=0.8)
        
        ax3.legend(loc='upper right')
        
        # ===== CONNECTIVITY METRICS BAR CHART (ax4) =====
        ax4.set_title("System Performance Metrics")
        ax4.set_ylabel("Count / Percentage")
        
        metric_labels = ['Connected\nRobots', 'At Goal', 'Chain\nLeaders', 'Connectivity\n%']
        metric_values = [
            metrics['connected_robots'],
            metrics['at_goal'], 
            metrics['chain_leaders'],
            int(metrics['connectivity_ratio'] * 100)  # Convert to percentage
        ]
        colors = ['green', 'gold', 'red', 'blue']
        
        bars = ax4.bar(metric_labels, metric_values, color=colors, alpha=0.7)
        ax4.set_ylim(0, max(CENTRALIZED_NUM_ROBOTS + 5, 105))
        
        # Add value labels on bars
        for bar, value in zip(bars, metric_values):
            height = bar.get_height()
            if height > 0:
                ax4.text(bar.get_x() + bar.get_width()/2., height + 1, f'{value}',
                        ha='center', va='bottom', fontweight='bold')
        
        # Add reference lines
        ax4.axhline(y=CENTRALIZED_NUM_ROBOTS, color='black', linestyle='-', alpha=0.3, label='Total Robots')
        ax4.axhline(y=90, color='orange', linestyle='--', alpha=0.5, label='90% Connectivity')
        
        # Print progress with connectivity emphasis
        if iteration % 25 == 0:
            print(f"Animation Frame {iteration}: "
                  f"🔗 Connectivity: {metrics['connectivity_ratio']:.1%} "
                  f"({metrics['connected_robots']}/{CENTRALIZED_NUM_ROBOTS}) | "
                  f"🎯 At Goal: {metrics['at_goal']} | "
                  f"🚀 Leaders: {metrics['chain_leaders']}")
        
        iteration += 1
    
    # Create and start animation
    print("🎬 Starting real-time animated visualization...")
    print("🔗 Focus: Maintaining network connectivity while reaching goal")
    print("Close the animation window to stop the simulation.")
    
    anim = animation.FuncAnimation(fig, animate_centralized_mlccst, 
                                 interval=75,  # Slower for better observation
                                 repeat=False,
                                 cache_frame_data=False)
    
    plt.tight_layout()
    plt.show()
    
    # Final results
    print("\n" + "=" * 80)
    print("SINGLE-GOAL ANIMATED SIMULATION COMPLETED")
    print("=" * 80)
    
    final_metrics = controller.get_connectivity_metrics()
    
    print(f"Final Results after {iteration-1} iterations:")
    print(f"  🔗 Final Connectivity: {final_metrics['connectivity_ratio']:.1%} ({final_metrics['connected_robots']}/{CENTRALIZED_NUM_ROBOTS})")
    print(f"  🎯 Robots at goal: {final_metrics['at_goal']}")
    print(f"  🚀 Chain leaders: {final_metrics['chain_leaders']}")
    print(f"  ⚡ Chain active: {final_metrics['chain_active']}")
    
    # Connectivity statistics
    if connectivity_history:
        avg_connectivity = np.mean(connectivity_history)
        min_connectivity = np.min(connectivity_history)
        print(f"  📊 Average connectivity: {avg_connectivity:.1%}")
        print(f"  📉 Minimum connectivity: {min_connectivity:.1%}")
        
        # Success metric
        high_connectivity_steps = sum(1 for c in connectivity_history if c >= 0.8)
        success_rate = high_connectivity_steps / len(connectivity_history)
        print(f"  ✅ High connectivity success rate (≥80%): {success_rate:.1%}")
    
    print("=" * 80)
    
    # Plot CLF convergence after simulation
    if clf_distance_history:
        print("\n📊 Generating CLF Convergence Analysis...")
        
        # Create CLF convergence plot
        plt.figure(figsize=(12, 8))
        
        # Main CLF convergence plot
        plt.subplot(2, 2, 1)
        iterations = list(range(len(clf_distance_history)))
        plt.plot(iterations, clf_distance_history, 'b-', linewidth=1, label='Average Distance to Goal')
        plt.axhline(y=CENTRALIZED_GOAL_DETECTION_RADIUS, color='red', linestyle='--', alpha=0.7, label='Goal Threshold')
        plt.title('CLF Convergence', fontweight='bold')
        plt.xlabel('Iteration')
        plt.ylabel('Average Distance to Goal', fontsize=14)
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Connectivity over time
        plt.subplot(2, 2, 2)
        plt.plot(iterations, [c * 100 for c in connectivity_history], 'g-', linewidth=2, label='Connectivity %')
        plt.axhline(y=80, color='orange', linestyle='--', alpha=0.7, label='80% Threshold')
        plt.title('Network Connectivity Over Time', fontweight='bold')
        plt.xlabel('Iteration')
        plt.ylabel('Connectivity Percentage')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.ylim(0, 105)
        
        # CLF convergence rate analysis
        plt.subplot(2, 2, 3)
        if len(clf_distance_history) > 10:
            # Calculate moving average convergence rate
            window_size = 10
            convergence_rates = []
            for i in range(window_size, len(clf_distance_history)):
                initial = np.mean(clf_distance_history[i-window_size:i-window_size//2])
                final = np.mean(clf_distance_history[i-window_size//2:i])
                if initial > 0:
                    rate = (initial - final) / initial * 100
                    convergence_rates.append(rate)
                else:
                    convergence_rates.append(0)
            
            conv_iterations = list(range(window_size, len(clf_distance_history)))
            plt.plot(conv_iterations, convergence_rates, 'r-', linewidth=2, label='Convergence Rate')
            plt.title('CLF Convergence Rate (%)', fontweight='bold')
            plt.xlabel('Iteration')
            plt.ylabel('Convergence Rate (%)')
            plt.legend()
            plt.grid(True, alpha=0.3)
        
        # Final statistics
        plt.subplot(2, 2, 4)
        final_distance = clf_distance_history[-1]
        initial_distance = clf_distance_history[0]
        total_convergence = (initial_distance - final_distance) / initial_distance * 100 if initial_distance > 0 else 0
        
        stats_text = f"CLF Analysis Summary:\n\n"
        stats_text += f"Initial Avg Distance: {initial_distance:.3f}\n"
        stats_text += f"Final Avg Distance: {final_distance:.3f}\n"
        stats_text += f"Total Convergence: {total_convergence:.1f}%\n"
        stats_text += f"Goal Threshold: {CENTRALIZED_GOAL_DETECTION_RADIUS:.3f}\n"
        stats_text += f"Goal Achievement: {'✅ Yes' if final_distance <= CENTRALIZED_GOAL_DETECTION_RADIUS else '❌ No'}\n\n"
        stats_text += f"Simulation completed in {len(clf_distance_history)} iterations\n"
        stats_text += f"CLF shows {'hyperbolic' if total_convergence > 50 else 'gradual'} convergence pattern"
        
        plt.text(0.05, 0.95, stats_text, transform=plt.gca().transAxes, 
                fontsize=11, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        plt.axis('off')
        plt.title('CLF Convergence Statistics', fontweight='bold')
        
        plt.suptitle('Centralized MLCCST: CLF Convergence Analysis', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.show()
        
        print(f"✅ CLF Analysis: {total_convergence:.1f}% convergence achieved")
        print(f"   Final average distance to goal: {final_distance:.3f}")
    
    return final_metrics

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
        
        # Flow force (water pushes robot along with current)
        flow_force = self.underwater_env.water_density * node.volume * flow_velocity * 2.0
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
    
    def calculate_current_tree_edge_length_sum(self) -> float:
        """
        Calculate the sum of all edge lengths in the current connectivity tree/graph.
        This represents the total connectivity cost of the current algorithm's solution.
        """
        total_edge_length = 0.0
        counted_edges = set()
        
        for node_id, node in self.nodes.items():
            for neighbor_id in node.direct_neighbors:
                # Avoid double counting edges by using a canonical edge representation
                edge = tuple(sorted([node_id, neighbor_id]))
                if edge not in counted_edges:
                    # Calculate Euclidean distance between the two nodes
                    node_pos = node.position
                    neighbor_pos = self.nodes[neighbor_id].position
                    edge_length = np.linalg.norm(node_pos - neighbor_pos)
                    total_edge_length += edge_length
                    counted_edges.add(edge)
        
        return total_edge_length
    
    def calculate_minimum_chain_edge_length_sum(self) -> float:
        """
        Calculate the sum of edge lengths for a minimum chain path visiting all robots sequentially.
        This creates a simple path: robot1 → robot2 → robot3 → ... → robotN
        This represents the baseline minimum connectivity cost.
        """
        if len(self.nodes) <= 1:
            return 0.0
        
        # Get sorted list of robot IDs for consistent chain ordering
        robot_ids = sorted(list(self.nodes.keys()))
        
        total_chain_length = 0.0
        
        # Calculate chain: robot[0] → robot[1] → robot[2] → ... → robot[N-1]
        for i in range(len(robot_ids) - 1):
            current_id = robot_ids[i]
            next_id = robot_ids[i + 1]
            
            current_pos = self.nodes[current_id].position
            next_pos = self.nodes[next_id].position
            
            # Add edge length to chain
            edge_length = np.linalg.norm(current_pos - next_pos)
            total_chain_length += edge_length
        
        return total_chain_length
    
    def calculate_true_mst_edge_length_sum(self) -> float:
        """
        Calculate the sum of edge lengths in the TRUE Minimum Spanning Tree at current time step.
        Uses Kruskal's algorithm to find the MST from ALL possible edges between robots.
        This represents the mathematically optimal connectivity cost.
        """
        if len(self.nodes) <= 1:
            return 0.0
        
        # Get all robot IDs
        robot_ids = list(self.nodes.keys())
        
        # Create all possible edges with their weights (distances)
        all_edges = []
        for i in range(len(robot_ids)):
            for j in range(i + 1, len(robot_ids)):
                robot1_id = robot_ids[i]
                robot2_id = robot_ids[j]
                
                pos1 = self.nodes[robot1_id].position
                pos2 = self.nodes[robot2_id].position
                distance = np.linalg.norm(pos1 - pos2)
                
                all_edges.append((distance, robot1_id, robot2_id))
        
        # Sort edges by weight (distance) for Kruskal's algorithm
        all_edges.sort()
        
        # Kruskal's algorithm using Union-Find
        parent = {robot_id: robot_id for robot_id in robot_ids}
        
        def find(x):
            if parent[x] != x:
                parent[x] = find(parent[x])
            return parent[x]
        
        def union(x, y):
            px, py = find(x), find(y)
            if px != py:
                parent[px] = py
                return True
            return False
        
        mst_total_length = 0.0
        mst_edges_count = 0
        
        # Process edges in order of increasing weight
        for weight, u, v in all_edges:
            if union(u, v):
                mst_total_length += weight
                mst_edges_count += 1
                # MST for n nodes has exactly n-1 edges
                if mst_edges_count == len(robot_ids) - 1:
                    break
        
        return mst_total_length
    
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
    
    def generate_valid_goal(self, x_min=0, x_max=10, y_min=0, y_max=10):
        """
        Generate a valid goal location within workspace bounds.
        Single source of truth for goal generation with validation.
        """
        # Generate random goal within bounds
        goal_x = np.random.uniform(x_min, x_max)
        goal_y = np.random.uniform(y_min, y_max)
        
        # Clamp to ensure bounds (safety measure)
        goal_x = max(x_min, min(x_max, goal_x))
        goal_y = max(y_min, min(y_max, goal_y))
        
        return np.array([goal_x, goal_y])
    
    def run_mlccst_controller(self):
        """
        Centralized Minimum Length Connected Spanning Tree (MLCCST) Controller
        
        Implements centralized MLCCST algorithm for maintaining connectivity
        while optimizing for minimum total edge length in the spanning tree.
        
        Key Features:
        - Centralized computation of optimal spanning tree
        - Line-of-sight connectivity preservation
        - Minimum total edge length optimization
        - Real-time spanning tree updates
        """
        print("\n🌲 Running Centralized MLCCST Controller...")
        print("   Algorithm: Minimum Length Connected Spanning Tree")
        print("   Approach: Centralized optimization with real-time updates")
        
        # Step 1: Compute current minimum spanning tree using Kruskal's algorithm
        mst_edges = self.compute_minimum_spanning_tree()
        
        # Step 2: Update network topology to match MST
        changes = {'added': [], 'removed': []}
        
        # Get current edges
        current_edges = set()
        for node_id, node in self.nodes.items():
            for neighbor_id in node.direct_neighbors:
                if node_id < neighbor_id:  # Avoid duplicates
                    current_edges.add((node_id, neighbor_id))
        
        # Convert MST edges to set for comparison
        mst_edge_set = set()
        for edge in mst_edges:
            node1, node2 = edge['nodes']
            mst_edge_set.add(tuple(sorted([node1, node2])))
        
        # Remove edges not in MST
        edges_to_remove = current_edges - mst_edge_set
        for edge in edges_to_remove:
            node1, node2 = edge
            if node1 in self.nodes and node2 in self.nodes:
                self.nodes[node1].direct_neighbors.discard(node2)
                self.nodes[node2].direct_neighbors.discard(node1)
                changes['removed'].append(edge)
                print(f"   ✂️ Removed non-MST edge: {node1}↔{node2}")
        
        # Add MST edges that are missing
        edges_to_add = mst_edge_set - current_edges
        for edge in edges_to_add:
            node1, node2 = edge
            if node1 in self.nodes and node2 in self.nodes:
                # Check if edge is within communication range
                pos1 = self.nodes[node1].position
                pos2 = self.nodes[node2].position
                distance = np.linalg.norm(pos1 - pos2)
                
                if distance <= 1.2:  # Within communication range
                    self.nodes[node1].direct_neighbors.add(node2)
                    self.nodes[node2].direct_neighbors.add(node1)
                    changes['added'].append(edge)
                    print(f"   🔗 Added MST edge: {node1}↔{node2} (dist: {distance:.2f})")
                else:
                    print(f"   ⚠️ MST edge {node1}↔{node2} out of range (dist: {distance:.2f})")
        
        # Step 3: Verify connectivity
        if self.is_network_connected():
            print(f"✅ MLCCST update complete: {len(changes['added'])} added, {len(changes['removed'])} removed")
            print(f"🌲 Network topology now matches optimal MST structure")
        else:
            print("❌ WARNING: MLCCST update resulted in disconnected network!")
        
        return changes
    
    def compute_minimum_spanning_tree(self):
        """
        Compute minimum spanning tree using Kruskal's algorithm
        Returns list of edges in MST with their weights
        """
        # Get all possible edges with their weights (distances)
        edges = []
        for node1_id, node1 in self.nodes.items():
            for node2_id, node2 in self.nodes.items():
                if node1_id < node2_id:  # Avoid duplicates
                    distance = np.linalg.norm(node1.position - node2.position)
                    edges.append({
                        'nodes': (node1_id, node2_id),
                        'weight': distance
                    })
        
        # Sort edges by weight (distance)
        edges.sort(key=lambda x: x['weight'])
        
        # Kruskal's algorithm: Union-Find data structure
        parent = {node_id: node_id for node_id in self.nodes.keys()}
        rank = {node_id: 0 for node_id in self.nodes.keys()}
        
        def find(node):
            if parent[node] != node:
                parent[node] = find(parent[node])  # Path compression
            return parent[node]
        
        def union(node1, node2):
            root1, root2 = find(node1), find(node2)
            if root1 != root2:
                # Union by rank
                if rank[root1] < rank[root2]:
                    parent[root1] = root2
                elif rank[root1] > rank[root2]:
                    parent[root2] = root1
                else:
                    parent[root2] = root1
                    rank[root1] += 1
                return True
            return False
        
        # Build MST
        mst_edges = []
        for edge in edges:
            node1, node2 = edge['nodes']
            if union(node1, node2):
                mst_edges.append(edge)
                if len(mst_edges) == len(self.nodes) - 1:  # MST has n-1 edges
                    break
        
        return mst_edges

    def update_edges_by_distance_threshold(self, communication_range=1.2):
        """
        Simple distance-based edge management for Brownian motion mode.
        
        - Add edges when robots come within communication range
        - Remove edges when robots move beyond communication range
        - No connectivity preservation - purely distance-based
        
        Args:
            communication_range: Distance threshold for edge addition/removal
        """
        changes = {'added': [], 'removed': []}
        
        # Check all pairs of robots
        robot_ids = list(self.nodes.keys())
        
        for i, robot1_id in enumerate(robot_ids):
            for j, robot2_id in enumerate(robot_ids):
                if i >= j:  # Avoid duplicates and self-connections
                    continue
                    
                robot1 = self.nodes[robot1_id]
                robot2 = self.nodes[robot2_id]
                distance = np.linalg.norm(robot1.position - robot2.position)
                
                # Check if edge currently exists
                edge_exists = robot2_id in robot1.direct_neighbors
                
                # Add edge if robots are close and no edge exists
                if distance <= communication_range and not edge_exists:
                    robot1.direct_neighbors.add(robot2_id)
                    robot2.direct_neighbors.add(robot1_id)
                    changes['added'].append((robot1_id, robot2_id))
                    print(f"   🔗 Brownian mode: Added edge {robot1_id}↔{robot2_id} (dist: {distance:.2f})")
                
                # Remove edge if robots are too far and edge exists
                elif distance > communication_range and edge_exists:
                    robot1.direct_neighbors.discard(robot2_id)
                    robot2.direct_neighbors.discard(robot1_id)
                    changes['removed'].append((robot1_id, robot2_id))
                    print(f"   ✂️ Brownian mode: Removed edge {robot1_id}↔{robot2_id} (dist: {distance:.2f})")
        
        # Print summary if changes were made
        if changes['added'] or changes['removed']:
            total_edges = sum(len(node.direct_neighbors) for node in self.nodes.values()) // 2
            print(f"   📊 Brownian edge update: +{len(changes['added'])} -{len(changes['removed'])} edges (total: {total_edges})")
        
        return changes

    def goal_seeking_simulation(self, goal_x, goal_y, use_controller=True, pruning_method="triangular"):
        """
        Goal-seeking simulation with connectivity preservation options
        
        Args:
            goal_x, goal_y: Goal coordinates
            use_controller: If True, use CBF control; if False, pure Brownian motion
            pruning_method: "triangular", "distance", or "mlccst"
        """
        import matplotlib.animation as animation
        
        print("\n" + "=" * 70)
        if use_controller:
            print("GOAL-SEEKING SIMULATION WITH CBF CONNECTIVITY PRESERVATION")
            print("Closest robot moves toward goal while preserving network connectivity")
            if pruning_method == "mlccst":
                print(f"🌲 Control Method: MLCCST (Centralized Minimum Length Connected Spanning Tree)")
            else:
                print(f"🔧 Pruning Method: {pruning_method.upper()}")
        else:
            print("🚨 BROWNIAN MOTION SIMULATION - NO CBF CONTROL")
            print("⚠️  Pure Brownian motion and flow field effects only!")
            print("🔬 Purpose: Demonstrate what happens WITHOUT CBF control")
        print(f"Goal location: ({goal_x}, {goal_y})")
        print("=" * 70)
        
        # Use clean goal generation method
        self.goal_location = self.generate_valid_goal()
        print(f"✅ Clean goal generated: ({self.goal_location[0]:.2f}, {self.goal_location[1]:.2f})")
        
        # Initialize dynamic goal management
        self.goal_switch_interval = 100   # frames to show each goal
        self.goal_visible = True         # goals are always visible
        self.goal_switch_counter = 0
        
        # FIXED: Match workspace bounds with actual displayed workspace (0,0) to (10,10)
        self.workspace_bounds = [0, 10, 0, 10]  # [x_min, x_max, y_min, y_max] - matches Rectangle((0,0), 10, 10)
        
        # Initialize simulation parameters
        self.goal_reached = False
        self.goal_tolerance = 0.4  # How close to consider goal reached
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
        print(f"\n🔄 Running initial {pruning_method} connectivity optimization...")
        if pruning_method == "distance":
            print("   📏 Enhanced distance-based method: pruning longest edges + adding beneficial short edges")
            initial_changes = self.run_distance_based_edge_pruning()
        elif pruning_method == "mlccst":
            print("   🌲 Centralized MLCCST method: minimum length connected spanning tree optimization")
            initial_changes = self.run_mlccst_controller()
        else:  # Default to triangular
            print("   📐 Triangular geometric method: k-hop discovery + geometric optimization")
            initial_changes = self.run_decentralized_edge_optimization()
        
        if initial_changes['added']:
            print(f"🔗 Added {len(initial_changes['added'])} proximity-based edges: {initial_changes['added']}")
        if initial_changes['removed']:
            print(f"✂️ Removed {len(initial_changes['removed'])} redundant edges: {initial_changes['removed']}")
        
        # Create figure with 2 subplots side by side (simulation + MST graph only)
        fig, (ax_network, ax_stats) = plt.subplots(1, 2, figsize=(18, 8))
        
        # Set main title with larger font
        fig.suptitle('Robot Network - MST Connectivity Analysis', 
                     fontsize=18, fontweight='bold', y=0.95)
        
        # Configure main simulation subplot
        ax_network.set_title('Network Simulation', fontsize=14, fontweight='bold', pad=15)
        
        # Configure MST error subplot  
        ax_stats.set_title('MST Error Analysis', fontsize=14, fontweight='bold', pad=15)
        
        # Initialize data for animation
        self.animation_step = 0
        self.closest_robot_id = None
        self.goal_distance_history = []
        self.connectivity_history = []
        self.clf_distance_history = []  # Store CLF data (average distance to goal)
        
        # Initialize MST comparison tracking
        self.current_tree_edge_sum_history = []  # Total edge length of current algorithm
        self.true_mst_edge_sum_history = []      # Total edge length of TRUE MST at each step
        self.mst_error_history = []              # Error: current_sum - mst_sum (excess connectivity cost)
        
        # Calculate initial comparison values
        initial_current_sum = self.calculate_current_tree_edge_length_sum()
        initial_mst_sum = self.calculate_true_mst_edge_length_sum()
        initial_error = initial_current_sum - initial_mst_sum
        
        print(f"🔗 Initial MST Comparison:")
        print(f"   Current algorithm edge sum: {initial_current_sum:.2f}")
        print(f"   True MST edge sum: {initial_mst_sum:.2f}")
        print(f"   Initial error (excess cost): {initial_error:.2f}")
        print("   Error will be tracked at each time step (goal: minimize error → 0)")
        
        def animate_goal_seeking(frame):
            """Animation function for goal-seeking simulation"""
            # Dynamic goal switching logic
            self.goal_switch_counter += 1
            
            # Check if it's time to switch to next goal location
            if self.goal_switch_counter >= self.goal_switch_interval:
                # Use clean goal generation method
                x_min, x_max, y_min, y_max = self.workspace_bounds
                self.goal_location = self.generate_valid_goal(x_min, x_max, y_min, y_max)
                self.goal_switch_counter = 0
                self.goal_reached = False  # Reset goal reached status for new goal
                print(f"🎯 NEW GOAL SWITCH at frame {frame}! New goal: ({self.goal_location[0]:.1f}, {self.goal_location[1]:.1f})")
            
            for ax in [ax_network, ax_stats]:
                ax.clear()
                
            # Update goal_x and goal_y for the rest of the function
            goal_x, goal_y = self.goal_location[0], self.goal_location[1]
                
            # Find closest robot to goal and calculate CLF data
            min_distance = float('inf')
            closest_id = None
            distances_to_goal = []
            
            for node_id, node in self.nodes.items():
                distance = np.linalg.norm(node.position - self.goal_location)
                distances_to_goal.append(distance)
                if distance < min_distance:
                    min_distance = distance
                    closest_id = node_id
            
            # Store average distance to goal for CLF analysis
            avg_distance_to_goal = np.mean(distances_to_goal)
            self.clf_distance_history.append(avg_distance_to_goal)
            
            self.closest_robot_id = closest_id
            
            # Check if goal is reached
            if min_distance <= self.goal_tolerance and not self.goal_reached:
                self.goal_reached = True
                print(f"\n🎯 GOAL REACHED! Robot {closest_id} reached the goal!")
            
            # Update robot positions if goal not reached
            if not self.goal_reached:
                if frame % 30 == 0:
                    print(f"   -> Calling update_positions_goal_seeking()...")
                self.update_positions_goal_seeking(use_controller)
                
                # Run edge optimization periodically (every 10 frames) - ONLY when controller is enabled
                if frame % 10 == 0 and use_controller:
                    if pruning_method == "distance":
                        self.run_distance_based_edge_pruning()
                    elif pruning_method == "mlccst":
                        self.run_mlccst_controller()
                    else:  # Default to triangular
                        self.run_decentralized_edge_optimization()
                elif frame % 10 == 0 and not use_controller:
                    # Brownian motion mode: Add/remove edges based on distance threshold only
                    self.update_edges_by_distance_threshold()
            else:
                if frame % 30 == 0:
                    print(f"   -> Goal reached, no position update")
            
            # Record statistics
            self.goal_distance_history.append(min_distance)
            total_edges = sum(len(node.direct_neighbors) for node in self.nodes.values()) // 2
            self.connectivity_history.append(total_edges)
            
            # *** NEW: MST COMPARISON CALCULATIONS ***
            # Calculate current algorithm's total edge length
            current_tree_edge_sum = self.calculate_current_tree_edge_length_sum()
            self.current_tree_edge_sum_history.append(current_tree_edge_sum)
            
            # Calculate TRUE MST edge length at current time step
            true_mst_edge_sum = self.calculate_true_mst_edge_length_sum()
            self.true_mst_edge_sum_history.append(true_mst_edge_sum)
            
            # Calculate error: difference between current algorithm and optimal MST
            mst_error = current_tree_edge_sum - true_mst_edge_sum
            self.mst_error_history.append(mst_error)
            
            # Print MST comparison every 30 frames
            if frame % 30 == 0:
                print(f"📊 MST Comparison at frame {frame}:")
                print(f"   Current algorithm edge sum: {current_tree_edge_sum:.2f}")
                print(f"   True MST edge sum: {true_mst_edge_sum:.2f}")
                print(f"   Error (excess cost): {mst_error:.2f} (goal: minimize to 0)")
            
            # *** END MST COMPARISON ***
            
            # Draw main network
            environment_type = "Underwater" if self.enable_underwater_physics else "Surface"
            ax_network.set_title(f"{environment_type} Robot Network", 
                                fontsize=16, fontweight='bold', pad=20)
            
            # Set workspace limits with padding
            ax_network.set_xlim(-1, 11)
            ax_network.set_ylim(-1, 11)
            ax_network.grid(True, alpha=0.3)
            
            # Add axes labels with larger fonts
            ax_network.set_xlabel('X Position (meters)', fontsize=14, fontweight='bold')
            ax_network.set_ylabel('Y Position (meters)', fontsize=14, fontweight='bold')
            ax_network.tick_params(axis='both', labelsize=12)
            
            # Draw workspace boundary
            boundary_color = 'navy' if self.enable_underwater_physics else 'black'
            boundary_style = '-' if self.enable_underwater_physics else '--'
            boundary = plt.Rectangle((0, 0), 10, 10, fill=False, linewidth=3, 
                                   edgecolor=boundary_color, linestyle=boundary_style)
            ax_network.add_patch(boundary)
            
            # ENHANCED DEBUG: Add workspace boundary labels to make it crystal clear
            ax_network.text(5, -0.5, 'WORKSPACE: [0,10] x [0,10]', ha='center', va='top', 
                          fontsize=10, fontweight='bold', color=boundary_color)
            ax_network.text(-0.8, 5, 'Y: 0-10', ha='center', va='center', rotation=90,
                          fontsize=8, color=boundary_color)
            ax_network.text(5, 10.3, 'X: 0-10', ha='center', va='bottom',
                          fontsize=8, color=boundary_color)
            
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
            
            # Check if goal is within workspace bounds for visualization
            goal_in_workspace = (0 <= goal_x <= 10) and (0 <= goal_y <= 10)
            goal_color = 'gold' if goal_in_workspace else 'red'
            
            # Alert if goal is outside workspace (should not happen with clean generation)
            if not goal_in_workspace:
                print(f"🚨 ERROR: Goal outside workspace at frame {frame}: ({goal_x:.2f}, {goal_y:.2f})")
            
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
                            width = 4.0  # Significantly increased for better visibility
                            alpha = 1.0
                            style = '-'
                        elif edge in current_non_critical:
                            # Non-critical edges - can be pruned (BLUE) - Made much more prominent
                            color = 'blue' 
                            width = 3.5  # Significantly increased for better visibility
                            alpha = 0.9  # Increased for better visibility
                            style = '--'
                        else:
                            # Default (shouldn't happen but just in case)
                            color = 'gray'
                            width = 3.0  # Significantly increased for better visibility
                            alpha = 0.8  # Increased for better visibility
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
                        # Normalize velocity to unit vector and use fixed small length
                        vel_magnitude = np.linalg.norm(node.velocity)
                        vel_direction = node.velocity / vel_magnitude
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
                    Line2D([0], [0], color='yellow', marker='*', markersize=10, label='Goal', linestyle='None'),
                    Line2D([0], [0], color='red', linewidth=4.0, label='Critical Edges', linestyle='-'),
                    Line2D([0], [0], color='blue', linewidth=3.5, label='Non-Critical Edges', linestyle='--'),
                    Line2D([0], [0], marker='o', color='darkred', markersize=7, label='Active AUV', linestyle='None'),
                    Line2D([0], [0], marker='o', color='steelblue', markersize=5, label='Other AUVs', linestyle='None'),
                    Line2D([0], [0], color='cyan', marker='>', markersize=5, label='Flow Field', linestyle='None'),
                    Line2D([0], [0], color='orange', marker='>', markersize=5, label='Velocity', linestyle='None'),
                    Line2D([0], [0], color='red', marker='>', markersize=4, label='Drag Force', linestyle='--')
                ]
            else:
                legend_elements = [
                    Line2D([0], [0], color='gold', marker='*', markersize=10, label='Goal', linestyle='None'),
                    Line2D([0], [0], color='red', linewidth=4.0, label='Critical Edges', linestyle='-'),
                    Line2D([0], [0], color='blue', linewidth=3.5, label='Non-Critical Edges', linestyle='--'),
                    Line2D([0], [0], marker='o', color='red', markersize=7, label='Active Robot', linestyle='None'),
                    Line2D([0], [0], marker='o', color='lightblue', markersize=5, label='Other Robots', linestyle='None')
                ]
            ax_network.legend(handles=legend_elements, loc='lower right', 
                            fontsize=14, framealpha=0.95, fancybox=True, shadow=True, 
                            edgecolor='black', borderpad=0.8)
            
            # Draw statistics
            ax_stats.set_title("MST Error Analysis", fontsize=16, fontweight='bold', pad=20)
            if len(self.goal_distance_history) > 1:
                # Plot only the error for real-time view
                ax_stats.plot(self.mst_error_history, 'r-', linewidth=2.5, 
                             label='MST Error (Current - Optimal)', alpha=0.9)
                
                # Set labels and formatting with increased font sizes
                ax_stats.set_xlabel('Animation Steps', fontsize=14, fontweight='bold')
                ax_stats.set_ylabel('MST Error (Excess Cost)', color='red', fontsize=14, fontweight='bold')
                ax_stats.tick_params(axis='y', labelcolor='red', labelsize=12)
                ax_stats.tick_params(axis='x', labelsize=12)
                
                # Add horizontal line at error = 0 (perfect MST)
                ax_stats.axhline(y=0.0, color='orange', linestyle=':', linewidth=2, alpha=0.8, label='Perfect MST (Error = 0)')
                
                # Add legend with better formatting - positioned on upper right
                ax_stats.legend(loc='upper right', fontsize=13, framealpha=0.95, fancybox=True, 
                               shadow=True, edgecolor='black', borderpad=0.8)
                
                ax_stats.grid(True, alpha=0.3)
                
                # Add text summary of current error with better positioning
                if len(self.mst_error_history) > 0:
                    current_error = self.mst_error_history[-1]
                    error_text = f"Current Error: {current_error:.2f}"
                    if current_error <= 0.5:
                        error_color = 'green'
                        error_status = "🎯 NEAR OPTIMAL"
                    elif current_error <= 2.0:
                        error_color = 'orange'  
                        error_status = "⚠️ MODERATE ERROR"
                    else:
                        error_color = 'red'
                        error_status = "🔴 HIGH ERROR"
                    
                    ax_stats.text(0.02, 0.95, f"{error_status}\n{error_text}", 
                                 transform=ax_stats.transAxes, fontsize=9, fontweight='bold',
                                 verticalalignment='top', color=error_color,
                                 bbox=dict(boxstyle="round,pad=0.4", facecolor='white', alpha=0.9, edgecolor=error_color))
                
            else:
                ax_stats.text(0.5, 0.5, 'Collecting MST error data...\nWait for algorithm to generate data', 
                             ha='center', va='center', transform=ax_stats.transAxes, fontsize=11, 
                             fontweight='bold', color='gray',
                             bbox=dict(boxstyle="round,pad=0.5", facecolor='lightblue', alpha=0.7))
            
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
        
        # Improve layout with better spacing for 2-subplot layout - reduced gap
        plt.tight_layout(rect=[0, 0.05, 1, 0.92])  # Better spacing for 2 subplots
        plt.subplots_adjust(wspace=0.15)  # Reduced horizontal spacing between subplots
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
        
        # Generate comprehensive MST analysis plot after simulation
        if self.current_tree_edge_sum_history and self.true_mst_edge_sum_history and self.mst_error_history:
            print("\n📊 Generating Comprehensive MST Analysis...")
            
            # Create comprehensive MST plot with better spacing
            fig_mst, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 10))
            fig_mst.suptitle('Complete MST Analysis - Goal-Seeking Simulation Results', fontsize=14, fontweight='bold', y=0.98)
            
            frames = list(range(len(self.current_tree_edge_sum_history)))
            
            # Plot 1: Current Algorithm Edge Sum
            ax1.plot(frames, self.current_tree_edge_sum_history, 'g-', linewidth=2, alpha=0.8)
            ax1.set_title('Current Algorithm Edge Sum', fontweight='bold', fontsize=11)
            ax1.set_xlabel('Animation Steps', fontsize=9)
            ax1.set_ylabel('Edge Length', fontsize=9)
            ax1.tick_params(axis='both', which='major', labelsize=8)
            ax1.grid(True, alpha=0.3)
            ax1.text(0.98, 0.95, f'Final: {self.current_tree_edge_sum_history[-1]:.2f}', 
                    transform=ax1.transAxes, fontsize=9, fontweight='bold',
                    verticalalignment='top', horizontalalignment='right',
                    bbox=dict(boxstyle="round,pad=0.3", facecolor='lightgreen', alpha=0.8))
            
            # Plot 2: True MST Edge Sum
            ax2.plot(frames, self.true_mst_edge_sum_history, 'b-', linewidth=2, alpha=0.8)
            ax2.set_title('True MST Edge Sum (Optimal)', fontweight='bold', fontsize=11)
            ax2.set_xlabel('Animation Steps', fontsize=9)
            ax2.set_ylabel('Edge Length', fontsize=9)
            ax2.tick_params(axis='both', which='major', labelsize=8)
            ax2.grid(True, alpha=0.3)
            ax2.text(0.98, 0.95, f'Final: {self.true_mst_edge_sum_history[-1]:.2f}', 
                    transform=ax2.transAxes, fontsize=9, fontweight='bold',
                    verticalalignment='top', horizontalalignment='right',
                    bbox=dict(boxstyle="round,pad=0.3", facecolor='lightblue', alpha=0.8))
            
            # Plot 3: MST Error (The main result!)
            ax3.plot(frames, self.mst_error_history, 'r-', linewidth=2.5, alpha=0.9)
            ax3.axhline(y=0.0, color='orange', linestyle=':', linewidth=2, alpha=0.8, label='Perfect MST')
            ax3.set_title('MST Error: Algorithm vs Optimal', fontweight='bold', fontsize=11)
            ax3.set_xlabel('Animation Steps', fontsize=9)
            ax3.set_ylabel('Error (Current - MST)', fontsize=9)
            ax3.tick_params(axis='both', which='major', labelsize=8)
            ax3.legend(fontsize=8, loc='upper right')
            ax3.grid(True, alpha=0.3)
            
            # Add error statistics (smaller, better positioned)
            final_error = self.mst_error_history[-1]
            initial_error = self.mst_error_history[0]
            improvement = initial_error - final_error
            if final_error <= 0.5:
                error_color = 'green'
                error_status = "🎯 EXCELLENT"
            elif final_error <= 2.0:
                error_color = 'orange'  
                error_status = "⚠️ GOOD"
            else:
                error_color = 'red'
                error_status = "🔴 IMPROVE"
                
            ax3.text(0.02, 0.98, f'{error_status}\nFinal: {final_error:.2f}\nΔ: -{improvement:.2f}', 
                    transform=ax3.transAxes, fontsize=8, fontweight='bold',
                    verticalalignment='top', color=error_color,
                    bbox=dict(boxstyle="round,pad=0.25", facecolor='white', alpha=0.9))
            
            # Plot 4: All three lines together (simplified)
            ax4.plot(frames, self.current_tree_edge_sum_history, 'g-', linewidth=1.5, alpha=0.8, label='Algorithm')
            ax4.plot(frames, self.true_mst_edge_sum_history, 'b-', linewidth=1.5, alpha=0.8, label='MST Optimal')
            ax4_twin = ax4.twinx()
            ax4_twin.plot(frames, self.mst_error_history, 'r-', linewidth=2, alpha=0.9, label='Error')
            ax4_twin.axhline(y=0.0, color='orange', linestyle=':', linewidth=1, alpha=0.7)
            
            ax4.set_title('Complete Comparison', fontweight='bold', fontsize=20)
            ax4.set_xlabel('Animation Steps', fontsize=9)
            ax4.set_ylabel('Edge Length', fontsize=9, color='black')
            ax4_twin.set_ylabel('Error', fontsize=9, color='red')
            ax4.tick_params(axis='both', which='major', labelsize=8)
            ax4_twin.tick_params(axis='y', labelcolor='red', labelsize=8)
            
            # Simplified legends
            lines1, labels1 = ax4.get_legend_handles_labels()
            lines2, labels2 = ax4_twin.get_legend_handles_labels()
            ax4.legend(lines1 + lines2, labels1 + labels2, fontsize=8, loc='upper right')
            ax4.grid(True, alpha=0.3)
            
            # Adjust layout to prevent overlaps
            plt.subplots_adjust(left=0.08, bottom=0.08, right=0.92, top=0.92, wspace=0.25, hspace=0.35)
            plt.show()
            
            print("✅ MST Analysis Complete!")
            print(f"Final MST Error: {final_error:.2f} (Lower is better)")
            if initial_error > 0:
                print(f"Error Reduction: {improvement:.2f} ({((improvement/initial_error)*100):.1f}% improvement)")
            else:
                print("Error Reduction: N/A (started at optimal)")
        
        # Plot CLF convergence after simulation
        if self.clf_distance_history:
            print("\n📊 Generating CLF Convergence Analysis...")
            
            # Create CLF convergence plot
            plt.figure(figsize=(12, 8))
            
            # Main CLF convergence plot
            plt.subplot(2, 2, 1)
            frames = list(range(len(self.clf_distance_history)))
            plt.plot(frames, self.clf_distance_history, 'b-', linewidth=2, label='Average Distance to Goal')
            plt.axhline(y=self.goal_tolerance, color='red', linestyle='--', alpha=0.7, label='Goal Threshold')
            plt.title('CLF Convergence: Distance to Goal Over Time', fontweight='bold', fontsize = 20)
            plt.xlabel('Animation Frame')
            plt.ylabel('Average Distance to Goal')
            plt.legend()
            plt.grid(True, alpha=0.3)
            
            # Connectivity over time
            plt.subplot(2, 2, 2)
            if self.connectivity_history:
                plt.plot(list(range(len(self.connectivity_history))), self.connectivity_history, 'g-', linewidth=2, label='Edge Count')
                plt.title('Network Edge Count Over Time', fontweight='bold')
                plt.xlabel('Animation Frame')
                plt.ylabel('Number of Edges')
                plt.legend()
                plt.grid(True, alpha=0.3)
            
            # Closest robot distance over time
            plt.subplot(2, 2, 3)
            if self.goal_distance_history:
                plt.plot(list(range(len(self.goal_distance_history))), self.goal_distance_history, 'r-', linewidth=2, label='Closest Robot Distance')
                plt.axhline(y=self.goal_tolerance, color='red', linestyle='--', alpha=0.7, label='Goal Threshold')
                plt.title('Closest Robot Distance to Goal', fontweight='bold', fontsize=20)
                plt.xlabel('Animation Frame')
                plt.ylabel('Distance', fontsize=14)
                plt.legend()
                plt.grid(True, alpha=0.3)
            
            # Final statistics
            plt.subplot(2, 2, 4)
            final_distance = self.clf_distance_history[-1]
            initial_distance = self.clf_distance_history[0]
            total_convergence = (initial_distance - final_distance) / initial_distance * 100 if initial_distance > 0 else 0
            
            stats_text = f"CLF Analysis Summary:\n\n"
            stats_text += f"Initial Avg Distance: {initial_distance:.3f}\n"
            stats_text += f"Final Avg Distance: {final_distance:.3f}\n"
            stats_text += f"Total Convergence: {total_convergence:.1f}%\n"
            stats_text += f"Goal Threshold: {self.goal_tolerance:.3f}\n"
            stats_text += f"Goal Achievement: {'✅ Yes' if self.goal_reached else '❌ No'}\n\n"
            stats_text += f"Simulation completed in {len(self.clf_distance_history)} frames\n"
            stats_text += f"Network connectivity maintained: {'✅ Yes' if self.is_network_connected() else '❌ No'}\n"
            stats_text += f"CLF shows {'hyperbolic' if total_convergence > 50 else 'gradual'} convergence pattern"
            
            plt.text(0.05, 0.95, stats_text, transform=plt.gca().transAxes, 
                    fontsize=11, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))
            plt.axis('off')
            plt.title('CLF Convergence Statistics', fontweight='bold')
            
            plt.suptitle('Decentralized Goal-Seeking: CLF Convergence Analysis', fontsize=14, fontweight='bold')
            plt.tight_layout()
            plt.show()
            
            print(f"✅ CLF Analysis: {total_convergence:.1f}% convergence achieved")
            print(f"   Final average distance to goal: {final_distance:.3f}")
        
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
        Compute Control Barrier Function for connectivity constraint
        CBF: h(x) = R^2 - ||x - x_neighbor||^2 for each neighbor
        CBF derivative: dh/dx = -2(x - x_neighbor)
        Returns constraints for maintaining connectivity
        """
        cbf_constraints = []
        
        for neighbor_pos in neighbor_positions:
            # Distance to neighbor
            diff = robot_position - neighbor_pos
            distance_sq = np.dot(diff, diff)
            
            # CBF: h(x) = R^2 - ||x - x_neighbor||^2
            # We want h(x) >= 0 to stay within communication range
            h = max_range**2 - distance_sq
            dh_dx = -2 * diff  # Gradient of h
            
            cbf_constraints.append({
                'h': h,
                'dh_dx': dh_dx,
                'neighbor_pos': neighbor_pos,
                'distance': np.sqrt(distance_sq),
                'max_range': max_range  # Include max_range for emergency braking logic
            })
        
        return cbf_constraints
    
    def compute_safety_constraints(self, robot_position, all_robot_positions):
        """
        Compute safety distance constraints for all other robots
        Safety CBF: h_safety(x) = ||x - x_other||^2 - d_min^2
        We want h_safety(x) >= 0 to maintain minimum distance
        """
        safety_distance = 0.6  # Minimum distance - INCREASED for better visibility
        safety_constraints = []
        
        for other_pos in all_robot_positions:
            # Distance to other robot
            diff = robot_position - other_pos
            distance_sq = np.dot(diff, diff)
            distance = np.sqrt(distance_sq)
            
            # Safety CBF: h = ||x - x_other||^2 - d_min^2
            # We want h >= 0, so distance >= d_min
            h = distance_sq - safety_distance**2
            dh_dx = 2 * diff  # Gradient points away from other robot
            
            safety_constraints.append({
                'h': h,
                'dh_dx': dh_dx,
                'other_pos': other_pos,
                'distance': distance,
                'min_distance': safety_distance
            })
        
        return safety_constraints
    
    def solve_clf_cbf_qp(self, robot_position, goal_position, neighbor_positions, all_robot_positions, max_range):
        """
        Solve CLF-CBF Quadratic Program to find optimal control input
        
        Formulation:
        minimize: u^T u + gamma^2
        subject to: 
        - CLF constraint: dV/dx * u <= -alpha * V + gamma (goal reaching)
        - CBF constraint: dh/dx * u >= -beta * h (connectivity maintenance)
        - Safety constraint: maintain minimum distance from all other robots
        - Control bounds: ||u|| <= u_max
        """
        try:
            # Compute CLF for goal-seeking
            V, dV_dx = self.compute_clf_goal_seeking(robot_position, goal_position)
            
            # Compute CBF for connectivity (only direct neighbors)
            cbf_constraints = self.compute_cbf_connectivity(robot_position, neighbor_positions, max_range)
            
            # Compute safety constraints for all robots
            safety_constraints = self.compute_safety_constraints(robot_position, all_robot_positions)
            
            if CVXPY_AVAILABLE:
                return self._solve_clf_cbf_cvxpy(V, dV_dx, cbf_constraints, safety_constraints)
            else:
                return self._solve_clf_cbf_scipy(V, dV_dx, cbf_constraints, safety_constraints)
                
        except Exception as e:
            print(f"CLF-CBF QP failed: {e}, using fallback control")
            # Fallback: simple proportional control toward goal with reasonable speed
            direction_to_goal = goal_position - robot_position
            if np.linalg.norm(direction_to_goal) > 0:
                return 0.10 * direction_to_goal / np.linalg.norm(direction_to_goal)  # INCREASED
            else:
                return np.zeros(2)
    
    def _solve_clf_cbf_cvxpy(self, V, dV_dx, cbf_constraints, safety_constraints):
        """Solve CLF-CBF QP using cvxpy with safety constraints"""
        # Control input (2D velocity)
        u = cp.Variable(2)
        gamma = cp.Variable(1)  # Relaxation variable for CLF
        
        # Parameters - TUNED FOR FASTER GOAL APPROACH while maintaining stability
        alpha = 1.5  # CLF decay rate - REDUCED from 3.0 to allow faster movement
        beta = 5.0   # CBF safety margin - INCREASED from 1.0
        beta_safety = 10.0  # Safety CBF margin - VERY HIGH PRIORITY
        u_max = 0.40  # Maximum control input magnitude - INCREASED from 0.25 for faster movement
        
        # Objective: minimize control effort + relaxation penalty - REDUCED PENALTY for faster movement
        objective = cp.Minimize(cp.sum_squares(u) + 50 * cp.sum_squares(gamma))
        
        constraints = []
        
        # CLF constraint: dV/dx * u <= -alpha * V + gamma
        if V > 1e-6:  # Only apply if not already at goal
            constraints.append(dV_dx @ u <= -alpha * V + gamma)
            constraints.append(gamma >= 0)  # Relaxation variable is non-negative
        
        # CBF constraints: dh/dx * u >= -beta * h for each neighbor (connectivity)
        emergency_connectivity = False
        for cbf in cbf_constraints:
            if cbf['h'] > 0:  # Only apply if constraint is active (still connected)
                # Check for emergency connectivity preservation condition
                max_range_sq = cbf.get('max_range', 1.2)**2
                if cbf['h'] < 0.25 * max_range_sq:  # Within 25% of communication limit
                    emergency_connectivity = True
                    # Strong constraint: force velocity toward neighbor to maintain connectivity
                    # dh/dx points toward neighbor, so positive dh/dx @ u means moving toward neighbor
                    constraints.append(cbf['dh_dx'] @ u >= 0.20)  # Strong attractive constraint
                else:
                    # Normal CBF constraint
                    constraints.append(cbf['dh_dx'] @ u >= -beta * cbf['h'])
            elif cbf['h'] <= 0:
                # Connection lost! Force strong movement toward neighbor
                emergency_connectivity = True
                constraints.append(cbf['dh_dx'] @ u >= 0.50)  # Very strong attractive constraint
        
        # SAFETY CBF constraints: dh_safety/dx * u >= -beta_safety * h_safety (collision avoidance)
        emergency_safety = False
        for safety in safety_constraints:
            if safety['h'] < 0.1:  # Very close to minimum distance or violating it
                emergency_safety = True
                # Force movement away from other robot
                constraints.append(safety['dh_dx'] @ u >= 1.0)  # VERY strong repulsive constraint
            elif safety['h'] < 0.5:  # Getting close to minimum distance
                # Normal safety constraint
                constraints.append(safety['dh_dx'] @ u >= -beta_safety * safety['h'])
        
        # Adjust control limits based on emergency conditions
        if emergency_connectivity:
            u_max = min(u_max, 0.40)  # Allow larger movements to restore connectivity
        if emergency_safety:
            u_max = min(u_max, 0.30)  # Limit movement when safety is critical
        
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
    
    def _solve_clf_cbf_scipy(self, V, dV_dx, cbf_constraints, safety_constraints):
        """Solve CLF-CBF QP using scipy (fallback when cvxpy not available) with safety constraints"""
        from scipy.optimize import minimize
        
        # Parameters - TUNED FOR FASTER GOAL APPROACH while maintaining stability
        alpha = 1.5  # REDUCED from 3.0 to allow faster movement toward goal
        beta = 5.0   # INCREASED from 1.0
        u_max = 0.40  # INCREASED from 0.25 for faster movement
        
        def objective(x):
            u = x[:2]
            gamma = x[2] if len(x) > 2 else 0
            return np.dot(u, u) + 50 * gamma**2  # REDUCED penalty from 200 to 50
        
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
        
        # Add safety distance constraints
        if safety_constraints is not None:
            for constraint in safety_constraints:
                h_safety = constraint['h']
                dh_dx_safety = constraint['dh_dx']
                
                if h_safety <= 0.05:  # Very close - emergency repulsion
                    def emergency_safety_constraint(x, dh_dx=dh_dx_safety):
                        u = x[:2]
                        return np.dot(dh_dx, u) + 1.0  # Strong repulsive force
                    constraints.append({'type': 'ineq', 'fun': emergency_safety_constraint})
                elif h_safety <= 0.2:  # Close - apply safety CBF
                    beta_safety = 10.0  # High priority for safety
                    def safety_cbf_constraint(x, h=h_safety, dh_dx=dh_dx_safety):
                        u = x[:2]
                        return np.dot(dh_dx, u) + beta_safety * h
                    constraints.append({'type': 'ineq', 'fun': safety_cbf_constraint})
        
        # CBF constraints with emergency connectivity preservation - CORRECTED
        emergency_connectivity = False
        for cbf in cbf_constraints:
            if cbf['h'] > 0:
                # Check for emergency connectivity preservation condition
                max_range_sq = cbf.get('max_range', 1.2)**2
                if cbf['h'] < 0.25 * max_range_sq:  # Within 25% of communication limit
                    emergency_connectivity = True
                    # Strong constraint: force velocity toward neighbor to maintain connectivity
                    def emergency_cbf_constraint(x, dh_dx=cbf['dh_dx']):
                        u = x[:2]
                        return np.dot(dh_dx, u) - 0.20  # Strong attractive force
                    constraints.append({'type': 'ineq', 'fun': emergency_cbf_constraint})
                else:
                    def cbf_constraint(x, dh_dx=cbf['dh_dx'], h=cbf['h']):
                        u = x[:2]
                        return np.dot(dh_dx, u) + beta * h
                    constraints.append({'type': 'ineq', 'fun': cbf_constraint})
            elif cbf['h'] <= 0:
                # Connection lost! Force strong movement toward neighbor
                emergency_connectivity = True
                def lost_connection_constraint(x, dh_dx=cbf['dh_dx']):
                    u = x[:2]
                    return np.dot(dh_dx, u) - 0.50  # Very strong attractive force
                constraints.append({'type': 'ineq', 'fun': lost_connection_constraint})
        
        # If emergency connectivity preservation is active, allow larger movements
        if emergency_connectivity:
            u_max = min(u_max, 0.40)  # Allow larger movements to restore connectivity
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

    def run_distance_based_edge_pruning(self):
        """
        Enhanced distance-based edge pruning algorithm with edge recoupling:
        Phase 1: Remove longest edges that don't disconnect the network
        Phase 2: Add beneficial shorter edges from k-hop neighbors
        
        This combines the simplicity of distance-based pruning with the 
        edge recoupling benefits of the triangular method.
        """
        print("\n📏 Running Enhanced Distance-Based Edge Pruning with Recoupling...")
        all_changes = {'added': [], 'removed': []}
        
        # Randomize order to simulate asynchronous execution
        robot_ids = list(self.nodes.keys())
        random.shuffle(robot_ids)
        
        edge_changes_made = False
        total_edges_checked = 0
        
        # Phase 1: Distance-based pruning (remove longest edges)
        print("\n🔸 PHASE 1: Distance-based edge removal")
        for robot_id in robot_ids:
            changes = self.prune_longest_edges(robot_id)
            all_changes['added'].extend(changes['added'])
            all_changes['removed'].extend(changes['removed'])
            total_edges_checked += 1
            
            if changes['added'] or changes['removed']:
                edge_changes_made = True
        
        # Phase 2: Edge recoupling (add beneficial shorter edges)
        print("\n🔸 PHASE 2: Edge recoupling with k-hop neighbors")
        for robot_id in robot_ids:
            changes = self.add_beneficial_edges(robot_id, k_max=2)
            all_changes['added'].extend(changes['added'])
            all_changes['removed'].extend(changes['removed'])
            
            if changes['added'] or changes['removed']:
                edge_changes_made = True
        
        if edge_changes_made:
            print(f"✅ Enhanced distance-based pruning complete:")
            print(f"   🔗 {len(all_changes['added'])} edges added")
            print(f"   ✂️ {len(all_changes['removed'])} edges removed")
            print(f"📊 Checked {total_edges_checked} robots for edge optimization")
            
            # Print current network statistics
            total_edges = sum(len(node.direct_neighbors) for node in self.nodes.values()) // 2
            avg_degree = total_edges * 2 / len(self.nodes)
            print(f"📊 Network stats: {total_edges} edges, avg degree: {avg_degree:.2f}")
        else:
            print("✅ No edges modified - network is optimally configured")
        
        return all_changes
    
    def prune_longest_edges(self, robot_id):
        """
        Phase 1: Remove longest edges for a single robot (if safe)
        """
        changes = {'added': [], 'removed': []}
        robot = self.nodes[robot_id]
        
        if len(robot.direct_neighbors) <= 1:
            return changes  # Skip robots with 0 or 1 neighbors
        
        # Get all neighbors and their distances
        neighbor_distances = []
        for neighbor_id in robot.direct_neighbors:
            neighbor_pos = self.nodes[neighbor_id].position
            distance = np.linalg.norm(robot.position - neighbor_pos)
            neighbor_distances.append((neighbor_id, distance))
        
        # Sort by distance (shortest first)
        neighbor_distances.sort(key=lambda x: x[1])
        
        # Try to remove the longest distance edge (if it doesn't disconnect network)
        if len(neighbor_distances) > 1:  # Need at least 2 neighbors to remove one
            longest_neighbor_id, longest_distance = neighbor_distances[-1]
            
            # Check if removing this edge would disconnect the network
            if not self.would_disconnect_network(robot_id, longest_neighbor_id):
                # Safe to remove
                robot.direct_neighbors.discard(longest_neighbor_id)
                self.nodes[longest_neighbor_id].direct_neighbors.discard(robot_id)
                
                edge_removed = tuple(sorted([robot_id, longest_neighbor_id]))
                changes['removed'].append(edge_removed)
                
                print(f"   🔸 Robot {robot_id}: Removed longest edge to {longest_neighbor_id} (distance: {longest_distance:.2f})")
        
        return changes
    
    def add_beneficial_edges(self, robot_id, k_max=2):
        """
        Phase 2: Add beneficial shorter edges using conservative edge swapping
        
        Like the triangular method, only add edges if they can replace longer ones,
        ensuring network improvement without over-connectivity.
        """
        changes = {'added': [], 'removed': []}
        
        # Discover k-hop neighbors
        k_hop_neighbors = self.discover_k_hop_neighbors(robot_id, k_max)
        
        if len(k_hop_neighbors) == 0:
            return changes
        
        current_neighbors = list(self.nodes[robot_id].direct_neighbors)
        robot_pos = self.nodes[robot_id].position
        
        # Find longest current connection (candidate for replacement)
        longest_current_distance = 0.0
        longest_current_neighbor = None
        
        for neighbor_id in current_neighbors:
            distance = np.linalg.norm(robot_pos - self.nodes[neighbor_id].position)
            if distance > longest_current_distance:
                longest_current_distance = distance
                longest_current_neighbor = neighbor_id
        
        # Find shortest potential new connection within communication range
        shortest_new_distance = float('inf')
        shortest_new_neighbor = None
        
        for target_id, info in k_hop_neighbors.items():
            if target_id not in current_neighbors:  # Not already connected
                distance = np.linalg.norm(robot_pos - info['position'])
                
                # Must be within communication range
                if distance <= 1.2 and distance < shortest_new_distance:
                    shortest_new_distance = distance
                    shortest_new_neighbor = target_id
        
        # Conservative edge swapping: only add if we can replace a longer edge
        if (shortest_new_neighbor and longest_current_neighbor and 
            shortest_new_distance < longest_current_distance and 
            len(current_neighbors) > 0):  # Ensure we don't disconnect
            
            improvement = longest_current_distance - shortest_new_distance
            improvement_threshold = 0.05  # Minimum improvement required
            
            if improvement > improvement_threshold:
                # Check connectivity before edge swap
                # Add new edge temporarily 
                self.nodes[robot_id].direct_neighbors.add(shortest_new_neighbor)
                if shortest_new_neighbor in self.nodes:
                    self.nodes[shortest_new_neighbor].direct_neighbors.add(robot_id)
                
                # Test: can we remove the old edge without breaking connectivity?
                if self.is_network_connected_without_edge(robot_id, longest_current_neighbor):
                    # Safe to remove old edge
                    self.nodes[robot_id].direct_neighbors.discard(longest_current_neighbor)
                    if longest_current_neighbor in self.nodes:
                        self.nodes[longest_current_neighbor].direct_neighbors.discard(robot_id)
                    
                    edge_added = tuple(sorted([robot_id, shortest_new_neighbor]))
                    edge_removed = tuple(sorted([robot_id, longest_current_neighbor]))
                    changes['added'].append(edge_added)
                    changes['removed'].append(edge_removed)
                    
                    print(f"   � Robot {robot_id}: Edge swap - Added {shortest_new_neighbor} (dist: {shortest_new_distance:.2f}), Removed {longest_current_neighbor} (dist: {longest_current_distance:.2f})")
                    
                else:
                    # Unsafe - revert the temporary addition
                    self.nodes[robot_id].direct_neighbors.discard(shortest_new_neighbor)
                    if shortest_new_neighbor in self.nodes:
                        self.nodes[shortest_new_neighbor].direct_neighbors.discard(robot_id)
                    
                    print(f"   ❌ Robot {robot_id}: Edge swap would break connectivity - skipped")
        
        return changes
    
    def would_disconnect_network(self, node1_id, node2_id):
        """
        Check if removing edge between node1 and node2 would disconnect the network.
        Uses BFS to verify connectivity after temporarily removing the edge.
        """
        # Temporarily remove the edge
        original_neighbors_1 = self.nodes[node1_id].direct_neighbors.copy()
        original_neighbors_2 = self.nodes[node2_id].direct_neighbors.copy()
        
        self.nodes[node1_id].direct_neighbors.discard(node2_id)
        self.nodes[node2_id].direct_neighbors.discard(node1_id)
        
        # Check if network is still connected
        is_connected = self.is_network_connected()
        
        # Restore the edge
        self.nodes[node1_id].direct_neighbors = original_neighbors_1
        self.nodes[node2_id].direct_neighbors = original_neighbors_2
        
        return not is_connected

    def update_positions_goal_seeking(self, use_controller=True):
        """Update robot positions with goal-seeking and critical edge preservation
        If use_controller=False, apply only Brownian motion and flow field effects
        """
        if self.closest_robot_id is None:
            return
            
        # If no controller, apply simple Brownian motion to all robots
        if not use_controller:
            print("🚨 BROWNIAN MOTION ONLY - NO CBF CONTROL")
            for node_id, node in self.nodes.items():
                # Reset velocity
                node.velocity = np.zeros(2)
                
                # Apply very strong Brownian motion for dramatic effect
                brownian_intensity = 0.8  # Increased from 0.35 to 0.8 for much stronger motion
                brownian_force = np.random.normal(0, brownian_intensity, 2)
                node.velocity += brownian_force
                
                # Apply stronger flow field effects if underwater environment exists
                if hasattr(self, 'underwater_env') and self.underwater_env is not None:
                    flow_effect = self.underwater_env.get_flow_field(node.position) * 0.5  # Increased from 0.3 to 0.5
                    node.velocity += flow_effect
                
                # Update position with larger time step for more visible movement
                new_position = node.position + node.velocity * 0.15  # Increased from 0.1 to 0.15
                
                # Apply workspace boundaries (0,0) to (10,10) for goal-seeking simulation
                x_min, x_max, y_min, y_max = self.workspace_bounds  # [0, 10, 0, 10]
                new_position[0] = np.clip(new_position[0], x_min, x_max)
                new_position[1] = np.clip(new_position[1], y_min, y_max)
                
                node.position = new_position
            
            # Natural edge removal - remove edges when robots are too far apart (no connectivity preservation)
            max_natural_range = 1.5  # Natural communication range without CBF preservation
            edges_to_remove = []
            
            for node_id, node in self.nodes.items():
                neighbors_to_remove = []
                for neighbor_id in node.direct_neighbors.copy():  # Use copy to avoid modification during iteration
                    distance = np.linalg.norm(node.position - self.nodes[neighbor_id].position)
                    if distance > max_natural_range:
                        edge = tuple(sorted([node_id, neighbor_id]))
                        if edge not in edges_to_remove:
                            edges_to_remove.append(edge)
            
            # Remove edges that are too far - NO connectivity preservation
            for edge in edges_to_remove:
                node1, node2 = edge
                if node2 in self.nodes[node1].direct_neighbors:
                    self.nodes[node1].direct_neighbors.remove(node2)
                if node1 in self.nodes[node2].direct_neighbors:
                    self.nodes[node2].direct_neighbors.remove(node1)
                print(f"   -> Natural edge removal: {node1}-{node2} (distance too large)")
            
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
        max_communication_range = 1.2  # Reduced from 2.8 for more aggressive pruning
        
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
        
        # STEP 3: Aggressively prune non-critical edges that exceed communication range
        edges_to_remove = []
        for edge in non_critical_edges:
            node1, node2 = edge
            distance = np.linalg.norm(self.nodes[node1].position - self.nodes[node2].position)
            
            # More aggressive pruning - remove non-critical edges at 80% of max range
            if distance > max_communication_range * 0.8:
                edges_to_remove.append(edge)
        
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
            elif edge_existed:
                # Network would disconnect, restore the edge
                self.nodes[node1].direct_neighbors.add(node2)
                self.nodes[node2].direct_neighbors.add(node1)
        
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
        spring_strength = 0.12  # INCREASED from 0.06 for better movement
        max_communication_range = 1.2  # Same as pruning logic
        
        for node_id, node in self.nodes.items():
            # Initialize velocities if not present
            if not hasattr(node, 'velocity'):
                node.velocity = np.random.normal(0, 0.02, 2)
            
            # CLF-CBF CONTROL for active robot, force-based for others
            if node_id == self.closest_robot_id:
                # === CLF-CBF CONTROL FOR ACTIVE ROBOT ===
                # DEBUG: Only print occasionally to avoid spam
                if hasattr(self, 'clf_print_counter'):
                    self.clf_print_counter += 1
                else:
                    self.clf_print_counter = 0
                
                if self.clf_print_counter % 50 == 0:  # Print every 50 calls instead of every call
                    print(f"🤖 Using CLF-CBF control for active robot {node_id}")
                
                # Get neighbor positions for CBF constraints
                neighbor_positions = []
                for neighbor_id in node.direct_neighbors:
                    neighbor_positions.append(self.nodes[neighbor_id].position)
                
                # Get ALL robot positions for safety distance constraints
                all_robot_positions = []
                for other_id, other_node in self.nodes.items():
                    if other_id != node_id:  # Exclude self
                        all_robot_positions.append(other_node.position)
                
                # Solve CLF-CBF QP with safety constraints
                if len(neighbor_positions) > 0 or len(all_robot_positions) > 0:
                    control_input = self.solve_clf_cbf_qp(
                        robot_position=node.position,
                        goal_position=self.goal_location,
                        neighbor_positions=neighbor_positions,
                        all_robot_positions=all_robot_positions,
                        max_range=max_communication_range
                    )
                else:
                    # No neighbors - just go toward goal with faster speed
                    direction_to_goal = self.goal_location - node.position
                    if np.linalg.norm(direction_to_goal) > 0:
                        control_input = 0.25 * direction_to_goal / np.linalg.norm(direction_to_goal)  # INCREASED from 0.10
                    else:
                        control_input = np.zeros(2)
                
                # Apply CLF-CBF control
                node.velocity = control_input
                
                # Debug output for CLF-CBF
                if hasattr(self, 'clf_cbf_debug_counter'):
                    self.clf_cbf_debug_counter += 1
                else:
                    self.clf_cbf_debug_counter = 0
                
                if self.clf_cbf_debug_counter % 30 == 0:  # Every 30 frames
                    dist_to_goal = np.linalg.norm(node.position - self.goal_location)
                    print(f"🎯 CLF-CBF Control Robot {node_id}:")
                    print(f"   Distance to goal: {dist_to_goal:.3f}")
                    print(f"   Control input: [{control_input[0]:.4f}, {control_input[1]:.4f}]")
                    print(f"   Connected neighbors: {len(neighbor_positions)}")
                    for i, neighbor_pos in enumerate(neighbor_positions):
                        dist = np.linalg.norm(node.position - neighbor_pos)
                        print(f"   Neighbor {i+1} distance: {dist:.3f} (max: {max_communication_range:.3f})")
            
            else:
                # === CBF-ENHANCED FORCE-BASED CONTROL FOR NON-ACTIVE ROBOTS ===
                # Apply CBF constraints to ALL robots for connectivity preservation
                
                # Get neighbor positions for CBF constraints
                neighbor_positions = []
                for neighbor_id in node.direct_neighbors:
                    neighbor_positions.append(self.nodes[neighbor_id].position)
                
                if len(neighbor_positions) > 0:
                    # Compute CBF constraints for this robot
                    cbf_constraints = self.compute_cbf_connectivity(
                        node.position, neighbor_positions, max_communication_range
                    )
                    
                    # Check if any CBF constraint is being violated
                    cbf_force = np.zeros(2)
                    for cbf in cbf_constraints:
                        # CBF h = R^2 - distance^2
                        # h > 0: robots connected, h ≤ 0: robots disconnected
                        current_distance = cbf['distance']
                        max_range = max_communication_range
                        
                        # Apply CBF if robots are too far apart or disconnected
                        should_apply_cbf = (current_distance > 0.6 * max_range) or (cbf['h'] <= 0)
                        
                        if should_apply_cbf and current_distance > 0:
                            if cbf['h'] <= 0:
                                # DISCONNECTED: Apply very strong attractive force to reconnect
                                force_magnitude = 2.0  # Very strong force for disconnected robots
                                if node_id == 1 and self.debug_position_counter % 30 == 0:
                                    print(f"🚨 CBF DISCONNECTED Robot {node_id}: dist={current_distance:.3f}, max={max_range:.3f}, force_mag={force_magnitude:.3f}")
                            else:
                                # CONNECTED but getting far: Apply proportional attractive force
                                distance_ratio = current_distance / max_range  # 0.7 to 1.0
                                force_magnitude = 1.0 * (distance_ratio - 0.7) / 0.3  # Scale from 0 to 1
                                force_magnitude = min(1.5, max(0.1, force_magnitude))  # Clamp between 0.1 and 1.5
                                if node_id == 1 and self.debug_position_counter % 30 == 0:
                                    print(f"🛡️ CBF CONNECTED Robot {node_id}: dist={current_distance:.3f}, ratio={distance_ratio:.3f}, force_mag={force_magnitude:.3f}")
                            
                            # Direction: toward neighbor to reduce distance
                            neighbor_pos = cbf['neighbor_pos']
                            direction_to_neighbor = neighbor_pos - node.position
                            direction_normalized = direction_to_neighbor / np.linalg.norm(direction_to_neighbor)
                            
                            cbf_force += direction_normalized * force_magnitude
                
                # Base Brownian motion for exploration (REDUCED due to CBF)
                brownian_force = np.random.normal(0, 0.01, 2)  # FURTHER REDUCED to let CBF dominate
                total_force = brownian_force + cbf_force  # Add CBF force
                
                # Add minimum distance constraints (safety distance) - STRENGTHENED
                safety_distance = 0.6  # Minimum distance between robots - INCREASED for better visibility
                safety_force = np.zeros(2)
                for other_id, other_node in self.nodes.items():
                    if other_id != node_id:
                        direction_away = node.position - other_node.position
                        distance = np.linalg.norm(direction_away)
                        
                        if distance > 0 and distance < safety_distance:
                            # MUCH stronger repulsive force when too close - dominates all other forces
                            force_magnitude = 2.5 * (safety_distance - distance) / safety_distance  # INCREASED from 0.10
                            safety_force += (direction_away / distance) * force_magnitude
                            
                            # Debug safety enforcement
                            if node_id == 1 and self.debug_position_counter % 30 == 0:
                                print(f"⚠️ SAFETY Robot {node_id}: too close to robot {other_id}, dist={distance:.3f}, force_mag={force_magnitude:.3f}")
                
                total_force += safety_force
                
                # Smart connectivity forces based on CRITICAL EDGES ONLY (REDUCED due to CBF)
                for neighbor_id in node.direct_neighbors:
                    neighbor = self.nodes[neighbor_id]
                    direction = neighbor.position - node.position
                    distance = np.linalg.norm(direction)
                    
                    if distance > 0:
                        edge = tuple(sorted([node_id, neighbor_id]))
                        is_critical = edge in critical_edges
                        
                        if is_critical:
                            # MODERATE force to maintain critical connections (REDUCED)
                            if distance > max_communication_range * 0.8:
                                # Pull toward critical neighbors when far
                                spring_force = (direction / distance) * spring_strength * 1.5  # REDUCED from 3.0
                                total_force += spring_force
                        else:
                            # WEAK force for non-critical connections - allow natural pruning
                            if distance > max_communication_range * 0.9:
                                spring_force = (direction / distance) * spring_strength * 0.3  # REDUCED
                                total_force += spring_force
                
                # Spreading force - encourage robots to spread out intelligently
                spreading_force = np.zeros(2)
                for other_id, other_node in self.nodes.items():
                    if other_id != node_id:
                        direction_away = node.position - other_node.position
                        distance = np.linalg.norm(direction_away)
                        
                        # Only spread from very close robots (avoid clustering)
                        if distance > 0 and distance < 1.2:
                            edge = tuple(sorted([node_id, other_id]))
                            is_connected = other_id in node.direct_neighbors
                            is_critical = edge in critical_edges if is_connected else False
                            
                            # Stronger spreading if not critically connected
                            spread_strength = 0.015 if is_critical else 0.025
                            spreading_force += (direction_away / distance) * spread_strength * (1.2 - distance)
                
                total_force += spreading_force
                
                # Update velocity with momentum (force-based control)
                if not self.enable_underwater_physics:
                    # Original simple dynamics
                    node.velocity = 0.75 * node.velocity + total_force
                else:
                    # Underwater physics will handle velocity update
                    control_force = total_force
            
            # Apply underwater dynamics or simple position update
            if self.enable_underwater_physics:
                # Use advanced underwater physics
                if node_id == self.closest_robot_id:
                    # Active robot uses CLF-CBF control
                    self.apply_underwater_dynamics(node, control_input)
                else:
                    # Other robots use force-based control
                    self.apply_underwater_dynamics(node, control_force)
                
                # Update position after applying physics
                old_position = node.position.copy()
                node.position += node.velocity * self.dt
                position_change = np.linalg.norm(node.position - old_position)
                
                # Debug position updates
                if node_id <= 2 and self.debug_position_counter % 20 == 0:
                    force_mag = np.linalg.norm(control_force) if node_id != self.closest_robot_id else np.linalg.norm(control_input)
                    print(f"Robot {node_id}: pos_change={position_change:.6f}, vel_mag={np.linalg.norm(node.velocity):.6f}, force_mag={force_mag:.6f}")
                
                # Update environmental time
                if node_id == 1:  # Update time only once per frame
                    self.underwater_env.update_time(self.dt)
            else:
                # Original simple dynamics
                node.position += node.velocity
            
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
        use_controller = True  # Default to controlled version
        
        if use_goal_seeking:
            # Sub-menu for goal-seeking: controlled vs uncontrolled
            print(f"\nGoal-Seeking Simulation Options:")
            print("1. With CBF Control (Standard goal-seeking with connectivity preservation)")
            print("2. Pure Brownian Motion (NO control - demonstrates need for CBF)")
            
            while True:
                try:
                    control_choice = int(input("Enter your choice [1-2]: "))
                    if 1 <= control_choice <= 2:
                        break
                    else:
                        print("Please enter 1 or 2.")
                except ValueError:
                    print("Please enter a valid integer.")
            
            use_controller = (control_choice == 1)
            
            # NEW: Choose connectivity preservation algorithm if using controller
            if use_controller:
                print(f"\nConnectivity Preservation Algorithm:")
                print("1. Triangular Pruning (Original geometric method with k-hop discovery)")
                print("2. Distance-Based Pruning (Remove longest + add shortest edges with recoupling)")
                print("3. MLCCST Controller (Centralized Minimum Length Connected Spanning Tree)")
                
                while True:
                    try:
                        pruning_alg_choice = int(input("Enter your choice [1-3]: "))
                        if 1 <= pruning_alg_choice <= 3:
                            break
                        else:
                            print("Please enter 1, 2, or 3.")
                    except ValueError:
                        print("Please enter a valid integer.")
                
                if pruning_alg_choice == 1:
                    edge_pruning_algorithm = "triangular"
                elif pruning_alg_choice == 2:
                    edge_pruning_algorithm = "distance"
                else:  # pruning_alg_choice == 3
                    edge_pruning_algorithm = "mlccst"
                
                pruning_method = f"Goal-Seeking with CBF Control ({edge_pruning_algorithm.upper()} algorithm)"
            else:
                edge_pruning_algorithm = "triangular"  # Default when no control
                pruning_method = "Pure Brownian Motion (No CBF Control)"
                
        elif use_simultaneous_pruning:
            edge_pruning_algorithm = "triangular"  # Default for other methods
            pruning_method = "Simultaneous Batch Pruning"
        else:
            edge_pruning_algorithm = "triangular"  # Default for other methods
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
        edge_pruning_algorithm = "triangular"  # Default
        pruning_method = "Traditional 1-by-1 Pruning"
    
    return num_nodes, graph_type, description, edge_prob, graph_option, use_simultaneous_pruning, use_goal_seeking, pruning_method, use_controller, edge_pruning_algorithm

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
        print("7. Centralized MLCCST CBF simulation in underwater environment")
        print("8. Exit")
        print("9. Test edge optimization (DEBUG)")
        
        try:
            choice = input("\nEnter your choice (1-9): ").strip()
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        
        if choice == '8':
            print("Thank you for using the Critical Edge Detection tool!")
            break
        
        if choice == '7':
            # Run centralized MLCCST CBF simulation
            print("\n🌊 Initializing Underwater Environment...")
            underwater_env = UnderwaterEnvironment(workspace_size=(10, 10))
            run_centralized_mlccst_simulation(underwater_env)
            continue
            
        if choice == '9':
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
                num_nodes, preferred_type, description, edge_prob, graph_option, use_simultaneous_pruning, use_goal_seeking, pruning_method, use_controller, edge_pruning_algorithm = get_user_input()
                
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
                    # Goal-seeking behavior - use clean goal generation
                    goal_x = np.random.uniform(0, 10)
                    goal_y = np.random.uniform(0, 10)
                    print(f"\n🎯 Starting goal-seeking simulation...")
                    print(f"Initial goal location: ({goal_x:.2f}, {goal_y:.2f})")
                    print("Goals will switch every 50 frames. Robots move toward goal while preserving connectivity!")
                    network.run_n_step_algorithm(n_steps=min(num_nodes + 2, 15))
                    network.goal_seeking_simulation(goal_x, goal_y, use_controller, edge_pruning_algorithm)
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
                # Temporarily auto-select 'n' for testing safety distance
                continue_simulation = 'n'  # input("\nWould you like to run another simulation? (y/n): ").lower().strip()
                if continue_simulation != 'y':
                    continue
            
            # All other options (1-5) - existing functionality
            else:
                # Get user input (for non-goal-seeking options, we only need first 5 values)
                num_nodes, preferred_type, description, edge_prob, _, _, _, _, _, _ = get_user_input()
                
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
    # Demo function to easily test both pruning methods
    def test_pruning_methods():
        """Test both triangular and distance-based pruning methods with MST comparison"""
        print("🔬 PRUNING METHOD COMPARISON DEMO")
        print("=" * 60)
        
        # Test with distance-based pruning
        print("\n1️⃣  TESTING DISTANCE-BASED PRUNING")
        print("-" * 40)
        network1 = DistributedNetwork(create_default_graph=True, enable_underwater_physics=False)
        network1.goal_seeking_simulation(8.0, 8.0, use_controller=True, pruning_method="distance")
        
        # Test with triangular pruning  
        print("\n2️⃣  TESTING TRIANGULAR PRUNING (ORIGINAL)")
        print("-" * 40)
        network2 = DistributedNetwork(create_default_graph=True, enable_underwater_physics=False)
        network2.goal_seeking_simulation(8.0, 8.0, use_controller=True, pruning_method="triangular")
        
        print("\n🎯 COMPARISON COMPLETE!")
        print("Check the MST Error Analysis plots to compare the performance of both methods.")
    
    # Uncomment the line below to run the demo
    # test_pruning_methods()
    
    # Original main function
    main()
