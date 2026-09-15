import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import networkx as nx

# Parameters - Enhanced for goal area formation
NUM_ROBOTS = 40  
TIME_STEP = 0.1
MAX_SPEED = 0.3
BROWNIAN_INTENSITY = 0.03  
COMM_RADIUS = 0.8
MIN_SAFETY_DISTANCE = 0.15
MAX_ITERATIONS = 550
BROWNIAN_PHASE_DURATION = 75
RETRACTION_START_ITERATION = 400
FORMATION_RADIUS = 0.8
GOAL_AREA_RADIUS = 0.4  # Radius for goal area formation
GOAL_DETECTION_RADIUS = 0.5  # Distance to consider "at goal"
RETRACTION_RADIUS = 1.5
SAVE_GIF = False

# NEW: Obstacle parameters
NUM_OBSTACLES = 10  # Number of circular obstacles
MIN_OBSTACLE_RADIUS = 0.2
MAX_OBSTACLE_RADIUS = 0.4
OBSTACLE_BUFFER = 0.5  # Minimum distance from origin/goals

# CLF-CBF Parameters
CLF_GAIN = 1.2
CBF_SAFETY_GAIN = 1.5  
CBF_CONNECTIVITY_GAIN = 3.0
CONNECTIVITY_THRESHOLD = 0.85

# Enhanced expansion parameters
MIN_ORIGIN_NEIGHBORS = 2
MAX_ORIGIN_NEIGHBORS = 3
EXPANSION_RATE = 0.5
OPTIMAL_CHAIN_SPACING = COMM_RADIUS * 0.75
MAX_ROBOTS_PER_CHAIN = 12
MIN_ROBOTS_TO_START_EXPANSION = 6
CONNECTIVITY_CHECK_INTERVAL = 10
EXPANSION_AGGRESSIVENESS = 0.8

# CBF-CBF Parameters
CLF_GAIN = 1.2
CBF_SAFETY_GAIN = 1.5  
CBF_CONNECTIVITY_GAIN = 3.0
CBF_OBSTACLE_GAIN = 2.0  # NEW: Obstacle avoidance gain
CONNECTIVITY_THRESHOLD = 0.85
OBSTACLE_SAFETY_DISTANCE = 0.2  # NEW: Minimum distance from obstacles

# **NEW: Goal balancing parameters**
GOAL_BALANCE_THRESHOLD = 3  # Max difference between goal assignments
FORCED_BALANCE_INTERVAL = 20  # Force rebalancing every N iterations

# NEW: Obstacle class
class Obstacle:
    def __init__(self, center, width, height, angle=0):
        self.center = np.array(center, dtype=float)
        self.width = width
        self.height = height
        self.angle = angle  # rotation angle in radians
    
    def distance_to_point(self, point):
        """Distance from point to obstacle surface (negative if inside)"""
        # Transform point to obstacle's local coordinate system
        cos_a = np.cos(-self.angle)
        sin_a = np.sin(-self.angle)
        
        # Translate to obstacle center
        dx = point[0] - self.center[0]
        dy = point[1] - self.center[1]
        
        # Rotate to align with obstacle axes
        local_x = dx * cos_a - dy * sin_a
        local_y = dx * sin_a + dy * cos_a
        
        # Distance to rectangle
        dx_rect = max(0, abs(local_x) - self.width/2)
        dy_rect = max(0, abs(local_y) - self.height/2)
        
        if abs(local_x) <= self.width/2 and abs(local_y) <= self.height/2:
            # Point is inside rectangle
            return -min(self.width/2 - abs(local_x), self.height/2 - abs(local_y))
        else:
            # Point is outside rectangle
            return np.sqrt(dx_rect**2 + dy_rect**2)
    
    def is_point_inside(self, point):
        """Check if point is inside obstacle"""
        return self.distance_to_point(point) < 0

class Robot:
    def __init__(self, id, initial_position):
        self.id = id
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

        # **NEW: Goal preference tracking**
        self.goal_preference_score = 0.0  # Positive = Goal 1, Negative = Goal 2
        self.assignment_locked = False  # Prevent reassignment once in chain
    
    def apply_brownian_motion(self, intensity):
        random_force = np.random.randn(2) * intensity
        self.velocity += random_force
    
    def apply_control_force(self, force):
        self.velocity += force
        speed = np.linalg.norm(self.velocity)
        if speed > MAX_SPEED:
            self.velocity = self.velocity * (MAX_SPEED / speed)
    
    def update_position(self):
        self.position += self.velocity * TIME_STEP
        self.velocity *= 0.8
    
    def distance_to(self, other_robot):
        return np.linalg.norm(self.position - other_robot.position)
    
    def distance_to_point(self, point):
        return np.linalg.norm(self.position - point)

class CLFCBFController:
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
        
        safety_distance = MIN_SAFETY_DISTANCE * 2
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
            max_distance = COMM_RADIUS * 0.95  # Very flexible for goal area robots
        elif robot.is_expansion_candidate or robot.expansion_weight > 1.5:
            max_distance = COMM_RADIUS * 0.9
        else:
            max_distance = COMM_RADIUS * CONNECTIVITY_THRESHOLD
            
        barrier_value = max_distance**2 - distance**2
        
        if distance > 1e-6:
            barrier_gradient = -2 * distance_vector
        else:
            barrier_gradient = np.zeros(2)
        
        return barrier_value, barrier_gradient
    
    def control_barrier_function_obstacle(self, robot, obstacle):
        """NEW: CBF for obstacle avoidance"""
        # Distance from robot to obstacle surface
        distance_to_surface = obstacle.distance_to_point(robot.position)
        
        # Barrier function: h(x) = distance_to_surface - safety_distance
        barrier_value = distance_to_surface - OBSTACLE_SAFETY_DISTANCE
        
        # Compute gradient numerically for robustness
        epsilon = 1e-4
        grad_x = (obstacle.distance_to_point(robot.position + np.array([epsilon, 0])) - 
                 obstacle.distance_to_point(robot.position - np.array([epsilon, 0]))) / (2 * epsilon)
        grad_y = (obstacle.distance_to_point(robot.position + np.array([0, epsilon])) - 
                 obstacle.distance_to_point(robot.position - np.array([0, epsilon]))) / (2 * epsilon)
        
        barrier_gradient = np.array([grad_x, grad_y])
        
        return barrier_value, barrier_gradient
    
    def compute_clf_cbf_control(self, robot, target_position, other_robots, parent_robot):
        """Enhanced CLF-CBF control with obstacle avoidance"""
        # CLF computation
        clf_value, clf_grad = self.control_lyapunov_function(robot, target_position)
        
        # Dynamic CLF gain based on goal proximity and role
        if not robot.is_connected_to_origin:
            clf_gain = CLF_GAIN * 0.1
        elif robot.is_at_goal:
            clf_gain = CLF_GAIN * 0.5  # Moderate for goal area positioning
        elif robot.connectivity_priority > 7:
            clf_gain = CLF_GAIN * 0.3
        elif robot.is_chain_leader:
            if robot.assigned_goal == 1:
                distance_to_goal = robot.distance_to_goal1
            elif robot.assigned_goal == 2:
                distance_to_goal = robot.distance_to_goal2
            else:
                distance_to_goal = float('inf')
            
            # Reduce CLF gain as leader approaches goal to prevent overshoot
            if distance_to_goal < GOAL_DETECTION_RADIUS:
                clf_gain = CLF_GAIN * 0.8 * robot.expansion_weight  # Reduced near goal
            elif distance_to_goal < OPTIMAL_CHAIN_SPACING:
                clf_gain = CLF_GAIN * 1.5 * robot.expansion_weight
            else:
                clf_gain = CLF_GAIN * 1.8 * robot.expansion_weight
        elif robot.is_chain_active:
            clf_gain = CLF_GAIN * 1.0 * robot.expansion_weight
        elif robot.is_expansion_candidate:
            clf_gain = CLF_GAIN * 0.8 * robot.expansion_weight
        else:
            clf_gain = CLF_GAIN * 0.4
            
        control = -clf_gain * clf_grad
        
        # OBSTACLE AVOIDANCE CBF - NEW!
        for obstacle in self.obstacles:
            cbf_val, cbf_grad = self.control_barrier_function_obstacle(robot, obstacle)
            
            # Intervention thresholds based on distance to obstacle
            if cbf_val < 0.3:  # Close to obstacle
                intervention_threshold = 0.15
                critical_threshold = 0.05
                obstacle_gain = CBF_OBSTACLE_GAIN
                
                if cbf_val < intervention_threshold:
                    grad_norm = np.linalg.norm(cbf_grad)
                    if grad_norm > 1e-6:
                        # Repulsive force away from obstacle
                        obstacle_correction = obstacle_gain * (-cbf_val) * cbf_grad / grad_norm
                        control += obstacle_correction
                        
                        # Emergency override if very close to obstacle
                        if cbf_val < critical_threshold:
                            if robot.is_at_goal:
                                # Even goal robots must avoid obstacles
                                control = obstacle_correction * 1.2
                            elif robot.is_chain_leader:
                                # Leaders get strong obstacle avoidance
                                control = obstacle_correction * 1.5
                            else:
                                # Strong avoidance for all robots
                                control = obstacle_correction * 2.0
        
        # CONNECTIVITY CBF
        if parent_robot is not None:
            cbf_val, cbf_grad = self.control_barrier_function_connectivity(robot, parent_robot)
            
            # Dynamic intervention based on role and goal status
            if robot.is_at_goal:
                intervention_threshold = 0.1
                critical_threshold = 0.03
                connectivity_gain = CBF_CONNECTIVITY_GAIN * 0.6
            elif robot.is_chain_leader:
                if robot.assigned_goal == 1:
                    goal_distance = robot.distance_to_goal1
                elif robot.assigned_goal == 2:
                    goal_distance = robot.distance_to_goal2
                else:
                    goal_distance = float('inf')
                
                if goal_distance < GOAL_DETECTION_RADIUS:
                    intervention_threshold = 0.08
                    critical_threshold = 0.02
                    connectivity_gain = CBF_CONNECTIVITY_GAIN * 0.5
                else:
                    intervention_threshold = 0.1
                    critical_threshold = 0.03
                    connectivity_gain = CBF_CONNECTIVITY_GAIN * 0.7
            elif robot.is_expansion_candidate or robot.expansion_weight > 1.5:
                intervention_threshold = 0.15
                critical_threshold = 0.05
                connectivity_gain = CBF_CONNECTIVITY_GAIN * 0.8
            else:
                intervention_threshold = 0.2
                critical_threshold = 0.08
                connectivity_gain = CBF_CONNECTIVITY_GAIN
            
            if cbf_val < intervention_threshold:
                grad_norm = np.linalg.norm(cbf_grad)
                if grad_norm > 1e-6:
                    connectivity_correction = connectivity_gain * (-cbf_val) * cbf_grad / grad_norm
                    control += connectivity_correction
                    
                    if cbf_val < critical_threshold:
                        if robot.is_at_goal:
                            control = connectivity_correction * 0.6  # Minimal override for goal robots
                        elif robot.is_chain_leader and goal_distance < GOAL_DETECTION_RADIUS:
                            control = connectivity_correction * 0.8
                        else:
                            control = connectivity_correction * 1.5
        
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
        if control_magnitude > MAX_SPEED:
            control = control * (MAX_SPEED / control_magnitude)
        
        return control

class MultiGoalMLCCSTController:
    def __init__(self, robots, goal1_position, goal2_position, origin_position, obstacles=None):
        self.robots = robots
        self.goal1_position = goal1_position
        self.goal2_position = goal2_position
        self.origin_position = origin_position
        self.obstacles = obstacles or []  # NEW: Store obstacles
        self.connectivity_graph = nx.Graph()
        self.spanning_tree = nx.Graph()
        self.is_active = False
        self.retraction_mode = False
        self.origin_guardians = []
        
        # Enhanced chain management
        self.goal1_chain = []
        self.goal2_chain = []
        self.goal1_area_robots = []  # New list for robots at goal 1
        self.goal2_area_robots = []  # New list for robots at goal 2
        self.chains_initialized = False
        self.expansion_active = False
        self.last_connectivity_check = 0
        self.connectivity_violations = 0
        self.expansion_candidates = []
        self.last_expansion_attempt = 0
        
        # CLF-CBF Controller with obstacles
        self.clf_cbf_controller = CLFCBFController(obstacles=self.obstacles)
        # **NEW: Goal balancing tracking**
        self.last_balance_check = 0
        self.goal_assignment_history = []
        self.forced_balance_counter = 0
    
    def activate(self):
        self.is_active = True
        print("Enhanced Chain Expansion Controller activated!")
    
    def activate_retraction_mode(self):
        self.retraction_mode = True
        print("RETRACTION MODE ACTIVATED!")
    
    def check_goal_arrivals(self):
        """Check if robots have arrived at goals and manage goal area formation"""
        current_time = self.last_expansion_attempt
        
        for robot in self.robots:
            # Check if robot has arrived at goal 1
            if (robot.assigned_goal == 1 and 
                robot.distance_to_goal1 <= GOAL_DETECTION_RADIUS and 
                not robot.is_at_goal):
                
                robot.is_at_goal = True
                robot.goal_arrival_time = current_time
                robot.goal_area_position = len(self.goal1_area_robots)
                robot.role = "goal1_area"
                self.goal1_area_robots.append(robot)
                
                # Remove from chain if it was the leader
                if robot.is_chain_leader and robot in self.goal1_chain:
                    robot.is_chain_leader = False
                    self.goal1_chain.remove(robot)
                    
                    # Promote next robot in chain to leader
                    if self.goal1_chain:
                        new_leader = self.goal1_chain[-1]
                        new_leader.is_chain_leader = True
                        new_leader.role = "goal1_leader"
                        new_leader.expansion_weight = 2.5
                
                print(f"Robot {robot.id} arrived at Goal 1! Goal area now has {len(self.goal1_area_robots)} robots")
            
            # Check if robot has arrived at goal 2
            elif (robot.assigned_goal == 2 and 
                  robot.distance_to_goal2 <= GOAL_DETECTION_RADIUS and 
                  not robot.is_at_goal):
                
                robot.is_at_goal = True
                robot.goal_arrival_time = current_time
                robot.goal_area_position = len(self.goal2_area_robots)
                robot.role = "goal2_area"
                self.goal2_area_robots.append(robot)
                
                # Remove from chain if it was the leader
                if robot.is_chain_leader and robot in self.goal2_chain:
                    robot.is_chain_leader = False
                    self.goal2_chain.remove(robot)
                    
                    # Promote next robot in chain to leader
                    if self.goal2_chain:
                        new_leader = self.goal2_chain[-1]
                        new_leader.is_chain_leader = True
                        new_leader.role = "goal2_leader"
                        new_leader.expansion_weight = 2.5
                
                print(f"Robot {robot.id} arrived at Goal 2! Goal area now has {len(self.goal2_area_robots)} robots")
    
    def compute_goal_area_position(self, robot, goal_position, goal_area_robots):
        """Compute target position for robot in goal area formation"""
        num_robots_in_area = len(goal_area_robots)
        
        if num_robots_in_area == 0:
            return goal_position.copy()
        
        # Create circular formation around goal
        if robot.goal_area_position == 0:
            # First robot stays close to goal
            return goal_position.copy()
        else:
            # Other robots form concentric circles
            robots_per_circle = 6  # Number of robots per circle
            circle_number = (robot.goal_area_position - 1) // robots_per_circle
            position_in_circle = (robot.goal_area_position - 1) % robots_per_circle
            
            # Calculate angle for this position
            angle = (position_in_circle * 2 * np.pi) / robots_per_circle
            
            # Calculate radius (inner circle closer to goal)
            radius = GOAL_AREA_RADIUS * (0.5 + circle_number * 0.4)
            
            # Compute position
            target_x = goal_position[0] + radius * np.cos(angle)
            target_y = goal_position[1] + radius * np.sin(angle)
            
            return np.array([target_x, target_y])
    
    def check_connectivity_health(self):
        """Comprehensive connectivity health check"""
        total_robots = len(self.robots)
        connected_robots = sum(1 for r in self.robots if r.is_connected_to_origin)
        connectivity_ratio = connected_robots / total_robots if total_robots > 0 else 0
        
        isolated_robots = [r for r in self.robots if not r.is_connected_to_origin]
        
        if connectivity_ratio < 0.8:
            self.connectivity_violations += 1
            if len(isolated_robots) > 0:
                print(f"CONNECTIVITY WARNING: Only {connected_robots}/{total_robots} robots connected ({connectivity_ratio:.1%})")
                print(f"Isolated robots: {[r.id for r in isolated_robots[:5]]}")
            return False
        else:
            self.connectivity_violations = max(0, self.connectivity_violations - 1)
            return True
    
    def emergency_connectivity_recovery(self):
        """Emergency procedure to recover connectivity"""
        if self.connectivity_violations > 2:
            print("EMERGENCY CONNECTIVITY RECOVERY ACTIVATED!")
            
            global EXPANSION_AGGRESSIVENESS
            EXPANSION_AGGRESSIVENESS *= 0.5
            
            disconnected_robots = [r for r in self.robots if not r.is_connected_to_origin]
            connected_robots = [r for r in self.robots if r.is_connected_to_origin]
            
            for disc_robot in disconnected_robots:
                if connected_robots:
                    nearest_connected = min(connected_robots, 
                                          key=lambda r: disc_robot.distance_to(r))
                    
                    direction = nearest_connected.position - disc_robot.position
                    distance = np.linalg.norm(direction)
                    if distance > 0.1:
                        target_distance = COMM_RADIUS * 0.7
                        disc_robot.target_position = nearest_connected.position - (direction / distance) * target_distance
                        disc_robot.expansion_weight = 0.1
                        disc_robot.connectivity_priority = 10
                        disc_robot.is_expansion_candidate = False
    
    def balanced_goal_assignment(self):
        """Force balanced assignment between goals"""
        connected_robots = [r for r in self.robots 
                        if r.is_connected_to_origin 
                        and not r.should_stay_near_origin 
                        and not r.is_at_goal]
        
        if len(connected_robots) < 4:
            return
        
        # Clear current assignments for non-locked robots
        unassigned_robots = [r for r in connected_robots if not r.assignment_locked]
        
        if len(unassigned_robots) < 2:
            return
        
        # **FORCED BALANCED SPLIT**
        num_per_goal = len(unassigned_robots) // 2
        
        # Sort by position relative to goals for natural assignment
        unassigned_robots.sort(key=lambda r: r.distance_to_goal1 - r.distance_to_goal2)
        
        # **STRICT ALTERNATING ASSIGNMENT**
        for i, robot in enumerate(unassigned_robots):
            if i < num_per_goal:
                robot.assigned_goal = 1
                robot.goal_preference_score = 1.0
            else:
                robot.assigned_goal = 2
                robot.goal_preference_score = -1.0
            
            print(f"BALANCED ASSIGNMENT: Robot {robot.id} assigned to Goal {robot.assigned_goal}")
        
        print(f"BALANCED ASSIGNMENT COMPLETE: Goal1={len([r for r in unassigned_robots if r.assigned_goal == 1])}, Goal2={len([r for r in unassigned_robots if r.assigned_goal == 2])}")

    def identify_expansion_candidates(self):
        """Identify robots that could join expansion efforts with balanced goal preference"""
        self.expansion_candidates = []
        
        connected_robots = [r for r in self.robots if r.is_connected_to_origin]
        
        for robot in connected_robots:
            if (robot.should_stay_near_origin or 
                robot.is_chain_active or 
                robot.assigned_goal is not None or
                robot.is_at_goal):  # Don't recruit robots at goals
                continue
            
            robot.is_expansion_candidate = False
            
            min_distance_to_goal1_chain = float('inf')
            min_distance_to_goal2_chain = float('inf')
            
            if self.goal1_chain:
                min_distance_to_goal1_chain = min(robot.distance_to(chain_robot) 
                                                for chain_robot in self.goal1_chain)
            
            if self.goal2_chain:
                min_distance_to_goal2_chain = min(robot.distance_to(chain_robot) 
                                                for chain_robot in self.goal2_chain)
            
            if (robot.distance_to_origin <= COMM_RADIUS * 2 or
                min_distance_to_goal1_chain <= COMM_RADIUS * 1.5 or
                min_distance_to_goal2_chain <= COMM_RADIUS * 1.5):
                
                robot.is_expansion_candidate = True
                robot.expansion_weight = 1.2
                
                # **FIXED: Balanced goal preference**
                goal1_chain_length = len(self.goal1_chain)
                goal2_chain_length = len(self.goal2_chain)
                
                # Prefer the goal with shorter chain
                if goal1_chain_length < goal2_chain_length:
                    robot.preferred_goal = 1
                elif goal2_chain_length < goal1_chain_length:
                    robot.preferred_goal = 2
                else:
                    # If equal, use distance
                    if robot.distance_to_goal1 < robot.distance_to_goal2:
                        robot.preferred_goal = 1
                    else:
                        robot.preferred_goal = 2
                
                self.expansion_candidates.append(robot)
    
    def initialize_chains_balanced(self):
        if self.chains_initialized:
            return
            
        connected_robots = [r for r in self.robots if r.is_connected_to_origin]
        
        if len(connected_robots) < MIN_ROBOTS_TO_START_EXPANSION:
            return
        
        if not self.check_connectivity_health():
            return
        
        print(f"BALANCED chain initialization with {len(connected_robots)} connected robots")
        
        robots_by_origin_distance = sorted(connected_robots, key=lambda r: r.distance_to_origin)
        num_guardians = MIN_ORIGIN_NEIGHBORS
        
        self.origin_guardians = robots_by_origin_distance[:num_guardians]
        for robot in self.origin_guardians:
            robot.should_stay_near_origin = True
            robot.role = "origin_guardian"
            robot.expansion_weight = 0.1
            robot.connectivity_priority = 8
        
        available_robots = robots_by_origin_distance[num_guardians:]
        
        if len(available_robots) >= 4:
            # **EXACTLY EQUAL INITIAL ASSIGNMENT**
            robots_per_goal = len(available_robots) // 2
            
            # **FORCE ALTERNATING ASSIGNMENT BY POSITION**
            available_robots.sort(key=lambda r: np.arctan2(r.position[1] - self.origin_position[1], 
                                                        r.position[0] - self.origin_position[0]))
            
            # Goal1 chain initialization
            for i in range(robots_per_goal):
                robot = available_robots[i * 2]  # Take every other robot
                robot.assigned_goal = 1
                robot.is_chain_active = True
                robot.chain_position = i
                robot.expansion_weight = 1.8
                robot.connectivity_priority = 5
                robot.assignment_locked = True  # Lock assignment
                self.goal1_chain.append(robot)
                
                if i == robots_per_goal - 1:
                    robot.is_chain_leader = True
                    robot.role = "goal1_leader"
                    robot.expansion_weight = 2.5
                else:
                    robot.role = "goal1_chain"
            
            # Goal2 chain initialization
            for i in range(robots_per_goal):
                if i * 2 + 1 < len(available_robots):
                    robot = available_robots[i * 2 + 1]  # Take the alternating robots
                    robot.assigned_goal = 2
                    robot.is_chain_active = True
                    robot.chain_position = i
                    robot.expansion_weight = 1.8
                    robot.connectivity_priority = 5
                    robot.assignment_locked = True  # Lock assignment
                    self.goal2_chain.append(robot)
                    
                    if i == robots_per_goal - 1:
                        robot.is_chain_leader = True
                        robot.role = "goal2_leader"
                        robot.expansion_weight = 2.5
                    else:
                        robot.role = "goal2_chain"
            
            self.chains_initialized = True
            self.expansion_active = True
            print(f"BALANCED initialization - Goal1 chain: {len(self.goal1_chain)}, Goal2 chain: {len(self.goal2_chain)}")

    def update_distances(self):
        """Update distance metrics"""
        for robot in self.robots:
            robot.distance_to_goal1 = robot.distance_to_point(self.goal1_position)
            robot.distance_to_goal2 = robot.distance_to_point(self.goal2_position)
            robot.distance_to_origin = robot.distance_to_point(self.origin_position)
    
    def update_connectivity_graph(self):
        """Build connectivity graph"""
        self.connectivity_graph.clear()
        
        for robot in self.robots:
            self.connectivity_graph.add_node(robot.id, pos=robot.position.copy())
        
        self.connectivity_graph.add_node("origin", pos=self.origin_position)
        
        for i, robot1 in enumerate(self.robots):
            if robot1.distance_to_point(self.origin_position) <= COMM_RADIUS:
                self.connectivity_graph.add_edge(
                    robot1.id, "origin", 
                    weight=robot1.distance_to_point(self.origin_position)
                )
            
            for j, robot2 in enumerate(self.robots[i+1:], i+1):
                distance = robot1.distance_to(robot2)
                if distance <= COMM_RADIUS:
                    self.connectivity_graph.add_edge(robot1.id, robot2.id, weight=distance)
    
    def build_spanning_tree(self):
        """Build minimum spanning tree ensuring origin connectivity"""
        try:
            if len(self.connectivity_graph.nodes()) > 1:
                self.spanning_tree = nx.minimum_spanning_tree(self.connectivity_graph, weight='weight')
                self.update_connectivity_status()
                self.calculate_chain_distances()
            else:
                self.spanning_tree.clear()
                
        except Exception as e:
            print(f"Error building spanning tree: {e}")
            self.spanning_tree = nx.Graph()
    
    def calculate_chain_distances(self):
        """Calculate chain distance from origin for each robot"""
        for robot in self.robots:
            robot.chain_distance = float('inf')
        
        if "origin" not in self.spanning_tree:
            return
        
        visited = {"origin"}
        queue = [("origin", 0)]
        
        while queue:
            current_id, distance = queue.pop(0)
            
            for neighbor in self.spanning_tree.neighbors(current_id):
                if neighbor not in visited:
                    visited.add(neighbor)
                    
                    if neighbor != "origin":
                        robot = next((r for r in self.robots if r.id == neighbor), None)
                        if robot:
                            robot.chain_distance = distance + 1
                    
                    queue.append((neighbor, distance + 1))
    
    def update_connectivity_status(self):
        """Update parent-child relationships from spanning tree"""
        for robot in self.robots:
            robot.is_connected_to_origin = False
            robot.parent = None
            robot.children = []
        
        if "origin" not in self.spanning_tree:
            return
        
        visited = {"origin"}
        queue = [("origin", None)]
        
        while queue:
            current_id, parent_id = queue.pop(0)
            
            if current_id != "origin":
                current_robot = next((r for r in self.robots if r.id == current_id), None)
                if current_robot:
                    current_robot.is_connected_to_origin = True
                    current_robot.last_connected_time = 0
                    
                    if parent_id and parent_id != "origin":
                        parent_robot = next((r for r in self.robots if r.id == parent_id), None)
                        if parent_robot:
                            current_robot.parent = parent_robot
                            parent_robot.children.append(current_robot)
            
            for neighbor in self.spanning_tree.neighbors(current_id):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, current_id))
    
    def enhanced_chain_expansion(self):
        """Enhanced chain expansion with goal area management"""
        if not self.expansion_active:
            return
        
        if not self.check_connectivity_health():
            self.emergency_connectivity_recovery()
            return
        
        # Check for goal arrivals
        self.check_goal_arrivals()
        
        self.identify_expansion_candidates()
        
        if self.last_expansion_attempt % 5 == 0:
            
            for goal_num in [1, 2]:
                if goal_num == 1:
                    current_chain = self.goal1_chain
                    goal_pos = self.goal1_position
                else:
                    current_chain = self.goal2_chain
                    goal_pos = self.goal2_position
                
                if len(current_chain) < MAX_ROBOTS_PER_CHAIN:
                    current_leader = next((r for r in current_chain if r.is_chain_leader), None)
                    
                    if (current_leader and 
                        current_leader.is_connected_to_origin and
                        current_leader.distance_to_point(goal_pos) > OPTIMAL_CHAIN_SPACING * 0.5):
                        
                        preferred_candidates = [r for r in self.expansion_candidates 
                                             if r.preferred_goal == goal_num and r.is_connected_to_origin]
                        
                        close_candidates = [r for r in self.expansion_candidates 
                                          if any(r.distance_to(chain_robot) <= COMM_RADIUS * 0.9 
                                                for chain_robot in current_chain)]
                        
                        all_candidates = list(set(preferred_candidates + close_candidates))
                        
                        if all_candidates:
                            best_candidate = min(all_candidates, 
                                               key=lambda r: r.distance_to_point(goal_pos))
                            
                            best_candidate.assigned_goal = goal_num
                            best_candidate.is_chain_active = True
                            best_candidate.chain_position = len(current_chain)
                            best_candidate.expansion_weight = 2.0
                            best_candidate.connectivity_priority = 5
                            best_candidate.is_expansion_candidate = False
                            current_chain.append(best_candidate)
                            
                            current_leader.is_chain_leader = False
                            if goal_num == 1:
                                current_leader.role = "goal1_chain"
                                best_candidate.role = "goal1_leader"
                            else:
                                current_leader.role = "goal2_chain"
                                best_candidate.role = "goal2_leader"
                            
                            best_candidate.is_chain_leader = True
                            best_candidate.expansion_weight = 2.8
                            
                            print(f"Enhanced expansion - Goal{goal_num} chain now has {len(current_chain)} robots")
        
        self.last_expansion_attempt += 1
    
    def assign_roles(self):
        """Enhanced role assignment with goal area management"""
        if not self.is_active:
            return
        
        if self.retraction_mode:
            for robot in self.robots:
                robot.role = "retracting"
                robot.expansion_weight = 0.1
            return
        
        self.initialize_chains_balanced()
        self.enhanced_chain_expansion()
        
        for robot in self.robots:
            if robot.role in ["origin_guardian", "goal1_leader", "goal2_leader", 
                            "goal1_chain", "goal2_chain", "goal1_area", "goal2_area"]:
                continue
            elif robot.is_expansion_candidate:
                robot.role = "expansion_candidate"
                robot.expansion_weight = 1.2
            else:
                robot.role = "follower"
                robot.expansion_weight = 0.5
    
    def set_robot_targets_enhanced(self):
        """Enhanced target setting with goal area formation and overshoot prevention"""
        for robot in self.robots:
            if self.retraction_mode:
                robot.target_position = self.origin_position.copy()
                
            elif robot.should_stay_near_origin:
                direction = robot.position - self.origin_position
                distance = np.linalg.norm(direction)
                if distance > 0.1:
                    target_distance = COMM_RADIUS * 0.3
                    robot.target_position = self.origin_position + (direction / distance) * target_distance
                else:
                    guardian_index = self.origin_guardians.index(robot) if robot in self.origin_guardians else 0
                    angle = (guardian_index * 2 * np.pi) / len(self.origin_guardians) if self.origin_guardians else 0
                    robot.target_position = self.origin_position + np.array([
                        COMM_RADIUS * 0.3 * np.cos(angle),
                        COMM_RADIUS * 0.3 * np.sin(angle)
                    ])
            
            elif robot.is_at_goal:
                # Robots at goals form circular formations
                if robot.assigned_goal == 1:
                    robot.target_position = self.compute_goal_area_position(
                        robot, self.goal1_position, self.goal1_area_robots)
                elif robot.assigned_goal == 2:
                    robot.target_position = self.compute_goal_area_position(
                        robot, self.goal2_position, self.goal2_area_robots)
                else:
                    robot.target_position = robot.position.copy()
                    
            elif robot.is_chain_leader and robot.assigned_goal == 1 and robot.is_connected_to_origin:
                # Leaders approach goals but slow down near goal to prevent overshoot
                if robot.distance_to_goal1 > GOAL_DETECTION_RADIUS * 1.5:
                    robot.target_position = self.goal1_position.copy()
                else:
                    # Slow approach to goal
                    direction = self.goal1_position - robot.position
                    distance = np.linalg.norm(direction)
                    if distance > 0.1:
                        # Move only partway to goal to prevent overshoot
                        robot.target_position = robot.position + direction * 0.5
                    else:
                        robot.target_position = self.goal1_position.copy()
                
            elif robot.is_chain_leader and robot.assigned_goal == 2 and robot.is_connected_to_origin:
                # Leaders approach goals but slow down near goal to prevent overshoot
                if robot.distance_to_goal2 > GOAL_DETECTION_RADIUS * 1.5:
                    robot.target_position = self.goal2_position.copy()
                else:
                    # Slow approach to goal
                    direction = self.goal2_position - robot.position
                    distance = np.linalg.norm(direction)
                    if distance > 0.1:
                        # Move only partway to goal to prevent overshoot
                        robot.target_position = robot.position + direction * 0.5
                    else:
                        robot.target_position = self.goal2_position.copy()
                
            elif robot.is_chain_active and robot.is_connected_to_origin:
                # Chain robots maintain optimal spacing
                if robot.assigned_goal == 1:
                    current_leader = next((r for r in self.goal1_chain if r.is_chain_leader), None)
                    if current_leader and robot.chain_position < len(self.goal1_chain) - 1:
                        leader_direction = current_leader.position - self.origin_position
                        leader_distance = np.linalg.norm(leader_direction)
                        
                        if leader_distance > 0.1:
                            leader_direction = leader_direction / leader_distance
                            target_distance = robot.chain_position * OPTIMAL_CHAIN_SPACING
                            robot.target_position = self.origin_position + leader_direction * target_distance
                        else:
                            robot.target_position = robot.position.copy()
                    else:
                        # If this robot somehow became the leader, approach goal carefully
                        if robot.distance_to_goal1 > GOAL_DETECTION_RADIUS * 1.5:
                            robot.target_position = self.goal1_position.copy()
                        else:
                            direction = self.goal1_position - robot.position
                            distance = np.linalg.norm(direction)
                            if distance > 0.1:
                                robot.target_position = robot.position + direction * 0.5
                            else:
                                robot.target_position = self.goal1_position.copy()
                        
                elif robot.assigned_goal == 2:
                    current_leader = next((r for r in self.goal2_chain if r.is_chain_leader), None)
                    if current_leader and robot.chain_position < len(self.goal2_chain) - 1:
                        leader_direction = current_leader.position - self.origin_position
                        leader_distance = np.linalg.norm(leader_direction)
                        
                        if leader_distance > 0.1:
                            leader_direction = leader_direction / leader_distance
                            target_distance = robot.chain_position * OPTIMAL_CHAIN_SPACING
                            robot.target_position = self.origin_position + leader_direction * target_distance
                        else:
                            robot.target_position = robot.position.copy()
                    else:
                        # If this robot somehow became the leader, approach goal carefully
                        if robot.distance_to_goal2 > GOAL_DETECTION_RADIUS * 1.5:
                            robot.target_position = self.goal2_position.copy()
                        else:
                            direction = self.goal2_position - robot.position
                            distance = np.linalg.norm(direction)
                            if distance > 0.1:
                                robot.target_position = robot.position + direction * 0.5
                            else:
                                robot.target_position = self.goal2_position.copy()
                else:
                    robot.target_position = robot.position.copy()
                    
            elif robot.is_expansion_candidate and robot.is_connected_to_origin:
                # Expansion candidates move toward their preferred goal direction
                if robot.preferred_goal == 1:
                    direction = self.goal1_position - robot.position
                    if np.linalg.norm(direction) > 0.1:
                        robot.target_position = robot.position + direction * 0.3
                    else:
                        robot.target_position = self.goal1_position.copy()
                elif robot.preferred_goal == 2:
                    direction = self.goal2_position - robot.position
                    if np.linalg.norm(direction) > 0.1:
                        robot.target_position = robot.position + direction * 0.3
                    else:
                        robot.target_position = self.goal2_position.copy()
                else:
                    robot.target_position = robot.position.copy()
                    
            else:
                # Disconnected or unassigned robots
                if not robot.is_connected_to_origin:
                    connected_robots = [r for r in self.robots if r.is_connected_to_origin]
                    if connected_robots:
                        nearest = min(connected_robots, key=lambda r: robot.distance_to(r))
                        direction = nearest.position - robot.position
                        distance = np.linalg.norm(direction)
                        if distance > 0.1:
                            robot.target_position = robot.position + direction * 0.5
                        else:
                            robot.target_position = robot.position.copy()
                    else:
                        robot.target_position = robot.position.copy()
                else:
                    offset = np.random.randn(2) * 0.1
                    robot.target_position = robot.position + offset
    
    def compute_control_forces(self):
        """Main control computation with goal area management"""
        if not self.is_active:
            return [np.zeros(2) for _ in self.robots]
        
        self.update_distances()
        self.update_connectivity_graph()
        self.build_spanning_tree()
        
        if self.last_connectivity_check % CONNECTIVITY_CHECK_INTERVAL == 0:
            self.check_connectivity_health()
        self.last_connectivity_check += 1
        
        self.assign_roles()
        self.set_robot_targets_enhanced()
        
        control_forces = []
        
        for robot in self.robots:
            control_force = self.clf_cbf_controller.compute_clf_cbf_control(
                robot, robot.target_position, self.robots, robot.parent
            )
            control_forces.append(control_force)
        
        return control_forces

# NEW: Obstacle generation function
def generate_random_obstacles(num_obstacles, cluster_center, goal1_position, goal2_position, arena_size=10.0):
    """Generate random rectangular bar obstacles avoiding important areas"""
    obstacles = []
    
    # Important areas to avoid
    protected_areas = [
        (cluster_center, 1.0),  # Origin area
        (goal1_position, 0.8),  # Goal 1 area
        (goal2_position, 0.8),  # Goal 2 area
    ]
    
    max_attempts = 100
    
    for i in range(num_obstacles):
        attempts = 0
        while attempts < max_attempts:
            # Random position
            x = np.random.uniform(1.5, arena_size - 1.5)
            y = np.random.uniform(1.5, arena_size - 1.5)
            center = np.array([x, y])
            
            # Random bar dimensions (longer than wide)
            width = np.random.uniform(0.8, 1.5)  # length of bar
            height = np.random.uniform(0.15, 0.3)  # thickness of bar
            angle = np.random.uniform(0, 2*np.pi)  # random orientation
            
            # Check if obstacle conflicts with protected areas
            valid = True
            for protected_center, protected_radius in protected_areas:
                distance = np.linalg.norm(center - np.array(protected_center))
                # Use diagonal of rectangle as approximation for collision check
                obstacle_radius = np.sqrt(width**2 + height**2) / 2
                if distance < (obstacle_radius + protected_radius + OBSTACLE_BUFFER):
                    valid = False
                    break
            
            # Check if obstacle conflicts with existing obstacles
            if valid:
                for existing_obstacle in obstacles:
                    distance = np.linalg.norm(center - existing_obstacle.center)
                    existing_radius = np.sqrt(existing_obstacle.width**2 + existing_obstacle.height**2) / 2
                    current_radius = np.sqrt(width**2 + height**2) / 2
                    if distance < (current_radius + existing_radius + 0.3):
                        valid = False
                        break
            
            if valid:
                obstacles.append(Obstacle(center, width, height, angle))
                print(f"Generated obstacle {i+1}: center=({x:.2f}, {y:.2f}), size=({width:.2f}x{height:.2f}), angle={angle:.2f}")
                break
            
            attempts += 1
        
        if attempts >= max_attempts:
            print(f"Could not place obstacle {i+1} after {max_attempts} attempts")
    
    return obstacles

# Utility functions remain the same
def create_clustered_formation(num_robots, cluster_center=[5.0, 5.0], cluster_radius=0.4):
    robots = []
    for i in range(num_robots):
        angle = np.random.uniform(0, 2 * np.pi)
        radius = np.random.uniform(0, cluster_radius)
        
        x = cluster_center[0] + radius * np.cos(angle)
        y = cluster_center[1] + radius * np.sin(angle)
        
        robots.append(Robot(i+1, [x, y]))
    return robots

def generate_two_random_goals(cluster_center=[5.0, 5.0], cluster_radius=0.4, min_distance=3.5, arena_size=10.0):
    goals = []
    
    for i in range(2):
        while True:
            goal_x = np.random.uniform(1.5, arena_size - 1.5)
            goal_y = np.random.uniform(1.5, arena_size - 1.5)
            goal_position = np.array([goal_x, goal_y])
            
            distance_from_cluster = np.linalg.norm(goal_position - np.array(cluster_center))
            
            valid = distance_from_cluster >= min_distance
            for existing_goal in goals:
                if np.linalg.norm(goal_position - existing_goal) < min_distance:
                    valid = False
                    break
            
            if valid:
                goals.append(goal_position)
                break
    
    return goals[0], goals[1]

def run_simulation():
    print("Initializing Enhanced Chain Expansion with Goal Area Formation and Obstacles...")
    
    cluster_center = [5.0, 5.0]
    robots = create_clustered_formation(NUM_ROBOTS, cluster_center)
    
    goal1_position, goal2_position = generate_two_random_goals(cluster_center)
    print(f"Goal 1 position: ({goal1_position[0]:.2f}, {goal1_position[1]:.2f})")
    print(f"Goal 2 position: ({goal2_position[0]:.2f}, {goal2_position[1]:.2f})")
    
    # NEW: Generate obstacles
    obstacles = generate_random_obstacles(NUM_OBSTACLES, cluster_center, goal1_position, goal2_position)
    print(f"Generated {len(obstacles)} obstacles")
    
    origin_position = np.array(cluster_center)
    # Pass obstacles to controller
    controller = MultiGoalMLCCSTController(robots, goal1_position, goal2_position, origin_position, obstacles)

    all_positions = []
    all_roles = []
    all_edges = []
    all_phases = []
    
    print("Running Enhanced Chain Expansion simulation with obstacles...")
    for iteration in range(MAX_ITERATIONS):
        if iteration % 100 == 0:
            print(f"Iteration {iteration}/{MAX_ITERATIONS}")
            connected_count = sum(1 for r in robots if r.is_connected_to_origin)
            connectivity_ratio = connected_count / len(robots)
            goal1_chain_length = len(controller.goal1_chain) if hasattr(controller, 'goal1_chain') else 0
            goal2_chain_length = len(controller.goal2_chain) if hasattr(controller, 'goal2_chain') else 0
            goal1_area_count = len(controller.goal1_area_robots) if hasattr(controller, 'goal1_area_robots') else 0
            goal2_area_count = len(controller.goal2_area_robots) if hasattr(controller, 'goal2_area_robots') else 0
            guardian_count = len(controller.origin_guardians) if hasattr(controller, 'origin_guardians') else 0
            candidate_count = len(controller.expansion_candidates) if hasattr(controller, 'expansion_candidates') else 0
            violations = controller.connectivity_violations if hasattr(controller, 'connectivity_violations') else 0
            forced_balances = controller.forced_balance_counter if hasattr(controller, 'forced_balance_counter') else 0


            print(f"  Connected: {connected_count}/{len(robots)} ({connectivity_ratio:.1%})")
            print(f"  Guardians: {guardian_count}, Goal1 chain: {goal1_chain_length}, Goal2 chain: {goal2_chain_length}")
            print(f"  Goal1 area: {goal1_area_count}, Goal2 area: {goal2_area_count}")
            print(f"  Expansion candidates: {candidate_count}, Violations: {violations}")
            print(f"  Forced balanced assignments: {forced_balances}")
            
        if iteration < BROWNIAN_PHASE_DURATION:
            phase = "brownian"
            if iteration == 0:
                print("Starting Brownian motion phase...")
        elif iteration >= RETRACTION_START_ITERATION:
            phase = "retraction"
            if iteration == RETRACTION_START_ITERATION:
                controller.activate_retraction_mode()
        else:
            phase = "enhanced_expansion"
            if iteration == BROWNIAN_PHASE_DURATION:
                controller.activate()
        
        if phase == "brownian":
            for robot in robots:
                robot.apply_brownian_motion(BROWNIAN_INTENSITY * 2)
                robot.update_position()
            edges = []
        else:
            control_forces = controller.compute_control_forces()
            
            for i, robot in enumerate(robots):
                robot.apply_brownian_motion(BROWNIAN_INTENSITY * 0.01)
                robot.apply_control_force(control_forces[i])
                robot.update_position()
            
            edges = []
            for edge in controller.spanning_tree.edges():
                if edge[0] == "origin":
                    pos1 = origin_position
                else:
                    robot1 = next((r for r in robots if r.id == edge[0]), None)
                    pos1 = robot1.position if robot1 else None
                
                if edge[1] == "origin":
                    pos2 = origin_position
                else:
                    robot2 = next((r for r in robots if r.id == edge[1]), None)
                    pos2 = robot2.position if robot2 else None
                
                if pos1 is not None and pos2 is not None:
                    edges.append((pos1.copy(), pos2.copy()))
        
        all_positions.append([robot.position.copy() for robot in robots])
        all_roles.append([robot.role for robot in robots])
        all_edges.append(edges)
        all_phases.append(phase)
    
    print("Enhanced Chain Expansion Simulation complete. Starting animation...")
    animate_simulation(all_positions, all_roles, all_edges, all_phases, 
                      goal1_position, goal2_position, origin_position, robots, obstacles)

def animate_simulation(all_positions, all_roles, all_edges, all_phases, 
                      goal1_position, goal2_position, origin_position, robots, obstacles):
    print("Creating Enhanced Chain Expansion animation with obstacles...")
    
    fig, ax = plt.subplots(figsize=(14, 10))
    
    def update(frame):
        ax.clear()
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        
        phase = all_phases[frame]
        if phase == "brownian":
            ax.set_title(f"Brownian Motion with Obstacles - Frame {frame+1}/{len(all_positions)}", fontsize=14)
            title_color = 'blue'
        elif phase == "retraction":
            ax.set_title(f"RETRACTION with Obstacles - Frame {frame+1}/{len(all_positions)}", fontsize=14)
            title_color = 'purple'
        else:
            ax.set_title(f"BALANCED Chain Expansion with Goal Areas & Obstacles - Frame {frame+1}/{len(all_positions)}", fontsize=14)
            title_color = 'darkgreen'
        
        ax.title.set_color(title_color)
        
        # NEW: Plot obstacles FIRST (so they appear behind other elements)
                # NEW: Plot obstacles FIRST (so they appear behind other elements)
        for obstacle in obstacles:
            # Create rectangle patch
            from matplotlib.patches import Rectangle
            import matplotlib.transforms as transforms
            
            # Create rectangle
            rect = Rectangle((obstacle.center[0] - obstacle.width/2, obstacle.center[1] - obstacle.height/2),
                           obstacle.width, obstacle.height,
                           color='black', fill=True, alpha=0.9, edgecolor='darkgray', linewidth=1)
            
            # Apply rotation
            if obstacle.angle != 0:
                t = transforms.Affine2D().rotate_around(obstacle.center[0], obstacle.center[1], obstacle.angle) + ax.transData
                rect.set_transform(t)
            
            ax.add_patch(rect)
            
            # Add obstacle label
            ax.text(obstacle.center[0], obstacle.center[1], 'BAR', 
                   ha='center', va='center', color='white', fontsize=7, fontweight='bold')
        # Plot spanning tree
        if phase != "brownian" and all_edges[frame]:
            for edge in all_edges[frame]:
                ax.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], 'k-', linewidth=2.5, alpha=0.8)
        
        # Plot communication radius around origin
        if phase != "brownian":
            origin_circle = plt.Circle((origin_position[0], origin_position[1]), COMM_RADIUS, 
                                     color='green', fill=False, linestyle=':', linewidth=2, alpha=0.7)
            ax.add_patch(origin_circle)
        
        # Plot goal detection areas
        if phase == "enhanced_expansion":
            goal1_area = plt.Circle((goal1_position[0], goal1_position[1]), GOAL_DETECTION_RADIUS, 
                                  color='orange', fill=False, linestyle=':', linewidth=2, alpha=0.5)
            ax.add_patch(goal1_area)
            
            goal2_area = plt.Circle((goal2_position[0], goal2_position[1]), GOAL_DETECTION_RADIUS, 
                                  color='cyan', fill=False, linestyle=':', linewidth=2, alpha=0.5)
            ax.add_patch(goal2_area)
            
            # Plot optimal spacing circles
            for i in range(1, 4):
                spacing_circle = plt.Circle((origin_position[0], origin_position[1]), OPTIMAL_CHAIN_SPACING * i, 
                                          color='lightblue', fill=False, linestyle='--', linewidth=1, alpha=0.3)
                ax.add_patch(spacing_circle)
        
        # Plot retraction radius
        if phase == "retraction":
            retraction_circle = plt.Circle((origin_position[0], origin_position[1]), RETRACTION_RADIUS, 
                                         color='purple', fill=False, linestyle='-', linewidth=2, alpha=0.8)
            ax.add_patch(retraction_circle)
        
        positions = all_positions[frame]
        roles = all_roles[frame]
        
        # Plot robots with enhanced role-based colors
        for i, (pos, role) in enumerate(zip(positions, roles)):
            if phase == "brownian":
                color = 'blue'
                size = 80
            elif role == "retracting":
                color = 'purple'
                size = 100
            elif role == "origin_guardian":
                color = 'red'
                size = 110
            elif role == "goal1_leader":
                color = 'orange'
                size = 140
            elif role == "goal2_leader":
                color = 'cyan'
                size = 140
            elif role == "goal1_chain":
                color = 'gold'
                size = 120
            elif role == "goal2_chain":
                color = 'lightcyan'
                size = 120
            elif role == "goal1_area":
                color = 'darkorange'
                size = 130
            elif role == "goal2_area":
                color = 'darkturquoise'
                size = 130
            elif role == "expansion_candidate":
                color = 'yellow'
                size = 90
            else:  # follower
                color = 'lightblue'
                size = 70
            
            ax.scatter(pos[0], pos[1], c=color, s=size, alpha=0.9, edgecolors='black', linewidth=1.5)
            ax.text(pos[0], pos[1], str(i+1), ha='center', va='center', color='white', 
                   fontsize=7, fontweight='bold')
        
        # Mark origin and goals
        ax.plot(origin_position[0], origin_position[1], 'ko', markersize=18, markerfacecolor='black')
        ax.text(origin_position[0], origin_position[1] - 0.3, 'ORIGIN', 
                ha='center', va='center', color='black', fontsize=11, fontweight='bold')
        
        ax.plot(goal1_position[0], goal1_position[1], 'g*', markersize=30)
        ax.text(goal1_position[0], goal1_position[1] + 0.3, 'GOAL 1', 
                ha='center', va='center', color='green', fontsize=11, fontweight='bold')
        
        ax.plot(goal2_position[0], goal2_position[1], 'c*', markersize=30)
        ax.text(goal2_position[0], goal2_position[1] + 0.3, 'GOAL 2', 
                ha='center', va='center', color='cyan', fontsize=11, fontweight='bold')
        
        # Algorithm indicator
        ax.text(0.02, 0.98, 'Enhanced Chain\nExpansion with\nGoal Areas &\nObstacles', transform=ax.transAxes, 
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.9),
                fontsize=10, fontweight='bold', va='top')
        
        # Enhanced legend
        if phase == "brownian":
            ax.scatter([], [], c='blue', s=80, label='Brownian Motion', edgecolors='black')
        elif phase == "retraction":
            ax.scatter([], [], c='purple', s=100, label='Retracting', edgecolors='black')
            ax.plot([], [], 'purple', linestyle='-', linewidth=2, label='Retraction Zone')
        else:
            ax.scatter([], [], c='red', s=110, label='Origin Guardians', edgecolors='black')
            ax.scatter([], [], c='orange', s=140, label='Goal 1 Leader', edgecolors='black')
            ax.scatter([], [], c='cyan', s=140, label='Goal 2 Leader', edgecolors='black')
            ax.scatter([], [], c='gold', s=120, label='Goal 1 Chain', edgecolors='black')
            ax.scatter([], [], c='lightcyan', s=120, label='Goal 2 Chain', edgecolors='black')
            ax.scatter([], [], c='darkorange', s=130, label='Goal 1 Area', edgecolors='black')
            ax.scatter([], [], c='darkturquoise', s=130, label='Goal 2 Area', edgecolors='black')
            ax.scatter([], [], c='yellow', s=90, label='Expansion Candidates', edgecolors='black')
            ax.scatter([], [], c='lightblue', s=70, label='Followers', edgecolors='black')
            ax.plot([], [], 'k-', linewidth=2.5, label='Spanning Tree', alpha=0.8)
            ax.plot([], [], 'orange', linestyle=':', linewidth=2, label='Goal Areas', alpha=0.5)
        
        # NEW: Obstacle legend
        ax.scatter([], [], c='black', s=100, label='Bar Obstacles', alpha=0.9, edgecolors='darkgray', marker='s')        
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        
        return []
    
    try:
        anim = animation.FuncAnimation(fig, update, frames=len(all_positions),
                                       blit=False, interval=100, repeat=True)
        
        if SAVE_GIF:
            try:
                anim.save('enhanced_chain_expansion_with_obstacles.gif', writer='pillow', fps=10)
                print("Animation saved!")
            except Exception as e:
                print(f"Error saving animation: {e}")
        
        plt.show(block=True)
        
    except Exception as e:
        print(f"Animation error: {e}")

if __name__ == "__main__":
    try:
        run_simulation()
    except Exception as e:
        print(f"Simulation error: {e}")
        import traceback
        traceback.print_exc()