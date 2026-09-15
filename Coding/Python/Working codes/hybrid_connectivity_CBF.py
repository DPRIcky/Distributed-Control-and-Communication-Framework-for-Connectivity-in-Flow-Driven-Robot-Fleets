"""
Hybrid Decentralized MLCCST with CLF-CBF Control
Complete implementation with aggressive chain control and connectivity bypass

This implements a hybrid decentralized multi-robot coordination system that transitions
from centralized to fully decentralized control using CLF-CBF framework.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import networkx as nx
from scipy.optimize import minimize
from scipy.spatial.distance import cdist
import random
from typing import List, Tuple, Dict, Set, Optional
import time

class Robot:
    """Individual robot with local communication and consensus capabilities"""
    
    def __init__(self, robot_id: int, position: np.ndarray, comm_radius: float):
        self.id = robot_id
        self.position = position.copy()
        self.velocity = np.zeros(2)
        self.comm_radius = comm_radius
        
        # Local state
        self.local_neighbors = set()
        self.local_goals = {}  # goal_id: (position, priority)
        self.local_role = 'origin_guardian'  # origin_guardian, chain_leader, chain_member, goal_area
        self.assigned_goal_id = None
        self.chain_id = None
        self.is_chain_leader = False
        self.chain_target = None  # Specific target for chain connectivity
        self.bridging_target = None  # For origin guardians acting as bridges
        
        # Consensus variables
        self.goal_consensus_state = {}
        self.role_consensus_state = {}
        
        # Communication
        self.message_buffer = []
        self.last_communication_time = 0
        
    def discover_local_neighbors(self, all_robots: List['Robot']) -> Set[int]:
        """Discover neighbors within communication radius"""
        neighbors = set()
        for other_robot in all_robots:
            if other_robot.id != self.id:
                distance = np.linalg.norm(self.position - other_robot.position)
                if distance <= self.comm_radius:
                    neighbors.add(other_robot.id)
        
        self.local_neighbors = neighbors
        return neighbors
    
    def send_message(self, message_type: str, data: dict, recipient_id: int = None):
        """Send message to neighbors or specific robot"""
        message = {
            'sender_id': self.id,
            'type': message_type,
            'data': data,
            'timestamp': time.time(),
            'recipient_id': recipient_id
        }
        self.message_buffer.append(message)
    
    def receive_messages(self, all_robots: List['Robot']):
        """Receive and process messages from neighbors"""
        received_messages = []
        for other_robot in all_robots:
            if other_robot.id in self.local_neighbors:
                for message in other_robot.message_buffer:
                    if (message['recipient_id'] is None or 
                        message['recipient_id'] == self.id):
                        received_messages.append(message)
        
        # Process messages
        for message in received_messages:
            self._process_message(message)
    
    def _process_message(self, message: dict):
        """Process individual message"""
        if message['type'] == 'goal_update':
            goal_data = message['data']
            self.local_goals.update(goal_data)
        elif message['type'] == 'role_update':
            role_data = message['data']
            if role_data['robot_id'] in self.local_neighbors:
                self.role_consensus_state[role_data['robot_id']] = role_data['role']
    
    def local_goal_consensus(self) -> Dict:
        """Perform local consensus on goal assignments"""
        # Simple averaging consensus for goal priorities
        for goal_id in self.local_goals:
            if goal_id not in self.goal_consensus_state:
                self.goal_consensus_state[goal_id] = {
                    'position': self.local_goals[goal_id][0],
                    'priority': self.local_goals[goal_id][1],
                    'update_count': 1
                }
            else:
                # Update consensus
                old_priority = self.goal_consensus_state[goal_id]['priority']
                new_priority = self.local_goals[goal_id][1]
                self.goal_consensus_state[goal_id]['priority'] = (old_priority + new_priority) / 2
                self.goal_consensus_state[goal_id]['update_count'] += 1
        
        return self.goal_consensus_state
    
    def local_role_consensus(self, all_robots: List['Robot']) -> str:
        """Determine role based on local consensus"""
        neighbor_roles = {}
        for neighbor_id in self.local_neighbors:
            neighbor_robot = next((r for r in all_robots if r.id == neighbor_id), None)
            if neighbor_robot:
                neighbor_roles[neighbor_id] = neighbor_robot.local_role
        
        # Role decision logic based on local information
        if self.assigned_goal_id is not None:
            distance_to_goal = np.linalg.norm(
                self.position - self.local_goals.get(self.assigned_goal_id, (np.zeros(2), 0))[0]
            )
            
            if distance_to_goal > 2.0 * self.comm_radius:
                if not any(role == 'chain_leader' for role in neighbor_roles.values()):
                    return 'chain_leader'
                else:
                    return 'chain_member'
            else:
                return 'goal_area'
        
        return 'origin_guardian'


class CLFCBFController:
    """Enhanced CLF-CBF controller with aggressive goal-seeking and connectivity bypass"""
    
    def __init__(self, obstacles: List['Obstacle'], comm_radius: float):
        self.obstacles = obstacles
        self.comm_radius = comm_radius
        
        # Aggressive control parameters
        self.clf_gain = 2.0
        self.cbf_gain = 1.0
        self.max_velocity = 0.3
        self.safety_margin = 0.1
        
        # Ultra-aggressive parameters for chain robots
        self.chain_leader_clf_multiplier = 20.0  # Ultra-high gain for leaders
        self.chain_member_clf_multiplier = 15.0  # Very high gain for members
        self.chain_speed_multiplier = 5.0  # Much faster movement for leaders
        self.chain_member_speed_multiplier = 4.0  # Faster movement for members
        
    def compute_clf_cbf_control(self, robot: Robot, robots: List[Robot], 
                               goals: List[np.ndarray], spanning_tree: nx.Graph) -> np.ndarray:
        """Compute control input using CLF-CBF with aggressive connectivity bypass"""
        
        # Determine if robot is in chain role
        is_chain_robot = robot.local_role in ['chain_leader', 'chain_member']
        is_chain_leader = robot.local_role == 'chain_leader'
        
        # Get target position
        target = self._get_robot_target(robot, goals)
        if target is None:
            return np.zeros(2)
        
        # Compute nominal control (CLF)
        u_clf = self._compute_clf_control(robot, target, is_chain_robot, is_chain_leader)
        
        # For chain robots, use ultra-aggressive control and bypass connectivity
        if is_chain_robot:
            # Apply ultra-high gains and speeds
            if is_chain_leader:
                u_clf *= self.chain_leader_clf_multiplier
                max_vel = self.max_velocity * self.chain_speed_multiplier
            else:
                u_clf *= self.chain_member_clf_multiplier
                max_vel = self.max_velocity * self.chain_member_speed_multiplier
            
            # Only apply safety CBF (no connectivity CBF)
            u_final = self._apply_safety_cbf_only(robot, u_clf, max_vel)
            
            print(f"Chain robot {robot.id} ({robot.local_role}): "
                  f"pos={robot.position}, target={target}, "
                  f"control_mag={np.linalg.norm(u_final):.3f}")
            
            return u_final
        
        # For non-chain robots, use standard control with all constraints
        constraints = self._get_cbf_constraints(robot, robots, spanning_tree)
        
        if len(constraints) == 0:
            return np.clip(u_clf, -self.max_velocity, self.max_velocity)
        
        # Solve QP with constraints
        u_safe = self._solve_clf_cbf_qp(u_clf, constraints)
        return np.clip(u_safe, -self.max_velocity, self.max_velocity)
    
    def _apply_safety_cbf_only(self, robot: Robot, u_clf: np.ndarray, max_vel: float) -> np.ndarray:
        """Apply only safety CBF constraints for chain robots"""
        # Get only obstacle constraints
        safety_constraints = []
        
        for obstacle in self.obstacles:
            h, dh_dx = self._obstacle_barrier_function(robot.position, obstacle)
            if h < 0.5:  # Only apply when close to obstacles
                constraint = {
                    'type': 'ineq',
                    'fun': lambda u, h=h, dh_dx=dh_dx: dh_dx @ u + self.cbf_gain * h
                }
                safety_constraints.append(constraint)
        
        if len(safety_constraints) == 0:
            return np.clip(u_clf, -max_vel, max_vel)
        
        # Solve QP with only safety constraints
        def objective(u):
            return np.linalg.norm(u - u_clf)**2
        
        result = minimize(objective, u_clf, method='SLSQP', constraints=safety_constraints)
        
        if result.success:
            return np.clip(result.x, -max_vel, max_vel)
        else:
            return np.clip(u_clf, -max_vel, max_vel)
    
    def _compute_clf_control(self, robot: Robot, target: np.ndarray, 
                           is_chain_robot: bool, is_chain_leader: bool) -> np.ndarray:
        """Compute CLF control with role-based gains"""
        error = target - robot.position
        distance = np.linalg.norm(error)
        
        if distance < 0.05:
            return np.zeros(2)
        
        # Base control
        u_clf = self.clf_gain * error
        
        # Enhanced control for chain robots with direct goal targeting
        if is_chain_robot:
            # Direct proportional control toward goal
            if is_chain_leader:
                u_clf = 3.0 * error  # Very aggressive for leaders
            else:
                u_clf = 2.5 * error  # Aggressive for chain members
        
        return u_clf
    
    def _get_robot_target(self, robot: Robot, goals: List[np.ndarray]) -> Optional[np.ndarray]:
        """Get target position for robot with obstacle avoidance"""
        # Use chain target if available (for chain connectivity)
        if robot.chain_target is not None:
            # Check if direct path to chain target is obstacle-free
            safe_target = self._get_obstacle_aware_target(robot.position, robot.chain_target)
            return safe_target
        
        # For origin guardians without specific targets, maintain tight origin cluster
        if robot.local_role == 'origin_guardian':
            origin = np.array([0.0, 0.0])
            
            # Keep origin guardians extremely close together for connectivity
            offset_angle = robot.id * 0.6
            offset_distance = self.comm_radius * 0.05  # Extremely close - 5% of comm radius
            target = origin + offset_distance * np.array([np.cos(offset_angle), np.sin(offset_angle)])
            
            # Make sure this target is obstacle-free
            safe_target = self._get_obstacle_aware_target(robot.position, target)
            return safe_target
        
        # Otherwise use assigned goal
        if robot.assigned_goal_id is not None and robot.assigned_goal_id < len(goals):
            goal_target = goals[robot.assigned_goal_id]
            safe_target = self._get_obstacle_aware_target(robot.position, goal_target)
            return safe_target
        return None

    def _get_obstacle_aware_target(self, current_pos: np.ndarray, desired_target: np.ndarray) -> np.ndarray:
        """Get a safe target that avoids obstacles"""
        # Check if direct path is clear
        if self._is_path_clear(current_pos, desired_target):
            return desired_target
        
        # If path is blocked, find alternative path around obstacles
        return self._find_safe_target_around_obstacles(current_pos, desired_target)
    
    def _is_path_clear(self, start: np.ndarray, end: np.ndarray) -> bool:
        """Check if straight line path between two points is obstacle-free"""
        # Sample points along the path
        num_samples = int(np.linalg.norm(end - start) / 0.1) + 10  # Sample every 0.1 units
        for i in range(num_samples + 1):
            t = i / num_samples
            point = start + t * (end - start)
            
            # Check collision with each obstacle
            for obstacle in self.obstacles:
                if obstacle.distance_to_point(point) < self.safety_margin:
                    return False
        return True
    
    def _find_safe_target_around_obstacles(self, current_pos: np.ndarray, desired_target: np.ndarray) -> np.ndarray:
        """Find a safe intermediate target that goes around obstacles"""
        # Simple obstacle avoidance: move perpendicular to closest obstacle
        closest_obstacle = None
        min_distance = float('inf')
        
        for obstacle in self.obstacles:
            dist = obstacle.distance_to_point(current_pos)
            if dist < min_distance:
                min_distance = dist
                closest_obstacle = obstacle
        
        if closest_obstacle and min_distance < self.comm_radius:
            # Get direction away from obstacle
            closest_point = closest_obstacle.closest_point_to(current_pos)
            away_from_obstacle = (current_pos - closest_point)
            if np.linalg.norm(away_from_obstacle) > 0:
                away_from_obstacle = away_from_obstacle / np.linalg.norm(away_from_obstacle)
                
                # Move in a direction that's away from obstacle but still towards target
                to_target = desired_target - current_pos
                if np.linalg.norm(to_target) > 0:
                    to_target = to_target / np.linalg.norm(to_target)
                    
                    # Blend the two directions
                    safe_direction = 0.7 * away_from_obstacle + 0.3 * to_target
                    safe_direction = safe_direction / np.linalg.norm(safe_direction)
                    
                    # Move a small step in the safe direction
                    step_size = min(0.3, np.linalg.norm(desired_target - current_pos))
                    return current_pos + step_size * safe_direction
        
        return desired_target
    
    def _get_cbf_constraints(self, robot: Robot, robots: List[Robot], 
                           spanning_tree: nx.Graph) -> List[dict]:
        """Get all CBF constraints (safety + connectivity)"""
        constraints = []
        
        # Safety constraints (obstacle avoidance)
        for obstacle in self.obstacles:
            h, dh_dx = self._obstacle_barrier_function(robot.position, obstacle)
            if h < 1.0:  # Apply when close to obstacles
                constraint = {
                    'type': 'ineq',
                    'fun': lambda u, h=h, dh_dx=dh_dx: dh_dx @ u + self.cbf_gain * h
                }
                constraints.append(constraint)
        
        # Connectivity constraints (only for non-chain robots)
        if robot.local_role == 'origin_guardian':
            for other_robot in robots:
                if (spanning_tree.has_edge(robot.id, other_robot.id) and 
                    other_robot.id != robot.id):
                    h_conn, dh_conn_dx = self._connectivity_barrier_function(
                        robot.position, other_robot.position
                    )
                    if h_conn < 0.5:
                        constraint = {
                            'type': 'ineq',
                            'fun': lambda u, h=h_conn, dh_dx=dh_conn_dx: dh_dx @ u + self.cbf_gain * h
                        }
                        constraints.append(constraint)
        
        return constraints
    
    def _solve_clf_cbf_qp(self, u_clf: np.ndarray, constraints: List[dict]) -> np.ndarray:
        """Solve CLF-CBF quadratic program"""
        def objective(u):
            return np.linalg.norm(u - u_clf)**2
        
        result = minimize(objective, u_clf, method='SLSQP', constraints=constraints)
        
        if result.success:
            return result.x
        else:
            return u_clf
    
    def _obstacle_barrier_function(self, position: np.ndarray, obstacle: 'Obstacle') -> Tuple[float, np.ndarray]:
        """Compute barrier function for obstacle avoidance"""
        distance = obstacle.distance_to_point(position)
        h = distance - self.safety_margin
        
        # Gradient computation
        if distance > 0:
            closest_point = obstacle.closest_point_to(position)
            direction = (position - closest_point) / distance
            dh_dx = direction
        else:
            dh_dx = np.array([1.0, 0.0])  # Default direction
        
        return h, dh_dx
    
    def _connectivity_barrier_function(self, pos1: np.ndarray, pos2: np.ndarray) -> Tuple[float, np.ndarray]:
        """Compute barrier function for connectivity maintenance"""
        distance = np.linalg.norm(pos1 - pos2)
        h = self.comm_radius - distance
        
        if distance > 0:
            dh_dx = -(pos1 - pos2) / distance
        else:
            dh_dx = np.array([0.0, 0.0])
        
        return h, dh_dx


class HybridDecentralizedController:
    """Main controller for hybrid centralized-to-decentralized transition"""
    
    def __init__(self, robots: List[Robot], goals: List[np.ndarray], 
                 obstacles: List['Obstacle'], comm_radius: float):
        self.robots = robots
        self.goals = goals
        self.obstacles = obstacles
        self.comm_radius = comm_radius
        
        # Control components
        self.clf_cbf_controller = CLFCBFController(obstacles, comm_radius)
        
        # State management
        self.centralization_factor = 1.0  # 1.0 = fully centralized, 0.0 = fully decentralized
        self.decentralization_rate = 0.02
        self.spanning_tree = nx.Graph()
        
        # Goal assignment
        self.goal_assignments = {}  # robot_id: goal_id
        self.chain_assignments = {}  # robot_id: chain_id
        self.chain_leaders = set()
        
        # Force immediate aggressive chain assignment
        self.force_immediate_chain_assignment()
        
        print(f"Initialized hybrid controller with {len(robots)} robots and {len(goals)} goals")
        print(f"Aggressive chain assignment completed")
    
    def force_immediate_chain_assignment(self):
        """Force immediate chain assignment with global connectivity"""
        print("Forcing immediate chain assignment with global connectivity...")
        
        # Clear existing assignments
        for robot in self.robots:
            robot.local_role = 'unassigned'
            robot.assigned_goal_id = None
            robot.chain_id = None
            robot.is_chain_leader = False
            robot.chain_target = None
        
        self.goal_assignments.clear()
        self.chain_assignments.clear()
        self.chain_leaders.clear()
        
        # Origin center (where most robots are clustered)
        origin = np.array([0.0, 0.0])
        
        # FORCE 4 robots to be origin guardians FIRST (reduced from 6 to leave more for chains)
        origin_guardian_count = 4  # Reduced to ensure enough robots for all chains
        robots_by_distance = [(i, np.linalg.norm(robot.position - origin)) 
                             for i, robot in enumerate(self.robots)]
        robots_by_distance.sort(key=lambda x: x[1])
        
        print(f"FORCING {origin_guardian_count} robots to be origin guardians:")
        for i in range(min(origin_guardian_count, len(robots_by_distance))):
            robot_id, distance = robots_by_distance[i]
            robot = self.robots[robot_id]
            robot.local_role = 'origin_guardian'
            robot.assigned_goal_id = None
            print(f"  Robot {robot_id} -> FORCED origin guardian (dist: {distance:.3f})")
        
        # Now create chains with REMAINING robots only
        available_for_chains = [i for i, robot in enumerate(self.robots) 
                               if robot.local_role != 'origin_guardian']
        
        print(f"Available for chains: {len(available_for_chains)} robots")
        
        # Create chains for goals in order of difficulty - ENSURE every goal gets at least 1 robot
        goal_distances = [(i, np.linalg.norm(goal_pos - origin)) 
                         for i, goal_pos in enumerate(self.goals)]
        goal_distances.sort(key=lambda x: x[1], reverse=True)
        
        # First pass: Give every goal at least 1 robot
        for goal_id, distance in goal_distances:
            remaining_robots = [i for i in available_for_chains 
                              if self.robots[i].assigned_goal_id is None]
            if len(remaining_robots) > 0:
                print(f"  Goal {goal_id} (dist: {distance:.2f}) - Assigning at least 1 robot")
                self._create_connected_chain_to_goal(goal_id, self.goals[goal_id], origin, force_single=True)
        
        # Second pass: Distribute remaining robots to extend chains
        for goal_id, distance in goal_distances:
            remaining_robots = [i for i in available_for_chains 
                              if self.robots[i].assigned_goal_id is None]
            if len(remaining_robots) > 0:
                print(f"  Goal {goal_id} (dist: {distance:.2f}) - Extending chain with {len(remaining_robots)} remaining")
                self._create_connected_chain_to_goal(goal_id, self.goals[goal_id], origin, force_single=False)
        
        # Final summary
        origin_guardians = [r for r in self.robots if r.local_role == 'origin_guardian']
        chain_robots = [r for r in self.robots if r.local_role in ['chain_leader', 'chain_member']]
        
        print(f"FINAL ASSIGNMENT:")
        print(f"  Origin guardians: {len(origin_guardians)} robots")
        print(f"  Chain robots: {len(chain_robots)} robots")
        print(f"  Total assigned: {len(origin_guardians) + len(chain_robots)}")
        
        for goal_id in range(len(self.goals)):
            goal_robots = [r for r in self.robots if r.assigned_goal_id == goal_id]
            leaders = [r for r in goal_robots if r.local_role == 'chain_leader']
            members = [r for r in goal_robots if r.local_role == 'chain_member']
            print(f"    Goal {goal_id}: {len(leaders)} leaders, {len(members)} members")
    
    def _assign_origin_guardians(self, origin: np.ndarray):
        """Simple origin guardian assignment - will be called after chain assignment"""
        # This method is now unused - origin guardians are assigned after chains
        pass

    def _create_connected_chain_to_goal(self, goal_id: int, goal_pos: np.ndarray, origin: np.ndarray, force_single=False):
        """Create a connectivity chain from origin network to goal"""
        distance_to_goal = np.linalg.norm(goal_pos - origin)
        
        # Count available robots
        available_robots = [i for i, robot in enumerate(self.robots) 
                           if robot.assigned_goal_id is None]
        
        if len(available_robots) == 0:
            print(f"  No robots available for goal {goal_id}")
            return
        
        if force_single:
            # Force only 1 robot assignment for initial distribution
            actual_chain_length = 1
            print(f"  Goal {goal_id}: FORCED single robot assignment")
        else:
            # Calculate how many robots needed for chain (more conservative spacing)
            chain_length = int(np.ceil(distance_to_goal / (self.comm_radius * 0.6)))  # 0.6 for better connectivity
            print(f"  Goal {goal_id}: needs {chain_length} robots, have {len(available_robots)} available")
            
            # If we don't have enough robots for full chain, create as long a chain as possible
            actual_chain_length = min(chain_length, len(available_robots))
        
        if actual_chain_length <= 1:
            # Assign single robot to move towards goal (even if can't reach it)
            closest_idx = available_robots[0]
            # Find best position this robot can reach towards goal
            origin_edge = self._find_origin_network_edge_towards_goal(goal_pos, origin)
            direction = (goal_pos - origin_edge) / np.linalg.norm(goal_pos - origin_edge)
            
            # Position robot as far towards goal as possible while maintaining connectivity
            max_distance = self.comm_radius * 0.7  # Conservative reach
            target_pos = origin_edge + direction * max_distance
            
            self._assign_robot_to_chain(closest_idx, goal_id, 'chain_leader', target_pos)
            print(f"  Assigned single robot {closest_idx} towards goal {goal_id} (force_single: {force_single})")
            return
        
        # Create chain positions from origin network edge towards goal
        origin_edge = self._find_origin_network_edge_towards_goal(goal_pos, origin)
        chain_positions = []
        
        # Create positions for partial chain (might not reach goal fully)
        max_reachable_distance = actual_chain_length * self.comm_radius * 0.6
        actual_distance = min(distance_to_goal, max_reachable_distance)
        
        for i in range(actual_chain_length):
            t = (i + 1) / actual_chain_length
            pos = origin_edge + t * (actual_distance / distance_to_goal) * (goal_pos - origin_edge)
            chain_positions.append(pos)
        
        # Assign robots to chain positions
        assigned_robots = []
        for pos_idx, target_pos in enumerate(chain_positions):
            if not available_robots:
                break
                
            # Find closest available robot to this chain position
            distances = [np.linalg.norm(self.robots[i].position - target_pos) for i in available_robots]
            closest_idx = available_robots[np.argmin(distances)]
            
            # Determine role: last robot in chain is leader, others are members
            role = 'chain_leader' if pos_idx == len(chain_positions) - 1 else 'chain_member'
            
            self._assign_robot_to_chain(closest_idx, goal_id, role, target_pos)
            assigned_robots.append(closest_idx)
            available_robots.remove(closest_idx)
        
        print(f"  Created chain for goal {goal_id} with {len(assigned_robots)} robots (reach: {actual_distance:.2f}/{distance_to_goal:.2f})")

    def _find_origin_network_edge_towards_goal(self, goal_pos: np.ndarray, origin: np.ndarray) -> np.ndarray:
        """Find the edge of origin network closest to the goal direction"""
        direction_to_goal = goal_pos - origin
        direction_to_goal = direction_to_goal / np.linalg.norm(direction_to_goal)
        
        # Move one communication radius out from origin in goal direction
        edge_point = origin + direction_to_goal * (self.comm_radius * 0.5)
        return edge_point
    
    def _assign_robot_to_chain(self, robot_id: int, goal_id: int, role: str, target_pos: np.ndarray):
        """Assign a robot to a chain with specific role"""
        robot = self.robots[robot_id]
        
        # CRITICAL FIX: Don't reassign origin guardians!
        if robot.local_role == 'origin_guardian':
            print(f"ERROR: Attempting to assign origin guardian robot {robot_id} to chain! BLOCKING!")
            return
        
        robot.assigned_goal_id = goal_id
        robot.local_role = role
        robot.chain_id = goal_id
        robot.is_chain_leader = (role == 'chain_leader')
        
        self.goal_assignments[robot_id] = goal_id
        self.chain_assignments[robot_id] = goal_id
        
        if role == 'chain_leader':
            self.chain_leaders.add(robot_id)
        
        print(f"Robot {robot_id} assigned as {role} for goal {goal_id} at {target_pos}")
    
    def maintain_chain_connectivity(self):
        """Maintain connectivity within chains and overall network during simulation"""
        # First, update the communication graph
        self._update_spanning_tree()
        
        # Position origin guardians for connectivity
        self._position_origin_guardians_for_connectivity()
        
        # Then maintain chain-specific connectivity with strict range enforcement
        for goal_id in range(len(self.goals)):
            chain_robots = [robot for robot in self.robots 
                          if robot.chain_id == goal_id and robot.assigned_goal_id is not None]
            
            if len(chain_robots) == 0:
                continue
                
            # Sort chain robots by distance from origin to form proper chain order
            origin = np.array([0.0, 0.0])
            chain_robots.sort(key=lambda r: np.linalg.norm(r.position - origin))
            
            # Set targets for chain formation with strict communication range
            goal_pos = self.goals[goal_id]
            
            if len(chain_robots) == 1:
                # Single chain robot: check if goal is reachable
                robot = chain_robots[0]
                origin_guardians = [r for r in self.robots if r.local_role == 'origin_guardian']
                
                if origin_guardians:
                    # Find the closest guardian to the origin (not to this robot)
                    closest_guardian = min(origin_guardians, 
                                         key=lambda r: np.linalg.norm(r.position - origin))
                    
                    goal_distance = np.linalg.norm(goal_pos - origin)
                    direction = (goal_pos - origin) / goal_distance
                    
                    # Calculate maximum reachable distance from origin network
                    guardian_dist_from_origin = np.linalg.norm(closest_guardian.position - origin)
                    max_reach = guardian_dist_from_origin + self.comm_radius * 0.7
                    
                    if goal_distance <= max_reach + 0.25:  # Add buffer for goal area
                        # Goal is reachable - position outside goal with obstacle avoidance
                        safe_distance = 0.25
                        desired_target = goal_pos - direction * safe_distance
                        robot.chain_target = self._get_obstacle_aware_target(robot.position, desired_target)
                    else:
                        # Goal too far - extend towards goal more aggressively
                        # Try to get as close as possible to goal
                        desired_target = origin + direction * (max_reach * 1.2)  # Be more aggressive
                        robot.chain_target = self._get_obstacle_aware_target(robot.position, desired_target)
                        
            else:
                # Multiple chain robots: form stepping stones with strict range limits
                self._form_chain_with_range_limits(chain_robots, goal_pos, origin)

    def _form_chain_with_range_limits(self, chain_robots, goal_pos, origin):
        """Form chain with strict communication range limits"""
        # Calculate spacing for optimal connectivity
        num_robots = len(chain_robots)
        max_range_per_hop = self.comm_radius * 0.7  # Conservative spacing
        
        goal_distance = np.linalg.norm(goal_pos - origin)
        direction = (goal_pos - origin) / goal_distance
        
        # Find the closest origin guardian to start the chain from
        origin_guardians = [r for r in self.robots if r.local_role == 'origin_guardian']
        if origin_guardians:
            closest_guardian = min(origin_guardians, key=lambda r: np.linalg.norm(r.position - origin))
            chain_start_distance = np.linalg.norm(closest_guardian.position - origin)
        else:
            chain_start_distance = 0.5  # Default start distance
        
        # Calculate maximum reachable distance with current chain
        max_total_reach = chain_start_distance + (num_robots * max_range_per_hop)
        
        if goal_distance <= max_total_reach + 0.25:  # Goal is reachable
            # Position chain to reach goal - work forwards from origin
            chain_leader = chain_robots[-1]
            
            # Chain leader targets near goal
            safe_distance = 0.25
            chain_leader.chain_target = goal_pos - direction * safe_distance
            
            # Position intermediate robots progressively
            for i, robot in enumerate(chain_robots[:-1]):  # Exclude chain leader
                if i == 0:
                    # First robot starts from closest guardian
                    if origin_guardians:
                        closest_guardian = min(origin_guardians, 
                                             key=lambda g: np.linalg.norm(g.position - robot.position))
                        start_pos = closest_guardian.position
                    else:
                        start_pos = origin
                    
                    robot.chain_target = start_pos + direction * max_range_per_hop
                else:
                    # Subsequent robots extend from previous robot
                    prev_target = chain_robots[i-1].chain_target
                    robot.chain_target = prev_target + direction * max_range_per_hop
        
        else:
            # Goal too far - extend chain to maximum reach more aggressively
            aggressive_spacing = self.comm_radius * 0.8  # More aggressive spacing
            
            for i, robot in enumerate(chain_robots):
                if i == 0 and origin_guardians:
                    # Start from closest guardian
                    closest_guardian = min(origin_guardians, 
                                         key=lambda g: np.linalg.norm(g.position - robot.position))
                    start_pos = closest_guardian.position
                    robot.chain_target = start_pos + direction * aggressive_spacing
                else:
                    # Extend further with more aggressive spacing
                    base_distance = chain_start_distance if i == 0 else 0
                    hop_distance = base_distance + ((i + 1) * aggressive_spacing)
                    robot.chain_target = origin + direction * hop_distance

    def _position_origin_guardians_for_connectivity(self):
        """Position origin guardians to maintain connectivity between chains and origin"""
        origin = np.array([0.0, 0.0])
        origin_guardians = [robot for robot in self.robots if robot.local_role == 'origin_guardian']
        chain_robots = [robot for robot in self.robots if robot.local_role in ['chain_leader', 'chain_member']]
        
        if not origin_guardians or not chain_robots:
            return
            
        # Find chain robots that are disconnected from origin guardians
        for chain_robot in chain_robots:
            # Check if this chain robot has connectivity to any origin guardian
            connected_to_origin = False
            closest_guardian = None
            min_dist = float('inf')
            
            for guardian in origin_guardians:
                dist = np.linalg.norm(chain_robot.position - guardian.position)
                if dist <= self.comm_radius:
                    connected_to_origin = True
                    break
                if dist < min_dist:
                    min_dist = dist
                    closest_guardian = guardian
            
            # If not connected, try to bridge the gap with an available guardian
            if not connected_to_origin and closest_guardian and min_dist < self.comm_radius * 2:
                # Find an unoccupied guardian to act as bridge
                available_guardians = [g for g in origin_guardians 
                                     if not hasattr(g, 'bridging_target') or g.bridging_target is None]
                
                if available_guardians:
                    bridge_guardian = min(available_guardians,
                                        key=lambda g: np.linalg.norm(g.position - chain_robot.position))
                    
                    # Position bridge guardian between origin cluster and chain robot
                    to_chain = chain_robot.position - origin
                    if np.linalg.norm(to_chain) > 0:
                        direction = to_chain / np.linalg.norm(to_chain)
                        bridge_distance = min(self.comm_radius * 0.6, np.linalg.norm(to_chain) * 0.5)
                        bridge_guardian.chain_target = origin + direction * bridge_distance
                        bridge_guardian.bridging_target = chain_robot.id

    def _get_obstacle_aware_target(self, start_pos: np.ndarray, desired_target: np.ndarray) -> np.ndarray:
        """Get obstacle-aware target position using simple obstacle avoidance"""
        # Check if direct path to target intersects with obstacles
        for obstacle in self.clf_cbf_controller.obstacles:
            if self._path_intersects_obstacle(start_pos, desired_target, obstacle):
                # Find alternative target that avoids obstacle
                return self._find_alternative_target(start_pos, desired_target, obstacle)
        
        return desired_target
    
    def _path_intersects_obstacle(self, start: np.ndarray, end: np.ndarray, obstacle) -> bool:
        """Check if path from start to end intersects with obstacle"""
        # Get obstacle center and radius (use diagonal as radius for rectangular obstacles)
        center = obstacle.center
        radius = np.sqrt(obstacle.width**2 + obstacle.height**2) / 2
        
        # Check distance from obstacle center to line segment
        path_vec = end - start
        path_length = np.linalg.norm(path_vec)
        
        if path_length < 1e-6:
            return False
            
        path_unit = path_vec / path_length
        to_center = center - start
        projection_length = np.dot(to_center, path_unit)
        
        # Clamp projection to line segment
        projection_length = max(0, min(projection_length, path_length))
        closest_point = start + projection_length * path_unit
        
        distance_to_obstacle = np.linalg.norm(closest_point - center)
        return distance_to_obstacle < (radius + 0.2)  # Add safety margin
    
    def _find_alternative_target(self, start: np.ndarray, desired_target: np.ndarray, obstacle) -> np.ndarray:
        """Find alternative target that avoids obstacle"""
        # Get obstacle center and radius
        center = obstacle.center
        radius = np.sqrt(obstacle.width**2 + obstacle.height**2) / 2
        
        # Vector from obstacle to desired target
        to_target = desired_target - center
        if np.linalg.norm(to_target) < 1e-6:
            to_target = np.array([1.0, 0.0])
        
        to_target = to_target / np.linalg.norm(to_target)
        
        # Create alternative targets by going around obstacle
        avoidance_distance = radius + 0.3  # Safety margin
        
        # Try two directions around obstacle
        perpendicular = np.array([-to_target[1], to_target[0]])
        
        option1 = center + perpendicular * avoidance_distance
        option2 = center - perpendicular * avoidance_distance
        
        # Choose the option closer to desired target
        dist1 = np.linalg.norm(option1 - desired_target)
        dist2 = np.linalg.norm(option2 - desired_target)
        
        return option1 if dist1 < dist2 else option2
    
    def step(self, dt: float):
        """Execute one control step"""
        # Update decentralization
        self.centralization_factor = max(0.0, self.centralization_factor - self.decentralization_rate * dt)
        
        # Update robot neighborhoods
        for robot in self.robots:
            robot.discover_local_neighbors(self.robots)
        
        # Handle communication and consensus
        if self.centralization_factor > 0.5:
            # Centralized phase: direct goal assignment
            self.hybrid_target_assignment()
        else:
            # Decentralized phase: local consensus
            self._perform_local_consensus()
        
        # Update spanning tree
        self._update_spanning_tree()
        
        # Maintain chain connectivity
        self.maintain_chain_connectivity()
        
        # Compute control inputs
        for robot in self.robots:
            control_input = self.clf_cbf_controller.compute_clf_cbf_control(
                robot, self.robots, self.goals, self.spanning_tree
            )
            
            # Update robot state
            robot.velocity = control_input
            robot.position += robot.velocity * dt
            
            # Ensure robots stay in bounds
            robot.position = np.clip(robot.position, [-5, -5], [5, 5])
    
    def hybrid_target_assignment(self):
            """Enhanced hybrid target assignment with proper chain targeting"""
            if self.centralization_factor > 0.5:
                # Maintain chain connectivity during centralized phase
                self.maintain_chain_connectivity()
                
                # Ensure chain robots maintain their assignments and roles
                for robot in self.robots:
                    if robot.id in self.goal_assignments:
                        goal_id = self.goal_assignments[robot.id]
                        robot.assigned_goal_id = goal_id
                        
                        # Maintain chain roles
                        if robot.id in self.chain_leaders:
                            robot.local_role = 'chain_leader'
                        elif robot.id in self.chain_assignments:
                            robot.local_role = 'chain_member'
                        
                        # Set proper chain targets based on role
                        if robot.local_role == 'chain_leader':
                            # Chain leaders position outside goal area towards goal
                            goal_pos = self.goals[goal_id]
                            origin = np.array([0.0, 0.0])
                            direction = (goal_pos - origin) / np.linalg.norm(goal_pos - origin)
                            safe_distance = 0.25  # Stay outside goal area
                            desired_target = goal_pos - direction * safe_distance
                            
                            # Make sure target is obstacle-free
                            robot.chain_target = self._get_obstacle_aware_target(robot.position, desired_target)
                        
                        # Chain members get their targets set in maintain_chain_connectivity
            
            # Update local goals for all robots
            for robot in self.robots:
                robot.local_goals = {i: (goal, 1.0) for i, goal in enumerate(self.goals)}
        
    def _perform_local_consensus(self):
            """Perform local consensus among robots"""
            # Exchange messages
            for robot in self.robots:
                robot.send_message('goal_update', robot.local_goals)
                robot.send_message('role_update', {
                    'robot_id': robot.id,
                    'role': robot.local_role
                })
            
            # Receive and process messages
            for robot in self.robots:
                robot.receive_messages(self.robots)
                robot.local_goal_consensus()
                robot.local_role = robot.local_role_consensus(self.robots)
        
    def _update_spanning_tree(self):
        """Update robot connections based on communication range"""
        num_robots = len(self.robots)
        positions = np.array([robot.position for robot in self.robots])
        
        # Create communication graph - use ALL connections within comm radius
        A = np.zeros((num_robots, num_robots))
        origin_guardians = []
        for i in range(num_robots):
            if self.robots[i].local_role == 'origin_guardian':
                origin_guardians.append(i)
            for j in range(i + 1, num_robots):
                dist = np.linalg.norm(positions[i] - positions[j])
                if dist <= self.comm_radius:
                    A[i, j] = A[j, i] = 1
        
        # Debug origin guardian connectivity
        print(f"Origin guardians: {origin_guardians}")
        if len(origin_guardians) > 1:
            for i in range(len(origin_guardians)):
                for j in range(i + 1, len(origin_guardians)):
                    idx1, idx2 = origin_guardians[i], origin_guardians[j]
                    dist = np.linalg.norm(positions[idx1] - positions[idx2])
                    connected = "YES" if A[idx1, idx2] == 1 else "NO"
                    print(f"  Guardian {idx1} <-> {idx2}: dist={dist:.3f}, connected={connected}")
        
        # Update robot connections with ALL valid connections (not just MST)
        for robot in self.robots:
            robot.local_neighbors.clear()
        
        # Add all communication-range connections
        for i in range(num_robots):
            for j in range(i + 1, num_robots):
                if A[i, j] == 1:
                    self.robots[i].local_neighbors.add(j)
                    self.robots[j].local_neighbors.add(i)
        
        # Check connectivity for debugging
        connected_components = self._find_connected_components(A)
        if len(connected_components) > 1:
            print(f"Network fragmented into {len(connected_components)} components")
            for comp_idx, component in enumerate(connected_components):
                roles = [self.robots[i].local_role for i in component]
                print(f"  Component {comp_idx}: robots {component}, roles {roles}")
        
        return A
    
    def _find_connected_components(self, adjacency_matrix):
        """Find connected components in the communication graph"""
        num_robots = len(adjacency_matrix)
        visited = [False] * num_robots
        components = []
        
        def dfs(node, component):
            visited[node] = True
            component.append(node)
            for neighbor in range(num_robots):
                if adjacency_matrix[node][neighbor] == 1 and not visited[neighbor]:
                    dfs(neighbor, component)
        
        for i in range(num_robots):
            if not visited[i]:
                component = []
                dfs(i, component)
                components.append(component)
        
        return components
    
    def _add_essential_connections(self, adjacency_matrix, positions, components):
        """Add minimum connections to ensure global connectivity"""
        A_enhanced = adjacency_matrix.copy()
        
        # Connect components by finding closest robot pairs between components
        while len(components) > 1:
            min_dist = float('inf')
            best_connection = None
            comp1_idx, comp2_idx = None, None
            
            # Find closest pair between different components
            for i, comp1 in enumerate(components):
                for j, comp2 in enumerate(components[i+1:], i+1):
                    for robot1 in comp1:
                        for robot2 in comp2:
                            dist = np.linalg.norm(positions[robot1] - positions[robot2])
                            if dist < min_dist:
                                min_dist = dist
                                best_connection = (robot1, robot2)
                                comp1_idx, comp2_idx = i, j
            
            # Add the connection
            if best_connection:
                r1, r2 = best_connection
                A_enhanced[r1, r2] = A_enhanced[r2, r1] = 1
                print(f"Added essential connection: Robot {r1} ↔ Robot {r2} (dist: {min_dist:.2f})")
                
                # Merge the components
                merged_component = components[comp1_idx] + components[comp2_idx]
                new_components = []
                for i, comp in enumerate(components):
                    if i != comp1_idx and i != comp2_idx:
                        new_components.append(comp)
                new_components.append(merged_component)
                components = new_components
            else:
                break
        
        return A_enhanced
    
    def _find_mst(self, distance_matrix):
        """Find minimum spanning tree using Prim's algorithm"""
        num_robots = len(distance_matrix)
        if num_robots == 0:
            return []
        
        mst_edges = []
        in_mst = [False] * num_robots
        in_mst[0] = True
        
        while len(mst_edges) < num_robots - 1:
            min_weight = float('inf')
            min_edge = None
            
            for i in range(num_robots):
                if in_mst[i]:
                    for j in range(num_robots):
                        if not in_mst[j] and distance_matrix[i][j] < min_weight:
                            min_weight = distance_matrix[i][j]
                            min_edge = (i, j)
            
            if min_edge:
                mst_edges.append(min_edge)
                in_mst[min_edge[1]] = True
            else:
                break
        
        return mst_edges
        
    def get_debug_info(self) -> Dict:
            """Get comprehensive debug information"""
            chain_robots = []
            for robot in self.robots:
                if robot.local_role in ['chain_leader', 'chain_member']:
                    chain_robots.append({
                        'id': robot.id,
                        'role': robot.local_role,
                        'goal_id': robot.assigned_goal_id,
                        'position': robot.position.copy(),
                        'target': self.goals[robot.assigned_goal_id] if robot.assigned_goal_id is not None else None,
                        'distance_to_goal': np.linalg.norm(robot.position - self.goals[robot.assigned_goal_id]) if robot.assigned_goal_id is not None else None
                    })
            
            return {
                'centralization_factor': self.centralization_factor,
                'num_chain_leaders': len(self.chain_leaders),
                'num_chain_members': len([r for r in self.robots if r.local_role == 'chain_member']),
                'chain_robots': chain_robots,
                'total_assigned': len(self.goal_assignments)
            }


class Obstacle:
    """Rectangular obstacle for navigation challenges"""
    
    def __init__(self, center: np.ndarray, width: float, height: float):
        self.center = center
        self.width = width
        self.height = height
        
        # Compute corners
        self.corners = np.array([
            [center[0] - width/2, center[1] - height/2],
            [center[0] + width/2, center[1] - height/2],
            [center[0] + width/2, center[1] + height/2],
            [center[0] - width/2, center[1] + height/2]
        ])
    
    def distance_to_point(self, point: np.ndarray) -> float:
        """Compute minimum distance from point to obstacle"""
        # Distance to rectangle
        dx = max(self.center[0] - self.width/2 - point[0], 0, point[0] - (self.center[0] + self.width/2))
        dy = max(self.center[1] - self.height/2 - point[1], 0, point[1] - (self.center[1] + self.height/2))
        return np.sqrt(dx**2 + dy**2)
    
    def closest_point_to(self, point: np.ndarray) -> np.ndarray:
        """Find closest point on obstacle to given point"""
        closest_x = np.clip(point[0], self.center[0] - self.width/2, self.center[0] + self.width/2)
        closest_y = np.clip(point[1], self.center[1] - self.height/2, self.center[1] + self.height/2)
        return np.array([closest_x, closest_y])
    
    def is_point_inside(self, point: np.ndarray) -> bool:
        """Check if point is inside obstacle"""
        return (abs(point[0] - self.center[0]) <= self.width/2 and 
                abs(point[1] - self.center[1]) <= self.height/2)


class SimulationEnvironment:
    """Complete simulation environment for hybrid decentralized MLCCST"""
    
    def __init__(self, num_robots: int = 40, num_goals: int = 8, comm_radius: float = 0.8):
            self.num_robots = num_robots
            self.num_goals = num_goals
            self.comm_radius = comm_radius
            
            # Initialize environment
            self.robots = []
            self.goals = []
            self.obstacles = []
            self.controller = None
            
            # Simulation parameters
            self.dt = 0.05
            self.max_time = 100.0
            self.time = 0.0
            
            # Visualization
            self.fig = None
            self.ax = None
            self.robot_plots = []
            self.goal_plots = []
            self.obstacle_plots = []
            
            # Data collection
            self.trajectory_data = []
            self.performance_data = []
            
    def setup_scenario(self):
            """Set up the complete simulation scenario"""
            print("Setting up hybrid decentralized MLCCST scenario...")
            
            # Create robots in clustered formation around origin
            self._create_robots_clustered()
            
            # Create goals in strategic positions
            self._create_strategic_goals()
            
            # Create obstacle environment
            self._create_obstacle_environment()
            
            # Initialize controller
            self.controller = HybridDecentralizedController(
                self.robots, self.goals, self.obstacles, self.comm_radius
            )
            
            print(f"Scenario setup complete:")
            print(f"  - {len(self.robots)} robots")
            print(f"  - {len(self.goals)} goals")
            print(f"  - {len(self.obstacles)} obstacles")
            
    def _create_robots_clustered(self):
            """Create robots in clustered formation around origin"""
            self.robots = []
            
            # Create robots in tight cluster around origin
            center = np.array([0.0, 0.0])
            cluster_radius = 0.6
            
            for i in range(self.num_robots):
                angle = 2 * np.pi * i / self.num_robots
                radius = cluster_radius * np.sqrt(np.random.uniform(0, 1))
                
                position = center + radius * np.array([np.cos(angle), np.sin(angle)])
                
                # Add small random offset
                position += np.random.normal(0, 0.05, 2)
                
                robot = Robot(i, position, self.comm_radius)
                self.robots.append(robot)
            
            print(f"Created {len(self.robots)} robots in clustered formation")
        
    def _create_strategic_goals(self):
            """Create goals in random positions to test algorithm robustness"""
            self.goals = []
            
            # Define workspace bounds (leaving some margin from edges)
            x_min, x_max = -4.0, 4.0
            y_min, y_max = -4.0, 4.0
            min_distance_from_origin = 1.5  # Ensure goals are not too close to origin
            min_distance_between_goals = 1.0  # Ensure goals are not too close to each other
            max_attempts = 100  # Prevent infinite loops
            
            print(f"Generating {self.num_goals} random goals...")
            
            for goal_idx in range(self.num_goals):
                attempts = 0
                valid_position = False
                
                while not valid_position and attempts < max_attempts:
                    # Generate random position
                    x = random.uniform(x_min, x_max)
                    y = random.uniform(y_min, y_max)
                    candidate_pos = np.array([x, y])
                    
                    # Check distance from origin
                    distance_from_origin = np.linalg.norm(candidate_pos)
                    if distance_from_origin < min_distance_from_origin:
                        attempts += 1
                        continue
                    
                    # Check distance from existing goals
                    too_close_to_existing = False
                    for existing_goal in self.goals:
                        if np.linalg.norm(candidate_pos - existing_goal) < min_distance_between_goals:
                            too_close_to_existing = True
                            break
                    
                    if too_close_to_existing:
                        attempts += 1
                        continue
                    
                    # Check if position overlaps with obstacles (will be checked later if obstacles exist)
                    # For now, we'll just place goals and check obstacle overlap in setup validation
                    overlaps_obstacle = False
                    # Note: Obstacle checking will be done after obstacles are created
                    
                    if overlaps_obstacle:
                        attempts += 1
                        continue
                    
                    # Position is valid
                    self.goals.append(candidate_pos)
                    print(f"  Goal {goal_idx}: [{x:.2f}, {y:.2f}]")
                    valid_position = True
                    
                    attempts += 1
                
                if not valid_position:
                    # Fallback to a safe position if we couldn't find a valid random one
                    fallback_angle = (goal_idx * 2 * np.pi) / self.num_goals
                    fallback_pos = 3.0 * np.array([np.cos(fallback_angle), np.sin(fallback_angle)])
                    self.goals.append(fallback_pos)
                    print(f"  Goal {goal_idx}: [{fallback_pos[0]:.2f}, {fallback_pos[1]:.2f}] (fallback)")
            
            print(f"Created {len(self.goals)} random goals")
        
    def _create_obstacle_environment(self):
            """Create challenging obstacle environment"""
            self.obstacles = []
            
            # Create strategic obstacles that require coordination (simplified - no vertical barriers)
            obstacle_configs = [
                # Central barrier
                {"center": np.array([0.0, 1.5]), "width": 2.0, "height": 0.3},
                {"center": np.array([0.0, -1.5]), "width": 2.0, "height": 0.3},
                
                # Corner obstacles
                {"center": np.array([2.5, 2.5]), "width": 0.5, "height": 0.5},
                {"center": np.array([-2.5, 2.5]), "width": 0.5, "height": 0.5},
                {"center": np.array([2.5, -2.5]), "width": 0.5, "height": 0.5},
                {"center": np.array([-2.5, -2.5]), "width": 0.5, "height": 0.5}
            ]
            
            for config in obstacle_configs:
                obstacle = Obstacle(config["center"], config["width"], config["height"])
                self.obstacles.append(obstacle)
            
            print(f"Created {len(self.obstacles)} strategic obstacles")
        
    def run_simulation(self, animate: bool = True, save_animation: bool = False):
            """Run the complete simulation"""
            print("Starting hybrid decentralized MLCCST simulation...")
            
            if animate:
                self._setup_visualization()
                if save_animation:
                    self._run_animated_simulation_with_save()
                else:
                    self._run_animated_simulation()
            else:
                self._run_headless_simulation()
        
    def _setup_visualization(self):
            """Set up matplotlib visualization"""
            self.fig, self.ax = plt.subplots(figsize=(12, 10))
            self.ax.set_xlim(-5, 5)
            self.ax.set_ylim(-5, 5)
            self.ax.set_aspect('equal')
            self.ax.grid(True, alpha=0.3)
            self.ax.set_title('Hybrid Decentralized MLCCST with Aggressive Chain Control', fontsize=14)
            
            # Initialize plots
            self.robot_plots = []
            self.goal_plots = []
            self.obstacle_plots = []
            
            # Plot goals
            for i, goal in enumerate(self.goals):
                circle = plt.Circle(goal, 0.15, color='red', alpha=0.7, zorder=5)
                self.ax.add_patch(circle)
                self.ax.text(goal[0]+0.2, goal[1]+0.2, f'G{i}', fontsize=8, fontweight='bold')
                self.goal_plots.append(circle)
            
            # Plot obstacles
            for obstacle in self.obstacles:
                rect = patches.Rectangle(
                    (obstacle.center[0] - obstacle.width/2, obstacle.center[1] - obstacle.height/2),
                    obstacle.width, obstacle.height,
                    linewidth=2, edgecolor='black', facecolor='gray', alpha=0.8, zorder=3
                )
                self.ax.add_patch(rect)
                self.obstacle_plots.append(rect)
            
            # Initialize robot plots
            for i, robot in enumerate(self.robots):
                color = self._get_robot_color(robot)
                circle = plt.Circle(robot.position, 0.05, color=color, zorder=4)
                self.ax.add_patch(circle)
                self.robot_plots.append(circle)
        
            # Add communication radius indicator
            comm_circle = plt.Circle((0, 0), self.comm_radius, fill=False, 
                                    linestyle='--', color='green', alpha=0.5, zorder=1)
            self.ax.add_patch(comm_circle)
            self.ax.text(-4.5, 4.5, f'Comm Radius: {self.comm_radius:.1f}', 
                        bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen"), fontsize=10)
            
            # Add legend
            self._add_legend()
            
            # Initialize chain lines list
            self.chain_lines = []
    
    def _get_robot_color(self, robot: Robot) -> str:
        """Get color based on robot role"""
        if robot.local_role == 'chain_leader':
            return 'blue'
        elif robot.local_role == 'chain_member':
            return 'cyan'
        elif robot.local_role == 'goal_area':
            return 'green'
        else:
            return 'orange'  # origin_guardian
        
    def _add_legend(self):
        """Add legend to visualization"""
        legend_elements = [
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', 
                      markersize=8, label='Chain Leader'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='cyan', 
                      markersize=8, label='Chain Member'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='green', 
                      markersize=8, label='Goal Area'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='orange', 
                      markersize=8, label='Origin Guardian'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='red', 
                      markersize=8, label='Goals')
        ]
        self.ax.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(1, 1))
    
    def _draw_chain_connections(self):
        """Draw lines showing chain connectivity and spanning tree"""
        if not hasattr(self, 'chain_lines'):
            self.chain_lines = []
            
        origin = np.array([0.0, 0.0])
        
        # Draw chains for each goal
        for goal_id in range(len(self.goals)):
            chain_robots = [robot for robot in self.robots 
                          if robot.chain_id == goal_id and robot.assigned_goal_id is not None]
            
            if len(chain_robots) <= 1:
                continue
                
            # Sort chain robots by distance from origin to show proper chain order
            chain_robots.sort(key=lambda r: np.linalg.norm(r.position - origin))
            
            # Draw lines between consecutive robots in chain
            for i in range(len(chain_robots) - 1):
                robot1 = chain_robots[i]
                robot2 = chain_robots[i + 1]
                
                line = self.ax.plot([robot1.position[0], robot2.position[0]], 
                                  [robot1.position[1], robot2.position[1]], 
                                  'b-', alpha=0.6, linewidth=2, label='Chain Connection')[0]
                self.chain_lines.append(line)
        
        # Draw spanning tree connections (global connectivity)
        for robot in self.robots:
            for neighbor_id in robot.local_neighbors:
                if neighbor_id > robot.id:  # Avoid drawing duplicate lines
                    neighbor = self.robots[neighbor_id]
                    
                    # Different line style for spanning tree vs chain connections
                    is_chain_connection = (robot.chain_id == neighbor.chain_id and 
                                         robot.chain_id is not None)
                    
                    if not is_chain_connection:
                        # This is a spanning tree connection between different chains or to origin
                        line = self.ax.plot([robot.position[0], neighbor.position[0]], 
                                          [robot.position[1], neighbor.position[1]], 
                                          'violet', alpha=0.6, linewidth=1.5, linestyle=':', label='Spanning Tree')[0]
                        self.chain_lines.append(line)
    
    def _update_visualization(self, frame):
        """Update visualization for animation"""
        # Run simulation step
        self.controller.step(self.dt)
        self.time += self.dt
        
        # Clear previous chain lines
        for line in getattr(self, 'chain_lines', []):
            line.remove()
        self.chain_lines = []
        
        # Update robot positions and colors
        for i, robot in enumerate(self.robots):
            self.robot_plots[i].center = tuple(robot.position)
            self.robot_plots[i].set_color(self._get_robot_color(robot))
        
        # Draw chain connectivity lines
        self._draw_chain_connections()
        
        # Update title with debug info
        debug_info = self.controller.get_debug_info()
        title = (f'Hybrid Decentralized MLCCST - Time: {self.time:.1f}s - '
                f'Centralization: {debug_info["centralization_factor"]:.2f}\n'
                f'Chain Leaders: {debug_info["num_chain_leaders"]}, '
                f'Chain Members: {debug_info["num_chain_members"]}, '
                f'Total Assigned: {debug_info["total_assigned"]}')
        self.ax.set_title(title, fontsize=12)
        
        # Collect performance data
        self._collect_performance_data()
        
        # Print debug info periodically
        if frame % 20 == 0:  # Every second at 20 FPS
            print(f"Time {self.time:.1f}s: {debug_info['num_chain_leaders']} leaders, "
                  f"{debug_info['num_chain_members']} members")
            
            # Print chain robot positions
            for chain_robot in debug_info['chain_robots'][:3]:  # Show first 3
                print(f"  Robot {chain_robot['id']} ({chain_robot['role']}): "
                      f"goal {chain_robot['goal_id']}, dist {chain_robot['distance_to_goal']:.3f}")
        
        return self.robot_plots
        
    def _run_animated_simulation(self):
            """Run simulation with real-time animation"""
            frames = int(self.max_time / self.dt)
            ani = FuncAnimation(self.fig, self._update_visualization, frames=frames, 
                            interval=50, blit=False, repeat=False)
            plt.show()
        
    def _run_animated_simulation_with_save(self):
            """Run simulation and save animation"""
            frames = int(self.max_time / self.dt)
            ani = FuncAnimation(self.fig, self._update_visualization, frames=frames, 
                            interval=50, blit=False, repeat=False)
            
            # Save animation
            print("Saving animation...")
            ani.save('hybrid_decentralized_mlccst_simulation.gif', writer='pillow', fps=20)
            print("Animation saved as 'hybrid_decentralized_mlccst_simulation.gif'")
            
            plt.show()
        
    def _run_headless_simulation(self):
            """Run simulation without visualization"""
            steps = int(self.max_time / self.dt)
            
            print("Running headless simulation...")
            for step in range(steps):
                self.controller.step(self.dt)
                self.time += self.dt
                
                # Collect data
                self._collect_performance_data()
                
                # Print progress
                if step % 200 == 0:
                    debug_info = self.controller.get_debug_info()
                    print(f"Step {step}/{steps}: Time {self.time:.1f}s, "
                        f"Leaders: {debug_info['num_chain_leaders']}, "
                        f"Members: {debug_info['num_chain_members']}")
            
            print("Headless simulation complete")
            self._generate_performance_report()
        
    def _collect_performance_data(self):
            """Collect performance metrics"""
            # Calculate goal coverage
            goals_reached = 0
            total_distance_to_goals = 0
            
            for robot in self.robots:
                if robot.assigned_goal_id is not None:
                    goal_pos = self.goals[robot.assigned_goal_id]
                    distance = np.linalg.norm(robot.position - goal_pos)
                    total_distance_to_goals += distance
                    
                    if distance < 0.2:  # Goal reached threshold
                        goals_reached += 1
            
            # Calculate connectivity
            connected_pairs = 0
            total_pairs = 0
            
            for i, robot1 in enumerate(self.robots):
                for j, robot2 in enumerate(self.robots[i+1:], i+1):
                    total_pairs += 1
                    distance = np.linalg.norm(robot1.position - robot2.position)
                    if distance <= self.comm_radius:
                        connected_pairs += 1
            
            connectivity_ratio = connected_pairs / total_pairs if total_pairs > 0 else 0
            
            # Store performance data
            performance_entry = {
                'time': self.time,
                'goals_reached': goals_reached,
                'avg_distance_to_goals': total_distance_to_goals / len(self.robots),
                'connectivity_ratio': connectivity_ratio,
                'centralization_factor': self.controller.centralization_factor
            }
            
            self.performance_data.append(performance_entry)
        
    def _generate_performance_report(self):
            """Generate comprehensive performance report"""
            print("\n" + "="*60)
            print("HYBRID DECENTRALIZED MLCCST PERFORMANCE REPORT")
            print("="*60)
            
            final_data = self.performance_data[-1]
            
            print(f"Final Results after {self.time:.1f} seconds:")
            print(f"  Goals Reached: {final_data['goals_reached']}/{len(self.goals)}")
            print(f"  Average Distance to Goals: {final_data['avg_distance_to_goals']:.3f}")
            print(f"  Connectivity Ratio: {final_data['connectivity_ratio']:.3f}")
            print(f"  Final Centralization Factor: {final_data['centralization_factor']:.3f}")
            
            # Calculate improvement over time
            initial_data = self.performance_data[0]
            improvement = {
                'goals': final_data['goals_reached'] - initial_data['goals_reached'],
                'distance': initial_data['avg_distance_to_goals'] - final_data['avg_distance_to_goals'],
                'connectivity': final_data['connectivity_ratio'] - initial_data['connectivity_ratio']
            }
            
            print(f"\nImprovement over simulation:")
            print(f"  Goals reached improvement: +{improvement['goals']}")
            print(f"  Distance improvement: {improvement['distance']:.3f}")
            print(f"  Connectivity improvement: {improvement['connectivity']:.3f}")
            
            # Algorithm-specific metrics
            debug_info = self.controller.get_debug_info()
            print(f"\nChain Formation Analysis:")
            print(f"  Chain Leaders: {debug_info['num_chain_leaders']}")
            print(f"  Chain Members: {debug_info['num_chain_members']}")
            print(f"  Total Chain Robots: {debug_info['num_chain_leaders'] + debug_info['num_chain_members']}")
            print(f"  Chain Formation Rate: {(debug_info['num_chain_leaders'] + debug_info['num_chain_members'])/len(self.robots)*100:.1f}%")
            
            print("="*60)


def main():
        """Main execution function"""
        print("Initializing Hybrid Decentralized MLCCST Simulation")
        print("="*60)
        
        # Create simulation environment
        sim = SimulationEnvironment(
            num_robots=40,
            num_goals=8,
            comm_radius=0.8
        )
        
        # Setup scenario
        sim.setup_scenario()
        
        # Run simulation
        print("\nChoose simulation mode:")
        print("1. Animated simulation (real-time)")
        print("2. Animated simulation with save")
        print("3. Headless simulation (fast)")
        
        try:
            choice = input("Enter choice (1-3): ").strip()
            
            if choice == "1":
                sim.run_simulation(animate=True, save_animation=False)
            elif choice == "2":
                sim.run_simulation(animate=True, save_animation=True)
            elif choice == "3":
                sim.run_simulation(animate=False)
            else:
                print("Invalid choice, running animated simulation...")
                sim.run_simulation(animate=True, save_animation=False)
        
        except KeyboardInterrupt:
            print("\nSimulation interrupted by user")
        except Exception as e:
            print(f"Error during simulation: {e}")
            print("Running headless simulation as fallback...")
            sim.run_simulation(animate=False)
        
        print("\nSimulation complete!")


if __name__ == "__main__":
    main()