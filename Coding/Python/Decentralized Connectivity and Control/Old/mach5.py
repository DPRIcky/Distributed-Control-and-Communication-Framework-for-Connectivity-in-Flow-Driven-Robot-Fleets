import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import cvxpy as cp
import matplotlib.animation as animation

# Parameters
NUM_ROBOTS = 20
COMM_RADIUS = 1.0
TIME_STEP = 0.1
MAX_SPEED = 0.5
FORMATION_GAIN = 0.8  # Controls how strictly to maintain formation
GOAL_GAIN = 0.5       # Controls attraction to goal
MAX_ITERATIONS = 300
SAVE_GIF = True       # Whether to save animation as a GIF

class Robot:
    def __init__(self, id, initial_position):
        self.id = id
        self.position = np.array(initial_position, dtype=float)
        self.velocity = np.zeros(2)
        self.neighbors = []
        self.initial_position = np.array(initial_position)  # Save initial relative position
        
    def update_neighbors(self, all_robots):
        self.neighbors = []
        for robot in all_robots:
            if robot.id != self.id:
                dist = np.linalg.norm(self.position - robot.position)
                if dist <= COMM_RADIUS:
                    self.neighbors.append(robot)
    
    def compute_control_input(self, leader_position, goal_position):
        """Compute control input using CLF-CBF approach"""
        # Setup QP variables
        u = cp.Variable(2)  # Control input (velocity)
        
        # CLF term to move toward goal (leader handles this)
        if self.id == 1:  # Leader robot (at center)
            # Leader moves directly toward goal
            goal_direction = goal_position - self.position
            goal_distance = np.linalg.norm(goal_direction)
            if goal_distance > 0.01:
                u_des = GOAL_GAIN * goal_direction / goal_distance
            else:
                u_des = np.zeros(2)
        else:
            # Followers maintain formation relative to leader
            # Calculate where this robot should be relative to leader
            desired_rel_pos = self.initial_position - np.array([0, 0])  # Relative to origin
            desired_position = leader_position + desired_rel_pos
            
            # Move toward the desired position
            formation_direction = desired_position - self.position
            formation_distance = np.linalg.norm(formation_direction)
            if formation_distance > 0.01:
                u_des = FORMATION_GAIN * formation_direction
            else:
                u_des = np.zeros(2)
        
        # Objective: minimize the difference between desired and actual control
        objective = cp.Minimize(cp.sum_squares(u - u_des))
        
        constraints = []
        
        # CBF to maintain connectivity with leader/neighbors
        if self.id != 1:  # For non-leader robots
            # Maintain distance to leader
            leader_diff = leader_position - self.position
            leader_dist = np.linalg.norm(leader_diff)
            
            # CBF constraint: ensure robots stay within comm radius of leader
            h = COMM_RADIUS**2 - leader_dist**2
            if h > 0:  # Only add constraint if within comm radius
                dh_dx = 2 * leader_diff
                alpha = 0.5  # CBF gain
                constraints.append(dh_dx @ u >= -alpha * h)
        
        # Speed limit constraint
        constraints.append(cp.norm(u, 2) <= MAX_SPEED)
        
        # Solve QP
        prob = cp.Problem(objective, constraints)
        try:
            prob.solve()
            if u.value is not None:
                return u.value
            else:
                return np.zeros(2)
        except:
            return np.zeros(2)  # Fallback if optimization fails

# Initialize robots in a star formation
def create_star_formation(num_robots, radius=0.9):
    robots = []
    
    # Center robot (ID = 1)
    robots.append(Robot(1, [0, 0]))
    
    # Outer robots in a circle
    for i in range(1, num_robots):
        angle = 2 * np.pi * (i-1) / (num_robots-1)
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        robots.append(Robot(i+1, [x, y]))
    
    return robots

# Create robots in star formation
robots = create_star_formation(NUM_ROBOTS)

# Create a single goal position at (5,5)
goal_position = np.array([5.0, 5.0])  # Position at (5,5)

# For animation
robot_positions = []  # Store positions for each frame
robot_edges = []      # Store edges for each frame

# Run simulation
for iteration in range(MAX_ITERATIONS):
    # Store current positions for animation
    current_positions = {robot.id: robot.position.copy() for robot in robots}
    robot_positions.append(current_positions)
    
    # Get leader position (robot 1)
    leader_position = robots[0].position
    
    # Update robot positions
    for robot in robots:
        # Compute control input
        u = robot.compute_control_input(leader_position, goal_position)
        # Update position
        robot.position = robot.position + u * TIME_STEP
        
    # Store current edges
    current_edges = []
    for robot in robots:
        if robot.id != 1:
            current_edges.append((1, robot.id))
    robot_edges.append(current_edges)
    
    # Check if leader has reached the goal
    dist_to_goal = np.linalg.norm(robots[0].position - goal_position)
    if dist_to_goal < 0.2:
        print(f"Goal reached in {iteration} iterations!")
        break

# Create a graph for visualization
G = nx.Graph()

# Create the animation
fig, ax = plt.subplots(figsize=(10, 8))

def init():
    ax.clear()
    # Draw custom coordinate system
    ax.axhline(y=0, color='k', linestyle='-', alpha=0.7, zorder=-1)
    ax.axvline(x=0, color='k', linestyle='-', alpha=0.7, zorder=-1)
    
    # Add grid
    for i in range(-2, 6):
        ax.axhline(y=i, color='gray', linestyle=':', alpha=0.4, zorder=-2)
        ax.axvline(x=i, color='gray', linestyle=':', alpha=0.4, zorder=-2)
    
    # Add x and y axis labels
    for i in range(-2, 6):
        if i != 0:
            ax.text(i, -0.2, str(i), ha='center', va='center', fontsize=12)
            ax.text(-0.2, i, str(i), ha='center', va='center', fontsize=12)
    
    # Add origin label
    ax.text(-0.2, -0.2, "0", ha='center', va='center', fontsize=12)
    
    # X and Y axis arrows and labels
    ax.annotate('X', xy=(5.5, 0), xytext=(5.3, 0), 
                arrowprops=dict(facecolor='black', shrink=0.05),
                fontsize=14)
    ax.annotate('Y', xy=(0, 5.5), xytext=(0, 5.3), 
                arrowprops=dict(facecolor='black', shrink=0.05),
                fontsize=14)
    
    ax.set_xlim([-2.5, 5.5])
    ax.set_ylim([-2.5, 5.5])
    return []

def update(frame):
    ax.clear()
    
    # Draw grid and axes
    ax.axhline(y=0, color='k', linestyle='-', alpha=0.7, zorder=-1)
    ax.axvline(x=0, color='k', linestyle='-', alpha=0.7, zorder=-1)
    
    for i in range(-2, 6):
        ax.axhline(y=i, color='gray', linestyle=':', alpha=0.4, zorder=-2)
        ax.axvline(x=i, color='gray', linestyle=':', alpha=0.4, zorder=-2)
    
    for i in range(-2, 6):
        if i != 0:
            ax.text(i, -0.2, str(i), ha='center', va='center', fontsize=12)
            ax.text(-0.2, i, str(i), ha='center', va='center', fontsize=12)
    
    ax.text(-0.2, -0.2, "0", ha='center', va='center', fontsize=12)
    
    # Get positions for this frame
    positions = robot_positions[frame]
    edges = robot_edges[frame]
    
    # Draw the goal
    ax.plot(goal_position[0], goal_position[1], 'r*', markersize=20)
    ax.text(goal_position[0], goal_position[1]+0.25, "GOAL", color='red', 
            ha='center', va='center', fontweight='bold', fontsize=14)
    
    # Clear and recreate the graph
    G.clear()
    for robot_id, position in positions.items():
        G.add_node(robot_id)
    
    # Add edges from this frame
    for edge in edges:
        G.add_edge(edge[0], edge[1])
    
    # Draw the network
    pos = positions
    nx.draw_networkx_nodes(G, pos, ax=ax, 
                          node_size=300, 
                          node_color='blue')
    nx.draw_networkx_labels(G, pos, ax=ax,
                           font_color='white',
                           font_weight='bold')
    nx.draw_networkx_edges(G, pos, ax=ax,
                          width=1.5,
                          edge_color='black')
    
    ax.set_title(f"Star Formation Moving to Goal - Frame {frame}", fontsize=14, fontweight='bold')
    ax.set_xlim([-2.5, 5.5])
    ax.set_ylim([-2.5, 5.5])
    
    return []

# Create animation
ani = animation.FuncAnimation(fig, update, frames=len(robot_positions),
                              init_func=init, blit=True, interval=100)

# Save animation if requested
if SAVE_GIF:
    ani.save('star_formation_cbf.gif', writer='pillow', fps=10)

plt.show()