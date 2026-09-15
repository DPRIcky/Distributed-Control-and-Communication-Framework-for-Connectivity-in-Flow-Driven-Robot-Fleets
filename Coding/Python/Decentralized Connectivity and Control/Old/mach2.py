import matplotlib.pyplot as plt
import networkx as nx
import numpy as np

plt.ion()

COMM_RADIUS = 0.5  # Communication range (tune as needed)
TIME_STEP = 0.1    # Time step for movement
SPEED = 0.05       # Max speed per step

class Robot:
    def __init__(self, id, name, position):
        self.id = id
        self.name = name
        self.position = np.array(position)
        self.velocity = SPEED * (np.random.rand(2) - 0.5)  # Random initial velocity
        self.neighbors = []
        self.comm_done = set()

    def get_pending_neighbors(self):
        return [n for n in self.neighbors if n.id not in self.comm_done]

    def choose_target(self):
        pending = self.get_pending_neighbors()
        return min(pending, key=lambda r: r.id) if pending else None

    def update_position(self):
        # Decentralized connectivity maintenance: don't move too far from neighbors
        for neighbor in self.neighbors:
            dist = np.linalg.norm(self.position + self.velocity * TIME_STEP - neighbor.position)
            if dist > COMM_RADIUS:
                # Adjust velocity to stay within range (project back)
                direction = neighbor.position - self.position
                direction = direction / np.linalg.norm(direction)
                self.velocity = direction * SPEED * 0.9  # Move towards neighbor
        self.position += self.velocity * TIME_STEP

def distance(pos1, pos2):
    return np.linalg.norm(np.array(pos1) - np.array(pos2))

def prune_global_redundant_edges(graph, positions):
    edges_to_remove = []
    for u, v in list(graph.edges()):
        original_distance = distance(positions[u], positions[v])
        graph.remove_edge(u, v)
        if nx.has_path(graph, u, v):
            paths = list(nx.all_simple_paths(graph, u, v, cutoff=3))
            for path in paths:
                max_seg = max(distance(positions[path[i]], positions[path[i+1]]) for i in range(len(path)-1))
                if max_seg < original_distance:
                    edges_to_remove.append((u, v))
                    break
        graph.add_edge(u, v)
    for u, v in edges_to_remove:
        graph.remove_edge(u, v)
    return set(graph.edges())

# Setup
robot_count = 6
positions = {chr(65 + i): np.random.rand(2) for i in range(robot_count)}
robots = [Robot(i, chr(65 + i), positions[chr(65 + i)]) for i in range(robot_count)]
for robot in robots:
    robot.neighbors = [r for r in robots if r.id != robot.id]

G = nx.Graph()
for robot in robots:
    G.add_node(robot.name)
pos = {r.name: r.position for r in robots}

permanent_edges = set()
rounds = 0
max_rounds = 30

while rounds < max_rounds:
    rounds += 1

    # --- Robot Movement and Connectivity Maintenance ---
    for robot in robots:
        robot.update_position()
    pos = {r.name: r.position for r in robots}

    # --- Communication Protocol (unchanged) ---
    print(f"\n--- Round {rounds} ---")
    choices = {}
    temp_edges = []

    for robot in robots:
        target = robot.choose_target()
        if target:
            choices[robot.id] = target.id

    executed = set()
    for sender_id, target_id in choices.items():
        if target_id in choices and choices[target_id] == sender_id:
            sender = robots[sender_id]
            receiver = robots[target_id]

            if (sender.id, receiver.id) not in executed and (receiver.id, sender.id) not in executed:
                executed.add((sender.id, receiver.id))
                sender.comm_done.add(receiver.id)
                receiver.comm_done.add(sender.id)

                edge = (sender.name, receiver.name)
                temp_edges.append(edge)
                
                # CORRECT ORDER: Add edge first, then decide if it should be kept
                G.add_edge(*edge)
                
                # Test pruning with this new edge added
                test_permanent_edges = prune_global_redundant_edges(G, pos)
                
                if edge in test_permanent_edges:
                    # Edge survived pruning - keep it permanently
                    permanent_edges.add(edge)
                    print(f"{edge[0]} <--> {edge[1]} communicated (added - kept after pruning)")
                else:
                    # Edge was pruned - remove it from graph
                    G.remove_edge(*edge)
                    print(f"{edge[0]} <--> {edge[1]} communicated (added but pruned - removed)")

    # Final update of permanent edges after all communications
    permanent_edges = prune_global_redundant_edges(G, pos)

    plt.clf()
    nx.draw(G, pos, with_labels=True, node_color='lightgreen', node_size=1200, font_weight='bold')

    # Draw edges that were attempted but pruned (dashed black)
    for edge in temp_edges:
        if edge not in permanent_edges:
            nx.draw_networkx_edges(G, pos, edgelist=[edge], edge_color='black', style='dashed', width=2)

    # Draw permanent edges (solid red)
    if permanent_edges:
        nx.draw_networkx_edges(G, pos, edgelist=list(permanent_edges), edge_color='red', width=2)

    plt.title(f"Communication Graph - Round {rounds}")
    plt.pause(1)

    if all(len(r.comm_done) == len(r.neighbors) for r in robots):
        print("All communications attempted.")
        permanent_edges = prune_global_redundant_edges(G, pos)
        plt.clf()
        nx.draw(G, pos, with_labels=True, node_color='lightgreen', node_size=1200, font_weight='bold')
        nx.draw_networkx_edges(G, pos, edgelist=list(G.edges()), edge_color='red', width=2)
        plt.title("Final Communication Graph")
        plt.pause(2)
        break

plt.ioff()
plt.show()