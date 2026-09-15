import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import cvxpy as cp
from matplotlib.cm import get_cmap

plt.ion()

COMM_RADIUS = 0.25
TIME_STEP = 0.05
SPEED = 0.05
MAX_STEP = SPEED * 1.2

blues = get_cmap('Blues')
oranges = get_cmap('Oranges')

class Robot:
    def __init__(self, id, name, position):
        self.id = id
        self.name = name
        self.position = np.array(position)
        self.velocity = np.zeros(2)
        self.neighbors = []
        self.comm_done = set()
        self.group = 0
        self.relay = False

    def get_pending_neighbors(self):
        return [n for n in self.neighbors if n.id not in self.comm_done]

    def choose_target(self):
        pending = self.get_pending_neighbors()
        return min(pending, key=lambda r: r.id) if pending else None

    def update_position(self, G, goal):
        goal_vec = goal - self.position
        norm = np.linalg.norm(goal_vec)
        if norm < 1e-6:
            v_desired = np.zeros(2)
        else:
            v_desired = SPEED * goal_vec / norm

        if self.relay:
            v_desired *= 0.2  # Reduce motion for relay robots

        v = cp.Variable(2)
        constraints = []

        for neighbor_name in G.neighbors(self.name):
            neighbor_pos = G.nodes[neighbor_name]['pos']
            diff = neighbor_pos - self.position
            dist = np.linalg.norm(diff)
            if dist < 1e-4:
                continue
            h = dist ** 2 - (COMM_RADIUS / 1.5) ** 2
            A = -2 * diff.reshape(1, -1)
            b = -(diff @ (neighbor_pos + self.position)) + h
            constraints.append(A @ v <= b)

        constraints.append(cp.norm(v, 2) <= MAX_STEP / TIME_STEP)

        prob = cp.Problem(cp.Minimize(cp.sum_squares(v - v_desired)), constraints)
        try:
            prob.solve()
            self.velocity = v.value if v.value is not None else np.zeros(2)
        except:
            self.velocity = np.zeros(2)

        self.position += self.velocity * TIME_STEP


def distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

def prune_global_redundant_edges(G, pos):
    edges_to_remove = []
    for u, v in list(G.edges()):
        d_uv = distance(pos[u], pos[v])
        G.remove_edge(u, v)
        if nx.has_path(G, u, v):
            paths = list(nx.all_simple_paths(G, u, v, cutoff=3))
            for path in paths:
                max_seg = max(distance(pos[path[i]], pos[path[i+1]]) for i in range(len(path)-1))
                if max_seg < d_uv:
                    edges_to_remove.append((u, v))
                    break
        G.add_edge(u, v)
    for u, v in edges_to_remove:
        G.remove_edge(u, v)
    return set(G.edges())

# Setup
robot_count = 40
positions = {str(i+1): np.random.rand(2) for i in range(robot_count)}
robots = [Robot(i, str(i+1), positions[str(i+1)]) for i in range(robot_count)]
for r in robots:
    r.neighbors = [n for n in robots if n.id != r.id]

goal_positions = [np.array([0.2, 0.2]), np.array([0.8, 0.8])]

G = nx.Graph()
for r in robots:
    G.add_node(r.name, pos=r.position)

permanent_edges = set()
rounds = 0
max_rounds = 100

while rounds < max_rounds:
    rounds += 1

    # Assign closest goal
    for r in robots:
        dists = [np.linalg.norm(r.position - g) for g in goal_positions]
        r.group = int(np.argmin(dists))

    pos = {r.name: r.position for r in robots}
    for name in pos:
        G.nodes[name]['pos'] = pos[name]

    # Choose anchor robots and relay path
    group0 = [r for r in robots if r.group == 0]
    group1 = [r for r in robots if r.group == 1]
    anchor0 = min(group0, key=lambda r: np.linalg.norm(r.position - goal_positions[0]))
    anchor1 = min(group1, key=lambda r: np.linalg.norm(r.position - goal_positions[1]))
    try:
        relay_path = nx.shortest_path(G, source=anchor0.name, target=anchor1.name)
    except:
        relay_path = []

    # Mark relay status
    for r in robots:
        r.relay = (r.name in relay_path)

    for r in robots:
        r.update_position(G, goal_positions[r.group])

    pos = {r.name: r.position for r in robots}
    for name in pos:
        G.nodes[name]['pos'] = pos[name]

    robot_colors = []
    for i, r in enumerate(robots):
        norm_idx = i / (robot_count - 1)
        base = blues(0.6 + 0.4 * norm_idx) if r.group == 0 else oranges(0.6 + 0.4 * norm_idx)
        robot_colors.append('black' if r.relay else base)

    choices, temp_edges = {}, []
    for r in robots:
        tgt = r.choose_target()
        if tgt:
            choices[r.id] = tgt.id

    executed = set()
    for i, j in choices.items():
        if j in choices and choices[j] == i and (i, j) not in executed:
            r1, r2 = robots[i], robots[j]
            executed.add((i, j))
            r1.comm_done.add(j)
            r2.comm_done.add(i)
            edge = (r1.name, r2.name)
            temp_edges.append(edge)
            G.add_edge(*edge)
            permanent_edges.add(edge)

    permanent_edges = prune_global_redundant_edges(G, pos)

    plt.clf()
    nx.draw(G, pos, with_labels=False, node_color=robot_colors, node_size=60)
    nx.draw_networkx_edges(G, pos, edgelist=list(permanent_edges), edge_color='red', width=1.5)
    for g in goal_positions:
        plt.plot(g[0], g[1], 'yo', markersize=8)
    plt.title(f"Relay Path Connectivity - Round {rounds}")
    plt.pause(0.3)

    for edge in temp_edges:
        if edge not in permanent_edges and G.has_edge(*edge):
            G.remove_edge(*edge)

    if all(len(r.comm_done) == len(r.neighbors) for r in robots):
        break

plt.ioff()
plt.show()
