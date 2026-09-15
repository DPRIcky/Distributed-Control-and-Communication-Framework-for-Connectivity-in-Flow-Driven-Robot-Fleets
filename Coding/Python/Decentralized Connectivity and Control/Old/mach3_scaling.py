import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import cvxpy as cp

plt.ion()

COMM_RADIUS = 0.25
TIME_STEP = 0.05
SPEED = 0.05
MAX_STEP = SPEED * 1.2  # maximum allowed movement per step

class Robot:
    def __init__(self, id, name, position):
        self.id = id
        self.name = name
        self.position = np.array(position)
        self.velocity = SPEED * (np.random.rand(2) - 0.5)
        self.neighbors = []
        self.comm_done = set()

    def get_pending_neighbors(self):
        return [n for n in self.neighbors if n.id not in self.comm_done]

    def choose_target(self):
        pending = self.get_pending_neighbors()
        return min(pending, key=lambda r: r.id) if pending else None

    def update_position(self, graph):
        angle = np.random.uniform(0, 2 * np.pi)
        v_desired = SPEED * np.array([np.cos(angle), np.sin(angle)])
        v = cp.Variable(2)
        constraints = []

        for neighbor_name in graph.neighbors(self.name):
            neighbor_pos = graph.nodes[neighbor_name]['pos']
            diff = neighbor_pos - self.position
            dist = np.linalg.norm(diff)
            if dist < 1e-4:
                continue
            h = dist ** 2 - (COMM_RADIUS / 1.5) ** 2
            A = -2 * diff.reshape(1, -1)
            b = -(diff @ (neighbor_pos + self.position)) + h
            constraints.append(A @ v <= b)

        # 🚫 Prevent large jumps
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

def prune_global_redundant_edges(G, pos, log_removals=False):
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
                    if log_removals:
                        print(f"{u} <--> {v} communicated (removed)")
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

G = nx.Graph()
for r in robots:
    G.add_node(r.name, pos=r.position)
pos = {r.name: r.position for r in robots}

permanent_edges = set()
rounds = 0
max_rounds = 80

while rounds < max_rounds:
    rounds += 1

    for r in robots:
        r.update_position(G)
    pos = {r.name: r.position for r in robots}
    for name in pos:
        G.nodes[name]['pos'] = pos[name]

    print(f"\n--- Round {rounds} ---")
    choices = {}
    temp_edges = []

    for r in robots:
        tgt = r.choose_target()
        if tgt:
            choices[r.id] = tgt.id

    executed = set()
    for i, j in choices.items():
        if j in choices and choices[j] == i:
            r1, r2 = robots[i], robots[j]
            if (i, j) not in executed and (j, i) not in executed:
                executed.add((i, j))
                r1.comm_done.add(j)
                r2.comm_done.add(i)
                edge = (r1.name, r2.name)
                temp_edges.append(edge)
                
                # CORRECT ORDER: Add edge first, then decide if it should be kept
                G.add_edge(*edge)
                
                # Test pruning with this new edge added
                test_permanent_edges = prune_global_redundant_edges(G, pos, log_removals=False)
                
                if edge in test_permanent_edges:
                    # Edge survived pruning - keep it permanently
                    permanent_edges.add(edge)
                    print(f"{edge[0]} <--> {edge[1]} communicated (added - kept after pruning)")
                else:
                    # Edge was pruned - remove it from graph
                    G.remove_edge(*edge)
                    print(f"{edge[0]} <--> {edge[1]} communicated (added but pruned - removed)")

    # Final update of permanent edges after all communications
    permanent_edges = prune_global_redundant_edges(G, pos, log_removals=True)

    plt.clf()
    nx.draw(G, pos, with_labels=True, node_color='lightgreen',
            node_size=40, font_size=5, font_weight='bold')

    # Draw edges that were attempted but pruned (dashed black)
    for edge in temp_edges:
        if edge not in permanent_edges:
            nx.draw_networkx_edges(G, pos, edgelist=[edge],
                                   edge_color='black', style='dashed', width=1)

    # Draw permanent edges (solid red)
    if permanent_edges:
        nx.draw_networkx_edges(G, pos, edgelist=list(permanent_edges),
                               edge_color='red', width=1.5)

    plt.title(f"Communication Graph - Round {rounds}")
    plt.pause(0.3)

    # No need to remove edges here - already handled during communication

    if all(len(r.comm_done) == len(r.neighbors) for r in robots):
        print("All communications attempted.")
        permanent_edges = prune_global_redundant_edges(G, pos, log_removals=True)
        plt.clf()
        nx.draw(G, pos, with_labels=True, node_color='lightgreen',
                node_size=40, font_size=5, font_weight='bold')
        nx.draw_networkx_edges(G, pos, edgelist=list(G.edges()), edge_color='red', width=1.5)
        plt.title("Final Communication Graph")
        plt.pause(3)
        break

plt.ioff()
plt.show()