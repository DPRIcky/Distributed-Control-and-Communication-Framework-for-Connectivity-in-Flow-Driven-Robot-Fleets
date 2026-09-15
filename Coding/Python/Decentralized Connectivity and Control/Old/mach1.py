import matplotlib.pyplot as plt
import networkx as nx
import time

plt.ion()

class Robot:
    def __init__(self, id, name):
        self.id = id
        self.name = name
        self.neighbors = []
        self.comm_done = set()

    def get_pending_neighbors(self):
        return [n for n in self.neighbors if n.id not in self.comm_done]

    def choose_target(self):
        pending = self.get_pending_neighbors()
        return min(pending, key=lambda r: r.id) if pending else None

def has_common_neighbor(graph, node1, node2):
    neighbors1 = set(graph.neighbors(node1))
    neighbors2 = set(graph.neighbors(node2))
    return len(neighbors1.intersection(neighbors2)) > 0

# Increase number of robots to 'F' (6 total: A-F)
robot_count = 6
robots = [Robot(i, chr(65 + i)) for i in range(robot_count)]
for robot in robots:
    robot.neighbors = [r for r in robots if r.id != robot.id]

G = nx.Graph()
for robot in robots:
    G.add_node(robot.name)
pos = nx.spring_layout(G, seed=42)

permanent_edges = set()
communication_log = []
rounds = 0
max_rounds = 30

while rounds < max_rounds:
    rounds += 1
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

                if not has_common_neighbor(G, sender.name, receiver.name):
                    G.add_edge(*edge)
                    permanent_edges.add(edge)
                    communication_log.append(edge)
                    print(f"{edge[0]} <--> {edge[1]} communicated (added)")
                else:
                    print(f"{edge[0]} <--> {edge[1]} skipped (common neighbor exists)")

    plt.clf()
    nx.draw(G, pos, with_labels=True, node_color='lightgreen', node_size=1200, font_weight='bold')

    for edge in temp_edges:
        if edge not in permanent_edges:
            nx.draw_networkx_edges(G, pos, edgelist=[edge], edge_color='black', style='dashed', width=2)

    if permanent_edges:
        nx.draw_networkx_edges(G, pos, edgelist=list(permanent_edges), edge_color='red', width=2)

    plt.title(f"Communication Graph - Round {rounds}")
    plt.pause(1)

    for edge in temp_edges:
        if edge not in permanent_edges and G.has_edge(*edge):
            G.remove_edge(*edge)

    if all(len(r.comm_done) == len(r.neighbors) for r in robots):
        print("All communications attempted.")
        for edge in temp_edges:
            if edge not in permanent_edges and G.has_edge(*edge):
                G.remove_edge(*edge)
        plt.clf()
        nx.draw(G, pos, with_labels=True, node_color='lightgreen', node_size=1200, font_weight='bold')
        nx.draw_networkx_edges(G, pos, edgelist=list(permanent_edges), edge_color='red', width=2)
        plt.title("Final Communication Graph")
        plt.pause(2)
        break

plt.ioff()
plt.show()
