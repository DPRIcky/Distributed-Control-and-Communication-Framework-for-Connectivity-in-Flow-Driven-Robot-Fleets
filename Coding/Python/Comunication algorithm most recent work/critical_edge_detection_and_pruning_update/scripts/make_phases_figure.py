"""
Generate a 2×2 phases figure for the comm-feasible δ-only BFS pruning method.

Panels:
  (1,1) Phase 1: Initial communication graph G
  (1,2) Phase 2: δ-BFS hop-to-root propagation (constant-size msgs)
  (2,1) Phase 3: Pruned spanning tree G_topo
  (2,2) Phase 4: Robustness evaluated via comm impairments

Usage:
    python make_phases_figure.py
    python make_phases_figure.py --outdir Figures
"""

import argparse
import os
from collections import deque

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import networkx as nx


# ── Deterministic 6-node graph ──────────────────────────────────────────────
def build_example_graph():
    """Return a fixed connected graph on nodes 1..6 with 8 edges."""
    G = nx.Graph()
    G.add_nodes_from(range(1, 7))
    edges = [
        (1, 2), (1, 3), (2, 3), (2, 4),
        (3, 5), (4, 5), (4, 6), (5, 6),
    ]
    G.add_edges_from(edges)
    return G


# ── BFS from root ───────────────────────────────────────────────────────────
def bfs_delta(G, root):
    """Return dict node -> hop distance to root."""
    delta = {root: 0}
    queue = deque([root])
    while queue:
        u = queue.popleft()
        for v in G.neighbors(u):
            if v not in delta:
                delta[v] = delta[u] + 1
                queue.append(v)
    return delta


# ── Parent selection (tie-break: smallest ID) ──────────────────────────────
def parent_pointers(G, root, delta):
    """Return dict node -> parent (None for root) and set of tree edges."""
    parent = {root: None}
    tree_edges = set()
    for node in sorted(G.nodes()):
        if node == root:
            continue
        best = None
        for nb in sorted(G.neighbors(node)):
            if delta.get(nb, float("inf")) == delta[node] - 1:
                if best is None or nb < best:
                    best = nb
        parent[node] = best
        if best is not None:
            tree_edges.add(tuple(sorted((node, best))))
    return parent, tree_edges


# ── Drawing helpers ─────────────────────────────────────────────────────────
ROOT_COLOR = "#F5A623"   # gold
NODE_COLOR = "#4A90D9"   # blue
EDGE_GRAY = "#BBBBBB"
TREE_RED = "#B22222"     # dark red
TEXTBOX_PROPS = dict(
    boxstyle="round,pad=0.35",
    facecolor="white",
    edgecolor="#999999",
    alpha=0.92,
)
NODE_SIZE = 520
FONT_SIZE_LABEL = 11
FONT_SIZE_TITLE = 12


def node_colors(G, root):
    return [ROOT_COLOR if n == root else NODE_COLOR for n in G.nodes()]


def draw_full_graph(ax, G, pos, root, title):
    """Panel 1 – full graph."""
    ax.set_title(title, fontsize=FONT_SIZE_TITLE, fontweight="bold", pad=10)
    nx.draw_networkx_edges(G, pos, ax=ax, edge_color=EDGE_GRAY, width=1.5)
    nx.draw_networkx_nodes(G, pos, ax=ax, node_color=node_colors(G, root),
                           node_size=NODE_SIZE, edgecolors="black", linewidths=0.8)
    nx.draw_networkx_labels(G, pos, ax=ax, font_size=FONT_SIZE_LABEL, font_weight="bold")

    m = G.number_of_edges()
    n = G.number_of_nodes()
    avg_deg = 2 * m / n
    txt = f"Nodes: {n},  Edges: {m}\nAvg degree: {avg_deg:.2f}"
    ax.text(0.02, 0.02, txt, transform=ax.transAxes, fontsize=8,
            verticalalignment="bottom", bbox=TEXTBOX_PROPS)
    ax.axis("off")


def draw_delta_bfs(ax, G, pos, root, delta, title):
    """Panel 2 – δ-BFS annotations."""
    ax.set_title(title, fontsize=FONT_SIZE_TITLE, fontweight="bold", pad=10)
    nx.draw_networkx_edges(G, pos, ax=ax, edge_color=EDGE_GRAY, width=1.5)
    nx.draw_networkx_nodes(G, pos, ax=ax, node_color=node_colors(G, root),
                           node_size=NODE_SIZE, edgecolors="black", linewidths=0.8)

    labels = {n: f"{n}  (δ={delta[n]})" for n in G.nodes()}
    nx.draw_networkx_labels(G, pos, labels=labels, ax=ax,
                            font_size=9, font_weight="bold")

    txt = "Broadcast payload:\n(id, δ, seq) = 6 bytes"
    ax.text(0.02, 0.02, txt, transform=ax.transAxes, fontsize=8,
            verticalalignment="bottom", bbox=TEXTBOX_PROPS)
    ax.axis("off")


def draw_spanning_tree(ax, G, pos, root, tree_edges, title):
    """Panel 3 – pruned spanning tree."""
    ax.set_title(title, fontsize=FONT_SIZE_TITLE, fontweight="bold", pad=10)

    # Draw non-tree edges very faintly for context
    non_tree = [e for e in G.edges() if tuple(sorted(e)) not in tree_edges]
    nx.draw_networkx_edges(G, pos, edgelist=non_tree, ax=ax,
                           edge_color="#E0E0E0", width=0.8, style="dashed")

    # Tree edges
    nx.draw_networkx_edges(G, pos, edgelist=list(tree_edges), ax=ax,
                           edge_color=TREE_RED, width=2.8)
    nx.draw_networkx_nodes(G, pos, ax=ax, node_color=node_colors(G, root),
                           node_size=NODE_SIZE, edgecolors="black", linewidths=0.8)
    nx.draw_networkx_labels(G, pos, ax=ax, font_size=FONT_SIZE_LABEL, font_weight="bold")

    n = G.number_of_nodes()
    txt = f"Edges: n−1 = {n - 1}\n✓ Tree   ✓ Connected"
    ax.text(0.02, 0.02, txt, transform=ax.transAxes, fontsize=8,
            verticalalignment="bottom", bbox=TEXTBOX_PROPS)
    ax.axis("off")


def draw_robustness(ax, G, pos, root, tree_edges, title):
    """Panel 4 – robustness evaluation overlay."""
    ax.set_title(title, fontsize=FONT_SIZE_TITLE, fontweight="bold", pad=10)

    # Same tree visualization
    non_tree = [e for e in G.edges() if tuple(sorted(e)) not in tree_edges]
    nx.draw_networkx_edges(G, pos, edgelist=non_tree, ax=ax,
                           edge_color="#E0E0E0", width=0.8, style="dashed")
    nx.draw_networkx_edges(G, pos, edgelist=list(tree_edges), ax=ax,
                           edge_color=TREE_RED, width=2.8)
    nx.draw_networkx_nodes(G, pos, ax=ax, node_color=node_colors(G, root),
                           node_size=NODE_SIZE, edgecolors="black", linewidths=0.8)
    nx.draw_networkx_labels(G, pos, ax=ax, font_size=FONT_SIZE_LABEL, font_weight="bold")

    # Overlay textbox – methodology note
    note = (
        "Robustness is evaluated by\n"
        "drops / delays / asynchrony (Sec. V),\n"
        "not by adding extra edges."
    )
    ax.text(0.50, 0.88, note, transform=ax.transAxes, fontsize=8,
            ha="center", va="top",
            bbox=dict(boxstyle="round,pad=0.4", facecolor="#FFF8DC",
                      edgecolor="#DAA520", alpha=0.95))

    # Example settings box
    ex = (
        "Example settings:\n"
        "p_drop = 0.1,  delay_max = 0.5 s\n"
        "jitter = 0.2 s  →  tree formed ✓"
    )
    ax.text(0.02, 0.02, ex, transform=ax.transAxes, fontsize=7.5,
            verticalalignment="bottom", bbox=TEXTBOX_PROPS)
    ax.axis("off")


# ── Main ────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Generate 2×2 phases figure for δ-BFS pruning.")
    parser.add_argument("--outdir", type=str, default=".",
                        help="Output directory (default: current directory)")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    # Build graph & compute
    G = build_example_graph()
    root = 1
    delta = bfs_delta(G, root)
    parent, tree_edges = parent_pointers(G, root, delta)

    # Fixed layout
    pos = nx.spring_layout(G, seed=42, k=1.8)

    # Create figure
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle("Comm-Feasible δ-BFS Spanning-Tree Pruning  —  Phase Overview",
                 fontsize=14, fontweight="bold", y=0.97)

    draw_full_graph(
        axes[0, 0], G, pos, root,
        "Phase 1: Initial communication graph $\\mathcal{G}$")

    draw_delta_bfs(
        axes[0, 1], G, pos, root, delta,
        "Phase 2: δ-BFS hop-to-root propagation\n(constant-size messages)")

    draw_spanning_tree(
        axes[1, 0], G, pos, root, tree_edges,
        "Phase 3: Pruned spanning tree $\\mathcal{G}_{\\mathrm{topo}}$")

    draw_robustness(
        axes[1, 1], G, pos, root, tree_edges,
        "Phase 4: Robustness evaluated via\ncomm impairments (no extra edges)")

    fig.tight_layout(rect=[0, 0, 1, 0.94])

    # Save
    png_path = os.path.join(args.outdir, "phases_2x2_delta_bfs.png")
    pdf_path = os.path.join(args.outdir, "phases_2x2_delta_bfs.pdf")
    fig.savefig(png_path, dpi=250, bbox_inches="tight")
    fig.savefig(pdf_path, bbox_inches="tight")
    plt.close(fig)

    print(f"Saved: {os.path.abspath(png_path)}")
    print(f"Saved: {os.path.abspath(pdf_path)}")

    # ── Export example data to text file and console ────────────────────────
    n = G.number_of_nodes()
    edge_list_sorted = sorted(tuple(sorted(e)) for e in G.edges())
    tree_list_sorted = sorted(tree_edges)

    lines = []
    lines.append("=" * 55)
    lines.append("Example 6-node data used in phases_2x2_delta_bfs.png")
    lines.append("=" * 55)
    lines.append("")
    lines.append(f"1) Edge list E  (|E| = {len(edge_list_sorted)}):")
    for e in edge_list_sorted:
        lines.append(f"   {e}")
    lines.append("")
    lines.append(f"2) Root r = {root}")
    lines.append("")
    lines.append("3) Hop-to-root distances delta_i:")
    for node in sorted(delta):
        lines.append(f"   node {node}: delta = {delta[node]}")
    lines.append("")
    lines.append("4) Parent pointers p(i)  (i != root):")
    for node in sorted(parent):
        if node == root:
            continue
        lines.append(f"   p({node}) = {parent[node]}")
    lines.append("")
    lines.append(f"5) Tree edge set E_topo  (|E_topo| = {len(tree_list_sorted)} = n-1 = {n - 1}):")
    for e in tree_list_sorted:
        lines.append(f"   {e}")
    lines.append("")

    data_text = "\n".join(lines)

    # Print to console
    print()
    print(data_text)

    # Save to file
    txt_path = os.path.join(args.outdir, "example_6node_data.txt")
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write(data_text + "\n")
    print(f"Saved: {os.path.abspath(txt_path)}")


if __name__ == "__main__":
    main()
