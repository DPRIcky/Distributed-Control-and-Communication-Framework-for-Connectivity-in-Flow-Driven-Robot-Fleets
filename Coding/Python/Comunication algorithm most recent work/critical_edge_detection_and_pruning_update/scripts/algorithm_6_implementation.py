"""
Algorithm 6: Distributed Edge Pruning - Implementation with Examples
Demonstrates step-by-step pruning to reach Minimal Bridge Graph (MBG)
"""

import networkx as nx
from collections import defaultdict, deque
from typing import Dict, Set, List, Tuple
import numpy as np


class DistributedEdgePruning:
    """
    Implementation of Algorithm 6: Distributed Edge Pruning
    
    Removes non-critical edges while maintaining connectivity
    using fully distributed message passing (O(D) complexity)
    """
    
    def __init__(self, graph: nx.Graph, critical_edges: Set[Tuple[int, int]]):
        """
        Initialize pruning algorithm.
        
        Parameters:
        -----------
        graph : nx.Graph
            The network to prune
        critical_edges : Set[Tuple]
            Critical edges from Algorithm 3 Bridge Detection
            Format: {(u,v), (v,w), ...} - normalized edges
        """
        self.graph = graph.copy()
        self.n = graph.number_of_nodes()
        self.m = graph.number_of_edges()
        self.critical_edges = critical_edges
        
        # Normalize critical edges
        self.critical_edges_normalized = {
            (min(u, v), max(u, v)) for u, v in critical_edges
        }
        
        # Local state at each node
        self.node_neighbors: Dict[int, Set[int]] = {}
        self.local_results: Dict[int, bool] = {}
        self.iteration = 0
        self.mbg_found = False
        self.history = []
        
        self._initialize_node_states()
    
    def _initialize_node_states(self):
        """Initialize node neighborhood information"""
        for node in self.graph.nodes():
            self.node_neighbors[node] = set(self.graph.neighbors(node))
            self.local_results[node] = None
    
    def _normalize_edge(self, u: int, v: int) -> Tuple[int, int]:
        """Normalize edge representation"""
        return (min(u, v), max(u, v))
    
    def _get_candidate_edges(self) -> Set[Tuple[int, int]]:
        """Get non-critical edges marked for potential removal"""
        candidates = set()
        for u, v in self.graph.edges():
            edge_normalized = self._normalize_edge(u, v)
            if edge_normalized not in self.critical_edges_normalized:
                candidates.add(edge_normalized)
        return candidates
    
    def _distributed_bfs_phase2(self, active_edges: Dict[int, Set[int]]) -> Dict[int, bool]:
        """
        PHASE 2: Distributed BFS with simultaneous testing
        
        Returns local_result[i] = True if node i can reach all other nodes
        using only the active_edges (after removing candidates)
        """
        print("\n    PHASE 2: Distributed BFS (Testing Connectivity)")
        print("    " + "=" * 70)
        
        # Each node independently performs BFS
        node_visited = {}
        
        for initiator in self.graph.nodes():
            # Each node runs BFS to see what it can reach
            visited = {initiator}
            queue = deque([initiator])
            level = 0
            
            print(f"\n    Node {initiator} BFS Trace:")
            print(f"      t=0: queue=[{initiator}], visited={{{initiator}}}")
            
            while queue and level < 20:  # Max iterations for safety
                next_queue = deque()
                level_nodes = []
                
                while queue:
                    current = queue.popleft()
                    
                    # Send to neighbors via active edges only
                    for neighbor in active_edges.get(current, set()):
                        if neighbor not in visited:
                            visited.add(neighbor)
                            next_queue.append(neighbor)
                            level_nodes.append(neighbor)
                
                level += 1
                if level_nodes:
                    print(f"      t={level}: discovered={{{', '.join(map(str, level_nodes))}}}, " 
                          f"visited={visited}")
                queue = next_queue
            
            # Local decision: Can reach all nodes?
            can_reach_all = len(visited) == self.n
            node_visited[initiator] = visited
            self.local_results[initiator] = can_reach_all
            
            status = "[OK] YES - can reach all" if can_reach_all else "[NO] NO - isolated from some"
            print(f"      Result: Visited {len(visited)}/{self.n} nodes -> {status}")
        
        return self.local_results
    
    def _compute_distributed_consensus(self) -> bool:
        """
        Compute global consensus using distributed AND
        
        If any node reports FALSE, global result is FALSE
        If all report TRUE, global result is TRUE
        """
        print("\n    Distributed Consensus Computation:")
        print("    " + "=" * 70)
        
        all_true = True
        for node_id, result in self.local_results.items():
            status = "TRUE [OK]" if result else "FALSE [NO]"
            print(f"      Node {node_id}: local_result = {status}")
            if not result:
                all_true = False
        
        print(f"\n      Global AND = {' AND '.join([str(r) for r in self.local_results.values()])}")
        print(f"                = {all_true}")
        
        return all_true
    
    def run_iteration(self) -> bool:
        """
        Run one iteration of Algorithm 6
        
        Returns: True if MBG found, False if needs another iteration
        """
        self.iteration += 1
        print(f"\n{'='*80}")
        print(f"ITERATION {self.iteration}")
        print(f"{'='*80}\n")
        
        # Get candidate edges
        candidates = self._get_candidate_edges()
        edges_str = ', '.join([f"({u},{v})" for u, v in sorted(candidates)])
        print(f"\nCandidates for removal: {edges_str if candidates else 'NONE'}")
        print(f"Total: {len(candidates)} edges marked for removal")
        
        if not candidates:
            print("\n[OK] No more non-critical edges - MBG FOUND!")
            self.mbg_found = True
            return True
        
        # PHASE 1: Mark candidates (O(1))
        print("\nPHASE 1: Mark Candidate Edges")
        print("=" * 70)
        print(f"  [OK] All nodes locally mark {len(candidates)} edges for removal")
        print(f"  [OK] No messages needed")
        
        # Create active edges (only critical edges) for Phase 2
        active_edges = {}
        for node in self.graph.nodes():
            active_edges[node] = set()
            for neighbor in self.node_neighbors[node]:
                edge = self._normalize_edge(node, neighbor)
                if edge in self.critical_edges_normalized:
                    active_edges[node].add(neighbor)
        
        # PHASE 2: Test removing all candidates (O(D))
        local_results = self._distributed_bfs_phase2(active_edges)
        
        # PHASE 3: Compute consensus (O(1) or O(D) depending on method)
        all_connected = self._compute_distributed_consensus()
        
        # PHASE 3b: Atomic removal decision
        print("\nPHASE 3: Atomic Removal Decision")
        print("=" * 70)
        
        if all_connected:
            print(f"\n  [OK] All nodes can reach all other nodes!")
            print(f"  [OK] Decision: REMOVE all {len(candidates)} candidate edges")
            print(f"  [OK] Graph remains connected after removal")
            
            # All nodes atomically update
            for node in self.graph.nodes():
                self.node_neighbors[node] = active_edges[node]
            
            # Remove edges from actual graph
            for u, v in candidates:
                if self.graph.has_edge(u, v):
                    self.graph.remove_edge(u, v)
                if self.graph.has_edge(v, u):
                    self.graph.remove_edge(v, u)
            
            print(f"  [OK] Edges remaining: {self.graph.number_of_edges()}")
        else:
            print(f"\n  [NO] Some nodes cannot reach all others!")
            print(f"  [NO] Decision: KEEP all edges (unsafe to remove)")
            print(f"  [NO] Proceeding to re-detect bridges")
        
        # PHASE 4: Re-detect bridges (O(D))
        print("\nPHASE 4: Re-detect Bridges")
        print("=" * 70)
        new_critical = self._detect_bridges_algorithm3()
        self.critical_edges_normalized = {
            (min(u, v), max(u, v)) for u, v in new_critical
        }
        
        print(f"  Bridges found: {len(new_critical)}")
        if new_critical:
            edges_str = ', '.join([f"({u},{v})" for u, v in sorted(new_critical)])
            print(f"  Critical edges: {edges_str}")
        
        # PHASE 5: Convergence check
        print("\nPHASE 5: Convergence Check")
        print("=" * 70)
        
        current_non_critical = self._get_candidate_edges()
        
        if not current_non_critical:
            print(f"  [OK] All remaining edges are critical!")
            print(f"  [OK] MINIMAL BRIDGE GRAPH REACHED [OK]")
            self.mbg_found = True
            return True
        else:
            print(f"  [*] Still have {len(current_non_critical)} non-critical edges")
            print(f"  [*] Continue to next iteration")
            return False
    
    def _detect_bridges_algorithm3(self) -> Set[Tuple[int, int]]:
        """Re-run bridge detection (simplified Algorithm 3)"""
        bridges = set()
        
        # For each edge, check if it's a bridge
        for u, v in self.graph.edges():
            self.graph.remove_edge(u, v)
            
            # Check connectivity
            try:
                nx.shortest_path(self.graph, u, v)
                is_bridge = False
            except nx.NetworkXNoPath:
                is_bridge = True
            
            self.graph.add_edge(u, v)
            
            if is_bridge:
                bridges.add((min(u, v), max(u, v)))
        
        return bridges
    
    def prune_to_mbg(self, max_iterations: int = 10) -> tuple:
        """
        Run Algorithm 6 until MBG is found
        
        Returns: (E_pruned, iteration_count, history)
        """
        print("\n" + "=" * 80)
        print("ALGORITHM 6: DISTRIBUTED EDGE PRUNING")
        print("=" * 80)
        print(f"\nInitial State:")
        print(f"  Nodes: {self.n}")
        print(f"  Edges: {self.m}")
        print(f"  Critical edges: {len(self.critical_edges_normalized)}")
        print(f"  Non-critical: {self.m - len(self.critical_edges_normalized)}")
        
        for _ in range(max_iterations):
            if self.run_iteration():
                break
        
        print("\n" + "=" * 80)
        print("FINAL RESULT")
        print("=" * 80)
        print(f"\nMinimal Bridge Graph Statistics:")
        print(f"  Original edges: {self.m}")
        print(f"  Final edges: {self.graph.number_of_edges()}")
        print(f"  Edges removed: {self.m - self.graph.number_of_edges()}")
        print(f"  All edges critical: {self.graph.number_of_edges() == len(self.critical_edges_normalized)}")
        print(f"  Iterations needed: {self.iteration}")
        
        return self.graph, self.iteration, self.history


def create_example_networks():
    """Create 4 test networks with different characteristics"""
    networks = {}
    
    # Network 1: 6-node Mesh (from our example)
    print("\nCreating Network 1: 6-Node Mesh with Redundancy...")
    G1 = nx.Graph()
    G1.add_edges_from([
        (0, 1), (0, 3), (1, 2), (1, 4), 
        (2, 5), (3, 4), (4, 5),  # Critical backbone
        (0, 2), (3, 5), (0, 4)    # Redundant edges (added in Alg 5)
    ])
    
    # Manually set critical edges (from Algorithm 3)
    critical1 = {
        (0, 1), (1, 2), (2, 5), (3, 4), (0, 3)
    }
    networks['6_node_mesh'] = (G1, critical1, "6-Node Mesh with Redundant Edges")
    
    # Network 2: Path Network (High redundancy in cycle)
    print("Creating Network 2: Path to Cycle Network...")
    G2 = nx.Graph()
    G2.add_edges_from([
        (0, 1), (1, 2), (2, 3), (3, 4),  # Linear backbone
        (0, 4),  # Close the loop (from Alg 5)
        (0, 2), (1, 3), (2, 4), (1, 4)   # Multiple redundant edges
    ])
    
    critical2 = {(0, 1), (1, 2), (2, 3), (3, 4), (0, 4)}  # Cycle is minimal
    networks['cycle_network'] = (G2, critical2, "5-Node Cycle with Redundancy")
    
    # Network 3: Two Clusters (Bridge pattern)
    print("Creating Network 3: Two Clusters Connected by Bridge...")
    G3 = nx.Graph()
    # Left cluster (3 nodes)
    G3.add_edges_from([(0, 1), (1, 2), (0, 2)])
    # Right cluster (3 nodes)
    G3.add_edges_from([(3, 4), (4, 5), (3, 5)])
    # Bridges
    G3.add_edges_from([(2, 3), (0, 4), (1, 3), (2, 4)])  # Multiple bridges from Alg 5
    
    critical3 = {(2, 3), (0, 4)}  # Minimum bridges needed
    networks['two_clusters'] = (G3, critical3, "Two Clusters with Bridge Edges")
    
    # Network 4: Dense Small Network
    print("Creating Network 4: Dense 5-Node Network...")
    G4 = nx.complete_graph(5)  # K5 - complete graph
    
    # In complete graph, all edges are non-critical (many paths exist)
    # After improvement, span a tree with few edges
    G4_tree = nx.Graph()
    G4_tree.add_edges_from([(0,1), (1,2), (2,3), (3,4)])
    # Add some redundancy
    G4_tree.add_edges_from([(0,2), (1,3), (2,4), (0,3), (1,4)])
    
    critical4 = {(0,1), (1,2), (2,3), (3,4)}  # Minimum spanning tree
    networks['complete_5'] = (G4_tree, critical4, "Dense 5-Node with Low Redundancy")
    
    return networks


def print_network_info(name: str, graph: nx.Graph, critical_edges: Set):
    """Print network information"""
    non_critical = set()
    for u, v in graph.edges():
        edge = (min(u, v), max(u, v))
        if edge not in critical_edges:
            # Normalize critical edges too
            is_critical = any(edge == (min(cu, cv), max(cu, cv)) 
                            for cu, cv in critical_edges)
            if not is_critical:
                non_critical.add(edge)
    
    print(f"\n{'-' * 80}")
    print(f"NETWORK: {name}")
    print(f"{'-' * 80}")
    print(f"Nodes: {graph.number_of_nodes()}")
    print(f"Total Edges: {graph.number_of_edges()}")
    print(f"Critical Edges: {len(critical_edges)}")
    print(f"Non-Critical Edges: {len(non_critical)}")
    print(f"Density: {2*graph.number_of_edges()/(graph.number_of_nodes()*(graph.number_of_nodes()-1)):.3f}")


def main():
    """Run Algorithm 6 on all example networks"""
    
    networks = create_example_networks()
    
    for name, (graph, critical_edges, description) in networks.items():
        print_network_info(description, graph, critical_edges)
        
        # Run Algorithm 6
        pruner = DistributedEdgePruning(graph, critical_edges)
        pruned_graph, iterations, _ = pruner.prune_to_mbg()
        
        print(f"[OK] Network {name} complete.")


if __name__ == "__main__":
    main()
