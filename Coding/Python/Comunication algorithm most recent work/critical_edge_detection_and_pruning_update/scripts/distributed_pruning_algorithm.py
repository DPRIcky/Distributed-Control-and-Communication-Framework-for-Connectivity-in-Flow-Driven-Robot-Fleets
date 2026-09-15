"""
Distributed Spanning-Tree Pruning Algorithm (Comm-Feasible Refactor)
====================================================================

This is a communication-feasible implementation that uses ONLY scalar δ (hop distance
to root) propagation. No O(n) distance vectors are broadcast.

Protocol:
- Phase 1: δ-BFS - Each node computes its hop distance to the root via distributed
           Bellman-Ford. Messages contain only (sender_id, delta, seq) = O(1) payload.
- Phase 3: Spanning-tree construction using parent pointers built during δ-BFS.

REMOVED (Comm-Infeasible):
- Phase 2 (Connectivity Assurance): Required O(n) distance vectors to detect components.
- Phase 4 (Robustness Enhancement): Required all-pairs distances for alternate path checks.

These phases would require redesign under O(1) communication constraints.
See COMM_FEASIBLE_PRUNING.md for details.

Comm Simulation Features:
- Configurable packet drop probability (p_drop)
- Configurable random delay (delay_max)
- Asynchronous broadcast scheduling with jitter
- TX-attempted vs RX-delivered bandwidth accounting

==============================================================================
HOLOOCEAN / REAL SIMULATOR INTEGRATION NOTES
==============================================================================
This reference implementation is designed to map cleanly into a real multi-agent
simulator (e.g., HoloOcean). The integration looks like:

1. AGENT BROADCAST:
   - Each agent periodically broadcasts (agent_id, delta_to_root, seq_number).
   - Payload is O(1) = 6 bytes, fits in any realistic radio frame.
   - Broadcast period = t_broadcast (e.g., 1 Hz for underwater acoustic).
   - Optional jitter prevents synchronization collisions.

2. NETWORK LAYER:
   - Simulator's network model applies:
     * Packet drop with probability p_drop (lossy channel).
     * Random propagation delay ~ Uniform(0, delay_max).
   - On successful delivery, call receiver's on_receive callback.

3. EPOCH STRUCTURE:
   - One "topology update epoch" = t_prune seconds of δ-BFS.
   - After epoch: each agent keeps edge to its parent => pruned topology.
   - Pruned topology passed to CLF-CBF controller for formation control.
   - Next epoch can run periodically or on topology change detection.

4. WHAT NOT TO DO:
   - Never broadcast O(n) vectors (distance_vector, component lists, etc.).
   - Never require global synchronization barriers.
   - Never assume reliable delivery - algorithm must tolerate drops.

This file is a reference/simulation; actual integration requires replacing the
message queue with the simulator's network API.
==============================================================================
"""

import networkx as nx
from typing import Dict, Set, Tuple, List, Optional
from dataclasses import dataclass, field
import math
import heapq
import random
import json
import csv
import os
from pathlib import Path
from datetime import datetime
import platform
import sys


# =============================================================================
# Communication Constants (payload size assumptions, excluding transport overhead)
# =============================================================================
BYTES_SENDER_ID = 2   # uint16
BYTES_DELTA = 2       # uint16 (could be uint8, but uint16 for safety)
BYTES_SEQ = 2         # uint16
PAYLOAD_BYTES = BYTES_SENDER_ID + BYTES_DELTA + BYTES_SEQ  # = 6 bytes per message


# =============================================================================
# Message Dataclass
# =============================================================================
@dataclass(order=True)
class Message:
    """A message in the simulation's delivery queue."""
    deliver_time: float
    sender: int = field(compare=False)
    receiver: int = field(compare=False)
    delta: int = field(compare=False)
    seq: int = field(compare=False)
    send_time: float = field(compare=False)


# =============================================================================
# NodeState (Simplified - No O(n) vectors)
# =============================================================================
@dataclass
class NodeState:
    """Per-node state during algorithm execution.
    
    REMOVED from original:
    - distance_vector: Dict[target_id -> distance]  # Was O(n) per node
    - connected_component: Set[int]                 # Required distance_vector
    - rewire_candidates, best_candidate             # Required all-pairs distances
    """
    node_id: int
    neighbors: Set[int] = field(default_factory=set)
    
    # δ-BFS state (scalar, not vector!)
    delta_to_root: float = float('inf')  # Hop distance to root (δ_i)
    parent: Optional[int] = None         # Parent in spanning tree
    seq: int = 0                         # Sequence number for broadcasts
    
    # For duplicate/stale message suppression
    last_seq_from_neighbor: Dict[int, int] = field(default_factory=dict)
    
    # Timing for convergence detection
    last_delta_update_time: float = 0.0
    
    # Spanning tree edges (computed after convergence)
    kept_neighbors: Set[int] = field(default_factory=set)


# =============================================================================
# DistributedPruningAlgorithm (Comm-Feasible Refactor)
# =============================================================================
class DistributedPruningAlgorithm:
    """
    Distributed spanning-tree pruning using δ-only BFS.
    
    This implementation is communication-feasible:
    - Messages are O(1) size: (sender_id, delta, seq) = 6 bytes
    - No O(n) distance vectors are ever broadcast
    - Supports async operation with packet drops and delays
    
    Removed Phases (would require O(n) comm):
    - Phase 2: Connectivity assurance (needs distance vectors)
    - Phase 4: Robustness enhancement (needs all-pairs distances)
    """
    
    def __init__(
        self,
        graph: nx.Graph,
        root: int = 1,
        p_drop: float = 0.0,
        delay_max: float = 0.0,
        t_broadcast: float = 1.0,
        jitter: float = 0.0,
        seed: Optional[int] = None
    ):
        """
        Initialize algorithm on a graph.
        
        Args:
            graph: NetworkX undirected graph
            root: Root node for spanning tree
            p_drop: Probability of dropping a message (0.0 to 1.0)
            delay_max: Maximum random delay for message delivery (seconds)
            t_broadcast: Period between node broadcasts (seconds)
            jitter: Max jitter added to broadcast period (+/- jitter seconds)
            seed: Random seed for reproducibility
        """
        if seed is not None:
            random.seed(seed)
        
        self.graph = graph.copy()
        self.root = root
        self.n = len(graph)
        self.nodes = sorted(graph.nodes())
        
        # Comm simulation parameters
        self.p_drop = p_drop
        self.delay_max = delay_max
        self.t_broadcast = t_broadcast
        self.jitter = jitter
        
        # Per-node state
        self.node_state: Dict[int, NodeState] = {}
        for node in self.nodes:
            self.node_state[node] = NodeState(node_id=node)
            self.node_state[node].neighbors = set(self.graph.neighbors(node))
        
        # Initialize root with δ = 0
        if self.root in self.node_state:
            self.node_state[self.root].delta_to_root = 0
        
        # Message queue (priority queue by deliver_time)
        self.message_queue: List[Message] = []
        
        # Comm accounting (TX = attempted transmissions, RX = successful deliveries)
        self.messages_sent_total = 0         # TX attempts (before drop check)
        self.messages_dropped_total = 0      # Dropped by channel
        self.messages_delivered_total = 0    # RX successful deliveries
        self.payload_bytes_sent_total = 0    # TX bytes (attempted)
        self.payload_bytes_delivered_total = 0  # RX bytes (delivered)
        
        # Broadcast scheduling (per-node next broadcast time)
        self.next_broadcast_time: Dict[int, float] = {}
        
        # Algorithm state
        self.final_edges: Set[Tuple[int, int]] = set()
        self.run_duration: float = 0.0
    
    # =========================================================================
    # Message Queue Simulation
    # =========================================================================
    
    def _send_message(self, sender: int, receiver: int, delta: int, seq: int, t_now: float):
        """
        Send a message from sender to receiver with drop/delay modeling.
        
        Increments comm counters regardless of drop (counts attempted sends).
        """
        self.messages_sent_total += 1
        self.payload_bytes_sent_total += PAYLOAD_BYTES
        
        # Drop check
        if self.p_drop > 0 and random.random() < self.p_drop:
            self.messages_dropped_total += 1
            return
        
        # Compute delivery time with random delay
        if self.delay_max > 0:
            delay = random.uniform(0, self.delay_max)
        else:
            delay = 0.0
        
        deliver_time = t_now + delay
        
        msg = Message(
            deliver_time=deliver_time,
            sender=sender,
            receiver=receiver,
            delta=delta,
            seq=seq,
            send_time=t_now
        )
        heapq.heappush(self.message_queue, msg)
    
    def _deliver_messages(self, t_now: float) -> int:
        """
        Deliver all messages with deliver_time <= t_now.
        
        Returns number of messages delivered.
        Increments RX counters (messages_delivered_total, payload_bytes_delivered_total).
        """
        delivered_count = 0
        
        while self.message_queue and self.message_queue[0].deliver_time <= t_now:
            msg = heapq.heappop(self.message_queue)
            self._on_receive(msg, t_now)
            delivered_count += 1
            self.messages_delivered_total += 1
            self.payload_bytes_delivered_total += PAYLOAD_BYTES
        
        return delivered_count
    
    def _on_receive(self, msg: Message, t_now: float):
        """
        Process a received message at the receiver node.
        
        δ-BFS update rule:
            candidate = δ_j + 1
            if candidate < δ_i:
                δ_i = candidate
                parent_i = j
            elif candidate == δ_i and j < parent_i:
                parent_i = j  # Tie-break: prefer lower-ID parent
        """
        receiver_state = self.node_state.get(msg.receiver)
        if receiver_state is None:
            return
        
        # Sequence number check (optional: suppress stale messages)
        last_seq = receiver_state.last_seq_from_neighbor.get(msg.sender, -1)
        if msg.seq <= last_seq:
            # Stale or duplicate message, ignore
            return
        receiver_state.last_seq_from_neighbor[msg.sender] = msg.seq
        
        # δ-BFS update
        candidate_delta = msg.delta + 1
        current_delta = receiver_state.delta_to_root
        
        updated = False
        if candidate_delta < current_delta:
            receiver_state.delta_to_root = candidate_delta
            receiver_state.parent = msg.sender
            updated = True
        elif candidate_delta == current_delta:
            # Tie-break: prefer lower-ID parent for determinism
            if receiver_state.parent is None or msg.sender < receiver_state.parent:
                receiver_state.parent = msg.sender
                updated = True
        
        if updated:
            receiver_state.last_delta_update_time = t_now
    
    # =========================================================================
    # Broadcast Scheduling
    # =========================================================================
    
    def _init_broadcast_schedule(self):
        """Initialize per-node broadcast times with jitter."""
        for node in self.nodes:
            # Stagger initial broadcasts with jitter to avoid synchronization
            initial_offset = random.uniform(0, self.t_broadcast)
            self.next_broadcast_time[node] = initial_offset
    
    def _broadcast(self, node: int, t_now: float):
        """
        Broadcast (node_id, delta, seq) to all neighbors.
        """
        state = self.node_state[node]
        state.seq += 1
        
        delta_value = state.delta_to_root
        if delta_value == float('inf'):
            # Don't broadcast infinity (no useful information)
            return
        
        # Convert to int for message (inf check above ensures this is safe)
        delta_int = int(delta_value)
        
        for neighbor in state.neighbors:
            self._send_message(
                sender=node,
                receiver=neighbor,
                delta=delta_int,
                seq=state.seq,
                t_now=t_now
            )
    
    def _schedule_next_broadcast(self, node: int, t_now: float):
        """Schedule the next broadcast for a node with jitter."""
        jitter_val = 0.0
        if self.jitter > 0:
            jitter_val = random.uniform(-self.jitter, self.jitter)
        self.next_broadcast_time[node] = t_now + self.t_broadcast + jitter_val
    
    # =========================================================================
    # Phase 1: δ-BFS (Scalar Distance to Root)
    # =========================================================================
    
    def run_delta_bfs(
        self,
        t_prune: float,
        dt: float = 0.1,
        t_stable: Optional[float] = None,
        verbose: bool = False
    ) -> float:
        """
        Run δ-BFS for up to t_prune seconds.
        
        Args:
            t_prune: Maximum time to run (seconds)
            dt: Time step for simulation (seconds)
            t_stable: If no δ updates for this duration, stop early (optional)
            verbose: Print progress messages
        
        Returns:
            Actual duration run (seconds)
        """
        self._init_broadcast_schedule()
        
        # Track last update time across all nodes for stable stopping
        last_global_update = 0.0
        
        t = 0.0
        while t < t_prune:
            # Check each node for broadcast
            for node in self.nodes:
                if t >= self.next_broadcast_time[node]:
                    self._broadcast(node, t)
                    self._schedule_next_broadcast(node, t)
            
            # Deliver messages
            self._deliver_messages(t)
            
            # Check for global stability (optional early stopping)
            if t_stable is not None:
                max_update_time = max(
                    self.node_state[node].last_delta_update_time
                    for node in self.nodes
                )
                if max_update_time > last_global_update:
                    last_global_update = max_update_time
                
                if t - last_global_update >= t_stable and t > 0:
                    if verbose:
                        print(f"δ-BFS converged at t={t:.2f}s (stable for {t_stable}s)")
                    break
            
            t += dt
        
        self.run_duration = t
        
        # Deliver any remaining messages
        self._deliver_messages(t + self.delay_max + 1.0)
        
        if verbose:
            print(f"δ-BFS complete: t={t:.2f}s, messages_sent={self.messages_sent_total}")
        
        return t
    
    # =========================================================================
    # Phase 2: Connectivity Assurance (REMOVED - Requires O(n) comm)
    # =========================================================================
    
    def run_phase2(self):
        """
        REMOVED: Phase 2 - Connectivity Assurance
        
        The original implementation required O(n) distance vectors to detect
        disconnected components. This is not feasible under O(1) communication
        constraints.
        
        Requires redesign under O(1) comm; intentionally omitted in comm-feasible refactor.
        
        Possible future approaches:
        - Leader election within each component
        - Spanning forest detection via parent pointer analysis
        - Timeout-based connectivity inference
        """
        raise NotImplementedError(
            "Phase 2 (Connectivity Assurance) requires O(n) distance vectors. "
            "Requires redesign under O(1) comm; intentionally omitted in comm-feasible refactor."
        )
    
    # =========================================================================
    # Phase 3: Spanning Tree Construction (from δ-BFS parents)
    # =========================================================================
    
    def build_spanning_tree(self, verbose: bool = False) -> Set[Tuple[int, int]]:
        """
        Build spanning tree from parent pointers computed during δ-BFS.
        
        For each node i != root: keep edge (i, parent_i) if parent_i exists.
        
        Returns:
            Set of kept edges as undirected tuples (min_id, max_id)
        """
        kept_edges: Set[Tuple[int, int]] = set()
        
        for node in self.nodes:
            if node == self.root:
                continue
            
            state = self.node_state[node]
            if state.parent is not None:
                # Store as undirected edge (min, max)
                edge = tuple(sorted([node, state.parent]))
                kept_edges.add(edge)
                state.kept_neighbors.add(state.parent)
                self.node_state[state.parent].kept_neighbors.add(node)
        
        self.final_edges = kept_edges
        
        if verbose:
            print(f"Spanning tree built: {len(kept_edges)} edges")
        
        return kept_edges
    
    # =========================================================================
    # Epoch-Based Execution (for HoloOcean integration)
    # =========================================================================
    
    def reset_epoch_state(self, reset_deltas: bool = True):
        """
        Reset state for a new epoch so run_epoch() can be called repeatedly.
        
        Args:
            reset_deltas: If True, reset delta_to_root and parent pointers.
                         If False, keep previous δ values (warm start).
        """
        # Clear message queue
        self.message_queue.clear()
        
        # Reset comm counters
        self.messages_sent_total = 0
        self.messages_delivered_total = 0
        self.messages_dropped_total = 0
        self.payload_bytes_sent_total = 0
        self.payload_bytes_delivered_total = 0
        
        # Reset broadcast schedule
        self.next_broadcast_time.clear()
        
        # Reset per-node state
        for node in self.nodes:
            state = self.node_state[node]
            state.kept_neighbors.clear()
            state.last_seq_from_neighbor.clear()
            state.last_delta_update_time = 0.0
            state.seq = 0  # Reset sequence numbers
            
            if reset_deltas:
                if node == self.root:
                    state.delta_to_root = 0
                    state.parent = None
                else:
                    state.delta_to_root = float('inf')
                    state.parent = None
        
        # Reset algorithm outputs
        self.final_edges = set()
        self.run_duration = 0.0
    
    def run_epoch(
        self,
        t_prune: float = 10.0,
        dt: float = 0.1,
        t_stable: Optional[float] = None,
        verbose: bool = False,
        reset_deltas: bool = True
    ) -> Tuple[Set[Tuple[int, int]], float, Dict]:
        """
        Run one topology update epoch: δ-BFS + spanning tree construction.
        
        This method is designed for integration with simulators like HoloOcean,
        where topology updates happen periodically.
        
        Automatically resets epoch state before running, so this method
        can be called repeatedly on the same object.
        
        Args:
            t_prune: Maximum wall-time for this epoch (seconds)
            dt: Simulation time step (seconds)
            t_stable: Early stopping if no δ updates for this duration
            verbose: Print progress messages
            reset_deltas: If True, reset δ values (cold start each epoch).
                         If False, keep previous δ values (warm start).
        
        Returns:
            Tuple of:
            - kept_edges: Set of edges forming the spanning tree
            - run_duration: Actual time the epoch ran (seconds)
            - comm_stats: Dict with TX/RX comm statistics for this epoch
        """
        # Reset state for this epoch
        self.reset_epoch_state(reset_deltas=reset_deltas)
        
        # Run δ-BFS
        duration = self.run_delta_bfs(
            t_prune=t_prune,
            dt=dt,
            t_stable=t_stable,
            verbose=verbose
        )
        
        # Build spanning tree from parent pointers
        kept_edges = self.build_spanning_tree(verbose=verbose)
        
        # Collect comm stats
        comm_stats = self.summary_comm_stats(duration_seconds=duration)
        
        return kept_edges, duration, comm_stats
    
    # =========================================================================
    # Phase 4: Robustness Enhancement (REMOVED - Requires O(n) comm)
    # =========================================================================
    
    def run_phase4(self):
        """
        REMOVED: Phase 4 - Robustness Enhancement
        
        The original implementation required all-pairs distances to:
        - Compute delta values for alternate path validation
        - Check equidistant conditions across all targets
        
        This requires O(n) distance vectors per node, which is not feasible
        under O(1) communication constraints.
        
        Requires redesign under O(1) comm; intentionally omitted in comm-feasible refactor.
        
        Possible future approaches:
        - Local 2-hop neighborhood analysis (limited scope)
        - Probabilistic robustness sampling
        - Separate robustness protocol with bounded messages
        """
        raise NotImplementedError(
            "Phase 4 (Robustness Enhancement) requires all-pairs distances. "
            "Requires redesign under O(1) comm; intentionally omitted in comm-feasible refactor."
        )
    
    # =========================================================================
    # Main Execution (Comm-Feasible Pipeline)
    # =========================================================================
    
    def run(
        self,
        t_prune: float = 10.0,
        dt: float = 0.1,
        t_stable: Optional[float] = None,
        verbose: bool = True
    ) -> Set[Tuple[int, int]]:
        """
        Run the comm-feasible pruning algorithm.
        
        This executes:
        1. δ-BFS: Compute hop distances to root (O(1) messages)
        2. Spanning tree construction from parent pointers
        
        Args:
            t_prune: Maximum time for δ-BFS (seconds)
            dt: Simulation time step (seconds)
            t_stable: Early stopping if no updates for this duration
            verbose: Print progress messages
        
        Returns:
            Set of kept edges forming a spanning tree
        """
        if verbose:
            print("\n" + "=" * 60)
            print("Distributed Spanning-Tree Pruning (Comm-Feasible)")
            print("=" * 60)
            print(f"Nodes: {self.n}, Original edges: {len(self.graph.edges())}")
            print(f"Root: {self.root}")
            print(f"Comm params: p_drop={self.p_drop}, delay_max={self.delay_max}")
            print()
        
        # Phase 1: δ-BFS
        self.run_delta_bfs(t_prune=t_prune, dt=dt, t_stable=t_stable, verbose=verbose)
        
        # Phase 3: Build spanning tree
        kept_edges = self.build_spanning_tree(verbose=verbose)
        
        if verbose:
            print()
            print("=" * 60)
            print("Algorithm Complete")
            print("=" * 60)
            self.print_results()
        
        return kept_edges
    
    # Legacy method for backward compatibility
    def run_full_pipeline(self):
        """
        Legacy method - runs the comm-feasible pipeline.
        
        Note: Phase 2 and Phase 4 are no longer available as they require
        O(n) communication. Use run() for the comm-feasible protocol.
        """
        return self.run(verbose=True)
    
    # =========================================================================
    # Comm Stats & Accounting
    # =========================================================================
    
    def get_comm_stats(self) -> Dict:
        """Return communication statistics (TX = attempted, RX = delivered)."""
        return {
            'messages_sent_total': self.messages_sent_total,        # TX attempts
            'messages_dropped_total': self.messages_dropped_total,  # Lost in channel
            'messages_delivered_total': self.messages_delivered_total,  # RX success
            'payload_bytes_sent_total': self.payload_bytes_sent_total,  # TX bytes
            'payload_bytes_delivered_total': self.payload_bytes_delivered_total,  # RX bytes
            'payload_bytes_per_message': PAYLOAD_BYTES,
            'drop_rate_actual': (
                self.messages_dropped_total / self.messages_sent_total
                if self.messages_sent_total > 0 else 0.0
            ),
        }
    
    def summary_comm_stats(self, duration_seconds: Optional[float] = None) -> Dict:
        """
        Return comm stats summary with per-second rates (TX and RX).
        
        Args:
            duration_seconds: Duration to use for rate calculation.
                             Defaults to self.run_duration if available.
        
        Returns:
            Dict with TX/RX messages/sec and bytes/sec
        """
        if duration_seconds is None:
            duration_seconds = self.run_duration if self.run_duration > 0 else 1.0
        
        stats = self.get_comm_stats()
        
        return {
            **stats,
            'duration_seconds': duration_seconds,
            # TX rates (attempted)
            'tx_messages_per_second': stats['messages_sent_total'] / duration_seconds,
            'tx_bytes_per_second': stats['payload_bytes_sent_total'] / duration_seconds,
            # RX rates (delivered)
            'rx_messages_per_second': stats['messages_delivered_total'] / duration_seconds,
            'rx_bytes_per_second': stats['payload_bytes_delivered_total'] / duration_seconds,
            # Legacy aliases
            'messages_per_second': stats['messages_sent_total'] / duration_seconds,
            'bytes_per_second': stats['payload_bytes_sent_total'] / duration_seconds,
        }
    
    # =========================================================================
    # Output & Analysis
    # =========================================================================
    
    def get_pruned_graph(self) -> nx.Graph:
        """Return pruned network topology."""
        pruned = nx.Graph()
        pruned.add_nodes_from(self.nodes)
        pruned.add_edges_from(self.final_edges)
        return pruned
    
    def get_node_deltas(self) -> Dict[int, float]:
        """Return hop distance to root for each node."""
        return {node: self.node_state[node].delta_to_root for node in self.nodes}
    
    def get_parent_map(self) -> Dict[int, Optional[int]]:
        """Return parent pointer for each node."""
        return {node: self.node_state[node].parent for node in self.nodes}
    
    def get_statistics(self) -> Dict:
        """Return algorithm statistics.
        
        Note: reachable_from_root is computed from the actual pruned graph
        connectivity, NOT from counting nodes with finite delta_to_root.
        This is more robust under lossy channels where parent chains may
        be inconsistent.
        """
        pruned = self.get_pruned_graph()
        num_edges = len(self.final_edges)
        
        # Compute reachability from actual pruned graph connectivity
        if self.root not in pruned:
            reachable_nodes = 0
        elif num_edges == 0:
            # Graph has no edges - only root is reachable (itself)
            reachable_nodes = 1 if self.n >= 1 else 0
        else:
            # Use connected component containing root
            reachable_nodes = len(nx.node_connected_component(pruned, self.root))
        
        # Handle connected status for edge cases
        if self.n <= 1:
            is_connected = True
        elif num_edges == 0:
            is_connected = False
        else:
            is_connected = nx.is_connected(pruned)
        
        # A tree has exactly n-1 edges and is connected
        if self.n <= 1:
            is_tree = True
        else:
            is_tree = is_connected and (num_edges == self.n - 1)
        
        return {
            'original_edges': len(self.graph.edges()),
            'final_edges': num_edges,
            'nodes': self.n,
            'reachable_from_root': reachable_nodes,
            'connected': is_connected,
            'is_tree': is_tree,
            'avg_degree_original': 2 * len(self.graph.edges()) / self.n if self.n > 0 else 0,
            'avg_degree_final': 2 * num_edges / self.n if self.n > 0 else 0,
        }
    
    def print_results(self):
        """Print algorithm results with TX/RX breakdown."""
        stats = self.get_statistics()
        comm = self.summary_comm_stats()
        
        print(f"Original edges: {stats['original_edges']}")
        print(f"Final edges: {stats['final_edges']}")
        print(f"Nodes: {stats['nodes']}")
        print(f"Reachable from root: {stats['reachable_from_root']}")
        print(f"Connected: {stats['connected']}")
        print(f"Is tree: {stats['is_tree']}")
        
        if stats['original_edges'] > 0:
            print(f"Pruning ratio: {(1 - stats['final_edges'] / stats['original_edges']) * 100:.1f}%")
        
        print()
        print("Communication Stats (TX=attempted, RX=delivered):")
        print(f"  Messages: TX={comm['messages_sent_total']}, "
              f"RX={comm['messages_delivered_total']}, "
              f"Dropped={comm['messages_dropped_total']}")
        print(f"  Bytes: TX={comm['payload_bytes_sent_total']}, "
              f"RX={comm['payload_bytes_delivered_total']}")
        print(f"  Rates: TX={comm['tx_bytes_per_second']:.1f} B/s, "
              f"RX={comm['rx_bytes_per_second']:.1f} B/s")
        print(f"  Drop rate: {comm['drop_rate_actual']*100:.1f}%")
    
    def print_node_states(self):
        """Print per-node state (for debugging)."""
        print("\nNode States:")
        print("-" * 50)
        for node in self.nodes:
            state = self.node_state[node]
            print(f"Node {node}: δ={state.delta_to_root}, parent={state.parent}")


# =============================================================================
# Demo / Main
# =============================================================================

def create_demo_graph(n: int = 10, edge_prob: float = 0.4, seed: int = 42) -> nx.Graph:
    """Create a random connected graph for demo."""
    random.seed(seed)
    
    # Generate Erdos-Renyi graph
    G = nx.erdos_renyi_graph(n, edge_prob, seed=seed)
    
    # Relabel nodes to start from 1
    mapping = {i: i + 1 for i in range(n)}
    G = nx.relabel_nodes(G, mapping)
    
    # Ensure connected
    if not nx.is_connected(G):
        # Add edges to connect components
        components = list(nx.connected_components(G))
        for i in range(len(components) - 1):
            node_a = min(components[i])
            node_b = min(components[i + 1])
            G.add_edge(node_a, node_b)
    
    return G


# =============================================================================
# Stress-Test Harness for Paper (Reviewer Requirement B)
# =============================================================================

def run_stress_grid(
    n: int = 10,
    edge_prob: float = 0.4,
    trials: int = 10,
    drop_list: List[float] = None,
    delay_list: List[float] = None,
    jitter_list: List[float] = None,
    t_broadcast: float = 1.0,
    t_prune: float = 20.0,
    dt: float = 0.1,
    t_stable: float = 5.0,
    base_seed: int = 1000,
    verbose: bool = False
) -> List[Dict]:
    """
    Run stress-test grid over (p_drop, delay_max, jitter) configurations.
    
    For each configuration, runs `trials` Monte Carlo simulations with different
    random graphs and seeds. Reports success rate, timing, and comm stats.
    
    Args:
        n: Number of nodes in graph
        edge_prob: Edge probability for random graph generation
        trials: Number of Monte Carlo runs per configuration
        drop_list: List of drop probabilities to test
        delay_list: List of max delays to test
        jitter_list: List of jitter values to test
        t_broadcast: Broadcast period for all tests
        t_prune: Maximum epoch duration
        dt: Simulation time step
        t_stable: Stable stopping threshold
        base_seed: Base seed for reproducibility
        verbose: Print progress during runs
    
    Returns:
        List of result dicts, one per configuration
    """
    if drop_list is None:
        drop_list = [0.0, 0.1, 0.3, 0.5]
    if delay_list is None:
        delay_list = [0.0, 0.2, 0.5, 1.0]
    if jitter_list is None:
        jitter_list = [0.0, 0.2]
    
    results = []
    total_configs = len(drop_list) * len(delay_list) * len(jitter_list)
    config_idx = 0
    
    for p_drop in drop_list:
        for delay_max in delay_list:
            for jitter in jitter_list:
                config_idx += 1
                
                # Accumulators for this configuration
                successes = 0
                durations = []
                msg_sent_list = []
                msg_delivered_list = []
                msg_dropped_list = []
                tx_bps_list = []
                rx_bps_list = []
                
                for trial in range(trials):
                    # Generate a new random connected graph
                    graph_seed = base_seed + config_idx * 1000 + trial
                    G = create_demo_graph(n=n, edge_prob=edge_prob, seed=graph_seed)
                    
                    # Run the algorithm
                    alg_seed = base_seed + config_idx * 10000 + trial
                    alg = DistributedPruningAlgorithm(
                        graph=G,
                        root=1,
                        p_drop=p_drop,
                        delay_max=delay_max,
                        t_broadcast=t_broadcast,
                        jitter=jitter,
                        seed=alg_seed
                    )
                    
                    kept_edges, duration, comm = alg.run_epoch(
                        t_prune=t_prune,
                        dt=dt,
                        t_stable=t_stable,
                        verbose=verbose
                    )
                    
                    # Check success
                    stats = alg.get_statistics()
                    success = (
                        stats['reachable_from_root'] == n and
                        stats['connected'] and
                        stats['is_tree']
                    )
                    
                    if success:
                        successes += 1
                    
                    durations.append(duration)
                    msg_sent_list.append(comm['messages_sent_total'])
                    msg_delivered_list.append(comm['messages_delivered_total'])
                    msg_dropped_list.append(comm['messages_dropped_total'])
                    tx_bps_list.append(comm['tx_bytes_per_second'])
                    rx_bps_list.append(comm['rx_bytes_per_second'])
                
                # Compute averages
                avg_duration = sum(durations) / len(durations)
                avg_msg_sent = sum(msg_sent_list) / len(msg_sent_list)
                avg_msg_delivered = sum(msg_delivered_list) / len(msg_delivered_list)
                avg_msg_dropped = sum(msg_dropped_list) / len(msg_dropped_list)
                avg_tx_bps = sum(tx_bps_list) / len(tx_bps_list)
                avg_rx_bps = sum(rx_bps_list) / len(rx_bps_list)
                observed_drop_rate = avg_msg_dropped / avg_msg_sent if avg_msg_sent > 0 else 0.0
                
                result = {
                    'p_drop': p_drop,
                    'delay_max': delay_max,
                    'jitter': jitter,
                    'trials': trials,
                    'success_rate': successes / trials,
                    'avg_duration': avg_duration,
                    'avg_msg_sent': avg_msg_sent,
                    'avg_msg_delivered': avg_msg_delivered,
                    'avg_msg_dropped': avg_msg_dropped,
                    'avg_tx_Bps': avg_tx_bps,
                    'avg_rx_Bps': avg_rx_bps,
                    'observed_drop_rate': observed_drop_rate,
                }
                results.append(result)
    
    return results


def print_stress_grid_results(results: List[Dict], params: Optional[Dict] = None) -> str:
    """Print stress grid results as a formatted table.
    
    Args:
        results: List of result dicts from run_stress_grid()
        params: Optional dict of parameters used for the run
    
    Returns:
        The formatted table as a string (for saving to file)
    """
    lines = []
    
    lines.append("")
    lines.append("=" * 100)
    lines.append("STRESS-TEST RESULTS: δ-BFS Spanning Tree Pruning")
    lines.append("=" * 100)
    
    if params:
        lines.append("")
        lines.append("Parameters:")
        lines.append(f"  n={params.get('n', '?')}, edge_prob={params.get('edge_prob', '?')}, "
                    f"trials={params.get('trials', '?')}")
        lines.append(f"  drop_list={params.get('drop_list', '?')}")
        lines.append(f"  delay_list={params.get('delay_list', '?')}")
        lines.append(f"  jitter_list={params.get('jitter_list', '?')}")
        lines.append(f"  t_broadcast={params.get('t_broadcast', '?')}, "
                    f"t_prune={params.get('t_prune', '?')}, "
                    f"dt={params.get('dt', '?')}, "
                    f"t_stable={params.get('t_stable', '?')}")
        lines.append("")
    
    # Header
    header = (
        f"{'p_drop':>7} {'delay':>6} {'jitter':>6} | "
        f"{'success':>7} {'duration':>8} | "
        f"{'TX_msg':>8} {'RX_msg':>8} | "
        f"{'TX_B/s':>8} {'RX_B/s':>8} | "
        f"{'drop%':>6}"
    )
    lines.append(header)
    lines.append("-" * 100)
    
    for r in results:
        row = (
            f"{r['p_drop']:>7.2f} {r['delay_max']:>6.2f} {r['jitter']:>6.2f} | "
            f"{r['success_rate']*100:>6.1f}% {r['avg_duration']:>7.2f}s | "
            f"{r['avg_msg_sent']:>8.0f} {r['avg_msg_delivered']:>8.0f} | "
            f"{r['avg_tx_Bps']:>8.1f} {r['avg_rx_Bps']:>8.1f} | "
            f"{r['observed_drop_rate']*100:>5.1f}%"
        )
        lines.append(row)
    
    lines.append("=" * 100)
    lines.append(f"Payload per message: {PAYLOAD_BYTES} bytes (O(1), NOT O(n))")
    lines.append("=" * 100)
    
    output = "\n".join(lines)
    print(output)
    return output


def save_stress_grid_results(
    results: List[Dict],
    params: Dict,
    outdir: str = "stress_results",
    tag: Optional[str] = None
) -> str:
    """
    Save stress grid results to files.
    
    Creates:
    - results.json: List of result dicts
    - results.csv: Same results in CSV format
    - summary.txt: Human-readable table with parameters and metadata
    
    Args:
        results: List of result dicts from run_stress_grid()
        params: Dict of parameters used for the run
        outdir: Base output directory
        tag: Run label (defaults to timestamp)
    
    Returns:
        Path to the output folder
    """
    # Generate tag if not provided
    if tag is None:
        tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Create output folder
    output_path = Path(outdir) / tag
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Save results.json
    json_path = output_path / "results.json"
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(results, f, indent=2)
    
    # Save results.csv
    csv_path = output_path / "results.csv"
    if results:
        fieldnames = list(results[0].keys())
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results)
    
    # Save summary.txt (human-readable table + metadata)
    summary_path = output_path / "summary.txt"
    with open(summary_path, 'w', encoding='utf-8') as f:
        # Header info
        f.write("=" * 100 + "\n")
        f.write("STRESS-TEST RESULTS: delta-BFS Spanning Tree Pruning\n")
        f.write("=" * 100 + "\n\n")
        
        # Metadata
        f.write("Run Information:\n")
        f.write(f"  Date/Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"  Python version: {platform.python_version()}\n")
        f.write(f"  Platform: {platform.platform()}\n")
        f.write(f"  Payload bytes per message: {PAYLOAD_BYTES}\n")
        f.write("\n")
        
        # Parameters
        f.write("Parameters:\n")
        f.write(f"  n = {params.get('n', '?')}\n")
        f.write(f"  edge_prob = {params.get('edge_prob', '?')}\n")
        f.write(f"  trials = {params.get('trials', '?')}\n")
        f.write(f"  drop_list = {params.get('drop_list', '?')}\n")
        f.write(f"  delay_list = {params.get('delay_list', '?')}\n")
        f.write(f"  jitter_list = {params.get('jitter_list', '?')}\n")
        f.write(f"  t_broadcast = {params.get('t_broadcast', '?')}\n")
        f.write(f"  t_prune = {params.get('t_prune', '?')}\n")
        f.write(f"  dt = {params.get('dt', '?')}\n")
        f.write(f"  t_stable = {params.get('t_stable', '?')}\n")
        f.write(f"  base_seed = {params.get('base_seed', '?')}\n")
        f.write("\n")
        
        # Results table
        f.write("Results Table:\n")
        f.write("-" * 100 + "\n")
        
        header = (
            f"{'p_drop':>7} {'delay':>6} {'jitter':>6} | "
            f"{'success':>7} {'duration':>8} | "
            f"{'TX_msg':>8} {'RX_msg':>8} | "
            f"{'TX_B/s':>8} {'RX_B/s':>8} | "
            f"{'drop%':>6}\n"
        )
        f.write(header)
        f.write("-" * 100 + "\n")
        
        for r in results:
            row = (
                f"{r['p_drop']:>7.2f} {r['delay_max']:>6.2f} {r['jitter']:>6.2f} | "
                f"{r['success_rate']*100:>6.1f}% {r['avg_duration']:>7.2f}s | "
                f"{r['avg_msg_sent']:>8.0f} {r['avg_msg_delivered']:>8.0f} | "
                f"{r['avg_tx_Bps']:>8.1f} {r['avg_rx_Bps']:>8.1f} | "
                f"{r['observed_drop_rate']*100:>5.1f}%\n"
            )
            f.write(row)
        
        f.write("=" * 100 + "\n")
        f.write(f"Payload per message: {PAYLOAD_BYTES} bytes (O(1), NOT O(n))\n")
        f.write("=" * 100 + "\n")
    
    return str(output_path)


def demo():
    """Demonstrate the comm-feasible distributed pruning algorithm."""
    print("\n" + "=" * 70)
    print("DEMO: Comm-Feasible Distributed Spanning-Tree Pruning")
    print("=" * 70)
    
    # Create a demo graph
    G = create_demo_graph(n=10, edge_prob=0.4, seed=42)
    
    print(f"\nCreated random connected graph:")
    print(f"  Nodes: {G.number_of_nodes()}")
    print(f"  Edges: {G.number_of_edges()}")
    print(f"  Edge list: {sorted(G.edges())}")
    
    # Run the algorithm with default parameters (no drops/delays)
    print("\n--- Run 1: Ideal channel (no drops, no delay) ---")
    alg1 = DistributedPruningAlgorithm(
        graph=G,
        root=1,
        p_drop=0.0,
        delay_max=0.0,
        t_broadcast=1.0,
        seed=123
    )
    
    kept_edges = alg1.run(t_prune=15.0, dt=0.1, t_stable=3.0, verbose=True)
    
    print(f"\nKept edges (spanning tree): {sorted(kept_edges)}")
    
    # Run with packet drops and delays
    print("\n\n--- Run 2: Lossy channel (10% drop, max 0.5s delay) ---")
    alg2 = DistributedPruningAlgorithm(
        graph=G,
        root=1,
        p_drop=0.1,
        delay_max=0.5,
        t_broadcast=1.0,
        jitter=0.2,
        seed=456
    )
    
    kept_edges2 = alg2.run(t_prune=20.0, dt=0.1, t_stable=5.0, verbose=True)
    
    print(f"\nKept edges (spanning tree): {sorted(kept_edges2)}")
    
    # Summary comparison
    print("\n" + "=" * 70)
    print("SUMMARY (TX=attempted, RX=delivered)")
    print("=" * 70)
    
    comm1 = alg1.summary_comm_stats()
    comm2 = alg2.summary_comm_stats()
    
    print(f"\nIdeal channel:")
    print(f"  Messages: TX={comm1['messages_sent_total']}, RX={comm1['messages_delivered_total']}")
    print(f"  Bandwidth: TX={comm1['tx_bytes_per_second']:.1f} B/s, "
          f"RX={comm1['rx_bytes_per_second']:.1f} B/s")
    
    print(f"\nLossy channel:")
    print(f"  Messages: TX={comm2['messages_sent_total']}, "
          f"RX={comm2['messages_delivered_total']}, "
          f"Dropped={comm2['messages_dropped_total']}")
    print(f"  Bandwidth: TX={comm2['tx_bytes_per_second']:.1f} B/s, "
          f"RX={comm2['rx_bytes_per_second']:.1f} B/s")
    print(f"  Drop rate: {comm2['drop_rate_actual']*100:.1f}%")
    
    print("\n" + "=" * 70)
    print("KEY IMPROVEMENT: No O(n) vectors broadcast!")
    print(f"Message payload: {PAYLOAD_BYTES} bytes (sender_id + delta + seq)")
    print("=" * 70)


def run_demo_with_stress_test(
    run_stress: bool = True,
    outdir: Optional[str] = None,
    tag: Optional[str] = None
):
    """Run demo and optionally the stress test grid.
    
    Args:
        run_stress: Whether to run the stress test grid
        outdir: Output directory for stress test results (None = no save)
        tag: Run label for output folder (defaults to timestamp)
    """
    demo()
    
    if run_stress:
        print("\n\n")
        print("#" * 100)
        print("# STRESS TEST GRID (small example: n=10, trials=5)")
        print("#" * 100)
        
        # Define parameters
        params = {
            'n': 10,
            'edge_prob': 0.4,
            'trials': 5,
            'drop_list': [0.0, 0.1, 0.3],
            'delay_list': [0.0, 0.5],
            'jitter_list': [0.0, 0.2],
            't_broadcast': 1.0,
            't_prune': 15.0,
            'dt': 0.1,
            't_stable': 4.0,
            'base_seed': 42,
        }
        
        results = run_stress_grid(
            n=params['n'],
            edge_prob=params['edge_prob'],
            trials=params['trials'],
            drop_list=params['drop_list'],
            delay_list=params['delay_list'],
            jitter_list=params['jitter_list'],
            t_broadcast=params['t_broadcast'],
            t_prune=params['t_prune'],
            dt=params['dt'],
            t_stable=params['t_stable'],
            base_seed=params['base_seed']
        )
        
        print_stress_grid_results(results, params=params)
        
        # Save results if outdir specified
        if outdir is not None:
            output_path = save_stress_grid_results(
                results=results,
                params=params,
                outdir=outdir,
                tag=tag
            )
            print(f"\n>> Results saved to: {output_path}")
            print(f"   - results.json")
            print(f"   - results.csv")
            print(f"   - summary.txt")


def parse_cli_args(argv: List[str]) -> Dict:
    """Parse command line arguments.
    
    Args:
        argv: List of command line arguments (sys.argv)
    
    Returns:
        Dict with parsed arguments
    """
    args = {
        'run_stress': False,
        'outdir': None,
        'tag': None,
    }
    
    i = 1  # Skip script name
    while i < len(argv):
        arg = argv[i]
        
        if arg == '--stress':
            args['run_stress'] = True
        elif arg == '--outdir':
            if i + 1 < len(argv):
                args['outdir'] = argv[i + 1]
                i += 1
            else:
                print("Error: --outdir requires a path argument")
                sys.exit(1)
        elif arg == '--tag':
            if i + 1 < len(argv):
                args['tag'] = argv[i + 1]
                i += 1
            else:
                print("Error: --tag requires a string argument")
                sys.exit(1)
        elif arg in ['--help', '-h']:
            print("Usage: python distributed_pruning_algorithm.py [OPTIONS]")
            print("")
            print("Options:")
            print("  --stress          Run stress test grid after demo")
            print("  --outdir <path>   Save stress results to <path>/<tag>/")
            print("  --tag <string>    Run label (default: timestamp)")
            print("  --help, -h        Show this help message")
            print("")
            print("Examples:")
            print("  python distributed_pruning_algorithm.py")
            print("  python distributed_pruning_algorithm.py --stress")
            print("  python distributed_pruning_algorithm.py --stress --outdir stress_results")
            print("  python distributed_pruning_algorithm.py --stress --outdir results --tag run1")
            sys.exit(0)
        else:
            print(f"Unknown argument: {arg}")
            print("Use --help for usage information")
            sys.exit(1)
        
        i += 1
    
    # If --stress with --outdir, default outdir if not specified
    if args['run_stress'] and args['outdir'] is None:
        # Check if user wants to save (they specified --outdir somewhere)
        pass  # outdir stays None, no save
    
    return args


if __name__ == "__main__":
    args = parse_cli_args(sys.argv)
    
    run_demo_with_stress_test(
        run_stress=args['run_stress'],
        outdir=args['outdir'],
        tag=args['tag']
    )
