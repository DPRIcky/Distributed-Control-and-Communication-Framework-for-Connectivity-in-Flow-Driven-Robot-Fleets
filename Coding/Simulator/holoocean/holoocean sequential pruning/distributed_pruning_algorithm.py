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
   - Holoocean's network model applies packet loss and delay

3. EPOCH STRUCTURE:
   - Topology update epochs run for `t_prune` seconds
   - After each epoch, agents maintain edges to their parents (pruned topology)
   - Pruned topology feeds into CLF-CBF controllers for formation control

4. PERIODIC UPDATES:
   - Epochs can run periodically or on topology change detection

==============================================================================
"""

# =============================================================================
# Imports
# =============================================================================
import random
from typing import Dict, Set, Tuple, List, Optional, Any
import networkx as nx
from dataclasses import dataclass, field

# =============================================================================
# Constants
# =============================================================================
BYTES_SENDER_ID = 4         # uint32
BYTES_DELTA = 2             # uint16 (sufficient for hop counts < 65535)
BYTES_SEQ = 2         # uint16
PAYLOAD_BYTES = BYTES_SENDER_ID + BYTES_DELTA + BYTES_SEQ  # = 6 bytes per message
# =============================================================================
# Edge status constants (for sequential pruning)
# =============================================================================
EDGE_STATUS_PARENT = "PARENT"
EDGE_STATUS_BACKUP = "BACKUP"
EDGE_STATUS_GRACE = "GRACE"
EDGE_STATUS_DELETE_CANDIDATE = "DELETE_CANDIDATE"
EDGE_STATUS_UNUSED = "UNUSED"

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

    # === Sequential pruning fields (to be used in future phases) ===
    # Note: current delta-BFS algorithm uses `parent` as the candidate parent.
    committed_parent: Optional[int] = None   # The parent that is currently committed (stable)
    backup_parent: Optional[int] = None      # A backup parent for fast failover

    # Per-neighbor state for edge status and timers
    edge_status: Dict[int, str] = field(default_factory=dict)   # Maps neighbor ID to edge status string
    delete_counter: Dict[int, float] = field(default_factory=dict) # Maps neighbor ID to timestamp when edge entered DELETE_CANDIDATE state (0.0 if not in DELETE_CANDIDATE)
    grace_counter: Dict[int, float] = field(default_factory=dict)  # Maps neighbor ID to timestamp when grace period started for this edge (0.0 if not in grace)
    last_heard_time: Dict[int, float] = field(default_factory=dict) # Maps neighbor ID to last heard simulation time
    last_candidate: Optional[int] = None       # The candidate parent from the previous update (used for grace period triggering)

# =============================================================================
# DistributedPruningAlgorithm (Comm-Feasible Refactor)
# =============================================================================
class DistributedPruningAlgorithm:
    def __init__(
        self,
        graph: nx.Graph,
        root: int = 1,
        p_drop: float = 0.0,
        delay_max: float = 0.0,
        t_broadcast: float = 1.0,
        jitter: float = 0.0,
        seed: Optional[int] = None,
        delete_threshold_h: float = 3.0,
        grace_period_g: float = 2.0,
        enable_backup: bool = True
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
            delete_threshold_h: Time threshold (seconds) for deleting an edge in DELETE_CANDIDATE state.
            grace_period_g: Time threshold (seconds) for grace period after parent switching.
            enable_backup: Whether to enable backup parent selection.
        """
        self.graph = graph
        self.root = root
        self.nodes = list(graph.nodes())
        self.p_drop = p_drop
        self.delay_max = delay_max
        self.t_broadcast = t_broadcast
        self.jitter = jitter
        self.seed = seed

        # Sequential pruning parameters
        self.delete_threshold_h = delete_threshold_h
        self.grace_period_g = grace_period_g
        self.enable_backup = enable_backup

        # Per-node state
        self.node_state: Dict[int, NodeState] = {}
        for node in self.nodes:
            self.node_state[node] = NodeState(node_id=node)
            self.node_state[node].neighbors = set(self.graph.neighbors(node))
            # Initialize per-neighbor state for sequential pruning fields
            state = self.node_state[node]
            for neighbor in state.neighbors:
                state.edge_status[neighbor] = EDGE_STATUS_UNUSED
                state.delete_counter[neighbor] = -1.0  # -1.0 means not currently deleting
                state.grace_counter[neighbor] = 0.0
                state.last_heard_time[neighbor] = 0.0
                state.last_candidate = None  # Initialize the new field
            # Initialize root node's delta_to_root to 0
            if node == self.root:
                state.delta_to_root = 0.0
                state.parent = None  # Root has no parent

        # Message queue for simulating network delay
        self.message_queue: List[Message] = []

        # Comm statistics
        self.messages_sent_total = 0
        self.messages_delivered_total = 0
        self.messages_dropped_total = 0
        self.payload_bytes_sent_total = 0
        self.payload_bytes_delivered_total = 0

        # Broadcast schedule
        self.next_broadcast_time: Dict[int, float] = {}

        # Algorithm state
        self.final_edges: Set[Tuple[int, int]] = set()
        self.run_duration: float = 0.0

        # Set random seed
        if seed is not None:
            random.seed(seed)

        # Initialize broadcast schedule
        self._init_broadcast_schedule()

    # =========================================================================
    # Sequential Pruning Helper Methods
    # =========================================================================

    def _update_edge_status_for_node(self, node_id: int, t_now: float) -> None:
        """Update the edge status for all neighbors of a node based on the current state and time.
        This method updates the edge_status, the timers (grace_counter and delete_counter) when entering states,
        and the kept_neighbors set.
        Implements a proper state machine for edge status transitions.
        """
        state = self.node_state[node_id]
        for neighbor in state.neighbors:
            # Remember the old status to detect transitions
            old_status = state.edge_status.get(neighbor, EDGE_STATUS_UNUSED)

            # First, determine the "desired" status ignoring delete timers
            # (i.e., what the status should be if we were to ignore deletion delays)
            if state.committed_parent == neighbor:
                desired_status = EDGE_STATUS_PARENT
            elif self.enable_backup and state.backup_parent == neighbor:
                desired_status = EDGE_STATUS_BACKUP
            else:
                # Check grace period: if we have a grace timer started and it hasn't expired
                grace_start = state.grace_counter.get(neighbor, 0.0)
                grace_active = grace_start > 0.0 and (t_now - grace_start) < self.grace_period_g
                if grace_active:
                    desired_status = EDGE_STATUS_GRACE
                else:
                    desired_status = EDGE_STATUS_UNUSED  # meaning we want to delete it, but haven't started delete timer yet

            # Now handle the delete counter logic for transitions to/from UNUSED
            if desired_status == EDGE_STATUS_UNUSED:
                # We want to delete this edge (it's not PARENT, BACKUP, or GRACE)
                delete_start = state.delete_counter.get(neighbor, -1.0)
                if delete_start >= 0.0:
                    # We are already in the process of deleting
                    if (t_now - delete_start) >= self.delete_threshold_h:
                        # Delete timer expired -> transition to UNUSED
                        new_status = EDGE_STATUS_UNUSED
                        state.delete_counter[neighbor] = -1.0  # -1.0 means not currently deleting  # reset delete counter
                    else:
                        new_status = EDGE_STATUS_DELETE_CANDIDATE
                else:
                    # Not currently deleting -> start the delete timer
                    state.delete_counter[neighbor] = t_now
                    new_status = EDGE_STATUS_DELETE_CANDIDATE
            else:
                # The desired status is PARENT, BACKUP, or GRACE -> we want to keep the edge
                new_status = desired_status
                state.delete_counter[neighbor] = -1.0  # -1.0 means not currently deleting  # reset delete counter because we are keeping the edge

            # Update the edge status
            state.edge_status[neighbor] = new_status

            # Update the kept_neighbors set: add if the edge is in PARENT, BACKUP, GRACE, or DELETE_CANDIDATE; remove otherwise
            if new_status in (EDGE_STATUS_PARENT, EDGE_STATUS_BACKUP, EDGE_STATUS_GRACE, EDGE_STATUS_DELETE_CANDIDATE):
                state.kept_neighbors.add(neighbor)
            else:
                if neighbor in state.kept_neighbors:
                    state.kept_neighbors.remove(neighbor)

    def _update_backup_parent(self, node_id: int) -> None:
        """Select a backup parent for the node if backup is enabled.
        The backup parent is the neighbor (not the committed parent) with the smallest delta_to_root
        (and smallest node ID in case of tie) that has a finite delta_to_root.
        """
        state = self.node_state[node_id]
        if not self.enable_backup:
            state.backup_parent = None
            return

        best_neighbor = None
        best_delta = float('inf')
        for neighbor in state.neighbors:
            if neighbor == state.committed_parent:
                continue
            neighbor_state = self.node_state[neighbor]
            delta = neighbor_state.delta_to_root
            # Skip neighbors with no path to root (infinite delta)
            if delta == float('inf'):
                continue
            if delta < best_delta or (delta == best_delta and (best_neighbor is None or neighbor < best_neighbor)):
                best_delta = delta
                best_neighbor = neighbor
        state.backup_parent = best_neighbor

    def _update_committed_parent(self, node_id: int, t_now: float) -> None:
        """Update the committed parent based on the candidate parent and grace period timers.
        This method implements a grace period delay when the candidate parent changes to avoid flapping.
        The grace timer is only started when the candidate parent changes from the previous update.
        """
        state = self.node_state[node_id]
        old_committed = state.committed_parent
        candidate = state.parent   # candidate parent from delta-BFS

        # If there is no candidate parent, we cannot commit to a parent
        if candidate is None:
            state.committed_parent = None
            # Clear any grace period for the old committed parent edge
            if old_committed is not None and old_committed in state.neighbors:
                state.grace_counter[old_committed] = 0.0
            # Also clear last_candidate since there's no valid candidate
            state.last_candidate = None
            return

        # If the candidate parent is the same as the old committed parent, clear any grace period
        if candidate == old_committed:
            if old_committed is not None and old_committed in state.neighbors:
                state.grace_counter[old_committed] = 0.0
            # Clear last_candidate since we have a stable candidate matching committed parent
            state.last_candidate = None
            # No change to committed_parent needed
            return

        # Candidate parent is different from old committed parent
        # Check if the candidate parent has changed since the last update
        if state.last_candidate != candidate:
            # The candidate parent has changed -> start grace period for the old committed parent edge
            if old_committed is not None and old_committed in state.neighbors:
                state.grace_counter[old_committed] = t_now
        # Update last_candidate for next update
        state.last_candidate = candidate

        # Check if the grace period for the old committed parent edge has expired
        if old_committed is not None and old_committed in state.neighbors:
            grace_start = state.grace_counter[old_committed]
            if grace_start > 0.0 and (t_now - grace_start) > self.grace_period_g:
                # Grace period expired: we can now update the committed parent to the candidate
                state.committed_parent = candidate
                # Clear the grace counter for the old committed parent edge
                state.grace_counter[old_committed] = 0.0
        else:
            # No old committed parent or not a neighbor: we can update the committed parent immediately
            state.committed_parent = candidate

    def _update_sequential_state_for_node(self, node_id: int, t_now: float, dt: float = 0.0) -> None:
        """Update the sequential pruning state for a node.
        This includes updating backup parent, committed parent, and edge status.
        The order is important: we must update committed/backup state BEFORE edge classification
        so that edge status uses the most recent state.
        The dt parameter is kept for compatibility but not used in this implementation.
        """
        # Update backup parent and committed parent FIRST
        self._update_backup_parent(node_id)
        self._update_committed_parent(node_id, t_now)
        # THEN update edge status (which uses the updated committed/backup state)
        self._update_edge_status_for_node(node_id, t_now)

    # =========================================================================
    # Message Handling
    # =========================================================================

    def _on_receive(self, msg: Message, t_now: float) -> None:
        """Process a received message."""
        # δ-BFS update
        receiver_state = self.node_state[msg.receiver]

        # Only process if the message is not stale (based on sequence number)
        last_seq = receiver_state.last_seq_from_neighbor.get(msg.sender, -1)
        if msg.seq <= last_seq:
            # Stale or duplicate message
            return

        receiver_state.last_seq_from_neighbor[msg.sender] = msg.seq
        receiver_state.last_heard_time[msg.sender] = t_now

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

        # Update sequential pruning state for the receiver node
        # Note: dt is not used in this implementation but kept for compatibility
        self._update_sequential_state_for_node(msg.receiver, t_now, 0.0)

    # =========================================================================
    # Message Delivery Simulation
    # =========================================================================

    def _deliver_messages(self, t_now: float) -> None:
        """Deliver messages that are due at time t_now."""
        # Find messages to deliver
        to_deliver = []
        remaining = []
        for msg in self.message_queue:
            if msg.deliver_time <= t_now:
                to_deliver.append(msg)
            else:
                remaining.append(msg)
        self.message_queue = remaining

        # Deliver each message
        for msg in to_deliver:
            self._on_receive(msg, t_now)
            self.messages_delivered_total += 1
            self.payload_bytes_delivered_total += PAYLOAD_BYTES

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

        # Create messages to all neighbors
        for neighbor in state.neighbors:
            # Simulate packet loss
            if random.random() < self.p_drop:
                self.messages_dropped_total += 1
                continue

            # Simulate random delay
            delay = random.uniform(0, self.delay_max)
            deliver_time = t_now + delay

            # Create message
            msg = Message(
                deliver_time=deliver_time,
                sender=node,
                receiver=neighbor,
                delta=state.delta_to_root,
                seq=state.seq,
                send_time=t_now
            )
            self.message_queue.append(msg)
            self.messages_sent_total += 1
            self.payload_bytes_sent_total += PAYLOAD_BYTES

    def _schedule_next_broadcast(self, node: int, t_now: float):
        """Schedule the next broadcast for a node."""
        # Add jitter to prevent synchronization
        jitter_offset = random.uniform(-self.jitter, self.jitter)
        next_time = t_now + self.t_broadcast + jitter_offset
        # Ensure next time is at least t_broadcast/2 in the future to avoid too frequent broadcasts
        self.next_broadcast_time[node] = max(next_time, t_now + self.t_broadcast * 0.5)

    # =========================================================================
    # Core Algorithm Loop
    # =========================================================================

    def run(self, t_prune: float, dt: float = 0.1, t_stable: Optional[float] = None, verbose: bool = False) -> Set[Tuple[int, int]]:
        """
        Run the algorithm for a specified duration.

        Args:
            t_prune: Duration to run the algorithm (seconds)
            dt: Time step for simulation (seconds)
            t_stable: Optional time after which to check for stability (for early stopping)
            verbose: Whether to print verbose output

        Returns:
            Set of edges in the final spanning tree
        """
        # Reset state for this run
        self.reset_epoch_state(reset_deltas=True)

        # Timing variables
        t = 0.0
        last_global_update = 0.0

        while t < t_prune:
            # Check each node for broadcast
            for node in self.nodes:
                if t >= self.next_broadcast_time[node]:
                    self._broadcast(node, t)
                    self._schedule_next_broadcast(node, t)

            # Deliver messages
            self._deliver_messages(t)

            # Update sequential pruning state for each node
            for node in self.nodes:
                self._update_sequential_state_for_node(node, t, dt)

            # Check for global stability (optional early stopping)
            if t_stable is not None:
                max_update_time = max(
                    state.last_delta_update_time
                    for state in self.node_state.values()
                )
                if max_update_time - last_global_update > t_stable:
                    if verbose:
                        print(f"Early stopping at t={t:.2f}s (stable for {t_stable}s)")
                    break
                last_global_update = max_update_time

            # Increment time
            t += dt

        # Build final spanning tree
        kept_edges = self.build_spanning_tree(verbose=verbose)
        self.run_duration = t

        if verbose:
            print(f"Algorithm finished after {t:.2f} seconds")
            print(f"Messages sent: {self.messages_sent_total}, delivered: {self.messages_delivered_total}")
            print(f"Spanning tree has {len(kept_edges)} edges")

        return kept_edges

    # =========================================================================
    # Spanning Tree Construction
    # =========================================================================

    def build_spanning_tree(self, verbose: bool = False) -> Set[Tuple[int, int]]:
        """
        Build the spanning tree from the committed parent pointers.

        Args:
            verbose: Whether to print verbose output

        Returns:
            Set of edges in the spanning tree (as tuples of (min_node_id, max_node_id))
        """
        kept_edges: Set[Tuple[int, int]] = set()

        for node in self.nodes:
            if node == self.root:
                continue

            state = self.node_state[node]
            if state.committed_parent is not None:
                # Store as undirected edge (min, max)
                edge = tuple(sorted([node, state.committed_parent]))
                kept_edges.add(edge)

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
        self._init_broadcast_schedule()  # Reinitialize broadcast schedule

        # Reset per-node state
        for node in self.nodes:
            state = self.node_state[node]
            state.kept_neighbors.clear()
            state.last_seq_from_neighbor.clear()
            state.last_delta_update_time = 0.0
            state.seq = 0  # Reset sequence numbers
            # Reset sequential pruning state
            state.committed_parent = None
            state.backup_parent = None
            state.edge_status.clear()
            state.delete_counter.clear()
            state.grace_counter.clear()
            state.last_heard_time.clear()
            state.last_candidate = None  # Reset the new field
            # Re-initialize per-neighbor state
            for neighbor in state.neighbors:
                state.edge_status[neighbor] = EDGE_STATUS_UNUSED
                state.delete_counter[neighbor] = -1.0  # -1.0 means not currently deleting
                state.grace_counter[neighbor] = 0.0
                state.last_heard_time[neighbor] = 0.0

        # Optionally reset delta_BFS state
        if reset_deltas:
            for node in self.nodes:
                state = self.node_state[node]
                if node == self.root:
                    # Keep root's correct values
                    state.delta_to_root = 0.0
                    state.parent = None
                else:
                    state.delta_to_root = float('inf')
                    state.parent = None
                state.last_delta_update_time = 0.0

    # =========================================================================
    # Debugging and Inspection
    # =========================================================================

    def get_state(self) -> Dict[int, Dict[str, Any]]:
        """Get the current state of all nodes for debugging/inspection."""
        result = {}
        for node_id, state in self.node_state.items():
            result[node_id] = {
                'delta_to_root': state.delta_to_root,
                'parent': state.parent,
                'committed_parent': state.committed_parent,
                'backup_parent': state.backup_parent,
                'seq': state.seq,
                'kept_neighbors': list(state.kept_neighbors),
                'edge_status': dict(state.edge_status),
                'delete_counter': dict(state.delete_counter),
                'grace_counter': dict(state.grace_counter),
                'last_heard_time': dict(state.last_heard_time),
                'last_candidate': state.last_candidate
            }
        return result