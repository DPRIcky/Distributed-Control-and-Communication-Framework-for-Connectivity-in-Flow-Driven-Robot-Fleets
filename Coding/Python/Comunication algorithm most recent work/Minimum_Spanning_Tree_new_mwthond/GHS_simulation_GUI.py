"""GHS Algorithm Simulation: Distributed MST Discovery.

Implements the Gallager-Humblet-Spira algorithm for distributed minimum
spanning tree discovery on a fully connected graph.

MST edges are colored green as they're discovered.
Non-MST edges remain purple/violet.
"""

import sys
from pathlib import Path
parent_dir = Path(__file__).parent.parent
sys.path.insert(0, str(parent_dir))

import numpy as np
import matplotlib.pyplot as plt
from baseline_simulation import FullyConnectedSimulation, FullyConnectedAnimator
from config import SimulationConfig, ControlConfig, VisualizationConfig
from enum import Enum
from collections import defaultdict, deque


# GHS States
class NodeState(Enum):
    SLEEPING = "SLEEPING"
    FIND = "FIND"
    FOUND = "FOUND"


class EdgeState(Enum):
    BASIC = "BASIC"
    BRANCH = "BRANCH"  # MST edge (will be green)
    REJECTED = "REJECTED"


# GHS Messages
class Message:
    def __init__(self, msg_type, sender, receiver, **kwargs):
        self.type = msg_type
        self.sender = sender
        self.receiver = receiver
        self.data = kwargs


class GHSSimulation(FullyConnectedSimulation):
    """GHS Algorithm for distributed MST discovery.
    
    Each robot runs the GHS protocol independently to discover the MST.
    MST edges (BRANCH) are colored green, other edges stay purple.
    """
    
    def __init__(self, sim_config, control_config):
        """Initialize GHS simulation."""
        super().__init__(sim_config, control_config)
        
        n = self.num_robots
        
        # GHS state for each robot
        self.node_state = [NodeState.SLEEPING] * n
        self.fragment_id = list(range(n))  # Initially each robot is its own fragment
        self.level = [0] * n
        self.parent_edge = [None] * n
        self.best_edge = [None] * n
        self.best_weight = [(float('inf'), float('inf'), float('inf'))] * n
        self.test_edge = [None] * n
        self.accepted_edges = [set() for _ in range(n)]  # Track edges that received ACCEPT (to avoid re-testing)
        self.find_count = [0] * n
        self.in_branch = [set() for _ in range(n)]  # Edges in MST for each robot
        
        # Queued messages (from reference implementation pattern)
        self.connect_requests = [{} for _ in range(n)]  # robot_id -> {sender: level}
        self.test_requests = [set() for _ in range(n)]  # robot_id -> {(level, frag_id, sender)}
        
        # State flags for condition-based processing
        self.sent_connect_to = [None] * n  # Track which neighbor we sent CONNECT to
        self.test_over = [None] * n  # True when no more BASIC edges to test
        self.report_over = [False] * n  # True when REPORT sent
        self.expect_core_report = [None] * n  # True when waiting for merge report
        self.find_source = [None] * n  # Who initiated the FIND
        
        # Edge states (shared view from both endpoints)
        self.edge_states = {}  # (i, j) -> EdgeState
        edges = self.current_edges()
        for edge in edges:
            i, j = edge
            self.edge_states[(i, j)] = EdgeState.BASIC
            self.edge_states[(j, i)] = EdgeState.BASIC
        
        # Message queue
        self.message_queue = deque()
        self.messages_processed = 0
        
        # Edge weights (using current distances with tie-breaking)
        # GHS requires unique edge weights, so we use (distance, edge_id) for comparison
        self.edge_weights = {}
        self._update_edge_weights()
        
        # Deferred messages (for TEST when Level(sender) > Level(receiver))
        self.deferred_messages = defaultdict(list)  # robot_id -> [messages]
        
        # Message transmission delay simulation (messages arrive after delay)
        self.pending_messages = []  # List of (arrival_time, message)
        self.message_delay = 0.0  # NO delay - immediate delivery for GHS
        
        # Debug tracking
        self.last_debug_time = 0
        self.debug_interval = 5.0  # Print debug every 5 seconds
        
        # Edge pruning statistics
        self.initial_edge_count = len(edges)
        self.pruned_edge_count = 0
        
        # Initial spontaneous wakeup: All robots can wake (reference behavior)
        # In GHS, multiple robots can spontaneously wake and trigger merges
        self.ghs_started = False
        self.initial_wakeup_done = False
        
        # Start GHS: Each robot wakes up at its scheduled time
        self.ghs_started = False
        self.ghs_converged = False
        self.convergence_time = None
        
        print("\n[GHS Simulation Initialized]")
        print(f"  Robots: {n}")
        print(f"  Initial edges: {len(edges)} (fully connected)")
        print(f"  Target MST edges: {n-1}")
        print(f"  GHS Algorithm following Wikipedia specification")
        print(f"  ✓ Real-time edge pruning ENABLED")
        print(f"  Assumptions: Unique edge weights, FIFO delivery, asynchronous")
    
    def _update_edge_weights(self):
        """Update edge weights based on current distances.
        
        To ensure unique weights (GHS assumption), we use lexicographic ordering:
        (distance, min(i,j), max(i,j)) for comparison.
        """
        edges = self.current_edges()
        for i, j in edges:
            dist = np.linalg.norm(self.robots[i].position - self.robots[j].position)
            # Store as (distance, edge_tuple) for unique ordering
            edge_id = self._normalize_edge(i, j)
            weight = (dist, edge_id[0], edge_id[1])  # Unique weight with tie-breaking
            self.edge_weights[(i, j)] = weight
            self.edge_weights[(j, i)] = weight
    
    def _normalize_edge(self, i, j):
        """Normalize edge to (min, max) order."""
        return (min(i, j), max(i, j))
    
    def _get_edge_state(self, i, j):
        """Get edge state."""
        return self.edge_states.get((i, j), EdgeState.BASIC)
    
    def _set_edge_state(self, i, j, state):
        """Set edge state (both directions)."""
        self.edge_states[(i, j)] = state
        self.edge_states[(j, i)] = state
        
        # If edge is marked as REJECTED, prune it from the graph
        if state == EdgeState.REJECTED:
            self._prune_edge(i, j)
    
    def _prune_edge(self, i, j):
        """Remove edge from topology manager's neighbor graph.
        
        This physically removes the edge so it no longer appears in visualizations
        or neighbor queries. Called automatically when edge is marked REJECTED.
        
        Args:
            i: First robot ID
            j: Second robot ID
        """
        # Remove j from i's neighbors
        if j in self.topology_manager.neighbor_graph[i]:
            self.topology_manager.neighbor_graph[i].remove(j)
        
        # Remove i from j's neighbors
        if i in self.topology_manager.neighbor_graph[j]:
            self.topology_manager.neighbor_graph[j].remove(i)
        
        # Remove edge weights
        self.edge_weights.pop((i, j), None)
        self.edge_weights.pop((j, i), None)
        
        # Update statistics
        self.pruned_edge_count += 1
        
        if self.verbose:
            remaining = self.initial_edge_count - self.pruned_edge_count
            print(f"  [PRUNED] Edge ({i}, {j}) | Pruned: {self.pruned_edge_count}, Remaining: {remaining}")
    
    def _get_neighbors(self, robot_id):
        """Get neighbors of a robot."""
        neighbors = []
        edges = self.current_edges()
        for i, j in edges:
            if i == robot_id:
                neighbors.append(j)
            elif j == robot_id:
                neighbors.append(i)
        return neighbors
    
    def _get_minimum_edge(self, robot_id, edge_filter=None):
        """Get minimum weight edge for a robot.
        
        Args:
            robot_id: Robot ID
            edge_filter: Optional function to filter edges (returns True to include)
        """
        neighbors = self._get_neighbors(robot_id)
        min_weight = (float('inf'), float('inf'), float('inf'))
        min_neighbor = None
        
        for neighbor in neighbors:
            if edge_filter and not edge_filter(robot_id, neighbor):
                continue
            weight = self.edge_weights.get((robot_id, neighbor), (float('inf'), 0, 0))
            if weight < min_weight:
                min_weight = weight
                min_neighbor = neighbor
        
        return min_neighbor, min_weight
    
    def _wakeup(self, robot_id):
        """Level-0 wakeup: Each node finds min edge and sends CONNECT(0).
        
        Per Wikipedia: In level-0 fragments:
        1. Choose minimum-weight incident edge
        2. Send CONNECT(0) on that edge
        3. Edge becomes BRANCH only after merge/absorb is confirmed
        """
        if self.node_state[robot_id] != NodeState.SLEEPING:
            return
        
        # Find minimum weight edge among all incident edges
        min_neighbor, min_weight = self._get_minimum_edge(
            robot_id,
            edge_filter=lambda i, j: self._get_edge_state(i, j) == EdgeState.BASIC
        )
        
        if min_neighbor is not None:
            # DON'T mark as BRANCH yet - wait for merge/absorb confirmation
            # The edge will be marked BRANCH in _merge() or _absorb_node()
            
            # Set to level 0 (will transition to FIND when absorbed/merged)
            self.level[robot_id] = 0
            self.node_state[robot_id] = NodeState.FOUND  # Temporary state until absorbed
            self.find_count[robot_id] = 0
            
            # Track who we sent CONNECT to (for merge detection)
            self.sent_connect_to[robot_id] = min_neighbor
            
            # Send CONNECT(0) to initiate fragment merge
            self._send_message_with_delay(Message(
                'CONNECT', robot_id, min_neighbor, level=0
            ))
    
    def _send_initiate(self, robot_id, neighbor, level, frag_id, state):
        """Send INITIATE message to neighbor."""
        self._send_message_with_delay(Message(
            'INITIATE', robot_id, neighbor,
            level=level, fragment_id=frag_id, state=state
        ))
    
    def _process_connect(self, msg):
        """Process CONNECT message."""
        sender = msg.sender
        receiver = msg.receiver
        sender_level = msg.data['level']
        
        # If receiver is sleeping, wake up
        if self.node_state[receiver] == NodeState.SLEEPING:
            self._wakeup(receiver)
        
        receiver_level = self.level[receiver]
        
        if sender_level < receiver_level:
            # Absorb sender's fragment
            self._set_edge_state(receiver, sender, EdgeState.BRANCH)
            self.in_branch[receiver].add(sender)
            self.in_branch[sender].add(receiver)
            
            # Send INITIATE to sender
            self._send_initiate(receiver, sender, self.level[receiver],
                              self.fragment_id[receiver], self.node_state[receiver])
            
            if self.node_state[receiver] == NodeState.FIND:
                self.find_count[receiver] += 1
        
        elif sender_level == receiver_level:
            # Merge fragments - mark edge as BRANCH
            self._set_edge_state(receiver, sender, EdgeState.BRANCH)
            self.in_branch[receiver].add(sender)
            self.in_branch[sender].add(receiver)
            
            # New fragment at higher level
            new_level = receiver_level + 1
            new_frag_id = min(self.fragment_id[receiver], self.fragment_id[sender])
            
            # Send INITIATE to both sides to propagate new fragment identity
            # Use the edge as the core of the new fragment
            self._send_message_with_delay(Message(
                'INITIATE', receiver, receiver,  # Self-initiate
                level=new_level, fragment_id=new_frag_id, state=NodeState.FIND,
                parent=sender
            ))
            self._send_message_with_delay(Message(
                'INITIATE', sender, sender,  # Self-initiate
                level=new_level, fragment_id=new_frag_id, state=NodeState.FIND,
                parent=receiver
            ))
        
        else:  # sender_level > receiver_level
            # Defer message (receiver will catch up)
            pass
    
    def _test_minimum_edge(self, robot_id):
        """Test minimum weight basic edge (excluding already accepted edges)."""
        neighbors = self._get_neighbors(robot_id)
        min_weight = (float('inf'), float('inf'), float('inf'))
        min_neighbor = None
        
        for neighbor in neighbors:
            # Skip edges that have already been tested and accepted
            if neighbor in self.accepted_edges[robot_id]:
                continue
                
            edge_state = self._get_edge_state(robot_id, neighbor)
            if edge_state == EdgeState.BASIC:
                weight = self.edge_weights.get((robot_id, neighbor), (float('inf'), float('inf'), float('inf')))
                if weight < min_weight:
                    min_weight = weight
                    min_neighbor = neighbor
        
        if min_neighbor is not None:
            # Send TEST message
            self.test_edge[robot_id] = min_neighbor
            self.test_over[robot_id] = False
            self._send_message_with_delay(Message(
                'TEST', robot_id, min_neighbor,
                level=self.level[robot_id],
                fragment_id=self.fragment_id[robot_id]
            ))
        else:
            # No more basic edges to test
            self.test_over[robot_id] = True
    
    def _merge(self, robot_id, other_node):
        """Merge two fragments of same level.
        
        When two same-level fragments merge, they form a new fragment at level+1.
        The edge between them becomes the core of the new fragment.
        Both endpoints of the core edge become temporary co-roots.
        """
        connection_edge_weight = self.edge_weights.get((robot_id, other_node), (float('inf'), 0, 0))
        
        # Mark edge as BRANCH
        self._set_edge_state(robot_id, other_node, EdgeState.BRANCH)
        self.in_branch[robot_id].add(other_node)
        self.in_branch[other_node].add(robot_id)
        
        # New fragment at higher level
        new_level = self.level[robot_id] + 1
        new_frag_id = connection_edge_weight  # Use edge weight as fragment ID
        
        # Update this node immediately to avoid processing same merge twice
        self.level[robot_id] = new_level
        self.fragment_id[robot_id] = new_frag_id
        self.node_state[robot_id] = NodeState.FIND
        self.parent_edge[robot_id] = other_node  # Parent is the other endpoint of core edge
        self.best_edge[robot_id] = None
        self.best_weight[robot_id] = (float('inf'), float('inf'), float('inf'))
        self.find_count[robot_id] = 0
        self.report_over[robot_id] = False
        
        # Send INITIATE to both sides of the merge
        # 1. Send to other_node to update it with new fragment info
        self._send_message_with_delay(Message(
            'INITIATE', robot_id, other_node,
            level=new_level, fragment_id=new_frag_id, state=NodeState.FIND,
            merge=False  # Not a merge from other_node's perspective, just an update
        ))
        
        # 2. Broadcast to all our BRANCH children (if any)
        for neighbor in self.in_branch[robot_id]:
            if neighbor != other_node:  # Don't send back to other_node (already sending above)
                self._send_initiate(robot_id, neighbor, new_level, new_frag_id, NodeState.FIND)
                self.find_count[robot_id] += 1
        
        # Start testing for minimum outgoing edge
        self._test_minimum_edge(robot_id)
    
    def _absorb_node(self, robot_id, other_node):
        """Absorb lower-level fragment.
        
        The absorbing robot adds the other node as a child.
        The absorbed node adopts this fragment's identity.
        """
        # Mark edge as BRANCH
        self._set_edge_state(robot_id, other_node, EdgeState.BRANCH)
        self.in_branch[robot_id].add(other_node)
        self.in_branch[other_node].add(robot_id)
        
        # If in FOUND state, transition to FIND to search for outgoing edges
        if self.node_state[robot_id] == NodeState.FOUND:
            # Start a FIND phase
            self.node_state[robot_id] = NodeState.FIND
            self.best_edge[robot_id] = None
            self.best_weight[robot_id] = (float('inf'), float('inf'), float('inf'))
            self.find_count[robot_id] = 0
            self.report_over[robot_id] = False
            # Note: We'll increment find_count and test after sending INITIATE
        
        # Send INITIATE to absorbed node with FIND state
        # The absorbed node will adopt our fragment identity
        self._send_initiate(robot_id, other_node, self.level[robot_id],
                          self.fragment_id[robot_id], NodeState.FIND)
        
        # Increment find_count for this new child
        self.find_count[robot_id] += 1
        
        # Start testing for minimum outgoing edge (if we just entered FIND)
        if self.node_state[robot_id] == NodeState.FIND and self.test_edge[robot_id] is None:
            # Clear accepted edges from previous FIND phase
            self.accepted_edges[robot_id].clear()
            self._test_minimum_edge(robot_id)
    
    def _process_test_requests(self, robot_id, processable_tests):
        """Process TEST requests where conditions are met (reference pattern)."""
        for (L, F, sender) in processable_tests:
            # Same fragment -> REJECT
            if F == self.fragment_id[robot_id]:
                self._send_message_with_delay(Message('REJECT', robot_id, sender))
            # Different fragment and level <= ours -> ACCEPT
            elif L <= self.level[robot_id]:
                self._send_message_with_delay(Message('ACCEPT', robot_id, sender))
            
            # Remove from queue
            self.test_requests[robot_id].discard((L, F, sender))
    
    def _do_report(self, robot_id):
        """Send REPORT to parent or initiate CHANGEROOT if root."""
        self.test_over[robot_id] = None
        self.node_state[robot_id] = NodeState.FOUND
        self.report_over[robot_id] = True
        
        # If we have a parent (find_source), send REPORT
        if self.find_source[robot_id] is not None:
            self._send_message_with_delay(Message(
                'REPORT', robot_id, self.find_source[robot_id],
                best_weight=self.best_weight[robot_id],
                best_edge=self.best_edge[robot_id]
            ))
        # If no parent (we're root) and we found a best edge, initiate CHANGEROOT
        elif self.best_edge[robot_id] is not None and \
             self.best_weight[robot_id] < (float('inf'), float('inf'), float('inf')):
            # Validate best_edge is still BASIC (may have become BRANCH during testing)
            best_edge_state = self._get_edge_state(robot_id, self.best_edge[robot_id])
            
            if best_edge_state == EdgeState.BASIC:
                # Edge still valid, send CHANGEROOT
                self._send_message_with_delay(Message(
                    'CHANGEROOT', robot_id, self.best_edge[robot_id]
                ))
            else:
                # Edge became BRANCH/REJECTED, rescan for valid outgoing edge
                # This can happen if another fragment merged via this edge during our FIND phase
                neighbors = self._get_neighbors(robot_id)
                min_weight = (float('inf'), float('inf'), float('inf'))
                min_neighbor = None
                
                for neighbor in neighbors:
                    if self._get_edge_state(robot_id, neighbor) == EdgeState.BASIC:
                        weight = self.edge_weights.get((robot_id, neighbor), (float('inf'), float('inf'), float('inf')))
                        if weight < min_weight:
                            min_weight = weight
                            min_neighbor = neighbor
                
                if min_neighbor is not None:
                    # Found a new outgoing edge
                    self.best_edge[robot_id] = min_neighbor
                    self.best_weight[robot_id] = min_weight
                    self._send_message_with_delay(Message(
                        'CHANGEROOT', robot_id, min_neighbor
                    ))
                # If no BASIC edge found, this fragment is complete (shouldn't happen if multiple fragments exist)
    
    def _fragment_connect(self, robot_id):
        """Root initiates CHANGEROOT to connect fragments (reference pattern)."""
        if self.best_edge[robot_id] is not None:
            self._send_message_with_delay(Message(
                'CHANGEROOT', robot_id, self.best_edge[robot_id]
            ))
            self.report_over[robot_id] = False
            self.expect_core_report[robot_id] = None
    
    def _process_test(self, msg):
        """Process TEST message with proper deferral logic.
        
        Per Wikipedia, three cases:
        1. FragmentID(n) == FragmentID(n'): Same fragment → REJECT
        2. FragmentID(n) != FragmentID(n') and Level(n) <= Level(n'): Different fragments → ACCEPT
        3. FragmentID(n) != FragmentID(n') and Level(n) > Level(n'): DEFER until n' catches up
        """
        sender = msg.sender
        receiver = msg.receiver
        sender_level = msg.data['level']
        sender_frag_id = msg.data['fragment_id']
        
        # If receiver is sleeping, wake up
        if self.node_state[receiver] == NodeState.SLEEPING:
            self._wakeup(receiver)
            # Re-check after wakeup
        
        receiver_level = self.level[receiver]
        receiver_frag_id = self.fragment_id[receiver]
        
        # Case 1: Same fragment → REJECT
        if sender_frag_id == receiver_frag_id:
            self._send_message_with_delay(Message('REJECT', receiver, sender))
        
        # Case 2: Different fragments and sender level <= receiver level → ACCEPT
        elif sender_level <= receiver_level:
            self._send_message_with_delay(Message('ACCEPT', receiver, sender))
        
        # Case 3: Different fragments but sender level > receiver level → DEFER
        else:
            # Receiver hasn't caught up yet, defer TEST message
            self.deferred_messages[receiver].append(msg)
    
    def _process_deferred_messages(self, robot_id):
        """Process deferred TEST messages after level update.
        
        Called after a node's level increases via INITIATE message.
        """
        if robot_id not in self.deferred_messages:
            return
        
        # Process deferred messages
        deferred = self.deferred_messages[robot_id][:]
        self.deferred_messages[robot_id].clear()
        
        for msg in deferred:
            if msg.type == 'TEST':
                # Re-process with updated level
                self._process_test(msg)
    
    def _process_accept(self, msg):
        """Process ACCEPT message - record edge and continue testing to find minimum outgoing edge."""
        sender = msg.sender
        receiver = msg.receiver
        
        # Found outgoing edge
        test_edge = self.test_edge[receiver]
        if test_edge == sender:
            test_weight = self.edge_weights.get((receiver, sender), (float('inf'), float('inf'), float('inf')))
            
            if test_weight < self.best_weight[receiver]:
                self.best_edge[receiver] = sender
                self.best_weight[receiver] = test_weight
            
            # Mark this edge as accepted (don't test again in this FIND phase)
            self.accepted_edges[receiver].add(sender)
            
            # Continue testing other BASIC edges to find true minimum
            # (GHS requires testing ALL edges, not just stopping at first ACCEPT)
            self._test_minimum_edge(receiver)
    
    def _process_reject(self, msg):
        """Process REJECT message."""
        sender = msg.sender
        receiver = msg.receiver
        
        # Mark edge as rejected
        if self._get_edge_state(receiver, sender) == EdgeState.BASIC:
            self._set_edge_state(receiver, sender, EdgeState.REJECTED)
        
        # Test next edge
        self._test_minimum_edge(receiver)
    
    def _report(self, robot_id):
        """Report best edge to parent."""
        self.test_edge[robot_id] = None
        
        # If not root, wait for all children to report
        if self.find_count[robot_id] > 0:
            return  # Still waiting for children
        
        # Only send message if transitioning to FOUND (not already FOUND)
        already_found = (self.node_state[robot_id] == NodeState.FOUND)
        self.node_state[robot_id] = NodeState.FOUND
        
        if already_found:
            return  # Already sent REPORT/CHANGEROOT, don't send again
        
        # Send REPORT to parent (if exists)
        if self.parent_edge[robot_id] is not None:
            parent = self.parent_edge[robot_id]
            self._send_message_with_delay(Message(
                'REPORT', robot_id, parent,
                best_weight=self.best_weight[robot_id],
                best_edge=self.best_edge[robot_id]
            ))
        elif self.best_edge[robot_id] is not None and self.best_weight[robot_id] < (float('inf'), float('inf'), float('inf')):
            # Root node found best edge, send CHANGEROOT
            self._send_message_with_delay(Message(
                'CHANGEROOT', robot_id, self.best_edge[robot_id]
            ))
    
    def _process_report(self, msg):
        """Process REPORT message from a child."""
        sender = msg.sender
        receiver = msg.receiver
        sender_best_weight = msg.data['best_weight']
        sender_best_edge = msg.data.get('best_edge')
        
        # Update find count (child has reported)
        if self.find_count[receiver] > 0:
            self.find_count[receiver] -= 1
        
        # Update best edge if this one is better
        if sender_best_weight < self.best_weight[receiver]:
            self.best_weight[receiver] = sender_best_weight
            self.best_edge[receiver] = sender_best_edge
        
        # If all children have reported and we've finished testing, report upward
        # This is checked in condition-based processing (Condition 4)
    
    def _process_changeroot(self, msg):
        """Process CHANGEROOT message."""
        receiver = msg.receiver
        
        # If receiver's best edge is the one being changed to, connect
        if self.best_edge[receiver] == msg.sender:
            # This edge becomes the new MST edge
            self._set_edge_state(receiver, msg.sender, EdgeState.BRANCH)
            self.in_branch[receiver].add(msg.sender)
            self.in_branch[msg.sender].add(receiver)
            
            # Send CONNECT on this edge
            self._send_message_with_delay(Message(
                'CONNECT', receiver, msg.sender,
                level=self.level[receiver]
            ))
        else:
            # Forward CHANGEROOT toward best edge (but not back to sender!)
            if self.best_edge[receiver] is not None and self.best_edge[receiver] != msg.sender:
                self._send_message_with_delay(Message(
                    'CHANGEROOT', receiver, self.best_edge[receiver]
                ))
    
    def _send_message_with_delay(self, message):
        """Send message with transmission delay (or immediate if delay=0)."""
        if self.message_delay > 0:
            arrival_time = self.time + self.message_delay
            self.pending_messages.append((arrival_time, message))
        else:
            # Immediate delivery
            self.message_queue.append(message)
    
    def step(self):
        """Execute one GHS step."""
        # DON'T call super().step() because it rebuilds neighbor_graph and undoes pruning!
        # Instead, manually handle robot dynamics and control
        
        # Compute control for each robot (CBF + CLF)
        for robot_id, robot in enumerate(self.robots):
            control_force = self.controller.compute_control(robot_id, self.robots)
            robot.step(self.flow, self.dt, self.time, self.workspace, control_force)
        
        # Update time
        self.time += self.dt
        
        # Update edge weights (but don't rebuild neighbor graph!)
        self._update_edge_weights()
        
        # Initial spontaneous wakeup: All robots can wake (reference behavior)
        # Each robot independently chooses its minimum edge and sends CONNECT
        if not self.initial_wakeup_done and self.time >= 0.5:
            for robot_id in range(self.num_robots):
                self._wakeup(robot_id)  # All wake simultaneously
            self.initial_wakeup_done = True
            self.ghs_started = True
            print(f"\n[All robots spontaneously awakened at t={self.time:.2f}s]")
            print(f"  Each robot sends CONNECT to its minimum edge...")
        
        # Move pending messages to queue if they've arrived
        arrived_messages = []
        remaining_messages = []
        for arrival_time, msg in self.pending_messages:
            if self.time >= arrival_time:
                arrived_messages.append(msg)
            else:
                remaining_messages.append((arrival_time, msg))
        self.pending_messages = remaining_messages
        
        # Add arrived messages to queue
        for msg in arrived_messages:
            self.message_queue.append(msg)
        
        # Process messages (20 per step for faster convergence)
        messages_this_step = 0
        max_messages_per_step = 20  # Faster processing
        
        while self.message_queue and messages_this_step < max_messages_per_step:
            msg = self.message_queue.popleft()
            
            # Debug: Log first 10 messages only  
            if self.messages_processed < 10:
                print(f"  [Msg {self.messages_processed}] {msg.type}: {msg.sender}->{msg.receiver}")
            
            if msg.type == 'CONNECT':
                # Queue CONNECT for condition-based processing (reference pattern)
                receiver = msg.receiver
                sender = msg.sender
                sender_level = msg.data['level']
                
                # Wake receiver if sleeping
                if self.node_state[receiver] == NodeState.SLEEPING:
                    self._wakeup(receiver)
                
                # Queue the CONNECT request
                self.connect_requests[receiver][sender] = sender_level
                
            elif msg.type == 'TEST':
                # Queue TEST for condition-based processing (reference pattern)
                receiver = msg.receiver
                sender = msg.sender
                sender_level = msg.data['level']
                sender_frag_id = msg.data['fragment_id']
                
                # Wake receiver if sleeping
                if self.node_state[receiver] == NodeState.SLEEPING:
                    self._wakeup(receiver)
                
                # Queue the TEST request as (Level, FragmentID, Sender)
                self.test_requests[receiver].add((sender_level, sender_frag_id, sender))
            elif msg.type == 'ACCEPT':
                self._process_accept(msg)
            elif msg.type == 'REJECT':
                self._process_reject(msg)
            elif msg.type == 'INITIATE':
                # Process INITIATE
                receiver = msg.receiver
                level = msg.data['level']
                frag_id = msg.data['fragment_id']
                state = msg.data['state']
                merge = msg.data.get('merge', False)
                
                # Update fragment identity
                old_level = self.level[receiver]
                old_frag = self.fragment_id[receiver]
                self.level[receiver] = level
                self.fragment_id[receiver] = frag_id
                self.node_state[receiver] = state
                self.best_edge[receiver] = None
                self.best_weight[receiver] = (float('inf'), float('inf'), float('inf'))
                self.find_count[receiver] = 0  # Reset find_count before counting children
                self.report_over[receiver] = False
                
                # Set parent edge: sender is our parent in the fragment tree
                self.parent_edge[receiver] = msg.sender
                
                # Process deferred TEST messages now that level increased
                if level > old_level:
                    self._process_deferred_messages(receiver)
                
                # Forward INITIATE to all BRANCH children (except parent)
                for neighbor in self.in_branch[receiver]:
                    if neighbor != msg.sender:  # Don't send back to parent
                        self._send_initiate(receiver, neighbor, level, frag_id, state)
                        if state == NodeState.FIND:
                            self.find_count[receiver] += 1
                
                # Start testing for minimum outgoing edge
                if state == NodeState.FIND:
                    self.find_source[receiver] = self.parent_edge[receiver]
                    # Clear accepted edges from previous FIND phase
                    self.accepted_edges[receiver].clear()
                    self._test_minimum_edge(receiver)
            elif msg.type == 'REPORT':
                self._process_report(msg)
            elif msg.type == 'CHANGEROOT':
                self._process_changeroot(msg)
            
            self.messages_processed += 1
            messages_this_step += 1
        
        # CONDITION-BASED PROCESSING (from reference implementation pattern)
        # Process queued CONNECT/TEST based on state conditions
        for robot_id in range(self.num_robots):
            if self.node_state[robot_id] == NodeState.SLEEPING:
                continue
            
            # Condition 1: Merge - if we sent CONNECT and received CONNECT back
            if self.sent_connect_to[robot_id] is not None and \
               self.sent_connect_to[robot_id] in self.connect_requests[robot_id]:
                other = self.sent_connect_to[robot_id]
                # Check if other node hasn't already processed the merge
                # (to avoid both nodes trying to merge simultaneously)
                if self.level[robot_id] == self.level[other] and \
                   self.fragment_id[robot_id] != self.fragment_id[other]:
                    self._merge(robot_id, other)
                elif self.fragment_id[robot_id] == self.fragment_id[other]:
                    # Already merged, just clean up
                    pass
                del self.connect_requests[robot_id][other]
                self.sent_connect_to[robot_id] = None
            
            # Condition 2: Absorb - received CONNECT from lower OR same level
            # At level 0, absorb any CONNECT to grow the fragment (chain pattern)
            elif self.connect_requests[robot_id]:
                min_sender = min(self.connect_requests[robot_id],
                                key=lambda s: self.connect_requests[robot_id][s])
                if self.connect_requests[robot_id][min_sender] <= self.level[robot_id]:
                    self._absorb_node(robot_id, min_sender)
                    del self.connect_requests[robot_id][min_sender]
            
            # Condition 3: Process TEST requests when conditions met  
            elif self.test_requests[robot_id]:
                processable = {(L, F, j) for (L, F, j) in self.test_requests[robot_id]
                              if F == self.fragment_id[robot_id] or
                              (F != self.fragment_id[robot_id] and L <= self.level[robot_id])}
                if processable:
                    self._process_test_requests(robot_id, processable)
            
            # Condition 4: Report when testing done and all children reported
            # Also handle case where test_over is None but no more BASIC edges exist (edge became BRANCH during testing)
            elif self.find_count[robot_id] == 0 and not self.report_over[robot_id]:
                if self.test_over[robot_id]:
                    # Normal case: testing completed
                    self._do_report(robot_id)
                elif self.test_over[robot_id] is None:
                    # Edge case: test_over never set (edge became BRANCH while testing)
                    # Check if there are any untested BASIC edges remaining
                    has_untested_basic = False
                    neighbors = self._get_neighbors(robot_id)
                    for neighbor in neighbors:
                        if neighbor in self.accepted_edges[robot_id]:
                            continue
                        if self._get_edge_state(robot_id, neighbor) == EdgeState.BASIC:
                            has_untested_basic = True
                            break
                    
                    if not has_untested_basic:
                        # No more edges to test, force reporting
                        self.test_over[robot_id] = True
                        self._do_report(robot_id)
        
        # Check convergence: All robots in FOUND state, same fragment, and no pending messages
        if not self.ghs_converged:
            all_found = all(state == NodeState.FOUND for state in self.node_state)
            unique_fragments = len(set(self.fragment_id))
            no_messages = len(self.message_queue) == 0 and len(self.pending_messages) == 0
            mst_complete = self._count_mst_edges() == self.num_robots - 1
            
            # Debug output every 5 seconds with convergence diagnostics
            if self.ghs_started and self.time - self.last_debug_time >= self.debug_interval:
                self.last_debug_time = self.time
                found_count = sum(1 for s in self.node_state if s == NodeState.FOUND)
                find_count = sum(1 for s in self.node_state if s == NodeState.FIND)
                sleeping_count = sum(1 for s in self.node_state if s == NodeState.SLEEPING)
                
                print(f"\n[DEBUG t={self.time:.1f}s]")
                print(f"  States: FOUND={found_count}, FIND={find_count}, SLEEP={sleeping_count}")
                print(f"  Fragments={unique_fragments}, MST edges={self._count_mst_edges()}/{self.num_robots-1}")
                print(f"  Messages: Queued={len(self.message_queue)}, Transit={len(self.pending_messages)}")
                print(f"  Fragment IDs: {set(self.fragment_id)}")
                print(f"  Levels: {set(self.level)}")
                
                # Detailed convergence check
                print(f"\n  Convergence Check:")
                print(f"    ✓ All FOUND? {all_found} (need: True)")
                print(f"    ✓ Single fragment? {unique_fragments == 1} (fragments={unique_fragments})")
                print(f"    ✓ No messages? {no_messages} (need: True)")
                print(f"    ✓ MST complete? {mst_complete} (edges={self._count_mst_edges()}/{self.num_robots-1})")
                
                # Identify blocking robots
                if not all_found:
                    print(f"\n  Robots NOT in FOUND state:")
                    for i, state in enumerate(self.node_state):
                        if state != NodeState.FOUND:
                            print(f"    Robot {i}: {state.value}, Level={self.level[i]}, Frag={self.fragment_id[i]}, FindCount={self.find_count[i]}")
                            print(f"      test_over={self.test_over[i]}, report_over={self.report_over[i]}, test_edge={self.test_edge[i]}")
                            print(f"      accepted_edges={self.accepted_edges[i]}")
                
                if unique_fragments > 1:
                    print(f"\n  Multiple fragments detected:")
                    frag_groups = defaultdict(list)
                    for i, fid in enumerate(self.fragment_id):
                        frag_groups[fid].append(i)
                    for fid, robots in frag_groups.items():
                        print(f"    Fragment {fid}: Robots {robots}")
            
            # DEADLOCK DETECTION AND RECOVERY
            # If all FOUND, multiple fragments, no messages, and MST incomplete -> deadlock
            if all_found and unique_fragments > 1 and no_messages and not mst_complete:
                # Identify fragment roots (nodes with find_source=None or parent_edge pointing to self)
                # Force them to restart FIND phase with fresh scan for BASIC edges
                print(f"\n  [DEADLOCK DETECTED] Forcing fragment roots to restart FIND...")
                
                for robot_id in range(self.num_robots):
                    # Root detection: no parent or parent points to neighbor in same fragment
                    is_root = (self.find_source[robot_id] is None or 
                              self.parent_edge[robot_id] is None)
                    
                    if not is_root:
                        continue
                    
                    # This is a fragment root - rescan for valid BASIC edges
                    neighbors = self._get_neighbors(robot_id)
                    min_weight = (float('inf'), float('inf'), float('inf'))
                    min_neighbor = None
                    
                    for neighbor in neighbors:
                        if self._get_edge_state(robot_id, neighbor) == EdgeState.BASIC:
                            weight = self.edge_weights.get((robot_id, neighbor), (float('inf'), float('inf'), float('inf')))
                            if weight < min_weight:
                                min_weight = weight
                                min_neighbor = neighbor
                    
                    if min_neighbor is not None:
                        # Found a valid outgoing edge - restart FIND phase
                        self.node_state[robot_id] = NodeState.FIND
                        self.best_edge[robot_id] = min_neighbor
                        self.best_weight[robot_id] = min_weight
                        self.test_over[robot_id] = True
                        self.report_over[robot_id] = False
                        self.find_count[robot_id] = 0
                        self.accepted_edges[robot_id].clear()
                        
                        # Send CHANGEROOT on the minimum outgoing edge
                        self._send_message_with_delay(Message(
                            'CHANGEROOT', robot_id, min_neighbor
                        ))
                        print(f"    Root {robot_id}: Sending CHANGEROOT to {min_neighbor} (weight={min_weight[0]:.3f})")
            
            if all_found and unique_fragments == 1 and no_messages:
                self.ghs_converged = True
                self.convergence_time = self.time
                print(f"\n{'='*70}")
                print(f"[GHS CONVERGED at t={self.time:.2f}s]")
                print(f"  MST edges: {self._count_mst_edges()}/{self.num_robots-1}")
                print(f"  Initial edges: {self.initial_edge_count} (fully connected)")
                print(f"  Edges pruned: {self.pruned_edge_count}")
                print(f"  Current edges: {len(self.current_edges())}")
                print(f"  Messages processed: {self.messages_processed}")
                print(f"  Fragment ID: {self.fragment_id[0]}")
                print(f"  Final level: {max(self.level)}")
                print(f"{'='*70}")
                print("\n✓✓✓ SUCCESS: Distributed MST discovery complete! ✓✓✓\n")
    
    def _count_mst_edges(self):
        """Count MST (BRANCH) edges."""
        count = 0
        edges = self.current_edges()
        for i, j in edges:
            if self._get_edge_state(i, j) == EdgeState.BRANCH:
                count += 1
        return count  # current_edges() already returns unique edges (i < j)
    
    def get_mst_edges(self):
        """Get list of MST edges (for green coloring)."""
        mst_edges = []
        edges = self.current_edges()
        for i, j in edges:
            if self._get_edge_state(i, j) == EdgeState.BRANCH:
                mst_edges.append(self._normalize_edge(i, j))
        # Remove duplicates
        return list(set(mst_edges))
    
    def get_state(self):
        """Override to include GHS state."""
        state = super().get_state()
        state['ghs_converged'] = self.ghs_converged
        state['mst_edges'] = self.get_mst_edges()
        state['messages_queued'] = len(self.message_queue)
        state['messages_in_transit'] = len(self.pending_messages)
        state['fragment_count'] = len(set(self.fragment_id))
        state['awakened_count'] = sum(1 for s in self.node_state if s != NodeState.SLEEPING)
        state['pruned_edges'] = self.pruned_edge_count
        state['initial_edges'] = self.initial_edge_count
        return state


class GHSAnimator(FullyConnectedAnimator):
    """Custom animator for GHS with green MST edges."""
    
    def __init__(self, simulation, vis_config):
        """Initialize GHS animator with dual edge collections."""
        super().__init__(simulation, vis_config)
        
        # Remove default edge collection and create two: MST (green) and non-MST (purple)
        self.active_lines.remove()
        
        # MST edges (green)
        from matplotlib.collections import LineCollection
        self.mst_lines = LineCollection(
            [], colors="#00ff00", linewidths=2.5, zorder=4, alpha=0.8
        )
        self.ax.add_collection(self.mst_lines)
        
        # Non-MST edges (purple)
        self.nonmst_lines = LineCollection(
            [], colors="#9b59b6", linewidths=1.5, zorder=3, alpha=0.5
        )
        self.ax.add_collection(self.nonmst_lines)
        
        # Update legend
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], color='#00ff00', linewidth=2.5, label='MST (BRANCH)'),
            Line2D([0], [0], color='#9b59b6', linewidth=1.5, label='Non-MST'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#e74c3c',
                   markersize=8, label='Robot'),
        ]
        self.ax.legend(handles=legend_elements, loc='upper right', fontsize=7, framealpha=0.9)
    
    def update(self, frame):
        """Update with MST edge coloring."""
        # Step simulation
        self.sim.step()
        state = self.sim.get_state()
        
        # Update time history
        current_time = self.sim.time
        self.time_history.append(current_time)
        self.edge_count_history.append(state['num_edges'])
        
        # Calculate average distance to goal
        distances_to_goal = np.linalg.norm(state['positions'] - state['goal'], axis=1)
        avg_dist = np.mean(distances_to_goal)
        self.distance_history.append(avg_dist)
        
        # Limit history
        if len(self.time_history) > self.max_time_window * 10:
            self.time_history.pop(0)
            self.edge_count_history.pop(0)
            self.distance_history.pop(0)
        
        # Update metrics plot
        self.edge_count_line.set_data(self.time_history, self.edge_count_history)
        self.distance_line.set_data(self.time_history, self.distance_history)
        
        # Adjust x-axis
        if self.time_history:
            max_time = max(self.time_history)
            self.ax_metrics.set_xlim(max(0, max_time - 30), max_time + 3)
        
        # Update main plot - robot positions
        self.node_scatter.set_offsets(state['positions'])
        
        # Update labels
        for label in self.labels:
            label.remove()
        self.labels = []
        for i, pos in enumerate(state['positions']):
            label = self.ax.text(pos[0], pos[1] + 0.3, str(i), ha='center', 
                               fontsize=8, color='white', weight='bold', zorder=7)
            self.labels.append(label)
        
        # Update edges - separate MST and non-MST
        mst_edges = state.get('mst_edges', [])
        mst_segments = []
        nonmst_segments = []
        
        for i, j in state['edges']:
            edge_normalized = self.sim._normalize_edge(i, j)
            segment = [state['positions'][i], state['positions'][j]]
            
            if edge_normalized in mst_edges:
                mst_segments.append(segment)
            else:
                nonmst_segments.append(segment)
        
        self.mst_lines.set_segments(mst_segments)
        self.nonmst_lines.set_segments(nonmst_segments)
        
        # Update flow field
        flow_velocities = np.array(
            [self.sim.flow.velocity(p, self.sim.time) for p in self.flow_grid_points]
        )
        self.quiver.set_UVC(flow_velocities[:, 0], flow_velocities[:, 1])
        
        # Update status text
        max_dist = np.max(distances_to_goal) if len(distances_to_goal) > 0 else 0
        converged_str = "✓ CONVERGED" if state.get('ghs_converged', False) else "RUNNING"
        
        status_lines = [
            f"Time: {state['time']:6.2f}s",
            f"GHS Status: {converged_str}",
            f"MST Edges: {len(mst_edges)}",
            f"Total Edges: {state['num_edges']}",
        ]
        self.status.set_text("\n".join(status_lines))
        
        # Update info text
        info_lines = self._get_info_text(state, avg_dist, max_dist, mst_edges)
        self.info_text.set_text("\n".join(info_lines))
        
        return []
    
    def _get_info_text(self, state, avg_dist, max_dist, mst_edges):
        """Generate info text for display."""
        converged_str = "✓ YES" if state.get('ghs_converged', False) else "NO"
        awakened = state.get('awakened_count', 0)
        total_robots = self.sim.num_robots
        
        # Get convergence check details from simulation
        all_found = all(s == NodeState.FOUND for s in self.sim.node_state)
        unique_frags = len(set(self.sim.fragment_id))
        no_msgs = len(self.sim.message_queue) == 0 and len(self.sim.pending_messages) == 0
        
        found_count = sum(1 for s in self.sim.node_state if s == NodeState.FOUND)
        find_count = sum(1 for s in self.sim.node_state if s == NodeState.FIND)
        
        # Get pruning stats
        initial_edges = self.sim.initial_edge_count
        pruned = self.sim.pruned_edge_count
        current_edges = state['num_edges']
        
        return [
            f"GHS ALGORITHM",
            f"Converged: {converged_str}",
            f"",
            f"MST Edges: {len(mst_edges)}/{total_robots-1}",
            f"Graph: {initial_edges}→{current_edges} edges",
            f"Pruned: {pruned} edges",
            f"",
            f"Robots Awake: {awakened}/{total_robots}",
            f"FOUND: {found_count}, FIND: {find_count}",
            f"Fragments: {unique_frags}",
            f"Msgs: Q={state.get('messages_queued', 0)} T={state.get('messages_in_transit', 0)}",
            f"",
            f"Converge: {'✓' if all_found else '✗'}FOUND {'✓' if unique_frags==1 else '✗'}1Frag {'✓' if no_msgs else '✗'}NoMsg",
            f"",
            f"Avg Dist: {avg_dist:.2f}m",
        ]


def main():
    """Run GHS distributed MST discovery simulation."""
    print("\n" + "="*70)
    print("GHS ALGORITHM: DISTRIBUTED MST DISCOVERY")
    print("(Using baseline_simulation module)")
    print("="*70)
    
    # Configuration
    sim_config = SimulationConfig(
        num_robots=10,
        communication_radius=2.0,
        verbose=False
    )
    control_config = ControlConfig()
    vis_config = VisualizationConfig(interval_ms=100)
    
    print(f"\nConfiguration:")
    print(f"  Robots: {sim_config.num_robots}")
    print(f"  Communication Radius: {sim_config.communication_radius}m")
    print(f"  Algorithm: GHS (Gallager-Humblet-Spira)")
    print(f"\nVisualization:")
    print(f"  GREEN edges = MST (BRANCH)")
    print(f"  PURPLE edges = Non-MST (BASIC/REJECTED)")
    print(f"\nGoal: Discover MST in a distributed manner")
    print("="*70 + "\n")
    
    # Create GHS simulation
    sim = GHSSimulation(sim_config, control_config)
    
    # Create GHS animator
    animator = GHSAnimator(sim, vis_config)
    anim = animator.animate()
    
    plt.tight_layout()
    plt.show()
    
    print()
    print("Simulation completed.")
    print(f"GHS Converged: {sim.ghs_converged}")
    if sim.convergence_time:
        print(f"Convergence Time: {sim.convergence_time:.2f}s")
    print(f"MST Edges: {sim._count_mst_edges()}")
    print(f"Total Messages: {sim.messages_processed}")


if __name__ == "__main__":
    main()
