# Algorithm 6: Distributed Edge Pruning - Quick Reference

## Problem Statement

After Algorithm 5 (Improvement), we have:
- **Goal:** Remove non-critical edges to reach **Minimal Bridge Graph (MBG)**
- **Challenge:** Do this **distributedly** with only neighbor-to-neighbor communication
- **Success Metric:** All remaining edges are critical (bridges)

## Why Purely Distributed?

### ❌ Centralized (Wrong Approach)
```
Server gathers all edges
Clear all non-critical edges
Send update to network
Problem: Single point of failure, scalability bottleneck
```

### ✅ Distributed (Algorithm 6)
```
Each node locally decides about its edges
Nodes communicate peer-to-peer (endpoints only)
Consensus via message passing
All decisions made locally without global knowledge
```

---

## Algorithm 6 at a Glance

| Phase | What Happens | Duration | Messages |
|-------|------------|----------|----------|
| **Phase 1** | Each node marks its non-critical edges for (possible) removal | $O(1)$ | 0 |
| **Phase 2** | For each non-critical edge $e=(u,v)$, endpoints $u,v$ run parallel BFS to verify alternative paths exist without $e$ | $O(D)$ | $O(n+m)$ per edge |
| **Phase 3** | Endpoints of removable edges reach consensus: exchange "PROPOSE_REMOVE" → "CONFIRM_REMOVE" messages | $O(D)$ | 2 per edge |
| **Phase 4** | Re-run Algorithm 3 (Bridge Detection) on pruned graph to verify all remaining edges are still critical | $O(D)$ | $O(m)$ |
| **Convergence Check** | If all edges are critical: **DONE**. Else: repeat | $O(D)$ | 1 |

**Per Iteration:** $O(D)$ rounds  
**Total:** $O(k \cdot D)$ where $k$ = number of iterations

---

## Key Distributed Characteristics

### ✅ No Centralized Coordinator
```python
# Each node acts independently
for each non_critical_edge in my_edges:
    if alternative_path_exists:
        propose_to_peer("REMOVE_EDGE")
        wait_for_confirmation()
```

### ✅ Neighbor-Only Communication
```
Edge (u,v) decision involves ONLY nodes u and v
No global broadcast
Message flow: u ↔ v (peer-to-peer)
```

### ✅ Synchronous Rounds
```
Round 1: Phase 1 (mark edges)
Round 2-1+D: Phase 2 (verify paths via BFS)
Round 3+D: Phase 3 (consensus)
Round 4+2D: Phase 4 (verify critical)
```

### ✅ Fault Tolerant
```
If node u fails: edge (u,v) kept (safe)
If communication link fails: edge kept (conservative)
No single point of failure
```

---

## Mathematical Core: Alternative Path Verification

**For each non-critical edge $e = (u,v)$:**

$$\text{Is } e \text{ removable?} \iff \exists \text{ path } P: u \to v \text{ in } G \setminus \{e\}$$

**Distributed Mechanism:**

```
Node u:                          Node v:
  queue = [u]                      queue = [v]
  visited = {u}                    visited = {v}
  
  FOR D rounds:                  FOR D rounds:
    FOR each node in queue:        FOR each node in queue:
      FOR each neighbor w:           FOR each neighbor w:
        IF edge ≠ (u,v):             IF edge ≠ (u,v):
          IF w ∉ visited:              IF w ∉ visited:
            visited += w               visited += w
            queue += w                 queue += w
            
            IF w == v:               IF w == u:
              found_path = TRUE        found_path = TRUE
              SEND to v:               SEND to u:
                "PATH_FOUND"             "PATH_FOUND"
```

**Convergence:** After $O(D)$ rounds, both endpoints know if path exists

---

## Consensus Protocol (Phase 3)

**Two-Phase Commit for Edge Removal:**

```
Step 1: u proposes removal
u → v: "EDGE(u,v), can_remove_after_bfs? PROPOSE_REMOVE"

Step 2: v receives and confirms
v → u: "CONFIRM_REMOVE(u,v)"

Step 3: Both update locally
if both agreed:
    local_edges[i] -= edge(u,v)
```

**Safety:** Edge only removed if BOTH endpoints confirm alternative path

---

## Example: From Redundant to Critical

### Start: Custom Grid After Improvement
```
100% of edges are NON-CRITICAL (all redundant)
Left grid ←→ Right grid (two clusters joined by edges)
Problem: Cannot distinguish which edges to keep
```

### Phase 2 Executes:
```
Edge (0,1) test: Alternative path 0→3→6→...→1 ✓ REMOVABLE
Edge (1,2) test: Alternative path 1→0→3→...→2 ✓ REMOVABLE
Edge (2,3) test: Alternative path exists ✓ REMOVABLE
...
(repeat for all 10 edges)
```

### Phase 3 Executes:
```
ALL endpoints confirm removal
Delete all 10 edges
Graph now: Just the 8 nodes, 0 edges (DISCONNECTED!)
```

### Phase 4 - Bridge Detection:
```
OOPS! All previous edges were bridges after all!
No edge actually redundant
Each was a bridge relative to the others
```

### Next Iteration - Recalibration:
```
Start with cleaner understanding
This time: Some edges ARE truly removable
Remove only those
Continue until convergence
```

### Final: MBG Reached
```
~5-6 edges remaining
ALL: CRIT(e) = 1 ✓
Network is "at bottleneck" = minimal critical edge configuration
```

---

## Comparison: Centralized vs. Distributed

### Centralized Approach (WRONG for this paper)
```python
# Server collects all edges
all_edges = collect_from_network()  # ❌ Centralized

# Clear non-critical
for e in all_edges:
    if not is_critical[e]:
        remove(e)

# Broadcast to network
broadcast(all_edges)  # ❌ Single message

# Problems:
# - Server becomes bottleneck
# - Privacy: server sees entire topology
# - Scalability: message size = O(m)
# - Fault: server failure = network failure
```

### Distributed Algorithm 6 (CORRECT)
```python
# Each node decides locally
for e in my_incident_edges:
    if non_critical:
        # Run BFS with peer (only neighbors involved)
        if alternative_path_exists_without(e):
            # Two-phase commit with peer only (2 messages)
            propose_remove_to_peer(e)
            wait_for_confirmation(e)
            
            # Update local state only
            my_edges.remove(e)

# Advantages:
# ✓ No bottleneck (all decisions distributed)
# ✓ Privacy: each node sees only neighbors
# ✓ Scalability: messages O(n+m) per edge, not O(m) total
# ✓ Fault tolerant: no single point of failure
```

---

## Integration with Other Algorithms

```
Algorithm 1-2: Build network topology
       ↓
Algorithm 3: Detect critical edges (bridges)
       ↓
Algorithm 4: Compute edge connectivity for each edge
       ↓
Algorithm 5: Add edges to remove bridges
       ↓
Algorithm 6: PRUNE edges to reach MBG ← YOU ARE HERE
       ↓
Result: Minimal set of critical edges
        All redundant edges removed
        Network at "bottleneck"
```

---

## Input/Output Contract

### Input to Algorithm 6
- Current graph $G = (V, E)$ (from Algorithm 5)
- Status of each edge: critical vs. non-critical (from Algorithm 3)

### Output from Algorithm 6
- Minimal Bridge Graph $G_{MBG} = (V, E_{pruned})$
- **Property:** $\forall e \in E_{pruned}: \text{CRIT}(e) = 1$
- **Distributed:** Each node knows only its local pruned edges

---

## Correctness Proof

**Theorem:** Algorithm 6 produces a valid MBG

**Proof Sketch:**
1. By Algorithm design, Phase 4 verifies all remaining edges are bridges
2. If they weren't, Phase 2 would have offered removal again
3. If no removal is possible, we're at minimal set
4. Process continues until fixed point
5. By uniqueness of MBG, we reach the one true minimal bridge graph

---

## Performance Metrics

| Metric | Value | Note |
|--------|-------|------|
| **Distributed Rounds** | $O(k \cdot D)$ | $k$ iterations, $D$ diameter |
| **Messages Total** | $O(k \cdot m^2)$ | $k$ iterations, $m$ edges |
| **Space per Node** | $O(n)$ | BFS state + local edges |
| **Worst Case Iterations** | $O(m)$ | If removing one edge per iteration |
| **Typical Iterations** | $O(1)$ or $O(\log m)$ | Often converges quickly |

---

## Summary: Why Algorithm 6 is Distributed

```
✓ Decisions made at edge endpoints (u,v) only
✓ No node has global view
✓ Messages: neighbor-to-neighbor only
✓ Synchronous rounds: all nodes progress together
✓ Consensus: peer agreement, not central decree
✓ Fault tolerant: no coordinator to fail

This is a TRUE distributed algorithm, not delegated centralized logic.
```
